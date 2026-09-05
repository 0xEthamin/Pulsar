//! Audio processing firmware for the STM32H743 board.
//!
//! Every crossover, equalisation and protection stage of the loudspeaker runs
//! here. The machine carries no analog filter, so a fault in this binary
//! reaches the drivers directly.
//!
//! The converters come out of reset muted, held there by the pull-down on their
//! XSMT pin. Nothing in this binary raises XSMT, so they stay muted.
//!
//! A Rust panic, a hard fault, and any exception or interrupt without a handler
//! of its own drive XSMT low and park the core, because a fault that mutes only
//! sometimes is not a mute.
//!
//! A stack overflow does not. PM0253 section 2.5.3 makes a processor store
//! fault asynchronous, so the overflowing push pends a `BusFault` rather than
//! trapping at the instruction, and execution carries on with a corrupt stack
//! pointer. A frame push, a peripheral read and a second frame push then stand
//! between the `BusFault` vector and the mute store, and both pushes write
//! through that same pointer. Section 2.5.2 exempts only the entry push, so
//! nothing carries the second one, and whether XSMT goes low on this path is
//! undetermined. Putting the mute in front of those pushes takes an entry that
//! touches no stack, which this binary does not have.
//!
//! `main` brings the audio kernel clock up, then the output transport that
//! carries it to the header. The clock bring-up reads the register fields the
//! output frequency depends on back and compares each against the plan, because
//! a lock bit reports a PLL fed from the wrong oscillator as ready. The
//! transport bring-up does the same for every field it drives off its reset
//! value in the two sub-blocks and the two transfer streams, because the
//! interface reports no bit meaning "configured as asked". Either one failing
//! to verify takes the fault path.
//!
//! Two inputs stay outside both read-backs. The crystal frequency is the one
//! the clock depends on and no register reports, and the port that carries the
//! frame to the header is written and read by nothing. So what comes back is a
//! witness that the part took the plan, not a proof of the frame rate and not a
//! proof that a pin moves.
//!
//! Once both are up, PE2 to PE6 carry a master clock, a bit clock, a frame
//! clock and two data lines, and a 1 kHz tone repeats on the four channels of
//! the frame with no further work from the core. PE7 is named by nothing but
//! the fault path, so it keeps the analog mode a reset leaves it in and the
//! 10 k pull-down on it holds the converters muted through all of this.
//!
//! Whichever path does reach the mute then latches the active exception number,
//! the fault status registers and the clock fault code into `FAULT_RECORD`,
//! between the mute and the park. The module documentation of
//! `pulsar_lib::postmortem` carries the offset by offset layout to read at that
//! symbol. Nothing in this binary reads the record back, and no fault reaches
//! the control link, so a probe on SWD is the only reader.
//!
//! Every path here parks in `wfi`, which is Sleep, and Sleep stops the
//! processor clock unless `DBGMCU_CR.DBGSLEEP_D1` is set. `main` sets it ahead
//! of everything else, so a parked core keeps answering the debug port. Without
//! that bit the record, the thirty-four clock fault codes it can carry and the
//! two guards that tell it from stale memory have no reader at all.

#![no_std]
#![no_main]

mod clock;
mod transport;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::Peripherals;
use cortex_m::asm;
use cortex_m::interrupt;
use cortex_m::peripheral::scb::Exception;
use cortex_m::peripheral::{AC, SCB};
use cortex_m_rt::{entry, exception};
use pulsar_lib::constants::{BOOT_CORE_CLOCK_HZ, MAX_CORE_CLOCK_HZ, mute_hold_iterations};
use pulsar_lib::postmortem::{FaultRecord, FaultRegisters};
use stm32h7::stm32h743v as device;
use stm32h7::stm32h743v::{GPIOE, RCC};

/// Port E pin wired to the XSMT input of both converter modules.
///
/// One line drives both, so a single register write silences the machine and
/// the two modules cannot disagree about being muted.
const XSMT_PIN: u8 = 7;

/// Delay loop iterations the fault path holds before it parks the core.
///
/// Sized for the highest core clock, so it is long enough at every slower one
/// the part can run, including the 64 MHz it boots on.
const MUTE_HOLD_ITERATIONS: u32 = mute_hold_iterations(MAX_CORE_CLOCK_HZ);

/// Address the processor reaches `DBGMCU_CR` at.
///
/// RM0433 section 60.5.8 maps the debug unit at `0x5C00_1000` for the processor
/// and puts `CR` at offset `0x004`.
const DBGMCU_CR: *mut u32 = 0x5C00_1004 as *mut u32;

/// `DBGMCU_CR` bit keeping the processor clock running in Sleep mode.
///
/// RM0433 section 60.5.8, `DBGSLEEP_D1` at bit 0. It reads zero after a power
/// on reset, where Sleep stops the processor clock and the debug port loses the
/// core.
const DBGSLEEP_D1: u32 = 1;

/// Bits of `ICSR` holding the exception the core is serving.
///
/// PM0253 section 4.3.3, `VECTACTIVE` occupies bits 8 to 0 and reads zero in
/// thread mode.
const VECTACTIVE_MASK: u32 = 0x1FF;

/// Clock fault code a fault path that no clock bring-up refused carries.
///
/// `ClockFault::code` numbers no fault zero, so the record tells the two apart.
const NO_CLOCK_FAULT: u32 = 0;

/// Clock fault code the fault path latches into the record.
///
/// A refused clock raises no exception, so it is the one cause `ICSR` cannot
/// carry. It travels here rather than in an argument, which leaves
/// `silence_and_park` with a signature a forwarding frame folds into its
/// caller, and keeps the mute store the first memory access of every vector.
///
/// The startup zero fill covers `.bss`, so the value out of reset is
/// `NO_CLOCK_FAULT` and only the one arm of `main` that answers a refused clock
/// writes it. `.bss` sits above STACK, where an overflowing stack does not
/// reach, so the fault path reads it back on that route as well.
static CLOCK_FAULT: AtomicU32 = AtomicU32::new(NO_CLOCK_FAULT);

const _: () = assert!
(
    XSMT_PIN == 7,
    "the register writers in the fault path name pin 7 directly"
);

/// Post-mortem the fault path leaves at a fixed address.
///
/// `.uninit` is a `NOLOAD` output section in RAM that starts at `__ebss`, where
/// the startup zero fill stops, so nothing writes the record but the fault
/// path. RAM starts above STACK and an overflowing stack descends below STACK,
/// so an overflow writes nowhere near the record. Cutting power loses it, and
/// nothing reads it then.
///
/// Surviving a reset is an ASSUMPTION, not a datasheet guarantee. RM0433
/// documents which domains retain their contents across the low-power modes and
/// says nothing about a reset, so the case is settled on the board or not at
/// all. Nothing in this binary depends on it: the reader is a probe attached to
/// a core the fault path has already parked.
///
/// The name is unmangled so that one string identifies the record in
/// `llvm-nm`, in a debugger and in the build gate, across rebuilds that would
/// otherwise move the hash in a mangled symbol.
#[expect
(
    unsafe_code,
    reason = "the record is placed by section and named for a debugger"
)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit.FAULT_RECORD")]
static mut FAULT_RECORD: MaybeUninit<FaultRecord> = MaybeUninit::uninit();

/// Sets `DBGMCU_CR.DBGSLEEP_D1`, which keeps a parked core on the debug port.
///
/// `wfi` is Sleep, and Sleep stops the processor clock while the bit is clear,
/// which is what a power on reset leaves it at. Every path of this binary ends
/// parked in `wfi`, and a probe is the only reader `FAULT_RECORD` has, so this
/// bit is what carries the record, its clock fault code and its two guards off
/// the board.
///
/// The other bits of the register are carried over rather than cleared. RM0433
/// section 60.5.8 exempts this block from the system reset, so a debugger
/// attached before this runs holds its own bits in it.
///
/// The block is reached by address rather than through the peripheral
/// singleton, which lets this run ahead of every `take` and cover the arms of
/// `main` that park when a `take` comes back empty.
#[expect
(
    unsafe_code,
    reason = "the debug unit is reached by raw pointer, ahead of the handle the \
              peripheral singleton hands out"
)]
fn keep_core_visible_in_sleep()
{
    // SAFETY: a word wide read-modify-write of DBGMCU_CR, which the part maps
    // at this address for the processor. It runs before any interrupt is
    // enabled and nothing else in this binary reaches the block, so no other
    // access falls between the read and the write.
    unsafe
    {
        ptr::write_volatile(DBGMCU_CR, ptr::read_volatile(DBGMCU_CR) | DBGSLEEP_D1);
    }
}

/// Keeps the parked core visible, arms `BusFault`, starts the clock and the
/// output transport.
///
/// `keep_core_visible_in_sleep` runs first, ahead of every arm below that can
/// park, so none of them takes the core off the debug port.
///
/// PM0253 section 2.5.2 escalates a fault to `HardFault` when the handler for
/// that fault is disabled, and exempts the stack push that enters an enabled
/// `BusFault` handler from escalation. Arming `SHCSR.BUSFAULTENA` is what lets
/// a faulted stack push reach a vector at all. It goes up before the clock, so
/// a fault in the bring-up reaches a vector rather than lockup.
///
/// Every arm below ends in silence, because no state of this binary means "the
/// guard is absent" or "the clock is close enough". The waits are sized for the
/// clock the part boots on, which is the one this function runs at, since
/// nothing here moves the system clock off the internal oscillator.
///
/// A refused clock is the one arm that names its cause. It writes the
/// `ClockFault` code to `CLOCK_FAULT` before it enters the fault path, so a
/// board that parks silent on the bench says which of the thirty-four refusals
/// it hit rather than only that it refused.
///
/// The transport takes the witness the clock bring-up returns by reference,
/// which is what leaves the start order to the compiler: RM0433 requires the
/// kernel clock present on the interface before its enable bit is set, and a
/// refused clock builds no witness to hand over.
///
/// The transport is what puts the master clock, the bit clock, the frame clock
/// and the two data lines on the header. It configures PE2 to PE6 and leaves
/// PE7 alone, so the converter mute keeps the pull-down that holds it.
///
/// A refused transport carries no code into the record. `FAULT_RECORD` names
/// one clock fault and a refusal here is not one, and the interface and stream
/// registers a refusal leaves behind say more than a code would: they are read
/// over the debug port, in place, by the probe that measures the frame.
#[entry]
fn main() -> !
{
    keep_core_visible_in_sleep();

    let Some(mut core) = Peripherals::take()
    else
    {
        silence_and_park()
    };

    core.SCB.enable(Exception::BusFault);

    if !core.SCB.is_enabled(Exception::BusFault)
    {
        silence_and_park()
    }

    let Some(part) = device::Peripherals::take()
    else
    {
        silence_and_park()
    };

    let audio_clock = match clock::start(&part.RCC, BOOT_CORE_CLOCK_HZ)
    {
        Ok(witness) => witness,
        Err(fault) =>
        {
            CLOCK_FAULT.store(fault.code(), Ordering::Relaxed);
            silence_and_park()
        }
    };

    let transport = transport::start
    (
        &audio_clock,
        &part.RCC,
        &part.SAI1,
        &part.DMA1,
        &part.DMAMUX1,
        &part.GPIOE,
        BOOT_CORE_CLOCK_HZ
    );

    if transport.is_err()
    {
        silence_and_park()
    }

    loop
    {
        asm::wfi();
    }
}

/// Silences the converters and parks the core.
///
/// The steps run in this order, and the order is the point.
///
/// The raw store driving XSMT low comes first, with no abstraction in the way,
/// because at this point the abstractions are what failed. A store to `BSRR`
/// needs no read-modify-write, so a concurrent write to the port cannot lose
/// it.
///
/// Interrupts go down second. The independent watchdog is reloaded from the
/// transfer interrupt alone, so leaving interrupts up would keep feeding it and
/// the reset that returns the pin to its pull-down would never arrive. Masking
/// also stops the faulted handler from raising XSMT again. `cpsid i` is the
/// instruction that follows the store, and an interrupt can still land on that
/// boundary and run to completion. The mute line goes down before anything
/// else, and that window is the price.
///
/// An `isb` follows the mask. ARMv7-M B5.2.1 makes a `PRIMASK` write visible to
/// later instructions only after a context synchronisation event, so without
/// the barrier an interrupt already recognised can still be taken.
///
/// The port clock and the pin direction come third, so the mute never waits on
/// a read-modify-write. RM0433 resets every pin of this port to analog mode and
/// nothing else in this binary names PE7, so the pad is high impedance until
/// this step and the 10 k pull-down is what holds XSMT low there. This step is
/// therefore what drives the pin, and the store above is what fixes the order:
/// the transport sets `GPIOEEN`, so on every path where the converter clocks
/// are running that store reaches `BSRR` rather than being dropped, ahead of
/// the read this step performs.
///
/// The post-mortem comes fourth, once the mute is complete, and it is the whole
/// diagnostic this binary produces. The exception number comes from `ICSR`
/// rather than from the argument of the default handler, because the hard fault
/// handler, the panic handler and the guard arms of `main` are handed no
/// argument and this routine is what all of them share. Latching a cause does
/// not make it reportable: this routine masks interrupts and never returns, so
/// no frame leaves the board after a fault, and `FAULT_RECORD` is read by a
/// probe alone.
///
/// `CLOCK_FAULT` is the one cause `ICSR` cannot carry, since a refused clock
/// raises no exception. This routine takes no argument, which is what lets the
/// compiler fold a forwarding handler into the trampoline above it and leaves
/// one frame push on each side of the vector rather than two. The code is read
/// from `.bss` here, after the mute store, so the mute stays the first memory
/// access of the path.
///
/// The record goes down one word at a time, so the fault path builds no copy of
/// it on the stack. The magic lands first and the checksum last, which leaves
/// an interrupted write failing validation rather than reading as a record.
///
/// Sealing it costs r4, r5, r6 and r8, measured on the linked image. Exception
/// entry stacks r0 to r3, r12, lr, pc and xpsr, and the prologue here pushes
/// r7, so those four are the registers of the faulting context that nothing
/// preserves. A probe on the parked core reads this routine's scratch in them.
/// The record buys the fault status at that price.
///
/// The hold covers `MUTE_SEQUENCE_US`, and it is load bearing. The converter
/// clocks run once the transport is up, and pulling XSMT low starts an
/// attenuation ramp the converter counts in its own sample periods, so the
/// clocks have to keep running for the length of that ramp. What a converter
/// left part way through one does is not something this firmware measures, and
/// the hold is what makes the question moot.
///
/// Nothing on this path stops those clocks: the mask above reaches interrupts
/// alone, and neither the interface enable nor the transfer streams are touched
/// here, so the frame keeps going out to a converter that is muting. The count
/// is sized for the highest clock the part runs at, so at the 64 MHz this
/// binary stays on it covers the sequence seven times over. The record is
/// written before the hold, so the hold costs the diagnostic nothing.
///
/// A watchdog would put a reset inside that window, so its period is derived
/// from `MUTE_SEQUENCE_US` when it arrives. This binary has none.
#[expect
(
    unsafe_code,
    reason = "the fault path reaches the mute pin, the fault status registers \
              and the post-mortem by raw pointer"
)]
fn silence_and_park() -> !
{
    // SAFETY: this function never returns, so no caller can observe the handle
    // it steals, and the store below is a write-only single bit set on BSRR
    // that cannot race with any other write to the port.
    unsafe
    {
        GPIOE::steal().bsrr().write(|w| w.br7().set_bit());
    }

    interrupt::disable();
    asm::isb();

    // SAFETY: interrupts are masked and this function never returns, so nothing
    // else can observe these handles or the read-modify-write below.
    unsafe
    {
        let rcc = RCC::steal();
        rcc.ahb4enr().modify(|_, w| w.gpioeen().set_bit());
        // RM0433 Rev 7 page 369, Clock enabling delays. The enable command
        // takes up to two periods of the enabled clock to reach the peripheral,
        // and until it has, a read of a port register returns invalid data and
        // a write is dropped. The prescribed sequence reads the enable register
        // back, then performs a dummy read of the peripheral. Both are
        // volatile, so neither is optimised away, and without them the
        // read-modify-write below can carry invalid data into the bits that
        // also hold PE2 to PE6 on the alternate function carrying the
        // converter clocks.
        let _ = rcc.ahb4enr().read().gpioeen().bit_is_set();
        let gpioe = GPIOE::steal();
        let _ = gpioe.moder().read().bits();
        gpioe.moder().modify(|_, w| w.moder7().output());
        gpioe.bsrr().write(|w| w.br7().set_bit());
    }

    // SAFETY: interrupts are masked and this function never returns, so nothing
    // else can observe these register blocks or the record. The reads are
    // volatile reads of status registers, which have no side effect, and the
    // writes cover the nine words of the record and nothing beyond them, in a
    // slot no other line of this binary touches. Volatile is what keeps those
    // writes, since nothing here reads the record back.
    unsafe
    {
        let scb = &*SCB::PTR;
        let ac = &*AC::PTR;

        let registers = FaultRegisters
        {
            exception: scb.icsr.read() & VECTACTIVE_MASK,
            cfsr: scb.cfsr.read(),
            hfsr: scb.hfsr.read(),
            mmfar: scb.mmfar.read(),
            bfar: scb.bfar.read(),
            abfsr: ac.abfsr.read(),
        };

        let slot = (&raw mut FAULT_RECORD).cast::<u32>();
        let record = FaultRecord::new(&registers, CLOCK_FAULT.load(Ordering::Relaxed))
            .to_words();

        for (index, word) in record.into_iter().enumerate()
        {
            ptr::write_volatile(slot.add(index), word);
        }
    }

    asm::delay(MUTE_HOLD_ITERATIONS);

    loop
    {
        asm::wfi();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> !
{
    silence_and_park()
}

/// Silences the converters after a fault the core cannot return from.
///
/// PM0253 section 2.5.2 escalates a fault whose own handler is disabled, and
/// `main` enables `BusFault` alone, so every `MemManage` and every
/// `UsageFault` lands here: an undefined instruction, an illegal unaligned
/// access, a fetch from an Execute Never region. A bus error on a vector read
/// arrives directly. None of them reach the panic handler, so this is the only
/// thing between them and a full level buffer replaying into the drivers. A
/// data bus error goes to the `BusFault` vector instead.
///
/// `trampoline = false` points the vector straight at this function and drops
/// the `ExceptionFrame` argument, and the compiler still emits a frame push of
/// its own, so the handler costs stack like any other. PM0253 section 2.5.5:
/// once the core is in lockup it executes no instruction until a reset, an NMI
/// or a debugger halt. Lockup leaves the port alone, so only a reset hands PE7
/// back to the 10 k pull-down.
#[expect
(
    unsafe_code,
    reason = "cortex-m-rt declares exception handlers as unsafe functions"
)]
#[exception(trampoline = false)]
unsafe fn HardFault() -> !
{
    silence_and_park()
}

/// Silences the converters on any exception or interrupt without its own
/// handler.
///
/// A peripheral left enabled by a half-finished initialisation raises its
/// interrupt here, and parking without muting would leave the transfers
/// running. `main` enables `BusFault` without giving it a handler of its own,
/// so a data bus error lands here too.
///
/// The interrupt number handed to this function goes unused. `FAULT_RECORD`
/// carries the active exception number, which `silence_and_park` reads from
/// `ICSR` for every entry it has.
///
/// The record reaches a probe and nothing else. `DspState::Faulted` and every
/// `Fault` variant of the control protocol name states this binary cannot
/// report, because the fault path masks interrupts and never returns.
#[allow
(
    unsafe_code,
    reason = "cortex-m-rt requires the unsafe keyword here and strips it from \
              the function it generates, so unsafe_code never fires and an \
              expect would stand unfulfilled"
)]
#[exception]
unsafe fn DefaultHandler(_irqn: i16) -> !
{
    silence_and_park()
}
