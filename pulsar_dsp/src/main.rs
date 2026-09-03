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
//! Whichever path does reach the mute then latches the active exception number
//! and the fault status registers into `FAULT_RECORD`, between the mute and the
//! park. The module documentation of `pulsar_lib::postmortem` carries the
//! offset by offset layout to read at that symbol. Nothing in this binary reads
//! the record back, and no fault reaches the control link.

#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::ptr;
use cortex_m::Peripherals;
use cortex_m::asm;
use cortex_m::interrupt;
use cortex_m::peripheral::scb::Exception;
use cortex_m::peripheral::{AC, SCB};
use cortex_m_rt::{entry, exception};
use pulsar_lib::constants::{MAX_CORE_CLOCK_HZ, mute_hold_iterations};
use pulsar_lib::postmortem::{FaultRecord, FaultRegisters};
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

/// Bits of `ICSR` holding the exception the core is serving.
///
/// PM0253 section 4.3.3, `VECTACTIVE` occupies bits 8 to 0 and reads zero in
/// thread mode.
const VECTACTIVE_MASK: u32 = 0x1FF;

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

/// Arms the `BusFault` handler, then parks the core.
///
/// PM0253 section 2.5.2 escalates a fault to `HardFault` when the handler for
/// that fault is disabled, and exempts the stack push that enters an enabled
/// `BusFault` handler from escalation. Arming `SHCSR.BUSFAULTENA` is what lets
/// a faulted stack push reach a vector at all.
///
/// The read-back and the `None` arm both end in silence, because no state of
/// this binary means "the guard is absent". Neither is reachable here, since
/// `Peripherals::take` runs once and `SHCSR` holds the bit.
#[entry]
fn main() -> !
{
    let Some(mut peripherals) = Peripherals::take()
    else
    {
        silence_and_park()
    };

    peripherals.SCB.enable(Exception::BusFault);

    if !peripherals.SCB.is_enabled(Exception::BusFault)
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
/// a read-modify-write. Configuring PE2 to PE6 as SAI alternate functions
/// already requires `GPIOEEN`, so on any path where the converters could have
/// been unmuted the first store has landed. This step covers a fault before
/// that point, where the pin is still high impedance and the 10 k pull-down is
/// what holds XSMT low.
///
/// The post-mortem comes fourth, once the mute is complete, and it is the whole
/// diagnostic this binary produces. The exception number comes from `ICSR`
/// rather than from the argument of the default handler, because the hard fault
/// handler, the panic handler and the two guard arms of `main` are handed no
/// argument and this routine is what all of them share. Latching a cause does
/// not make it reportable: this routine masks interrupts and never returns, so
/// no frame leaves the board after a fault, and `FAULT_RECORD` is read by a
/// probe alone.
///
/// The record goes down one word at a time, so the fault path builds no copy of
/// it on the stack. The magic lands first and the checksum last, which leaves
/// an interrupted write failing validation rather than reading as a record.
///
/// Sealing it costs r4, r5 and r6. Exception entry stacks r0 to r3, r12, lr, pc
/// and xpsr, and the prologue here pushes r7, so those three are the registers
/// of the faulting context that nothing preserves. A probe on the parked core
/// reads this routine's scratch in them. The record buys the fault status at
/// that price.
///
/// The hold covers `MUTE_SEQUENCE_US`. Nothing in this binary starts or stops
/// the audio clocks, so it becomes load bearing once a reset path exists, and
/// the watchdog period is then derived from `MUTE_SEQUENCE_US` so its reset
/// cannot land inside the converter ramp. The record is written before the
/// hold, so the hold costs the diagnostic nothing.
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
        // also select the alternate function of PE2 to PE6.
        let _ = rcc.ahb4enr().read().gpioeen().bit_is_set();
        let gpioe = GPIOE::steal();
        let _ = gpioe.moder().read().bits();
        gpioe.moder().modify(|_, w| w.moder7().output());
        gpioe.bsrr().write(|w| w.br7().set_bit());
    }

    // SAFETY: interrupts are masked and this function never returns, so nothing
    // else can observe these register blocks or the record. The reads are
    // volatile reads of status registers, which have no side effect, and the
    // writes cover the eight words of the record and nothing beyond them, in a
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
        let record = FaultRecord::new(&registers).to_words();

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
