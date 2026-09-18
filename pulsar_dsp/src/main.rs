//! Audio processing firmware for the STM32H743 board.
//!
//! Every crossover, equalisation and protection stage of the loudspeaker runs
//! here. The machine carries no analog filter, so a fault in this binary
//! reaches the drivers directly.
//!
//! The converters come out of reset muted, held there by the pull-down on their
//! XSMT pin. One stage of this binary raises XSMT, the release gate, and it
//! does so only after the audio clock read back as planned, the transport read
//! back as planned, the write that publishes the built crossover chain ran,
//! both transfer counters reloaded twice with no error flag raised over that
//! window, which is a whole lap of their buffers whatever position they were
//! found at, and a buffer of zeros was held over the converter unmute ramp. One
//! vector serves the transfer events of the input path and drives that pin low
//! only on an entry it refuses, which is what the third gap below covers. Every
//! other path here drives it low once the core reaches its first instruction,
//! and the two gaps after that are where it does not.
//!
//! Once it has risen, the machine is audible until something drives the pin
//! back down or a reset returns it to its pull-down. A core lockup does
//! neither: PM0253 section 2.5.5 stops the core executing and leaves the
//! peripherals alone, so the port keeps driving PE7 high with no instruction
//! running to change it. A handler entered on a corrupt stack pointer is the
//! second gap, described below, and what it leaves behind is undetermined
//! rather than audible for certain. The third is the vector that serves the
//! transfer events: it carries a block and returns, so a flag it fails to take
//! down has it re-enter for ever on audio that stopped moving with the pin
//! still high. A fault path that runs is none of the three, since it drives the
//! pin low as its first instruction. Nothing in this binary bounds any of them,
//! because the watchdog that would is not here yet. That is accepted while no
//! driver is connected and the only listeners are an oscilloscope and a pair of
//! headphones, and it stops being acceptable the moment one is.
//!
//! A Rust panic, a hard fault, and any exception or interrupt without a handler
//! of its own drive XSMT low and park the core, because a fault that mutes only
//! sometimes is not a mute. One interrupt has a handler of its own, and it
//! reaches the same routine for every entry it refuses.
//!
//! A stack overflow does not. PM0253 section 2.5.3 makes a processor store
//! fault asynchronous, so the overflowing push pends a `BusFault` rather than
//! trapping at the instruction, and execution carries on with a corrupt stack
//! pointer. A frame push, a peripheral read and a second frame push then stand
//! between the `BusFault` vector and the mute store, and both pushes write
//! through that same pointer. Section 2.5.2 exempts only the entry push, so
//! nothing carries the second one, and whether XSMT goes low on this path is
//! undetermined. Putting the mute in front of those pushes takes an entry that
//! touches no stack, which this binary does not have. Past the release gate
//! that undetermined case is a pin left high rather than a pin left on its
//! pull-down, so what an overflow costs grew with the gate even though the path
//! did not change.
//!
//! `main` brings the audio kernel clock up, then the output transport that
//! carries it to the header, then the input path that the output is wired back
//! into, then the gate that unmutes the converters, and last the block
//! structure that carries the input into the output buffers. The clock bring-up
//! reads the register fields the output frequency depends on back and compares
//! each against the plan, because a lock bit reports a PLL fed from the wrong
//! oscillator as ready. The transport bring-up does the same for every field it
//! drives off its reset value in the two sub-blocks and the two transfer
//! streams, because the interface reports no bit meaning "configured as asked".
//! The input bring-up does the same for the receiving sub-block and its stream,
//! and starts that stream at a chosen position of the transmitting one, so the
//! distance between the two read pointers is a constant of the plan rather than
//! a draw at each power-up. The gate then measures rather than reads: it watches
//! both transmitting counters until each has reloaded twice, which a bring-up
//! cannot do, since a counter that moves once does not tell a stream that runs
//! from one that advances a word and stalls. It takes the two FIFO error flags
//! down as that window opens, and no other flag, because the fill that starts
//! the transport raises those two before it has carried anything and every one
//! of these flags is sticky. The arming comes after the gate and after the tone,
//! and it waits for the tone to travel a whole lap of the receiving buffer
//! before it enables the transfer events, since a loop seeded with silence
//! carries silence for ever. Any of the five failing takes the fault path.
//!
//! Two inputs stay outside both read-backs. The crystal frequency is the one
//! the clock depends on and no register reports, and the port that carries the
//! frame to the header is written and read by nothing. So what comes back is a
//! witness that the part took the plan, not a proof of the frame rate and not a
//! proof that a pin moves.
//!
//! Once all five are up, PE2 to PE6 carry a master clock, a bit clock, a frame
//! clock and two data lines, PD11 carries that frame back in, and PE7 holds the
//! converters unmuted. A 1 kHz tone travels the loop, and the core carries one
//! half of the receiving buffer into both output buffers twice a lap. The tone
//! is what seeds that loop: it is written after the gate and against the permit
//! the gate returns, so the output buffers carry silence for the whole of the
//! clock bring-up, the transport bring-up, the input bring-up, the transfer
//! window and the unmute ramp. The arming takes that same permit by value, so
//! nothing can enable the events that write those buffers ahead of the gate
//! either.
//!
//! Whichever path does reach the mute then latches the active exception number,
//! the fault status registers and the refusal code into `FAULT_RECORD`, between
//! the mute and the park. The module documentation of `pulsar_lib::postmortem`
//! carries the offset by offset layout to read at that symbol. Nothing in this
//! binary reads the record back, and no fault reaches the control link, so a
//! probe on SWD is the only reader.
//!
//! Those two fields say between them how a parked board got there. Every arm of
//! `main` that parks writes a refusal code first, and each of them names one
//! guard, one clock refusal, one transport refusal, one input path refusal or
//! one release refusal. A release refusal says as well whether the mute line had
//! been raised when the gate refused, so a reader knows whether the board was
//! ever audible. Every vector but the one that serves the transfer events writes
//! none and is named by its exception number instead. That one writes an input
//! path refusal, so a record carrying exception 29 beside a refusal code is the
//! ordinary shape of a refused transfer event rather than a sign of a different
//! image. The one entry that would set neither is the panic handler, which runs
//! in thread mode and refuses nothing, and the compiler emits it only once
//! something in the image can panic. While nothing can, no path of this binary
//! seals a record with zero in both fields, so one that holds both came off a
//! different image than the one on the bench.
//!
//! Every path here ends parked in `wfi`, which is Sleep, on a core that gets
//! that far. The two gaps above are where one may not. Sleep stops the
//! processor clock unless `DBGMCU_CR.DBGSLEEP_D1` is set. `main` sets it ahead
//! of everything else, so a parked core keeps answering the debug port. Without
//! that bit the record, the refusal code it carries and the two guards that
//! tell it from stale memory have no reader at all.

#![no_std]
#![no_main]

mod clock;
mod link;
mod passthrough;
mod release;
mod transport;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::Peripherals;
use cortex_m::asm;
use cortex_m::peripheral::scb::Exception;
use cortex_m::peripheral::{AC, SCB};
use cortex_m_rt::{entry, exception};
use pulsar_lib::constants::{BOOT_CORE_CLOCK_HZ, MAX_CORE_CLOCK_HZ, mute_hold_iterations};
use pulsar_lib::postmortem::{self, FaultRecord, FaultRegisters, StartupFault};
use stm32h7::stm32h743v as device;
use stm32h7::stm32h743v::{GPIOE, RCC, interrupt};

/// Port E pin wired to the XSMT input of both converter modules.
///
/// One line drives both, so a single register write silences the machine and
/// the two modules cannot disagree about being muted.
const XSMT_PIN: u8 = 7;

/// Delay loop iterations covering the converter mute sequence.
///
/// The fault path holds it after driving XSMT low, and the release gate holds
/// it after raising XSMT, so the window that covers a ramp down and the window
/// that covers a ramp up are one count and cannot drift apart.
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

/// Refusal code a fault path no arm of `main` reached carries.
///
/// Every domain of the encoding is numbered from 1, so a zero word says no arm
/// of `main` wrote one and the path was entered from somewhere else.
const NO_REFUSAL: u32 = 0;

/// Refusal code the fault path latches into the record.
///
/// A refused start-up raises no exception, so it is the one cause `ICSR` cannot
/// carry. It travels here rather than in an argument, which leaves
/// `silence_and_park` with a signature a forwarding frame folds into its
/// caller, and keeps the mute store the first memory access of every vector.
///
/// One word carries every arm of `main`. The arms run one after another and
/// each parks, so no two can write it. The domain field of the word says which
/// stage refused, and the three guards below share one domain, so under it the
/// cause byte is what says which guard.
///
/// The startup zero fill covers `.bss`, so the value out of reset is
/// `NO_REFUSAL` and only the arms of `main` that park write it. `.bss` sits
/// above STACK, where an overflowing stack does not reach, so the fault path
/// reads it back on that route as well.
static REFUSAL: AtomicU32 = AtomicU32::new(NO_REFUSAL);

const _: () = assert!
(
    XSMT_PIN == 7,
    "the register writers in the fault path and in the release gate name pin 7 \
     directly"
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
/// which is what a power on reset leaves it at. Every path of this binary that
/// seals a record ends parked in `wfi`, and a probe is the only reader
/// `FAULT_RECORD` has, so this bit is what carries the record, its refusal code
/// and its two guards off the board.
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

/// Keeps the parked core visible, arms `BusFault`, starts the clock, the output
/// transport and the input path, opens the release gate, and arms the block
/// structure that carries one into the other.
///
/// `keep_core_visible_in_sleep` runs first, ahead of every arm below that can
/// park, so none of them takes the core off the debug port.
///
/// PM0253 section 2.5.2 escalates a fault to `HardFault` when the handler for
/// that fault is disabled, and both vectors of this binary mute, so a bus error
/// reaches the mute armed or not. What arming `SHCSR.BUSFAULTENA` buys is the
/// escalation left over the handler that runs. The same section exempts the
/// stack push that enters an enabled `BusFault` handler from escalation and
/// sends a fault raised inside that handler on to `HardFault`, which mutes as
/// well, while section 2.5.5 locks the core up on a fault taken inside the
/// `HardFault` handler. So the mute gets two attempts armed and one disabled.
/// It goes up before the clock, so the bring-up runs with the second one in
/// place.
///
/// Every arm below ends in silence, because no state of this binary means "the
/// guard is absent" or "the clock is close enough". The waits are sized for the
/// clock the part boots on, which is the one this function runs at, since
/// nothing here moves the system clock off the internal oscillator.
///
/// Every arm below names its cause. Each writes its code to `REFUSAL` before it
/// enters the fault path, so a board that parks silent on the bench says which
/// stage refused and what it refused on, rather than only that it refused. The
/// three guards here carry a domain of their own, because a handle that comes
/// back taken and a `BUSFAULTENA` that does not read back armed leave the same
/// registers behind as a panic does.
///
/// The transport takes the witness the clock bring-up returns by reference,
/// which is what leaves the start order to the compiler: RM0433 requires the
/// kernel clock present on the interface before its enable bit is set, and a
/// refused clock builds no witness to hand over.
///
/// The transport is what puts the master clock, the bit clock, the frame clock
/// and the two data lines on the header. It configures PE2 to PE6 and leaves
/// PE7 alone, so the converter mute is still on its pull-down when it returns,
/// and it leaves both buffers holding zeros.
///
/// A refused transport names where it refused, the bring-up sequence, the plan,
/// one of the two sub-blocks or one of the two streams, and what that place
/// refused with. The registers it leaves behind say more still, and a probe
/// reads them in place, but only once a person knows a stage refused and which
/// one.
///
/// The input path comes up between the transport and the gate, on the same
/// witness. It parks a silent crossover chain in the memory the carry reads,
/// configures the receiving sub-block, its stream and PD11, and starts that
/// stream where the shift the two read pointers leave lands in the band a carry
/// can work in. It then publishes the built chain over the silent one. It
/// writes no output buffer and leaves PE7 alone, so the gate that follows still
/// finds the buffers holding zeros, and what it returns is the witness of that
/// publication.
///
/// The release gate takes that witness and the clock one by reference and reads
/// neither. It watches the transfers, puts PE7 under the port and raises it,
/// and holds the buffers of zeros over the converter unmute ramp. This is the
/// first time this firmware makes the machine audible, and the permit it
/// returns is what the tone write needs, so nothing non-zero can reach the
/// converters ahead of it.
///
/// The control link comes up after the tone. It configures USART3 on PB10 and
/// PB11 and refuses nothing: the input bring-up parked the link state at a
/// gain of zero, so a receiver that never delivers a volume leaves the carry
/// silent. The served handler polls that receiver once per block, and no
/// interrupt serves it.
///
/// The arming runs last and takes that permit by value. It waits for the tone
/// to travel a whole lap of the receiving buffer, then enables the two transfer
/// events, which is the one door the block structure comes through. Nothing in
/// this binary can enable those events without the permit, so the buffers the
/// converters read carry zeros until the gate has run and the tone the gate
/// permitted after that.
#[entry]
fn main() -> !
{
    keep_core_visible_in_sleep();

    let Some(mut core) = Peripherals::take()
    else
    {
        REFUSAL.store
        (
            postmortem::startup_refusal(StartupFault::CoreHandleTaken),
            Ordering::Relaxed
        );
        silence_and_park()
    };

    core.SCB.enable(Exception::BusFault);

    if !core.SCB.is_enabled(Exception::BusFault)
    {
        REFUSAL.store
        (
            postmortem::startup_refusal(StartupFault::BusFaultNotArmed),
            Ordering::Relaxed
        );
        silence_and_park()
    }

    let Some(part) = device::Peripherals::take()
    else
    {
        REFUSAL.store
        (
            postmortem::startup_refusal(StartupFault::DeviceHandleTaken),
            Ordering::Relaxed
        );
        silence_and_park()
    };

    let audio_clock = match clock::start(&part.RCC, BOOT_CORE_CLOCK_HZ)
    {
        Ok(witness) => witness,
        Err(fault) =>
        {
            REFUSAL.store(postmortem::clock_refusal(fault), Ordering::Relaxed);
            silence_and_park()
        }
    };

    passthrough::prepare(&part.RCC);

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

    if let Err(fault) = transport
    {
        REFUSAL.store(postmortem::transport_refusal(fault), Ordering::Relaxed);
        silence_and_park()
    }

    let mut input = passthrough::observe(&part.SAI2, &part.DMA1, &part.DMAMUX1, &part.GPIOD);

    let filters = match passthrough::start(&audio_clock, &mut input, BOOT_CORE_CLOCK_HZ)
    {
        Ok(witness) => witness,
        Err(fault) =>
        {
            REFUSAL.store(postmortem::passthrough_refusal(fault), Ordering::Relaxed);
            silence_and_park()
        }
    };

    let released = release::start
    (
        &audio_clock,
        &filters,
        &part.RCC,
        &part.SAI1,
        &part.DMA1,
        &part.DMAMUX1,
        &part.GPIOE,
        BOOT_CORE_CLOCK_HZ
    );

    let permit = match released
    {
        Ok(permit) => permit,
        Err(fault) =>
        {
            REFUSAL.store(postmortem::release_refusal(fault), Ordering::Relaxed);
            silence_and_park()
        }
    };

    transport::write_tone(&permit);

    link::start(&part.RCC, &part.USART3, &part.GPIOB);

    let armed = passthrough::start_blocks(&audio_clock, &mut input, permit, BOOT_CORE_CLOCK_HZ);

    if let Err(fault) = armed
    {
        REFUSAL.store(postmortem::passthrough_refusal(fault), Ordering::Relaxed);
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
/// a read-modify-write. Which of the two steps drives the pad depends on how
/// far the start-up got, and there are three answers. Past the release gate PE7
/// is an output held high, and the store above is what takes it down. Short of
/// the gate the pad is in the mode a reset left it in, the 10 k pull-down is
/// what holds XSMT low, and this step is what puts a driver behind that level.
/// Inside the gate, between the pad going under the port and the gate refusing
/// or the raise landing, it is an output already held low and both steps find
/// it where they want it. The store comes first in all three, which is what
/// fixes the order: the transport and the gate both set `GPIOEEN`, so on every
/// path where the converter clocks are running that store reaches `BSRR` rather
/// than being dropped, ahead of the read this step performs.
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
/// `REFUSAL` is the one cause `ICSR` cannot carry, since a refused start-up
/// raises no exception. This routine takes no argument, which is what
/// lets the compiler fold a forwarding handler into the trampoline above it and
/// leaves one frame push on each side of the vector rather than two. The code
/// is read from `.bss` here, after the mute store, so the mute stays the first
/// memory access of the path.
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

    cortex_m::interrupt::disable();
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
        let record = FaultRecord::new(&registers, REFUSAL.load(Ordering::Relaxed))
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

/// Carries one half of the received buffer into the two output buffers.
///
/// This is the one vector of this binary that does not silence the machine on
/// entry, and the reason it does not is that the block structure of the input
/// path is armed to raise it twice a lap of the buffer.
///
/// What it does with an entry it was not armed for is refuse it and take the
/// fault path, which is the path `DefaultHandler` takes for an interrupt that
/// has no handler of its own.
/// `pulsar_lib::passthrough::next_block` is where that decision lives, and it
/// refuses an entry carrying neither transfer event flag, one carrying both, a
/// receiving sub-block or stream reporting an alarm, a counter reading past its
/// buffer, and a read pointer standing where the carry would write. Nothing has
/// been written when it refuses, so the machine goes silent on the entry rather
/// than on the block after it.
///
/// The flag of the half being served goes down before the carry runs. Left
/// standing it would raise the line again the moment this returns and the core
/// would re-enter for ever, on audio that stopped moving with the converters
/// still unmuted, and nothing in this binary bounds that: the watchdog that
/// would is not here yet.
///
/// `REFUSAL` carries the code as every arm of the entry function does. The arms
/// all park, and this handler runs only once the last of them has been passed,
/// so no two writers of that word can exist at one time.
#[interrupt]
fn DMA_STR2()
{
    if let Err(fault) = passthrough::serve_event()
    {
        REFUSAL.store(postmortem::passthrough_refusal(fault), Ordering::Relaxed);
        silence_and_park()
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
/// or a debugger halt. Lockup leaves the port alone, so no instruction of this
/// handler runs and only a reset hands PE7 back to the 10 k pull-down. Past the
/// release gate that pin is an output held high, so lockup leaves the machine
/// audible for as long as the board stays powered. The watchdog that would
/// bound it is not in this binary.
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
