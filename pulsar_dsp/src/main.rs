//! Audio processing firmware for the STM32H743 board.
//!
//! Every crossover, equalisation and protection stage of the loudspeaker runs
//! here. The machine carries no analog filter, so a fault in this binary
//! reaches the drivers directly.
//!
//! The converters come out of reset muted, held there by the pull-down on their
//! XSMT pin. Nothing in this binary raises XSMT, so they stay muted.
//!
//! Three entry points lead back to silence: a Rust panic, a hard fault, and any
//! exception or interrupt without a handler of its own. All three drive XSMT
//! low and park the core, because a fault that mutes only sometimes is not a
//! mute.

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::asm;
use cortex_m::interrupt;
use cortex_m_rt::{entry, exception};
use pulsar_lib::constants::{MAX_CORE_CLOCK_HZ, mute_hold_iterations};
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

const _: () = assert!
(
    XSMT_PIN == 7,
    "the register writers in the fault path name pin 7 directly"
);

#[entry]
fn main() -> !
{
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
/// The hold covers `MUTE_SEQUENCE_US`. Nothing in this binary starts or stops
/// the audio clocks, so it becomes load bearing once a reset path exists, and
/// the watchdog period is then derived from `MUTE_SEQUENCE_US` so its reset
/// cannot land inside the converter ramp.
#[expect
(
    unsafe_code,
    reason = "the fault path reaches the mute pin by raw register write"
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
/// A bad pointer or an unaligned access arrives here and never reaches the
/// panic handler, so this is the only thing between such a fault and a full
/// level buffer replaying into the drivers.
///
/// A stack overflow reaches it only while the exception entry can still stack
/// its frame. `trampoline = false` points the vector straight at this function
/// and drops the `ExceptionFrame` argument, but the compiler still emits a
/// frame push of its own, so the handler costs stack like any other. A stacking
/// error taken during hard fault entry escalates to lockup, where no handler
/// runs and the pin holds its reset high impedance state under the 10 k
/// pull-down. An MPU region marking the bottom of the stack as no-access turns
/// that case into a memory fault this handler catches.
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
/// running.
///
/// The interrupt number is discarded and no fault cause is latched, so
/// `DspState::Faulted` and every `Fault` variant of the control protocol name
/// states this binary cannot report.
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
