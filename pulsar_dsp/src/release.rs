//! Release of the converter mute line, at the register.
//!
//! `pulsar_lib::release` holds the terms, the sequence and the permit. This
//! module is the port pin that sequence drives and the delay it holds: it
//! writes PE7 low, hands it to the port driver, drives it high, and hands the
//! gate the transport to watch. The order those four run in belongs to the
//! gate, so a host test reaches it.
//!
//! One line drives XSMT on both converter modules, so a single register write
//! moves both and the two cannot disagree about being muted.
//!
//! This is the first and only place in this firmware that raises that pin.
//! Everything else that names it drives it low.

use cortex_m::asm;
use pulsar_lib::release::
{
    ConverterMute,
    ReleaseFault,
    ReleaseWaits,
    TonePermit,
    open,
};
use stm32h7::stm32h743v::gpioc::moder::MODE as PIN_MODE;
use stm32h7::stm32h743v::gpioc::ospeedr::OUTPUT_SPEED;
use stm32h7::stm32h743v::gpioc::otyper::OUTPUT_TYPE;
use stm32h7::stm32h743v::gpioc::pupdr::PULL;
use stm32h7::stm32h743v::{DMA1, DMAMUX1, GPIOE, RCC, SAI1};

use crate::clock::AudioClock;
use crate::passthrough::PublishedChain;
use crate::transport;

/// `MODER` value putting a pin on general purpose output. RM0433 section
/// 11.4.1.
const PIN_OUTPUT: u8 = 0b01;

/// `PUPDR` value arming the internal pull-down. RM0433 section 11.4.4.
///
/// Section 11.3.3 selects the pull whatever the direction of the pin, and
/// section 11.3.10 activates the resistors on the value of this register
/// alone, so this is armed under an output that drives it. The driver wins
/// while it drives, and the pull takes the pin back under an input or an
/// alternate function. It does not take it back under the analog mode a reset
/// leaves this port in: section 11.3.13 has the weak pull-up and pull-down
/// resistors disabled by hardware there, so what holds XSMT low on a pad that
/// drives nothing is the 10 k pull-down on the board and this field is not a
/// second one. It costs one field of one write.
const PIN_PULL_DOWN: u8 = 0b10;

/// `OSPEEDR` value the mute line is driven at.
///
/// The medium speed, and what picks it is the edge and not the frequency. The
/// PCM5102A datasheet 8.7 and 9.3.3 require a rise and a fall under 20 ns,
/// measured between a tenth and nine tenths of the supply, for the part to read
/// the transition as the mute request it is. Above that the datasheet specifies
/// nothing until the 6 ms that arms the undervoltage protection instead.
///
/// The STM32H743VI datasheet table 160 gives, at 3.3 V with 50 pF on the pin
/// and over the same tenth to nine tenths, 16.6 ns at the lowest setting and
/// 5.2 ns at this one. Its note 1 makes both figures guaranteed by design
/// rather than tested in production. The lowest leaves 17 percent under a bound
/// this pin has to hold on two converter inputs and the wiring between them, an
/// amount nothing here has measured. This setting leaves the load room to be
/// several times the 50 pF the figure is quoted at, and it is the last one the
/// table gives without the compensation cell this firmware does not enable.
///
/// The fall matters as much as the rise. Past this gate the pad keeps this
/// setting, so the fault path driving XSMT back low makes its edge at this
/// speed and not at the reset one.
///
/// A faster edge on a pin sitting beside five interface lines costs coupling.
/// This one changes level twice in a boot and once more on a fault, so it has
/// no repetition rate to couple, and it is the only pin of the port that can be
/// heard.
const MUTE_PIN_SPEED: u8 = 0b01;

// The mute pin is written and read back by nothing, here or in the plan. These
// assertions are the only check that stands between the three encodings and
// the pad, and the oscilloscope is what settles them.
const _: () = assert!
(
    PIN_OUTPUT == PIN_MODE::Output as u8
        && PIN_PULL_DOWN == PULL::PullDown as u8
        && MUTE_PIN_SPEED == OUTPUT_SPEED::MediumSpeed as u8,
    "the mute pin mode, pull and speed encode the way the port does"
);

const _: () = assert!
(
    OUTPUT_TYPE::PushPull as u8 == 0,
    "a cleared output type bit is the push-pull driver"
);

const _: () = assert!
(
    crate::XSMT_PIN == 7,
    "the register writers below name pin 7 directly"
);

/// The converter mute line at the register.
struct MuteLine<'a>
{
    port: &'a GPIOE,
}

impl ConverterMute for MuteLine<'_>
{
    /// Writes the reset half of `BSRR` for PE7.
    ///
    /// A store to `BSRR` needs no read-modify-write, so a concurrent write to
    /// the port cannot lose it, and it reaches the output data register
    /// whatever mode the pad is in.
    fn drive_low(&mut self)
    {
        self.port.bsrr().write(|w| w.br7().set_bit());
    }

    /// Puts PE7 on the port driver.
    ///
    /// The drive type, the speed and the pull come first, while the pad is
    /// still in the mode a reset left it in and drives nothing. `MODER` comes
    /// last, and that is the write that hands the pad to the driver, which
    /// finds the output data register already carrying the low the caller put
    /// there.
    ///
    /// The pull-down is armed under an output on purpose. RM0433 section
    /// 11.3.3 selects the pull whatever the direction of the pin, so the driver
    /// wins while it drives and the pull takes the pad back under an input or
    /// an alternate function. The 10 k pull-down on the board is what covers
    /// the analog mode, where section 11.3.13 disables the internal one.
    #[expect
    (
        unsafe_code,
        reason = "the pull field is the one of the three whose writer the \
                  peripheral crate leaves unsafe, its fourth value being \
                  reserved"
    )]
    fn take_output(&mut self)
    {
        self.port.otyper().modify(|_, w| w.ot7().clear_bit());
        self.port.ospeedr().modify(|_, w| w.ospeedr7().set(MUTE_PIN_SPEED));

        // SAFETY: RM0433 section 11.4.4 gives the field two bits and reserves
        // the value 3, which is why the peripheral crate leaves this writer
        // unsafe. The pull-down is 2 and is not the reserved one.
        self.port.pupdr().modify(|_, w| unsafe { w.pupdr7().bits(PIN_PULL_DOWN) });

        self.port.moder().modify(|_, w| w.moder7().set(PIN_OUTPUT));
    }

    fn drive_high(&mut self)
    {
        self.port.bsrr().write(|w| w.bs7().set_bit());
    }

    /// Holds the core for the converter mute sequence.
    ///
    /// The same count the fault path holds, so the window that covers a ramp
    /// down and the window that covers a ramp up cannot drift apart. It is a
    /// cycle count sized for the highest core clock the part runs at, and this
    /// binary stays on the one the part boots on, so the hold lasts several
    /// times the sequence it covers. Long is the safe direction for a hold over
    /// silence, and the cost is that it also carries the converter past the
    /// 1024 frame periods of zero data that arm its own analog mute.
    ///
    /// Nothing here stops the audio clocks: the converter counts its ramp in
    /// sample periods, so the frame has to keep going out through this.
    fn hold(&mut self)
    {
        asm::delay(crate::MUTE_HOLD_ITERATIONS);
    }
}

/// Enables the port clock the mute line is driven through.
///
/// RM0433, clock enabling delays: an enable command takes up to two periods of
/// the enabled clock to reach the peripheral, and until it has, a read of one
/// of its registers returns invalid data and a write is dropped. The prescribed
/// sequence reads the enable register back, which is what the volatile read
/// below is.
///
/// The output transport enables the same bit and is verified before this runs,
/// so this write changes nothing on the path that reaches here. It is made all
/// the same rather than inherited: a gate whose first write can be dropped is a
/// gate that reports a mute line it never armed, and the cost of saying so is
/// two instructions.
fn enable_port_clock(rcc: &RCC)
{
    rcc.ahb4enr().modify(|_, w| w.gpioeen().set_bit());
    let _ = rcc.ahb4enr().read().gpioeen().bit_is_set();
}

/// Raises the converter mute line once the transport has proved it runs.
///
/// `clock` is the witness that the audio kernel clock came up and read back as
/// planned, and `filters` the witness of the write that publishes the built
/// crossover chain. The gate reads neither. Taking them by reference is what
/// leaves the order to the compiler: a refused clock and a refused chain each
/// build no witness, so there is nothing to hand over, and this cannot run
/// ahead of either stage it depends on.
///
/// `core_clock_hz` sizes the window budget, and naming a clock above the one
/// the core runs at only lengthens it.
///
/// The plan comes off the same call the bring-up used, so the buffer length
/// the window is sized on is the buffer length the streams were armed with.
///
/// Once this returns, PE7 is an output held high, the converters are unmuted,
/// and the buffers still hold the silence the transport put in them. What
/// turns that into sound is `transport::write_tone`, which takes the permit
/// this returns.
///
/// # Errors
///
/// A `ReleaseFault` naming where the gate refused and, for every refusal but
/// the sequence one, whether the mute line had been raised when it was seen. A
/// refusal leaves PE7 low and builds no permit, so the caller answers by
/// staying silent.
///
/// # Why this is not inlined
///
/// The build gate reads the entry function to check that each arm that parks
/// stores its own cause into the refusal word, and it resolves the address of
/// that word out of the register the store goes through. Folded into the entry
/// function, this sequence takes enough registers that the one holding that
/// address is recycled and the last arm rebuilds it in a register the function
/// also uses for a dozen other addresses. The gate then cannot say where the
/// store lands and refuses rather than guessing, which is the behaviour it
/// documents. Keeping this out of line keeps the arms readable. It costs one
/// call on a path taken once.
#[expect
(
    clippy::too_many_arguments,
    reason = "the eight are two witnesses that order this call, the five \
              register blocks it reaches and the core clock frequency that \
              sizes its window, and a type grouping the blocks would leave \
              the blocks this call reaches unnamed where it is read"
)]
#[inline(never)]
pub(crate) fn start
(
    clock: &AudioClock,
    filters: &PublishedChain,
    rcc: &RCC,
    sai: &SAI1,
    dma: &DMA1,
    mux: &DMAMUX1,
    port: &GPIOE,
    core_clock_hz: u32
) -> Result<TonePermit, ReleaseFault>
{
    enable_port_clock(rcc);

    let plan = transport::plan(clock);
    let mut interface = transport::observe(sai, dma, mux, port);
    let mut line = MuteLine { port };

    open
    (
        &mut interface,
        &mut line,
        clock,
        filters,
        &plan,
        ReleaseWaits::for_transport(&plan, core_clock_hz)
    )
}
