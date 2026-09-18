//! Control link of the processing board, at the register.
//!
//! `pulsar_lib::link` holds the rate, the pin function, the reader of the
//! frames, the clock the volume ramp runs on and the link loss. This module is
//! the register block of USART3, the two pins it runs on, and the memory the
//! link state is parked in.
//!
//! It touches neither PE7 nor XSMT and enables no interrupt. The served handler
//! polls the receiver once per block through `Port`, and nothing else reads
//! it.
//!
//! # What a receiver that never came up costs
//!
//! Nothing but the volume. The link state builds at a gain of exactly zero, and
//! a receiver that is not enabled reports no character, so the carry runs on
//! silence. No step of this module can refuse, and none of them reaches the
//! fault path.

use core::mem::MaybeUninit;
use core::ptr;
use pulsar_lib::link::
{
    ControlLink,
    ControlPort,
    LINK_BAUD_DIVISOR,
    LINK_KERNEL_CLOCK_SELECT,
    LINK_PIN_FUNCTION,
    LINK_RX_PULL,
    LinkedGain,
    PortStatus,
};
use stm32h7::stm32h743v::gpiob::afrh::ALTERNATE_FUNCTION;
use stm32h7::stm32h743v::gpiob::pupdr::PULL;
use stm32h7::stm32h743v::rcc::d2ccip2r::USART234578SEL;
use stm32h7::stm32h743v::{GPIOB, RCC, USART3};

use crate::passthrough::{PIN_ALTERNATE, PIN_NO_PULL};

// The pin function, the pull and the kernel clock the plan names are pinned
// against the encodings the peripheral crate carries for them, for the reason
// the input path pins its own. None of the three is read back.
const _: () = assert!
(
    LINK_PIN_FUNCTION == ALTERNATE_FUNCTION::Af7 as u8
        && LINK_RX_PULL == PULL::PullUp as u8
        && LINK_KERNEL_CLOCK_SELECT == USART234578SEL::HsiKer as u8,
    "the pin function, the pull and the kernel clock encode the way AFR, PUPDR \
     and USART234578SEL do"
);

/// State of the control link the served handler keeps between two blocks.
///
/// `.axisram` is a `NOLOAD` output section, so the startup sequence neither
/// copies nor zeroes what lands here and `park` is what writes it. The name is
/// unmangled so that one string identifies it in `llvm-nm` and in a debugger.
///
/// It sits beside the filter chain, and for the same reason: a static in the
/// tightly coupled memory would move the refusal word and the fault record.
#[expect
(
    unsafe_code,
    reason = "the link state is placed by section and named for a debugger"
)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".axisram.CONTROL_LINK")]
static mut CONTROL_LINK: MaybeUninit<ControlLink> = MaybeUninit::uninit();

/// The receiver of the control link.
pub(crate) struct Port<'a>
{
    usart: &'a USART3,
}

impl ControlPort for Port<'_>
{
    /// Reads `RXFNE`, `ORE`, `NE`, `FE` and `PE` in one read of `USART_ISR`.
    ///
    /// RM0433 section 48.7.9: with the FIFO enabled, bit 5 is `RXFNE`, and
    /// `NE`, `FE` and `PE` belong to the character at the head of the FIFO.
    #[expect
    (
        clippy::inline_always,
        reason = "the served handler reaches this once per character, and the \
                  fold keeps its closure one frame, which is what the build \
                  gate reads"
    )]
    #[inline(always)]
    fn status(&self) -> PortStatus
    {
        let isr = self.usart.isr().read();

        PortStatus
        {
            data_ready: isr.rxne().bit_is_set(),
            overrun: isr.ore().bit_is_set(),
            noise: isr.nf().bit_is_set(),
            framing: isr.fe().bit_is_set(),
            parity: isr.pe().bit_is_set(),
        }
    }

    /// Reads `USART_RDR`, which frees the head of the FIFO.
    ///
    /// Eight data bits, so the ninth bit of the field reads zero and the
    /// narrowing drops nothing.
    #[expect
    (
        clippy::cast_possible_truncation,
        reason = "the frame carries eight data bits, so the field holds a byte"
    )]
    #[expect
    (
        clippy::inline_always,
        reason = "the served handler reaches this once per character, and the \
                  fold keeps its closure one frame, which is what the build \
                  gate reads"
    )]
    #[inline(always)]
    fn take_byte(&mut self) -> u8
    {
        self.usart.rdr().read().rdr().bits() as u8
    }

    /// Writes `ORECF`, `NECF`, `FECF` and `PECF` into `USART_ICR`.
    ///
    /// RM0433 section 48.7.11 makes each bit write one to clear, so this store
    /// reaches those four flags and no other.
    #[expect
    (
        clippy::inline_always,
        reason = "the served handler reaches this on a line error, and the \
                  fold keeps its closure one frame, which is what the build \
                  gate reads"
    )]
    #[inline(always)]
    fn clear_errors(&mut self)
    {
        self.usart.icr().write(|w| w
            .orecf().clear_bit_by_one()
            .ncf().clear_bit_by_one()
            .fecf().clear_bit_by_one()
            .pecf().clear_bit_by_one());
    }
}

/// Parks a link at power up, which carries a gain of exactly zero.
///
/// The caller runs this ahead of the write that unmasks the served line, so no
/// entry of the handler finds the memory unwritten.
#[expect
(
    unsafe_code,
    reason = "the link state is reached by raw pointer, since the handler that \
              reads it holds no reference this function could borrow from"
)]
pub(crate) fn park()
{
    // SAFETY: nothing else touches the link state while this runs. The caller
    // stands ahead of the unmask that puts the handler on the vector, and the
    // handler is the only other access.
    unsafe
    {
        ptr::write_volatile
        (
            (&raw mut CONTROL_LINK).cast::<ControlLink>(),
            ControlLink::at_power_up()
        );
    }
}

/// Lends the parked link and the receiver to `serve` for one block.
///
/// The handler is the only caller, and it steals the register block the way it
/// steals the input path.
#[expect
(
    unsafe_code,
    reason = "the handler reaches the receiver by stealing it and the link state \
              by raw pointer, since neither has an owner a vector can borrow from"
)]
pub(crate) fn with_gain<R, F>(serve: F) -> R
where
    F: FnOnce(&mut LinkedGain<'_, Port<'_>>) -> R,
{
    // SAFETY: the entry function parks in `wfi` once the block structure is
    // armed and touches this block no more, and the served handler is the only
    // one this firmware has, so no other access falls between its reads and
    // writes.
    let usart = unsafe { USART3::steal() };

    // SAFETY: `passthrough::start` parks the link before it returns, and the
    // arming unmasks the served line after that, so the memory holds a link
    // before this can run. Thread mode touches it no more once the block
    // structure is armed, and this is the only handler this firmware serves,
    // so this reference is the only one that exists while it lives.
    let link = unsafe { &mut *(&raw mut CONTROL_LINK).cast::<ControlLink>() };

    let mut port = Port { usart: &usart };
    let mut gain = LinkedGain::new(link, &mut port);

    serve(&mut gain)
}

/// Brings the receiver and the transmitter of the control link up.
///
/// The bus clocks go up first, each read back as RM0433 prescribes for the
/// clock enabling delay, then the kernel clock is selected, which section
/// 48.5.6 requires ahead of `UE`. The interface is configured while `UE` is
/// clear, since section 48.7.5 makes `USART_BRR` writable only then, and the
/// pins go to the alternate function last, so PB10 only drives once the
/// transmitter holds the line idle.
///
/// Nothing here is read back and nothing here refuses: a link that does not
/// come up leaves the volume at zero, which is silence.
///
/// # Why this is not inlined
///
/// The build gate reads the entry function to check that each arm that parks
/// stores its own cause into the refusal word. Kept out of line, this call
/// adds one branch there and takes no register from the arms around it.
#[inline(never)]
pub(crate) fn start(rcc: &RCC, usart: &USART3, port: &GPIOB)
{
    rcc.ahb4enr().modify(|_, w| w.gpioben().set_bit());
    let _ = rcc.ahb4enr().read().gpioben().bit_is_set();

    rcc.apb1lenr().modify(|_, w| w.usart3en().set_bit());
    let _ = rcc.apb1lenr().read().usart3en().bit_is_set();

    select_kernel_clock(rcc);

    // RM0433 section 48.7.1 resets every field of CR1 to zero, which is one
    // start bit, eight data bits, no parity and oversampling by 16, with UE
    // clear. CR2 and CR3 at their reset values add one stop bit and the three
    // sample majority vote.
    usart.cr1().reset();
    usart.cr2().reset();
    usart.cr3().reset();
    usart.presc().reset();
    usart.brr().write(|w| w.brr().set(LINK_BAUD_DIVISOR));

    // FIFOEN is writable only while UE is clear, so it lands in a write of its
    // own ahead of the enable.
    usart.cr1().write(|w| w.fifoen().set_bit());
    usart.cr1().modify(|_, w| w.ue().set_bit());
    usart.cr1().modify(|_, w| w.te().set_bit().re().set_bit());

    open_pins(port);
}

/// Selects `hsi_ker_ck` as the kernel clock of USART3.
#[expect
(
    unsafe_code,
    reason = "the selection field takes raw bits in the peripheral crate, two \
              of its values being reserved"
)]
fn select_kernel_clock(rcc: &RCC)
{
    // SAFETY: RM0433 section 8.7.21 defines 011 over the three bits of the
    // field, and the assertion at the head of this module ties the constant
    // to that variant.
    rcc.d2ccip2r().modify(|_, w| unsafe
    {
        w.usart234578sel().bits(LINK_KERNEL_CLOCK_SELECT)
    });
}

/// Puts PB10 and PB11 on the USART3 function, with a pull-up on the receive
/// pin.
///
/// The pull holds the line at its idle level while no sender drives it, so a
/// cable left off reads as a quiet line rather than as noise. The mode write
/// comes last, so the pads stay analog until the rest is in place.
#[expect
(
    unsafe_code,
    reason = "the pull field is the one of the three whose writer the \
              peripheral crate leaves unsafe, its fourth value being reserved"
)]
fn open_pins(port: &GPIOB)
{
    // SAFETY: RM0433 section 11.4.4 gives the field two bits and reserves the
    // value 3, and the two values written here are 0 and 1.
    port.pupdr().modify(|_, w| unsafe
    {
        w.pupdr10().bits(PIN_NO_PULL).pupdr11().bits(LINK_RX_PULL)
    });

    port.afrh().modify(|_, w| w
        .afr10().set(LINK_PIN_FUNCTION)
        .afr11().set(LINK_PIN_FUNCTION));

    port.moder().modify(|_, w| w
        .moder10().set(PIN_ALTERNATE)
        .moder11().set(PIN_ALTERNATE));
}
