//! Audio kernel clock of the processing board, at the register.
//!
//! `pulsar_lib::clock` holds the plan, the bounds and the sequence. This module
//! is the register block that sequence runs on, and nothing more: it writes
//! `RCC`, reads it back, and hands the plan the fields it read.
//!
//! It touches no port pin. XSMT stays where the pull-down on it leaves it,
//! whichever way a bring-up ends.
//!
//! What carries this clock to a pin is the output transport, which enables the
//! audio interface and puts PE2 to PE6 on their alternate function. It runs
//! only on the witness returned here, so a run that ends in a refusal leaves
//! the kernel clock reaching nothing, even though the PLL can be left going.

use pulsar_lib::clock::
{
    AUDIO_KERNEL_PLL3_P,
    AUDIO_PLAN,
    ClockFault,
    ClockPlan,
    ClockReadback,
    ClockTree,
    ClockWaits,
    PLL_SOURCE_EXTERNAL,
    ReferenceRange,
    bring_up,
};
use stm32h7::stm32h743v::RCC;
use stm32h7::stm32h743v::rcc::d2ccip1r::SAI1SEL;
use stm32h7::stm32h743v::rcc::pllcfgr::PLL1RGE;
use stm32h7::stm32h743v::rcc::pllckselr::PLLSRC;

const _: () = assert!
(
    PLL_SOURCE_EXTERNAL == PLLSRC::Hse as u8,
    "the value the plan checks for is the one PLLSRC::Hse encodes"
);

const _: () = assert!
(
    AUDIO_KERNEL_PLL3_P == SAI1SEL::Pll3P as u8,
    "the value the plan checks for is the one SAI1SEL::Pll3P encodes"
);

// The peripheral crate gives PLL1RGE, PLL2RGE and PLL3RGE one enum and one
// writer type, so the values pinned here are the ones the PLL3 field carries.
const _: () = assert!
(
    ReferenceRange::Mhz1To2.bits() == PLL1RGE::Range1 as u8
        && ReferenceRange::Mhz2To4.bits() == PLL1RGE::Range2 as u8
        && ReferenceRange::Mhz4To8.bits() == PLL1RGE::Range4 as u8
        && ReferenceRange::Mhz8To16.bits() == PLL1RGE::Range8 as u8,
    "the reference range the plan declares encodes the way PLLxRGE does"
);

/// Witness that a read-back agreed with the plan when the clock came up.
///
/// `start` is the only thing that builds one, and it builds one only after
/// `Tree::read` reported the plan field for field. It attests that one
/// read-back, at one instant, and says nothing about the tree afterwards.
///
/// What `Tree::read` itself reports is not attested here. This crate is a
/// `[[bin]]` built for the part, so no host test reaches the register mapping,
/// and a wrong field in it produces a witness just the same. A bench probe is
/// what settles that.
///
/// A stage that needs the kernel clock takes this by reference, which leaves
/// the start order to the compiler rather than to a comment.
pub(crate) struct AudioClock
{
    /// The plan the read-back was compared against.
    plan: ClockPlan,
}

impl AudioClock
{
    /// Returns the plan the read-back was compared against.
    ///
    /// The output transport takes its frame length and its master clock
    /// divider off this, so the interface writes the chain that was validated
    /// rather than a second copy of the same figures.
    pub(crate) const fn plan(&self) -> ClockPlan
    {
        self.plan
    }
}

/// The clock tree of the part, seen through its register block.
struct Tree<'a>
{
    rcc: &'a RCC,
}

impl ClockTree for Tree<'_>
{
    fn stop_pll(&mut self)
    {
        self.rcc.cr().modify(|_, w| w.pll3on().clear_bit());
    }

    fn start_source(&mut self)
    {
        // HSEBYP is left alone. Its reset value takes the crystal, and the
        // read-back refuses the bypass, so a plan cannot silently run off an
        // external clock.
        self.rcc.cr().modify(|_, w| w.hseon().set_bit());
    }

    fn write_source_and_reference(&mut self, plan: &ClockPlan)
    {
        self.rcc.pllckselr().modify(|_, w| w
            .pllsrc().hse()
            .divm3().set(plan.pll().reference_divider_field()));
    }

    fn write_configuration(&mut self, plan: &ClockPlan)
    {
        self.rcc.pllcfgr().modify(|_, w| w
            .pll3rge().set(plan.pll().range().bits())
            .pll3vcosel().bit(plan.pll().band().selection_bit())
            .pll3fracen().clear_bit()
            .divp3en().set_bit());
    }

    fn write_fraction(&mut self, plan: &ClockPlan)
    {
        self.rcc.pll3fracr().modify(|_, w| w.fracn3().set(plan.pll().fraction_field()));
    }

    #[expect
    (
        unsafe_code,
        reason = "the two divider fields take raw bits in the peripheral crate"
    )]
    fn write_dividers(&mut self, plan: &ClockPlan)
    {
        self.rcc.pll3divr().modify(|_, w|
        {
            // SAFETY: both values come from a plan validate accepted, which
            // bounds the multiplier at 512 and the output divider at 128. The
            // fields encode one less, so they carry at most 0x1FF over nine
            // bits and 0x7F over seven, the widths RM0433 section 8.7.17 gives
            // them.
            unsafe
            {
                w.divn3().bits(plan.pll().multiplier_field())
                    .divp3().bits(plan.pll().output_divider_field())
            }
        });
    }

    fn latch_fraction(&mut self)
    {
        self.rcc.pllcfgr().modify(|_, w| w.pll3fracen().set_bit());
    }

    fn start_pll(&mut self)
    {
        self.rcc.cr().modify(|_, w| w.pll3on().set_bit());
    }

    fn select_audio_kernel(&mut self)
    {
        self.rcc.d2ccip1r().modify(|_, w| w.sai1sel().pll3_p());
    }

    /// Reads the six RCC registers the plan is checked against.
    ///
    /// `bring_up` polls through this, so one poll of a wait costs six loads
    /// from the register block rather than one core cycle.
    fn read(&self) -> ClockReadback
    {
        let control = self.rcc.cr().read();
        let selection = self.rcc.pllckselr().read();
        let configuration = self.rcc.pllcfgr().read();
        let dividers = self.rcc.pll3divr().read();
        let fraction = self.rcc.pll3fracr().read();
        let kernels = self.rcc.d2ccip1r().read();

        ClockReadback
        {
            source_bits: selection.pllsrc().bits(),
            source_ready: control.hserdy().bit_is_set(),
            source_bypassed: control.hsebyp().bit_is_set(),
            reference_divider_field: selection.divm3().bits(),
            multiplier_field: dividers.divn3().bits(),
            fraction_field: fraction.fracn3().bits(),
            output_divider_field: dividers.divp3().bits(),
            reference_range_bits: configuration.pll3rge().bits(),
            vco_band_bit: configuration.pll3vcosel().bit_is_set(),
            fraction_latched: configuration.pll3fracen().bit_is_set(),
            output_enabled: configuration.divp3en().bit_is_set(),
            pll_on: control.pll3on().bit_is_set(),
            pll_ready: control.pll3rdy().bit_is_set(),
            audio_kernel_bits: kernels.sai1sel().bits(),
            pll1_on: control.pll1on().bit_is_set(),
            pll2_on: control.pll2on().bit_is_set(),
        }
    }
}

/// Brings the audio kernel clock up and proves the part took the plan.
///
/// `core_clock_hz` sizes the two waits, and naming a clock above the one the
/// core runs at only lengthens them.
///
/// The interface clock and the pins it drives are not touched here, so this
/// leaves the kernel clock ending inside the part.
///
/// # Errors
///
/// Every variant of `ClockFault`. A refusal reached after the PLL started
/// leaves it running, and it builds no witness, so the stage that would carry
/// the output to a pin cannot be reached and a caller that answers a refusal
/// by staying silent is silent.
pub(crate) fn start(rcc: &RCC, core_clock_hz: u32) -> Result<AudioClock, ClockFault>
{
    let mut tree = Tree { rcc };

    bring_up(&mut tree, &AUDIO_PLAN, &ClockWaits::for_core_clock(core_clock_hz))?;

    Ok(AudioClock { plan: AUDIO_PLAN })
}
