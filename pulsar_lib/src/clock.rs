//! Audio clock plan of the processing board, and the proof the part took it.
//!
//! A plan is data: the crystal frequency, then every divider between it and the
//! frame the converters are clocked by. Each frequency it reports is computed
//! from those dividers as an exact ratio of integers, so no derived figure is
//! written down twice.
//!
//! Nothing here touches a peripheral. `ClockTree` is the interface a register
//! block implements, and `bring_up` is the sequence run over it.
//!
//! # Where the bounds come from
//!
//! RM0433 section 8.7.18 gives the PLL reference range as 1 to 16 MHz with no
//! condition attached. The STM32H743VI datasheet splits it by VCO band, 2 to
//! 16 MHz in the wide band and 1 to 2 MHz in the medium band, and forbids the
//! sigma-delta modulator in the medium band outright. A plan checked against
//! the reference manual alone can therefore be illegal on silicon, so the
//! bounds below are the datasheet ones.
//!
//! The VCO ceiling is the one place the two datasheet revisions disagree: rev V
//! table 147 gives 960 MHz and rev Y table 50 gives 836 MHz. The lower figure
//! is the bound here, so a plan this module accepts is accepted on either die.
//!
//! # Why a lock bit is not a check
//!
//! RM0433 defines `PLLxRDY` as the output sitting within 2 per cent of what the
//! dividers ask for. It says nothing about which oscillator feeds those
//! dividers, nor about the declared reference range matching the reference the
//! PLL actually sees. A PLL fed from the wrong oscillator, running its VCO past
//! the band ceiling, locks and raises the bit just the same.
//!
//! `ClockPlan::verify` is what covers that. It reads back the register fields
//! the output frequency depends on, PLLSRC first, and compares each against the
//! plan. PLLSRC resets to the internal oscillator and a read-modify-write
//! preserves that silently, which makes it the field most easily left wrong and
//! the one a lock bit hides.
//!
//! # The assumption the read-back does not cover
//!
//! `source_hz` is the one input of the frequency no register reports. Swap the
//! crystal for an 8 MHz part and every field still reads back as the plan, so
//! `verify` returns `Ok` over a frame rate of 14112 Hz. The figure is a
//! measurement, not a read-back: it was counted on MCO1 with the output set to
//! `hse_ck`.

use crate::constants::{MICROSECONDS_PER_SECOND, SAMPLE_RATE_HZ};

/// Frequency of the crystal wired to the oscillator pins, in hertz.
///
/// Counted on the master clock output of the board.
pub const CRYSTAL_HZ: u32 = 25_000_000;

/// `PLLSRC` value selecting the external oscillator.
///
/// RM0433 section 8.7.11: 00 internal, 01 low power internal, 10 external, 11
/// no clock. The field resets to 00.
pub const PLL_SOURCE_EXTERNAL: u8 = 0b10;

/// `SAI1SEL` value selecting `pll3_p_ck` as the audio kernel clock.
///
/// RM0433 section 8.7.20.
pub const AUDIO_KERNEL_PLL3_P: u8 = 2;

/// Denominator of the fractional part of the VCO multiplier.
///
/// RM0433 section 8.7.18: the multiplier is `DIVN + FRACN / 2^13`.
const FRACTION_SCALE: u64 = 8_192;

/// Master clock periods in one audio frame.
///
/// RM0433, SAI clock generator programming with MCLK generation: with the
/// oversampling ratio cleared, the frame rate is the kernel clock divided by
/// `MCKDIV` and by this.
const MASTER_CLOCK_RATIO: u64 = 256;

/// Lowest reference divider the field encodes. RM0433 section 8.7.11.
const REFERENCE_DIVIDER_MIN: u8 = 1;

/// Highest reference divider the field encodes. RM0433 section 8.7.11.
const REFERENCE_DIVIDER_MAX: u8 = 63;

/// Lowest VCO multiplier the field encodes. RM0433 section 8.7.17.
const MULTIPLIER_MIN: u16 = 4;

/// Highest VCO multiplier the field encodes. RM0433 section 8.7.17.
const MULTIPLIER_MAX: u16 = 512;

/// Highest fraction the field encodes. RM0433 section 8.7.18.
const FRACTION_MAX: u16 = 8_191;

/// Lowest output divider the field encodes. RM0433 section 8.7.17.
const OUTPUT_DIVIDER_MIN: u8 = 1;

/// Highest output divider the field encodes. RM0433 section 8.7.17.
const OUTPUT_DIVIDER_MAX: u8 = 128;

/// Lowest master clock divider the field encodes.
///
/// RM0433 makes `MCKDIV` of 0 behave as 1, so a plan names 1 rather than 0 and
/// the two encodings cannot disagree. No plan reaches it: the duty cycle check
/// further down refuses every odd divider, which puts 2 at the bottom.
const MASTER_DIVIDER_MIN: u8 = 1;

/// Highest master clock divider the field encodes.
const MASTER_DIVIDER_MAX: u8 = 63;

/// Shortest frame the interface accepts with the master clock generated.
///
/// RM0433, SAI clock generator: `FRL + 1` runs from 8 to 256 and must be a
/// power of two.
const FRAME_BITS_MIN: u16 = 8;

/// Longest frame the interface accepts with the master clock generated.
const FRAME_BITS_MAX: u16 = 256;

/// Lowest crystal frequency the oscillator drives.
///
/// STM32H743VI datasheet table 141, 4 to 48 MHz.
const CRYSTAL_MIN_HZ: u32 = 4_000_000;

/// Highest crystal frequency the oscillator drives.
const CRYSTAL_MAX_HZ: u32 = 48_000_000;

/// Highest kernel clock the audio interface accepts, in hertz.
///
/// Two ceilings stand over `pll3_p_ck` on its way to the interface, and both
/// scale with the voltage scale the part runs on. The STM32H743VI datasheet
/// table 147 gives 200 MHz on a PLL P output at the lowest scale, and RM0433
/// table 59 gives 75 MHz at the output of the SAI1 kernel clock multiplexer at
/// that same scale. The part boots on the lowest scale and no Pulsar firmware
/// raises it, so the lower of the two is the bound that holds.
const AUDIO_KERNEL_MAX_HZ: u64 = 75_000_000;

/// Margin a plan keeps between the VCO and either edge of its band, in hertz.
///
/// This is a design margin rather than a datasheet bound. It holds a plan three
/// per cent of the band away from the edges, where the two datasheet revisions
/// disagree about the ceiling and where the part is characterised least. A
/// crystal tolerance does not enter: 50 ppm on 25 MHz walks the VCO by 24 kHz,
/// three orders of magnitude under this.
const VCO_MARGIN_HZ: u64 = 25_000_000;

/// Returns `SAMPLE_RATE_HZ` scaled by `multiple`.
const fn nominal_hz(multiple: u64) -> u64
{
    (SAMPLE_RATE_HZ as u64).saturating_mul(multiple)
}

/// Bits in one audio frame, which the converters take as their bit clock line.
///
/// PCM5102A datasheet table 11 carries the 64 fs line.
const NOMINAL_FRAME_BITS: u16 = 64;

/// Returns `bits` as a multiple the ratio arithmetic works in.
const fn widen(bits: u16) -> u64
{
    bits as u64
}

/// Master clock the converters are specified for, in hertz.
///
/// PCM5102A datasheet table 10 carries the 256 fs line, which at 44.1 kHz is
/// 11.2896 MHz.
const NOMINAL_MASTER_CLOCK_HZ: u64 = nominal_hz(MASTER_CLOCK_RATIO);

/// Bit clock the converters are specified for, in hertz.
///
/// 64 fs, which at 44.1 kHz is 2.8224 MHz.
const NOMINAL_BIT_CLOCK_HZ: u64 = nominal_hz(widen(NOMINAL_FRAME_BITS));

/// VCO band a PLL runs in, and the `PLLxVCOSEL` bit that selects it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VcoBand
{
    /// 192 to 836 MHz. The only band the sigma-delta modulator runs in.
    Wide,
    /// 150 to 420 MHz, integer multipliers only.
    Medium,
}

impl VcoBand
{
    /// Returns the `PLLxVCOSEL` bit that selects the band.
    #[must_use]
    pub const fn selection_bit(self) -> bool
    {
        matches!(self, Self::Medium)
    }

    /// Returns whether the band accepts a non-zero fraction.
    ///
    /// STM32H743VI datasheet table 148 marks sigma-delta mode forbidden in the
    /// medium band.
    #[must_use]
    pub const fn accepts_fraction(self) -> bool
    {
        matches!(self, Self::Wide)
    }

    /// Returns the lowest reference the band accepts, in hertz.
    const fn reference_min_hz(self) -> u64
    {
        match self
        {
            Self::Wide => 2_000_000,
            Self::Medium => 1_000_000,
        }
    }

    /// Returns the highest reference the band accepts, in hertz.
    const fn reference_max_hz(self) -> u64
    {
        match self
        {
            Self::Wide => 16_000_000,
            Self::Medium => 2_000_000,
        }
    }

    /// Returns the lowest VCO frequency the band guarantees, in hertz.
    const fn vco_min_hz(self) -> u64
    {
        match self
        {
            Self::Wide => 192_000_000,
            Self::Medium => 150_000_000,
        }
    }

    /// Returns the highest VCO frequency the band guarantees, in hertz.
    const fn vco_max_hz(self) -> u64
    {
        match self
        {
            Self::Wide => 836_000_000,
            Self::Medium => 420_000_000,
        }
    }

    /// Returns the lowest output the band guarantees, in hertz.
    const fn output_min_hz(self) -> u64
    {
        match self
        {
            Self::Wide => 1_500_000,
            Self::Medium => 1_170_000,
        }
    }
}

/// Reference range a plan declares, and the `PLLxRGE` value that carries it.
///
/// RM0433 section 8.7.12 names the four windows. Declaring one the reference
/// does not fall into misconfigures the phase detector, and no status bit
/// reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceRange
{
    /// 1 to 2 MHz.
    Mhz1To2,
    /// 2 to 4 MHz.
    Mhz2To4,
    /// 4 to 8 MHz.
    Mhz4To8,
    /// 8 to 16 MHz.
    Mhz8To16,
}

impl ReferenceRange
{
    /// Returns the `PLLxRGE` value.
    #[must_use]
    pub const fn bits(self) -> u8
    {
        match self
        {
            Self::Mhz1To2 => 0b00,
            Self::Mhz2To4 => 0b01,
            Self::Mhz4To8 => 0b10,
            Self::Mhz8To16 => 0b11,
        }
    }

    /// Returns the lowest reference the window covers, in hertz.
    const fn min_hz(self) -> u64
    {
        match self
        {
            Self::Mhz1To2 => 1_000_000,
            Self::Mhz2To4 => 2_000_000,
            Self::Mhz4To8 => 4_000_000,
            Self::Mhz8To16 => 8_000_000,
        }
    }

    /// Returns the highest reference the window covers, in hertz.
    ///
    /// The windows meet at their edges and RM0433 assigns neither edge, so a
    /// reference landing exactly on one satisfies both neighbours.
    const fn max_hz(self) -> u64
    {
        match self
        {
            Self::Mhz1To2 => 2_000_000,
            Self::Mhz2To4 => 4_000_000,
            Self::Mhz4To8 => 8_000_000,
            Self::Mhz8To16 => 16_000_000,
        }
    }
}

/// An exact frequency, held as a ratio of integers in hertz.
///
/// No division chain of this part lands on a whole number of hertz, so a
/// rounded figure would carry an error the checks are meant to measure. The
/// ratio is reduced on construction, which makes equality meaningful.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frequency
{
    numerator: u64,
    denominator: u64,
}

impl Frequency
{
    /// The frequency of a chain holding a divider of zero.
    ///
    /// Every predicate below refuses it and `as_hz` reports it as not a number,
    /// so a plan carrying one fails whichever bound it is held to.
    pub const UNDEFINED: Self = Self { numerator: 0, denominator: 0 };

    /// Builds the reduced ratio `numerator / denominator`.
    const fn new(numerator: u64, denominator: u64) -> Self
    {
        if denominator == 0
        {
            return Self::UNDEFINED;
        }

        let divisor = greatest_common_divisor(numerator, denominator);

        match (numerator.checked_div(divisor), denominator.checked_div(divisor))
        {
            (Some(reduced_numerator), Some(reduced_denominator)) => Self
            {
                numerator: reduced_numerator,
                denominator: reduced_denominator,
            },
            _ => Self::UNDEFINED,
        }
    }

    /// Returns the ratio scaled by `multiplier` over `divisor`.
    const fn scaled(self, multiplier: u64, divisor: u64) -> Self
    {
        match self.numerator.checked_mul(multiplier)
        {
            None => Self::UNDEFINED,
            Some(numerator) => match self.denominator.checked_mul(divisor)
            {
                None => Self::UNDEFINED,
                Some(denominator) => Self::new(numerator, denominator),
            },
        }
    }

    /// Returns the numerator of the reduced ratio.
    #[must_use]
    pub const fn numerator(self) -> u64
    {
        self.numerator
    }

    /// Returns the denominator of the reduced ratio.
    #[must_use]
    pub const fn denominator(self) -> u64
    {
        self.denominator
    }

    /// Returns whether a division chain produced this frequency.
    #[must_use]
    pub const fn is_defined(self) -> bool
    {
        self.denominator != 0
    }

    /// Returns the frequency in hertz, and not a number when it is undefined.
    #[must_use]
    #[expect
    (
        clippy::cast_precision_loss,
        reason = "the ratio is a report for a person, and the exact form stays \
                  available in numerator and denominator"
    )]
    pub fn as_hz(self) -> f64
    {
        self.numerator as f64 / self.denominator as f64
    }

    /// Returns whether the frequency reaches `hz`.
    ///
    /// An undefined frequency reaches nothing.
    #[must_use]
    pub const fn is_at_least(self, hz: u64) -> bool
    {
        if !self.is_defined()
        {
            return false;
        }

        match hz.checked_mul(self.denominator)
        {
            // Past the width of the type, the bound stands above every
            // numerator this type can hold.
            None => false,
            Some(scaled) => self.numerator >= scaled,
        }
    }

    /// Returns whether the frequency stays at or under `hz`.
    ///
    /// An undefined frequency stays under nothing.
    #[must_use]
    pub const fn is_at_most(self, hz: u64) -> bool
    {
        if !self.is_defined()
        {
            return false;
        }

        match hz.checked_mul(self.denominator)
        {
            None => true,
            Some(scaled) => self.numerator <= scaled,
        }
    }

    /// Returns whether the frequency sits within `ppb` parts per billion of
    /// `target_hz`.
    ///
    /// A ratio too wide to compare at that precision reports false, so the
    /// check refuses rather than rounds.
    #[must_use]
    pub const fn is_within_ppb(self, target_hz: u64, ppb: u64) -> bool
    {
        if !self.is_defined()
        {
            return false;
        }

        let Some(reference) = target_hz.checked_mul(self.denominator)
        else
        {
            return false;
        };

        let Some(deviation) = self.numerator.abs_diff(reference).checked_mul(1_000_000_000)
        else
        {
            return false;
        };

        match reference.checked_mul(ppb)
        {
            None => true,
            Some(allowed) => deviation <= allowed,
        }
    }
}

/// Returns the greatest common divisor of `first` and `second`.
const fn greatest_common_divisor(first: u64, second: u64) -> u64
{
    let mut left = first;
    let mut right = second;

    while right != 0
    {
        // The divisor is non-zero on every iteration, so the fallback stands
        // only to keep the expression total and it ends the loop.
        let remainder = match left.checked_rem(right)
        {
            Some(value) => value,
            None => 0,
        };

        left = right;
        right = remainder;
    }

    left
}

/// Divider chain of one PLL, and the two bands it declares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllPlan
{
    reference_divider: u8,
    multiplier: u16,
    fraction: u16,
    output_divider: u8,
    band: VcoBand,
    range: ReferenceRange,
}

impl PllPlan
{
    /// Builds a divider chain. `ClockPlan::validate` is what accepts it.
    #[must_use]
    pub const fn new
    (
        reference_divider: u8,
        multiplier: u16,
        fraction: u16,
        output_divider: u8,
        band: VcoBand,
        range: ReferenceRange
    ) -> Self
    {
        Self
        {
            reference_divider,
            multiplier,
            fraction,
            output_divider,
            band,
            range,
        }
    }

    /// Returns the integer part of the VCO multiplier.
    #[must_use]
    pub const fn multiplier(self) -> u16
    {
        self.multiplier
    }

    /// Returns the fractional part of the VCO multiplier, in 8192nds.
    ///
    /// `fraction_field` returns the same value because the field carries the
    /// fraction unshifted. The two are kept apart so that a caller reading the
    /// plan and a caller writing the register name what they are after, as they
    /// do for the three chain values whose field does carry an offset.
    #[must_use]
    pub const fn fraction(self) -> u16
    {
        self.fraction
    }

    /// Returns the divider between the VCO and the P output.
    #[must_use]
    pub const fn output_divider(self) -> u8
    {
        self.output_divider
    }

    /// Returns the declared VCO band.
    #[must_use]
    pub const fn band(self) -> VcoBand
    {
        self.band
    }

    /// Returns the declared reference range.
    #[must_use]
    pub const fn range(self) -> ReferenceRange
    {
        self.range
    }

    /// Returns the `DIVMx` field value, which is the divider itself.
    ///
    /// RM0433 section 8.7.11: 000001 divides by 1, up to 111111 by 63.
    #[must_use]
    pub const fn reference_divider_field(self) -> u8
    {
        self.reference_divider
    }

    /// Returns the `DIVNx` field value, one less than the multiplier.
    ///
    /// RM0433 section 8.7.17: 0x003 multiplies by 4, up to 0x1FF by 512.
    #[must_use]
    pub const fn multiplier_field(self) -> u16
    {
        self.multiplier.saturating_sub(1)
    }

    /// Returns the `FRACNx` field value, which is the fraction itself.
    #[must_use]
    pub const fn fraction_field(self) -> u16
    {
        self.fraction
    }

    /// Returns the `DIVPx` field value, one less than the divider.
    ///
    /// RM0433 section 8.7.17: 0000000 divides by 1, up to 1111111 by 128.
    #[must_use]
    pub const fn output_divider_field(self) -> u8
    {
        self.output_divider.saturating_sub(1)
    }
}

/// Every divider between the crystal and the frame the converters run on.
///
/// The audio interface takes its master clock divider and its frame length from
/// here so that one validated chain sets the sample rate. Nothing else about
/// the interface belongs in a clock plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockPlan
{
    source_hz: u32,
    pll: PllPlan,
    master_divider: u8,
    frame_bits: u16,
}

impl ClockPlan
{
    /// Builds a plan. `validate` is what accepts it.
    #[must_use]
    pub const fn new
    (
        source_hz: u32,
        pll: PllPlan,
        master_divider: u8,
        frame_bits: u16
    ) -> Self
    {
        Self
        {
            source_hz,
            pll,
            master_divider,
            frame_bits,
        }
    }

    /// Returns the source frequency the plan is built on, in hertz.
    #[must_use]
    pub const fn source_hz(self) -> u32
    {
        self.source_hz
    }

    /// Returns the divider chain of the PLL.
    #[must_use]
    pub const fn pll(self) -> PllPlan
    {
        self.pll
    }

    /// Returns the `MCKDIV` value the audio interface runs on.
    #[must_use]
    pub const fn master_divider(self) -> u8
    {
        self.master_divider
    }

    /// Returns the bits in one audio frame, which the `FRL` field encodes one
    /// less of.
    #[must_use]
    pub const fn frame_bits(self) -> u16
    {
        self.frame_bits
    }

    /// Returns the frequency the phase detector compares at.
    #[must_use]
    pub const fn reference_hz(self) -> Frequency
    {
        Frequency::new(self.source_hz as u64, self.pll.reference_divider as u64)
    }

    /// Returns the VCO frequency.
    #[must_use]
    pub const fn vco_hz(self) -> Frequency
    {
        let multiplier = match (self.pll.multiplier as u64).checked_mul(FRACTION_SCALE)
        {
            None => return Frequency::UNDEFINED,
            Some(scaled) => match scaled.checked_add(self.pll.fraction as u64)
            {
                None => return Frequency::UNDEFINED,
                Some(total) => total,
            },
        };

        self.reference_hz().scaled(multiplier, FRACTION_SCALE)
    }

    /// Returns `pll3_p_ck`, the kernel clock the audio interface runs on.
    #[must_use]
    pub const fn kernel_hz(self) -> Frequency
    {
        self.vco_hz().scaled(1, self.pll.output_divider as u64)
    }

    /// Returns the master clock the converters take.
    #[must_use]
    pub const fn master_clock_hz(self) -> Frequency
    {
        self.kernel_hz().scaled(1, self.master_divider as u64)
    }

    /// Returns the frame rate, which is the sample rate of the whole chain.
    #[must_use]
    pub const fn sample_rate_hz(self) -> Frequency
    {
        self.master_clock_hz().scaled(1, MASTER_CLOCK_RATIO)
    }

    /// Returns the bit clock.
    #[must_use]
    pub const fn bit_clock_hz(self) -> Frequency
    {
        self.sample_rate_hz().scaled(self.frame_bits as u64, 1)
    }

    /// Returns nothing when the part accepts the plan.
    ///
    /// # Errors
    ///
    /// One variant of `ClockPlanError` per bound, so a refusal names the bound
    /// it broke rather than the fact that something broke.
    pub const fn validate(self) -> Result<(), ClockPlanError>
    {
        match self.validate_fields()
        {
            Err(error) => Err(error),
            Ok(()) => match self.validate_bands()
            {
                Err(error) => Err(error),
                Ok(()) => self.validate_outputs(),
            },
        }
    }

    /// Checks every value against the width of the field that carries it.
    const fn validate_fields(self) -> Result<(), ClockPlanError>
    {
        if self.source_hz < CRYSTAL_MIN_HZ || self.source_hz > CRYSTAL_MAX_HZ
        {
            return Err(ClockPlanError::SourceOutOfRange);
        }

        if self.pll.reference_divider < REFERENCE_DIVIDER_MIN
            || self.pll.reference_divider > REFERENCE_DIVIDER_MAX
        {
            return Err(ClockPlanError::ReferenceDividerOutOfRange);
        }

        if self.pll.multiplier < MULTIPLIER_MIN || self.pll.multiplier > MULTIPLIER_MAX
        {
            return Err(ClockPlanError::MultiplierOutOfRange);
        }

        if self.pll.fraction > FRACTION_MAX
        {
            return Err(ClockPlanError::FractionOutOfRange);
        }

        if self.pll.output_divider < OUTPUT_DIVIDER_MIN
            || self.pll.output_divider > OUTPUT_DIVIDER_MAX
        {
            return Err(ClockPlanError::OutputDividerOutOfRange);
        }

        Ok(())
    }

    /// Checks the reference and the VCO against the bands the plan declares.
    const fn validate_bands(self) -> Result<(), ClockPlanError>
    {
        if self.pll.fraction != 0 && !self.pll.band.accepts_fraction()
        {
            return Err(ClockPlanError::FractionForbiddenInBand);
        }

        let reference = self.reference_hz();

        if !reference.is_at_least(self.pll.band.reference_min_hz())
            || !reference.is_at_most(self.pll.band.reference_max_hz())
        {
            return Err(ClockPlanError::ReferenceOutsideBand);
        }

        if !reference.is_at_least(self.pll.range.min_hz())
            || !reference.is_at_most(self.pll.range.max_hz())
        {
            return Err(ClockPlanError::ReferenceOutsideDeclaredRange);
        }

        let vco = self.vco_hz();

        if !vco.is_at_least(self.pll.band.vco_min_hz())
            || !vco.is_at_most(self.pll.band.vco_max_hz())
        {
            return Err(ClockPlanError::VcoOutsideBand);
        }

        let floor = self.pll.band.vco_min_hz().saturating_add(VCO_MARGIN_HZ);
        let ceiling = self.pll.band.vco_max_hz().saturating_sub(VCO_MARGIN_HZ);

        if !vco.is_at_least(floor) || !vco.is_at_most(ceiling)
        {
            return Err(ClockPlanError::VcoMarginTooSmall);
        }

        Ok(())
    }

    /// Checks the kernel clock and the interface dividers that follow it.
    const fn validate_outputs(self) -> Result<(), ClockPlanError>
    {
        // The floor of the P output has no check of its own. The band floor
        // divided by the widest output divider already clears it, which
        // KERNEL_FLOOR_FOLLOWS_THE_BAND proves.
        if !self.kernel_hz().is_at_most(AUDIO_KERNEL_MAX_HZ)
        {
            return Err(ClockPlanError::KernelAboveMaximum);
        }

        if self.master_divider < MASTER_DIVIDER_MIN || self.master_divider > MASTER_DIVIDER_MAX
        {
            return Err(ClockPlanError::MasterDividerOutOfRange);
        }

        // RM0433 leaves the master clock away from a 50 per cent duty cycle on
        // an odd divider, and the converters take that clock directly.
        if self.master_divider & 1 != 0
        {
            return Err(ClockPlanError::MasterDividerOdd);
        }

        if self.frame_bits < FRAME_BITS_MIN || self.frame_bits > FRAME_BITS_MAX
        {
            return Err(ClockPlanError::FrameOutOfRange);
        }

        if !self.frame_bits.is_power_of_two()
        {
            return Err(ClockPlanError::FrameNotPowerOfTwo);
        }

        Ok(())
    }

    /// Returns nothing when `seen` carries the plan.
    ///
    /// Every field the output frequency depends on is compared, including the
    /// ones a reset leaves usable. A field left out of this list is a field the
    /// firmware cannot tell apart from the one it asked for. The crystal itself
    /// is outside it, since no register reports the frequency of the part
    /// soldered to the oscillator pins.
    ///
    /// # Errors
    ///
    /// One variant of `ClockFault` per field, in a fixed order that runs from
    /// the oscillator outwards: the source first, then the divider chain, then
    /// the declared bands, the enables and what the kernel clock feeds. That
    /// order is not the order the bring-up writes the fields in, so the variant
    /// returned names the earliest disagreement in the chain rather than the
    /// earliest write that did not take.
    pub fn verify(self, seen: &ClockReadback) -> Result<(), ClockFault>
    {
        verify_source(seen)?;
        self.verify_dividers(seen)?;
        self.verify_configuration(seen)
    }

    /// Checks the four divider fields against the plan.
    fn verify_dividers(self, seen: &ClockReadback) -> Result<(), ClockFault>
    {
        if seen.reference_divider_field != self.pll.reference_divider_field()
        {
            return Err(ClockFault::ReferenceDividerWrong);
        }

        if seen.multiplier_field != self.pll.multiplier_field()
        {
            return Err(ClockFault::MultiplierWrong);
        }

        if seen.fraction_field != self.pll.fraction_field()
        {
            return Err(ClockFault::FractionWrong);
        }

        if seen.output_divider_field != self.pll.output_divider_field()
        {
            return Err(ClockFault::OutputDividerWrong);
        }

        Ok(())
    }

    /// Checks the declared bands, the latch, and what the kernel clock feeds.
    fn verify_configuration(self, seen: &ClockReadback) -> Result<(), ClockFault>
    {
        if seen.reference_range_bits != self.pll.range.bits()
        {
            return Err(ClockFault::ReferenceRangeWrong);
        }

        if seen.vco_band_bit != self.pll.band.selection_bit()
        {
            return Err(ClockFault::VcoBandWrong);
        }

        if !seen.fraction_latched
        {
            return Err(ClockFault::FractionNotLatched);
        }

        if !seen.output_enabled
        {
            return Err(ClockFault::OutputDisabled);
        }

        if !seen.pll_on
        {
            return Err(ClockFault::PllNotEnabled);
        }

        if !seen.pll_ready
        {
            return Err(ClockFault::PllNotLocked);
        }

        if seen.audio_kernel_bits != AUDIO_KERNEL_PLL3_P
        {
            return Err(ClockFault::AudioKernelSourceWrong);
        }

        Ok(())
    }
}

/// Checks the oscillator every divider is fed from.
///
/// The plan names no source of its own. Only the external oscillator drives
/// this board, and a plan that could name another would make the field a
/// setting rather than a requirement.
fn verify_source(seen: &ClockReadback) -> Result<(), ClockFault>
{
    if seen.source_bits != PLL_SOURCE_EXTERNAL
    {
        return Err(ClockFault::SourceNotSelected);
    }

    if seen.source_bypassed
    {
        return Err(ClockFault::SourceBypassed);
    }

    if !seen.source_ready
    {
        return Err(ClockFault::SourceNotReady);
    }

    Ok(())
}

/// Reason the part would not accept a plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockPlanError
{
    /// The source sits outside the 4 to 48 MHz the oscillator drives.
    SourceOutOfRange,
    /// `DIVMx` carries 1 to 63.
    ReferenceDividerOutOfRange,
    /// `DIVNx` carries 4 to 512.
    MultiplierOutOfRange,
    /// `FRACNx` carries 0 to 8191.
    FractionOutOfRange,
    /// `DIVPx` carries 1 to 128.
    OutputDividerOutOfRange,
    /// A non-zero fraction asks for a modulator the declared band forbids.
    FractionForbiddenInBand,
    /// The reference falls outside the range the declared band accepts.
    ReferenceOutsideBand,
    /// The reference falls outside the window `PLLxRGE` declares.
    ReferenceOutsideDeclaredRange,
    /// The VCO falls outside the declared band.
    VcoOutsideBand,
    /// The VCO sits inside the band but too close to one of its edges.
    VcoMarginTooSmall,
    /// The kernel clock passes what the audio interface takes at the voltage
    /// scale the part boots on.
    KernelAboveMaximum,
    /// `MCKDIV` carries 1 to 63.
    MasterDividerOutOfRange,
    /// An odd master divider leaves the master clock off a 50 per cent duty
    /// cycle.
    MasterDividerOdd,
    /// The frame runs from 8 to 256 bits.
    FrameOutOfRange,
    /// The frame length is not a power of two, which the master clock requires.
    FrameNotPowerOfTwo,
}

impl ClockPlanError
{
    /// Returns the code a fault record carries for this bound.
    ///
    /// The numbering is an interface a person reads off a probe, so it is
    /// written out rather than taken from the order of the variants. Zero is
    /// left free, which is what lets `ClockFault::code` mean no fault by it.
    #[must_use]
    pub const fn code(self) -> u32
    {
        match self
        {
            Self::SourceOutOfRange => 0x01,
            Self::ReferenceDividerOutOfRange => 0x02,
            Self::MultiplierOutOfRange => 0x03,
            Self::FractionOutOfRange => 0x04,
            Self::OutputDividerOutOfRange => 0x05,
            Self::FractionForbiddenInBand => 0x06,
            Self::ReferenceOutsideBand => 0x07,
            Self::ReferenceOutsideDeclaredRange => 0x08,
            Self::VcoOutsideBand => 0x09,
            Self::VcoMarginTooSmall => 0x0A,
            Self::KernelAboveMaximum => 0x0B,
            Self::MasterDividerOutOfRange => 0x0C,
            Self::MasterDividerOdd => 0x0D,
            Self::FrameOutOfRange => 0x0E,
            Self::FrameNotPowerOfTwo => 0x0F,
        }
    }
}

/// Bit a fault code carries when a plan was refused before any register moved.
///
/// The low byte then holds the `ClockPlanError` code, so one word names both
/// halves of the refusal.
const PLAN_REJECTED_FLAG: u32 = 0x0100;

/// Reason the audio clock is not running to plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockFault
{
    /// The plan itself is not one the part accepts.
    PlanRejected(ClockPlanError),
    /// The PLL still reported ready after it was told to stop, so the divider
    /// fields would not have taken.
    PllNeverStopped,
    /// A PLL is on, and RM0433 section 8.7.11 accepts a `PLLSRC` write only
    /// while every PLL is off, so the write would be dropped and the dividers
    /// would keep running off the internal oscillator. All three enables are
    /// read, the one this bring-up drives included, because the manual names
    /// them all and stopping one says nothing about the other two.
    OtherPllRunning,
    /// The fractional latch still stood after it was cleared, so setting it
    /// makes no edge and the modulator keeps its old value.
    FractionLatchNotCleared,
    /// The oscillator never reported ready.
    SourceNeverReady,
    /// The PLL never reported locked.
    PllNeverLocked,
    /// `PLLSRC` does not name the external oscillator.
    SourceNotSelected,
    /// `HSEBYP` is set, so the input is an external clock and not the crystal.
    SourceBypassed,
    /// `HSERDY` is clear.
    SourceNotReady,
    /// `DIVMx` does not carry the reference divider of the plan.
    ReferenceDividerWrong,
    /// `DIVNx` does not carry the multiplier of the plan.
    MultiplierWrong,
    /// `FRACNx` does not carry the fraction of the plan.
    FractionWrong,
    /// `DIVPx` does not carry the output divider of the plan.
    OutputDividerWrong,
    /// `PLLxRGE` does not carry the declared reference range.
    ReferenceRangeWrong,
    /// `PLLxVCOSEL` does not carry the declared band.
    VcoBandWrong,
    /// `PLLxFRACEN` is clear, so the modulator holds no fraction.
    FractionNotLatched,
    /// `DIVPxEN` is clear, so the P output is stopped.
    ///
    /// RM0433 section 8.7.12 sets the bit out of reset, so a bring-up that
    /// skipped the write that sets it would still pass. Reaching this takes a
    /// write that cleared the bit, or a part that refused the write.
    OutputDisabled,
    /// `PLLxON` is clear.
    PllNotEnabled,
    /// `PLLxRDY` is clear.
    PllNotLocked,
    /// The audio interface takes its kernel clock from somewhere else.
    AudioKernelSourceWrong,
}

impl ClockFault
{
    /// Returns the code a fault record carries for this fault.
    ///
    /// A probe reads the word and looks the value up here, so the numbering is
    /// written out rather than taken from the order of the variants. Zero names
    /// no fault, and a plan refusal sets `PLAN_REJECTED_FLAG` over the code of
    /// the bound it broke.
    #[must_use]
    pub const fn code(self) -> u32
    {
        match self
        {
            Self::PlanRejected(error) => PLAN_REJECTED_FLAG | error.code(),
            Self::PllNeverStopped => 0x01,
            Self::OtherPllRunning => 0x02,
            Self::FractionLatchNotCleared => 0x03,
            Self::SourceNeverReady => 0x04,
            Self::PllNeverLocked => 0x05,
            Self::SourceNotSelected => 0x06,
            Self::SourceBypassed => 0x07,
            Self::SourceNotReady => 0x08,
            Self::ReferenceDividerWrong => 0x09,
            Self::MultiplierWrong => 0x0A,
            Self::FractionWrong => 0x0B,
            Self::OutputDividerWrong => 0x0C,
            Self::ReferenceRangeWrong => 0x0D,
            Self::VcoBandWrong => 0x0E,
            Self::FractionNotLatched => 0x0F,
            Self::OutputDisabled => 0x10,
            Self::PllNotEnabled => 0x11,
            Self::PllNotLocked => 0x12,
            Self::AudioKernelSourceWrong => 0x13,
        }
    }
}

/// Every register field the audio kernel frequency depends on, and the two the
/// part requires clear before a `PLLSRC` write takes.
///
/// The set is closed on purpose. A field the output depends on and this struct
/// leaves out is a field no check can reach, and a reset value in it reads as
/// success.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[expect
(
    clippy::struct_excessive_bools,
    reason = "each field is one register bit, and naming them apart is what \
              lets a fault say which bit"
)]
pub struct ClockReadback
{
    /// `PLLSRC`, the oscillator every `DIVMx` divider is fed from.
    pub source_bits: u8,
    /// `HSERDY`.
    pub source_ready: bool,
    /// `HSEBYP`, set when the input is an external clock rather than a crystal.
    pub source_bypassed: bool,
    /// `DIVMx`.
    pub reference_divider_field: u8,
    /// `DIVNx`.
    pub multiplier_field: u16,
    /// `FRACNx`.
    pub fraction_field: u16,
    /// `DIVPx`.
    pub output_divider_field: u8,
    /// `PLLxRGE`.
    pub reference_range_bits: u8,
    /// `PLLxVCOSEL`.
    pub vco_band_bit: bool,
    /// `PLLxFRACEN`.
    pub fraction_latched: bool,
    /// `DIVPxEN`.
    pub output_enabled: bool,
    /// `PLLxON`.
    pub pll_on: bool,
    /// `PLLxRDY`.
    pub pll_ready: bool,
    /// `SAI1SEL`.
    pub audio_kernel_bits: u8,
    /// `PLL1ON`, which gates the `PLLSRC` write and nothing downstream.
    pub pll1_on: bool,
    /// `PLL2ON`, which gates the `PLLSRC` write and nothing downstream.
    pub pll2_on: bool,
}

/// The register writes and the one read a bring-up needs.
///
/// Each write covers one register, because the order they land in is what the
/// part specifies. `read` is the only way to observe the tree, so a wait and a
/// verification look at the same thing, and one poll of a wait costs a whole
/// read.
pub trait ClockTree
{
    /// Clears `PLLxON`, which is what makes the divider fields writable.
    fn stop_pll(&mut self);

    /// Sets `HSEON`.
    fn start_source(&mut self);

    /// Writes `PLLSRC` and `DIVMx`.
    fn write_source_and_reference(&mut self, plan: &ClockPlan);

    /// Writes `PLLxRGE`, `PLLxVCOSEL` and `DIVPxEN`, and clears `PLLxFRACEN`.
    fn write_configuration(&mut self, plan: &ClockPlan);

    /// Writes `FRACNx`.
    fn write_fraction(&mut self, plan: &ClockPlan);

    /// Writes `DIVNx` and `DIVPx`.
    fn write_dividers(&mut self, plan: &ClockPlan);

    /// Sets `PLLxFRACEN`, whose rising edge loads the modulator.
    fn latch_fraction(&mut self);

    /// Sets `PLLxON`.
    fn start_pll(&mut self);

    /// Points the audio interface at the P output of the PLL.
    fn select_audio_kernel(&mut self);

    /// Reads back every field the audio kernel frequency depends on.
    fn read(&self) -> ClockReadback;
}

/// Polls each wait holds before it gives up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockWaits
{
    /// Polls the oscillator gets to report ready.
    pub source_polls: u32,
    /// Polls the PLL gets to report locked, and to report stopped.
    pub pll_polls: u32,
}

impl ClockWaits
{
    /// Builds the waits a part running at `core_clock_hz` needs.
    ///
    /// The counts cover 20 ms for the oscillator and 10 ms for the PLL at one
    /// core cycle per poll. A poll is one `ClockTree::read`, which is many
    /// cycles rather than one, so the two figures are floors on the time a wait
    /// holds and not the time it takes. Overshooting a timeout only lengthens a
    /// boot that is going to end silent anyway, and undershooting is what these
    /// counts cannot do.
    ///
    /// The STM32H743VI datasheet table 141 gives 2 ms as the typical startup of
    /// a crystal in this range and no maximum, because the figure belongs to
    /// the resonator.
    ///
    /// Table 147 gives 166 us as the worst case PLL lock time in sigma-delta
    /// mode, under a condition this plan does not meet, its reference being
    /// under the 8 MHz the line is specified at.
    #[must_use]
    pub const fn for_core_clock(core_clock_hz: u32) -> Self
    {
        Self
        {
            source_polls: wait_polls(core_clock_hz, 20_000),
            pll_polls: wait_polls(core_clock_hz, 10_000),
        }
    }
}

/// Returns the polls covering `microseconds` at `core_clock_hz`.
///
/// One poll is a loop iteration carrying a whole `ClockTree::read`, which the
/// register block answers with six volatile reads and the core retires in at
/// least one cycle each, so the count is a lower bound on the time a wait holds
/// and never an upper one. A budget past the width of the count
/// saturates, which shortens the wait, and `for_core_clock` stays well inside
/// it at every clock the part runs.
#[must_use]
pub const fn wait_polls(core_clock_hz: u32, microseconds: u32) -> u32
{
    #[expect
    (
        clippy::cast_possible_truncation,
        reason = "the comparison above is what keeps the value inside u32"
    )]
    {
        let polls = wait_polls_wide(core_clock_hz, microseconds);

        if polls > u32::MAX as u64
        {
            u32::MAX
        }
        else
        {
            polls as u32
        }
    }
}

/// Returns the same count as `wait_polls`, before narrowing.
const fn wait_polls_wide(core_clock_hz: u32, microseconds: u32) -> u64
{
    let cycles = (core_clock_hz as u64).saturating_mul(microseconds as u64);
    cycles.div_ceil(MICROSECONDS_PER_SECOND as u64)
}

/// Brings the audio kernel clock up on `tree` and proves it took the plan.
///
/// The order is what the part specifies and what a warm restart needs.
///
/// The PLL stops first, because RM0433 makes `DIVNx` and `DIVPx` writable only
/// while it reports neither on nor ready, and a restart can find it running.
///
/// The oscillator starts next, and the source selection follows it. RM0433
/// section 8.7.11 accepts a `PLLSRC` write only while every PLL is off, so all
/// three enables are read before the write and any one of them set refuses the
/// bring-up. A dropped `PLLSRC` write is the fault a lock bit hides best: the
/// dividers stay on the internal oscillator and the PLL locks over them.
///
/// `PLLSRC` is one field for the three PLLs, so this refusal also fixes an
/// order the rest of the firmware keeps: this bring-up runs before PLL1 or PLL2
/// is started, and PLL1 is what will drive `sys_ck`. Starting a PLL first turns
/// the audio clock into a refusal, and the refusal is conservative: a part
/// already carrying `PLLSRC` for the external oscillator would have taken a
/// write that changed nothing, and it is refused all the same rather than
/// making the guard depend on the field it protects.
///
/// The configuration write clears the fractional latch, and the read after it
/// proves the bit went down. Only a bit observed low and then set carries the
/// rising edge that loads the modulator, and a restart can find it already set,
/// where writing it again loads nothing.
///
/// The verification is the return value. A lock bit alone would report success
/// on a PLL fed from the internal oscillator.
///
/// # Errors
///
/// `PlanRejected` before any register is touched, then one variant per step
/// that did not take. A refusal reached after `start_pll` leaves the PLL
/// running on whatever the writes did land, because putting the tree back
/// where a reset left it is not something a half-applied plan can do. Nothing
/// downstream of the PLL is enabled here, so the caller decides what a running
/// output is worth.
pub fn bring_up<T>
(
    tree: &mut T,
    plan: &ClockPlan,
    waits: &ClockWaits
) -> Result<(), ClockFault>
where
    T: ClockTree + ?Sized,
{
    if let Err(error) = plan.validate()
    {
        return Err(ClockFault::PlanRejected(error));
    }

    tree.stop_pll();

    if !poll_until(tree, waits.pll_polls, |seen| !seen.pll_ready && !seen.pll_on)
    {
        return Err(ClockFault::PllNeverStopped);
    }

    tree.start_source();

    if !poll_until(tree, waits.source_polls, |seen| seen.source_ready)
    {
        return Err(ClockFault::SourceNeverReady);
    }

    let armed = tree.read();

    if armed.pll_on || armed.pll1_on || armed.pll2_on
    {
        return Err(ClockFault::OtherPllRunning);
    }

    tree.write_source_and_reference(plan);
    tree.write_configuration(plan);

    if tree.read().fraction_latched
    {
        return Err(ClockFault::FractionLatchNotCleared);
    }

    tree.write_fraction(plan);
    tree.write_dividers(plan);
    tree.latch_fraction();
    tree.start_pll();

    if !poll_until(tree, waits.pll_polls, |seen| seen.pll_ready)
    {
        return Err(ClockFault::PllNeverLocked);
    }

    tree.select_audio_kernel();

    plan.verify(&tree.read())
}

/// Polls `tree` until `ready` holds, or `polls` more reads have gone by.
///
/// The condition is read before the budget is spent, so a budget of zero still
/// buys one look. Each poll is a whole `ClockTree::read`, whatever the predicate
/// goes on to look at, so the budget buys reads and not cycles.
fn poll_until<T, F>(tree: &T, polls: u32, ready: F) -> bool
where
    T: ClockTree + ?Sized,
    F: Fn(&ClockReadback) -> bool,
{
    let mut remaining = polls;

    loop
    {
        if ready(&tree.read())
        {
            return true;
        }

        if remaining == 0
        {
            return false;
        }

        remaining = remaining.saturating_sub(1);
    }
}

/// The clock plan the processing board runs on.
///
/// The crystal is 25 MHz, which no integer multiplier turns into 44100 Hz for
/// any legal set of dividers, so the fraction is non-zero by necessity. That
/// closes the medium VCO band, whose modulator is forbidden, and the wide band
/// then imposes a reference of at least 2 MHz.
///
/// The chain lands 0.0169 ppm above 44100 Hz. The converter senses the sampling
/// frequency itself and groups 32, 44.1 and 48 kHz as one rate with a plus or
/// minus 4 per cent tolerance, which the PCM5102A datasheet gives in its system
/// clock section, so the offset sits six orders of magnitude inside what the
/// part downstream cares about.
pub const AUDIO_PLAN: ClockPlan = ClockPlan::new
(
    CRYSTAL_HZ,
    PllPlan::new(5, 94, 6_821, 21, VcoBand::Wide, ReferenceRange::Mhz4To8),
    2,
    64
);

/// Whether the widest output divider can drop a P output under its floor.
///
/// The lowest VCO a band allows, divided by the widest divider the field
/// encodes, is what a plan in that band can reach at the bottom. Both bands
/// clear the floor the datasheet gives them, so no plan reaches it and
/// `validate` carries no check for it.
const KERNEL_FLOOR_FOLLOWS_THE_BAND: bool = VcoBand::Wide.vco_min_hz()
    >= VcoBand::Wide.output_min_hz().saturating_mul(OUTPUT_DIVIDER_MAX as u64)
    && VcoBand::Medium.vco_min_hz()
        >= VcoBand::Medium.output_min_hz().saturating_mul(OUTPUT_DIVIDER_MAX as u64);

const _: () = assert!
(
    KERNEL_FLOOR_FOLLOWS_THE_BAND,
    "the widest output divider cannot drop the P output under what the part guarantees"
);

const _: () = assert!
(
    matches!(AUDIO_PLAN.validate(), Ok(())),
    "the plan the board runs on is one the part accepts"
);

const _: () = assert!
(
    AUDIO_PLAN.master_clock_hz().is_within_ppb(NOMINAL_MASTER_CLOCK_HZ, 1_000),
    "the master clock lands on the 256 fs line of the converter"
);

const _: () = assert!
(
    AUDIO_PLAN.bit_clock_hz().is_within_ppb(NOMINAL_BIT_CLOCK_HZ, 1_000),
    "the bit clock lands on the 64 fs line of the converter"
);

const _: () = assert!
(
    AUDIO_PLAN.sample_rate_hz().is_within_ppb(nominal_hz(1), 50),
    "the frame rate lands within 0.05 ppm of the rate the chain runs at"
);

const _: () = assert!
(
    AUDIO_PLAN.frame_bits() == NOMINAL_FRAME_BITS,
    "the frame the plan carries is the one the bit clock target is taken from"
);

const _: () = assert!
(
    AUDIO_PLAN.pll().fraction() != 0,
    "a zero fraction would open the medium band, whose modulator is forbidden"
);


#[cfg(test)]
mod tests
{
    use super::*;
    use core::cell::Cell;

    /// Tolerance covering double precision rounding on a reported frequency.
    const EPSILON_HZ: f64 = 1e-3;

    /// One broken field of a readback, and the fault it must produce.
    type Mutation = (fn(&mut ClockReadback), ClockFault);

    /// A clock tree whose registers answer the way the part does.
    ///
    /// `image` holds the fields at their reset values, so a step the sequence
    /// skips shows up as a reset value rather than as the plan. The two ready
    /// bits are computed on each read from the polls since the thing they
    /// belong to was started, which is what lets a wait run out.
    #[expect
    (
        clippy::struct_excessive_bools,
        reason = "each flag is one way a register block misbehaves, and naming \
                  them apart is what lets a test pick one"
    )]
    struct MockTree
    {
        image: ClockReadback,
        polls: Cell<u32>,
        source_started: bool,
        source_ready_after: u32,
        pll_ready_after: u32,
        refuse_source_write: bool,
        keep_fraction_latched: bool,
        pll_stays_on: bool,
        pll_restarts_with_source: bool,
    }

    impl MockTree
    {
        /// Builds a tree that takes every step and reports ready on the second
        /// poll.
        ///
        /// The image is RM0433 sections 8.7.2, 8.7.11, 8.7.12, 8.7.17, 8.7.18
        /// and 8.7.20 read off their reset values: `DIVM3` divides by 32,
        /// `DIVN3` multiplies by 129, `DIVP3` divides by 2, and the three P
        /// output enables of `PLLCFGR` stand at 1. Section 8.7.2 is `RCC_CR`,
        /// which carries the six ready and enable bits of the image.
        fn healthy() -> Self
        {
            Self
            {
                image: ClockReadback
                {
                    source_bits: 0,
                    source_ready: false,
                    source_bypassed: false,
                    reference_divider_field: 0b10_0000,
                    multiplier_field: 0x80,
                    fraction_field: 0,
                    output_divider_field: 1,
                    reference_range_bits: 0,
                    vco_band_bit: false,
                    fraction_latched: false,
                    output_enabled: true,
                    pll_on: false,
                    pll_ready: false,
                    audio_kernel_bits: 0,
                    pll1_on: false,
                    pll2_on: false,
                },
                polls: Cell::new(0),
                source_started: false,
                source_ready_after: 2,
                pll_ready_after: 2,
                refuse_source_write: false,
                keep_fraction_latched: false,
                pll_stays_on: false,
                pll_restarts_with_source: false,
            }
        }
    }

    impl ClockTree for MockTree
    {
        fn stop_pll(&mut self)
        {
            if self.pll_stays_on
            {
                return;
            }

            self.image.pll_on = false;
        }

        fn start_source(&mut self)
        {
            self.source_started = true;
            self.polls.set(0);

            if self.pll_restarts_with_source
            {
                self.image.pll_on = true;
            }
        }

        fn write_source_and_reference(&mut self, plan: &ClockPlan)
        {
            if !self.refuse_source_write
            {
                self.image.source_bits = PLL_SOURCE_EXTERNAL;
            }

            self.image.reference_divider_field = plan.pll().reference_divider_field();
        }

        fn write_configuration(&mut self, plan: &ClockPlan)
        {
            self.image.reference_range_bits = plan.pll().range().bits();
            self.image.vco_band_bit = plan.pll().band().selection_bit();
            self.image.output_enabled = true;

            if !self.keep_fraction_latched
            {
                self.image.fraction_latched = false;
            }
        }

        fn write_fraction(&mut self, plan: &ClockPlan)
        {
            self.image.fraction_field = plan.pll().fraction_field();
        }

        fn write_dividers(&mut self, plan: &ClockPlan)
        {
            self.image.multiplier_field = plan.pll().multiplier_field();
            self.image.output_divider_field = plan.pll().output_divider_field();
        }

        fn latch_fraction(&mut self)
        {
            self.image.fraction_latched = true;
        }

        fn start_pll(&mut self)
        {
            self.image.pll_on = true;
            self.polls.set(0);
        }

        fn select_audio_kernel(&mut self)
        {
            self.image.audio_kernel_bits = AUDIO_KERNEL_PLL3_P;
        }

        fn read(&self) -> ClockReadback
        {
            let polls = self.polls.get().saturating_add(1);
            self.polls.set(polls);

            let mut seen = self.image;
            seen.source_ready = self.source_started && polls >= self.source_ready_after;
            seen.pll_ready = seen.pll_on && polls >= self.pll_ready_after;
            seen
        }
    }

    /// The waits a run at the clock the part boots on gets.
    fn waits() -> ClockWaits
    {
        ClockWaits::for_core_clock(64_000_000)
    }

    /// The readback a bring-up on a healthy tree leaves behind.
    fn verified_readback() -> ClockReadback
    {
        let mut tree = MockTree::healthy();

        assert_eq!(bring_up(&mut tree, &AUDIO_PLAN, &waits()), Ok(()));
        tree.read()
    }

    /// Returns the board plan with `pll` in place of its divider chain.
    fn with_pll(pll: PllPlan) -> ClockPlan
    {
        ClockPlan::new(CRYSTAL_HZ, pll, AUDIO_PLAN.master_divider(), AUDIO_PLAN.frame_bits())
    }

    #[test]
    fn the_reference_is_five_megahertz_exactly()
    {
        let reference = AUDIO_PLAN.reference_hz();

        assert_eq!(reference.numerator(), 5_000_000);
        assert_eq!(reference.denominator(), 1);
    }

    #[test]
    fn the_derived_clocks_match_the_bench_figures()
    {
        assert!((AUDIO_PLAN.vco_hz().as_hz() - 474_163_208.007_812_5).abs() < EPSILON_HZ);
        assert!((AUDIO_PLAN.kernel_hz().as_hz() - 22_579_200.381).abs() < EPSILON_HZ);
        assert!((AUDIO_PLAN.master_clock_hz().as_hz() - 11_289_600.190).abs() < EPSILON_HZ);
        assert!((AUDIO_PLAN.bit_clock_hz().as_hz() - 2_822_400.047).abs() < EPSILON_HZ);
        assert!((AUDIO_PLAN.sample_rate_hz().as_hz() - 44_100.000_744_8).abs() < 1e-6);
    }

    #[test]
    fn the_frame_rate_sits_a_sixtieth_of_a_part_per_million_high()
    {
        let rate = AUDIO_PLAN.sample_rate_hz();

        assert!(rate.is_within_ppb(44_100, 17));
        assert!(!rate.is_within_ppb(44_100, 16));
        assert!(!rate.is_at_most(44_100));
        assert!(rate.is_at_least(44_100));
    }

    #[test]
    fn every_derived_clock_divides_the_one_above_it()
    {
        let kernel = AUDIO_PLAN.kernel_hz();
        let master = AUDIO_PLAN.master_clock_hz();
        let rate = AUDIO_PLAN.sample_rate_hz();

        assert_eq!(master, kernel.scaled(1, 2));
        assert_eq!(rate, master.scaled(1, 256));
        assert_eq!(AUDIO_PLAN.bit_clock_hz(), rate.scaled(64, 1));
    }

    #[test]
    fn the_field_encodings_match_the_reference_manual()
    {
        // RM0433 section 8.7.17: 0x003 multiplies by 4, 0x1FF by 512, and
        // 0000000 divides by 1.
        let low = PllPlan::new(1, 4, 0, 1, VcoBand::Wide, ReferenceRange::Mhz4To8);
        let high = PllPlan::new(63, 512, 8_191, 128, VcoBand::Wide, ReferenceRange::Mhz4To8);

        assert_eq!(low.multiplier_field(), 0x003);
        assert_eq!(low.output_divider_field(), 0b000_0000);
        assert_eq!(low.reference_divider_field(), 1);
        assert_eq!(high.multiplier_field(), 0x1FF);
        assert_eq!(high.output_divider_field(), 0b111_1111);
        assert_eq!(high.reference_divider_field(), 63);
        assert_eq!(high.fraction_field(), 8_191);
    }

    #[test]
    fn the_board_plan_encodes_the_fields_the_bench_confirmed()
    {
        let pll = AUDIO_PLAN.pll();

        assert_eq!(pll.reference_divider_field(), 5);
        assert_eq!(pll.multiplier_field(), 93);
        assert_eq!(pll.fraction_field(), 6_821);
        assert_eq!(pll.output_divider_field(), 20);
        assert_eq!(pll.range().bits(), 0b10);
        assert!(!pll.band().selection_bit());
        assert_eq!(AUDIO_PLAN.master_divider(), 2);
        assert_eq!(AUDIO_PLAN.frame_bits(), 64);
        assert_eq!(AUDIO_PLAN.source_hz(), 25_000_000);
    }

    #[test]
    fn the_band_and_range_tables_follow_the_datasheet()
    {
        assert!(VcoBand::Wide.accepts_fraction());
        assert!(!VcoBand::Medium.accepts_fraction());
        assert!(!VcoBand::Wide.selection_bit());
        assert!(VcoBand::Medium.selection_bit());
        assert_eq!(VcoBand::Wide.reference_min_hz(), 2_000_000);
        assert_eq!(VcoBand::Wide.reference_max_hz(), 16_000_000);
        assert_eq!(VcoBand::Medium.reference_min_hz(), 1_000_000);
        assert_eq!(VcoBand::Medium.reference_max_hz(), 2_000_000);
        assert_eq!(VcoBand::Wide.vco_min_hz(), 192_000_000);
        assert_eq!(VcoBand::Wide.vco_max_hz(), 836_000_000);
        assert_eq!(VcoBand::Medium.vco_min_hz(), 150_000_000);
        assert_eq!(VcoBand::Medium.vco_max_hz(), 420_000_000);
        assert_eq!(VcoBand::Wide.output_min_hz(), 1_500_000);
        assert_eq!(VcoBand::Medium.output_min_hz(), 1_170_000);

        for (range, bits, low, high) in
        [
            (ReferenceRange::Mhz1To2, 0, 1_000_000, 2_000_000),
            (ReferenceRange::Mhz2To4, 1, 2_000_000, 4_000_000),
            (ReferenceRange::Mhz4To8, 2, 4_000_000, 8_000_000),
            (ReferenceRange::Mhz8To16, 3, 8_000_000, 16_000_000),
        ]
        {
            assert_eq!(range.bits(), bits);
            assert_eq!(range.min_hz(), low);
            assert_eq!(range.max_hz(), high);
        }
    }

    #[test]
    fn the_board_plan_is_accepted()
    {
        assert_eq!(AUDIO_PLAN.validate(), Ok(()));
    }

    #[test]
    fn a_field_outside_its_width_is_refused()
    {
        let cases =
        [
            (
                ClockPlan::new(3_999_999, AUDIO_PLAN.pll(), 2, 64),
                ClockPlanError::SourceOutOfRange,
            ),
            (
                ClockPlan::new(48_000_001, AUDIO_PLAN.pll(), 2, 64),
                ClockPlanError::SourceOutOfRange,
            ),
            (
                with_pll(PllPlan::new(0, 94, 6_821, 21, VcoBand::Wide, ReferenceRange::Mhz4To8)),
                ClockPlanError::ReferenceDividerOutOfRange,
            ),
            (
                with_pll(PllPlan::new(64, 94, 6_821, 21, VcoBand::Wide, ReferenceRange::Mhz4To8)),
                ClockPlanError::ReferenceDividerOutOfRange,
            ),
            (
                with_pll(PllPlan::new(5, 3, 6_821, 21, VcoBand::Wide, ReferenceRange::Mhz4To8)),
                ClockPlanError::MultiplierOutOfRange,
            ),
            (
                with_pll(PllPlan::new(5, 513, 6_821, 21, VcoBand::Wide, ReferenceRange::Mhz4To8)),
                ClockPlanError::MultiplierOutOfRange,
            ),
            (
                with_pll(PllPlan::new(5, 94, 8_192, 21, VcoBand::Wide, ReferenceRange::Mhz4To8)),
                ClockPlanError::FractionOutOfRange,
            ),
            (
                with_pll(PllPlan::new(5, 94, 6_821, 0, VcoBand::Wide, ReferenceRange::Mhz4To8)),
                ClockPlanError::OutputDividerOutOfRange,
            ),
        ];

        for (plan, expected) in cases
        {
            assert_eq!(plan.validate(), Err(expected));
        }
    }

    #[test]
    fn the_modulator_is_refused_in_the_medium_band()
    {
        let plan = with_pll(PllPlan::new
        (
            13,
            156,
            1,
            8,
            VcoBand::Medium,
            ReferenceRange::Mhz1To2
        ));

        assert_eq!(plan.validate(), Err(ClockPlanError::FractionForbiddenInBand));

        // The same chain with an integer multiplier is legal, so the fraction
        // is what the refusal names.
        let integer = with_pll(PllPlan::new
        (
            13,
            156,
            0,
            8,
            VcoBand::Medium,
            ReferenceRange::Mhz1To2
        ));

        assert_eq!(integer.validate(), Ok(()));
    }

    #[test]
    fn a_reference_under_two_megahertz_is_refused_in_the_wide_band()
    {
        // 25 MHz over 13 is 1.923 MHz, which RM0433 accepts and the datasheet
        // does not.
        let plan = with_pll(PllPlan::new
        (
            13,
            208,
            0,
            8,
            VcoBand::Wide,
            ReferenceRange::Mhz1To2
        ));

        assert_eq!(plan.validate(), Err(ClockPlanError::ReferenceOutsideBand));
    }

    #[test]
    fn a_declared_range_the_reference_misses_is_refused()
    {
        for range in [ReferenceRange::Mhz1To2, ReferenceRange::Mhz2To4, ReferenceRange::Mhz8To16]
        {
            let plan = with_pll(PllPlan::new(5, 94, 6_821, 21, VcoBand::Wide, range));

            assert_eq!(plan.validate(), Err(ClockPlanError::ReferenceOutsideDeclaredRange));
        }
    }

    #[test]
    fn a_vco_past_the_band_ceiling_is_refused()
    {
        // 5 MHz times 243 is 1215 MHz, the figure a PLL fed from the wrong
        // oscillator reached while it reported locked.
        let plan = with_pll(PllPlan::new
        (
            5,
            243,
            0,
            21,
            VcoBand::Wide,
            ReferenceRange::Mhz4To8
        ));

        assert_eq!(plan.validate(), Err(ClockPlanError::VcoOutsideBand));
    }

    #[test]
    fn a_vco_inside_the_band_but_against_an_edge_is_refused()
    {
        // 5 MHz times 40 is 200 MHz, which clears the 192 MHz floor by 8 MHz
        // and not by 25.
        let low = with_pll(PllPlan::new(5, 40, 0, 21, VcoBand::Wide, ReferenceRange::Mhz4To8));

        assert_eq!(low.validate(), Err(ClockPlanError::VcoMarginTooSmall));

        // 5 MHz times 166 is 830 MHz, 6 MHz under the ceiling.
        let high = with_pll(PllPlan::new(5, 166, 0, 21, VcoBand::Wide, ReferenceRange::Mhz4To8));

        assert_eq!(high.validate(), Err(ClockPlanError::VcoMarginTooSmall));

        // 5 MHz times 44 is 220 MHz, which clears both edges by more than 25.
        let clear = with_pll(PllPlan::new(5, 44, 0, 21, VcoBand::Wide, ReferenceRange::Mhz4To8));

        assert_eq!(clear.validate(), Ok(()));
    }

    #[test]
    fn a_kernel_clock_past_what_the_part_guarantees_is_refused()
    {
        // 8 MHz times 75 is 600 MHz, and a divider of 7 leaves 85.7 MHz, past
        // the 75 MHz the kernel clock multiplexer takes at the voltage scale
        // the part boots on.
        let fast = ClockPlan::new
        (
            8_000_000,
            PllPlan::new(1, 75, 0, 7, VcoBand::Wide, ReferenceRange::Mhz8To16),
            2,
            64
        );

        assert_eq!(fast.validate(), Err(ClockPlanError::KernelAboveMaximum));

        // A divider of 8 on the same VCO lands on 75 MHz, which is the bound
        // itself.
        let edge = ClockPlan::new
        (
            8_000_000,
            PllPlan::new(1, 75, 0, 8, VcoBand::Wide, ReferenceRange::Mhz8To16),
            2,
            64
        );

        assert_eq!(edge.validate(), Ok(()));

        // 200 MHz clears what the datasheet gives a P output at that scale and
        // is refused all the same, because the multiplexer is the lower of the
        // two ceilings.
        let output = ClockPlan::new
        (
            8_000_000,
            PllPlan::new(1, 50, 0, 2, VcoBand::Wide, ReferenceRange::Mhz8To16),
            2,
            64
        );

        assert_eq!(output.validate(), Err(ClockPlanError::KernelAboveMaximum));
    }

    #[test]
    fn no_plan_can_drop_the_kernel_clock_under_its_floor()
    {
        // The widest divider on the lowest VCO of each band, which is the
        // slowest P output a plan reaches.
        for band in [VcoBand::Wide, VcoBand::Medium]
        {
            let slowest = Frequency::new(band.vco_min_hz(), u64::from(OUTPUT_DIVIDER_MAX));

            assert!(slowest.is_at_least(band.output_min_hz()));
        }
    }

    #[test]
    fn the_interface_dividers_are_checked()
    {
        let pll = AUDIO_PLAN.pll();

        let cases =
        [
            (ClockPlan::new(CRYSTAL_HZ, pll, 0, 64), ClockPlanError::MasterDividerOutOfRange),
            (ClockPlan::new(CRYSTAL_HZ, pll, 64, 64), ClockPlanError::MasterDividerOutOfRange),
            (ClockPlan::new(CRYSTAL_HZ, pll, 3, 64), ClockPlanError::MasterDividerOdd),
            (ClockPlan::new(CRYSTAL_HZ, pll, 1, 64), ClockPlanError::MasterDividerOdd),
            (ClockPlan::new(CRYSTAL_HZ, pll, 2, 4), ClockPlanError::FrameOutOfRange),
            (ClockPlan::new(CRYSTAL_HZ, pll, 2, 512), ClockPlanError::FrameOutOfRange),
            (ClockPlan::new(CRYSTAL_HZ, pll, 2, 48), ClockPlanError::FrameNotPowerOfTwo),
        ];

        for (plan, expected) in cases
        {
            assert_eq!(plan.validate(), Err(expected));
        }

        assert_eq!(ClockPlan::new(CRYSTAL_HZ, pll, 2, 8).validate(), Ok(()));
        assert_eq!(ClockPlan::new(CRYSTAL_HZ, pll, 2, 256).validate(), Ok(()));
    }

    #[test]
    fn a_zero_divider_leaves_every_frequency_undefined()
    {
        let plan = with_pll(PllPlan::new(0, 94, 6_821, 0, VcoBand::Wide, ReferenceRange::Mhz4To8));

        assert!(!plan.reference_hz().is_defined());
        assert!(!plan.vco_hz().is_defined());
        assert!(!plan.kernel_hz().is_defined());
        assert!(!plan.master_clock_hz().is_defined());
        assert!(!plan.bit_clock_hz().is_defined());
        assert!(plan.sample_rate_hz().as_hz().is_nan());
        assert!(!plan.reference_hz().is_at_least(0));
        assert!(!plan.reference_hz().is_at_most(u64::MAX));
        assert!(!plan.reference_hz().is_within_ppb(0, u64::MAX));
    }

    #[test]
    fn a_ratio_reduces_so_equal_frequencies_compare_equal()
    {
        assert_eq!(Frequency::new(10, 4), Frequency::new(5, 2));
        assert_eq!(Frequency::new(0, 7), Frequency::new(0, 3));
        assert_eq!(Frequency::new(7, 0), Frequency::UNDEFINED);
        assert_eq!(greatest_common_divisor(0, 0), 0);
        assert_eq!(greatest_common_divisor(48, 18), 6);
        assert_eq!(greatest_common_divisor(17, 0), 17);
    }

    #[test]
    fn a_ratio_too_wide_to_compare_answers_the_bound_correctly()
    {
        let third = Frequency::new(1, 3);

        // The scaled bound leaves the width of the type, and a third of a
        // hertz is under it either way.
        assert!(!third.is_at_least(u64::MAX));
        assert!(third.is_at_most(u64::MAX));
        assert!(!third.is_within_ppb(u64::MAX, 1));

        // The tolerance leaves the width of the type over a deviation of zero.
        let large = Frequency::new(1_000_000_000_000_000_000, 1);

        assert!(large.is_within_ppb(1_000_000_000_000_000_000, 100));
    }

    #[test]
    fn a_healthy_tree_comes_up_and_verifies()
    {
        let seen = verified_readback();

        assert_eq!(seen.source_bits, PLL_SOURCE_EXTERNAL);
        assert_eq!(seen.reference_divider_field, 5);
        assert_eq!(seen.multiplier_field, 93);
        assert_eq!(seen.fraction_field, 6_821);
        assert_eq!(seen.output_divider_field, 20);
        assert_eq!(seen.reference_range_bits, 0b10);
        assert_eq!(seen.audio_kernel_bits, AUDIO_KERNEL_PLL3_P);
        assert!(seen.fraction_latched);
        assert!(seen.output_enabled);
        assert!(seen.pll_on);
        assert!(seen.pll_ready);
        assert!(!seen.vco_band_bit);
    }

    #[test]
    fn a_plan_the_part_refuses_never_reaches_a_register()
    {
        let bad = with_pll(PllPlan::new(5, 243, 0, 21, VcoBand::Wide, ReferenceRange::Mhz4To8));
        let mut tree = MockTree::healthy();
        let before = tree.image;

        assert_eq!
        (
            bring_up(&mut tree, &bad, &waits()),
            Err(ClockFault::PlanRejected(ClockPlanError::VcoOutsideBand))
        );
        assert_eq!(tree.image, before);
    }

    #[test]
    fn an_oscillator_that_never_reports_ready_is_a_fault()
    {
        let mut tree = MockTree::healthy();
        tree.source_ready_after = u32::MAX;

        assert_eq!
        (
            bring_up(&mut tree, &AUDIO_PLAN, &ClockWaits { source_polls: 4, pll_polls: 4 }),
            Err(ClockFault::SourceNeverReady)
        );
    }

    #[test]
    fn a_pll_that_never_locks_is_a_fault()
    {
        let mut tree = MockTree::healthy();
        tree.pll_ready_after = u32::MAX;

        assert_eq!
        (
            bring_up(&mut tree, &AUDIO_PLAN, &ClockWaits { source_polls: 4, pll_polls: 4 }),
            Err(ClockFault::PllNeverLocked)
        );
    }

    #[test]
    fn a_pll_that_will_not_stop_is_a_fault()
    {
        let mut tree = MockTree::healthy();
        tree.pll_stays_on = true;
        tree.image.pll_on = true;
        tree.pll_ready_after = 0;

        assert_eq!
        (
            bring_up(&mut tree, &AUDIO_PLAN, &ClockWaits { source_polls: 4, pll_polls: 4 }),
            Err(ClockFault::PllNeverStopped)
        );
    }

    #[test]
    fn a_latch_that_will_not_clear_is_a_fault()
    {
        let mut tree = MockTree::healthy();
        tree.keep_fraction_latched = true;
        tree.image.fraction_latched = true;

        assert_eq!
        (
            bring_up(&mut tree, &AUDIO_PLAN, &waits()),
            Err(ClockFault::FractionLatchNotCleared)
        );
    }

    #[test]
    fn a_pll_running_next_door_refuses_the_bring_up_before_the_source_write()
    {
        for (pll1_on, pll2_on) in [(true, false), (false, true), (true, true)]
        {
            let mut tree = MockTree::healthy();
            tree.image.pll1_on = pll1_on;
            tree.image.pll2_on = pll2_on;

            assert_eq!
            (
                bring_up(&mut tree, &AUDIO_PLAN, &waits()),
                Err(ClockFault::OtherPllRunning)
            );

            // The refusal lands before the write the part would have dropped.
            assert_eq!(tree.image.source_bits, 0);
            assert_eq!(tree.image.reference_divider_field, 0b10_0000);
        }
    }

    #[test]
    fn a_pll_that_comes_back_on_after_it_was_stopped_refuses_the_bring_up()
    {
        let mut tree = MockTree::healthy();
        tree.pll_restarts_with_source = true;

        assert_eq!
        (
            bring_up(&mut tree, &AUDIO_PLAN, &waits()),
            Err(ClockFault::OtherPllRunning)
        );

        assert_eq!(tree.image.source_bits, 0);
    }

    #[test]
    fn every_fault_names_itself_with_a_code_of_its_own()
    {
        let bounds =
        [
            ClockPlanError::SourceOutOfRange,
            ClockPlanError::ReferenceDividerOutOfRange,
            ClockPlanError::MultiplierOutOfRange,
            ClockPlanError::FractionOutOfRange,
            ClockPlanError::OutputDividerOutOfRange,
            ClockPlanError::FractionForbiddenInBand,
            ClockPlanError::ReferenceOutsideBand,
            ClockPlanError::ReferenceOutsideDeclaredRange,
            ClockPlanError::VcoOutsideBand,
            ClockPlanError::VcoMarginTooSmall,
            ClockPlanError::KernelAboveMaximum,
            ClockPlanError::MasterDividerOutOfRange,
            ClockPlanError::MasterDividerOdd,
            ClockPlanError::FrameOutOfRange,
            ClockPlanError::FrameNotPowerOfTwo,
        ];

        let faults =
        [
            ClockFault::PllNeverStopped,
            ClockFault::OtherPllRunning,
            ClockFault::FractionLatchNotCleared,
            ClockFault::SourceNeverReady,
            ClockFault::PllNeverLocked,
            ClockFault::SourceNotSelected,
            ClockFault::SourceBypassed,
            ClockFault::SourceNotReady,
            ClockFault::ReferenceDividerWrong,
            ClockFault::MultiplierWrong,
            ClockFault::FractionWrong,
            ClockFault::OutputDividerWrong,
            ClockFault::ReferenceRangeWrong,
            ClockFault::VcoBandWrong,
            ClockFault::FractionNotLatched,
            ClockFault::OutputDisabled,
            ClockFault::PllNotEnabled,
            ClockFault::PllNotLocked,
            ClockFault::AudioKernelSourceWrong,
        ];

        let mut codes = [0_u32; 34];

        // A zip stops at the shorter side, so a variant added to either list
        // above would go unread rather than unnumbered.
        assert_eq!(bounds.len() + faults.len(), codes.len());

        let seen = bounds
            .into_iter()
            .map(|bound| ClockFault::PlanRejected(bound).code())
            .chain(faults.into_iter().map(ClockFault::code));

        for (slot, code) in codes.iter_mut().zip(seen)
        {
            *slot = code;
        }

        for (index, code) in codes.into_iter().enumerate()
        {
            assert_ne!(code, 0, "fault {index} carries no code");
            assert_eq!
            (
                codes.iter().filter(|other| **other == code).count(),
                1,
                "code {code:#06x} is carried by more than one fault"
            );
        }

        // A refusal names both halves in one word.
        assert_eq!
        (
            ClockFault::PlanRejected(ClockPlanError::VcoOutsideBand).code(),
            PLAN_REJECTED_FLAG | ClockPlanError::VcoOutsideBand.code()
        );
        assert_eq!(ClockFault::PlanRejected(ClockPlanError::VcoOutsideBand).code(), 0x0109);
        assert_eq!(ClockFault::AudioKernelSourceWrong.code(), 0x0013);
    }

    #[test]
    fn a_source_selection_that_does_not_take_is_a_fault()
    {
        let mut tree = MockTree::healthy();
        tree.refuse_source_write = true;

        assert_eq!
        (
            bring_up(&mut tree, &AUDIO_PLAN, &waits()),
            Err(ClockFault::SourceNotSelected)
        );
    }

    #[test]
    fn every_field_the_frequency_depends_on_is_checked()
    {
        let sound = verified_readback();

        let cases: [Mutation; 14] =
        [
            (|seen| seen.source_bits = 0, ClockFault::SourceNotSelected),
            (|seen| seen.source_bypassed = true, ClockFault::SourceBypassed),
            (|seen| seen.source_ready = false, ClockFault::SourceNotReady),
            (|seen| seen.reference_divider_field = 4, ClockFault::ReferenceDividerWrong),
            (|seen| seen.multiplier_field = 94, ClockFault::MultiplierWrong),
            (|seen| seen.fraction_field = 0, ClockFault::FractionWrong),
            (|seen| seen.output_divider_field = 21, ClockFault::OutputDividerWrong),
            (|seen| seen.reference_range_bits = 3, ClockFault::ReferenceRangeWrong),
            (|seen| seen.vco_band_bit = true, ClockFault::VcoBandWrong),
            (|seen| seen.fraction_latched = false, ClockFault::FractionNotLatched),
            (|seen| seen.output_enabled = false, ClockFault::OutputDisabled),
            (|seen| seen.pll_on = false, ClockFault::PllNotEnabled),
            (|seen| seen.pll_ready = false, ClockFault::PllNotLocked),
            (|seen| seen.audio_kernel_bits = 0, ClockFault::AudioKernelSourceWrong),
        ];

        assert_eq!(AUDIO_PLAN.verify(&sound), Ok(()));

        for (break_one, expected) in cases
        {
            let mut seen = sound;
            break_one(&mut seen);

            assert_eq!(AUDIO_PLAN.verify(&seen), Err(expected));
        }
    }

    #[test]
    fn a_locked_pll_on_the_internal_oscillator_is_still_a_fault()
    {
        // The bit a bring-up would wait on, over the source a reset leaves.
        let mut seen = verified_readback();
        seen.source_bits = 0;

        assert!(seen.pll_ready);
        assert_eq!(AUDIO_PLAN.verify(&seen), Err(ClockFault::SourceNotSelected));
    }

    #[test]
    fn the_encoding_offsets_are_what_the_verification_compares()
    {
        // The field carries one less than the divider, so the divider itself
        // sitting in the register is a disagreement.
        let mut seen = verified_readback();
        seen.multiplier_field = AUDIO_PLAN.pll().multiplier();

        assert_eq!(AUDIO_PLAN.verify(&seen), Err(ClockFault::MultiplierWrong));

        let mut seen = verified_readback();
        seen.output_divider_field = AUDIO_PLAN.pll().output_divider();

        assert_eq!(AUDIO_PLAN.verify(&seen), Err(ClockFault::OutputDividerWrong));
    }

    #[test]
    fn a_wait_covers_the_time_it_names()
    {
        // 64 MHz for 20 ms is 1 280 000 polls, and a poll is at least a cycle.
        assert_eq!(wait_polls(64_000_000, 20_000), 1_280_000);
        assert_eq!(wait_polls(64_000_000, 10_000), 640_000);
        assert_eq!(ClockWaits::for_core_clock(64_000_000).source_polls, 1_280_000);
        assert_eq!(ClockWaits::for_core_clock(64_000_000).pll_polls, 640_000);
    }

    #[test]
    fn a_wait_rounds_up_and_saturates_rather_than_wrapping()
    {
        assert_eq!(wait_polls(1, 1), 1);
        assert_eq!(wait_polls(3, 1), 1);
        assert_eq!(wait_polls(0, 20_000), 0);
        assert_eq!(wait_polls(u32::MAX, u32::MAX), u32::MAX);
        assert!(wait_polls_wide(u32::MAX, u32::MAX) > u64::from(u32::MAX));
    }

    #[test]
    fn a_wait_of_zero_still_buys_one_look()
    {
        let mut tree = MockTree::healthy();
        tree.source_ready_after = 1;
        tree.pll_ready_after = 1;

        assert_eq!
        (
            bring_up(&mut tree, &AUDIO_PLAN, &ClockWaits { source_polls: 0, pll_polls: 0 }),
            Ok(())
        );
    }
}
