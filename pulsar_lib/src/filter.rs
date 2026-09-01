//! Biquad coefficients for the crossover and the protective high-pass.
//!
//! Every section is normalised on a0 and carries the five coefficients of the
//! RBJ Audio EQ Cookbook equation 2, in the same form the direct form 1
//! difference equation of that document runs.
//!
//! The build runs in double precision and narrows once, because the audio path
//! carries single precision floats. The subsonic high-pass carries the
//! narrowing worst: its poles sit closest to
//! z = 1, at radius 0.9961 and 0.9984, so rounding a coefficient moves them the
//! furthest. It reads 0.0101 dB away from its double precision design at 60 Hz
//! and 0.0040 dB at 100 Hz, both inside its pass band, and the gap widens as
//! the frequency falls, 0.0442 dB at 10 Hz where the filter is 38 dB down and
//! 0.0478 dB at 1 Hz where it is 118 dB down. The four Linkwitz-Riley filters
//! stay under 0.0003 dB from 0.1 Hz to Nyquist, and their high-passes widen
//! below that as the narrowed numerator cancels.
//! `the_coefficients_match_a_double_precision_build` is what bounds the
//! narrowing, by pinning each coefficient to two units in the last place of an
//! external double precision reference.
//!
//! Narrowing also decides stability. A rounded pair whose poles sit near the
//! unit circle can land on or outside it, so every section faces the Jury
//! triangle after it narrows and a design that fails is refused.
//!
//! Nothing here allocates. A cascade fills a buffer the caller owns and returns
//! how many sections it wrote.

use crate::constants::{LOW_MID_HZ, MID_HIGH_HZ, SAMPLE_RATE_HZ, SUBSONIC_HZ};
use core::f64::consts::{FRAC_1_SQRT_2, PI, TAU};
use libm::{cos, fabs, sin, sqrt};

/// Highest cascade order the module builds.
///
/// The cap sizes the buffer a caller reserves and bounds the sections one call
/// can write. `MAX_SECTIONS` follows from it.
pub const MAX_ORDER: u32 = 8;

/// Sections a cascade of `MAX_ORDER` fills.
pub const MAX_SECTIONS: usize = MAX_ORDER as usize / 2;

/// Order of both halves of a Linkwitz-Riley crossover corner.
///
/// Both halves of a corner run fourth order, so each falls at 24 dB per
/// octave and the two sum flat.
pub const LINKWITZ_RILEY_ORDER: u32 = 4;

/// Sections a `LINKWITZ_RILEY_ORDER` cascade fills.
pub const LINKWITZ_RILEY_SECTIONS: usize = LINKWITZ_RILEY_ORDER as usize / 2;

/// Order of the subsonic Butterworth high-pass.
///
/// Fourth order, so 24 dB per octave below the corner.
pub const SUBSONIC_ORDER: u32 = 4;

/// Sections a `SUBSONIC_ORDER` cascade fills.
pub const SUBSONIC_SECTIONS: usize = SUBSONIC_ORDER as usize / 2;

/// Sections the most demanding way of the crossover fills.
///
/// The low way carries the subsonic high-pass and one crossover half, and no
/// other way carries more. A buffer of this length holds any way.
pub const CROSSOVER_SECTIONS: usize = SUBSONIC_SECTIONS + LINKWITZ_RILEY_SECTIONS;

/// Reason a coefficient build refuses its arguments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterError
{
    /// The corner frequency is zero, negative, or not a number.
    FrequencyNotPositive,
    /// The corner frequency reaches or passes half the sample rate.
    FrequencyNotBelowNyquist,
    /// The quality factor is zero, negative, infinite, or not a number.
    QualityNotPositive,
    /// The sample rate is zero.
    SampleRateZero,
    /// The order is zero, odd, or above `MAX_ORDER`.
    OrderInvalid,
    /// The output buffer holds fewer slots than the cascade has sections.
    OutputTooShort,
    /// The narrowed section places a pole on or outside the unit circle.
    ///
    /// The Jury triangle of a normalised second order denominator, `a2 < 1` and
    /// `|a1| < 1 + a2`, read on the single precision coefficients the chain
    /// feeds back. A corner within a couple of hertz of either edge of the open
    /// band, and a quality factor near the largest the format carries, both
    /// round to a pair that fails it.
    UnstableDesign,
}

/// One way of the loudspeaker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Way
{
    /// Low way, from the subsonic corner up to the low crossover corner.
    Low,
    /// Mid way, between the two crossover corners.
    Mid,
    /// High way, above the high crossover corner.
    High,
}

/// One second order section, normalised on a0.
///
/// The chain runs it as the RBJ Audio EQ Cookbook equation 4:
/// `y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Biquad
{
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
}

impl Biquad
{
    /// Section that produces silence.
    ///
    /// A caller seeds its cascade buffer with this before a build. A slot no
    /// build reached then stops the signal instead of sending the full range
    /// into a way that has no filter on it.
    pub const SILENT: Self = Self
    {
        b0: 0.0,
        b1: 0.0,
        b2: 0.0,
        a1: 0.0,
        a2: 0.0,
    };

    /// Returns the coefficient on the current input sample.
    #[must_use]
    pub fn b0(self) -> f32
    {
        self.b0
    }

    /// Returns the coefficient on the input sample one step back.
    #[must_use]
    pub fn b1(self) -> f32
    {
        self.b1
    }

    /// Returns the coefficient on the input sample two steps back.
    #[must_use]
    pub fn b2(self) -> f32
    {
        self.b2
    }

    /// Returns the coefficient on the output sample one step back.
    #[must_use]
    pub fn a1(self) -> f32
    {
        self.a1
    }

    /// Returns the coefficient on the output sample two steps back.
    #[must_use]
    pub fn a2(self) -> f32
    {
        self.a2
    }

    /// Narrows a double precision design to the format the audio path carries.
    ///
    /// The stability check runs on the narrowed values, because those are the
    /// coefficients the chain feeds back. A design whose poles sit near the
    /// unit circle can cross it on the way through this step.
    ///
    /// Errors: `UnstableDesign`.
    #[expect
    (
        clippy::cast_possible_truncation,
        reason = "the audio path is single precision, and this is the step that gets it there"
    )]
    fn narrow
    (
        b0: f64,
        b1: f64,
        b2: f64,
        a1: f64,
        a2: f64
    ) -> Result<Self, FilterError>
    {
        let narrowed = Self
        {
            b0: b0 as f32,
            b1: b1 as f32,
            b2: b2 as f32,
            a1: a1 as f32,
            a2: a2 as f32,
        };
        if narrowed.poles_are_inside_the_unit_circle()
        {
            Ok(narrowed)
        }
        else
        {
            Err(FilterError::UnstableDesign)
        }
    }

    /// Returns whether both poles sit strictly inside the unit circle.
    ///
    /// The Jury triangle of a normalised second order denominator: `a2 < 1` and
    /// `|a1| < 1 + a2`. The comparison widens the single precision coefficients
    /// first, so the sum it forms carries no rounding of its own. A coefficient
    /// that is not a number fails every comparison, so a section carrying one
    /// is refused.
    fn poles_are_inside_the_unit_circle(self) -> bool
    {
        let a1 = f64::from(self.a1);
        let a2 = f64::from(self.a2);
        a2 < 1.0 && fabs(a1) < 1.0 + a2
    }
}

impl Default for Biquad
{
    /// Returns `SILENT`.
    fn default() -> Self
    {
        Self::SILENT
    }
}

/// Fills `out` with the protective crossover of `way`.
///
/// The low way takes the subsonic Butterworth high-pass then the
/// Linkwitz-Riley low-pass at `LOW_MID_HZ`, the mid way takes the
/// Linkwitz-Riley pair bounding its band, and the high way takes the
/// Linkwitz-Riley high-pass at `MID_HIGH_HZ`. The sections land in the order
/// the chain runs them.
///
/// `CROSSOVER_SECTIONS` slots hold any way. Returns the number of sections
/// written, and `out` keeps whatever it held past that point.
///
/// The length of the whole way is checked before the first write, so a buffer
/// too short comes back untouched. A caller that seeded its buffer with
/// `SILENT` and dropped the `Result` is then left with silence rather than
/// with the half of a way that fits.
///
/// # Errors
///
/// Returns `OutputTooShort` when `out` holds fewer slots than the way fills,
/// and `UnstableDesign` when a narrowed section places a pole on or outside the
/// unit circle.
pub fn crossover_sections(way: Way, out: &mut [Biquad]) -> Result<usize, FilterError>
{
    if out.len() < way_sections(way)
    {
        return Err(FilterError::OutputTooShort);
    }
    match way
    {
        Way::Low =>
        {
            let written = butterworth_high_pass(SUBSONIC_HZ, SUBSONIC_ORDER, SAMPLE_RATE_HZ, out)?;
            append(out, written, |rest| linkwitz_riley_low_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, rest))
        }
        Way::Mid =>
        {
            let written = linkwitz_riley_high_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, out)?;
            append(out, written, |rest| linkwitz_riley_low_pass(MID_HIGH_HZ, SAMPLE_RATE_HZ, rest))
        }
        Way::High => linkwitz_riley_high_pass(MID_HIGH_HZ, SAMPLE_RATE_HZ, out),
    }
}

/// Returns the sections `way` fills.
///
/// `crossover_sections` reads it before it writes anything, which is what makes
/// a refusal on the buffer length leave that buffer as it was.
const fn way_sections(way: Way) -> usize
{
    match way
    {
        Way::Low => CROSSOVER_SECTIONS,
        Way::Mid => LINKWITZ_RILEY_SECTIONS.saturating_mul(2),
        Way::High => LINKWITZ_RILEY_SECTIONS,
    }
}

/// Runs `build` on the slots of `out` past `written`, returning the total.
///
/// Errors: `OutputTooShort`, plus whatever `build` refuses.
fn append<F>(out: &mut [Biquad], written: usize, build: F) -> Result<usize, FilterError>
where
    F: FnOnce(&mut [Biquad]) -> Result<usize, FilterError>,
{
    let rest = out.get_mut(written..).ok_or(FilterError::OutputTooShort)?;
    Ok(written.saturating_add(build(rest)?))
}

/// Builds the cookbook low-pass section.
///
/// The RBJ Audio EQ Cookbook LPF, with `alpha = sin(w0) / (2 Q)`.
///
/// # Errors
///
/// Returns `SampleRateZero` for a rate of zero, `FrequencyNotPositive` or
/// `FrequencyNotBelowNyquist` for a corner outside the open band,
/// `QualityNotPositive` for a quality factor at or below zero or not finite,
/// and `UnstableDesign` when the narrowed section places a pole on or outside
/// the unit circle.
pub fn low_pass
(
    corner_hz: f32,
    quality: f32,
    sample_rate_hz: u32
) -> Result<Biquad, FilterError>
{
    let angular = angular_frequency(corner_hz, sample_rate_hz)?;
    let quality = checked_quality(quality)?;
    low_pass_at(angular, quality)
}

/// Builds the cookbook high-pass section.
///
/// The RBJ Audio EQ Cookbook HPF, with `alpha = sin(w0) / (2 Q)`.
///
/// # Errors
///
/// Returns `SampleRateZero` for a rate of zero, `FrequencyNotPositive` or
/// `FrequencyNotBelowNyquist` for a corner outside the open band,
/// `QualityNotPositive` for a quality factor at or below zero or not finite,
/// and `UnstableDesign` when the narrowed section places a pole on or outside
/// the unit circle.
pub fn high_pass
(
    corner_hz: f32,
    quality: f32,
    sample_rate_hz: u32
) -> Result<Biquad, FilterError>
{
    let angular = angular_frequency(corner_hz, sample_rate_hz)?;
    let quality = checked_quality(quality)?;
    high_pass_at(angular, quality)
}

/// Fills `out` with a Butterworth high-pass cascade of `order`.
///
/// The `order / 2` sections share the corner and take the quality factors of
/// the Butterworth pole angles, `Q_k = 1 / (2 cos((2k+1) pi / (2N)))` for k
/// from zero to `N/2 - 1`. No even order places a pole angle at pi/2, so the
/// cosine stays away from zero.
///
/// Returns the number of sections written. `out` keeps whatever it held past
/// that point.
///
/// # Errors
///
/// Returns `SampleRateZero` for a rate of zero, `FrequencyNotPositive` or
/// `FrequencyNotBelowNyquist` for a corner outside the open band,
/// `OrderInvalid` for an order that is zero, odd, or above `MAX_ORDER`,
/// `OutputTooShort` when `out` holds fewer slots than the cascade has sections,
/// and `UnstableDesign` when a narrowed section places a pole on or outside the
/// unit circle.
pub fn butterworth_high_pass
(
    corner_hz: f32,
    order: u32,
    sample_rate_hz: u32,
    out: &mut [Biquad]
) -> Result<usize, FilterError>
{
    let angular = angular_frequency(corner_hz, sample_rate_hz)?;
    let sections = section_count(order)?;
    if out.len() < sections
    {
        return Err(FilterError::OutputTooShort);
    }

    let step = PI / f64::from(order);
    let mut pole_angle = step / 2.0;
    for slot in out.iter_mut().take(sections)
    {
        *slot = high_pass_at(angular, 1.0 / (2.0 * cos(pole_angle)))?;
        pole_angle += step;
    }
    Ok(sections)
}

/// Fills `out` with a fourth order Linkwitz-Riley low-pass at `corner_hz`.
///
/// Two second order Butterworth sections at the same corner, each at
/// `Q = 1/sqrt(2)`. The pair reads -6.02 dB at the corner, which is where it
/// meets the matching high-pass.
///
/// Returns the number of sections written, `LINKWITZ_RILEY_SECTIONS`. `out`
/// keeps whatever it held past that point.
///
/// # Errors
///
/// Returns `SampleRateZero` for a rate of zero, `FrequencyNotPositive` or
/// `FrequencyNotBelowNyquist` for a corner outside the open band,
/// `OutputTooShort` when `out` holds fewer than `LINKWITZ_RILEY_SECTIONS`
/// slots, and `UnstableDesign` when the narrowed section places a pole on or
/// outside the unit circle.
pub fn linkwitz_riley_low_pass
(
    corner_hz: f32,
    sample_rate_hz: u32,
    out: &mut [Biquad]
) -> Result<usize, FilterError>
{
    let angular = angular_frequency(corner_hz, sample_rate_hz)?;
    write_pair(out, low_pass_at(angular, FRAC_1_SQRT_2)?)
}

/// Fills `out` with a fourth order Linkwitz-Riley high-pass at `corner_hz`.
///
/// Two second order Butterworth sections at the same corner, each at
/// `Q = 1/sqrt(2)`. Summed with the low-pass built at the same corner, the
/// magnitude is flat, which is what makes the pair a crossover.
///
/// Returns the number of sections written, `LINKWITZ_RILEY_SECTIONS`. `out`
/// keeps whatever it held past that point.
///
/// # Errors
///
/// Returns `SampleRateZero` for a rate of zero, `FrequencyNotPositive` or
/// `FrequencyNotBelowNyquist` for a corner outside the open band,
/// `OutputTooShort` when `out` holds fewer than `LINKWITZ_RILEY_SECTIONS`
/// slots, and `UnstableDesign` when the narrowed section places a pole on or
/// outside the unit circle.
pub fn linkwitz_riley_high_pass
(
    corner_hz: f32,
    sample_rate_hz: u32,
    out: &mut [Biquad]
) -> Result<usize, FilterError>
{
    let angular = angular_frequency(corner_hz, sample_rate_hz)?;
    write_pair(out, high_pass_at(angular, FRAC_1_SQRT_2)?)
}

/// Returns the magnitude of `sections` at `frequency_hz`, as a linear ratio.
///
/// The evaluation reads the single precision coefficients the sections hold and
/// runs in double precision, so it measures the cascade the chain runs rather
/// than the design it came from. An empty cascade reads one.
///
/// The preset authoring tool plots its response curves from here, so a curve
/// carries the coefficients the machine runs and not a second implementation of
/// them.
///
/// # Errors
///
/// Returns `SampleRateZero` for a rate of zero, and `FrequencyNotPositive` or
/// `FrequencyNotBelowNyquist` for a frequency outside the open band the chain
/// can represent.
pub fn cascade_magnitude
(
    sections: &[Biquad],
    frequency_hz: f32,
    sample_rate_hz: u32
) -> Result<f64, FilterError>
{
    let angular = angular_frequency(frequency_hz, sample_rate_hz)?;
    let (real, imaginary) = cascade_response(sections, angular);
    Ok(sqrt(real * real + imaginary * imaginary))
}

/// Returns the complex response of `sections` at angular frequency `angular`.
///
/// The two unit vectors are formed once, so a cascade of any length costs the
/// four trigonometric calls that build them.
fn cascade_response(sections: &[Biquad], angular: f64) -> (f64, f64)
{
    let unit_cos = cos(-angular);
    let unit_sin = sin(-angular);
    let double_cos = cos(-2.0 * angular);
    let double_sin = sin(-2.0 * angular);

    let mut real = 1.0;
    let mut imaginary = 0.0;
    for section in sections
    {
        let b0 = f64::from(section.b0);
        let b1 = f64::from(section.b1);
        let b2 = f64::from(section.b2);
        let a1 = f64::from(section.a1);
        let a2 = f64::from(section.a2);

        let top_real = b0 + b1 * unit_cos + b2 * double_cos;
        let top_imaginary = b1 * unit_sin + b2 * double_sin;
        let bottom_real = 1.0 + a1 * unit_cos + a2 * double_cos;
        let bottom_imaginary = a1 * unit_sin + a2 * double_sin;
        let bottom = bottom_real * bottom_real + bottom_imaginary * bottom_imaginary;

        let here_real = (top_real * bottom_real + top_imaginary * bottom_imaginary) / bottom;
        let here_imaginary = (top_imaginary * bottom_real - top_real * bottom_imaginary) / bottom;

        let carried_real = real * here_real - imaginary * here_imaginary;
        imaginary = real * here_imaginary + imaginary * here_real;
        real = carried_real;
    }
    (real, imaginary)
}

/// Builds the cookbook low-pass section from a checked angular frequency.
///
/// Errors: `UnstableDesign`.
fn low_pass_at(angular: f64, quality: f64) -> Result<Biquad, FilterError>
{
    let cos_w0 = cos(angular);
    let alpha = sin(angular) / (2.0 * quality);
    let a0 = 1.0 + alpha;
    let opening = 1.0 - cos_w0;
    let shared = opening / 2.0;
    Biquad::narrow
    (
        shared / a0,
        opening / a0,
        shared / a0,
        -2.0 * cos_w0 / a0,
        (1.0 - alpha) / a0,
    )
}

/// Builds the cookbook high-pass section from a checked angular frequency.
///
/// Errors: `UnstableDesign`.
fn high_pass_at(angular: f64, quality: f64) -> Result<Biquad, FilterError>
{
    let cos_w0 = cos(angular);
    let alpha = sin(angular) / (2.0 * quality);
    let a0 = 1.0 + alpha;
    let opening = 1.0 + cos_w0;
    let shared = opening / 2.0;
    Biquad::narrow
    (
        shared / a0,
        -opening / a0,
        shared / a0,
        -2.0 * cos_w0 / a0,
        (1.0 - alpha) / a0,
    )
}

/// Writes `section` into the first `LINKWITZ_RILEY_SECTIONS` slots of `out`.
///
/// Errors: `OutputTooShort` when `out` is shorter than the pair.
fn write_pair(out: &mut [Biquad], section: Biquad) -> Result<usize, FilterError>
{
    if out.len() < LINKWITZ_RILEY_SECTIONS
    {
        return Err(FilterError::OutputTooShort);
    }
    for slot in out.iter_mut().take(LINKWITZ_RILEY_SECTIONS)
    {
        *slot = section;
    }
    Ok(LINKWITZ_RILEY_SECTIONS)
}

/// Returns the angular frequency of `corner_hz` at `sample_rate_hz`.
///
/// Errors: `SampleRateZero`, `FrequencyNotPositive`, `FrequencyNotBelowNyquist`.
fn angular_frequency(corner_hz: f32, sample_rate_hz: u32) -> Result<f64, FilterError>
{
    if sample_rate_hz == 0
    {
        return Err(FilterError::SampleRateZero);
    }
    if corner_hz.is_nan() || corner_hz <= 0.0
    {
        return Err(FilterError::FrequencyNotPositive);
    }
    let corner = f64::from(corner_hz);
    let sample_rate = f64::from(sample_rate_hz);
    if corner * 2.0 >= sample_rate
    {
        return Err(FilterError::FrequencyNotBelowNyquist);
    }
    Ok(TAU * corner / sample_rate)
}

/// Returns `quality` widened, once it is a usable quality factor.
///
/// An infinite quality factor drives `alpha` to zero, which puts `a2` at
/// exactly one and both poles on the unit circle, so it is refused here rather
/// than left to the stability check.
///
/// Errors: `QualityNotPositive`.
fn checked_quality(quality: f32) -> Result<f64, FilterError>
{
    if !quality.is_finite() || quality <= 0.0
    {
        return Err(FilterError::QualityNotPositive);
    }
    Ok(f64::from(quality))
}

/// Returns the sections a cascade of `order` fills.
///
/// The table is the whole definition of the admissible orders: even, non zero,
/// and no higher than `MAX_ORDER`.
///
/// Errors: `OrderInvalid`.
const fn section_count(order: u32) -> Result<usize, FilterError>
{
    match order
    {
        2 => Ok(1),
        4 => Ok(2),
        6 => Ok(3),
        8 => Ok(4),
        _ => Err(FilterError::OrderInvalid),
    }
}

const _: () = assert!
(
    matches!(section_count(MAX_ORDER), Ok(MAX_SECTIONS)),
    "the section table reaches the highest order the module builds"
);

const _: () = assert!
(
    matches!(section_count(MAX_ORDER.saturating_add(2)), Err(FilterError::OrderInvalid)),
    "the section table refuses an order past the cap"
);

const _: () = assert!
(
    matches!(section_count(LINKWITZ_RILEY_ORDER), Ok(LINKWITZ_RILEY_SECTIONS)),
    "the crossover section count follows the order it is derived from"
);

const _: () = assert!
(
    matches!(section_count(SUBSONIC_ORDER), Ok(SUBSONIC_SECTIONS)),
    "the subsonic section count follows the order it is derived from"
);

const _: () = assert!
(
    CROSSOVER_SECTIONS <= MAX_SECTIONS,
    "the widest way fits the buffer a caller sizes from the cap"
);

const _: () = assert!
(
    way_sections(Way::Low) <= CROSSOVER_SECTIONS
        && way_sections(Way::Mid) <= CROSSOVER_SECTIONS
        && way_sections(Way::High) <= CROSSOVER_SECTIONS,
    "the buffer constant covers the length every way checks against"
);

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, which is the one place the
    // no-panic rule does not hold. Slicing a fixture buffer by a literal bound
    // is part of what the buffer tests measure.
    #![allow(clippy::panic, clippy::indexing_slicing)]

    use super::*;
    use crate::constants::NYQUIST_HZ;
    use libm::log10;

    /// Level one halving of amplitude reads, in decibels.
    ///
    /// 20 log10 2. Every level below is a multiple of it.
    const HALVING_DB: f64 = 6.020_599_913_279_624;

    /// Order every filter of the loudspeaker runs.
    ///
    /// All five are fourth order, so every slope below is a multiple of it.
    const PRODUCT_ORDER: f64 = 4.0;

    /// Level a Butterworth corner reads, in decibels.
    ///
    /// Half power, so half a halving of amplitude.
    const BUTTERWORTH_CORNER_DB: f64 = -HALVING_DB / 2.0;

    /// Level a Linkwitz-Riley corner reads, in decibels.
    ///
    /// Two Butterworth sections in cascade, so twice the level of one.
    const LINKWITZ_RILEY_CORNER_DB: f64 = 2.0 * BUTTERWORTH_CORNER_DB;

    /// Attenuation a fourth order asymptote adds over one octave, in decibels.
    ///
    /// One halving of amplitude per order. The figure is 24.0824, and the
    /// 0.0824 dB between it and the named 24 dB per octave eats 82 percent of
    /// the tolerance below, which is why the constant is derived here rather
    /// than written out.
    const OCTAVE_DB: f64 = HALVING_DB * PRODUCT_ORDER;

    /// Ratio between the two ends of an octave.
    const OCTAVE_RATIO: f32 = 2.0;

    /// Ratio between a high-pass corner and the octave its asymptote is read
    /// over.
    ///
    /// A decade below the corner, a fourth order high-pass reads the asymptote
    /// to well inside `SLOPE_TOLERANCE_DB`.
    const STOP_BAND_DECADE: f32 = 10.0;

    /// Tolerance on a corner level, in decibels.
    const CORNER_TOLERANCE_DB: f64 = 0.01;

    /// Tolerance on an octave of stop band slope, in decibels.
    ///
    /// The bilinear transform bends the frequency axis, so no band reads the
    /// asymptote exactly. The high-pass bands below stay inside this.
    const SLOPE_TOLERANCE_DB: f64 = 0.10;

    /// Tolerance on the magnitude of a summed crossover pair, in decibels.
    const SUM_TOLERANCE_DB: f64 = 0.01;

    /// Tolerance on a coefficient against its double precision reference.
    ///
    /// Two units in the last place of the single precision format. A build run
    /// in single precision throughout lands up to 1.2e-5 away in relative
    /// terms, fifty times this.
    const COEFFICIENT_TOLERANCE: f32 = 2.0 * f32::EPSILON;

    /// A cascade build with its corner and rate already bound.
    type Build = fn(&mut [Biquad]) -> Result<usize, FilterError>;

    /// How a fixture's stop band slope gets read.
    #[derive(Clone, Copy)]
    enum Slope
    {
        /// A high-pass, read over the octave a decade below its corner. Both
        /// the distance from the corner and the distance from Nyquist leave
        /// the asymptote holding in both directions there.
        HighPass,
        /// A low-pass, read over the octave starting at `start` times its
        /// corner. The bilinear transform puts a double zero at Nyquist, which
        /// only steepens a low-pass, so the asymptote is a floor there rather
        /// than a target. An octave that does read it reads it by cancellation.
        /// 900 to 1800 Hz on the 300 Hz low-pass lands 0.044 dB out, the analog
        /// shape being 0.100 dB short of the asymptote there and the warping
        /// 0.144 dB past it. The next octave up is 0.576 dB out and the one
        /// after 2.475 dB, so a two sided band would pin that cancellation
        /// rather than a property.
        ///
        /// The floor also bounds the corner from below. The margin over the
        /// band falls with the corner and reaches zero near a 250 Hz corner at
        /// a `start` of 3, and near 825 Hz at a `start` of 2. Under those, a
        /// filter of the specified slope reads as a defect.
        LowPass
        {
            /// Where the octave starts, as a multiple of the corner.
            start: f32,
        },
    }

    impl Slope
    {
        /// Returns the octave the slope is read over, low end first.
        fn octave(self, corner_hz: f32) -> (f32, f32)
        {
            let low_hz = match self
            {
                Self::HighPass => corner_hz / STOP_BAND_DECADE,
                Self::LowPass { start } => corner_hz * start,
            };
            (low_hz, low_hz * OCTAVE_RATIO)
        }
    }

    /// One of the five filters the loudspeaker runs.
    struct Fixture
    {
        name: &'static str,
        corner_hz: f32,
        corner_db: f64,
        slope: Slope,
        sections: [Biquad; MAX_SECTIONS],
        count: usize,
    }

    impl Fixture
    {
        /// Returns the sections the build wrote.
        fn active(&self) -> &[Biquad]
        {
            self.sections.get(..self.count).unwrap_or(&[])
        }
    }

    /// Runs a cascade build into a fresh buffer, failing the test on a refusal.
    fn fixture<F>
    (
        name: &'static str,
        corner_hz: f32,
        corner_db: f64,
        slope: Slope,
        build: F,
    ) -> Fixture
    where
        F: FnOnce(&mut [Biquad]) -> Result<usize, FilterError>,
    {
        let mut sections = [Biquad::SILENT; MAX_SECTIONS];
        match build(&mut sections)
        {
            Ok(count) => Fixture
            {
                name,
                corner_hz,
                corner_db,
                slope,
                sections,
                count,
            },
            Err(error) => panic!("{name} refused: {error:?}"),
        }
    }

    /// Builds the five filters of the loudspeaker.
    fn product_filters() -> [Fixture; 5]
    {
        [
            fixture
            (
                "low high-pass",
                SUBSONIC_HZ,
                BUTTERWORTH_CORNER_DB,
                Slope::HighPass,
                |out| butterworth_high_pass(SUBSONIC_HZ, SUBSONIC_ORDER, SAMPLE_RATE_HZ, out)
            ),
            fixture
            (
                "low low-pass",
                LOW_MID_HZ,
                LINKWITZ_RILEY_CORNER_DB,
                Slope::LowPass { start: 3.0 },
                |out| linkwitz_riley_low_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, out)
            ),
            fixture
            (
                "mid high-pass",
                LOW_MID_HZ,
                LINKWITZ_RILEY_CORNER_DB,
                Slope::HighPass,
                |out| linkwitz_riley_high_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, out)
            ),
            fixture
            (
                "mid low-pass",
                MID_HIGH_HZ,
                LINKWITZ_RILEY_CORNER_DB,
                Slope::LowPass { start: 2.0 },
                |out| linkwitz_riley_low_pass(MID_HIGH_HZ, SAMPLE_RATE_HZ, out)
            ),
            fixture
            (
                "high high-pass",
                MID_HIGH_HZ,
                LINKWITZ_RILEY_CORNER_DB,
                Slope::HighPass,
                |out| linkwitz_riley_high_pass(MID_HIGH_HZ, SAMPLE_RATE_HZ, out)
            ),
        ]
    }

    /// Returns the magnitude of `sections` at `frequency_hz`, in decibels.
    fn magnitude_db(sections: &[Biquad], frequency_hz: f32) -> f64
    {
        match cascade_magnitude(sections, frequency_hz, SAMPLE_RATE_HZ)
        {
            Ok(magnitude) => 20.0 * log10(magnitude),
            Err(error) => panic!("the magnitude was refused: {error:?}"),
        }
    }

    /// Returns the count a build wrote, failing the test on a refusal.
    fn built(result: Result<usize, FilterError>, name: &str) -> usize
    {
        match result
        {
            Ok(count) => count,
            Err(error) => panic!("{name} refused: {error:?}"),
        }
    }

    /// Returns the largest pole radius of `section`.
    fn pole_radius(section: Biquad) -> f64
    {
        let a1 = f64::from(section.a1());
        let a2 = f64::from(section.a2());
        let discriminant = a1 * a1 - 4.0 * a2;
        if discriminant < 0.0
        {
            // A conjugate pair sits at a distance of the square root of a2.
            return sqrt(a2);
        }
        let root = sqrt(discriminant);
        let above = root - a1;
        let below = -root - a1;
        let first = above / 2.0;
        let second = below / 2.0;
        if first.abs() > second.abs()
        {
            first.abs()
        }
        else
        {
            second.abs()
        }
    }

    #[test]
    fn each_way_meets_its_corner_at_its_reference_level()
    {
        for filter in product_filters()
        {
            let measured = magnitude_db(filter.active(), filter.corner_hz);
            assert!
            (
                (measured - filter.corner_db).abs() < CORNER_TOLERANCE_DB,
                "{} read {measured} dB at {} Hz",
                filter.name,
                filter.corner_hz
            );
        }
    }

    #[test]
    fn a_high_pass_stop_band_falls_at_the_fourth_order_asymptote()
    {
        // Measured low end first, so a high-pass has to RISE by the octave
        // figure. A band read backwards changes the sign and fails.
        let mut measured_bands = 0_u32;
        for filter in product_filters()
        {
            let Slope::HighPass = filter.slope
            else
            {
                continue;
            };
            let (low_hz, high_hz) = filter.slope.octave(filter.corner_hz);
            let rise = magnitude_db(filter.active(), high_hz) - magnitude_db(filter.active(), low_hz);
            assert!
            (
                (rise - OCTAVE_DB).abs() < SLOPE_TOLERANCE_DB,
                "{} moved {rise} dB between {low_hz} and {high_hz} Hz",
                filter.name
            );
            measured_bands = measured_bands.saturating_add(1);
        }
        assert_eq!(measured_bands, 3, "a high-pass band went unmeasured");
    }

    #[test]
    fn a_low_pass_falls_at_least_as_fast_as_fourth_order()
    {
        // The double zero the bilinear transform puts at Nyquist only steepens
        // a low-pass, so the asymptote is a floor and the test is one sided.
        // The drop is signed: a band read backwards fails.
        let mut measured_bands = 0_u32;
        for filter in product_filters()
        {
            let Slope::LowPass { .. } = filter.slope
            else
            {
                continue;
            };
            let (low_hz, high_hz) = filter.slope.octave(filter.corner_hz);
            let drop = magnitude_db(filter.active(), low_hz) - magnitude_db(filter.active(), high_hz);
            assert!
            (
                drop >= OCTAVE_DB,
                "{} moved only {drop} dB between {low_hz} and {high_hz} Hz",
                filter.name
            );
            measured_bands = measured_bands.saturating_add(1);
        }
        assert_eq!(measured_bands, 2, "a low-pass band went unmeasured");
    }

    #[test]
    fn a_crossover_pair_sums_flat()
    {
        // The two halves are summed as complex values, which is how the ways
        // meet in the air. A magnitude sum would hide the phase and pass a
        // crossover that cancels.
        for corner_hz in [LOW_MID_HZ, MID_HIGH_HZ]
        {
            let mut low = [Biquad::SILENT; MAX_SECTIONS];
            let mut high = [Biquad::SILENT; MAX_SECTIONS];
            let low_count = built
            (
                linkwitz_riley_low_pass(corner_hz, SAMPLE_RATE_HZ, &mut low),
                "the low half"
            );
            let high_count = built
            (
                linkwitz_riley_high_pass(corner_hz, SAMPLE_RATE_HZ, &mut high),
                "the high half"
            );

            let mut frequency_hz = 5.0_f64;
            while frequency_hz < 22_000.0
            {
                let angular = TAU * frequency_hz / f64::from(SAMPLE_RATE_HZ);
                let (low_real, low_imaginary) = cascade_response(&low[..low_count], angular);
                let (high_real, high_imaginary) = cascade_response(&high[..high_count], angular);
                let real = low_real + high_real;
                let imaginary = low_imaginary + high_imaginary;
                let level_db = 20.0 * log10(sqrt(real * real + imaginary * imaginary));
                assert!
                (
                    level_db.abs() < SUM_TOLERANCE_DB,
                    "the {corner_hz} Hz pair summed to {level_db} dB at {frequency_hz} Hz"
                );
                frequency_hz *= 1.002;
            }
        }
    }

    #[test]
    fn every_pole_stays_inside_the_unit_circle()
    {
        // A pole on or outside the circle turns a section into an oscillator
        // feeding 300 W of amplifier.
        for filter in product_filters()
        {
            for (index, section) in filter.active().iter().enumerate()
            {
                let radius = pole_radius(*section);
                assert!
                (
                    radius < 1.0,
                    "{} section {index} has a pole at radius {radius}",
                    filter.name
                );
            }
        }
    }

    /// Runs `body` on a grid covering the corner frequencies the guards accept.
    ///
    /// The grid walks down from Nyquist by a factor of 1.3 to well under a
    /// hundredth of a hertz, then climbs back by halving the gap to Nyquist, so
    /// it reaches the last hertz below the limit where a narrowed pole pair
    /// splits. The endpoints outside the band come along, and the caller has to
    /// accept a refusal for them.
    fn for_each_corner<F>(mut body: F)
    where
        F: FnMut(f32),
    {
        let mut corner_hz = NYQUIST_HZ;
        for _ in 0..80_u32
        {
            body(corner_hz);
            corner_hz /= 1.3;
        }
        let mut gap_hz = NYQUIST_HZ;
        for _ in 0..25_u32
        {
            gap_hz /= 2.0;
            body(NYQUIST_HZ - gap_hz);
        }
        for corner_hz in [f32::MIN_POSITIVE, 1e-6, 1e-3, NYQUIST_HZ + 1.0]
        {
            body(corner_hz);
        }
    }

    /// Fails unless `built` is a refusal or a section strictly inside the circle.
    fn refused_or_stable(built: Result<Biquad, FilterError>, corner_hz: f32, quality: f32)
    {
        if let Ok(section) = built
        {
            let radius = pole_radius(section);
            assert!
            (
                radius < 1.0,
                "a section at {corner_hz} Hz and Q {quality} was accepted with a pole at {radius}"
            );
        }
    }

    /// Fails unless `built` is a refusal or every section it wrote sits
    /// strictly inside the circle.
    fn cascade_refused_or_stable
    (
        built: Result<usize, FilterError>,
        sections: &[Biquad],
        what: &str,
        corner_hz: f32
    )
    {
        let Ok(count) = built
        else
        {
            return;
        };
        for (index, section) in sections.iter().take(count).enumerate()
        {
            let radius = pole_radius(*section);
            assert!
            (
                radius < 1.0,
                "{what} at {corner_hz} Hz was accepted with section {index} at radius {radius}"
            );
        }
    }

    /// Runs every single section build the module offers at `corner_hz`.
    ///
    /// The quality factors span what a caller can hand over, from the flattest
    /// a crossover uses to the values that drive `alpha` to zero and would put
    /// a pole on the circle.
    fn every_section_refused_or_stable(corner_hz: f32)
    {
        for quality in
        [
            0.1_f32, 0.5, 0.707_106_77, 1.0, 2.0, 10.0, 1_000.0, 1e6, 5e6, 1e30,
            f32::INFINITY
        ]
        {
            refused_or_stable(low_pass(corner_hz, quality, SAMPLE_RATE_HZ), corner_hz, quality);
            refused_or_stable(high_pass(corner_hz, quality, SAMPLE_RATE_HZ), corner_hz, quality);
        }
    }

    /// Runs every cascade build the module offers at `corner_hz`.
    fn every_cascade_refused_or_stable(corner_hz: f32)
    {
        let mut sections = [Biquad::SILENT; MAX_SECTIONS];
        for order in [2_u32, 4, 6, 8]
        {
            let built = butterworth_high_pass(corner_hz, order, SAMPLE_RATE_HZ, &mut sections);
            cascade_refused_or_stable(built, &sections, "a Butterworth high-pass", corner_hz);
        }
        let built = linkwitz_riley_low_pass(corner_hz, SAMPLE_RATE_HZ, &mut sections);
        cascade_refused_or_stable(built, &sections, "a Linkwitz-Riley low-pass", corner_hz);
        let built = linkwitz_riley_high_pass(corner_hz, SAMPLE_RATE_HZ, &mut sections);
        cascade_refused_or_stable(built, &sections, "a Linkwitz-Riley high-pass", corner_hz);
    }

    #[test]
    fn no_accepted_build_leaves_a_pole_on_or_outside_the_unit_circle()
    {
        // Every corner the guards accept, crossed with every quality factor the
        // module can be handed. A point is either refused or lands strictly
        // inside the circle. A pole ON the circle is an undamped oscillator at
        // the corner, in front of 300 W.
        for_each_corner(|corner_hz|
        {
            every_section_refused_or_stable(corner_hz);
            every_cascade_refused_or_stable(corner_hz);
        });
    }

    #[test]
    fn each_way_carries_the_filters_the_specification_lists()
    {
        // The five filters written out here against the assembly, rather than
        // read back out of it.
        let mut expected = [Biquad::SILENT; CROSSOVER_SECTIONS];

        let subsonic = built
        (
            butterworth_high_pass(SUBSONIC_HZ, SUBSONIC_ORDER, SAMPLE_RATE_HZ, &mut expected),
            "the subsonic high-pass"
        );
        let low = subsonic.saturating_add(built
        (
            linkwitz_riley_low_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, &mut expected[subsonic..]),
            "the low low-pass"
        ));
        assert_way(Way::Low, &expected[..low]);

        let mid_high_pass = built
        (
            linkwitz_riley_high_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, &mut expected),
            "the mid high-pass"
        );
        let mid = mid_high_pass.saturating_add(built
        (
            linkwitz_riley_low_pass(MID_HIGH_HZ, SAMPLE_RATE_HZ, &mut expected[mid_high_pass..]),
            "the mid low-pass"
        ));
        assert_way(Way::Mid, &expected[..mid]);

        let high = built
        (
            linkwitz_riley_high_pass(MID_HIGH_HZ, SAMPLE_RATE_HZ, &mut expected),
            "the high high-pass"
        );
        assert_way(Way::High, &expected[..high]);
    }

    /// Compares the cascade `way` builds against the sections it should carry.
    fn assert_way(way: Way, expected: &[Biquad])
    {
        let mut sections = [Biquad::SILENT; CROSSOVER_SECTIONS];
        let count = built(crossover_sections(way, &mut sections), "a way");
        assert_eq!(count, expected.len(), "{way:?} wrote {count} sections");
        for (index, (section, reference)) in sections.iter().zip(expected.iter()).enumerate()
        {
            assert_eq!(section, reference, "{way:?} section {index} is not the one specified");
        }
    }

    #[test]
    fn the_crossover_buffer_is_sized_on_the_most_demanding_way()
    {
        let mut widest = 0_usize;
        for way in [Way::Low, Way::Mid, Way::High]
        {
            let mut sections = [Biquad::SILENT; CROSSOVER_SECTIONS];
            let count = built(crossover_sections(way, &mut sections), "a way");
            widest = widest.max(count);
            assert_eq!
            (
                crossover_sections(way, &mut sections[..count.saturating_sub(1)]),
                Err(FilterError::OutputTooShort),
                "{way:?} wrote into a buffer one slot too short"
            );
        }
        assert_eq!(widest, CROSSOVER_SECTIONS, "the buffer constant misses the widest way");
    }

    #[test]
    fn a_way_refused_for_its_length_leaves_the_buffer_alone()
    {
        // Every buffer length short of a way, seeded with a section that is
        // neither SILENT nor anything the way builds. A caller that seeds with
        // SILENT and drops the Result would otherwise run the low way with its
        // subsonic high-pass and no low-pass, which puts the whole low band on
        // whatever that way drives.
        let sentinel = match high_pass(1_000.0, 0.5, SAMPLE_RATE_HZ)
        {
            Ok(section) => section,
            Err(error) => panic!("the sentinel refused: {error:?}"),
        };
        for way in [Way::Low, Way::Mid, Way::High]
        {
            let mut sections = [Biquad::SILENT; CROSSOVER_SECTIONS];
            let count = built(crossover_sections(way, &mut sections), "a way");
            for short in 0..count
            {
                let mut sections = [sentinel; CROSSOVER_SECTIONS];
                assert_eq!
                (
                    crossover_sections(way, &mut sections[..short]),
                    Err(FilterError::OutputTooShort),
                    "{way:?} accepted a buffer of {short} slots"
                );
                for (index, slot) in sections.iter().enumerate()
                {
                    assert_eq!
                    (
                        *slot, sentinel,
                        "{way:?} wrote slot {index} before refusing a buffer of {short} slots"
                    );
                }
            }
        }
    }

    #[test]
    fn the_coefficients_match_a_double_precision_build()
    {
        // Reference values from an evaluation of the cookbook formulas in
        // double precision outside this crate, narrowed once. The low-pass
        // numerator carries 1 - cos(w0), and at 300 Hz that subtraction leaves
        // five of the seven significant digits single precision holds, so a
        // build that ran in single precision throughout misses these by far
        // more than the tolerance. This is also the only test that reads a
        // numerator sign: a Linkwitz-Riley cascade squares its section, which
        // hides one.
        const SUBSONIC: [[f32; 5]; 2] =
        [
            [0.996_062_1, -1.992_124_2, 0.996_062_1, -1.992_115, 0.992_133_26],
            [0.998_362_4, -1.996_724_8, 0.998_362_4, -1.996_715_7, 0.996_733_96],
        ];
        const LOW_MID_LOW_PASS: [f32; 5] =
            [0.000_443_273_02, 0.000_886_546_04, 0.000_443_273_02, -1.939_570_2, 0.941_343_3];
        const LOW_MID_HIGH_PASS: [f32; 5] =
            [0.970_228_4, -1.940_456_7, 0.970_228_4, -1.939_570_2, 0.941_343_3];
        const MID_HIGH_LOW_PASS: [f32; 5] =
            [0.008_695_435, 0.017_390_87, 0.008_695_435, -1.719_434_3, 0.754_215_96];
        const MID_HIGH_HIGH_PASS: [f32; 5] =
            [0.868_412_55, -1.736_825_1, 0.868_412_55, -1.719_434_3, 0.754_215_96];

        let mut subsonic = [Biquad::SILENT; MAX_SECTIONS];
        let count = built
        (
            butterworth_high_pass(SUBSONIC_HZ, SUBSONIC_ORDER, SAMPLE_RATE_HZ, &mut subsonic),
            "the subsonic high-pass"
        );
        assert_eq!(count, SUBSONIC_SECTIONS);
        for (section, reference) in subsonic.iter().take(count).zip(SUBSONIC.iter())
        {
            assert_section(*section, *reference, "subsonic");
        }

        assert_pair(linkwitz_riley_low_pass, LOW_MID_HZ, LOW_MID_LOW_PASS, "low low-pass");
        assert_pair(linkwitz_riley_high_pass, LOW_MID_HZ, LOW_MID_HIGH_PASS, "mid high-pass");
        assert_pair(linkwitz_riley_low_pass, MID_HIGH_HZ, MID_HIGH_LOW_PASS, "mid low-pass");
        assert_pair(linkwitz_riley_high_pass, MID_HIGH_HZ, MID_HIGH_HIGH_PASS, "high high-pass");
    }

    /// Compares both sections of a Linkwitz-Riley build against `reference`.
    fn assert_pair<F>(build: F, corner_hz: f32, reference: [f32; 5], name: &str)
    where
        F: FnOnce(f32, u32, &mut [Biquad]) -> Result<usize, FilterError>,
    {
        let mut sections = [Biquad::SILENT; MAX_SECTIONS];
        let count = built(build(corner_hz, SAMPLE_RATE_HZ, &mut sections), name);
        assert_eq!(count, LINKWITZ_RILEY_SECTIONS, "{name} wrote {count} sections");
        for section in sections.iter().take(count)
        {
            assert_section(*section, reference, name);
        }
    }

    /// Compares one section against its reference coefficients.
    fn assert_section(section: Biquad, reference: [f32; 5], name: &str)
    {
        let built = [section.b0(), section.b1(), section.b2(), section.a1(), section.a2()];
        for (index, (value, expected)) in built.iter().zip(reference.iter()).enumerate()
        {
            let bound = expected.abs() * COEFFICIENT_TOLERANCE;
            assert!
            (
                (value - expected).abs() <= bound,
                "{name} coefficient {index} built {value} against {expected}"
            );
        }
    }

    #[test]
    fn a_single_section_matches_its_double_precision_reference()
    {
        // The two single section entries take a quality factor away from
        // 1/sqrt(2) on purpose. At the Linkwitz-Riley value a build that
        // dropped its argument would read the same, and the shapes differ, so
        // one that ran the other formula would too.
        const CORNER_HZ: f32 = 1_000.0;
        const QUALITY: f32 = 2.0;
        const LOW: [f32; 5] =
            [0.004_892_584, 0.009_785_168, 0.004_892_584, -1.911_866_4, 0.931_436_7];
        const HIGH: [f32; 5] =
            [0.960_825_8, -1.921_651_6, 0.960_825_8, -1.911_866_4, 0.931_436_7];

        match low_pass(CORNER_HZ, QUALITY, SAMPLE_RATE_HZ)
        {
            Ok(section) => assert_section(section, LOW, "low_pass"),
            Err(error) => panic!("low_pass refused: {error:?}"),
        }
        match high_pass(CORNER_HZ, QUALITY, SAMPLE_RATE_HZ)
        {
            Ok(section) => assert_section(section, HIGH, "high_pass"),
            Err(error) => panic!("high_pass refused: {error:?}"),
        }
    }

    #[test]
    fn a_butterworth_cascade_holds_its_corner_at_every_order()
    {
        // A Butterworth of any order reads -3.0103 dB at its corner, which is
        // what the quality factor table has to produce.
        for (order, expected) in [(2_u32, 1_usize), (4, 2), (6, 3), (8, 4)]
        {
            let mut sections = [Biquad::SILENT; MAX_SECTIONS];
            let count = built
            (
                butterworth_high_pass(LOW_MID_HZ, order, SAMPLE_RATE_HZ, &mut sections),
                "a Butterworth cascade"
            );
            assert_eq!(count, expected, "order {order} wrote {count} sections");
            let measured = magnitude_db(&sections[..count], LOW_MID_HZ);
            assert!
            (
                (measured - BUTTERWORTH_CORNER_DB).abs() < CORNER_TOLERANCE_DB,
                "order {order} read {measured} dB at its corner"
            );
        }
    }

    #[test]
    fn a_longer_buffer_keeps_what_the_build_did_not_write()
    {
        let sentinel = match high_pass(1_000.0, 0.5, SAMPLE_RATE_HZ)
        {
            Ok(section) => section,
            Err(error) => panic!("the sentinel refused: {error:?}"),
        };
        let builds: [(&str, Build); 2] =
        [
            ("the low low-pass", |out| linkwitz_riley_low_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, out)),
            ("the subsonic high-pass", |out|
            {
                butterworth_high_pass(SUBSONIC_HZ, SUBSONIC_ORDER, SAMPLE_RATE_HZ, out)
            }),
        ];

        for (name, build) in builds
        {
            let mut sections = [sentinel; MAX_SECTIONS];
            let count = built(build(&mut sections), name);
            assert!(count < MAX_SECTIONS, "{name} is as long as the buffer");

            for (index, slot) in sections.iter().enumerate()
            {
                if index < count
                {
                    assert_ne!(*slot, sentinel, "{name} left slot {index} unwritten");
                }
                else
                {
                    assert_eq!(*slot, sentinel, "{name} wrote past its count at slot {index}");
                }
            }
        }
    }

    #[test]
    fn every_refusal_names_its_cause()
    {
        let mut out = [Biquad::SILENT; MAX_SECTIONS];

        assert_eq!(low_pass(0.0, 0.7, SAMPLE_RATE_HZ), Err(FilterError::FrequencyNotPositive));
        assert_eq!(low_pass(-30.0, 0.7, SAMPLE_RATE_HZ), Err(FilterError::FrequencyNotPositive));
        assert_eq!
        (
            high_pass(f32::NAN, 0.7, SAMPLE_RATE_HZ),
            Err(FilterError::FrequencyNotPositive)
        );
        assert_eq!
        (
            cascade_magnitude(&out, f32::NAN, SAMPLE_RATE_HZ),
            Err(FilterError::FrequencyNotPositive)
        );

        assert_eq!
        (
            low_pass(NYQUIST_HZ, 0.7, SAMPLE_RATE_HZ),
            Err(FilterError::FrequencyNotBelowNyquist)
        );
        assert_eq!
        (
            high_pass(NYQUIST_HZ + 1.0, 0.7, SAMPLE_RATE_HZ),
            Err(FilterError::FrequencyNotBelowNyquist)
        );
        assert_eq!
        (
            high_pass(f32::INFINITY, 0.7, SAMPLE_RATE_HZ),
            Err(FilterError::FrequencyNotBelowNyquist)
        );
        assert_eq!
        (
            cascade_magnitude(&out, f32::INFINITY, SAMPLE_RATE_HZ),
            Err(FilterError::FrequencyNotBelowNyquist)
        );

        assert_eq!(low_pass(300.0, 0.0, SAMPLE_RATE_HZ), Err(FilterError::QualityNotPositive));
        assert_eq!(high_pass(300.0, -0.7, SAMPLE_RATE_HZ), Err(FilterError::QualityNotPositive));
        assert_eq!(low_pass(300.0, f32::NAN, SAMPLE_RATE_HZ), Err(FilterError::QualityNotPositive));
        assert_eq!
        (
            low_pass(300.0, f32::INFINITY, SAMPLE_RATE_HZ),
            Err(FilterError::QualityNotPositive)
        );
        assert_eq!
        (
            high_pass(300.0, f32::NEG_INFINITY, SAMPLE_RATE_HZ),
            Err(FilterError::QualityNotPositive)
        );

        assert_eq!(low_pass(300.0, 0.7, 0), Err(FilterError::SampleRateZero));
        assert_eq!(high_pass(300.0, 0.7, 0), Err(FilterError::SampleRateZero));
        assert_eq!
        (
            butterworth_high_pass(300.0, 4, 0, &mut out),
            Err(FilterError::SampleRateZero)
        );
        assert_eq!
        (
            linkwitz_riley_high_pass(300.0, 0, &mut out),
            Err(FilterError::SampleRateZero)
        );
        assert_eq!(cascade_magnitude(&out, 300.0, 0), Err(FilterError::SampleRateZero));

        for order in [0_u32, 1, 3, 5, 7, MAX_ORDER.saturating_add(1), MAX_ORDER.saturating_add(2)]
        {
            assert_eq!
            (
                butterworth_high_pass(300.0, order, SAMPLE_RATE_HZ, &mut out),
                Err(FilterError::OrderInvalid),
                "order {order} was accepted"
            );
        }

        assert_eq!
        (
            butterworth_high_pass(300.0, 4, SAMPLE_RATE_HZ, &mut out[..1]),
            Err(FilterError::OutputTooShort)
        );
        assert_eq!
        (
            linkwitz_riley_low_pass(300.0, SAMPLE_RATE_HZ, &mut out[..1]),
            Err(FilterError::OutputTooShort)
        );
        assert_eq!
        (
            linkwitz_riley_high_pass(300.0, SAMPLE_RATE_HZ, &mut []),
            Err(FilterError::OutputTooShort)
        );
        assert_eq!(crossover_sections(Way::Low, &mut []), Err(FilterError::OutputTooShort));
    }

    #[test]
    fn a_narrowing_that_leaves_the_unit_circle_is_refused()
    {
        // A corner one hertz under Nyquist, and one a hair above zero. Both
        // sit inside the open band the frequency guard accepts, and both narrow
        // to a pair the Jury triangle rejects. At 22049 Hz the conjugate pair
        // splits into two real poles, one of them at radius 1.000163, which
        // carries an f32 impulse response past 5e23 in nine seconds of audio.
        // At 1e-5 Hz the section rounds to a1 = -2 and a2 = 1, a double pole
        // at z = 1.
        let mut out = [Biquad::SILENT; MAX_SECTIONS];

        assert_eq!
        (
            low_pass(22_049.0, 0.707, SAMPLE_RATE_HZ),
            Err(FilterError::UnstableDesign)
        );
        assert_eq!
        (
            high_pass(22_049.0, 0.707, SAMPLE_RATE_HZ),
            Err(FilterError::UnstableDesign)
        );
        assert_eq!
        (
            butterworth_high_pass(1e-5, MAX_ORDER, SAMPLE_RATE_HZ, &mut out),
            Err(FilterError::UnstableDesign)
        );
        assert_eq!
        (
            linkwitz_riley_low_pass(1e-5, SAMPLE_RATE_HZ, &mut out),
            Err(FilterError::UnstableDesign)
        );
        assert_eq!
        (
            linkwitz_riley_high_pass(1e-5, SAMPLE_RATE_HZ, &mut out),
            Err(FilterError::UnstableDesign)
        );

        // A corner just inside the same edge still builds, so the guard is not
        // simply refusing the whole neighbourhood.
        assert!(low_pass(22_040.0, 0.707, SAMPLE_RATE_HZ).is_ok());
    }

    #[test]
    fn a_silent_section_passes_nothing()
    {
        // What an unwritten cascade slot holds. It has to stop the signal, not
        // carry it into a way with no filter on it.
        let sections = [Biquad::default(); MAX_SECTIONS];
        assert_eq!(Biquad::default(), Biquad::SILENT);
        for frequency_hz in [10.0_f32, 100.0, 1_000.0, 10_000.0, 20_000.0]
        {
            match cascade_magnitude(&sections, frequency_hz, SAMPLE_RATE_HZ)
            {
                Ok(magnitude) => assert!(magnitude.abs() < f64::EPSILON),
                Err(error) => panic!("the magnitude was refused: {error:?}"),
            }
        }
    }

    #[test]
    fn an_empty_cascade_reads_unity()
    {
        match cascade_magnitude(&[], 1_000.0, SAMPLE_RATE_HZ)
        {
            Ok(magnitude) => assert!((magnitude - 1.0).abs() < f64::EPSILON),
            Err(error) => panic!("the magnitude was refused: {error:?}"),
        }
    }
}
