//! Every fixed number of the machine, defined once.
//!
//! A value that follows from another is written as that derivation, and the
//! relations that must hold are compile time assertions at the bottom of the
//! module.
//!
//! Protection thresholds are absent. They are measured on the real system.

// Every integer converted to a float here is far below 2 to the 24th, so the
// conversion is exact.
#![allow(clippy::cast_precision_loss)]

/// Microseconds in one second.
pub const MICROSECONDS_PER_SECOND: u32 = 1_000_000;

/// Highest core clock the processing part runs at, in hertz.
///
/// STM32H743VI datasheet, voltage scale VOS0. A timing sized for this clock is
/// long enough at every slower one the part can run.
pub const MAX_CORE_CLOCK_HZ: u32 = 480_000_000;

/// Sample rate of the whole chain, in hertz.
///
/// SBC over A2DP delivers 44.1 kHz. Every stage runs at the source rate, so the
/// audio path holds no sample rate conversion.
pub const SAMPLE_RATE_HZ: u32 = 44_100;

/// Sample period, in microseconds.
pub const SAMPLE_PERIOD_US: f32 = MICROSECONDS_PER_SECOND as f32 / SAMPLE_RATE_HZ as f32;

/// Highest frequency the chain can represent, in hertz.
pub const NYQUIST_HZ: f32 = SAMPLE_RATE_HZ as f32 / 2.0;

/// Converter full scale as a normalised sample amplitude.
///
/// The audio path carries single precision floats, and no gain may cross this
/// boundary.
pub const FULL_SCALE: f32 = 1.0;

/// Subsonic high-pass corner protecting the low driver, in hertz.
///
/// Butterworth, fourth order, 24 dB per octave.
pub const SUBSONIC_HZ: f32 = 30.0;

/// Corner between the low and mid ways, in hertz.
///
/// Linkwitz-Riley, fourth order, on both sides.
pub const LOW_MID_HZ: f32 = 300.0;

/// Corner between the mid and high ways, in hertz.
///
/// Linkwitz-Riley, fourth order, on both sides. The spacing between the mid
/// driver and the horn sets the frequency above which the two ways lobe. A
/// lower corner narrows that lobing, a higher one spares the compression
/// driver, and this is the point taken between them.
pub const MID_HIGH_HZ: f32 = 1_400.0;

/// Samples the converter takes to attenuate its data to zero once XSMT falls.
///
/// PCM5102A datasheet 9.3.3, soft mute ramps 1 dB per sample over this many
/// samples.
pub const MUTE_RAMP_SAMPLES: u32 = 104;

/// Sample periods in the converter mute sequence.
///
/// PCM5102A datasheet 11.2, the sequence takes 150 tS plus a fixed term.
pub const MUTE_SEQUENCE_SAMPLES: u32 = 150;

/// Fixed term of the converter mute sequence, in microseconds.
///
/// PCM5102A datasheet 11.2, the 0.2 ms that follows the 150 sample periods.
pub const MUTE_SEQUENCE_FIXED_US: u32 = 200;

/// Length of the converter mute sequence, in microseconds scaled by the rate.
///
/// The microsecond figure and the loop iteration count both divide it by
/// `SAMPLE_RATE_HZ`, so the two cannot drift apart.
const MUTE_SEQUENCE_US_TIMES_RATE: u32 =
    MUTE_SEQUENCE_SAMPLES * MICROSECONDS_PER_SECOND + MUTE_SEQUENCE_FIXED_US * SAMPLE_RATE_HZ;

/// Time from XSMT falling to the hard analog mute, in microseconds.
///
/// The fault path lowers XSMT and leaves the audio clocks running for at least
/// this long. Stopping them sooner strands the converter part way through its
/// ramp, and it pops.
pub const MUTE_SEQUENCE_US: f32 = MUTE_SEQUENCE_US_TIMES_RATE as f32 / SAMPLE_RATE_HZ as f32;

/// Duration of a gain ramp, in milliseconds.
///
/// Every gain change is ramped, because a step pops. The ramp is long enough
/// to bury the step and short enough that a volume change still answers at
/// once, which places it between 10 and 50 ms.
pub const GAIN_RAMP_MS: u32 = 20;

/// Returns the delay loop iterations covering the converter mute sequence.
///
/// `core_clock_hz` is the clock the delay loop runs on. The count rounds up, so
/// the hold never stops inside the sequence.
///
/// The unit is iterations of `cortex_m::asm::delay`, which a Cortex-M7 retires
/// in one cycle at best. A count sized for the highest core clock therefore
/// covers the sequence at every slower one.
#[must_use]
pub const fn mute_hold_iterations(core_clock_hz: u32) -> u32
{
    #[expect
    (
        clippy::cast_possible_truncation,
        reason = "MUTE_HOLD_FITS_EVERY_CLOCK proves the result fits u32"
    )]
    {
        mute_hold_iterations_wide(core_clock_hz) as u32
    }
}

/// Returns the same count as `mute_hold_iterations`, before narrowing.
const fn mute_hold_iterations_wide(core_clock_hz: u32) -> u64
{
    let numerator = (MUTE_SEQUENCE_US_TIMES_RATE as u64).saturating_mul(core_clock_hz as u64);
    let denominator = (SAMPLE_RATE_HZ as u64).saturating_mul(MICROSECONDS_PER_SECOND as u64);
    numerator.div_ceil(denominator)
}

const _: () = assert!
(
    SAMPLE_RATE_HZ > 0,
    "the sample rate divides into every derived timing"
);

const _: () = assert!
(
    SUBSONIC_HZ > 0.0 && SUBSONIC_HZ < LOW_MID_HZ,
    "the subsonic corner sits inside the low band it protects"
);

const _: () = assert!
(
    LOW_MID_HZ < MID_HIGH_HZ,
    "the crossover corners follow the order of the ways"
);

const _: () = assert!
(
    MID_HIGH_HZ < NYQUIST_HZ,
    "a corner at or above Nyquist has no filter to build from it"
);

const _: () = assert!
(
    FULL_SCALE > 0.0 && FULL_SCALE <= 1.0,
    "full scale is a normalised amplitude, so it sits in zero exclusive to one"
);

const _: () = assert!
(
    GAIN_RAMP_MS > 0,
    "a ramp of zero length is a step, and a step pops"
);

/// Whether the count fits `u32` at every core clock.
///
/// It rises with the clock, so the largest clock decides.
/// `mute_hold_iterations` narrows on the strength of this.
const MUTE_HOLD_FITS_EVERY_CLOCK: bool = mute_hold_iterations_wide(u32::MAX) <= u32::MAX as u64;

const _: () = assert!
(
    MUTE_HOLD_FITS_EVERY_CLOCK,
    "the mute hold count fits its type at every core clock"
);

const _: () = assert!
(
    MUTE_SEQUENCE_SAMPLES > MUTE_RAMP_SAMPLES,
    "the full sequence outlasts the soft attenuation ramp inside it"
);

#[cfg(test)]
mod tests
{
    use super::*;

    /// Tolerance covering single precision rounding on a derived duration.
    const EPSILON_US: f32 = 0.01;

    /// Tolerance covering single precision rounding on a derived frequency.
    const EPSILON_HZ: f32 = 0.01;

    #[test]
    fn sample_period_matches_the_rate()
    {
        assert!((SAMPLE_PERIOD_US - 22.675_737).abs() < EPSILON_US);
    }

    #[test]
    fn mute_sequence_matches_the_datasheet_figure()
    {
        // 150 sample periods at 44.1 kHz plus 0.2 ms, so 3.6 ms.
        assert!((MUTE_SEQUENCE_US - 3_601.36).abs() < 1.0);
        assert!(MUTE_SEQUENCE_US > MUTE_RAMP_SAMPLES as f32 * SAMPLE_PERIOD_US);
    }

    #[test]
    fn nyquist_follows_the_sample_rate()
    {
        assert!((NYQUIST_HZ - 22_050.0).abs() < EPSILON_HZ);
    }

    #[test]
    fn the_mute_hold_covers_the_sequence_at_every_clock()
    {
        for clock_hz in [64_000_000_u32, 200_000_000, 400_000_000, MAX_CORE_CLOCK_HZ]
        {
            let iterations = f64::from(mute_hold_iterations(clock_hz));
            let held_us = iterations / f64::from(clock_hz) * 1e6;
            assert!(held_us >= f64::from(MUTE_SEQUENCE_US));
        }
    }

    #[test]
    fn the_mute_hold_rounds_up_rather_than_down()
    {
        // The exact figure at 480 MHz is 1 728 653.06, so truncation would stop
        // the hold inside the sequence.
        assert_eq!(mute_hold_iterations(MAX_CORE_CLOCK_HZ), 1_728_654);
    }

    #[test]
    fn the_mute_hold_rises_with_the_clock()
    {
        let slow = mute_hold_iterations(64_000_000);
        let fast = mute_hold_iterations(MAX_CORE_CLOCK_HZ);
        assert!(slow < fast);
        assert!(mute_hold_iterations(u32::MAX) > fast);
    }
}
