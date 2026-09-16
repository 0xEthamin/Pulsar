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

/// Core clock the processing part runs on out of reset, in hertz.
///
/// RM0433 section 8.7.2 divides the internal oscillator by one after a reset
/// and selects it as the system clock, which puts the core at 64 MHz. A timing
/// sized for this clock is only long enough while nothing has raised it.
pub const BOOT_CORE_CLOCK_HZ: u32 = 64_000_000;

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

/// Samples the converter takes to raise its data to full scale once XSMT rises.
///
/// PCM5102A datasheet 9.3.3: a low to high transition on XSMT starts a soft
/// digital un-mute that applies 1 dB gain steps every sample time from minus
/// infinity to 0 dBFS, and takes this many samples.
///
/// This is the ramp UPWARD, and it is a different sentence of the datasheet
/// from the one `MUTE_RAMP_SAMPLES` comes from. Section 11.2 gives a whole
/// power down sequence and no section faces it with a power up one, so the
/// figure here is the entire specification of the rise. The two counts
/// coincide at 104 and describe opposite directions, so neither stands in for
/// the other.
const UNMUTE_RAMP_SAMPLES: u32 = 104;

/// Length of the converter unmute ramp, in microseconds scaled by the rate.
///
/// It is here for the assertion below, which is what proves the hold a release
/// runs after raising XSMT outlasts the ramp that raise starts.
const UNMUTE_RAMP_US_TIMES_RATE: u32 = UNMUTE_RAMP_SAMPLES * MICROSECONDS_PER_SECOND;

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
///
/// A release reuses it as the window it holds zeros over after raising XSMT.
/// That window has to cover the unmute ramp, which is a shorter and separate
/// figure, and the assertion below is what proves it does rather than assuming
/// the two coincide.
pub const MUTE_SEQUENCE_US: f32 = MUTE_SEQUENCE_US_TIMES_RATE as f32 / SAMPLE_RATE_HZ as f32;

/// Frame clock periods of continuous zero data the converter analog mutes on.
///
/// PCM5102A datasheet 9.3.2.3: the part detects continuous zero data and
/// enters a full analog mute, counting the zeros over 1024 frame clock periods
/// before it does. In hardware mode both channels have to carry zero for the
/// count to run, and both slots of this frame carry the same sample, so the
/// condition is met whenever the buffer holds silence.
///
/// At this sample rate the count is 23.2 ms, six times the mute sequence, and
/// the assertion below is what holds those two figures against each other.
const ZERO_DATA_MUTE_FRAMES: u32 = 1_024;

/// Shortest gain ramp, in milliseconds.
///
/// Every gain change ramps for at least this long, because a step pops.
pub const GAIN_RAMP_MS: u32 = 20;

/// Longest ramp one step of either volume control takes, in milliseconds.
///
/// A ramp is long enough to bury the step and short enough that a volume change
/// still answers at once, which places it between 10 and 50 ms.
pub const GAIN_STEP_RAMP_MAX_MS: u32 = 50;

/// Time the gain takes to cross from 0 to 1 at its fastest, in milliseconds.
///
/// This bounds the speed of every gain change. The widest single step of the
/// volume law, 3 dB at the top of the coarse control, spans `1 - 10^(-3/20)`,
/// 0.29205 of full scale. Crossing it in `GAIN_STEP_RAMP_MAX_MS` puts a full
/// swing at 171.20 ms, and the count rounds down so that step lands inside the
/// bound, at 49.94 ms.
pub const GAIN_FULL_SWING_MS: u32 = 171;

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
    BOOT_CORE_CLOCK_HZ > 0 && BOOT_CORE_CLOCK_HZ <= MAX_CORE_CLOCK_HZ,
    "the boot clock is one the part runs at"
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

const _: () = assert!
(
    GAIN_RAMP_MS <= GAIN_STEP_RAMP_MAX_MS && GAIN_STEP_RAMP_MAX_MS <= GAIN_FULL_SWING_MS,
    "the shortest ramp fits inside the longest step, which fits inside a full swing"
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

const _: () = assert!
(
    MUTE_SEQUENCE_US_TIMES_RATE > UNMUTE_RAMP_US_TIMES_RATE,
    "the mute sequence a release holds zeros for outlasts the unmute ramp it \
     is held over, so the converter walks its gain steps over silence"
);

const _: () = assert!
(
    ZERO_DATA_MUTE_FRAMES as u64 * MICROSECONDS_PER_SECOND as u64
        > MUTE_SEQUENCE_US_TIMES_RATE as u64,
    "the converter counts zeros for longer than its own mute sequence lasts, \
     so what arms its analog mute is silence sent before the sequence and not \
     the sequence"
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
    fn the_unmute_ramp_is_its_own_figure_and_the_hold_covers_it()
    {
        // 104 sample periods at 44.1 kHz, so 2.4 ms, against the 3.6 ms of the
        // sequence the release hold is sized on. The two counts coincide and
        // the two durations do not, which is what stops one standing in for
        // the other.
        let ramp = UNMUTE_RAMP_SAMPLES as f32 * SAMPLE_PERIOD_US;

        assert_eq!(UNMUTE_RAMP_SAMPLES, 104);
        assert!((ramp - 2_358.28).abs() < 1.0);
        assert!((MUTE_SEQUENCE_US - ramp - 1_243.08).abs() < 1.0);
    }

    #[test]
    fn the_zero_data_mute_outlasts_the_mute_sequence()
    {
        // 1024 frame clock periods at 44.1 kHz, so 23.2 ms against the 3.6 ms
        // of the sequence, which is the ordering the compile time assertion
        // beside the constant carries. Nothing here says whether a release
        // reaches the count: what arms it is silence sent before the sequence
        // rather than the sequence, and that belongs to the gate.
        let zero_data = ZERO_DATA_MUTE_FRAMES as f32 * MICROSECONDS_PER_SECOND as f32
            / SAMPLE_RATE_HZ as f32;

        assert_eq!(ZERO_DATA_MUTE_FRAMES, 1_024);
        assert!((zero_data - 23_219.95).abs() < 1.0);
        assert!((zero_data - MUTE_SEQUENCE_US - 19_618.59).abs() < 1.0);
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
