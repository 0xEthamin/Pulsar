//! Output stage of the processing board, between the crossover and a buffer.
//!
//! The sample a way answers crosses the gain of that way, the product of its
//! sensitivity trim and the peak reserve formed once at compile time, then the
//! ceiling, which stands on the high way alone, then the conversion to the word
//! a buffer carries. The low and mid ways cross the gain and the conversion, the
//! high way crosses the wall between the two, and the order is most of the
//! design.
//!
//! `WayWords::of` is the whole of it and it is the only way to build the three
//! words. It takes the three samples the cascades answered and nothing else, so
//! a word that skipped the trim, the reserve or the ceiling is not a value this
//! module can produce.
//!
//! # The trim, which is where the design aims
//!
//! The three drivers do not turn one volt into one sound pressure alike, and
//! nothing in front of them corrects it: the two gain potentiometers of each
//! amplifier board sit at their minimum and the whole alignment is here. The
//! trim is the level setting that balances them, it is the normal operating
//! point, and it acts on every sample.
//!
//! The alignment is taken against the low way, so the low way carries unity and
//! the other two come down to it. The three figures are read off the datasheet
//! sensitivities of the three drivers, which makes them a starting point rather
//! than a calibration: half a decibel between two datasheets means nothing
//! until a measurement microphone has read the three in the cabinet.
//!
//! # The reserve, which is room for the peak between two samples
//!
//! A converter rebuilds a continuous wave from the samples, and its
//! reconstruction filter can peak between two samples above both of them. The
//! reserve takes two decibels off every way on top of its trim, so a way whose
//! trimmed samples stand inside the format reaches the converter with room for
//! a peak up to two decibels over its largest word. It is a constant of this
//! stage and of no level setting, so no edit of a setting takes it away.
//!
//! It does not hold the overshoot of the cascades themselves, which is larger
//! than the reserve on two ways. The loudest peak a source the format carries
//! puts on a way is the L1 norm of its impulse response, and after the trim and
//! the reserve that peak stands 5.42 dB over the format on the low way and
//! 1.73 dB over it on the mid way. On those two ways the conversion saturates
//! every sample a cascade takes past the format, on its own side of zero, and
//! that saturation distorts those samples. The reserve leaves them no room
//! between two samples. On the high way the same peak stands under the wall,
//! which the section below reads.
//!
//! It stands on the three ways alike, since taken off two ways out of three it
//! would move the alignment the trims hold by its own size.
//!
//! # The ceiling, which is the wall behind the target
//!
//! The high way drives a compression driver whose diaphragm is not replaceable,
//! and its crossover corner stands below the band the driver is rated over, so
//! the thermal rating of that driver does not cover the bottom of its band and
//! no figure quantifies what does. The ceiling is a wall on the INSTANTANEOUS
//! sample, which is what makes it hold for a waveform of any shape rather than
//! for a sine.
//!
//! With the trim and the reserve in place a full scale SAMPLE reaches 10.38 dB
//! under the wall, and the wall does nothing while the machine works. What it
//! is for is the day the gain is not what it should be: a sign error, a
//! coefficient that lands at unity, a preset that adds gain behind it, or
//! someone rebalancing the loudspeaker by removing the trim. In each of those
//! the driver sees past the power it is rated for, and the wall holds it at the
//! figure below. The wall binds on what the trim and the reserve leave, so its
//! level does not move with either of them.
//!
//! That 10.38 dB is the margin on one SAMPLE and not the margin on a peak, and
//! the two are far apart. A cascade answers a weighted sum of the samples
//! behind it, so the loudest peak any source the format carries can put on a
//! way is the sum of the magnitudes of its impulse response, the L1 norm of
//! that response, and the input that attains it carries the sign pattern of the
//! response read backwards. For the high way that norm is 2.47295, MEASURED off
//! the coefficients the cascade holds, and driving the cascade with the input
//! that attains it leaves 1000334656 words after the trim and the reserve
//! against the 1336379648 of the wall. The margin on a peak is therefore
//! 2.52 dB, and a stage placed in front of the wall that adds gain has two and
//! a half decibels to spend rather than ten.
//!
//! The wall stands at 3.3037 times what a full scale source leaves after the
//! trim and the reserve, and the norm is under that, so NO source the format
//! carries reaches the wall. That holds over every input rather than over one
//! waveform, which is what separates it from a reading taken on a square.
//!
//! It is the same shape as the pull-down on the converter mute line. It does
//! nothing until the rest has failed.
//!
//! # Where the numbers come from
//!
//! MEASURED on the built machine: the amplifier turns one volt into 15.3, and a
//! converter at full scale leaves 2.1 Vrms, so full scale at the loudspeaker
//! terminals is 32.13 Vrms. The compression driver takes 50 W and stands on 8
//! ohms, which is 20 Vrms, so it sits at 0.62247 of full scale and 4.118 dB
//! under it. The wall stands at 4.120 dB under full scale, a rounding on the
//! quiet side of that, which is 19.9945 Vrms and 49.97 W.
//!
//! Once the trim and the reserve have acted a full scale SAMPLE leaves the same
//! way at 0.18836 of full scale, which is 6.05 Vrms and 4.58 W into the same 8
//! ohms. That is the level of one sample and not the level of a peak: the
//! loudest peak a source the format carries puts on this way stands at 0.46582
//! of full scale, which the same conversion reads as 15.0 Vrms and 28.0 W, with
//! the wall above it at 0.62230 and 49.97 W.
//!
//! # What is NOT here
//!
//! The thermal limiter of the high way, which is a separate mechanism living
//! UNDER this wall and acting on the average of the square of the signal over
//! 0.3 seconds. A wall on one sample cannot tell a transient from a tone held
//! for ten seconds, and those two burn a voice coil differently.

/// Sample amplitude one converter full scale stands at.
///
/// A buffer word crosses into a sample as a signed integer, so the format
/// reaches two to the thirty first below zero and one word less above it. Every
/// figure below is a fraction of this one, and it is exact in single precision.
const FULL_SCALE_WORDS: f32 = 2_147_483_648.0;

/// Gain the low way leaves with.
///
/// The alignment of the three ways is taken against this one, so it carries
/// unity. It stands in the gain of the low way rather than being left out, so
/// the three gains share one shape.
const LOW_TRIM: f32 = 1.0;

/// Gain the mid way leaves with, half a decibel down.
const MID_TRIM: f32 = 0.944_060_86;

/// Gain the high way leaves with, twelve and a half decibels down.
const HIGH_TRIM: f32 = 0.237_137_38;

/// Gain every way leaves with on top of its trim, two decibels down.
///
/// The module documentation carries what it is room for. On the high way it
/// acts ahead of the wall, so the wall stands at the same level whatever the
/// reserve is.
const PEAK_RESERVE: f32 = 0.794_328_2;

/// Gain the stage applies to the low way, its trim and the peak reserve.
const LOW_GAIN: f32 = LOW_TRIM * PEAK_RESERVE;

/// Gain the stage applies to the mid way, its trim and the peak reserve.
const MID_GAIN: f32 = MID_TRIM * PEAK_RESERVE;

/// Gain the stage applies to the high way, its trim and the peak reserve.
const HIGH_GAIN: f32 = HIGH_TRIM * PEAK_RESERVE;

/// Fraction of full scale the ceiling of the high way stands at.
///
/// 4.12 dB down. The module documentation carries the two voltages and the
/// power it comes from.
const HIGH_CEILING_RATIO: f32 = 0.622_300_27;

/// Sample amplitude the ceiling of the high way stands at.
///
/// The product is exact: a single precision float of this magnitude steps in
/// units of 128 words and the ratio lands on one of those steps, so the wall is
/// a whole number of words and it is the same distance either side of zero.
const HIGH_CEILING_WORDS: f32 = FULL_SCALE_WORDS * HIGH_CEILING_RATIO;

/// The sample each way answered, before the stage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct WaySamples
{
    /// What the low way answered.
    pub(crate) low: f32,
    /// What the mid way answered.
    pub(crate) mid: f32,
    /// What the high way answered.
    pub(crate) high: f32,
}

/// The word each way leaves in a buffer.
///
/// The fields are read through the three methods and written by `of` alone, so
/// every value of this type crossed the gain of its way, the product of its trim
/// and the peak reserve, then the ceiling on the high way, then the conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct WayWords
{
    /// The word the low way leaves.
    low: u32,
    /// The word the mid way leaves.
    mid: u32,
    /// The word the high way leaves.
    high: u32,
}

impl WayWords
{
    /// Returns the words `samples` leave the machine with.
    ///
    /// It takes the samples the cascades answered and nothing else, so a word
    /// that skipped the trim or the reserve, or a high word that skipped the
    /// wall, is not a value this function can be made to return.
    pub(crate) fn of(samples: WaySamples) -> Self
    {
        Self
        {
            low: to_word(samples.low * LOW_GAIN),
            mid: to_word(samples.mid * MID_GAIN),
            high: to_word(under_ceiling(samples.high * HIGH_GAIN)),
        }
    }

    /// Returns the word the low way leaves.
    pub(crate) const fn low(self) -> u32
    {
        self.low
    }

    /// Returns the word the mid way leaves.
    pub(crate) const fn mid(self) -> u32
    {
        self.mid
    }

    /// Returns the word the high way leaves.
    pub(crate) const fn high(self) -> u32
    {
        self.high
    }
}

/// Returns the gain the stage applies to `way`, its trim and the peak reserve.
///
/// It is here for the tests of the loop under this stage, which read a response
/// in words and need the figure the stage applies rather than a second copy of
/// it beside it.
#[cfg(test)]
pub(crate) const fn gain_for_test(way: crate::filter::Way) -> f32
{
    match way
    {
        crate::filter::Way::Low => LOW_GAIN,
        crate::filter::Way::Mid => MID_GAIN,
        crate::filter::Way::High => HIGH_GAIN,
    }
}

/// Returns `value` held to the ceiling of the high way.
///
/// The three answers of a comparison are covered rather than two. A value that
/// is not a number compares false against both bounds, so it would cross here
/// untouched and reach a conversion that already answers zero for it. Naming it
/// first leaves the two stages agreeing rather than one of them deciding, and
/// it is what holds the day the conversion stops being the one that saturates.
/// An infinity carries its sign into a comparison, so it comes back as the
/// bound on its own side.
#[expect
(
    clippy::manual_clamp,
    reason = "the answer for a value that is not a number is silence and not a \
              bound, which is a fourth case a clamp does not carry"
)]
fn under_ceiling(value: f32) -> f32
{
    if value.is_nan()
    {
        0.0
    }
    else if value > HIGH_CEILING_WORDS
    {
        HIGH_CEILING_WORDS
    }
    else if value < -HIGH_CEILING_WORDS
    {
        -HIGH_CEILING_WORDS
    }
    else
    {
        value
    }
}

/// Returns the buffer word `value` makes, bounded to the sample format.
///
/// The bound is the conversion itself. A float to integer cast in this language
/// saturates: a value above the largest `i32` gives that largest one, a value
/// below the smallest gives that smallest one, and a value that is not a number
/// gives zero, which is silence. So the sample cannot come back round to the
/// opposite sign, and the one place that could break it is a change of this
/// line to a conversion that wraps.
#[expect
(
    clippy::cast_possible_truncation,
    reason = "the cast is the saturation, and inside the format it drops the \
              fraction of the sample and nothing else"
)]
fn to_word(value: f32) -> u32
{
    (value as i32).cast_unsigned()
}

const _: () = assert!
(
    LOW_TRIM > 0.0
        && LOW_TRIM <= 1.0
        && MID_TRIM > 0.0
        && MID_TRIM <= 1.0
        && HIGH_TRIM > 0.0
        && HIGH_TRIM <= 1.0,
    "a trim that adds gain would put a way past the format on a source that \
     fits it, and a trim at or below zero is a mute or a sign error"
);

const _: () = assert!
(
    PEAK_RESERVE > 0.0 && PEAK_RESERVE < 1.0,
    "a reserve at or above unity leaves no room over the largest sample, and a \
     reserve at or below zero is a mute or a sign error"
);

const _: () = assert!
(
    HIGH_CEILING_WORDS > 0.0 && HIGH_CEILING_WORDS < FULL_SCALE_WORDS,
    "the ceiling of the high way stands inside the format, so the wall binds \
     before the conversion does"
);

const _: () = assert!
(
    HIGH_GAIN * FULL_SCALE_WORDS < HIGH_CEILING_WORDS,
    "the trim and the reserve of the high way no longer hold a full scale \
     sample under the ceiling, so the wall has become the operating point \
     instead of the wall behind it"
);

#[cfg(test)]
mod tests
{
    // Every count below is a number of bit patterns of a single precision
    // float, and they are added in a type twice as wide as the widest of them.
    //
    // Every float compared for equality below is a constant of the stage or the
    // conversion of one, and both are exact, so a margin there would admit the
    // rounding this is written to refuse.
    #![allow
    (
        clippy::arithmetic_side_effects,
        clippy::cast_possible_truncation,
        clippy::float_cmp,
        clippy::panic
    )]

    use super::*;
    use libm::{log10, log10f, powf, sqrtf};

    /// Decibels a trim may stand from the figure it is written for.
    ///
    /// A single precision literal of this magnitude carries about seven digits,
    /// so a correct figure lands orders inside this and a figure off by a digit
    /// does not.
    const TRIM_EPSILON_DB: f32 = 1.0e-4;

    /// Bit patterns a single precision float holds.
    const SWEEP_PATTERNS: u64 = 1_u64 << 32;

    /// Stretches the exhaustive sweep is cut into.
    ///
    /// Each stretch opens where the one before it stopped and the sweep reads
    /// those two positions off the walk, so the stretches tile the patterns
    /// whatever this is and it decides how long the walk takes and never which
    /// values it reaches. MEASURED: sixteen of them take 13.6 s against 124.5 s
    /// for one.
    #[cfg(not(coverage))]
    const SWEEP_STRETCHES: usize = 16;

    /// The same cut under a coverage build, which takes one.
    ///
    /// The counters an instrumented build increments are shared between threads
    /// and are not atomic, so stretches running at once spend the walk bouncing
    /// one cache line between cores. MEASURED over the whole coverage stage, the
    /// instrumented build included: sixteen stretches take 17 min 41 s against
    /// 5 min 42 s for one, and 15765 seconds of processor against 353, for the
    /// same four billion patterns.
    #[cfg(coverage)]
    const SWEEP_STRETCHES: usize = 1;

    /// What a stretch that never ran brings back.
    ///
    /// It opens and stops at the same place, so it breaks the chain of the
    /// stretches rather than shortening it silently.
    const EMPTY_STRETCH: Swept = Swept
    {
        opens_at: 0,
        stops_at: 0,
        patterns: 0,
        low: 0,
        mid: 0,
        high: 0,
        low_flipped: 0,
        mid_flipped: 0,
        high_flipped: 0,
    };

    /// What the words of one way reached over a stretch of the sweep.
    #[derive(Clone, Copy)]
    struct Swept
    {
        /// Pattern the stretch was asked to open at.
        opens_at: u64,
        /// Pattern past the last one the stretch actually crossed.
        stops_at: u64,
        /// Patterns the stretch crossed.
        patterns: u64,
        /// Largest magnitude the low channel answered.
        low: u32,
        /// Largest magnitude the mid channel answered.
        mid: u32,
        /// Largest magnitude the high channel answered.
        high: u32,
        /// Patterns the low channel answered with a word of the opposite sign.
        low_flipped: u64,
        /// Patterns the mid channel answered with a word of the opposite sign.
        mid_flipped: u64,
        /// Patterns the high channel answered with a word of the opposite sign.
        high_flipped: u64,
    }

    /// Returns the decibels `gain` stands at.
    fn decibels(gain: f32) -> f32
    {
        20.0 * log10f(gain)
    }

    /// Returns the magnitude of the sample `word` carries.
    fn magnitude(word: u32) -> u32
    {
        word.cast_signed().unsigned_abs()
    }

    /// Returns the words one value answers on the three channels.
    fn words_of(value: f32) -> WayWords
    {
        WayWords::of(WaySamples { low: value, mid: value, high: value })
    }

    /// Returns one if `word` stands on the other side of zero from `value`, and
    /// zero otherwise.
    ///
    /// The gain of every way is positive, so a word on the other side of zero
    /// from the value it came from is a sample that came back round. The rule
    /// reads the sign of the value and the sign of the word and nothing of the
    /// stage between them. A value that is not a number has no sign and a word
    /// of zero has none either, so neither reads as flipped.
    fn flipped(value: f32, word: u32) -> u64
    {
        let word = word.cast_signed();

        u64::from((value > 0.0 && word < 0) || (value < 0.0 && word > 0))
    }

    /// Runs the stage over every bit pattern from `from` up to `upto`.
    ///
    /// `stops_at` is written from inside the walk and not from `upto`, so it
    /// reports the pattern the stretch actually finished on. That is what makes
    /// a stretch answer for the ground it covered rather than for the range it
    /// was handed.
    ///
    /// The join of the stretches and the count show that the walk crosses every
    /// pattern. They do not show that the stage runs on the pattern the walk
    /// stands on. A body that hands the stage, and the reading of the sign, a
    /// pattern derived from the current one, or a constant, keeps both and
    /// passes every reading of the test. A body that runs the stage on no
    /// pattern fails it, because the high channel never reaches the ceiling.
    /// What ties one crossing of the stage to each pattern is the shape of the
    /// loop body below, which a reader checks and no assertion does.
    fn sweep(from: u64, upto: u64) -> Swept
    {
        let mut swept = Swept { opens_at: from, stops_at: from, ..EMPTY_STRETCH };

        for pattern in from..upto
        {
            let value = f32::from_bits(pattern as u32);
            let words = words_of(value);

            swept.patterns += 1;
            swept.stops_at = pattern + 1;
            swept.low = swept.low.max(magnitude(words.low()));
            swept.mid = swept.mid.max(magnitude(words.mid()));
            swept.high = swept.high.max(magnitude(words.high()));
            swept.low_flipped += flipped(value, words.low());
            swept.mid_flipped += flipped(value, words.mid());
            swept.high_flipped += flipped(value, words.high());
        }

        swept
    }

    #[test]
    fn the_low_way_carries_the_alignment_the_other_two_come_down_to()
    {
        assert!(decibels(LOW_TRIM).abs() < TRIM_EPSILON_DB);
        assert!((decibels(MID_TRIM) + 0.5).abs() < TRIM_EPSILON_DB);
        assert!((decibels(HIGH_TRIM) + 12.5).abs() < TRIM_EPSILON_DB);
    }

    #[test]
    fn every_trim_is_written_as_the_nearest_float_to_its_own_decibels()
    {
        // The reading above is in decibels, so a literal off by one step of the
        // format passes it. This pins each literal to the conversion of its own
        // figure instead, which leaves no room at all.
        assert_eq!(LOW_TRIM.to_bits(), powf(10.0, 0.0).to_bits());
        assert_eq!(MID_TRIM.to_bits(), powf(10.0, -0.5 / 20.0).to_bits());
        assert_eq!(HIGH_TRIM.to_bits(), powf(10.0, -12.5 / 20.0).to_bits());
    }

    #[test]
    fn the_peak_reserve_is_written_as_the_nearest_float_to_two_decibels()
    {
        assert_eq!(PEAK_RESERVE.to_bits(), powf(10.0, -2.0 / 20.0).to_bits());
    }

    #[test]
    fn every_way_leaves_the_stage_two_decibels_under_its_own_trim()
    {
        // Read off the words the stage answers rather than off its gains, and
        // against the trim of each way, so what is read is the distance the
        // reserve alone puts between the two. A gain that lost the reserve on
        // one way reads zero on that way, and a recalibrated trim moves both
        // sides of the reading at once. The drive stands under the wall of the
        // high way, so every way answers its gain and nothing else.
        let drive = 1.0e9_f32;
        let words = words_of(drive);
        let reserve = |word: u32, trim: f32|
        {
            20.0 * log10(f64::from(word.cast_signed()) / f64::from(drive))
                - 20.0 * log10(f64::from(trim))
        };
        let epsilon = f64::from(TRIM_EPSILON_DB);

        for (name, word, trim) in
        [
            ("low", words.low(), LOW_TRIM),
            ("mid", words.mid(), MID_TRIM),
            ("high", words.high(), HIGH_TRIM),
        ]
        {
            let read = reserve(word, trim);

            assert!
            (
                (read + 2.0).abs() < epsilon,
                "the {name} way leaves the stage {read} dB from its own trim"
            );
        }
    }

    #[test]
    fn the_ceiling_is_the_level_the_compression_driver_takes_fifty_watts_at()
    {
        // Full scale is 2.1 Vrms out of the converter times the 15.3 the
        // amplifier gives, and 50 W into 8 ohms is 20 Vrms. The ceiling is the
        // ratio of the two, rounded to the quiet side, so the reading here is
        // that it stands a shade UNDER the level that figure names and not on
        // the wrong side of it.
        let full_scale_volts = 2.1_f32 * 15.3;
        let rated_volts = sqrtf(50.0 * 8.0);
        let rated_ratio = rated_volts / full_scale_volts;

        assert!((full_scale_volts - 32.13).abs() < 1.0e-4);
        assert!((rated_volts - 20.0).abs() < 1.0e-4);

        // MEASURED: the wall stands 0.0024 dB under the level the rating names,
        // which is the rounding of 4.118 to 4.12 and the whole of the distance
        // between the two. Reading it in decibels rather than in ratio is what
        // makes the figure the same one the rest of this module is written in.
        let rounding_db = decibels(rated_ratio / HIGH_CEILING_RATIO);

        assert!(HIGH_CEILING_RATIO < rated_ratio);
        assert!((rounding_db - 0.0024).abs() < 0.0005, "the rounding moved to {rounding_db} dB");

        let ceiling_volts = full_scale_volts * HIGH_CEILING_RATIO;
        let ceiling_watts = ceiling_volts * ceiling_volts / 8.0;

        assert!((ceiling_watts - 49.97).abs() < 0.01);
        assert!(ceiling_watts < 50.0);
    }

    #[test]
    fn the_ceiling_lands_on_a_whole_word_the_same_distance_either_side_of_zero()
    {
        // What makes the bound exact rather than a rounding: the wall is a
        // value the format holds with no fraction, so the conversion of the
        // wall is the wall and the two sides are one number apart from zero.
        assert_eq!(HIGH_CEILING_WORDS, 1_336_379_648.0);
        assert_eq!(to_word(HIGH_CEILING_WORDS).cast_signed(), 1_336_379_648);
        assert_eq!(to_word(-HIGH_CEILING_WORDS).cast_signed(), -1_336_379_648);
        assert!((decibels(HIGH_CEILING_RATIO) + 4.12).abs() < 1.0e-4);
    }

    #[test]
    fn the_trim_and_the_reserve_leave_the_high_way_ten_decibels_under_its_own_ceiling()
    {
        // A full scale sample reaches 4.58 W where the wall stands at 49.97 W.
        // That is the distance from one SAMPLE. The loudest peak the chain can
        // answer on this way reaches 28.0 W, 2.52 dB under the wall, and
        // `no_source_the_format_carries_reaches_the_wall_of_the_high_way` reads
        // it.
        let top = HIGH_GAIN * FULL_SCALE_WORDS;
        let headroom = decibels(HIGH_CEILING_WORDS / top);
        let volts = 2.1 * 15.3 * HIGH_GAIN;

        assert!((headroom - 10.38).abs() < 0.01, "the headroom reads {headroom} dB");
        assert!((volts * volts / 8.0 - 4.58).abs() < 0.01);
    }

    #[test]
    fn a_sample_inside_the_ceiling_crosses_the_high_way_untouched()
    {
        // The wall is a wall and not a gain: below it nothing is scaled, so the
        // trim and the reserve alone decide what the way carries.
        for sample in [0.0_f32, 1.0, -1.0, 1.0e6, -1.0e6, HIGH_CEILING_WORDS / HIGH_GAIN]
        {
            let want = to_word(sample * HIGH_GAIN).cast_signed();
            let got = words_of(sample).high().cast_signed();

            assert_eq!(got, want, "a sample of {sample} was moved by the wall");
        }
    }

    #[test]
    fn the_three_ways_leave_at_the_three_gains_of_the_alignment()
    {
        // The stage carries each way at its own gain, so a stage that ran one
        // gain three times, or crossed two of the three, answers alike on two
        // channels here.
        let words = words_of(1.0e9);

        assert_eq!(words.low().cast_signed(), to_word(1.0e9 * LOW_GAIN).cast_signed());
        assert_eq!(words.mid().cast_signed(), to_word(1.0e9 * MID_GAIN).cast_signed());
        assert_eq!(words.high().cast_signed(), to_word(1.0e9 * HIGH_GAIN).cast_signed());
        assert_ne!(words.low(), words.mid());
        assert_ne!(words.mid(), words.high());
        assert_ne!(words.low(), words.high());
    }

    #[test]
    fn a_value_that_is_not_a_number_leaves_silence_on_every_channel()
    {
        // The cast answers zero for it, and the wall names it before either
        // comparison, so the two agree instead of one of them deciding.
        let words = words_of(f32::NAN);

        assert_eq!(under_ceiling(f32::NAN), 0.0);
        assert_eq!(words.low(), 0);
        assert_eq!(words.mid(), 0);
        assert_eq!(words.high(), 0);
    }

    #[test]
    fn an_infinity_leaves_the_bound_on_its_own_side()
    {
        let high = words_of(f32::INFINITY);
        let low = words_of(f32::NEG_INFINITY);

        assert_eq!(high.high().cast_signed(), 1_336_379_648);
        assert_eq!(low.high().cast_signed(), -1_336_379_648);
        assert_eq!(high.low().cast_signed(), i32::MAX);
        assert_eq!(low.low().cast_signed(), i32::MIN);
        assert_eq!(high.mid().cast_signed(), i32::MAX);
        assert_eq!(low.mid().cast_signed(), i32::MIN);
    }

    #[test]
    fn a_finite_value_past_the_format_leaves_the_low_and_mid_ways_at_the_bound_on_its_own_side()
    {
        // The conversion is the bound of these two ways, and a source the
        // format carries takes the cascade of either past the format. These are
        // values the format holds, one a step over the format after the gain of
        // the way and two far past it, so a conversion that wraps from the
        // format up answers the opposite sign on the first of them. A conversion
        // that wraps over a band past the format these three miss answers the
        // opposite sign in the exhaustive sweep, which reads the sign of every
        // value the format holds on every way.
        let over = FULL_SCALE_WORDS * (1.0 + f32::EPSILON);
        let read = |name: &str, gain: f32, word: fn(WayWords) -> u32|
        {
            for sample in [over / gain, 4.0 * FULL_SCALE_WORDS, f32::MAX]
            {
                assert!(sample * gain > FULL_SCALE_WORDS, "{sample} stands inside the format");

                let high = word(words_of(sample)).cast_signed();
                let low = word(words_of(-sample)).cast_signed();

                assert_eq!(high, i32::MAX, "the {name} way answered {high} for {sample}");
                assert_eq!(low, i32::MIN, "the {name} way answered {low} for -{sample}");
            }
        };

        read("low", LOW_GAIN, WayWords::low);
        read("mid", MID_GAIN, WayWords::mid);
    }

    #[test]
    fn a_finite_value_above_the_ceiling_comes_back_at_the_ceiling()
    {
        // The readings beside this one take the wall from under it and from an
        // infinity, so a stage that answered the bound for an infinity and let
        // every finite value through would pass them both. These are values the
        // format holds, one a step over the wall and two far past it.
        for sample in [(HIGH_CEILING_WORDS + 128.0) / HIGH_GAIN, 4.0 * FULL_SCALE_WORDS, f32::MAX]
        {
            assert!(sample.is_finite(), "{sample} is not a value the reading is about");
            assert!(sample * HIGH_GAIN > HIGH_CEILING_WORDS, "{sample} stands under the wall");

            assert_eq!(words_of(sample).high().cast_signed(), 1_336_379_648);
            assert_eq!(words_of(-sample).high().cast_signed(), -1_336_379_648);
        }
    }

    #[test]
    fn no_value_of_any_kind_puts_the_high_channel_past_the_ceiling_or_a_way_across_zero()
    {
        // The wall bounds ONE sample, so what it claims is a property of a
        // total function of a single precision float, and the domain of that
        // function is four billion bit patterns rather than a shape of signal.
        // It is therefore walked whole, every pattern the format holds, values
        // that are not numbers and both infinities included. A draw of random
        // values would not be a proof of a bound.
        //
        // The same walk reads the sign of every word against the sign of the
        // value it came from, on the three ways. The gain of every way is
        // positive, so a word across zero from its value is a conversion that
        // came back round, and the walk hands every way every value its input
        // holds, so such a conversion shows here whatever band it wraps over.
        //
        // The stretches are run at once because the walk is four billion
        // crossings of the stage and a single file walk of it costs two
        // minutes. Cutting a domain into pieces is where an exhaustive reading
        // stops being exhaustive, so the pieces are joined twice over. Each one
        // is CUT to open where the one before it stopped, which leaves no
        // arithmetic that could place two of them on the same ground. And each
        // one REPORTS where it opened and where the walk left it, which is what
        // the reading below joins up, because sixteen stretches that covered
        // one eighth of the domain twice over would bring back four billion
        // crossings between them just the same.
        //
        // The join and the count hold the ground the walk covers. That the
        // stage runs on each pattern of that ground rests on the loop body of
        // `sweep`, and no reading below checks it.
        let ceiling = 1_336_379_648_u32;
        let stretch = SWEEP_PATTERNS / SWEEP_STRETCHES as u64;
        let mut opens_at = 0_u64;
        let bounds: [(u64, u64); SWEEP_STRETCHES] = core::array::from_fn
        (
            |index|
            {
                let upto = if index + 1 == SWEEP_STRETCHES
                {
                    SWEEP_PATTERNS
                }
                else
                {
                    opens_at + stretch
                };
                let bound = (opens_at, upto);

                opens_at = upto;
                bound
            }
        );
        let mut swept = EMPTY_STRETCH;

        let parts: [Swept; SWEEP_STRETCHES] = std::thread::scope
        (
            |scope|
            {
                let handles =
                    bounds.map(|(from, upto)| scope.spawn(move || sweep(from, upto)));

                // A stretch that dies brings back a stretch of no width, which
                // breaks the join below rather than shortening the count only.
                handles.map(|handle| handle.join().unwrap_or(EMPTY_STRETCH))
            }
        );

        let mut joined_at = 0_u64;

        for part in parts
        {
            assert_eq!
            (
                part.opens_at, joined_at,
                "a stretch opened at {} where the ground already walked ends at \
                 {joined_at}, so the stretches do not tile the domain",
                part.opens_at
            );

            joined_at = part.stops_at;
            swept.patterns += part.patterns;
            swept.low = swept.low.max(part.low);
            swept.mid = swept.mid.max(part.mid);
            swept.high = swept.high.max(part.high);
            swept.low_flipped += part.low_flipped;
            swept.mid_flipped += part.mid_flipped;
            swept.high_flipped += part.high_flipped;
        }

        assert_eq!
        (
            joined_at, SWEEP_PATTERNS,
            "the stretches joined up run to {joined_at} of the {SWEEP_PATTERNS} \
             patterns the format holds"
        );

        assert_eq!
        (
            swept.patterns, SWEEP_PATTERNS,
            "the sweep crossed {} patterns of the {SWEEP_PATTERNS} the format \
             holds, so it is not the whole domain",
            swept.patterns
        );

        assert!
        (
            swept.high <= ceiling,
            "a value of the format put {} words on the high channel, where the \
             ceiling stands at {ceiling}",
            swept.high
        );

        assert_eq!
        (
            swept.high, ceiling,
            "the sweep never reached the ceiling, so it read a bound nothing \
             in it came near"
        );

        for (name, flips) in
        [
            ("low", swept.low_flipped),
            ("mid", swept.mid_flipped),
            ("high", swept.high_flipped),
        ]
        {
            assert_eq!
            (
                flips, 0,
                "the {name} way answered a word of the opposite sign to its value \
                 on {flips} patterns"
            );
        }

        // The wall is on the high way and on no other, and the sweep drove all
        // three channels with the same value, so both other channels are
        // required to have stood past it. A stage that walled every way would
        // pass the reading above and fail this.
        assert!(swept.low > ceiling, "the low channel never passed the ceiling of the high way");
        assert!(swept.mid > ceiling, "the mid channel never passed the ceiling of the high way");
    }
}
