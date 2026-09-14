//! Output stage of the processing board, between the crossover and a buffer.
//!
//! The sample a way answers crosses the gain of that way, the product of its
//! sensitivity trim and the peak reserve formed once at compile time, then the
//! thermal limiter and the ceiling, which stand on the high way alone, then the
//! conversion to the word a buffer carries. The low and mid ways cross the gain
//! and the conversion. The high way crosses the limiter and then the wall
//! between the two, and the order is most of the design.
//!
//! `WayWords::of` is the whole of it and it is the only way to build the three
//! words. It takes the three samples the cascades answered and a limiter, so a
//! word that skipped the trim, the reserve, a limiter or the ceiling is not a
//! value this module can produce. The signature does not name WHICH limiter: a
//! caller in this crate that hands it a fresh one on every call gets unity gain
//! on every sample. What holds the limiter of the machine in place is that it
//! is a private field of `FilterChain`, which the carry hands to `of`, and a
//! test of `passthrough` that serves consecutive events and reads that field
//! run on from one event to the next.
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
//! # The thermal limiter, which holds the average under the rating
//!
//! What burns a voice coil is heat, and heat follows the power the coil takes
//! over time rather than the peak of one sample, so a wall on one sample cannot
//! tell a transient of a millisecond from a tone held for ten seconds at the
//! same level. `ThermalLimiter` averages the square of the high sample through
//! one pole with a time constant of 0.3 seconds. Once that average stands over
//! the threshold it brings the gain down to `1 / sqrt(average / threshold)`, so
//! the power it leaves stands at the threshold. The one pole is itself a first
//! order thermal model and the gain it drives follows it smoothly, so the
//! limiter carries one time constant and no separate attack or release.
//!
//! It reads the sample the trim and the reserve leave and multiplies that same
//! sample, which is feed forward: the average it reads is the average its gain
//! answers, so a held tone leaves at the threshold whatever its level. A
//! detector reading the sample its own gain has already brought down settles at
//! the geometric mean of the threshold and the input instead, 80 W for a tone
//! of 129 W. The wall stands behind the limiter and stays the last stage before
//! the conversion, so it binds on what the gain leaves.
//!
//! On a tone of 129 W the average crosses the threshold 0.147 seconds in, and on
//! one of 81.5 W 0.285 seconds in. A transient of one millisecond at 200 W moves
//! it to 0.664 W and limits nothing.
//!
//! The high cascade answers at or under unity at every frequency, to the
//! rounding of a double precision evaluation of its coefficients, and reaches
//! unity at the top of the band. Over a long run the mean square of any source
//! the format carries therefore leaves that cascade at no more than the square
//! of full scale, and leaves the way at no more than that times the square of
//! the gain of the way: 9.16 W, under a fifth of the threshold. That bounds a
//! long run mean. The detector averages over 0.3 seconds, and nothing here
//! bounds that shorter mean under the threshold, so
//! the limiter is there for the day the trim is not and for whatever a source
//! puts in one window of it.
//!
//! # The threshold, which names the rating of the wall
//!
//! It is the mean square of a sine whose PEAK stands at the wall, half the
//! square of the wall, so the thermal figure and the wall name one rating. The
//! square of the wall on its own names twice it.
//!
//! It then stands under that mean square by the stall of the recurrence. In
//! single precision a detector fed a constant square stops moving once its step
//! falls under half the last place of the average, which can leave it
//! `2^-23 / (2 a)` of the true mean away, 7.886e-4 of it or 0.0034 dB of power.
//! That is more than the 0.0024 dB the wall is rounded by, so the threshold
//! stands that far under the mean square and the rounding of the average falls
//! on the quiet side, as the rounding of the wall does.
//!
//! # What the limiter takes, and how fast its gain moves
//!
//! Before the detector and before the gain, a value that is not a number is
//! taken as silence and a magnitude past four times full scale as four times
//! full scale. A value that is not a number would stay in the recurrence for
//! good. The bound on the magnitude keeps the square finite for every input,
//! infinities and a diverging cascade included, and it bounds how far `r`, the
//! average over the threshold held at one or more, moves in one sample: by a
//! factor of at most `k = 1 + a (16 / threshold - 1)`, 1.006175. The average
//! itself moves by more from near zero, since one sample at the bound takes it
//! from 1e-10 to 1.2e-3, and `r` stands at one on both sides of that.
//!
//! The gain holds no square root, which the core library of this toolchain does
//! not expose and the math library computes in software on this target, and no
//! division, the threshold entering as its reciprocal formed at compile time.
//! It follows `1 / sqrt(r)`, with `r` the average over the threshold held at
//! one or more, by one Newton step a sample seeded with the gain before it:
//! `g (1.5 - 0.5 r g^2)`. That form peaks at `1 / sqrt(r)` with that value, so a
//! step lands at or under the exact law, to the rounding of the format, and at
//! or under unity. Since `r` moves by a factor of at most `k` a sample, the seed
//! stands within `e = sqrt(k) - 1` of the root, and the step departs from the
//! law by at most `1.5 e^2 + 0.5 e^3`, 1.43e-5 of it, to the rounding of the
//! format. The largest step down the gain takes is a factor of `1.5 - 0.5 k`,
//! 26.86 mdB.
//!
//! The same form lands at or under zero from a positive seed whose square times
//! `r` reaches three, and no limiter that ran from rest presents one. `at_rest`
//! presents `r g^2 = 1`. A gain at or under the law holds `r g^2 <= 1 + eps`,
//! with `eps` the rounding of the format, and the next sample takes `r` to `r'`
//! with `r' / r <= k`, so the next step presents `r' g^2 <= k (1 + eps)`, under
//! three. That step lands over zero and at or under the law again.
//!
//! A state outside that set, which only a corruption of the memory the limiter
//! sits in reaches, can hold a gain past unity, under zero, on an infinity or on
//! a value that is not a number. The form is odd in `g`, so a negative seed
//! whose square times `r` stands past three lands OVER zero, and far past unity
//! from a seed large enough. `hold` therefore stores the step inside `[0, 1]`:
//! a step that does not land over zero leaves the gain at zero, a step over
//! unity leaves it at unity, and a step from zero stays at zero. From the first
//! step such a state holds a gain in `[0, 1]`, and a step from a seed in that
//! interval lands over zero and at or under the law, to the rounding of the
//! format, or at or under zero. The state therefore returns under the law,
//! where the induction above takes over, or leaves the high way silent for
//! good. The one step it can leave over the law is the first, at or under
//! unity. Its polarity does not turn over, and the wall still binds behind it.
//!
//! # What an unbuilt one carries
//!
//! `silent` holds a gain of zero, and a step from zero stays at zero, so a
//! limiter no build reached answers silence on the high way for good. There is
//! no value of this type that means no limiting.
//!
//! A detector that falls under a floor 29 orders under the threshold is taken to
//! zero, so a long silence leaves it on zero rather than on the subnormal value
//! the recurrence would stop on.

use crate::constants::SAMPLE_RATE_HZ;

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

/// Gain the stage applies to the high way, as a fraction of full scale a word.
///
/// It turns a sample in words into the unit the limiter works in, in the same
/// multiplication as the gain. The division is by a power of two, so it is
/// exact and a product through this is the product through `HIGH_GAIN` scaled
/// by the same power.
const HIGH_GAIN_OF_FULL_SCALE: f32 = HIGH_GAIN / FULL_SCALE_WORDS;

/// Largest magnitude the limiter takes a sample at, as a fraction of full scale.
///
/// Twice the loudest peak a source the format carries puts on the high way with
/// the trim removed and the reserve in place, 1.964 of full scale, so no such
/// source reaches it and what it bounds is a defect: a value past that, an
/// infinity, a cascade that diverges. The vector unit encodes it as an
/// immediate.
const LIMITER_INPUT_BOUND: f32 = 4.0;

/// Seconds the detector of the limiter averages over.
const INTEGRATION_SECONDS: f64 = 0.3;

/// Coefficient of the one pole detector, `1 - exp(-1 / (tau fs))`.
///
/// The argument is 7.6e-5, so the series to its third term leaves a remainder
/// of the fourth power, which the narrowing to single precision cannot see.
#[expect
(
    clippy::cast_possible_truncation,
    reason = "the detector runs in single precision, and the narrowing is the \
              step that gets the coefficient there"
)]
const fn detector_coefficient() -> f32
{
    let x = 1.0 / (INTEGRATION_SECONDS * SAMPLE_RATE_HZ as f64);

    (x - x * x / 2.0 + x * x * x / 6.0) as f32
}

/// Coefficient of the one pole detector.
const DETECTOR_COEFFICIENT: f32 = detector_coefficient();

/// Relative distance from the true mean square the detector can stop at.
///
/// A step of the recurrence rounds away once it falls under half the last
/// place of the average, and the last place of a single precision float is at
/// most `f32::EPSILON` of its value.
const STALL_BOUND: f32 = f32::EPSILON / (2.0 * DETECTOR_COEFFICIENT);

/// Mean square of a sine whose peak stands at the ceiling, as a fraction of the
/// square of full scale.
const CEILING_MEAN_SQUARE: f32 = HIGH_CEILING_RATIO * HIGH_CEILING_RATIO / 2.0;

/// Mean square the limiter holds the high way at.
const THRESHOLD: f32 = CEILING_MEAN_SQUARE * (1.0 - STALL_BOUND);

/// Reciprocal of the threshold, which the limiter multiplies by.
const INVERSE_THRESHOLD: f32 = 1.0 / THRESHOLD;

/// Average under which the detector reads zero.
///
/// 29 orders under the threshold and 8 over the smallest normal float, so taking
/// the detector to zero moves it by less than the last place of any average
/// near the threshold, and leaves it on no subnormal value.
const AVERAGE_FLOOR: f32 = 1.0e-30;

/// Thermal limiter of the high way: its average and the gain that follows it.
///
/// The module documentation carries the law, the threshold and the bounds.
/// The two fields are private and `silent` and `at_rest` are the only values
/// built outside `hold`, so the state `hold` runs on is one those two start and
/// the samples it was fed reach.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ThermalLimiter
{
    /// Mean square of the high way, as a fraction of the square of full scale.
    average: f32,
    /// Gain applied to the high way, following `1 / sqrt(average / threshold)`.
    gain: f32,
}

impl ThermalLimiter
{
    /// Returns a limiter that passes no signal.
    ///
    /// Its gain is zero, and a step of the gain from zero lands on zero, so it
    /// answers silence for good.
    pub(crate) const fn silent() -> Self
    {
        Self { average: 0.0, gain: 0.0 }
    }

    /// Returns a limiter with no average behind it, at unity gain.
    pub(crate) const fn at_rest() -> Self
    {
        Self { average: 0.0, gain: 1.0 }
    }

    /// Returns `sample` under the gain the average of the way calls for, and
    /// advances the average and the gain by one sample.
    ///
    /// `sample` is the high sample after its gain, as a fraction of full scale,
    /// and so is the value returned. The detector reads the value `sample` is
    /// bounded to and the gain multiplies that same value, so what is returned
    /// is finite and within `LIMITER_INPUT_BOUND` of zero for every input.
    ///
    /// The gain it stores stands in `[0, 1]`. A step that does not land over
    /// zero, a value that is not a number included, leaves the gain at zero
    /// rather than at unity or under zero, and a step over unity leaves it at
    /// unity.
    fn hold(&mut self, sample: f32) -> f32
    {
        let held = inside_the_limiter_bound(sample);
        let average = self.average + DETECTOR_COEFFICIENT * (held * held - self.average);

        self.average = if average < AVERAGE_FLOOR { 0.0 } else { average };

        let scaled = self.average * INVERSE_THRESHOLD;
        let ratio = if scaled <= 1.0 { 1.0 } else { scaled };
        let gain = self.gain * (1.5 - 0.5 * (ratio * (self.gain * self.gain)));

        self.gain = if gain > 1.0
        {
            1.0
        }
        else if gain > 0.0
        {
            gain
        }
        else
        {
            0.0
        };

        held * self.gain
    }
}

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
/// and the peak reserve, then the limiter and the ceiling on the high way, then
/// the conversion.
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
    /// Returns the words `samples` leave the machine with, and advances
    /// `limiter` by one sample of the high way.
    ///
    /// It takes the samples the cascades answered and a limiter, so a word that
    /// skipped the trim or the reserve, or a high word that skipped a limiter
    /// or the wall, is not a value this function can be made to return. The
    /// limiter reads the high sample after its gain and before the wall, and
    /// the wall is the last stage that sample crosses.
    ///
    /// Nothing in this signature ties `limiter` to the one the machine runs. A
    /// caller that hands a fresh one on every call gets unity gain on every
    /// sample. The carry hands the private field of `FilterChain`, and a test
    /// of `passthrough` serves consecutive events and reads that field run on.
    pub(crate) fn of(samples: WaySamples, limiter: &mut ThermalLimiter) -> Self
    {
        let high = limiter.hold(samples.high * HIGH_GAIN_OF_FULL_SCALE) * FULL_SCALE_WORDS;

        Self
        {
            low: to_word(samples.low * LOW_GAIN),
            mid: to_word(samples.mid * MID_GAIN),
            high: to_word(under_ceiling(high)),
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

impl ThermalLimiter
{
    /// Returns the gain the limiter holds.
    ///
    /// It is here for the tests of the loop under this stage, which read that
    /// the limiter acted on a drive rather than inferring it from the words.
    #[cfg(test)]
    pub(crate) const fn current_gain_for_test(self) -> f32
    {
        self.gain
    }

    /// Returns the average the limiter holds.
    ///
    /// It is here for the tests of the loop under this stage, which read how
    /// far a drive took the detector.
    #[cfg(test)]
    pub(crate) const fn current_average_for_test(self) -> f32
    {
        self.average
    }

    /// Returns the mean square the limiter holds the high way at, and the
    /// largest magnitude it takes a sample at.
    ///
    /// It is here for the tests of the loop under this stage, which derive the
    /// range the state of the limiter reaches from these two.
    #[cfg(test)]
    pub(crate) const fn threshold_and_input_bound_for_test() -> (f32, f32)
    {
        (THRESHOLD, LIMITER_INPUT_BOUND)
    }
}

/// Returns `value` taken as silence when it is not a number and held to the
/// input bound of the limiter otherwise.
#[expect
(
    clippy::manual_clamp,
    reason = "the answer for a value that is not a number is silence and not a \
              bound, which is a fourth case a clamp does not carry"
)]
fn inside_the_limiter_bound(value: f32) -> f32
{
    if value.is_nan()
    {
        0.0
    }
    else if value > LIMITER_INPUT_BOUND
    {
        LIMITER_INPUT_BOUND
    }
    else if value < -LIMITER_INPUT_BOUND
    {
        -LIMITER_INPUT_BOUND
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
    HIGH_GAIN_OF_FULL_SCALE * FULL_SCALE_WORDS == HIGH_GAIN,
    "the gain of the high way in the unit of the limiter is the gain of the high \
     way scaled by an exact power of two"
);

const _: () = assert!
(
    DETECTOR_COEFFICIENT > 0.0 && DETECTOR_COEFFICIENT < 1.0,
    "a one pole detector with a coefficient outside the unit interval does not \
     average"
);

const _: () = assert!
(
    (CEILING_MEAN_SQUARE as f64) * (1.0 - STALL_BOUND as f64) - THRESHOLD as f64
        >= -(THRESHOLD as f64) * (f32::EPSILON as f64) / 2.0,
    "the threshold stands under the mean square the wall names by the stall of \
     the recurrence, to the rounding of the threshold itself"
);

const _: () = assert!
(
    AVERAGE_FLOOR >= f32::MIN_POSITIVE && AVERAGE_FLOOR * 1.0e29 < THRESHOLD,
    "the floor is a normal float and stands 29 orders under the threshold"
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
    use core::f64::consts::TAU;
    use libm::{exp, expm1, log, log10, log10f, powf, sin, sqrt, sqrtf};

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
    /// values it reaches.
    const SWEEP_STRETCHES: usize = 16;

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
        off_unity: 0,
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
        /// Patterns the limiter answered with a gain other than unity.
        off_unity: u64,
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

    /// Returns the words one value answers on the three channels, through a
    /// limiter at rest.
    ///
    /// A limiter at rest answers unity on any one sample: the average one
    /// sample leaves stands under the threshold even at the input bound, so the
    /// step of the gain lands on unity. What comes back is therefore the
    /// loudest the stage answers for that value, and the sweep below counts
    /// that it is so on every pattern.
    fn words_of(value: f32) -> WayWords
    {
        WayWords::of
        (
            WaySamples { low: value, mid: value, high: value },
            &mut ThermalLimiter::at_rest()
        )
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

    /// Runs the stage over every bit pattern from `from` up to `upto`, each
    /// through a limiter of its own at rest.
    ///
    /// The stage holds state on the high way, so it answers a pattern by a
    /// state as well as by the pattern. The gain is in the unit interval on
    /// every state a limiter reaches, and the stage after it is monotone in the
    /// magnitude and keeps the sign, so the loudest word and the only sign a
    /// pattern can take come off the state at unity gain. A limiter at rest is
    /// that state, and it stands in front of each pattern rather than running
    /// on, so no pattern reads a state another one left and the stretches stay
    /// independent of the order they are cut in. `off_unity` counts the
    /// patterns whose gain was anything but unity.
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
            let mut limiter = ThermalLimiter::at_rest();
            let words =
                WayWords::of(WaySamples { low: value, mid: value, high: value }, &mut limiter);

            swept.patterns += 1;
            swept.stops_at = pattern + 1;
            swept.low = swept.low.max(magnitude(words.low()));
            swept.mid = swept.mid.max(magnitude(words.mid()));
            swept.high = swept.high.max(magnitude(words.high()));
            swept.low_flipped += flipped(value, words.low());
            swept.mid_flipped += flipped(value, words.mid());
            swept.high_flipped += flipped(value, words.high());
            swept.off_unity += u64::from(limiter.gain != 1.0);
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
    #[cfg_attr(coverage, ignore = "the uninstrumented test run walks the same patterns, in a fraction of the time")]
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
        // crossings of the stage. Cutting a domain into pieces is where an
        // exhaustive reading stops being exhaustive, so the pieces are joined
        // twice over. Each one is CUT to open where the one before it stopped,
        // which leaves no arithmetic that could place two of them on the same
        // ground. And each one REPORTS where it opened and where the walk left
        // it, which is what the reading below joins up, because sixteen
        // stretches that covered one eighth of the domain twice over would
        // bring back four billion crossings between them just the same.
        //
        // The join and the count hold the ground the walk covers. That the
        // stage runs on each pattern of that ground rests on the loop body of
        // `sweep`, and no reading below checks it.
        //
        // The high way holds a limiter, so the stage answers a pattern by the
        // state of that limiter as well. Each pattern crosses a limiter at rest
        // of its own, which answers unity, the largest gain any state holds, and
        // the walk counts that no pattern met another gain. That is what makes
        // the readings below the worst case over every state and not over one.
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
            swept.off_unity += part.off_unity;
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

        assert_eq!
        (
            swept.off_unity, 0,
            "the limiter answered a gain other than unity on {} patterns, so the \
             sweep did not read the stage at its loudest",
            swept.off_unity
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

    /// Watts the rating of the wall is written in.
    ///
    /// A sine whose peak stands at the wall stands for this figure, so a power
    /// below is read in the same unit as the rating it is compared against.
    const RATED_WATTS: f64 = 50.0;

    /// Returns the mean square of a sine whose peak stands at the wall.
    ///
    /// Formed here from the wall rather than read off the stage, so a stage
    /// whose threshold names another level reads that level in these watts.
    fn wall_mean_square() -> f64
    {
        f64::from(HIGH_CEILING_RATIO) * f64::from(HIGH_CEILING_RATIO) / 2.0
    }

    /// Returns the power `mean_square` stands for, in the unit of the rating.
    fn watts(mean_square: f64) -> f64
    {
        RATED_WATTS * mean_square / wall_mean_square()
    }

    /// Returns the peak of a sine standing for `power` watts, as a fraction of
    /// full scale.
    fn sine_peak(power: f64) -> f64
    {
        sqrt(2.0 * wall_mean_square() * power / RATED_WATTS)
    }

    /// Returns the average `limiter` holds over the threshold, held at one or
    /// more, in double precision.
    fn ratio_of(limiter: ThermalLimiter) -> f64
    {
        (f64::from(limiter.average) / f64::from(THRESHOLD)).max(1.0)
    }

    /// Returns the high word the stage answers for a high sample that stands at
    /// `fraction` of full scale once the gain of the way has acted.
    fn high_word(limiter: &mut ThermalLimiter, fraction: f32) -> i32
    {
        let samples = WaySamples
        {
            low: 0.0,
            mid: 0.0,
            high: fraction / HIGH_GAIN_OF_FULL_SCALE,
        };

        WayWords::of(samples, limiter).high().cast_signed()
    }

    /// Returns the largest factor by which `r`, the average over the threshold
    /// held at one or more, moves in one sample.
    fn largest_ratio_step() -> f64
    {
        let bound = f64::from(LIMITER_INPUT_BOUND);

        1.0 + f64::from(DETECTOR_COEFFICIENT) * (bound * bound / f64::from(THRESHOLD) - 1.0)
    }

    /// Returns the largest relative distance under the exact law one step of
    /// the gain lands at, from a seed on the law before the average moved.
    fn largest_departure() -> f64
    {
        let error = sqrt(largest_ratio_step()) - 1.0;

        1.5 * error * error + 0.5 * error * error * error
    }

    #[test]
    fn the_detector_coefficient_is_the_exponential_of_the_integration_time()
    {
        // The series is what the constant is built with, so it is read here
        // against the two forms of the exponential it stands for rather than
        // against itself.
        let argument = 1.0 / (INTEGRATION_SECONDS * f64::from(SAMPLE_RATE_HZ));

        assert_eq!(DETECTOR_COEFFICIENT.to_bits(), ((1.0 - exp(-argument)) as f32).to_bits());
        assert_eq!(DETECTOR_COEFFICIENT.to_bits(), ((-expm1(-argument)) as f32).to_bits());
        assert!((DETECTOR_COEFFICIENT - 7.558_293e-5).abs() < 1.0e-11);

        // One time constant of samples takes what is left of a step to e to
        // the minus one.
        let left = powf(1.0 - DETECTOR_COEFFICIENT, 0.3 * 44_100.0);

        assert!((f64::from(left) - exp(-1.0)).abs() < 1.0e-3, "a time constant leaves {left}");
    }

    #[test]
    fn the_threshold_is_the_mean_square_of_a_sine_whose_peak_is_the_wall()
    {
        // A sine with its peak at the wall, read over a whole number of periods,
        // has the mean square the threshold is built from.
        const SAMPLES: u32 = 44_100;
        const PERIODS: f64 = 1_000.0;

        let mut square = 0.0_f64;

        for n in 0..SAMPLES
        {
            let value = f64::from(HIGH_CEILING_RATIO)
                * sin(TAU * PERIODS * f64::from(n) / f64::from(SAMPLES));

            square += value * value;
        }

        let mean = square / f64::from(SAMPLES);

        assert!
        (
            (mean - f64::from(CEILING_MEAN_SQUARE)).abs() < 1.0e-8,
            "the sine at the wall has a mean square of {mean}"
        );
        assert!((CEILING_MEAN_SQUARE - 0.193_628_82).abs() < 1.0e-8);

        // Without the halving the same constant names twice the rating.
        let unhalved = watts(f64::from(HIGH_CEILING_RATIO * HIGH_CEILING_RATIO));

        assert!((unhalved - 100.0).abs() < 1.0e-4, "the square of the wall names {unhalved} W");

        // The stall, in the relative unit and in decibels of power, and the
        // rounding of the wall it stands over.
        let stall_db = 10.0 * log10(1.0 / (1.0 - f64::from(STALL_BOUND)));
        let wall_rounding_db = 0.0024;

        assert!((STALL_BOUND - 7.886e-4).abs() < 1.0e-7, "the stall bound reads {STALL_BOUND}");
        assert!((stall_db - 0.0034).abs() < 0.00005, "the stall reads {stall_db} dB");
        assert!(stall_db > wall_rounding_db);

        let under = 1.0 - f64::from(THRESHOLD) / wall_mean_square();

        assert!
        (
            (under - f64::from(STALL_BOUND)).abs() < f64::from(f32::EPSILON),
            "the threshold stands {under} under the mean square of the wall"
        );
    }

    #[test]
    fn the_recurrence_stops_no_further_under_a_constant_square_than_the_stall_bound()
    {
        // A detector fed one square over and over converges on it until its
        // step rounds away. The distance it stops at is widest where the last
        // place of the average is widest against its value, which is a
        // mantissa just over one, so the levels are walked there on several
        // binades, from under the square and from over it.
        //
        // Stopping UNDER the square is the loud side, since the gain then
        // answers a smaller average than the signal carries, and that is the
        // side the threshold is taken down for. Stopping over it is the quiet
        // side and is bounded one factor of the bound wider.
        let mut loud = 0.0_f64;
        let mut quiet = 0.0_f64;

        for binade in [-20_i16, -9, -4, -1, 0, 2]
        {
            for step in 0..24_u16
            {
                let wanted = powf(2.0, f32::from(binade)) * (1.0 + f32::from(step) * 0.0005);
                let sample = sqrtf(wanted);
                let square = f64::from(sample * sample);

                for side in [0.99_f64, 1.01]
                {
                    let mut limiter = ThermalLimiter
                    {
                        average: (square * side) as f32,
                        gain: 1.0,
                    };

                    for _ in 0..400_000_u32
                    {
                        let before = limiter.average;

                        limiter.hold(sample);

                        if limiter.average == before
                        {
                            break;
                        }
                    }

                    let distance = (f64::from(limiter.average) - square) / square;

                    assert!(distance.abs() < 1.0e-3, "the detector did not stop near {square}");

                    if distance < 0.0
                    {
                        loud = loud.max(-distance);
                    }
                    else
                    {
                        quiet = quiet.max(distance);
                    }
                }
            }
        }

        let bound = f64::from(STALL_BOUND);

        assert!(loud <= bound * (1.0 + 1.0e-6), "the detector stopped {loud} under its square");
        assert!(quiet <= bound / (1.0 - bound) * (1.0 + 1.0e-6), "it stopped {quiet} over it");
        assert!(loud > 0.99 * bound, "the walk never came near the bound, at {loud}");
    }

    #[test]
    fn a_held_tone_over_the_threshold_leaves_at_the_power_the_threshold_names()
    {
        // Driven through the stage, so what is read is the word the high way
        // leaves and the limiter sits where the stage puts it: behind the gain
        // of the way and in front of the wall. The tone stands where the trim
        // lets nothing reach, which is the only place the limiter acts.
        //
        // Half a second at 2000 Hz holds a whole number of periods, so the
        // mean square of the tail reads no leakage. That tail opens 5.5 seconds
        // in, where what is left of the rise stands at e to the minus 18.3 of
        // it.
        const TONE_HZ: f64 = 2_000.0;
        const HELD_SAMPLES: u32 = 6 * 44_100;
        const READ_SAMPLES: u32 = 22_050;

        for (power, figure) in [(129.0_f64, 0.147_f64), (81.5, 0.285)]
        {
            let peak = sine_peak(power);
            let mut limiter = ThermalLimiter::at_rest();
            let mut crossed = None;
            let mut square = 0.0_f64;

            for n in 0..HELD_SAMPLES
            {
                let angle = TAU * TONE_HZ * f64::from(n) / f64::from(SAMPLE_RATE_HZ);
                let word = high_word(&mut limiter, (peak * sin(angle)) as f32);

                if crossed.is_none() && limiter.gain < 1.0
                {
                    crossed = Some(n);
                }

                if n >= HELD_SAMPLES - READ_SAMPLES
                {
                    let fraction = f64::from(word) / f64::from(FULL_SCALE_WORDS);

                    square += fraction * fraction;
                }
            }

            let left = watts(square / f64::from(READ_SAMPLES));
            let stall = f64::from(STALL_BOUND);

            assert!(left <= RATED_WATTS, "a tone of {power} W left at {left} W");
            assert!
            (
                left >= RATED_WATTS * (1.0 - 2.0 * stall),
                "a tone of {power} W left at {left} W, further under the rating than \
                 the stall takes the threshold"
            );

            let Some(crossed) = crossed
            else
            {
                panic!("a tone of {power} W never took the gain under unity");
            };
            let crossed_s = f64::from(crossed) / f64::from(SAMPLE_RATE_HZ);
            let mean_square = peak * peak / 2.0;
            let closed = log(1.0 - f64::from(THRESHOLD) / mean_square)
                / log(1.0 - f64::from(DETECTOR_COEFFICIENT))
                / f64::from(SAMPLE_RATE_HZ);

            assert!
            (
                (crossed_s - closed).abs() < 1.0e-3,
                "a tone of {power} W crossed at {crossed_s} s where the recurrence \
                 crosses at {closed} s"
            );
            assert!((crossed_s - figure).abs() < 1.0e-3, "a tone of {power} W crossed at {crossed_s} s");
        }
    }

    #[test]
    fn a_transient_of_one_millisecond_at_two_hundred_watts_limits_nothing()
    {
        // The same level, held, is what the limiter is for. Held one
        // millisecond it deposits a ten-thousandth of what ten seconds
        // deposit, and the average moves by two thirds of a watt.
        const TRANSIENT_SAMPLES: u32 = 44;
        const HELD_SAMPLES: u32 = 44_100;

        let level = (2.0 * sqrt(wall_mean_square())) as f32;
        let mut limiter = ThermalLimiter::at_rest();

        assert!((watts(f64::from(level * level)) - 200.0).abs() < 1.0e-3);

        for sample in 0..TRANSIENT_SAMPLES
        {
            let mut alone = ThermalLimiter::at_rest();

            assert_eq!(high_word(&mut limiter, level), high_word(&mut alone, level));
            assert_eq!(limiter.gain, 1.0, "the transient moved the gain at sample {sample}");
        }

        let moved = watts(f64::from(limiter.average));

        assert!((moved - 0.664).abs() < 0.001, "the transient moved the average to {moved} W");

        for _ in TRANSIENT_SAMPLES..HELD_SAMPLES
        {
            high_word(&mut limiter, level);
        }

        assert!(limiter.gain < 0.6, "a second at 200 W left the gain at {}", limiter.gain);
    }

    #[test]
    fn the_gain_lands_at_or_under_the_exact_law_and_under_unity_over_every_state_and_seed()
    {
        // One step of the gain, from every pair of an average and a seed on a
        // grid that covers both: averages from zero up past the input bound
        // squared, the floor and the threshold included, seeds from zero to
        // unity, and a sample from silence to the bound. The step reads the
        // average the sample leaves, so the law is read at that average.
        //
        // The form peaks at the law, so no pair lands above it past the
        // rounding of the format, and none lands above unity. Every seed here
        // is positive, and from a positive seed the form lands at or under zero
        // once the seed squared times the ratio reaches three, and over zero
        // under that. A limiter run from rest presents that product
        // at no more than the largest ratio step, which the test beside this
        // one reads.
        let mut averages = [0.0_f32; 72];
        let mut seeds = [0.0_f32; 64];

        for (index, average) in (0_u8..).zip(averages.iter_mut())
        {
            *average = match index
            {
                0 => 0.0,
                1 => AVERAGE_FLOOR,
                2 => THRESHOLD,
                3 => f32::from_bits(THRESHOLD.to_bits() + 1),
                4 => f32::from_bits(THRESHOLD.to_bits() - 1),
                _ => powf(10.0, -8.0 + 9.3 * f32::from(index - 5) / 66.0),
            };
        }

        for (index, seed) in (0_u8..).zip(seeds.iter_mut())
        {
            *seed = match index
            {
                0 => 0.0,
                1 => 1.0,
                2 => f32::from_bits(1.0_f32.to_bits() - 1),
                _ => powf(10.0, -3.0 * f32::from(63 - index) / 60.0),
            };
        }

        let mut above_law = 0.0_f64;
        let mut landed_on_unity = false;
        let mut past_three = 0_u32;

        for average in averages
        {
            for seed in seeds
            {
                for sample in [0.0_f32, 0.25, 1.0, 2.0, LIMITER_INPUT_BOUND]
                {
                    let mut limiter = ThermalLimiter { average, gain: seed };

                    limiter.hold(sample);

                    let ratio = ratio_of(limiter);
                    let law = 1.0 / sqrt(ratio);
                    let gain = f64::from(limiter.gain);
                    let product = ratio * f64::from(seed) * f64::from(seed);

                    assert!(gain <= 1.0, "a seed of {seed} on {average} landed at {gain}");

                    above_law = above_law.max(gain - law);
                    landed_on_unity |= limiter.gain == 1.0;

                    if product < 3.0
                    {
                        assert!
                        (
                            seed == 0.0 || gain > 0.0,
                            "a seed of {seed} on {average} landed at {gain} with the \
                             product at {product}"
                        );
                    }
                    else
                    {
                        past_three += 1;
                    }
                }
            }
        }

        assert!
        (
            above_law <= f64::from(f32::EPSILON),
            "a step landed {above_law} over the exact law"
        );
        assert!(landed_on_unity, "no step of the grid reached unity, so the bound read nothing");
        assert!(past_three > 0, "the grid never presented a product past three");
    }

    #[test]
    fn a_limiter_run_from_rest_holds_its_gain_within_the_departure_of_the_law()
    {
        // The step above is a bound over any pair. What keeps a running
        // limiter inside it is that `r` moves by at most the largest ratio
        // step in one sample, so the seed it presents stays that close
        // to the root. Walked here over drives that move the average as fast
        // as the input bound lets it, in both directions, with bursts, noise
        // and values that are not numbers, at every sample.
        let step = largest_ratio_step();
        let departure = largest_departure();
        let fall = 1.0 / (1.5 - 0.5 * step);
        let rounding = 4.0 * f64::from(f32::EPSILON);
        let bound = LIMITER_INPUT_BOUND;
        let mut noise = 0x1234_5678_u32;
        let mut next = move ||
        {
            noise = noise.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            noise
        };
        let mut worst_product = 1.0_f64;
        let mut worst_departure = 0.0_f64;
        let mut worst_fall = 1.0_f64;
        let mut lowest = 1.0_f64;

        let mut walk = |name: &str, samples: u32, sample: &mut dyn FnMut(u32) -> f32|
        {
            let mut limiter = ThermalLimiter::at_rest();

            for n in 0..samples
            {
                let seed = f64::from(limiter.gain);

                limiter.hold(sample(n));

                let ratio = ratio_of(limiter);
                let gain = f64::from(limiter.gain);
                let product = ratio * seed * seed;
                let off = 1.0 - gain * sqrt(ratio);

                assert!(product <= step * (1.0 + rounding), "{name}: the product reached {product}");
                assert!(gain > 0.0 && gain <= 1.0, "{name}: the gain reached {gain} at {n}");
                assert!(off <= departure * (1.0 + 1.0e-3) + rounding, "{name}: {off} under the law");
                assert!(off >= -rounding, "{name}: {off} over the law at {n}");
                assert!(seed / gain <= fall * (1.0 + rounding), "{name}: a fall of {} at {n}", seed / gain);

                worst_product = worst_product.max(product);
                worst_departure = worst_departure.max(off);
                worst_fall = worst_fall.max(seed / gain);
                lowest = lowest.min(gain);
            }
        };

        walk("the bound held then silence", 4 * 44_100, &mut |n| if n < 3 * 44_100 { bound } else { 0.0 });
        walk("the bound at alternate signs", 2 * 44_100, &mut |n| if n % 2 == 0 { bound } else { -bound });

        for (on, off) in [(1_u32, 1_u32), (10, 100), (441, 441), (4_410, 4_410), (1, 4_409)]
        {
            walk("bursts at the bound", 2 * 44_100, &mut |n| if n % (on + off) < on { bound } else { 0.0 });
        }

        walk
        (
            "noise inside the bound",
            2 * 44_100,
            &mut |_| (f64::from(next()) / f64::from(u32::MAX) * 8.0 - 4.0) as f32
        );
        walk
        (
            "noise past the bound, with values that are not numbers",
            2 * 44_100,
            &mut |_|
            {
                let draw = next();

                match draw % 100
                {
                    0 => f32::NAN,
                    1 => f32::INFINITY,
                    2 => f32::NEG_INFINITY,
                    _ => (f64::from(draw) / f64::from(u32::MAX) * 16.0 - 8.0) as f32,
                }
            }
        );
        walk
        (
            "a sine of 1550 W",
            3 * 44_100,
            &mut |n| (sine_peak(1_550.0) * sin(TAU * 3_000.0 * f64::from(n) / 44_100.0)) as f32
        );

        assert!((step - 1.006_175).abs() < 1.0e-6, "the largest ratio step reads {step}");
        assert!((departure - 1.427e-5).abs() < 1.0e-8, "the departure reads {departure}");
        assert!(worst_product > 1.0 + 0.9 * (step - 1.0), "the drives presented {worst_product}");
        assert!(worst_departure > 0.9 * departure, "the drives departed {worst_departure}");
        assert!(worst_fall > 1.0 + 0.9 * (fall - 1.0), "the drives fell by {worst_fall}");

        let floor = f64::from(sqrtf(THRESHOLD) / LIMITER_INPUT_BOUND);

        assert!(lowest >= floor * (1.0 - departure) - rounding, "the gain fell to {lowest}");
        assert!(lowest < floor * 1.01, "no drive took the gain near its floor, at {lowest}");
    }

    #[test]
    fn the_largest_step_down_of_the_gain_is_taken_from_the_threshold_at_the_input_bound()
    {
        // The largest factor `r` moves by is met with the average at the
        // threshold and the input at its bound, and the gain then stands at
        // unity, so the step lands at `1.5 - 0.5 k` exactly. The exact law
        // steps by `k` under a square root there, a little less.
        let mut limiter = ThermalLimiter { average: THRESHOLD, gain: 1.0 };

        limiter.hold(LIMITER_INPUT_BOUND);

        let step = largest_ratio_step();
        let taken = -20_000.0 * log10(f64::from(limiter.gain));
        let closed = -20_000.0 * log10(1.5 - 0.5 * step);
        let of_the_law = 10_000.0 * log10(step);

        assert!((ratio_of(limiter) - step).abs() < 1.0e-6);
        assert!((taken - closed).abs() < 0.001, "the step took {taken} mdB against {closed}");
        assert!((closed - 26.86).abs() < 0.005, "the closed form reads {closed} mdB");
        assert!((of_the_law - 26.73).abs() < 0.01, "the law steps by {of_the_law} mdB");
    }

    #[test]
    fn over_a_long_run_no_source_the_format_carries_leaves_the_high_way_over_a_fifth_of_the_threshold()
    {
        // The limiter reads the high sample through the high cascade and then
        // the gain of the way. The cascade answers at or under unity at every
        // frequency of the sweep, a step of half a hertz from the bottom of the
        // band to Nyquist, so over a long run the mean square it leaves stands
        // at or under the mean square of its source, and full scale bounds
        // that. What this bounds is a long run mean. The detector averages
        // over 0.3 seconds, and this reads nothing of that shorter mean.
        //
        // The cascade reaches unity at the top of the band, and there the
        // double precision evaluation reads it a few parts in 1e16 either side
        // of one, so the reading allows that rounding and nothing a filter
        // could add.
        const HALF_HERTZ_STEPS: u16 = 44_100;
        const EVALUATION_ROUNDING: f64 = 1.0e-12;

        let mut sections = [crate::filter::Biquad::SILENT; crate::filter::CROSSOVER_SECTIONS];

        let Ok(count) = crate::filter::crossover_sections(crate::filter::Way::High, &mut sections)
        else
        {
            panic!("the high way refused its own corners");
        };
        let active = sections.get(..count).unwrap_or(&[]);
        let mut peak = 0.0_f64;

        for step in 1..HALF_HERTZ_STEPS
        {
            let frequency = f32::from(step) * 0.5;
            let Ok(magnitude) = crate::filter::cascade_magnitude(active, frequency, SAMPLE_RATE_HZ)
            else
            {
                panic!("the sweep stood outside the band at {frequency} Hz");
            };

            assert!
            (
                magnitude <= 1.0 + EVALUATION_ROUNDING,
                "the high cascade answers {magnitude} at {frequency} Hz"
            );

            peak = peak.max(magnitude);
        }

        assert!(peak > 1.0 - 1.0e-9, "the sweep never reached the pass band, its peak is {peak}");

        let square = watts(f64::from(HIGH_GAIN) * f64::from(HIGH_GAIN) * peak * peak);

        assert!((square - 9.16).abs() < 0.005, "a full scale source leaves the way at {square} W");
        assert!(square < watts(f64::from(THRESHOLD)) / 5.0);
    }

    /// A drive of the limiter: what a failure calls it, the average it opens
    /// on, and the sample it holds at each index.
    type Drive<'a> = (&'a str, f32, &'a dyn Fn(u32) -> f32);

    #[test]
    fn a_gain_outside_the_reachable_set_returns_under_the_law_or_falls_silent_for_good()
    {
        // No limiter run from rest holds any of these seeds, so what reaches
        // them is a corruption of the memory the state sits in. They are driven
        // under silence and under two held tones that start on their own mean
        // square: 129 W, and the power whose average stands at 2.33 times the
        // threshold.
        //
        // The step is odd in the seed. A negative seed `g` takes the raw step
        // over zero once `r g^2` stands past 3, and past unity once it stands
        // past `3 + 2 / |g|`. At a ratio of one that is a seed under -2, and -2
        // lands on unity exactly. At each of the three ratios driven, the
        // negative seeds stand on both sides of both bounds.
        //
        // At every step, the first included, the gain stands in `[0, 1]` and
        // is finite. From the second step on, a gain over zero stands at or
        // under the law, and a gain that reached zero stays there. The first
        // step is the one a seed outside `[0, 1]` can leave over the law.
        const HELD_SAMPLES: u32 = 44_100;

        let loud = sine_peak(129.0);
        let escape = f64::from(THRESHOLD) * 2.33;
        let near = sqrt(2.0 * escape);
        let drives: [Drive<'_>; 3] =
        [
            ("silence", 0.0, &|_| 0.0),
            (
                "a tone of 129 W",
                (loud * loud / 2.0) as f32,
                &|n| (loud * sin(TAU * 2_000.0 * f64::from(n) / 44_100.0)) as f32
            ),
            (
                "a tone at 2.33 times the threshold",
                escape as f32,
                &|n| (near * sin(TAU * 2_000.0 * f64::from(n) / 44_100.0)) as f32
            ),
        ];
        let seeds =
        [
            1.05_f32,
            1.5,
            1.8,
            2.0,
            1_000.0,
            f32::INFINITY,
            -1.0,
            -1.33,
            -1.74,
            -1.8,
            -2.0,
            -3.0,
            -1_000.0,
            f32::NEG_INFINITY,
            f32::NAN,
        ];
        let mut returned = 0_u32;
        let mut silenced = 0_u32;
        let mut over_the_law_first = 0_u32;

        assert!
        (
            (f64::from(escape as f32) / f64::from(THRESHOLD) - 2.33).abs() < 1.0e-6,
            "the third drive opens at a ratio of {}",
            f64::from(escape as f32) / f64::from(THRESHOLD)
        );

        for (name, average, drive) in drives
        {
            for seed in seeds
            {
                let mut limiter = ThermalLimiter { average, gain: seed };
                let mut zero_at = None;

                for n in 0..HELD_SAMPLES
                {
                    limiter.hold(drive(n));

                    let gain = limiter.gain;
                    let law = 1.0 / sqrt(ratio_of(limiter));
                    let over_the_law = f64::from(gain) - law > f64::from(f32::EPSILON);

                    assert!
                    (
                        (0.0..=1.0).contains(&gain),
                        "{name}: a seed of {seed} stood at {gain} at {n}"
                    );

                    if let Some(at) = zero_at
                    {
                        assert_eq!(gain, 0.0, "{name}: a seed of {seed} left zero at {n}, from {at}");
                    }
                    else if gain == 0.0
                    {
                        zero_at = Some(n);
                    }
                    else if n == 0
                    {
                        over_the_law_first += u32::from(over_the_law);
                    }
                    else
                    {
                        assert!
                        (
                            !over_the_law,
                            "{name}: a seed of {seed} stood at {gain} over the law {law} at {n}"
                        );
                    }
                }

                if zero_at.is_some()
                {
                    silenced += 1;
                }
                else
                {
                    returned += 1;
                }
            }
        }

        assert!(returned > 0, "no seed returned under the law, so that branch read nothing");
        assert!(silenced > 0, "no seed fell silent, so that branch read nothing");
        assert!
        (
            over_the_law_first > 0,
            "no seed stood over the law at its first step, so the escape read nothing"
        );
    }

    #[test]
    fn the_input_bound_stands_twice_over_the_loudest_peak_without_the_trim()
    {
        // 2.472951 is the L1 norm of the high cascade, MEASURED off its
        // coefficients by the test of the chain that drives it with the source
        // that attains it.
        let loudest = 2.472_951_f32 * PEAK_RESERVE;

        assert!((loudest - 1.964).abs() < 0.001, "the loudest peak reads {loudest}");
        assert!(LIMITER_INPUT_BOUND >= 2.0 * loudest);
        assert!(LIMITER_INPUT_BOUND < 2.1 * loudest);
    }

    #[test]
    fn no_value_of_any_kind_leaves_the_limiter_off_its_domain_or_the_high_way_past_the_wall()
    {
        // Values that are not numbers, both infinities, values past the format
        // and past the bound, subnormals and the smallest normal, fed round
        // after round, straight into the limiter and through the whole stage.
        let values =
        [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::MAX,
            -f32::MAX,
            f32::MIN_POSITIVE / 2.0,
            -f32::from_bits(1),
            f32::from_bits(1),
            5.0e-40,
            f32::from_bits(LIMITER_INPUT_BOUND.to_bits() + 1),
            -f32::from_bits(LIMITER_INPUT_BOUND.to_bits() + 1),
            f32::MIN_POSITIVE,
        ];
        let wall = 1_336_379_648_i32;
        let mut limiter = ThermalLimiter::at_rest();
        let mut staged = ThermalLimiter::at_rest();

        let mut at_rest = ThermalLimiter::at_rest();

        assert_eq!(at_rest.hold(f32::NAN), 0.0);
        assert_eq!(at_rest, ThermalLimiter::at_rest(), "a value that is not a number moved the state");

        for round in 0..20_000_u32
        {
            for value in values
            {
                let held = limiter.hold(value);
                let word = WayWords::of(WaySamples { low: value, mid: value, high: value }, &mut staged);

                for state in [limiter, staged]
                {
                    assert!(state.average.is_finite() && state.gain.is_finite(), "round {round}");
                    assert!(state.average == 0.0 || state.average >= AVERAGE_FLOOR, "round {round}");
                    assert!(state.average <= LIMITER_INPUT_BOUND * LIMITER_INPUT_BOUND);
                    assert!(state.gain > 0.0 && state.gain <= 1.0, "round {round}");
                }

                assert!(held.is_finite() && held.abs() <= LIMITER_INPUT_BOUND, "{value} held at {held}");
                assert!(word.high().cast_signed().unsigned_abs() <= wall.unsigned_abs(), "{value}");
            }
        }

        // The values at the bound hold the average high, so the limiter acted
        // on this drive rather than standing at unity through it.
        assert!(limiter.gain < 0.5, "the drive left the gain at {}", limiter.gain);
        assert!(staged.gain < 1.0, "the drive left the gain of the stage at {}", staged.gain);
    }

    #[test]
    fn after_a_long_silence_the_average_goes_to_zero_and_the_gain_to_unity()
    {
        // Three seconds at the input bound, then silence. The recurrence alone
        // takes the average down geometrically and, in single precision, enters
        // the subnormal values 27 seconds in and stops on one. The floor takes
        // it to zero first, at the sample the geometric decay crosses the floor.
        // The silence runs past both, so a detector with no floor reaches a
        // subnormal value inside it.
        const LOUD_SAMPLES: u32 = 3 * 44_100;
        const QUIET_SAMPLES: u32 = 32 * 44_100;

        let mut limiter = ThermalLimiter::at_rest();

        for _ in 0..LOUD_SAMPLES
        {
            limiter.hold(LIMITER_INPUT_BOUND);
        }

        let loud = limiter.average;
        let mut zeroed = None;
        let mut unity_from = None;
        let mut under_threshold_at = None;

        for n in 0..QUIET_SAMPLES
        {
            let before = limiter.average;

            limiter.hold(0.0);

            assert!
            (
                limiter.average == 0.0 || limiter.average.is_normal(),
                "the average sits on {} after {n} samples of silence",
                limiter.average
            );

            if zeroed.is_none() && limiter.average == 0.0
            {
                assert!(before >= AVERAGE_FLOOR, "the average reached zero from {before}");
                assert!(before * (1.0 - DETECTOR_COEFFICIENT) < AVERAGE_FLOOR * 1.000_001);
                zeroed = Some(n);
            }

            if under_threshold_at.is_none() && limiter.average <= THRESHOLD
            {
                under_threshold_at = Some(n);
            }

            if limiter.gain == 1.0
            {
                unity_from = unity_from.or(Some(n));
            }
            else
            {
                unity_from = None;
            }
        }

        let (Some(zeroed), Some(unity_from), Some(under)) = (zeroed, unity_from, under_threshold_at)
        else
        {
            panic!("the silence left the average at {} and the gain at {}", limiter.average, limiter.gain);
        };
        let closed = log(f64::from(AVERAGE_FLOOR) / f64::from(loud))
            / log(1.0 - f64::from(DETECTOR_COEFFICIENT));

        assert!
        (
            (f64::from(zeroed) - closed).abs() < closed * 1.0e-3,
            "the floor took the average to zero after {zeroed} samples against {closed}"
        );
        assert_eq!(limiter.average, 0.0);
        assert_eq!(limiter.gain, 1.0);
        assert!(unity_from <= under + 16, "the gain reached unity {} samples late", unity_from - under);
    }

    #[test]
    fn an_average_that_is_not_a_number_takes_the_high_way_to_silence_and_not_to_unity()
    {
        // No input reaches this state, since a value that is not a number is
        // taken as silence before the detector. The reading is what the gain
        // does if the average ever holds one: it goes to zero rather than
        // standing at unity, and the high way answers silence.
        let mut limiter = ThermalLimiter { average: f32::NAN, gain: 0.5 };
        let word = high_word(&mut limiter, 1.0);

        assert_eq!(limiter.gain, 0.0, "the gain stood at {} on an average that is not a number", limiter.gain);
        assert_eq!(word, 0, "the high way left {word} on an average that is not a number");
    }

    #[test]
    fn the_floor_takes_no_average_a_source_of_one_word_leaves()
    {
        // The smallest step a source carries is one word, and the gain of the
        // way brings it to the level below before the limiter reads it. A
        // signal of that step, held, leaves an average orders over the floor,
        // and the average settles on it.
        const HELD_SAMPLES: u32 = 10 * 44_100;

        let step = HIGH_GAIN_OF_FULL_SCALE;
        let mut limiter = ThermalLimiter::at_rest();

        for n in 0..HELD_SAMPLES
        {
            limiter.hold(if n % 2 == 0 { step } else { -step });

            assert!(n < 1_000 || limiter.average > 0.0, "the floor zeroed a held signal at {n}");
        }

        let square = f64::from(step) * f64::from(step);

        assert!((f64::from(limiter.average) - square).abs() < square * 1.0e-3);
        assert!(f64::from(limiter.average) > f64::from(AVERAGE_FLOOR) * 1.0e9);
    }

    #[test]
    fn a_silent_limiter_answers_silence_on_the_high_way_for_good()
    {
        let mut limiter = ThermalLimiter::silent();

        for value in [1.0e9_f32, -1.0e9, f32::MAX, f32::INFINITY, f32::NEG_INFINITY, f32::NAN]
        {
            let words = WayWords::of(WaySamples { low: value, mid: value, high: value }, &mut limiter);

            assert_eq!(words.high(), 0, "a silent limiter let {value} through");
        }

        let words =
            WayWords::of(WaySamples { low: 1.0e9, mid: 1.0e9, high: 1.0e9 }, &mut limiter);

        assert_ne!(words.low(), 0, "the limiter stands on the high way and on no other");
        assert_ne!(words.mid(), 0, "the limiter stands on the high way and on no other");

        for n in 0..(2 * 44_100_u32)
        {
            let held = limiter.hold(if n < 44_100 { LIMITER_INPUT_BOUND } else { 0.0 });

            assert_eq!(held, 0.0, "a silent limiter answered {held} at {n}");
        }

        assert_eq!(limiter.gain, 0.0);
    }

    #[test]
    fn the_ceiling_binds_on_what_the_limiter_leaves()
    {
        // A limiter holding a gain of one half, on the law at four times the
        // threshold. A value past everything leaves at the wall and not at the
        // wall times that gain, so the wall stands after the gain. A value
        // whose product with the gain stands inside the wall leaves at that
        // product, so the gain acts on it and the wall does not.
        let wall = 1_336_379_648_i32;
        let holding = ThermalLimiter { average: 4.0 * THRESHOLD, gain: 0.5 };

        for (value, bound) in [(f32::MAX, wall), (-f32::MAX, -wall)]
        {
            let mut limiter = holding;
            let word = high_word(&mut limiter, value);

            assert!(limiter.gain < 0.51, "the limiter moved off its gain, to {}", limiter.gain);
            assert_eq!(word, bound, "a value past the wall left at {word}");
        }

        let mut limiter = holding;
        let word = high_word(&mut limiter, 1.0);
        let product = to_word(limiter.gain * FULL_SCALE_WORDS).cast_signed();

        assert!(product < wall);
        assert_eq!(word, product, "a sample inside the wall after the gain left at {word}");
    }
}
