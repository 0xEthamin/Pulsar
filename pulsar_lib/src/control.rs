//! The slew limit between a decoded control message and the audio chain.
//!
//! At 115200 baud the link carries up to 1645 volume messages a second, seven
//! bytes of at least ten bits each, and a chain applying each of them as a step
//! would turn the control link into a full scale amplitude modulator aimed at
//! the drivers. No transition this type hands out is abrupt.
//!
//! `ControlState` bounds the SPEED of the gain. A change ramps linearly over
//! its distance times `GAIN_FULL_SWING_MS`, and over no less than
//! `GAIN_RAMP_MS`, so no gain moves faster than full scale in
//! `GAIN_FULL_SWING_MS`. A request replaces the target of the ramp in flight
//! and ramps from the gain the chain holds, so the newest request wins and a
//! reversal starts where the gain stands. The gain is handed out at both ends
//! of a buffer so the caller walks the ramp sample by sample.
//!
//! The speed bound is also what bounds the envelope a sender draws by
//! reversing the setting, at any pace. Over any stretch of `W` ms of sound the
//! gain swings by less than `W + 1` ms over `GAIN_FULL_SWING_MS` when the
//! caller's count reads the sound produced in whole milliseconds. The ramp
//! advances by the difference of two such counts, and that difference exceeds
//! the sound between them by less than 1 ms. `poll` reads the count once per
//! buffer, so a count read up to one buffer late widens the bound by that
//! buffer, to `W` plus one buffer plus 1 ms. The test
//! `a_paced_sender_swings_the_gain_no_wider_than_the_slew_allows` holds both
//! bounds across sender periods, sender phases and buffer periods.
//!
//! A preset change leaves through the same ramp: the gain ramps to silence, the
//! preset moves on a buffer silent at both ends, and the gain rides back up
//! behind it, so the coefficients never move under signal. The filter state is
//! the caller's to clear, see `Applied::preset`.
//!
//! Two clocks run here. The ramp runs on audio produced, which is the caller's
//! elapsed count capped at one buffer. The heartbeat silence measure runs on
//! the raw elapsed count, because a link that stopped is measured in wall time
//! and a capped clock reads it short.
//!
//! Time arrives as a parameter, so nothing here reads a clock.

// A millisecond count converted to a float here is compared against a ramp of
// at most GAIN_FULL_SWING_MS, where it is exact, and a sample index is bounded
// by the length of one buffer. A count past 2 to the 24th loses precision and
// still reads as a ramp that has arrived.
#![allow(clippy::cast_precision_loss)]

use crate::constants::{GAIN_FULL_SWING_MS, GAIN_RAMP_MS};
use crate::protocol::{Preset, ToDsp, Volume};

/// Shortest buffer period `poll` runs on, in milliseconds.
///
/// The declared period caps one step of the ramp. A caller computing it as
/// samples times 1000 over the sample rate reads zero for any buffer under 45
/// samples, and a cap of zero would hold the gain where it stands and never
/// swap a preset. The floor rounds such a buffer up to the resolution of the
/// unit instead.
const MIN_BUFFER_MS: u32 = 1;

const _: () = assert!
(
    MIN_BUFFER_MS > 0,
    "a floor of zero would leave the gain frozen where it stands"
);

/// What the processing chain applies across one buffer.
///
/// The gain is carried at both ends of the buffer and `gain_at` walks between
/// them, which keeps a gain change a ramp rather than one step per buffer. A
/// buffer starts on the gain the previous one ended on, so consecutive buffers
/// meet with no discontinuity.
#[derive(Debug, Clone, Copy, PartialEq)]
#[must_use]
pub struct Applied
{
    gain_start: f32,
    gain_end: f32,
    preset: Preset,
}

impl Applied
{
    /// Returns the linear gain at the first sample of the buffer.
    #[must_use]
    pub fn gain_start(self) -> f32
    {
        self.gain_start
    }

    /// Returns the linear gain one sample past the end of the buffer.
    ///
    /// This is where the next buffer starts, so the two meet exactly.
    #[must_use]
    pub fn gain_end(self) -> f32
    {
        self.gain_end
    }

    /// Returns the linear gain for sample `index` of a buffer of `len` samples.
    ///
    /// The gain moves linearly from `gain_start` to `gain_end` across the
    /// buffer. An index at or past `len`, and a `len` of zero, return
    /// `gain_end`, so no index reads outside the pair.
    #[must_use]
    pub fn gain_at(self, index: usize, len: usize) -> f32
    {
        if index >= len
        {
            return self.gain_end;
        }
        let fraction = index as f32 / len as f32;
        self.gain_start + (self.gain_end - self.gain_start) * fraction
    }

    /// Returns the preset whose coefficients the chain runs.
    ///
    /// The value changes only on a buffer whose gain is zero at both ends, so
    /// no signal enters the sections while their coefficients move.
    ///
    /// The filter state is the caller's. A section still holds the tail of what
    /// it ran before the mute, and a fourth order section at `SUBSONIC_HZ`
    /// rings far past the one silent buffer the swap gets, so new coefficients
    /// over an old state produce a transient. A caller reloading coefficients
    /// here clears the state of every section it reloads, in the same step.
    #[must_use]
    pub fn preset(self) -> Preset
    {
        self.preset
    }

    /// Returns a pair walking from `gain_start` to `gain_end`, on the flat
    /// preset.
    ///
    /// A test of the carry drives a gain of its choosing, where a
    /// `ControlState` would take a ramp of many buffers to reach it.
    #[cfg(test)]
    pub(crate) const fn between_for_test(gain_start: f32, gain_end: f32) -> Self
    {
        Self
        {
            gain_start,
            gain_end,
            preset: Preset::Flat,
        }
    }
}

/// A linear move between two gains, at a speed bounded by
/// `GAIN_FULL_SWING_MS`.
#[derive(Debug, Clone, Copy)]
struct Ramp
{
    from: f32,
    to: f32,
    /// Milliseconds of audio produced since the ramp started, saturating.
    elapsed_ms: u32,
    /// Length of the ramp: the distance times `GAIN_FULL_SWING_MS`, and no
    /// less than `GAIN_RAMP_MS`.
    duration_ms: f32,
}

impl Ramp
{
    /// Builds a ramp from `from` to `to`.
    fn new(from: f32, to: f32) -> Self
    {
        let distance = (to - from).abs();
        let duration_ms = (distance * GAIN_FULL_SWING_MS as f32).max(GAIN_RAMP_MS as f32);
        Self
        {
            from,
            to,
            elapsed_ms: 0,
            duration_ms,
        }
    }

    /// Returns the gain the ramp holds.
    ///
    /// The value lands on `to` exactly once the elapsed time reaches the
    /// duration, and moves monotonically from `from` before that.
    fn value(self) -> f32
    {
        let elapsed = self.elapsed_ms as f32;
        if elapsed >= self.duration_ms
        {
            return self.to;
        }
        self.from + (self.to - self.from) * (elapsed / self.duration_ms)
    }
}

/// The only path from a decoded `ToDsp` message to a value the chain applies.
///
/// A request inside a ramp in flight replaces its target rather than queueing
/// behind it, so a burst lands on the newest setting.
///
/// `poll` advances the ramp by the smaller of the caller's elapsed count and
/// the period of the buffer being filled, since one poll produces one buffer of
/// audio and no more. A caller count that stalls, jumps forward or runs
/// backwards therefore cannot move the gain faster than the slew limit.
///
/// The heartbeat silence measure runs on the caller's raw elapsed count
/// instead, saturating, because a capped count reads a silence short.
///
/// The state is a machine, so it is neither `Copy` nor `Clone`. Two copies
/// would each hold their own ramp and disagree about what the chain applies.
#[derive(Debug)]
pub struct ControlState
{
    requested_volume: Volume,
    committed_volume: Volume,
    requested_preset: Preset,
    active_preset: Preset,
    ramp: Ramp,
    muting_for_swap: bool,
    /// Caller count the last `poll` read, for the elapsed difference.
    last_seen_ms: u32,
    /// Saturating milliseconds of wall time since the last heartbeat, or since
    /// the build of the state while none has arrived.
    silence_ms: u32,
}

impl ControlState
{
    /// Builds the state silent, on the protective crossover alone.
    ///
    /// `now_ms` is the caller's millisecond count at the build.
    #[must_use]
    pub fn new(now_ms: u32) -> Self
    {
        Self
        {
            requested_volume: Volume::MUTED,
            committed_volume: Volume::MUTED,
            requested_preset: Preset::Flat,
            active_preset: Preset::Flat,
            ramp: Ramp::new(0.0, 0.0),
            muting_for_swap: false,
            last_seen_ms: now_ms,
            silence_ms: 0,
        }
    }

    /// Records what the sender asked for.
    ///
    /// A setting does not reach the audio chain here. The next `poll` takes
    /// it, and a later request before that poll replaces this one.
    ///
    /// A heartbeat restarts the silence measure and touches nothing else.
    pub fn request(&mut self, message: ToDsp)
    {
        match message
        {
            ToDsp::SetVolume(volume) => self.requested_volume = volume,
            ToDsp::SelectPreset(preset) => self.requested_preset = preset,
            ToDsp::Heartbeat => self.silence_ms = 0,
        }
    }

    /// Advances the ramp and returns what the chain applies across one buffer.
    ///
    /// Call this once per buffer. `now_ms` is the caller's millisecond count
    /// and `buffer_ms` the period of the buffer about to be filled.
    ///
    /// The returned pair is the gain at both ends of that buffer. Applying
    /// either end to the whole buffer turns the ramp back into a staircase, so
    /// walk it with `Applied::gain_at`. A request this poll takes starts moving
    /// the gain on the next buffer.
    ///
    /// `buffer_ms` is held at `MIN_BUFFER_MS` or above, so a declared period of
    /// zero cannot freeze the ramp. It has no ceiling, and a caller declaring
    /// a period longer than the buffer it fills moves the gain faster than the
    /// slew limit across that buffer. Declaring the period of the buffer being
    /// filled is the caller's half of the contract.
    pub fn poll(&mut self, now_ms: u32, buffer_ms: u32) -> Applied
    {
        let gain_start = self.ramp.value();
        let step_ms = self.advance_clock(now_ms, buffer_ms);
        self.ramp.elapsed_ms = self.ramp.elapsed_ms.saturating_add(step_ms);
        let gain_end = self.ramp.value();
        self.commit(gain_start, gain_end);
        Applied
        {
            gain_start,
            gain_end,
            preset: self.active_preset,
        }
    }

    /// Returns the setting the chain holds or is ramping towards.
    ///
    /// This is what the processing board reports back, since the newest request
    /// may not have reached the chain.
    #[must_use]
    pub fn applied_volume(&self) -> Volume
    {
        self.committed_volume
    }

    /// Returns how long the link has been silent, in milliseconds.
    ///
    /// `now_ms` is the caller's millisecond count, the same one `poll` reads.
    /// The measure runs on wall time, so it keeps rising once `poll` stops,
    /// which is one of the failures it reports. It saturates rather than
    /// wrapping, so a long silence never reads as a fresh beat.
    ///
    /// A peer that has never sent a beat reads as silence growing since the
    /// build, and a count that runs backwards inflates the silence. Both err
    /// towards reporting the link dead, which costs a ramp down instead of a
    /// loud cabinet with no control.
    ///
    /// Nothing here acts on the value. The caller owns the duration worth
    /// acting on, and the ramp down and the alarm behind it.
    #[must_use]
    pub fn heartbeat_silence_ms(&self, now_ms: u32) -> u32
    {
        let since_poll = now_ms.wrapping_sub(self.last_seen_ms);
        self.silence_ms.saturating_add(since_poll)
    }

    /// Returns the audio one poll produces, in milliseconds, and advances the
    /// silence measure by the wall time that passed.
    ///
    /// The step is the caller's elapsed count capped at the declared period,
    /// held at `MIN_BUFFER_MS` or above. A count running backwards produces a
    /// wrapping distance far above any buffer period, so the cap catches it.
    ///
    /// The silence measure takes the raw elapsed count instead. Under the
    /// capped step it would count audio produced: lateness truncated, earliness
    /// never made up, and a poll that stopped freezing the measure.
    fn advance_clock(&mut self, now_ms: u32, buffer_ms: u32) -> u32
    {
        let elapsed = now_ms.wrapping_sub(self.last_seen_ms);
        let period_ms = buffer_ms.max(MIN_BUFFER_MS);
        self.last_seen_ms = now_ms;
        self.silence_ms = self.silence_ms.saturating_add(elapsed);
        elapsed.min(period_ms)
    }

    /// Takes the newest request of each kind into the ramp.
    ///
    /// `gain_start` and `gain_end` are the ends of the buffer this poll hands
    /// out. A pending preset moves first, because its coefficients may only
    /// change under silence. The volume rides back up with it.
    ///
    /// The swap waits for a buffer that STARTS on silence. A buffer starts on
    /// the gain the previous one ended on, so a swap taken on the buffer the
    /// mute ramp arrives in would move the coefficients across a first sample
    /// that is still audible. The ramp up it starts moves from the next buffer,
    /// so the swap buffer ends on silence too.
    fn commit(&mut self, gain_start: f32, gain_end: f32)
    {
        if self.muting_for_swap
        {
            if gain_start > 0.0
            {
                return;
            }
            self.active_preset = self.requested_preset;
            self.committed_volume = self.requested_volume;
            self.muting_for_swap = false;
            self.ramp = Ramp::new(gain_end, self.committed_volume.linear_gain());
            return;
        }

        if self.requested_preset != self.active_preset
        {
            self.muting_for_swap = true;
            self.ramp = Ramp::new(gain_end, 0.0);
            return;
        }

        if self.requested_volume != self.committed_volume
        {
            self.committed_volume = self.requested_volume;
            self.ramp = Ramp::new(gain_end, self.committed_volume.linear_gain());
        }
    }
}

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, which is the one place the
    // no-panic rule does not hold.
    #![allow(clippy::panic)]

    use std::vec::Vec;

    use super::*;
    use crate::constants::{GAIN_STEP_RAMP_MAX_MS, SAMPLE_RATE_HZ};
    use crate::protocol::{COARSE_DEFAULT, COARSE_MAX, FINE_MAX};

    /// Samples per buffer these tests drive the ramp with.
    ///
    /// The audio chain runs 256 samples a buffer at 44.1 kHz.
    const FIXTURE_BUFFER_SAMPLES: u32 = 256;

    /// Period of the fixture buffer, in milliseconds, rounded up.
    ///
    /// This is the period a caller passes, and what the ramp has to survive. A
    /// test written at one millisecond hides a staircase four times coarser
    /// than it measures.
    const FIXTURE_BUFFER_MS: u32 = (FIXTURE_BUFFER_SAMPLES * 1_000).div_ceil(SAMPLE_RATE_HZ);

    /// Tolerance covering single precision rounding on a gain comparison.
    const EPSILON: f32 = 1e-6;

    /// Buffers of the fixture that carry a full swing and a margin behind it.
    const FULL_SWING_BUFFERS: u32 = GAIN_FULL_SWING_MS.div_ceil(FIXTURE_BUFFER_MS) + 2;

    /// Fastest the gain may move, in full scale per millisecond.
    const SLEW_PER_MS: f32 = 1.0 / GAIN_FULL_SWING_MS as f32;

    /// Builds a volume pair, failing the test rather than returning an error.
    fn volume(coarse: u8, fine: u8) -> Volume
    {
        match Volume::new(coarse, fine)
        {
            Ok(volume) => volume,
            Err(error) => panic!("volume rejected: {error:?}"),
        }
    }

    /// Both controls at their maximum, a gain of exactly 1.
    fn full() -> Volume
    {
        volume(COARSE_MAX, FINE_MAX)
    }

    /// Returns the widest gain move `buffer_ms` of audio allows.
    fn rate_bound(buffer_ms: u32) -> f32
    {
        SLEW_PER_MS * buffer_ms as f32 + EPSILON
    }

    /// A caller filling buffers of a fixed number of samples.
    ///
    /// Its count is the whole milliseconds of audio produced before each poll,
    /// as a millisecond tick read once per buffer gives, so the count steps
    /// unevenly under the declared period whenever a buffer is not a whole
    /// number of milliseconds.
    struct Caller
    {
        samples: u32,
        polls: u64,
    }

    impl Caller
    {
        fn new(samples: u32) -> Self
        {
            Self
            {
                samples,
                polls: 0,
            }
        }

        /// Returns the period the caller declares, rounded up.
        fn buffer_ms(&self) -> u32
        {
            self.samples.saturating_mul(1_000).div_ceil(SAMPLE_RATE_HZ)
        }

        /// Returns the count the next poll reads.
        fn now_ms(&self) -> u32
        {
            let produced = self
                .polls
                .saturating_mul(u64::from(self.samples))
                .saturating_mul(1_000)
                .div_euclid(u64::from(SAMPLE_RATE_HZ));
            match u32::try_from(produced)
            {
                Ok(now_ms) => now_ms,
                Err(error) => panic!("the caller count overflowed: {error:?}"),
            }
        }

        /// Polls `state` once and returns the count it read with the result.
        fn poll(&mut self, state: &mut ControlState) -> (u32, Applied)
        {
            let now_ms = self.now_ms();
            self.polls = self.polls.saturating_add(1);
            (now_ms, state.poll(now_ms, self.buffer_ms()))
        }
    }

    /// Polls `state` for `buffers` buffers, handing each result to `observe`.
    ///
    /// Returns the caller count the next poll would use.
    fn drive<F>
    (
        state: &mut ControlState,
        start_ms: u32,
        buffer_ms: u32,
        buffers: u32,
        mut observe: F,
    ) -> u32
    where
        F: FnMut(u32, Applied),
    {
        let mut now_ms = start_ms;
        for index in 0..buffers
        {
            observe(index, state.poll(now_ms, buffer_ms));
            now_ms = now_ms.wrapping_add(buffer_ms);
        }
        now_ms
    }

    /// Requests `to` and polls until a buffer ends on its gain.
    ///
    /// Returns the caller count from the poll that takes the request to the
    /// poll whose buffer ends on the new gain, which is the audio the ramp
    /// took as the chain hands it out.
    fn ramp_ms(state: &mut ControlState, caller: &mut Caller, to: Volume) -> u32
    {
        state.request(ToDsp::SetVolume(to));
        let (taken_ms, _) = caller.poll(state);
        let target = to.linear_gain().to_bits();
        loop
        {
            let (now_ms, applied) = caller.poll(state);
            let ramp_ms = now_ms.saturating_sub(taken_ms);
            if applied.gain_end().to_bits() == target
            {
                return ramp_ms;
            }
            assert!(ramp_ms <= 2 * GAIN_FULL_SWING_MS, "the ramp to {to:?} never arrived");
        }
    }

    #[test]
    fn a_new_state_is_silent_on_the_protective_crossover()
    {
        let mut state = ControlState::new(0);
        let applied = state.poll(0, FIXTURE_BUFFER_MS);
        assert!(applied.gain_start().abs() < f32::EPSILON);
        assert!(applied.gain_end().abs() < f32::EPSILON);
        assert_eq!(applied.preset(), Preset::Flat);
        assert_eq!(state.applied_volume(), Volume::MUTED);
        assert_eq!(state.heartbeat_silence_ms(0), 0);
    }

    #[test]
    fn the_applied_pair_walks_the_buffer_from_end_to_end()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(full()));
        let _ = state.poll(0, FIXTURE_BUFFER_MS);
        let applied = state.poll(FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS);
        assert!(applied.gain_end() > applied.gain_start());

        let len = FIXTURE_BUFFER_SAMPLES as usize;
        assert!((applied.gain_at(0, len) - applied.gain_start()).abs() < f32::EPSILON);
        assert!((applied.gain_at(len, len) - applied.gain_end()).abs() < f32::EPSILON);
        assert!((applied.gain_at(0, 0) - applied.gain_end()).abs() < f32::EPSILON);

        // Past the end the pair is the bound, so no index extrapolates beyond
        // gain_end and out of the range the two ends hold.
        assert!((applied.gain_at(len + 1, len) - applied.gain_end()).abs() < f32::EPSILON);
        assert!((applied.gain_at(usize::MAX, len) - applied.gain_end()).abs() < f32::EPSILON);

        let mut previous = applied.gain_start();
        for index in 0..len
        {
            let gain = applied.gain_at(index, len);
            assert!(gain >= previous, "the gain fell at sample {index}");
            assert!(gain <= applied.gain_end());
            previous = gain;
        }
    }

    #[test]
    fn the_gain_never_moves_faster_than_the_slew_at_any_buffer_period()
    {
        // Requests land mid ramp in both directions and as a single 3 dB step,
        // at every buffer length from one sample to 60 ms. The bound holds from
        // each sample to the next, across the seam between buffers included.
        let schedule =
        [
            (0_u32, full()),
            (100, Volume::MUTED),
            (160, full()),
            (400, volume(COARSE_MAX - 1, FINE_MAX)),
            (500, full()),
        ];
        for samples in (1..=64_u32).chain((65..=2_646).step_by(13))
        {
            let mut caller = Caller::new(samples);
            let len = samples as usize;
            let sample_bound = SLEW_PER_MS * caller.buffer_ms() as f32 / len as f32 + EPSILON;
            let mut state = ControlState::new(0);
            let mut previous_sample = 0.0_f32;
            let mut previous_end = 0.0_f32;

            while caller.now_ms() <= 1_000
            {
                for (at_ms, setting) in schedule
                {
                    if at_ms <= caller.now_ms()
                    {
                        state.request(ToDsp::SetVolume(setting));
                    }
                }
                let (now_ms, applied) = caller.poll(&mut state);
                assert!
                (
                    (applied.gain_start() - previous_end).abs() < EPSILON,
                    "buffers do not meet at {now_ms} ms with {samples} samples"
                );
                for sample in 0..len
                {
                    let gain = applied.gain_at(sample, len);
                    assert!
                    (
                        (gain - previous_sample).abs() <= sample_bound,
                        "the gain outran the slew at sample {sample} at {now_ms} ms with {samples} samples"
                    );
                    previous_sample = gain;
                }
                previous_end = applied.gain_end();
            }
            assert!((previous_end - 1.0).abs() < EPSILON, "the ramp never arrived with {samples} samples");
        }
    }

    /// Returns every single step of the volume as a pair of settings, in the
    /// order the ramp test walks them: every step of the coarse control at
    /// every fine value, then every fine move of one to nine values at every
    /// coarse step.
    fn single_steps() -> Vec<(Volume, Volume)>
    {
        let coarse_steps = (1..=FINE_MAX).flat_map(|fine|
        {
            (1..=COARSE_MAX).map(move |coarse| (volume(coarse, fine), volume(coarse.saturating_sub(1), fine)))
        });
        let fine_moves = (1..=COARSE_MAX).flat_map(|coarse|
        {
            (1..=FINE_MAX).flat_map(move |fine|
            {
                (1..=9_u8.min(fine)).map(move |moved| (volume(coarse, fine), volume(coarse, fine.saturating_sub(moved))))
            })
        });
        coarse_steps.chain(fine_moves).collect()
    }

    /// Asserts that `from` to `to` and back each ramp over `GAIN_RAMP_MS` to
    /// `ceiling_ms`, starting from a gain settled on `from`.
    fn assert_step_ramps_within
    (
        state: &mut ControlState,
        caller: &mut Caller,
        from: Volume,
        to: Volume,
        ceiling_ms: u32
    )
    {
        let _ = ramp_ms(state, caller, from);
        for (start, end) in [(from, to), (to, from)]
        {
            let took_ms = ramp_ms(state, caller, end);
            assert!
            (
                (GAIN_RAMP_MS..=ceiling_ms).contains(&took_ms),
                "{start:?} to {end:?} took {took_ms} ms with {} samples",
                caller.samples
            );
        }
    }

    #[test]
    fn every_single_step_ramps_between_the_shortest_and_the_longest_ramp()
    {
        // Every step of the coarse control at every fine value, and every fine
        // move of one to nine values at every coarse step, both ways. Nine
        // values is 1.7 dB, a phone with fifteen volume steps. The chain hands
        // the ramp out a buffer at a time, so its end lands on the first
        // buffer boundary past the ramp and the ceiling grows by one buffer
        // less a millisecond.
        let steps = single_steps();
        assert_eq!(steps.len(), 16_042, "the ramp sweep no longer covers 16042 single steps");
        for samples in [44_u32, 220, FIXTURE_BUFFER_SAMPLES, 441]
        {
            let mut caller = Caller::new(samples);
            let ceiling_ms = GAIN_STEP_RAMP_MAX_MS + caller.buffer_ms() - 1;
            let mut state = ControlState::new(0);

            for &(from, to) in &steps
            {
                assert_step_ramps_within(&mut state, &mut caller, from, to, ceiling_ms);
            }
        }
    }

    #[test]
    fn the_widest_step_and_a_full_swing_take_their_derived_durations()
    {
        // At one millisecond a buffer the ramp reads to the millisecond. The 3
        // dB step at the top spans 0.29205 of full scale, 49.94 ms at the slew
        // limit. A full swing takes 171 ms, the longest that keeps that step
        // under 50 ms. A 1.5 dB phone step, eight fine values, spans 0.15975
        // and takes 27.32 ms. A single fine value ramps over the shortest
        // ramp, 20 ms. The durations are literals, so a constant that drifts
        // fails here.
        let mut caller = Caller::new(44);
        let mut state = ControlState::new(0);
        let top_less_3_db = volume(COARSE_MAX - 1, FINE_MAX);
        let phone_step_down = volume(COARSE_MAX, FINE_MAX - 8);

        assert_eq!(ramp_ms(&mut state, &mut caller, full()), 171);
        assert_eq!(ramp_ms(&mut state, &mut caller, top_less_3_db), 50);
        assert_eq!(ramp_ms(&mut state, &mut caller, full()), 50);
        assert_eq!(ramp_ms(&mut state, &mut caller, phone_step_down), 28);
        assert_eq!(ramp_ms(&mut state, &mut caller, full()), 28);
        assert_eq!(ramp_ms(&mut state, &mut caller, volume(COARSE_MAX, FINE_MAX - 1)), 20);
        assert_eq!(ramp_ms(&mut state, &mut caller, full()), 20);
        assert_eq!(ramp_ms(&mut state, &mut caller, Volume::MUTED), 171);
    }

    #[test]
    fn a_ramp_lands_on_its_target_when_its_elapsed_time_reaches_the_duration()
    {
        // From coarse 1 fine 1 to coarse 1 fine 51 the gains sit more than a
        // factor two apart, and `from + (to - from)` misses `to` in single
        // precision. The ramp is the shortest, 20 ms, and the buffer ending on
        // those 20 ms ends on the target to the bit.
        let mut caller = Caller::new(44);
        let mut state = ControlState::new(0);
        let low = volume(1, 1);
        let high = volume(1, 51);
        let from = low.linear_gain();
        let to = high.linear_gain();
        assert_ne!((from + (to - from)).to_bits(), to.to_bits(), "the pair no longer rounds away");

        let _ = ramp_ms(&mut state, &mut caller, low);
        assert_eq!(ramp_ms(&mut state, &mut caller, high), GAIN_RAMP_MS);
    }

    #[test]
    fn a_gap_in_the_caller_count_never_arrives_as_a_step()
    {
        // A late interrupt or a stalled loop: poll at zero, then at five
        // seconds. The ramp moves by one buffer, which is all the audio the
        // chain produced.
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(full()));

        let first = state.poll(0, FIXTURE_BUFFER_MS);
        let second = state.poll(5_000, FIXTURE_BUFFER_MS);
        assert!((second.gain_start() - first.gain_end()).abs() < EPSILON);
        assert!
        (
            second.gain_end() - second.gain_start() <= rate_bound(FIXTURE_BUFFER_MS),
            "the gap arrived as a step of {}",
            second.gain_end() - second.gain_start()
        );
    }

    #[test]
    fn a_count_that_runs_backwards_never_steps_the_gain()
    {
        // One millisecond back reads as four billion forward under a bare
        // wrapping difference. So does a jump across the wrap.
        for back_ms in [1_u32, 100, 100_000, u32::MAX / 2]
        {
            let mut state = ControlState::new(1_000_000);
            state.request(ToDsp::SetVolume(full()));

            let bound = rate_bound(FIXTURE_BUFFER_MS);
            let mut now_ms = 1_000_000_u32;
            let mut previous_end = 0.0_f32;

            for step in 0..128_u32
            {
                let applied = state.poll(now_ms, FIXTURE_BUFFER_MS);
                assert!
                (
                    (applied.gain_start() - previous_end).abs() < EPSILON,
                    "buffers do not meet at step {step} going back {back_ms}"
                );
                assert!
                (
                    (applied.gain_end() - applied.gain_start()).abs() <= bound,
                    "the gain stepped at {step} going back {back_ms}"
                );
                previous_end = applied.gain_end();
                now_ms = if step % 2 == 0
                {
                    now_ms.wrapping_sub(back_ms)
                }
                else
                {
                    now_ms.wrapping_add(FIXTURE_BUFFER_MS)
                };
            }
        }
    }

    #[test]
    fn a_burst_lands_on_the_newest_request_from_the_gain_in_flight()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(full()));
        let now_ms = drive(&mut state, 0, FIXTURE_BUFFER_MS, 8, |_, _| ());
        let before = state.poll(now_ms, FIXTURE_BUFFER_MS);
        assert!(before.gain_end() > 0.0 && before.gain_end() < 1.0, "the ramp is not in flight");

        // A burst before one poll: only the last request reaches the ramp, and
        // the ramp turns where the gain stands.
        let newest = volume(COARSE_DEFAULT, 0x40);
        state.request(ToDsp::SetVolume(Volume::MUTED));
        state.request(ToDsp::SetVolume(full()));
        state.request(ToDsp::SetVolume(newest));

        let bound = rate_bound(FIXTURE_BUFFER_MS);
        let mut previous_end = before.gain_end();
        let _ = drive(&mut state, now_ms + FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS, 40, |index, applied|
        {
            assert!((applied.gain_start() - previous_end).abs() < EPSILON, "a step at buffer {index}");
            assert!((applied.gain_end() - applied.gain_start()).abs() <= bound);
            previous_end = applied.gain_end();
        });
        assert_eq!(state.applied_volume(), newest);
        assert_eq!(previous_end.to_bits(), newest.linear_gain().to_bits());
    }

    /// Returns the widest swing of the gain over any stretch of `width` samples.
    ///
    /// `ends` holds the gain at every buffer end, `samples` apart, and the gain
    /// runs linearly between two ends. On linear pieces the extremes of a
    /// stretch sit on its two edges or on a buffer end inside it, and the swing
    /// is convex between two positions where an edge crosses a buffer end, so
    /// the widest stretch starts or finishes on a buffer end.
    #[expect
    (
        clippy::arithmetic_side_effects,
        reason = "the positions count samples of a trace held in memory, and a buffer holds at \
                  least one sample"
    )]
    fn widest_swing(ends: &[f32], samples: usize, width: usize) -> f32
    {
        let span = (ends.len() - 1) * samples;
        let gain_at = |at: usize| -> f32
        {
            let index = at / samples;
            let fraction = (at % samples) as f32 / samples as f32;
            match (ends.get(index), ends.get(index + 1))
            {
                (Some(&low), Some(&high)) => low + (high - low) * fraction,
                (Some(&last), None) => last,
                _ => panic!("sample {at} lies past the trace"),
            }
        };

        let mut widest = 0.0_f32;
        for index in 0..ends.len()
        {
            let edge = index * samples;
            for start in [Some(edge), edge.checked_sub(width)].into_iter().flatten()
            {
                let finish = start + width;
                if finish > span
                {
                    continue;
                }
                let mut lowest = gain_at(start).min(gain_at(finish));
                let mut highest = gain_at(start).max(gain_at(finish));
                for &gain in ends.iter().take(finish.div_ceil(samples)).skip(start / samples + 1)
                {
                    lowest = lowest.min(gain);
                    highest = highest.max(gain);
                }
                widest = widest.max(highest - lowest);
            }
        }
        widest
    }

    /// Returns the half periods of the paced sender, in microseconds: from
    /// 0.3 ms, faster than any buffer, each 15 percent and 131 us past the one
    /// before, up to the largest value not above 120 ms.
    fn sender_half_periods_us() -> Vec<u64>
    {
        core::iter::successors(Some(300_u64), |&half_us| Some(half_us.saturating_mul(23).div_euclid(20).saturating_add(131)))
            .take_while(|&half_us| half_us <= 120_000)
            .collect()
    }

    /// Returns what the sender requests at `sender_us` on its own clock:
    /// full scale in the first half of each period, silence in the second.
    fn sender_level(sender_us: u64, half_us: u64) -> Volume
    {
        if sender_us.div_euclid(half_us).is_multiple_of(2)
        {
            full()
        }
        else
        {
            Volume::MUTED
        }
    }

    /// Returns how many samples past `position` the count of poll `polls`
    /// reads, a lateness below one buffer changing on every poll when `late`
    /// holds and none otherwise.
    fn caller_lag(polls: u64, samples: u32, late: bool) -> u64
    {
        if late
        {
            (polls.wrapping_mul(2_654_435_761) >> 7).rem_euclid(u64::from(samples))
        }
        else
        {
            0
        }
    }

    /// Returns the gain at every buffer end over one second of a sender
    /// reversing every `half_us` us from `phase_us`, polled every `samples`
    /// samples at a count `late` or on time.
    fn paced_sender_ends(samples: u32, half_us: u64, phase_us: u64, late: bool) -> Vec<f32>
    {
        let rate = u64::from(SAMPLE_RATE_HZ);
        let declared_ms = samples.saturating_mul(1_000).div_ceil(SAMPLE_RATE_HZ);
        let mut state = ControlState::new(0);
        let mut ends: Vec<f32> = Vec::new();
        let mut polls = 0_u64;
        while polls.saturating_mul(u64::from(samples)) <= rate
        {
            let position = polls.saturating_mul(u64::from(samples));
            let lag = caller_lag(polls, samples, late);
            let sender_us = position.saturating_mul(1_000_000).div_euclid(rate).saturating_add(phase_us);
            state.request(ToDsp::SetVolume(sender_level(sender_us, half_us)));
            let now_ms = match u32::try_from(position.saturating_add(lag).saturating_mul(1_000).div_euclid(rate))
            {
                Ok(now_ms) => now_ms,
                Err(error) => panic!("the caller count overflowed: {error:?}"),
            };
            let applied = state.poll(now_ms, declared_ms);
            if polls == 0
            {
                ends.push(applied.gain_start());
            }
            ends.push(applied.gain_end());
            polls = polls.saturating_add(1);
        }
        ends
    }

    #[test]
    fn a_paced_sender_swings_the_gain_no_wider_than_the_slew_allows()
    {
        // A sender reversing between silence and full scale on its own clock,
        // off the millisecond grid, at half periods from 0.3 ms, faster than
        // any buffer, to 120 ms, and at three phases. Fixed stretches of sound
        // slide over the gain the chain hands out, whatever the sender period.
        //
        // A count reading the sound produced in whole milliseconds bounds the
        // swing over `W` ms by `W + 1` ms at the slew limit. A count read up to
        // one buffer late, at a lateness changing on every poll, widens it by
        // that buffer.
        let half_periods_us = sender_half_periods_us();
        assert_eq!(half_periods_us.len(), 34, "the sender sweep no longer covers 34 half periods");
        assert_eq!(half_periods_us.first(), Some(&300), "the sender sweep no longer starts at 300 us");
        assert_eq!(half_periods_us.last(), Some(&116_981), "the sender sweep no longer ends at 116981 us");
        let cases = [128_u32, 240, 250, 255, FIXTURE_BUFFER_SAMPLES, 257, 265, 272, 384, 512]
            .into_iter()
            .flat_map(|samples| half_periods_us.iter().map(move |&half_us| (samples, half_us)))
            .flat_map(|(samples, half_us)| (0..3_u64).map(move |phase| (samples, half_us, phase)))
            .flat_map(|(samples, half_us, phase)| [false, true].map(|late| (samples, half_us, phase, late)));
        for (samples, half_us, phase, late) in cases
        {
            let phase_us = phase * half_us / 3 + 17;
            let buffer_ms = samples as f32 * 1_000.0 / SAMPLE_RATE_HZ as f32;
            let widened_ms = if late { buffer_ms } else { 0.0 };
            let ends = paced_sender_ends(samples, half_us, phase_us, late);

            for window_ms in [10_u32, 20, 40, 100]
            {
                let width = (window_ms * SAMPLE_RATE_HZ / 1_000) as usize;
                let bound = SLEW_PER_MS * (window_ms as f32 + widened_ms + 1.0) + EPSILON;
                let swing = widest_swing(&ends, samples as usize, width);
                assert!
                (
                    swing <= bound,
                    "a sender at {half_us} us, phase {phase_us} us, swung the gain by \
                     {swing} over {window_ms} ms against {bound}, with {samples} \
                     samples, late {late}"
                );
            }
        }
    }

    #[test]
    fn spaced_changes_each_arrive_on_their_target()
    {
        let mut state = ControlState::new(0);
        let mut now_ms = 0_u32;
        for coarse in [1_u8, 4, COARSE_DEFAULT, 9, COARSE_MAX]
        {
            let expected = volume(coarse, FINE_MAX);
            state.request(ToDsp::SetVolume(expected));
            let _ = state.poll(now_ms, FIXTURE_BUFFER_MS);
            assert_eq!(state.applied_volume(), expected);

            let buffers = GAIN_FULL_SWING_MS.div_ceil(FIXTURE_BUFFER_MS);
            now_ms = drive(&mut state, now_ms + FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS, buffers, |_, _| ());
            let applied = state.poll(now_ms, FIXTURE_BUFFER_MS);
            now_ms += FIXTURE_BUFFER_MS;
            assert_eq!(applied.gain_end().to_bits(), expected.linear_gain().to_bits());
        }
    }

    #[test]
    fn the_ramp_is_monotonic_between_its_endpoints()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(full()));

        let mut previous = -1.0_f32;
        let _ = drive(&mut state, 0, FIXTURE_BUFFER_MS, FULL_SWING_BUFFERS, |index, applied|
        {
            assert!(applied.gain_start() >= previous, "the gain fell at buffer {index}");
            assert!(applied.gain_end() >= applied.gain_start(), "the gain fell inside {index}");
            previous = applied.gain_end();
        });
        assert!((previous - 1.0).abs() < EPSILON);
    }

    #[test]
    fn a_ramp_down_is_monotonic_too()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(full()));
        let now_ms = drive(&mut state, 0, FIXTURE_BUFFER_MS, FULL_SWING_BUFFERS, |_, _| ());

        state.request(ToDsp::SetVolume(Volume::MUTED));
        let mut previous = 2.0_f32;
        let _ = drive(&mut state, now_ms, FIXTURE_BUFFER_MS, FULL_SWING_BUFFERS, |index, applied|
        {
            assert!(applied.gain_start() <= previous, "the gain rose at buffer {index}");
            assert!(applied.gain_end() <= applied.gain_start(), "the gain rose inside {index}");
            previous = applied.gain_end();
        });
        assert!(previous.abs() < EPSILON);
    }

    /// Settles `state` on `start`, then, when `in_flight` holds, requests a
    /// volume away from it and polls two buffers into that ramp.
    ///
    /// Returns the volume the gain settles on and the gain the last buffer
    /// ended on.
    fn settle_before_preset
    (
        state: &mut ControlState,
        caller: &mut Caller,
        start: Volume,
        in_flight: bool
    ) -> (Volume, f32)
    {
        let _ = ramp_ms(state, caller, start);
        let mut settled = start;
        let mut previous_end = start.linear_gain();
        if in_flight
        {
            settled = if start == full()
            {
                volume(COARSE_DEFAULT, FINE_MAX)
            }
            else
            {
                full()
            };
            state.request(ToDsp::SetVolume(settled));
            for _ in 0..2
            {
                let (_, applied) = caller.poll(state);
                previous_end = applied.gain_end();
            }
        }
        (settled, previous_end)
    }

    #[test]
    fn a_preset_change_is_never_applied_as_a_step()
    {
        // Each starting gain and buffer period leaves a different last value
        // above silence on the mute ramp, some of them under a thousandth. The
        // coefficients move only on a buffer whose two ends are zero to the
        // bit. The preset lands on a settled gain, and on a volume ramp in
        // flight two buffers after its request.
        let mut starts: Vec<Volume> = (1..=COARSE_MAX).map(|coarse| volume(coarse, FINE_MAX)).collect();
        starts.extend
        (
            [1_u8, COARSE_DEFAULT, COARSE_MAX]
                .into_iter()
                .flat_map(|coarse| [1_u8, 2, 8, 0x20, 0x40, 0x60].map(|fine| volume(coarse, fine)))
        );

        let cases = [44_u32, 220, FIXTURE_BUFFER_SAMPLES, 441]
            .into_iter()
            .flat_map(|samples| starts.iter().map(move |&start| (samples, start)))
            .flat_map(|(samples, start)| [false, true].map(|in_flight| (samples, start, in_flight)));
        for (samples, start, in_flight) in cases
        {
            let mut caller = Caller::new(samples);
            let mut state = ControlState::new(0);
            let (settled, mut previous_end) = settle_before_preset(&mut state, &mut caller, start, in_flight);

            state.request(ToDsp::SelectPreset(Preset::Garden));
            let bound = rate_bound(caller.buffer_ms());
            let target = settled.linear_gain();
            let mut swapped = false;

            for _ in 0..3 * GAIN_FULL_SWING_MS
            {
                let (now_ms, applied) = caller.poll(&mut state);
                assert!((applied.gain_start() - previous_end).abs() < EPSILON);
                assert!
                (
                    (applied.gain_end() - applied.gain_start()).abs() <= bound,
                    "the gain jumped at {now_ms} ms from {start:?} with {samples} samples"
                );
                if applied.preset() == Preset::Garden && !swapped
                {
                    // The coefficients may only move while nothing comes out.
                    assert_eq!
                    (
                        (applied.gain_start().to_bits(), applied.gain_end().to_bits()),
                        (0, 0),
                        "coefficients moved under signal from {start:?} with {samples} samples"
                    );
                    swapped = true;
                }
                previous_end = applied.gain_end();
            }

            assert!(swapped, "the preset never reached the chain from {start:?}");
            assert_eq!
            (
                previous_end.to_bits(),
                target.to_bits(),
                "the gain never came back from {start:?} with {samples} samples"
            );
        }
    }

    #[test]
    fn a_ramp_that_arrived_long_ago_stays_on_its_target()
    {
        // The elapsed time of a ramp saturates, so a gain held for longer than
        // a `u32` of milliseconds, 49.7 days, never replays the ramp that
        // brought it there.
        let moves =
        [
            (full(), Volume::MUTED),
            (Volume::MUTED, full()),
            (full(), volume(COARSE_DEFAULT, 0x40)),
        ];
        for (first, then) in moves
        {
            let mut caller = Caller::new(FIXTURE_BUFFER_SAMPLES);
            let mut state = ControlState::new(0);
            let _ = ramp_ms(&mut state, &mut caller, first);
            let _ = ramp_ms(&mut state, &mut caller, then);

            state.ramp.elapsed_ms = u32::MAX - 1;
            let target = then.linear_gain().to_bits();
            for index in 0..8
            {
                let (_, applied) = caller.poll(&mut state);
                assert_eq!
                (
                    (applied.gain_start().to_bits(), applied.gain_end().to_bits()),
                    (target, target),
                    "the gain left {then:?} at buffer {index} past the end of the count"
                );
            }
        }
    }

    #[test]
    fn a_heartbeat_changes_no_setting()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(volume(COARSE_DEFAULT, 0x40)));
        let _ = state.poll(0, FIXTURE_BUFFER_MS);
        let before = state.applied_volume();

        state.request(ToDsp::Heartbeat);
        let now_ms = drive(&mut state, FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS, FULL_SWING_BUFFERS, |_, _| ());
        assert_eq!(state.applied_volume(), before);
        assert_eq!(state.poll(now_ms, FIXTURE_BUFFER_MS).preset(), Preset::Flat);
    }

    #[test]
    fn a_heartbeat_restarts_the_silence_measure()
    {
        let mut state = ControlState::new(0);
        let now_ms = drive(&mut state, 0, FIXTURE_BUFFER_MS, 10, |_, _| ());
        let _ = state.poll(now_ms, FIXTURE_BUFFER_MS);
        assert!(state.heartbeat_silence_ms(now_ms) > 0, "the measure never started");

        state.request(ToDsp::Heartbeat);
        assert_eq!(state.heartbeat_silence_ms(now_ms), 0);

        // And it runs again behind the beat.
        let later_ms = drive(&mut state, now_ms + FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS, 10, |_, _| ());
        assert_eq!(state.heartbeat_silence_ms(later_ms), later_ms - now_ms);
    }

    #[test]
    fn the_silence_measure_reads_wall_time_through_jitter()
    {
        // A millisecond source that runs early and late by turns. The measure
        // is wall time, so it matches the caller's own count whatever the
        // jitter.
        for jitter_ms in 1..=FIXTURE_BUFFER_MS / 2
        {
            let mut state = ControlState::new(0);
            state.request(ToDsp::Heartbeat);

            for index in 0..400_u32
            {
                let nominal_ms = index * FIXTURE_BUFFER_MS;
                let now_ms = if index % 2 == 0
                {
                    nominal_ms + jitter_ms
                }
                else
                {
                    nominal_ms - jitter_ms
                };
                let _ = state.poll(now_ms, FIXTURE_BUFFER_MS);
                assert_eq!
                (
                    state.heartbeat_silence_ms(now_ms),
                    now_ms,
                    "the measure lost wall time at buffer {index} with {jitter_ms} ms of jitter"
                );
            }
        }
    }

    #[test]
    fn a_half_and_full_transfer_pair_does_not_shorten_the_silence_measure()
    {
        // A half transfer and a transfer complete interrupt land as a pair, so
        // the caller's count moves by nothing and then by two periods. Capping
        // each at one period loses half the wall time.
        let mut state = ControlState::new(0);
        state.request(ToDsp::Heartbeat);

        let mut now_ms = 0_u32;
        for index in 0..400_u32
        {
            let _ = state.poll(now_ms, FIXTURE_BUFFER_MS);
            assert_eq!
            (
                state.heartbeat_silence_ms(now_ms),
                now_ms,
                "the measure lost wall time at interrupt {index}"
            );
            if index % 2 == 1
            {
                now_ms += 2 * FIXTURE_BUFFER_MS;
            }
        }
    }

    #[test]
    fn a_stopped_poll_does_not_freeze_the_silence_measure()
    {
        // The transfer interrupt stopping is one of the failures the measure
        // reports, so it cannot be what stops the measure.
        let mut state = ControlState::new(0);
        state.request(ToDsp::Heartbeat);
        let now_ms = drive(&mut state, 0, FIXTURE_BUFFER_MS, 4, |_, _| ());

        // Nothing polls again. The caller keeps reading its own count.
        for silent_ms in [0_u32, 1, 100, 5_000, 60_000]
        {
            let read_at_ms = now_ms + silent_ms;
            assert_eq!
            (
                state.heartbeat_silence_ms(read_at_ms),
                read_at_ms,
                "the measure froze {silent_ms} ms after the last poll"
            );
        }
    }

    #[test]
    fn a_peer_that_never_starts_reads_as_silence_since_the_build()
    {
        // A link that has said nothing must never read as recently alive.
        let mut state = ControlState::new(0);
        let now_ms = drive(&mut state, 0, FIXTURE_BUFFER_MS, 100, |_, _| ());
        assert_eq!(state.heartbeat_silence_ms(now_ms), now_ms);
    }

    #[test]
    fn a_declared_period_of_zero_does_not_freeze_the_ramp()
    {
        // A caller computing its period as samples times 1000 over the sample
        // rate reads zero for any buffer under 45 samples.
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(volume(COARSE_DEFAULT, 0x10)));
        let _ = state.poll(0, 0);
        assert_eq!(state.applied_volume(), volume(COARSE_DEFAULT, 0x10));

        state.request(ToDsp::SetVolume(full()));
        let mut now_ms = 0_u32;
        let mut applied = state.poll(now_ms, 0);
        while now_ms < 2 * GAIN_FULL_SWING_MS
        {
            now_ms += 1;
            applied = state.poll(now_ms, 0);
        }

        assert_eq!(state.applied_volume(), full(), "the request never reached the ramp");
        assert!
        (
            (applied.gain_end() - 1.0).abs() < EPSILON,
            "the gain froze at {}",
            applied.gain_end()
        );
        assert_eq!(state.heartbeat_silence_ms(now_ms), now_ms);
    }

    #[test]
    fn a_wrapping_millisecond_count_does_not_jump_the_ramp()
    {
        let start = u32::MAX - 4;
        let mut state = ControlState::new(start);
        state.request(ToDsp::SetVolume(full()));

        let bound = rate_bound(FIXTURE_BUFFER_MS);
        let mut previous_end = 0.0_f32;
        let _ = drive(&mut state, start, FIXTURE_BUFFER_MS, FULL_SWING_BUFFERS, |index, applied|
        {
            assert!
            (
                (applied.gain_start() - previous_end).abs() < EPSILON,
                "buffers do not meet across the wrap at {index}"
            );
            assert!(applied.gain_end() >= applied.gain_start());
            assert!(applied.gain_end() - applied.gain_start() <= bound);
            previous_end = applied.gain_end();
        });
        assert!((previous_end - 1.0).abs() < EPSILON);
    }

    #[test]
    fn a_long_silence_never_reads_as_a_fresh_beat()
    {
        // Both additions carrying the measure saturate. A wrapping one at
        // either site turns a dead link into one reporting itself alive, and
        // the ramp down never fires.
        let mut state = ControlState::new(0);
        state.request(ToDsp::Heartbeat);
        let _ = state.poll(0, FIXTURE_BUFFER_MS);
        let _ = state.poll(u32::MAX / 2, FIXTURE_BUFFER_MS);
        let _ = state.poll(1, FIXTURE_BUFFER_MS);
        assert_eq!(state.heartbeat_silence_ms(1), u32::MAX);
        assert_eq!(state.heartbeat_silence_ms(u32::MAX), u32::MAX);
    }
}
