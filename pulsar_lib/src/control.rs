//! The rate limit between a decoded control message and the audio chain.
//!
//! At 115200 baud the link carries roughly 1400 volume messages a second, and a
//! chain applying each of them as a step would turn the control link into a
//! full scale amplitude modulator aimed at the drivers. `docs/PRODUCT-SPEC.md`
//! section 3 rule 7 answers it: no transition is abrupt.
//!
//! `ControlState` bounds the RATE at which the applied setting moves, to one
//! change per `GAIN_RAMP_MS`, and hands out the gain at both ends of a buffer
//! so the caller walks the ramp sample by sample.
//!
//! It does not bound the envelope that survives the rate limit. A sender pacing
//! itself to the window drives the gain between its extremes at 25 Hz, whose
//! modulation products land beside the programme rather than under any corner
//! frequency. The volume gain sits upstream of the limiters, so holding that
//! envelope off the drivers is what the limiters are for.
//! `a_paced_sender_still_swings_the_gain` measures what is left.
//!
//! A preset change leaves through the same gate: the gain ramps to silence, the
//! preset moves on a buffer silent at both ends, and the gain rides back up
//! behind it, so the coefficients never move under signal. The filter state is
//! the caller's to clear, see `Applied::preset`.
//!
//! Two clocks run here. The ramp and the commit gate run on audio produced,
//! which is the caller's elapsed count capped at one buffer. The heartbeat
//! silence measure runs on the raw elapsed count, because a link that stopped
//! is measured in wall time and a capped clock reads it short.
//!
//! Time arrives as a parameter, so nothing here reads a clock.

// The millisecond counts converted to a float here are bounded by GAIN_RAMP_MS,
// and a sample index by the length of one buffer, so both conversions are
// exact.
#![allow(clippy::cast_precision_loss)]

use crate::constants::GAIN_RAMP_MS;
use crate::protocol::{Preset, ToDsp, Volume};

/// Shortest buffer period `poll` runs on, in milliseconds.
///
/// The declared period caps one step of the owned clock. A caller computing it
/// as samples times 1000 over the sample rate reads zero for any buffer under
/// 45 samples, and a cap of zero would freeze the clock, hold the gain where it
/// stands and never reopen the commit window. The floor rounds such a buffer up
/// to the resolution of the unit instead.
const MIN_BUFFER_MS: u32 = 1;

const _: () = assert!
(
    MIN_BUFFER_MS > 0,
    "a floor of zero would leave the owned clock frozen at its build value"
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
}

/// A linear move between two gains over `GAIN_RAMP_MS`.
#[derive(Debug, Clone, Copy)]
struct Ramp
{
    from: f32,
    to: f32,
    started_ms: u32,
}

impl Ramp
{
    /// Returns the gain the ramp holds at `now_ms`.
    ///
    /// Elapsed time is a wrapping difference, which reads a count running
    /// backwards as a large forward jump. `ControlState` owns the count this
    /// takes and advances it by at most one buffer per poll, so the difference
    /// is the forward distance it looks like, and the value moves monotonically
    /// to `to`.
    fn value_at(self, now_ms: u32) -> f32
    {
        let elapsed = now_ms.wrapping_sub(self.started_ms);
        if elapsed >= GAIN_RAMP_MS
        {
            return self.to;
        }
        let fraction = elapsed as f32 / GAIN_RAMP_MS as f32;
        self.from + (self.to - self.from) * fraction
    }

    /// Reports whether the ramp has arrived on `to` by `now_ms`.
    ///
    /// The caller reads this rather than comparing the gain against its target,
    /// which no float comparison decides.
    fn settled_at(self, now_ms: u32) -> bool
    {
        now_ms.wrapping_sub(self.started_ms) >= GAIN_RAMP_MS
    }
}

/// The only path from a decoded `ToDsp` message to a value the chain applies.
///
/// A request inside an open ramp replaces the previous one rather than queueing
/// behind it, so a burst costs one change and lands on the newest setting.
///
/// The type owns the millisecond count the ramp and the gate run on. `poll`
/// advances it by the smaller of the caller's elapsed count and the period of
/// the buffer being filled, since one poll produces one buffer of audio and no
/// more. A caller count that stalls, jumps forward or runs backwards therefore
/// cannot move the gain faster than the ramp, nor open the gate early.
///
/// The heartbeat silence measure runs on the caller's raw elapsed count
/// instead, saturating, because a capped count reads a silence short.
///
/// The state is a machine, so it is neither `Copy` nor `Clone`. Two copies
/// would each hold their own gate and disagree about what the chain applies.
#[derive(Debug)]
pub struct ControlState
{
    requested_volume: Volume,
    committed_volume: Volume,
    requested_preset: Preset,
    active_preset: Preset,
    ramp: Ramp,
    /// Count the ramp and the gate run on, advanced by `poll` alone.
    clock_ms: u32,
    /// Caller count the last `poll` read, for the elapsed difference.
    last_seen_ms: u32,
    /// Value of `clock_ms` when the last ramp started.
    last_commit_ms: u32,
    /// Gain the previous buffer ended on, which the next one starts from.
    last_gain: f32,
    /// Whether that gain is the target of the ramp rather than a point on it.
    last_gain_settled: bool,
    muting_for_swap: bool,
    /// Saturating milliseconds of wall time since the last heartbeat, or since
    /// the build of the state while none has arrived.
    silence_ms: u32,
}

impl ControlState
{
    /// Builds the state silent, on the protective crossover alone.
    ///
    /// `now_ms` is the caller's millisecond count at the build. The first
    /// change applies without waiting out a ramp, since no ramp is in flight.
    #[must_use]
    pub fn new(now_ms: u32) -> Self
    {
        Self
        {
            requested_volume: Volume::MUTED,
            committed_volume: Volume::MUTED,
            requested_preset: Preset::Flat,
            active_preset: Preset::Flat,
            ramp: Ramp
            {
                from: 0.0,
                to: 0.0,
                started_ms: now_ms,
            },
            clock_ms: now_ms,
            last_seen_ms: now_ms,
            last_commit_ms: now_ms.wrapping_sub(GAIN_RAMP_MS),
            last_gain: 0.0,
            last_gain_settled: true,
            muting_for_swap: false,
            silence_ms: 0,
        }
    }

    /// Records what the sender asked for.
    ///
    /// A setting does not reach the audio chain here. `poll` decides when it
    /// may, and a later request inside the same window replaces this one.
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
    /// walk it with `Applied::gain_at`.
    ///
    /// `buffer_ms` is held at `MIN_BUFFER_MS` or above, so a declared period of
    /// zero cannot freeze the clock. It has no ceiling, and a caller declaring
    /// a period longer than `GAIN_RAMP_MS` walks its whole ramp inside one
    /// buffer. Declaring the period of the buffer being filled is the caller's
    /// half of the contract.
    pub fn poll(&mut self, now_ms: u32, buffer_ms: u32) -> Applied
    {
        let gain_start = self.last_gain;
        self.advance_clock(now_ms, buffer_ms);

        if self.clock_ms.wrapping_sub(self.last_commit_ms) >= GAIN_RAMP_MS
        {
            self.commit();
        }

        let gain_end = self.ramp.value_at(self.clock_ms);
        self.last_gain = gain_end;
        self.last_gain_settled = self.ramp.settled_at(self.clock_ms);
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
    /// Nothing here acts on the value. `docs/PRODUCT-SPEC.md` section 4.4 owns
    /// the duration worth acting on, and the ramp and alarm behind it.
    #[must_use]
    pub fn heartbeat_silence_ms(&self, now_ms: u32) -> u32
    {
        let since_poll = now_ms.wrapping_sub(self.last_seen_ms);
        self.silence_ms.saturating_add(since_poll)
    }

    /// Advances the owned count by the audio one poll produces, and the silence
    /// measure by the wall time that passed.
    ///
    /// The step of the owned count is the caller's elapsed count capped at the
    /// declared period, held at `MIN_BUFFER_MS` or above. A count running
    /// backwards produces a wrapping distance far above any buffer period, so
    /// the cap catches it.
    ///
    /// The silence measure takes the raw elapsed count instead. Under the
    /// capped step it would count audio produced: lateness truncated, earliness
    /// never made up, and a poll that stopped freezing the measure.
    fn advance_clock(&mut self, now_ms: u32, buffer_ms: u32)
    {
        let elapsed = now_ms.wrapping_sub(self.last_seen_ms);
        let period_ms = buffer_ms.max(MIN_BUFFER_MS);
        let step = elapsed.min(period_ms);
        self.last_seen_ms = now_ms;
        self.clock_ms = self.clock_ms.wrapping_add(step);
        self.silence_ms = self.silence_ms.saturating_add(elapsed);
    }

    /// Starts at most one ramp, taking the newest request of each kind.
    ///
    /// A pending preset moves first, because its coefficients may only change
    /// under silence. The volume rides back up with it in the same window.
    ///
    /// The swap waits for the mute ramp to reach the caller, not merely to
    /// expire. A buffer starts on the gain the previous one ended on, so a swap
    /// taken as the ramp expires would move the coefficients across a buffer
    /// whose first sample is still audible.
    fn commit(&mut self)
    {
        let from = self.ramp.value_at(self.clock_ms);

        if self.muting_for_swap
        {
            if !self.last_gain_settled
            {
                return;
            }
            self.active_preset = self.requested_preset;
            self.committed_volume = self.requested_volume;
            self.muting_for_swap = false;
            self.start_ramp(from, self.committed_volume.linear_gain());
            return;
        }

        if self.requested_preset != self.active_preset
        {
            self.muting_for_swap = true;
            self.start_ramp(from, 0.0);
            return;
        }

        if self.requested_volume != self.committed_volume
        {
            self.committed_volume = self.requested_volume;
            self.start_ramp(from, self.committed_volume.linear_gain());
        }
    }

    /// Replaces the running ramp and closes the window behind it.
    fn start_ramp(&mut self, from: f32, to: f32)
    {
        self.ramp = Ramp
        {
            from,
            to,
            started_ms: self.clock_ms,
        };
        self.last_commit_ms = self.clock_ms;
    }
}

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, which is the one place the
    // no-panic rule does not hold.
    #![allow(clippy::panic)]

    use super::*;
    use crate::constants::SAMPLE_RATE_HZ;
    use crate::protocol::VOLUME_MAX;

    /// Samples per buffer these tests drive the gate with.
    ///
    /// A fixture, not a decision. The audio block size is still open, since it
    /// trades latency against interrupt load and against the look-ahead of the
    /// high way limiter.
    const FIXTURE_BUFFER_SAMPLES: u32 = 256;

    /// Period of the fixture buffer, in milliseconds, rounded up.
    ///
    /// This is the period a caller passes, and what the ramp has to survive. A
    /// test written at one millisecond hides a staircase four times coarser
    /// than it measures.
    const FIXTURE_BUFFER_MS: u32 = (FIXTURE_BUFFER_SAMPLES * 1_000).div_ceil(SAMPLE_RATE_HZ);

    /// Tolerance covering single precision rounding on a gain comparison.
    const EPSILON: f32 = 1e-6;

    /// Builds a volume pair, failing the test rather than returning an error.
    fn volume(coarse: u8, fine: u8) -> Volume
    {
        match Volume::new(coarse, fine)
        {
            Ok(volume) => volume,
            Err(error) => panic!("volume rejected: {error:?}"),
        }
    }

    /// Returns the widest gain move `buffer_ms` of ramp allows.
    fn rate_bound(buffer_ms: u32) -> f32
    {
        buffer_ms as f32 / GAIN_RAMP_MS as f32 + EPSILON
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
        state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));
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
    fn the_gain_rate_holds_at_every_buffer_period()
    {
        // The buffer period is the caller's choice, and none of the periods it
        // can take may turn the ramp into a step.
        for buffer_ms in 1..=60_u32
        {
            let mut state = ControlState::new(0);
            state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));

            let len = FIXTURE_BUFFER_SAMPLES as usize;
            let bound = rate_bound(buffer_ms);
            let sample_bound = bound / len as f32 + EPSILON;
            let mut previous_end = 0.0_f32;
            let mut previous_sample = 0.0_f32;

            let observe = |index: u32, applied: Applied|
            {
                assert!
                (
                    (applied.gain_start() - previous_end).abs() < EPSILON,
                    "buffers {index} and the one before it do not meet at {buffer_ms} ms"
                );
                assert!
                (
                    (applied.gain_end() - applied.gain_start()).abs() <= bound,
                    "the gain outran the ramp over buffer {index} at {buffer_ms} ms"
                );
                for sample in 0..len
                {
                    let gain = applied.gain_at(sample, len);
                    assert!
                    (
                        (gain - previous_sample).abs() <= sample_bound,
                        "the gain stepped at sample {sample} of buffer {index} at {buffer_ms} ms"
                    );
                    previous_sample = gain;
                }
                previous_end = applied.gain_end();
            };

            let _ = drive(&mut state, 0, buffer_ms, 200, observe);
            assert!((previous_end - 1.0).abs() < EPSILON, "the ramp never arrived");
        }
    }

    #[test]
    fn a_gap_in_the_caller_count_never_arrives_as_a_step()
    {
        // A late interrupt or a stalled loop: poll at zero, then at five
        // seconds. The ramp moves by one buffer, which is all the audio the
        // chain produced.
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));

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
            state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));

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
    fn a_wobbling_count_does_not_open_the_commit_window_early()
    {
        // A millisecond source that jitters by one count either way. The gate
        // runs on audio produced, so the jitter buys no extra windows.
        let mut state = ControlState::new(0);
        let span_ms = 500_u32;
        let buffers = span_ms / FIXTURE_BUFFER_MS;
        let mut now_ms = 0_u32;
        let mut previous = state.applied_volume();
        let mut changes = 0_u32;

        for index in 0..buffers
        {
            let level = if index % 2 == 0
            {
                VOLUME_MAX
            }
            else
            {
                0x00
            };
            state.request(ToDsp::SetVolume(volume(level, VOLUME_MAX)));
            let _ = state.poll(now_ms, FIXTURE_BUFFER_MS);
            let current = state.applied_volume();
            if current != previous
            {
                changes += 1;
                previous = current;
            }
            now_ms = now_ms.wrapping_add(FIXTURE_BUFFER_MS);
            now_ms = if index % 2 == 0
            {
                now_ms.wrapping_sub(1)
            }
            else
            {
                now_ms.wrapping_add(1)
            };
        }

        assert!(changes <= span_ms / GAIN_RAMP_MS + 1, "applied {changes} changes");
    }

    #[test]
    fn a_burst_inside_one_window_applies_once_and_lands_on_the_newest_target()
    {
        let mut state = ControlState::new(0);
        // A first setting closes the window behind it.
        state.request(ToDsp::SetVolume(volume(0x10, 0x10)));
        let _ = state.poll(0, FIXTURE_BUFFER_MS);
        assert_eq!(state.applied_volume(), volume(0x10, 0x10));

        // A burst inside that window reaches nothing.
        let mut now_ms = FIXTURE_BUFFER_MS;
        while now_ms < GAIN_RAMP_MS
        {
            state.request(ToDsp::SetVolume(volume(0x00, 0x00)));
            state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));
            let applied = state.poll(now_ms, FIXTURE_BUFFER_MS);
            assert_eq!(state.applied_volume(), volume(0x10, 0x10));
            assert!(applied.gain_end() >= 0.0);
            now_ms += FIXTURE_BUFFER_MS;
        }

        // The window opens on the newest request, not on the queue behind it.
        let _ = state.poll(GAIN_RAMP_MS, FIXTURE_BUFFER_MS);
        assert_eq!(state.applied_volume(), volume(VOLUME_MAX, VOLUME_MAX));
    }

    #[test]
    fn an_alternating_flood_cannot_modulate_the_output()
    {
        // Valid frames at full and at zero, one per buffer, for a second. What
        // reaches the drivers is the gain trace, so the bound is on the trace.
        // A reversal costs a whole ramp, one per GAIN_RAMP_MS at most.
        let mut state = ControlState::new(0);
        let span_ms = 1_000_u32;
        let buffers = span_ms / FIXTURE_BUFFER_MS;
        let bound = rate_bound(FIXTURE_BUFFER_MS);
        let mut previous_end = 0.0_f32;
        let mut reversals = 0_u32;
        let mut rising = true;

        for index in 0..buffers
        {
            let level = if index % 2 == 0
            {
                VOLUME_MAX
            }
            else
            {
                0x00
            };
            state.request(ToDsp::SetVolume(volume(level, VOLUME_MAX)));
            let applied = state.poll(index * FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS);

            assert!((applied.gain_start() - previous_end).abs() < EPSILON);
            let moved = applied.gain_end() - applied.gain_start();
            assert!(moved.abs() <= bound, "the gain moved {moved} over buffer {index}");
            if moved.abs() > EPSILON
            {
                let now_rising = moved > 0.0;
                if now_rising != rising
                {
                    reversals += 1;
                    rising = now_rising;
                }
            }
            previous_end = applied.gain_end();
        }

        assert!
        (
            reversals <= span_ms / GAIN_RAMP_MS + 1,
            "the gain reversed {reversals} times"
        );
    }

    #[test]
    fn a_paced_sender_still_swings_the_gain()
    {
        // What the gate does NOT do. A sender pacing itself to the window
        // drives the gain between its extremes at one half of 1000 /
        // GAIN_RAMP_MS hertz. The rate is bounded, the envelope is not, and the
        // limiters have to absorb it.
        let mut state = ControlState::new(0);
        let mut now_ms = 0_u32;
        let mut lowest = 1.0_f32;
        let mut highest = 0.0_f32;

        for index in 0..40_u32
        {
            let level = if index % 2 == 0
            {
                VOLUME_MAX
            }
            else
            {
                0x00
            };
            state.request(ToDsp::SetVolume(volume(level, VOLUME_MAX)));
            for _ in 0..GAIN_RAMP_MS
            {
                let applied = state.poll(now_ms, 1);
                now_ms = now_ms.wrapping_add(1);
                if index >= 4
                {
                    lowest = lowest.min(applied.gain_end());
                    highest = highest.max(applied.gain_end());
                }
            }
        }

        assert!
        (
            highest - lowest > 0.9,
            "the residual envelope measured {}",
            highest - lowest
        );
    }

    #[test]
    fn changes_spaced_wider_than_the_window_each_apply()
    {
        let mut state = ControlState::new(0);
        let mut now_ms = 0_u32;
        for level in [0x10_u8, 0x20, 0x30, 0x40]
        {
            let expected = volume(level, VOLUME_MAX);
            state.request(ToDsp::SetVolume(expected));
            let _ = state.poll(now_ms, FIXTURE_BUFFER_MS);
            assert_eq!(state.applied_volume(), expected);

            now_ms = drive(&mut state, now_ms + FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS, GAIN_RAMP_MS, |_, _| ());
            let applied = state.poll(now_ms, FIXTURE_BUFFER_MS);
            now_ms += FIXTURE_BUFFER_MS;
            assert!((applied.gain_end() - expected.linear_gain()).abs() < EPSILON);
        }
    }

    #[test]
    fn the_ramp_is_monotonic_between_its_endpoints()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));

        let mut previous = -1.0_f32;
        let _ = drive(&mut state, 0, FIXTURE_BUFFER_MS, 2 * GAIN_RAMP_MS, |index, applied|
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
        state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));
        let now_ms = drive(&mut state, 0, FIXTURE_BUFFER_MS, 2 * GAIN_RAMP_MS, |_, _| ());

        state.request(ToDsp::SetVolume(volume(0x00, 0x00)));
        let mut previous = 2.0_f32;
        let _ = drive(&mut state, now_ms, FIXTURE_BUFFER_MS, 2 * GAIN_RAMP_MS, |index, applied|
        {
            assert!(applied.gain_start() <= previous, "the gain rose at buffer {index}");
            assert!(applied.gain_end() <= applied.gain_start(), "the gain rose inside {index}");
            previous = applied.gain_end();
        });
        assert!(previous.abs() < EPSILON);
    }

    #[test]
    fn a_preset_change_is_never_applied_as_a_step()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));
        let now_ms = drive(&mut state, 0, FIXTURE_BUFFER_MS, 2 * GAIN_RAMP_MS, |_, _| ());

        state.request(ToDsp::SelectPreset(Preset::Garden));
        let bound = rate_bound(FIXTURE_BUFFER_MS);
        let mut previous_preset = Preset::Flat;
        let mut previous_end = 1.0_f32;
        let mut swapped_at_silence = false;
        let mut final_gain = 0.0_f32;

        let _ = drive(&mut state, now_ms, FIXTURE_BUFFER_MS, 6 * GAIN_RAMP_MS, |index, applied|
        {
            assert!((applied.gain_start() - previous_end).abs() < EPSILON);
            assert!
            (
                (applied.gain_end() - applied.gain_start()).abs() <= bound,
                "the gain jumped over buffer {index}"
            );
            if applied.preset() != previous_preset
            {
                // The coefficients may only move while nothing comes out.
                assert!(applied.gain_start().abs() < EPSILON, "coefficients moved under signal");
                assert!(applied.gain_end().abs() < EPSILON, "coefficients moved under signal");
                swapped_at_silence = true;
                previous_preset = applied.preset();
            }
            previous_end = applied.gain_end();
            final_gain = applied.gain_end();
        });

        assert!(swapped_at_silence, "the preset never reached the chain");
        assert_eq!(previous_preset, Preset::Garden);
        assert!((final_gain - 1.0).abs() < EPSILON);
    }

    #[test]
    fn a_heartbeat_changes_no_setting()
    {
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(volume(0x40, 0x40)));
        let _ = state.poll(0, FIXTURE_BUFFER_MS);
        let before = state.applied_volume();

        state.request(ToDsp::Heartbeat);
        let now_ms = drive(&mut state, FIXTURE_BUFFER_MS, FIXTURE_BUFFER_MS, 4 * GAIN_RAMP_MS, |_, _| ());
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
    fn a_declared_period_of_zero_does_not_freeze_the_gate()
    {
        // A caller computing its period as samples times 1000 over the sample
        // rate reads zero for any buffer under 45 samples.
        let mut state = ControlState::new(0);
        state.request(ToDsp::SetVolume(volume(0x10, 0x10)));
        let _ = state.poll(0, 0);
        assert_eq!(state.applied_volume(), volume(0x10, 0x10));

        state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));
        let mut now_ms = 0_u32;
        let mut applied = state.poll(now_ms, 0);
        while now_ms < 4 * GAIN_RAMP_MS
        {
            now_ms += 1;
            applied = state.poll(now_ms, 0);
        }

        assert_eq!
        (
            state.applied_volume(),
            volume(VOLUME_MAX, VOLUME_MAX),
            "the commit window never reopened"
        );
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
        state.request(ToDsp::SetVolume(volume(VOLUME_MAX, VOLUME_MAX)));

        let bound = rate_bound(FIXTURE_BUFFER_MS);
        let mut previous_end = 0.0_f32;
        let _ = drive(&mut state, start, FIXTURE_BUFFER_MS, 2 * GAIN_RAMP_MS, |index, applied|
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
