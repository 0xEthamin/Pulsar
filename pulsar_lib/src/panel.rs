//! The controls of the interface board and the traffic they produce.
//!
//! Two controls set the level: the encoder on the cabinet owns the coarse
//! step, and the phone owns the fine value through AVRCP absolute volume. A
//! schedule puts the pair on the control link often enough for the processing
//! board to keep the gain up, and rarely enough for its receiver to hold every
//! character between two polls.
//!
//! Nothing here touches a peripheral. `CoarseControl` takes the reading of a
//! pulse counter, `PhoneVolume` takes the events of a remote control
//! connection, and `Uplink` hands out the message to send next.
//!
//! # What the phone owns
//!
//! AVRCP 1.6.3 makes absolute volume mandatory for a controller advertising
//! Category 2 and excluded for one that does not (table 3.1 item 16, note C4),
//! so the category bit of its service record is what the law reads:
//!
//! - while the connection has answered nothing, the fine value is 0, which is
//!   silence. No default stands in for a value the phone owns.
//! - a phone carrying absolute volume owns the fine value, 0 included. A 0 is
//!   a mute the listener asked for and never a missing answer.
//! - a phone without it scales the stream it sends, so the fine value sits at
//!   `FINE_MAX` and the encoder alone moves the level.
//! - with no phone on the link the fine value is 0.
//!
//! The cabinet never pushes a volume of its own at a phone: the coarse step
//! stays on this side of the link, so a phone that connects keeps the volume
//! it arrived with.
//!
//! # Pacing
//!
//! `Uplink` sends the volume on every change, resends it every
//! `RESEND_PERIOD_MS`, and sends a heartbeat every `RESEND_PERIOD_MS`. Only a
//! heartbeat restarts the silence measure of `crate::control::ControlState`,
//! and the resend is what repairs a volume frame lost to a line error. A
//! heartbeat that is due leaves ahead of the volume, so a knob turning under
//! the hand cannot hold it back.
//!
//! Two frames are never closer than `MIN_FRAME_GAP_MS`, which bounds what one
//! poll of the receiver of `crate::link` finds waiting for it. The assertions
//! below hold that gap against what the receiver holds over one of its polls,
//! and the period against the silence limit that drops the gain.

use crate::link::{LINK_SILENCE_LIMIT_MS, RX_HOLD_BYTES, RX_POLL_PERIOD_MS};
use crate::protocol::{
    COARSE_DEFAULT,
    FINE_MAX,
    MAX_FRAME_LEN,
    ToDsp,
    Volume,
    fine_from_absolute_volume,
    step_coarse,
};

/// Milliseconds between two heartbeats, and between two resends of the volume.
pub const RESEND_PERIOD_MS: u32 = 250;

/// Shortest time between two frames, in milliseconds.
pub const MIN_FRAME_GAP_MS: u32 = 5;

/// Counter steps one detent of the encoder produces.
///
/// One channel of the pulse counter reads both edges of A over one quadrature
/// cycle, and a detent of the cabinet encoder is one cycle.
pub const COUNTS_PER_DETENT: i32 = 2;

const _: () = assert!
(
    MIN_FRAME_GAP_MS > 0
        && MAX_FRAME_LEN.saturating_mul(1 + (RX_POLL_PERIOD_MS / MIN_FRAME_GAP_MS) as usize)
            <= RX_HOLD_BYTES as usize,
    "the frames one gap apart that reach the receiver inside one of its polls \
     fit in what it holds"
);

const _: () = assert!
(
    RESEND_PERIOD_MS + MIN_FRAME_GAP_MS < LINK_SILENCE_LIMIT_MS / 4,
    "the processing board reads several heartbeats before it calls the link lost"
);

/// The coarse step, as the encoder on the cabinet moves it.
///
/// The step starts at `COARSE_DEFAULT` on every power up, since the encoder is
/// incremental and nothing stores a setting.
#[derive(Debug)]
pub struct CoarseControl
{
    coarse: u8,
    last_count: i32,
    remainder: i32,
}

impl CoarseControl
{
    /// Builds the control on `COARSE_DEFAULT`, reading a counter that stands
    /// at `count`.
    #[must_use]
    pub const fn at_power_up(count: i32) -> Self
    {
        Self
        {
            coarse: COARSE_DEFAULT,
            last_count: count,
            remainder: 0,
        }
    }

    /// Returns the step the control holds.
    #[must_use]
    pub const fn coarse(&self) -> u8
    {
        self.coarse
    }

    /// Moves the step by the detents `count` adds to the previous reading, and
    /// returns where it lands.
    ///
    /// Counts that do not fill a detent are kept for the next reading, so a
    /// detent turned across two readings still moves the step once and the
    /// count never drifts. A big jump saturates at 0 or `COARSE_MAX`, and the
    /// step lost at an end does not come back, exactly as on the knob.
    ///
    /// The distance between two readings is taken modulo the width of the
    /// counter, so a move of more than half that width reads as one in the
    /// other direction. A hand on the knob is many orders of magnitude away
    /// from that.
    pub fn read(&mut self, count: i32) -> u8
    {
        let moved = count.wrapping_sub(self.last_count);
        self.last_count = count;

        let total = self.remainder.saturating_add(moved);
        let detents = total.checked_div(COUNTS_PER_DETENT).unwrap_or(0);
        self.remainder = total.checked_rem(COUNTS_PER_DETENT).unwrap_or(0);
        self.coarse = step_coarse(self.coarse, detents);
        self.coarse
    }
}

/// What the phone on the other end of the remote control connection carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhoneVolume
{
    /// No remote control connection, so no phone owns the fine value.
    Absent,
    /// Connected, and the service record of the phone has not been read yet.
    Undecided,
    /// The phone carries absolute volume and has sent no value yet.
    Waiting,
    /// The value the phone set, 0 included.
    Set(u8),
    /// The phone has no absolute volume and scales the stream it sends.
    Scaling,
}

/// What the remote control connection reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhoneEvent
{
    /// A remote control connection came up.
    Connected,
    /// The remote control connection went away.
    Disconnected,
    /// An audio connection came up while no remote control connection was
    /// known.
    ///
    /// A remote control connection outlives an audio one, and the stack
    /// reports it only when it opens, so this is what says a phone is back
    /// after the audio path reported one gone. It says nothing about what the
    /// phone carries.
    MediaConnected,
    /// The service record of the phone as a controller, read over SDP.
    Controller
    {
        /// Whether the record advertises Category 2, which makes absolute
        /// volume mandatory for it.
        category_two: bool,
    },
    /// The phone set an absolute volume, as the octet it sent.
    AbsoluteVolume(u8),
    /// The phone registered for the volume change notification, which only a
    /// controller carrying absolute volume sends.
    VolumeNotificationRegistered,
}

impl PhoneVolume
{
    /// Word of the `Absent` state, which a shared cell starts on so that the
    /// cabinet is silent before the first event.
    pub const SILENT_BITS: u16 = Self::ABSENT_BITS;

    /// Word of each state. The high byte names the state and the low byte
    /// carries the value of `Set`, so an unknown high byte reads as `Absent`.
    const ABSENT_BITS: u16 = 0x0000;
    const UNDECIDED_BITS: u16 = 0x0100;
    const WAITING_BITS: u16 = 0x0200;
    const SCALING_BITS: u16 = 0x0300;
    const SET_BITS: u16 = 0x0400;
    const VALUE_MASK: u16 = 0x00FF;

    /// Returns the fine control the state stands for.
    ///
    /// Every state that has not heard a value from a phone carrying absolute
    /// volume returns 0, which mutes the chain.
    #[must_use]
    pub const fn fine(self) -> u8
    {
        match self
        {
            Self::Absent | Self::Undecided | Self::Waiting => 0,
            Self::Set(value) => value,
            Self::Scaling => FINE_MAX,
        }
    }

    /// Returns the volume to report in a notification response, which is the
    /// absolute volume the phone last set, or 0 while it has set none.
    ///
    /// Reporting the fine value keeps the cabinet from pushing a level at a
    /// phone that just connected.
    #[must_use]
    pub const fn reported(self) -> u8
    {
        match self
        {
            Self::Set(value) => value,
            _ => 0,
        }
    }

    /// Returns the state `event` leads to.
    ///
    /// A value from the phone outranks its service record, and so does a
    /// registration: a record is what the phone claims, while a value and a
    /// registration are what it does, and only a controller carrying absolute
    /// volume registers. `Connected` and `MediaConnected` are the two events
    /// that leave `Absent`, and both land on a state that is silent, so a
    /// command arriving with no connection cannot raise the gain and neither
    /// can the connection itself.
    #[must_use]
    pub fn apply(self, event: PhoneEvent) -> Self
    {
        match event
        {
            PhoneEvent::Disconnected => Self::Absent,
            PhoneEvent::Connected => Self::Undecided,
            PhoneEvent::MediaConnected => match self
            {
                Self::Absent => Self::Undecided,
                _ => self,
            },
            PhoneEvent::AbsoluteVolume(octet) => match self
            {
                Self::Absent => Self::Absent,
                _ => Self::Set(fine_from_absolute_volume(octet)),
            },
            PhoneEvent::VolumeNotificationRegistered => match self
            {
                Self::Absent | Self::Set(_) => self,
                _ => Self::Waiting,
            },
            PhoneEvent::Controller { category_two } => match self
            {
                Self::Absent | Self::Set(_) | Self::Waiting => self,
                _ if category_two => Self::Waiting,
                _ => Self::Scaling,
            },
        }
    }

    /// Returns the state as one word, for a shared cell the callback of the
    /// radio writes and the interface task reads.
    #[must_use]
    pub fn to_bits(self) -> u16
    {
        match self
        {
            Self::Absent => Self::ABSENT_BITS,
            Self::Undecided => Self::UNDECIDED_BITS,
            Self::Waiting => Self::WAITING_BITS,
            Self::Scaling => Self::SCALING_BITS,
            Self::Set(value) => Self::SET_BITS | u16::from(value),
        }
    }

    /// Reads back a state written by `to_bits`.
    ///
    /// A word naming no state reads as `Absent`, which is silence.
    #[must_use]
    pub fn from_bits(bits: u16) -> Self
    {
        let value = u8::try_from(bits & Self::VALUE_MASK).unwrap_or(0);
        match bits & !Self::VALUE_MASK
        {
            Self::UNDECIDED_BITS => Self::Undecided,
            Self::WAITING_BITS => Self::Waiting,
            Self::SCALING_BITS => Self::Scaling,
            Self::SET_BITS => Self::Set(fine_from_absolute_volume(value)),
            _ => Self::Absent,
        }
    }
}

/// Returns the volume the two controls stand for.
///
/// A pair out of range gives `Volume::MUTED`, since a level the machine cannot
/// name is not a level to guess at. Neither control can produce one.
#[must_use]
pub fn volume_of(coarse: u8, phone: PhoneVolume) -> Volume
{
    Volume::new(coarse, phone.fine()).unwrap_or(Volume::MUTED)
}

/// The schedule of what the interface board sends on the control link.
///
/// It runs on the time its caller measures between two polls, not on a clock
/// it reads, so a count that jumps or stands still cannot make it send a burst.
#[derive(Debug)]
pub struct Uplink
{
    heartbeat_due_ms: u32,
    volume_due_ms: u32,
    gap_ms: u32,
    sent: Option<Volume>,
}

impl Uplink
{
    /// Builds a schedule that sends a heartbeat on its first poll and the
    /// volume one gap later.
    #[must_use]
    pub const fn at_power_up() -> Self
    {
        Self
        {
            heartbeat_due_ms: 0,
            volume_due_ms: 0,
            gap_ms: 0,
            sent: None,
        }
    }

    /// Advances the schedule by `elapsed_ms` and returns the message to send.
    ///
    /// `volume` is what the controls hold now. A poll sends one message at
    /// most, and none until `MIN_FRAME_GAP_MS` has passed since the last one.
    /// The heartbeat goes out ahead of the volume when both are due: it comes
    /// round once per `RESEND_PERIOD_MS` and the volume can be due on every
    /// poll, so the other order lets a knob under the hand starve the proof of
    /// life the processing board waits for.
    pub fn poll(&mut self, elapsed_ms: u32, volume: Volume) -> Option<ToDsp>
    {
        self.heartbeat_due_ms = self.heartbeat_due_ms.saturating_sub(elapsed_ms);
        self.volume_due_ms = self.volume_due_ms.saturating_sub(elapsed_ms);
        self.gap_ms = self.gap_ms.saturating_sub(elapsed_ms);

        if self.gap_ms > 0
        {
            return None;
        }

        if self.heartbeat_due_ms == 0
        {
            self.heartbeat_due_ms = RESEND_PERIOD_MS;
            self.gap_ms = MIN_FRAME_GAP_MS;
            return Some(ToDsp::Heartbeat);
        }

        if self.sent != Some(volume) || self.volume_due_ms == 0
        {
            self.sent = Some(volume);
            self.volume_due_ms = RESEND_PERIOD_MS;
            self.gap_ms = MIN_FRAME_GAP_MS;
            return Some(ToDsp::SetVolume(volume));
        }

        None
    }
}

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, and every count below is a
    // number of milliseconds or bytes far from the width of its type.
    #![allow(clippy::panic, clippy::arithmetic_side_effects, clippy::float_cmp)]

    use std::vec::Vec;

    use super::*;
    use crate::protocol::{COARSE_MAX, Message};

    fn volume(coarse: u8, fine: u8) -> Volume
    {
        match Volume::new(coarse, fine)
        {
            Ok(volume) => volume,
            Err(error) => panic!("volume rejected: {error:?}"),
        }
    }

    /// Returns the length of the frame `message` travels as.
    fn frame_len(message: ToDsp) -> usize
    {
        let mut out = [0_u8; MAX_FRAME_LEN];
        match message.encode_into(&mut out)
        {
            Ok(used) => used,
            Err(error) => panic!("{message:?} did not encode: {error:?}"),
        }
    }

    #[test]
    fn the_coarse_control_starts_on_the_default_step()
    {
        let control = CoarseControl::at_power_up(0);
        assert_eq!(control.coarse(), COARSE_DEFAULT);
        assert_eq!(COARSE_DEFAULT, 5);
    }

    #[test]
    fn one_detent_moves_one_step()
    {
        let mut control = CoarseControl::at_power_up(0);
        assert_eq!(control.read(COUNTS_PER_DETENT), COARSE_DEFAULT + 1);
        assert_eq!(control.read(0), COARSE_DEFAULT);
    }

    #[test]
    fn counts_below_a_detent_are_kept_and_never_drift()
    {
        let mut control = CoarseControl::at_power_up(0);
        let mut count = 0;

        // Half a detent per reading, so every second reading moves one step.
        for step in 1..=20
        {
            count += 1;
            let coarse = control.read(count);
            let expected = step_coarse(COARSE_DEFAULT, step / 2);
            assert_eq!(coarse, expected, "reading {step} landed wrong");
        }
    }

    #[test]
    fn a_reading_that_does_not_move_holds_the_step()
    {
        let mut control = CoarseControl::at_power_up(7);
        assert_eq!(control.read(7), COARSE_DEFAULT);
        assert_eq!(control.read(7), COARSE_DEFAULT);
    }

    #[test]
    fn the_step_is_held_inside_its_range()
    {
        let mut control = CoarseControl::at_power_up(0);
        assert_eq!(control.read(1_000_000), COARSE_MAX);
        assert_eq!(control.read(2_000_000), COARSE_MAX);
        assert_eq!(control.read(-1_000_000), 0);
        assert_eq!(control.read(0), COARSE_MAX);
        assert_eq!(control.read(i32::MIN / 2), 0);
    }

    #[test]
    fn turning_back_and_forth_inside_the_range_returns_to_the_same_step()
    {
        let mut control = CoarseControl::at_power_up(0);
        for turn in 0..=4
        {
            control.read(turn * COUNTS_PER_DETENT);
        }
        assert_eq!(control.coarse(), COARSE_DEFAULT + 4);

        for turn in (0..=4).rev()
        {
            control.read(turn * COUNTS_PER_DETENT);
        }
        assert_eq!(control.coarse(), COARSE_DEFAULT);
    }

    #[test]
    fn a_step_lost_to_the_end_of_the_range_does_not_come_back()
    {
        // The control saturates at both ends, so turning past the top and
        // straight back lands lower than it started, like the knob it reads.
        let mut control = CoarseControl::at_power_up(0);
        control.read(100 * COUNTS_PER_DETENT);
        assert_eq!(control.coarse(), COARSE_MAX);
        control.read(0);
        assert_eq!(control.coarse(), 0);
    }

    #[test]
    fn nothing_is_heard_before_a_phone_answers()
    {
        assert_eq!(PhoneVolume::Absent.fine(), 0);
        assert_eq!(PhoneVolume::Undecided.fine(), 0);
        assert_eq!(PhoneVolume::Waiting.fine(), 0);
    }

    #[test]
    fn a_phone_without_absolute_volume_leaves_the_level_to_the_encoder()
    {
        let state = PhoneVolume::Absent
            .apply(PhoneEvent::Connected)
            .apply(PhoneEvent::Controller { category_two: false });
        assert_eq!(state, PhoneVolume::Scaling);
        assert_eq!(state.fine(), FINE_MAX);
    }

    #[test]
    fn a_phone_with_absolute_volume_is_silent_until_it_sends_a_value()
    {
        let waiting = PhoneVolume::Absent
            .apply(PhoneEvent::Connected)
            .apply(PhoneEvent::Controller { category_two: true });
        assert_eq!(waiting, PhoneVolume::Waiting);
        assert_eq!(waiting.fine(), 0);

        let set = waiting.apply(PhoneEvent::AbsoluteVolume(0x40));
        assert_eq!(set, PhoneVolume::Set(0x40));
        assert_eq!(set.fine(), 0x40);
    }

    #[test]
    fn a_registration_alone_proves_absolute_volume()
    {
        let state = PhoneVolume::Absent
            .apply(PhoneEvent::Connected)
            .apply(PhoneEvent::VolumeNotificationRegistered);
        assert_eq!(state, PhoneVolume::Waiting);
    }

    #[test]
    fn a_zero_from_the_phone_is_a_mute_and_not_a_missing_answer()
    {
        let state = PhoneVolume::Waiting.apply(PhoneEvent::AbsoluteVolume(0));
        assert_eq!(state, PhoneVolume::Set(0));
        assert_eq!(state.fine(), 0);
        assert_eq!(volume_of(COARSE_MAX, state).linear_gain(), 0.0);
    }

    #[test]
    fn the_reserved_bit_of_an_absolute_volume_octet_is_ignored()
    {
        for value in 0..=FINE_MAX
        {
            let plain = PhoneVolume::Waiting.apply(PhoneEvent::AbsoluteVolume(value));
            let reserved = PhoneVolume::Waiting.apply(PhoneEvent::AbsoluteVolume(value | 0x80));
            assert_eq!(plain, PhoneVolume::Set(value));
            assert_eq!(reserved, plain, "the reserved bit changed value {value}");
        }
    }

    #[test]
    fn a_registration_outranks_a_record_that_denies_absolute_volume()
    {
        // A registration comes only from a controller carrying absolute
        // volume, so a record that denies it must not move the state the loud
        // way afterwards.
        let state = PhoneVolume::Absent
            .apply(PhoneEvent::Connected)
            .apply(PhoneEvent::VolumeNotificationRegistered)
            .apply(PhoneEvent::Controller { category_two: false });
        assert_eq!(state, PhoneVolume::Waiting);
        assert_eq!(state.fine(), 0);
    }

    #[test]
    fn an_audio_connection_makes_the_state_reachable_again()
    {
        // The audio path reports a phone gone while its remote control
        // connection survives, and the stack then reports no new one. The
        // audio connection is what lets the state be left again.
        let gone = PhoneVolume::Waiting
            .apply(PhoneEvent::AbsoluteVolume(0x40))
            .apply(PhoneEvent::Disconnected);
        assert_eq!(gone, PhoneVolume::Absent);

        let back = gone.apply(PhoneEvent::MediaConnected);
        assert_eq!(back, PhoneVolume::Undecided);
        assert_eq!(back.fine(), 0, "an audio connection raised the level on its own");
        assert_eq!(back.apply(PhoneEvent::AbsoluteVolume(0x20)), PhoneVolume::Set(0x20));
        assert_eq!
        (
            back.apply(PhoneEvent::VolumeNotificationRegistered),
            PhoneVolume::Waiting
        );
    }

    #[test]
    fn an_audio_connection_replaces_nothing_the_phone_said()
    {
        for state in
        [
            PhoneVolume::Undecided,
            PhoneVolume::Waiting,
            PhoneVolume::Scaling,
            PhoneVolume::Set(0),
            PhoneVolume::Set(FINE_MAX),
        ]
        {
            assert_eq!(state.apply(PhoneEvent::MediaConnected), state, "{state:?} moved");
        }
    }

    #[test]
    fn a_value_outranks_a_record_that_denies_absolute_volume()
    {
        let state = PhoneVolume::Waiting
            .apply(PhoneEvent::AbsoluteVolume(0x20))
            .apply(PhoneEvent::Controller { category_two: false });
        assert_eq!(state, PhoneVolume::Set(0x20));
    }

    #[test]
    fn a_command_without_a_connection_stays_silent()
    {
        for event in
        [
            PhoneEvent::AbsoluteVolume(FINE_MAX),
            PhoneEvent::VolumeNotificationRegistered,
            PhoneEvent::Controller { category_two: false },
        ]
        {
            assert_eq!(PhoneVolume::Absent.apply(event), PhoneVolume::Absent);
        }
    }

    #[test]
    fn a_disconnection_silences_and_a_new_connection_keeps_nothing()
    {
        let set = PhoneVolume::Waiting.apply(PhoneEvent::AbsoluteVolume(FINE_MAX));
        let gone = set.apply(PhoneEvent::Disconnected);
        assert_eq!(gone, PhoneVolume::Absent);
        assert_eq!(gone.fine(), 0);
        assert_eq!(set.apply(PhoneEvent::Connected), PhoneVolume::Undecided);
    }

    #[test]
    fn the_cabinet_reports_the_value_of_the_phone_and_never_one_of_its_own()
    {
        assert_eq!(PhoneVolume::Absent.reported(), 0);
        assert_eq!(PhoneVolume::Undecided.reported(), 0);
        assert_eq!(PhoneVolume::Waiting.reported(), 0);
        assert_eq!(PhoneVolume::Scaling.reported(), 0);
        assert_eq!(PhoneVolume::Set(0x33).reported(), 0x33);
    }

    #[test]
    fn a_state_survives_the_trip_through_one_word()
    {
        let mut states: Vec<PhoneVolume> = Vec::new();
        states.extend
        ([
            PhoneVolume::Absent,
            PhoneVolume::Undecided,
            PhoneVolume::Waiting,
            PhoneVolume::Scaling,
        ]);
        states.extend((0..=FINE_MAX).map(PhoneVolume::Set));

        for state in states
        {
            assert_eq!(PhoneVolume::from_bits(state.to_bits()), state);
        }
    }

    #[test]
    fn a_shared_cell_starts_silent()
    {
        assert_eq!(PhoneVolume::from_bits(PhoneVolume::SILENT_BITS), PhoneVolume::Absent);
        assert_eq!(PhoneVolume::Absent.to_bits(), PhoneVolume::SILENT_BITS);
        assert_eq!(PhoneVolume::from_bits(PhoneVolume::SILENT_BITS).fine(), 0);
    }

    #[test]
    fn a_word_naming_no_state_reads_as_silence()
    {
        for bits in 0..=u16::MAX
        {
            let state = PhoneVolume::from_bits(bits);
            assert_eq!(PhoneVolume::from_bits(state.to_bits()), state);
            if bits & !PhoneVolume::VALUE_MASK > PhoneVolume::SET_BITS
            {
                assert_eq!(state, PhoneVolume::Absent, "word {bits:#06x} named a level");
            }
        }
    }

    #[test]
    fn every_pair_of_controls_names_a_volume()
    {
        for coarse in 0..=u8::MAX
        {
            for fine in 0..=u8::MAX
            {
                let phone = PhoneVolume::from_bits(PhoneVolume::SET_BITS | u16::from(fine));
                let pair = volume_of(coarse, phone);
                if coarse <= COARSE_MAX
                {
                    assert_eq!(pair, volume(coarse, fine & FINE_MAX));
                }
                else
                {
                    assert_eq!(pair, Volume::MUTED);
                }
            }
        }
    }

    /// Runs the schedule for `total_ms` at `period_ms` per poll and returns
    /// every message sent, with the millisecond it left on.
    fn run(uplink: &mut Uplink, total_ms: u32, period_ms: u32, volume: Volume) -> Vec<(u32, ToDsp)>
    {
        let mut sent = Vec::new();
        let mut now = 0;
        while now < total_ms
        {
            now += period_ms;
            if let Some(message) = uplink.poll(period_ms, volume)
            {
                sent.push((now, message));
            }
        }
        sent
    }

    #[test]
    fn the_first_poll_sends_the_heartbeat_and_the_volume_follows()
    {
        let mut uplink = Uplink::at_power_up();
        let level = volume(COARSE_DEFAULT, FINE_MAX);
        let sent = run(&mut uplink, 20, 5, level);
        assert_eq!(sent.first().map(|pair| pair.1), Some(ToDsp::Heartbeat));
        assert_eq!(sent.get(1).map(|pair| pair.1), Some(ToDsp::SetVolume(level)));
        assert_eq!(sent.first().map(|pair| pair.0), Some(5));
        assert_eq!(sent.get(1).map(|pair| pair.0), Some(10));
    }

    #[test]
    fn a_change_goes_out_on_the_first_poll_that_may_send()
    {
        let mut uplink = Uplink::at_power_up();
        let quiet = volume(COARSE_DEFAULT, FINE_MAX);
        assert_eq!(uplink.poll(5, quiet), Some(ToDsp::Heartbeat));
        assert_eq!(uplink.poll(5, quiet), Some(ToDsp::SetVolume(quiet)));

        let loud = volume(COARSE_MAX, FINE_MAX);
        assert_eq!(uplink.poll(1, loud), None, "the last frame left 1 ms ago");
        assert_eq!(uplink.poll(4, loud), Some(ToDsp::SetVolume(loud)));
    }

    #[test]
    fn a_volume_changing_on_every_poll_never_starves_the_heartbeat()
    {
        // A knob under the hand, or two input pins floating with no external
        // pull, moves the step on nearly every poll. Only a heartbeat restarts
        // the silence measure, so a schedule that lets the volume take every
        // turn cuts the sound after `LINK_SILENCE_LIMIT_MS`.
        for period_ms in [1, 5, 6, 10, 13]
        {
            let mut uplink = Uplink::at_power_up();
            let mut now = 0;
            let mut turn = 0;
            let mut last_beat = 0;
            let mut worst = 0;
            let mut beats = 0;

            while now < 4 * LINK_SILENCE_LIMIT_MS
            {
                now += period_ms;
                turn += 1;
                let level = volume(u8::try_from(turn % 14).unwrap_or(0), FINE_MAX);
                if uplink.poll(period_ms, level) == Some(ToDsp::Heartbeat)
                {
                    worst = worst.max(now - last_beat);
                    last_beat = now;
                    beats += 1;
                }
            }
            worst = worst.max(now - last_beat);

            assert!(beats > 0, "polls every {period_ms}ms sent no heartbeat at all");
            assert!
            (
                worst * 4 <= LINK_SILENCE_LIMIT_MS,
                "polls every {period_ms}ms left {worst}ms without a heartbeat"
            );
        }
    }

    #[test]
    fn the_volume_and_the_heartbeat_both_repeat_inside_the_period()
    {
        let mut uplink = Uplink::at_power_up();
        let level = volume(COARSE_DEFAULT, 0x40);
        let sent = run(&mut uplink, 10_000, 5, level);

        let mut last_volume = 0;
        let mut last_beat = 0;
        for (now, message) in &sent
        {
            match message
            {
                ToDsp::SetVolume(_) =>
                {
                    assert!
                    (
                        now - last_volume <= RESEND_PERIOD_MS + MIN_FRAME_GAP_MS + 5,
                        "{}ms without a volume", now - last_volume
                    );
                    last_volume = *now;
                }
                ToDsp::Heartbeat =>
                {
                    assert!
                    (
                        now - last_beat <= RESEND_PERIOD_MS + MIN_FRAME_GAP_MS + 5,
                        "{}ms without a heartbeat", now - last_beat
                    );
                    last_beat = *now;
                }
                ToDsp::SelectPreset(_) => panic!("the interface board sends no preset"),
            }
        }
        assert!(last_volume >= 9_000 && last_beat >= 9_000, "a stream dried up");
    }

    #[test]
    fn the_silence_the_processing_board_measures_stays_far_under_its_limit()
    {
        for period_ms in [1, 5, 7, 10, 20, 100]
        {
            let mut uplink = Uplink::at_power_up();
            let level = volume(COARSE_DEFAULT, FINE_MAX);
            let sent = run(&mut uplink, 60_000, period_ms, level);

            let mut last_beat = 0;
            let mut worst = 0;
            for (now, message) in &sent
            {
                if *message == ToDsp::Heartbeat
                {
                    worst = worst.max(now - last_beat);
                    last_beat = *now;
                }
            }
            worst = worst.max(60_000 - last_beat);
            assert!
            (
                worst * 4 <= LINK_SILENCE_LIMIT_MS,
                "polls every {period_ms}ms left {worst}ms without a heartbeat"
            );
        }
    }

    #[test]
    fn no_window_of_the_receiver_ever_takes_more_than_it_holds()
    {
        // The processing board polls its receiver once per served block, and
        // `RX_POLL_PERIOD_MS` is the length of the longer of the two. Every
        // window of that length is measured, at several poll periods and with
        // a volume that changes on every poll.
        for period_ms in [1, 2, 5, 6, 10, 13, 250]
        {
            let mut uplink = Uplink::at_power_up();
            let mut now = 0;
            let mut sent: Vec<(u32, usize)> = Vec::new();
            let mut turn = 0;

            while now < 30_000
            {
                now += period_ms;
                turn += 1;
                let level = volume(u8::try_from(turn % 14).unwrap_or(0), FINE_MAX);
                if let Some(message) = uplink.poll(period_ms, level)
                {
                    sent.push((now, frame_len(message)));
                }
            }

            for (index, (start, _)) in sent.iter().enumerate()
            {
                let held: usize = sent
                    .get(index..)
                    .unwrap_or(&[])
                    .iter()
                    .take_while(|(at, _)| at <= &(start + RX_POLL_PERIOD_MS))
                    .map(|(_, bytes)| *bytes)
                    .sum();
                assert!
                (
                    held <= RX_HOLD_BYTES as usize,
                    "polls every {period_ms}ms put {held} bytes in one block"
                );
            }
        }
    }

    #[test]
    fn a_poll_that_arrives_late_sends_one_frame_and_not_a_burst()
    {
        let mut uplink = Uplink::at_power_up();
        let level = volume(COARSE_DEFAULT, FINE_MAX);
        assert!(uplink.poll(1, level).is_some());
        assert_eq!(uplink.poll(0, level), None);
        assert!(uplink.poll(10_000, level).is_some());
        assert_eq!(uplink.poll(0, level), None);
        assert!(uplink.poll(u32::MAX, level).is_some());
        assert_eq!(uplink.poll(0, level), None);
    }
}
