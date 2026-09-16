//! Control protocol between the two boards.
//!
//! Control traffic never rides inside the audio stream. It is allowed to wait,
//! so the wire format buys self synchronisation and error detection rather than
//! throughput.
//!
//! A frame is laid out as:
//!
//! ```text
//! 0xA5 | length | payload | checksum low | checksum high
//! ```
//!
//! `length` counts the payload alone. The checksum is CRC-16/CCITT-FALSE over
//! the length byte and the payload, and it is what decides whether a frame is
//! accepted.
//!
//! # Reading a stream
//!
//! `decode` reports what a frame violates, never how far to advance, because
//! past `Truncated` the length byte is unverified.
//!
//! - `Truncated` means wait for more bytes and decode the same offset again.
//! - Any other error means discard `resync_offset` bytes and decode again.
//!
//! Every routine here works on caller owned buffers and allocates nothing.

use core::num::NonZeroUsize;

use crate::constants::FULL_SCALE;

/// Start of frame marker.
const START_OF_FRAME: u8 = 0xA5;

/// Bytes a frame carries besides its payload: start, length, two of checksum.
const FRAME_OVERHEAD: usize = 4;

/// Fields a heartbeat carries. The tag alone is the whole message.
const HEARTBEAT_FIELDS: usize = 0;

/// Fields a volume message carries: the coarse and fine controls.
const VOLUME_FIELDS: usize = 2;

/// Fields a preset message carries: the preset code.
const PRESET_FIELDS: usize = 1;

/// Fields a state message carries: the state code.
const STATE_FIELDS: usize = 1;

/// Fields a fault message carries: the fault code.
const FAULT_FIELDS: usize = 1;

/// Returns the payload length of a message carrying `fields` fields.
const fn payload_len(fields: usize) -> usize
{
    fields.saturating_add(1)
}

/// Longest payload any message produces: a tag byte and its fields.
///
/// `SetVolume` is the widest message. The assertions below hold every other one
/// under it.
pub const MAX_PAYLOAD_LEN: usize = payload_len(VOLUME_FIELDS);

/// Longest frame any message produces.
///
/// A caller sizes its transmit buffer from this.
pub const MAX_FRAME_LEN: usize = MAX_PAYLOAD_LEN + FRAME_OVERHEAD;

/// Highest step of the coarse control on the cabinet, which sits at 0 dB.
///
/// Step `n` stands for `(n - 13) x 3` dB and step 0 mutes, so the control
/// covers the 36 dB below full scale in 3 dB steps.
pub const COARSE_MAX: u8 = 13;

/// Coarse step the cabinet starts on at power up: -24 dB.
///
/// The encoder is incremental and no step survives a power cycle.
pub const COARSE_DEFAULT: u8 = 5;

/// Highest value of the fine control, which sits at 0 dB.
///
/// AVRCP 1.6.3 section 6.13.1 carries absolute volume as one octet from 0x00 to
/// 0x7F. Value `v` stands for `(v - 127) x 24 / 127` dB and 0 mutes.
pub const FINE_MAX: u8 = 0x7F;

/// Bit 7 of an AVRCP absolute volume octet.
///
/// AVRCP 1.6.3 section 6.13.1 reserves it for future additions, and section
/// 1.3.2.1 tells a receiver to ignore such bits.
const ABSOLUTE_VOLUME_RFA: u8 = 0x80;

/// Ratio of the linear gains of two neighbouring coarse steps, `10^(-3/20)`.
///
/// The exhaustive test of the volume law holds the literal to the formula.
const COARSE_STEP_RATIO: f64 = 0.707_945_784_384_137_9;

/// Ratio of the linear gains of two neighbouring fine values,
/// `10^(-24/(127 x 20))`.
///
/// The exhaustive test of the volume law holds the literal to the formula.
const FINE_STEP_RATIO: f64 = 0.978_478_260_521_376_1;

/// Linear gain of each coarse step, 0 at step 0 and 1 at `COARSE_MAX`.
const COARSE_GAIN: [f32; COARSE_MAX as usize + 1] = gain_table(COARSE_STEP_RATIO);

/// Linear gain of each fine value, 0 at value 0 and 1 at `FINE_MAX`.
const FINE_GAIN: [f32; FINE_MAX as usize + 1] = gain_table(FINE_STEP_RATIO);

/// Builds a table whose last entry is 1, each entry below it `ratio` times the
/// one above, and whose first entry is 0.
///
/// The powers accumulate in double precision and each entry narrows once, so
/// an entry carries the rounding of one narrowing over a double precision
/// error below 1e-13. No exponentiation runs on the target.
#[expect
(
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::cast_possible_truncation,
    reason = "the table is built in a constant, where an index out of bounds \
              fails the build, and narrowing to single precision is what the \
              audio path carries"
)]
const fn gain_table<const N: usize>(ratio: f64) -> [f32; N]
{
    let mut table = [0.0_f32; N];
    let mut power = 1.0_f64;
    let mut index = N;
    while index > 1
    {
        index -= 1;
        table[index] = power as f32;
        power *= ratio;
    }
    table
}

/// Moves a coarse step by `steps` encoder detents, held to 0 and `COARSE_MAX`.
///
/// A negative count turns the level down. A `coarse` above `COARSE_MAX` counts
/// from `COARSE_MAX`.
#[must_use]
pub fn step_coarse(coarse: u8, steps: i32) -> u8
{
    let ceiling = i32::from(COARSE_MAX);
    let moved = i32::from(coarse.min(COARSE_MAX))
        .saturating_add(steps)
        .clamp(0, ceiling);
    u8::try_from(moved).unwrap_or(COARSE_MAX)
}

/// Returns the fine value an AVRCP absolute volume octet carries.
///
/// AVRCP 1.6.3 section 6.13.1: "The top bit (bit 7) is reserved for future
/// addition (RFA)." Section 1.3.2.1: "Receivers shall ignore these bits." The
/// remaining seven bits are the value, 0x00 for 0 percent and 0x7F for 100.
#[must_use]
pub const fn fine_from_absolute_volume(octet: u8) -> u8
{
    octet & !ABSOLUTE_VOLUME_RFA
}

// Tags of the two directions occupy disjoint ranges, so a frame that arrives on
// the wrong link fails as an unknown tag rather than decoding into a plausible
// command.
const TAG_TO_DSP_HEARTBEAT: u8 = 0x01;
const TAG_TO_DSP_VOLUME: u8 = 0x02;
const TAG_TO_DSP_PRESET: u8 = 0x03;

const TAG_TO_CTRL_HEARTBEAT: u8 = 0x81;
const TAG_TO_CTRL_STATE: u8 = 0x82;
const TAG_TO_CTRL_FAULT: u8 = 0x83;

const _: () = assert!
(
    payload_len(HEARTBEAT_FIELDS) <= MAX_PAYLOAD_LEN
        && payload_len(PRESET_FIELDS) <= MAX_PAYLOAD_LEN
        && payload_len(STATE_FIELDS) <= MAX_PAYLOAD_LEN
        && payload_len(FAULT_FIELDS) <= MAX_PAYLOAD_LEN,
    "no message is wider than the bound every frame buffer is sized from"
);

const _: () = assert!
(
    MAX_PAYLOAD_LEN <= u8::MAX as usize,
    "the length byte carries the payload length"
);

/// Reason a frame could not be built or read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError
{
    /// The output buffer is shorter than the frame to write.
    BufferTooSmall,
    /// The first byte is not the start of frame marker.
    NoStartOfFrame,
    /// The input ends before the announced frame does.
    Truncated,
    /// The announced payload length is zero or above `MAX_PAYLOAD_LEN`.
    BadLength,
    /// The checksum does not match the payload.
    BadChecksum,
    /// The tag is not one this direction defines.
    UnknownTag,
    /// The field count does not match the tag, or a field is out of range.
    BadPayload,
}

/// Equalisation preset the user selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Preset
{
    /// Protective crossover alone, no comfort equalisation on top.
    Flat = 0x00,
    /// Indoor listening.
    Home = 0x01,
    /// Outdoor listening.
    Garden = 0x02,
    /// Lifted low band.
    BassBoosted = 0x03,
}

impl Preset
{
    /// Returns the wire code of the preset.
    pub(crate) fn code(self) -> u8
    {
        self as u8
    }

    /// Reads a preset from its wire code.
    ///
    /// # Errors
    ///
    /// Returns `BadPayload` when the code names no preset.
    pub(crate) fn from_code(code: u8) -> Result<Self, ProtocolError>
    {
        match code
        {
            0x00 => Ok(Self::Flat),
            0x01 => Ok(Self::Home),
            0x02 => Ok(Self::Garden),
            0x03 => Ok(Self::BassBoosted),
            _ => Err(ProtocolError::BadPayload),
        }
    }
}

/// State the processing board reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DspState
{
    /// Outputs muted. Initialisation has not met all of its conditions.
    Muted = 0x00,
    /// Converters unmuted and the chain processing audio.
    Running = 0x01,
    /// Music ramped down and the alarm pattern playing, after the link fell
    /// silent.
    Alarm = 0x02,
    /// A fault drove the outputs back to mute.
    Faulted = 0x03,
}

impl DspState
{
    /// Returns the wire code of the state.
    pub(crate) fn code(self) -> u8
    {
        self as u8
    }

    /// Reads a state from its wire code.
    ///
    /// # Errors
    ///
    /// Returns `BadPayload` when the code names no state.
    pub(crate) fn from_code(code: u8) -> Result<Self, ProtocolError>
    {
        match code
        {
            0x00 => Ok(Self::Muted),
            0x01 => Ok(Self::Running),
            0x02 => Ok(Self::Alarm),
            0x03 => Ok(Self::Faulted),
            _ => Err(ProtocolError::BadPayload),
        }
    }
}

/// Fault the processing board reports.
///
/// Each variant names a condition that mutes the outputs or keeps them muted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Fault
{
    /// An audio clock lost lock, so the sample rate is no longer the one the
    /// filters were built for.
    ClockUnlocked = 0x00,
    /// The transfer interrupt stopped arriving.
    TransferStalled = 0x01,
    /// A converter transfer ran dry.
    OutputUnderrun = 0x03,
    /// The audio input overflowed its buffer.
    InputOverrun = 0x04,
    /// The audio input stopped delivering samples while the clocks still run,
    /// as the interface board rebooting mid stream would.
    InputSilent = 0x05,
}

impl Fault
{
    /// Returns the wire code of the fault.
    pub(crate) fn code(self) -> u8
    {
        self as u8
    }

    /// Reads a fault from its wire code.
    ///
    /// # Errors
    ///
    /// Returns `BadPayload` when the code names no fault.
    pub(crate) fn from_code(code: u8) -> Result<Self, ProtocolError>
    {
        match code
        {
            0x00 => Ok(Self::ClockUnlocked),
            0x01 => Ok(Self::TransferStalled),
            0x03 => Ok(Self::OutputUnderrun),
            0x04 => Ok(Self::InputOverrun),
            0x05 => Ok(Self::InputSilent),
            _ => Err(ProtocolError::BadPayload),
        }
    }
}

/// The two volume controls: a coarse step from the cabinet and a fine value
/// from AVRCP absolute volume.
///
/// The pair travels in one frame because the processing board sums both in
/// decibels into a single digital gain. Split across two messages, one could
/// arrive without the other and move the level twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Volume
{
    coarse: u8,
    fine: u8,
}

impl Volume
{
    /// Both controls at zero, which is the level the chain starts from.
    pub const MUTED: Self = Self
    {
        coarse: 0,
        fine: 0,
    };

    /// Builds a volume pair.
    ///
    /// # Errors
    ///
    /// Returns `BadPayload` when `coarse` exceeds `COARSE_MAX` or `fine`
    /// exceeds `FINE_MAX`.
    pub fn new(coarse: u8, fine: u8) -> Result<Self, ProtocolError>
    {
        if coarse > COARSE_MAX || fine > FINE_MAX
        {
            return Err(ProtocolError::BadPayload);
        }
        let volume = Self
        {
            coarse,
            fine,
        };
        Ok(volume)
    }

    /// Returns the coarse setting from the cabinet control.
    pub(crate) fn coarse(self) -> u8
    {
        self.coarse
    }

    /// Returns the fine setting from AVRCP absolute volume.
    pub(crate) fn fine(self) -> u8
    {
        self.fine
    }

    /// Returns the linear gain of both controls, held at `FULL_SCALE`.
    ///
    /// The gain is `10^((coarse dB + fine dB) / 20)`: exactly 1 with both
    /// controls at their maximum, and exactly 0 with either at zero.
    ///
    /// This is the raw setting. `crate::control::ControlState` turns it into a
    /// value the chain applies, because a setting applied as a step pops.
    pub(crate) fn linear_gain(self) -> f32
    {
        let coarse = COARSE_GAIN.get(usize::from(self.coarse)).copied().unwrap_or(0.0);
        let fine = FINE_GAIN.get(usize::from(self.fine)).copied().unwrap_or(0.0);
        let gain = coarse * fine;
        if gain > FULL_SCALE
        {
            return FULL_SCALE;
        }
        gain
    }
}

/// A message that travels as one frame.
pub trait Message: Sized
{
    /// Writes the message into `out` as a complete frame.
    ///
    /// Returns the number of bytes written.
    ///
    /// # Errors
    ///
    /// Returns `BufferTooSmall` when `out` cannot hold the frame.
    fn encode_into(&self, out: &mut [u8]) -> Result<usize, ProtocolError>;

    /// Reads the frame that starts at the beginning of `input`.
    ///
    /// Returns the message and the number of bytes it consumed, so a caller
    /// reading a stream can advance past it.
    ///
    /// # Errors
    ///
    /// Returns `NoStartOfFrame`, `Truncated`, `BadLength`, `BadChecksum`,
    /// `UnknownTag` or `BadPayload` according to what the input violates.
    ///
    /// `Truncated` alone leaves the same input worth decoding again. On every
    /// other error the caller discards `resync_offset` bytes.
    fn decode(input: &[u8]) -> Result<(Self, usize), ProtocolError>;
}

/// Message the interface board sends to the processing board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToDsp
{
    /// Proof the link is alive.
    Heartbeat,
    /// New setting of the two volume controls.
    SetVolume(Volume),
    /// Preset the user selected.
    SelectPreset(Preset),
}

impl Message for ToDsp
{
    fn encode_into(&self, out: &mut [u8]) -> Result<usize, ProtocolError>
    {
        let mut payload = [0_u8; MAX_PAYLOAD_LEN];
        let used = match *self
        {
            Self::Heartbeat =>
            {
                let fields: [u8; HEARTBEAT_FIELDS] = [];
                build_payload(&mut payload, TAG_TO_DSP_HEARTBEAT, &fields)?
            }
            Self::SetVolume(volume) =>
            {
                let fields: [u8; VOLUME_FIELDS] = [volume.coarse(), volume.fine()];
                build_payload(&mut payload, TAG_TO_DSP_VOLUME, &fields)?
            }
            Self::SelectPreset(preset) =>
            {
                let fields: [u8; PRESET_FIELDS] = [preset.code()];
                build_payload(&mut payload, TAG_TO_DSP_PRESET, &fields)?
            }
        };
        let body = payload.get(..used).ok_or(ProtocolError::BadPayload)?;
        frame_into(body, out)
    }

    fn decode(input: &[u8]) -> Result<(Self, usize), ProtocolError>
    {
        let (payload, consumed) = payload_of(input)?;
        let [tag, fields @ ..] = payload
        else
        {
            return Err(ProtocolError::BadLength);
        };
        let message = match *tag
        {
            TAG_TO_DSP_HEARTBEAT =>
            {
                if !fields.is_empty()
                {
                    return Err(ProtocolError::BadPayload);
                }
                Self::Heartbeat
            }
            TAG_TO_DSP_VOLUME =>
            {
                let [coarse, fine] = fields
                else
                {
                    return Err(ProtocolError::BadPayload);
                };
                Self::SetVolume(Volume::new(*coarse, *fine)?)
            }
            TAG_TO_DSP_PRESET =>
            {
                let [code] = fields
                else
                {
                    return Err(ProtocolError::BadPayload);
                };
                Self::SelectPreset(Preset::from_code(*code)?)
            }
            _ => return Err(ProtocolError::UnknownTag),
        };
        Ok((message, consumed))
    }
}

/// Message the processing board sends to the interface board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToCtrl
{
    /// Proof the link is alive.
    Heartbeat,
    /// Current state of the processing chain.
    State(DspState),
    /// A fault the interface board reports to the user.
    Fault(Fault),
}

impl Message for ToCtrl
{
    fn encode_into(&self, out: &mut [u8]) -> Result<usize, ProtocolError>
    {
        let mut payload = [0_u8; MAX_PAYLOAD_LEN];
        let used = match *self
        {
            Self::Heartbeat =>
            {
                let fields: [u8; HEARTBEAT_FIELDS] = [];
                build_payload(&mut payload, TAG_TO_CTRL_HEARTBEAT, &fields)?
            }
            Self::State(state) =>
            {
                let fields: [u8; STATE_FIELDS] = [state.code()];
                build_payload(&mut payload, TAG_TO_CTRL_STATE, &fields)?
            }
            Self::Fault(fault) =>
            {
                let fields: [u8; FAULT_FIELDS] = [fault.code()];
                build_payload(&mut payload, TAG_TO_CTRL_FAULT, &fields)?
            }
        };
        let body = payload.get(..used).ok_or(ProtocolError::BadPayload)?;
        frame_into(body, out)
    }

    fn decode(input: &[u8]) -> Result<(Self, usize), ProtocolError>
    {
        let (payload, consumed) = payload_of(input)?;
        let [tag, fields @ ..] = payload
        else
        {
            return Err(ProtocolError::BadLength);
        };
        let message = match *tag
        {
            TAG_TO_CTRL_HEARTBEAT =>
            {
                if !fields.is_empty()
                {
                    return Err(ProtocolError::BadPayload);
                }
                Self::Heartbeat
            }
            TAG_TO_CTRL_STATE =>
            {
                let [code] = fields
                else
                {
                    return Err(ProtocolError::BadPayload);
                };
                Self::State(DspState::from_code(*code)?)
            }
            TAG_TO_CTRL_FAULT =>
            {
                let [code] = fields
                else
                {
                    return Err(ProtocolError::BadPayload);
                };
                Self::Fault(Fault::from_code(*code)?)
            }
            _ => return Err(ProtocolError::UnknownTag),
        };
        Ok((message, consumed))
    }
}

/// Returns the bytes to discard before scanning `input` for a frame again.
///
/// The offset lands on the next start byte at index 1 or beyond, or on the end
/// of the input when there is none. Every offset is at least one, so a loop
/// that advances by it makes progress. `None` says the input is empty.
///
/// A caller uses this after any `decode` error other than `Truncated`, since
/// the announced length of a rejected frame may walk over the genuine frame
/// behind it.
///
/// The start byte occurs inside valid frames, so a scan can stop on a false
/// start. The checksum is what rejects the frame that follows.
#[must_use]
pub fn resync_offset(input: &[u8]) -> Option<NonZeroUsize>
{
    // The tail is absent for an empty input only.
    let tail = input.get(1..)?;
    let offset = match tail.iter().position(|byte| *byte == START_OF_FRAME)
    {
        Some(offset) => offset.saturating_add(1),
        None => input.len(),
    };
    // Both arms are at least one, since the input holds the byte the scan
    // skipped.
    NonZeroUsize::new(offset)
}

/// Writes a tag and its fields into `buffer` and returns the length used.
fn build_payload
(
    buffer: &mut [u8; MAX_PAYLOAD_LEN],
    tag: u8,
    fields: &[u8],
) -> Result<usize, ProtocolError>
{
    let used = payload_len(fields.len());
    let slot = buffer.get_mut(..used).ok_or(ProtocolError::BadPayload)?;
    let [tag_slot, rest @ ..] = slot
    else
    {
        return Err(ProtocolError::BadPayload);
    };
    *tag_slot = tag;
    rest.copy_from_slice(fields);
    Ok(used)
}

/// Wraps `payload` in a frame written to `out` and returns the frame length.
fn frame_into(payload: &[u8], out: &mut [u8]) -> Result<usize, ProtocolError>
{
    let length = u8::try_from(payload.len()).map_err(|_| ProtocolError::BadLength)?;
    if payload.is_empty() || payload.len() > MAX_PAYLOAD_LEN
    {
        return Err(ProtocolError::BadLength);
    }

    let total = payload.len().saturating_add(FRAME_OVERHEAD);
    let frame = out.get_mut(..total).ok_or(ProtocolError::BufferTooSmall)?;
    let [start, length_slot, rest @ ..] = frame
    else
    {
        return Err(ProtocolError::BufferTooSmall);
    };
    let (body, checksum) = rest
        .split_at_mut_checked(payload.len())
        .ok_or(ProtocolError::BufferTooSmall)?;

    *start = START_OF_FRAME;
    *length_slot = length;
    body.copy_from_slice(payload);
    checksum.copy_from_slice(&checksum_of(length, payload).to_le_bytes());
    Ok(total)
}

/// Returns the payload of the frame starting at `input`, and the frame length.
fn payload_of(input: &[u8]) -> Result<(&[u8], usize), ProtocolError>
{
    // The start byte decides first, so a reader holding one byte learns whether
    // to discard it or to wait for the rest of a frame.
    let [start, after_start @ ..] = input
    else
    {
        return Err(ProtocolError::Truncated);
    };
    if *start != START_OF_FRAME
    {
        return Err(ProtocolError::NoStartOfFrame);
    }

    let [length, rest @ ..] = after_start
    else
    {
        return Err(ProtocolError::Truncated);
    };

    let payload_len = usize::from(*length);
    if payload_len == 0 || payload_len > MAX_PAYLOAD_LEN
    {
        return Err(ProtocolError::BadLength);
    }

    let (payload, tail) = rest
        .split_at_checked(payload_len)
        .ok_or(ProtocolError::Truncated)?;
    let [low, high, ..] = tail
    else
    {
        return Err(ProtocolError::Truncated);
    };
    if u16::from_le_bytes([*low, *high]) != checksum_of(*length, payload)
    {
        return Err(ProtocolError::BadChecksum);
    }

    Ok((payload, payload_len + FRAME_OVERHEAD))
}

/// Computes CRC-16/CCITT-FALSE over the length byte and the payload.
fn checksum_of(length: u8, payload: &[u8]) -> u16
{
    let mut crc: u16 = 0xFFFF;
    crc = fold(crc, length);
    for byte in payload
    {
        crc = fold(crc, *byte);
    }
    crc
}

/// Folds one byte into a CRC-16/CCITT-FALSE accumulator.
fn fold(mut crc: u16, byte: u8) -> u16
{
    crc ^= u16::from(byte) << 8;
    for _ in 0..8
    {
        if crc & 0x8000 == 0
        {
            crc <<= 1;
        }
        else
        {
            crc = (crc << 1) ^ 0x1021;
        }
    }
    crc
}

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, which is the one place the
    // no-panic rule does not hold.
    #![allow(clippy::panic)]
    #![allow(clippy::indexing_slicing)]

    use super::*;

    /// Encodes a message and decodes it back, checking both halves agree.
    fn round_trip<M>(message: M)
    where
        M: Message + Copy + core::fmt::Debug + PartialEq,
    {
        let mut buffer = [0_u8; MAX_FRAME_LEN];
        let written = match message.encode_into(&mut buffer)
        {
            Ok(written) => written,
            Err(error) => panic!("encoding failed: {error:?}"),
        };
        assert!(written <= MAX_FRAME_LEN);

        match M::decode(&buffer)
        {
            Ok((decoded, consumed)) =>
            {
                assert_eq!(decoded, message);
                assert_eq!(consumed, written);
            }
            Err(error) => panic!("decoding failed: {error:?}"),
        }
    }

    /// Builds a frame for a message, returning the buffer and its length.
    fn encoded<M: Message>(message: &M) -> ([u8; MAX_FRAME_LEN], usize)
    {
        let mut buffer = [0_u8; MAX_FRAME_LEN];
        let written = match message.encode_into(&mut buffer)
        {
            Ok(written) => written,
            Err(error) => panic!("encoding failed: {error:?}"),
        };
        (buffer, written)
    }

    #[test]
    fn every_to_dsp_message_round_trips()
    {
        round_trip(ToDsp::Heartbeat);
        round_trip(ToDsp::SelectPreset(Preset::Flat));
        round_trip(ToDsp::SelectPreset(Preset::Home));
        round_trip(ToDsp::SelectPreset(Preset::Garden));
        round_trip(ToDsp::SelectPreset(Preset::BassBoosted));
        for (coarse, fine) in [(0, 0), (COARSE_DEFAULT, 0x40), (COARSE_MAX, FINE_MAX)]
        {
            match Volume::new(coarse, fine)
            {
                Ok(volume) => round_trip(ToDsp::SetVolume(volume)),
                Err(error) => panic!("volume rejected: {error:?}"),
            }
        }
    }

    #[test]
    fn every_to_ctrl_message_round_trips()
    {
        round_trip(ToCtrl::Heartbeat);
        round_trip(ToCtrl::State(DspState::Muted));
        round_trip(ToCtrl::State(DspState::Running));
        round_trip(ToCtrl::State(DspState::Alarm));
        round_trip(ToCtrl::State(DspState::Faulted));
        round_trip(ToCtrl::Fault(Fault::ClockUnlocked));
        round_trip(ToCtrl::Fault(Fault::TransferStalled));
        round_trip(ToCtrl::Fault(Fault::OutputUnderrun));
        round_trip(ToCtrl::Fault(Fault::InputOverrun));
        round_trip(ToCtrl::Fault(Fault::InputSilent));
    }

    #[test]
    fn a_frame_of_the_other_direction_is_refused()
    {
        let (buffer, _) = encoded(&ToCtrl::State(DspState::Running));
        assert_eq!(ToDsp::decode(&buffer), Err(ProtocolError::UnknownTag));

        let (buffer, _) = encoded(&ToDsp::Heartbeat);
        assert_eq!(ToCtrl::decode(&buffer), Err(ProtocolError::UnknownTag));
    }

    #[test]
    fn a_flipped_bit_fails_the_checksum()
    {
        let (mut buffer, written) = encoded(&ToCtrl::Fault(Fault::TransferStalled));
        let payload_index = 2;
        assert!(payload_index < written);
        match buffer.get_mut(payload_index)
        {
            Some(byte) => *byte ^= 0x01,
            None => panic!("frame shorter than its own payload"),
        }
        assert_eq!(ToCtrl::decode(&buffer), Err(ProtocolError::BadChecksum));
    }

    #[test]
    fn a_wrong_start_byte_is_refused()
    {
        let (mut buffer, _) = encoded(&ToDsp::Heartbeat);
        match buffer.first_mut()
        {
            Some(byte) => *byte = 0x00,
            None => panic!("empty frame"),
        }
        assert_eq!(ToDsp::decode(&buffer), Err(ProtocolError::NoStartOfFrame));
    }

    #[test]
    fn one_wrong_byte_reads_as_a_discard_rather_than_a_wait()
    {
        // A reader must be able to tell "throw this byte away" from "the frame
        // has not arrived yet" on the shortest possible input.
        assert_eq!(ToDsp::decode(&[0x00]), Err(ProtocolError::NoStartOfFrame));
        assert_eq!(ToDsp::decode(&[]), Err(ProtocolError::Truncated));
        assert_eq!(ToDsp::decode(&[START_OF_FRAME]), Err(ProtocolError::Truncated));
    }

    #[test]
    fn a_frame_cut_short_is_refused()
    {
        let (buffer, written) = encoded(&ToDsp::SelectPreset(Preset::Home));
        for cut in 0..written
        {
            match buffer.get(..cut)
            {
                Some(partial) => assert!(ToDsp::decode(partial).is_err()),
                None => panic!("cut beyond the frame"),
            }
        }
    }

    #[test]
    fn an_announced_length_beyond_the_maximum_is_refused()
    {
        let frame = [START_OF_FRAME, 0xFF, 0x00, 0x00, 0x00];
        assert_eq!(ToDsp::decode(&frame), Err(ProtocolError::BadLength));

        let empty_payload = [START_OF_FRAME, 0x00, 0x00, 0x00];
        assert_eq!(ToDsp::decode(&empty_payload), Err(ProtocolError::BadLength));
    }

    #[test]
    fn a_short_output_buffer_is_refused()
    {
        let mut buffer = [0_u8; FRAME_OVERHEAD];
        assert_eq!
        (
            ToDsp::Heartbeat.encode_into(&mut buffer),
            Err(ProtocolError::BufferTooSmall)
        );
    }

    #[test]
    fn a_tag_carrying_the_wrong_field_count_is_refused()
    {
        // One field too many, then one too few. Each checksum is recomputed, so
        // the frame fails on its shape rather than on its checksum.
        for payload in [&[TAG_TO_DSP_HEARTBEAT, 0x00][..], &[TAG_TO_DSP_PRESET][..]]
        {
            let mut frame = [0_u8; MAX_FRAME_LEN];
            match frame_into(payload, &mut frame)
            {
                Ok(_) => (),
                Err(error) => panic!("framing failed: {error:?}"),
            }
            assert_eq!(ToDsp::decode(&frame), Err(ProtocolError::BadPayload));
        }
    }

    /// Builds a volume pair, failing the test rather than returning an error.
    fn volume(coarse: u8, fine: u8) -> Volume
    {
        match Volume::new(coarse, fine)
        {
            Ok(volume) => volume,
            Err(error) => panic!("volume rejected: {error:?}"),
        }
    }

    /// Returns the gain the volume law defines, in double precision.
    ///
    /// The formula is written out here from the decibel figures, so the tables
    /// and the ratios they are built from are what the tests hold against it.
    fn reference_gain(coarse: u8, fine: u8) -> f64
    {
        if coarse == 0 || fine == 0
        {
            return 0.0;
        }
        let coarse_db = (f64::from(coarse) - 13.0) * 3.0;
        let fine_db = (f64::from(fine) - 127.0) * 24.0 / 127.0;
        10.0_f64.powf((coarse_db + fine_db) / 20.0)
    }

    #[test]
    fn a_volume_outside_either_range_is_refused()
    {
        assert_eq!(Volume::new(COARSE_MAX + 1, 0), Err(ProtocolError::BadPayload));
        assert_eq!(Volume::new(u8::MAX, 0), Err(ProtocolError::BadPayload));
        assert_eq!(Volume::new(0, FINE_MAX + 1), Err(ProtocolError::BadPayload));
        assert_eq!(Volume::new(0, u8::MAX), Err(ProtocolError::BadPayload));
        assert!(Volume::new(COARSE_MAX, FINE_MAX).is_ok());
    }

    #[test]
    fn a_volume_frame_with_a_coarse_step_above_the_range_is_refused()
    {
        // The checksum is recomputed, so the frame fails on the field rather
        // than on its checksum. The step just below the bound decodes.
        for (coarse, expected) in
        [
            (COARSE_MAX, None),
            (COARSE_MAX + 1, Some(ProtocolError::BadPayload)),
            (0x7F, Some(ProtocolError::BadPayload)),
        ]
        {
            let mut frame = [0_u8; MAX_FRAME_LEN];
            match frame_into(&[TAG_TO_DSP_VOLUME, coarse, FINE_MAX], &mut frame)
            {
                Ok(_) => (),
                Err(error) => panic!("framing failed: {error:?}"),
            }
            match (ToDsp::decode(&frame), expected)
            {
                (Err(error), Some(refusal)) => assert_eq!(error, refusal),
                (Ok((ToDsp::SetVolume(decoded), _)), None) =>
                {
                    assert_eq!(decoded, volume(coarse, FINE_MAX));
                }
                (outcome, _) => panic!("coarse step {coarse} decoded as {outcome:?}"),
            }
        }
    }

    #[test]
    fn the_volume_law_matches_the_decibel_formula_at_every_setting()
    {
        // Each table entry is the nearest single precision value to a double
        // precision power, which is off by at most half of one unit in the last
        // place, a relative error of `f32::EPSILON / 2`. The product rounds
        // once more. Three such roundings stay under `1.5 * f32::EPSILON`
        // relative, and the powers built in double precision add a relative
        // error below 1e-13. Two `f32::EPSILON` covers all of it, which is
        // 2.1e-6 dB.
        let tolerance = 2.0 * f64::from(f32::EPSILON);
        for coarse in 0..=COARSE_MAX
        {
            for fine in 0..=FINE_MAX
            {
                let gain = volume(coarse, fine).linear_gain();
                let reference = reference_gain(coarse, fine);
                assert!
                (
                    (f64::from(gain) - reference).abs() <= tolerance * reference,
                    "coarse {coarse} fine {fine} gives {gain} against {reference}"
                );
                assert!((0.0..=1.0).contains(&gain), "coarse {coarse} fine {fine} gives {gain}");
            }
        }
    }

    #[test]
    fn the_volume_law_is_exact_at_its_ends()
    {
        // Bit patterns, so a gain a rounding away from 1 or a negative zero
        // fails where an epsilon would let it through.
        assert_eq!(volume(COARSE_MAX, FINE_MAX).linear_gain().to_bits(), 1.0_f32.to_bits());
        for other in 0..=u8::MAX
        {
            if let Ok(fine_only) = Volume::new(0, other)
            {
                assert_eq!(fine_only.linear_gain().to_bits(), 0.0_f32.to_bits());
            }
            if let Ok(coarse_only) = Volume::new(other, 0)
            {
                assert_eq!(coarse_only.linear_gain().to_bits(), 0.0_f32.to_bits());
            }
        }
        assert_eq!(Volume::MUTED.linear_gain().to_bits(), 0.0_f32.to_bits());
    }

    #[test]
    fn the_volume_law_rises_with_each_control()
    {
        for coarse in 1..=COARSE_MAX
        {
            for fine in 1..=FINE_MAX
            {
                let here = volume(coarse, fine).linear_gain();
                let below_fine = volume(coarse, fine - 1).linear_gain();
                let below_coarse = volume(coarse - 1, fine).linear_gain();
                assert!(here > below_fine, "fine {fine} at coarse {coarse} does not rise");
                assert!(here > below_coarse, "coarse {coarse} at fine {fine} does not rise");
            }
        }
    }

    #[test]
    fn the_default_coarse_step_sits_at_minus_24_db()
    {
        let gain = f64::from(volume(COARSE_DEFAULT, FINE_MAX).linear_gain());
        let expected = 10.0_f64.powf(-24.0 / 20.0);
        assert!((gain - expected).abs() <= 2.0 * f64::from(f32::EPSILON) * expected);
    }

    #[test]
    fn a_coarse_step_moves_by_its_detents_and_holds_at_both_ends()
    {
        let ceiling = i64::from(COARSE_MAX);
        for coarse in (0..=COARSE_MAX + 2).chain([u8::MAX])
        {
            for steps in (-30..=30).chain([i32::MIN, i32::MIN + 1, i32::MAX - 1, i32::MAX])
            {
                let start = i64::from(coarse).min(ceiling);
                let expected = (start + i64::from(steps)).clamp(0, ceiling);
                assert_eq!
                (
                    i64::from(step_coarse(coarse, steps)),
                    expected,
                    "step {coarse} moved by {steps}"
                );
            }
        }
    }

    #[test]
    fn the_absolute_volume_octet_drops_its_reserved_bit()
    {
        for octet in 0..=u8::MAX
        {
            let fine = fine_from_absolute_volume(octet);
            assert_eq!(fine, octet % 0x80, "octet {octet:#04x}");
            assert!(Volume::new(COARSE_MAX, fine).is_ok());
        }
        assert_eq!(fine_from_absolute_volume(0x80), 0);
        assert_eq!(fine_from_absolute_volume(0xFF), FINE_MAX);
    }

    #[test]
    fn unknown_codes_are_refused()
    {
        assert_eq!(Preset::from_code(0x04), Err(ProtocolError::BadPayload));
        assert_eq!(DspState::from_code(0x04), Err(ProtocolError::BadPayload));
        assert_eq!(Fault::from_code(0x02), Err(ProtocolError::BadPayload));
        assert_eq!(Fault::from_code(0x06), Err(ProtocolError::BadPayload));
    }

    #[test]
    fn decoding_reports_the_length_of_one_frame_inside_a_stream()
    {
        let (first, first_len) = encoded(&ToDsp::Heartbeat);
        let (second, second_len) = encoded(&ToDsp::SelectPreset(Preset::Garden));

        let mut stream = [0_u8; MAX_FRAME_LEN * 2];
        match (stream.get_mut(..first_len), first.get(..first_len))
        {
            (Some(slot), Some(source)) => slot.copy_from_slice(source),
            _ => panic!("stream shorter than the first frame"),
        }
        match (stream.get_mut(first_len..first_len + second_len), second.get(..second_len))
        {
            (Some(slot), Some(source)) => slot.copy_from_slice(source),
            _ => panic!("stream shorter than both frames"),
        }

        match ToDsp::decode(&stream)
        {
            Ok((message, consumed)) =>
            {
                assert_eq!(message, ToDsp::Heartbeat);
                assert_eq!(consumed, first_len);
                match stream.get(consumed..)
                {
                    Some(tail) => assert_eq!
                    (
                        ToDsp::decode(tail),
                        Ok((ToDsp::SelectPreset(Preset::Garden), second_len))
                    ),
                    None => panic!("no tail after the first frame"),
                }
            }
            Err(error) => panic!("decoding failed: {error:?}"),
        }
    }

    /// Reads every frame a stream holds, resynchronising on any other error.
    ///
    /// This is the reader contract `decode` documents, written once.
    fn drain(stream: &[u8]) -> (usize, [Option<ToDsp>; 4])
    {
        let mut found: [Option<ToDsp>; 4] = [None; 4];
        let mut count = 0;
        let mut offset = 0;

        while offset < stream.len()
        {
            let Some(window) = stream.get(offset..)
            else
            {
                break;
            };
            match ToDsp::decode(window)
            {
                Ok((message, consumed)) =>
                {
                    if let Some(slot) = found.get_mut(count)
                    {
                        *slot = Some(message);
                    }
                    count = count.saturating_add(1);
                    offset = offset.saturating_add(consumed);
                }
                Err(ProtocolError::Truncated) => break,
                Err(_) => match resync_offset(window)
                {
                    Some(discard) => offset = offset.saturating_add(discard.get()),
                    None => break,
                },
            }
        }
        (count, found)
    }

    #[test]
    fn a_reader_recovers_after_garbage()
    {
        let (frame, len) = encoded(&ToDsp::SelectPreset(Preset::Garden));
        let mut stream = [0_u8; 8 + MAX_FRAME_LEN];
        // Garbage that carries a start byte of its own, so the reader has to
        // survive a false start before the real frame.
        let noise = [0x00_u8, 0xFF, START_OF_FRAME, 0x03, 0x11, 0x22, 0x33, 0x44];
        stream[..noise.len()].copy_from_slice(&noise);
        stream[noise.len()..noise.len() + len].copy_from_slice(&frame[..len]);

        let (count, found) = drain(&stream);
        assert_eq!(count, 1);
        assert_eq!(found[0], Some(ToDsp::SelectPreset(Preset::Garden)));
    }

    #[test]
    fn a_reader_recovers_after_a_truncated_frame()
    {
        let (first, first_len) = encoded(&ToDsp::SetVolume(volume(0x0A, 0x20)));
        let (second, second_len) = encoded(&ToDsp::SelectPreset(Preset::Home));

        // The head of a frame, cut before its checksum, then a whole frame.
        let cut = first_len - 1;
        let mut stream = [0_u8; MAX_FRAME_LEN * 2];
        stream[..cut].copy_from_slice(&first[..cut]);
        stream[cut..cut + second_len].copy_from_slice(&second[..second_len]);

        let (count, found) = drain(&stream[..cut + second_len]);
        assert_eq!(count, 1);
        assert_eq!(found[0], Some(ToDsp::SelectPreset(Preset::Home)));
    }

    #[test]
    fn a_length_byte_from_a_failed_frame_never_moves_the_reader()
    {
        // A frame whose length is valid but whose checksum is wrong. Advancing
        // by the announced length would consume the genuine frame behind it.
        let (good, good_len) = encoded(&ToDsp::SelectPreset(Preset::BassBoosted));
        let forged = [START_OF_FRAME, 0x03, 0x02, 0x00, 0x00, 0x00];
        let mut stream = [0_u8; 6 + MAX_FRAME_LEN];
        stream[..forged.len()].copy_from_slice(&forged);
        stream[forged.len()..forged.len() + good_len].copy_from_slice(&good[..good_len]);

        let (count, found) = drain(&stream[..forged.len() + good_len]);
        assert_eq!(count, 1);
        assert_eq!(found[0], Some(ToDsp::SelectPreset(Preset::BassBoosted)));
    }

    #[test]
    fn resynchronising_always_makes_progress()
    {
        assert_eq!(resync_offset(&[]), None);
        assert_eq!(resync_offset(&[START_OF_FRAME]), NonZeroUsize::new(1));
        assert_eq!(resync_offset(&[START_OF_FRAME, 0x00, START_OF_FRAME]), NonZeroUsize::new(2));
        assert_eq!(resync_offset(&[0x00, 0x01, 0x02]), NonZeroUsize::new(3));

        // Every input a reader can hand it, of every shape a byte can take.
        for length in 1..=4_usize
        {
            let mut input = [0_u8; 4];
            for pattern in 0..1_u32 << (2 * length)
            {
                for (index, slot) in input.iter_mut().enumerate().take(length)
                {
                    *slot = match (pattern >> (2 * index)) & 0x03
                    {
                        0 => 0x00,
                        1 => START_OF_FRAME,
                        2 => 0xFF,
                        _ => 0x5A,
                    };
                }
                match input.get(..length).and_then(resync_offset)
                {
                    Some(offset) => assert!(offset.get() <= length),
                    None => panic!("an input of {length} bytes reported nothing to discard"),
                }
            }
        }
    }

    #[test]
    fn the_payload_bound_is_the_width_of_the_widest_message()
    {
        let (_, widest) = encoded(&ToDsp::SetVolume(volume(COARSE_MAX, FINE_MAX)));
        assert_eq!(widest, MAX_FRAME_LEN);

        for message in
        [
            ToDsp::Heartbeat,
            ToDsp::SelectPreset(Preset::Flat),
        ]
        {
            let (_, len) = encoded(&message);
            assert!(len <= MAX_FRAME_LEN);
        }
    }

    #[test]
    fn the_checksum_matches_the_reference_vector()
    {
        // CRC-16/CCITT-FALSE over "123456789" is 0x29B1.
        let mut crc: u16 = 0xFFFF;
        for byte in b"123456789"
        {
            crc = fold(crc, *byte);
        }
        assert_eq!(crc, 0x29B1);

        // The length byte folds in ahead of the payload, so two frames
        // announcing different lengths over the same bytes differ.
        assert_ne!(checksum_of(1, &[0x01]), checksum_of(2, &[0x01]));
    }
}
