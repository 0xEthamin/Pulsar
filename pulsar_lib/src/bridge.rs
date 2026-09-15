//! Bridge from the A2DP sink of the control board to its I2S slot stream.
//!
//! Bluedroid hands the sink 16-bit signed PCM, two channels interleaved, left
//! first, little endian. The processing board clocks the I2S link as master,
//! Philips format, two 32-bit slots per frame, and reads slot 0. Everything
//! between the two that touches a sample lives here: the frame ring that
//! absorbs the drift between the clock of the phone and the clock of the link,
//! the stream gate, the widening of each sample into its slot, and the pump that
//! moves one block from the ring to the link through `SlotWriter`.
//!
//! # Frames, never bytes
//!
//! The ring stores whole frames, so no push, drop or drain can split a frame or
//! swap its two channels. A chunk whose length is not a whole number of frames
//! loses its trailing partial frame, and the next chunk starts on a frame
//! boundary again. Bluedroid decodes whole SBC frames, so a trailing partial
//! frame only comes from a malformed chunk.
//!
//! # Drift
//!
//! A push into a full ring drops the incoming frames that do not fit, whole.
//! A drain that runs the ring dry leaves the rest of its block to the pump,
//! which pads it with silence, and the ring then gives out nothing until it has
//! refilled to its prefill level. An underrun therefore costs one gap.
//!
//! # The stream gate
//!
//! The whole chain runs at 44.1 kHz on two channels. The gate opens for SBC at
//! 44.1 kHz in joint stereo, stereo or dual channel mode, and every negotiation
//! decides again from the codec it carries. Any other stream leaves the gate
//! closed: every push drops its frames and the pump writes silence. Bluedroid
//! sets the PCM stride of its SBC decoder to the channel count, so a mono stream
//! arrives as one 16-bit sample per frame, which the ring would pair into false
//! stereo.

use crate::constants::SAMPLE_RATE_HZ;

/// Bytes of one decoded PCM frame: two 16-bit samples.
pub const PCM_FRAME_BYTES: usize = 4;

/// Bytes of one I2S frame on the link: two 32-bit slots.
pub const SLOT_FRAME_BYTES: usize = 8;

/// Ratio of slot bytes to PCM bytes.
const WIDENING: usize = 2;

/// Left shift that places a 16-bit sample in the upper half of its slot.
const SLOT_SHIFT: u32 = 16;

/// Frames the ring of the control board holds, 92.9 ms at 44.1 kHz.
pub const RING_FRAMES: usize = 4096;

/// Level the ring refills to before it gives out frames, 46.4 ms at 44.1 kHz.
///
/// Half the ring, so the ring has the same room for a burst from the radio as
/// for a stall of it.
pub const PREFILL_FRAMES: usize = 2048;

/// Frames one pump moves, 5.4 ms at 44.1 kHz.
pub const BLOCK_FRAMES: usize = 240;

/// Failure of a conversion between PCM bytes and slot bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeError
{
    /// The PCM bytes do not hold a whole number of frames.
    PartialFrame,
    /// The slot buffer cannot hold the widened frames.
    OutputTooShort,
}

/// Mask of the sampling frequency bits in octet 0 of the SBC codec information
/// element (A2DP specification, SBC codec specific information elements).
const SBC_FREQUENCY_MASK: u8 = 0xF0;

/// Mask of the channel mode bits in octet 0 of the SBC codec information element.
const SBC_CHANNEL_MODE_MASK: u8 = 0x0F;

/// Sampling frequency bit of 16 kHz.
const SBC_16_KHZ: u8 = 0x80;

/// Sampling frequency bit of 32 kHz.
const SBC_32_KHZ: u8 = 0x40;

/// Sampling frequency bit of 44.1 kHz.
const SBC_44_1_KHZ: u8 = 0x20;

/// Sampling frequency bit of 48 kHz.
const SBC_48_KHZ: u8 = 0x10;

/// Channel mode bit of mono.
const SBC_MONO: u8 = 0x08;

/// Codec of a negotiated A2DP stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamCodec
{
    /// SBC, carrying octet 0 of its codec information element: the sampling
    /// frequency in the upper four bits, the channel mode in the lower four.
    Sbc(u8),
    /// Any codec other than SBC.
    Other,
}

/// Verdict of the stream gate on a negotiated stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamVerdict
{
    /// SBC at the rate of the chain on two channels. The bridge forwards it.
    Forward,
    /// A codec other than SBC.
    NotSbc,
    /// SBC at another sampling frequency, in hertz.
    WrongRate(u32),
    /// SBC in mono.
    Mono,
    /// SBC whose octet 0 names no single sampling frequency or no single
    /// channel mode.
    Malformed,
}

/// Returns the verdict of the stream gate on `codec`.
///
/// Forwards SBC at 44.1 kHz in joint stereo, stereo or dual channel mode. A
/// configured stream names exactly one sampling frequency and one channel
/// mode, so an octet with none or several of either is `Malformed`.
#[must_use]
pub const fn stream_verdict(codec: StreamCodec) -> StreamVerdict
{
    let StreamCodec::Sbc(octet) = codec
    else
    {
        return StreamVerdict::NotSbc;
    };

    let frequency = octet & SBC_FREQUENCY_MASK;
    let mode = octet & SBC_CHANNEL_MODE_MASK;

    if mode.count_ones() != 1
    {
        return StreamVerdict::Malformed;
    }

    let rate_hz = match frequency
    {
        SBC_16_KHZ => 16_000,
        SBC_32_KHZ => 32_000,
        SBC_44_1_KHZ => 44_100,
        SBC_48_KHZ => 48_000,
        _ => return StreamVerdict::Malformed,
    };

    if rate_hz != SAMPLE_RATE_HZ
    {
        return StreamVerdict::WrongRate(rate_hz);
    }

    if mode == SBC_MONO
    {
        return StreamVerdict::Mono;
    }

    StreamVerdict::Forward
}

/// What a push did with a chunk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Pushed
{
    /// Frames the ring took, the leading frames of the chunk.
    pub accepted: usize,
    /// Whole frames dropped, because the ring was full or the gate is closed.
    pub dropped: usize,
    /// Bytes of the trailing partial frame, discarded.
    pub trailing_bytes: usize,
}

/// Widens interleaved 16-bit PCM frames into 32-bit I2S slot frames.
///
/// Writes each sample, in order, into the upper 16 bits of its slot, little
/// endian, so the left sample of a frame lands in slot 0 and the right one in
/// slot 1. The lower 16 bits of every slot are zero.
///
/// Returns the bytes written to `slots`, twice the length of `pcm`.
///
/// # Errors
///
/// `PartialFrame` when `pcm` is not a whole number of frames, `OutputTooShort`
/// when `slots` is shorter than twice `pcm`. Both checks run before the first
/// write.
fn widen_into(pcm: &[u8], slots: &mut [u8]) -> Result<usize, BridgeError>
{
    let (frames, partial) = pcm.as_chunks::<PCM_FRAME_BYTES>();

    if !partial.is_empty()
    {
        return Err(BridgeError::PartialFrame);
    }

    let Some(needed) = pcm.len().checked_mul(WIDENING)
    else
    {
        return Err(BridgeError::OutputTooShort);
    };

    if slots.len() < needed
    {
        return Err(BridgeError::OutputTooShort);
    }

    let (links, _) = slots.as_chunks_mut::<SLOT_FRAME_BYTES>();

    for (&[l0, l1, r0, r1], link) in frames.iter().zip(links.iter_mut())
    {
        let left = widen_sample(l0, l1);
        let right = widen_sample(r0, r1);

        for (out, byte) in link.iter_mut().zip(left.into_iter().chain(right))
        {
            *out = byte;
        }
    }

    Ok(needed)
}

/// Returns the slot bytes of the little endian sample `[low, high]`.
fn widen_sample(low: u8, high: u8) -> [u8; 4]
{
    (i32::from(i16::from_le_bytes([low, high])) << SLOT_SHIFT).to_le_bytes()
}

/// Fixed capacity queue of whole PCM frames.
struct FrameRing<const FRAMES: usize>
{
    frames: [[u8; PCM_FRAME_BYTES]; FRAMES],
    head: usize,
    len: usize,
}

impl<const FRAMES: usize> FrameRing<FRAMES>
{
    /// Builds an empty ring.
    const fn new() -> Self
    {
        Self
        {
            frames: [[0; PCM_FRAME_BYTES]; FRAMES],
            head: 0,
            len: 0,
        }
    }

    /// Empties the ring.
    fn clear(&mut self)
    {
        self.head = 0;
        self.len = 0;
    }

    /// Appends `frame`. Returns false, leaving the ring as it was, when full.
    fn push(&mut self, frame: [u8; PCM_FRAME_BYTES]) -> bool
    {
        if self.len >= FRAMES
        {
            return false;
        }

        let Some(tail) = self
            .head
            .checked_add(self.len)
            .and_then(|end| end.checked_rem(FRAMES))
        else
        {
            return false;
        };

        let Some(slot) = self.frames.get_mut(tail)
        else
        {
            return false;
        };

        *slot = frame;
        self.len = self.len.saturating_add(1);
        true
    }

    /// Removes and returns the oldest frame, or `None` when empty.
    fn pop(&mut self) -> Option<[u8; PCM_FRAME_BYTES]>
    {
        if self.len == 0
        {
            return None;
        }

        let frame = *self.frames.get(self.head)?;
        self.head = self
            .head
            .checked_add(1)
            .and_then(|next| next.checked_rem(FRAMES))
            .unwrap_or(0);
        self.len = self.len.saturating_sub(1);
        Some(frame)
    }
}

/// Frame ring, stream gate and prefill between the A2DP sink and the I2S pump.
pub struct Bridge<const FRAMES: usize>
{
    ring: FrameRing<FRAMES>,
    prefill: usize,
    forwarding: bool,
    primed: bool,
}

impl<const FRAMES: usize> Bridge<FRAMES>
{
    /// Builds a bridge with its gate closed and its ring empty.
    ///
    /// The ring gives out frames once it holds `prefill_frames`. A prefill
    /// above `FRAMES` stands at `FRAMES`.
    #[must_use]
    pub const fn new(prefill_frames: usize) -> Self
    {
        let prefill = if prefill_frames > FRAMES
        {
            FRAMES
        }
        else
        {
            prefill_frames
        };

        Self
        {
            ring: FrameRing::new(),
            prefill,
            forwarding: false,
            primed: false,
        }
    }

    /// Starts a stream negotiated on `codec` and empties the ring.
    ///
    /// Returns the verdict of `stream_verdict` on `codec`. The gate opens on
    /// `Forward` and closes on every other verdict, whatever the previous
    /// stream was.
    pub fn open(&mut self, codec: StreamCodec) -> StreamVerdict
    {
        self.flush();
        let verdict = stream_verdict(codec);
        self.forwarding = matches!(verdict, StreamVerdict::Forward);
        verdict
    }

    /// Closes the gate and empties the ring.
    pub fn close(&mut self)
    {
        self.flush();
        self.forwarding = false;
    }

    /// Empties the ring and waits for the prefill again. The gate stays as is.
    pub fn flush(&mut self)
    {
        self.ring.clear();
        self.primed = false;
    }

    /// Pushes a chunk of decoded PCM.
    ///
    /// The ring takes the leading whole frames of `pcm` that fit and drops the
    /// rest of the whole frames. A closed gate drops them all. A trailing
    /// partial frame is discarded in every case.
    pub fn push(&mut self, pcm: &[u8]) -> Pushed
    {
        let (frames, partial) = pcm.as_chunks::<PCM_FRAME_BYTES>();
        let mut pushed = Pushed
        {
            trailing_bytes: partial.len(),
            ..Pushed::default()
        };

        for &frame in frames
        {
            if self.forwarding && self.ring.push(frame)
            {
                pushed.accepted = pushed.accepted.saturating_add(1);
            }
            else
            {
                pushed.dropped = pushed.dropped.saturating_add(1);
            }
        }

        pushed
    }

    /// Drains whole frames into `pcm`, oldest first.
    ///
    /// Returns the bytes written, a whole number of frames, from the start of
    /// `pcm`. Bytes past them are left untouched. Returns 0 while the ring
    /// waits for its prefill. A drain that empties the ring before it fills
    /// `pcm` makes the ring wait for the prefill again.
    pub fn drain_into(&mut self, pcm: &mut [u8]) -> usize
    {
        if !self.primed
        {
            if self.ring.len == 0 || self.ring.len < self.prefill
            {
                return 0;
            }

            self.primed = true;
        }

        let mut written: usize = 0;

        let (outs, _) = pcm.as_chunks_mut::<PCM_FRAME_BYTES>();

        for out in outs
        {
            let Some(frame) = self.ring.pop()
            else
            {
                self.primed = false;
                break;
            };

            *out = frame;
            written = written.saturating_add(PCM_FRAME_BYTES);
        }

        written
    }
}

/// Source of whole PCM frames for the pump.
pub trait FrameSource
{
    /// Writes whole frames from the start of `pcm` and returns the bytes written.
    fn take_frames(&mut self, pcm: &mut [u8]) -> usize;
}

impl<const FRAMES: usize> FrameSource for Bridge<FRAMES>
{
    fn take_frames(&mut self, pcm: &mut [u8]) -> usize
    {
        self.drain_into(pcm)
    }
}

/// Transmit side of the I2S link.
pub trait SlotWriter
{
    /// Error the channel reports.
    type Error;

    /// Writes a prefix of `bytes` and returns its length.
    ///
    /// Returns 0 when the channel took nothing before its timeout.
    ///
    /// # Errors
    ///
    /// Whatever the channel reports.
    fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error>;
}

/// Failure of one pump.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PumpError<E>
{
    /// The scratch buffers cannot carry a block. Nothing was taken.
    Layout(BridgeError),
    /// The channel failed. The block is lost from the byte the channel stopped at.
    Write(E),
}

/// Moves one block from `source` to `writer`.
///
/// Takes as many whole frames as `pcm` holds from `source`, pads the rest of
/// `pcm` with silence, widens the block into `slots` and writes all of it,
/// calling the writer again while it takes nothing. A closed gate, an empty
/// ring or a ring waiting for its prefill therefore writes a block of silence.
///
/// Returns the frames taken from `source`.
///
/// # Errors
///
/// `Layout` when `pcm` is not a whole number of frames or `slots` is shorter
/// than twice `pcm`, before `source` is touched. `Write` when the writer fails.
pub fn pump<S, W>
(
    source: &mut S,
    pcm: &mut [u8],
    slots: &mut [u8],
    writer: &mut W
) -> Result<usize, PumpError<W::Error>>
where
    S: FrameSource,
    W: SlotWriter,
{
    if !pcm.as_chunks::<PCM_FRAME_BYTES>().1.is_empty()
    {
        return Err(PumpError::Layout(BridgeError::PartialFrame));
    }

    if pcm.len().checked_mul(WIDENING).is_none_or(|needed| slots.len() < needed)
    {
        return Err(PumpError::Layout(BridgeError::OutputTooShort));
    }

    let reported = source.take_frames(pcm).min(pcm.len());
    let taken = reported.saturating_sub(reported.checked_rem(PCM_FRAME_BYTES).unwrap_or(0));

    if let Some(rest) = pcm.get_mut(taken..)
    {
        rest.fill(0);
    }

    let written = widen_into(pcm, slots).map_err(PumpError::Layout)?;
    let mut done: usize = 0;

    while let Some(rest) = slots.get(done..written)
    {
        if rest.is_empty()
        {
            break;
        }

        let accepted = writer.write(rest).map_err(PumpError::Write)?;
        done = done.saturating_add(accepted.min(rest.len()));
    }

    Ok(taken.checked_div(PCM_FRAME_BYTES).unwrap_or(0))
}

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, which is the one place the
    // no-panic rule does not hold.
    #![allow
    (
        clippy::panic,
        clippy::indexing_slicing,
        clippy::arithmetic_side_effects,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::chunks_exact_to_as_chunks
    )]

    use super::*;
    use core::iter::repeat_n;
    use std::vec::Vec;

    /// Ring capacity of the fixture, small so a run crosses every fill level.
    const FIXTURE_FRAMES: usize = 37;

    /// Prefill of the fixture.
    const FIXTURE_PREFILL: usize = 11;

    /// Byte a malformed chunk trails with. A reader that loses the frame
    /// boundary reads it as part of a sample.
    const JUNK: u8 = 0xEE;

    /// Builds frame `index`: the left sample carries the index, the right one
    /// its complement, so no frame is silence and a shifted read shows.
    fn frame(index: u16) -> [u8; PCM_FRAME_BYTES]
    {
        let [l0, l1] = index.to_le_bytes();
        let [r0, r1] = (!index).to_le_bytes();
        [l0, l1, r0, r1]
    }

    /// Builds the bytes of frames `first..first + count`.
    fn frames(first: u16, count: usize) -> Vec<u8>
    {
        (0..count)
            .flat_map(|offset| frame(first.wrapping_add(offset as u16)))
            .collect()
    }

    /// Returns the index a PCM frame carries, failing on a misaligned frame.
    fn index_of(bytes: &[u8]) -> u16
    {
        let left = u16::from_le_bytes([bytes[0], bytes[1]]);
        let right = u16::from_le_bytes([bytes[2], bytes[3]]);
        assert_eq!(right, !left, "frame {bytes:02x?} lost its alignment");
        left
    }

    /// Returns the two slot words of a link frame.
    fn slots_of(bytes: &[u8]) -> (i32, i32)
    {
        (
            i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            i32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        )
    }

    /// Deterministic pseudo random sizes.
    struct Lcg(u64);

    impl Lcg
    {
        /// Returns a value in `0..bound`.
        fn below(&mut self, bound: usize) -> usize
        {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 33) as usize) % bound
        }
    }

    /// SBC at 44.1 kHz in joint stereo, the stream phones negotiate most.
    const JOINT_STEREO_44_1: StreamCodec = StreamCodec::Sbc(0x21);

    /// Builds an open fixture bridge with `prefill` frames of prefill.
    fn open_bridge(prefill: usize) -> Bridge<FIXTURE_FRAMES>
    {
        let mut bridge = Bridge::new(prefill);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), StreamVerdict::Forward);
        bridge
    }

    /// Asserts that `bridge` drops every pushed frame and pumps silence.
    fn assert_silent(bridge: &mut Bridge<FIXTURE_FRAMES>, context: &str)
    {
        assert_eq!
        (
            bridge.push(&frames(1, 3)),
            Pushed { accepted: 0, dropped: 3, trailing_bytes: 0 },
            "{context}"
        );

        let mut pcm = [0x33; 3 * PCM_FRAME_BYTES];
        let mut slots = [0x5A; 3 * SLOT_FRAME_BYTES];
        let mut link = ChokedLink::new();
        assert_eq!(pump(bridge, &mut pcm, &mut slots, &mut link), Ok(0), "{context}");
        assert_eq!(link.bytes, [0; 3 * SLOT_FRAME_BYTES], "{context}");
    }

    /// Link that takes a varying prefix, and nothing on every third call.
    struct ChokedLink
    {
        bytes: Vec<u8>,
        calls: usize,
        fail_on_call: Option<usize>,
    }

    impl ChokedLink
    {
        fn new() -> Self
        {
            Self
            {
                bytes: Vec::new(),
                calls: 0,
                fail_on_call: None,
            }
        }
    }

    impl SlotWriter for ChokedLink
    {
        type Error = u8;

        fn write(&mut self, bytes: &[u8]) -> Result<usize, u8>
        {
            self.calls += 1;

            if self.fail_on_call == Some(self.calls)
            {
                return Err(7);
            }

            if self.calls.is_multiple_of(3)
            {
                return Ok(0);
            }

            let taken = bytes.len().min(1 + self.calls % 13);
            self.bytes.extend_from_slice(&bytes[..taken]);
            Ok(taken)
        }
    }

    /// Widens one frame.
    fn widen_frame(left: i16, right: i16) -> [u8; SLOT_FRAME_BYTES]
    {
        let [l0, l1] = left.to_le_bytes();
        let [r0, r1] = right.to_le_bytes();
        let mut slots = [0x5A; SLOT_FRAME_BYTES];
        assert_eq!(widen_into(&[l0, l1, r0, r1], &mut slots), Ok(SLOT_FRAME_BYTES));
        slots
    }

    #[test]
    fn every_sample_lands_in_the_upper_half_of_its_slot()
    {
        for sample in i16::MIN..=i16::MAX
        {
            let (left, right) = slots_of(&widen_frame(sample, sample.wrapping_neg()));
            assert_eq!(left, i32::from(sample) * 65_536, "left slot of {sample}");
            assert_eq!(right, i32::from(sample.wrapping_neg()) * 65_536, "right slot of {sample}");
            assert_eq!(left & 0xFFFF, 0);
            assert_eq!(right & 0xFFFF, 0);
        }
    }

    #[test]
    fn full_scale_zero_and_sign_widen_to_their_exact_bytes()
    {
        assert_eq!(widen_frame(i16::MAX, i16::MIN), [0, 0, 0xFF, 0x7F, 0, 0, 0x00, 0x80]);
        assert_eq!(widen_frame(0, -1), [0, 0, 0, 0, 0, 0, 0xFF, 0xFF]);
        assert_eq!(widen_frame(1, -2), [0, 0, 0x01, 0x00, 0, 0, 0xFE, 0xFF]);
    }

    #[test]
    fn the_output_is_little_endian_with_the_left_sample_first()
    {
        let mut slots = [0; 2 * SLOT_FRAME_BYTES];
        let pcm = [0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A, 0xF0, 0xDE];
        assert_eq!(widen_into(&pcm, &mut slots), Ok(16));
        assert_eq!
        (
            slots,
            [
                0, 0, 0x34, 0x12, 0, 0, 0x78, 0x56,
                0, 0, 0xBC, 0x9A, 0, 0, 0xF0, 0xDE,
            ]
        );
    }

    #[test]
    fn widening_refuses_a_partial_frame_or_a_short_output_without_writing()
    {
        for len in [1, 2, 3, 5, 6, 7]
        {
            let mut slots = [0x5A; 16];
            assert_eq!(widen_into(&[1; 7][..len], &mut slots), Err(BridgeError::PartialFrame));
            assert_eq!(slots, [0x5A; 16]);
        }

        let mut short = [0x5A; 15];
        assert_eq!(widen_into(&[1; 8], &mut short), Err(BridgeError::OutputTooShort));
        assert_eq!(short, [0x5A; 15]);

        let mut long = [0x5A; 20];
        assert_eq!(widen_into(&[0; 8], &mut long), Ok(16));
        assert_eq!(long[..16], [0; 16]);
        assert_eq!(long[16..], [0x5A; 4]);

        assert_eq!(widen_into(&[], &mut []), Ok(0));
    }

    #[test]
    fn chained_pushes_and_drains_keep_every_accepted_frame_in_order()
    {
        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut rng = Lcg(0x5EED);
        let mut next_index: u16 = 0;
        let mut expected: Vec<u16> = Vec::new();
        let mut output: Vec<u16> = Vec::new();
        let mut level = 0;
        let mut dropped = 0;
        let mut ragged_drains = 0;
        let mut drain = [0; 64 * PCM_FRAME_BYTES + 3];

        for _ in 0..20_000
        {
            let count = rng.below(FIXTURE_FRAMES + 8);
            let trailing = rng.below(PCM_FRAME_BYTES);
            let mut chunk = frames(next_index, count);
            chunk.extend(repeat_n(JUNK, trailing));

            let pushed = bridge.push(&chunk);
            let room = FIXTURE_FRAMES - level;
            assert_eq!
            (
                pushed,
                Pushed
                {
                    accepted: count.min(room),
                    dropped: count.saturating_sub(room),
                    trailing_bytes: trailing,
                }
            );
            expected.extend((0..pushed.accepted).map(|offset| next_index.wrapping_add(offset as u16)));
            next_index = next_index.wrapping_add(count as u16);
            level += pushed.accepted;
            dropped += pushed.dropped;

            let len = rng.below(drain.len() + 1);
            drain.fill(JUNK);
            let written = bridge.drain_into(&mut drain[..len]);
            assert_eq!(written % PCM_FRAME_BYTES, 0);
            assert!(written <= len);
            assert!(drain[written..len].iter().all(|&byte| byte == JUNK));
            ragged_drains += usize::from(!len.is_multiple_of(PCM_FRAME_BYTES) && written > 0);
            output.extend(drain[..written].chunks_exact(PCM_FRAME_BYTES).map(index_of));
            level -= written / PCM_FRAME_BYTES;
        }

        assert_eq!(output[..], expected[..output.len()]);
        assert_eq!(expected.len() - output.len(), level);
        assert!(dropped > 1_000, "the run dropped {dropped} frames");
        assert!(output.len() > 100_000, "the run drained {} frames", output.len());
        assert!(ragged_drains > 1_000, "the run drained {ragged_drains} ragged buffers");
    }

    #[test]
    fn a_full_ring_drops_whole_incoming_frames_at_every_fill_level()
    {
        let mut out = [0; (FIXTURE_FRAMES + 1) * PCM_FRAME_BYTES];

        for level in 0..=FIXTURE_FRAMES
        {
            for incoming in 0..=FIXTURE_FRAMES + 2
            {
                for trailing in 0..PCM_FRAME_BYTES
                {
                    let mut bridge = open_bridge(0);

                    let rotation = (level * 7 + incoming + trailing) % FIXTURE_FRAMES;
                    assert_eq!(bridge.push(&frames(5_000, rotation)).accepted, rotation);
                    assert_eq!(bridge.drain_into(&mut out), rotation * PCM_FRAME_BYTES);

                    assert_eq!(bridge.push(&frames(0, level)).accepted, level);

                    let mut chunk = frames(1_000, incoming);
                    chunk.extend(repeat_n(JUNK, trailing));
                    let room = FIXTURE_FRAMES - level;
                    assert_eq!
                    (
                        bridge.push(&chunk),
                        Pushed
                        {
                            accepted: incoming.min(room),
                            dropped: incoming.saturating_sub(room),
                            trailing_bytes: trailing,
                        }
                    );

                    let written = bridge.drain_into(&mut out);
                    let got: Vec<u16> = out[..written].chunks_exact(PCM_FRAME_BYTES).map(index_of).collect();
                    let want: Vec<u16> = (0..level as u16)
                        .chain((0..incoming.min(room) as u16).map(|offset| 1_000 + offset))
                        .collect();
                    assert_eq!(got, want, "level {level}, incoming {incoming}");

                    assert_eq!(bridge.push(&frame(2_000)).accepted, 1);
                    let written = bridge.drain_into(&mut out);
                    assert_eq!(written, PCM_FRAME_BYTES);
                    assert_eq!(index_of(&out[..PCM_FRAME_BYTES]), 2_000);
                }
            }
        }
    }

    #[test]
    fn the_ring_waits_for_its_prefill_before_and_after_an_underrun()
    {
        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut out = [0; 4 * PCM_FRAME_BYTES];

        for index in 0..FIXTURE_PREFILL - 1
        {
            assert_eq!(bridge.push(&frame(index as u16)).accepted, 1);
            assert_eq!(bridge.drain_into(&mut out), 0, "gave out at level {}", index + 1);
        }

        assert_eq!(bridge.push(&frame(10)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(bridge.drain_into(&mut out), 12);
        assert_eq!(index_of(&out[8..12]), 10);

        assert_eq!(bridge.push(&frames(20, FIXTURE_PREFILL - 1)).accepted, FIXTURE_PREFILL - 1);
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frame(99)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(index_of(&out[..4]), 20);
    }

    #[test]
    fn a_prefill_above_the_capacity_stands_at_the_capacity()
    {
        let mut bridge = Bridge::<4>::new(100);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), StreamVerdict::Forward);
        let mut out = [0; 4 * PCM_FRAME_BYTES];
        assert_eq!(bridge.push(&frames(0, 3)).accepted, 3);
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frames(3, 2)), Pushed { accepted: 1, dropped: 1, trailing_bytes: 0 });
        assert_eq!(bridge.drain_into(&mut out), 16);
    }

    #[test]
    fn a_drain_from_an_empty_ring_writes_nothing()
    {
        let mut bridge = open_bridge(0);
        let mut out = [0xA5; 12];
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(out, [0xA5; 12]);

        let mut closed = Bridge::<FIXTURE_FRAMES>::new(0);
        assert_eq!(closed.drain_into(&mut out), 0);
        assert_eq!(out, [0xA5; 12]);
    }

    #[test]
    fn a_ragged_drain_writes_whole_frames_and_leaves_the_tail()
    {
        let mut bridge = open_bridge(0);
        assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
        let mut out = [JUNK; 7];
        assert_eq!(bridge.drain_into(&mut out), 4);
        assert_eq!(index_of(&out[..4]), 0);
        assert_eq!(out[4..], [JUNK; 3]);
        let mut rest = [0; 16];
        assert_eq!(bridge.drain_into(&mut rest), 16);
        assert_eq!(rest.chunks_exact(4).map(index_of).collect::<Vec<_>>(), [1, 2, 3, 4]);
    }

    #[test]
    fn the_stream_verdict_reads_rate_and_channel_mode()
    {
        let cases =
        [
            (StreamCodec::Sbc(0x21), StreamVerdict::Forward),
            (StreamCodec::Sbc(0x22), StreamVerdict::Forward),
            (StreamCodec::Sbc(0x24), StreamVerdict::Forward),
            (StreamCodec::Sbc(0x28), StreamVerdict::Mono),
            (StreamCodec::Sbc(0x11), StreamVerdict::WrongRate(48_000)),
            (StreamCodec::Sbc(0x18), StreamVerdict::WrongRate(48_000)),
            (StreamCodec::Sbc(0x42), StreamVerdict::WrongRate(32_000)),
            (StreamCodec::Sbc(0x84), StreamVerdict::WrongRate(16_000)),
            (StreamCodec::Sbc(0x20), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x01), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x00), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x23), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x29), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0x31), StreamVerdict::Malformed),
            (StreamCodec::Sbc(0xFF), StreamVerdict::Malformed),
            (StreamCodec::Other, StreamVerdict::NotSbc),
        ];

        for (codec, verdict) in cases
        {
            assert_eq!(stream_verdict(codec), verdict, "{codec:?}");
        }

        let forwarded: Vec<u8> = (0..=u8::MAX)
            .filter(|&octet| stream_verdict(StreamCodec::Sbc(octet)) == StreamVerdict::Forward)
            .collect();
        assert_eq!(forwarded, [0x21, 0x22, 0x24]);

        let mono: Vec<u8> = (0..=u8::MAX)
            .filter(|&octet| stream_verdict(StreamCodec::Sbc(octet)) == StreamVerdict::Mono)
            .collect();
        assert_eq!(mono, [0x28]);
    }

    #[test]
    fn only_a_forwarded_stream_opens_the_gate()
    {
        let mut pcm = [0; 3 * PCM_FRAME_BYTES];
        let mut slots = [0x5A; 3 * SLOT_FRAME_BYTES];

        for octet in [0x28, 0x11, 0x42, 0x84, 0x20, 0x23]
        {
            let mut bridge = Bridge::<FIXTURE_FRAMES>::new(0);
            assert_ne!(bridge.open(StreamCodec::Sbc(octet)), StreamVerdict::Forward);
            assert_silent(&mut bridge, &std::format!("octet {octet:#04x}"));
        }

        let mut other = Bridge::<FIXTURE_FRAMES>::new(0);
        assert_eq!(other.open(StreamCodec::Other), StreamVerdict::NotSbc);
        assert_silent(&mut other, "codec other than SBC");

        let mut bridge = Bridge::<FIXTURE_FRAMES>::new(0);
        assert_eq!(bridge.open(JOINT_STEREO_44_1), StreamVerdict::Forward);
        assert_eq!(bridge.push(&frames(1, 3)).accepted, 3);
        let mut link = ChokedLink::new();
        assert_eq!(pump(&mut bridge, &mut pcm, &mut slots, &mut link), Ok(3));
        let words: Vec<(i32, i32)> = link.bytes.chunks_exact(SLOT_FRAME_BYTES).map(slots_of).collect();
        assert_eq!
        (
            words,
            [
                (0x0001_0000, !0x0001_i32 << 16),
                (0x0002_0000, !0x0002_i32 << 16),
                (0x0003_0000, !0x0003_i32 << 16),
            ]
        );
    }

    #[test]
    fn a_renegotiation_closes_the_gate_and_discards_the_old_stream()
    {
        let mut out = [0; 8 * PCM_FRAME_BYTES];

        for refused in
        [
            StreamCodec::Sbc(0x11),
            StreamCodec::Sbc(0x28),
            StreamCodec::Sbc(0x20),
            StreamCodec::Other,
        ]
        {
            let mut bridge = open_bridge(0);
            assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
            assert_ne!(bridge.open(refused), StreamVerdict::Forward);
            assert_eq!(bridge.drain_into(&mut out), 0, "{refused:?}");
            assert_silent(&mut bridge, &std::format!("renegotiated to {refused:?}"));

            assert_eq!(bridge.open(JOINT_STEREO_44_1), StreamVerdict::Forward);
            assert_eq!(bridge.drain_into(&mut out), 0);
            assert_eq!(bridge.push(&frame(9)).accepted, 1);
            assert_eq!(bridge.drain_into(&mut out), 4);
            assert_eq!(index_of(&out[..4]), 9);
        }
    }

    #[test]
    fn flush_makes_the_ring_wait_for_its_prefill_again()
    {
        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut out = [0; 4 * PCM_FRAME_BYTES];

        assert_eq!(bridge.push(&frames(0, FIXTURE_PREFILL)).accepted, FIXTURE_PREFILL);
        assert_eq!(bridge.drain_into(&mut out), 16);

        bridge.flush();
        assert_eq!(bridge.push(&frames(100, FIXTURE_PREFILL - 1)).accepted, FIXTURE_PREFILL - 1);
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frame(200)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 16);
        assert_eq!(index_of(&out[..4]), 100);
    }

    #[test]
    fn close_drops_everything_and_flush_keeps_the_gate()
    {
        let mut out = [0; 8 * PCM_FRAME_BYTES];

        let mut bridge = open_bridge(0);
        assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
        bridge.flush();
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frame(7)).accepted, 1);
        assert_eq!(bridge.drain_into(&mut out), 4);
        assert_eq!(index_of(&out[..4]), 7);

        assert_eq!(bridge.push(&frames(0, 5)).accepted, 5);
        bridge.close();
        assert_eq!(bridge.drain_into(&mut out), 0);
        assert_eq!(bridge.push(&frames(0, 2)), Pushed { accepted: 0, dropped: 2, trailing_bytes: 0 });
    }

    #[test]
    fn chained_pumps_keep_the_link_frame_aligned_through_partial_writes()
    {
        const BLOCK: usize = 6;
        const PUMPS: usize = 2_000;

        let mut bridge = open_bridge(FIXTURE_PREFILL);
        let mut rng = Lcg(0xB10C);
        let mut link = ChokedLink::new();
        let mut pcm = [0; BLOCK * PCM_FRAME_BYTES];
        let mut slots = [0; BLOCK * SLOT_FRAME_BYTES];
        let mut next_index: u16 = 0;
        let mut expected: Vec<u16> = Vec::new();
        let mut taken = 0;

        for _ in 0..PUMPS
        {
            let count = rng.below(BLOCK + 4);
            let mut chunk = frames(next_index, count);
            chunk.extend(repeat_n(JUNK, rng.below(PCM_FRAME_BYTES)));
            let pushed = bridge.push(&chunk);
            expected.extend((0..pushed.accepted).map(|offset| next_index.wrapping_add(offset as u16)));
            next_index = next_index.wrapping_add(count as u16);

            match pump(&mut bridge, &mut pcm, &mut slots, &mut link)
            {
                Ok(frames) => taken += frames,
                Err(error) => panic!("pump failed: {error:?}"),
            }
        }

        assert_eq!(link.bytes.len(), PUMPS * BLOCK * SLOT_FRAME_BYTES);

        let mut carried: Vec<u16> = Vec::new();
        let mut silent = 0;

        for bytes in link.bytes.chunks_exact(SLOT_FRAME_BYTES)
        {
            let (left, right) = slots_of(bytes);
            assert_eq!(left & 0xFFFF, 0, "link frame {bytes:02x?} lost its alignment");
            assert_eq!(right & 0xFFFF, 0, "link frame {bytes:02x?} lost its alignment");

            if left == 0 && right == 0
            {
                silent += 1;
                continue;
            }

            assert_eq!(right >> 16, !(left >> 16), "link frame {bytes:02x?} lost its alignment");
            carried.push((left >> 16) as u16);
        }

        assert_eq!(carried.len(), taken);
        assert_eq!(carried[..], expected[..carried.len()]);
        assert!(silent > BLOCK, "the run never underran");
        assert!(taken > PUMPS, "the run carried {taken} frames");
    }

    #[test]
    fn a_writer_error_stops_the_pump()
    {
        let mut bridge = open_bridge(0);
        let mut pcm = [0; 2 * PCM_FRAME_BYTES];
        let mut slots = [0; 2 * SLOT_FRAME_BYTES];
        let mut link = ChokedLink::new();
        link.fail_on_call = Some(2);
        assert_eq!(pump(&mut bridge, &mut pcm, &mut slots, &mut link), Err(PumpError::Write(7)));
    }

    #[test]
    fn the_pump_refuses_buffers_that_cannot_carry_a_block_before_taking_frames()
    {
        let mut bridge = open_bridge(0);
        assert_eq!(bridge.push(&frames(0, 2)).accepted, 2);
        let mut link = ChokedLink::new();

        let mut ragged = [0; 5];
        let mut slots = [0; 16];
        assert_eq!
        (
            pump(&mut bridge, &mut ragged, &mut slots, &mut link),
            Err(PumpError::Layout(BridgeError::PartialFrame))
        );

        let mut pcm = [0; 8];
        let mut short = [0; 15];
        assert_eq!
        (
            pump(&mut bridge, &mut pcm, &mut short, &mut link),
            Err(PumpError::Layout(BridgeError::OutputTooShort))
        );

        assert!(link.bytes.is_empty());
        let mut out = [0; 8];
        assert_eq!(bridge.drain_into(&mut out), 8);
    }

    /// Source that fills its whole buffer and reports a byte count of its own.
    struct Misreporting(usize);

    impl FrameSource for Misreporting
    {
        fn take_frames(&mut self, pcm: &mut [u8]) -> usize
        {
            pcm.fill(0x11);
            self.0
        }
    }

    #[test]
    fn the_pump_cuts_an_overstated_take_to_whole_frames_of_its_buffer()
    {
        let mut pcm = [0; 2 * PCM_FRAME_BYTES];
        let mut slots = [0; 2 * SLOT_FRAME_BYTES];
        let mut link = ChokedLink::new();
        assert_eq!(pump(&mut Misreporting(99), &mut pcm, &mut slots, &mut link), Ok(2));
        assert_eq!(link.bytes.len(), 16);
        assert_eq!(slots_of(&link.bytes[..8]), (0x1111_0000, 0x1111_0000));
        assert_eq!(slots_of(&link.bytes[8..]), (0x1111_0000, 0x1111_0000));
    }

    #[test]
    fn the_pump_cuts_a_ragged_take_to_whole_frames_and_pads_the_rest()
    {
        for reported in 5..=7
        {
            let mut pcm = [0; 2 * PCM_FRAME_BYTES];
            let mut slots = [0; 2 * SLOT_FRAME_BYTES];
            let mut link = ChokedLink::new();
            assert_eq!(pump(&mut Misreporting(reported), &mut pcm, &mut slots, &mut link), Ok(1));
            assert_eq!(slots_of(&link.bytes[..8]), (0x1111_0000, 0x1111_0000));
            assert_eq!(link.bytes[8..], [0; 8], "reported {reported}");
        }
    }

    #[test]
    fn the_production_sizes_hold_together()
    {
        const { assert!(PREFILL_FRAMES <= RING_FRAMES) };
        const { assert!(BLOCK_FRAMES <= PREFILL_FRAMES) };
        assert_eq!(PCM_FRAME_BYTES * WIDENING, SLOT_FRAME_BYTES);
    }
}
