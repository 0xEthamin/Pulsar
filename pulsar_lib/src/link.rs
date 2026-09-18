//! The control link as the processing board reads it.
//!
//! The interface board sends `ToDsp` frames over a UART at `LINK_BAUD`, eight
//! data bits, no parity, one stop bit. The processing board reads that receiver
//! by polling it once per served block, and turns what it reads into the gain
//! the carry walks across the block. No interrupt serves the receiver.
//!
//! Nothing here touches a peripheral. `ControlPort` is the interface a register
//! block implements, `ControlLink` is the state the served path keeps between
//! two blocks, and `LinkedGain` is the pair of the two that `serve` asks for a
//! gain.
//!
//! # What one block can receive
//!
//! RM0433 section 48.4 gives the receiver a FIFO of 16 characters, and the
//! note under `RXFT` in section 48.7.9 places one more in the shift register:
//! the 17th character does not overrun, the 18th does. `RX_HOLD_BYTES` is that
//! count. A character takes ten bit periods on this frame, so the line carries
//! 11520 characters a second, 57.6 in one block of 5.000 milliseconds. A
//! sender that fills the line therefore overruns this receiver, and the sender
//! is what paces the traffic: 17 characters or fewer between two polls. A
//! `SetVolume` frame is 7 characters and a heartbeat 5, so that is two volume
//! frames, or one and two heartbeats.
//!
//! # A line error
//!
//! An overrun, a framing error, a noise error or a parity error clears the
//! flags, drops the character the receiver flagged, and discards the frame the
//! reader was assembling. The next frame start resynchronises the reader. None
//! of them reaches the fault path, and none of them moves the gain: a lost
//! frame leaves the gain where the last accepted one put it.
//!
//! # The clock
//!
//! `ControlState` runs on a millisecond count its caller owns. Here that count
//! is the audio the served path produced, advanced by the frame count of each
//! block it carries, with the remainder of each division carried into the
//! next. It therefore moves exactly as fast as the converters play and no
//! faster, whatever the core does between two blocks. A block that is never
//! served advances nothing, so a core that stops freezes this count while the
//! output transfers keep replaying their buffers at full level. Nothing here
//! bounds that, and nothing else does until a watchdog reloaded from the
//! transfer interrupt exists.
//!
//! # Link loss
//!
//! `ControlState::heartbeat_silence_ms` measures the silence since the last
//! heartbeat. Once it reaches `LINK_SILENCE_LIMIT_MS`, the link requests a
//! volume of zero, and the gain descends through the ramp every volume change
//! takes. A volume that arrives while the link is silent is kept and not
//! applied. The next heartbeat applies the last volume the sender asked for,
//! and the gain climbs back through the same ramp.
//!
//! # What the listener hears at power up
//!
//! Silence. `ControlState` builds at a volume of zero, so the gain is exactly
//! zero until the first `SetVolume` arrives, and the ramp starts there.
//!
//! # Presets
//!
//! The chain carries the protective crossover and nothing else, so a
//! `SelectPreset` reaches no coefficient and the link drops it. Wiring presets
//! takes a chain that reloads sections and clears their history on the silent
//! block `ControlState` swaps on.

use crate::clock::AUDIO_PLAN;
use crate::constants::SAMPLE_RATE_HZ;
use crate::control::{Applied, ControlState};
use crate::passthrough::{BlockGain, Half, InputPlan};
use crate::protocol::{MAX_FRAME_LEN, Message, ProtocolError, ToDsp, Volume, resync_offset};
use crate::transport::{TONE_SAMPLE_COUNT, TransportPlan};

/// Baud rate of the control link.
pub const LINK_BAUD: u32 = 115_200;

/// Frequency of the kernel clock the link receiver runs on, in hertz.
///
/// RM0433 section 8.7.2, `HSIDIV` resets to a division by one, which gives
/// `hsi_ker_ck` at 64 MHz. The firmware leaves `HSIDIV` at its reset value, and
/// selects that clock for the receiver through `LINK_KERNEL_CLOCK_SELECT`, so
/// the rate does not follow the bus dividers.
pub const LINK_KERNEL_CLOCK_HZ: u32 = 64_000_000;

/// `USART234578SEL` value selecting `hsi_ker_ck`.
///
/// RM0433 section 8.7.21, value `011`.
pub const LINK_KERNEL_CLOCK_SELECT: u8 = 0b011;

/// `USART_BRR` value for `LINK_BAUD` at `LINK_KERNEL_CLOCK_HZ`, oversampling
/// by 16.
///
/// RM0433 section 48.5.7, the rate is the kernel clock over `USARTDIV`, and
/// with `OVER8` at 0 the register holds `USARTDIV` whole. This rounds to the
/// nearest divisor: 556, for 115108 baud.
pub const LINK_BAUD_DIVISOR: u16 = baud_divisor(LINK_KERNEL_CLOCK_HZ, LINK_BAUD);

/// Alternate function putting `USART3_TX` on PB10 and `USART3_RX` on PB11.
///
/// STM32H743VI datasheet, port B alternate function table: both rows carry
/// USART3 in the AF7 column.
pub const LINK_PIN_FUNCTION: u8 = 7;

/// `PUPDR` value holding the receive pin high, the idle level of the line,
/// while no sender drives it. RM0433 section 11.4.4, value `01`.
pub const LINK_RX_PULL: u8 = 0b01;

/// Characters the receiver holds between two polls without an overrun.
///
/// RM0433 section 48.7.9, note under `RXFT`: 16 characters in the FIFO and
/// the register, and the 17th in the shift register. The 18th overruns.
pub const RX_HOLD_BYTES: u32 = 17;

/// Characters one poll reads at most.
///
/// Twice what the receiver holds, which bounds the time a poll adds to the
/// served block and covers a character landing while the poll runs.
const RECEIVE_BUDGET: u32 = 2 * RX_HOLD_BYTES;

const _: () = assert!
(
    RECEIVE_BUDGET >= RX_HOLD_BYTES && RECEIVE_BUDGET <= 2 * RX_HOLD_BYTES,
    "one poll empties the receiver, and what it adds to the served block stays \
     bounded by what the receiver holds"
);

/// Words the input buffer holds, two per frame over one lap of it.
const SERVED_BUFFER_WORDS: u32 = TONE_SAMPLE_COUNT * 2;

/// The input plan the processing board serves, built from the audio clock and
/// the length of the input buffer the way the firmware builds it.
///
/// No count read off it depends on where the buffers sit, so the two addresses
/// are zero here.
const SERVED_PLAN: InputPlan = InputPlan::for_transport
(
    &TransportPlan::for_clock(&AUDIO_PLAN, 0, 0, SERVED_BUFFER_WORDS),
    0,
);

/// Milliseconds of audio the longer served block carries, rounded up, which is
/// how often the receiver is polled.
///
/// A lap holds an odd number of frames, so the two blocks differ by one frame
/// and a sender has to fit its traffic in the longer of the two.
pub const RX_POLL_PERIOD_MS: u32 = block_ms(SERVED_PLAN.carry_frames(Half::Second));

/// Silence on the link, in milliseconds of audio, after which the gain
/// descends to zero.
pub const LINK_SILENCE_LIMIT_MS: u32 = 3_000;

/// Milliseconds in one second.
const MILLISECONDS_PER_SECOND: u32 = 1_000;

/// Largest deviation of the link rate from `LINK_BAUD`, in parts per ten
/// thousand, that the divisor may leave.
///
/// RM0433 table 398 tolerates 3.33 per cent on the receiver with this frame,
/// oversampling by 16 and a divisor whose low four bits are not zero. The
/// STM32H743VI datasheet table 47 gives the internal oscillator 0.47 per cent
/// at 30 degrees, 1 per cent over temperature and 0.12 per cent over supply.
/// Half a per cent keeps the divisor a small part of what remains.
const MAX_BAUD_DEVIATION_PER_TEN_THOUSAND: u64 = 50;

/// Returns the divisor nearest `clock_hz` over `baud`.
///
/// A zero `baud` returns zero, which the assertion below refuses.
#[expect
(
    clippy::cast_possible_truncation,
    reason = "the value is compared against the width of the field before it \
              narrows, and the assertion below refuses anything that does not fit"
)]
const fn baud_divisor(clock_hz: u32, baud: u32) -> u16
{
    let Some(rounded) = clock_hz.saturating_add(baud / 2).checked_div(baud)
    else
    {
        return 0;
    };
    if rounded > u16::MAX as u32
    {
        return 0;
    }
    rounded as u16
}

const _: () = assert!
(
    LINK_BAUD_DIVISOR >= 16,
    "RM0433 section 48.5.7 requires USARTDIV at 16 or above"
);

const _: () = assert!
(
    (LINK_KERNEL_CLOCK_HZ as u64).abs_diff(LINK_BAUD_DIVISOR as u64 * LINK_BAUD as u64)
        * 10_000
        <= MAX_BAUD_DEVIATION_PER_TEN_THOUSAND * LINK_KERNEL_CLOCK_HZ as u64,
    "the divisor leaves the link rate too far from the rate the sender runs at"
);

const _: () = assert!
(
    MAX_FRAME_LEN <= RX_HOLD_BYTES as usize,
    "a frame the receiver cannot hold whole between two polls overruns on every \
     message"
);

/// What the receiver reports about the character at the head of its FIFO.
///
/// Each field is one flag of `USART_ISR`, read as the register holds it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[expect
(
    clippy::struct_excessive_bools,
    reason = "each field is one status bit of the register, and grouping them \
              would name a state the part does not report"
)]
pub struct PortStatus
{
    /// `RXFNE`: the FIFO holds a character.
    pub data_ready: bool,
    /// `ORE`: a character arrived while the FIFO was full, and was lost.
    pub overrun: bool,
    /// `NE`: the character at the head of the FIFO was sampled over noise.
    pub noise: bool,
    /// `FE`: the character at the head of the FIFO has no stop bit.
    pub framing: bool,
    /// `PE`: the character at the head of the FIFO fails its parity.
    pub parity: bool,
}

impl PortStatus
{
    /// Returns whether any of the four error flags stands.
    const fn faulted(self) -> bool
    {
        self.overrun || self.noise || self.framing || self.parity
    }
}

/// The receiver of the control link, as a register block exposes it.
///
/// The three methods carry no decision: which character to keep and when to
/// clear a flag is `ControlLink::drain`, on the side of the build a host test
/// reaches.
pub trait ControlPort
{
    /// Reads the receive flags of `USART_ISR`.
    fn status(&self) -> PortStatus;

    /// Reads `USART_RDR`, which takes the character at the head of the FIFO
    /// off it.
    fn take_byte(&mut self) -> u8;

    /// Clears `ORE`, `NE`, `FE` and `PE` through `USART_ICR`, and no other
    /// flag.
    ///
    /// RM0433 section 48.5.5 stores `NE`, `FE` and `PE` beside the character
    /// they belong to, and reading `USART_RDR` brings the next character to the
    /// head, so a caller clears before it reads.
    fn clear_errors(&mut self);
}

/// The bytes of a frame the link has started to receive.
///
/// It holds at most `MAX_FRAME_LEN` bytes. A decode runs after every byte, and
/// a frame of that length decodes or fails at its last byte, so the buffer
/// never needs a byte more.
#[derive(Debug)]
struct FrameReader
{
    bytes: [u8; MAX_FRAME_LEN],
    held: usize,
}

impl FrameReader
{
    /// Returns a reader holding nothing.
    const fn empty() -> Self
    {
        Self
        {
            bytes: [0; MAX_FRAME_LEN],
            held: 0,
        }
    }

    /// Drops every byte held.
    fn discard(&mut self)
    {
        self.held = 0;
    }

    /// Appends `byte` and hands every message the held bytes complete to
    /// `deliver`.
    ///
    /// The protocol rules of `crate::protocol` apply: `Truncated` waits for
    /// the next byte, and any other error drops `resync_offset` bytes and
    /// decodes again.
    fn push<F>(&mut self, byte: u8, mut deliver: F)
    where
        F: FnMut(ToDsp),
    {
        if self.held >= MAX_FRAME_LEN
        {
            self.drop_front(1);
        }
        if let Some(slot) = self.bytes.get_mut(self.held)
        {
            *slot = byte;
            self.held = self.held.saturating_add(1);
        }

        while self.held > 0
        {
            let held = self.bytes.get(..self.held).unwrap_or(&[]);
            match ToDsp::decode(held)
            {
                Ok((message, used)) =>
                {
                    self.drop_front(used);
                    deliver(message);
                }
                Err(ProtocolError::Truncated) => return,
                Err(_) =>
                {
                    let skip = resync_offset(held).map_or(self.held, usize::from);
                    self.drop_front(skip);
                }
            }
        }
    }

    /// Drops the first `count` bytes held and moves the rest to the front.
    fn drop_front(&mut self, count: usize)
    {
        let count = count.min(self.held);
        for from in count..self.held
        {
            let byte = self.bytes.get(from).copied().unwrap_or(0);
            if let Some(slot) = self.bytes.get_mut(from.saturating_sub(count))
            {
                *slot = byte;
            }
        }
        self.held = self.held.saturating_sub(count);
    }
}

/// A millisecond count advanced by the audio the served path produces.
#[derive(Debug)]
struct AudioMillis
{
    /// Whole milliseconds produced, wrapping.
    ms: u32,
    /// Frames times 1000 not yet counted as a whole millisecond, below
    /// `SAMPLE_RATE_HZ`.
    remainder: u32,
}

impl AudioMillis
{
    /// Returns a count at zero.
    const fn zero() -> Self
    {
        Self
        {
            ms: 0,
            remainder: 0,
        }
    }

    /// Advances the count by `frames` frames and returns it.
    ///
    /// The sum stays in 32 bits for any block this path serves: the remainder
    /// is below the sample rate, and a frame count stays below a lap of the
    /// buffer.
    fn advance(&mut self, frames: u32) -> u32
    {
        let total = self
            .remainder
            .saturating_add(frames.saturating_mul(MILLISECONDS_PER_SECOND));
        let whole = total / SAMPLE_RATE_HZ;
        self.remainder = total % SAMPLE_RATE_HZ;
        self.ms = self.ms.wrapping_add(whole);
        self.ms
    }
}

/// Returns the period of a block of `frames` frames, in milliseconds, rounded
/// up.
///
/// Rounded up, the cap `ControlState::poll` applies never cuts the elapsed
/// count `AudioMillis` hands it, since that count moves by the rounded down
/// period plus at most one carried millisecond.
const fn block_ms(frames: u32) -> u32
{
    frames
        .saturating_mul(MILLISECONDS_PER_SECOND)
        .div_ceil(SAMPLE_RATE_HZ)
}

/// What the link lets through to `ControlState`, and the link loss behind it.
#[derive(Debug)]
struct LinkGate
{
    state: ControlState,
    /// The last volume the sender asked for.
    volume: Volume,
    /// The link fell silent and the gain was sent to zero.
    silenced: bool,
}

impl LinkGate
{
    /// Takes one decoded message.
    fn take(&mut self, message: ToDsp)
    {
        match message
        {
            ToDsp::SetVolume(volume) =>
            {
                self.volume = volume;
                if !self.silenced
                {
                    self.state.request(message);
                }
            }
            ToDsp::Heartbeat =>
            {
                self.state.request(message);
                if self.silenced
                {
                    self.silenced = false;
                    self.state.request(ToDsp::SetVolume(self.volume));
                }
            }
            ToDsp::SelectPreset(_) => {}
        }
    }

    /// Sends the gain to zero once the link has been silent long enough, then
    /// advances the ramp over one block.
    fn poll(&mut self, now_ms: u32, buffer_ms: u32) -> Applied
    {
        if !self.silenced && self.state.heartbeat_silence_ms(now_ms) >= LINK_SILENCE_LIMIT_MS
        {
            self.silenced = true;
            self.state.request(ToDsp::SetVolume(Volume::MUTED));
        }
        self.state.poll(now_ms, buffer_ms)
    }
}

/// The state the served path keeps for the control link between two blocks.
///
/// The firmware parks one next to the filter chain and lends it to each
/// served block through `LinkedGain`. It is neither `Copy` nor `Clone`, for the
/// reason `ControlState` is not.
#[derive(Debug)]
pub struct ControlLink
{
    reader: FrameReader,
    clock: AudioMillis,
    gate: LinkGate,
}

impl ControlLink
{
    /// Returns the link as it stands at power up: no byte held, the clock at
    /// zero, and a gain of exactly zero.
    #[must_use]
    pub fn at_power_up() -> Self
    {
        Self
        {
            reader: FrameReader::empty(),
            clock: AudioMillis::zero(),
            gate: LinkGate
            {
                state: ControlState::new(0),
                volume: Volume::MUTED,
                silenced: false,
            },
        }
    }

    /// Reads what the receiver holds, up to `RECEIVE_BUDGET` characters, and
    /// takes every message it completes.
    ///
    /// The flags of a character flagged with an error are cleared, that
    /// character is read and dropped, and the frame in assembly is discarded.
    /// The clear comes first because the flags belong to the character at the
    /// head of the receiver and the read brings the next one there.
    pub fn drain<P>(&mut self, port: &mut P)
    where
        P: ControlPort,
    {
        let Self { reader, gate, .. } = self;
        for _ in 0..RECEIVE_BUDGET
        {
            let status = port.status();
            if status.faulted()
            {
                port.clear_errors();
                if status.data_ready
                {
                    let _ = port.take_byte();
                }
                reader.discard();
                continue;
            }
            if !status.data_ready
            {
                return;
            }
            let byte = port.take_byte();
            reader.push(byte, |message| gate.take(message));
        }
    }

    /// Advances the clock by one block of `frames` frames and returns the gain
    /// that block walks.
    pub fn next_block(&mut self, frames: u32) -> Applied
    {
        let now_ms = self.clock.advance(frames);
        self.gate.poll(now_ms, block_ms(frames))
    }
}

/// A control link and the receiver it reads, lent to `serve` for one block.
pub struct LinkedGain<'a, P>
{
    link: &'a mut ControlLink,
    port: &'a mut P,
}

impl<'a, P> LinkedGain<'a, P>
where
    P: ControlPort,
{
    /// Pairs `link` with the receiver `port`.
    pub fn new(link: &'a mut ControlLink, port: &'a mut P) -> Self
    {
        Self
        {
            link,
            port,
        }
    }
}

impl<P> BlockGain for LinkedGain<'_, P>
where
    P: ControlPort,
{
    /// Reads the receiver, then advances the ramp over the block.
    ///
    /// A message read here moves the gain from the next block on, which is
    /// the order `ControlState::poll` documents.
    fn block(&mut self, frames: u32) -> Applied
    {
        self.link.drain(self.port);
        self.link.next_block(frames)
    }
}

#[cfg(test)]
mod tests
{
    // A test reports a broken invariant by failing, and every count below is a
    // number of blocks or bytes far from the width of its type.
    #![allow
    (
        clippy::panic,
        clippy::arithmetic_side_effects,
        clippy::cast_possible_truncation,
        clippy::cast_precision_loss,
        clippy::float_cmp
    )]

    use std::collections::VecDeque;
    use std::vec::Vec;

    use super::*;
    use crate::constants::GAIN_FULL_SWING_MS;
    use crate::protocol::{COARSE_DEFAULT, COARSE_MAX, FINE_MAX, Preset};

    /// Frame counts of the two blocks the served plan carries in turn.
    ///
    /// A lap of the buffer holds 441 frames, and the frame that straddles the
    /// two halves belongs to the second. This is the period of the caller every
    /// timebase below runs on, so the assertion under it ties the pair to
    /// `SERVED_PLAN` rather than leaving it a list written by hand.
    const BLOCKS: [u32; 2] = [220, 221];

    const _: () = assert!
    (
        BLOCKS[0] == SERVED_PLAN.carry_frames(Half::First)
            && BLOCKS[1] == SERVED_PLAN.carry_frames(Half::Second),
        "the blocks these tests measure time on are the blocks the served plan \
         carries"
    );

    /// One character the mock receiver holds, and the flags it arrived with.
    #[derive(Clone, Copy)]
    struct Char
    {
        byte: u8,
        framing: bool,
    }

    /// A receiver fed by the test.
    #[derive(Default)]
    struct MockPort
    {
        fifo: VecDeque<Char>,
        overrun: bool,
        taken: u32,
        cleared: u32,
    }

    impl MockPort
    {
        fn send(&mut self, bytes: &[u8])
        {
            for byte in bytes
            {
                self.fifo.push_back(Char { byte: *byte, framing: false });
            }
        }
    }

    impl ControlPort for MockPort
    {
        fn status(&self) -> PortStatus
        {
            let head = self.fifo.front();
            PortStatus
            {
                data_ready: head.is_some(),
                overrun: self.overrun,
                framing: head.is_some_and(|c| c.framing),
                ..PortStatus::default()
            }
        }

        fn take_byte(&mut self) -> u8
        {
            self.taken += 1;
            self.fifo.pop_front().map_or(0, |c| c.byte)
        }

        fn clear_errors(&mut self)
        {
            // The framing flag belongs to the character at the head of the
            // FIFO, so a clear reaches that one and leaves the characters
            // behind it with theirs.
            self.cleared += 1;
            self.overrun = false;
            if let Some(head) = self.fifo.front_mut()
            {
                head.framing = false;
            }
        }
    }

    /// Returns the frame `message` travels as.
    fn frame(message: ToDsp) -> Vec<u8>
    {
        let mut out = [0_u8; MAX_FRAME_LEN];
        match message.encode_into(&mut out)
        {
            Ok(used) => out.get(..used).unwrap_or(&[]).to_vec(),
            Err(error) => panic!("{message:?} did not encode: {error:?}"),
        }
    }

    fn volume(coarse: u8, fine: u8) -> Volume
    {
        match Volume::new(coarse, fine)
        {
            Ok(volume) => volume,
            Err(error) => panic!("volume rejected: {error:?}"),
        }
    }

    /// Serves `count` blocks at the real lengths and returns every pair handed
    /// out, sending a heartbeat ahead of each block whose index `beat` accepts.
    fn run<B>(link: &mut ControlLink, port: &mut MockPort, count: u32, beat: B) -> Vec<Applied>
    where
        B: Fn(u32) -> bool,
    {
        let mut out = Vec::new();
        for block in 0..count
        {
            if beat(block)
            {
                port.send(&frame(ToDsp::Heartbeat));
            }
            let frames = BLOCKS.get(block as usize % 2).copied().unwrap_or(220);
            out.push(LinkedGain::new(link, port).block(frames));
        }
        out
    }

    /// Blocks in `ms` milliseconds of audio, rounded up.
    ///
    /// One lap of the buffer is the two blocks of `BLOCKS` together, so the
    /// count comes off the same source the block lengths do.
    fn blocks_in(ms: u32) -> u32
    {
        let lap_frames: u32 = BLOCKS.iter().sum();
        let blocks_per_lap = BLOCKS.len() as u32;
        (ms * blocks_per_lap * SAMPLE_RATE_HZ).div_ceil(lap_frames * 1000)
    }

    /// Heartbeat every 20 blocks, about 100 ms.
    fn beating(block: u32) -> bool
    {
        block.is_multiple_of(20)
    }

    #[test]
    fn the_timebase_of_these_tests_is_the_one_the_served_plan_carries()
    {
        assert_eq!(BLOCKS.first().copied(), Some(SERVED_PLAN.carry_frames(Half::First)));
        assert_eq!(BLOCKS.get(1).copied(), Some(SERVED_PLAN.carry_frames(Half::Second)));

        // The two blocks cover a lap of the buffer exactly once between them,
        // which is what makes the pair a period and not two numbers.
        let lap_frames: u32 = BLOCKS.iter().sum();
        assert_eq!(lap_frames, SERVED_BUFFER_WORDS / SERVED_PLAN.frame_slots());
        assert_eq!(RX_POLL_PERIOD_MS, block_ms(lap_frames - BLOCKS.first().copied().unwrap_or(0)));
        assert_eq!(RX_POLL_PERIOD_MS, 6);
    }

    #[test]
    fn the_divisor_lands_near_the_link_rate()
    {
        assert_eq!(LINK_BAUD_DIVISOR, 556);
        let rate = f64::from(LINK_KERNEL_CLOCK_HZ) / f64::from(LINK_BAUD_DIVISOR);
        let deviation = (rate - f64::from(LINK_BAUD)).abs() / f64::from(LINK_BAUD);
        assert!(deviation < 0.001, "the link runs {deviation} off its rate");
        assert_eq!(baud_divisor(LINK_KERNEL_CLOCK_HZ, 0), 0);
        assert_eq!(baud_divisor(u32::MAX, 1), 0);
    }

    #[test]
    fn the_audio_clock_counts_every_frame_it_is_handed()
    {
        // Twenty million blocks is more than a day of audio, so the count
        // wraps and the remainder has carried through every phase.
        let mut clock = AudioMillis::zero();
        let mut frames = 0_u64;
        let mut last = 0_u32;
        for block in 0..20_000_000_u32
        {
            let length = BLOCKS.get(block as usize % 2).copied().unwrap_or(220);
            frames += u64::from(length);
            let now = clock.advance(length);
            let step = now.wrapping_sub(last);
            assert!(step <= block_ms(length), "block {block} moved the clock {step} ms");
            last = now;
            if block % 1_000_003 == 0
            {
                assert_eq!(u64::from(now), (frames * 1000 / 44_100) % (1_u64 << 32));
            }
        }
        assert_eq!(u64::from(last), (frames * 1000 / 44_100) % (1_u64 << 32));
    }

    #[test]
    fn a_block_declares_a_period_no_step_of_the_clock_exceeds()
    {
        for length in 0..=882
        {
            assert!(u64::from(block_ms(length)) * 44_100 >= u64::from(length) * 1000);
            let mut clock = AudioMillis::zero();
            let mut last = 0;
            for _ in 0..500
            {
                let now = clock.advance(length);
                assert!(now - last <= block_ms(length), "a block of {length} frames");
                last = now;
            }
        }
        assert_eq!(block_ms(220), 5);
        assert_eq!(block_ms(221), 6);
    }

    #[test]
    fn the_gain_is_exactly_zero_until_a_volume_arrives()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        for applied in run(&mut link, &mut port, 2_000, beating)
        {
            assert_eq!(applied.gain_start(), 0.0);
            assert_eq!(applied.gain_end(), 0.0);
        }
    }

    #[test]
    fn a_volume_arriving_byte_by_byte_moves_the_gain_once_complete()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        let wanted = volume(COARSE_MAX, FINE_MAX);
        let bytes = frame(ToDsp::SetVolume(wanted));

        port.send(&frame(ToDsp::Heartbeat));
        for (at, byte) in bytes.iter().enumerate()
        {
            port.send(&[*byte]);
            let applied = LinkedGain::new(&mut link, &mut port).block(220);
            assert_eq!(applied.gain_end(), 0.0, "the gain moved at byte {at}");
        }
        assert_eq!(link.gate.state.applied_volume(), wanted);

        let after = run(&mut link, &mut port, blocks_in(GAIN_FULL_SWING_MS) + 2, beating);
        assert!(after.first().is_some_and(|a| a.gain_end() > 0.0), "the ramp did not start");
        assert_eq!(after.last().map(|a| a.gain_end()), Some(1.0));
    }

    #[test]
    fn every_frame_survives_the_noise_ahead_of_it()
    {
        let mut seed = 0x9E37_79B9_u32;
        let mut next = move ||
        {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed
        };

        let mut sent = Vec::new();
        let mut line = Vec::new();
        for _ in 0..20_000
        {
            for _ in 0..next() % 12
            {
                // Noise leans on the start byte, which is what makes a false
                // start and a resync.
                let byte = if next() % 3 == 0 { 0xA5 } else { next() as u8 };
                line.push(byte);
            }
            let message = match next() % 3
            {
                0 => ToDsp::Heartbeat,
                1 => ToDsp::SelectPreset(Preset::Garden),
                _ => ToDsp::SetVolume(volume((next() % 14) as u8, (next() % 128) as u8)),
            };
            sent.push(message);
            line.extend(frame(message));
        }

        let mut reader = FrameReader::empty();
        let mut got = Vec::new();
        for byte in &line
        {
            reader.push(*byte, |message| got.push(message));
            assert!(reader.held < MAX_FRAME_LEN, "the reader filled up");
        }

        let mut want = sent.iter().peekable();
        let mut extra = 0;
        for message in &got
        {
            if want.peek() == Some(&message)
            {
                want.next();
            }
            else
            {
                extra += 1;
            }
        }
        assert_eq!(want.count(), 0, "a frame behind noise was lost");
        assert_eq!(extra, 0, "noise decoded as a frame");
    }

    #[test]
    fn a_line_error_drops_the_frame_in_assembly_and_nothing_else()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        let first = frame(ToDsp::SetVolume(volume(COARSE_MAX, FINE_MAX)));
        let second = frame(ToDsp::SetVolume(volume(COARSE_DEFAULT, FINE_MAX)));

        port.send(&frame(ToDsp::Heartbeat));
        port.send(first.get(..3).unwrap_or(&[]));
        port.fifo.push_back(Char { byte: 0x5A, framing: true });
        port.send(first.get(3..).unwrap_or(&[]));
        port.send(&second);
        link.drain(&mut port);

        assert_eq!(port.cleared, 1);
        assert!(port.fifo.is_empty());
        assert_eq!(link.gate.state.applied_volume(), Volume::MUTED, "a torn frame was taken");
        let _ = link.next_block(220);
        assert_eq!(link.gate.state.applied_volume(), volume(COARSE_DEFAULT, FINE_MAX));
    }

    #[test]
    fn an_overrun_is_cleared_and_the_link_carries_on()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort { overrun: true, ..MockPort::default() };
        link.drain(&mut port);
        assert_eq!((port.cleared, port.taken), (1, 0), "a flag with no character read one");

        let wanted = volume(COARSE_DEFAULT, 100);
        port.overrun = true;
        port.send(&[0xA5, 0x03]);
        port.send(&frame(ToDsp::SetVolume(wanted)));
        link.drain(&mut port);
        let _ = link.next_block(221);
        assert_eq!(port.cleared, 2);
        assert_eq!(link.gate.state.applied_volume(), wanted);
    }

    #[test]
    fn a_poll_reads_no_more_than_its_budget()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        port.send(&[0_u8; 100]);
        link.drain(&mut port);
        assert_eq!(port.taken, RECEIVE_BUDGET);

        // The bound is what the receiver holds, not the constant itself: a
        // budget that grew past it would stretch the served block instead of
        // just emptying a receiver that cannot hold more.
        assert!
        (
            port.taken >= RX_HOLD_BYTES && port.taken <= 2 * RX_HOLD_BYTES,
            "a poll read {} characters and the receiver holds {RX_HOLD_BYTES}",
            port.taken
        );

        let mut flagged = MockPort::default();
        for _ in 0..100
        {
            flagged.fifo.push_back(Char { byte: 0xA5, framing: true });
        }
        link.drain(&mut flagged);
        assert_eq!(flagged.taken + flagged.cleared, 2 * RECEIVE_BUDGET);
    }

    #[test]
    fn a_flag_of_the_next_character_is_not_cleared_with_the_one_before_it()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        let wanted = volume(COARSE_DEFAULT, 0x11);

        // Two characters in a row arrive torn. Clearing after the read would
        // wipe the flag the second one brought with it, and that character
        // would enter a frame with only the checksum against it.
        port.fifo.push_back(Char { byte: 0x5A, framing: true });
        port.fifo.push_back(Char { byte: 0x5B, framing: true });
        port.send(&frame(ToDsp::SetVolume(wanted)));
        link.drain(&mut port);

        assert_eq!(port.cleared, 2, "one clear covered two flagged characters");
        assert_eq!(port.taken, 2 + frame(ToDsp::SetVolume(wanted)).len() as u32);
        let _ = link.next_block(220);
        assert_eq!(link.gate.state.applied_volume(), wanted);
    }

    #[test]
    fn a_frame_split_across_two_polls_is_taken()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        let wanted = volume(7, 7);
        let bytes = frame(ToDsp::SetVolume(wanted));
        port.send(bytes.get(..4).unwrap_or(&[]));
        link.drain(&mut port);
        port.send(bytes.get(4..).unwrap_or(&[]));
        link.drain(&mut port);
        let _ = link.next_block(220);
        assert_eq!(link.gate.state.applied_volume(), wanted);
    }

    #[test]
    fn a_silent_link_ramps_the_gain_down_and_a_heartbeat_brings_it_back()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        port.send(&frame(ToDsp::SetVolume(volume(COARSE_MAX, FINE_MAX))));
        let warm = run(&mut link, &mut port, 200, beating);
        assert_eq!(warm.last().map(|a| a.gain_end()), Some(1.0));

        // The last beat went out ahead of block 180, and two blocks run 10 ms,
        // so the limit falls a little under 600 blocks after it.
        let quiet = run(&mut link, &mut port, 1_000, |_| false);
        let mut first_move = None;
        for (block, pair) in quiet.iter().enumerate()
        {
            // `ControlState` bounds a swing by the audio produced plus the one
            // millisecond the clock carries, which a block of 221 frames takes.
            let frames = BLOCKS.get(block % 2).copied().unwrap_or(220);
            let audio_ms = frames as f32 * 1000.0 / SAMPLE_RATE_HZ as f32;
            let bound = (audio_ms + 1.0) / GAIN_FULL_SWING_MS as f32 + 1e-6;
            let step = (pair.gain_end() - pair.gain_start()).abs();
            assert!(step <= bound, "block {block} stepped {step}");
            assert!(pair.gain_end() <= pair.gain_start(), "the gain rose at block {block}");
            if first_move.is_none() && pair.gain_end() < 1.0
            {
                first_move = Some(block);
            }
        }
        // The beat is taken on block 180 of the warm run, whose audio the
        // silence counts, and the gain moves on the block after the one that
        // requested the descent. So the silence at the request covers the 20
        // blocks left of the warm run and `first_move` blocks of this one.
        let first_move = first_move.unwrap_or(usize::MAX) as u32;
        let silent_ms = (20 + first_move) * 5;
        assert!
        (
            (LINK_SILENCE_LIMIT_MS..LINK_SILENCE_LIMIT_MS + 15).contains(&silent_ms),
            "the descent started {silent_ms} ms after the last beat"
        );
        assert!(link.gate.silenced);
        assert_eq!(quiet.last().map(|a| a.gain_end()), Some(0.0));

        // A volume on a silent link is kept, not applied.
        let later = volume(COARSE_DEFAULT, FINE_MAX);
        port.send(&frame(ToDsp::SetVolume(later)));
        for pair in run(&mut link, &mut port, 100, |_| false)
        {
            assert_eq!(pair.gain_end(), 0.0, "a volume moved a silent link");
        }

        let back = run(&mut link, &mut port, 200, beating);
        assert!(!link.gate.silenced);
        assert_eq!(back.last().map(|a| a.gain_end()), Some(later.linear_gain()));
    }

    #[test]
    fn a_link_that_never_beats_falls_silent_at_the_limit()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        port.send(&frame(ToDsp::SetVolume(volume(COARSE_MAX, FINE_MAX))));
        let pairs = run(&mut link, &mut port, blocks_in(LINK_SILENCE_LIMIT_MS) + 100, |_| false);
        assert!(pairs.iter().any(|a| a.gain_end() == 1.0), "the volume never applied");
        assert_eq!(pairs.last().map(|a| a.gain_end()), Some(0.0));
    }

    #[test]
    fn a_preset_request_moves_nothing()
    {
        let mut link = ControlLink::at_power_up();
        let mut port = MockPort::default();
        port.send(&frame(ToDsp::SetVolume(volume(COARSE_MAX, FINE_MAX))));
        let _ = run(&mut link, &mut port, 100, beating);
        port.send(&frame(ToDsp::SelectPreset(Preset::BassBoosted)));
        for pair in run(&mut link, &mut port, 200, beating)
        {
            assert_eq!(pair.gain_start(), 1.0);
            assert_eq!(pair.gain_end(), 1.0);
            assert_eq!(pair.preset(), Preset::Flat);
        }
    }
}
