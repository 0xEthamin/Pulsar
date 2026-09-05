//! Output transport plan of the processing board, and the proof it took.
//!
//! Four converter channels are carried by one audio interface: sub-block A is
//! the master transmitter and sub-block B is a transmitter synchronous with it,
//! so the two share one bit clock and one frame clock and cannot drift a sample
//! apart. One circular transfer stream feeds each sub-block from a buffer of
//! whole tone periods, which is why nothing here runs per sample.
//!
//! Nothing in this module touches a peripheral. `AudioInterface` is the
//! interface a register block implements, and `bring_up` is the sequence run
//! over it.
//!
//! # The frame this builds
//!
//! Standard I2S, 32 bit slots, two slots per frame, most significant bit
//! first, 64 bit clock periods per frame. The converter clocks the data line on
//! the rising edge of the bit clock, so the interface changes it on the falling
//! edge. Its left channel is the low half of the frame clock and the word
//! starts one bit clock after the frame clock moves, which is what the frame
//! offset and the frame polarity below encode.
//!
//! The frame length and the master clock divider are not settings here. They
//! come off the clock plan, so one validated divider chain sets the sample
//! rate, the bit clock and the frame length together.
//!
//! # Why a read-back
//!
//! The interface reports no bit meaning "configured as asked". It reports one
//! error flag for a frame the master clock cannot divide into, `WCKCFG`, and
//! one for a starved transmitter, `OVRUDR`, and both are read here. Everything
//! else, a slot the transfer never fills, a stream pointed at the other
//! sub-block, a buffer the transfer controllers read as zeros, comes back as a
//! plausible frame carrying the wrong thing. So every field the bring-up drives
//! off its reset value in the two sub-blocks and the two streams is read back
//! and compared against the plan.
//!
//! # What the read-back does not reach
//!
//! The port. `AudioInterface::open_pins` puts five pins on the alternate
//! function that carries the frame, and no field of it appears in a read-back
//! here or anywhere in this firmware. A port write that does not land leaves
//! the pins where a reset put them, the frame reaches no pad, and every field
//! this module does compare still reads back as the plan. What covers it is the
//! register block: it pins the function, the mode and the speed to the
//! encodings the port carries, at compile time. What settles it is an
//! oscilloscope.
//!
//! That is the shape of the limit generally. The read-back proves the fields,
//! never the wires. Which physical pin carries which signal, and whether the
//! converter downstream reads the format this frame writes, are settled on the
//! bench.

use crate::clock::{ClockPlan, wait_polls};
use crate::constants::SAMPLE_RATE_HZ;

/// Samples in the tone table.
///
/// Ten periods of the tone over this many samples at 44100 Hz, so the table
/// ends one sample short of repeating. A circular replay of a whole number of
/// copies therefore carries no discontinuity at the wrap and needs no phase
/// accumulator.
const TONE_SAMPLE_COUNT: u32 = 441;

/// Samples in the tone table, as a length.
pub const TONE_SAMPLES: usize = TONE_SAMPLE_COUNT as usize;

/// Whole periods of the tone the table holds.
const TONE_PERIODS: u32 = 10;

/// Frequency of the tone the table holds, in hertz.
const TONE_HZ: u32 = SAMPLE_RATE_HZ * TONE_PERIODS / TONE_SAMPLE_COUNT;

/// Peak of the tone as a fraction of converter full scale.
///
/// One eighth, which is 18.06 dB below full scale. Exact in binary, so the
/// table carries no rounding of its own, and an exact fraction of full scale is
/// what turns a reading at a probe tip back into dBFS once the measurement
/// chain has to be calibrated against a level the drivers depend on.
///
/// This value protects nothing and no run time path reads it. What sets it is
/// that a person eventually puts a headphone on a converter line output to hear
/// this tone. A sustained tone at one kilohertz sits where the ear is most
/// sensitive, and the converter module drives its output through a passive
/// filter with no buffer behind it, so a low impedance load on a large signal
/// would judge the bench wiring rather than the converter. One eighth of the
/// 2.1 Vrms the converter reaches at full scale is 0.263 Vrms, an ordinary line
/// level.
///
/// The amplifier is not on this path. It reaches its measured clipping point
/// 0.23 dB above converter full scale, and that figure bounds the ceiling the
/// high way carries, not a stimulus that never reaches an amplifier.
const TONE_PEAK: f64 = 0.125;

/// Sample value the tone peak scales to.
const TONE_AMPLITUDE: f64 = TONE_PEAK * i32::MAX as f64;

/// Ratio of a circle to its diameter, to the precision of the type.
const PI: f64 = core::f64::consts::PI;

/// Terms of the sine series past the first.
///
/// The reduction below hands the series an angle of at most pi/2, where the
/// term in x^19 is under 5e-14 of the result, so nine terms past the first
/// reach the precision of the type.
const SINE_TERMS: u32 = 9;

/// Slots in one audio frame.
///
/// One per converter channel. RM0433 section 51.4.7 requires an even count
/// while the frame clock also identifies the channel side, which the I2S frame
/// below has it do.
const SLOTS_PER_FRAME: u8 = 2;

/// Bit clock periods in one slot.
const SLOT_BITS: u16 = 32;

/// `MODE` selecting master transmitter. RM0433 section 51.6.2.
pub const MODE_MASTER_TRANSMITTER: u8 = 0b00;

/// `MODE` selecting slave transmitter. RM0433 section 51.6.2.
pub const MODE_SLAVE_TRANSMITTER: u8 = 0b10;

/// `PRTCFG` selecting the free protocol, the one the frame registers shape.
///
/// RM0433 section 51.6.2. The other values force the AC97 or SPDIF frame and
/// ignore the frame configuration register.
pub const PROTOCOL_FREE: u8 = 0b00;

/// `DS` selecting 32 bit data. RM0433 section 51.6.2.
pub const DATA_SIZE_32: u8 = 0b111;

/// `SYNCEN` selecting an asynchronous sub-block. RM0433 section 51.6.2.
pub const SYNC_ASYNCHRONOUS: u8 = 0b00;

/// `SYNCEN` selecting synchronism with the other sub-block of the interface.
///
/// RM0433 section 51.6.2. This is what shares the bit clock and the frame clock
/// between the two sub-blocks, and section 51.4.4 releases the clock pins of
/// the synchronous one back to the port.
pub const SYNC_INTERNAL: u8 = 0b01;

/// `SLOTSZ` selecting a 32 bit slot. RM0433 section 51.6.8.
pub const SLOT_SIZE_32: u8 = 0b10;

/// `SLOTEN` value activating slot 0 and slot 1. RM0433 section 51.6.8.
const SLOTS_ENABLED: u16 = 0b11;

/// `FTH` selecting the half full transfer request. RM0433 section 51.6.4.
///
/// The request rises while under half of the eight word FIFO holds data, which
/// leaves four words of margin ahead of the underrun the empty setting would
/// run against.
pub const FIFO_THRESHOLD_HALF: u8 = 0b010;

/// `FLVL` reporting an empty FIFO. RM0433 section 51.6.12.
const FIFO_LEVEL_EMPTY: u8 = 0b000;

/// `CKSTR` putting the data change on the edge the converter does not sample.
///
/// RM0433 section 51.6.2 names the edge the interface CHANGES its outputs on:
/// 0 changes on the rising edge, 1 on the falling edge. The PCM5102A datasheet
/// section 9.3.2.1 clocks the data line in on the rising edge of the bit clock,
/// so the transmitter has to change it on the falling one.
///
/// The name says which edge the SAI changes on, not which edge anything
/// strobes on. The register field is called the clock strobing edge and its
/// two values are named after the strobing edge in the peripheral crate, so
/// the word strobe on this bit means the opposite of this value.
pub const CHANGE_ON_FALLING_EDGE: bool = true;

/// `PL` giving both output streams the high priority. RM0433 section 15.5.5.
///
/// An underrun on either sub-block is an audible defect the interface reports
/// as an error, so these two outrank anything left at the level a reset gives.
pub const STREAM_PRIORITY: u8 = 0b10;

/// `DIR` selecting a memory to peripheral transfer. RM0433 section 15.5.5.
pub const TRANSFER_MEMORY_TO_PERIPHERAL: u8 = 0b01;

/// `MSIZE` and `PSIZE` selecting a word. RM0433 section 15.5.5.
pub const TRANSFER_WORD: u8 = 0b10;

/// `DMAREQ_ID` routing the master sub-block. RM0433 section 17.3.2 table.
pub const MASTER_REQUEST: u8 = 87;

/// `DMAREQ_ID` routing the slave sub-block. RM0433 section 17.3.2 table.
pub const SLAVE_REQUEST: u8 = 88;

/// Address one sample of the master sub-block is written to.
///
/// The memory map puts the first audio interface at `0x4001_5800` and
/// RM0433 section 51.6.16 puts its data register at offset `0x020`.
const MASTER_DATA_ADDRESS: u32 = 0x4001_5820;

/// Address one sample of the slave sub-block is written to.
///
/// RM0433 section 51.6.17 puts the data register of the second sub-block at
/// offset 0x040.
const SLAVE_DATA_ADDRESS: u32 = 0x4001_5840;

/// First address of the memory the output buffers live in.
///
/// RM0433 section 2.4 puts the AXI SRAM here. It is one of several memories the
/// two system transfer controllers reach, and the one this transport places its
/// buffers in, so the bound below is where the buffers belong rather than the
/// whole of what a transfer can address.
///
/// What the bound is for is the memory a transfer CANNOT reach. Section 2.1.6
/// keeps both tightly coupled memories out of their range, and that is where
/// this firmware runs its stack and holds its statics, so an output buffer
/// declared without a section lands in one of them. What such a transfer
/// returns is not written down: section 2.1.6 gives the reachability and
/// attaches no behaviour to an access outside it, so reading zeros is the
/// expectation rather than the documented answer, and no flag is expected
/// either. A window the buffers are inside refuses the case whatever the answer
/// turns out to be, and a window the buffers are outside would need one bound
/// per reachable memory to refuse the same thing.
const BUFFER_REGION_BASE: u32 = 0x2400_0000;

/// Bytes of the memory the output buffers live in. RM0433 section 2.4.
const BUFFER_REGION_BYTES: u32 = 512 * 1024;

/// Bytes in one buffer entry.
const WORD_BYTES: u32 = 4;

/// Lowest master clock divider the field encodes. RM0433 section 51.6.2.
const MASTER_DIVIDER_MIN: u8 = 1;

/// Highest master clock divider the field encodes. RM0433 section 51.6.2.
const MASTER_DIVIDER_MAX: u8 = 63;

/// Returns the terms of the sine series summed at `angle`.
///
/// The Taylor expansion of sine about zero, carried to the term in
/// `x^(2 * SINE_TERMS + 1)`.
const fn sine_series(angle: f64) -> f64
{
    let square = angle * angle;
    let mut term = angle;
    let mut sum = angle;
    let mut order = 1_u32;

    while order <= SINE_TERMS
    {
        let even = order.saturating_mul(2);
        let odd = even.saturating_add(1);
        term = -term * square / (even as f64 * odd as f64);
        sum += term;
        order = order.saturating_add(1);
    }

    sum
}

/// Returns the sine of `numerator / denominator` of one turn.
///
/// The fold onto a quarter turn runs on the integers, so the angle handed to
/// the series carries no cancellation from a rounded phase, and the series then
/// sees at most pi/2.
///
/// A denominator of zero describes no angle and gives zero.
#[expect
(
    clippy::cast_precision_loss,
    reason = "the fold holds both values at or under the denominator, and the \
              only caller passes the table length, so neither reaches the \
              mantissa of the type"
)]
const fn sine_of_turn(numerator: u64, denominator: u64) -> f64
{
    let Some(turn) = numerator.checked_rem(denominator)
    else
    {
        return 0.0;
    };

    let halves = turn.saturating_mul(2);

    let (sign, folded) = if halves >= denominator
    {
        (-1.0, halves.saturating_sub(denominator))
    }
    else
    {
        (1.0, halves)
    };

    let quarter = if folded.saturating_mul(2) > denominator
    {
        denominator.saturating_sub(folded)
    }
    else
    {
        folded
    };

    sign * sine_series(PI * quarter as f64 / denominator as f64)
}

/// Returns `value` rounded to the nearest sample, halves away from zero.
///
/// `f64::round` is not available in a constant, and the table is built in one.
#[expect
(
    clippy::cast_possible_truncation,
    reason = "the caller scales by TONE_AMPLITUDE, which is an eighth of \
              i32::MAX, so the value handed here is well inside the type"
)]
const fn round_to_sample(value: f64) -> i32
{
    let biased = if value >= 0.0
    {
        value + 0.5
    }
    else
    {
        value - 0.5
    };

    biased as i32
}

/// Builds the tone table.
#[expect
(
    clippy::indexing_slicing,
    reason = "the loop bound is the length of the array, and a constant \
              evaluation of an index past the end fails the build"
)]
const fn build_tone_table() -> [i32; TONE_SAMPLES]
{
    let mut table = [0_i32; TONE_SAMPLES];
    let mut index = 0;

    while index < TONE_SAMPLES
    {
        let phase = (TONE_PERIODS as u64).saturating_mul(index as u64);
        let turn = sine_of_turn(phase, TONE_SAMPLE_COUNT as u64);
        table[index] = round_to_sample(TONE_AMPLITUDE * turn);
        index = index.saturating_add(1);
    }

    table
}

/// One period-aligned tone, as the converters take it.
///
/// Two samples of the table cover one bit of a 32 bit slot each, so a buffer
/// holding a whole number of copies of it replays through a circular stream
/// with no seam and no work between transfers.
pub const TONE_TABLE: [i32; TONE_SAMPLES] = build_tone_table();

/// Reason the part would not accept a transport plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportPlanError
{
    /// The frame does not hold the slots the plan declares.
    FrameLengthWrong,
    /// `MCKDIV` carries 1 to 63.
    MasterDividerOutOfRange,
    /// The buffer holds no whole number of frames.
    BufferNotWholeFrames,
    /// The buffer holds no whole number of tone periods, so a circular replay
    /// of it steps across a discontinuity once per lap.
    BufferNotWholePeriods,
    /// The buffer is empty.
    BufferEmpty,
    /// The buffer holds more words than `NDTR` counts.
    BufferTooLong,
    /// A buffer address is not on a word boundary, which a word wide transfer
    /// requires.
    BufferUnaligned,
    /// A buffer falls outside the memory this transport places its buffers in.
    BufferUnreachable,
    /// The two buffers overlap, so one sub-block reads what the other sends.
    BuffersOverlap,
}

/// Reason one audio sub-block is not running to plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockFault
{
    /// `MODE` does not carry the direction and mastership of the plan.
    ModeWrong,
    /// `PRTCFG` does not select the free protocol.
    ProtocolWrong,
    /// `DS` does not carry the data size of the plan.
    DataSizeWrong,
    /// `LSBFIRST` is set, so the frame carries the least significant bit first.
    BitOrderWrong,
    /// `CKSTR` does not put the data change on the edge opposite the one the
    /// converter samples.
    ClockStrobingWrong,
    /// `SYNCEN` does not carry the synchronisation of the plan.
    SynchronisationWrong,
    /// `MONO` is set, so slot 0 is duplicated over slot 1.
    MonoWrong,
    /// `OUTDRIV` is set, so the data line is driven before the block runs.
    OutputDriveWrong,
    /// `TRIS` is set, so the data line is released between slots.
    TristateWrong,
    /// `FTH` does not carry the transfer request threshold of the plan.
    FifoThresholdWrong,
    /// `NOMCK` is set on the master, so no master clock reaches the converter.
    MasterClockDisabled,
    /// `MCKDIV` does not carry the master clock divider of the clock plan.
    MasterDividerWrong,
    /// `OSR` is set, so the master clock is 512 frame periods and not 256.
    OversamplingWrong,
    /// `FRL` does not carry the frame length of the clock plan.
    FrameLengthWrong,
    /// `FSALL` does not hold the frame clock active for half the frame.
    FrameActiveLengthWrong,
    /// `FSDEF` is clear, so the frame clock marks no channel side.
    FrameDefinitionWrong,
    /// `FSPOL` is set, so the frame starts on the rising edge.
    FramePolarityWrong,
    /// `FSOFF` is clear, so the frame clock moves on the first bit rather than
    /// one bit ahead of it.
    FrameOffsetWrong,
    /// `FBOFF` is not zero, so the word does not start at the top of its slot.
    FirstBitOffsetWrong,
    /// `SLOTSZ` does not carry the slot size of the plan.
    SlotSizeWrong,
    /// `NBSLOT` does not carry the slot count of the plan.
    SlotCountWrong,
    /// `SLOTEN` leaves a slot of the frame inactive.
    SlotsNotEnabled,
    /// `DMAEN` is clear, so nothing fills the FIFO.
    TransferDisabled,
    /// `SAIEN` is clear.
    NotEnabled,
    /// `OVRUDR` is set, so the transmitter has already sent a frame it had no
    /// data for.
    Underrun,
    /// `WCKCFG` is set, so the part refuses the frame against the master clock
    /// it was asked to generate.
    ClockConfigurationRejected,
    /// A sub-block interrupt is enabled, and no handler of this firmware serves
    /// one, so it would reach the fault path and silence the machine.
    InterruptEnabled,
}

/// Reason one transfer stream is not running to plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamFault
{
    /// `DMAREQ_ID` routes another peripheral to the stream.
    RequestWrong,
    /// `SE` is set, so a synchronisation input gates the requests.
    SynchronisationEnabled,
    /// `DIR` does not carry a memory to peripheral transfer.
    DirectionWrong,
    /// `CIRC` is clear, so the stream stops at the end of the buffer.
    NotCircular,
    /// `MINC` is clear, so every transfer reads the same word.
    MemoryNotIncrementing,
    /// `PINC` is set, so the stream walks off the data register.
    PeripheralIncrementing,
    /// `MSIZE` does not carry a word.
    MemoryWidthWrong,
    /// `PSIZE` does not carry a word.
    PeripheralWidthWrong,
    /// `DBM` is set, so the stream expects a second buffer address.
    DoubleBuffered,
    /// `PL` does not carry the priority of the plan.
    PriorityWrong,
    /// A stream interrupt is enabled, and no handler of this firmware serves
    /// one, so it would reach the fault path and silence the machine.
    InterruptEnabled,
    /// `PAR` does not address the data register of the sub-block the stream
    /// feeds.
    PeripheralAddressWrong,
    /// `M0AR` does not carry the buffer address of the plan.
    MemoryAddressWrong,
    /// `NDTR` did not carry the buffer length of the plan when the stream was
    /// armed.
    ItemCountWrong,
    /// `NDTR` reads past the buffer length while the stream runs, so the
    /// counter belongs to no lap of it.
    CounterOutOfRange,
    /// `EN` is clear.
    NotEnabled,
}

/// Reason the output transport is not running to plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportFault
{
    /// The plan itself is not one the part accepts.
    PlanRejected(TransportPlanError),
    /// A stream still reported enabled after it was told to stop, so its
    /// configuration fields would not have taken.
    StreamNeverStopped,
    /// A sub-block still reported enabled after it was told to stop, so its
    /// configuration fields would not have taken.
    BlockNeverStopped,
    /// A FIFO never left empty once the streams were running. Enabling a slave
    /// transmitter on an empty FIFO is what RM0433 section 51.4.3 forbids.
    FifoNeverFilled,
    /// A transfer counter never moved once both sub-blocks were running, so
    /// the frame going out carries whatever the FIFO held and no more.
    TransferNeverAdvanced,
    /// The master sub-block does not carry the plan.
    MasterBlock(BlockFault),
    /// The slave sub-block does not carry the plan.
    SlaveBlock(BlockFault),
    /// The stream feeding the master sub-block does not carry the plan.
    MasterStream(StreamFault),
    /// The stream feeding the slave sub-block does not carry the plan.
    SlaveStream(StreamFault),
}

/// Which sub-block of the interface a read-back belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockRole
{
    /// Drives the bit clock, the frame clock and the master clock.
    Master,
    /// Shares the clocks of the master through the internal synchronisation.
    Slave,
}

impl BlockRole
{
    /// Returns the `MODE` value the role takes.
    const fn mode_bits(self) -> u8
    {
        match self
        {
            Self::Master => MODE_MASTER_TRANSMITTER,
            Self::Slave => MODE_SLAVE_TRANSMITTER,
        }
    }

    /// Returns the `SYNCEN` value the role takes.
    const fn sync_bits(self) -> u8
    {
        match self
        {
            Self::Master => SYNC_ASYNCHRONOUS,
            Self::Slave => SYNC_INTERNAL,
        }
    }

    /// Returns whether the clock generator of the role is in use.
    ///
    /// RM0433 section 51.4.8 turns the generator off on a slave and ignores
    /// `NOMCK`, `MCKDIV` and `OSR` there, so checking them would be checking a
    /// value the part does not read.
    const fn generates_clocks(self) -> bool
    {
        matches!(self, Self::Master)
    }
}

/// Every field of one audio sub-block the register block drives off its reset
/// value, and the two error flags.
///
/// The set is closed against that boundary rather than against the register
/// map, and the boundary is what makes it a check. Every sub-block register the
/// bring-up writes has at least one field here, and the writes it makes land
/// whole or not at all, so a field left out is a field whose reset value is the
/// wanted one and whose register a listed neighbour already witnesses.
/// `SAI_xCR2` is the case to hold this against: RM0433 section 51.6.4 resets it
/// to `0x0000_0000`, so `MUTE` and `COMP` come up clear, which is what this
/// frame wants, and `FTH` and `TRIS` below witness that the write landed.
///
/// One field of a sub-block does not follow that rule, and the sequence is what
/// keeps it out of reach. RM0433 section 51.6.2 discards a `SAIEN` write that
/// finds the bit already set while the rest of the word lands, so a word
/// carrying it is not all or nothing. `bring_up` polls the bit to zero before
/// it writes any of these registers, so no write it makes meets that case.
///
/// The claim ranges over the sub-block alone. `AudioInterface::open_pins`
/// drives fifteen port fields off their reset value and no read-back reaches
/// one, which the module documentation sets out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[expect
(
    clippy::struct_excessive_bools,
    reason = "each field is one register bit, and naming them apart is what \
              lets a fault say which bit"
)]
pub struct BlockReadback
{
    /// `MODE`.
    pub mode_bits: u8,
    /// `PRTCFG`.
    pub protocol_bits: u8,
    /// `DS`.
    pub data_size_bits: u8,
    /// `LSBFIRST`.
    pub lsb_first: bool,
    /// `CKSTR`, read as the edge the interface changes its outputs on.
    ///
    /// The bit reads 1 when the SAI changes on the falling edge, which is the
    /// value the peripheral crate names after the RISING edge because it names
    /// the edge a receiver strobes on. The two are opposite, and this field is
    /// named for what RM0433 section 51.6.2 gives the bit rather than for that.
    pub changes_on_falling_edge: bool,
    /// `SYNCEN`.
    pub sync_bits: u8,
    /// `MONO`.
    pub mono: bool,
    /// `OUTDRIV`.
    pub driven_before_start: bool,
    /// `TRIS`.
    pub tristate: bool,
    /// `FTH`.
    pub fifo_threshold_bits: u8,
    /// `NOMCK`.
    pub no_master_clock: bool,
    /// `MCKDIV`.
    pub master_divider_field: u8,
    /// `OSR`.
    pub oversampling: bool,
    /// `FRL`.
    pub frame_length_field: u8,
    /// `FSALL`.
    pub frame_active_field: u8,
    /// `FSDEF`.
    pub frame_marks_channel: bool,
    /// `FSPOL`.
    pub frame_active_high: bool,
    /// `FSOFF`.
    pub frame_leads_first_bit: bool,
    /// `FBOFF`.
    pub first_bit_offset_field: u8,
    /// `SLOTSZ`.
    pub slot_size_bits: u8,
    /// `NBSLOT`.
    pub slot_count_field: u8,
    /// `SLOTEN`.
    pub slot_enable_bits: u16,
    /// `DMAEN`.
    pub transfer_enabled: bool,
    /// `SAIEN`.
    pub enabled: bool,
    /// `FLVL`.
    pub fifo_level_bits: u8,
    /// `OVRUDR`.
    pub underrun: bool,
    /// `WCKCFG`.
    pub clock_configuration_rejected: bool,
    /// The seven bits of `SAI_xIM` together.
    pub any_interrupt_enabled: bool,
}

/// Every field of one transfer stream the register block drives off its reset
/// value, and the two addresses.
///
/// Closed against the same boundary as `BlockReadback`, and ranging over the
/// stream and its multiplexer channel alone. RM0433 section 15.5.5 resets
/// `DMA_SxCR` to `0x0000_0000` and section 15.5.10 resets `DMA_SxFCR` to
/// `0x0000_0021`, so `PFCTRL`, `MBURST`, `PBURST` and `DMDIS` come up carrying
/// what this transport wants and the bring-up writes none of them to anything
/// else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[expect
(
    clippy::struct_excessive_bools,
    reason = "each field is one register bit, and naming them apart is what \
              lets a fault say which bit"
)]
pub struct StreamReadback
{
    /// `DMAREQ_ID` of the multiplexer channel the stream is served by.
    pub request_bits: u8,
    /// `SE` of that channel.
    pub sync_enabled: bool,
    /// `DIR`.
    pub direction_bits: u8,
    /// `CIRC`.
    pub circular: bool,
    /// `MINC`.
    pub memory_increments: bool,
    /// `PINC`.
    pub peripheral_increments: bool,
    /// `MSIZE`.
    pub memory_width_bits: u8,
    /// `PSIZE`.
    pub peripheral_width_bits: u8,
    /// `DBM`.
    pub double_buffered: bool,
    /// `PL`.
    pub priority_bits: u8,
    /// `TCIE`, `HTIE`, `TEIE`, `DMEIE` and `FEIE` together.
    pub any_interrupt_enabled: bool,
    /// `PAR`.
    pub peripheral_address: u32,
    /// `M0AR`.
    pub memory_address: u32,
    /// `NDTR`.
    pub items: u32,
    /// `EN`.
    pub enabled: bool,
}

/// Everything a bring-up reads back off the interface and its two streams.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransportReadback
{
    /// The sub-block driving the clocks.
    pub master: BlockReadback,
    /// The sub-block synchronous with it.
    pub slave: BlockReadback,
    /// The stream feeding the master sub-block.
    pub master_stream: StreamReadback,
    /// The stream feeding the slave sub-block.
    pub slave_stream: StreamReadback,
}

/// The frame the converters take, and the buffers the streams replay.
///
/// The frame length and the master clock divider are taken off a clock plan
/// rather than named here, so the interface writes the chain the clock module
/// validated and the two cannot disagree about the sample rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransportPlan
{
    frame_bits: u16,
    master_divider: u8,
    master_buffer: u32,
    slave_buffer: u32,
    buffer_words: u32,
}

impl TransportPlan
{
    /// Builds the plan a clock plan and a pair of buffers make. `validate` is
    /// what accepts it.
    #[must_use]
    pub const fn for_clock
    (
        clock: &ClockPlan,
        master_buffer: u32,
        slave_buffer: u32,
        buffer_words: u32
    ) -> Self
    {
        Self
        {
            frame_bits: clock.frame_bits(),
            master_divider: clock.master_divider(),
            master_buffer,
            slave_buffer,
            buffer_words,
        }
    }

    /// Returns the `FRL` field value, one less than the frame length.
    ///
    /// RM0433 section 51.6.6: the frame carries `FRL + 1` bit clock periods.
    #[must_use]
    #[expect
    (
        clippy::cast_possible_truncation,
        reason = "validate refuses a frame that is not two 32 bit slots, so \
                  the value is 63"
    )]
    pub const fn frame_length_field(self) -> u8
    {
        self.frame_bits.saturating_sub(1) as u8
    }

    /// Returns the `FSALL` field value, one less than the active length.
    ///
    /// RM0433 section 51.6.6. The frame clock holds each level for half the
    /// frame, which is what marks the channel side.
    #[must_use]
    #[expect
    (
        clippy::cast_possible_truncation,
        reason = "validate refuses a frame that is not two 32 bit slots, so \
                  the value is 31"
    )]
    pub const fn frame_active_field(self) -> u8
    {
        (self.frame_bits / 2).saturating_sub(1) as u8
    }

    /// Returns the `NBSLOT` field value, one less than the slot count.
    ///
    /// RM0433 section 51.6.8.
    #[must_use]
    pub const fn slot_count_field(self) -> u8
    {
        SLOTS_PER_FRAME.saturating_sub(1)
    }

    /// Returns the `MCKDIV` field value, which is the divider itself.
    ///
    /// RM0433 section 51.6.2.
    #[must_use]
    pub const fn master_divider_field(self) -> u8
    {
        self.master_divider
    }

    /// Returns the `DS` field value.
    #[must_use]
    pub const fn data_size_field(self) -> u8
    {
        DATA_SIZE_32
    }

    /// Returns the `PRTCFG` field value.
    #[must_use]
    pub const fn protocol_field(self) -> u8
    {
        PROTOCOL_FREE
    }

    /// Returns the `CKSTR` bit value.
    #[must_use]
    pub const fn clock_strobing_field(self) -> bool
    {
        CHANGE_ON_FALLING_EDGE
    }

    /// Returns the `PL` field value.
    #[must_use]
    pub const fn priority_field(self) -> u8
    {
        STREAM_PRIORITY
    }

    /// Returns the `SLOTSZ` field value.
    #[must_use]
    pub const fn slot_size_field(self) -> u8
    {
        SLOT_SIZE_32
    }

    /// Returns the `SLOTEN` field value.
    #[must_use]
    pub const fn slot_enable_field(self) -> u16
    {
        SLOTS_ENABLED
    }

    /// Returns the `FTH` field value.
    #[must_use]
    pub const fn fifo_threshold_field(self) -> u8
    {
        FIFO_THRESHOLD_HALF
    }

    /// Returns the `MODE` field value the role takes.
    #[must_use]
    pub const fn mode_field(self, role: BlockRole) -> u8
    {
        role.mode_bits()
    }

    /// Returns the `SYNCEN` field value the role takes.
    #[must_use]
    pub const fn sync_field(self, role: BlockRole) -> u8
    {
        role.sync_bits()
    }

    /// Returns the `DMAREQ_ID` field value the role takes.
    #[must_use]
    pub const fn request_field(self, role: BlockRole) -> u8
    {
        match role
        {
            BlockRole::Master => MASTER_REQUEST,
            BlockRole::Slave => SLAVE_REQUEST,
        }
    }

    /// Returns the address the stream of `role` writes every sample to.
    #[must_use]
    pub const fn data_address(self, role: BlockRole) -> u32
    {
        match role
        {
            BlockRole::Master => MASTER_DATA_ADDRESS,
            BlockRole::Slave => SLAVE_DATA_ADDRESS,
        }
    }

    /// Returns the buffer address the stream of `role` replays.
    #[must_use]
    pub const fn buffer_address(self, role: BlockRole) -> u32
    {
        match role
        {
            BlockRole::Master => self.master_buffer,
            BlockRole::Slave => self.slave_buffer,
        }
    }

    /// Returns the `NDTR` value, which is the buffer length in words.
    ///
    /// `validate` bounds it at the width of the field, so a plan it accepted
    /// narrows into `NDTR` unchanged.
    #[must_use]
    pub const fn transfer_items(self) -> u32
    {
        self.buffer_words
    }

    /// Returns the `DIR` field value.
    #[must_use]
    pub const fn direction_field(self) -> u8
    {
        TRANSFER_MEMORY_TO_PERIPHERAL
    }

    /// Returns the `MSIZE` and `PSIZE` field value.
    #[must_use]
    pub const fn width_field(self) -> u8
    {
        TRANSFER_WORD
    }

    /// Returns nothing when the part accepts the plan.
    ///
    /// # Errors
    ///
    /// One variant of `TransportPlanError` per bound, so a refusal names the
    /// bound it broke rather than the fact that something broke.
    pub const fn validate(self) -> Result<(), TransportPlanError>
    {
        match self.validate_frame()
        {
            Err(error) => Err(error),
            Ok(()) => self.validate_buffers(),
        }
    }

    /// Checks the frame against the slots it has to hold.
    const fn validate_frame(self) -> Result<(), TransportPlanError>
    {
        if self.frame_bits != SLOTS_PER_FRAME as u16 * SLOT_BITS
        {
            return Err(TransportPlanError::FrameLengthWrong);
        }

        if self.master_divider < MASTER_DIVIDER_MIN || self.master_divider > MASTER_DIVIDER_MAX
        {
            return Err(TransportPlanError::MasterDividerOutOfRange);
        }

        Ok(())
    }

    /// Checks both buffers against the memory and the tone they have to hold.
    const fn validate_buffers(self) -> Result<(), TransportPlanError>
    {
        if self.buffer_words == 0
        {
            return Err(TransportPlanError::BufferEmpty);
        }

        if self.buffer_words > u16::MAX as u32
        {
            return Err(TransportPlanError::BufferTooLong);
        }

        let slots = SLOTS_PER_FRAME as u32;

        if !self.buffer_words.is_multiple_of(slots)
        {
            return Err(TransportPlanError::BufferNotWholeFrames);
        }

        // The divisor is the slot count, which is two, so the fallback stands
        // only to keep the expression total and it cannot be the answer.
        let frames = match self.buffer_words.checked_div(slots)
        {
            Some(count) => count,
            None => 0,
        };

        if !frames.is_multiple_of(TONE_SAMPLE_COUNT)
        {
            return Err(TransportPlanError::BufferNotWholePeriods);
        }

        let bytes = self.buffer_words.saturating_mul(WORD_BYTES);

        if !self.master_buffer.is_multiple_of(WORD_BYTES)
            || !self.slave_buffer.is_multiple_of(WORD_BYTES)
        {
            return Err(TransportPlanError::BufferUnaligned);
        }

        if !reaches_memory(self.master_buffer, bytes) || !reaches_memory(self.slave_buffer, bytes)
        {
            return Err(TransportPlanError::BufferUnreachable);
        }

        if self.master_buffer < self.slave_buffer.saturating_add(bytes)
            && self.slave_buffer < self.master_buffer.saturating_add(bytes)
        {
            return Err(TransportPlanError::BuffersOverlap);
        }

        Ok(())
    }

    /// Returns nothing when `seen` carries the plan.
    ///
    /// # Errors
    ///
    /// One variant of `TransportFault` per place, carrying the field of that
    /// place that disagreed. The master sub-block is checked first, because it
    /// is the one whose clocks the slave runs on.
    pub fn verify(self, seen: &TransportReadback) -> Result<(), TransportFault>
    {
        if let Err(fault) = self.verify_block(&seen.master, BlockRole::Master)
        {
            return Err(TransportFault::MasterBlock(fault));
        }

        if let Err(fault) = self.verify_block(&seen.slave, BlockRole::Slave)
        {
            return Err(TransportFault::SlaveBlock(fault));
        }

        if let Err(fault) = self.verify_stream(&seen.master_stream, BlockRole::Master)
        {
            return Err(TransportFault::MasterStream(fault));
        }

        if let Err(fault) = self.verify_stream(&seen.slave_stream, BlockRole::Slave)
        {
            return Err(TransportFault::SlaveStream(fault));
        }

        Ok(())
    }

    /// Checks one sub-block against the plan and against its own error flags.
    fn verify_block(self, seen: &BlockReadback, role: BlockRole) -> Result<(), BlockFault>
    {
        self.verify_block_shape(seen, role)?;
        self.verify_block_frame(seen)?;
        verify_block_state(seen)
    }

    /// Checks the direction, the protocol and the clock generator.
    fn verify_block_shape(self, seen: &BlockReadback, role: BlockRole) -> Result<(), BlockFault>
    {
        if seen.mode_bits != self.mode_field(role)
        {
            return Err(BlockFault::ModeWrong);
        }

        if seen.protocol_bits != self.protocol_field()
        {
            return Err(BlockFault::ProtocolWrong);
        }

        if seen.sync_bits != self.sync_field(role)
        {
            return Err(BlockFault::SynchronisationWrong);
        }

        if seen.data_size_bits != self.data_size_field()
        {
            return Err(BlockFault::DataSizeWrong);
        }

        if seen.lsb_first
        {
            return Err(BlockFault::BitOrderWrong);
        }

        if seen.changes_on_falling_edge != self.clock_strobing_field()
        {
            return Err(BlockFault::ClockStrobingWrong);
        }

        if !role.generates_clocks()
        {
            return Ok(());
        }

        if seen.no_master_clock
        {
            return Err(BlockFault::MasterClockDisabled);
        }

        if seen.master_divider_field != self.master_divider_field()
        {
            return Err(BlockFault::MasterDividerWrong);
        }

        if seen.oversampling
        {
            return Err(BlockFault::OversamplingWrong);
        }

        Ok(())
    }

    /// Checks the frame and the slots inside it.
    fn verify_block_frame(self, seen: &BlockReadback) -> Result<(), BlockFault>
    {
        if seen.frame_length_field != self.frame_length_field()
        {
            return Err(BlockFault::FrameLengthWrong);
        }

        if seen.frame_active_field != self.frame_active_field()
        {
            return Err(BlockFault::FrameActiveLengthWrong);
        }

        if !seen.frame_marks_channel
        {
            return Err(BlockFault::FrameDefinitionWrong);
        }

        if seen.frame_active_high
        {
            return Err(BlockFault::FramePolarityWrong);
        }

        if !seen.frame_leads_first_bit
        {
            return Err(BlockFault::FrameOffsetWrong);
        }

        if seen.first_bit_offset_field != 0
        {
            return Err(BlockFault::FirstBitOffsetWrong);
        }

        if seen.slot_size_bits != self.slot_size_field()
        {
            return Err(BlockFault::SlotSizeWrong);
        }

        if seen.slot_count_field != self.slot_count_field()
        {
            return Err(BlockFault::SlotCountWrong);
        }

        if seen.slot_enable_bits != self.slot_enable_field()
        {
            return Err(BlockFault::SlotsNotEnabled);
        }

        if seen.fifo_threshold_bits != self.fifo_threshold_field()
        {
            return Err(BlockFault::FifoThresholdWrong);
        }

        Ok(())
    }

    /// Checks one stream against the plan.
    fn verify_stream(self, seen: &StreamReadback, role: BlockRole) -> Result<(), StreamFault>
    {
        if seen.request_bits != self.request_field(role)
        {
            return Err(StreamFault::RequestWrong);
        }

        if seen.sync_enabled
        {
            return Err(StreamFault::SynchronisationEnabled);
        }

        if seen.direction_bits != self.direction_field()
        {
            return Err(StreamFault::DirectionWrong);
        }

        if !seen.circular
        {
            return Err(StreamFault::NotCircular);
        }

        if !seen.memory_increments
        {
            return Err(StreamFault::MemoryNotIncrementing);
        }

        if seen.peripheral_increments
        {
            return Err(StreamFault::PeripheralIncrementing);
        }

        if seen.memory_width_bits != self.width_field()
        {
            return Err(StreamFault::MemoryWidthWrong);
        }

        if seen.peripheral_width_bits != self.width_field()
        {
            return Err(StreamFault::PeripheralWidthWrong);
        }

        if seen.double_buffered
        {
            return Err(StreamFault::DoubleBuffered);
        }

        if seen.priority_bits != self.priority_field()
        {
            return Err(StreamFault::PriorityWrong);
        }

        if seen.any_interrupt_enabled
        {
            return Err(StreamFault::InterruptEnabled);
        }

        if seen.peripheral_address != self.data_address(role)
        {
            return Err(StreamFault::PeripheralAddressWrong);
        }

        if seen.memory_address != self.buffer_address(role)
        {
            return Err(StreamFault::MemoryAddressWrong);
        }

        // The counter is not compared for equality here. It counts down as the
        // stream runs and reloads at the end of each lap, so what a running
        // stream carries is a position inside the buffer. `verify_armed`
        // compares it against the plan while the stream is stopped, and the
        // advance check of `bring_up` is what proves it moves.
        if seen.items > self.transfer_items()
        {
            return Err(StreamFault::CounterOutOfRange);
        }

        if !seen.enabled
        {
            return Err(StreamFault::NotEnabled);
        }

        Ok(())
    }

    /// Returns nothing when both streams were armed with the plan.
    ///
    /// Called while the streams are stopped, where the transfer counter still
    /// reads the length it was loaded with. Once a stream runs, that register
    /// is a position rather than a length.
    ///
    /// # Errors
    ///
    /// `MasterStream` or `SlaveStream`, carrying the field that disagreed.
    pub fn verify_armed(self, seen: &TransportReadback) -> Result<(), TransportFault>
    {
        if let Err(fault) = self.verify_armed_stream(&seen.master_stream, BlockRole::Master)
        {
            return Err(TransportFault::MasterStream(fault));
        }

        if let Err(fault) = self.verify_armed_stream(&seen.slave_stream, BlockRole::Slave)
        {
            return Err(TransportFault::SlaveStream(fault));
        }

        Ok(())
    }

    /// Checks the three values that place one armed stream.
    fn verify_armed_stream(self, seen: &StreamReadback, role: BlockRole)
        -> Result<(), StreamFault>
    {
        if seen.peripheral_address != self.data_address(role)
        {
            return Err(StreamFault::PeripheralAddressWrong);
        }

        if seen.memory_address != self.buffer_address(role)
        {
            return Err(StreamFault::MemoryAddressWrong);
        }

        if seen.items != self.transfer_items()
        {
            return Err(StreamFault::ItemCountWrong);
        }

        Ok(())
    }
}

/// Checks the fields a sub-block carries whatever its role, and its two flags.
///
/// The underrun flag is read at one instant, the one the bring-up ends at. It
/// says the transmitter had data for every frame it has sent so far, and
/// nothing about the ones it has not. Freedom from underrun over a window is a
/// measurement, not a return value.
fn verify_block_state(seen: &BlockReadback) -> Result<(), BlockFault>
{
    if seen.mono
    {
        return Err(BlockFault::MonoWrong);
    }

    if seen.driven_before_start
    {
        return Err(BlockFault::OutputDriveWrong);
    }

    if seen.tristate
    {
        return Err(BlockFault::TristateWrong);
    }

    if seen.any_interrupt_enabled
    {
        return Err(BlockFault::InterruptEnabled);
    }

    if !seen.transfer_enabled
    {
        return Err(BlockFault::TransferDisabled);
    }

    if !seen.enabled
    {
        return Err(BlockFault::NotEnabled);
    }

    if seen.clock_configuration_rejected
    {
        return Err(BlockFault::ClockConfigurationRejected);
    }

    if seen.underrun
    {
        return Err(BlockFault::Underrun);
    }

    Ok(())
}

/// Returns whether `bytes` from `address` lie in the memory the buffers belong
/// in.
const fn reaches_memory(address: u32, bytes: u32) -> bool
{
    let Some(end) = address.checked_add(bytes)
    else
    {
        return false;
    };

    let Some(region_end) = BUFFER_REGION_BASE.checked_add(BUFFER_REGION_BYTES)
    else
    {
        return false;
    };

    address >= BUFFER_REGION_BASE && end <= region_end
}

/// The register writes and the one read a bring-up needs.
///
/// Each write covers one step of the order the part specifies, because that
/// order is what makes the writes take. `read` is the only way to observe the
/// interface, so a wait and a verification look at the same thing.
pub trait AudioInterface
{
    /// Clears `EN` on both streams.
    fn stop_streams(&mut self);

    /// Clears `SAIEN` on both sub-blocks, the master first.
    ///
    /// RM0433 section 51.4.15 requires the master of a synchronous pair to be
    /// disabled before the block that runs off its clocks.
    fn stop_blocks(&mut self);

    /// Puts the interface pins on their alternate function.
    ///
    /// This is the one step of the sequence `read` does not observe, so a
    /// register block that gets it wrong produces a verified transport that
    /// reaches no pad. What answers for it is the register block itself and the
    /// oscilloscope.
    fn open_pins(&mut self);

    /// Writes every configuration register of the master sub-block, `SAIEN`
    /// apart.
    fn write_master(&mut self, plan: &TransportPlan);

    /// Writes every configuration register of the slave sub-block, `SAIEN`
    /// apart.
    fn write_slave(&mut self, plan: &TransportPlan);

    /// Routes and writes the stream feeding the master sub-block, but not `EN`.
    fn write_master_stream(&mut self, plan: &TransportPlan);

    /// Routes and writes the stream feeding the slave sub-block, but not `EN`.
    fn write_slave_stream(&mut self, plan: &TransportPlan);

    /// Sets `EN` on both streams, which starts them filling the two FIFOs.
    fn start_streams(&mut self);

    /// Sets `SAIEN` on the slave sub-block.
    fn start_slave(&mut self);

    /// Sets `SAIEN` on the master sub-block, which starts the clocks.
    fn start_master(&mut self);

    /// Reads back the two sub-blocks and the two streams.
    ///
    /// The port is not among them. Nothing here observes it.
    fn read(&self) -> TransportReadback;
}

/// Polls each wait holds before it gives up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransportWaits
{
    /// Polls a stream gets to report stopped.
    pub stream_polls: u32,
    /// Polls a sub-block gets to report stopped.
    pub block_polls: u32,
    /// Polls the two FIFOs get to leave empty once the streams run.
    pub fifo_polls: u32,
    /// Polls the two transfer counters get to move once the frame runs.
    pub transfer_polls: u32,
}

impl TransportWaits
{
    /// Builds the waits a part running at `core_clock_hz` needs.
    ///
    /// Each count covers 1 ms at one core cycle per poll, and a poll is one
    /// `AudioInterface::read`, which is many cycles rather than one, so the
    /// figures are floors on the time a wait holds and not the time it takes.
    ///
    /// 1 ms is 44 frames at the rate this chain runs. RM0433 section 51.4.15
    /// completes a started frame before a sub-block stops, and section 15.5.5
    /// completes a started transfer before a stream stops, so both bound out at
    /// a small multiple of one frame. The FIFO wait covers the eight word fill
    /// that follows a stream starting, which the transfer controller does at
    /// bus speed. The transfer wait covers 44 frames, which draw 88 words out
    /// of each buffer, against the one word a counter has to move by.
    #[must_use]
    pub const fn for_core_clock(core_clock_hz: u32) -> Self
    {
        Self
        {
            stream_polls: wait_polls(core_clock_hz, 1_000),
            block_polls: wait_polls(core_clock_hz, 1_000),
            fifo_polls: wait_polls(core_clock_hz, 1_000),
            transfer_polls: wait_polls(core_clock_hz, 1_000),
        }
    }
}

/// Brings the output transport up on `interface` and proves it took the plan.
///
/// The order is what the part specifies, and every step of it is a requirement
/// rather than a habit.
///
/// The streams stop first. RM0433 section 51.4.16 requires the transfer channel
/// of a sub-block to be disabled before the sub-block is configured, and
/// section 15.5.5 makes every stream field writable only while `EN` reads 0.
///
/// The sub-blocks stop next, and the read after that proves both went down.
/// RM0433 section 51.6.2 drops a `SAIEN` write that finds the bit already set,
/// and section 51.4.15 completes the frame in flight before the bit falls, so a
/// warm restart can find one running.
///
/// The pins open before anything drives them, and the two sub-blocks are then
/// written whole while both are stopped, which is where section 51.6.2 requires
/// `MODE`, `SYNCEN`, `CKSTR`, `LSBFIRST` and `DS` to be written, and where
/// section 51.6.8 requires the slots.
///
/// The streams start before either sub-block, so the FIFOs fill first. Section
/// 51.4.3 makes that mandatory for a slave transmitter, whose first frame would
/// otherwise be an underrun, and the wait on the FIFO level is that step.
///
/// The slave starts before the master. Section 51.4.3 requires the slave armed
/// when the master sends its first clock edge, and that is what puts the two
/// data lines on the same bit of the same frame.
///
/// The streams are verified twice. While they are stopped their counter still
/// reads the length it was loaded with, so that is where it is compared against
/// the plan. Once the frame runs the same register is a position inside the
/// buffer, so what is required of it there is that it moves, which is the one
/// check that separates a configured transport from a running one.
///
/// The verification is the return value. Nothing the interface reports means
/// "configured as asked", so a bring-up that skipped a write ends here rather
/// than at a flag. What it compares is every field the register block drives
/// off its reset value in the two sub-blocks and the two streams, which
/// `BlockReadback` and `StreamReadback` set out. The port is outside it.
///
/// # Errors
///
/// `PlanRejected` before any register is touched, then one variant per step
/// that did not take. A refusal reached after `start_master` leaves the clocks
/// running on whatever the writes did land, which is what the caller wants: the
/// converter mute sequence needs its clocks, and putting the interface back
/// where a reset left it is not something a half-applied plan can do.
pub fn bring_up<T>
(
    interface: &mut T,
    plan: &TransportPlan,
    waits: &TransportWaits
) -> Result<(), TransportFault>
where
    T: AudioInterface + ?Sized,
{
    if let Err(error) = plan.validate()
    {
        return Err(TransportFault::PlanRejected(error));
    }

    interface.stop_streams();

    if !poll_until(interface, waits.stream_polls, streams_stopped)
    {
        return Err(TransportFault::StreamNeverStopped);
    }

    interface.stop_blocks();

    if !poll_until(interface, waits.block_polls, blocks_stopped)
    {
        return Err(TransportFault::BlockNeverStopped);
    }

    interface.open_pins();
    interface.write_master(plan);
    interface.write_slave(plan);
    interface.write_master_stream(plan);
    interface.write_slave_stream(plan);
    plan.verify_armed(&interface.read())?;
    interface.start_streams();

    if !poll_until(interface, waits.fifo_polls, fifos_filled)
    {
        return Err(TransportFault::FifoNeverFilled);
    }

    interface.start_slave();
    interface.start_master();

    let running = interface.read();

    plan.verify(&running)?;

    if !poll_until(interface, waits.transfer_polls, |seen| advanced(seen, &running))
    {
        return Err(TransportFault::TransferNeverAdvanced);
    }

    Ok(())
}

/// Returns whether both transfer counters have moved off `first`.
fn advanced(seen: &TransportReadback, first: &TransportReadback) -> bool
{
    seen.master_stream.items != first.master_stream.items
        && seen.slave_stream.items != first.slave_stream.items
}

/// Returns whether both streams report stopped.
fn streams_stopped(seen: &TransportReadback) -> bool
{
    !seen.master_stream.enabled && !seen.slave_stream.enabled
}

/// Returns whether both sub-blocks report stopped.
fn blocks_stopped(seen: &TransportReadback) -> bool
{
    !seen.master.enabled && !seen.slave.enabled
}

/// Returns whether both FIFOs hold at least one word.
fn fifos_filled(seen: &TransportReadback) -> bool
{
    seen.master.fifo_level_bits != FIFO_LEVEL_EMPTY
        && seen.slave.fifo_level_bits != FIFO_LEVEL_EMPTY
}

/// Polls `interface` until `ready` holds, or `polls` more reads have gone by.
///
/// The condition is read before the budget is spent, so a budget of zero still
/// buys one look. Each poll is a whole `AudioInterface::read`, whatever the
/// predicate goes on to look at, so the budget buys reads and not cycles.
fn poll_until<T, F>(interface: &T, polls: u32, ready: F) -> bool
where
    T: AudioInterface + ?Sized,
    F: Fn(&TransportReadback) -> bool,
{
    let mut remaining = polls;

    loop
    {
        if ready(&interface.read())
        {
            return true;
        }

        if remaining == 0
        {
            return false;
        }

        remaining = remaining.saturating_sub(1);
    }
}

const _: () = assert!
(
    TONE_HZ * TONE_SAMPLE_COUNT == SAMPLE_RATE_HZ * TONE_PERIODS,
    "the tone frequency divides the table into whole periods"
);

const _: () = assert!
(
    SLOTS_PER_FRAME as u16 * SLOT_BITS == 64,
    "two 32 bit slots make the 64 bit frame the clock plan is built on"
);

const _: () = assert!
(
    MASTER_DATA_ADDRESS < SLAVE_DATA_ADDRESS,
    "the two data registers are distinct, so one stream cannot feed both"
);

#[cfg(test)]
mod tests
{
    // The reference sine is computed in double precision from the table index
    // and the table length, both under 500, so every conversion between an
    // integer and a float below is exact.
    #![allow
    (
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_possible_wrap
    )]

    use super::*;
    use crate::clock::AUDIO_PLAN;
    use core::cell::Cell;

    /// Words in the buffer the tests plan against, one tone period per channel.
    const TEST_WORDS: u32 = TONE_SAMPLE_COUNT * 2;

    /// Address of the buffer the master stream replays in the tests.
    const TEST_MASTER_BUFFER: u32 = BUFFER_REGION_BASE;

    /// Address of the buffer the slave stream replays in the tests.
    const TEST_SLAVE_BUFFER: u32 = BUFFER_REGION_BASE + TEST_WORDS * WORD_BYTES;

    /// Largest error a table entry may carry against the reference sine.
    ///
    /// One sample step, which covers the rounding of the series and of the
    /// reference together.
    const TABLE_TOLERANCE: i64 = 1;

    /// One broken field of a read-back, and the fault it must produce.
    type Mutation = (fn(&mut TransportReadback), TransportFault);

    /// Returns the plan the tests are built on.
    fn plan() -> TransportPlan
    {
        TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            TEST_MASTER_BUFFER,
            TEST_SLAVE_BUFFER,
            TEST_WORDS
        )
    }

    /// The waits a run at the clock the part boots on gets.
    fn waits() -> TransportWaits
    {
        TransportWaits::for_core_clock(64_000_000)
    }

    /// An interface whose registers answer the way the part does.
    ///
    /// `image` holds the fields at their reset values, so a step the sequence
    /// skips shows up as a reset value rather than as the plan. The FIFO levels
    /// and the two enable bits move on the calls that move them on the part.
    #[expect
    (
        clippy::struct_excessive_bools,
        reason = "each flag is one way a register block misbehaves, and naming \
                  them apart is what lets a test pick one"
    )]
    struct MockInterface
    {
        image: TransportReadback,
        polls: Cell<u32>,
        streams_started: bool,
        fifo_filled_after: u32,
        stream_stays_enabled: bool,
        block_stays_enabled: bool,
        fifo_stays_empty: bool,
        refuse_slave_sync: bool,
        transfer_stalls: bool,
        short_master_count: bool,
        steps: u32,
        streams_at: u32,
        slave_at: u32,
        master_at: u32,
    }

    impl MockInterface
    {
        /// Builds an interface that takes every step.
        ///
        /// The image is the reset value of every register: RM0433 section
        /// 51.6.2 resets `SAI_xCR1` to `0x0000_0040`, which is `DS` at 8 bits
        /// and every other field clear, section 51.6.6 resets `SAI_xFRCR` to
        /// `0x0000_0007`, which is a frame of eight bit clock periods, and
        /// sections 51.6.4, 51.6.8 and 15.5.5 reset their registers to zero.
        fn healthy() -> Self
        {
            Self
            {
                image: TransportReadback
                {
                    master: reset_block(),
                    slave: reset_block(),
                    master_stream: reset_stream(),
                    slave_stream: reset_stream(),
                },
                polls: Cell::new(0),
                streams_started: false,
                fifo_filled_after: 2,
                stream_stays_enabled: false,
                block_stays_enabled: false,
                fifo_stays_empty: false,
                refuse_slave_sync: false,
                transfer_stalls: false,
                short_master_count: false,
                steps: 0,
                streams_at: 0,
                slave_at: 0,
                master_at: 0,
            }
        }

        /// Returns the number of the step about to run.
        fn step(&mut self) -> u32
        {
            self.steps = self.steps.saturating_add(1);
            self.steps
        }
    }

    /// Returns one sub-block at the reset value of its registers.
    fn reset_block() -> BlockReadback
    {
        BlockReadback
        {
            mode_bits: 0,
            protocol_bits: 0,
            data_size_bits: 0b010,
            lsb_first: false,
            changes_on_falling_edge: false,
            sync_bits: 0,
            mono: false,
            driven_before_start: false,
            tristate: false,
            fifo_threshold_bits: 0,
            no_master_clock: false,
            master_divider_field: 0,
            oversampling: false,
            frame_length_field: 7,
            frame_active_field: 0,
            frame_marks_channel: false,
            frame_active_high: false,
            frame_leads_first_bit: false,
            first_bit_offset_field: 0,
            slot_size_bits: 0,
            slot_count_field: 0,
            slot_enable_bits: 0,
            transfer_enabled: false,
            enabled: false,
            fifo_level_bits: FIFO_LEVEL_EMPTY,
            underrun: false,
            clock_configuration_rejected: false,
            any_interrupt_enabled: false,
        }
    }

    /// Returns one stream at the reset value of its registers.
    fn reset_stream() -> StreamReadback
    {
        StreamReadback
        {
            request_bits: 0,
            sync_enabled: false,
            direction_bits: 0,
            circular: false,
            memory_increments: false,
            peripheral_increments: false,
            memory_width_bits: 0,
            peripheral_width_bits: 0,
            double_buffered: false,
            priority_bits: 0,
            any_interrupt_enabled: false,
            peripheral_address: 0,
            memory_address: 0,
            items: 0,
            enabled: false,
        }
    }

    /// Writes one sub-block the way the register block does.
    fn apply_block(block: &mut BlockReadback, plan: &TransportPlan, role: BlockRole)
    {
        block.mode_bits = plan.mode_field(role);
        block.protocol_bits = PROTOCOL_FREE;
        block.data_size_bits = plan.data_size_field();
        block.changes_on_falling_edge = true;
        block.sync_bits = plan.sync_field(role);
        block.fifo_threshold_bits = plan.fifo_threshold_field();
        block.frame_length_field = plan.frame_length_field();
        block.frame_active_field = plan.frame_active_field();
        block.frame_marks_channel = true;
        block.frame_leads_first_bit = true;
        block.slot_size_bits = plan.slot_size_field();
        block.slot_count_field = plan.slot_count_field();
        block.slot_enable_bits = plan.slot_enable_field();
        block.transfer_enabled = true;

        if role == BlockRole::Master
        {
            block.master_divider_field = plan.master_divider_field();
        }
    }

    /// Writes one stream the way the register block does.
    fn apply_stream(stream: &mut StreamReadback, plan: &TransportPlan, role: BlockRole)
    {
        stream.request_bits = plan.request_field(role);
        stream.direction_bits = plan.direction_field();
        stream.circular = true;
        stream.memory_increments = true;
        stream.memory_width_bits = plan.width_field();
        stream.peripheral_width_bits = plan.width_field();
        stream.priority_bits = plan.priority_field();
        stream.peripheral_address = plan.data_address(role);
        stream.memory_address = plan.buffer_address(role);
        stream.items = plan.transfer_items();
    }

    impl AudioInterface for MockInterface
    {
        fn stop_streams(&mut self)
        {
            if self.stream_stays_enabled
            {
                return;
            }

            self.image.master_stream.enabled = false;
            self.image.slave_stream.enabled = false;
        }

        fn stop_blocks(&mut self)
        {
            if self.block_stays_enabled
            {
                return;
            }

            self.image.master.enabled = false;
            self.image.slave.enabled = false;
        }

        fn open_pins(&mut self)
        {
        }

        fn write_master(&mut self, plan: &TransportPlan)
        {
            apply_block(&mut self.image.master, plan, BlockRole::Master);
        }

        fn write_slave(&mut self, plan: &TransportPlan)
        {
            apply_block(&mut self.image.slave, plan, BlockRole::Slave);

            if self.refuse_slave_sync
            {
                self.image.slave.sync_bits = SYNC_ASYNCHRONOUS;
            }
        }

        fn write_master_stream(&mut self, plan: &TransportPlan)
        {
            apply_stream(&mut self.image.master_stream, plan, BlockRole::Master);

            if self.short_master_count
            {
                self.image.master_stream.items = self.image.master_stream.items
                    .saturating_sub(2);
            }
        }

        fn write_slave_stream(&mut self, plan: &TransportPlan)
        {
            apply_stream(&mut self.image.slave_stream, plan, BlockRole::Slave);
        }

        fn start_streams(&mut self)
        {
            self.streams_at = self.step();
            self.image.master_stream.enabled = true;
            self.image.slave_stream.enabled = true;
            self.streams_started = true;
            self.polls.set(0);
        }

        fn start_slave(&mut self)
        {
            self.slave_at = self.step();
            self.image.slave.enabled = true;
        }

        fn start_master(&mut self)
        {
            self.master_at = self.step();
            self.image.master.enabled = true;
        }

        fn read(&self) -> TransportReadback
        {
            let polls = self.polls.get().saturating_add(1);
            self.polls.set(polls);

            let mut seen = self.image;
            let filled = self.streams_started
                && !self.fifo_stays_empty
                && polls >= self.fifo_filled_after;

            if filled
            {
                seen.master.fifo_level_bits = 0b101;
                seen.slave.fifo_level_bits = 0b101;
            }

            // The counter walks down one word a read once the master clocks the
            // frame out, and it is the register the plan reads as a length only
            // while the stream is stopped.
            if self.image.master.enabled && !self.transfer_stalls
            {
                let drawn = polls.checked_rem(seen.master_stream.items.max(1)).unwrap_or(0);
                seen.master_stream.items = seen.master_stream.items.saturating_sub(drawn);
                seen.slave_stream.items = seen.slave_stream.items.saturating_sub(drawn);
            }

            seen
        }
    }

    /// Breaks one field of a verified read-back per case and checks the fault.
    fn run_mutations(mutations: &[Mutation])
    {
        for (break_it, expected) in mutations
        {
            let mut seen = verified_readback();
            break_it(&mut seen);

            assert_eq!(plan().verify(&seen), Err(*expected));
        }
    }

    /// The read-back a bring-up on a healthy interface leaves behind.
    fn verified_readback() -> TransportReadback
    {
        let mut interface = MockInterface::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(()));
        interface.read()
    }

    #[test]
    fn the_tone_is_one_kilohertz()
    {
        assert_eq!(TONE_HZ, 1_000);
        assert_eq!(TONE_SAMPLES, 441);
        assert_eq!(SAMPLE_RATE_HZ * TONE_PERIODS, TONE_HZ * TONE_SAMPLES as u32);
    }

    #[test]
    fn the_tone_table_matches_the_reference_sine()
    {
        for (index, sample) in TONE_TABLE.iter().enumerate()
        {
            let phase = 2.0 * PI * f64::from(TONE_PERIODS) * index as f64 / TONE_SAMPLES as f64;
            let reference = (TONE_AMPLITUDE * libm::sin(phase)).round() as i64;

            assert!
            (
                (i64::from(*sample) - reference).abs() <= TABLE_TOLERANCE,
                "sample {index} is {sample}, the reference is {reference}"
            );
        }
    }

    #[test]
    fn the_tone_table_peaks_at_an_eighth_of_scale()
    {
        let peak = TONE_TABLE.iter().map(|sample| sample.unsigned_abs()).max();

        // An eighth of i32::MAX is 268435455.875. The table samples the crest
        // rather than landing on it, and sample 11 is the closest at 0.9999937
        // of it, so the largest entry sits just under an eighth of scale.
        assert_eq!(peak, Some(268_433_753));
        assert!(peak <= Some(i32::MAX.unsigned_abs() / 8));
    }

    #[test]
    fn the_tone_table_closes_on_itself()
    {
        // The step across the wrap is the step between the last entry and the
        // first, and it must be no larger than the largest step inside the
        // table, or a circular replay would carry a discontinuity once a lap.
        let mut widest = 0_i64;
        let mut previous = i64::from(TONE_TABLE[0]);

        for sample in TONE_TABLE.iter().skip(1)
        {
            widest = widest.max((i64::from(*sample) - previous).abs());
            previous = i64::from(*sample);
        }

        let wrap = (i64::from(TONE_TABLE[0]) - previous).abs();

        assert!(wrap <= widest, "the wrap steps {wrap}, the table steps {widest}");
    }

    #[test]
    fn the_tone_table_carries_no_offset()
    {
        // Whole periods of a sine sum to zero, so what is left is rounding.
        let sum: i64 = TONE_TABLE.iter().map(|sample| i64::from(*sample)).sum();

        assert!(sum.abs() <= i64::from(TONE_SAMPLE_COUNT), "the table sums to {sum}");
    }

    #[test]
    fn the_series_reduces_over_the_whole_turn()
    {
        // The fold runs on the integers, so every quadrant has to come back
        // right, including the two the reflection and the sign change build.
        for numerator in 0..2_000_u64
        {
            let ours = sine_of_turn(numerator, 441);
            let reference = libm::sin(2.0 * PI * numerator as f64 / 441.0);

            assert!
            (
                (ours - reference).abs() < 1e-12,
                "turn {numerator}/441 gives {ours}, the reference is {reference}"
            );
        }
    }

    #[test]
    fn a_denominator_of_zero_gives_no_angle()
    {
        assert_eq!(sine_of_turn(1, 0).to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn the_board_plan_is_accepted()
    {
        assert_eq!(plan().validate(), Ok(()));
    }

    #[test]
    fn the_plan_encodes_the_fields_the_manual_gives()
    {
        let plan = plan();

        assert_eq!(plan.frame_length_field(), 63);
        assert_eq!(plan.frame_active_field(), 31);
        assert_eq!(plan.slot_count_field(), 1);
        assert_eq!(plan.master_divider_field(), 2);
        assert_eq!(plan.data_size_field(), 0b111);
        assert_eq!(plan.slot_size_field(), 0b10);
        assert_eq!(plan.slot_enable_field(), 0b11);
        assert_eq!(plan.fifo_threshold_field(), 0b010);
        assert_eq!(plan.direction_field(), 0b01);
        assert_eq!(plan.width_field(), 0b10);
        assert_eq!(plan.mode_field(BlockRole::Master), 0b00);
        assert_eq!(plan.mode_field(BlockRole::Slave), 0b10);
        assert_eq!(plan.sync_field(BlockRole::Master), 0b00);
        assert_eq!(plan.sync_field(BlockRole::Slave), 0b01);
        assert_eq!(plan.request_field(BlockRole::Master), 87);
        assert_eq!(plan.request_field(BlockRole::Slave), 88);
        assert_eq!(plan.data_address(BlockRole::Master), 0x4001_5820);
        assert_eq!(plan.data_address(BlockRole::Slave), 0x4001_5840);
        assert_eq!(plan.protocol_field(), 0b00);
        assert!(plan.clock_strobing_field());
        assert_eq!(plan.priority_field(), 0b10);
        assert_eq!(plan.transfer_items(), TEST_WORDS);
    }

    #[test]
    fn a_frame_that_does_not_hold_the_slots_is_refused()
    {
        let clock = crate::clock::ClockPlan::new
        (
            AUDIO_PLAN.source_hz(),
            AUDIO_PLAN.pll(),
            AUDIO_PLAN.master_divider(),
            32
        );
        let plan = TransportPlan::for_clock
        (
            &clock,
            TEST_MASTER_BUFFER,
            TEST_SLAVE_BUFFER,
            TEST_WORDS
        );

        assert_eq!(plan.validate(), Err(TransportPlanError::FrameLengthWrong));
    }

    #[test]
    fn a_master_divider_outside_its_width_is_refused()
    {
        for divider in [0_u8, 64]
        {
            let clock = crate::clock::ClockPlan::new
            (
                AUDIO_PLAN.source_hz(),
                AUDIO_PLAN.pll(),
                divider,
                AUDIO_PLAN.frame_bits()
            );
            let plan = TransportPlan::for_clock
            (
                &clock,
                TEST_MASTER_BUFFER,
                TEST_SLAVE_BUFFER,
                TEST_WORDS
            );

            assert_eq!(plan.validate(), Err(TransportPlanError::MasterDividerOutOfRange));
        }
    }

    #[test]
    fn a_buffer_that_is_not_whole_periods_is_refused()
    {
        let cases =
        [
            (0_u32, TransportPlanError::BufferEmpty),
            (0x1_0000, TransportPlanError::BufferTooLong),
            (TEST_WORDS - 1, TransportPlanError::BufferNotWholeFrames),
            (TEST_WORDS - 2, TransportPlanError::BufferNotWholePeriods),
        ];

        for (words, expected) in cases
        {
            let plan = TransportPlan::for_clock
            (
                &AUDIO_PLAN,
                TEST_MASTER_BUFFER,
                TEST_SLAVE_BUFFER,
                words
            );

            assert_eq!(plan.validate(), Err(expected));
        }
    }

    #[test]
    fn a_buffer_off_a_word_boundary_is_refused()
    {
        let plan = TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            TEST_MASTER_BUFFER + 2,
            TEST_SLAVE_BUFFER,
            TEST_WORDS
        );

        assert_eq!(plan.validate(), Err(TransportPlanError::BufferUnaligned));
    }

    #[test]
    fn a_buffer_the_transfer_controllers_cannot_reach_is_refused()
    {
        // The tightly coupled memory, which only the master transfer controller
        // reaches, and which a buffer placed by habit rather than by section
        // lands in.
        let cases =
        [
            0x2000_0000_u32,
            0x0000_0000,
            BUFFER_REGION_BASE - 4,
            BUFFER_REGION_BASE + BUFFER_REGION_BYTES,
        ];

        for address in cases
        {
            let plan = TransportPlan::for_clock
            (
                &AUDIO_PLAN,
                address,
                TEST_SLAVE_BUFFER,
                TEST_WORDS
            );

            assert_eq!(plan.validate(), Err(TransportPlanError::BufferUnreachable));
        }
    }

    #[test]
    fn a_buffer_running_off_the_end_of_the_memory_is_refused()
    {
        let last = BUFFER_REGION_BASE + BUFFER_REGION_BYTES - TEST_WORDS * WORD_BYTES;
        let fits = TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            TEST_MASTER_BUFFER,
            last,
            TEST_WORDS
        );
        let over = TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            TEST_MASTER_BUFFER,
            last + 4,
            TEST_WORDS
        );

        assert_eq!(fits.validate(), Ok(()));
        assert_eq!(over.validate(), Err(TransportPlanError::BufferUnreachable));
    }

    #[test]
    fn two_buffers_that_share_a_word_are_refused()
    {
        let bytes = TEST_WORDS * WORD_BYTES;
        let master = TEST_MASTER_BUFFER + bytes;
        let cases = [master, master + bytes - 4, master + 4 - bytes];

        for slave in cases
        {
            let plan = TransportPlan::for_clock(&AUDIO_PLAN, master, slave, TEST_WORDS);

            assert_eq!(plan.validate(), Err(TransportPlanError::BuffersOverlap));
        }
    }

    #[test]
    fn two_buffers_that_touch_are_accepted()
    {
        let bytes = TEST_WORDS * WORD_BYTES;
        let plan = TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            TEST_MASTER_BUFFER,
            TEST_MASTER_BUFFER + bytes,
            TEST_WORDS
        );

        assert_eq!(plan.validate(), Ok(()));
    }

    #[test]
    fn a_bring_up_leaves_the_interface_carrying_the_plan()
    {
        assert_eq!(plan().verify(&verified_readback()), Ok(()));
    }

    #[test]
    fn a_bring_up_starts_the_streams_then_the_slave_then_the_master()
    {
        // RM0433 section 51.4.3 wants the slave armed when the master sends its
        // first clock edge, and both FIFOs holding data before either block
        // runs. A one sample offset between two ways is 24 degrees of phase at
        // the low to mid crossover, so this order is the whole alignment.
        let mut interface = MockInterface::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(()));
        assert!(interface.streams_at < interface.slave_at);
        assert!(interface.slave_at < interface.master_at);

        let seen = interface.read();

        assert!(seen.master.enabled);
        assert!(seen.slave.enabled);
        assert!(seen.master_stream.enabled);
        assert!(seen.slave_stream.enabled);
    }

    #[test]
    fn a_refused_plan_touches_no_register()
    {
        let mut interface = MockInterface::healthy();
        let plan = TransportPlan::for_clock(&AUDIO_PLAN, TEST_MASTER_BUFFER, 0, TEST_WORDS);

        assert_eq!
        (
            bring_up(&mut interface, &plan, &waits()),
            Err(TransportFault::PlanRejected(TransportPlanError::BufferUnreachable))
        );
        assert_eq!(interface.image, MockInterface::healthy().image);
    }

    #[test]
    fn a_stream_that_will_not_stop_refuses_the_bring_up()
    {
        let mut interface = MockInterface::healthy();
        interface.image.master_stream.enabled = true;
        interface.stream_stays_enabled = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(TransportFault::StreamNeverStopped)
        );
    }

    #[test]
    fn a_block_that_will_not_stop_refuses_the_bring_up()
    {
        let mut interface = MockInterface::healthy();
        interface.image.slave.enabled = true;
        interface.block_stays_enabled = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(TransportFault::BlockNeverStopped)
        );
    }

    #[test]
    fn a_fifo_that_stays_empty_refuses_the_bring_up()
    {
        let mut interface = MockInterface::healthy();
        interface.fifo_stays_empty = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(TransportFault::FifoNeverFilled)
        );
    }

    #[test]
    fn a_counter_that_never_moves_refuses_the_bring_up()
    {
        let mut interface = MockInterface::healthy();
        interface.transfer_stalls = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(TransportFault::TransferNeverAdvanced)
        );
    }

    #[test]
    fn a_stream_armed_with_the_wrong_length_refuses_the_bring_up()
    {
        // The counter is compared against the plan while the stream is
        // stopped, because once it runs the same register is a position.
        let mut interface = MockInterface::healthy();
        interface.short_master_count = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(TransportFault::MasterStream(StreamFault::ItemCountWrong))
        );
    }

    #[test]
    fn a_counter_past_the_buffer_is_refused_while_running()
    {
        let mut seen = verified_readback();
        seen.master_stream.items = TEST_WORDS + 1;

        assert_eq!
        (
            plan().verify(&seen),
            Err(TransportFault::MasterStream(StreamFault::CounterOutOfRange))
        );
    }

    #[test]
    fn a_counter_inside_the_buffer_is_accepted_while_running()
    {
        // A running stream reads a position, so every value the buffer holds
        // has to pass, or the bring-up would refuse itself once it worked.
        for items in [0, 1, TEST_WORDS / 2, TEST_WORDS]
        {
            let mut seen = verified_readback();
            seen.master_stream.items = items;
            seen.slave_stream.items = items;

            assert_eq!(plan().verify(&seen), Ok(()));
        }
    }

    #[test]
    fn a_slave_left_asynchronous_refuses_the_bring_up()
    {
        let mut interface = MockInterface::healthy();
        interface.refuse_slave_sync = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(TransportFault::SlaveBlock(BlockFault::SynchronisationWrong))
        );
    }

    #[test]
    fn every_frame_field_of_a_sub_block_is_compared()
    {
        let mutations: [Mutation; 19] =
        [
            (
                |seen| seen.master.mode_bits = MODE_SLAVE_TRANSMITTER,
                TransportFault::MasterBlock(BlockFault::ModeWrong),
            ),
            (
                |seen| seen.master.protocol_bits = 0b10,
                TransportFault::MasterBlock(BlockFault::ProtocolWrong),
            ),
            (
                |seen| seen.master.sync_bits = SYNC_INTERNAL,
                TransportFault::MasterBlock(BlockFault::SynchronisationWrong),
            ),
            (
                |seen| seen.master.data_size_bits = 0b110,
                TransportFault::MasterBlock(BlockFault::DataSizeWrong),
            ),
            (
                |seen| seen.master.lsb_first = true,
                TransportFault::MasterBlock(BlockFault::BitOrderWrong),
            ),
            (
                |seen| seen.master.changes_on_falling_edge = false,
                TransportFault::MasterBlock(BlockFault::ClockStrobingWrong),
            ),
            (
                |seen| seen.master.no_master_clock = true,
                TransportFault::MasterBlock(BlockFault::MasterClockDisabled),
            ),
            (
                |seen| seen.master.master_divider_field = 4,
                TransportFault::MasterBlock(BlockFault::MasterDividerWrong),
            ),
            (
                |seen| seen.master.oversampling = true,
                TransportFault::MasterBlock(BlockFault::OversamplingWrong),
            ),
            (
                |seen| seen.master.frame_length_field = 31,
                TransportFault::MasterBlock(BlockFault::FrameLengthWrong),
            ),
            (
                |seen| seen.master.frame_active_field = 15,
                TransportFault::MasterBlock(BlockFault::FrameActiveLengthWrong),
            ),
            (
                |seen| seen.master.frame_marks_channel = false,
                TransportFault::MasterBlock(BlockFault::FrameDefinitionWrong),
            ),
            (
                |seen| seen.master.frame_active_high = true,
                TransportFault::MasterBlock(BlockFault::FramePolarityWrong),
            ),
            (
                |seen| seen.master.frame_leads_first_bit = false,
                TransportFault::MasterBlock(BlockFault::FrameOffsetWrong),
            ),
            (
                |seen| seen.master.first_bit_offset_field = 8,
                TransportFault::MasterBlock(BlockFault::FirstBitOffsetWrong),
            ),
            (
                |seen| seen.master.slot_size_bits = 0b01,
                TransportFault::MasterBlock(BlockFault::SlotSizeWrong),
            ),
            (
                |seen| seen.master.slot_count_field = 3,
                TransportFault::MasterBlock(BlockFault::SlotCountWrong),
            ),
            (
                |seen| seen.master.slot_enable_bits = 0b01,
                TransportFault::MasterBlock(BlockFault::SlotsNotEnabled),
            ),
            (
                |seen| seen.master.fifo_threshold_bits = 0b000,
                TransportFault::MasterBlock(BlockFault::FifoThresholdWrong),
            ),
        ];

        run_mutations(&mutations);
    }

    #[test]
    fn every_state_field_of_a_sub_block_is_compared()
    {
        let mutations: [Mutation; 8] =
        [
            (
                |seen| seen.master.any_interrupt_enabled = true,
                TransportFault::MasterBlock(BlockFault::InterruptEnabled),
            ),
            (
                |seen| seen.master.mono = true,
                TransportFault::MasterBlock(BlockFault::MonoWrong),
            ),
            (
                |seen| seen.master.driven_before_start = true,
                TransportFault::MasterBlock(BlockFault::OutputDriveWrong),
            ),
            (
                |seen| seen.master.tristate = true,
                TransportFault::MasterBlock(BlockFault::TristateWrong),
            ),
            (
                |seen| seen.master.transfer_enabled = false,
                TransportFault::MasterBlock(BlockFault::TransferDisabled),
            ),
            (
                |seen| seen.master.enabled = false,
                TransportFault::MasterBlock(BlockFault::NotEnabled),
            ),
            (
                |seen| seen.master.clock_configuration_rejected = true,
                TransportFault::MasterBlock(BlockFault::ClockConfigurationRejected),
            ),
            (
                |seen| seen.master.underrun = true,
                TransportFault::MasterBlock(BlockFault::Underrun),
            ),
        ];

        run_mutations(&mutations);
    }

    #[test]
    fn every_field_of_a_stream_is_compared()
    {
        let mutations: [Mutation; 15] =
        [
            (
                |seen| seen.master_stream.request_bits = SLAVE_REQUEST,
                TransportFault::MasterStream(StreamFault::RequestWrong),
            ),
            (
                |seen| seen.master_stream.sync_enabled = true,
                TransportFault::MasterStream(StreamFault::SynchronisationEnabled),
            ),
            (
                |seen| seen.master_stream.direction_bits = 0b00,
                TransportFault::MasterStream(StreamFault::DirectionWrong),
            ),
            (
                |seen| seen.master_stream.circular = false,
                TransportFault::MasterStream(StreamFault::NotCircular),
            ),
            (
                |seen| seen.master_stream.memory_increments = false,
                TransportFault::MasterStream(StreamFault::MemoryNotIncrementing),
            ),
            (
                |seen| seen.master_stream.peripheral_increments = true,
                TransportFault::MasterStream(StreamFault::PeripheralIncrementing),
            ),
            (
                |seen| seen.master_stream.memory_width_bits = 0b01,
                TransportFault::MasterStream(StreamFault::MemoryWidthWrong),
            ),
            (
                |seen| seen.master_stream.peripheral_width_bits = 0b01,
                TransportFault::MasterStream(StreamFault::PeripheralWidthWrong),
            ),
            (
                |seen| seen.master_stream.double_buffered = true,
                TransportFault::MasterStream(StreamFault::DoubleBuffered),
            ),
            (
                |seen| seen.master_stream.priority_bits = 0,
                TransportFault::MasterStream(StreamFault::PriorityWrong),
            ),
            (
                |seen| seen.master_stream.any_interrupt_enabled = true,
                TransportFault::MasterStream(StreamFault::InterruptEnabled),
            ),
            (
                |seen| seen.master_stream.peripheral_address = SLAVE_DATA_ADDRESS,
                TransportFault::MasterStream(StreamFault::PeripheralAddressWrong),
            ),
            (
                |seen| seen.master_stream.memory_address = TEST_SLAVE_BUFFER,
                TransportFault::MasterStream(StreamFault::MemoryAddressWrong),
            ),
            (
                |seen| seen.master_stream.items = TEST_WORDS + 2,
                TransportFault::MasterStream(StreamFault::CounterOutOfRange),
            ),
            (
                |seen| seen.master_stream.enabled = false,
                TransportFault::MasterStream(StreamFault::NotEnabled),
            ),
        ];

        run_mutations(&mutations);
    }

    #[test]
    fn the_two_streams_are_told_apart()
    {
        // Feeding the slave sub-block from the buffer and the request of the
        // master is a frame that looks right and carries the wrong channel, so
        // the check has to name the place as well as the field.
        let mut seen = verified_readback();
        seen.slave_stream.peripheral_address = MASTER_DATA_ADDRESS;

        assert_eq!
        (
            plan().verify(&seen),
            Err(TransportFault::SlaveStream(StreamFault::PeripheralAddressWrong))
        );

        let mut seen = verified_readback();
        seen.slave_stream.request_bits = MASTER_REQUEST;

        assert_eq!
        (
            plan().verify(&seen),
            Err(TransportFault::SlaveStream(StreamFault::RequestWrong))
        );
    }

    #[test]
    fn the_clock_generator_of_the_slave_is_not_compared()
    {
        // RM0433 section 51.4.8 turns the generator off on a slave and ignores
        // these three fields there, so a reset value in them is not a fault.
        let mut seen = verified_readback();
        seen.slave.no_master_clock = true;
        seen.slave.master_divider_field = 0;
        seen.slave.oversampling = true;

        assert_eq!(plan().verify(&seen), Ok(()));
    }

    #[test]
    fn a_wait_of_zero_polls_still_looks_once()
    {
        let mut interface = MockInterface::healthy();
        interface.fifo_filled_after = 1;
        let waits = TransportWaits
        {
            stream_polls: 0,
            block_polls: 0,
            fifo_polls: 0,
            transfer_polls: 0,
        };

        assert_eq!(bring_up(&mut interface, &plan(), &waits), Ok(()));
    }

    #[test]
    fn the_waits_rise_with_the_clock()
    {
        let slow = TransportWaits::for_core_clock(64_000_000);
        let fast = TransportWaits::for_core_clock(480_000_000);

        assert!(slow.stream_polls < fast.stream_polls);
        assert!(slow.block_polls < fast.block_polls);
        assert!(slow.fifo_polls < fast.fifo_polls);
        assert!(slow.transfer_polls < fast.transfer_polls);
        assert_eq!(slow.stream_polls, 64_000);
    }
}
