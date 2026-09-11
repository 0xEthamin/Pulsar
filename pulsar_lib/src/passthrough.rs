//! Input path of the processing board, and the block structure it feeds.
//!
//! One converter channel arrives on sub-block A of the second audio interface,
//! run as a slave receiver synchronised on the first interface, and one
//! circular transfer stream drops it into a buffer the length of the two output
//! buffers. What runs on top of that is a block structure: each time the
//! receiving stream finishes a half of its buffer, one half of samples is
//! carried to the two output buffers, at the positions those samples were sent
//! from.
//!
//! Nothing here touches a peripheral. `InputInterface` is the interface a
//! register block implements, `bring_up` is the sequence run over it, `arm` is
//! what puts the block structure into service, `next_block` is the decision one
//! transfer event produces, and `carry_block` is the loop that walks the
//! samples of one block through the two accessors of that interface.
//!
//! The loop lives here rather than behind the interface so that a host test
//! walks it, which buffers each word reaches included. A register block owns
//! the two accessors and nothing else of the carry, which leaves the part of
//! the audio path that touches samples on the side of the build `cargo test`
//! reaches. A filter stage is inserted into that loop.
//!
//! # The synchronisation
//!
//! RM0433 section 51.4.4 gives the form: the source interface names which of
//! its sub-blocks drives `FS` and `SCK` through `SYNCOUT`, the receiving
//! interface names which interface it listens to through `SYNCIN`, and each
//! sub-block of the receiving interface declares itself synchronous through
//! `SYNCEN`. Table 420 gives `SYNCIN` 0 for a second interface taking its
//! synchronisation from the first. Section 51.6.1 requires both fields written
//! while the audio blocks are disabled, which is why `SYNCOUT` is written
//! inside the output bring-up rather than here, and `SYNCIN` before this
//! sequence writes anything else.
//!
//! A sub-block that follows another interface takes its bit clock and its frame
//! clock from outside, so section 51.4.8 turns its own clock generator off and
//! ignores `NOMCK`, `MCKDIV` and `OSR`. Those three are written to the value a
//! reset gives them and compared against nothing, because comparing them would
//! be comparing a value the part does not read.
//!
//! The receiver takes the same `CKSTR` as the transmitter. RM0433 section
//! 51.6.2 gives that bit two halves in one sentence: at 1 the signals the
//! interface generates change on the falling edge, while the signals it
//! receives are sampled on the rising edge. The transmitter uses the first
//! half and the receiver the second, and both are the same bit.
//!
//! # The offset between the two sides, and where it puts a word
//!
//! The receiving stream and the two transmitting streams run off one frame
//! clock, so each of them advances exactly one word per slot and the distance
//! between their positions never changes. What that distance IS gets fixed when
//! the receiver starts, and no register reports it and no constant predicts it.
//!
//! It decides where a word goes. The two buffers share no origin: while the
//! receiving stream writes index `i`, the transmitting streams read index
//! `i + offset`. The word that lands at `i` left the output buffers before that
//! read, by the pipeline the section below names, so it belongs at
//! `i + offset` less that pipeline, and `shift` is what this module calls that
//! position. A carry that wrote index for index would put every word `shift`
//! short of the position it belongs to, and in a loop, where the receiving
//! buffer holds what the output buffers sent, the content of the output buffers
//! then advances by `shift` words every lap for as long as the board runs. On
//! this board that is the 188 words the shift stands at and not the 196 the two
//! counters read. That is a pitch shift and a step of up to full scale at each
//! block boundary, not a delay.
//!
//! So the destination is derived from the distance rather than assumed equal to
//! the source: `CarryShift` is that distance measured once, less the pipeline
//! the section below names, and the carry adds it to every index. The block it
//! writes is then the block the read pointer has just left, which is also the
//! placement that leaves the whole of one block of margin ahead of the carry.
//!
//! The distance itself is not left to chance either. `bring_up` holds the
//! receiver stopped until the transmitting side has walked its read pointer to
//! a chosen position, starts the receiver there, and then reads both counters
//! and refuses unless the shift they leave stands in the band it aimed at.
//! Aiming at half of one half of the buffer leaves the same margin on both
//! sides, and `InputPlan::offset_low` and `InputPlan::offset_high` are the band
//! that shift may land in.
//! `next_block` measures the read pointer again on every event and refuses an
//! entry whose read pointer stands where the carry would write.
//!
//! # What a slow carry does, and why nothing here catches it
//!
//! `next_block` reads both counters at the entry and answers for that instant.
//! It cannot refuse a carry that crosses the read pointer WHILE it runs, and
//! what keeps the carry off the pointer once it has started is one property no
//! check here holds: the carry moves a word faster than the streams read one.
//!
//! The streams take 11.3 microseconds a word at this frame rate, so a block of
//! 441 words is 5.000 milliseconds and a new event lands every 5.000
//! milliseconds. The carry spends 31 instructions and 11 memory accesses on a
//! word, eight of those accesses the history the filter reads back and writes
//! forward, which comes to 1.8 to 2.0 microseconds a word and 806 to 896
//! microseconds a block on the clock the part boots on. That is 16 to 18 per
//! cent of the period, and it leaves the carry about six times faster than the
//! streams it has to stay ahead of.
//!
//! Those figures are a count of instructions and of memory accesses on the
//! linked image, not a reading taken on the part. What carries their
//! uncertainty is the cost of one access to the memory the buffers and the
//! filter history sit in, taken at nine core cycles, which is what the
//! allowance this firmware already runs on works out to. A bench closes it.
//!
//! The two accessors fold into the loop, and that is what keeps the count at 31
//! instructions. Left out of line the same word costs 88 instructions and three
//! calls for the same 11 accesses, counted the same way, so the fold buys about
//! a third of the budget.
//!
//! A carry starts writing where the read pointer has just left, so it has a
//! whole block of clearance ahead of it, 441 words or 5.000 milliseconds, at an
//! entry taken on time. One that fits inside a block therefore never crosses
//! the pointer.
//!
//! One that does not fit never catches up either: `travelled` grows by the
//! overrun at every entry, the clearance shrinks by as much, and the streams
//! overtake the carry inside the block. Every word behind the crossing goes out
//! holding the lap before, a step of up to full scale on a way with no analog
//! filter in front of it. `next_block` refuses once `travelled` reaches one
//! block less the margin, which bounds how long that lasts but does not come
//! first: the damage leaves the machine before the silence does.
//!
//! That is what makes the cost of this loop a real-time budget rather than a
//! preference, and it is why the two accessors under it are asked to fold and
//! why the gate of this firmware holds a ceiling on the code the served handler
//! runs.
//!
//! # What the passthrough costs in delay
//!
//! A word received at input position `p` is written into output position
//! `p + shift`, the position it was sent from, and the read pointer stood
//! `PIPELINE_WORDS` past that position when the word was counted in. The
//! transmitting streams therefore reach it one lap less that pipeline later,
//! 874 words. Crossing the FIFOs and the shift registers costs those eight
//! words back, before the count at the input and after the read at the output,
//! so pin to pin the delay is one whole lap of the buffer, 882 words or 10.0
//! milliseconds. It is the same for every word rather than an average, because
//! neither the distance nor the pipeline changes.
//!
//! Half a lap of it is the block structure itself, since no word can leave
//! before the half holding it has been received. The other half is the margin
//! that keeps the carry off the read pointer.
//!
//! # What this cannot test
//!
//! Drift. Both ends of the link run off one frame clock, so the distance
//! between them is constant by construction and a corrector would have nothing
//! to correct. Two independent oscillators are what makes drift measurable, and
//! neither this module nor anything it depends on models one.
//!
//! # What no counter reports, and what it costs in a loop
//!
//! The distance between the two transfer counters is not the distance the data
//! travels. A counter counts words the controller has moved into or out of a
//! FIFO rather than words on the wire, and between the two stand the
//! transmitting FIFO, the shift registers and the receiving FIFO, which hold a
//! few words each in steady state that no register reports to the word. So the
//! word the receiving buffer takes at index `i` left the output buffers at
//! `i + offset` less that pipeline.
//!
//! `PIPELINE_WORDS` is how many words that is, and `CarryShift::measure` takes
//! it off the counter distance, so the carry writes each word back at the
//! position it was sent from. A shift built from the counters alone lands every
//! word that many positions past it. With a source of its own on the data line
//! that difference is part of the delay and nothing else. In a loop, where the
//! data line carries what this board sent, it is a rotation of the content at
//! every lap, and both the pitch and the block boundaries show it: the eight
//! words this board crosses take the tone 0.9 per cent low and leave a step at
//! each boundary of 33 degrees of it. That rotation is what makes the size
//! readable on a bench, since a pipeline of `n` words moves the tone of this
//! firmware 1.13 `n` hertz down.

use crate::clock::wait_polls;
use crate::constants::{MICROSECONDS_PER_SECOND, SAMPLE_RATE_HZ};
use crate::filter::{Biquad, BiquadState};
use crate::release::{Laps, TonePermit};
use crate::transport::
{
    BlockRole,
    FIFO_THRESHOLD_HALF,
    SLOTS_PER_FRAME,
    SYNC_OUT_NONE,
    TONE_SAMPLE_COUNT,
    TransportPlan,
    reaches_memory,
};

/// `MODE` selecting a receiver whose bit clock comes from outside.
///
/// RM0433 section 51.6.2.
pub const MODE_SLAVE_RECEIVER: u8 = 0b11;

/// `SYNCEN` selecting synchronism with the other audio interface.
///
/// RM0433 section 51.6.2. The value beside it, 0b01, is synchronism with the
/// other sub-block of the SAME interface, which is what the output transport
/// runs on and is not this.
pub const SYNC_EXTERNAL: u8 = 0b10;

/// `SYNCIN` naming the first audio interface as the synchronisation source.
///
/// RM0433 section 51.4.4, table 420: for a sub-block of the second interface,
/// 0 selects the first.
pub const SYNC_IN_FIRST_INTERFACE: u8 = 0b00;

/// `DMAREQ_ID` routing sub-block A of the second interface.
///
/// RM0433 section 17.3.2 table.
pub const INPUT_REQUEST: u8 = 89;

/// `DIR` selecting a peripheral to memory transfer. RM0433 section 15.5.5.
pub const TRANSFER_PERIPHERAL_TO_MEMORY: u8 = 0b00;

/// Address one sample of the receiving sub-block is read from.
///
/// The memory map puts the second audio interface at `0x4001_5C00` and RM0433
/// section 51.6.16 puts the data register of its first sub-block at offset
/// `0x020`.
const INPUT_DATA_ADDRESS: u32 = 0x4001_5C20;

/// Bytes in one buffer entry.
const WORD_BYTES: u32 = 4;

/// Halves of the buffer the block structure walks.
const HALVES: u32 = 2;

/// Fraction of one half the carry must still have ahead of the read pointer.
///
/// A carry writes a whole half of the output buffers as fast as the core and
/// the memory allow, while the transmitting streams keep reading at one word
/// per slot. What this fixes is the shortest distance an entry may find between
/// the read pointer and the end of the block it is about to carry.
///
/// An eighth of a half is 55 words at the length these buffers hold, so an
/// entry this admits leaves the read pointer at least 55 words, 624
/// microseconds, short of the first word the carry writes. That is the FLOOR
/// over the entries the check accepts, not the distance an armed path leaves:
/// the carry writes the block the read pointer has just finished, so an entry
/// taken on time starts a whole block, 441 words and 5.000 milliseconds, ahead
/// of it, and the floor is what a handler entered late enough runs into.
///
/// A carry of 441 words takes 806 to 896 microseconds on the clock the part
/// boots on, which is LONGER than the 624 microseconds an entry at the floor
/// leaves. What answers that is the rate rather than the total: the carry
/// spends 1.8 to 2.0 microseconds on a word against the 11.3 the streams spend
/// reading one, so from the tightest entry this admits it starts 55 words ahead
/// of the read pointer and gains on it at every word. The total would only
/// matter to a carry that ran no faster than the streams, and the module
/// documentation carries what such a carry does.
const GUARD_DIVISOR: u32 = 8;

/// Lowest shift, as a fraction of one half, a start-up may leave.
///
/// A quarter of a half. Below it the carry runs close behind a read pointer
/// that has just left the block, and a distance that reads as nearly zero is
/// one wrap away from reading as nearly a whole lap, which is the case
/// `next_block` refuses.
const OFFSET_LOW_DIVISOR: u32 = 4;

/// Highest shift, as a fraction of one half, a start-up may leave.
///
/// Three quarters of a half, which stands clear of the bound `next_block`
/// enforces at one half less the margin above.
const OFFSET_HIGH_NUMERATOR: u32 = 3;

/// Denominator of the bound above.
const OFFSET_HIGH_DIVISOR: u32 = 4;

/// Half the width of the window the receiver is started in, as a fraction of
/// one half of the buffer.
///
/// The aim is the middle of the band the two bounds above leave, taken on the
/// counters, so the shift a start-up on the aim leaves stands `PIPELINE_WORDS`
/// under that middle. This is how wide a target the polling loop shoots at. A
/// sixteenth of a half is 27 words after the division truncates, which is 306
/// microseconds, and every poll of the counter is orders below that, so the
/// loop lands inside on the first lap it sees.
const ARMING_WINDOW_DIVISOR: u32 = 16;

/// Laps of the buffer the seed wait covers.
///
/// The tone is written into the output buffers while the transmitting streams
/// already replay them, so it takes up to one lap for every word going out to
/// carry it, and one more for the receiving buffer to hold nothing else. Three
/// is those two and the margin that keeps a healthy board off the timeout.
const SEED_LAPS: u32 = 3;

/// Words standing between what the two transfer counters describe and what the
/// data travels.
///
/// A counter counts words a controller has moved into or out of a FIFO, never
/// words on the wire, and the transmitting FIFO, the two shift registers and
/// the receiving FIFO sit between those two points. So the word the receiving
/// buffer takes at index `i` left the output buffers at `i + offset` less this,
/// and `CarryShift::measure` takes it off the counter distance.
///
/// MEASURED on the board over a closed link. A shift published at 196 words
/// brings a 1000 Hz tone back at 990.9 Hz on a frequency counter, one published
/// at 188 brings it back at 999.98 to 1000.02 Hz with a curve that stands still
/// on an oscilloscope. The counter therefore reads the two publications 9.08 to
/// 9.12 Hz apart, and eight words of rotation come to 1000 x 8 / 882, or 9.07
/// Hz, which is a figure computed from the eight and not one the counter shows.
/// At 188 two dumps of the buffers fifty one seconds apart, some five thousand
/// laps, hold 882 words of 882 unchanged.
///
/// Nothing here splits those eight words between the four stages, and the split
/// is not what the value rests on. `FLVL` of `SAI_xSR` reports a FIFO in bands
/// rather than in words, RM0433 section 51.6.12, and it reports nothing at all
/// of the two shift registers. The bench read it at 010 on the receiving
/// sub-block, which is two or three words of the eight that sub-block holds,
/// and at 011 on the transmitting one, which is five or six. Those two bands
/// plus one word in each shift register come to nine through eleven, while the
/// tone and the dumps say eight. That gap is open.
const PIPELINE_WORDS: u32 = 8;

/// `PIPELINE_WORDS` is a bench number taken under one transfer request
/// threshold, and that threshold is what sets how full the transmitting FIFO
/// stands. A different `FTH` moves the occupancy, the pipeline stops being
/// eight words, and nothing at run time would say so: the carry would place
/// every word off by the difference and the loop would come back at another
/// pitch. So the build stops here instead, and the number is measured again.
const _: () = assert!
(
    FIFO_THRESHOLD_HALF == 0b010,
    "FTH moved, so PIPELINE_WORDS has to be measured again on the board"
);

/// A shift moves a whole number of frames, so a word received in slot 0 leaves
/// in slot 0 and comes back on the converter channel it belongs to. `measure`
/// takes the counter distance down to a frame boundary and then takes this off
/// it, so a pipeline standing a slot inside a frame would move every word onto
/// the other channel, and no shift the band accepts puts it back. The frame the
/// transport writes is `SLOTS_PER_FRAME` slots wide.
///
/// A plan carrying a wider frame than that is caught at run time instead:
/// `measure` leaves a shift that `CarryShift::from_words` refuses, and the
/// bring-up answers that refusal with silence.
const _: () = assert!
(
    PIPELINE_WORDS.is_multiple_of(SLOTS_PER_FRAME as u32),
    "PIPELINE_WORDS is no whole number of frames, so a carry built on it would \
     put every word on the other converter channel"
);

/// Words in the shortest block the transport accepts.
///
/// `TransportPlan::validate` refuses a buffer that is not a whole number of
/// tone periods, so the shortest buffer it accepts holds `TONE_SAMPLE_COUNT`
/// frames of `SLOTS_PER_FRAME` slots, and a block is half of that.
///
/// The band, the margin and the arming window below are fractions of a block,
/// while `PIPELINE_WORDS` is a count of words. This is the block length that
/// makes the two comparable where no plan exists, which is where the assertions
/// at the foot of this file stand.
const SHORTEST_BLOCK_WORDS: u32 =
    TONE_SAMPLE_COUNT * SLOTS_PER_FRAME as u32 / HALVES;

/// Reason the part would not accept an input plan.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it, which is the number a person with a probe looks up. Two variants that
/// carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InputPlanError
{
    /// The buffer is empty.
    BufferEmpty = 0x01,
    /// The buffer holds more words than `NDTR` counts.
    BufferTooLong = 0x02,
    /// The buffer holds no whole number of frames.
    BufferNotWholeFrames = 0x03,
    /// The buffer splits into no two halves of equal length, so the transfer
    /// event that marks the middle of it marks no block boundary.
    BufferHalfNotWhole = 0x04,
    /// The buffer address is not on a word boundary, which a word wide
    /// transfer requires.
    BufferUnaligned = 0x05,
    /// The buffer falls outside the memory the transfer controllers reach.
    BufferUnreachable = 0x06,
    /// The buffer overlaps an output buffer, so the carry would read what it
    /// writes.
    BufferOverlapsOutput = 0x07,
    /// The buffer is shorter than the margin the carry needs, so no position
    /// of the read pointer is far enough from a block for one to be carried.
    BufferTooShortForGuard = 0x08,
    /// The buffer is not the length of the output buffers, so an index of one
    /// names another sample in the other.
    BufferLengthDiffers = 0x09,
}

/// Reason the receiving sub-block is not running to plan.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it. Two variants that carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InputBlockFault
{
    /// `MODE` does not carry a receiver taking its bit clock from outside.
    ModeWrong = 0x01,
    /// `PRTCFG` does not select the free protocol.
    ProtocolWrong = 0x02,
    /// `DS` does not carry the data size of the frame.
    DataSizeWrong = 0x03,
    /// `LSBFIRST` is set, so the frame is read least significant bit first.
    BitOrderWrong = 0x04,
    /// `CKSTR` does not sample the data line on the edge the transmitter keeps
    /// it still through.
    ClockStrobingWrong = 0x05,
    /// `SYNCEN` does not follow the other audio interface.
    SynchronisationWrong = 0x06,
    /// `SYNCIN` does not name the first audio interface.
    SynchronisationSourceWrong = 0x07,
    /// `MONO` is set, so one slot is taken for both.
    MonoWrong = 0x08,
    /// `TRIS` is set.
    TristateWrong = 0x09,
    /// `FTH` does not carry the transfer request threshold of the plan.
    FifoThresholdWrong = 0x0A,
    /// `FRL` does not carry the frame length the transmitter emits.
    FrameLengthWrong = 0x0B,
    /// `FSALL` does not hold the frame clock active for half the frame.
    FrameActiveLengthWrong = 0x0C,
    /// `FSDEF` is clear, so the frame clock marks no channel side.
    FrameDefinitionWrong = 0x0D,
    /// `FSPOL` is set, so the frame is read as starting on the rising edge.
    FramePolarityWrong = 0x0E,
    /// `FSOFF` is clear, so the frame clock is read as moving on the first bit
    /// rather than one bit ahead of it.
    FrameOffsetWrong = 0x0F,
    /// `FBOFF` is not zero, so the word is not taken from the top of its slot.
    FirstBitOffsetWrong = 0x10,
    /// `SLOTSZ` does not carry the slot size of the frame.
    SlotSizeWrong = 0x11,
    /// `NBSLOT` does not carry the slot count of the frame.
    SlotCountWrong = 0x12,
    /// `SLOTEN` leaves a slot of the frame inactive.
    SlotsNotEnabled = 0x13,
    /// `DMAEN` is clear, so nothing drains the FIFO.
    TransferDisabled = 0x14,
    /// `SAIEN` is clear.
    NotEnabled = 0x15,
    /// A sub-block interrupt is enabled, and the vector the receiving interface
    /// raises is not one this firmware serves, so it would reach the fault path
    /// and silence the machine.
    InterruptEnabled = 0x16,
    /// `OVRUDR` is set, so a received word was dropped before a transfer took
    /// it.
    Overrun = 0x17,
    /// `AFSDET` or `LFSDET` is set, so the frame arriving is not the frame this
    /// sub-block declares.
    FrameMismatch = 0x18,
    /// `CNRDY` is set, so the line carried no valid frame when the sub-block
    /// started.
    CodecNotReady = 0x19,
    /// `FLVL` reports a FIFO that never took a word, so nothing arrives on the
    /// data line.
    FifoNeverFilled = 0x1A,
    /// `SYNCOUT` puts the clocks of this interface out, so it is a second
    /// source beside the one it follows.
    ///
    /// The number sits at the end rather than beside the `SYNCIN` one, because
    /// these are the cause bytes a person reads off a parked board and moving
    /// an existing one would change what every earlier number means.
    SynchronisationOutputWrong = 0x1B,
}

/// Reason the receiving transfer stream is not running to plan.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it. Two variants that carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InputStreamFault
{
    /// `DMAREQ_ID` routes another peripheral to the stream.
    RequestWrong = 0x01,
    /// `SE` is set, so a synchronisation input gates the requests.
    SynchronisationEnabled = 0x02,
    /// `DIR` does not carry a peripheral to memory transfer.
    DirectionWrong = 0x03,
    /// `CIRC` is clear, so the stream stops at the end of the buffer.
    NotCircular = 0x04,
    /// `MINC` is clear, so every transfer writes the same word.
    MemoryNotIncrementing = 0x05,
    /// `PINC` is set, so the stream walks off the data register.
    PeripheralIncrementing = 0x06,
    /// `MSIZE` does not carry a word.
    MemoryWidthWrong = 0x07,
    /// `PSIZE` does not carry a word.
    PeripheralWidthWrong = 0x08,
    /// `DBM` is set, so the stream expects a second buffer address.
    DoubleBuffered = 0x09,
    /// `PL` does not carry the priority of the plan.
    PriorityWrong = 0x0A,
    /// `TEIE`, `DMEIE` or `FEIE` is set. The three share the vector the two
    /// transfer events raise, so one of them would enter the handler carrying
    /// no transfer event and be refused there rather than carried, which
    /// silences the machine on an entry the arming never asked for.
    ErrorInterruptEnabled = 0x0B,
    /// A transfer event interrupt is enabled before the gate has permitted the
    /// block structure to run.
    EventInterruptEnabled = 0x0C,
    /// `PAR` does not address the data register of the receiving sub-block.
    PeripheralAddressWrong = 0x0D,
    /// `M0AR` does not carry the buffer address of the plan.
    MemoryAddressWrong = 0x0E,
    /// `NDTR` did not carry the buffer length of the plan when the stream was
    /// armed.
    ItemCountWrong = 0x0F,
    /// `NDTR` reads past the buffer length while the stream runs, so the
    /// counter belongs to no lap of it.
    CounterOutOfRange = 0x10,
    /// `EN` is clear.
    NotEnabled = 0x11,
    /// `TEIF` is set, so a transfer took a bus error.
    TransferError = 0x12,
    /// `DMEIF` is set.
    ///
    /// RM0433 section 15.3.20 confines the flag to a peripheral to memory
    /// stream in direct mode with `MINC` clear, and this stream carries `MINC`
    /// set, so the part raises it on this one for no reason the manual gives.
    DirectModeError = 0x13,
    /// `FEIF` is set, so the memory bus was not granted before a request.
    FifoError = 0x14,
}

/// Reason the input bring-up gave up on a step of its own sequence.
///
/// These are the steps the sequence takes rather than a field it reads back.
/// The discriminant of a variant is the cause byte a fault record carries for
/// it. Two variants that carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InputSequenceFault
{
    /// The stream still reported enabled after it was told to stop, so its
    /// configuration fields would not have taken.
    StreamNeverStopped = 0x01,
    /// The sub-block still reported enabled after it was told to stop, so its
    /// configuration fields would not have taken.
    BlockNeverStopped = 0x02,
    /// The transmitting read pointer never reached the position the receiver
    /// is started at, so the distance between the two sides was never chosen.
    OutputPhaseNeverReached = 0x03,
    /// The transfer counter of the receiver never moved, so nothing arrives on
    /// the data line or nothing drains the FIFO.
    ReceiverNeverAdvanced = 0x04,
    /// The shift the two counters leave stands outside the band, so a block
    /// carried on it would fall on the read pointer sooner or later.
    OffsetOutOfBand = 0x05,
    /// The transfer counter of the receiver did not reload twice inside the
    /// seed budget, so the tone has not been round the link and the buffer
    /// still holds what was in it before.
    SeedNeverLapped = 0x06,
    /// The coefficient build refused the corner the carry filters on, so the
    /// stage carries no section and the machine stays silent rather than
    /// carrying a way with no filter on it.
    FilterRefused = 0x07,
}

/// Reason one transfer event is not one the block structure serves.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it. Two variants that carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EventFault
{
    /// Neither transfer event flag is set, so nothing this handler serves
    /// raised it.
    Spurious = 0x01,
    /// Both transfer event flags are set, so a block went by unserved and one
    /// of the two halves holds samples nothing carried.
    BothHalvesPending = 0x02,
    /// `OVRUDR` of the receiving sub-block is set, so a received word was
    /// dropped before a transfer took it.
    Overrun = 0x03,
    /// `AFSDET` or `LFSDET` is set, so the frame arriving stopped being the
    /// frame the sub-block declares.
    FrameMismatch = 0x04,
    /// `CNRDY` is set.
    CodecNotReady = 0x05,
    /// `SAIEN` is clear, so the receiver has stopped since it was armed.
    BlockStopped = 0x06,
    /// `TEIF` is set, so a transfer took a bus error.
    TransferError = 0x07,
    /// `DMEIF` is set.
    DirectModeError = 0x08,
    /// `FEIF` is set, so the memory bus was not granted before a request.
    FifoError = 0x09,
    /// `EN` is clear, so the receiving stream has stopped since it was armed.
    StreamStopped = 0x0A,
    /// The transfer counter of the receiver reads past its buffer.
    CounterOutOfRange = 0x0B,
    /// The transfer counter of the transmitter reads past its buffer, so the
    /// read pointer the carry has to stay clear of cannot be placed.
    OutputCounterOutOfRange = 0x0C,
    /// The transmitting read pointer is inside the block, or close enough
    /// behind it that the carry would not finish first.
    ReadPointerInBlock = 0x0D,
    /// The shift the bring-up measured did not reach the handler, so where a
    /// word belongs in the output buffers is not known and no position is a
    /// safe guess.
    CarryShiftUnpublished = 0x0E,
}

/// Where the input path refused.
///
/// The discriminant is the place byte of a fault code. Two places that carried
/// one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum InputPlace
{
    /// The bring-up sequence itself, which names no register.
    Sequence = 0x00,
    /// The plan, refused before any register moved.
    Plan = 0x01,
    /// The receiving sub-block.
    Block = 0x02,
    /// The transfer stream draining it.
    Stream = 0x03,
    /// One transfer event, seen by the handler that serves them.
    Event = 0x04,
}

/// Reason the input path is not running to plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PassthroughFault
{
    /// The bring-up sequence gave up on a step.
    Sequence(InputSequenceFault),
    /// The plan itself is not one the part accepts.
    PlanRejected(InputPlanError),
    /// The receiving sub-block does not carry the plan.
    Block(InputBlockFault),
    /// The transfer stream does not carry the plan.
    Stream(InputStreamFault),
    /// One transfer event is not one the block structure serves.
    Event(EventFault),
}

/// Widest word the input encoding can carry.
///
/// A code is a place byte over a cause byte, so the two types bound it. Nothing
/// here numbers a place or a cause past its byte, so no variant added to any of
/// the five enums can pass this, and a fault record therefore keeps the code
/// out of the domain field it sits beside. The bound is not a value the
/// encoding reaches.
pub(crate) const PASSTHROUGH_CODE_CEILING: u32 = u16::MAX as u32;

impl PassthroughFault
{
    /// Returns the code a fault record carries for this fault.
    ///
    /// A probe reads the word and looks the value up here. The code is a place
    /// byte over a cause byte: the place says WHERE the input path refused, the
    /// cause says what that place refused with.
    #[must_use]
    pub const fn code(self) -> u32
    {
        u16::from_be_bytes([self.place() as u8, self.cause()]) as u32
    }

    /// Returns the place byte of the code.
    const fn place(self) -> InputPlace
    {
        match self
        {
            Self::Sequence(_) => InputPlace::Sequence,
            Self::PlanRejected(_) => InputPlace::Plan,
            Self::Block(_) => InputPlace::Block,
            Self::Stream(_) => InputPlace::Stream,
            Self::Event(_) => InputPlace::Event,
        }
    }

    /// Returns the cause byte of the code, which is the discriminant of the
    /// fault the place carries.
    const fn cause(self) -> u8
    {
        match self
        {
            Self::Sequence(fault) => fault as u8,
            Self::PlanRejected(error) => error as u8,
            Self::Block(fault) => fault as u8,
            Self::Stream(fault) => fault as u8,
            Self::Event(fault) => fault as u8,
        }
    }
}

/// Which half of the buffers one block covers.
///
/// # Where a block starts inside a frame
///
/// A block is half a buffer, and half a buffer is a whole number of frames only
/// when the buffer holds an even number of them. This one holds 441, so the
/// second block starts on the second slot of a frame rather than on the first.
///
/// A carry does not care, since it moves a word to the same index and the slot
/// a word belongs to is the parity of that index either way. A stage that reads
/// per channel does: the parity of `block_start` is the slot its first word
/// belongs to, and it is 0 for the first block and 1 for the second on a buffer
/// of this length. It follows from the length rather than being a property of
/// the halves, so it is read off `block_start` and not written down as a fact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Half
{
    /// The half a transfer stream fills first, from index zero.
    First,
    /// The half it fills second, up to the end of the buffer.
    Second,
}

/// Every field of the receiving sub-block the register block drives off its
/// reset value, and the flags it raises.
///
/// The set is closed against that boundary rather than against the register
/// map. Every sub-block register the bring-up writes has at least one field
/// here, and the writes it makes land whole or not at all, so a field left out
/// is a field whose reset value is the wanted one and whose register a listed
/// neighbour already witnesses.
///
/// Three fields the bring-up writes are deliberately absent. `NOMCK`, `MCKDIV`
/// and `OSR` drive the clock generator, which RM0433 section 51.4.8 turns off
/// on a sub-block taking its clocks from outside, so what they hold is a value
/// the part does not read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[expect
(
    clippy::struct_excessive_bools,
    reason = "each field is one register bit, and naming them apart is what \
              lets a fault say which bit"
)]
pub struct InputBlockReadback
{
    /// `MODE`.
    pub mode_bits: u8,
    /// `PRTCFG`.
    pub protocol_bits: u8,
    /// `DS`.
    pub data_size_bits: u8,
    /// `LSBFIRST`.
    pub lsb_first: bool,
    /// `CKSTR`, read as the edge the interface changes its outputs on, which
    /// is the edge opposite the one it samples an input on.
    pub changes_on_falling_edge: bool,
    /// `SYNCEN`.
    pub sync_bits: u8,
    /// `SYNCIN` of `SAI_GCR`, which belongs to the interface rather than to
    /// this sub-block and is the field that names which interface it follows.
    pub sync_in_bits: u8,
    /// `SYNCOUT` of `SAI_GCR`, which belongs to the interface rather than to
    /// this sub-block and is the field that says whether it puts its own clocks
    /// out for another interface to follow.
    ///
    /// A receiver that also puts clocks out is a second source beside the one
    /// it follows, which a warm restart can leave behind. It reads the same at
    /// reset as it does written, so this comparison separates that case from a
    /// healthy one and separates neither from a write that never ran.
    pub sync_out_bits: u8,
    /// `MONO`.
    pub mono: bool,
    /// `TRIS`.
    pub tristate: bool,
    /// `FTH`.
    pub fifo_threshold_bits: u8,
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
    /// The seven bits of `SAI_xIM` together.
    pub any_interrupt_enabled: bool,
    /// `FLVL`.
    pub fifo_level_bits: u8,
    /// `OVRUDR`.
    pub overrun: bool,
    /// `AFSDET` and `LFSDET` together, which report a frame that is not the one
    /// this sub-block declares.
    pub frame_mismatch: bool,
    /// `CNRDY`.
    pub codec_not_ready: bool,
}

/// Every field of the receiving transfer stream the register block drives off
/// its reset value, the two addresses, and the three error flags.
///
/// Closed against the same boundary as `InputBlockReadback`. RM0433 section
/// 15.5.5 resets `DMA_SxCR` to `0x0000_0000` and section 15.5.10 resets
/// `DMA_SxFCR` to `0x0000_0021`, so `PFCTRL`, `MBURST`, `PBURST` and `DMDIS`
/// come up carrying what this path wants and the bring-up writes none of them
/// to anything else.
///
/// The two transfer event enables are read apart from the three error ones,
/// because they mean different things and change at different moments. All
/// five read clear at the bring-up. `arm` raises the two event ones and none of
/// the three others, which is what puts a served interrupt on that vector and
/// leaves every unserved one where it was.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[expect
(
    clippy::struct_excessive_bools,
    reason = "each field is one register bit, and naming them apart is what \
              lets a fault say which bit"
)]
pub struct InputStreamReadback
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
    /// `HTIE` and `TCIE` together.
    pub event_interrupt_enabled: bool,
    /// `TEIE`, `DMEIE` and `FEIE` together.
    pub error_interrupt_enabled: bool,
    /// `PAR`.
    pub peripheral_address: u32,
    /// `M0AR`.
    pub memory_address: u32,
    /// `NDTR`.
    pub items: u32,
    /// `EN`.
    pub enabled: bool,
    /// `TEIF` of this stream.
    pub transfer_error: bool,
    /// `DMEIF` of this stream.
    pub direct_mode_error: bool,
    /// `FEIF` of this stream.
    pub fifo_error: bool,
}

/// Everything a bring-up reads back off the input path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputReadback
{
    /// The receiving sub-block.
    pub block: InputBlockReadback,
    /// The stream draining it.
    pub stream: InputStreamReadback,
    /// `NDTR` of the stream feeding the master output sub-block.
    ///
    /// It is the read pointer a carry has to stay clear of. The stream feeding
    /// the other sub-block runs off the same frame clock and was started in the
    /// same call, so it trails this one by at most the word or two between the
    /// two enables, which is inside the margin the carry keeps.
    pub output_items: u32,
}

/// What one transfer event of the receiving stream carries.
///
/// It is the read the handler takes on entry, and it is separate from
/// `InputReadback` because a handler reads what tells it whether to carry a
/// block, and nothing else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[expect
(
    clippy::struct_excessive_bools,
    reason = "each field is one register bit, and naming them apart is what \
              lets a fault say which bit"
)]
pub struct Event
{
    /// `HTIF` of the receiving stream.
    pub half_transfer: bool,
    /// `TCIF` of the receiving stream.
    pub transfer_complete: bool,
    /// `TEIF` of the receiving stream.
    pub transfer_error: bool,
    /// `DMEIF` of the receiving stream.
    pub direct_mode_error: bool,
    /// `FEIF` of the receiving stream.
    pub fifo_error: bool,
    /// `EN` of the receiving stream.
    pub stream_enabled: bool,
    /// `NDTR` of the receiving stream.
    pub items: u32,
    /// `OVRUDR` of the receiving sub-block.
    pub overrun: bool,
    /// `AFSDET` and `LFSDET` of the receiving sub-block together.
    pub frame_mismatch: bool,
    /// `CNRDY` of the receiving sub-block.
    pub codec_not_ready: bool,
    /// `SAIEN` of the receiving sub-block.
    pub block_enabled: bool,
    /// `NDTR` of the stream feeding the master output sub-block.
    pub output_items: u32,
}

/// The frame the receiver declares and the buffer its stream fills.
///
/// Every value comes off the output transport plan rather than being named
/// here, so the frame the receiver declares is the frame the transmitter emits
/// and the two cannot disagree about the format, the slots or the length.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputPlan
{
    output: TransportPlan,
    buffer: u32,
    buffer_words: u32,
}

impl InputPlan
{
    /// Builds the plan an output transport plan and one buffer make.
    ///
    /// `validate` is what accepts it.
    #[must_use]
    pub const fn for_transport(output: &TransportPlan, buffer: u32) -> Self
    {
        Self
        {
            output: *output,
            buffer,
            buffer_words: output.transfer_items(),
        }
    }

    /// Returns the `NDTR` value, which is the buffer length in words.
    #[must_use]
    pub const fn transfer_items(self) -> u32
    {
        self.buffer_words
    }

    /// Returns the buffer address the stream fills.
    #[must_use]
    pub const fn buffer_address(self) -> u32
    {
        self.buffer
    }

    /// Returns the address every sample is read from.
    #[must_use]
    pub const fn data_address(self) -> u32
    {
        INPUT_DATA_ADDRESS
    }

    /// Returns the `MODE` field value.
    #[must_use]
    pub const fn mode_field(self) -> u8
    {
        MODE_SLAVE_RECEIVER
    }

    /// Returns the `SYNCEN` field value.
    #[must_use]
    pub const fn sync_field(self) -> u8
    {
        SYNC_EXTERNAL
    }

    /// Returns the `SYNCIN` field value.
    #[must_use]
    pub const fn sync_in_field(self) -> u8
    {
        SYNC_IN_FIRST_INTERFACE
    }

    /// Returns the `DMAREQ_ID` field value.
    #[must_use]
    pub const fn request_field(self) -> u8
    {
        INPUT_REQUEST
    }

    /// Returns the `DIR` field value.
    #[must_use]
    pub const fn direction_field(self) -> u8
    {
        TRANSFER_PERIPHERAL_TO_MEMORY
    }

    /// Returns the output transport plan the frame comes off.
    #[must_use]
    pub const fn frame(self) -> TransportPlan
    {
        self.output
    }

    /// Returns the slots one frame of the transmitted format carries.
    #[must_use]
    pub const fn frame_slots(self) -> u32
    {
        (self.output.slot_count_field() as u32).saturating_add(1)
    }

    /// Returns the words in one half of the buffer, which is one block.
    ///
    /// `validate` refuses a buffer of odd length, so an accepted plan splits
    /// exactly and the fallback below cannot be the answer.
    #[must_use]
    pub const fn block_words(self) -> u32
    {
        match self.buffer_words.checked_div(HALVES)
        {
            Some(words) => words,
            None => 0,
        }
    }

    /// Returns the first index of one block.
    #[must_use]
    pub const fn block_start(self, half: Half) -> u32
    {
        match half
        {
            Half::First => 0,
            Half::Second => self.block_words(),
        }
    }

    /// Returns the index one past the last of one block.
    #[must_use]
    pub const fn block_end(self, half: Half) -> u32
    {
        match half
        {
            Half::First => self.block_words(),
            Half::Second => self.buffer_words,
        }
    }

    /// Returns the words the read pointer must still have ahead of it when a
    /// carry starts.
    #[must_use]
    pub const fn guard_words(self) -> u32
    {
        match self.block_words().checked_div(GUARD_DIVISOR)
        {
            Some(words) => words,
            None => 0,
        }
    }

    /// Returns the position the receiver is started at.
    ///
    /// It is the distance the start-up aims to leave between the two COUNTERS,
    /// since the receiver writes its first word at position zero of its own
    /// buffer while the transmitter stands here. The shift a start-up on the
    /// aim leaves is this less the pipeline the two counters do not report.
    #[must_use]
    pub const fn arming_position(self) -> u32
    {
        match self.block_words().checked_div(HALVES)
        {
            Some(words) => words,
            None => 0,
        }
    }

    /// Returns half the width of the window the receiver is started in.
    #[must_use]
    pub const fn arming_window(self) -> u32
    {
        match self.block_words().checked_div(ARMING_WINDOW_DIVISOR)
        {
            Some(words) => words,
            None => 0,
        }
    }

    /// Returns the lowest shift a start-up may leave.
    #[must_use]
    pub const fn offset_low(self) -> u32
    {
        match self.block_words().checked_div(OFFSET_LOW_DIVISOR)
        {
            Some(words) => words,
            None => 0,
        }
    }

    /// Returns the highest shift a start-up may leave.
    #[must_use]
    pub const fn offset_high(self) -> u32
    {
        match self.block_words().saturating_mul(OFFSET_HIGH_NUMERATOR)
            .checked_div(OFFSET_HIGH_DIVISOR)
        {
            Some(words) => words,
            None => 0,
        }
    }

    /// Returns the position a transfer counter reading `items` stands at.
    ///
    /// RM0433 section 15.5.6 has the counter decrement after each transfer and
    /// reload at the end of a lap, so a counter of `n` on a buffer of `w` words
    /// says the next word moves at index `w - n`.
    ///
    /// A counter past the buffer describes no position and gives none.
    #[must_use]
    pub const fn position_of(self, items: u32) -> Option<u32>
    {
        if items > self.buffer_words
        {
            return None;
        }

        Some(self.buffer_words.saturating_sub(items))
    }

    /// Returns the words the transmitting side stands ahead of the receiving
    /// one.
    ///
    /// Both counters walk one word per slot off one frame clock, so this does
    /// not change once the receiver has started.
    ///
    /// Either counter past its buffer describes no distance and gives none.
    #[must_use]
    pub const fn offset_of(self, output_items: u32, items: u32) -> Option<u32>
    {
        let (Some(output), Some(input)) =
            (self.position_of(output_items), self.position_of(items))
        else
        {
            return None;
        };

        wrap(output.wrapping_add(self.buffer_words).wrapping_sub(input), self.buffer_words)
    }

    /// Returns whether the read pointer at `position` stands clear of a carry
    /// into `half` by the margin, at the instant `position` was read.
    ///
    /// The block a carry WRITES is the one the shift names, not the one the
    /// event named, so this measures the destination. Measuring the source
    /// would guard a stretch of buffer nothing is about to be written to.
    ///
    /// It answers for that instant alone. Whether the carry then STAYS clear of
    /// the pointer is the speed of the carry against the speed of the streams,
    /// which no counter reports, and the module documentation carries what a
    /// carry too slow to hold that does.
    ///
    /// The read pointer has to stand in the other half of the buffer AND still
    /// have the margin ahead of it, and the one expression below is both: the
    /// distance from the end of the destination forward to the read pointer is
    /// under one block less the margin exactly when the pointer is past that
    /// block and short of returning to it.
    #[must_use]
    pub const fn block_is_clear(self, half: Half, position: u32, shift: CarryShift) -> bool
    {
        let end = shift.destination(self, self.block_end(half));
        let raw = position.wrapping_add(self.buffer_words).wrapping_sub(end);

        let Some(travelled) = wrap(raw, self.buffer_words)
        else
        {
            return false;
        };

        travelled < self.block_words().saturating_sub(self.guard_words())
    }

    /// Returns nothing when the part accepts the plan.
    ///
    /// # Errors
    ///
    /// One variant of `InputPlanError` per bound, so a refusal names the bound
    /// it broke rather than the fact that something broke.
    pub const fn validate(self) -> Result<(), InputPlanError>
    {
        if self.buffer_words == 0
        {
            return Err(InputPlanError::BufferEmpty);
        }

        if self.buffer_words > u16::MAX as u32
        {
            return Err(InputPlanError::BufferTooLong);
        }

        if self.buffer_words != self.output.transfer_items()
        {
            return Err(InputPlanError::BufferLengthDiffers);
        }

        let slots = self.frame_slots();

        if !self.buffer_words.is_multiple_of(slots)
        {
            return Err(InputPlanError::BufferNotWholeFrames);
        }

        if !self.buffer_words.is_multiple_of(HALVES)
        {
            return Err(InputPlanError::BufferHalfNotWhole);
        }

        if self.guard_words() >= self.block_words()
        {
            return Err(InputPlanError::BufferTooShortForGuard);
        }

        if !self.buffer.is_multiple_of(WORD_BYTES)
        {
            return Err(InputPlanError::BufferUnaligned);
        }

        let bytes = self.buffer_words.saturating_mul(WORD_BYTES);

        if !reaches_memory(self.buffer, bytes)
        {
            return Err(InputPlanError::BufferUnreachable);
        }

        self.validate_against_output(bytes)
    }

    /// Checks the buffer against the two the transmitting streams replay.
    const fn validate_against_output(self, bytes: u32) -> Result<(), InputPlanError>
    {
        let master = self.output.buffer_address(crate::transport::BlockRole::Master);
        let slave = self.output.buffer_address(crate::transport::BlockRole::Slave);

        if overlaps(self.buffer, master, bytes) || overlaps(self.buffer, slave, bytes)
        {
            return Err(InputPlanError::BufferOverlapsOutput);
        }

        Ok(())
    }

    /// Returns nothing when both streams were armed with the plan.
    ///
    /// Called while the receiving stream is stopped, where its transfer counter
    /// still reads the length it was loaded with.
    ///
    /// # Errors
    ///
    /// `Stream`, carrying the field that disagreed.
    pub fn verify_armed(self, seen: &InputReadback) -> Result<(), PassthroughFault>
    {
        if seen.stream.peripheral_address != self.data_address()
        {
            return Err(PassthroughFault::Stream(InputStreamFault::PeripheralAddressWrong));
        }

        if seen.stream.memory_address != self.buffer_address()
        {
            return Err(PassthroughFault::Stream(InputStreamFault::MemoryAddressWrong));
        }

        if seen.stream.items != self.transfer_items()
        {
            return Err(PassthroughFault::Stream(InputStreamFault::ItemCountWrong));
        }

        Ok(())
    }

    /// Returns nothing when `seen` carries the plan.
    ///
    /// # Errors
    ///
    /// `Block` or `Stream`, carrying the field of that place that disagreed.
    /// The sub-block is checked first, because what it reports is the frame on
    /// the wire and the stream only moves what that frame produced.
    pub fn verify(self, seen: &InputReadback) -> Result<(), PassthroughFault>
    {
        if let Err(fault) = self.verify_block(&seen.block)
        {
            return Err(PassthroughFault::Block(fault));
        }

        if let Err(fault) = self.verify_stream(&seen.stream)
        {
            return Err(PassthroughFault::Stream(fault));
        }

        Ok(())
    }

    /// Checks the receiving sub-block against the plan and its own flags.
    fn verify_block(self, seen: &InputBlockReadback) -> Result<(), InputBlockFault>
    {
        self.verify_block_shape(seen)?;
        self.verify_block_frame(seen)?;
        verify_block_state(seen)
    }

    /// Checks the direction, the protocol and the synchronisation.
    fn verify_block_shape(self, seen: &InputBlockReadback) -> Result<(), InputBlockFault>
    {
        if seen.mode_bits != self.mode_field()
        {
            return Err(InputBlockFault::ModeWrong);
        }

        if seen.protocol_bits != self.output.protocol_field()
        {
            return Err(InputBlockFault::ProtocolWrong);
        }

        if seen.sync_bits != self.sync_field()
        {
            return Err(InputBlockFault::SynchronisationWrong);
        }

        if seen.sync_in_bits != self.sync_in_field()
        {
            return Err(InputBlockFault::SynchronisationSourceWrong);
        }

        if seen.sync_out_bits != SYNC_OUT_NONE
        {
            return Err(InputBlockFault::SynchronisationOutputWrong);
        }

        if seen.data_size_bits != self.output.data_size_field()
        {
            return Err(InputBlockFault::DataSizeWrong);
        }

        if seen.lsb_first
        {
            return Err(InputBlockFault::BitOrderWrong);
        }

        if seen.changes_on_falling_edge != self.output.clock_strobing_field()
        {
            return Err(InputBlockFault::ClockStrobingWrong);
        }

        Ok(())
    }

    /// Checks the frame and the slots inside it.
    fn verify_block_frame(self, seen: &InputBlockReadback) -> Result<(), InputBlockFault>
    {
        if seen.frame_length_field != self.output.frame_length_field()
        {
            return Err(InputBlockFault::FrameLengthWrong);
        }

        if seen.frame_active_field != self.output.frame_active_field()
        {
            return Err(InputBlockFault::FrameActiveLengthWrong);
        }

        if !seen.frame_marks_channel
        {
            return Err(InputBlockFault::FrameDefinitionWrong);
        }

        if seen.frame_active_high
        {
            return Err(InputBlockFault::FramePolarityWrong);
        }

        if !seen.frame_leads_first_bit
        {
            return Err(InputBlockFault::FrameOffsetWrong);
        }

        if seen.first_bit_offset_field != 0
        {
            return Err(InputBlockFault::FirstBitOffsetWrong);
        }

        if seen.slot_size_bits != self.output.slot_size_field()
        {
            return Err(InputBlockFault::SlotSizeWrong);
        }

        if seen.slot_count_field != self.output.slot_count_field()
        {
            return Err(InputBlockFault::SlotCountWrong);
        }

        if seen.slot_enable_bits != self.output.slot_enable_field()
        {
            return Err(InputBlockFault::SlotsNotEnabled);
        }

        if seen.fifo_threshold_bits != self.output.fifo_threshold_field()
        {
            return Err(InputBlockFault::FifoThresholdWrong);
        }

        Ok(())
    }

    /// Checks the receiving stream against the plan and its own flags.
    fn verify_stream(self, seen: &InputStreamReadback) -> Result<(), InputStreamFault>
    {
        if seen.request_bits != self.request_field()
        {
            return Err(InputStreamFault::RequestWrong);
        }

        if seen.sync_enabled
        {
            return Err(InputStreamFault::SynchronisationEnabled);
        }

        if seen.direction_bits != self.direction_field()
        {
            return Err(InputStreamFault::DirectionWrong);
        }

        if !seen.circular
        {
            return Err(InputStreamFault::NotCircular);
        }

        if !seen.memory_increments
        {
            return Err(InputStreamFault::MemoryNotIncrementing);
        }

        if seen.peripheral_increments
        {
            return Err(InputStreamFault::PeripheralIncrementing);
        }

        if seen.memory_width_bits != self.output.width_field()
        {
            return Err(InputStreamFault::MemoryWidthWrong);
        }

        if seen.peripheral_width_bits != self.output.width_field()
        {
            return Err(InputStreamFault::PeripheralWidthWrong);
        }

        if seen.double_buffered
        {
            return Err(InputStreamFault::DoubleBuffered);
        }

        if seen.priority_bits != self.output.priority_field()
        {
            return Err(InputStreamFault::PriorityWrong);
        }

        if seen.error_interrupt_enabled
        {
            return Err(InputStreamFault::ErrorInterruptEnabled);
        }

        if seen.event_interrupt_enabled
        {
            return Err(InputStreamFault::EventInterruptEnabled);
        }

        if seen.peripheral_address != self.data_address()
        {
            return Err(InputStreamFault::PeripheralAddressWrong);
        }

        if seen.memory_address != self.buffer_address()
        {
            return Err(InputStreamFault::MemoryAddressWrong);
        }

        verify_stream_state(seen, self)
    }
}

/// Words a carry adds to a source index to reach the output position the word
/// it holds was sent from.
///
/// The receiving buffer and the two output buffers share no origin. While the
/// receiving stream writes index `i`, the transmitting streams read index
/// `i + offset`, and the word arriving at `i` left the output buffers
/// `PIPELINE_WORDS` slots before that read, so it belongs at `i + offset` less
/// the pipeline. This is that distance.
///
/// It is measured once, at the bring-up, and is a constant of the run. A
/// mapping taken again at every event would carry the jitter of two register
/// reads into the position words land at, and a block landing a word off the
/// one before it is a step in the middle of the signal rather than a rounding.
///
/// It moves a whole number of frames, so a word received in slot 0 leaves in
/// slot 0. Two things hold that, and `measure` needs both: it takes the counter
/// distance down to the frame below, and the pipeline it then subtracts is a
/// whole number of frames as well. A shift an odd number of words long would
/// put the two channels one slot apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CarryShift(u32);

impl CarryShift
{
    /// Returns the shift the two transfer counters of one read describe, less
    /// the pipeline the data crosses between them.
    ///
    /// The distance goes down to the frame below first, since the two counters
    /// are read one after the other and the pair reads a word or two long when
    /// the reader takes the transmitting counter last. The pipeline comes off
    /// that, so a pipeline standing a slot inside a frame leaves a value
    /// `from_words` refuses and the machine goes silent instead of carrying
    /// every word onto the other converter channel.
    ///
    /// Gives nothing when either counter reads past its buffer, when the
    /// distance is shorter than the pipeline, and when what is left is not a
    /// shift `from_words` accepts.
    #[must_use]
    pub const fn measure(plan: InputPlan, output_items: u32, items: u32) -> Option<Self>
    {
        let Some(offset) = plan.offset_of(output_items, items)
        else
        {
            return None;
        };

        let Some(over) = offset.checked_rem(plan.frame_slots())
        else
        {
            return None;
        };

        let Some(carried) = offset.saturating_sub(over).checked_sub(PIPELINE_WORDS)
        else
        {
            return None;
        };

        Self::from_words(plan, carried)
    }

    /// Returns the shift `words` names.
    ///
    /// Gives nothing unless `words` is a whole number of frames and stands
    /// inside the band a start-up may leave a shift in, which is what a value
    /// that reached this through a word of memory has to prove before a carry
    /// is placed on it. This is the one place that band is applied, so a
    /// refusal here is the only reading of it.
    #[must_use]
    pub const fn from_words(plan: InputPlan, words: u32) -> Option<Self>
    {
        let slots = plan.frame_slots();

        if slots == 0 || !words.is_multiple_of(slots)
        {
            return None;
        }

        if words < plan.offset_low() || words > plan.offset_high()
        {
            return None;
        }

        Some(Self(words))
    }

    /// Returns the shift in words.
    #[must_use]
    pub const fn words(self) -> u32
    {
        self.0
    }

    /// Returns the output index the word at `index` of the receiving buffer is
    /// carried to.
    ///
    /// A buffer of no words names no position and gives zero, which is the case
    /// `validate` refuses ahead of every caller.
    #[must_use]
    pub const fn destination(self, plan: InputPlan, index: u32) -> u32
    {
        match wrap(index.wrapping_add(self.0), plan.transfer_items())
        {
            Some(at) => at,
            None => 0,
        }
    }
}

/// Checks the flags and the enable of the receiving sub-block.
fn verify_block_state(seen: &InputBlockReadback) -> Result<(), InputBlockFault>
{
    if seen.mono
    {
        return Err(InputBlockFault::MonoWrong);
    }

    if seen.tristate
    {
        return Err(InputBlockFault::TristateWrong);
    }

    if seen.any_interrupt_enabled
    {
        return Err(InputBlockFault::InterruptEnabled);
    }

    if !seen.transfer_enabled
    {
        return Err(InputBlockFault::TransferDisabled);
    }

    if !seen.enabled
    {
        return Err(InputBlockFault::NotEnabled);
    }

    if seen.overrun
    {
        return Err(InputBlockFault::Overrun);
    }

    if seen.frame_mismatch
    {
        return Err(InputBlockFault::FrameMismatch);
    }

    if seen.codec_not_ready
    {
        return Err(InputBlockFault::CodecNotReady);
    }

    if seen.fifo_level_bits == FIFO_LEVEL_EMPTY
    {
        return Err(InputBlockFault::FifoNeverFilled);
    }

    Ok(())
}

/// Checks the flags, the counter and the enable of the receiving stream.
fn verify_stream_state(seen: &InputStreamReadback, plan: InputPlan)
    -> Result<(), InputStreamFault>
{
    if seen.transfer_error
    {
        return Err(InputStreamFault::TransferError);
    }

    if seen.direct_mode_error
    {
        return Err(InputStreamFault::DirectModeError);
    }

    if seen.fifo_error
    {
        return Err(InputStreamFault::FifoError);
    }

    // The counter is a position inside the buffer once the stream runs, so what
    // is required of it here is that it names one. `verify_armed` compares it
    // against the plan while the stream is stopped.
    if seen.items > plan.transfer_items()
    {
        return Err(InputStreamFault::CounterOutOfRange);
    }

    if !seen.enabled
    {
        return Err(InputStreamFault::NotEnabled);
    }

    Ok(())
}

/// `FLVL` reporting an empty FIFO. RM0433 section 51.6.12.
const FIFO_LEVEL_EMPTY: u8 = 0b000;

/// Returns `value` folded onto one lap of `words`.
///
/// A lap of zero words describes no position and gives none, which is the case
/// `validate` refuses ahead of every caller.
const fn wrap(value: u32, words: u32) -> Option<u32>
{
    value.checked_rem(words)
}

/// Returns whether `bytes` from `one` and `bytes` from `other` share an address.
const fn overlaps(one: u32, other: u32, bytes: u32) -> bool
{
    one < other.saturating_add(bytes) && other < one.saturating_add(bytes)
}

/// What the input path reaches memory and the registers through.
///
/// Ten of the twelve methods are register access. Each write covers one step of
/// the order the part specifies, because that order is what makes the writes
/// take, `read` is the only way to observe the path outside a transfer event,
/// and `event` is the read one transfer event takes.
///
/// The last two are the buffers rather than the registers: `read_word` reads
/// the receiving buffer and `write_word` writes an output one. They carry no
/// decision, so an implementation of this trait holds no part of the carry but
/// the two accesses themselves.
///
/// Every method is `Sized`, and none of the generic functions over this trait
/// admits `?Sized`. A call through `dyn InputInterface` is a call the compiler
/// cannot fold, and the carry costs 88 instructions a word out of line against
/// 31 folded, so the door is shut rather than left open for a caller that does
/// not exist.
pub trait InputInterface
{
    /// Clears `EN` on the stream and `SAIEN` on the sub-block.
    fn stop(&mut self);

    /// Writes both fields of the `SAI_GCR` of the receiving interface, `SYNCIN`
    /// to the source it follows and `SYNCOUT` to the value that puts nothing
    /// out.
    ///
    /// Both, rather than one, because a warm restart can find either field
    /// where a previous image left it, and two interfaces each putting their
    /// clocks out is not a pair with one master.
    ///
    /// RM0433 section 51.6.1 requires the field written while both sub-blocks
    /// of that interface are disabled, which is what places this call after the
    /// step that stops them.
    fn declare_sync_input(&mut self, plan: &InputPlan);

    /// Puts the data pin on its alternate function.
    ///
    /// It is the one step of the sequence `read` does not observe, so a
    /// register block that gets it wrong produces a verified path that reaches
    /// no pad. What answers for it is the register block itself and the
    /// oscilloscope.
    fn open_pin(&mut self);

    /// Writes every configuration register of the receiving sub-block, `SAIEN`
    /// apart.
    fn write_block(&mut self, plan: &InputPlan);

    /// Routes and writes the receiving stream, but not `EN`.
    fn write_stream(&mut self, plan: &InputPlan);

    /// Clears the five interrupt flags of the receiving stream, and none of any
    /// other stream.
    fn clear_stream_flags(&mut self);

    /// Sets `EN` on the stream, then `SAIEN` on the sub-block.
    ///
    /// The stream first, so that no word the sub-block latches waits on a
    /// controller that is not listening.
    fn start(&mut self);

    /// Reads the sub-block, the stream and the transmitting counter back.
    fn read(&self) -> InputReadback;

    /// Sets `HTIE` and `TCIE` on the receiving stream and unmasks its line.
    ///
    /// It is the one call in this firmware that puts a served interrupt on a
    /// vector, so it is the one door the block structure comes through, and it
    /// takes the permit of the release gate BY VALUE to open it.
    ///
    /// That is the third term of the gate held by the type rather than by a
    /// comment. The permit has no public constructor and is neither `Copy` nor
    /// `Clone`, `arm` is the only holder of one, and nothing else in any crate
    /// can reach this method without one, so no interrupt able to write an
    /// output buffer can be enabled before the gate has raised the mute line
    /// over a buffer of zeros.
    fn enable_events(&mut self, permit: TonePermit);

    /// Reads what one transfer event carries.
    fn event(&self) -> Event;

    /// Clears the transfer event flag of one half, and no other flag.
    fn clear_event(&mut self, half: Half);

    /// Returns the word the receiving buffer holds at one index.
    ///
    /// The index is one `carry_block` produced from the plan, so it stands
    /// inside the buffer. An implementation that cannot reach the index returns
    /// zero, which is silence, rather than reading somewhere else.
    fn read_word(&self, index: u32) -> u32;

    /// Writes one word into the output buffer of one sub-block, at one index.
    ///
    /// The role names which buffer, so an implementation reaches memory and
    /// decides nothing: which buffers a word goes to, and at which index, is
    /// `carry_block` above. That puts the fan-out on the side of the build
    /// `cargo test` reaches, where dropping one of the two destinations turns a
    /// test red rather than leaving one converter replaying whatever its buffer
    /// last held.
    ///
    /// An implementation that cannot reach the index writes nothing.
    ///
    /// This is the only way the block structure reaches an output buffer, and
    /// it moves one word at a time, so no caller of this trait holds a writable
    /// pointer into either buffer.
    fn write_word(&mut self, role: BlockRole, index: u32, word: u32);
}

/// One second order section and the history it carries between blocks.
///
/// # Where it lives
///
/// The caller owns it and lends it to the carry. Nothing here is a `static`, so
/// a host test holds one per case and no case can reach the state of another,
/// and this crate keeps `unsafe` forbidden. A handler has no caller to own
/// anything, so the crate that owns the registers parks one and lends it the
/// same way, and it parks it before the write that puts a served interrupt on a
/// vector rather than after.
///
/// # Why it carries one history per slot
///
/// A buffer word is one slot of a frame, and the two slots of a frame are the
/// two channels of a converter, so consecutive words belong to different
/// channels. One history over that stream would feed each channel the samples
/// of the other, and the shape it produced would be no filter the coefficients
/// describe and nothing `cascade_magnitude` can be compared against.
///
/// So the section is one and the history is one per slot, which is the same
/// filter running on each channel. A shift moves a whole number of frames, so a
/// word keeps its slot from the receiving buffer to the output buffers and the
/// parity of the source index is the channel.
///
/// # What an unpublished one carries
///
/// `silent` holds `Biquad::SILENT`, which stops the signal. A stage that never
/// took its coefficients therefore carries silence rather than the full range
/// of a way with no filter on it, and there is no value of this type that means
/// no filtering.
///
/// # What bounds the sample it returns
///
/// A section overshoots. A crossover half answers a full scale step above full
/// scale, so a signal near the ceiling leaves the section past it, and the
/// conversion back to a buffer word SATURATES there. What a conversion that
/// wrapped would answer is a sample of the opposite sign at full scale, which
/// is a step of two full scales into a way with no analog filter in front of
/// it.
///
/// The saturation stands at the conversion and nowhere else: the history the
/// section feeds back is the value it computed. Feeding back the bounded value
/// instead would put a limiter inside the feedback of a filter, which is no
/// longer the filter the coefficients describe and no longer a shape
/// `cascade_magnitude` can be compared against.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilterStage
{
    section: Biquad,
    first_slot: BiquadState,
    second_slot: BiquadState,
}

/// Words between two samples of one channel.
///
/// A word is one slot of a frame and the slots of a frame are the channels of a
/// converter, so one channel is every `SLOTS_PER_FRAME` words of a buffer.
const CHANNEL_STRIDE: u32 = SLOTS_PER_FRAME as u32;

/// A frame of another width would leave a channel with no history of its own,
/// and the two above are the whole of what this type holds.
///
/// The transport pins the frame at 64 bits rather than at two slots, and a
/// frame of four 16 bit slots satisfies that while leaving two channels of this
/// stage sharing one history, so this stands on its own rather than repeating
/// it.
const _: () = assert!
(
    CHANNEL_STRIDE == 2,
    "a frame carries a slot this stage keeps no history for"
);

impl FilterStage
{
    /// Returns a stage that stops the signal.
    #[must_use]
    pub const fn silent() -> Self
    {
        Self::of(Biquad::SILENT)
    }

    /// Returns a stage running `section` on each slot, with no history behind
    /// either.
    #[must_use]
    pub const fn of(section: Biquad) -> Self
    {
        Self
        {
            section,
            first_slot: BiquadState::AT_REST,
            second_slot: BiquadState::AT_REST,
        }
    }

    /// Runs the word standing at `index` through the section, advancing the
    /// history of the slot that index names.
    fn carry(&mut self, index: u32, word: u32) -> u32
    {
        let section = self.section;

        let state = if index.is_multiple_of(CHANNEL_STRIDE)
        {
            &mut self.first_slot
        }
        else
        {
            &mut self.second_slot
        };

        carry_word(section, state, word)
    }
}

/// Runs one buffer word through `section` and advances `state`.
///
/// A buffer word carries one sample as an `i32` at full scale. It widens to the
/// format the section runs, crosses the section, and comes back through
/// `to_word`, which is what bounds it.
#[expect
(
    clippy::cast_precision_loss,
    reason = "the audio path is single precision, and this is the step that \
              gets a sample there"
)]
fn carry_word(section: Biquad, state: &mut BiquadState, word: u32) -> u32
{
    to_word(section.step(state, word.cast_signed() as f32))
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

/// Carries one half of the receiving buffer into both output buffers, at the
/// positions the words it holds were sent from.
///
/// `shift` is what turns a source index into that position, and it moves a
/// whole number of frames, so a word received in slot 0 of a frame leaves in
/// slot 0 of a frame and the channel it belongs to is the channel it comes back
/// on. `CarryShift` holds that over the counter distance and over the pipeline
/// taken off it both. Into both buffers, because the two converters replay one
/// each and a word that reaches only one of them leaves the other replaying the
/// lap before.
///
/// `stage` filters the word ONCE, between the read and the two writes, so the
/// two converters replay one sample rather than two computed apart, and the
/// section crosses a word once however many buffers it fans out to. The history
/// the stage carries runs on past the end of this call, which is what makes a
/// block boundary nothing the signal can hear.
///
/// The loop lives here rather than behind the trait so that a host test walks
/// it, the fan-out onto the two buffers included. The register block owns the
/// two accessors and nothing else of the carry, which leaves the part of this
/// path that touches samples on the side of the build that `cargo test`
/// reaches.
///
/// `plan` is the geometry the decision ahead of this was taken on, and `half`
/// the block that decision named. A block of the plan lies inside the buffer,
/// so every index this walks is one the two accessors accept.
///
/// # What it costs, and what a slower one does
///
/// The streams that replay the output buffers take one word every 11.3
/// microseconds at this frame rate, and this loop has a whole block to run in,
/// which is 441 words and 5.000 milliseconds. Folded into the handler it spends
/// 31 instructions and 11 memory accesses on a word, 1.8 to 2.0 microseconds,
/// so a block costs 806 to 896 microseconds and the carry moves a word about
/// six times faster than the streams read one.
///
/// A loop that costs more than 5.000 milliseconds a block never catches up
/// again, and the streams then overtake it inside the block: the words behind
/// the crossing leave holding the previous lap, which is a step of up to full
/// scale on a way with no analog filter in front of it. `next_block` cannot see
/// that, since it reads the counters at the entry and the crossing happens
/// between two entries. It refuses at the entry that follows, so the damage
/// comes out before the silence does. The module documentation carries the
/// whole of it.
pub(crate) fn carry_block<I>
(
    interface: &mut I,
    plan: &InputPlan,
    half: Half,
    shift: CarryShift,
    stage: &mut FilterStage
)
where
    I: InputInterface,
{
    let first = plan.block_start(half);
    let count = plan.block_words();
    let words = plan.transfer_items();
    let mut at = shift.destination(*plan, first);

    for step in 0..count
    {
        let index = first.saturating_add(step);
        let word = stage.carry(index, interface.read_word(index));

        interface.write_word(BlockRole::Master, at, word);
        interface.write_word(BlockRole::Slave, at, word);

        at = at.saturating_add(1);

        if at >= words
        {
            at = 0;
        }
    }
}

/// Polls each wait of the input bring-up holds before it gives up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputWaits
{
    /// Polls the stream and the sub-block get to report stopped.
    pub stop_polls: u32,
    /// Polls the transmitting read pointer gets to reach the arming window.
    pub phase_polls: u32,
    /// Polls the receiving counter gets to move once the sub-block runs.
    pub start_polls: u32,
    /// Polls the receiving counter gets to reload twice once the tone is in
    /// the output buffers.
    pub seed_polls: u32,
}

impl InputWaits
{
    /// Builds the waits a part running at `core_clock_hz` needs for `plan`.
    ///
    /// Each count covers a span at one core cycle per poll, and a poll is one
    /// whole read of the path, which is many cycles rather than one, so the
    /// figures are floors on the time a wait holds and not the time it takes.
    ///
    /// The stop and start waits cover 1 ms, which is 44 frames at the rate this
    /// chain runs, against the one frame RM0433 sections 51.4.16 and 15.3.14
    /// bound a stop at and the handful the receiver takes to fill its FIFO. The
    /// phase wait covers a whole lap of the buffer, since the read pointer
    /// passes every position once a lap. The seed wait covers `SEED_LAPS` of
    /// them.
    #[must_use]
    pub const fn for_plan(plan: &InputPlan, core_clock_hz: u32) -> Self
    {
        let lap = lap_microseconds(plan, 1);

        Self
        {
            stop_polls: wait_polls(core_clock_hz, 1_000),
            phase_polls: wait_polls(core_clock_hz, lap),
            start_polls: wait_polls(core_clock_hz, 1_000),
            seed_polls: wait_polls(core_clock_hz, lap_microseconds(plan, SEED_LAPS)),
        }
    }
}

/// Returns the microseconds `laps` of the buffer of `plan` take.
#[expect
(
    clippy::cast_possible_truncation,
    reason = "the comparison below is what keeps the value inside u32"
)]
const fn lap_microseconds(plan: &InputPlan, laps: u32) -> u32
{
    let slots = (plan.frame().slot_count_field() as u32).saturating_add(1);

    let frames = match plan.transfer_items().checked_div(slots)
    {
        Some(count) => count,
        None => 0,
    };

    let microseconds = (frames as u64)
        .saturating_mul(MICROSECONDS_PER_SECOND as u64)
        .saturating_mul(laps as u64)
        .div_ceil(SAMPLE_RATE_HZ as u64);

    if microseconds > u32::MAX as u64
    {
        u32::MAX
    }
    else
    {
        microseconds as u32
    }
}

/// Permission for the block structure to write the output buffers.
///
/// `arm` is the only thing that builds one, and it builds one only after it has
/// consumed the permit the release gate returns. The field is private, so no
/// other module and no other crate can construct one.
///
/// The state the carry advances does NOT live in here, and the reason is the
/// instant this is minted. `arm` unmasks the line inside the call that returns
/// this, so a handler entering between the unmask and the caller parking it
/// would find no state at all, and the handler answers a thing it cannot find
/// by silencing the machine. A `FilterStage` is parked ahead of the unmask
/// instead, the way the carry shift is, and the property this type holds is
/// unchanged: nothing reaches the vector without the permit.
#[derive(Debug, PartialEq, Eq)]
#[must_use]
pub struct Passthrough(());

/// Brings the input path up and proves it took the plan.
///
/// The order is what the part specifies, and every step of it is a requirement
/// rather than a habit.
///
/// The stream and the sub-block stop first, and the read after that proves both
/// went down. RM0433 section 15.5.5 makes every stream field writable only
/// while `EN` reads 0, and section 51.6.2 drops a `SAIEN` write that finds the
/// bit already set, so a warm restart can find either running.
///
/// `SYNCIN` is written there, with the sub-blocks of the receiving interface
/// disabled, which is where section 51.6.1 requires it. The other half of that
/// pair, `SYNCOUT` of the transmitting interface, belongs to the output
/// bring-up and is written inside it.
///
/// The pin opens next, then the sub-block and the stream while both are
/// stopped, and the counter is compared against the plan there, since that is
/// where it still reads the length it was loaded with.
///
/// The receiver is then held stopped until the transmitting read pointer
/// reaches `InputPlan::arming_position`. That is the whole of what fixes the
/// distance between the two sides: the receiver writes its first word at
/// position zero of its own buffer, so wherever the transmitter stands when it
/// starts is that distance for as long as the board runs. Starting it at any
/// position the loop happened to find would leave a board that tears every
/// carry, or does not, depending on nothing anyone can see.
///
/// The counter is then watched until it moves, which is what separates a
/// receiver that is configured from one that is receiving, and the read-back is
/// compared field by field. The shift is measured last, off both counters at
/// once less the pipeline they do not report, and refused unless it stands in
/// the band the arming step aims at. That check is the aim proved rather than
/// assumed, and `next_block` measures the read pointer again on every event it
/// serves.
///
/// The `CarryShift` this returns is where every carry of the run puts its
/// words. Measuring it here rather than at each event is what makes it one
/// mapping instead of a per-block decision.
///
/// XSMT is not touched here and neither output buffer is written, so the whole
/// of this runs under the mute the converters come out of reset with.
///
/// # Errors
///
/// `PlanRejected` before any register is touched, then `Sequence` carrying one
/// `InputSequenceFault` per step that did not take, and one of the two
/// read-back variants per field that disagreed.
pub fn bring_up<I>
(
    interface: &mut I,
    plan: &InputPlan,
    waits: &InputWaits
) -> Result<CarryShift, PassthroughFault>
where
    I: InputInterface,
{
    if let Err(error) = plan.validate()
    {
        return Err(PassthroughFault::PlanRejected(error));
    }

    interface.stop();

    if !poll_until(interface, waits.stop_polls, |seen| !seen.stream.enabled)
    {
        return Err(PassthroughFault::Sequence(InputSequenceFault::StreamNeverStopped));
    }

    if !poll_until(interface, waits.stop_polls, |seen| !seen.block.enabled)
    {
        return Err(PassthroughFault::Sequence(InputSequenceFault::BlockNeverStopped));
    }

    interface.declare_sync_input(plan);
    interface.open_pin();
    interface.write_block(plan);
    interface.write_stream(plan);
    plan.verify_armed(&interface.read())?;

    if !poll_until(interface, waits.phase_polls, |seen| at_arming_position(seen, plan))
    {
        return Err(PassthroughFault::Sequence(InputSequenceFault::OutputPhaseNeverReached));
    }

    interface.start();

    let armed = plan.transfer_items();

    if !poll_until(interface, waits.start_polls, |seen| seen.stream.items != armed)
    {
        return Err(PassthroughFault::Sequence(InputSequenceFault::ReceiverNeverAdvanced));
    }

    let running = interface.read();

    plan.verify(&running)?;

    // The band bounds the SHIFT, and this checks it in one place rather than
    // twice on two different numbers. `from_words` is the door a shift coming
    // back out of a word of memory passes and it bounds the shift, and
    // `block_is_clear` places the carry off the shift, so the number the bounds
    // describe is the one a carry uses. Checking the raw counter distance here
    // as well would put a second bound on a number that is `PIPELINE_WORDS`
    // larger, behind the same cause byte: a distance admitted here and refused
    // there would come out as `OffsetOutOfBand` with nothing saying which of
    // the two spoke.
    let Some(shift) = CarryShift::measure(*plan, running.output_items, running.stream.items)
    else
    {
        return Err(PassthroughFault::Sequence(InputSequenceFault::OffsetOutOfBand));
    };

    Ok(shift)
}

/// Puts the block structure into service.
///
/// `permit` is the release gate having verified the clocks, watched the
/// transfers over a whole lap and held a buffer of zeros over the converter
/// unmute ramp. It is taken BY VALUE, carried through this sequence and spent
/// on `InputInterface::enable_events`, which is the one door onto the vector.
/// So the interrupt that lets anything write the output buffers continuously
/// cannot be enabled by any path that does not hold the permit, and the permit
/// has no public constructor. That is the third term of the gate held by the
/// type rather than by a comment.
///
/// The seed wait comes first. In local loopback the link carries what this
/// board sends, so a passthrough started over an output buffer of zeros carries
/// silence round for ever and there is nothing in the machine that could break
/// the loop. The caller writes the tone into the output buffers before it calls
/// this, and this waits for the receiving counter to reload twice, which is a
/// whole lap of the buffer from wherever it was found, so what the first block
/// carries is the tone that went out and came back rather than the power-on
/// noise the receiving buffer was filled over.
///
/// The stream flags go down next, and the read after that is what the enable is
/// decided on: a flag raised during the bring-up belongs to the bring-up, and
/// every flag the handler goes on to read belongs to the run in front of it.
///
/// The interrupt goes up last. Until it does, that vector reaches the handler
/// that silences the machine, like every other vector of this firmware.
///
/// # Errors
///
/// `Sequence` when the counter never lapped inside the budget, and one of the
/// read-back variants when the path stopped carrying the plan while the seed
/// travelled. A refusal enables no interrupt and builds no `Passthrough`.
pub fn arm<I>
(
    interface: &mut I,
    permit: TonePermit,
    plan: &InputPlan,
    waits: &InputWaits
) -> Result<Passthrough, PassthroughFault>
where
    I: InputInterface,
{
    let opening = interface.read();

    plan.verify(&opening)?;

    let mut lap = Laps::opening(opening.stream.items);
    let mut remaining = waits.seed_polls;

    while !lap.lapped()
    {
        if remaining == 0
        {
            return Err(PassthroughFault::Sequence(InputSequenceFault::SeedNeverLapped));
        }

        remaining = remaining.saturating_sub(1);

        let seen = interface.read();

        plan.verify(&seen)?;
        lap.fold(seen.stream.items);
    }

    interface.clear_stream_flags();
    plan.verify(&interface.read())?;
    interface.enable_events(permit);

    Ok(Passthrough(()))
}

/// Returns the block one transfer event asks for.
///
/// This is the whole decision a handler makes. It runs before anything is
/// written, so a fault it returns has cost nothing but the entry.
///
/// The sub-block comes first, because what it reports is the frame on the wire
/// and a stream only moves what that frame produced. The stream comes next. The
/// two event flags decide which block, and then the read pointer of the
/// transmitting side decides whether that block can be written at all.
///
/// Both flags set is refused rather than served twice. It says a block went by
/// with nothing carrying it, so one half of the output buffers holds samples
/// older than the other and the seam between them is audible whatever this
/// handler does next.
///
/// Neither flag set is the trigger this handler did not ask for. It is refused
/// for the same reason the default handler of this firmware refuses one: no
/// path of this binary raises this line for any other reason, so an entry
/// without a flag says the part is not the one the plan armed.
///
/// # Errors
///
/// `Event`, carrying what the entry refused with.
pub fn next_block(seen: &Event, plan: &InputPlan, shift: CarryShift)
    -> Result<Half, PassthroughFault>
{
    if let Err(fault) = check_event_state(seen)
    {
        return Err(PassthroughFault::Event(fault));
    }

    let half = match (seen.half_transfer, seen.transfer_complete)
    {
        (false, false) => return Err(PassthroughFault::Event(EventFault::Spurious)),
        (true, true) => return Err(PassthroughFault::Event(EventFault::BothHalvesPending)),
        (true, false) => Half::First,
        (false, true) => Half::Second,
    };

    let Some(position) = plan.position_of(seen.output_items)
    else
    {
        return Err(PassthroughFault::Event(EventFault::OutputCounterOutOfRange));
    };

    if plan.position_of(seen.items).is_none()
    {
        return Err(PassthroughFault::Event(EventFault::CounterOutOfRange));
    }

    if !plan.block_is_clear(half, position, shift)
    {
        return Err(PassthroughFault::Event(EventFault::ReadPointerInBlock));
    }

    Ok(half)
}

/// Checks everything one event carries but the two flags and the counters.
fn check_event_state(seen: &Event) -> Result<(), EventFault>
{
    if seen.overrun
    {
        return Err(EventFault::Overrun);
    }

    if seen.frame_mismatch
    {
        return Err(EventFault::FrameMismatch);
    }

    if seen.codec_not_ready
    {
        return Err(EventFault::CodecNotReady);
    }

    if !seen.block_enabled
    {
        return Err(EventFault::BlockStopped);
    }

    if seen.transfer_error
    {
        return Err(EventFault::TransferError);
    }

    if seen.direct_mode_error
    {
        return Err(EventFault::DirectModeError);
    }

    if seen.fifo_error
    {
        return Err(EventFault::FifoError);
    }

    if !seen.stream_enabled
    {
        return Err(EventFault::StreamStopped);
    }

    Ok(())
}

/// Serves one transfer event.
///
/// The flag goes down BEFORE the block is carried. A flag left standing has the
/// part raise the line again the moment this returns, and the handler enters
/// for ever on audio that stopped moving while the converters stay unmuted.
/// Nothing in this firmware bounds that, since the watchdog is not here yet.
///
/// Taking it down first also means a block whose carry is somehow interrupted
/// is not carried twice. What it costs is that a second event landing during
/// the carry is served on the next entry rather than merged into this one,
/// which is what `EventFault::BothHalvesPending` reports.
///
/// # Errors
///
/// `Event`, carrying what the entry refused with. Nothing has been written when
/// one comes back and `stage` has not moved, so the caller answers by silencing
/// the machine.
pub fn serve<I>
(
    interface: &mut I,
    plan: &InputPlan,
    shift: CarryShift,
    stage: &mut FilterStage
) -> Result<(), PassthroughFault>
where
    I: InputInterface,
{
    let half = next_block(&interface.event(), plan, shift)?;

    interface.clear_event(half);
    carry_block(interface, plan, half, shift, stage);

    Ok(())
}

/// Returns whether the transmitting read pointer stands in the arming window.
fn at_arming_position(seen: &InputReadback, plan: &InputPlan) -> bool
{
    let Some(position) = plan.position_of(seen.output_items)
    else
    {
        return false;
    };

    let aim = plan.arming_position();
    let window = plan.arming_window();

    position >= aim.saturating_sub(window) && position <= aim.saturating_add(window)
}

/// Polls `interface` until `ready` holds, or `polls` more reads have gone by.
///
/// The condition is read before the budget is spent, so a budget of zero still
/// buys one look.
fn poll_until<T, F>(interface: &T, polls: u32, ready: F) -> bool
where
    T: InputInterface,
    F: Fn(&InputReadback) -> bool,
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

// The three assertions below carry the arithmetic that ties the divisors of the
// band, the margin and the arming window together. Each is written so that the
// divisor it names appears on one side alone: a form where a divisor cancels
// off both sides holds nothing, whatever its message says, and reads as a
// derivation while a mutation of that divisor walks past it.
//
// The aim and the window bound a distance between the two counters while the
// band bounds a shift, and `PIPELINE_WORDS` is what stands between the two. It
// is a count of words rather than a fraction of a block, so the assertion that
// has to hold the aim above the low bound carries it over
// `SHORTEST_BLOCK_WORDS` and the other two stay dimensionless. The pipeline
// only ever lowers a shift, which is why the high bound needs none of it.

// offset_high < 1 - 1/guard, over one block, with no division.
const _: () = assert!
(
    OFFSET_HIGH_NUMERATOR * GUARD_DIVISOR
        < (GUARD_DIVISOR - 1) * OFFSET_HIGH_DIVISOR,
    "the highest shift a start-up may leave stands clear of the bound every \
     event is checked against, which is one block less the margin"
);

// pipeline + 1/window < 1/halves - 1/offset_low, over the shortest block, with
// no division.
const _: () = assert!
(
    PIPELINE_WORDS * HALVES * OFFSET_LOW_DIVISOR * ARMING_WINDOW_DIVISOR
        + SHORTEST_BLOCK_WORDS * HALVES * OFFSET_LOW_DIVISOR
        < SHORTEST_BLOCK_WORDS * ARMING_WINDOW_DIVISOR
            * (OFFSET_LOW_DIVISOR - HALVES),
    "the window the receiver is started in, plus the pipeline that comes off \
     the distance it leaves, is narrower than the drop from the aim to the low \
     bound, so a start-up that lands inside the window leaves a shift inside \
     the band"
);

// 1/window < offset_high - 1/halves, over one block, with no division.
const _: () = assert!
(
    HALVES * OFFSET_HIGH_DIVISOR
        < ARMING_WINDOW_DIVISOR
            * (OFFSET_HIGH_NUMERATOR * HALVES - OFFSET_HIGH_DIVISOR),
    "the window the receiver is started in is narrower than the climb from the \
     aim up to the high bound, and the pipeline takes a shift down from there, \
     so a start-up that lands inside the window leaves a shift under the high \
     bound"
);

#[cfg(test)]
mod tests
{
    // Every value below is a position or a length inside one buffer of 882
    // words, or that buffer address plus a multiple of it, so no expression
    // here comes near the width of the type.
    #![allow(clippy::arithmetic_side_effects, clippy::cast_possible_truncation)]

    use super::*;
    use crate::clock::AUDIO_PLAN;
    use crate::constants::LOW_MID_HZ;
    use crate::filter::{LINKWITZ_RILEY_SECTIONS, cascade_magnitude, linkwitz_riley_low_pass};
    use crate::transport::{TONE_SAMPLES, TONE_TABLE};
    use core::cell::Cell;
    use core::f64::consts::TAU;
    use libm::{cos, round, sin, sqrt};

    /// Words in each buffer of the tests, one tone period per channel.
    const TEST_WORDS: u32 = TONE_SAMPLES as u32 * 2;

    /// Output buffers one word of a carry lands in, one per converter.
    const OUTPUT_BUFFERS: u32 = 2;

    /// Bytes in each buffer of the tests.
    const TEST_BYTES: u32 = TEST_WORDS * WORD_BYTES;

    /// First address of the memory the buffers live in.
    const REGION_BASE: u32 = 0x2400_0000;

    /// Address of the buffer the master output stream replays.
    const MASTER_BUFFER: u32 = REGION_BASE;

    /// Address of the buffer the slave output stream replays.
    const SLAVE_BUFFER: u32 = REGION_BASE + TEST_BYTES;

    /// Address of the buffer the receiving stream fills.
    const INPUT_BUFFER: u32 = REGION_BASE + 2 * TEST_BYTES;

    /// `FLVL` reporting a FIFO holding words.
    const FIFO_LEVEL_QUARTER: u8 = 0b001;

    /// One broken field of a read-back, and the fault it must produce.
    type Mutation = (fn(&mut InputReadback), PassthroughFault);

    /// One broken field of a transfer event, and the fault it must produce.
    type EventMutation = (fn(&mut Event), EventFault);

    /// Returns the output transport plan the tests are built on.
    fn output_plan() -> TransportPlan
    {
        TransportPlan::for_clock(&AUDIO_PLAN, MASTER_BUFFER, SLAVE_BUFFER, TEST_WORDS)
    }

    /// Returns the input plan the tests are built on.
    fn plan() -> InputPlan
    {
        InputPlan::for_transport(&output_plan(), INPUT_BUFFER)
    }

    /// The waits a run at the clock the part boots on gets.
    fn waits() -> InputWaits
    {
        InputWaits::for_plan(&plan(), 64_000_000)
    }

    /// The shift a bring-up leaves on an interface armed at the aim.
    ///
    /// It is derived from the two counters the way the bring-up derives it, so
    /// a test that carries a block puts its words where the run would.
    fn shift() -> CarryShift
    {
        shift_at(plan().arming_position())
    }

    /// The shift a distance of `offset` words between the two COUNTERS makes.
    ///
    /// Built here rather than measured, so a test comparing what the bring-up
    /// returns against it compares two derivations and not one with itself. The
    /// pipeline comes off here for the same reason it comes off in `measure`:
    /// the counters stand `offset` apart while the data crossed that many words
    /// less the pipeline. The two operations run in the order `measure` runs
    /// them, so a reader comparing the two reads one rule.
    fn shift_at(offset: u32) -> CarryShift
    {
        let truncated = offset - offset % plan().frame_slots();

        CarryShift(truncated - PIPELINE_WORDS)
    }

    /// Returns the section every test below runs.
    ///
    /// One section of the four pole low-pass the low way carries, which is the
    /// section the register crate publishes, so a test measures the filter the
    /// machine runs rather than one written for the occasion.
    fn section() -> Biquad
    {
        let mut sections = [Biquad::SILENT; LINKWITZ_RILEY_SECTIONS];
        let built = linkwitz_riley_low_pass(LOW_MID_HZ, SAMPLE_RATE_HZ, &mut sections);

        assert_eq!(built, Ok(LINKWITZ_RILEY_SECTIONS), "the low-pass refused its own corner");

        sections.first().copied().unwrap_or(Biquad::SILENT)
    }

    /// Returns a stage at rest running that section.
    fn stage() -> FilterStage
    {
        FilterStage::of(section())
    }

    /// Returns a stage that answers every word unchanged.
    ///
    /// The two loop tests below measure WHERE a word lands over laps of a
    /// closed link, and a closed link crosses the section once a lap, so a
    /// shape that attenuates takes the content to zero and leaves them
    /// comparing silence against silence. A transparent section is what keeps
    /// the placement readable. The response of the real section is measured
    /// over a driven source instead, where it belongs.
    fn transparent_stage() -> FilterStage
    {
        FilterStage::of(crate::filter::unit_section_for_test())
    }

    /// Returns the word `word` becomes once it has crossed the stage.
    ///
    /// A transparent section changes no value, and the sample still crosses
    /// single precision, which holds 24 bits of a sample that carries 32. So a
    /// word comes back rounded, by at most half of one unit in the last place
    /// of its own magnitude, and the rounded value is a fixed point of the trip:
    /// it is already a float, so a second crossing leaves it where it is. That
    /// is what makes this an exact expectation over any number of laps rather
    /// than a tolerance.
    #[expect
    (
        clippy::cast_precision_loss,
        reason = "the narrowing is the property this measures"
    )]
    fn narrowed(word: u32) -> u32
    {
        ((word.cast_signed() as f32) as i32).cast_unsigned()
    }

    /// Returns `buffer` with every word narrowed.
    fn narrowed_buffer(buffer: &[u32; TEST_WORDS as usize]) -> [u32; TEST_WORDS as usize]
    {
        let mut out = [0_u32; TEST_WORDS as usize];

        for (slot, word) in out.iter_mut().zip(buffer.iter())
        {
            *slot = narrowed(*word);
        }

        out
    }

    /// Builds a permit the way the release gate does.
    ///
    /// The type has no public constructor, which is the whole point of it, so a
    /// test that needs one takes it from the gate that builds one.
    fn permit() -> TonePermit
    {
        crate::release::tone_permit_for_test()
    }

    /// An interface whose registers answer the way the part does.
    ///
    /// `image` holds the fields at their reset values, so a step the sequence
    /// skips shows up as a reset value rather than as the plan. Every field a
    /// read reports is derived from that image, so nothing here states what a
    /// register is supposed to say.
    #[expect
    (
        clippy::struct_excessive_bools,
        reason = "each flag is one way a register block misbehaves, and naming \
                  them apart is what lets a test pick one"
    )]
    struct MockInput
    {
        image: InputReadback,
        polls: Cell<u32>,
        running: bool,
        /// Words the transmitting counter stands ahead of the receiving one,
        /// which is what the two counters read and not the shift a bring-up
        /// publishes off them.
        offset: u32,
        stream_stays_enabled: bool,
        block_stays_enabled: bool,
        receiver_stalls: bool,
        phase_never_moves: bool,
        refuse_sync_input: bool,
        events_enabled: bool,
        /// The half whose transfer event the part raises next, so a test serves
        /// either one and not only the one a fresh mock happens to hold.
        next_event: Half,
        steps: u32,
        /// The receiving buffer, so a host test walks the real carry rather
        /// than a note that one was asked for.
        source: [u32; TEST_WORDS as usize],
        /// The two output buffers the carry writes.
        master: [u32; TEST_WORDS as usize],
        slave: [u32; TEST_WORDS as usize],
        /// How many writes `write_word` was given, so a test reads that a whole
        /// block reached both buffers and not only that something did.
        written: u32,
        carried: Option<(Half, u32)>,
        cleared: Option<(Half, u32)>,
        flags_cleared_at: Option<u32>,
        events_enabled_at: Option<u32>,
    }

    impl MockInput
    {
        /// Returns the number of the step about to run.
        fn step(&mut self) -> u32
        {
            self.steps = self.steps.saturating_add(1);
            self.steps
        }
    }

    impl MockInput
    {
        /// Builds an interface that takes every step.
        ///
        /// RM0433 section 51.6.2 resets `SAI_xCR1` to `0x0000_0040`, which is
        /// `DS` at 8 bits and every other field clear, section 51.6.6 resets
        /// `SAI_xFRCR` to `0x0000_0007`, which is a frame of eight bit clock
        /// periods, and sections 51.6.4, 51.6.8 and 15.5.5 reset their
        /// registers to zero.
        fn healthy() -> Self
        {
            Self::healthy_at_phase(0)
        }

        /// Builds a healthy interface whose counters open `phase` words into
        /// the lap.
        ///
        /// The mock walks one word a read, so the poll count IS the position,
        /// and biasing it is what puts the pair of sequences under test at a
        /// chosen phase of the buffer. A window test that only ever opens at
        /// one phase tests that phase and not the window.
        fn healthy_at_phase(phase: u32) -> Self
        {
            let mut built = Self::at_reset();
            built.polls = Cell::new(phase);
            built
        }

        fn at_reset() -> Self
        {
            Self
            {
                image: InputReadback
                {
                    block: reset_block(),
                    stream: reset_stream(),
                    output_items: TEST_WORDS,
                },
                polls: Cell::new(0),
                running: false,
                offset: plan().arming_position(),
                stream_stays_enabled: false,
                block_stays_enabled: false,
                receiver_stalls: false,
                phase_never_moves: false,
                refuse_sync_input: false,
                events_enabled: false,
                next_event: Half::First,
                steps: 0,
                source: [0; TEST_WORDS as usize],
                master: [0; TEST_WORDS as usize],
                slave: [0; TEST_WORDS as usize],
                written: 0,
                carried: None,
                cleared: None,
                flags_cleared_at: None,
                events_enabled_at: None,
            }
        }
    }

    /// Returns the receiving sub-block at the reset value of its registers.
    fn reset_block() -> InputBlockReadback
    {
        InputBlockReadback
        {
            mode_bits: 0,
            protocol_bits: 0,
            data_size_bits: 0b010,
            lsb_first: false,
            changes_on_falling_edge: false,
            sync_bits: 0,
            sync_in_bits: 0,
            sync_out_bits: 0,
            mono: false,
            tristate: false,
            fifo_threshold_bits: 0,
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
            any_interrupt_enabled: false,
            fifo_level_bits: FIFO_LEVEL_EMPTY,
            overrun: false,
            frame_mismatch: false,
            codec_not_ready: false,
        }
    }

    /// Returns the receiving stream at the reset value of its registers.
    fn reset_stream() -> InputStreamReadback
    {
        InputStreamReadback
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
            event_interrupt_enabled: false,
            error_interrupt_enabled: false,
            peripheral_address: 0,
            memory_address: 0,
            items: 0,
            enabled: false,
            transfer_error: false,
            direct_mode_error: false,
            fifo_error: false,
        }
    }

    impl InputInterface for MockInput
    {
        fn stop(&mut self)
        {
            if !self.stream_stays_enabled
            {
                self.image.stream.enabled = false;
            }

            if !self.block_stays_enabled
            {
                self.image.block.enabled = false;
            }
        }

        fn declare_sync_input(&mut self, plan: &InputPlan)
        {
            if self.refuse_sync_input
            {
                return;
            }

            self.image.block.sync_in_bits = plan.sync_in_field();
            self.image.block.sync_out_bits = SYNC_OUT_NONE;
        }

        fn open_pin(&mut self)
        {
        }

        fn write_block(&mut self, plan: &InputPlan)
        {
            let frame = plan.frame();
            let block = &mut self.image.block;

            block.mode_bits = plan.mode_field();
            block.sync_bits = plan.sync_field();
            block.protocol_bits = frame.protocol_field();
            block.data_size_bits = frame.data_size_field();
            block.changes_on_falling_edge = frame.clock_strobing_field();
            block.fifo_threshold_bits = frame.fifo_threshold_field();
            block.frame_length_field = frame.frame_length_field();
            block.frame_active_field = frame.frame_active_field();
            block.frame_marks_channel = true;
            block.frame_leads_first_bit = true;
            block.slot_size_bits = frame.slot_size_field();
            block.slot_count_field = frame.slot_count_field();
            block.slot_enable_bits = frame.slot_enable_field();
            block.transfer_enabled = true;
        }

        fn write_stream(&mut self, plan: &InputPlan)
        {
            let frame = plan.frame();
            let stream = &mut self.image.stream;

            stream.request_bits = plan.request_field();
            stream.direction_bits = plan.direction_field();
            stream.circular = true;
            stream.memory_increments = true;
            stream.memory_width_bits = frame.width_field();
            stream.peripheral_width_bits = frame.width_field();
            stream.priority_bits = frame.priority_field();
            stream.peripheral_address = plan.data_address();
            stream.memory_address = plan.buffer_address();
            stream.items = plan.transfer_items();
        }

        fn clear_stream_flags(&mut self)
        {
            self.flags_cleared_at = Some(self.polls.get());
            self.image.stream.transfer_error = false;
            self.image.stream.direct_mode_error = false;
            self.image.stream.fifo_error = false;
        }

        fn start(&mut self)
        {
            self.image.stream.enabled = true;
            self.image.block.enabled = true;
            self.image.block.fifo_level_bits = FIFO_LEVEL_QUARTER;
            self.running = true;
            self.polls.set(0);
        }

        /// Walks both counters one word a read once the receiver runs.
        ///
        /// They walk together, because one frame clock drives both, and the
        /// transmitting one is placed `offset` words ahead of the receiving one
        /// so a test can put the read pointer wherever it needs it.
        fn read(&self) -> InputReadback
        {
            let polls = self.polls.get().saturating_add(1);
            self.polls.set(polls);

            let mut seen = self.image;

            if !self.running
            {
                if !self.phase_never_moves
                {
                    let position = polls % TEST_WORDS;
                    seen.output_items = TEST_WORDS - position;
                }

                return seen;
            }

            if !self.receiver_stalls
            {
                let position = polls % TEST_WORDS;
                seen.stream.items = TEST_WORDS - position;
                seen.output_items = TEST_WORDS - (position + self.offset) % TEST_WORDS;
            }

            seen
        }

        fn enable_events(&mut self, _permit: TonePermit)
        {
            self.events_enabled_at = Some(self.polls.get());
            self.events_enabled = true;
            self.image.stream.event_interrupt_enabled = true;
        }

        /// Reports the entry the part raises when a half has just been filled.
        ///
        /// The receiving counter stands at the end of that half and the
        /// transmitting one `offset` words further on, which is what the whole
        /// path is arranged to leave between them, so the two flags and the two
        /// counters of one entry come off one position rather than being set
        /// side by side.
        fn event(&self) -> Event
        {
            let plan = plan();
            let words = plan.transfer_items();
            let position = plan.block_end(self.next_event) % words;
            let output = (position + self.offset) % words;
            let seen = self.image;

            Event
            {
                half_transfer: matches!(self.next_event, Half::First),
                transfer_complete: matches!(self.next_event, Half::Second),
                transfer_error: seen.stream.transfer_error,
                direct_mode_error: seen.stream.direct_mode_error,
                fifo_error: seen.stream.fifo_error,
                stream_enabled: seen.stream.enabled,
                items: words - position,
                overrun: seen.block.overrun,
                frame_mismatch: seen.block.frame_mismatch,
                codec_not_ready: seen.block.codec_not_ready,
                block_enabled: seen.block.enabled,
                output_items: words - output,
            }
        }

        fn clear_event(&mut self, half: Half)
        {
            let at = self.step();
            self.cleared = Some((half, at));
        }

        fn read_word(&self, index: u32) -> u32
        {
            self.source.get(index as usize).copied().unwrap_or(0)
        }

        fn write_word(&mut self, role: BlockRole, index: u32, word: u32)
        {
            // The step is recorded on the FIRST word alone, so the ordering
            // test still reads one number for the carry, and the half is
            // derived from the index rather than announced.
            if self.written == 0
            {
                let at = self.step();
                let half = if index < plan().block_words()
                {
                    Half::First
                }
                else
                {
                    Half::Second
                };

                self.carried = Some((half, at));
            }

            self.written = self.written.saturating_add(1);

            // One buffer, the one the role names. A carry that reaches only one
            // of the two leaves the other holding the lap before, so the two
            // arrays are kept apart here rather than written together.
            let buffer = match role
            {
                BlockRole::Master => &mut self.master,
                BlockRole::Slave => &mut self.slave,
            };

            if let Some(slot) = buffer.get_mut(index as usize)
            {
                *slot = word;
            }
        }
    }

    /// Returns a read-back a bring-up on a healthy interface leaves behind.
    fn verified_readback() -> InputReadback
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));

        let seen = interface.read();

        assert_eq!(plan().verify(&seen), Ok(()));

        seen
    }

    /// Returns an event a running path with a clear read pointer produces.
    fn healthy_event(half: Half) -> Event
    {
        let plan = plan();
        let position = (plan.block_end(half) + plan.arming_position())
            % plan.transfer_items();

        Event
        {
            half_transfer: matches!(half, Half::First),
            transfer_complete: matches!(half, Half::Second),
            transfer_error: false,
            direct_mode_error: false,
            fifo_error: false,
            stream_enabled: true,
            items: plan.transfer_items() - plan.block_end(half) % plan.transfer_items(),
            overrun: false,
            frame_mismatch: false,
            codec_not_ready: false,
            block_enabled: true,
            output_items: plan.transfer_items() - position,
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

    #[test]
    fn the_plan_takes_its_frame_off_the_output_transport()
    {
        let plan = plan();

        assert_eq!(plan.frame(), output_plan());
        assert_eq!(plan.transfer_items(), output_plan().transfer_items());
        assert_eq!(plan.data_address(), INPUT_DATA_ADDRESS);
    }

    #[test]
    fn a_healthy_plan_is_accepted()
    {
        assert_eq!(plan().validate(), Ok(()));
    }

    #[test]
    fn a_buffer_overlapping_an_output_buffer_is_refused()
    {
        for buffer in [MASTER_BUFFER, SLAVE_BUFFER, MASTER_BUFFER + 4]
        {
            let plan = InputPlan::for_transport(&output_plan(), buffer);

            assert_eq!(plan.validate(), Err(InputPlanError::BufferOverlapsOutput));
        }
    }

    #[test]
    fn a_buffer_off_a_word_boundary_is_refused()
    {
        let plan = InputPlan::for_transport(&output_plan(), INPUT_BUFFER + 2);

        assert_eq!(plan.validate(), Err(InputPlanError::BufferUnaligned));
    }

    #[test]
    fn a_buffer_outside_the_reachable_memory_is_refused()
    {
        let plan = InputPlan::for_transport(&output_plan(), 0x2000_0000);

        assert_eq!(plan.validate(), Err(InputPlanError::BufferUnreachable));
    }

    #[test]
    fn the_slot_a_block_starts_on_is_the_parity_of_its_first_index()
    {
        // 441 frames is odd, so the second block starts on the second slot of a
        // frame. A carry moves a whole number of frames and does not care, and a
        // stage that reads per channel takes the parity off block_start.
        let plan = plan();

        assert_eq!(plan.block_start(Half::First) % 2, 0);
        assert_eq!(plan.block_start(Half::Second) % 2, 1);
    }

    #[test]
    fn the_two_halves_split_the_buffer_and_nothing_else()
    {
        let plan = plan();

        assert_eq!(plan.block_start(Half::First), 0);
        assert_eq!(plan.block_end(Half::First), plan.block_words());
        assert_eq!(plan.block_start(Half::Second), plan.block_words());
        assert_eq!(plan.block_end(Half::Second), plan.transfer_items());
        assert_eq!(plan.block_words() * HALVES, plan.transfer_items());
    }

    #[test]
    fn a_counter_reads_as_the_position_of_the_next_word()
    {
        let plan = plan();

        assert_eq!(plan.position_of(plan.transfer_items()), Some(0));
        assert_eq!(plan.position_of(1), Some(plan.transfer_items() - 1));
        assert_eq!(plan.position_of(plan.transfer_items() + 1), None);
    }

    #[test]
    fn the_distance_between_the_two_sides_is_read_off_both_counters()
    {
        let plan = plan();
        let words = plan.transfer_items();

        // The transmitter stands at 100 and the receiver at 40, so it leads by
        // 60, and the same pair the other way round wraps rather than going
        // negative.
        assert_eq!(plan.offset_of(words - 100, words - 40), Some(60));
        assert_eq!(plan.offset_of(words - 40, words - 100), Some(words - 60));
        assert_eq!(plan.offset_of(words + 1, words), None);
        assert_eq!(plan.offset_of(words, words + 1), None);
    }

    #[test]
    fn a_block_is_clear_exactly_while_the_read_pointer_is_past_it_with_room()
    {
        let plan = plan();
        let words = plan.transfer_items();
        let block = plan.block_words();
        let guard = plan.guard_words();

        for half in [Half::First, Half::Second]
        {
            let end = shift().destination(plan, plan.block_end(half));

            // Every position of the lap is swept, so the boundary is read off
            // the whole domain rather than off the two ends of it.
            for step in 0..words
            {
                let position = (end + step) % words;

                assert_eq!
                (
                    plan.block_is_clear(half, position, shift()),
                    step < block - guard,
                    "half {half:?} at position {position}"
                );
            }
        }
    }

    #[test]
    fn a_read_pointer_inside_the_block_is_never_clear()
    {
        let plan = plan();

        // The block a carry writes is the one the shift names, so the stretch
        // no read pointer may stand in is that one and not the half the event
        // named.
        for half in [Half::First, Half::Second]
        {
            let first = shift().destination(plan, plan.block_start(half));

            for step in 0..plan.block_words()
            {
                let position = (first + step) % plan.transfer_items();

                assert!
                (
                    !plan.block_is_clear(half, position, shift()),
                    "half {half:?} at position {position}"
                );
            }
        }
    }

    #[test]
    fn the_band_a_start_up_may_land_in_is_inside_what_every_event_accepts()
    {
        let plan = plan();
        let words = plan.transfer_items();

        // The distance the start-up leaves is the read pointer position at the
        // moment the receiver is at zero, so an event on either half finds the
        // pointer one block plus that distance along. The shift is derived from
        // that same distance, which is why it is built inside the sweep. The
        // band bounds the SHIFT, so the counter distances that reach it are the
        // band moved up by the pipeline, and at the top the truncation to the
        // frame below leaves one more frame of them whose shift lands on the
        // bound. These two bounds are the ones
        // `a_distance_outside_the_band_refuses_the_bring_up` admits, written
        // the same way, so the two tests carry one definition of admitted.
        let low = plan.offset_low() + PIPELINE_WORDS;
        let high = plan.offset_high() + PIPELINE_WORDS + plan.frame_slots() - 1;

        for offset in low..=high
        {
            let shift = shift_at(offset);

            assert_eq!
            (
                CarryShift::measure(plan, words - offset, words),
                Some(shift),
                "no shift for a distance of {offset} words"
            );

            for half in [Half::First, Half::Second]
            {
                let position = (plan.block_end(half) + offset) % words;

                assert!
                (
                    plan.block_is_clear(half, position, shift),
                    "offset {offset} on half {half:?}"
                );
            }
        }
    }

    #[test]
    fn the_arming_window_lies_inside_the_band()
    {
        // The window is a band of COUNTER distances and the bounds are a band
        // of shifts, so the pipeline comes off before the two are compared. A
        // pipeline large enough to walk the window under the floor would refuse
        // a start-up the polling loop calls a hit.
        let plan = plan();
        let aim = plan.arming_position();
        let window = plan.arming_window();

        assert!(aim - window - PIPELINE_WORDS >= plan.offset_low());
        assert!(aim + window - PIPELINE_WORDS <= plan.offset_high());
    }

    #[test]
    fn a_healthy_bring_up_is_accepted()
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
    }

    #[test]
    fn the_bring_up_starts_the_receiver_inside_the_arming_window()
    {
        // The mock places the transmitting counter from the receiving one, so
        // what the sequence controls is the moment it calls start, and that is
        // read back as the distance the run then carries.
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));

        let seen = interface.read();
        let plan = plan();
        let offset = plan.offset_of(seen.output_items, seen.stream.items);

        assert_eq!(offset, Some(plan.arming_position()));
    }

    #[test]
    fn a_read_pointer_that_never_reaches_the_window_refuses_the_bring_up()
    {
        let mut interface = MockInput::healthy();
        interface.phase_never_moves = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(PassthroughFault::Sequence(InputSequenceFault::OutputPhaseNeverReached))
        );
    }

    #[test]
    fn a_distance_outside_the_band_refuses_the_bring_up()
    {
        // The band is read on the shift, so the counter distances that break it
        // are the bounds moved up by the pipeline. At the top the truncation to
        // the frame below leaves one frame of distances whose shift still lands
        // on the bound, which is why the first refused one is a frame further
        // on. The last two distances the band admits are swept beside the first
        // two it refuses, so a bring-up that refused the whole neighbourhood
        // would not read here as a refusal of the band.
        let plan = plan();
        let slots = plan.frame_slots();
        let low = plan.offset_low() + PIPELINE_WORDS;
        let high = plan.offset_high() + PIPELINE_WORDS + slots - 1;

        for offset in [low, high]
        {
            let mut interface = MockInput::healthy();
            interface.offset = offset;

            assert_eq!
            (
                bring_up(&mut interface, &plan, &waits()),
                Ok(shift_at(offset)),
                "the bring-up refused a distance of {offset} words"
            );
        }

        for offset in [low - 1, high + 1]
        {
            let mut interface = MockInput::healthy();
            interface.offset = offset;

            assert_eq!
            (
                bring_up(&mut interface, &plan, &waits()),
                Err(PassthroughFault::Sequence(InputSequenceFault::OffsetOutOfBand)),
                "the bring-up admitted a distance of {offset} words"
            );
        }
    }

    #[test]
    fn a_stream_that_never_stops_refuses_the_bring_up()
    {
        let mut interface = MockInput::healthy();
        interface.image.stream.enabled = true;
        interface.stream_stays_enabled = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(PassthroughFault::Sequence(InputSequenceFault::StreamNeverStopped))
        );
    }

    #[test]
    fn a_sub_block_that_never_stops_refuses_the_bring_up()
    {
        let mut interface = MockInput::healthy();
        interface.image.block.enabled = true;
        interface.block_stays_enabled = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(PassthroughFault::Sequence(InputSequenceFault::BlockNeverStopped))
        );
    }

    #[test]
    fn a_receiver_whose_counter_never_moves_refuses_the_bring_up()
    {
        let mut interface = MockInput::healthy();
        interface.receiver_stalls = true;

        assert_eq!
        (
            bring_up(&mut interface, &plan(), &waits()),
            Err(PassthroughFault::Sequence(InputSequenceFault::ReceiverNeverAdvanced))
        );
    }

    #[test]
    fn a_synchronisation_input_left_at_reset_reads_as_the_plan()
    {
        // RM0433 section 51.4.4 numbers the first interface 0 and a reset
        // leaves SYNCIN there, so this field separates a write naming another
        // interface from one naming the first, and separates neither from a
        // write that never ran. A receiver with no frame clock is what the
        // sequence catches instead, at the step that watches its counter move.
        let mut interface = MockInput::healthy();
        interface.refuse_sync_input = true;

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
    }

    #[test]
    fn every_field_of_the_receiving_sub_block_is_compared()
    {
        let mutations: [Mutation; 23] =
        [
            (
                |seen| seen.block.mode_bits = 0b00,
                PassthroughFault::Block(InputBlockFault::ModeWrong),
            ),
            (
                |seen| seen.block.protocol_bits = 0b10,
                PassthroughFault::Block(InputBlockFault::ProtocolWrong),
            ),
            (
                |seen| seen.block.sync_bits = SYNC_IN_FIRST_INTERFACE,
                PassthroughFault::Block(InputBlockFault::SynchronisationWrong),
            ),
            (
                |seen| seen.block.sync_in_bits = 0b01,
                PassthroughFault::Block(InputBlockFault::SynchronisationSourceWrong),
            ),
            (
                |seen| seen.block.sync_out_bits = 0b01,
                PassthroughFault::Block(InputBlockFault::SynchronisationOutputWrong),
            ),
            (
                |seen| seen.block.data_size_bits = 0b110,
                PassthroughFault::Block(InputBlockFault::DataSizeWrong),
            ),
            (
                |seen| seen.block.lsb_first = true,
                PassthroughFault::Block(InputBlockFault::BitOrderWrong),
            ),
            (
                |seen| seen.block.changes_on_falling_edge = false,
                PassthroughFault::Block(InputBlockFault::ClockStrobingWrong),
            ),
            (
                |seen| seen.block.frame_length_field = 31,
                PassthroughFault::Block(InputBlockFault::FrameLengthWrong),
            ),
            (
                |seen| seen.block.frame_active_field = 15,
                PassthroughFault::Block(InputBlockFault::FrameActiveLengthWrong),
            ),
            (
                |seen| seen.block.frame_marks_channel = false,
                PassthroughFault::Block(InputBlockFault::FrameDefinitionWrong),
            ),
            (
                |seen| seen.block.frame_active_high = true,
                PassthroughFault::Block(InputBlockFault::FramePolarityWrong),
            ),
            (
                |seen| seen.block.frame_leads_first_bit = false,
                PassthroughFault::Block(InputBlockFault::FrameOffsetWrong),
            ),
            (
                |seen| seen.block.first_bit_offset_field = 1,
                PassthroughFault::Block(InputBlockFault::FirstBitOffsetWrong),
            ),
            (
                |seen| seen.block.slot_size_bits = 0b01,
                PassthroughFault::Block(InputBlockFault::SlotSizeWrong),
            ),
            (
                |seen| seen.block.slot_count_field = 3,
                PassthroughFault::Block(InputBlockFault::SlotCountWrong),
            ),
            (
                |seen| seen.block.slot_enable_bits = 0b01,
                PassthroughFault::Block(InputBlockFault::SlotsNotEnabled),
            ),
            (
                |seen| seen.block.fifo_threshold_bits = 0b100,
                PassthroughFault::Block(InputBlockFault::FifoThresholdWrong),
            ),
            (
                |seen| seen.block.mono = true,
                PassthroughFault::Block(InputBlockFault::MonoWrong),
            ),
            (
                |seen| seen.block.tristate = true,
                PassthroughFault::Block(InputBlockFault::TristateWrong),
            ),
            (
                |seen| seen.block.any_interrupt_enabled = true,
                PassthroughFault::Block(InputBlockFault::InterruptEnabled),
            ),
            (
                |seen| seen.block.transfer_enabled = false,
                PassthroughFault::Block(InputBlockFault::TransferDisabled),
            ),
            (
                |seen| seen.block.enabled = false,
                PassthroughFault::Block(InputBlockFault::NotEnabled),
            ),
        ];

        run_mutations(&mutations);
    }

    #[test]
    fn every_alarm_of_the_receiving_sub_block_is_read()
    {
        let mutations: [Mutation; 4] =
        [
            (
                |seen| seen.block.overrun = true,
                PassthroughFault::Block(InputBlockFault::Overrun),
            ),
            (
                |seen| seen.block.frame_mismatch = true,
                PassthroughFault::Block(InputBlockFault::FrameMismatch),
            ),
            (
                |seen| seen.block.codec_not_ready = true,
                PassthroughFault::Block(InputBlockFault::CodecNotReady),
            ),
            (
                |seen| seen.block.fifo_level_bits = FIFO_LEVEL_EMPTY,
                PassthroughFault::Block(InputBlockFault::FifoNeverFilled),
            ),
        ];

        run_mutations(&mutations);
    }

    #[test]
    fn every_field_of_the_receiving_stream_is_compared()
    {
        let mutations: [Mutation; 17] =
        [
            (
                |seen| seen.stream.request_bits = 87,
                PassthroughFault::Stream(InputStreamFault::RequestWrong),
            ),
            (
                |seen| seen.stream.sync_enabled = true,
                PassthroughFault::Stream(InputStreamFault::SynchronisationEnabled),
            ),
            (
                |seen| seen.stream.direction_bits = 0b01,
                PassthroughFault::Stream(InputStreamFault::DirectionWrong),
            ),
            (
                |seen| seen.stream.circular = false,
                PassthroughFault::Stream(InputStreamFault::NotCircular),
            ),
            (
                |seen| seen.stream.memory_increments = false,
                PassthroughFault::Stream(InputStreamFault::MemoryNotIncrementing),
            ),
            (
                |seen| seen.stream.peripheral_increments = true,
                PassthroughFault::Stream(InputStreamFault::PeripheralIncrementing),
            ),
            (
                |seen| seen.stream.memory_width_bits = 0b01,
                PassthroughFault::Stream(InputStreamFault::MemoryWidthWrong),
            ),
            (
                |seen| seen.stream.peripheral_width_bits = 0b01,
                PassthroughFault::Stream(InputStreamFault::PeripheralWidthWrong),
            ),
            (
                |seen| seen.stream.double_buffered = true,
                PassthroughFault::Stream(InputStreamFault::DoubleBuffered),
            ),
            (
                |seen| seen.stream.priority_bits = 0b00,
                PassthroughFault::Stream(InputStreamFault::PriorityWrong),
            ),
            (
                |seen| seen.stream.error_interrupt_enabled = true,
                PassthroughFault::Stream(InputStreamFault::ErrorInterruptEnabled),
            ),
            (
                |seen| seen.stream.event_interrupt_enabled = true,
                PassthroughFault::Stream(InputStreamFault::EventInterruptEnabled),
            ),
            (
                |seen| seen.stream.peripheral_address = 0x4001_5820,
                PassthroughFault::Stream(InputStreamFault::PeripheralAddressWrong),
            ),
            (
                |seen| seen.stream.memory_address = MASTER_BUFFER,
                PassthroughFault::Stream(InputStreamFault::MemoryAddressWrong),
            ),
            (
                |seen| seen.stream.transfer_error = true,
                PassthroughFault::Stream(InputStreamFault::TransferError),
            ),
            (
                |seen| seen.stream.direct_mode_error = true,
                PassthroughFault::Stream(InputStreamFault::DirectModeError),
            ),
            (
                |seen| seen.stream.fifo_error = true,
                PassthroughFault::Stream(InputStreamFault::FifoError),
            ),
        ];

        run_mutations(&mutations);
    }

    #[test]
    fn a_stopped_stream_and_a_counter_past_the_buffer_are_refused()
    {
        let mutations: [Mutation; 2] =
        [
            (
                |seen| seen.stream.items = TEST_WORDS + 1,
                PassthroughFault::Stream(InputStreamFault::CounterOutOfRange),
            ),
            (
                |seen| seen.stream.enabled = false,
                PassthroughFault::Stream(InputStreamFault::NotEnabled),
            ),
        ];

        run_mutations(&mutations);
    }

    #[test]
    fn a_counter_the_stream_was_armed_short_of_is_refused()
    {
        let mut seen = verified_readback();
        seen.stream.items = TEST_WORDS - 2;

        assert_eq!
        (
            plan().verify_armed(&seen),
            Err(PassthroughFault::Stream(InputStreamFault::ItemCountWrong))
        );
    }

    #[test]
    fn one_event_flag_names_the_half_it_belongs_to()
    {
        assert_eq!(next_block(&healthy_event(Half::First), &plan(), shift()), Ok(Half::First));
        assert_eq!(next_block(&healthy_event(Half::Second), &plan(), shift()), Ok(Half::Second));
    }

    #[test]
    fn an_entry_with_no_event_flag_is_refused()
    {
        let mut seen = healthy_event(Half::First);
        seen.half_transfer = false;
        seen.transfer_complete = false;

        assert_eq!
        (
            next_block(&seen, &plan(), shift()),
            Err(PassthroughFault::Event(EventFault::Spurious))
        );
    }

    #[test]
    fn an_entry_with_both_event_flags_is_refused()
    {
        let mut seen = healthy_event(Half::First);
        seen.transfer_complete = true;

        assert_eq!
        (
            next_block(&seen, &plan(), shift()),
            Err(PassthroughFault::Event(EventFault::BothHalvesPending))
        );
    }

    #[test]
    fn every_alarm_one_entry_carries_is_read()
    {
        let cases: [EventMutation; 7] =
        [
            (|seen| seen.overrun = true, EventFault::Overrun),
            (|seen| seen.frame_mismatch = true, EventFault::FrameMismatch),
            (|seen| seen.codec_not_ready = true, EventFault::CodecNotReady),
            (|seen| seen.block_enabled = false, EventFault::BlockStopped),
            (|seen| seen.transfer_error = true, EventFault::TransferError),
            (|seen| seen.direct_mode_error = true, EventFault::DirectModeError),
            (|seen| seen.fifo_error = true, EventFault::FifoError),
        ];

        for (break_it, expected) in cases
        {
            for half in [Half::First, Half::Second]
            {
                let mut seen = healthy_event(half);
                break_it(&mut seen);

                assert_eq!
                (
                    next_block(&seen, &plan(), shift()),
                    Err(PassthroughFault::Event(expected))
                );
            }
        }
    }

    #[test]
    fn a_stopped_stream_refuses_an_entry()
    {
        let mut seen = healthy_event(Half::First);
        seen.stream_enabled = false;

        assert_eq!
        (
            next_block(&seen, &plan(), shift()),
            Err(PassthroughFault::Event(EventFault::StreamStopped))
        );
    }

    #[test]
    fn a_counter_past_its_buffer_refuses_an_entry()
    {
        let cases: [EventMutation; 2] =
        [
            (
                |seen| seen.output_items = TEST_WORDS + 1,
                EventFault::OutputCounterOutOfRange,
            ),
            (
                |seen| seen.items = TEST_WORDS + 1,
                EventFault::CounterOutOfRange,
            ),
        ];

        for (break_it, expected) in cases
        {
            let mut seen = healthy_event(Half::First);
            break_it(&mut seen);

            assert_eq!
            (
                next_block(&seen, &plan(), shift()),
                Err(PassthroughFault::Event(expected))
            );
        }
    }

    #[test]
    fn an_entry_finding_the_read_pointer_in_the_block_is_refused()
    {
        let plan = plan();

        // Every position of the lap is swept on both halves, so what the check
        // accepts is read off the whole domain and not off one sample of it.
        for half in [Half::First, Half::Second]
        {
            for position in 0..plan.transfer_items()
            {
                let mut seen = healthy_event(half);
                seen.output_items = plan.transfer_items() - position;

                let answered = next_block(&seen, &plan, shift());

                if plan.block_is_clear(half, position, shift())
                {
                    assert_eq!(answered, Ok(half));
                }
                else
                {
                    assert_eq!
                    (
                        answered,
                        Err(PassthroughFault::Event(EventFault::ReadPointerInBlock))
                    );
                }
            }
        }
    }

    /// Fills the receiving buffer with a word that names its own position.
    ///
    /// Every word of a lap is distinct, so a carry that moves a word to the
    /// wrong index, swaps two of them, or reads one half and writes the other
    /// shows up as a value that does not match its slot rather than as silence.
    fn seed_source(interface: &mut MockInput)
    {
        for index in 0..TEST_WORDS
        {
            if let Some(slot) = interface.source.get_mut(index as usize)
            {
                *slot = 0xC0DE_0000 | index;
            }
        }
    }

    /// Writes into `want` what a carry of `half` through a TRANSPARENT section
    /// leaves in an output buffer.
    ///
    /// Both halves of the expectation are closed forms, and neither calls the
    /// code under test. The destination comes from the distance the mock is
    /// armed at, less the pipeline the data crosses, rather than from
    /// `CarryShift`. The value comes from `narrowed`, since a transparent
    /// section changes nothing but the single precision the sample crosses.
    ///
    /// The tests that use this measure WHERE a word lands and how many buffers
    /// it reaches. What the real section does to a word is measured against
    /// `cascade_magnitude` and against a straight walk of the same samples,
    /// neither of which knows anything about this loop.
    fn carry_into_image(half: Half, want: &mut [u32; TEST_WORDS as usize])
    {
        let plan = plan();

        for step in 0..plan.block_words()
        {
            let index = plan.block_start(half) + step;
            let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;

            if let Some(slot) = want.get_mut(at as usize)
            {
                *slot = narrowed(0xC0DE_0000 | index);
            }
        }
    }

    /// Returns what the two output buffers hold once the words of `half` have
    /// been carried onto an empty pair of them through a transparent section.
    fn carried_image(half: Half) -> [u32; TEST_WORDS as usize]
    {
        let mut want = [0_u32; TEST_WORDS as usize];

        carry_into_image(half, &mut want);

        want
    }

    #[test]
    fn a_carry_moves_every_word_of_its_half_to_the_position_it_was_sent_from()
    {
        for half in [Half::First, Half::Second]
        {
            let mut interface = MockInput::healthy();
            seed_source(&mut interface);

            carry_block(&mut interface, &plan(), half, shift(), &mut transparent_stage());

            let count = plan().block_words();

            assert_eq!
            (
                interface.written, count * OUTPUT_BUFFERS,
                "a carry of {half:?} made {} writes, a block is {count} words \
                 into each of {OUTPUT_BUFFERS} buffers",
                interface.written
            );

            let want = carried_image(half);

            for index in 0..TEST_WORDS as usize
            {
                assert_eq!
                (
                    interface.master.get(index).copied(),
                    want.get(index).copied(),
                    "master word {index} after a carry of {half:?}"
                );
                assert_eq!
                (
                    interface.slave.get(index).copied(),
                    want.get(index).copied(),
                    "slave word {index} after a carry of {half:?}"
                );
            }
        }
    }

    #[test]
    fn a_carry_writes_nothing_outside_the_half_it_was_given()
    {
        // The two halves together cover the lap, so carrying one and finding a
        // word standing where the other one lands is what a swapped block start
        // looks like. The destination wraps the end of the buffer, so the
        // stretch that must stay empty is walked rather than sliced.
        let mut interface = MockInput::healthy();
        seed_source(&mut interface);

        carry_block(&mut interface, &plan(), Half::Second, shift(), &mut transparent_stage());

        let plan = plan();
        let first = (plan.block_start(Half::First) + plan.arming_position() - PIPELINE_WORDS)
            % TEST_WORDS;

        for step in 0..plan.block_words()
        {
            let index = (first + step) % TEST_WORDS;

            assert_eq!
            (
                interface.master.get(index as usize).copied(),
                Some(0),
                "word {index} belongs to the other block and was written"
            );
        }
    }

    #[test]
    fn the_two_halves_of_a_lap_carry_the_whole_buffer_between_them()
    {
        let mut interface = MockInput::healthy();
        seed_source(&mut interface);

        // One stage crosses both halves, so this walks the block boundary as
        // well as the lap: the history the second carry opens on is the history
        // the first left.
        let mut running = transparent_stage();

        carry_block(&mut interface, &plan(), Half::First, shift(), &mut running);
        carry_block(&mut interface, &plan(), Half::Second, shift(), &mut running);

        assert_eq!
        (
            interface.written, TEST_WORDS * OUTPUT_BUFFERS,
            "the two halves left {} writes, a lap is {TEST_WORDS} words into \
             each of {OUTPUT_BUFFERS} buffers",
            interface.written
        );

        let mut want = [0_u32; TEST_WORDS as usize];

        carry_into_image(Half::First, &mut want);
        carry_into_image(Half::Second, &mut want);

        for index in 0..TEST_WORDS as usize
        {
            assert_eq!
            (
                interface.master.get(index).copied(),
                want.get(index).copied(),
                "word {index} did not come through a whole lap"
            );
        }

        assert_eq!(interface.master, interface.slave);
    }

    /// Samples one channel holds in one lap of the buffer.
    const LAP_SAMPLES: u32 = TEST_WORDS / CHANNEL_STRIDE;

    /// Amplitude the response sweep drives, as a sample.
    ///
    /// Under half of full scale, so the pass band of the section leaves room
    /// for the overshoot and nothing in the sweep meets the saturation the
    /// tests below it measure on purpose.
    const SWEEP_AMPLITUDE: f64 = 1.0e9;

    /// Laps a sweep runs before it reads the one it measures.
    ///
    /// The section places its poles at radius 0.970, so its impulse response
    /// falls by that factor a sample and what is left of a transient after `n`
    /// samples is 0.970 to the power of `n`. Six laps are 2646 samples of one
    /// channel, which is e to the minus eighty, so what the seventh lap carries
    /// is the steady state and not the start-up.
    const SWEEP_WARM_LAPS: u32 = 6;

    /// Largest relative departure the ARITHMETIC may put between a measured
    /// gain and the designed one, once the rounding of a sample is taken off.
    ///
    /// MEASURED over the whole sweep below rather than read at one point, on
    /// both slots, as the departure less the rounding floor beside it. The
    /// largest of the set is 2.33e-6, at 200 Hz, and the residue falls under
    /// the floor entirely above 3000 Hz. The bound is a little over twice the
    /// worst of them, which is what covers the frequencies between the ones the
    /// sweep stands on.
    ///
    /// What it covers is the single precision difference equation against a
    /// double precision evaluation of the same coefficients.
    const RESPONSE_TOLERANCE: f64 = 5.0e-6;

    /// Words of amplitude the rounding of a sample can move one measurement by.
    ///
    /// A sample leaves each end of the chain rounded to a whole word, so it
    /// stands at most half a word from the value it carries. One bin of the
    /// transform sums those over a lap and scales by two over the lap, so the
    /// amplitude it reads stands at most `2 * (0.5 * N) / N` from the true one,
    /// which is one word whatever the length. It enters the gain twice, once
    /// through the measurement of the drive and once through the measurement of
    /// what came back, and it is what a gain of five parts in a hundred million
    /// runs into: at 21900 Hz the section leaves 52 words of amplitude out of
    /// the billion it was driven with.
    const QUANTISATION_WORDS: f64 = 1.0;

    /// Returns the word a channel of a sine holds at `index` of lap `lap`.
    ///
    /// The sample number of a word is the lap it belongs to times the samples a
    /// lap holds, plus the position of its frame, so the two channels carry the
    /// same sine and a lap joins the one before it. `cycles` periods fill one
    /// lap exactly, which is what puts the whole of the signal in one bin of
    /// the measurement below.
    fn sine_word(lap: u32, index: u32, cycles: u32, amplitude: f64) -> u32
    {
        let sample = lap * LAP_SAMPLES + index / CHANNEL_STRIDE;
        let angle = TAU * f64::from(cycles) * f64::from(sample) / f64::from(LAP_SAMPLES);

        (round(amplitude * sin(angle)) as i32).cast_unsigned()
    }

    /// Carries `laps` laps of blocks, filling each source half from `word`
    /// before it is carried, and leaves in `out` what the master buffer holds
    /// at the destination of every source index of the LAST lap.
    ///
    /// It drives the source rather than closing the link on the output, which
    /// is the path the product runs: a source this board does not write, one
    /// filter, one set of output buffers. A closed link would cross the section
    /// once a lap and take any signal to silence.
    fn run_laps<F>
    (
        stage: &mut FilterStage,
        laps: u32,
        word: F,
        out: &mut [u32; TEST_WORDS as usize]
    )
    where
        F: Fn(u32, u32) -> u32,
    {
        let plan = plan();
        let mut interface = MockInput::healthy();

        for lap in 0..laps
        {
            for half in [Half::First, Half::Second]
            {
                let first = plan.block_start(half);

                for step in 0..plan.block_words()
                {
                    let index = first + step;

                    if let Some(slot) = interface.source.get_mut(index as usize)
                    {
                        *slot = word(lap, index);
                    }
                }

                carry_block(&mut interface, &plan, half, shift(), stage);

                for step in 0..plan.block_words()
                {
                    let index = first + step;
                    let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;

                    if let (Some(slot), Some(carried)) =
                        (out.get_mut(index as usize), interface.master.get(at as usize))
                    {
                        *slot = *carried;
                    }
                }
            }
        }
    }

    /// Returns the amplitude one slot of `buffer` carries at `cycles` periods
    /// per lap.
    ///
    /// One bin of a discrete Fourier transform over a whole lap of one channel.
    /// A sine of `cycles` periods a lap fills that bin and nothing else, so this
    /// reads its amplitude with no window and no leakage.
    fn amplitude_at(buffer: &[u32; TEST_WORDS as usize], slot: u32, cycles: u32) -> f64
    {
        let mut real = 0.0_f64;
        let mut imaginary = 0.0_f64;

        for sample in 0..LAP_SAMPLES
        {
            let index = sample * CHANNEL_STRIDE + slot;
            let word = buffer.get(index as usize).copied().unwrap_or(0);
            let value = f64::from(word.cast_signed());
            let angle = -TAU * f64::from(cycles) * f64::from(sample) / f64::from(LAP_SAMPLES);

            real += value * cos(angle);
            imaginary += value * sin(angle);
        }

        2.0 * sqrt(real * real + imaginary * imaginary) / f64::from(LAP_SAMPLES)
    }

    /// Returns the magnitude the design gives at `cycles` periods per lap.
    fn designed_magnitude(cycles: u32) -> f64
    {
        let frequency_hz =
            f64::from(cycles) * f64::from(SAMPLE_RATE_HZ) / f64::from(LAP_SAMPLES);

        cascade_magnitude(&[section()], frequency_hz as f32, SAMPLE_RATE_HZ).unwrap_or(0.0)
    }

    /// Periods per lap the sweep walks.
    ///
    /// One lap holds 441 samples of a channel at 44100 Hz, so a period count is
    /// a frequency in hundreds of hertz and the set below runs 100 Hz to 21900
    /// Hz. It crosses the 300 Hz corner, stands either side of it, and reaches
    /// the last bin under Nyquist, so the bound it produces is a bound over the
    /// band and not a reading at one point.
    const SWEEP_CYCLES: [u32; 16] =
        [1, 2, 3, 4, 5, 7, 10, 14, 20, 30, 45, 70, 110, 160, 200, 219];

    #[test]
    fn the_filtered_response_matches_the_designed_magnitude()
    {
        // The whole point of the stage, measured against an evaluation of the
        // same coefficients that knows nothing of this loop. The gain is a
        // ratio of two measurements taken the same way, so the rounding of a
        // sample to a whole number at the source cancels out of it.
        let mut points = 0_u32;

        for cycles in SWEEP_CYCLES
        {
            let mut carried = [0_u32; TEST_WORDS as usize];
            let mut driven = [0_u32; TEST_WORDS as usize];

            run_laps
            (
                &mut stage(),
                SWEEP_WARM_LAPS + 1,
                |lap, index| sine_word(lap, index, cycles, SWEEP_AMPLITUDE),
                &mut carried
            );

            for index in 0..TEST_WORDS
            {
                if let Some(slot) = driven.get_mut(index as usize)
                {
                    *slot = sine_word(SWEEP_WARM_LAPS, index, cycles, SWEEP_AMPLITUDE);
                }
            }

            let designed = designed_magnitude(cycles);

            for slot in 0..CHANNEL_STRIDE
            {
                let drive = amplitude_at(&driven, slot, cycles);
                let measured = amplitude_at(&carried, slot, cycles) / drive;
                let departure = (measured - designed).abs() / designed;

                // The floor is built from the amplitude the DESIGN calls for,
                // never from the one that came back. Built from what came back,
                // a stage that answers nothing at all in this bin divides by
                // nothing and buys itself a tolerance without bound, which is a
                // test that passes on a filter with its feedback cut.
                let floor = QUANTISATION_WORDS
                    * (1.0 / (designed * drive) + 1.0 / drive);

                points = points.saturating_add(1);

                assert!
                (
                    departure <= RESPONSE_TOLERANCE + floor,
                    "slot {slot} at {cycles}00 Hz read a gain of {measured} \
                     against the designed {designed}, a departure of {departure}"
                );
            }
        }

        assert_eq!
        (
            points,
            SWEEP_CYCLES.len() as u32 * CHANNEL_STRIDE,
            "a point of the sweep went unmeasured"
        );
    }

    #[test]
    fn a_stage_at_rest_answers_a_silent_block_with_silence()
    {
        // The safety property of the state. A history that was never cleared,
        // or one a start-up never wrote, shows as a word that is not zero on a
        // source that is.
        let mut carried = [0xDEAD_BEEF_u32; TEST_WORDS as usize];

        run_laps(&mut stage(), 2, |_, _| 0, &mut carried);

        for index in 0..TEST_WORDS as usize
        {
            assert_eq!
            (
                carried.get(index).copied(),
                Some(0),
                "word {index} of a silent block came out loud"
            );
        }
    }

    #[test]
    fn a_history_a_loud_block_left_shows_on_the_silent_block_that_follows()
    {
        // The other side of the property above, and what tells a state that
        // PERSISTS from one that is thrown away at every block. A section fed
        // silence after a signal answers the tail of its own impulse response,
        // so the first silent block is not silent, and the tail dies.
        let mut stage = stage();
        let mut carried = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut stage,
            2,
            |lap, index| sine_word(lap, index, 3, SWEEP_AMPLITUDE),
            &mut carried
        );

        let mut tail = [0_u32; TEST_WORDS as usize];

        run_laps(&mut stage, 1, |_, _| 0, &mut tail);

        let loudest = tail
            .iter()
            .map(|word| word.cast_signed().unsigned_abs())
            .max()
            .unwrap_or(0);

        assert!(loudest > 0, "a stage carried no history across the block it left");

        let mut quiet = [0_u32; TEST_WORDS as usize];

        run_laps(&mut stage, 20, |_, _| 0, &mut quiet);

        let left = quiet
            .iter()
            .map(|word| word.cast_signed().unsigned_abs())
            .max()
            .unwrap_or(0);

        assert!(left < loudest, "the tail of {loudest} did not decay, it stands at {left}");
    }

    #[test]
    fn each_slot_carries_a_history_of_its_own()
    {
        // The two slots of a frame are the two channels of a converter. One
        // history over both would feed each channel the samples of the other,
        // so a channel driven with silence is what reads that: it comes back
        // silent or it does not.
        let mut carried = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut stage(),
            SWEEP_WARM_LAPS + 1,
            |lap, index| if index.is_multiple_of(CHANNEL_STRIDE)
            {
                sine_word(lap, index, 3, SWEEP_AMPLITUDE)
            }
            else
            {
                0
            },
            &mut carried
        );

        for sample in 0..LAP_SAMPLES
        {
            let index = sample * CHANNEL_STRIDE + 1;

            assert_eq!
            (
                carried.get(index as usize).copied(),
                Some(0),
                "the silent slot carried word {index}, so the other channel \
                 reached its history"
            );
        }

        assert!
        (
            amplitude_at(&carried, 0, 3) > 0.0,
            "the driven slot carried nothing, so the test above proves nothing"
        );
    }

    #[test]
    fn a_run_of_blocks_answers_what_one_unbroken_run_answers()
    {
        // The lesson of the lot that built this loop: a test that reads ONE
        // block cannot see a state that is dropped at a block boundary, or a
        // history that crosses between the two channels at the seam. So this
        // runs the block structure over several blocks and compares it against
        // a straight walk of the same samples through the same section, which
        // knows nothing of halves, of positions or of shifts. The two blocks of
        // a lap start on different slots of a frame, which is what makes the
        // seam a real one to cross.
        const LAPS: u32 = 4;

        let mut carried = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut stage(),
            LAPS,
            |lap, index| sine_word(lap, index, 7, SWEEP_AMPLITUDE),
            &mut carried
        );

        let mut straight = [BiquadState::AT_REST; CHANNEL_STRIDE as usize];
        let mut want = [0_u32; TEST_WORDS as usize];

        for lap in 0..LAPS
        {
            for index in 0..TEST_WORDS
            {
                let slot = (index % CHANNEL_STRIDE) as usize;
                let driven = sine_word(lap, index, 7, SWEEP_AMPLITUDE);

                if let Some(state) = straight.get_mut(slot)
                {
                    let answered = carry_word(section(), state, driven);

                    if let Some(place) = want.get_mut(index as usize)
                    {
                        *place = answered;
                    }
                }
            }
        }

        for index in 0..TEST_WORDS as usize
        {
            assert_eq!
            (
                carried.get(index).copied(),
                want.get(index).copied(),
                "word {index} of the last lap does not continue the run"
            );
        }
    }

    /// Returns the last lap one channel answers when the recurrence runs in
    /// DOUBLE precision with its feedback unbounded, a sample bounded only
    /// where it becomes a word.
    ///
    /// This is the reference for the property the stage documents: the history
    /// carries the value the section computed, and the bound stands at the
    /// conversion alone. A section that fed back the bounded value is a
    /// different filter, and it differs exactly where the signal is loudest.
    ///
    /// Double precision rather than single, so this is an EVALUATION of the
    /// recurrence and not a transcription of the one under test. A difference
    /// of association or of rounding order then shows as a departure of a few
    /// parts in a million, which stands orders below what a bounded feedback
    /// moves.
    fn unbounded_channel<F>(section: Biquad, laps: u32, slot: u32, word: F)
        -> [u32; LAP_SAMPLES as usize]
    where
        F: Fn(u32, u32) -> u32,
    {
        let b0 = f64::from(section.b0());
        let b1 = f64::from(section.b1());
        let b2 = f64::from(section.b2());
        let a1 = f64::from(section.a1());
        let a2 = f64::from(section.a2());

        let mut previous_input = 0.0_f64;
        let mut older_input = 0.0_f64;
        let mut previous_output = 0.0_f64;
        let mut older_output = 0.0_f64;
        let mut out = [0_u32; LAP_SAMPLES as usize];

        for lap in 0..laps
        {
            for sample in 0..LAP_SAMPLES
            {
                let index = sample * CHANNEL_STRIDE + slot;
                let driven = f64::from(word(lap, index).cast_signed());

                let answered = b0 * driven + b1 * previous_input + b2 * older_input
                    - a1 * previous_output
                    - a2 * older_output;

                older_input = previous_input;
                previous_input = driven;
                older_output = previous_output;
                previous_output = answered;

                if let Some(place) = out.get_mut(sample as usize)
                {
                    *place = (answered as i32).cast_unsigned();
                }
            }
        }

        out
    }

    /// Words a carried sample may stand from the double precision evaluation of
    /// the same recurrence.
    ///
    /// MEASURED on the drive below, over both slots and over the whole lap
    /// rather than at one sample: the largest gap a healthy stage leaves is
    /// 44005 words, on a format that runs to 2147483647, which is two parts in
    /// a hundred thousand. What it covers is single precision against double
    /// through a recursion, which carries its own rounding forward.
    ///
    /// The same drive through a section whose feedback is bounded to the format
    /// moves a sample by 8066527 words, 183 times as far. The bound below
    /// stands between the two: four and a half times the worst a healthy stage
    /// leaves, and forty times under what a bounded feedback produces.
    const FEEDBACK_TOLERANCE_WORDS: i64 = 200_000;

    #[test]
    fn a_drive_past_the_format_feeds_back_what_the_section_computed()
    {
        // The stage documents that the bound stands at the conversion and
        // nowhere else, and that the history carries the computed value. That
        // sentence is on the way to the drivers: a bound inside the feedback of
        // a low-pass is a filter whose shape changes when the signal is
        // loudest, which is when the compression driver is most exposed. So it
        // carries its test rather than standing on its own.
        //
        // The drive is a step from the bottom of the format to the top, which
        // takes the section past the format on its overshoot. The two runs part
        // company there and the parting shows in the ringing that follows,
        // where the output is back inside the format and readable.
        const STEP_LAP: u32 = 2;
        const LAPS: u32 = STEP_LAP + 1;

        let drive = |lap: u32, _index: u32| if lap < STEP_LAP
        {
            i32::MIN.cast_unsigned()
        }
        else
        {
            i32::MAX.cast_unsigned()
        };

        let mut carried = [0_u32; TEST_WORDS as usize];

        run_laps(&mut stage(), LAPS, drive, &mut carried);

        for slot in 0..CHANNEL_STRIDE
        {
            let want = unbounded_channel(section(), LAPS, slot, drive);

            for sample in 0..LAP_SAMPLES
            {
                let index = sample * CHANNEL_STRIDE + slot;
                let answered = carried.get(index as usize).copied().unwrap_or(0).cast_signed();
                let reference = want.get(sample as usize).copied().unwrap_or(0).cast_signed();
                let gap = i64::from(answered) - i64::from(reference);

                assert!
                (
                    gap.abs() <= FEEDBACK_TOLERANCE_WORDS,
                    "slot {slot} sample {sample} came out at {answered} against \
                     {reference}, a gap of {gap} words"
                );
            }
        }
    }

    #[test]
    fn an_overshoot_saturates_instead_of_coming_back_round()
    {
        // A section answers a step above the step. Driven at full scale it
        // therefore leaves the format, and what it must NOT do there is come
        // back round to the opposite sign, which is a swing of two full scales
        // into a way with no analog filter in front of it.
        //
        // The drive is a step from the bottom of the format to the top, held
        // long enough for the section to settle, which is the input that
        // reaches the largest overshoot this section can produce.
        const STEP_LAP: u32 = 2;

        let mut carried = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut stage(),
            STEP_LAP + 1,
            |lap, _| if lap < STEP_LAP
            {
                i32::MIN.cast_unsigned()
            }
            else
            {
                i32::MAX.cast_unsigned()
            },
            &mut carried
        );

        let first_bound = carried
            .iter()
            .position(|word| word.cast_signed() == i32::MAX)
            .unwrap_or(TEST_WORDS as usize);

        assert!
        (
            first_bound < TEST_WORDS as usize,
            "the drive never took the section past the format, so this measures \
             nothing about the bound"
        );

        // From the first word the bound holds, the section stands at or above
        // full scale for the whole of its overshoot. A conversion that came
        // back round would answer that stretch with the opposite sign, so this
        // is where a wrap shows and nowhere else: before it the section is
        // still climbing out of the bottom of the format, legitimately below
        // zero.
        for index in first_bound..TEST_WORDS as usize
        {
            let word = carried.get(index).copied().unwrap_or(0).cast_signed();

            assert!
            (
                word > 0,
                "word {index} came out at {word} after the bound first held at \
                 word {first_bound}, which is the sign a conversion that \
                 wrapped produces"
            );
        }
    }

    #[test]
    fn serving_an_entry_takes_the_flag_down_before_it_carries_the_block()
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
        assert_eq!(serve(&mut interface, &plan(), shift(), &mut stage()), Ok(()));

        let (cleared, cleared_at) = interface.cleared.unwrap_or((Half::Second, 0));
        let (carried, carried_at) = interface.carried.unwrap_or((Half::Second, 0));

        assert_eq!(cleared, Half::First);
        assert_eq!(carried, Half::First);
        assert!(cleared_at < carried_at);
    }

    #[test]
    fn serving_an_event_carries_the_half_that_event_names()
    {
        // The seam between the decision and the carry, walked on BOTH events.
        // The three carry tests above call `carry_block` with a half written
        // out, so what they hold is the loop. What decides that half is
        // `next_block`, and a `serve` that passes the carry a half of its own
        // writes the block the transmitting streams are reading: at the aim of
        // 220 words their read pointer stands inside the first half when the
        // event names the second.
        for half in [Half::First, Half::Second]
        {
            let mut interface = MockInput::healthy();
            seed_source(&mut interface);
            interface.next_event = half;

            assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
            assert_eq!
            (
                serve(&mut interface, &plan(), shift(), &mut transparent_stage()),
                Ok(())
            );

            assert_eq!
            (
                interface.cleared.map(|(which, _)| which),
                Some(half),
                "the {half:?} event took another flag down"
            );

            let want = carried_image(half);

            for index in 0..TEST_WORDS as usize
            {
                assert_eq!
                (
                    interface.master.get(index).copied(),
                    want.get(index).copied(),
                    "master word {index} after serving the {half:?} event"
                );
                assert_eq!
                (
                    interface.slave.get(index).copied(),
                    want.get(index).copied(),
                    "slave word {index} after serving the {half:?} event"
                );
            }
        }
    }

    #[test]
    fn a_refused_entry_carries_nothing()
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
        interface.image.block.overrun = true;

        assert_eq!
        (
            serve(&mut interface, &plan(), shift(), &mut stage()),
            Err(PassthroughFault::Event(EventFault::Overrun))
        );
        assert_eq!(interface.carried, None);
        assert_eq!(interface.cleared, None);
    }

    #[test]
    fn arming_enables_the_interrupt_after_a_whole_lap_of_the_buffer()
    {
        // Swept over every phase the window can open at, not run at one. The
        // seed wait counts reloads of the receiving counter, and a count of
        // reloads taken from wherever the first poll happened to find that
        // counter is a FRACTION of a lap at every phase but one. Opening at a
        // single phase is what hides that, so this walks the whole lap.
        for phase in 0..TEST_WORDS
        {
            let mut interface = MockInput::healthy_at_phase(phase);

            assert_eq!
            (
                bring_up(&mut interface, &plan(), &waits()),
                Ok(shift()),
                "bring-up refused at phase {phase}"
            );

            let opened = interface.polls.get();

            assert!
            (
                arm(&mut interface, permit(), &plan(), &waits()).is_ok(),
                "arming refused at phase {phase}"
            );

            let enabled = interface.events_enabled_at.unwrap_or(0);

            assert!(interface.events_enabled, "nothing enabled at phase {phase}");
            // The mock walks one word a read, so the reads the wait spends are
            // the words the buffer moved, and a whole lap is what the seed
            // needs.
            assert!
            (
                enabled - opened >= TEST_WORDS,
                "phase {phase} armed after {} words, one lap is {TEST_WORDS}",
                enabled - opened
            );
        }
    }

    #[test]
    fn arming_takes_the_stream_flags_down_before_it_enables_the_interrupt()
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
        assert!(arm(&mut interface, permit(), &plan(), &waits()).is_ok());

        let cleared = interface.flags_cleared_at.unwrap_or(u32::MAX);
        let enabled = interface.events_enabled_at.unwrap_or(0);

        assert!(cleared < enabled);
    }

    #[test]
    fn arming_refuses_a_path_that_stopped_carrying_the_plan()
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
        interface.image.block.overrun = true;

        assert_eq!
        (
            arm(&mut interface, permit(), &plan(), &waits()),
            Err(PassthroughFault::Block(InputBlockFault::Overrun))
        );
        assert!(!interface.events_enabled);
    }

    #[test]
    fn arming_refuses_a_counter_that_never_lapped_and_enables_nothing()
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
        interface.receiver_stalls = true;

        let waits = InputWaits { seed_polls: 4, ..waits() };

        assert_eq!
        (
            arm(&mut interface, permit(), &plan(), &waits),
            Err(PassthroughFault::Sequence(InputSequenceFault::SeedNeverLapped))
        );
        assert!(!interface.events_enabled);
    }

    #[test]
    fn every_place_and_cause_of_the_encoding_fits_one_word()
    {
        let faults =
        [
            PassthroughFault::Sequence(InputSequenceFault::StreamNeverStopped),
            PassthroughFault::PlanRejected(InputPlanError::BufferEmpty),
            PassthroughFault::Block(InputBlockFault::ModeWrong),
            PassthroughFault::Stream(InputStreamFault::RequestWrong),
            PassthroughFault::Event(EventFault::Spurious),
        ];

        for (index, fault) in faults.into_iter().enumerate()
        {
            let place = u32::try_from(index).unwrap_or(u32::MAX);

            assert_eq!(fault.code(), place << 8 | 0x01);
            assert!(fault.code() <= PASSTHROUGH_CODE_CEILING);
        }
    }

    #[test]
    fn no_two_places_of_the_encoding_share_a_code()
    {
        let faults =
        [
            PassthroughFault::Sequence(InputSequenceFault::StreamNeverStopped),
            PassthroughFault::PlanRejected(InputPlanError::BufferEmpty),
            PassthroughFault::Block(InputBlockFault::ModeWrong),
            PassthroughFault::Stream(InputStreamFault::RequestWrong),
            PassthroughFault::Event(EventFault::Spurious),
        ];

        for (index, one) in faults.into_iter().enumerate()
        {
            for other in faults.into_iter().skip(index + 1)
            {
                assert_ne!(one.code(), other.code());
            }
        }
    }

    /// Fills both output buffers with the tone the loop is seeded with.
    ///
    /// One word per slot, the same sample in the two slots of a frame, which is
    /// what the firmware writes into the buffers the transmitting streams
    /// replay.
    fn seed_tone(interface: &mut MockInput)
    {
        for (frame, sample) in TONE_TABLE.iter().enumerate()
        {
            let word = sample.cast_unsigned();

            for slot in 0..2
            {
                let at = frame * 2 + slot;

                if let Some(place) = interface.master.get_mut(at)
                {
                    *place = word;
                }

                if let Some(place) = interface.slave.get_mut(at)
                {
                    *place = word;
                }
            }
        }
    }

    /// Runs the block structure over a closed link for `laps` laps.
    ///
    /// The link is the one the bench wires: the data line carries what the
    /// transmitting stream reads, so this walks the two positions one word a
    /// slot and puts a word the transmitting stream read into the receiving
    /// buffer at `p`. A transfer event is raised at the slot that fills a half,
    /// which is where the part raises one, and every one of them is served.
    ///
    /// The word it takes is the one read at `p + offset` less
    /// `PIPELINE_WORDS`, and that is the whole of what makes this a model of
    /// the wire rather than of the two counters. The counters stand `offset`
    /// apart, while the transmitting FIFO, the two shift registers and the
    /// receiving FIFO hold the data that many words behind them. A run that
    /// read at `p + offset` would close the loop on a shift carrying no
    /// pipeline correction, which is the defect this is here to catch.
    ///
    /// The distance is even at every offset a caller may pass, because both
    /// buffers take their frame boundary from one `FS` and the receiver starts
    /// on one: index 0 of either buffer holds slot 0 of a frame, so the two
    /// positions carry the same slot parity at every instant. `PIPELINE_WORDS`
    /// is a whole number of frames, which an assertion beside it holds, so it
    /// moves no word out of its slot either.
    fn run_loop
    (
        interface: &mut MockInput,
        laps: u32,
        shift: CarryShift,
        stage: &mut FilterStage
    ) -> Result<(), PassthroughFault>
    {
        let plan = plan();
        let words = plan.transfer_items();
        let block = plan.block_words();

        for slot in 0..laps * words
        {
            let at = slot % words;
            let read = ((at + interface.offset + words - PIPELINE_WORDS) % words) as usize;
            let word = interface.master.get(read).copied().unwrap_or(0);

            if let Some(place) = interface.source.get_mut(at as usize)
            {
                *place = word;
            }

            let filled = match at + 1
            {
                position if position == block => Half::First,
                position if position == words => Half::Second,
                _ => continue,
            };

            interface.next_event = filled;
            interface.written = 0;
            serve(interface, &plan, shift, stage)?;
        }

        Ok(())
    }

    /// Returns the largest step between two neighbouring samples of one channel
    /// of `buffer`, taken round the lap.
    ///
    /// One channel, so the walk steps a frame at a time: the two slots of a
    /// frame hold the same sample here and a step between them is not a step of
    /// the signal. Round the lap, because the last frame of the buffer is
    /// followed by the first and the streams replay it that way for ever.
    fn largest_step(buffer: &[u32; TEST_WORDS as usize], slot: usize) -> i64
    {
        let sample = |frame: usize| -> i64
        {
            i64::from(buffer.get(frame % TONE_SAMPLES * 2 + slot).copied()
                .unwrap_or(0).cast_signed())
        };

        (0..TONE_SAMPLES)
            .map(|frame| (sample(frame + 1) - sample(frame)).abs())
            .max()
            .unwrap_or(0)
    }

    /// The counter distances a start-up can leave that the loop tests are run
    /// at.
    ///
    /// The band the bring-up admits, walked at both ends, at the aim, and
    /// between them. A carry that only lands right at the aim is a carry that
    /// works on the phase the poll happened to catch. These are distances
    /// between the two COUNTERS, so the band of shifts stands `PIPELINE_WORDS`
    /// below them and the ends of it are 118 and 338. Each is even, which is
    /// what `run_loop` takes.
    const LOOP_OFFSETS: [u32; 6] = [118, 160, 200, 220, 282, 338];

    #[test]
    fn the_tone_comes_back_to_the_position_it_left_lap_after_lap()
    {
        // The defect this holds is the one no single block shows. Every other
        // carry test here writes one block onto a buffer nothing else feeds, so
        // a carry that puts the block a fixed distance from where it belongs
        // reads as correct in all of them. Closed on itself, that distance is
        // added at every lap: the content walks round the buffer, the tone
        // comes out at another pitch, and each block boundary carries the step
        // between two halves the carry reached at different laps.
        for offset in LOOP_OFFSETS
        {
            let mut interface = MockInput::healthy();

            interface.offset = offset;

            let shift = shift_at(offset);

            assert_eq!
            (
                bring_up(&mut interface, &plan(), &waits()),
                Ok(shift),
                "the bring-up refused a distance of {offset} words"
            );

            seed_tone(&mut interface);

            // The seed narrowed once. Every sample crosses single precision on
            // its way through the stage, so what a lap returns is the rounded
            // word rather than the seeded one, and it returns the same one at
            // every lap after that.
            let seed = narrowed_buffer(&interface.master);

            assert_eq!(run_loop(&mut interface, 12, shift, &mut transparent_stage()), Ok(()));

            for index in 0..TEST_WORDS as usize
            {
                assert_eq!
                (
                    interface.master.get(index).copied(),
                    seed.get(index).copied(),
                    "master word {index} moved over twelve laps at offset {offset}"
                );
                assert_eq!
                (
                    interface.slave.get(index).copied(),
                    seed.get(index).copied(),
                    "slave word {index} moved over twelve laps at offset {offset}"
                );
            }
        }
    }

    /// Fall a closed link of twelve laps must leave, as a divisor of the seed.
    ///
    /// The measurement beside the test that reads it stands at a forty-sixth,
    /// so this is half of what was measured.
    const DECAY_FLOOR: u32 = 20;

    /// Returns the largest magnitude any word of `buffer` carries.
    fn largest_magnitude(buffer: &[u32; TEST_WORDS as usize]) -> u32
    {
        buffer
            .iter()
            .map(|word| word.cast_signed().unsigned_abs())
            .max()
            .unwrap_or(0)
    }

    #[test]
    fn a_closed_link_through_the_section_falls_silent()
    {
        // The two tests below run a transparent section, and this is what that
        // trades away, measured instead of left unsaid. A closed link carries
        // what this board sent, so its content crosses the section once a lap
        // and what survives is the part of it the section passes.
        //
        // MEASURED here, from a seed of 268433753: one lap leaves 54774476, and
        // twelve leave 5754772, a forty-sixth of the seed. The fall is steep
        // over the first laps and then slow, because what is left by then sits
        // where the section barely attenuates. The bound below is a twentieth,
        // which is half the measured fall.
        //
        // It is also the reason a closed link can measure WHERE a word lands
        // and never what the section does to it: a bench reading this loop
        // reads a content the section has already eaten.
        let mut interface = MockInput::healthy();

        interface.offset = plan().arming_position();

        let shift = shift();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift));

        seed_tone(&mut interface);

        let seeded = largest_magnitude(&interface.master);

        assert!(seeded > 0, "the seed carries a tone");
        assert_eq!(run_loop(&mut interface, 12, shift, &mut stage()), Ok(()));

        let left = largest_magnitude(&interface.master);

        assert!
        (
            left.saturating_mul(DECAY_FLOOR) < seeded,
            "twelve crossings of the section left {left} of {seeded}"
        );
        assert_eq!(interface.master, interface.slave);
    }

    #[test]
    fn every_block_of_a_run_joins_the_one_before_it()
    {
        // What a bench hears rather than what a counter reads. A block carried
        // to a position that does not continue the block before it puts a step
        // in the middle of the signal, twice a lap and two hundred times a
        // second, and the tone crackles. So this measures the largest step
        // between neighbouring samples against the largest step the seeded tone
        // has of its own: a signal that still joins cannot exceed it.
        for offset in LOOP_OFFSETS
        {
            let mut interface = MockInput::healthy();

            interface.offset = offset;

            let shift = shift_at(offset);

            assert_eq!
            (
                bring_up(&mut interface, &plan(), &waits()),
                Ok(shift),
                "the bring-up refused a distance of {offset} words"
            );

            seed_tone(&mut interface);

            let bound = largest_step(&narrowed_buffer(&interface.master), 0);

            assert!(bound > 0, "the seeded tone moves");
            assert_eq!(run_loop(&mut interface, 12, shift, &mut transparent_stage()), Ok(()));

            for slot in 0..2
            {
                assert_eq!
                (
                    largest_step(&interface.master, slot),
                    bound,
                    "master slot {slot} at offset {offset} carries a step the \
                     tone does not"
                );
                assert_eq!
                (
                    largest_step(&interface.slave, slot),
                    bound,
                    "slave slot {slot} at offset {offset} carries a step the \
                     tone does not"
                );
            }
        }
    }

    #[test]
    fn the_shift_stands_a_whole_pipeline_under_the_counter_distance()
    {
        // The bench numbers, read straight off `measure`. A counter distance of
        // 196 words is what the board leaves, and 188 is the shift that brought
        // a 1000 Hz tone back at 1000 Hz over a closed link, against 990.9 Hz
        // at 196. The buffers here hold the 882 words the board runs, so the
        // two cases are one.
        //
        // Read off `measure` rather than out of a loop, so a reader looking for
        // the correction finds it at one assertion instead of at the pitch of a
        // tone twelve laps on.
        let plan = plan();
        let words = plan.transfer_items();
        let aim = plan.arming_position();

        assert_eq!(words, 882);
        assert_eq!
        (
            CarryShift::measure(plan, words - 196, words).map(CarryShift::words),
            Some(188)
        );
        assert_eq!
        (
            CarryShift::measure(plan, words - aim, words).map(CarryShift::words),
            Some(aim - PIPELINE_WORDS)
        );
    }

    #[test]
    fn a_word_leaves_in_the_slot_it_arrived_in()
    {
        // The two channels are one frame of two slots, so a shift of an odd
        // number of words would take a word out of the other one. The counters
        // are read one after the other rather than together, so a distance that
        // IS a whole number of frames can read one word off, and the distance
        // is taken to the frame below rather than believed. The pipeline comes
        // off that, and it is a whole number of frames itself, so both halves
        // of the shift land on a frame boundary.
        let plan = plan();
        let words = plan.transfer_items();

        for offset in [192_u32, 193, 220, 221]
        {
            let shift = shift_at(offset);

            assert_eq!
            (
                CarryShift::measure(plan, words - offset, words),
                Some(shift),
                "no shift for a distance of {offset} words"
            );
            assert_eq!(shift.words() % plan.frame_slots(), 0, "offset {offset}");
            assert!(shift.words() + PIPELINE_WORDS <= offset, "offset {offset}");
            assert!
            (
                offset - shift.words() - PIPELINE_WORDS < plan.frame_slots(),
                "offset {offset}"
            );

            for index in 0..words
            {
                assert_eq!
                (
                    shift.destination(plan, index) % plan.frame_slots(),
                    index % plan.frame_slots(),
                    "index {index} at offset {offset}"
                );
            }
        }
    }

    #[test]
    fn a_shift_no_bring_up_published_is_refused()
    {
        // What the handler rebuilds one from is a word of memory, so the values
        // that word can hold before a bring-up has written it are refused
        // rather than carried on.
        let plan = plan();

        assert_eq!(CarryShift::from_words(plan, u32::MAX), None);
        assert_eq!(CarryShift::from_words(plan, 0), None);
        assert_eq!(CarryShift::from_words(plan, plan.offset_low() - 1), None);
        assert_eq!(CarryShift::from_words(plan, plan.offset_high() + 1), None);

        // The shift a start-up on the aim publishes, and the odd word beside
        // it. What a bring-up leaves in that word is the aim less the pipeline,
        // not the aim.
        let aim = shift_at(plan.arming_position()).words();

        assert_eq!(CarryShift::from_words(plan, aim + 1), None);
        assert_eq!
        (
            CarryShift::from_words(plan, aim).map(CarryShift::words),
            Some(aim)
        );
    }

    #[test]
    fn the_waits_cover_the_laps_they_name()
    {
        let plan = plan();
        let waits = InputWaits::for_plan(&plan, 64_000_000);

        // One lap of 441 frames at 44100 Hz is 10 ms, so the phase budget
        // covers a lap and the seed budget covers three of them.
        assert_eq!(waits.phase_polls, wait_polls(64_000_000, 10_000));
        assert_eq!(waits.seed_polls, wait_polls(64_000_000, 30_000));
        assert!(waits.stop_polls > 0);
        assert!(waits.start_polls > 0);
    }
}
