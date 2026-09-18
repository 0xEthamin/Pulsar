//! Input path of the processing board, and the block structure it feeds.
//!
//! Frames of two slots arrive on sub-block A of the second audio interface,
//! run as a slave receiver synchronised on the first interface, and one
//! circular transfer stream drops them into a buffer the length of the two
//! output buffers. What runs on top of that is a block structure: each time the
//! receiving stream finishes a half of its buffer, the frames of that half are
//! carried to the two output buffers, at the positions those frames were sent
//! from.
//!
//! Nothing here touches a peripheral. `InputInterface` is the interface a
//! register block implements, `bring_up` is the sequence run over it, `arm` is
//! what puts the block structure into service, `next_block` is the decision one
//! transfer event produces, and `carry_block` is the loop that walks the frames
//! of one block through the two accessors of that interface.
//!
//! # What the carry does to a frame
//!
//! It reads the two slots of the frame, the left and the right channel of the
//! sender, and takes their mean, since one loudspeaker plays both. It scales
//! that mean by the volume gain the block walks, runs the sample through the
//! three cascades of the crossover and through the output stage, and writes the
//! three words that stage answers plus one silent word across the four
//! converter channels: the low way and the mid way into the two slots of the
//! master frame, the high way and the channel no way drives into the two slots
//! of the slave frame. `FilterChain` is those three cascades, the history
//! behind every section of them and the thermal limiter of the high way, and
//! the output stage beside it carries the alignment of the three ways, the peak
//! reserve, that limiter and the wall on the high way, and the conversion back
//! to a buffer word.
//!
//! The loop lives here rather than behind the interface so that a host test
//! walks it, the fan-out onto the four channels included. A register block owns
//! the two accessors and nothing else of the carry, which leaves the part of
//! the audio path that touches samples on the side of the build `cargo test`
//! reaches.
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
//! placement that leaves 433 words of margin ahead of the first carry of a lap
//! and 432 ahead of the second.
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
//! The streams take 11.34 microseconds a word at this frame rate, so a frame is
//! 22.68 microseconds, a block of 441 words is 5.000 milliseconds, and a new
//! event lands every 5.000 milliseconds. The carry runs once a block and its
//! loop once a frame. COUNTED on the text of the loop in the linked image, a
//! frame holds 354 instructions, 83 accesses to the memory the buffers and the
//! cascades sit in, 13 reads of the literal pool of the code, and 63 accesses to
//! the stack. A frame whose high sample stands inside the wall runs 12 of those
//! reads, and the thirteenth runs on a high sample under the negative side of
//! it. One of the instructions is the division that places the frame on the
//! gain ramp, which the count takes at one cycle like the rest.
//!
//! ESTIMATED from those counts, since no cycle counter has read this image. An
//! instruction is taken at one core cycle, an access to the stack at one more,
//! the stack sitting in the data memory coupled to the core, and an access to
//! the memory the buffers and the cascades sit in, or a read of the literal
//! pool, at `c` cycles. A frame is then `354 + 96 * c + 63` cycles at 64 MHz.
//! At a `c` of 3.9 that is 791 cycles, about 175000 for an entry of 221 frames,
//! 2.73 milliseconds and 55 per cent of the period. That is 12.4 microseconds a
//! frame, and it leaves the carry 1.8 times faster than the streams it has to
//! stay ahead of. It would take 10.8 cycles an access to bring the carry down
//! to the speed of the streams, 2.8 times that `c`.
//!
//! Ahead of the loop, once a block, the entry reads the control link: up to
//! 34 characters off its receiver, and one step of the volume ramp. That work
//! does not grow with the frames and is not in the count above. A character
//! can force more than one decode, since the reader decodes again after every
//! resynchronisation, so the worst stream is not the one with the most
//! characters. SEARCHED over the rules of the reader, the worst 34 characters
//! are a start byte and a length of three repeated, which force 52 decodes and
//! 59 bytes of checksum. Charging every one of those decodes at the deepest
//! path through the decoder gives about 26000 cycles, 0.41 milliseconds and
//! 8 per cent of the period, and charging only the decodes that reach the
//! checksum gives about 17000 cycles, 0.27 milliseconds and 5 per cent.
//!
//! `c` is a CALIBRATION taken off the part and not a reading of this image. The
//! cycle counter of the data watchpoint unit, read on the clock the part boots
//! on over some 2200 entries of a carry whose frame counted 273 instructions,
//! 82 accesses charged at `c` and 48 to the stack, put the longest entry of
//! 221 frames at 141595 cycles, and `(141595 / 221 - 273 - 48) / 82` is 3.9,
//! the work done once a block included. The same reading on this image is what
//! replaces the estimate.
//!
//! Those 83 accesses are one number for one quantity, made in 83 transactions
//! of one word each. 75 of them are the cascades: 35
//! coefficient reads and 40 writes of the four history words each of the ten
//! sections carries. 2 are the thermal limiter of the high way, its average and
//! its gain written back. The other 6 are the buffers, the two slots of the
//! source frame and the four output words. The 13 reads of the literal pool
//! reach eleven words, the
//! constants the vector unit cannot encode as an immediate: the gains of the
//! low and mid ways, each a trim times the peak reserve, the gain of the high
//! way in the unit of the limiter, full scale, the coefficient, the floor and
//! the reciprocal of the threshold of the limiter, the two sides of the
//! ceiling, and the float zero in two places, which answers a value that is not
//! a number, an average under the floor, a step of the gain that does not land
//! over zero and a source index out of range on either slot.
//!
//! The cascade of a way is folded into the loop, and that is what holds the
//! count there: it hoists fifteen of the twenty coefficients of the low way out
//! of that memory for the whole block, onto the stack, and reads the other five
//! back every frame. The forty history words and the state of the limiter are
//! written back every frame whichever way that falls, each being live into the
//! next sample. Left out of line, every section reads its five coefficients and
//! its four history words back through a pointer on every sample, which is 57
//! accesses a frame more than the folded carry and 10 reads of the pool fewer.
//! That form is not measured either: by the count it reaches the speed of the
//! streams near twice `c`, against 2.8 times for the folded carry.
//!
//! A carry starts writing where the read pointer has just left, so at an entry
//! taken on time it has 433 words of clearance ahead of it on the first block
//! and 432 on the second, 4.90 milliseconds at the least. One that fits inside
//! a block therefore never crosses the pointer.
//!
//! One that does not fit never catches up either: `travelled` grows by the
//! overrun at every entry, the clearance shrinks by as much, and the streams
//! overtake the carry inside the block. Every word behind the crossing goes out
//! holding the lap before, a step of up to full scale on a way with no analog
//! filter in front of it. `next_block` refuses once `travelled` reaches what is
//! left of the lap once the span the carry writes and the margin are taken out,
//! which bounds how long that lasts but does not come first: the damage leaves
//! the machine before the silence does.
//!
//! That is what makes the cost of this loop a real-time budget rather than a
//! preference. It is why the two accessors under it and the cascade of a way
//! are all asked to fold, and why the gate of this firmware holds a ceiling on
//! the code the served handler runs.
//!
//! # What the passthrough costs in delay
//!
//! A frame received at input position `p` is written into output position
//! `p + shift`, the position it was sent from, and the read pointer stood
//! `PIPELINE_WORDS` past that position when the frame was counted in. The
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
use crate::control::Applied;
use crate::filter::{FilterError, Way, WayCascade, way_sections};
use crate::output::{ThermalLimiter, WaySamples, WayWords};
use crate::readback::refuse_unless;
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
/// taken on time starts at least 432 words and 4.90 milliseconds ahead of it,
/// and the floor is what a handler entered late enough runs into.
///
/// A carry of one block takes LONGER than the 624 microseconds an entry at the
/// floor leaves, by the estimate the module documentation carries. What
/// answers that is the rate rather than the total: by the same estimate the
/// carry spends less on a frame than the 22.68 microseconds the streams spend
/// reading one, so from the tightest entry this admits it starts 55 words
/// ahead of the read pointer and gains on it at every frame. The total would
/// only matter to a carry that ran no faster than the streams, and the module
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
    /// The crossover build refused a corner, or filled a way with a count of
    /// sections its cascade holds no room for, so the chain carries no
    /// coefficients and the machine stays silent rather than carrying a way
    /// with no filter on it.
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
/// second block starts on the second slot of a frame rather than on the first,
/// and one frame straddles the boundary between the two blocks.
///
/// The carry runs on frames, so it cares. It walks the frames whose LAST slot
/// falls inside the block and reads and writes both words of each, which
/// carries the straddling frame whole from the block its last slot belongs to.
/// Every word a carry reads then stands in the half whose event started it or
/// in the half before, so it is in by that event whatever order the carry reads
/// it in. `InputPlan::carry_start` and `InputPlan::carry_frames` are that
/// reading, so it follows from the length of the buffer rather than being
/// written down as a property of the halves.
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

    /// Returns the first source index the carry of one block reads.
    ///
    /// The carry walks the frames whose last slot falls inside the block, so
    /// the carry of a block beginning part way into a frame begins at the start
    /// of that frame. On a buffer of an odd number of frames the second block
    /// begins one slot inside a frame, and this places its carry one slot
    /// before the block, on a word the first half holds.
    ///
    /// A frame of no slots names no boundary and gives the block start.
    #[must_use]
    pub(crate) const fn carry_start(self, half: Half) -> u32
    {
        let start = self.block_start(half);

        match start.checked_rem(self.frame_slots())
        {
            Some(0) | None => start,
            Some(over) => start.saturating_sub(over),
        }
    }

    /// Returns the frames the carry of one block reads.
    ///
    /// A frame counts when its last slot stands inside the block, so the frame
    /// holding the last slot of a block of an odd number of words counts in the
    /// block after it, whole. That is what makes the two blocks of a lap cover
    /// the buffer exactly once between them on a buffer whose blocks are an odd
    /// number of words.
    ///
    /// A frame of no slots names no count and gives none.
    #[must_use]
    pub(crate) const fn carry_frames(self, half: Half) -> u32
    {
        let words = self.block_end(half).saturating_sub(self.carry_start(half));

        match words.checked_div(self.frame_slots())
        {
            Some(frames) => frames,
            None => 0,
        }
    }

    /// Returns the words the carry of one block writes into each output buffer.
    ///
    /// It is the SPAN the guard protects, and it is not the length of the
    /// block: a block whose last slot opens a frame leaves that frame to the
    /// block after it, which starts that much earlier.
    #[must_use]
    pub(crate) const fn carry_words(self, half: Half) -> u32
    {
        self.carry_frames(half).saturating_mul(self.frame_slots())
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
    /// The span it measures is `carry_words`, taken from `carry_start`, which
    /// is the span the carry itself loops over. The length of the BLOCK is not
    /// that span: on a buffer of an odd number of frames the first carry stops
    /// one slot short of the end of its block, and the second starts one slot
    /// before its own. A guard reading the block instead admits a read pointer
    /// one word closer to the start of the second carry than the margin, and
    /// refuses one standing on the word the first carry leaves to the second.
    ///
    /// It answers for that instant alone. Whether the carry then STAYS clear of
    /// the pointer is the speed of the carry against the speed of the streams,
    /// which no counter reports, and the module documentation carries what a
    /// carry too slow to hold that does.
    ///
    /// The read pointer has to stand outside the span AND still have the margin
    /// ahead of it, and the one expression below is both. The distance forward
    /// from the end of the span to the pointer is what is left of the buffer
    /// once the span is taken out, less the words the pointer has yet to walk
    /// before it reaches the first word the carry writes, so bounding it by the
    /// buffer less the span less the margin says the pointer is past the span
    /// and at least the margin short of returning to it.
    #[must_use]
    pub const fn block_is_clear(self, half: Half, position: u32, shift: CarryShift) -> bool
    {
        let carried = self.carry_words(half);
        let last = self.carry_start(half).saturating_add(carried);
        let end = shift.destination(self, last);
        let raw = position.wrapping_add(self.buffer_words).wrapping_sub(end);

        let Some(travelled) = wrap(raw, self.buffer_words)
        else
        {
            return false;
        };

        travelled < self.buffer_words
            .saturating_sub(carried)
            .saturating_sub(self.guard_words())
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

        if !self.guard_leaves_room()
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

    /// Returns whether some position of the read pointer clears both blocks.
    ///
    /// `block_is_clear` admits a pointer standing outside the span a carry
    /// writes with the margin still ahead of it, so a buffer whose span plus
    /// margin covers the whole lap admits no position at all and every entry
    /// would be refused. Both spans are read, since the two blocks of a lap
    /// write a different number of words when the buffer holds an odd number of
    /// frames.
    const fn guard_leaves_room(self) -> bool
    {
        let first = self.carry_words(Half::First).saturating_add(self.guard_words());
        let second = self.carry_words(Half::Second).saturating_add(self.guard_words());

        first < self.buffer_words && second < self.buffer_words
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
        refuse_unless!
        {
            seen, InputBlockFault,
            ModeWrong unless mode_bits carries self.mode_field(),
            ProtocolWrong unless protocol_bits carries self.output.protocol_field(),
            SynchronisationWrong unless sync_bits carries self.sync_field(),
            SynchronisationSourceWrong unless sync_in_bits carries self.sync_in_field(),
            SynchronisationOutputWrong unless sync_out_bits carries SYNC_OUT_NONE,
            DataSizeWrong unless data_size_bits carries self.output.data_size_field(),
            BitOrderWrong unless lsb_first clear,
            ClockStrobingWrong unless changes_on_falling_edge
                carries self.output.clock_strobing_field(),
        }

        Ok(())
    }

    /// Checks the frame and the slots inside it.
    fn verify_block_frame(self, seen: &InputBlockReadback) -> Result<(), InputBlockFault>
    {
        refuse_unless!
        {
            seen, InputBlockFault,
            FrameLengthWrong unless frame_length_field carries self.output.frame_length_field(),
            FrameActiveLengthWrong unless frame_active_field
                carries self.output.frame_active_field(),
            FrameDefinitionWrong unless frame_marks_channel set,
            FramePolarityWrong unless frame_active_high clear,
            FrameOffsetWrong unless frame_leads_first_bit set,
            FirstBitOffsetWrong unless first_bit_offset_field carries 0,
            SlotSizeWrong unless slot_size_bits carries self.output.slot_size_field(),
            SlotCountWrong unless slot_count_field carries self.output.slot_count_field(),
            SlotsNotEnabled unless slot_enable_bits carries self.output.slot_enable_field(),
            FifoThresholdWrong unless fifo_threshold_bits
                carries self.output.fifo_threshold_field(),
        }

        Ok(())
    }

    /// Checks the receiving stream against the plan and its own flags.
    fn verify_stream(self, seen: &InputStreamReadback) -> Result<(), InputStreamFault>
    {
        refuse_unless!
        {
            seen, InputStreamFault,
            RequestWrong unless request_bits carries self.request_field(),
            SynchronisationEnabled unless sync_enabled clear,
            DirectionWrong unless direction_bits carries self.direction_field(),
            NotCircular unless circular set,
            MemoryNotIncrementing unless memory_increments set,
            PeripheralIncrementing unless peripheral_increments clear,
            MemoryWidthWrong unless memory_width_bits carries self.output.width_field(),
            PeripheralWidthWrong unless peripheral_width_bits carries self.output.width_field(),
            DoubleBuffered unless double_buffered clear,
            PriorityWrong unless priority_bits carries self.output.priority_field(),
            ErrorInterruptEnabled unless error_interrupt_enabled clear,
            EventInterruptEnabled unless event_interrupt_enabled clear,
            PeripheralAddressWrong unless peripheral_address carries self.data_address(),
            MemoryAddressWrong unless memory_address carries self.buffer_address(),
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
    refuse_unless!
    {
        seen, InputBlockFault,
        MonoWrong unless mono clear,
        TristateWrong unless tristate clear,
        InterruptEnabled unless any_interrupt_enabled clear,
        TransferDisabled unless transfer_enabled set,
        NotEnabled unless enabled set,
        Overrun unless overrun clear,
        FrameMismatch unless frame_mismatch clear,
        CodecNotReady unless codec_not_ready clear,
        FifoNeverFilled unless fifo_level_bits rejects FIFO_LEVEL_EMPTY,
    }

    Ok(())
}

/// Checks the flags, the counter and the enable of the receiving stream.
fn verify_stream_state(seen: &InputStreamReadback, plan: InputPlan)
    -> Result<(), InputStreamFault>
{
    refuse_unless!
    {
        seen, InputStreamFault,
        TransferError unless transfer_error clear,
        DirectModeError unless direct_mode_error clear,
        FifoError unless fifo_error clear,
        // The counter is a position inside the buffer once the stream runs, so
        // what is required of it here is that it names one. `verify_armed`
        // compares it against the plan while the stream is stopped.
        CounterOutOfRange unless items at most plan.transfer_items(),
        NotEnabled unless enabled set,
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
/// cannot fold, and the carry reaches the accessors six times a frame, so the
/// door is shut rather than left open for a caller that does not exist.
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
    /// decides nothing: which buffer a way leaves in, and at which index, is
    /// `carry_block` above. That puts the fan-out on the side of the build
    /// `cargo test` reaches, where dropping one of the four destinations turns
    /// a test red rather than leaving one converter channel replaying whatever
    /// its buffer last held.
    ///
    /// An implementation that cannot reach the index writes nothing.
    ///
    /// This is the only way the block structure reaches an output buffer, and
    /// it moves one word at a time, so no caller of this trait holds a writable
    /// pointer into either buffer.
    fn write_word(&mut self, role: BlockRole, index: u32, word: u32);
}

/// The cascade of the low way.
type LowCascade = WayCascade<{ way_sections(Way::Low) }>;

/// The cascade of the mid way.
type MidCascade = WayCascade<{ way_sections(Way::Mid) }>;

/// The cascade of the high way.
type HighCascade = WayCascade<{ way_sections(Way::High) }>;

/// Sections one sample of the source crosses.
///
/// The three ways run in parallel over the same sample, so a sample crosses
/// the sum of the three cascades rather than the longest of them.
const SECTIONS_PER_SAMPLE: usize =
    way_sections(Way::Low) + way_sections(Way::Mid) + way_sections(Way::High);

/// The module documentation counts the cost of the carry on this many sections
/// a sample and estimates the time of the carry from that count. A crossover
/// that grows or loses a section therefore stops the build rather than leaving
/// a budget nobody took again.
const _: () = assert!
(
    SECTIONS_PER_SAMPLE == 10,
    "the sections one sample crosses moved, so the cost of the carry against \
     the block period is due a measurement"
);

/// Slot of a master frame the low way leaves in.
///
/// The master converter carries the low way and the mid way, the slave carries
/// the high way and the channel no way drives. Which slot of a module a way
/// leaves in is settled by the wiring, so the four constants here are what a
/// wiring change turns, and nothing else reads a slot of a frame.
const LOW_SLOT: u32 = 0;

/// Slot of a master frame the mid way leaves in.
const MID_SLOT: u32 = 1;

/// Slot of a slave frame the high way leaves in.
const HIGH_SLOT: u32 = 0;

/// Slot of a slave frame no way drives.
///
/// Three ways across two stereo converters leave one channel over. It is driven
/// with silence rather than left out of the carry, since a channel nothing
/// writes replays whatever its buffer last held.
const SPARE_SLOT: u32 = 1;

/// The word the channel no way drives carries.
const SILENT_WORD: u32 = 0;

/// The fan-out names one slot per way of a module plus the spare, so two of
/// them sharing a slot would leave one of the pair replaying the lap before,
/// and one past the frame would reach a word of the next frame.
const _: () = assert!
(
    LOW_SLOT != MID_SLOT
        && HIGH_SLOT != SPARE_SLOT
        && LOW_SLOT < SLOTS_PER_FRAME as u32
        && MID_SLOT < SLOTS_PER_FRAME as u32
        && HIGH_SLOT < SLOTS_PER_FRAME as u32
        && SPARE_SLOT < SLOTS_PER_FRAME as u32,
    "two ways of one module share a slot, or a way leaves in a slot the frame \
     does not carry"
);

/// The fan-out drives two slots of each module and the source is read at the
/// two slots of a frame, so a frame of another width would leave slots of every
/// frame replaying the lap before, or leave a channel of the sender out of the
/// mean. The transport pins the frame at 64 bit clock periods rather than at
/// two slots, and a frame of four 16 bit slots satisfies that, so this stands
/// on its own rather than repeating it.
const _: () = assert!
(
    SLOTS_PER_FRAME == 2
        && LEFT_SLOT != RIGHT_SLOT
        && LEFT_SLOT < SLOTS_PER_FRAME as u32
        && RIGHT_SLOT < SLOTS_PER_FRAME as u32,
    "a frame carries a slot the fan-out drives no way into, or the source is \
     read at a slot the frame does not carry"
);

/// Slot of a source frame the left channel of the sender arrives in.
const LEFT_SLOT: u32 = 0;

/// Slot of a source frame the right channel of the sender arrives in.
const RIGHT_SLOT: u32 = 1;

/// The three cascades of the crossover, the history behind each section, and
/// the thermal limiter of the high way.
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
/// # Why there is one history per way and not one per slot
///
/// The chain runs ONCE per frame, over the mean of the two channels the source
/// carries in the two slots, and the three words it answers with are fanned out
/// across the slots of the
/// two output frames. So the sequence each way filters is one sample per frame
/// at the frame rate, and one history per way is the whole of what a way
/// remembers. A history per slot would be a second, interleaved run of the same
/// way over samples no source carries.
///
/// # What an unbuilt one carries
///
/// `silent` holds a silent cascade on each way, which stops the signal, and a
/// silent limiter on the high way, which stops it again. A chain that never
/// took its coefficients therefore carries silence rather than the full range
/// of a way with no filter on it, and there is no value of this type that means
/// no filtering. The limiter is a field of every value, so there is no value
/// either that carries a high way with no limiter behind it.
///
/// # Why the limiter lives here
///
/// Its average runs on from one frame to the next and from one block to the
/// next, exactly as the history of a section does, so it is state of the chain
/// and the caller owns it and lends it to the carry beside the cascades.
/// `built` holds one at rest: unity gain and no average behind it.
///
/// # What bounds the samples it returns
///
/// Nothing here. A cascade overshoots: a crossover half answers a full scale
/// step above full scale, so a signal near the top of the format leaves a way
/// past it. What each way answers is handed to the output stage, which carries
/// the alignment of the three ways, the peak reserve, the thermal limiter and
/// the wall on the high way, and the conversion back to a buffer word. The
/// conversion SATURATES, so a sample past the format cannot come back round to
/// the opposite sign, which would be a step of two full scales into a way with
/// no analog filter in front of it.
///
/// Every bound stands in that stage and none of them inside the recurrence: the
/// history a section feeds back is the value it computed. Feeding back a
/// bounded value instead would put a limiter inside the feedback of a filter,
/// which is no longer the filter the coefficients describe and no longer a
/// shape `cascade_magnitude` can be compared against.
#[derive(Debug, PartialEq)]
#[cfg_attr(test, derive(Clone))]
pub struct FilterChain
{
    low: LowCascade,
    mid: MidCascade,
    high: HighCascade,
    limiter: ThermalLimiter,
}

impl FilterChain
{
    /// Returns a chain that stops the signal on every way.
    #[must_use]
    pub const fn silent() -> Self
    {
        Self
        {
            low: LowCascade::silent(),
            mid: MidCascade::silent(),
            high: HighCascade::silent(),
            limiter: ThermalLimiter::silent(),
        }
    }

    /// Returns the chain the crossover of the loudspeaker makes, with the
    /// history of every section and the limiter of the high way at rest.
    ///
    /// # Errors
    ///
    /// Whatever a coefficient build refuses, and `WayLengthWrong` when a way
    /// fills a count of sections its cascade holds no room for. A refusal
    /// builds no chain, so the caller is left holding `silent` and the output
    /// stays muted rather than carrying a way with no filter on it.
    pub fn built() -> Result<Self, FilterError>
    {
        Ok
        (
            Self
            {
                low: LowCascade::of(Way::Low)?,
                mid: MidCascade::of(Way::Mid)?,
                high: HighCascade::of(Way::High)?,
                limiter: ThermalLimiter::at_rest(),
            }
        )
    }

    /// Runs the two words of one source frame through the three ways and the
    /// output stage, at `gain`.
    ///
    /// The mean of the two channels, times `gain`, crosses each way once, so
    /// the three answers come off one sample rather than three. What each way
    /// answers then crosses the output stage, which carries the alignment of
    /// the three ways, the peak reserve, the limiter and the ceiling of the
    /// high way and the conversion back to a word. The limiter is the one this
    /// chain holds, so its average runs on into the next frame.
    ///
    /// # Where the gain applies
    ///
    /// Ahead of every section, which is what lets a gain of zero stop the
    /// signal before it reaches a history: `Applied::preset` moves the
    /// coefficients only under such a gain. `gain` stands in `[0, 1]`, see
    /// `carry_block`, so the product stands at full scale or under like the
    /// mean it scales, and the limiter and the ceiling behind the ways see no
    /// sample louder than one slot carries on its own.
    ///
    /// # Where the mean forms, and what bounds it
    ///
    /// In single precision, after each word has crossed into the format on
    /// its own. The sum of two slot words taken as integers needs a 33rd bit,
    /// and a 32-bit sum would come back round to the opposite sign on two loud
    /// samples of one sign. Two converted words stand at full scale or under,
    /// their sum at twice that, which the format holds without rounding past
    /// it, and the halving is exact. So the mean stands at full scale or under
    /// whatever the two words hold, and the chain and every protection behind
    /// it see no sample louder than one slot on its own carries. Two equal
    /// words give that word, and two opposite words give zero exactly, since
    /// the conversion rounds a word and its negation alike.
    #[expect
    (
        clippy::cast_precision_loss,
        reason = "the audio path is single precision, and this is the step that \
                  gets a sample there"
    )]
    #[expect
    (
        clippy::manual_midpoint,
        reason = "the two operands stand at full scale or under, where the sum \
                  cannot overflow, and `f32::midpoint` compiles on this target to \
                  a guard on both magnitudes and a read of the literal pool that \
                  those operands never need, once a frame"
    )]
    fn carry(&mut self, left: u32, right: u32, gain: f32) -> WayWords
    {
        let mean = (left.cast_signed() as f32 + right.cast_signed() as f32) * 0.5;
        let sample = mean * gain;

        WayWords::of
        (
            WaySamples
            {
                low: self.low.step(sample),
                mid: self.mid.step(sample),
                high: self.high.step(sample),
            },
            &mut self.limiter
        )
    }

    /// Returns a chain whose every section answers a sample unchanged.
    ///
    /// What it is for is a test that measures WHERE a word lands, which needs
    /// the value to cross unchanged to be read at all.
    #[cfg(test)]
    fn transparent() -> Self
    {
        let section = crate::filter::unit_section_for_test();

        Self
        {
            low: LowCascade::of_section(section),
            mid: MidCascade::of_section(section),
            high: HighCascade::of_section(section),
            limiter: ThermalLimiter::at_rest(),
        }
    }
}

/// Carries the frames of one block of the receiving buffer into both output
/// buffers, at the positions those frames were sent from.
///
/// # Which frames, and which words it writes
///
/// It walks the frames whose LAST slot falls inside the block, and it reads
/// both words of each of them and writes both. A buffer holding an odd number
/// of frames splits into two blocks of an odd number of words, so one frame
/// straddles the boundary between them, and this rule is what carries that
/// frame whole from the block its last slot belongs to. The two blocks of a
/// lap then write the buffer entire, exactly once, and nothing of a half frame
/// is held between two entries.
///
/// The receiving stream raises the event of a half once the last word of that
/// half is in. Every word this reads stands in that half or, for the first
/// slot of the straddling frame, in the half before it, so every one of them
/// is in when the event is raised, and the order this reads them in changes
/// nothing of what it reads.
///
/// `InputPlan::carry_start` and `InputPlan::carry_words` are that rule, and the
/// guard every entry passes reads the same two, so the stretch it protects is
/// the stretch this writes.
///
/// # Where a word goes
///
/// `shift` turns a source index into the output position the frame was sent
/// from, and it moves a whole number of frames, so slot 0 of a source frame
/// lands on slot 0 of an output frame. The destination of an even index is
/// therefore even, and a slot of the frame above it stands inside the buffer
/// without wrapping, since the buffer holds a whole number of frames.
///
/// # What each way reaches
///
/// The chain runs once on the mean of the two slots and answers three words,
/// one per way. The master converter takes the low way and the mid way, the
/// slave takes the high way and the channel no way drives, which is written
/// with silence. Four converter channels for three ways leave one over, and it
/// is driven rather than skipped because a channel nothing writes replays
/// whatever its buffer last held.
///
/// The mean is the source contract: the two slots of a source frame are the
/// left and the right channel of one sender, and one loudspeaker plays both. A
/// channel the sender holds silent arrives at half its level, two channels in
/// opposition cancel, and the mean never stands past full scale, so the chain
/// sees nothing louder than one channel carries on its own.
///
/// # The gain
///
/// `applied` is the volume the block carries, as a pair of gains at its two
/// ends. Frame `n` of the block takes `applied.gain_at(n, frames)`, so a ramp
/// moves a step per frame rather than a step per block, and the next block
/// opens on the gain this one ends on.
///
/// That gain stands in `[0, 1]`. `ControlState` hands out ends in that range,
/// and `gain_at` walks `s + (e - s) x f` with `f` in `[0, 1]`. Rounding to
/// nearest is monotone, so the walked value stands between the values the
/// same expression takes at `e = 0` and at `e = 1`, `f = 1`, and those round
/// to exactly 0 and exactly 1. The test
/// `a_walked_gain_never_leaves_the_unit_range` sweeps the ends and every frame
/// of both block lengths against that bound.
///
/// # Where the loop lives
///
/// Here rather than behind the trait, so that a host test walks it, the fan-out
/// onto the four channels included. The register block owns the two accessors
/// and nothing else of the carry, which leaves the part of this path that
/// touches samples on the side of the build that `cargo test` reaches.
///
/// `plan` is the geometry the decision ahead of this was taken on, and `half`
/// the block that decision named. The history the chain carries runs on past
/// the end of this call, which is what makes a block boundary nothing the
/// signal can hear.
///
/// # What it costs, and what a slower one does
///
/// The streams that replay the output buffers take one word every 11.34
/// microseconds at this frame rate, so a frame is 22.68, and this loop has a
/// whole block to run in, which is 441 words and 5.000 milliseconds. Folded
/// into the handler it moves a frame faster than the streams read one. The
/// module documentation carries the count of a frame on the linked image, and
/// the cost ESTIMATED from it.
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
    chain: &mut FilterChain,
    applied: Applied
)
where
    I: InputInterface,
{
    let first = plan.carry_start(half);
    let frames = plan.carry_frames(half);
    let slots = plan.frame_slots();
    let words = plan.transfer_items();
    let mut at = shift.destination(*plan, first);

    for step in 0..frames
    {
        let index = first.saturating_add(step.saturating_mul(slots));
        let ways = chain.carry
        (
            interface.read_word(index.saturating_add(LEFT_SLOT)),
            interface.read_word(index.saturating_add(RIGHT_SLOT)),
            applied.gain_at(step as usize, frames as usize)
        );

        interface.write_word(BlockRole::Master, at.saturating_add(LOW_SLOT), ways.low());
        interface.write_word(BlockRole::Master, at.saturating_add(MID_SLOT), ways.mid());
        interface.write_word(BlockRole::Slave, at.saturating_add(HIGH_SLOT), ways.high());
        interface.write_word(BlockRole::Slave, at.saturating_add(SPARE_SLOT), SILENT_WORD);

        at = at.saturating_add(slots);

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
/// by silencing the machine. A `FilterChain` is parked ahead of the unmask
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
/// `permit` is the release gate having watched the transfers over a whole lap
/// and held a buffer of zeros over the converter unmute ramp, once the clock
/// and the chain had reported. It is taken BY VALUE, carried through this
/// sequence and spent on `InputInterface::enable_events`, which is the one door
/// onto the vector. So the interrupt that lets anything write the output
/// buffers continuously cannot be enabled by any path that does not hold the
/// permit, and the permit has no public constructor. That is the third term of
/// the gate held by the type rather than by a comment.
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
/// The gain is asked for once the entry is accepted and the flag is down, for
/// the frame count of the block about to be carried, so `gain` sees one call
/// per block carried and none for an entry refused.
///
/// # Errors
///
/// `Event`, carrying what the entry refused with. Nothing has been written when
/// one comes back, and neither `chain` nor `gain` has moved, so the caller
/// answers by silencing the machine.
pub fn serve<I, G>
(
    interface: &mut I,
    plan: &InputPlan,
    shift: CarryShift,
    chain: &mut FilterChain,
    gain: &mut G
) -> Result<(), PassthroughFault>
where
    I: InputInterface,
    G: BlockGain,
{
    let half = next_block(&interface.event(), plan, shift)?;

    interface.clear_event(half);

    let applied = gain.block(plan.carry_frames(half));

    carry_block(interface, plan, half, shift, chain, applied);

    Ok(())
}

/// The source of the gain one served block carries.
///
/// `crate::link::LinkedGain` is the one the firmware serves with: it reads the
/// control link and advances the volume ramp. A test hands `serve` a gain that
/// holds still instead.
pub trait BlockGain
{
    /// Returns the gain a block of `frames` frames walks.
    ///
    /// `serve` calls this once per block it carries, ahead of the carry, so an
    /// implementation that counts time counts `frames` of audio per call.
    fn block(&mut self, frames: u32) -> Applied;
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

// The three assertions below carry the arithmetic that ties the margin, the
// band and the arming window together. Each is written so that the divisor it
// names appears on one side alone: a form where a divisor cancels off both
// sides holds nothing, whatever its message says, and reads as a derivation
// while a mutation of that divisor walks past it.
//
// The aim and the window bound a distance between the two counters while the
// band bounds a shift, and `PIPELINE_WORDS` is what stands between the two. It
// is a count of words rather than a fraction of a block, so the assertion that
// has to hold the aim above the low bound carries it over
// `SHORTEST_BLOCK_WORDS` and the other two stay dimensionless. The pipeline
// only ever lowers a shift, which is why the high bound needs none of it.

// span + margin < buffer, over the shortest buffer the transport accepts, with
// no division. The longest span a carry writes is one block plus one frame and
// the buffer is two blocks, so what has to be left over is a frame and the
// margin.
const _: () = assert!
(
    SLOTS_PER_FRAME as u32 * GUARD_DIVISOR
        < SHORTEST_BLOCK_WORDS * (GUARD_DIVISOR - 1),
    "the span a carry writes plus the margin covers the whole lap on the \
     shortest buffer the transport accepts, so no position of the read pointer \
     would clear a block and every entry would be refused"
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
    #![allow
    (
        clippy::arithmetic_side_effects,
        clippy::cast_possible_truncation,
        clippy::panic
    )]

    use super::*;
    use crate::clock::AUDIO_PLAN;
    use crate::filter::
    {
        Biquad,
        CROSSOVER_SECTIONS,
        cascade_magnitude,
        crossover_sections,
    };
    use crate::readback::pin_cause_bytes;
    use crate::transport::{TONE_SAMPLES, TONE_TABLE};
    use core::cell::Cell;
    use core::f64::consts::TAU;
    use libm::{cos, log10, round, sin, sqrt};

    /// Words in each buffer of the tests, one tone period per channel.
    const TEST_WORDS: u32 = TONE_SAMPLES as u32 * 2;

    /// Words one frame of the transmitted format holds.
    const WORDS_PER_FRAME: u32 = SLOTS_PER_FRAME as u32;

    /// Frames one lap of a buffer of the tests holds.
    const LAP_FRAMES: u32 = TEST_WORDS / WORDS_PER_FRAME;

    /// Converter channels one frame of a carry reaches.
    ///
    /// Two slots of the master buffer and two of the slave, which is the whole
    /// of what the fan-out drives.
    const CARRIED_CHANNELS: u32 = 4;

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

    /// One way of the loudspeaker, its cascade and the channel it leaves on.
    ///
    /// The cascade is built from `crossover_sections` rather than read out of
    /// the chain, so a test comparing the two compares the chain against the
    /// design and not against itself.
    struct WayUnderTest
    {
        /// What a failure calls it.
        name: &'static str,
        /// Which way of the crossover it is.
        way: Way,
        /// The sections the way fills, in the order they run.
        sections: [Biquad; CROSSOVER_SECTIONS],
        /// How many of them the build wrote.
        count: usize,
        /// Which output buffer the way leaves in.
        role: BlockRole,
        /// Which slot of a frame of that buffer it leaves in.
        slot: u32,
        /// Words its measured response may stand from its design.
        response_words: f64,
        /// Periods per lap of the square that takes it past the format.
        square_cycles: u32,
        /// Words one of its samples may stand from the double precision
        /// evaluation of the same recurrence.
        feedback_words: f64,
        /// Whether that square drive takes the way as far as its own bound.
        ///
        /// The low way and the mid way are bounded by the conversion, at the
        /// format, and the high way by the wall. A full scale square in the
        /// pass band of the low way overshoots past the format once the stage
        /// has acted. The trim and the reserve of the mid way hold its square
        /// 0.79 dB under the format, and those of the high way hold its square
        /// 4.04 dB under the wall. Those are readings on one waveform: the
        /// loudest peak a source the format carries puts on the high way stands
        /// 2.52 dB under the wall. The wall is where the machine goes when the
        /// gain is wrong, so a square reaching it while the gain is right would
        /// be the defect rather than the measurement.
        reaches_its_bound: bool,
    }

    impl WayUnderTest
    {
        /// Returns the sections the build wrote.
        fn active(&self) -> &[Biquad]
        {
            self.sections.get(..self.count).unwrap_or(&[])
        }

        /// Returns the gain the output stage applies to this way.
        fn gain(&self) -> f64
        {
            f64::from(crate::output::gain_for_test(self.way))
        }

        /// Returns the word this way comes out at for a sample past every
        /// bound, on the side `positive` names.
        ///
        /// The low way and the mid way stand at the format and the high way at
        /// the wall, and this writes those figures down as literals. A bound
        /// read by handing the stage an infinity moves with the conversion it
        /// bounds, so a conversion that answered the opposite side past the
        /// format would move the expectation with the answer and pass.
        fn bound(&self, positive: bool) -> i32
        {
            match (self.way, positive)
            {
                (Way::Low | Way::Mid, true) => i32::MAX,
                (Way::Low | Way::Mid, false) => i32::MIN,
                (Way::High, true) => 1_336_379_648,
                (Way::High, false) => -1_336_379_648,
            }
        }

        /// Returns the magnitude a sample of this way cannot pass, as the
        /// output stage answers it on the side `positive` names.
        fn bound_magnitude(&self, positive: bool) -> f64
        {
            f64::from(self.bound(positive).unsigned_abs())
        }
    }

    /// Returns the three ways of the crossover, with the channel each reaches.
    ///
    /// The channels are read off the four constants the fan-out turns, so a
    /// wiring change moves the expectation with the code rather than leaving
    /// the two to be reconciled by hand.
    fn ways_under_test() -> [WayUnderTest; 3]
    {
        [
            built_way(LOW_UNDER_TEST),
            built_way(MID_UNDER_TEST),
            built_way(HIGH_UNDER_TEST),
        ]
    }

    /// Everything about one way that does not come out of a build.
    type WaySetting = (&'static str, Way, BlockRole, u32, f64, u32, f64, bool);

    /// The low way, on the first slot of the master converter.
    const LOW_UNDER_TEST: WaySetting =
    (
        "the low way",
        Way::Low,
        BlockRole::Master,
        LOW_SLOT,
        LOW_RESPONSE_WORDS,
        LOW_SQUARE_CYCLES,
        LOW_FEEDBACK_WORDS,
        true,
    );

    /// The mid way, on the other slot of the same converter.
    const MID_UNDER_TEST: WaySetting =
    (
        "the mid way",
        Way::Mid,
        BlockRole::Master,
        MID_SLOT,
        MID_RESPONSE_WORDS,
        MID_SQUARE_CYCLES,
        MID_FEEDBACK_WORDS,
        false,
    );

    /// The high way, on the first slot of the slave converter.
    const HIGH_UNDER_TEST: WaySetting =
    (
        "the high way",
        Way::High,
        BlockRole::Slave,
        HIGH_SLOT,
        HIGH_RESPONSE_WORDS,
        HIGH_SQUARE_CYCLES,
        HIGH_FEEDBACK_WORDS,
        false,
    );

    /// Builds the cascade of one way, failing the test on a refusal.
    fn built_way(setting: WaySetting) -> WayUnderTest
    {
        let
        (
            name,
            way,
            role,
            slot,
            response_words,
            square_cycles,
            feedback_words,
            reaches_its_bound
        ) = setting;
        let mut sections = [Biquad::SILENT; CROSSOVER_SECTIONS];

        let Ok(count) = crossover_sections(way, &mut sections)
        else
        {
            panic!("{name} refused its own corners");
        };

        WayUnderTest
        {
            name,
            way,
            sections,
            count,
            role,
            slot,
            response_words,
            square_cycles,
            feedback_words,
            reaches_its_bound,
        }
    }

    /// Returns the chain the machine runs, at rest.
    fn chain() -> FilterChain
    {
        match FilterChain::built()
        {
            Ok(built) => built,
            Err(error) => panic!("the chain refused: {error:?}"),
        }
    }

    /// Returns a chain that answers every sample unchanged.
    ///
    /// The loop tests below measure WHERE a word lands over laps of a closed
    /// link, and a closed link crosses the chain once a lap, so a shape that
    /// attenuates takes the content to zero and leaves them comparing silence
    /// against silence. A transparent chain is what keeps the placement
    /// readable. The response of the real cascades is measured over a driven
    /// source instead, where it belongs.
    fn transparent_chain() -> FilterChain
    {
        FilterChain::transparent()
    }

    /// Returns a gain of exactly 1 across a whole block.
    ///
    /// A sample times 1 is that sample, so a test driving the carry at this
    /// gain reads the same words it read before the gain existed.
    fn unity() -> Applied
    {
        Applied::between_for_test(1.0, 1.0)
    }

    /// A gain source that hands out `unity` for every block.
    struct Steady;

    impl BlockGain for Steady
    {
        fn block(&mut self, _frames: u32) -> Applied
        {
            unity()
        }
    }

    /// Returns the word `word` becomes once it has crossed a transparent chain.
    ///
    /// A transparent cascade changes no value, and the sample still crosses
    /// single precision, which holds 24 bits of a sample that carries 32. So a
    /// word comes back rounded, by at most half of one unit in the last place
    /// of its own magnitude, and the rounded value is a fixed point of the
    /// narrowing: it is already a float, so a second crossing leaves it where
    /// it is. A lap of a closed link hands the stage that value, so an
    /// expectation that replays the stage lap for lap from it is exact rather
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

    /// Returns the words the output stage leaves for a source frame carrying
    /// `left` and `right`, read through cascades that change nothing.
    ///
    /// A transparent chain hands the same sample to all three ways, so what
    /// comes back is the stage alone: the alignment of the three ways, the peak
    /// reserve, the limiter at rest and the wall on the high way, and the
    /// conversion. The tests of PLACEMENT below expect their words through this
    /// rather than through a second copy of the alignment, and the figures the
    /// alignment stands at are read where they are written.
    fn staged(left: u32, right: u32) -> WayWords
    {
        staged_sample(source_sample(left, right))
    }

    /// Returns the sample the carry reads a source frame holding `left` and
    /// `right` as.
    ///
    /// Each word crosses single precision as the carry converts it, and the
    /// mean then forms in DOUBLE precision, where the sum of the two is exact,
    /// before it narrows once. Halving commutes with that rounding, so this is
    /// the value the single precision sum and halving of the carry leave,
    /// derived by another route rather than transcribed.
    #[expect
    (
        clippy::cast_precision_loss,
        reason = "the carry crosses single precision at the same step, and this \
                  is what makes the two answer alike"
    )]
    fn source_sample(left: u32, right: u32) -> f32
    {
        let sum = f64::from(left.cast_signed() as f32) + f64::from(right.cast_signed() as f32);

        (sum / 2.0) as f32
    }

    /// Returns the sample the carry reads the frame of `buffer` opening at
    /// `index` as.
    fn frame_sample(buffer: &[u32; TEST_WORDS as usize], index: u32) -> f32
    {
        source_sample
        (
            carried_word(buffer, index + LEFT_SLOT),
            carried_word(buffer, index + RIGHT_SLOT)
        )
    }

    /// Returns the words the output stage leaves when all three ways answer
    /// `sample`, through a limiter at rest.
    ///
    /// A limiter at rest answers unity on any one sample. On the drives the
    /// tests below give a chain with the trim in place, a running limiter
    /// stands at unity too, and every test that reads a carried word against
    /// this reads that as well. A test that drives past the trim reads the
    /// limiter it runs on instead.
    fn staged_sample(sample: f32) -> WayWords
    {
        WayWords::of
        (
            WaySamples { low: sample, mid: sample, high: sample },
            &mut ThermalLimiter::at_rest()
        )
    }

    /// Returns the word `way` leaves out of `words`.
    fn way_word(way: Way, words: WayWords) -> u32
    {
        match way
        {
            Way::Low => words.low(),
            Way::Mid => words.mid(),
            Way::High => words.high(),
        }
    }

    /// Returns the image `role` holds once the source `buffer` has crossed a
    /// transparent chain and the output stage, frame for frame.
    ///
    /// The sample of a frame is the mean of its two slots, which is the source
    /// contract, so a buffer seeded with two different slots comes back
    /// carrying their mean on every channel.
    fn staged_buffer(buffer: &[u32; TEST_WORDS as usize], role: BlockRole)
        -> [u32; TEST_WORDS as usize]
    {
        let mut out = [0_u32; TEST_WORDS as usize];

        for index in 0..TEST_WORDS
        {
            let first = index - index % WORDS_PER_FRAME;
            let words = staged_sample(frame_sample(buffer, first));
            let slot = index % WORDS_PER_FRAME;

            let word = match (role, slot)
            {
                (BlockRole::Master, LOW_SLOT) => words.low(),
                (BlockRole::Master, _) => words.mid(),
                (BlockRole::Slave, HIGH_SLOT) => words.high(),
                (BlockRole::Slave, _) => SILENT_WORD,
            };

            place(&mut out, index, word);
        }

        out
    }

    /// Returns what a closed link seeded with `seed` hands a transparent chain
    /// at lap `laps`, counting from one.
    ///
    /// The link closes on the master buffer, whose two slots carry the low way
    /// and the mid way, so each lap reads the mean of the words those two ways
    /// left at the lap before, and the first lap reads the seed.
    fn looped_buffer
    (
        seed: &[u32; TEST_WORDS as usize],
        laps: u32
    ) -> [u32; TEST_WORDS as usize]
    {
        let mut entered = *seed;

        for _ in 1..laps
        {
            entered = staged_buffer(&entered, BlockRole::Master);
        }

        entered
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
        /// Which indices of an output buffer a carry reached, whichever buffer
        /// it reached them in.
        ///
        /// It is how a test reads the span the carry WRITES out of the carry
        /// itself, rather than deriving that span a second time beside it.
        touched: [bool; TEST_WORDS as usize],
        /// How many writes `write_word` was given, so a test reads that a whole
        /// block reached every channel and not only that something did.
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
                touched: [false; TEST_WORDS as usize],
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

            if let Some(seen) = self.touched.get_mut(index as usize)
            {
                *seen = true;
            }

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

    /// Breaks one field of a verified read-back per case and checks the fault,
    /// then sweeps the same table for the order the requirements are written
    /// in.
    ///
    /// Every table passed here therefore stands in the order its requirements
    /// are checked. A table that does not fails the sweep, which is what keeps
    /// the two readings of one table from drifting apart.
    fn run_mutations(mutations: &[Mutation])
    {
        for (break_it, expected) in mutations
        {
            let mut seen = verified_readback();
            break_it(&mut seen);

            assert_eq!(plan().verify(&seen), Err(*expected));
        }

        run_precedence(mutations);
    }

    /// Breaks two fields of a verified read-back per pair and checks that the
    /// fault of the earlier entry is the one returned.
    ///
    /// The entries stand in the order the requirements are written, so a pair
    /// measures that order. One bad field cannot: it leaves every other
    /// requirement holding, so the fault comes back whatever the order of the
    /// rest. The sweep takes every pair rather than a chosen few, since a pair
    /// nobody thought to name is where an order goes wrong.
    fn run_precedence(ordered: &[Mutation])
    {
        for (earlier, (break_earlier, expected)) in ordered.iter().enumerate()
        {
            for (break_later, _) in ordered.iter().skip(earlier).skip(1)
            {
                let mut seen = verified_readback();

                break_earlier(&mut seen);
                break_later(&mut seen);

                assert_eq!(plan().verify(&seen), Err(*expected));
            }
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
        // frame and one frame straddles the boundary. The carry runs on frames,
        // so the block that carries the straddling frame is the one holding its
        // last slot, and that block starts its carry one slot early, on the
        // first slot of the frame.
        let plan = plan();

        assert_eq!(plan.block_start(Half::First) % WORDS_PER_FRAME, 0);
        assert_eq!(plan.block_start(Half::Second) % WORDS_PER_FRAME, 1);

        assert_eq!(plan.carry_start(Half::First), 0);
        assert_eq!(plan.carry_start(Half::Second), plan.block_start(Half::Second) - 1);
    }

    #[test]
    fn the_two_carries_of_a_lap_write_the_buffer_once_between_them()
    {
        // The whole of what the frame rule buys. A block of an odd number of
        // words leaves its last slot, the first slot of the straddling frame,
        // to the block after it, which carries that frame whole, and the two
        // come to the lap exactly. A rule that took the frame in both blocks or
        // in neither would write the straddling frame twice or never, and no
        // state of half a frame is held between the two entries either way.
        let plan = plan();
        let first = plan.carry_words(Half::First);
        let second = plan.carry_words(Half::Second);

        assert_eq!(plan.carry_frames(Half::First), 220);
        assert_eq!(plan.carry_frames(Half::Second), 221);
        assert_eq!(first, 440);
        assert_eq!(second, 442);
        assert_eq!(first + second, plan.transfer_items());

        // The second carry starts where the first one stopped, so the two spans
        // meet rather than overlapping or leaving a word between them.
        assert_eq!(plan.carry_start(Half::First) + first, plan.carry_start(Half::Second));
        assert_eq!(plan.carry_start(Half::Second) + second, plan.transfer_items());

        // No span reaches past the end of its own block, so no carry reads a
        // word of a half the receiving stream has not finished when its event
        // is raised. The span of the first block stops one slot short of it.
        assert_eq!(first, plan.block_words() - 1);
        assert_eq!(plan.carry_start(Half::First) + first + 1, plan.block_end(Half::First));
        assert_eq!(plan.carry_start(Half::Second) + second, plan.block_end(Half::Second));
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
        let guard = plan.guard_words();

        for half in [Half::First, Half::Second]
        {
            let carried = plan.carry_words(half);
            let end = shift().destination(plan, plan.carry_start(half) + carried);

            // Every position of the lap is swept, so the boundary is read off
            // the whole domain rather than off the two ends of it. The span the
            // guard has to clear is what the carry WRITES, which on a block of
            // an odd number of words is not the length of the block.
            for step in 0..words
            {
                let position = (end + step) % words;

                assert_eq!
                (
                    plan.block_is_clear(half, position, shift()),
                    step < words - carried - guard,
                    "half {half:?} at position {position}"
                );
            }
        }
    }

    #[test]
    fn the_guard_refuses_every_word_a_carry_writes_and_the_margin_behind_it()
    {
        // The correction this lot carries, measured against the loop instead of
        // against a second derivation of its span. The carry runs on a fresh
        // mock, the mock records which indices it reached, and every one of
        // those indices is then offered to the guard as a read pointer
        // position. A guard reading the BLOCK rather than the span refuses one
        // position of the lap too few on the second carry and one too many on
        // the first, which the count below reads.
        //
        // The count closes the other direction. A guard that refused more than
        // the span and the margin would be one that refuses entries a healthy
        // board produces, and the two readings together leave no slack either
        // way.
        let plan = plan();

        for half in [Half::First, Half::Second]
        {
            let mut interface = MockInput::healthy();

            carry_block(&mut interface, &plan, half, shift(), &mut transparent_chain(), unity());

            let mut written = 0_u32;
            let mut refused = 0_u32;

            for position in 0..TEST_WORDS
            {
                let clear = plan.block_is_clear(half, position, shift());
                let touched = interface.touched.get(position as usize).copied()
                    .unwrap_or(false);

                if touched
                {
                    written = written.saturating_add(1);

                    assert!
                    (
                        !clear,
                        "half {half:?} admits position {position}, which the \
                         carry writes"
                    );
                }

                if !clear
                {
                    refused = refused.saturating_add(1);
                }
            }

            assert_eq!
            (
                written, plan.carry_words(half),
                "the carry of {half:?} reached {written} indices"
            );
            assert_eq!
            (
                refused, written + plan.guard_words(),
                "the guard of {half:?} refuses {refused} positions of the lap, \
                 against {written} the carry writes and a margin of {}",
                plan.guard_words()
            );
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
            let first = shift().destination(plan, plan.carry_start(half));

            for step in 0..plan.carry_words(half)
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
    fn an_entry_on_time_leaves_each_carry_the_clearance_the_margin_is_measured_against()
    {
        // At an entry taken on time the read pointer stands one block past the
        // receiving counter plus the counter distance, and the first word the
        // carry writes stands the shift past the start of the carry. What lies
        // between the two is the clearance the module documentation and the
        // margin name: 433 words on the first block, 432 on the second, whose
        // carry starts one slot before its block. It does not move with the
        // distance a start-up leaves, which is swept, and each of those entries
        // clears the guard.
        let plan = plan();
        let words = plan.transfer_items();

        for offset in LOOP_OFFSETS
        {
            let shift = shift_at(offset);

            for (half, clearance) in [(Half::First, 433), (Half::Second, 432)]
            {
                let position = (plan.block_end(half) + offset) % words;
                let first = shift.destination(plan, plan.carry_start(half));

                assert_eq!
                (
                    (first + words - position) % words,
                    clearance,
                    "the {half:?} carry at a distance of {offset} words"
                );
                assert!
                (
                    plan.block_is_clear(half, position, shift),
                    "the {half:?} entry on time at a distance of {offset} words was refused"
                );
            }
        }
    }

    #[test]
    fn a_buffer_leaving_the_guard_no_room_is_refused()
    {
        // The shortest buffer whose halves split and whose frames are whole. Its
        // span plus margin covers the lap, so no read pointer position would
        // clear a block and every entry would be refused. It is named at the
        // plan rather than discovered at the first event.
        let output = TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            MASTER_BUFFER,
            SLAVE_BUFFER,
            WORDS_PER_FRAME
        );
        let plan = InputPlan::for_transport(&output, INPUT_BUFFER);

        assert_eq!(plan.validate(), Err(InputPlanError::BufferTooShortForGuard));

        // A lap of two frames leaves room, so the bound is read rather than the
        // whole neighbourhood refused.
        let wider = TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            MASTER_BUFFER,
            SLAVE_BUFFER,
            WORDS_PER_FRAME * 2
        );

        assert_eq!(InputPlan::for_transport(&wider, INPUT_BUFFER).validate(), Ok(()));
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

    /// Writes into `master` and `slave` what a carry of `half` through a
    /// TRANSPARENT chain leaves in the two output buffers.
    ///
    /// Every half of the expectation is a closed form, and none of them calls
    /// the code under test. The destination comes from the distance the mock is
    /// armed at, less the pipeline the data crosses, rather than from
    /// `CarryShift`. The value comes from the output stage, since a transparent
    /// cascade changes nothing but the single precision the sample crosses and
    /// the stage is what stands between that and a buffer. The four channels
    /// come from the four constants the fan-out turns.
    ///
    /// The three channels a way reaches therefore hold three DIFFERENT words,
    /// each at the alignment of its own way, so a fan-out that sent one way to
    /// the channel of another shows here as well as in the response.
    ///
    /// The tests that use this measure WHERE a word lands and which channels it
    /// reaches. What the real cascades do to a sample is measured against
    /// `cascade_magnitude` and against a straight walk of the same samples,
    /// neither of which knows anything about this loop.
    fn carry_into_image
    (
        half: Half,
        master: &mut [u32; TEST_WORDS as usize],
        slave: &mut [u32; TEST_WORDS as usize]
    )
    {
        let plan = plan();

        for frame in 0..plan.carry_frames(half)
        {
            let index = plan.carry_start(half) + frame * WORDS_PER_FRAME;
            let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;
            let ways = staged(0xC0DE_0000 | index, 0xC0DE_0000 | (index + RIGHT_SLOT));

            place(master, at + LOW_SLOT, ways.low());
            place(master, at + MID_SLOT, ways.mid());
            place(slave, at + HIGH_SLOT, ways.high());
            place(slave, at + SPARE_SLOT, SILENT_WORD);
        }
    }

    /// Writes `word` into `buffer` at `index`.
    fn place(buffer: &mut [u32; TEST_WORDS as usize], index: u32, word: u32)
    {
        if let Some(slot) = buffer.get_mut(index as usize)
        {
            *slot = word;
        }
    }

    /// Returns what the two output buffers hold once the frames of `half` have
    /// been carried onto an empty pair of them through a transparent chain.
    fn carried_image(half: Half)
        -> ([u32; TEST_WORDS as usize], [u32; TEST_WORDS as usize])
    {
        let mut master = [0_u32; TEST_WORDS as usize];
        let mut slave = [0_u32; TEST_WORDS as usize];

        carry_into_image(half, &mut master, &mut slave);

        (master, slave)
    }

    /// Fails unless the two output buffers of `interface` hold `master` and
    /// `slave`.
    fn assert_buffers
    (
        interface: &MockInput,
        master: &[u32; TEST_WORDS as usize],
        slave: &[u32; TEST_WORDS as usize],
        what: &str
    )
    {
        for index in 0..TEST_WORDS as usize
        {
            assert_eq!
            (
                interface.master.get(index).copied(),
                master.get(index).copied(),
                "master word {index} {what}"
            );
            assert_eq!
            (
                interface.slave.get(index).copied(),
                slave.get(index).copied(),
                "slave word {index} {what}"
            );
        }
    }

    #[test]
    fn a_carry_moves_every_frame_of_its_block_to_the_position_it_was_sent_from()
    {
        for half in [Half::First, Half::Second]
        {
            let mut interface = MockInput::healthy();
            seed_source(&mut interface);

            carry_block(&mut interface, &plan(), half, shift(), &mut transparent_chain(), unity());

            let frames = plan().carry_frames(half);

            assert_eq!
            (
                interface.written, frames * CARRIED_CHANNELS,
                "a carry of {half:?} made {} writes, a block is {frames} frames \
                 into each of {CARRIED_CHANNELS} channels",
                interface.written
            );

            let (master, slave) = carried_image(half);

            assert_buffers(&interface, &master, &slave, carry_case(half));
        }
    }

    /// Returns the words a failure of a carry test names its case by.
    fn carry_case(half: Half) -> &'static str
    {
        match half
        {
            Half::First => "on the first block",
            Half::Second => "on the second block",
        }
    }

    /// A gain source that records what `serve` asked it for, and hands out
    /// `unity`.
    #[derive(Default)]
    struct Recording
    {
        calls: u32,
        frames: [u32; 2],
    }

    impl BlockGain for Recording
    {
        fn block(&mut self, frames: u32) -> Applied
        {
            if let Some(slot) = self.frames.get_mut(self.calls as usize)
            {
                *slot = frames;
            }
            self.calls += 1;
            unity()
        }
    }

    #[test]
    fn a_carry_scales_each_frame_by_the_gain_it_walks()
    {
        // The gain walks from one end to the other across the block, so a
        // carry that applied either end to the whole block, or that walked the
        // frames one place off, leaves a different word on every frame. The
        // expectation takes the sample by the route `carry_into_image` takes,
        // and scales it by the gain of ITS frame index.
        let pairs = [(0.0, 1.0), (1.0, 0.0), (0.25, 0.75), (0.5, 0.5), (0.0, 0.0)];

        for half in [Half::First, Half::Second]
        {
            for (start, end) in pairs
            {
                let applied = Applied::between_for_test(start, end);
                let mut interface = MockInput::healthy();
                seed_source(&mut interface);

                carry_block(&mut interface, &plan(), half, shift(), &mut transparent_chain(), applied);

                let plan = plan();
                let frames = plan.carry_frames(half);
                let mut master = [0_u32; TEST_WORDS as usize];
                let mut slave = [0_u32; TEST_WORDS as usize];

                for frame in 0..frames
                {
                    let index = plan.carry_start(half) + frame * WORDS_PER_FRAME;
                    let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;
                    let sample = source_sample(0xC0DE_0000 | index, 0xC0DE_0000 | (index + RIGHT_SLOT));
                    let gain = applied.gain_at(frame as usize, frames as usize);
                    let ways = staged_sample(sample * gain);

                    place(&mut master, at + LOW_SLOT, ways.low());
                    place(&mut master, at + MID_SLOT, ways.mid());
                    place(&mut slave, at + HIGH_SLOT, ways.high());
                    place(&mut slave, at + SPARE_SLOT, SILENT_WORD);
                }

                assert_buffers(&interface, &master, &slave, carry_case(half));

                if start.to_bits() != end.to_bits()
                {
                    let first = (plan.carry_start(half) + plan.arming_position()
                        - PIPELINE_WORDS) % TEST_WORDS;
                    let last = (first + (frames - 1) * WORDS_PER_FRAME) % TEST_WORDS;

                    assert_ne!
                    (
                        interface.master.get(first as usize),
                        interface.master.get(last as usize),
                        "a ramp from {start} to {end} left the first and the last \
                         frame alike"
                    );
                }
            }
        }
    }

    #[test]
    fn a_silent_gain_keeps_every_history_of_the_chain_at_rest()
    {
        // `Applied::preset` moves coefficients only under a gain of zero at both
        // ends of a buffer, on the ground that no signal then enters a section.
        // Two full laps of a full scale tone at that gain have to leave every
        // history, and the limiter, exactly where a built chain opens.
        let plan = plan();
        let mut served = chain();
        let mut interface = MockInput::healthy();
        let silent = Applied::between_for_test(0.0, 0.0);

        for lap in 0..2
        {
            for index in 0..TEST_WORDS
            {
                place(&mut interface.source, index, sine_word(lap, index, 30, f64::from(i32::MAX)));
            }

            for half in [Half::First, Half::Second]
            {
                carry_block(&mut interface, &plan, half, shift(), &mut served, silent);
            }
        }

        assert!(served == chain(), "a silent gain let the tone into the chain");
        assert!(interface.master.iter().all(|word| *word == 0), "a silent gain left a word");
        assert!(interface.slave.iter().all(|word| *word == 0), "a silent gain left a word");
    }

    #[test]
    fn serving_asks_the_gain_once_for_the_frames_of_the_block_it_carries()
    {
        for half in [Half::First, Half::Second]
        {
            let mut interface = MockInput::healthy();
            interface.next_event = half;
            let mut asked = Recording::default();

            assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
            assert_eq!
            (
                serve(&mut interface, &plan(), shift(), &mut transparent_chain(), &mut asked),
                Ok(())
            );
            assert_eq!(asked.calls, 1, "the {half:?} block asked the gain {} times", asked.calls);
            assert_eq!(asked.frames.first().copied(), Some(plan().carry_frames(half)));
        }
    }

    #[test]
    fn a_walked_gain_never_leaves_the_unit_range()
    {
        // `carry_block` states the walked gain stands in [0, 1] whenever its
        // two ends do. This drives the ends over the edges of the range, the
        // values next to them, and a spread of values in between, and reads
        // every frame of both block lengths the served plan carries.
        let mut ends = std::vec![
            0.0_f32,
            1.0,
            f32::MIN_POSITIVE,
            f32::from_bits(1),
            1.0_f32.next_down(),
            0.5,
            0.5_f32.next_down(),
            0.5_f32.next_up(),
            1e-7,
        ];
        let mut seed = 0x2545_F491_u32;

        for _ in 0..200
        {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            ends.push(f32::from(u16::try_from(seed >> 16).unwrap_or(0)) / 65_536.0);
        }

        let lengths = [plan().carry_frames(Half::First), plan().carry_frames(Half::Second)];

        for &start in &ends
        {
            for &end in &ends
            {
                let applied = Applied::between_for_test(start, end);

                for frames in lengths
                {
                    for frame in 0..=frames
                    {
                        let gain = applied.gain_at(frame as usize, frames as usize);

                        assert!
                        (
                            (0.0..=1.0).contains(&gain),
                            "ends {start} and {end} walked to {gain} at frame {frame} \
                             of {frames}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn a_carry_writes_nothing_outside_the_span_it_was_given()
    {
        // The two spans together cover the lap, so carrying one and finding a
        // word standing where the other one lands is what a swapped block start
        // looks like. The destination wraps the end of the buffer, so the
        // stretch that must stay empty is walked rather than sliced. The master
        // buffer is the one read, since the channel no way drives is written
        // with silence and a silent word cannot be told from an untouched one.
        let mut interface = MockInput::healthy();
        seed_source(&mut interface);

        carry_block(&mut interface, &plan(), Half::Second, shift(), &mut transparent_chain(), unity());

        let plan = plan();
        let first = (plan.carry_start(Half::First) + plan.arming_position()
            - PIPELINE_WORDS) % TEST_WORDS;

        for step in 0..plan.carry_words(Half::First)
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

        // One chain crosses both halves, so this walks the block boundary as
        // well as the lap: the history the second carry opens on is the history
        // the first left.
        let mut running = transparent_chain();

        carry_block(&mut interface, &plan(), Half::First, shift(), &mut running, unity());
        carry_block(&mut interface, &plan(), Half::Second, shift(), &mut running, unity());

        assert_eq!
        (
            interface.written, LAP_FRAMES * CARRIED_CHANNELS,
            "the two halves left {} writes, a lap is {LAP_FRAMES} frames into \
             each of {CARRIED_CHANNELS} channels",
            interface.written
        );

        let mut master = [0_u32; TEST_WORDS as usize];
        let mut slave = [0_u32; TEST_WORDS as usize];

        carry_into_image(Half::First, &mut master, &mut slave);
        carry_into_image(Half::Second, &mut master, &mut slave);

        assert_buffers(&interface, &master, &slave, "after a whole lap");

        // Every word of both buffers was reached, and the two do not hold the
        // same thing: the slave carries the high way against the mid way of the
        // master, and its spare channel is silent while no channel of the master
        // is. A fan-out that wrote one value to every channel would pass every
        // reading above and fail this.
        assert!(interface.touched.iter().all(|seen| *seen), "a word of the lap went unwritten");
        assert_ne!(interface.master, interface.slave);
    }

    #[test]
    fn the_limiter_runs_on_from_block_to_block_at_the_period_of_the_caller()
    {
        // The average of the limiter is state of the chain and runs on from one
        // block to the next. The blocks alternate at 220 and 221 frames, as the
        // plan cuts them, over one second of a full scale tone loud enough to
        // take the gain down, and every word of every block is read against a
        // straight walk of the same samples through the output stage with ONE
        // limiter, which knows nothing of blocks. A limiter rebuilt at a
        // boundary, or at a frame, parts from the walk there.
        //
        // The designed high cascade answers at or under unity, so with the trim
        // in place no source takes the long run mean of the way near the
        // threshold. The high cascade here is two sections of a gain that
        // takes most of the trim back: the tone reaches the limiter at 0.954
        // of full scale, 117 W.
        const LAPS: u32 = 100;
        const TONE_CYCLES: u32 = 30;
        const HIGH_SECTION_GAIN: f32 = 2.25;

        let plan = plan();
        let loud = crate::filter::gain_section_for_test(HIGH_SECTION_GAIN);
        let mut chain = FilterChain
        {
            high: HighCascade::of_section(loud),
            ..FilterChain::transparent()
        };
        let mut walked_high = HighCascade::of_section(loud);
        let mut walked = ThermalLimiter::at_rest();
        let mut interface = MockInput::healthy();
        let mut lengths = [false; 2];
        let mut joined_under_unity = 0_u32;

        for lap in 0..LAPS
        {
            for index in 0..TEST_WORDS
            {
                place(&mut interface.source, index, sine_word(lap, index, TONE_CYCLES, f64::from(i32::MAX)));
            }

            for half in [Half::First, Half::Second]
            {
                carry_block(&mut interface, &plan, half, shift(), &mut chain, unity());

                let frames = plan.carry_frames(half);

                match frames
                {
                    220 => lengths[0] = true,
                    221 => lengths[1] = true,
                    _ => panic!("the plan cut a block of {frames} frames"),
                }

                if walked.current_gain_for_test() < 1.0
                {
                    joined_under_unity += 1;
                }

                for frame in 0..frames
                {
                    let index = plan.carry_start(half) + frame * WORDS_PER_FRAME;
                    let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;
                    let sample = frame_sample(&interface.source, index);
                    let want = WayWords::of
                    (
                        WaySamples { low: sample, mid: sample, high: walked_high.step(sample) },
                        &mut walked
                    );

                    for (buffer, slot, word) in
                    [
                        (&interface.slave, HIGH_SLOT, want.high()),
                        (&interface.master, LOW_SLOT, want.low()),
                        (&interface.master, MID_SLOT, want.mid()),
                    ]
                    {
                        assert_eq!
                        (
                            carried_word(buffer, at + slot),
                            word,
                            "slot {slot} of frame {frame} of the {half:?} block of lap {lap} \
                             parted from the walk"
                        );
                    }
                }
            }
        }

        assert_eq!(lengths, [true, true], "the run did not cross blocks of both lengths");

        // The gain stood under unity at most of the joins, so the joins were
        // read while the limiter moved and not only while it stood at rest.
        let gain = walked.current_gain_for_test();

        assert!(gain < 0.7, "the tone left the gain at {gain}");
        assert!(joined_under_unity > LAPS, "the gain stood under unity at {joined_under_unity} joins");
    }

    #[test]
    fn serving_consecutive_events_runs_the_limiter_and_every_history_on_from_one_event_to_the_next()
    {
        // The test above calls the carry, and the handler calls `serve`. What
        // the machine runs is therefore a chain lent to `serve` over event
        // after event, and a `serve` that carried a copy of it, or rebuilt its
        // limiter at an entry, answers the one entry it is given correctly and
        // leaves the chain of the handler where it found it.
        //
        // So this serves 7.7 seconds of events of both halves, 220 and 221
        // frames, on a chain whose low and mid ways are the designed cascades
        // and whose high way carries two sections of a gain of 4.75, which
        // puts a full scale tone at the limiter at 4.25 of full scale, past its
        // input bound. The tone rises from silence through the threshold to
        // 117 W, holds there and eases to 81 W. A full scale square then holds
        // every sample at the input bound for 3 seconds, which takes the
        // average to the square of that bound, the largest it reaches, and the
        // gain to the law there, the lowest it follows. The drive then steps
        // down through a full scale tone and 117 W to a silence that takes the
        // gain back to unity. The state of the limiter therefore crosses its
        // whole reachable range down and back up, and a reset of the chain
        // keyed on any level of the gain or of the average it holds lands
        // inside this run.
        //
        // After every event the WHOLE chain is compared against a straight
        // walk of the same samples, which knows nothing of events: the average
        // and the gain of the limiter, the history of every section of the
        // three ways, and any field a later form of the chain adds. Every word
        // of the block is read against the words of that walk as well, the
        // first frame of the block included, which is the join.
        //
        // A reset driven by state outside the chain, a static counter for one,
        // with a period longer than the 1540 events served here passes every
        // reading below.
        const LAPS: u32 = 770;

        let plan = plan();
        let loud = HighCascade::of_section(crate::filter::gain_section_for_test(SERVED_SECTION_GAIN));
        let mut served = FilterChain { high: loud, ..chain() };
        let at_rest = FilterChain { high: loud, ..chain() };
        let mut walked = FilterChain { high: loud, ..chain() };
        let mut interface = MockInput::healthy();
        let mut lengths = [false; 2];
        let mut after_held = 0.0_f32;
        let mut after_eased = 0.0_f32;
        let mut lowest = 1.0_f32;
        let mut largest = 0.0_f32;
        let mut limiter_moved = false;
        let mut low_moved = false;
        let mut mid_moved = false;

        assert_eq!(bring_up(&mut interface, &plan, &waits()), Ok(shift()));

        for lap in 0..LAPS
        {
            for index in 0..TEST_WORDS
            {
                place(&mut interface.source, index, served_word(lap, index));
            }

            for half in [Half::First, Half::Second]
            {
                interface.next_event = half;

                assert_eq!
                (
                    serve(&mut interface, &plan, shift(), &mut served, &mut Steady),
                    Ok(()),
                    "the {half:?} event of lap {lap} was refused"
                );

                let frames = plan.carry_frames(half);

                match frames
                {
                    220 => lengths[0] = true,
                    221 => lengths[1] = true,
                    _ => panic!("the plan cut a block of {frames} frames"),
                }

                walk_served_block(&mut walked, &interface, half, lap);

                assert!
                (
                    served == walked,
                    "the chain the {half:?} event of lap {lap} left parted from the walk: its \
                     limiter holds {:?} against {:?}, and its histories agree with the walk: {}",
                    served.limiter,
                    walked.limiter,
                    served.low == walked.low && served.mid == walked.mid && served.high == walked.high
                );

                let gain = served.limiter.current_gain_for_test();

                lowest = lowest.min(gain);
                largest = largest.max(served.limiter.current_average_for_test());
                limiter_moved |= served.limiter != at_rest.limiter;
                low_moved |= served.low != at_rest.low;
                mid_moved |= served.mid != at_rest.mid;

                match (lap, half)
                {
                    (99, Half::Second) => after_held = gain,
                    (169, Half::Second) => after_eased = gain,
                    _ => {}
                }
            }
        }

        assert_eq!(lengths, [true, true], "the run did not serve blocks of both lengths");

        // The state the readings above compared moved over the operating
        // points the run names: the gain held near the law of 117 W and rose
        // towards that of 81 W. The square took the average within a thousandth
        // of the square of the input bound, which no input passes, and the
        // gain within a thousandth of the law there, `sqrt(threshold) / bound`.
        // The detector stops short of that average by its stall, under a
        // thousandth. The silence took the gain to unity or to the float one
        // step under it: at a ratio of one the step rounds that gain to a tie
        // and leaves it there. The history of each designed way moved too.
        let gain = served.limiter.current_gain_for_test();
        let (threshold, bound) = ThermalLimiter::threshold_and_input_bound_for_test();
        let ceiling = f64::from(bound) * f64::from(bound);
        let floor = sqrt(f64::from(threshold)) / f64::from(bound);

        assert!((0.6..0.75).contains(&after_held), "the 117 W hold left the gain at {after_held}");
        assert!((0.72..0.82).contains(&after_eased), "the 81 W hold left the gain at {after_eased}");
        assert!(after_eased > after_held, "easing to 81 W took the gain from {after_held} to {after_eased}");
        assert!
        (
            f64::from(largest) >= ceiling * (1.0 - 1.0e-3) && f64::from(largest) <= ceiling,
            "the square took the average to {largest}, against a ceiling of {ceiling}"
        );
        assert!
        (
            (f64::from(lowest) / floor - 1.0).abs() < 1.0e-3,
            "the square took the gain no lower than {lowest}, against a floor of {floor}"
        );
        assert!(gain >= 1.0 - f32::EPSILON / 2.0, "the silence left the gain at {gain}");
        assert!(limiter_moved, "the limiter never left its rest");
        assert!(low_moved, "the history of the low way never moved");
        assert!(mid_moved, "the history of the mid way never moved");
    }

    /// Gain of each of the two sections the served high way carries.
    ///
    /// A full scale tone reaches the limiter at 4.25 of full scale through it,
    /// which is 2332 W, and a full scale square holds every sample past the
    /// input bound of the limiter.
    const SERVED_SECTION_GAIN: f32 = 4.75;

    /// Watts a full scale tone stands for at the limiter through
    /// `SERVED_SECTION_GAIN`, before the input bound.
    const SERVED_FULL_SCALE_WATTS: f64 = 2332.1;

    /// Returns the word the served drive holds at `index` of lap `lap`.
    ///
    /// A tone of 30 periods a lap rises from silence to 117 W over 30 laps,
    /// holds 117 W to lap 100 and 81 W to lap 170. A full scale square of as
    /// many periods holds to lap 470, then a full scale tone to lap 520, 117 W
    /// to lap 570 and silence after that.
    fn served_word(lap: u32, index: u32) -> u32
    {
        const CYCLES: u32 = 30;

        let level = match lap
        {
            0..30 => sqrt(117.0 / SERVED_FULL_SCALE_WATTS) * f64::from(lap) / 30.0,
            30..100 | 520..570 => sqrt(117.0 / SERVED_FULL_SCALE_WATTS),
            100..170 => sqrt(81.0 / SERVED_FULL_SCALE_WATTS),
            170..470 => return square_word(lap, index, CYCLES),
            470..520 => 1.0,
            _ => 0.0,
        };

        sine_word(lap, index, CYCLES, f64::from(i32::MAX) * level)
    }

    /// Walks the frames the `half` event of lap `lap` carried through
    /// `walked`, and fails unless every word of the block `interface` holds is
    /// the word that walk answers.
    fn walk_served_block(walked: &mut FilterChain, interface: &MockInput, half: Half, lap: u32)
    {
        let plan = plan();

        for frame in 0..plan.carry_frames(half)
        {
            let index = plan.carry_start(half) + frame * WORDS_PER_FRAME;
            let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;
            let sample = frame_sample(&interface.source, index);
            let want = WayWords::of
            (
                WaySamples
                {
                    low: walked.low.step(sample),
                    mid: walked.mid.step(sample),
                    high: walked.high.step(sample),
                },
                &mut walked.limiter
            );

            for (buffer, slot, word) in
            [
                (&interface.master, LOW_SLOT, want.low()),
                (&interface.master, MID_SLOT, want.mid()),
                (&interface.slave, HIGH_SLOT, want.high()),
            ]
            {
                assert_eq!
                (
                    carried_word(buffer, at + slot),
                    word,
                    "slot {slot} of frame {frame} of the {half:?} event of lap {lap} \
                     parted from the walk"
                );
            }
        }
    }

    #[test]
    fn every_chain_carries_a_limiter_and_an_unbuilt_chain_carries_a_silent_one()
    {
        // `silent` is a constant, so parking it runs no build that could
        // refuse.
        const PARKED: FilterChain = FilterChain::silent();

        assert_eq!(PARKED.limiter, ThermalLimiter::silent());
        assert_eq!(chain().limiter, ThermalLimiter::at_rest());

        // The silent limiter stops the high way on its own, with cascades that
        // pass every sample, and leaves the other two ways alone.
        let mut stopped = FilterChain
        {
            limiter: ThermalLimiter::silent(),
            ..FilterChain::transparent()
        };

        for word in [i32::MAX, i32::MIN, 0x4000_0000, -0x4000_0000, 1]
        {
            let ways = stopped.carry(word.cast_unsigned(), word.cast_unsigned(), 1.0);

            assert_eq!(ways.high(), 0, "a silent limiter let {word} through");
            assert_eq!(ways.low(), staged(word.cast_unsigned(), word.cast_unsigned()).low());
        }
    }

    /// Amplitude the response sweep drives, as a sample.
    ///
    /// Under half of full scale, so the pass band of a way leaves room for the
    /// overshoot and nothing in the sweep meets the saturation the tests below
    /// it measure on purpose.
    const SWEEP_AMPLITUDE: f64 = 1.0e9;

    /// Laps a sweep runs before it reads the one it measures.
    ///
    /// The subsonic high-pass of the low way places its slowest pole at radius
    /// 0.99837, so what is left of a transient after `n` samples is that factor
    /// to the power of `n`. Forty laps are 17640 samples, which is e to the
    /// minus twenty eight, so what the forty first lap carries is the steady
    /// state and not the start-up. The other two ways settle orders faster and
    /// ride on the same run.
    const SWEEP_WARM_LAPS: u32 = 40;

    /// Words of amplitude the measured response of the low way may stand from
    /// the amplitude its design calls for, at the drive of the sweep.
    ///
    /// MEASURED over the whole sweep rather than read at one point, as the
    /// departure of the bin the drive fills. The worst of the set is 86997
    /// words at 100 Hz, and this is a little over twice it, which is what
    /// covers the frequencies between the ones the sweep stands on.
    ///
    /// What it covers is the single precision difference equation of a cascade
    /// against a double precision evaluation of the SAME coefficients, so it is
    /// arithmetic rather than design. The low way carries it two orders worse
    /// than the other two ways because the subsonic high-pass computes a second
    /// difference of three terms that nearly cancel: at 100 Hz on a drive of a
    /// billion words those three stand within a part in five thousand of each
    /// other, so four of the seven digits the format holds go in the
    /// subtraction, and the poles at radius 0.99606 and 0.99837 then amplify
    /// what is left of the rounding.
    ///
    /// 86997 words against the 785 million the design calls for is 0.000963 dB.
    /// Read at the SAME frequency, the narrowing of that same filter costs
    /// 0.00404 dB, so the arithmetic adds a quarter of what the narrowing
    /// already costs there. The 0.01006 dB that filter is better known for is
    /// its narrowing at 60 Hz, and the two do not compare.
    const LOW_RESPONSE_WORDS: f64 = 200_000.0;

    /// The same bound for the mid way. MEASURED at 1009 words, at 400 Hz.
    const MID_RESPONSE_WORDS: f64 = 2_400.0;

    /// The same bound for the high way. MEASURED at 41 words, at 1800 Hz.
    ///
    /// A way is read at the level it leaves the machine at, so the alignment
    /// brings its departure down with its amplitude and this bound stands two
    /// thousand times under the one the low way needs.
    const HIGH_RESPONSE_WORDS: f64 = 100.0;

    /// Periods per lap of the full scale square the low way is driven with
    /// where the bound of the conversion is measured.
    ///
    /// A square wave in a way's own pass band is what takes the recurrence of
    /// that way past the format, on the overshoot of its edges, and the three
    /// pass bands are disjoint, so no single drive reaches all three: a 100 Hz
    /// square leaves the mid way inside the format for the whole lap, a 3000 Hz
    /// square leaves the low way two orders under it. Each way is therefore
    /// driven in its own band and read on its own channel.
    const LOW_SQUARE_CYCLES: u32 = 1;

    /// The same drive for the mid way, 700 Hz, between its two corners.
    const MID_SQUARE_CYCLES: u32 = 7;

    /// The same drive for the high way, 3000 Hz, above its corner.
    const HIGH_SQUARE_CYCLES: u32 = 30;

    /// Words a carried sample of the low way may stand from the double
    /// precision evaluation of the same recurrence, over its own square drive.
    ///
    /// MEASURED over the samples this way carries whole, on the whole lap, at
    /// 304746 words. This is a little over twice it.
    ///
    /// What separates a bound at the conversion from a bound INSIDE the feedback
    /// is the measurement beside it: the same drive through a cascade whose
    /// feedback is held to the format, section by section, moves a sample of
    /// this way by 1713488681 words, 2636 times the bound here.
    const LOW_FEEDBACK_WORDS: f64 = 650_000.0;

    /// The same bound for the mid way. MEASURED at 21216 words, against
    /// 2447593060 for a bounded feedback.
    const MID_FEEDBACK_WORDS: f64 = 48_000.0;

    /// The same bound for the high way. MEASURED at 419 words, against
    /// 1128614437 for a bounded feedback.
    const HIGH_FEEDBACK_WORDS: f64 = 950.0;

    /// Returns the word every slot of a frame holds at `index` of lap `lap`.
    ///
    /// The sample number of a frame is the lap it belongs to times the frames a
    /// lap holds, plus the position of the frame, so a lap joins the one before
    /// it. `cycles` periods fill one lap exactly, which is what puts the whole
    /// of the signal in one bin of the measurement below.
    ///
    /// Every slot of the frame carries the same sample, which is what a sender
    /// of one channel puts on the link, and the mean the chain reads is then
    /// that sample. The tests that measure the mean drive the two slots apart
    /// on purpose.
    fn sine_word(lap: u32, index: u32, cycles: u32, amplitude: f64) -> u32
    {
        let angle = TAU * f64::from(cycles) * f64::from(sample_of(lap, index))
            / f64::from(LAP_FRAMES);

        (round(amplitude * sin(angle)) as i32).cast_unsigned()
    }

    /// Returns the word a full scale square of `cycles` periods a lap holds at
    /// `index` of lap `lap`.
    ///
    /// It swings the whole format at every edge. In the pass band of a way the
    /// overshoot of those edges takes the recurrence of that way past the
    /// format. The input that puts the loudest peak on a way is another one: it
    /// carries the sign pattern of the impulse response of that way read
    /// backwards.
    fn square_word(lap: u32, index: u32, cycles: u32) -> u32
    {
        let half = LAP_FRAMES / (2 * cycles);

        if (sample_of(lap, index) / half).is_multiple_of(2)
        {
            i32::MIN.cast_unsigned()
        }
        else
        {
            i32::MAX.cast_unsigned()
        }
    }

    /// Returns the sample number the frame holding `index` carries in lap `lap`.
    fn sample_of(lap: u32, index: u32) -> u32
    {
        lap * LAP_FRAMES + index / WORDS_PER_FRAME
    }

    /// Carries `laps` laps of blocks, filling each source half from `word`
    /// before it is carried, and leaves in `master` and `slave` what each
    /// output buffer holds at the destination of every source index of the LAST
    /// lap.
    ///
    /// A half is filled whole before its carry and the half after it is not,
    /// which is where the receiving stream stands at the event of that half. So
    /// a carry reading a word past its half reads what the lap before left
    /// there. Each destination is read right after the carry that writes it.
    ///
    /// It drives the source rather than closing the link on the output, which is
    /// the path the product runs: a source this board does not write, one chain,
    /// one pair of output buffers. A closed link would cross the chain once a lap
    /// and take any signal to silence.
    ///
    /// The two images are indexed by SOURCE index, so the slot of an index is
    /// the slot of the output frame the fan-out put that frame in, and the four
    /// constants of the fan-out are what a reader of a measurement below looks
    /// up.
    fn run_laps<F>
    (
        chain: &mut FilterChain,
        laps: u32,
        word: F,
        master: &mut [u32; TEST_WORDS as usize],
        slave: &mut [u32; TEST_WORDS as usize]
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

                carry_block(&mut interface, &plan, half, shift(), chain, unity());

                for step in 0..plan.carry_words(half)
                {
                    let index = plan.carry_start(half) + step;
                    let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;

                    place(master, index, carried_word(&interface.master, at));
                    place(slave, index, carried_word(&interface.slave, at));
                }
            }
        }
    }

    /// Returns the word `buffer` holds at `at`.
    fn carried_word(buffer: &[u32; TEST_WORDS as usize], at: u32) -> u32
    {
        buffer.get(at as usize).copied().unwrap_or(0)
    }

    /// Returns the amplitude one slot of `buffer` carries at `cycles` periods
    /// per lap.
    ///
    /// One bin of a discrete Fourier transform over a whole lap of one slot. A
    /// sine of `cycles` periods a lap fills that bin and nothing else, so this
    /// reads its amplitude with no window and no leakage.
    fn amplitude_at(buffer: &[u32; TEST_WORDS as usize], slot: u32, cycles: u32) -> f64
    {
        let mut real = 0.0_f64;
        let mut imaginary = 0.0_f64;

        for frame in 0..LAP_FRAMES
        {
            let index = frame * WORDS_PER_FRAME + slot;
            let value = f64::from(carried_word(buffer, index).cast_signed());
            let angle = -TAU * f64::from(cycles) * f64::from(frame) / f64::from(LAP_FRAMES);

            real += value * cos(angle);
            imaginary += value * sin(angle);
        }

        2.0 * sqrt(real * real + imaginary * imaginary) / f64::from(LAP_FRAMES)
    }

    /// Returns the magnitude `sections` give at `cycles` periods per lap.
    fn designed_magnitude(sections: &[Biquad], cycles: u32) -> f64
    {
        let frequency_hz =
            f64::from(cycles) * f64::from(SAMPLE_RATE_HZ) / f64::from(LAP_FRAMES);

        cascade_magnitude(sections, frequency_hz as f32, SAMPLE_RATE_HZ).unwrap_or(0.0)
    }

    /// Periods per lap the sweep walks.
    ///
    /// One lap holds 441 frames at 44100 Hz, so a period count is a frequency in
    /// hundreds of hertz and the set below runs 100 Hz to 21900 Hz. It crosses
    /// both crossover corners, stands either side of each, and reaches the last
    /// bin under Nyquist, so the bound it produces is a bound over the band and
    /// not a reading at one point.
    ///
    /// 1800 Hz is in it because that is where the high way departs furthest from
    /// its design, 41 words against the 34 of the next worst point, and
    /// because it is the lowest frequency the compression driver is rated from.
    /// A set that stepped over it would name a bound the band breaks.
    const SWEEP_CYCLES: [u32; 19] =
        [1, 2, 3, 4, 5, 7, 10, 14, 18, 20, 26, 30, 40, 45, 70, 110, 160, 200, 219];

    #[test]
    #[expect
    (
        clippy::float_cmp,
        reason = "the channel no way drives is written with the silent word, so \
                  the amplitude of its bin is exactly zero and a margin there \
                  would admit a channel carrying a signal"
    )]
    fn the_response_of_each_way_matches_the_designed_magnitude()
    {
        // The whole point of the chain, measured against an evaluation of the
        // same coefficients that knows nothing of this loop.
        //
        // The reading is an AMPLITUDE in words rather than a ratio, and that is
        // the instrument rather than a convenience. A ratio divides by the
        // magnitude the design calls for, and at the top of the sweep the low
        // way is 170 dB down while single precision holds about 144, so the
        // ratio there divides the arithmetic noise of the format by a number
        // beneath it and answers a departure of a hundred million. Read as an
        // amplitude, the same point says the way answered 603 words where the
        // design called for none, which is what it did.
        //
        // Each way is read off the channel the fan-out puts it on, so a chain
        // that ran the right cascades and sent them to the wrong channels fails
        // this rather than passing on three shapes nobody located.
        //
        // The design a way is read against carries the alignment of that way,
        // because the alignment is part of what leaves the machine. A stage that
        // dropped it, or that put one way at the gain of another, moves the
        // whole band of that way and fails every point of the sweep on it.
        let mut points = 0_u32;

        for cycles in SWEEP_CYCLES
        {
            let mut master = [0_u32; TEST_WORDS as usize];
            let mut slave = [0_u32; TEST_WORDS as usize];

            run_laps
            (
                &mut chain(),
                SWEEP_WARM_LAPS + 1,
                |lap, index| sine_word(lap, index, cycles, SWEEP_AMPLITUDE),
                &mut master,
                &mut slave
            );

            let mut driven = [0_u32; TEST_WORDS as usize];

            for index in 0..TEST_WORDS
            {
                place
                (
                    &mut driven,
                    index,
                    sine_word(SWEEP_WARM_LAPS, index, cycles, SWEEP_AMPLITUDE)
                );
            }

            let drive = amplitude_at(&driven, LOW_SLOT, cycles);

            for way in ways_under_test()
            {
                let carried = match way.role
                {
                    BlockRole::Master => &master,
                    BlockRole::Slave => &slave,
                };

                let designed = designed_magnitude(way.active(), cycles) * drive * way.gain();
                let measured = amplitude_at(carried, way.slot, cycles);
                let gap = (measured - designed).abs();

                points = points.saturating_add(1);

                assert!
                (
                    gap <= way.response_words,
                    "{} at {cycles}00 Hz answered {measured} words where the \
                     design calls for {designed}, a gap of {gap} words",
                    way.name
                );
            }

            // The channel no way drives stays silent whatever the drive.
            assert_eq!
            (
                amplitude_at(&slave, SPARE_SLOT, cycles),
                0.0,
                "the spare channel answered at {cycles}00 Hz"
            );
        }

        assert_eq!
        (
            points,
            SWEEP_CYCLES.len() as u32 * 3,
            "a point of the sweep went unmeasured"
        );
    }

    #[test]
    #[expect
    (
        clippy::float_cmp,
        reason = "what this reads is that two channels are not the SAME \
                  amplitude, so a margin would be a second claim about how far \
                  apart the three designs stand and the sweep beside this one \
                  already carries that"
    )]
    fn the_three_ways_of_the_chain_are_the_three_of_the_crossover()
    {
        // The sweep above measures each way against its own design, so three
        // ways all running one cascade would pass it only if the three designs
        // agreed. They do not, and this reads that directly: the three answer
        // different amplitudes either side of both corners, so a chain that ran
        // one cascade three times, or put two ways on one channel, fails here.
        for cycles in [3_u32, 30, 200]
        {
            let mut master = [0_u32; TEST_WORDS as usize];
            let mut slave = [0_u32; TEST_WORDS as usize];

            run_laps
            (
                &mut chain(),
                SWEEP_WARM_LAPS + 1,
                |lap, index| sine_word(lap, index, cycles, SWEEP_AMPLITUDE),
                &mut master,
                &mut slave
            );

            let low = amplitude_at(&master, LOW_SLOT, cycles);
            let mid = amplitude_at(&master, MID_SLOT, cycles);
            let high = amplitude_at(&slave, HIGH_SLOT, cycles);

            assert!(low != mid, "the low and mid channels answered alike at {cycles}00 Hz");
            assert!(mid != high, "the mid and high channels answered alike at {cycles}00 Hz");
            assert!(low != high, "the low and high channels answered alike at {cycles}00 Hz");
        }
    }

    /// Returns a word no lap repeats at `index`, eleven and a half periods a
    /// lap at 0.6 of `SWEEP_AMPLITUDE`.
    ///
    /// Half a period over, so the word a position holds turns over from one lap
    /// to the next, and a reading of a position the source has not yet written
    /// this lap answers the lap before rather than the same value.
    fn turning_word(lap: u32, index: u32) -> u32
    {
        let angle = TAU * 11.5 * f64::from(sample_of(lap, index)) / f64::from(LAP_FRAMES);

        (round(0.6 * SWEEP_AMPLITUDE * sin(angle)) as i32).cast_unsigned()
    }

    #[test]
    #[expect
    (
        clippy::cast_precision_loss,
        reason = "the walk converts a word the way a single slot converts it, \
                  which is what the chain is compared against"
    )]
    fn a_frame_holding_one_word_twice_answers_what_that_word_answers_alone()
    {
        // A sender of one channel puts one word on both slots. The chain then
        // answers exactly what it answers that word converted on its own, so
        // the mean costs such a sender nothing, not even a rounding. The walk
        // beside the chain converts the word as one slot converts it and knows
        // nothing of a mean. The words run a full scale tone, then both ends of
        // the format and the odd words next to them, so a mean that doubled the
        // sample, or rounded the sum past what the format holds, parts from the
        // walk.
        const EDGES: [i32; 8] = [i32::MAX, i32::MIN, i32::MAX - 1, i32::MIN + 1, 1, -1, 0, 3];

        let mut carried = chain();
        let mut walked = chain();
        let mut compared = 0_u32;

        for lap in 0..2
        {
            for frame in 0..LAP_FRAMES
            {
                let tone = sine_word(lap, frame * WORDS_PER_FRAME, 7, f64::from(i32::MAX));
                let edge = EDGES.get((frame as usize) % EDGES.len()).copied().unwrap_or(0);

                for word in [tone, edge.cast_unsigned()]
                {
                    let sample = word.cast_signed() as f32;
                    let want = WayWords::of
                    (
                        WaySamples
                        {
                            low: walked.low.step(sample),
                            mid: walked.mid.step(sample),
                            high: walked.high.step(sample),
                        },
                        &mut walked.limiter
                    );

                    assert_eq!
                    (
                        carried.carry(word, word, 1.0),
                        want,
                        "a frame holding {word:#010x} twice parted from the word alone \
                         at frame {frame} of lap {lap}"
                    );

                    compared += 1;
                }
            }
        }

        assert_eq!(compared, 4 * LAP_FRAMES, "a frame went uncompared");
        assert!(carried == walked, "the chain and the walk left different states");
    }

    #[test]
    fn two_channels_in_opposition_answer_silence_and_one_channel_arrives_at_half()
    {
        // The mean, read through the carry of whole blocks rather than one
        // call. Two channels in opposition cancel to zero exactly, so every
        // word of both buffers is silent. One channel driven with the other
        // silent reaches the chain at half its level, on either slot, and every
        // word is read against a walk of the chain on that half sample, which
        // knows nothing of slots, blocks or positions.
        const LAPS: u32 = 3;

        let mut master = [0xDEAD_BEEF_u32; TEST_WORDS as usize];
        let mut slave = [0xDEAD_BEEF_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut chain(),
            LAPS,
            |lap, index|
            {
                let frame = index - index % WORDS_PER_FRAME;
                let word = sine_word(lap, frame, 7, SWEEP_AMPLITUDE).cast_signed();

                if index % WORDS_PER_FRAME == LEFT_SLOT { word } else { -word }.cast_unsigned()
            },
            &mut master,
            &mut slave
        );

        for index in 0..TEST_WORDS as usize
        {
            assert_eq!(master.get(index).copied(), Some(0), "master word {index} of two channels in opposition");
            assert_eq!(slave.get(index).copied(), Some(0), "slave word {index} of two channels in opposition");
        }

        for driven in [LEFT_SLOT, RIGHT_SLOT]
        {
            let mut master = [0_u32; TEST_WORDS as usize];
            let mut slave = [0_u32; TEST_WORDS as usize];

            run_laps
            (
                &mut chain(),
                LAPS,
                |lap, index| if index % WORDS_PER_FRAME == driven
                {
                    sine_word(lap, index, 7, SWEEP_AMPLITUDE)
                }
                else
                {
                    0
                },
                &mut master,
                &mut slave
            );

            let mut walked = chain();

            for lap in 0..LAPS
            {
                for frame in 0..LAP_FRAMES
                {
                    let index = frame * WORDS_PER_FRAME;
                    let whole = frame_of(sine_word(lap, index, 7, SWEEP_AMPLITUDE).cast_signed());
                    let half = (f64::from(whole) * 0.5) as f32;
                    let want = WayWords::of
                    (
                        WaySamples
                        {
                            low: walked.low.step(half),
                            mid: walked.mid.step(half),
                            high: walked.high.step(half),
                        },
                        &mut walked.limiter
                    );

                    if lap + 1 < LAPS
                    {
                        continue;
                    }

                    for (buffer, slot, word) in
                    [
                        (&master, LOW_SLOT, want.low()),
                        (&master, MID_SLOT, want.mid()),
                        (&slave, HIGH_SLOT, want.high()),
                    ]
                    {
                        assert_eq!
                        (
                            carried_word(buffer, index + slot),
                            word,
                            "slot {slot} of frame {frame} with slot {driven} alone driven"
                        );
                    }
                }
            }

            assert!
            (
                amplitude_at(&master, MID_SLOT, 7) > 0.25 * SWEEP_AMPLITUDE,
                "slot {driven} alone reached the mid way at nothing, so the reading \
                 above compared silence"
            );
        }
    }

    #[test]
    fn the_loudest_frames_of_the_format_do_not_come_back_round()
    {
        // The mean forms from two words of 32 bits, whose sum takes a 33rd.
        // Every pairing of the two ends of the format and of the words beside
        // them is carried through a block, and each answers the frame it holds
        // read as the mean in double precision, through a transparent chain so
        // the sample the chain was handed shows in every channel. A sum that
        // wrapped answers two loud words of one sign as a quiet word of the
        // other, and fails on the sign as well as on the value.
        const WORDS: [i32; 4] = [i32::MAX, i32::MAX - 1, i32::MIN, i32::MIN + 1];

        let plan = plan();
        let mut interface = MockInput::healthy();
        let mut pairs = 0_u32;

        for frame in 0..plan.carry_frames(Half::First)
        {
            let left = WORDS.get((frame as usize) % 4).copied().unwrap_or(0);
            let right = WORDS.get((frame as usize / 4) % 4).copied().unwrap_or(0);
            let index = frame * WORDS_PER_FRAME;

            place(&mut interface.source, index + LEFT_SLOT, left.cast_unsigned());
            place(&mut interface.source, index + RIGHT_SLOT, right.cast_unsigned());
        }

        carry_block(&mut interface, &plan, Half::First, shift(), &mut transparent_chain(), unity());

        for frame in 0..plan.carry_frames(Half::First)
        {
            let index = frame * WORDS_PER_FRAME;
            let left = carried_word(&interface.source, index + LEFT_SLOT).cast_signed();
            let right = carried_word(&interface.source, index + RIGHT_SLOT).cast_signed();
            let at = (index + plan.arming_position() - PIPELINE_WORDS) % TEST_WORDS;
            let want = staged(left.cast_unsigned(), right.cast_unsigned());
            let low = carried_word(&interface.master, at + LOW_SLOT);

            assert_eq!(low, want.low(), "the low way of the frame {left} and {right}");
            assert_eq!(carried_word(&interface.master, at + MID_SLOT), want.mid(), "the mid way of the frame {left} and {right}");
            assert_eq!(carried_word(&interface.slave, at + HIGH_SLOT), want.high(), "the high way of the frame {left} and {right}");

            // Two words of one sign leave on that sign, at the level one of
            // them leaves at on its own.
            if (left > 0) == (right > 0)
            {
                let alone = staged(left.cast_unsigned(), left.cast_unsigned()).low().cast_signed();

                assert_eq!(low.cast_signed().signum(), left.signum(), "the frame {left} and {right} came back round");
                assert!
                (
                    low.cast_signed().unsigned_abs() <= alone.unsigned_abs().max(1_u32 << 31),
                    "the frame {left} and {right} left louder than one word"
                );
                pairs += 1;
            }
        }

        assert!(pairs >= 8, "the block carried {pairs} frames of one sign");
    }

    #[test]
    fn the_two_channels_of_every_frame_join_across_the_blocks_of_a_run()
    {
        // The mean across several blocks, the straddling frame included. The
        // two slots carry different signals, a tone on the left and on the
        // right a word no lap repeats, so a carry that read the left slot twice,
        // or the right slot of the frame before, or the straddling frame before
        // the receiving stream finished it, parts from a straight walk of the
        // same frames. The walk forms each mean from the two
        // signals rather than from a buffer, and knows nothing of halves,
        // positions or shifts. The two blocks of a lap start on different slots
        // of a frame, which is what makes the seam a real one to cross.
        const LAPS: u32 = 4;

        let left = |lap: u32, index: u32| sine_word(lap, index, 7, SWEEP_AMPLITUDE);
        let mut master = [0_u32; TEST_WORDS as usize];
        let mut slave = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut chain(),
            LAPS,
            |lap, index| if index % WORDS_PER_FRAME == LEFT_SLOT
            {
                left(lap, index)
            }
            else
            {
                turning_word(lap, index)
            },
            &mut master,
            &mut slave
        );

        let mut straight = chain();
        let plan = plan();
        let straddling = plan.carry_start(Half::Second);

        assert!
        (
            straddling < plan.block_end(Half::First)
                && straddling + RIGHT_SLOT == plan.block_start(Half::Second),
            "frame {straddling} does not straddle the two blocks"
        );
        assert_ne!
        (
            turning_word(LAPS - 1, straddling + RIGHT_SLOT),
            turning_word(LAPS - 2, straddling + RIGHT_SLOT),
            "the right slot of the straddling frame repeats the lap before, so a \
             stale reading of it would pass"
        );

        for lap in 0..LAPS
        {
            for frame in 0..LAP_FRAMES
            {
                let index = frame * WORDS_PER_FRAME;
                let sample = source_sample(left(lap, index), turning_word(lap, index + RIGHT_SLOT));
                let want = WayWords::of
                (
                    WaySamples
                    {
                        low: straight.low.step(sample),
                        mid: straight.mid.step(sample),
                        high: straight.high.step(sample),
                    },
                    &mut straight.limiter
                );

                if lap + 1 < LAPS
                {
                    continue;
                }

                for (buffer, slot, word) in
                [
                    (&master, LOW_SLOT, want.low()),
                    (&master, MID_SLOT, want.mid()),
                    (&slave, HIGH_SLOT, want.high()),
                ]
                {
                    assert_eq!
                    (
                        carried_word(buffer, index + slot),
                        word,
                        "slot {slot} of frame {frame} of the last lap does not continue \
                         the run of the two channels"
                    );
                }
            }
        }
    }

    #[test]
    fn a_chain_at_rest_answers_a_silent_block_with_silence()
    {
        // The safety property of the state. A history that was never cleared, or
        // one a start-up never wrote, shows as a word that is not zero on a
        // source that is.
        let mut master = [0xDEAD_BEEF_u32; TEST_WORDS as usize];
        let mut slave = [0xDEAD_BEEF_u32; TEST_WORDS as usize];

        run_laps(&mut chain(), 2, |_, _| 0, &mut master, &mut slave);

        for index in 0..TEST_WORDS as usize
        {
            assert_eq!
            (
                master.get(index).copied(),
                Some(0),
                "master word {index} of a silent block came out loud"
            );
            assert_eq!
            (
                slave.get(index).copied(),
                Some(0),
                "slave word {index} of a silent block came out loud"
            );
        }
    }

    #[test]
    fn a_silent_chain_answers_a_loud_block_with_silence()
    {
        // What the machine runs when a start-up refuses, which is the value of
        // the type that carries no coefficients. It has to stop a full scale
        // source rather than pass it into a way with no filter on it.
        let mut master = [0_u32; TEST_WORDS as usize];
        let mut slave = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut FilterChain::silent(),
            2,
            |_, _| i32::MAX.cast_unsigned(),
            &mut master,
            &mut slave
        );

        for index in 0..TEST_WORDS as usize
        {
            assert_eq!(master.get(index).copied(), Some(0), "master word {index}");
            assert_eq!(slave.get(index).copied(), Some(0), "slave word {index}");
        }
    }

    #[test]
    fn a_history_a_loud_block_left_shows_on_the_silent_block_that_follows()
    {
        // The other side of the property above, and what tells a state that
        // PERSISTS from one that is thrown away at every block. A cascade fed
        // silence after a signal answers the tail of its own impulse response,
        // so the first silent block is not silent, and the tail dies.
        let mut chain = chain();
        let mut master = [0_u32; TEST_WORDS as usize];
        let mut slave = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut chain,
            2,
            |lap, index| sine_word(lap, index, 3, SWEEP_AMPLITUDE),
            &mut master,
            &mut slave
        );

        let mut tail_master = [0_u32; TEST_WORDS as usize];
        let mut tail_slave = [0_u32; TEST_WORDS as usize];

        run_laps(&mut chain, 1, |_, _| 0, &mut tail_master, &mut tail_slave);

        let loudest = largest_magnitude(&tail_master);

        assert!(loudest > 0, "a chain carried no history across the block it left");

        let mut quiet_master = [0_u32; TEST_WORDS as usize];
        let mut quiet_slave = [0_u32; TEST_WORDS as usize];

        run_laps(&mut chain, 200, |_, _| 0, &mut quiet_master, &mut quiet_slave);

        let left = largest_magnitude(&quiet_master);

        assert!(left < loudest, "the tail of {loudest} did not decay, it stands at {left}");
    }

    #[test]
    fn a_run_of_blocks_answers_what_one_unbroken_run_answers()
    {
        // The lesson of the lot that built this loop: a test that reads ONE
        // block cannot see a state that is dropped at a block boundary, or a
        // frame carried twice at the seam. So this runs the block structure over
        // several blocks and compares it against a straight walk of the same
        // samples through the same chain, which knows nothing of halves, of
        // positions or of shifts. The two blocks of a lap start on different
        // slots of a frame, which is what makes the seam a real one to cross.
        const LAPS: u32 = 4;

        let mut master = [0_u32; TEST_WORDS as usize];
        let mut slave = [0_u32; TEST_WORDS as usize];

        run_laps
        (
            &mut chain(),
            LAPS,
            |lap, index| sine_word(lap, index, 7, SWEEP_AMPLITUDE),
            &mut master,
            &mut slave
        );

        let mut straight = chain();
        let mut want_master = [0_u32; TEST_WORDS as usize];
        let mut want_slave = [0_u32; TEST_WORDS as usize];

        for lap in 0..LAPS
        {
            for frame in 0..LAP_FRAMES
            {
                let index = frame * WORDS_PER_FRAME;
                let ways = straight.carry
                (
                    sine_word(lap, index + LEFT_SLOT, 7, SWEEP_AMPLITUDE),
                    sine_word(lap, index + RIGHT_SLOT, 7, SWEEP_AMPLITUDE),
                    1.0
                );

                place(&mut want_master, index + LOW_SLOT, ways.low());
                place(&mut want_master, index + MID_SLOT, ways.mid());
                place(&mut want_slave, index + HIGH_SLOT, ways.high());
                place(&mut want_slave, index + SPARE_SLOT, SILENT_WORD);
            }
        }

        for index in 0..TEST_WORDS as usize
        {
            assert_eq!
            (
                master.get(index).copied(),
                want_master.get(index).copied(),
                "master word {index} of the last lap does not continue the run"
            );
            assert_eq!
            (
                slave.get(index).copied(),
                want_slave.get(index).copied(),
                "slave word {index} of the last lap does not continue the run"
            );
        }
    }

    /// The four samples one section of the double precision reference carries
    /// from a step to the next.
    #[derive(Clone, Copy, Default)]
    struct WideState
    {
        previous_input: f64,
        older_input: f64,
        previous_output: f64,
        older_output: f64,
    }

    /// Returns the last lap one way answers when its recurrence runs in DOUBLE
    /// precision with its feedback unbounded, a sample bounded nowhere at all.
    ///
    /// This is the reference for the property the chain documents: the history
    /// carries the value a section computed, and the bound stands at the
    /// conversion alone. A cascade that fed back the bounded value is a
    /// different filter, and it differs exactly where the signal is loudest.
    ///
    /// Double precision rather than single, so this is an EVALUATION of the
    /// recurrence and not a transcription of the one under test. A difference of
    /// association or of rounding order then shows as a departure of a few parts
    /// in ten thousand of the amplitude, which stands orders below what a
    /// bounded feedback moves.
    fn unbounded_way<F>(sections: &[Biquad], laps: u32, word: F)
        -> [f64; LAP_FRAMES as usize]
    where
        F: Fn(u32, u32) -> u32,
    {
        let mut states = [WideState::default(); CROSSOVER_SECTIONS];
        let mut out = [0.0_f64; LAP_FRAMES as usize];

        for lap in 0..laps
        {
            for frame in 0..LAP_FRAMES
            {
                let index = frame * WORDS_PER_FRAME;
                let mut value = f64::from(word(lap, index).cast_signed());

                for (section, state) in sections.iter().zip(states.iter_mut())
                {
                    let answered = f64::from(section.b0()) * value
                        + f64::from(section.b1()) * state.previous_input
                        + f64::from(section.b2()) * state.older_input
                        - f64::from(section.a1()) * state.previous_output
                        - f64::from(section.a2()) * state.older_output;

                    state.older_input = state.previous_input;
                    state.previous_input = value;
                    state.older_output = state.previous_output;
                    state.previous_output = answered;
                    value = answered;
                }

                if let Some(place) = out.get_mut(frame as usize)
                {
                    *place = value;
                }
            }
        }

        out
    }

    /// Laps the square drive of the two tests below runs.
    ///
    /// Four, so the lap they read opens on a history several edges old rather
    /// than on a cascade at rest.
    const SQUARE_LAPS: u32 = 4;

    #[test]
    fn a_drive_past_the_format_feeds_back_what_the_cascade_computed()
    {
        // The chain documents that the bound stands at the conversion and
        // nowhere else, and that the history carries the computed value. That
        // sentence is on the way to the drivers: a bound inside the feedback of
        // a crossover half is a filter whose shape changes when the signal is
        // loudest, which is when the compression driver is most exposed. So it
        // carries its test rather than standing on its own.
        //
        // Each way is driven with a full scale square in its own pass band,
        // which takes its recurrence past the format on the overshoot of every
        // edge. The two runs part company there, and the parting shows in the
        // ringing that follows, where the output is back inside the bound of
        // the way and readable.
        //
        // The reference carries the alignment of the way, since the alignment
        // stands between the recurrence and the buffer, and the bound a sample
        // is read against is the one the output stage answers for that way.
        let mut compared = 0_u32;

        for way in ways_under_test()
        {
            let mut master = [0_u32; TEST_WORDS as usize];
            let mut slave = [0_u32; TEST_WORDS as usize];
            let cycles = way.square_cycles;

            run_laps
            (
                &mut chain(),
                SQUARE_LAPS,
                |lap, index| square_word(lap, index, cycles),
                &mut master,
                &mut slave
            );

            let carried = match way.role
            {
                BlockRole::Master => &master,
                BlockRole::Slave => &slave,
            };
            let want = unbounded_way
            (
                way.active(),
                SQUARE_LAPS,
                |lap, index| square_word(lap, index, cycles)
            );
            let mut inside = 0_u32;

            for frame in 0..LAP_FRAMES
            {
                let reference = want.get(frame as usize).copied().unwrap_or(0.0)
                    * way.gain();

                // Only the samples this way carries whole are compared. Where
                // the recurrence stands past the bound of the way the stage
                // holds it there on purpose, and the test beside this one is
                // what reads those.
                if reference.abs() >= way.bound_magnitude(reference > 0.0)
                {
                    continue;
                }

                let index = frame * WORDS_PER_FRAME + way.slot;
                let answered = f64::from(carried_word(carried, index).cast_signed());
                let gap = (answered - reference).abs();

                inside = inside.saturating_add(1);
                compared = compared.saturating_add(1);

                assert!
                (
                    gap <= way.feedback_words,
                    "{} frame {frame} came out at {answered} against \
                     {reference}, a gap of {gap} words",
                    way.name
                );
            }

            assert!
            (
                inside > 0,
                "{} held no sample inside its own bound, so this compared \
                 nothing",
                way.name
            );
        }

        assert!(compared > 0, "no way of the chain was compared");
    }

    #[test]
    fn an_overshoot_is_held_at_the_bound_of_its_own_way()
    {
        // A cascade answers a step above the step. Driven at full scale it
        // therefore leaves the format, and what it must NOT do there is come
        // back round to the opposite sign, which is a swing of two full scales
        // into a way with no analog filter in front of it.
        //
        // Every sample whose reference stands past the bound of its way is read
        // against that bound on its OWN side, so a conversion that wrapped
        // answers the opposite bound and fails.
        //
        // The two bounds are not the same bound. The low way and the mid way
        // are held by the conversion, at the format. The high way is held by the
        // wall, 4.12 dB under it. The trim and the reserve keep the mid way
        // under the format and the high way under the wall on this drive, so
        // each way declares whether the drive reaches its bound rather than
        // every way being required to reach its own.
        //
        // How far under the wall this way stands is NOT a distance this drive
        // can give. A square in the pass band is one waveform out of every
        // waveform the format carries, and the reading below is what bounds
        // them all.
        for way in ways_under_test()
        {
            let mut master = [0_u32; TEST_WORDS as usize];
            let mut slave = [0_u32; TEST_WORDS as usize];
            let cycles = way.square_cycles;

            run_laps
            (
                &mut chain(),
                SQUARE_LAPS,
                |lap, index| square_word(lap, index, cycles),
                &mut master,
                &mut slave
            );

            let carried = match way.role
            {
                BlockRole::Master => &master,
                BlockRole::Slave => &slave,
            };
            let want = unbounded_way
            (
                way.active(),
                SQUARE_LAPS,
                |lap, index| square_word(lap, index, cycles)
            );
            let mut bounded = 0_u32;
            let mut loudest = 0.0_f64;

            for frame in 0..LAP_FRAMES
            {
                let reference = want.get(frame as usize).copied().unwrap_or(0.0)
                    * way.gain();
                let positive = reference > 0.0;

                loudest = loudest.max(reference.abs());

                if reference.abs() < way.bound_magnitude(positive)
                {
                    continue;
                }

                let index = frame * WORDS_PER_FRAME + way.slot;
                let answered = carried_word(carried, index).cast_signed();

                bounded = bounded.saturating_add(1);

                assert_eq!
                (
                    answered,
                    way.bound(positive),
                    "{} frame {frame} came out at {answered} where the \
                     recurrence stands at {reference}, which is what a \
                     conversion that wrapped produces",
                    way.name
                );
            }

            assert_eq!
            (
                bounded > 0,
                way.reaches_its_bound,
                "{} reached its bound on {bounded} samples of the lap, and its \
                 loudest stands at {loudest} words against a bound of {}",
                way.name,
                way.bound_magnitude(true)
            );
        }
    }

    /// Returns the sample the carry reads a source frame holding `word` in both
    /// slots as.
    ///
    /// The loudest sources below are frames of that shape. The mean of two
    /// words stands at full scale or under, and a frame holding one word twice
    /// answers that word, so a frame of this shape reaches every sample any
    /// frame reaches, full scale on either side included, and a bound it
    /// attains is the bound over every frame the format carries.
    fn frame_of(word: i32) -> f32
    {
        source_sample(word.cast_unsigned(), word.cast_unsigned())
    }

    /// Returns the loudest word the high way leaves for a full scale square of
    /// `half` samples to the half period, read through the output stage.
    ///
    /// The first four periods are dropped, so what comes back is the steady
    /// state rather than the answer to switching the drive on.
    fn loudest_square_word(half: u32) -> i32
    {
        let Ok(mut cascade) = HighCascade::of(Way::High)
        else
        {
            panic!("the high way did not build");
        };
        let mut loudest = 0_i32;

        for step in 0..(64 * half)
        {
            let sample = if (step / half).is_multiple_of(2) { i32::MIN } else { i32::MAX };
            let answered =
                staged_sample(cascade.step(frame_of(sample))).high();

            if step >= 8 * half
            {
                loudest = loudest.max(answered.cast_signed().abs());
            }
        }

        loudest
    }

    #[test]
    fn no_source_the_format_carries_reaches_the_wall_of_the_high_way()
    {
        // The 10.38 dB the trim and the reserve leave is the distance from one
        // SAMPLE at full scale. What the wall has to stand above is a PEAK, and
        // a cascade answers a weighted sum of the samples behind it, so the
        // loudest peak it can be made to answer over every source the format
        // carries is the sum of the magnitudes of its impulse response. The
        // source that attains that sum carries the sign pattern of the response
        // read backwards, which is why the figure below is a bound REACHED and
        // not one computed and hoped to be loose.
        //
        // The reading above drives this way with a square, which is one
        // waveform out of that set and not the worst of it, so a distance read
        // off a square is a point and not a bound. That is the whole reason
        // this stands beside it.
        const TAIL: usize = 1_024;
        const SQUARE_HALVES: u32 = 64;

        let Ok(mut impulse) = HighCascade::of(Way::High)
        else
        {
            panic!("the high way did not build");
        };
        let mut positive = [false; TAIL];
        let mut norm = 0.0_f64;

        for (index, sign) in positive.iter_mut().enumerate()
        {
            let answer = impulse.step(if index == 0 { 1.0 } else { 0.0 });

            *sign = answer >= 0.0;
            norm += f64::from(answer.abs());
        }

        let Ok(mut driven) = HighCascade::of(Way::High)
        else
        {
            panic!("the high way did not build");
        };
        let mut worst = 0_i32;
        let mut running = ThermalLimiter::at_rest();

        for sign in positive.iter().rev()
        {
            let sample = if *sign { i32::MAX } else { i32::MIN };
            let answer = driven.step(frame_of(sample));
            let answered = staged_sample(answer).high();
            let carried = WayWords::of
            (
                WaySamples { low: answer, mid: answer, high: answer },
                &mut running
            )
                .high();

            // The limiter the carry runs on reads the same word, since the
            // trim holds this drive under the threshold, so the bound read at
            // unity gain is the bound of the path the machine runs.
            assert_eq!(carried, answered, "the limiter acted on the loudest source");

            worst = worst.max(answered.cast_signed().abs());
        }

        // The wall is read off the stage rather than written down here, so it
        // is the wall the machine holds and moving it moves this reading with
        // it.
        let wall = staged_sample(f32::INFINITY).high().cast_signed();
        let margin_db = 20.0 * log10(f64::from(wall) / f64::from(worst));

        assert!((norm - 2.472_951).abs() < 1.0e-5, "the L1 norm of the high way reads {norm}");
        assert_eq!(worst, 1_000_334_656, "the source that attains the norm left {worst} words");
        assert!(worst < wall, "a source the format carries reached the wall at {worst} words");
        assert!
        (
            (margin_db - 2.516).abs() < 0.005,
            "the loudest peak this way can answer stands {margin_db} dB under the wall"
        );

        // And the square family is walked, to show what a reading taken on one
        // waveform is worth. The loudest square of the family stands UNDER the
        // bound above, and it stands OVER the square the reading beside this one
        // drives with, so that drive is not the worst square either and a
        // distance read off it is a point twice over. The family peaks at eight
        // samples to the half period. Past the peak it falls to a trough at 18,
        // rises to a second crest at 35, and from 128 up to 512 it holds between
        // 610115520 and 610115776 words, all under the peak. The walk runs to
        // 64, eight times the half period of the peak, and the loudest square it
        // reads is the loudest of every half period up to 512.
        let named_half = LAP_FRAMES / (2 * HIGH_SQUARE_CYCLES);
        let mut loudest_square = 0_i32;

        for half in 1..=SQUARE_HALVES
        {
            let answered = loudest_square_word(half);

            loudest_square = loudest_square.max(answered);
        }

        assert!
        (
            loudest_square_word(named_half) < loudest_square,
            "the square of {named_half} samples a half is the loudest of the \
             family, at {loudest_square} words"
        );

        assert!
        (
            loudest_square < worst,
            "the loudest square left {loudest_square} words where the bound over \
             every source stands at {worst}"
        );
    }

    /// Drives `way` with the source that attains the L1 norm of its cascade,
    /// then with that source turned over, and returns the loudest sample the
    /// stage is handed after the gain of the way, with the count of samples the
    /// bound of the way holds above zero and below it.
    ///
    /// The source takes the way past its bound on one side and the source
    /// turned over takes it past on the other. Every sample at or past the
    /// bound is read against the bound on its OWN side, and `WayUnderTest::bound`
    /// writes that bound down rather than reading it off the stage, so a
    /// conversion that wraps anywhere these two sources take the way answers the
    /// opposite bound and fails. The product is the one the stage forms, in
    /// single precision, so a sample read as past the bound is one the stage
    /// bounds.
    ///
    /// The tail is long enough for the slowest pole of the low way. MEASURED on
    /// that way, 8192 samples of response leave the loudest sample within a
    /// thousandth of a decibel of what 131072 leave.
    fn loudest_source_held<const N: usize>(way: &WayUnderTest) -> (f64, u32, u32)
    {
        const TAIL: usize = 8_192;

        let gain = crate::output::gain_for_test(way.way);
        let Ok(mut impulse) = WayCascade::<N>::of(way.way)
        else
        {
            panic!("{} did not build", way.name);
        };
        let mut positive = [false; TAIL];

        for (index, sign) in positive.iter_mut().enumerate()
        {
            *sign = impulse.step(if index == 0 { 1.0 } else { 0.0 }) >= 0.0;
        }

        let mut loudest = 0.0_f64;
        let mut above = 0_u32;
        let mut below = 0_u32;

        for turned in [false, true]
        {
            let Ok(mut driven) = WayCascade::<N>::of(way.way)
            else
            {
                panic!("{} did not build", way.name);
            };

            for sign in positive.iter().rev()
            {
                let sample = if *sign == turned { i32::MIN } else { i32::MAX };
                let answer = driven.step(frame_of(sample));
                let reference = f64::from(answer * gain);
                let side = reference > 0.0;

                loudest = loudest.max(reference.abs());

                if reference.abs() < way.bound_magnitude(side)
                {
                    continue;
                }

                let answered = way_word(way.way, staged_sample(answer)).cast_signed();

                if side
                {
                    above = above.saturating_add(1);
                }
                else
                {
                    below = below.saturating_add(1);
                }

                assert_eq!
                (
                    answered,
                    way.bound(side),
                    "{} came out at {answered} where the stage was handed {reference}, \
                     which is what a conversion that wrapped produces",
                    way.name
                );
            }
        }

        (loudest, above, below)
    }

    #[test]
    fn the_loudest_source_the_format_carries_is_held_at_the_format_on_the_low_and_mid_ways()
    {
        // The conversion is the bound of these two ways, and the square
        // `an_overshoot_is_held_at_the_bound_of_its_own_way` drives them with is
        // one waveform: it takes the low way past the format and leaves the mid
        // way under it. The source that attains the L1 norm of a cascade is the
        // loudest any source the format carries puts on the way, and after the
        // trim and the reserve it takes both ways past the format. It runs as it
        // is and turned over, which puts that peak on each side of zero, and
        // every sample of the two the conversion bounds is read on its own side.
        // These are the two figures the documentation of the output stage
        // carries.
        let low = built_way(LOW_UNDER_TEST);
        let mid = built_way(MID_UNDER_TEST);
        let over_format = |loudest: f64| 20.0 * log10(loudest / 2_147_483_648.0);

        for (way, (loudest, above, below), figure) in
        [
            (&low, loudest_source_held::<{ way_sections(Way::Low) }>(&low), 5.42),
            (&mid, loudest_source_held::<{ way_sections(Way::Mid) }>(&mid), 1.73),
        ]
        {
            let read = over_format(loudest);

            assert!
            (
                (read - figure).abs() < 0.005,
                "the loudest source puts {} {read} dB over the format",
                way.name
            );
            assert!(above > 0, "the conversion bounded no sample of {} above zero", way.name);
            assert!(below > 0, "the conversion bounded no sample of {} below zero", way.name);
        }
    }

    #[test]
    fn serving_an_entry_takes_the_flag_down_before_it_carries_the_block()
    {
        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift()));
        assert_eq!(serve(&mut interface, &plan(), shift(), &mut chain(), &mut Steady), Ok(()));

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
                serve(&mut interface, &plan(), shift(), &mut transparent_chain(), &mut Steady),
                Ok(())
            );

            assert_eq!
            (
                interface.cleared.map(|(which, _)| which),
                Some(half),
                "the {half:?} event took another flag down"
            );

            let (master, slave) = carried_image(half);

            assert_buffers(&interface, &master, &slave, carry_case(half));
        }
    }

    #[test]
    fn a_refused_entry_carries_nothing()
    {
        // The chain is taken off its rest first, a tone of 2332 W at the
        // limiter over ten laps, so the limiter stands under unity and every
        // history has moved. A refusal that rebuilt any part of the chain then
        // shows in the comparison of the whole of it.
        const WARM_LAPS: u32 = 10;

        let plan = plan();
        let loud = HighCascade::of_section(crate::filter::gain_section_for_test(SERVED_SECTION_GAIN));
        let mut served = FilterChain { high: loud, ..chain() };
        let mut warm = MockInput::healthy();

        for lap in 0..WARM_LAPS
        {
            for index in 0..TEST_WORDS
            {
                place(&mut warm.source, index, sine_word(lap, index, 30, f64::from(i32::MAX)));
            }

            for half in [Half::First, Half::Second]
            {
                carry_block(&mut warm, &plan, half, shift(), &mut served, unity());
            }
        }

        let before = served.clone();
        let gain = served.limiter.current_gain_for_test();

        assert!(gain < 1.0, "the warm up left the gain at {gain}");

        let mut interface = MockInput::healthy();

        assert_eq!(bring_up(&mut interface, &plan, &waits()), Ok(shift()));
        interface.image.block.overrun = true;

        let mut asked = Recording::default();

        assert_eq!
        (
            serve(&mut interface, &plan, shift(), &mut served, &mut asked),
            Err(PassthroughFault::Event(EventFault::Overrun))
        );
        assert_eq!(interface.carried, None);
        assert_eq!(interface.cleared, None);
        assert!(served == before, "the refused entry moved the chain");
        assert_eq!(asked.calls, 0, "the refused entry advanced the gain");
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
        chain: &mut FilterChain
    ) -> Result<(), PassthroughFault>
    {
        let words = plan().transfer_items();

        run_wire
        (
            interface,
            laps,
            shift,
            chain,
            |interface, _, at|
            {
                carried_word(&interface.master, (at + interface.offset + words - PIPELINE_WORDS) % words)
            }
        )
    }

    /// Runs the block structure for `laps` laps over a link that puts the word
    /// `link` names into the receiving buffer, one word a slot.
    ///
    /// `link` is handed the interface as it stands at that slot, the lap and
    /// the position. A transfer event is raised at the slot that fills a half,
    /// which is where the part raises one, and it is served before the next
    /// word arrives. So a carry runs with the half its event names written and
    /// every word after that half still holding what the lap before left there,
    /// or zero on the first lap, which is where the receiving stream stands
    /// when the part raises the event.
    fn run_wire<F>
    (
        interface: &mut MockInput,
        laps: u32,
        shift: CarryShift,
        chain: &mut FilterChain,
        link: F
    ) -> Result<(), PassthroughFault>
    where
        F: Fn(&MockInput, u32, u32) -> u32,
    {
        let plan = plan();
        let words = plan.transfer_items();
        let block = plan.block_words();

        for slot in 0..laps * words
        {
            let at = slot % words;
            let word = link(interface, slot / words, at);

            place(&mut interface.source, at, word);

            let filled = match at + 1
            {
                position if position == block => Half::First,
                position if position == words => Half::Second,
                _ => continue,
            };

            interface.next_event = filled;
            interface.written = 0;
            serve(interface, &plan, shift, chain, &mut Steady)?;
        }

        Ok(())
    }

    #[test]
    fn a_carry_reads_no_word_the_link_has_not_delivered_when_its_event_is_raised()
    {
        // The rhythm of the wire against the event. The link delivers one word
        // a slot and each event is served at the slot that raises it, so the
        // half a carry answers is in and the word after it is not. The part
        // raises the event at that same slot, so a carry reading a word past
        // its half depends on outrunning the wire to find it, and here it reads
        // the lap before whatever order its loop reads in. Every word of the
        // source turns over from one lap to the next, so such a reading parts
        // from a straight walk of the frames, which knows nothing of events,
        // halves or positions. The run is taken at every counter distance of
        // the loop tests, since the distance moves the destinations against
        // the event.
        const LAPS: u32 = 3;

        let plan = plan();

        for offset in LOOP_OFFSETS
        {
            let mut interface = MockInput::healthy();

            interface.offset = offset;

            let shift = shift_at(offset);

            assert_eq!
            (
                bring_up(&mut interface, &plan, &waits()),
                Ok(shift),
                "the bring-up refused a distance of {offset} words"
            );
            assert_eq!
            (
                run_wire(&mut interface, LAPS, shift, &mut chain(), |_, lap, at| turning_word(lap, at)),
                Ok(()),
                "an event was refused at a distance of {offset} words"
            );

            let mut straight = chain();

            for lap in 0..LAPS
            {
                for frame in 0..LAP_FRAMES
                {
                    let index = frame * WORDS_PER_FRAME;
                    let sample = source_sample
                    (
                        turning_word(lap, index + LEFT_SLOT),
                        turning_word(lap, index + RIGHT_SLOT)
                    );
                    let want = WayWords::of
                    (
                        WaySamples
                        {
                            low: straight.low.step(sample),
                            mid: straight.mid.step(sample),
                            high: straight.high.step(sample),
                        },
                        &mut straight.limiter
                    );

                    if lap + 1 < LAPS
                    {
                        continue;
                    }

                    let at = (index + offset - PIPELINE_WORDS) % TEST_WORDS;

                    for (buffer, slot, word) in
                    [
                        (&interface.master, LOW_SLOT, want.low()),
                        (&interface.master, MID_SLOT, want.mid()),
                        (&interface.slave, HIGH_SLOT, want.high()),
                    ]
                    {
                        assert_eq!
                        (
                            carried_word(buffer, at + slot),
                            word,
                            "slot {slot} of frame {frame} of the last lap at a distance of \
                             {offset} words does not continue the walk"
                        );
                    }
                }
            }
        }
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
            // its way through the chain, so what a lap returns is the rounded
            // word rather than the seeded one.
            //
            // What each channel comes back holding is the word that entered the
            // last lap at the gain of its own way. The link closes on the two
            // slots the low way and the mid way leave in, and the chain reads
            // their mean, so the content falls at every lap by the gain those
            // two ways carry and stays in place. The expectation walks the
            // stage the same number of laps.
            let entered = looped_buffer(&narrowed_buffer(&interface.master), 12);
            let want_master = staged_buffer(&entered, BlockRole::Master);
            let want_slave = staged_buffer(&entered, BlockRole::Slave);

            assert_eq!(run_loop(&mut interface, 12, shift, &mut transparent_chain()), Ok(()));

            for index in 0..TEST_WORDS as usize
            {
                assert_eq!
                (
                    interface.master.get(index).copied(),
                    want_master.get(index).copied(),
                    "master word {index} moved over twelve laps at offset {offset}"
                );
                assert_eq!
                (
                    interface.slave.get(index).copied(),
                    want_slave.get(index).copied(),
                    "slave word {index} moved over twelve laps at offset {offset}"
                );
            }
        }
    }

    /// Fall a closed link of twelve laps must leave, as a divisor of the seed.
    ///
    /// The measurement beside the test that reads it stands at a 2476th, so
    /// this is half of what was measured.
    const DECAY_FLOOR: u32 = 1_237;

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
    fn a_closed_link_through_the_chain_falls_silent()
    {
        // The two tests around this one run a transparent chain, and this is
        // what that trades away, measured instead of left unsaid. A closed link
        // carries what this board sent, so its content crosses a way once a lap
        // and what survives is the part of it that way passes.
        //
        // The link closes on the master buffer, whose two slots carry the LOW
        // way and the MID way, so the seeded 1000 Hz tone re-enters the chain
        // at every lap as the mean of what those two ways left. The mid way
        // passes that tone and the mean halves it, the low way is 42 dB down
        // there.
        //
        // MEASURED here, from a seed of 268433753, on the largest word of the
        // master buffer. For the first six laps it stands on the mid slot, at
        // 175635152 after one lap and 1292714 after six. From the seventh lap
        // on it stands on the low slot, and it does not fall by a fixed step a
        // lap: eleven laps leave 172062 and twelve leave 108416, a 2476th of
        // the seed. The bound below is half that measured fall.
        //
        // It is also the reason a closed link can measure WHERE a word lands and
        // never what the chain does to it: a bench reading this loop reads a
        // content the chain has already eaten.
        let mut interface = MockInput::healthy();

        interface.offset = plan().arming_position();

        let shift = shift();

        assert_eq!(bring_up(&mut interface, &plan(), &waits()), Ok(shift));

        seed_tone(&mut interface);

        let seeded = largest_magnitude(&interface.master);

        assert!(seeded > 0, "the seed carries a tone");
        assert_eq!(run_loop(&mut interface, 12, shift, &mut chain()), Ok(()));

        let left = largest_magnitude(&interface.master);

        assert!
        (
            left.saturating_mul(DECAY_FLOOR) < seeded,
            "twelve crossings of the chain left {left} of {seeded}"
        );
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

            // The tone each channel is read against is the tone that entered
            // the last lap at the gain of that channel, since the stage under
            // the carry brings each of the three down and the link feeds the
            // mean of the low way and the mid way back at every lap. A channel
            // is compared against its own,
            // which leaves the reading here about the JOIN and about nothing
            // else.
            let entered = looped_buffer(&narrowed_buffer(&interface.master), 12);
            let want_master = staged_buffer(&entered, BlockRole::Master);
            let want_slave = staged_buffer(&entered, BlockRole::Slave);

            assert_eq!(run_loop(&mut interface, 12, shift, &mut transparent_chain()), Ok(()));

            for slot in 0..WORDS_PER_FRAME as usize
            {
                let bound = largest_step(&want_master, slot);

                assert!(bound > 0, "the seeded tone moves on master slot {slot}");
                assert_eq!
                (
                    largest_step(&interface.master, slot),
                    bound,
                    "master slot {slot} at offset {offset} carries a step the \
                     tone does not"
                );
            }

            // The slave carries the high way on one slot and silence on the
            // other, so only the driven one is read against the tone and the
            // other is read against nothing moving at all.
            let high_bound = largest_step(&want_slave, HIGH_SLOT as usize);

            assert!(high_bound > 0, "the seeded tone moves on the high channel");
            assert_eq!
            (
                largest_step(&interface.slave, HIGH_SLOT as usize),
                high_bound,
                "the slave channel of the high way at offset {offset} carries a \
                 step the tone does not"
            );
            assert_eq!
            (
                largest_step(&interface.slave, SPARE_SLOT as usize),
                0,
                "the channel no way drives moved at offset {offset}"
            );
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

    #[test]
    fn every_sub_block_cause_byte_stays_where_a_probe_reads_it()
    {
        pin_cause_bytes!
        (
            InputBlockFault
            {
                ModeWrong = 0x01,
                ProtocolWrong = 0x02,
                DataSizeWrong = 0x03,
                BitOrderWrong = 0x04,
                ClockStrobingWrong = 0x05,
                SynchronisationWrong = 0x06,
                SynchronisationSourceWrong = 0x07,
                MonoWrong = 0x08,
                TristateWrong = 0x09,
                FifoThresholdWrong = 0x0A,
                FrameLengthWrong = 0x0B,
                FrameActiveLengthWrong = 0x0C,
                FrameDefinitionWrong = 0x0D,
                FramePolarityWrong = 0x0E,
                FrameOffsetWrong = 0x0F,
                FirstBitOffsetWrong = 0x10,
                SlotSizeWrong = 0x11,
                SlotCountWrong = 0x12,
                SlotsNotEnabled = 0x13,
                TransferDisabled = 0x14,
                NotEnabled = 0x15,
                InterruptEnabled = 0x16,
                Overrun = 0x17,
                FrameMismatch = 0x18,
                CodecNotReady = 0x19,
                FifoNeverFilled = 0x1A,
                SynchronisationOutputWrong = 0x1B,
            }
        );
    }

    #[test]
    fn every_sequence_cause_byte_stays_where_a_probe_reads_it()
    {
        // These name the STEP of the bring-up that gave up rather than a
        // register field, so a person reading a parked board looks the number
        // up here and nowhere else. The macro builds an exhaustive match, so a
        // variant added without a number beside it does not compile.
        pin_cause_bytes!
        (
            InputSequenceFault
            {
                StreamNeverStopped = 0x01,
                BlockNeverStopped = 0x02,
                OutputPhaseNeverReached = 0x03,
                ReceiverNeverAdvanced = 0x04,
                OffsetOutOfBand = 0x05,
                SeedNeverLapped = 0x06,
                FilterRefused = 0x07,
            }
        );
    }

    #[test]
    fn every_event_cause_byte_stays_where_a_probe_reads_it()
    {
        pin_cause_bytes!
        (
            EventFault
            {
                Spurious = 0x01,
                BothHalvesPending = 0x02,
                Overrun = 0x03,
                FrameMismatch = 0x04,
                CodecNotReady = 0x05,
                BlockStopped = 0x06,
                TransferError = 0x07,
                DirectModeError = 0x08,
                FifoError = 0x09,
                StreamStopped = 0x0A,
                CounterOutOfRange = 0x0B,
                OutputCounterOutOfRange = 0x0C,
                ReadPointerInBlock = 0x0D,
                CarryShiftUnpublished = 0x0E,
            }
        );
    }

    #[test]
    fn every_plan_cause_byte_stays_where_a_probe_reads_it()
    {
        pin_cause_bytes!
        (
            InputPlanError
            {
                BufferEmpty = 0x01,
                BufferTooLong = 0x02,
                BufferNotWholeFrames = 0x03,
                BufferHalfNotWhole = 0x04,
                BufferUnaligned = 0x05,
                BufferUnreachable = 0x06,
                BufferOverlapsOutput = 0x07,
                BufferTooShortForGuard = 0x08,
                BufferLengthDiffers = 0x09,
            }
        );
    }

    #[test]
    fn every_stream_cause_byte_stays_where_a_probe_reads_it()
    {
        pin_cause_bytes!
        (
            InputStreamFault
            {
                RequestWrong = 0x01,
                SynchronisationEnabled = 0x02,
                DirectionWrong = 0x03,
                NotCircular = 0x04,
                MemoryNotIncrementing = 0x05,
                PeripheralIncrementing = 0x06,
                MemoryWidthWrong = 0x07,
                PeripheralWidthWrong = 0x08,
                DoubleBuffered = 0x09,
                PriorityWrong = 0x0A,
                ErrorInterruptEnabled = 0x0B,
                EventInterruptEnabled = 0x0C,
                PeripheralAddressWrong = 0x0D,
                MemoryAddressWrong = 0x0E,
                ItemCountWrong = 0x0F,
                CounterOutOfRange = 0x10,
                NotEnabled = 0x11,
                TransferError = 0x12,
                DirectModeError = 0x13,
                FifoError = 0x14,
            }
        );
    }

}
