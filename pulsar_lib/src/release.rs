//! Release gate of the converter mute line.
//!
//! The converters come out of reset muted, held there by the pull-down on
//! their XSMT pin, and this is what raises that pin. It is the one place a
//! firmware turns silence into sound, so it sequences rather than decides:
//! verify, raise, hold, permit.
//!
//! Nothing here touches a peripheral. `AudioInterface` is how it observes the
//! transport and how it takes the two FIFO error flags down as its window
//! opens, `ConverterMute` is the mute line a register block implements, and
//! `open` is the sequence run over the two.
//!
//! # The terms
//!
//! XSMT rises only once every stage feeding the converter has reported ready.
//! That is four terms: clocks verified, transfers running, the buffer filled
//! with zeros, filters initialised. This gate carries all four, and a stage
//! added between a sample and a converter adds its term here rather than
//! leaning on the stage ahead of it.
//!
//! **Clocks verified** is the witness the clock bring-up returns after
//! comparing its read-back field by field against its plan. The type that
//! carries it belongs to the firmware that reads the registers, so this module
//! names `VerifiedClock` rather than that type, and `open` takes one by
//! reference. That leaves the start order to the compiler instead of to a
//! comment: a refused clock builds no witness, so there is nothing to hand
//! over.
//!
//! **Transfers running** is measured here rather than asserted. A bring-up
//! proves a counter moved once, which does not tell a stream that runs from a
//! stream that advances one word and stalls. So `open` watches both transfer
//! counters until each has reloaded twice, and reads the error flags of the
//! two sub-blocks and the two streams on every poll of that window.
//!
//! The window opens on a cleared pair of `FEIF` flags, and on nothing else
//! cleared. Both streams raise that flag while their FIFOs fill from empty,
//! before either buffer has moved a half, and every one of these flags is
//! sticky, so a window that kept it would refuse on the start-up of the very
//! transport it is measuring. What makes taking it down lossless is RM0433
//! section 15.3.20. It reads two ways, and the window catches both without the
//! flag: a burst against a FIFO threshold clears `EN` in hardware, which is
//! `StreamAlarm::NotEnabled`, and an underrun carries "there is no data loss
//! when this kind of errors occur" with the one way out running through the
//! peripheral, which is `BlockAlarm::Underrun` and is not cleared. `TEIF` and
//! `DMEIF` are not cleared either, so a bus error or a part that is not the one
//! the manual describes refuses whenever it appeared.
//!
//! The wipe runs once, ahead of the opening read. Inside the polling loop it
//! would erase a flag raised while the window ran, which is the reading the
//! window exists to take, and `watch` holds the interface by shared reference
//! so that it cannot.
//!
//! Two reloads are what makes the window a lap. The counters are found wherever
//! they stand, so the run up to a FIRST reload is a fraction of a lap and can
//! be one poll of it. Between two reloads a whole buffer has gone out whatever
//! that opening position was, so the second reload is the one that bounds the
//! window from below. RM0433 section 15.5.6 has the counter decrement on each
//! transfer and reload at the end of a lap in circular mode, so a reading above
//! the one before it is a reload and can be nothing else. Polling slower than a
//! lap under-counts laps, which lengthens the window, and never invents one.
//!
//! **Buffer zeroed** is the caller writing zeros before the streams start and
//! writing nothing else until this gate returns. The zeros therefore run
//! through the whole of the window above, then through the hold that follows
//! the raise, and the hold covers the unmute ramp of the converter. `TonePermit`
//! is what a caller needs to write a non-zero sample, and only a completed
//! sequence builds one.
//!
//! **Filters initialised** is the witness the input bring-up returns once the
//! crossover chain stands in the memory the carry reads. The type that carries
//! it belongs to the firmware that parks that chain, so this module names
//! `InitialisedFilters` rather than that type, and `open` takes one by
//! reference for the reason it takes the clock one: a refused chain builds no
//! witness, so there is nothing to hand over and a run that never published a
//! chain never reaches the raise.
//!
//! # What the converter does with a long silence
//!
//! The converter counts zeros of its own. Continuous zero data on both
//! channels over 1024 frame clock periods puts it into a full analog mute,
//! which is 23.2 ms at this sample rate. Both slots of this frame carry the
//! same sample, so the condition is met the moment the buffer holds silence.
//!
//! Those zeros do not start here. The caller fills both buffers before it
//! enables either stream, so silence has been going out since the transport
//! came up and the window adds one to two laps of the buffer to a count that
//! was already running. Whether it has run out by the time the line rises
//! depends on how long the bring-up ahead of it took, which this module does
//! not measure. The hold that follows the raise is a cycle count sized for the
//! fastest core clock the part runs at, so on a slower core it lasts
//! proportionally longer than the sequence it covers. That is a mute added to a
//! mute and it can only make the output quieter, so nothing here works around
//! it.
//!
//! What it costs is a reading. Once the analog mute has armed, the step between
//! XSMT rising and the first audible output is that mute releasing rather than
//! the unmute ramp, and the part adds the group delay of its interpolation
//! filter on top before its analog output starts ramping. A measurement of that
//! step can be reading either mechanism, so what says which is how long silence
//! ran before the raise rather than the ramp figure on its own.
//!
//! # What a refusal leaves behind
//!
//! The mute line low and no permit. A refusal seen before the raise leaves a
//! line that was never raised, and one seen after it drives the line back
//! down. `Phase` is the half of a fault code that says which of the two
//! happened, so a reader of a parked board knows whether anything was ever
//! audible.

use crate::clock::wait_polls;
use crate::constants::{MICROSECONDS_PER_SECOND, SAMPLE_RATE_HZ};
use crate::transport::{AudioInterface, TransportPlan, TransportReadback};

/// Laps of the buffer the window budget covers.
///
/// The window ends on the second reload of each counter, so it spends the
/// fraction of a lap left when it opened and then a whole one. Two laps is what
/// that can take, and the third is the margin that keeps a healthy transport
/// off the timeout.
const WINDOW_LAPS: u32 = 3;

/// Attests that the audio kernel clock came up and read back as planned.
///
/// The type carrying the attestation belongs to the firmware that reads the
/// registers, so this trait is what the gate names. Implement it on that type
/// and on nothing else: what it claims is a field by field read-back against a
/// validated plan, and a lock bit is not one.
pub trait VerifiedClock
{
}

/// Attests that the crossover chain stands in the memory the carry reads.
///
/// The type carrying the attestation belongs to the firmware that parks that
/// chain, so this trait is what the gate names. Implement it on that type and
/// on nothing else: what it claims is a built chain published where the carry
/// reads it, and a chain a caller merely holds is not one.
pub trait InitialisedFilters
{
}

/// The converter mute line, and the hold that covers its unmute ramp.
///
/// Every method is a register write on the real part, which is why they are
/// separate: `open` decides when the line is armed and when it goes high, and
/// the implementation decides how.
pub trait ConverterMute
{
    /// Drives the mute line low.
    ///
    /// On a line still held by its pull-down this reaches the output register
    /// of a pad that drives nothing, which is what `take_output` then finds.
    /// On one already under the port it takes the level down.
    fn drive_low(&mut self);

    /// Hands the mute line to the port driver.
    ///
    /// `open` calls it only after `drive_low`, so the level the driver takes
    /// up is the low that call left behind and the pad never passes through a
    /// high it was not asked for. Splitting it from `drive_low` is what puts
    /// that order in this module, where a test reaches it, rather than in the
    /// register block, where none does.
    fn take_output(&mut self);

    /// Drives the mute line high, which starts the converter unmute ramp.
    fn drive_high(&mut self);

    /// Holds for at least the converter mute sequence.
    ///
    /// The audio clocks keep running through it, since the converter counts
    /// its ramp in sample periods.
    fn hold(&mut self);
}

/// Which side of the raise a fault was seen on.
///
/// The value is the high nibble of the place byte of a fault code, so one
/// refusal word says where the gate refused and whether the mute line had gone
/// up by then.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Phase
{
    /// Before the mute line was raised, so nothing was ever audible.
    Muted = 0x00,
    /// After it was raised. The gate has driven it back low.
    Unmuted = 0x10,
}

/// Declares the places the gate refuses at, and holds every one of them under
/// the phase bit.
///
/// A place byte is a phase over a site, so a site numbering into the phase
/// nibble would read back as another pair and turn the answer to "was anything
/// ever audible" around. Each site is named here once and the expansion carries
/// its own assertion, so what the check covers is the sites that exist rather
/// than the last one written by hand. A site cannot be declared anywhere else,
/// since the enum has no other declaration.
macro_rules! release_sites
{
    (
        $(
            $(#[$note:meta])*
            $name:ident = $place:literal
        ),+
        $(,)?
    ) =>
    {
        /// Where the release gate refused.
        ///
        /// The discriminant is the low nibble of the place byte of a fault
        /// code. Two sites that carried one number would not compile.
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[repr(u8)]
        enum ReleaseSite
        {
            $(
                $(#[$note])*
                $name = $place,
            )+
        }

        $(
            const _: () = assert!
            (
                (ReleaseSite::$name as u8) < (Phase::Unmuted as u8),
                "this site numbers below the phase bit, so a place byte splits \
                 into one phase and one site and never reads as another pair"
            );
        )+
    };
}

release_sites!
{
    /// The gate sequence itself, which names no sub-block and no stream.
    Sequence = 0x00,
    /// The master sub-block.
    MasterBlock = 0x01,
    /// The slave sub-block.
    SlaveBlock = 0x02,
    /// The stream feeding the master sub-block.
    MasterStream = 0x03,
    /// The stream feeding the slave sub-block.
    SlaveStream = 0x04,
}

/// Reason the gate gave up on a step of its own sequence.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it, which is the number a person with a probe looks up. Two variants that
/// carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SequenceRefusal
{
    /// A transfer counter did not reload twice inside the window, so at least
    /// one stream did not replay a whole buffer.
    TransfersNeverLapped = 0x01,
}

/// Reason one audio sub-block refused the release.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it. Two variants that carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BlockAlarm
{
    /// `OVRUDR` is set, so the transmitter has sent a frame it had no data
    /// for, and what went out in that frame is not the buffer.
    Underrun = 0x01,
    /// `WCKCFG` is set, so the part refuses the frame against the master clock
    /// it was asked to generate.
    ClockConfigurationRejected = 0x02,
    /// `SAIEN` is clear, so the sub-block has stopped since the bring-up.
    NotEnabled = 0x03,
}

/// Reason one transfer stream refused the release.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it. Two variants that carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StreamAlarm
{
    /// `TEIF` is set, so a transfer took a bus error.
    TransferError = 0x01,
    /// `FEIF` is set, and the gate took it down as it opened the window, so
    /// the condition was met while the window ran.
    ///
    /// RM0433 section 15.3.20 raises it in direct mode when the memory bus is
    /// not granted before a peripheral request, which is this stream running
    /// out of data. The same section attaches no data loss to that, so what
    /// this refuses is a transport working at the edge of its bus rather than
    /// a sample already gone. `BlockAlarm::Underrun` is where the loss the
    /// manual does leave open arrives.
    ///
    /// Direct mode is `DMDIS` clear, which nothing reads back, so the
    /// alternative reading is a burst against a FIFO threshold. That one has
    /// the part clear `EN` in hardware, so `NotEnabled` refuses it whether or
    /// not this flag does.
    FifoError = 0x02,
    /// `DMEIF` is set.
    ///
    /// RM0433 section 15.3.20 confines the flag to a peripheral to memory
    /// stream in direct mode with `MINC` clear, and both streams here are
    /// memory to peripheral with `MINC` set, so the part raises it on neither.
    /// A set bit therefore says the stream is not the one the plan armed, or
    /// the part is not the one the manual describes. The gate refuses either
    /// way rather than deciding which.
    DirectModeError = 0x03,
    /// `EN` is clear, so the stream has stopped since the bring-up.
    NotEnabled = 0x04,
    /// `NDTR` reads past the buffer length, so the counter belongs to no lap
    /// of it and the reload the window watches for cannot be told from it.
    CounterOutOfRange = 0x05,
}

/// Reason the release gate refused to raise the mute line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReleaseFault
{
    /// The gate gave up on a step of its own sequence, before the line moved.
    Sequence(SequenceRefusal),
    /// The master sub-block raised an alarm.
    MasterBlock(Phase, BlockAlarm),
    /// The slave sub-block raised an alarm.
    SlaveBlock(Phase, BlockAlarm),
    /// The stream feeding the master sub-block raised an alarm.
    MasterStream(Phase, StreamAlarm),
    /// The stream feeding the slave sub-block raised an alarm.
    SlaveStream(Phase, StreamAlarm),
}

/// Widest word the release encoding can carry.
///
/// A code is a place byte over a cause byte, so the two types bound it.
/// Nothing here numbers a phase, a site or a cause past its byte, so no
/// variant added to any of the three cause enums can pass this, and a fault
/// record keeps the code out of the domain field it sits beside. The bound is
/// not a value the encoding reaches.
pub(crate) const RELEASE_CODE_CEILING: u32 = u16::MAX as u32;

impl ReleaseFault
{
    /// Returns the code a fault record carries for this fault.
    ///
    /// A probe reads the word and looks the value up here. The code is a place
    /// byte over a cause byte: the place says WHERE the gate refused and on
    /// which side of the raise, the cause says what that place refused with.
    #[must_use]
    pub const fn code(self) -> u32
    {
        let (place, cause) = self.place_and_cause();

        u16::from_be_bytes([place, cause]) as u32
    }

    /// Returns the two bytes of the code, the place then the cause.
    ///
    /// The place is a phase over a site and the cause is the discriminant of
    /// the alarm that site carries, and both come out of one match, so a
    /// variant cannot pick up the site of one arm and the alarm of another.
    const fn place_and_cause(self) -> (u8, u8)
    {
        match self
        {
            Self::Sequence(refusal) =>
                (ReleaseSite::Sequence as u8, refusal as u8),
            Self::MasterBlock(phase, alarm) =>
                (phase as u8 | ReleaseSite::MasterBlock as u8, alarm as u8),
            Self::SlaveBlock(phase, alarm) =>
                (phase as u8 | ReleaseSite::SlaveBlock as u8, alarm as u8),
            Self::MasterStream(phase, alarm) =>
                (phase as u8 | ReleaseSite::MasterStream as u8, alarm as u8),
            Self::SlaveStream(phase, alarm) =>
                (phase as u8 | ReleaseSite::SlaveStream as u8, alarm as u8),
        }
    }
}

/// Permission to write a non-zero sample into a buffer the streams replay.
///
/// `open` is the only thing that builds one, and it builds one only after the
/// four terms held and the hold ran to the end. The field is private, so no
/// other module and no other crate can construct one, and the type is neither
/// `Copy` nor `Clone`, so what a caller holds is the one the gate returned.
///
/// A firmware that writes a tone takes one rather than deciding for itself
/// that the gate ran.
#[derive(Debug)]
#[must_use]
pub struct TonePermit(());

/// Polls the gate holds before it gives up on the transfer window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReleaseWaits
{
    /// Polls both transfer counters get to reload.
    pub lap_polls: u32,
}

impl ReleaseWaits
{
    /// Builds the waits a part running at `core_clock_hz` needs to watch the
    /// buffers of `plan`.
    ///
    /// The budget covers `WINDOW_LAPS` laps of the buffer at one core cycle
    /// per poll, and a poll is one `AudioInterface::read`, which is many cycles
    /// rather than one. So the count is a floor on the time the window holds
    /// and never a ceiling, and overshooting only lengthens a boot that is
    /// going to end silent anyway.
    ///
    /// The lap comes off the plan rather than off a figure written here, so a
    /// buffer of another length carries its own window.
    #[must_use]
    pub const fn for_transport(plan: &TransportPlan, core_clock_hz: u32) -> Self
    {
        Self
        {
            lap_polls: wait_polls(core_clock_hz, window_microseconds(plan)),
        }
    }
}

/// Returns the microseconds the window budget covers for `plan`.
///
/// A plan `validate` accepted holds a whole number of frames, so the division
/// below is exact and its fallback stands only to keep the expression total.
#[expect
(
    clippy::cast_possible_truncation,
    reason = "the comparison below is what keeps the value inside u32"
)]
const fn window_microseconds(plan: &TransportPlan) -> u32
{
    let slots = (plan.slot_count_field() as u32).saturating_add(1);

    let frames = match plan.transfer_items().checked_div(slots)
    {
        Some(count) => count,
        None => 0,
    };

    let microseconds = (frames as u64)
        .saturating_mul(MICROSECONDS_PER_SECOND as u64)
        .saturating_mul(WINDOW_LAPS as u64)
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

/// One transfer counter, and the reloads seen on it.
pub(crate) struct Laps
{
    last: u32,
    count: u32,
}

impl Laps
{
    /// Starts the count from the reading the window opened on.
    pub(crate) const fn opening(items: u32) -> Self
    {
        Self { last: items, count: 0 }
    }

    /// Folds one reading in.
    ///
    /// RM0433 section 15.5.6 has the counter decrement on each transfer and
    /// reload at the end of a lap in circular mode, so a reading above the one
    /// before it is a reload. Reading slower than a lap misses reloads and
    /// cannot manufacture one.
    pub(crate) fn fold(&mut self, items: u32)
    {
        if items > self.last
        {
            self.count = self.count.saturating_add(1);
        }

        self.last = items;
    }

    /// Returns whether a whole lap of the buffer has gone out since the window
    /// opened.
    ///
    /// Two reloads, not one. The first is reached from wherever the window
    /// found the counter, so it bounds nothing. The second is a whole buffer
    /// after the first, whatever that opening position was.
    pub(crate) const fn lapped(&self) -> bool
    {
        self.count > 1
    }
}

/// Raises the converter mute line once the transport carries the plan.
///
/// The steps run in this order, and the order is the point.
///
/// The two FIFO error flags go down first, in one call, because the fill that
/// starts the transport raises them before it has carried anything. Nothing
/// else is cleared, and nothing is cleared again after this, so every flag the
/// window goes on to read belongs to the window.
///
/// The transfers are watched next, while the line is still low, so nothing
/// this step refuses was ever audible. The window ends when both counters have
/// reloaded twice, which is a whole lap of the buffer from wherever they were
/// found, and every poll of it reads the two sub-blocks and the two streams for
/// an alarm.
///
/// The line is driven low next and only then handed to the port driver, which
/// takes it from a pull-down to a driver without passing through a high state.
/// The raise comes after both.
///
/// The hold follows the raise. The caller has the buffers holding zeros, so
/// what the converter walks its unmute ramp over is silence.
///
/// The transport is read once more after the hold. An alarm there means the
/// zeros did not run for the whole ramp, and the line goes back down before
/// the fault is returned.
///
/// `_clock` and `_filters` are not read. They are the witnesses that the audio
/// kernel clock came up to its plan and that the crossover chain stands in the
/// memory the carry reads, and taking them by reference is what leaves the
/// order to the compiler rather than to a comment.
///
/// # Errors
///
/// `Sequence` when the counters never lapped inside the budget, and one of the
/// four site variants per alarm, each carrying the phase that says whether the
/// mute line had been raised when it was seen. A refusal leaves the line low
/// and builds no `TonePermit`, so a caller that answers a refusal by staying
/// silent is silent.
pub fn open<I, M, W, F>
(
    interface: &mut I,
    line: &mut M,
    _clock: &W,
    _filters: &F,
    plan: &TransportPlan,
    waits: ReleaseWaits
) -> Result<TonePermit, ReleaseFault>
where
    I: AudioInterface + ?Sized,
    M: ConverterMute + ?Sized,
    W: VerifiedClock + ?Sized,
    F: InitialisedFilters + ?Sized,
{
    interface.clear_fifo_errors();

    watch(interface, plan, waits)?;

    line.drive_low();
    line.take_output();
    line.drive_high();
    line.hold();

    if let Err(fault) = check(&interface.read(), plan, Phase::Unmuted)
    {
        line.drive_low();
        return Err(fault);
    }

    Ok(TonePermit(()))
}

/// Watches both transfer counters until each has reloaded twice.
///
/// The first reading opens the window and the budget is spent on the ones
/// after it, so a budget of zero still buys one look at every alarm.
///
/// The interface comes in by shared reference, which is what leaves every
/// write of it, the wipe of the FIFO error flags included, outside the loop. A
/// wipe per poll would take down a flag raised between two of them.
fn watch<I>
(
    interface: &I,
    plan: &TransportPlan,
    waits: ReleaseWaits
) -> Result<(), ReleaseFault>
where
    I: AudioInterface + ?Sized,
{
    let opening = interface.read();

    check(&opening, plan, Phase::Muted)?;

    let mut master = Laps::opening(opening.master_stream.items);
    let mut slave = Laps::opening(opening.slave_stream.items);
    let mut remaining = waits.lap_polls;

    while !master.lapped() || !slave.lapped()
    {
        if remaining == 0
        {
            return Err(ReleaseFault::Sequence(SequenceRefusal::TransfersNeverLapped));
        }

        remaining = remaining.saturating_sub(1);

        let seen = interface.read();

        check(&seen, plan, Phase::Muted)?;
        master.fold(seen.master_stream.items);
        slave.fold(seen.slave_stream.items);
    }

    Ok(())
}

/// Checks one reading of the transport for an alarm.
///
/// The two sub-blocks come first, because an underrun there is what a stream
/// that stops produces at the pins.
fn check
(
    seen: &TransportReadback,
    plan: &TransportPlan,
    phase: Phase
) -> Result<(), ReleaseFault>
{
    if let Err(alarm) = check_block(&seen.master)
    {
        return Err(ReleaseFault::MasterBlock(phase, alarm));
    }

    if let Err(alarm) = check_block(&seen.slave)
    {
        return Err(ReleaseFault::SlaveBlock(phase, alarm));
    }

    if let Err(alarm) = check_stream(&seen.master_stream, plan)
    {
        return Err(ReleaseFault::MasterStream(phase, alarm));
    }

    if let Err(alarm) = check_stream(&seen.slave_stream, plan)
    {
        return Err(ReleaseFault::SlaveStream(phase, alarm));
    }

    Ok(())
}

/// Checks the two error flags of one sub-block, and that it still runs.
fn check_block(seen: &crate::transport::BlockReadback) -> Result<(), BlockAlarm>
{
    if seen.underrun
    {
        return Err(BlockAlarm::Underrun);
    }

    if seen.clock_configuration_rejected
    {
        return Err(BlockAlarm::ClockConfigurationRejected);
    }

    if !seen.enabled
    {
        return Err(BlockAlarm::NotEnabled);
    }

    Ok(())
}

/// Checks the three error flags of one stream, that it still runs, and that
/// its counter is a position inside the buffer.
fn check_stream
(
    seen: &crate::transport::StreamReadback,
    plan: &TransportPlan
) -> Result<(), StreamAlarm>
{
    if seen.transfer_error
    {
        return Err(StreamAlarm::TransferError);
    }

    if seen.fifo_error
    {
        return Err(StreamAlarm::FifoError);
    }

    if seen.direct_mode_error
    {
        return Err(StreamAlarm::DirectModeError);
    }

    if !seen.enabled
    {
        return Err(StreamAlarm::NotEnabled);
    }

    if seen.items > plan.transfer_items()
    {
        return Err(StreamAlarm::CounterOutOfRange);
    }

    Ok(())
}

/// Builds a permit without running the gate.
///
/// The type has no public constructor, which is what makes one evidence that
/// the gate ran, so a test of another module that needs one takes it from here
/// rather than reaching into the field.
#[cfg(test)]
pub(crate) const fn tone_permit_for_test() -> TonePermit
{
    TonePermit(())
}

const _: () = assert!
(
    Phase::Muted as u8 == 0,
    "the muted phase adds no bit to a place byte, so a site on its own is the \
     place a fault seen before the raise reads at"
);

#[cfg(test)]
mod tests
{
    // The tone table is 441 samples and every buffer figure below follows from
    // it, so every conversion between an integer type here is exact.
    #![allow(clippy::cast_possible_truncation)]

    use super::*;
    use crate::clock::AUDIO_PLAN;
    use crate::transport::
    {
        BlockReadback,
        BlockRole,
        SYNC_OUT_BLOCK_A,
        SYNC_OUT_NONE,
        StreamReadback,
        TONE_SAMPLES,
    };
    use core::cell::{Cell, RefCell};

    /// Words in the buffer the tests plan against, one tone period per channel.
    const TEST_WORDS: u32 = TONE_SAMPLES as u32 * 2;

    /// Address of the buffer the master stream replays in the tests.
    const TEST_MASTER_BUFFER: u32 = 0x2400_0000;

    /// Address of the buffer the slave stream replays in the tests.
    const TEST_SLAVE_BUFFER: u32 = TEST_MASTER_BUFFER + TEST_WORDS * 4;

    /// Words a poll of the mock draws out of each buffer.
    ///
    /// One, so a lap costs `TEST_WORDS` polls and the window spends a
    /// countable number of them.
    const WORDS_PER_POLL: u32 = 1;

    /// Polls of the window an alarm is armed on, one at a time.
    ///
    /// The first is the read that opens the window, the rest are reads the
    /// loop makes. The window runs past `TEST_WORDS` polls from any opening
    /// position, so every one of these lands inside it.
    const ALARM_POLLS: [u32; 5] = [1, 2, 3, TEST_WORDS / 2, TEST_WORDS];

    /// Polls a stalling stream runs for before its counter stops.
    ///
    /// Past the first reload of either stream and short of the second, from
    /// both of the opening positions the stall test uses. So the stream has
    /// advanced the way a bring-up sees it, has replayed a whole buffer, and
    /// still owes the window the lap it claims.
    const POLLS_BEFORE_A_STALL: u32 = TEST_WORDS + TEST_WORDS / 3;

    /// Opening positions the two streams are given where they are told apart,
    /// the master then the slave.
    ///
    /// A fraction of a lap between them, so the two counters read different
    /// values on every poll and neither is a reading of the other. Both orders
    /// are swept, because a count folded against the position the other stream
    /// opened at only over-counts when that position is the lower of the two.
    const OPENING_PAIRS: [(u32, u32); 2] =
    [
        (TEST_WORDS / 3, 0),
        (0, TEST_WORDS / 3),
    ];

    /// One broken reading of the transport, and the fault it must produce.
    type Mutation = (fn(&mut TransportReadback), ReleaseFault);

    /// One flag the transport holds, and the fault it must produce.
    type StandingFlag = (fn(&mut MockFlags), ReleaseFault);

    /// One step of the mute line, in the order the gate asks for it.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Step
    {
        DriveLow,
        TakeOutput,
        DriveHigh,
        Hold,
    }

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
    fn waits() -> ReleaseWaits
    {
        ReleaseWaits::for_transport(&plan(), 64_000_000)
    }

    /// A clock witness, which the gate takes and does not read.
    struct TestClock;

    impl VerifiedClock for TestClock
    {
    }

    /// A filter witness, which the gate takes and does not read.
    struct TestFilters;

    impl InitialisedFilters for TestFilters
    {
    }

    /// A mute line that records what the gate asked of it.
    struct MockLine
    {
        steps: RefCell<[Option<Step>; 8]>,
        taken: Cell<usize>,
        high: Cell<bool>,
        driven: Cell<bool>,
    }

    impl MockLine
    {
        fn new() -> Self
        {
            Self
            {
                steps: RefCell::new([None; 8]),
                taken: Cell::new(0),
                high: Cell::new(false),
                driven: Cell::new(false),
            }
        }

        fn record(&self, step: Step)
        {
            let index = self.taken.get();

            if let Some(slot) = self.steps.borrow_mut().get_mut(index)
            {
                *slot = Some(step);
            }

            self.taken.set(index.saturating_add(1));
        }

        /// Returns the steps taken, in order.
        fn taken(&self) -> [Option<Step>; 8]
        {
            *self.steps.borrow()
        }
    }

    impl ConverterMute for MockLine
    {
        fn drive_low(&mut self)
        {
            self.record(Step::DriveLow);
            self.high.set(false);
        }

        fn take_output(&mut self)
        {
            self.record(Step::TakeOutput);

            // A pad handed to the driver takes up whatever the output register
            // holds, which is what the call before this one put there.
            self.driven.set(true);
        }

        fn drive_high(&mut self)
        {
            self.record(Step::DriveHigh);
            self.high.set(true);
        }

        fn hold(&mut self)
        {
            self.record(Step::Hold);
        }
    }

    /// One stream of the mock, and how its counter moves.
    ///
    /// The two streams of an interface carry one of these each, so the mock
    /// can put the master and the slave in different states. Nothing here is
    /// shared between them: the position the window opens on, the poll the
    /// counter stops at and the buffer the stream replays all belong to the
    /// stream, the way the part derives each of the three per stream.
    #[derive(Debug, Clone, Copy)]
    struct MockStream
    {
        /// Words drawn out of the buffer before the first poll, which is what
        /// sets the position the window finds the counter at.
        opened_at: u32,
        /// Polls after which the counter stops moving.
        stalls_after: u32,
        /// Buffer this stream replays.
        buffer: u32,
    }

    impl MockStream
    {
        /// Builds a stream replaying `buffer` from the top of it.
        const fn running(buffer: u32) -> Self
        {
            Self { opened_at: 0, stalls_after: u32::MAX, buffer }
        }

        /// Returns the reading this stream gives on poll `polls`.
        fn reading(&self, polls: u32) -> StreamReadback
        {
            let advanced = polls.min(self.stalls_after).saturating_mul(WORDS_PER_POLL);
            let drawn = self.opened_at.saturating_add(advanced);

            running_stream(counter(drawn), self.buffer)
        }
    }

    /// The error flags the interface holds, as its registers hold them.
    ///
    /// Every one of them is sticky: the part raises it and it stands until
    /// software writes the clear bit that names it. `clear_fifo_errors` is the
    /// only thing here that writes one, so what a reading carries is what was
    /// raised minus what that call took down.
    #[derive(Debug, Clone, Copy)]
    #[expect
    (
        clippy::struct_excessive_bools,
        reason = "each field is one register flag, and naming them apart is \
                  what lets a test raise one"
    )]
    struct MockFlags
    {
        /// `FEIF` of both streams.
        fifo_error: bool,
        /// `TEIF` of both streams.
        transfer_error: bool,
        /// `DMEIF` of both streams.
        direct_mode_error: bool,
        /// `OVRUDR` of both sub-blocks.
        underrun: bool,
    }

    impl MockFlags
    {
        /// Returns the flags of a transport with none of them raised.
        const fn none() -> Self
        {
            Self
            {
                fifo_error: false,
                transfer_error: false,
                direct_mode_error: false,
                underrun: false,
            }
        }
    }

    /// Puts the flags the interface holds into one reading of it.
    fn apply_flags(seen: &mut TransportReadback, flags: MockFlags)
    {
        seen.master.underrun = flags.underrun;
        seen.slave.underrun = flags.underrun;

        for stream in [&mut seen.master_stream, &mut seen.slave_stream]
        {
            stream.fifo_error = flags.fifo_error;
            stream.transfer_error = flags.transfer_error;
            stream.direct_mode_error = flags.direct_mode_error;
        }
    }

    /// An interface whose transfer counters walk down and reload.
    ///
    /// The only write it answers is `clear_fifo_errors`. The gate makes no
    /// other one, and every remaining method is a no-op that says so.
    struct MockInterface
    {
        polls: Cell<u32>,
        master: MockStream,
        slave: MockStream,
        /// Flags standing right now. Sticky, and held behind a cell because a
        /// read is what raises one.
        flags: Cell<MockFlags>,
        /// Poll after which the transport holds `FEIF` up.
        raises_fifo_error_at: u32,
        alarm_at: u32,
        alarm_polls: u32,
        alarm: fn(&mut TransportReadback),
        /// `SYNCOUT` of the interface, at its reset value until the sequence
        /// writes it. Derived rather than announced, so a sequence that stops
        /// declaring the source reads back as one that never did.
        sync_out_bits: Cell<u8>,
    }

    impl MockInterface
    {
        /// Builds an interface whose two streams run to plan.
        fn healthy() -> Self
        {
            Self
            {
                polls: Cell::new(0),
                master: MockStream::running(TEST_MASTER_BUFFER),
                slave: MockStream::running(TEST_SLAVE_BUFFER),
                flags: Cell::new(MockFlags::none()),
                raises_fifo_error_at: u32::MAX,
                alarm_at: u32::MAX,
                alarm_polls: 0,
                alarm: |_| {},
                sync_out_bits: Cell::new(SYNC_OUT_NONE),
            }
        }

        /// Raises a flag, the way the part does, before anything reads it.
        fn hold(&mut self, flags: MockFlags)
        {
            self.flags.set(flags);
        }

        /// Arms `raise_it` on `polls` reads starting at poll `at`.
        ///
        /// An alarm that lasts one poll is what tells the read that saw it
        /// from the reads around it, so dropping one of the two alarm checks
        /// stops the gate seeing the alarm rather than moving which check
        /// catches it.
        fn arm(&mut self, at: u32, polls: u32, raise_it: fn(&mut TransportReadback))
        {
            self.alarm_at = at;
            self.alarm_polls = polls;
            self.alarm = raise_it;
        }
    }

    /// Returns the counter reading after `drawn` words have gone out.
    fn counter(drawn: u32) -> u32
    {
        let position = drawn.checked_rem(TEST_WORDS).unwrap_or(0);

        TEST_WORDS.saturating_sub(position)
    }

    /// Returns one sub-block reading the way a running one does.
    fn running_block() -> BlockReadback
    {
        BlockReadback
        {
            mode_bits: 0,
            protocol_bits: 0,
            data_size_bits: 0b111,
            lsb_first: false,
            changes_on_falling_edge: true,
            sync_bits: 0,
            mono: false,
            driven_before_start: false,
            tristate: false,
            fifo_threshold_bits: 0b010,
            no_master_clock: false,
            master_divider_field: 2,
            oversampling: false,
            frame_length_field: 63,
            frame_active_field: 31,
            frame_marks_channel: true,
            frame_active_high: false,
            frame_leads_first_bit: true,
            first_bit_offset_field: 0,
            slot_size_bits: 0b10,
            slot_count_field: 1,
            slot_enable_bits: 0b11,
            transfer_enabled: true,
            enabled: true,
            fifo_level_bits: 0b101,
            underrun: false,
            clock_configuration_rejected: false,
            any_interrupt_enabled: false,
        }
    }

    /// Returns one stream reading the way a running one replaying `buffer` does.
    fn running_stream(items: u32, buffer: u32) -> StreamReadback
    {
        StreamReadback
        {
            request_bits: 87,
            sync_enabled: false,
            direction_bits: 0b01,
            circular: true,
            memory_increments: true,
            peripheral_increments: false,
            memory_width_bits: 0b10,
            peripheral_width_bits: 0b10,
            double_buffered: false,
            priority_bits: 0b10,
            any_interrupt_enabled: false,
            peripheral_address: 0x4001_5820,
            memory_address: buffer,
            items,
            enabled: true,
            transfer_error: false,
            direct_mode_error: false,
            fifo_error: false,
        }
    }

    impl AudioInterface for MockInterface
    {
        fn stop_streams(&mut self)
        {
        }

        fn stop_blocks(&mut self)
        {
        }

        fn declare_sync_source(&mut self)
        {
            self.sync_out_bits.set(SYNC_OUT_BLOCK_A);
        }

        fn open_pins(&mut self)
        {
        }

        fn write_master(&mut self, _plan: &TransportPlan)
        {
        }

        fn write_slave(&mut self, _plan: &TransportPlan)
        {
        }

        fn write_master_stream(&mut self, _plan: &TransportPlan)
        {
        }

        fn write_slave_stream(&mut self, _plan: &TransportPlan)
        {
        }

        fn start_streams(&mut self)
        {
        }

        fn start_slave(&mut self)
        {
        }

        fn start_master(&mut self)
        {
        }

        fn read(&self) -> TransportReadback
        {
            let polls = self.polls.get().saturating_add(1);
            self.polls.set(polls);

            let mut seen = TransportReadback
            {
                master: running_block(),
                slave: running_block(),
                master_stream: self.master.reading(polls),
                slave_stream: self.slave.reading(polls),
                sync_out_bits: self.sync_out_bits.get(),
            };

            apply_flags(&mut seen, self.flags.get());

            if polls >= self.alarm_at
                && polls.saturating_sub(self.alarm_at) < self.alarm_polls
            {
                (self.alarm)(&mut seen);
            }

            // The raise lands after the reading has been taken, which is where
            // a condition met between two polls puts it. So a flag raised on
            // poll n is one poll n + 1 finds standing, and a wipe run between
            // the two is a wipe that takes it down unread.
            if polls >= self.raises_fifo_error_at
            {
                let mut flags = self.flags.get();
                flags.fifo_error = true;
                self.flags.set(flags);
            }

            seen
        }

        /// Writes the clear bit of `FEIF`, and of no other flag.
        fn clear_fifo_errors(&mut self)
        {
            let mut flags = self.flags.get();
            flags.fifo_error = false;
            self.flags.set(flags);
        }
    }

    /// Runs the gate on an interface and returns what it did.
    fn run(interface: &mut MockInterface) -> (Result<TonePermit, ReleaseFault>, MockLine)
    {
        let mut line = MockLine::new();
        let outcome = open
        (
            interface,
            &mut line,
            &TestClock,
            &TestFilters,
            &plan(),
            waits()
        );

        (outcome, line)
    }

    /// Opening positions the window is exercised from.
    ///
    /// The two ends of a lap and a few points inside it. `TEST_WORDS - 2` is
    /// the adverse one: it puts a reload one poll after the window opens, so a
    /// gate that stopped on a first reload would run for two polls there.
    const OPENING_POSITIONS: [u32; 8] =
    [
        0,
        1,
        2,
        TEST_WORDS / 4,
        TEST_WORDS / 2,
        TEST_WORDS - 2,
        TEST_WORDS - 1,
        TEST_WORDS,
    ];

    #[test]
    fn a_healthy_transport_gets_the_line_raised_and_a_permit()
    {
        let mut interface = MockInterface::healthy();
        let (outcome, line) = run(&mut interface);

        assert!(outcome.is_ok());
        assert!(line.high.get());
        assert!(line.driven.get());
    }

    #[test]
    fn the_line_is_driven_low_before_the_port_takes_it_and_the_hold_follows()
    {
        // The pad is driven low before it is handed to the driver, so it never
        // passes through a high nobody asked for, and the hold comes after the
        // raise rather than before it, or the zeros would run out before the
        // converter has finished ramping.
        let mut interface = MockInterface::healthy();
        let (outcome, line) = run(&mut interface);

        assert!(outcome.is_ok());
        assert_eq!
        (
            line.taken(),
            [
                Some(Step::DriveLow),
                Some(Step::TakeOutput),
                Some(Step::DriveHigh),
                Some(Step::Hold),
                None,
                None,
                None,
                None,
            ]
        );
    }

    #[test]
    fn the_window_watches_a_whole_lap_from_every_opening_position()
    {
        // The window opens on whatever the counters read, so the run up to a
        // first reload is a fraction of a lap and says nothing. What the gate
        // claims is a whole lap, which only the second reload buys, and a
        // single opening position cannot tell the two apart: from one poll
        // before a reload, stopping on the first would end the window in two
        // polls. So the position is swept.
        //
        // The mock draws one word a poll, so a lap costs TEST_WORDS polls, and
        // open reads once more after the hold than the window spent.
        for opened_at in OPENING_POSITIONS
        {
            let mut interface = MockInterface::healthy();
            interface.master.opened_at = opened_at;
            interface.slave.opened_at = opened_at;

            let (outcome, _line) = run(&mut interface);
            let watched = interface.polls.get().saturating_sub(1);

            assert!
            (
                outcome.is_ok(),
                "opening at {opened_at} refused a healthy transport"
            );
            assert!
            (
                watched > TEST_WORDS,
                "opening at {opened_at} closed the window after {watched} polls, \
                 a lap of the buffer costs {TEST_WORDS}"
            );
        }
    }

    #[test]
    fn a_counter_that_never_reloads_refuses_the_release()
    {
        let mut interface = MockInterface::healthy();
        interface.master.stalls_after = 0;
        interface.slave.stalls_after = 0;

        let (outcome, line) = run(&mut interface);

        assert_eq!
        (
            outcome.err(),
            Some(ReleaseFault::Sequence(SequenceRefusal::TransfersNeverLapped))
        );
        assert!(!line.high.get());
        assert_eq!(line.taken().first(), Some(&None));
    }

    #[test]
    fn a_counter_that_advances_once_and_stops_refuses_the_release()
    {
        // This is the case a bring-up cannot see. One advance proves a
        // transfer happened, and the window is what proves the stream keeps
        // going.
        let mut interface = MockInterface::healthy();
        interface.master.stalls_after = 1;
        interface.slave.stalls_after = 1;

        let (outcome, line) = run(&mut interface);

        assert_eq!
        (
            outcome.err(),
            Some(ReleaseFault::Sequence(SequenceRefusal::TransfersNeverLapped))
        );
        assert!(!line.high.get());
    }

    #[test]
    fn one_stream_that_stalls_refuses_the_release_whichever_of_the_two_it_is()
    {
        // The slave stream feeds the sub-block carrying two of the four
        // converter channels, so a window that watched the master alone would
        // raise the line over a slave that had stopped. The stall comes after
        // that stream has replayed a whole buffer, which is the case a bring-up
        // passes and the one a window ending on a first reload would accept.
        //
        // The two streams open a third of a lap apart, so the count one carries
        // is a count of its own readings and of nothing else. A count folded
        // against the position the other stream opened at reads a reload that
        // stream never ran, and the stall is what is left to catch it. Which of
        // the two counts that over-reads depends on which position is the
        // lower, so both orders are swept.
        for (master_opens_at, slave_opens_at) in OPENING_PAIRS
        {
            for stalling_slave in [false, true]
            {
                let mut interface = MockInterface::healthy();
                interface.master.opened_at = master_opens_at;
                interface.slave.opened_at = slave_opens_at;

                if stalling_slave
                {
                    interface.slave.stalls_after = POLLS_BEFORE_A_STALL;
                }
                else
                {
                    interface.master.stalls_after = POLLS_BEFORE_A_STALL;
                }

                let (outcome, line) = run(&mut interface);
                let stream = if stalling_slave { "slave" } else { "master" };

                assert_eq!
                (
                    outcome.err(),
                    Some(ReleaseFault::Sequence(SequenceRefusal::TransfersNeverLapped)),
                    "a stall on the {stream} stream did not refuse, with the \
                     master opening at {master_opens_at} and the slave at \
                     {slave_opens_at}"
                );
                assert!(!line.high.get());
                assert_eq!(line.taken().first(), Some(&None));
            }
        }
    }

    #[test]
    fn each_stream_reads_back_the_buffer_the_plan_gave_it()
    {
        // The plan names two buffers and the two streams replay one each, so a
        // reading that gave both the same address would describe a transport
        // this plan does not build.
        let interface = MockInterface::healthy();
        let seen = interface.read();

        assert_eq!
        (
            seen.master_stream.memory_address,
            plan().buffer_address(BlockRole::Master)
        );
        assert_eq!
        (
            seen.slave_stream.memory_address,
            plan().buffer_address(BlockRole::Slave)
        );
        assert_ne!(seen.master_stream.memory_address, seen.slave_stream.memory_address);
    }

    #[test]
    fn every_alarm_of_the_window_refuses_before_the_line_moves()
    {
        let cases: [Mutation; 11] =
        [
            (
                |seen| seen.master.underrun = true,
                ReleaseFault::MasterBlock(Phase::Muted, BlockAlarm::Underrun),
            ),
            (
                |seen| seen.master.clock_configuration_rejected = true,
                ReleaseFault::MasterBlock
                (
                    Phase::Muted,
                    BlockAlarm::ClockConfigurationRejected
                ),
            ),
            (
                |seen| seen.master.enabled = false,
                ReleaseFault::MasterBlock(Phase::Muted, BlockAlarm::NotEnabled),
            ),
            (
                |seen| seen.slave.underrun = true,
                ReleaseFault::SlaveBlock(Phase::Muted, BlockAlarm::Underrun),
            ),
            (
                |seen| seen.master_stream.transfer_error = true,
                ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::TransferError),
            ),
            (
                |seen| seen.master_stream.fifo_error = true,
                ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::FifoError),
            ),
            (
                |seen| seen.master_stream.direct_mode_error = true,
                ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::DirectModeError),
            ),
            (
                |seen| seen.master_stream.enabled = false,
                ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::NotEnabled),
            ),
            (
                |seen| seen.master_stream.items = TEST_WORDS + 1,
                ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::CounterOutOfRange),
            ),
            (
                |seen| seen.slave_stream.transfer_error = true,
                ReleaseFault::SlaveStream(Phase::Muted, StreamAlarm::TransferError),
            ),
            (
                |seen| seen.slave_stream.fifo_error = true,
                ReleaseFault::SlaveStream(Phase::Muted, StreamAlarm::FifoError),
            ),
        ];

        // The alarm lasts ONE poll and the poll it lands on is swept across
        // the window: the opening read, the reads just after it, and reads
        // well inside it. A persistent alarm cannot separate the two reads the
        // gate makes, since either one catches it and the other is free to go.
        // A single-poll alarm at poll 1 is seen only by the opening read, and
        // one at any later poll only by the read inside the loop.
        for at in ALARM_POLLS
        {
            for (raise_it, expected) in cases
            {
                let mut interface = MockInterface::healthy();
                interface.arm(at, 1, raise_it);

                let (outcome, line) = run(&mut interface);

                assert_eq!
                (
                    outcome.err(),
                    Some(expected),
                    "{expected:?} at poll {at} was not reported"
                );
                assert!(!line.high.get(), "{expected:?} at poll {at} left the line high");
                assert_eq!
                (
                    line.taken().first(),
                    Some(&None),
                    "{expected:?} at poll {at} moved the line"
                );
            }
        }
    }

    #[test]
    fn a_fifo_error_the_start_up_left_standing_does_not_refuse()
    {
        // Both streams raise FEIF while their FIFOs fill from empty, so the
        // flag is up before the gate has read anything, and it is sticky. The
        // window opens on the read that follows the wipe, so this is the one
        // reading of that flag the gate is not entitled to.
        let mut interface = MockInterface::healthy();
        interface.hold(MockFlags { fifo_error: true, ..MockFlags::none() });

        let (outcome, line) = run(&mut interface);

        assert!
        (
            outcome.is_ok(),
            "a FIFO error the wipe took down before the window still refused"
        );
        assert!(line.high.get(), "the window refused and left the mute line low");
    }

    #[test]
    fn a_fifo_error_raised_inside_the_window_refuses_wherever_it_lands()
    {
        // The wipe runs once and ahead of the opening read, so every flag from
        // there on is one the window measures. The poll the transport raises it
        // on is swept over the window, and it stands from that poll to the end,
        // which is what a wipe inside the polling loop would take down.
        for at in ALARM_POLLS
        {
            let mut interface = MockInterface::healthy();
            interface.raises_fifo_error_at = at;

            let (outcome, line) = run(&mut interface);

            assert_eq!
            (
                outcome.err(),
                Some(ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::FifoError)),
                "a FIFO error the transport raised on poll {at} did not refuse"
            );
            assert!
            (
                !line.high.get(),
                "a FIFO error the transport raised on poll {at} left the line high"
            );
        }
    }

    #[test]
    fn the_wipe_reaches_the_fifo_error_and_leaves_every_other_flag_standing()
    {
        // Each of these three is raised by something the start-up does not do:
        // RM0433 section 15.3.20 gives TEIF a bus error and confines DMEIF to a
        // peripheral to memory stream in direct mode, and a sub-block raises
        // OVRUDR on a frame it had no data for. So each of them is significant
        // at any instant, and a wipe that reached one would drop a reading
        // nothing else carries. Each is held from the construction of the mock,
        // beside the FIFO error the wipe is entitled to.
        let cases: [StandingFlag; 3] =
        [
            (
                |flags| flags.transfer_error = true,
                ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::TransferError),
            ),
            (
                |flags| flags.direct_mode_error = true,
                ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::DirectModeError),
            ),
            (
                |flags| flags.underrun = true,
                ReleaseFault::MasterBlock(Phase::Muted, BlockAlarm::Underrun),
            ),
        ];

        for (raise_it, expected) in cases
        {
            let mut flags = MockFlags { fifo_error: true, ..MockFlags::none() };
            raise_it(&mut flags);

            let mut interface = MockInterface::healthy();
            interface.hold(flags);

            let (outcome, line) = run(&mut interface);

            assert_eq!
            (
                outcome.err(),
                Some(expected),
                "the wipe of the FIFO error took {expected:?} down with it"
            );
            assert!(!line.high.get(), "{expected:?} left the mute line high");
        }
    }

    #[test]
    fn an_alarm_raised_across_the_hold_drives_the_line_back_low()
    {
        // The read that follows the hold is the last one the gate makes, so a
        // healthy run is what says which poll it is rather than a figure
        // written here. An alarm armed there is one the gate can only see with
        // the mute line already high.
        let mut healthy = MockInterface::healthy();
        let (settled, _line) = run(&mut healthy);

        assert!(settled.is_ok());

        let mut interface = MockInterface::healthy();
        interface.arm(healthy.polls.get(), 1, |seen| seen.master.underrun = true);

        let (outcome, line) = run(&mut interface);

        assert_eq!
        (
            outcome.err(),
            Some(ReleaseFault::MasterBlock(Phase::Unmuted, BlockAlarm::Underrun))
        );
        assert!(!line.high.get());
        assert_eq!
        (
            line.taken(),
            [
                Some(Step::DriveLow),
                Some(Step::TakeOutput),
                Some(Step::DriveHigh),
                Some(Step::Hold),
                Some(Step::DriveLow),
                None,
                None,
                None,
            ]
        );
    }

    #[test]
    fn a_code_names_the_phase_the_site_and_the_cause_apart()
    {
        // The place byte is a phase over a site and the cause byte is the
        // discriminant of the alarm, so rustc keeps two causes of one site
        // apart. What is left to read here is that the three fields separate.
        assert_eq!
        (
            ReleaseFault::Sequence(SequenceRefusal::TransfersNeverLapped).code(),
            0x0001
        );
        assert_eq!
        (
            ReleaseFault::MasterBlock(Phase::Muted, BlockAlarm::Underrun).code(),
            0x0101
        );
        assert_eq!
        (
            ReleaseFault::SlaveBlock(Phase::Muted, BlockAlarm::NotEnabled).code(),
            0x0203
        );
        assert_eq!
        (
            ReleaseFault::MasterStream(Phase::Muted, StreamAlarm::TransferError).code(),
            0x0301
        );
        assert_eq!
        (
            ReleaseFault::SlaveStream(Phase::Muted, StreamAlarm::CounterOutOfRange).code(),
            0x0405
        );
        assert_eq!
        (
            ReleaseFault::MasterBlock(Phase::Unmuted, BlockAlarm::Underrun).code(),
            0x1101
        );
        assert_eq!
        (
            ReleaseFault::SlaveStream(Phase::Unmuted, StreamAlarm::FifoError).code(),
            0x1402
        );
    }

    #[test]
    fn one_alarm_under_two_phases_reads_as_two_codes()
    {
        // Whether the mute line went up is the one thing a person reading a
        // parked board most needs off this word, and nothing in the type
        // system says the two phases number apart, so it is read here.
        let sites =
        [
            (
                ReleaseFault::MasterBlock(Phase::Muted, BlockAlarm::Underrun),
                ReleaseFault::MasterBlock(Phase::Unmuted, BlockAlarm::Underrun),
            ),
            (
                ReleaseFault::SlaveStream(Phase::Muted, StreamAlarm::NotEnabled),
                ReleaseFault::SlaveStream(Phase::Unmuted, StreamAlarm::NotEnabled),
            ),
        ];

        for (muted, unmuted) in sites
        {
            assert_eq!(muted.code() & 0xFF, unmuted.code() & 0xFF);
            assert_ne!(muted.code(), unmuted.code());
            assert_eq!(muted.code() >> 12, 0);
            assert_eq!(unmuted.code() >> 12, 1);
        }
    }

    #[test]
    fn the_widest_code_the_encoding_can_carry_stays_under_the_ceiling()
    {
        // The ceiling bounds the whole encoding, not the values it reaches.
        let widest =
            ReleaseFault::SlaveStream(Phase::Unmuted, StreamAlarm::CounterOutOfRange).code();

        assert_eq!(widest, 0x1405);
        assert!(widest < RELEASE_CODE_CEILING);
        assert_eq!(RELEASE_CODE_CEILING, 0xFFFF);
    }

    #[test]
    fn the_window_budget_covers_the_laps_the_window_can_spend()
    {
        // 441 frames at 44.1 kHz is a lap of 10 ms. The window spends the rest
        // of the lap it opened in and then a whole one, so two laps is what it
        // can take and the budget carries a third. It comes off the plan
        // rather than off a figure written here.
        assert_eq!(WINDOW_LAPS, 3);
        assert_eq!(window_microseconds(&plan()), 30_000);
        assert_eq!(waits().lap_polls, wait_polls(64_000_000, 30_000));
    }

    #[test]
    fn a_longer_buffer_carries_a_longer_window()
    {
        let long = TransportPlan::for_clock
        (
            &AUDIO_PLAN,
            TEST_MASTER_BUFFER,
            TEST_MASTER_BUFFER + TEST_WORDS * 4 * 3,
            TEST_WORDS * 3
        );

        assert_eq!(long.validate(), Ok(()));
        assert_eq!(window_microseconds(&long), 90_000);
    }

    #[test]
    fn a_reading_that_only_falls_never_counts_a_lap()
    {
        // The one direction the counter moves in on its own is down, so this
        // is what says a fall is not read as a reload.
        let mut laps = Laps::opening(882);

        for items in [881_u32, 400, 2, 1]
        {
            laps.fold(items);
            assert_eq!(laps.count, 0);
        }

        laps.fold(882);
        assert_eq!(laps.count, 1);
    }

    #[test]
    fn one_reload_is_not_a_whole_lap()
    {
        // The window opens wherever the counter stands, so the first reload
        // comes a fraction of a lap later and bounds nothing. The second is a
        // whole buffer after the first, and that is what the gate waits for.
        let mut laps = Laps::opening(400);

        laps.fold(882);
        assert_eq!(laps.count, 1);
        assert!(!laps.lapped());

        for items in [881_u32, 400, 1]
        {
            laps.fold(items);
            assert!(!laps.lapped());
        }

        laps.fold(882);
        assert_eq!(laps.count, 2);
        assert!(laps.lapped());
    }

    #[test]
    fn a_wait_of_zero_polls_still_looks_once()
    {
        let mut interface = MockInterface::healthy();
        let mut line = MockLine::new();
        let waits = ReleaseWaits { lap_polls: 0 };
        let outcome = open
        (
            &mut interface,
            &mut line,
            &TestClock,
            &TestFilters,
            &plan(),
            waits
        );

        assert_eq!
        (
            outcome.err(),
            Some(ReleaseFault::Sequence(SequenceRefusal::TransfersNeverLapped))
        );
        assert_eq!(interface.polls.get(), 1);
    }
}
