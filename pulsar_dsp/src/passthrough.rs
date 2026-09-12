//! Input path of the processing board, at the register.
//!
//! `pulsar_lib::passthrough` holds the plan, the buffer rules, the bring-up
//! sequence, the decision one transfer event produces, the crossover chain, and
//! the loop that walks one half of the receiving buffer into the two output
//! buffers. This module is the register block that sequence runs on, the buffer
//! the receiving stream fills, the memory the chain is parked in, and the two
//! accessors that loop reaches memory through.
//!
//! It touches neither PE7 nor XSMT, and it writes no output buffer until the
//! release gate has handed its permit to `arm`.
//!
//! # Where the buffer lives
//!
//! In the AXI SRAM, through the `.axisram` section, for the reason the output
//! buffers are there: the two system transfer controllers reach every internal
//! memory except the two tightly coupled ones, so a buffer in the data memory
//! this firmware runs its stack and its statics from would be written nowhere
//! with no flag raised.
//!
//! The section is not loaded at startup, so what lands there out of reset is
//! whatever the memory held. `start` fills it with zeros before the stream that
//! writes it is enabled, so a data line carrying nothing puts silence into the
//! output buffers rather than full scale noise.
//!
//! The data cache is not enabled anywhere in this binary, so a buffer the
//! transfer controller writes is a buffer the core reads. Enabling it makes
//! these reads need maintenance before the core sees what arrived.
//!
//! # What runs after the bring-up
//!
//! One interrupt, and it is the first this firmware serves. The receiving
//! stream raises its line at each half of its buffer, and the handler walks the
//! frames of that half through the crossover and out across the four converter
//! channels, at the positions those frames were sent from, through
//! `pulsar_lib::passthrough::carry_block` and the two accessors below it. Where
//! that is comes off the distance between the two transfer counters, less the
//! pipeline of the FIFOs and the shift registers that no counter reports, which
//! the bring-up measures and leaves in `CARRY_SHIFT`.
//!
//! That vector no longer reaches the handler that silences the machine, so this
//! module carries the net for it: `pulsar_lib::passthrough::next_block` refuses
//! every entry that is not one half transfer or one transfer complete on a path
//! still running to plan, and refuses one that would write where the
//! transmitting streams are reading. The caller answers a refusal by silencing
//! the machine, which is what the default handler would have done.

use core::mem::MaybeUninit;
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m::peripheral::NVIC;
use pulsar_lib::passthrough::
{
    CarryShift,
    Event,
    EventFault,
    FilterChain,
    Half,
    INPUT_REQUEST,
    InputBlockReadback,
    InputInterface,
    InputPlan,
    InputReadback,
    InputSequenceFault,
    InputStreamReadback,
    InputWaits,
    MODE_SLAVE_RECEIVER,
    PassthroughFault,
    Passthrough,
    SYNC_EXTERNAL,
    TRANSFER_PERIPHERAL_TO_MEMORY,
    arm,
    bring_up,
};
use pulsar_lib::clock::{AUDIO_PLAN, ClockPlan};
use pulsar_lib::release::{InitialisedFilters, TonePermit};
use pulsar_lib::transport::{BlockRole, STREAM_PRIORITY, SYNC_OUT_NONE, TRANSFER_WORD};
use stm32h7::stm32h743v::dma1::st::cr::{DIR, PL, PSIZE};
use stm32h7::stm32h743v::dmamux1::ccr::DMAREQ_ID;
use stm32h7::stm32h743v::gpioc::afrh::ALTERNATE_FUNCTION;
use stm32h7::stm32h743v::gpioc::moder::MODE as PIN_MODE;
use stm32h7::stm32h743v::gpioc::pupdr::PULL;
use stm32h7::stm32h743v::sai1::ch::cr1::{MODE, SYNCEN};
use stm32h7::stm32h743v::{DMA1, DMAMUX1, GPIOD, Interrupt, RCC, SAI2};

use crate::clock::AudioClock;
use crate::transport::{self, BUFFER_WORDS};

// Every value the plan names for a field this module writes is pinned here
// against the encoding the peripheral crate carries for it, for the reason the
// output transport pins its own: a plan value that does not mean what the
// register means passes every host test and reaches the pins.
//
// Two of them cannot be pinned and are closed another way. `SYNCIN` is a plain
// two bit field in the peripheral crate with no variant named for any of its
// values, so no assertion can call one, and what stands there is RM0433 section
// 51.4.4 table 420 quoted at the constant. The data register address is a
// constant of the plan, and the write below takes it from the peripheral crate
// instead, so the read-back comparing the two is what closes it: a constant
// naming another address refuses the bring-up.

const _: () = assert!
(
    MODE_SLAVE_RECEIVER == MODE::SlaveRx as u8
        && SYNC_EXTERNAL == SYNCEN::External as u8,
    "the direction and the synchronisation the plan names encode the way MODE \
     and SYNCEN do"
);

const _: () = assert!
(
    INPUT_REQUEST == DMAREQ_ID::Sai2aDma as u8
        && TRANSFER_PERIPHERAL_TO_MEMORY == DIR::PeripheralToMemory as u8,
    "the request and the direction the plan names encode the way DMAREQ_ID and \
     DIR do"
);

const _: () = assert!
(
    TRANSFER_WORD == PSIZE::Bits32 as u8 && STREAM_PRIORITY == PL::High as u8,
    "the transfer width and priority encode the way PSIZE and PL do"
);

/// Stream of the first transfer controller draining the receiving sub-block.
///
/// Streams 0 and 1 carry the two output sub-blocks, so this is the first free
/// one, and its flags share `DMA_LISR` and `DMA_LIFCR` with theirs.
const INPUT_STREAM: usize = 2;

/// Stream of the first transfer controller feeding the master output
/// sub-block.
///
/// Its transfer counter is the read pointer a carry has to stay clear of.
const OUTPUT_STREAM: usize = 0;

/// Alternate function putting `SAI2_SD_A` on PD11.
///
/// The STM32H743VI datasheet, port D alternate function table: the PD11 row
/// carries `SAI2_SD_A` in the tenth column, whose header groups
/// `SAI2/4/TIM8/QUADSPI/SDMMC2/OTG1_HS/OTG2_FS/LCD`. The first audio interface
/// sits in the sixth column instead, which is where PE2 to PE6 are.
const DATA_PIN_FUNCTION: u8 = 10;

/// `MODER` value putting a pin on its alternate function. RM0433 section 11.4.1.
const PIN_ALTERNATE: u8 = 0b10;

/// `PUPDR` value leaving a pin with neither pull. RM0433 section 11.4.4.
const PIN_NO_PULL: u8 = 0b00;

// The pin is written and read back by nothing, here or in the plan, so a wrong
// value in it produces a verified path fed from no pad. These assertions are
// the only check that stands between the three encodings and the pin, and what
// settles them is the receiving counter refusing to move.
const _: () = assert!
(
    DATA_PIN_FUNCTION == ALTERNATE_FUNCTION::Af10 as u8
        && PIN_ALTERNATE == PIN_MODE::Alternate as u8
        && PIN_NO_PULL == PULL::Floating as u8,
    "the pin function, mode and pull encode the way the port does"
);

/// Buffer the receiving stream fills.
///
/// `.axisram` is a `NOLOAD` output section, so the startup sequence neither
/// copies nor zeroes what lands here and `fill_silence` is what puts zeros in
/// it before the stream runs. The name is unmangled so that one string
/// identifies the buffer in `llvm-nm` and in a debugger across rebuilds.
#[expect
(
    unsafe_code,
    reason = "the buffer is placed by section and named for a debugger"
)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".axisram.INPUT_BUFFER")]
static mut INPUT_BUFFER: MaybeUninit<[u32; BUFFER_WORDS]> = MaybeUninit::uninit();

/// Words a carry adds to a source index, as the bring-up measured them.
///
/// The distance between the two sides is fixed when the receiver starts, so
/// the shift is known to the bring-up and to nothing else. The handler rebuilds
/// the plan it serves on from constants at every entry and cannot be handed a
/// value, so the bring-up leaves this one here.
///
/// A word that never went through the bring-up is refused rather than carried:
/// `CarryShift::from_words` takes a whole number of frames inside the band a
/// start-up may leave a shift in and nothing else, and the value it opens on is
/// none of those. Storing it is ordered ahead of the write that unmasks the
/// line, and reading it is ordered behind the entry, so the handler cannot see
/// the value this opens on once the interrupt exists.
static CARRY_SHIFT: AtomicU32 = AtomicU32::new(u32::MAX);

/// The three cascades the carry runs, and the history behind each section.
///
/// This is the first state of this firmware that outlives one entry into a
/// handler. `pulsar_lib` owns the type and forbids `unsafe`, so a chain is
/// owned by whoever calls the carry and lent to it. A handler has no caller, so
/// this module is where the one the handler uses is parked.
///
/// # Where it sits, and what that costs
///
/// In the AXI SRAM, through the `.axisram` section, which the startup sequence
/// neither loads nor zeroes. The statics of the tightly coupled memory carry
/// the refusal word and the fault record, and that record is placed to sit
/// clear of the startup zero fill, so a static added beside them moves both.
///
/// It is 360 bytes, four history words then five coefficients for each section
/// of a way, and the memory it sits in is NOT free to the carry. The
/// disassembly is what says so: the loop reads and writes this static on every
/// frame, 30 coefficient reads and 40 writes of the four history words each
/// section carries, which is 70 of the 75 accesses a frame makes to this memory
/// and the buffers together, and the largest term of the 14.2 to 15.4
/// microseconds a frame the carry spends. The remaining 5 are the source word
/// and the four output words.
///
/// The 20 coefficients of the low way are the ones the compiler hoists into
/// registers for a whole block. The 40 history words are written back every
/// frame, each being live into the next sample of its own section, so no form of
/// the loop keeps them out of here.
///
/// It is paid rather than moved because the budget holds: the streams take
/// 22.68 microseconds a frame, so the carry still moves 1.5 to 1.6 times faster
/// than the thing it has to stay ahead of. Moving these statics to the tightly
/// coupled memory would move the fault record and the refusal word with them,
/// which is the machine form of the mute path, so it is a measurement of its
/// own rather than a step of this one.
///
/// # What keeps an unwritten one out of the carry
///
/// `start` writes a silent chain into it before it touches anything else, and
/// the built one over that, and both writes stand ahead of the call that
/// unmasks the line. So a handler can only be entered after the chain is
/// written, and a start-up that refuses leaves it silent AND enables no
/// interrupt.
#[expect
(
    unsafe_code,
    reason = "the chain is placed by section and named for a debugger"
)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".axisram.FILTER_CHAIN")]
static mut FILTER_CHAIN: MaybeUninit<FilterChain> = MaybeUninit::uninit();

/// The receiving sub-block, its transfer stream and the pin that feeds it.
pub(crate) struct Input<'a>
{
    sai: &'a SAI2,
    dma: &'a DMA1,
    mux: &'a DMAMUX1,
    port: &'a GPIOD,
}

impl Input<'_>
{
    /// Clears the five interrupt flags of the receiving stream, in one write.
    ///
    /// RM0433 section 15.5.3 makes every bit of `DMA_LIFCR` write one to clear
    /// and gives one bit to one flag of one stream, so this store covers stream
    /// 2 and reaches no bit belonging to the two output streams.
    fn wipe_stream_flags(&mut self)
    {
        self.dma.lifcr().write(|w| w
            .cfeif2().set_bit()
            .cdmeif2().set_bit()
            .cteif2().set_bit()
            .chtif2().set_bit()
            .ctcif2().set_bit());
    }

    /// Reads the receiving sub-block back.
    fn read_block(&self) -> InputBlockReadback
    {
        let block = self.sai.cha();
        let control = block.cr1().read();
        let fifo = block.cr2().read();
        let frame = block.frcr().read();
        let slots = block.slotr().read();
        let status = block.sr().read();
        let masks = block.im().read();

        InputBlockReadback
        {
            mode_bits: control.mode().bits(),
            protocol_bits: control.prtcfg().bits(),
            data_size_bits: control.ds().bits(),
            lsb_first: control.lsbfirst().bit_is_set(),
            changes_on_falling_edge: control.ckstr().bit_is_set(),
            sync_bits: control.syncen().bits(),
            sync_in_bits: self.sai.gcr().read().syncin().bits(),
            sync_out_bits: self.sai.gcr().read().syncout().bits(),
            mono: control.mono().bit_is_set(),
            tristate: fifo.tris().bit_is_set(),
            fifo_threshold_bits: fifo.fth().bits(),
            frame_length_field: frame.frl().bits(),
            frame_active_field: frame.fsall().bits(),
            frame_marks_channel: frame.fsdef().bit_is_set(),
            frame_active_high: frame.fspol().bit_is_set(),
            frame_leads_first_bit: frame.fsoff().bit_is_set(),
            first_bit_offset_field: slots.fboff().bits(),
            slot_size_bits: slots.slotsz().bits(),
            slot_count_field: slots.nbslot().bits(),
            slot_enable_bits: slots.sloten().bits(),
            transfer_enabled: control.dmaen().bit_is_set(),
            enabled: control.saien().bit_is_set(),
            any_interrupt_enabled: masks.ovrudrie().bit_is_set()
                || masks.mutedetie().bit_is_set()
                || masks.wckcfgie().bit_is_set()
                || masks.freqie().bit_is_set()
                || masks.cnrdyie().bit_is_set()
                || masks.afsdetie().bit_is_set()
                || masks.lfsdetie().bit_is_set(),
            fifo_level_bits: status.flvl().bits(),
            overrun: status.ovrudr().bit_is_set(),
            frame_mismatch: status.afsdet().bit_is_set() || status.lfsdet().bit_is_set(),
            codec_not_ready: status.cnrdy().bit_is_set(),
        }
    }

    /// Reads the receiving stream and the multiplexer channel serving it back.
    fn read_stream(&self) -> InputStreamReadback
    {
        let stream = self.dma.st(INPUT_STREAM);
        let control = stream.cr().read();
        let fifo = stream.fcr().read();
        let route = self.mux.ccr(INPUT_STREAM).read();

        // RM0433 section 15.5.1 packs the flags of the first four streams into
        // one register and puts `FEIF`, `DMEIF` and `TEIF` of stream 2 at bits
        // 16, 18 and 19. They are read through the field names of the
        // peripheral crate rather than through masks, and only the names
        // carrying this stream number are read here, so a flag of an output
        // stream cannot be reported on this one.
        let status = self.dma.lisr().read();

        InputStreamReadback
        {
            request_bits: route.dmareq_id().bits(),
            sync_enabled: route.se().bit_is_set(),
            direction_bits: control.dir().bits(),
            circular: control.circ().bit_is_set(),
            memory_increments: control.minc().bit_is_set(),
            peripheral_increments: control.pinc().bit_is_set(),
            memory_width_bits: control.msize().bits(),
            peripheral_width_bits: control.psize().bits(),
            double_buffered: control.dbm().bit_is_set(),
            priority_bits: control.pl().bits(),
            event_interrupt_enabled: control.htie().bit_is_set()
                || control.tcie().bit_is_set(),
            error_interrupt_enabled: control.teie().bit_is_set()
                || control.dmeie().bit_is_set()
                || fifo.feie().bit_is_set(),
            peripheral_address: stream.par().read().pa().bits(),
            memory_address: stream.m0ar().read().m0a().bits(),
            items: u32::from(stream.ndtr().read().ndt().bits()),
            enabled: control.en().bit_is_set(),
            transfer_error: status.teif2().bit_is_set(),
            direct_mode_error: status.dmeif2().bit_is_set(),
            fifo_error: status.feif2().bit_is_set(),
        }
    }

    /// Reads the transfer counter of the master output stream.
    fn output_items(&self) -> u32
    {
        u32::from(self.dma.st(OUTPUT_STREAM).ndtr().read().ndt().bits())
    }
}

impl InputInterface for Input<'_>
{
    fn stop(&mut self)
    {
        self.sai.cha().cr1().modify(|_, w| w.saien().clear_bit());
        self.dma.st(INPUT_STREAM).cr().modify(|_, w| w.en().clear_bit());
    }

    /// Writes `SYNCIN` into `SAI_GCR`, and `SYNCOUT` to the value that puts
    /// nothing out.
    ///
    /// This interface is followed by none, so its own `SYNCOUT` is written to
    /// zero rather than left where a warm restart found it. RM0433 section
    /// 51.6.1 requires both fields written while the two sub-blocks of this
    /// interface are disabled, which the step ahead of this one delivers.
    #[expect
    (
        unsafe_code,
        reason = "the two synchronisation fields take raw bits in the \
                  peripheral crate"
    )]
    fn declare_sync_input(&mut self, plan: &InputPlan)
    {
        // SAFETY: both values are two bit fields, and RM0433 section 51.4.4
        // table 420 defines 0b00 for the SYNCIN of a second interface following
        // the first, and section 51.6.1 defines 0b00 for a SYNCOUT that puts
        // nothing out.
        self.sai.gcr().write(|w| unsafe
        {
            w.syncin().bits(plan.sync_in_field()).syncout().bits(SYNC_OUT_NONE)
        });
    }

    /// Puts PD11 on the data function of the receiving sub-block.
    ///
    /// A receiver takes `SD` as an input, so the drive strength and the output
    /// type of the pad describe nothing and this leaves them where a reset put
    /// them. Nothing here writes `BSRR`, and the only field of any register
    /// named is the one of PD11, so the pad this board wires to a driven output
    /// never drives back.
    #[expect
    (
        unsafe_code,
        reason = "the pull field is the one of the three whose writer the \
                  peripheral crate leaves unsafe, its fourth value being \
                  reserved"
    )]
    fn open_pin(&mut self)
    {
        // SAFETY: RM0433 section 11.4.4 gives the field two bits and reserves
        // the value 3, which is why the peripheral crate leaves this writer
        // unsafe. Zero is the no-pull value and is not the reserved one.
        self.port.pupdr().modify(|_, w| unsafe
        {
            w.pupdr11().bits(PIN_NO_PULL)
        });

        self.port.afrh().modify(|_, w| w.afr11().set(DATA_PIN_FUNCTION));

        // The mode write comes last. Until it lands the pin stays analog, so
        // the settings above are in place before the pad carries anything.
        self.port.moder().modify(|_, w| w.moder11().set(PIN_ALTERNATE));
    }

    /// Writes every configuration register of the receiving sub-block, `SAIEN`
    /// apart.
    ///
    /// The frame comes off the plan the transmitting side runs on, so the
    /// receiver declares the frame the transmitter emits and a disagreement
    /// there is what `AFSDET` and `LFSDET` report rather than something this
    /// firmware could hold two opinions about.
    ///
    /// `MODE` lands in the same write as the rest and `DMAEN` follows it, which
    /// RM0433 section 51.6.2 requires: a sub-block resets to transmitter, so
    /// raising `DMAEN` first would raise a request in the wrong direction.
    ///
    /// `MCKDIV`, `NOMCK` and `OSR` go to the value a reset gives them. Section
    /// 51.4.8 turns the clock generator off on a sub-block taking its clocks
    /// from outside and reads none of the three, which is why no read-back
    /// compares them.
    #[expect
    (
        unsafe_code,
        reason = "the frame, slot and mode fields take raw bits in the \
                  peripheral crate"
    )]
    fn write_block(&mut self, plan: &InputPlan)
    {
        let frame = plan.frame();
        let block = self.sai.cha();

        // Every multi-bit field comes from the plan rather than from a variant
        // named in the peripheral crate, and every single bit is written as a
        // bit. The assertions at the head of this module are what tie the plan
        // values to the encodings.
        //
        // SAFETY: MODE, SYNCEN and PRTCFG carry two bits, DS three, and MCKDIV
        // six, which are the widths RM0433 section 51.6.2 gives them.
        block.cr1().write(|w| unsafe
        {
            w.mode().bits(plan.mode_field())
                .syncen().bits(plan.sync_field())
                .prtcfg().bits(frame.protocol_field())
                .ds().bits(frame.data_size_field())
                .mckdiv().bits(0)
        }
            .lsbfirst().clear_bit()
            .ckstr().bit(frame.clock_strobing_field())
            .mono().clear_bit()
            .outdriv().clear_bit()
            .nodiv().clear_bit()
            .osr().clear_bit());

        // RM0433 section 51.6.4 resets this register to zero, so MUTE and COMP
        // come up carrying what this frame wants and the write below drives
        // them nowhere else. FFLUSH is legal here because the stream that
        // serves this sub-block is stopped.
        //
        // SAFETY: the threshold is one of the five values RM0433 section 51.6.4
        // defines over three bits.
        block.cr2().write(|w| unsafe
        {
            w.fth().bits(frame.fifo_threshold_field())
        }
            .fflush().set_bit()
            .tris().clear_bit()
            .mute().clear_bit());

        // SAFETY: both lengths come off a transport plan validate accepted,
        // which fixes the frame at 64 bit clock periods, so they carry 63 over
        // eight bits and 31 over seven, the widths RM0433 section 51.6.6 gives
        // them.
        block.frcr().write(|w| unsafe
        {
            w.frl().bits(frame.frame_length_field())
                .fsall().bits(frame.frame_active_field())
                .fsdef().set_bit()
                .fspol().clear_bit()
                .fsoff().set_bit()
        });

        // SAFETY: the four values come off the same plan, which fixes two 32
        // bit slots, so in the order written they carry 0 over five bits, 0b10
        // over two, 1 over four and 0b11 over sixteen, the widths RM0433
        // section 51.6.8 gives them.
        block.slotr().write(|w| unsafe
        {
            w.fboff().bits(0)
                .slotsz().bits(frame.slot_size_field())
                .nbslot().bits(frame.slot_count_field())
                .sloten().bits(frame.slot_enable_field())
        });

        // A warm restart can find an interrupt unmasked, and no handler of this
        // firmware serves a sub-block interrupt, so the mask register goes back
        // to its reset value rather than being left where it was found.
        block.im().reset();

        block.clrfr().write(|w| w
            .covrudr().set_bit()
            .cmutedet().set_bit()
            .cwckcfg().set_bit()
            .ccnrdy().set_bit()
            .cafsdet().set_bit()
            .clfsdet().set_bit());

        block.cr1().modify(|_, w| w.dmaen().set_bit());
    }

    /// Routes and writes the receiving stream, but not `EN`.
    ///
    /// RM0433 section 15.5.5 makes every field below writable only while `EN`
    /// reads 0, which the step that stops the stream is what delivers.
    ///
    /// The peripheral address is taken from the peripheral crate rather than
    /// from the plan. The plan names it too, and the read-back compares the
    /// two, so a plan constant naming another address refuses the bring-up
    /// instead of pointing a transfer at a register nobody meant.
    #[expect
    (
        unsafe_code,
        reason = "the request identifier, the addresses and the transfer widths \
                  take raw bits in the peripheral crate"
    )]
    fn write_stream(&mut self, plan: &InputPlan)
    {
        let frame = plan.frame();
        let stream = self.dma.st(INPUT_STREAM);

        // SAFETY: the request identifier is 89, inside the seven bits RM0433
        // section 17.6.1 gives the field.
        self.mux.ccr(INPUT_STREAM).write(|w| unsafe
        {
            w.dmareq_id().bits(plan.request_field())
        }
            .se().clear_bit()
            .ege().clear_bit()
            .soie().clear_bit());

        self.wipe_stream_flags();

        // SAFETY: the address is the data register of the receiving sub-block,
        // taken from the peripheral crate, and the field carries a whole word.
        stream.par().write(|w| unsafe
        {
            w.pa().bits(self.sai.cha().dr().as_ptr() as u32)
        });

        // SAFETY: the address comes from a plan validate accepted, which holds
        // it inside the memory the transfer controllers reach and on a word
        // boundary.
        stream.m0ar().write(|w| unsafe { w.m0a().bits(plan.buffer_address()) });

        // A plan validate accepted holds the count inside the field, so the
        // fallback below is unreachable. A count that reached it would come
        // back short in the read-back rather than run.
        let items = u16::try_from(plan.transfer_items()).unwrap_or(u16::MAX);

        stream.ndtr().write(|w| w.ndt().set(items));

        // RM0433 section 15.5.5 resets this register to zero, so PFCTRL, the
        // two burst fields and the second buffer come up carrying what this
        // path wants, and the write below drives them nowhere else.
        //
        // All five interrupt enables go down here and with the register below.
        // The two transfer event ones come back up in `enable_events`, and the
        // three error ones never do, because no handler of this firmware serves
        // one.
        //
        // SAFETY: the direction, the two widths and the priority are values
        // RM0433 section 15.5.5 defines over two bits each.
        stream.cr().write(|w| unsafe
        {
            w.dir().bits(plan.direction_field())
                .msize().bits(frame.width_field())
                .psize().bits(frame.width_field())
                .pl().bits(frame.priority_field())
        }
            .circ().set_bit()
            .minc().set_bit()
            .pinc().clear_bit()
            .dbm().clear_bit()
            .pfctrl().clear_bit()
            .tcie().clear_bit()
            .htie().clear_bit()
            .teie().clear_bit()
            .dmeie().clear_bit());

        // Direct mode, which is what a peripheral holding one word a transfer
        // takes, and the value RM0433 section 15.5.10 resets DMDIS to. It
        // leaves the threshold beside it unused.
        stream.fcr().write(|w| w.dmdis().clear_bit().feie().clear_bit());
    }

    fn clear_stream_flags(&mut self)
    {
        self.wipe_stream_flags();
    }

    /// Sets `EN` on the stream, then `SAIEN` on the sub-block.
    ///
    /// The barrier comes first. The buffer was filled as ordinary memory and
    /// the enable is a device store, and ARMv7-M orders the two only across a
    /// `DSB`, so without it the first transfer can land in a buffer the core
    /// has not finished zeroing.
    fn start(&mut self)
    {
        asm::dsb();
        self.dma.st(INPUT_STREAM).cr().modify(|_, w| w.en().set_bit());
        self.sai.cha().cr1().modify(|_, w| w.saien().set_bit());
    }

    /// Reads the sub-block, the receiving counter and the transmitting counter,
    /// in that order.
    ///
    /// The order is a precondition of the shift. `CarryShift::measure` takes
    /// the distance between the two counters down to the frame below, and that
    /// corrects a pair read at two instants only while the error runs one way.
    /// The distance is how far the transmitting read pointer stands ahead of
    /// the receiving write pointer, and this reads the transmitting counter
    /// LAST, so that pointer has moved the further of the two and the distance
    /// reads long by the words the streams passed in between. Truncating down
    /// takes those back off. Read the other way round it would read short, the
    /// truncation would drop a whole frame instead, and every word would land a
    /// frame from where it belongs with no flag raised.
    fn read(&self) -> InputReadback
    {
        InputReadback
        {
            block: self.read_block(),
            stream: self.read_stream(),
            output_items: self.output_items(),
        }
    }

    /// Raises `HTIE` and `TCIE` and unmasks the line they drive.
    ///
    /// This is the one call in this firmware that puts a served interrupt on a
    /// vector, and the permit of the release gate is what opens it. Taking that
    /// permit by value here, at the trait, is what leaves no second door: a
    /// register block of another shape faces the same signature.
    ///
    /// The pending bit goes down between the two writes, and what it covers is
    /// a start that no reset preceded. PM0253 section 4.2.5 latches a pending
    /// interrupt whether or not the line is unmasked, and the NVIC keeps that
    /// bit across a re-run started from the debug probe, so a bit left by the
    /// previous run would otherwise enter the handler the instant the line is
    /// unmasked. `next_block` refuses such an entry and the machine goes
    /// silent, which is correct and is still not what a fresh start should do.
    ///
    /// It covers nothing on a cold start. `write_stream` is the only writer of
    /// `HTIE` and `TCIE` besides this call and it clears both, so the
    /// controller drives no line between the two, and the bit cannot be set by
    /// this run before this point. Nor can the call lose an event: the line is
    /// level, so a transfer flag still standing pends it again on the next
    /// cycle.
    #[expect
    (
        unsafe_code,
        reason = "unmasking a line is unsafe in the cortex-m crate, since a \
                  handler can then preempt any critical section built on \
                  masking"
    )]
    fn enable_events(&mut self, _permit: TonePermit)
    {
        self.dma.st(INPUT_STREAM).cr().modify(|_, w| w
            .htie().set_bit()
            .tcie().set_bit());

        NVIC::unpend(Interrupt::DMA_STR2);

        // SAFETY: this firmware builds no critical section on interrupt
        // masking. The one place that masks is the fault path, which masks and
        // never returns, and every other path is either ahead of this call or
        // parked in `wfi`.
        unsafe
        {
            NVIC::unmask(Interrupt::DMA_STR2);
        }
    }

    fn event(&self) -> Event
    {
        let status = self.dma.lisr().read();
        let stream = self.dma.st(INPUT_STREAM);
        let block = self.sai.cha();
        let sai_status = block.sr().read();

        Event
        {
            half_transfer: status.htif2().bit_is_set(),
            transfer_complete: status.tcif2().bit_is_set(),
            transfer_error: status.teif2().bit_is_set(),
            direct_mode_error: status.dmeif2().bit_is_set(),
            fifo_error: status.feif2().bit_is_set(),
            stream_enabled: stream.cr().read().en().bit_is_set(),
            items: u32::from(stream.ndtr().read().ndt().bits()),
            overrun: sai_status.ovrudr().bit_is_set(),
            frame_mismatch: sai_status.afsdet().bit_is_set()
                || sai_status.lfsdet().bit_is_set(),
            codec_not_ready: sai_status.cnrdy().bit_is_set(),
            block_enabled: block.cr1().read().saien().bit_is_set(),
            output_items: self.output_items(),
        }
    }

    /// Writes `CHTIF2` or `CTCIF2` into `DMA_LIFCR`, and no other bit.
    ///
    /// What holds the other flags up is the one way down they have. RM0433
    /// section 15.5.1 gives `HTIF`, `TCIF`, `TEIF`, `DMEIF` and `FEIF` the same
    /// sentence: each is set by hardware and cleared by software writing 1 to
    /// the corresponding bit of `DMA_LIFCR`, and the section describes no
    /// second route. A bit this write leaves at zero therefore takes nothing
    /// down, so the three error flags stand for the next entry to read.
    ///
    /// The other half of the pair is left standing too. An entry that finds
    /// both raised is one that missed a block, which is what
    /// `EventFault::BothHalvesPending` reports, and taking one down here would
    /// turn that reading into silence.
    fn clear_event(&mut self, half: Half)
    {
        match half
        {
            Half::First =>
            {
                self.dma.lifcr().write(|w| w.chtif2().set_bit());
            }
            Half::Second =>
            {
                self.dma.lifcr().write(|w| w.ctcif2().set_bit());
            }
        }
    }

    /// Returns the word the receiving buffer holds at one index.
    ///
    /// The read is volatile because the transfer controller is the other end of
    /// this buffer, and nothing in this binary writes it.
    ///
    /// # Why this is inlined
    ///
    /// The carry walks a whole block through this accessor and the one below,
    /// and the loop that walks it is generic, so the two fold into it and the
    /// whole of a frame is one straight line of 256 instructions with no call
    /// in it, counted on the linked image. The budget it is held against is the
    /// 5.000 milliseconds the transmitting streams take to read a block, since a
    /// carry slower than that is one they overtake, and the folded carry spends
    /// 3.14 to 3.39 milliseconds of it.
    #[expect
    (
        unsafe_code,
        reason = "the receiving buffer is reached by raw pointer, since a \
                  reference to a static a transfer controller also writes \
                  would claim exclusive access this firmware does not have"
    )]
    #[expect
    (
        clippy::inline_always,
        reason = "the carry reaches this once a frame, and the fold is what \
                  leaves the whole frame one straight line of 256 instructions \
                  with no call in it, counted on the linked image, which is \
                  3140 to 3390 microseconds a block on a block the streams \
                  read in 5000"
    )]
    #[inline(always)]
    fn read_word(&self, index: u32) -> u32
    {
        let at = index as usize;

        if at >= BUFFER_WORDS
        {
            return 0;
        }

        // SAFETY: the index stands below BUFFER_WORDS, which is the length of
        // the array, so the read is inside the object. `validate` refuses a
        // receiving buffer that overlaps either output buffer, so this reads an
        // object neither write below touches.
        unsafe
        {
            ptr::read_volatile((&raw const INPUT_BUFFER).cast::<u32>().add(at))
        }
    }

    /// Writes one word into the output buffer of one sub-block, at one index.
    ///
    /// The output buffers are reached through `transport::write_output_word`
    /// rather than through a pointer of their own, so no writable pointer into
    /// either of them exists outside the module that owns them. Which buffers a
    /// word reaches is `pulsar_lib::passthrough::carry_block`, which calls this
    /// once per role, so no step of the fan-out lives in this crate where no
    /// host test walks it.
    #[expect
    (
        clippy::inline_always,
        reason = "the carry reaches this four times a frame, and the fold is \
                  what leaves the whole frame one straight line of 256 \
                  instructions with no call in it, counted on the linked \
                  image, which is 3140 to 3390 microseconds a block on a block \
                  the streams read in 5000"
    )]
    #[inline(always)]
    fn write_word(&mut self, role: BlockRole, index: u32, word: u32)
    {
        transport::write_output_word(role, index, word);
    }
}

/// Raises the bus clocks the input path is written through, and points the
/// kernel clock of its interface at the configured PLL.
///
/// RM0433, clock enabling delays: an enable command takes up to two periods of
/// the enabled clock to reach the peripheral, and until it has, a read of one
/// of its registers returns invalid data and a write is dropped. The prescribed
/// sequence reads the enable register back, which is what the volatile reads
/// below are.
///
/// It runs at the head of the start-up rather than inside the step that writes
/// the input path, and the reason is that the steps move. Reordering two
/// sub-systems moves the side effects of their start-up functions with them,
/// and raising a bus clock is one, so this stands ahead of the first register
/// write of either path instead of travelling with the step that happens to
/// need it today. The transfer controller clock is raised here as well as by
/// the output transport, which costs one write and one read.
///
/// RM0433 section 21.6: "The peripheral clocks of the SAI2 are: the kernel
/// clock selected by `SAI23SEL` and provided to `sai_a_ker_ck` and
/// `sai_b_ker_ck` inputs, and the `rcc_pclk2` bus interface clock." That field
/// resets to the second output of the first PLL, which no part of this firmware
/// configures.
///
/// A sub-block taking its clocks from another interface has its own generator
/// turned off by section 51.4.8, so what the selection reaches drives nothing
/// on this path and the frame is decoded off the bus clock. It is written all
/// the same: a field pointing at a PLL nothing configures is wrong whether or
/// not anything reads it, and the cost of saying so is one write.
pub(crate) fn prepare(rcc: &RCC)
{
    rcc.ahb4enr().modify(|_, w| w.gpioden().set_bit());
    let _ = rcc.ahb4enr().read().gpioden().bit_is_set();

    rcc.ahb1enr().modify(|_, w| w.dma1en().set_bit());
    let _ = rcc.ahb1enr().read().dma1en().bit_is_set();

    rcc.apb2enr().modify(|_, w| w.sai2en().set_bit());
    let _ = rcc.apb2enr().read().sai2en().bit_is_set();

    rcc.d2ccip1r().modify(|_, w| w.sai23sel().pll3_p());
}

/// Fills the receiving buffer with silence.
///
/// It runs before the stream that writes the buffer is enabled, so nothing else
/// touches it here. What it covers is the `NOLOAD` section: a data line
/// carrying nothing would otherwise leave power-on noise in the buffer, and the
/// carry would put it at full scale into a pair of headphones.
#[expect
(
    unsafe_code,
    reason = "the buffer is reached by raw pointer, since a reference to a \
              static the transfer controller also writes would claim exclusive \
              access this firmware does not have"
)]
fn fill_silence()
{
    let base = (&raw mut INPUT_BUFFER).cast::<u32>();

    for index in 0..BUFFER_WORDS
    {
        // SAFETY: the array holds BUFFER_WORDS words and this loop turns
        // BUFFER_WORDS times, so the last index written is the last of it.
        unsafe
        {
            ptr::write_volatile(base.add(index), 0);
        }
    }
}

/// The two publications of the crossover chain, and the witness the second one
/// builds.
///
/// The write is private to this module and neither publication reaches it on
/// its own terms. `park_silent` runs first and hands back a handle onto that
/// memory, `ParkedChain::publish` spends the handle, builds the chain itself
/// and returns the witness of the write that publishes it. So a witness comes
/// out of the write of the built chain rather than standing beside it, and no
/// caller picks what that write carries.
///
/// Nothing here forbids a second parking after a publication. What such a run
/// leaves is a silent chain and a witness that attests the write it came out
/// of, which is why the witness is read as one write at one instant and not as
/// the state of that memory from then on.
mod chain
{
    use core::ptr;

    use super::
    {
        FILTER_CHAIN,
        FilterChain,
        InitialisedFilters,
        PassthroughFault,
        built_chain,
    };

    /// Witness that the built crossover chain stands in the memory the carry
    /// reads.
    ///
    /// `ParkedChain::publish` is the only thing that builds one, and it returns
    /// it from the write that publishes that chain. The field is private to
    /// this module, so no caller can put one beside a publication it did not
    /// come out of, and the publication that stops the ways builds none: a
    /// stopped way is not a filtered way.
    ///
    /// It attests that one write, at one instant, and says nothing about the
    /// memory afterwards. Where that write lands is not attested either. This
    /// crate is a `[[bin]]` built for the part, so no host test reaches the
    /// pointer the write goes through, and a write that lands elsewhere builds
    /// a witness just the same. A bench probe is what settles that.
    ///
    /// A stage that needs the ways filtered takes this by reference, which
    /// leaves the start order to the compiler rather than to a comment.
    pub(crate) struct PublishedChain(());

    /// The release gate takes one of these and reads nothing out of it. What it
    /// needs is that one exists, since `start` is the only thing that builds one
    /// and it builds one only once the chain is where the handler reads it.
    impl InitialisedFilters for PublishedChain
    {
    }

    /// The parked chain, and the handle a publication spends to write over it.
    ///
    /// `park_silent` is the only thing that builds one. A handle dropped rather
    /// than spent is a silent chain nothing replaced, so this is `must_use`.
    #[must_use]
    pub(super) struct ParkedChain(());

    impl ParkedChain
    {
        /// Builds the crossover chain, publishes it over the parked one and
        /// returns the witness of that write.
        ///
        /// Building the chain here rather than taking it is what puts one
        /// chain on the path that reaches a witness. A caller holds no way to
        /// hand this a chain that stops the ways, so a witness of a silent
        /// publication cannot be written. The handle goes by value, so one
        /// parking answers one publication.
        ///
        /// # Errors
        ///
        /// `FilterRefused`, whatever the build refused with. A refusal writes
        /// nothing and returns no witness, so the parked silent chain stands.
        #[expect
        (
            clippy::unused_self,
            reason = "the handle carries no data, so spending it is the whole \
                      of what this takes from it, and that is the order the \
                      witness stands on"
        )]
        pub(super) fn publish(self) -> Result<PublishedChain, PassthroughFault>
        {
            write_chain(&built_chain()?);

            Ok(PublishedChain(()))
        }
    }

    /// Parks cascades that stop the signal, and returns the handle onto them.
    ///
    /// It builds no witness. The memory the carry would read holds a way the
    /// signal does not cross from here on, which is not a way with a filter on
    /// it, so nothing a stage downstream takes comes out of this.
    pub(super) fn park_silent() -> ParkedChain
    {
        write_chain(&FilterChain::silent());

        ParkedChain(())
    }

    /// Writes `chain` into the parked one.
    ///
    /// `write_volatile` is what keeps the write, since nothing in this crate
    /// reads the memory back. It carries no store sequence of its own: a chain
    /// is 360 bytes, so the image reaches that memory through a call to the
    /// runtime copy routine, the same symbol an ordinary move of a chain goes
    /// through, and what holds the write in place is a call the compiler
    /// cannot see into. What reads the memory is one handler, and what orders
    /// the two is the unmask that stands after every call to this.
    #[expect
    (
        unsafe_code,
        reason = "the chain is reached by raw pointer, since the handler that \
                  reads it holds no reference this function could borrow from"
    )]
    fn write_chain(chain: &FilterChain)
    {
        // SAFETY: nothing else touches the chain while this runs. Every call to
        // this stands ahead of the unmask that puts the handler on the vector,
        // and the handler is the only other access.
        unsafe
        {
            ptr::write_volatile((&raw mut FILTER_CHAIN).cast::<FilterChain>(), *chain);
        }
    }
}

pub(crate) use chain::PublishedChain;

/// Returns the crossover the carry runs on every frame.
///
/// `pulsar_lib` builds the three ways and this crate calls it, so the
/// coefficients are derived once and nothing here re-derives a corner. The
/// corners and the sample rate are constants of the build, so nothing the part
/// does at run time reaches the refusal below.
///
/// # Errors
///
/// `FilterRefused`, whatever the build refused with. A refusal leaves the
/// parked chain silent rather than carrying a way with no filter on it.
fn built_chain() -> Result<FilterChain, PassthroughFault>
{
    let Ok(chain) = FilterChain::built()
    else
    {
        return Err(PassthroughFault::Sequence(InputSequenceFault::FilterRefused));
    };

    Ok(chain)
}

/// Returns the plan the handler serves its events on.
///
/// It is rebuilt rather than stored, through the one body `plan` also derives
/// through, so the geometry the carry walks and the geometry the bring-up
/// verified cannot be derived two different ways. What makes the two INPUTS the
/// same clock plan is that `clock::start` is the only thing that builds an
/// `AudioClock` and it builds one carrying `AUDIO_PLAN`, so a second builder
/// carrying anything else is what would part them.
///
/// The witness the bring-up takes is what orders it behind the clock, and the
/// handler is already behind everything: the interrupt reaching it is enabled
/// by one call, and that call needs the permit of the release gate.
fn served_plan() -> InputPlan
{
    plan_from(&AUDIO_PLAN)
}

/// Returns the plan one clock plan makes for the input path.
///
/// The one place the geometry of the input path is derived. Both callers reach
/// it, so a change to the derivation reaches the bring-up and the handler
/// together rather than one of them.
fn plan_from(clock: &ClockPlan) -> InputPlan
{
    InputPlan::for_transport(&transport::plan_of(clock), buffer_address())
}

/// Returns the address of the receiving buffer.
fn buffer_address() -> u32
{
    (&raw const INPUT_BUFFER) as u32
}

/// Returns a view of the receiving sub-block, its stream and its pin.
pub(crate) fn observe<'a>
(
    sai: &'a SAI2,
    dma: &'a DMA1,
    mux: &'a DMAMUX1,
    port: &'a GPIOD
) -> Input<'a>
{
    Input { sai, dma, mux, port }
}

/// Returns the plan the input path runs on.
pub(crate) fn plan(clock: &AudioClock) -> InputPlan
{
    plan_from(&clock.plan())
}

/// Brings the input path up and proves the part took the plan.
///
/// `clock` is the witness that the audio kernel clock came up and read back as
/// planned. The receiving sub-block runs on the clocks of the transmitting one
/// rather than on its own, so what the witness buys here is the order: it is
/// built by the bring-up the output transport also depends on, and taking it by
/// reference leaves that order to the compiler.
///
/// The buffer is filled with silence before the stream that writes it is
/// enabled, which is what keeps power-on noise out of the carry.
///
/// The filter chain is published twice. A silent one goes down first, so the
/// memory the carry would read holds cascades that stop the signal from the
/// first instruction of this sequence, and the built one replaces it once the
/// path is up. Both stand ahead of the call that unmasks the line, so no entry
/// can find a chain that was never written.
///
/// Once this returns, PD11 carries the data line of a receiver following the
/// frame the transmitter emits, one transfer stream fills the buffer, and the
/// shift the two counters leave stands in the band the plan aims at and has
/// been read back. Nothing is enabled that could write an output buffer, and
/// XSMT is untouched.
///
/// What comes back is the witness that the built chain is in that memory. The
/// release gate takes it and reads nothing out of it, so what it buys is the
/// order: a run that never published a chain hands the gate nothing.
///
/// # Errors
///
/// A `PassthroughFault` naming where the bring-up refused and what that place
/// refused with. A refusal leaves the input path in whatever state the writes
/// that did land left, which reaches no converter, and the caller answers by
/// staying silent.
///
/// # Why this is not inlined
///
/// The build gate reads the entry function to check that each arm that parks
/// stores its own cause into the refusal word, and it resolves the address of
/// that word out of the register the store goes through. Folded into the entry
/// function, the polling loops of this sequence take enough registers that the
/// one holding that address is recycled as a loop counter, and the gate then
/// cannot say which value is live where an arm stores and refuses rather than
/// guessing, which is the behaviour it documents. Keeping this out of line
/// keeps the arms readable. It costs one call on a path taken once.
#[inline(never)]
pub(crate) fn start
(
    clock: &AudioClock,
    input: &mut Input<'_>,
    core_clock_hz: u32
) -> Result<PublishedChain, PassthroughFault>
{
    fill_silence();

    let parked = chain::park_silent();

    let plan = plan(clock);
    let shift = bring_up(input, &plan, &InputWaits::for_plan(&plan, core_clock_hz))?;

    let published = parked.publish()?;
    CARRY_SHIFT.store(shift.words(), Ordering::Release);

    Ok(published)
}

/// Puts the block structure into service.
///
/// `permit` is taken BY VALUE. The release gate builds one only after it has
/// watched the transfers over a whole lap and held a buffer of zeros over the
/// converter unmute ramp, once the clock and the chain had reported, and
/// consuming it here is what stops the interrupt going up while anything else
/// could still write an output buffer.
///
/// # Errors
///
/// A `PassthroughFault`. A refusal enables no interrupt, so the block structure
/// never runs and the caller answers by staying silent.
///
/// # Why this is not inlined
///
/// The build gate reads the entry function to check that each arm that parks
/// stores its own cause into the refusal word, and it resolves the address of
/// that word out of the register the store goes through. Folded into the entry
/// function, the polling loops of this sequence take enough registers that the
/// one holding that address is recycled as a loop counter, and the gate then
/// cannot say which value is live where an arm stores and refuses rather than
/// guessing, which is the behaviour it documents. Keeping this out of line
/// keeps the arms readable. It costs one call on a path taken once.
#[inline(never)]
pub(crate) fn start_blocks
(
    clock: &AudioClock,
    input: &mut Input<'_>,
    permit: TonePermit,
    core_clock_hz: u32
) -> Result<Passthrough, PassthroughFault>
{
    let plan = plan(clock);

    arm(input, permit, &plan, &InputWaits::for_plan(&plan, core_clock_hz))
}

/// Serves one transfer event of the receiving stream.
///
/// The handler steals the register blocks. Nothing in thread mode touches any
/// of them once the block structure is armed, which is the last thing the entry
/// function does before it parks, so there is no access for this to race.
///
/// # Errors
///
/// A `PassthroughFault` when the entry is not one this path serves. Nothing has
/// been written when one comes back.
#[expect
(
    unsafe_code,
    reason = "the handler reaches the register blocks by stealing them and the \
              filter chain by raw pointer, since the singletons the entry \
              function took cannot be borrowed from a vector and the chain has \
              no owner in thread mode"
)]
pub(crate) fn serve_event() -> Result<(), PassthroughFault>
{
    // SAFETY: the entry function parks in `wfi` as soon as the block structure
    // is armed and touches none of these blocks again, and this handler is the
    // only one this firmware serves, so no other access can fall between the
    // reads and the writes below.
    let (sai, dma, mux, port) = unsafe
    {
        (SAI2::steal(), DMA1::steal(), DMAMUX1::steal(), GPIOD::steal())
    };

    // SAFETY: `start` writes the chain twice before it returns and `arm`
    // unmasks this line afterwards, so the memory holds a chain before this
    // handler can be entered at all. Thread mode touches it no more once the
    // block structure is armed, and this is the only handler this firmware
    // serves, so this reference is the only one that exists while it lives.
    let chain = unsafe
    {
        &mut *(&raw mut FILTER_CHAIN).cast::<FilterChain>()
    };

    let mut input = observe(&sai, &dma, &mux, &port);
    let plan = served_plan();

    let Some(shift) = CarryShift::from_words(plan, CARRY_SHIFT.load(Ordering::Acquire))
    else
    {
        return Err(PassthroughFault::Event(EventFault::CarryShiftUnpublished));
    };

    pulsar_lib::passthrough::serve(&mut input, &plan, shift, chain)
}
