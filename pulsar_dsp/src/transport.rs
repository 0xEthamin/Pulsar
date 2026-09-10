//! Output transport of the processing board, at the register.
//!
//! `pulsar_lib::transport` holds the frame, the buffer rules and the sequence.
//! This module is the register block that sequence runs on, and the buffers it
//! replays: it writes the port, the audio interface, the transfer multiplexer
//! and two transfer streams, reads them back, and hands the plan the fields it
//! read.
//!
//! It touches neither PE7 nor XSMT. The converters stay where they are,
//! whichever way a bring-up ends, so what runs here is a frame on five pins
//! and no change to the mute line. Two other places in this firmware drive
//! PE7: the release gate raises it, and the fault path of `main` drives it
//! low.
//!
//! # What the buffers carry
//!
//! Silence, until the release gate has run. `start` fills both with zeros
//! before either stream is enabled, which is the term of the gate that has the
//! converter walk its unmute ramp over silence. `write_tone` is what replaces
//! them with the tone, and it takes the permit the gate returns, so a caller
//! cannot decide for itself that the mute line went up.
//!
//! # Where the buffers live
//!
//! In the AXI SRAM, through the `.axisram` section. The two system transfer
//! controllers reach every internal memory except the two tightly coupled ones,
//! which only the master transfer controller reaches, so a buffer in the data
//! memory this firmware runs its stack and its statics from would be read as
//! zeros with no flag raised. The section is not loaded at startup, so this
//! module fills both buffers itself before either stream is enabled.
//!
//! The data cache is not enabled anywhere in this binary, so a buffer written
//! by the core is a buffer the transfer controller reads. Enabling it makes
//! these writes need maintenance before the transfer sees them.
//!
//! # What runs after the bring-up
//!
//! Nothing. Each buffer holds a whole number of tone periods and each stream is
//! circular, so the frame repeats with no interrupt, no refill and no work in
//! any handler. Not one transfer interrupt is enabled either, which is what the
//! read-back checks: this binary serves none, so one would reach the fault path
//! and silence the machine.
//!
//! `write_tone` therefore writes into buffers the transfer controllers are
//! already reading, and puts a step at whichever position they have reached.
//! That is one discontinuity of at most an eighth of full scale, once, on a
//! path that reaches an oscilloscope and a pair of headphones. Removing it
//! would take a refill served from a transfer interrupt, so it is measured
//! rather than removed.

use core::mem::MaybeUninit;
use core::ptr;
use cortex_m::asm;
use pulsar_lib::transport::
{
    AudioInterface,
    BlockReadback,
    BlockRole,
    CHANGE_ON_FALLING_EDGE,
    DATA_SIZE_32,
    FIFO_THRESHOLD_HALF,
    MASTER_REQUEST,
    MODE_MASTER_TRANSMITTER,
    MODE_SLAVE_TRANSMITTER,
    PROTOCOL_FREE,
    SLAVE_REQUEST,
    SLOT_SIZE_32,
    STREAM_PRIORITY,
    SYNC_ASYNCHRONOUS,
    SYNC_INTERNAL,
    StreamReadback,
    TONE_SAMPLES,
    TONE_TABLE,
    TRANSFER_MEMORY_TO_PERIPHERAL,
    TRANSFER_WORD,
    TransportFault,
    TransportPlan,
    TransportReadback,
    TransportWaits,
    bring_up,
};
use pulsar_lib::release::TonePermit;
use stm32h7::stm32h743v::dma1::st::cr::{DIR, PL, PSIZE};
use stm32h7::stm32h743v::dmamux1::ccr::DMAREQ_ID;
use stm32h7::stm32h743v::sai1::ch::cr1::{CKSTR, DS, MODE, PRTCFG, SYNCEN};
use stm32h7::stm32h743v::sai1::ch::cr2::FTH;
use stm32h7::stm32h743v::gpioc::afrl::ALTERNATE_FUNCTION;
use stm32h7::stm32h743v::gpioc::moder::MODE as PIN_MODE;
use stm32h7::stm32h743v::gpioc::ospeedr::OUTPUT_SPEED;
use stm32h7::stm32h743v::gpioc::otyper::OUTPUT_TYPE;
use stm32h7::stm32h743v::gpioc::pupdr::PULL;
use stm32h7::stm32h743v::sai1::ch::slotr::SLOTSZ;
use stm32h7::stm32h743v::{DMA1, DMAMUX1, GPIOE, RCC, SAI1};

use crate::clock::AudioClock;

// Every value the plan names for a field this module writes is pinned here
// against the encoding the peripheral crate carries for it. Those values are a
// seam no host test crosses: `pulsar_lib` is tested against its own plan, so a
// plan value that does not mean what the register means passes every test and
// reaches the pins.
//
// The other seams of that kind name flag fields of `DMA_LISR` and `DMA_LIFCR`
// rather than plan values, and nothing pins them. The peripheral crate leaves
// no way to: its field readers are not `const`, so no assertion can call one,
// and its register reader carries private bits with no public constructor, so
// no host test can build one either. What stands there is the register section
// quoted at each of them, and, for the write that wipes the two FIFO error
// flags, the read the release gate makes immediately after it.
//
// The strobing edge is why the seam is checked rather than trusted. RM0433
// section 51.6.2 names the edge the interface CHANGES its outputs on, and the
// peripheral crate names the edge a receiver STROBES the line on. The two are
// opposite, so the bit that puts the data change on the falling edge is the one
// `CKSTR::RisingEdge` carries.
const _: () = assert!
(
    CHANGE_ON_FALLING_EDGE == (CKSTR::RisingEdge as u8 != 0),
    "the strobing edge the plan names is the one CKSTR encodes"
);

const _: () = assert!
(
    MODE_MASTER_TRANSMITTER == MODE::MasterTx as u8
        && MODE_SLAVE_TRANSMITTER == MODE::SlaveTx as u8,
    "the two directions the plan names encode the way MODE does"
);

const _: () = assert!
(
    SYNC_ASYNCHRONOUS == SYNCEN::Asynchronous as u8
        && SYNC_INTERNAL == SYNCEN::Internal as u8,
    "the two synchronisations the plan names encode the way SYNCEN does"
);

const _: () = assert!
(
    PROTOCOL_FREE == PRTCFG::Free as u8
        && DATA_SIZE_32 == DS::Bit32 as u8
        && SLOT_SIZE_32 == SLOTSZ::Bit32 as u8
        && FIFO_THRESHOLD_HALF == FTH::Quarter2 as u8,
    "the frame the plan names encodes the way PRTCFG, DS, SLOTSZ and FTH do"
);

const _: () = assert!
(
    TRANSFER_MEMORY_TO_PERIPHERAL == DIR::MemoryToPeripheral as u8
        && TRANSFER_WORD == PSIZE::Bits32 as u8
        && STREAM_PRIORITY == PL::High as u8,
    "the transfer the plan names encodes the way DIR, PSIZE and PL do"
);

const _: () = assert!
(
    MASTER_REQUEST == DMAREQ_ID::Sai1aDma as u8
        && SLAVE_REQUEST == DMAREQ_ID::Sai1bDma as u8,
    "the two requests the plan names encode the way DMAREQ_ID does"
);

/// Words in one sub-block buffer.
///
/// One frame is two words, one slot per converter channel, and the buffer holds
/// one lap of the tone table. A circular stream over it therefore replays whole
/// periods for ever.
const BUFFER_WORDS: usize = TONE_SAMPLES * 2;

/// Stream of the first transfer controller feeding the master sub-block.
const MASTER_STREAM: usize = 0;

/// Stream of the first transfer controller feeding the slave sub-block.
const SLAVE_STREAM: usize = 1;

/// Alternate function putting the audio interface on PE2 to PE6.
///
/// The STM32H743VI datasheet, port E alternate function table: `SAI1_MCLK_A`,
/// `SAI1_SD_B`, `SAI1_FS_A`, `SAI1_SCK_A` and `SAI1_SD_A` all sit in the same
/// column, which is the sixth.
const AUDIO_ALTERNATE_FUNCTION: u8 = 6;

/// `OSPEEDR` value the interface pins are driven at.
///
/// The medium speed. At 3.3 V with 50 pF hanging on the pin the STM32H743VI
/// datasheet table 160 gives 60 MHz there and a 5.2 ns edge, against the
/// 11.2896 MHz of the fastest signal in the group. The lowest setting gives
/// 12 MHz and a 16.6 ns edge, which is a fifth of a master clock period. Its
/// note 1 makes every figure of that table guaranteed by design rather than
/// tested in production.
const AUDIO_PIN_SPEED: u8 = 0b01;

/// `MODER` value putting a pin on its alternate function. RM0433 section 11.4.1.
const PIN_ALTERNATE: u8 = 0b10;

/// `PUPDR` value leaving a pin with neither pull. RM0433 section 11.4.4.
const PIN_NO_PULL: u8 = 0b00;

// The port is written and never read back, here or in the plan, so a wrong
// value in it produces a verified transport that reaches no pad. These
// assertions are the only check that stands between the four encodings and the
// pins, and the oscilloscope is what settles them.
const _: () = assert!
(
    AUDIO_ALTERNATE_FUNCTION == ALTERNATE_FUNCTION::Af6 as u8
        && PIN_ALTERNATE == PIN_MODE::Alternate as u8
        && AUDIO_PIN_SPEED == OUTPUT_SPEED::MediumSpeed as u8
        && PIN_NO_PULL == PULL::Floating as u8,
    "the pin function, mode, speed and pull encode the way the port does"
);

const _: () = assert!
(
    OUTPUT_TYPE::PushPull as u8 == 0,
    "a cleared output type bit is the push-pull driver"
);

/// Buffer the stream of the master sub-block replays.
///
/// `.axisram` is a `NOLOAD` output section, so the startup sequence neither
/// copies nor zeroes what lands here and `fill_buffer` is what puts silence in
/// it before either stream runs. The name is unmangled so that one string
/// identifies the buffer in `llvm-nm` and in a debugger across rebuilds.
#[expect
(
    unsafe_code,
    reason = "the buffer is placed by section and named for a debugger"
)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".axisram.MASTER_BUFFER")]
static mut MASTER_BUFFER: MaybeUninit<[u32; BUFFER_WORDS]> = MaybeUninit::uninit();

/// Buffer the stream of the slave sub-block replays.
#[expect
(
    unsafe_code,
    reason = "the buffer is placed by section and named for a debugger"
)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".axisram.SLAVE_BUFFER")]
static mut SLAVE_BUFFER: MaybeUninit<[u32; BUFFER_WORDS]> = MaybeUninit::uninit();

/// The audio interface, its two transfer streams and the pins that carry them.
pub(crate) struct Interface<'a>
{
    sai: &'a SAI1,
    dma: &'a DMA1,
    mux: &'a DMAMUX1,
    port: &'a GPIOE,
}

impl Interface<'_>
{
    /// Writes every configuration register of one sub-block, `SAIEN` apart.
    ///
    /// `MODE` lands in the same write as `DMAEN`, which RM0433 section 51.6.2
    /// requires: the sub-block defaults to transmitter after reset, so setting
    /// `DMAEN` first on a receiver would raise a request in the wrong
    /// direction.
    #[expect
    (
        unsafe_code,
        reason = "the frame, slot and divider fields take raw bits in the \
                  peripheral crate"
    )]
    fn write_block(&mut self, plan: &TransportPlan, role: BlockRole)
    {
        let block = self.sai.ch(block_index(role));

        // Every multi-bit field comes from the plan rather than from a variant
        // named in the peripheral crate, and every single bit is written as a
        // bit. A name is the one thing in a register write that can mean the
        // opposite of what it says, and the assertions at the head of this
        // module are what tie the plan values to the encodings.
        //
        // SAFETY: the five values below come from a plan validate accepted.
        // MODE, PRTCFG and SYNCEN carry two bits, DS three, and the divider is
        // bounded at 63 over six, which are the widths RM0433 section 51.6.2
        // gives them.
        block.cr1().write(|w| unsafe
        {
            w.mode().bits(plan.mode_field(role))
                .syncen().bits(plan.sync_field(role))
                .prtcfg().bits(plan.protocol_field())
                .ds().bits(plan.data_size_field())
                .mckdiv().bits(plan.master_divider_field())
        }
            .lsbfirst().clear_bit()
            .ckstr().bit(plan.clock_strobing_field())
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
            w.fth().bits(plan.fifo_threshold_field())
        }
            .fflush().set_bit()
            .tris().clear_bit()
            .mute().clear_bit());

        // SAFETY: both lengths come from a plan validate accepted, which fixes
        // the frame at 64 bit clock periods, so they carry 63 over eight bits
        // and 31 over seven, the widths RM0433 section 51.6.6 gives them.
        block.frcr().write(|w| unsafe
        {
            w.frl().bits(plan.frame_length_field())
                .fsall().bits(plan.frame_active_field())
                .fsdef().set_bit()
                .fspol().clear_bit()
                .fsoff().set_bit()
        });

        // SAFETY: the four values come from a plan validate accepted, which
        // fixes two 32 bit slots, so in the order written they carry 0 over
        // five bits, 0b10 over two, 1 over four and 0b11 over sixteen, the
        // widths RM0433 section 51.6.8 gives them.
        block.slotr().write(|w| unsafe
        {
            w.fboff().bits(0)
                .slotsz().bits(plan.slot_size_field())
                .nbslot().bits(plan.slot_count_field())
                .sloten().bits(plan.slot_enable_field())
        });

        // A warm restart can find an interrupt unmasked, and no handler of this
        // firmware serves one, so the mask register goes back to its reset
        // value rather than being left where it was found.
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

    /// Routes and writes one transfer stream, but not `EN`.
    ///
    /// RM0433 section 15.5.5 makes every field below writable only while `EN`
    /// reads 0, which the step that stops the streams is what delivers.
    #[expect
    (
        unsafe_code,
        reason = "the request identifier and the transfer widths take raw bits \
                  in the peripheral crate"
    )]
    fn write_stream(&mut self, plan: &TransportPlan, role: BlockRole)
    {
        let index = stream_index(role);
        let stream = self.dma.st(index);

        // SAFETY: the request identifier is 87 or 88, inside the seven bits
        // RM0433 section 17.6.1 gives the field.
        self.mux.ccr(index).write(|w| unsafe
        {
            w.dmareq_id().bits(plan.request_field(role))
        }
            .se().clear_bit()
            .ege().clear_bit()
            .soie().clear_bit());

        clear_stream_flags(self.dma, role);

        // SAFETY: the address is the data register of the sub-block this stream
        // feeds, and the field carries a whole word.
        stream.par().write(|w| unsafe { w.pa().bits(plan.data_address(role)) });

        // SAFETY: the address comes from a plan validate accepted, which holds
        // it inside the memory the buffers belong in and on a word boundary.
        stream.m0ar().write(|w| unsafe { w.m0a().bits(plan.buffer_address(role)) });

        // A plan validate accepted holds the count inside the field, so the
        // fallback below is unreachable. A count that reached it would come
        // back short in the read-back rather than run.
        let items = u16::try_from(plan.transfer_items()).unwrap_or(u16::MAX);

        stream.ndtr().write(|w| w.ndt().set(items));

        // RM0433 section 15.5.5 resets this register to zero, so PFCTRL, the
        // two burst fields and the second buffer come up carrying what this
        // transport wants, and the write below drives them nowhere else.
        //
        // The four transfer interrupt enables go down here and the FIFO one
        // goes down with the register below. No handler of this firmware serves
        // one, so an enabled interrupt would reach the fault path and silence
        // the machine.
        //
        // SAFETY: the direction, the two widths and the priority are values
        // RM0433 section 15.5.5 defines over two bits each.
        stream.cr().write(|w| unsafe
        {
            w.dir().bits(plan.direction_field())
                .msize().bits(plan.width_field())
                .psize().bits(plan.width_field())
                .pl().bits(plan.priority_field())
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

    /// Reads one sub-block back.
    fn read_block(&self, role: BlockRole) -> BlockReadback
    {
        let block = self.sai.ch(block_index(role));
        let control = block.cr1().read();
        let fifo = block.cr2().read();
        let frame = block.frcr().read();
        let slots = block.slotr().read();
        let status = block.sr().read();
        let masks = block.im().read();

        BlockReadback
        {
            mode_bits: control.mode().bits(),
            protocol_bits: control.prtcfg().bits(),
            data_size_bits: control.ds().bits(),
            lsb_first: control.lsbfirst().bit_is_set(),
            changes_on_falling_edge: control.ckstr().bit_is_set(),
            sync_bits: control.syncen().bits(),
            mono: control.mono().bit_is_set(),
            driven_before_start: control.outdriv().bit_is_set(),
            tristate: fifo.tris().bit_is_set(),
            fifo_threshold_bits: fifo.fth().bits(),
            no_master_clock: control.nodiv().bit_is_set(),
            master_divider_field: control.mckdiv().bits(),
            oversampling: control.osr().bit_is_set(),
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
            fifo_level_bits: status.flvl().bits(),
            underrun: status.ovrudr().bit_is_set(),
            clock_configuration_rejected: status.wckcfg().bit_is_set(),
            any_interrupt_enabled: masks.ovrudrie().bit_is_set()
                || masks.mutedetie().bit_is_set()
                || masks.wckcfgie().bit_is_set()
                || masks.freqie().bit_is_set()
                || masks.cnrdyie().bit_is_set()
                || masks.afsdetie().bit_is_set()
                || masks.lfsdetie().bit_is_set(),
        }
    }

    /// Reads one transfer stream and the multiplexer channel serving it back.
    fn read_stream(&self, role: BlockRole) -> StreamReadback
    {
        let index = stream_index(role);
        let stream = self.dma.st(index);
        let control = stream.cr().read();
        let fifo = stream.fcr().read();
        let route = self.mux.ccr(index).read();

        // RM0433 section 15.5.1 packs the flags of the first four streams into
        // one register, `TEIF`, `DMEIF` and `FEIF` of stream 0 at bits 3, 2 and
        // 0 and of stream 1 at bits 9, 8 and 6. The two sets are read through
        // the field names of the peripheral crate rather than through masks,
        // and they are read per role, so the flags of one stream cannot be
        // reported on the other.
        let status = self.dma.lisr().read();

        let (transfer_error, direct_mode_error, fifo_error) = match role
        {
            BlockRole::Master =>
            (
                status.teif0().bit_is_set(),
                status.dmeif0().bit_is_set(),
                status.feif0().bit_is_set(),
            ),
            BlockRole::Slave =>
            (
                status.teif1().bit_is_set(),
                status.dmeif1().bit_is_set(),
                status.feif1().bit_is_set(),
            ),
        };

        StreamReadback
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
            any_interrupt_enabled: control.tcie().bit_is_set()
                || control.htie().bit_is_set()
                || control.teie().bit_is_set()
                || control.dmeie().bit_is_set()
                || fifo.feie().bit_is_set(),
            peripheral_address: stream.par().read().pa().bits(),
            memory_address: stream.m0ar().read().m0a().bits(),
            items: u32::from(stream.ndtr().read().ndt().bits()),
            enabled: control.en().bit_is_set(),
            transfer_error,
            direct_mode_error,
            fifo_error,
        }
    }
}

impl AudioInterface for Interface<'_>
{
    fn stop_streams(&mut self)
    {
        self.dma.st(MASTER_STREAM).cr().modify(|_, w| w.en().clear_bit());
        self.dma.st(SLAVE_STREAM).cr().modify(|_, w| w.en().clear_bit());
    }

    fn stop_blocks(&mut self)
    {
        self.sai.cha().cr1().modify(|_, w| w.saien().clear_bit());
        self.sai.chb().cr1().modify(|_, w| w.saien().clear_bit());
    }

    /// Puts PE2 to PE6 on the audio alternate function.
    ///
    /// Every value is a raw one rather than a variant named in the peripheral
    /// crate, and the assertions at the head of this module are what tie the
    /// four of them to the encodings the port carries. Nothing reads this back,
    /// so those assertions and the oscilloscope are the whole of what covers
    /// this step.
    ///
    /// PE7 is not named here. It carries the converter mute, and this module
    /// leaves it where it stands. The release gate is what puts it under the
    /// port and raises it, and the fault path of `main` is what drives it low.
    #[expect
    (
        unsafe_code,
        reason = "the pull field is the one of the four whose writer the \
                  peripheral crate leaves unsafe, its fourth value being \
                  reserved"
    )]
    fn open_pins(&mut self)
    {
        self.port.ospeedr().modify(|_, w| w
            .ospeedr2().set(AUDIO_PIN_SPEED)
            .ospeedr3().set(AUDIO_PIN_SPEED)
            .ospeedr4().set(AUDIO_PIN_SPEED)
            .ospeedr5().set(AUDIO_PIN_SPEED)
            .ospeedr6().set(AUDIO_PIN_SPEED));

        self.port.otyper().modify(|_, w| w
            .ot2().clear_bit()
            .ot3().clear_bit()
            .ot4().clear_bit()
            .ot5().clear_bit()
            .ot6().clear_bit());

        // SAFETY: RM0433 section 11.4.4 gives each of these fields two bits and
        // reserves the value 3, which is why the peripheral crate leaves this
        // writer unsafe. Zero is the no-pull value and is not the reserved one.
        self.port.pupdr().modify(|_, w| unsafe
        {
            w.pupdr2().bits(PIN_NO_PULL)
                .pupdr3().bits(PIN_NO_PULL)
                .pupdr4().bits(PIN_NO_PULL)
                .pupdr5().bits(PIN_NO_PULL)
                .pupdr6().bits(PIN_NO_PULL)
        });

        self.port.afrl().modify(|_, w| w
            .afr2().set(AUDIO_ALTERNATE_FUNCTION)
            .afr3().set(AUDIO_ALTERNATE_FUNCTION)
            .afr4().set(AUDIO_ALTERNATE_FUNCTION)
            .afr5().set(AUDIO_ALTERNATE_FUNCTION)
            .afr6().set(AUDIO_ALTERNATE_FUNCTION));

        // The mode write comes last. Until it lands the pins stay analog, so
        // the settings above are in place before anything drives a pad.
        self.port.moder().modify(|_, w| w
            .moder2().set(PIN_ALTERNATE)
            .moder3().set(PIN_ALTERNATE)
            .moder4().set(PIN_ALTERNATE)
            .moder5().set(PIN_ALTERNATE)
            .moder6().set(PIN_ALTERNATE));
    }

    fn write_master(&mut self, plan: &TransportPlan)
    {
        self.write_block(plan, BlockRole::Master);
    }

    fn write_slave(&mut self, plan: &TransportPlan)
    {
        self.write_block(plan, BlockRole::Slave);
    }

    fn write_master_stream(&mut self, plan: &TransportPlan)
    {
        self.write_stream(plan, BlockRole::Master);
    }

    fn write_slave_stream(&mut self, plan: &TransportPlan)
    {
        self.write_stream(plan, BlockRole::Slave);
    }

    /// Sets `EN` on both streams.
    ///
    /// The barrier comes first. The buffers were written as ordinary memory and
    /// the enable is a device store, and ARMv7-M orders the two only across a
    /// `DSB`, so without it the first transfer can read a buffer the core has
    /// not finished filling.
    fn start_streams(&mut self)
    {
        asm::dsb();
        self.dma.st(MASTER_STREAM).cr().modify(|_, w| w.en().set_bit());
        self.dma.st(SLAVE_STREAM).cr().modify(|_, w| w.en().set_bit());
    }

    fn start_slave(&mut self)
    {
        self.sai.chb().cr1().modify(|_, w| w.saien().set_bit());
    }

    fn start_master(&mut self)
    {
        self.sai.cha().cr1().modify(|_, w| w.saien().set_bit());
    }

    fn read(&self) -> TransportReadback
    {
        TransportReadback
        {
            master: self.read_block(BlockRole::Master),
            slave: self.read_block(BlockRole::Slave),
            master_stream: self.read_stream(BlockRole::Master),
            slave_stream: self.read_stream(BlockRole::Slave),
        }
    }

    /// Writes `CFEIF0` and `CFEIF1` into `DMA_LIFCR`, and no other bit.
    ///
    /// What holds the other flags up is not a property of the write but the one
    /// way down they have. RM0433 section 15.5.1 gives `HTIF`, `TCIF`, `TEIF`,
    /// `DMEIF` and `FEIF` the same sentence: each is set by hardware and
    /// "cleared by software writing 1 to the corresponding bit" of `DMA_LIFCR`,
    /// and the section describes no second route. A bit this write leaves at
    /// zero therefore takes nothing down. Section 15.5.3 makes that register
    /// write only, one flag per bit, and resets the word to zero.
    ///
    /// The `OVRUDR` of a sub-block is not reachable from here at all. It lives
    /// in `SAI_xSR` and section 51.4.14 clears it on `COVRUDR` of `SAI_xCLRFR`,
    /// which this module writes at bring-up and nowhere else.
    ///
    /// This is a seam nothing pins, like the read of the flags above, and it is
    /// the one seam of the two that closes itself. The read that opens the
    /// release window follows this write, so a bit that named the wrong stream
    /// or a write that did not land leaves both flags standing and the window
    /// refuses.
    fn clear_fifo_errors(&mut self)
    {
        self.dma.lifcr().write(|w| w.cfeif0().set_bit().cfeif1().set_bit());
    }
}

/// Returns the index of the sub-block a role names.
///
/// RM0433 section 51.6 puts sub-block A first, at offset `0x004`, and sub-block
/// B after it at `0x024`.
const fn block_index(role: BlockRole) -> usize
{
    match role
    {
        BlockRole::Master => 0,
        BlockRole::Slave => 1,
    }
}

/// Returns the index of the stream a role is fed by.
const fn stream_index(role: BlockRole) -> usize
{
    match role
    {
        BlockRole::Master => MASTER_STREAM,
        BlockRole::Slave => SLAVE_STREAM,
    }
}

/// Clears the five interrupt flags of one stream.
///
/// A flag left from an earlier run is read by a probe as a fault of this one.
/// It runs while the stream is stopped, so what it settles is `TEIF` and
/// `DMEIF`, which nothing raises again until the stream carries a transfer.
/// `FEIF` comes back up during the fill that follows, and the release gate is
/// what takes that one down as it opens its window.
fn clear_stream_flags(dma: &DMA1, role: BlockRole)
{
    match role
    {
        BlockRole::Master =>
        {
            dma.lifcr().write(|w| w
                .cfeif0().set_bit()
                .cdmeif0().set_bit()
                .cteif0().set_bit()
                .chtif0().set_bit()
                .ctcif0().set_bit());
        }
        BlockRole::Slave =>
        {
            dma.lifcr().write(|w| w
                .cfeif1().set_bit()
                .cdmeif1().set_bit()
                .cteif1().set_bit()
                .chtif1().set_bit()
                .ctcif1().set_bit());
        }
    }
}

/// Enables the port, the transfer controller and the audio interface clocks.
///
/// RM0433, clock enabling delays: an enable command takes up to two periods of
/// the enabled clock to reach the peripheral, and until it has, a read of one
/// of its registers returns invalid data and a write is dropped. The prescribed
/// sequence reads the enable register back, which is what the volatile reads
/// below are.
///
/// RCC carries no enable bit for the transfer request multiplexer. It is served
/// by the clock of the transfer controllers, so `DMA1EN` is what starts it.
fn enable_clocks(rcc: &RCC)
{
    rcc.ahb4enr().modify(|_, w| w.gpioeen().set_bit());
    let _ = rcc.ahb4enr().read().gpioeen().bit_is_set();

    rcc.ahb1enr().modify(|_, w| w.dma1en().set_bit());
    let _ = rcc.ahb1enr().read().dma1en().bit_is_set();

    rcc.apb2enr().modify(|_, w| w.sai1en().set_bit());
    let _ = rcc.apb2enr().read().sai1en().bit_is_set();
}

/// Writes one sample into every slot of `buffer`.
///
/// Both slots of a frame carry the same sample, so the four channels of the
/// frame send one waveform and any skew between the two data lines is the skew
/// between the two sub-blocks rather than a difference in what they carry.
///
/// `sample` is read from a function of the frame index, which is what lets one
/// walk write silence and another write the tone.
///
/// The stores are volatile because nothing in this binary reads the buffer
/// back. The transfer controller does, and no compiler sees that.
///
/// The parameter carries the length in its type, so the bound the writes stay
/// inside is the one the caller passed rather than one written down here.
#[expect
(
    unsafe_code,
    reason = "the buffer is reached by raw pointer, since a reference to a \
              static the transfer controller also reads would claim exclusive \
              access this firmware does not have"
)]
fn fill_buffer(buffer: *mut [u32; BUFFER_WORDS], sample: fn(usize) -> u32)
{
    let base = buffer.cast::<u32>();

    for index in 0..TONE_SAMPLES
    {
        let word = sample(index);

        // SAFETY: the array holds BUFFER_WORDS words, which is twice
        // TONE_SAMPLES, and this loop turns TONE_SAMPLES times writing the two
        // words at index * 2 and index * 2 + 1, so the last one written is the
        // last of the array. Nothing else in this firmware writes the buffer,
        // and the transfer controller only reads it.
        unsafe
        {
            let frame = base.add(index.saturating_mul(2));
            ptr::write_volatile(frame, word);
            ptr::write_volatile(frame.add(1), word);
        }
    }
}

/// Returns silence, whatever the frame index.
const fn silence(_index: usize) -> u32
{
    0
}

/// Returns the tone table entry for one frame of the lap.
///
/// A frame index past the table describes no entry and gives silence, which
/// the caller cannot reach: it walks `TONE_SAMPLES` frames and the table holds
/// `TONE_SAMPLES` entries.
fn tone(index: usize) -> u32
{
    match TONE_TABLE.get(index)
    {
        Some(sample) => sample.cast_unsigned(),
        None => 0,
    }
}

/// Writes the tone into both buffers, over the silence they were filled with.
///
/// `_permit` is what the release gate returns once the clocks were verified,
/// the transfers were watched over a whole lap of the buffer, and the mute line
/// had been high for the whole converter unmute ramp with zeros going out.
/// Taking it is what stops a caller deciding for itself that the gate ran.
///
/// The streams are already replaying these buffers, so the write puts a step
/// at whatever position the transfer controllers have reached. That step is
/// one eighth of full scale at worst, once, into an oscilloscope and a pair of
/// headphones. Removing it would take a refill from a transfer interrupt,
/// which this firmware does not serve.
pub(crate) fn write_tone(_permit: &TonePermit)
{
    fill_buffer((&raw mut MASTER_BUFFER).cast::<[u32; BUFFER_WORDS]>(), tone);
    fill_buffer((&raw mut SLAVE_BUFFER).cast::<[u32; BUFFER_WORDS]>(), tone);
}

/// Brings the output transport up and proves the part took the plan.
///
/// `clock` is the witness that the audio kernel clock came up and read back as
/// planned. RM0433 section 51.6.2 requires the clock present on the interface
/// before `SAIEN` is set, and taking the witness by reference is what leaves
/// that order to the compiler rather than to a comment. Its plan is where the
/// frame length and the master clock divider come from, so the interface writes
/// the chain the clock module validated.
///
/// `core_clock_hz` sizes the four waits, and naming a clock above the one the
/// core runs at only lengthens them.
///
/// Once this returns the five pins carry a master clock, a bit clock, a frame
/// clock and two data lines, and they keep carrying them with no further work
/// from the core. XSMT is untouched here, and both buffers hold zeros, so what
/// goes out is a frame of silence and the release gate is what decides whether
/// the mute line ever rises over it.
///
/// # Errors
///
/// A `TransportFault` naming where the bring-up refused, the sequence, the
/// plan, one of the two sub-blocks or one of the two streams, and what that
/// place refused with. A refusal reached after the interface started leaves the
/// clocks running, which is what the converter mute sequence needs, and the
/// caller answers by staying silent.
pub(crate) fn start
(
    clock: &AudioClock,
    rcc: &RCC,
    sai: &SAI1,
    dma: &DMA1,
    mux: &DMAMUX1,
    port: &GPIOE,
    core_clock_hz: u32
) -> Result<(), TransportFault>
{
    enable_clocks(rcc);

    fill_buffer((&raw mut MASTER_BUFFER).cast::<[u32; BUFFER_WORDS]>(), silence);
    fill_buffer((&raw mut SLAVE_BUFFER).cast::<[u32; BUFFER_WORDS]>(), silence);

    let mut interface = observe(sai, dma, mux, port);

    bring_up(&mut interface, &plan(clock), &TransportWaits::for_core_clock(core_clock_hz))
}

/// Returns the plan the interface and the release gate are both built on.
///
/// A pointer on this part is one word wide, which is what the transfer
/// register holds, so the two addresses reach the plan unchanged. The plan is
/// what refuses one outside the memory the buffers belong in.
pub(crate) fn plan(clock: &AudioClock) -> TransportPlan
{
    TransportPlan::for_clock
    (
        &clock.plan(),
        (&raw const MASTER_BUFFER) as u32,
        (&raw const SLAVE_BUFFER) as u32,
        buffer_words()
    )
}

/// Returns a view of the audio interface, its two streams and the pins.
///
/// The release gate holds one by exclusive reference and makes one write
/// through it, the wipe of the two FIFO error flags that opens its window.
/// Everything else it does through this is a read.
pub(crate) fn observe<'a>
(
    sai: &'a SAI1,
    dma: &'a DMA1,
    mux: &'a DMAMUX1,
    port: &'a GPIOE
) -> Interface<'a>
{
    Interface { sai, dma, mux, port }
}

/// Returns the length of one buffer, as the plan counts it.
///
/// The plan bounds it against the width of the transfer counter, so a length
/// past that width is refused rather than narrowed. The fallback below is what
/// hands it a length it refuses.
fn buffer_words() -> u32
{
    u32::try_from(BUFFER_WORDS).unwrap_or(u32::MAX)
}
