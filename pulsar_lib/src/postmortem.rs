//! Fault record the processing board leaves at a fixed address.
//!
//! The processing firmware writes one `FaultRecord` from its fault path, then
//! parks the core in a Sleep it keeps clocked so a debug port still reaches it.
//! Nothing on the board reads the record back. The reader is a person with a
//! probe on SWD, so the layout below is the interface, and the code here is the
//! reference that produces it.
//!
//! # Reading a record
//!
//! Take 36 bytes from the `FAULT_RECORD` symbol of the processing image, which
//! `llvm-nm` locates. Every word is little endian.
//!
//! ```text
//! +0x00  magic      0x534C_5550. The bytes read PULS. Anything else is not a
//!                   record.
//! +0x04  exception  ICSR.VECTACTIVE. 0 thread mode, 3 HardFault,
//!                   4 MemManage, 5 BusFault, 6 UsageFault, 16 + n for IRQ n.
//! +0x08  CFSR       every bit the register defines, low half first:
//!                   0 IACCVIOL, 1 DACCVIOL, 3 MUNSTKERR, 4 MSTKERR,
//!                   5 MLSPERR, 7 MMARVALID, 8 IBUSERR, 9 PRECISERR,
//!                   10 IMPRECISERR, 11 UNSTKERR, 12 STKERR, 13 LSPERR,
//!                   15 BFARVALID, 31 to 16 UsageFault.
//! +0x0C  HFSR       1 VECTTBL, 30 FORCED, 31 DEBUGEVT.
//! +0x10  MMFAR      an address only while CFSR bit 7 is set.
//! +0x14  BFAR       an address only while CFSR bit 15 is set.
//! +0x18  ABFSR      0 ITCM, 1 DTCM, 2 AHBP, 3 AXIM, 4 EPPB, and 9 to 8 the
//!                   AXI response type, which reads only while bit 3 is set.
//! +0x1C  refusal    the code a stage of the start-up refused with. Bits 31
//!                   to 16 are the domain, which says WHICH stage, and bits 15
//!                   to 0 the code that domain defines. Every domain is
//!                   numbered from 1, so the word reads 0 if and only if no
//!                   stage refused and the path was entered from somewhere
//!                   else.
//! +0x20  checksum   FNV-1a over the 28 bytes from +0x04 to +0x1F.
//! ```
//!
//! Bit positions come from PM0253 tables 64, 65, 67 and 109. `HFSR` bit 30 says
//! a configurable fault escalated, which is when `CFSR` rather than `HFSR`
//! names what happened.
//!
//! # Reading the refusal word
//!
//! The domains run in the order the start-up does. 1 is the guards the
//! processing firmware runs before it starts anything, 2 the audio clock, 3 the
//! output transport, 4 the gate that raises the converter mute line. The field
//! is wider than the domains named here, and a word carrying one they do not
//! name comes back as `Refusal::Unnamed` rather than being read as one of them.
//!
//! A clock, a transport and a release code are a place byte over a cause byte.
//! The place says where the stage refused, the cause what that place refused
//! with, and the enums that carry them are `pulsar_lib::clock::ClockFault`,
//! `pulsar_lib::transport::TransportFault` and
//! `pulsar_lib::release::ReleaseFault`. The guards refuse in one place, so a
//! domain 1 code is a cause byte alone, and the table shows the place byte it
//! always reads as.
//!
//! ```text
//! domain 1  place 0                    cause StartupFault
//! domain 2  place 0 the part           cause clock::TreeFault
//!           place 1 the plan           cause clock::ClockPlanError
//! domain 3  place 0 the sequence       cause transport::SequenceFault
//!           place 1 the plan           cause transport::TransportPlanError
//!           place 2, 3 the sub-blocks  cause transport::BlockFault
//!           place 4, 5 their streams   cause transport::StreamFault
//! domain 4  place 0 the gate sequence  cause release::SequenceRefusal
//!           place 1, 2 the sub-blocks  cause release::BlockAlarm
//!           place 3, 4 their streams   cause release::StreamAlarm
//!           place 0x11 to 0x14         the same four sites, the same causes,
//!                                      seen with the mute line already high
//! ```
//!
//! The place byte of domain 4 splits once more, into a phase over a site. A
//! high nibble of 0 says the mute line was still low when the gate refused, so
//! nothing was ever audible, and a high nibble of 1 says the line had been
//! raised and the gate drove it back down.
//!
//! The domains number their causes apart from one another, so one code reads
//! several ways and the domain is what settles it. Code `0x0001` is a clock
//! whose PLL never stopped, a transport whose stream never stopped, a release
//! whose transfer counters never lapped, and a start-up that found the core
//! handle already taken.
//!
//! The `CFSR` and `ABFSR` bits are sticky. PM0253 clears them on a write or a
//! reset and this firmware writes neither, so a record shows what has faulted
//! since reset and not only what faulted last.
//!
//! # Telling a record from stale memory
//!
//! Memory no fault has written holds whatever the previous run left there, so a
//! reader has to tell the two apart. Two guards cover the image between them
//! and neither covers what the other does: word 0 is a fixed magic, and word 8
//! is a checksum over the seven words between them. Both must hold.
//!
//! Nothing clears the record and nothing dates it, so a record that holds may
//! predate the reset being examined.
//!
//! # What this module is
//!
//! `FaultRecord::new` is the encoder the processing firmware calls, and
//! `startup_refusal`, `clock_refusal`, `transport_refusal` and
//! `release_refusal` are what tag a code with the domain it belongs to before
//! it gets there. The decoding half, `from_words` and the accessors, is the
//! reference that pins the encoding down and the tests exercise. No shipped
//! code decodes a record.
//!
//! No register is read here. The processing firmware reads them and fills
//! `FaultRegisters`.

use crate::clock::{CLOCK_CODE_CEILING, ClockFault};
use crate::release::{RELEASE_CODE_CEILING, ReleaseFault};
use crate::transport::{TRANSPORT_CODE_CEILING, TransportFault};

/// Words one record occupies. A reader takes this many from the record address.
pub const WORD_COUNT: usize = 9;

/// First word of a record.
///
/// It guards word 0, which the checksum does not reach. Its bytes are `PULS`,
/// so the record announces itself in the ASCII column of a byte dump, which is
/// the view a reader scanning memory for it has.
const MAGIC: u32 = 0x534C_5550;

/// FNV-1a 32 bit offset basis.
const FNV_OFFSET_BASIS: u32 = 0x811C_9DC5;

/// FNV-1a 32 bit prime.
const FNV_PRIME: u32 = 0x0100_0193;

/// `CFSR` bit that makes `MMFAR` an address. PM0253 table 64, `MMARVALID`.
const MMARVALID: u32 = 1 << 7;

/// `CFSR` bit that makes `BFAR` an address. PM0253 table 65, `BFARVALID`.
const BFARVALID: u32 = 1 << 15;

/// `CFSR` bits reporting a failed exception entry push.
///
/// PM0253 table 64 puts `MSTKERR` at bit 4 of the `MemManage` half, and table
/// 65 puts `STKERR` at bit 4 of the `BusFault` half, which the register carries
/// at bits 15 to 8.
const STACKING_ERRORS: u32 = (1 << 4) | (1 << 12);

/// `HFSR` bit set when a configurable fault escalated. PM0253 table 67,
/// `FORCED`.
const FORCED: u32 = 1 << 30;

/// Bit the domain field of a refusal word starts at.
const DOMAIN_SHIFT: u32 = 16;

/// Bits of the refusal word carrying the code the domain defines.
const CODE_MASK: u32 = (1 << DOMAIN_SHIFT) - 1;

/// Declares the stages a refusal word can name, and builds both readings of the
/// domain field from that one declaration.
///
/// Writing a domain and reading one back are the two halves of the encoding, and
/// a hand written pair of them can fall apart one half at a time. Each stage is
/// named here once, with the widest code its own encoding can carry, and the
/// expansion is what produces the enum, the reader and the table the sweep runs
/// over. A stage added here is added to all three at once, a stage declared
/// without its ceiling does not expand, and a stage cannot be added anywhere
/// else, since the enum has no other declaration.
macro_rules! refusal_domains
{
    (
        $(
            $(#[$note:meta])*
            $name:ident = $field:literal, codes to $ceiling:expr
        ),+
        $(,)?
    ) =>
    {
        /// Stage of the start-up a refusal word belongs to.
        ///
        /// The word carries one field naming the domain and one carrying the
        /// code that domain defines, so two stages that number a fault alike
        /// stay apart. The discriminant is the domain field. Two domains that
        /// carried one number would not compile, and none of them is zero,
        /// which is what makes a zero word mean no refusal whatever the domains
        /// go on to number.
        ///
        /// The field is wider than the stages below, which leaves the format
        /// room for one this firmware does not yet refuse.
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[repr(u16)]
        pub enum RefusalDomain
        {
            $(
                $(#[$note])*
                $name = $field,
            )+
        }

        impl RefusalDomain
        {
            /// Every domain, each with the widest code its stage can carry.
            #[cfg(test)]
            const ALL: &'static [(Self, u32)] = &[$((Self::$name, $ceiling)),+];

            /// Returns the domain `word` names, or `None` for one not declared.
            const fn of(word: u32) -> Option<Self>
            {
                let field = word & !CODE_MASK;

                $(
                    if field == Self::$name.field()
                    {
                        return Some(Self::$name);
                    }
                )+

                None
            }
        }
    };
}

refusal_domains!
{
    /// The guards the processing firmware runs before it starts anything,
    /// whose codes are `StartupFault`.
    Startup = 0x0001, codes to STARTUP_CODE_CEILING,
    /// The audio clock, whose codes `pulsar_lib::clock::ClockFault::code`
    /// numbers.
    Clock = 0x0002, codes to CLOCK_CODE_CEILING,
    /// The output transport, whose codes
    /// `pulsar_lib::transport::TransportFault::code` numbers.
    Transport = 0x0003, codes to TRANSPORT_CODE_CEILING,
    /// The gate that raises the converter mute line, whose codes
    /// `pulsar_lib::release::ReleaseFault::code` numbers. Its place byte
    /// carries a phase as well as a site, so a word of this domain says
    /// whether the line had gone high when the gate refused.
    ReleaseGate = 0x0004, codes to RELEASE_CODE_CEILING,
}

impl RefusalDomain
{
    /// Returns the domain field of a refusal word, in place.
    pub(crate) const fn field(self) -> u32
    {
        (self as u16 as u32) << DOMAIN_SHIFT
    }
}

/// Reason the processing firmware refused to start before any bring-up ran.
///
/// The discriminant of a variant is the cause byte a fault record carries for
/// it, which is the number a person with a probe looks up. Two variants that
/// carried one number would not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StartupFault
{
    /// The core peripheral handle came back already taken, so the fault
    /// handling of the core cannot be armed.
    CoreHandleTaken = 0x01,
    /// The device peripheral handle came back already taken, so no bring-up
    /// can reach a register block.
    DeviceHandleTaken = 0x02,
    /// The `BusFault` handler did not read back enabled after it was armed.
    ///
    /// PM0253 section 2.5.2 exempts the stack push that enters an enabled
    /// `BusFault` handler from escalation to `HardFault`, so that handler runs
    /// on the corrupt stack that faulted it. Disabled, the same section
    /// escalates the fault to `HardFault` instead, where section 2.5.5 turns a
    /// second fault into lockup. The vector is reached either way, and what the
    /// arming buys is the escalation left over it. This one is a read-back of
    /// the part and not a guard on a handle, so it can fail where the two above
    /// cannot.
    BusFaultNotArmed = 0x03,
}

/// Widest word the start-up encoding can carry.
///
/// The guards refuse in one place, so this code is a cause byte alone and its
/// type bounds it, where the clock and the transport carry a place byte over
/// theirs. No variant added to `StartupFault` can pass this, and a place byte
/// appearing over it would not fit under it. The bound is not a value the
/// encoding reaches.
const STARTUP_CODE_CEILING: u32 = u8::MAX as u32;

/// What the refusal word of a record says.
///
/// Both halves are the width the word gives them, so a value that fits neither
/// is not a `Refusal` to begin with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal
{
    /// A stage this module names refused with `code`.
    Named
    {
        /// Which stage refused.
        domain: RefusalDomain,
        /// The code that stage defines.
        code: u16,
    },
    /// A stage this module does not name refused with `code`. Both halves of
    /// the word come back as read, so a record sealed by a firmware that names
    /// more stages than this build still reads here.
    Unnamed
    {
        /// The domain field, as read.
        domain: u16,
        /// The code that domain defines, as read.
        code: u16,
    },
}

// The four below tag a code with its domain, and each is one `or` of two
// values. They are inlined so that the instructions building a refusal word
// stand in the arm that stores it: a disassembly of that arm then carries the
// domain immediate, and which stage a parked board refused at reads off the arm
// rather than off a call it makes.

/// Returns the refusal word a refused start-up guard leaves.
///
/// The guards run in one place, so the place byte of the code is zero and the
/// cause byte carries the whole of it.
#[must_use]
#[inline]
pub const fn startup_refusal(fault: StartupFault) -> u32
{
    RefusalDomain::Startup.field() | fault as u32
}

/// Returns the refusal word a refused audio clock bring-up leaves.
#[must_use]
#[inline]
pub const fn clock_refusal(fault: ClockFault) -> u32
{
    RefusalDomain::Clock.field() | fault.code()
}

/// Returns the refusal word a refused output transport bring-up leaves.
#[must_use]
#[inline]
pub const fn transport_refusal(fault: TransportFault) -> u32
{
    RefusalDomain::Transport.field() | fault.code()
}

/// Returns the refusal word a refused converter mute release leaves.
#[must_use]
#[inline]
pub const fn release_refusal(fault: ReleaseFault) -> u32
{
    RefusalDomain::ReleaseGate.field() | fault.code()
}

const _: () = assert!
(
    STARTUP_CODE_CEILING <= CODE_MASK
        && CLOCK_CODE_CEILING <= CODE_MASK
        && TRANSPORT_CODE_CEILING <= CODE_MASK
        && RELEASE_CODE_CEILING <= CODE_MASK,
    "no encoding reaches the domain field, so tagging a code leaves it bit for \
     bit and the field says which stage refused alone"
);

const _: () = assert!
(
    size_of::<FaultRecord>() == WORD_COUNT * size_of::<u32>(),
    "the record is the word image a debugger reads, with no padding in it"
);

const _: () = assert!
(
    align_of::<FaultRecord>() == align_of::<u32>(),
    "the record starts on a word boundary and holds nothing wider"
);

/// Fault status a Cortex-M7 core exposes once it has taken a fault.
///
/// Every field is the raw register value. The decoding lives on `FaultRecord`,
/// so a reader never applies a valid bit by hand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultRegisters
{
    /// `ICSR.VECTACTIVE`, the exception the core is serving. Zero means
    /// thread mode, so a record carrying zero was sealed by code that no
    /// exception had reached.
    pub exception: u32,
    /// `CFSR`, the `MemManage`, `BusFault` and `UsageFault` status registers.
    pub cfsr: u32,
    /// `HFSR`, which says whether a configurable fault escalated.
    pub hfsr: u32,
    /// `MMFAR`, an address only while `CFSR` sets `MMARVALID`.
    pub mmfar: u32,
    /// `BFAR`, an address only while `CFSR` sets `BFARVALID`.
    pub bfar: u32,
    /// `ABFSR`, the interface an asynchronous bus fault came from. PM0253
    /// section 4.9.5 keeps bits 4 to 0 valid until something writes the
    /// register, and no Pulsar firmware writes it, so they stand for a fault
    /// this record did not necessarily capture. Bits 9 to 8 carry the AXI
    /// response type and read only alongside bit 3. Nothing else names an
    /// interface, since such a fault writes no address to `BFAR`.
    pub abfsr: u32,
}

/// One captured register set and the start-up refusal that led to it, sealed so
/// a reader can tell the record from stale memory.
///
/// A record that holds says a fault path ran and sealed these words. It does
/// not say a hardware fault occurred: a path entered from ordinary code seals
/// whatever the status registers happen to hold, and an all zero register set
/// seals into as valid a record as any other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use]
#[repr(C)]
pub struct FaultRecord
{
    magic: u32,
    exception: u32,
    cfsr: u32,
    hfsr: u32,
    mmfar: u32,
    bfar: u32,
    abfsr: u32,
    refusal: u32,
    checksum: u32,
}

impl FaultRecord
{
    /// Seals `registers` and `refusal` into a record.
    ///
    /// `refusal` is what one of the three tagging functions of this module
    /// returns, or zero on a path no stage refused. Every domain is numbered
    /// from 1, so those two cases stay apart.
    pub fn new(registers: &FaultRegisters, refusal: u32) -> Self
    {
        let mut record = Self
        {
            magic: MAGIC,
            exception: registers.exception,
            cfsr: registers.cfsr,
            hfsr: registers.hfsr,
            mmfar: registers.mmfar,
            bfar: registers.bfar,
            abfsr: registers.abfsr,
            refusal,
            checksum: 0,
        };

        record.checksum = record.seal();
        record
    }

    /// Reads a record back out of the words a debugger lifted from memory.
    ///
    /// Returns `None` when the magic word or the checksum does not hold,
    /// which is what rejects memory no fault path wrote.
    #[must_use]
    pub fn from_words(words: &[u32; WORD_COUNT]) -> Option<Self>
    {
        let [magic, exception, cfsr, hfsr, mmfar, bfar, abfsr, refusal, checksum] =
            *words;

        if magic != MAGIC
        {
            return None;
        }

        let record = Self
        {
            magic,
            exception,
            cfsr,
            hfsr,
            mmfar,
            bfar,
            abfsr,
            refusal,
            checksum,
        };

        if record.checksum != record.seal()
        {
            return None;
        }

        Some(record)
    }

    /// Returns the word image of the record, in memory order.
    #[must_use]
    pub fn to_words(self) -> [u32; WORD_COUNT]
    {
        [
            self.magic,
            self.exception,
            self.cfsr,
            self.hfsr,
            self.mmfar,
            self.bfar,
            self.abfsr,
            self.refusal,
            self.checksum,
        ]
    }

    /// Returns what the refusal word says, or `None` when no stage refused.
    ///
    /// Every domain is numbered from 1, so a zero word is the one and only
    /// reading of "no stage refused" and `None` says that alone. A word naming
    /// a domain this module does not carry comes back as `Refusal::Unnamed`,
    /// which keeps both halves of it.
    #[must_use]
    pub fn refusal(self) -> Option<Refusal>
    {
        if self.refusal == 0
        {
            return None;
        }

        let code = (self.refusal & CODE_MASK) as u16;

        let refusal = match RefusalDomain::of(self.refusal)
        {
            Some(domain) => Refusal::Named { domain, code },
            None => Refusal::Unnamed
            {
                domain: (self.refusal >> DOMAIN_SHIFT) as u16,
                code,
            },
        };

        Some(refusal)
    }

    /// Returns the registers the fault path captured.
    #[must_use]
    pub fn registers(self) -> FaultRegisters
    {
        FaultRegisters
        {
            exception: self.exception,
            cfsr: self.cfsr,
            hfsr: self.hfsr,
            mmfar: self.mmfar,
            bfar: self.bfar,
            abfsr: self.abfsr,
        }
    }

    /// Returns the faulting address of a `MemManage` fault.
    ///
    /// `None` when `CFSR` leaves `MMARVALID` clear, where `MMFAR` holds an
    /// address from some earlier fault or nothing at all.
    #[must_use]
    pub fn memmanage_address(self) -> Option<u32>
    {
        if self.cfsr & MMARVALID == 0
        {
            return None;
        }

        Some(self.mmfar)
    }

    /// Returns the faulting address of a `BusFault`.
    ///
    /// `None` when `CFSR` leaves `BFARVALID` clear. PM0253 table 65: the
    /// processor writes no address to `BFAR` for an imprecise, a stacking, an
    /// unstacking or an instruction bus error, and a precise fault arriving
    /// before the handler runs leaves the bit set over an address belonging to
    /// that other fault.
    #[must_use]
    pub fn bus_address(self) -> Option<u32>
    {
        if self.cfsr & BFARVALID == 0
        {
            return None;
        }

        Some(self.bfar)
    }

    /// Returns whether the push that enters an exception handler faulted.
    ///
    /// `MSTKERR` and `STKERR` both mean the core adjusted the stack pointer
    /// and then failed to write the frame, so the frame a debugger finds under
    /// it is not the one this exception made.
    #[must_use]
    pub fn stacking_failed(self) -> bool
    {
        self.cfsr & STACKING_ERRORS != 0
    }

    /// Returns whether a configurable fault escalated into `HardFault`.
    ///
    /// When it did, `CFSR` names the original fault and `HFSR` alone does not.
    #[must_use]
    pub fn escalated(self) -> bool
    {
        self.hfsr & FORCED != 0
    }

    /// Returns the checksum of the seven words between the two guards.
    ///
    /// The magic is left out so the two guards stay independent. One of them
    /// covering the other would leave the covered one free to be wrong.
    fn seal(self) -> u32
    {
        let sealed =
        [
            self.exception,
            self.cfsr,
            self.hfsr,
            self.mmfar,
            self.bfar,
            self.abfsr,
            self.refusal,
        ];

        let mut hash = FNV_OFFSET_BASIS;

        for word in sealed
        {
            for byte in word.to_le_bytes()
            {
                hash ^= u32::from(byte);
                hash = hash.wrapping_mul(FNV_PRIME);
            }
        }

        hash
    }
}

#[cfg(test)]
mod tests
{
    #![allow(clippy::indexing_slicing)]

    use super::*;
    use crate::clock::{ClockPlanError, TreeFault};
    use crate::release::{BlockAlarm, Phase, SequenceRefusal};
    use crate::transport::{SequenceFault, TransportFault};

    /// A refusal word distinct from every register value of `sample`, so a
    /// word landing in the wrong slot cannot pass a round trip.
    const SAMPLE_REFUSAL: u32 = 0x0002_0109;

    /// A register set with a distinct value in every field, so a swapped pair
    /// cannot pass a round trip.
    fn sample() -> FaultRegisters
    {
        FaultRegisters
        {
            exception: 0x0000_0003,
            cfsr: 0x0000_8200,
            hfsr: 0x4000_0000,
            mmfar: 0x1111_1111,
            bfar: 0x2222_2222,
            abfsr: 0x0000_0002,
        }
    }

    #[test]
    fn a_sealed_record_reopens_to_the_same_registers()
    {
        let record = FaultRecord::new(&sample(), SAMPLE_REFUSAL);
        let reopened = FaultRecord::from_words(&record.to_words());

        assert_eq!(reopened, Some(record));
        assert_eq!(reopened.map(FaultRecord::registers), Some(sample()));
    }

    #[test]
    fn the_magic_reads_as_puls_in_a_byte_dump()
    {
        assert_eq!(MAGIC.to_le_bytes(), *b"PULS");
    }

    #[test]
    fn the_word_image_starts_with_the_magic_and_holds_the_registers()
    {
        let words = FaultRecord::new(&sample(), SAMPLE_REFUSAL).to_words();

        assert_eq!(words[0], MAGIC);
        assert_eq!(words[1], sample().exception);
        assert_eq!(words[2], sample().cfsr);
        assert_eq!(words[3], sample().hfsr);
        assert_eq!(words[4], sample().mmfar);
        assert_eq!(words[5], sample().bfar);
        assert_eq!(words[6], sample().abfsr);
        assert_eq!(words[7], SAMPLE_REFUSAL);
    }

    #[test]
    fn erased_and_saturated_memory_is_not_a_record()
    {
        assert_eq!(FaultRecord::from_words(&[0x0000_0000; WORD_COUNT]), None);
        assert_eq!(FaultRecord::from_words(&[0xFFFF_FFFF; WORD_COUNT]), None);
        assert_eq!(FaultRecord::from_words(&[MAGIC; WORD_COUNT]), None);
    }

    #[test]
    fn the_magic_alone_does_not_make_a_record()
    {
        let mut words = [0xDEAD_BEEF; WORD_COUNT];
        words[0] = MAGIC;

        assert_eq!(FaultRecord::from_words(&words), None);
    }

    #[test]
    fn a_record_missing_its_magic_is_rejected()
    {
        let mut words = FaultRecord::new(&sample(), SAMPLE_REFUSAL).to_words();
        words[0] = MAGIC ^ 1;

        assert_eq!(FaultRecord::from_words(&words), None);
    }

    #[test]
    fn one_flipped_bit_anywhere_in_a_record_is_rejected()
    {
        let sealed = FaultRecord::new(&sample(), SAMPLE_REFUSAL).to_words();

        for index in 1..WORD_COUNT
        {
            for bit in 0..u32::BITS
            {
                let mut words = sealed;
                words[index] ^= 1 << bit;

                assert_eq!
                (
                    FaultRecord::from_words(&words),
                    None,
                    "word {index} bit {bit} passed the checksum"
                );
            }
        }
    }

    #[test]
    fn the_fault_addresses_follow_their_valid_bits()
    {
        let mut registers = sample();
        registers.cfsr = 0;
        let record = FaultRecord::new(&registers, SAMPLE_REFUSAL);

        assert_eq!(record.memmanage_address(), None);
        assert_eq!(record.bus_address(), None);

        registers.cfsr = MMARVALID;
        let record = FaultRecord::new(&registers, SAMPLE_REFUSAL);
        assert_eq!(record.memmanage_address(), Some(registers.mmfar));
        assert_eq!(record.bus_address(), None);

        registers.cfsr = BFARVALID;
        let record = FaultRecord::new(&registers, SAMPLE_REFUSAL);
        assert_eq!(record.memmanage_address(), None);
        assert_eq!(record.bus_address(), Some(registers.bfar));
    }

    #[test]
    fn a_failed_entry_push_is_reported_from_either_half_of_cfsr()
    {
        let mut registers = sample();

        registers.cfsr = 0;
        assert!(!FaultRecord::new(&registers, SAMPLE_REFUSAL).stacking_failed());

        // MSTKERR, the MemManage half.
        registers.cfsr = 1 << 4;
        assert!(FaultRecord::new(&registers, SAMPLE_REFUSAL).stacking_failed());

        // STKERR, bit 4 of the BusFault half.
        registers.cfsr = 1 << 12;
        assert!(FaultRecord::new(&registers, SAMPLE_REFUSAL).stacking_failed());

        // The two neighbouring bits are unstacking errors, not entry pushes.
        registers.cfsr = (1 << 3) | (1 << 11);
        assert!(!FaultRecord::new(&registers, SAMPLE_REFUSAL).stacking_failed());
    }

    #[test]
    fn escalation_reads_the_forced_bit_alone()
    {
        let mut registers = sample();

        registers.hfsr = 0;
        assert!(!FaultRecord::new(&registers, SAMPLE_REFUSAL).escalated());

        // VECTTBL at bit 1 and DEBUGEVT at bit 31 are not escalation.
        registers.hfsr = (1 << 1) | (1 << 31);
        assert!(!FaultRecord::new(&registers, SAMPLE_REFUSAL).escalated());

        registers.hfsr = FORCED;
        assert!(FaultRecord::new(&registers, SAMPLE_REFUSAL).escalated());
    }

    #[test]
    fn a_record_no_stage_refused_carries_a_zero_word()
    {
        let none = FaultRecord::new(&sample(), 0);

        assert_eq!(none.refusal(), None);
        assert_eq!(none.to_words()[7], 0);

        let refused = ClockFault::PlanRejected(ClockPlanError::VcoOutsideBand);
        let record = FaultRecord::new(&sample(), clock_refusal(refused));

        assert_eq!(refused.code(), 0x0109);
        assert_eq!
        (
            record.refusal(),
            Some(Refusal::Named
            {
                domain: RefusalDomain::Clock,
                code: 0x0109,
            })
        );

        // The two records differ in one word, and the seal moves with it.
        assert_ne!(record.to_words()[8], none.to_words()[8]);
    }

    #[test]
    fn a_tagged_code_reads_back_under_its_own_domain_and_no_other()
    {
        // The sweep runs over the whole code space of each domain rather than
        // over the faults the firmware can build, so it holds for a numbering
        // that grows without this test being touched. The domains come from the
        // declaration that builds the enum and the reader together, so a domain
        // added there is swept here without this test being touched either.
        for &(domain, ceiling) in RefusalDomain::ALL
        {
            for code in 0..=ceiling
            {
                let word = domain.field() | code;
                let sealed = (word & CODE_MASK) as u16;

                // A tagged code never reads as zero, whatever the code is,
                // because the domain field alone is not zero.
                assert_ne!(word, 0);
                assert_eq!(u32::from(sealed), code);

                let record = FaultRecord::new(&sample(), word);

                assert_eq!
                (
                    record.refusal(),
                    Some(Refusal::Named { domain, code: sealed }),
                    "word {word:#010x} did not read back as it was sealed"
                );
            }
        }
    }

    #[test]
    fn the_reader_names_every_domain_the_declaration_carries()
    {
        // The sweep above is only as wide as this table, and the reader is
        // only as complete as the declaration that builds it. Both come from
        // the one declaration, and what is left to read here is that the
        // table carries the domains and the ceilings it is supposed to.
        assert_eq!
        (
            RefusalDomain::ALL,
            &[
                (RefusalDomain::Startup, STARTUP_CODE_CEILING),
                (RefusalDomain::Clock, CLOCK_CODE_CEILING),
                (RefusalDomain::Transport, TRANSPORT_CODE_CEILING),
                (RefusalDomain::ReleaseGate, RELEASE_CODE_CEILING),
            ]
        );

        for &(domain, _) in RefusalDomain::ALL
        {
            assert_eq!(RefusalDomain::of(domain.field()), Some(domain));
        }
    }

    #[test]
    fn a_start_up_code_is_a_cause_byte_with_no_place_over_it()
    {
        // The clock and the transport carry a place byte over the cause. The
        // guards refuse in one place, so a reader of a domain 1 word looks up
        // the whole of it in StartupFault, and this is what says so.
        let guards =
        [
            StartupFault::CoreHandleTaken,
            StartupFault::DeviceHandleTaken,
            StartupFault::BusFaultNotArmed,
        ];

        for guard in guards
        {
            let code = startup_refusal(guard) & CODE_MASK;

            assert_eq!(code >> 8, 0, "{guard:?} carries a place byte");
            assert_eq!(code, guard as u32);
            assert!(code <= STARTUP_CODE_CEILING);
        }
    }

    #[test]
    fn the_tagging_functions_agree_with_the_domains_they_name()
    {
        let clock = ClockFault::PartRefused(TreeFault::PllNeverStopped);
        let transport = TransportFault::Sequence(SequenceFault::StreamNeverStopped);
        let release = ReleaseFault::Sequence(SequenceRefusal::TransfersNeverLapped);
        let startup = StartupFault::CoreHandleTaken;

        // The module documentation names this set as the reason the domain
        // field has to be read before the code.
        assert_eq!(clock.code(), 0x0001);
        assert_eq!(transport.code(), 0x0001);
        assert_eq!(release.code(), 0x0001);
        assert_eq!(startup as u32, 0x0001);

        assert_eq!(clock_refusal(clock), RefusalDomain::Clock.field() | 0x0001);
        assert_eq!(transport_refusal(transport), RefusalDomain::Transport.field() | 0x0001);
        assert_eq!(release_refusal(release), RefusalDomain::ReleaseGate.field() | 0x0001);
        assert_eq!(startup_refusal(startup), RefusalDomain::Startup.field() | 0x0001);

        let words =
        [
            clock_refusal(clock),
            transport_refusal(transport),
            release_refusal(release),
            startup_refusal(startup),
        ];

        for (index, word) in words.into_iter().enumerate()
        {
            assert_eq!
            (
                words.iter().filter(|other| **other == word).count(),
                1,
                "word {index} is shared by two domains"
            );
        }
    }

    #[test]
    fn a_release_word_carries_the_phase_the_mute_line_was_in()
    {
        // The place byte of domain 4 splits into a phase over a site, and
        // whether the line had gone high is the one thing a reader of a parked
        // board most needs off it. The tagging leaves both halves of the code
        // bit for bit, so the phase survives into the record.
        let low = ReleaseFault::MasterBlock(Phase::Muted, BlockAlarm::Underrun);
        let high = ReleaseFault::MasterBlock(Phase::Unmuted, BlockAlarm::Underrun);

        let sealed = |fault| FaultRecord::new(&sample(), release_refusal(fault)).refusal();

        assert_eq!
        (
            sealed(low),
            Some(Refusal::Named { domain: RefusalDomain::ReleaseGate, code: 0x0101 })
        );
        assert_eq!
        (
            sealed(high),
            Some(Refusal::Named { domain: RefusalDomain::ReleaseGate, code: 0x1101 })
        );
    }

    #[test]
    fn a_domain_the_decoder_does_not_name_keeps_both_halves_of_its_word()
    {
        // The field is wider than the stages this firmware refuses at, and a
        // reader of a word from another one is left the word rather than a
        // wrong answer.
        let word = 0x0009_0109;
        let record = FaultRecord::new(&sample(), word);

        assert_eq!
        (
            record.refusal(),
            Some(Refusal::Unnamed { domain: 0x0009, code: 0x0109 })
        );
    }

    #[test]
    fn two_register_sets_that_differ_seal_differently()
    {
        let first = FaultRecord::new(&sample(), SAMPLE_REFUSAL);

        let mut registers = sample();
        registers.mmfar = sample().bfar;
        registers.bfar = sample().mmfar;

        let swapped = FaultRecord::new(&registers, SAMPLE_REFUSAL);

        assert_ne!(swapped, first);
        assert_ne!(swapped.to_words()[8], first.to_words()[8]);
    }
}
