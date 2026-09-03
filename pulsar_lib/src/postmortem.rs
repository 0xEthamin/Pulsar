//! Fault record the processing board leaves at a fixed address.
//!
//! The processing firmware writes one `FaultRecord` from its fault path and
//! parks the core. Nothing on the board reads it back. The reader is a person
//! with a probe on SWD, so the layout below is the interface, and the code here
//! is the reference that produces it.
//!
//! # Reading a record
//!
//! Take 32 bytes from the `FAULT_RECORD` symbol of the processing image, which
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
//! +0x1C  checksum   FNV-1a over the 24 bytes from +0x04 to +0x1B.
//! ```
//!
//! Bit positions come from PM0253 tables 64, 65, 67 and 109. `HFSR` bit 30 says
//! a configurable fault escalated, which is when `CFSR` rather than `HFSR`
//! names what happened.
//!
//! The `CFSR` and `ABFSR` bits are sticky. PM0253 clears them on a write or a
//! reset and this firmware writes neither, so a record shows what has faulted
//! since reset and not only what faulted last.
//!
//! # Telling a record from stale memory
//!
//! Memory no fault has written holds whatever the previous run left there, so a
//! reader has to tell the two apart. Two guards cover the image between them
//! and neither covers what the other does: word 0 is a fixed magic, and word 7
//! is a checksum over the six words between them. Both must hold.
//!
//! Nothing clears the record and nothing dates it, so a record that holds may
//! predate the reset being examined.
//!
//! # What this module is
//!
//! `FaultRecord::new` is the encoder the processing firmware calls. The
//! decoding half, `from_words` and the accessors, is the reference that pins
//! the encoding down and the tests exercise. No shipped code decodes a record.
//!
//! No register is read here. The processing firmware reads them and fills
//! `FaultRegisters`.

/// Words one record occupies. A reader takes this many from the record address.
pub const WORD_COUNT: usize = 8;

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

/// One captured register set, sealed so a reader can tell it from stale
/// memory.
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
    checksum: u32,
}

impl FaultRecord
{
    /// Seals `registers` into a record.
    pub fn new(registers: &FaultRegisters) -> Self
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
        let [magic, exception, cfsr, hfsr, mmfar, bfar, abfsr, checksum] =
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
            self.checksum,
        ]
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

    /// Returns the checksum of the six register words.
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
        let record = FaultRecord::new(&sample());
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
        let words = FaultRecord::new(&sample()).to_words();

        assert_eq!(words[0], MAGIC);
        assert_eq!(words[1], sample().exception);
        assert_eq!(words[2], sample().cfsr);
        assert_eq!(words[3], sample().hfsr);
        assert_eq!(words[4], sample().mmfar);
        assert_eq!(words[5], sample().bfar);
        assert_eq!(words[6], sample().abfsr);
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
        let mut words = FaultRecord::new(&sample()).to_words();
        words[0] = MAGIC ^ 1;

        assert_eq!(FaultRecord::from_words(&words), None);
    }

    #[test]
    fn one_flipped_bit_anywhere_in_a_record_is_rejected()
    {
        let sealed = FaultRecord::new(&sample()).to_words();

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
        let record = FaultRecord::new(&registers);

        assert_eq!(record.memmanage_address(), None);
        assert_eq!(record.bus_address(), None);

        registers.cfsr = MMARVALID;
        let record = FaultRecord::new(&registers);
        assert_eq!(record.memmanage_address(), Some(registers.mmfar));
        assert_eq!(record.bus_address(), None);

        registers.cfsr = BFARVALID;
        let record = FaultRecord::new(&registers);
        assert_eq!(record.memmanage_address(), None);
        assert_eq!(record.bus_address(), Some(registers.bfar));
    }

    #[test]
    fn a_failed_entry_push_is_reported_from_either_half_of_cfsr()
    {
        let mut registers = sample();

        registers.cfsr = 0;
        assert!(!FaultRecord::new(&registers).stacking_failed());

        // MSTKERR, the MemManage half.
        registers.cfsr = 1 << 4;
        assert!(FaultRecord::new(&registers).stacking_failed());

        // STKERR, bit 4 of the BusFault half.
        registers.cfsr = 1 << 12;
        assert!(FaultRecord::new(&registers).stacking_failed());

        // The two neighbouring bits are unstacking errors, not entry pushes.
        registers.cfsr = (1 << 3) | (1 << 11);
        assert!(!FaultRecord::new(&registers).stacking_failed());
    }

    #[test]
    fn escalation_reads_the_forced_bit_alone()
    {
        let mut registers = sample();

        registers.hfsr = 0;
        assert!(!FaultRecord::new(&registers).escalated());

        // VECTTBL at bit 1 and DEBUGEVT at bit 31 are not escalation.
        registers.hfsr = (1 << 1) | (1 << 31);
        assert!(!FaultRecord::new(&registers).escalated());

        registers.hfsr = FORCED;
        assert!(FaultRecord::new(&registers).escalated());
    }

    #[test]
    fn two_register_sets_that_differ_seal_differently()
    {
        let first = FaultRecord::new(&sample());

        let mut registers = sample();
        registers.mmfar = sample().bfar;
        registers.bfar = sample().mmfar;

        let swapped = FaultRecord::new(&registers);

        assert_ne!(swapped, first);
        assert_ne!(swapped.to_words()[7], first.to_words()[7]);
    }
}
