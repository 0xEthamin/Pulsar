//! Hardware independent core shared by both Pulsar firmwares.
//!
//! Holds the fixed constants of the machine, the control protocol the two
//! boards speak, the gate a control message passes before it reaches audio, the
//! coefficient maths of the crossover, the encoding of the fault record the
//! processing board leaves for a debugger, and the two hardware plans that
//! board runs on: the audio clock and the output transport. A plan carries the
//! field encodings, the bounds and the read-back comparison, so the firmware
//! that writes the registers is the register block and nothing else, and every
//! rule is tested on a host. Nothing here touches a peripheral.

#![no_std]

pub mod clock;
pub mod constants;
pub mod control;
pub mod filter;
pub mod postmortem;
pub mod protocol;
pub mod transport;
