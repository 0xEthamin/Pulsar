//! Hardware independent core shared by both Pulsar firmwares.
//!
//! Holds the fixed constants of the machine, the control protocol the two
//! boards speak, the gate a control message passes before it reaches audio, the
//! coefficient maths of the crossover, and the encoding of the fault record the
//! processing board leaves for a debugger. Nothing here touches a peripheral.

#![no_std]

pub mod clock;
pub mod constants;
pub mod control;
pub mod filter;
pub mod postmortem;
pub mod protocol;
