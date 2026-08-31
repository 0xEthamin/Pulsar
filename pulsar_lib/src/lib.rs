//! Hardware independent core shared by both Pulsar firmwares.
//!
//! Holds the fixed constants of the machine, the control protocol the two
//! boards speak, and the gate a control message passes before it reaches audio.
//! Nothing here touches a peripheral.

#![no_std]

pub mod constants;
pub mod control;
pub mod protocol;
