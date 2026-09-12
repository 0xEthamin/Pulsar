//! Hardware independent core shared by both Pulsar firmwares.
//!
//! Holds the fixed constants of the machine, the control protocol the two
//! boards speak, the gate a control message passes before it reaches audio, the
//! coefficient maths of the crossover, the output stage that carries the
//! sensitivity alignment and the ceiling of the high way, the encoding of the
//! fault record the processing board leaves for a debugger, the two hardware
//! plans that board runs on, the audio clock and the output transport, and the
//! gate that raises the converter mute line once every stage feeding the
//! converters has reported. A plan carries the field encodings, the bounds and
//! the read-back comparison, so the firmware that writes the registers is the
//! register block and nothing else, and every rule is tested on a host. Nothing
//! here touches a peripheral.

#![no_std]

// The exhaustive sweep of the output stage runs its stretches at once, which
// needs threads. Nothing outside a test build reaches this.
#[cfg(test)]
extern crate std;

pub mod clock;
pub mod constants;
pub mod control;
pub mod filter;
mod output;
pub mod passthrough;
pub mod postmortem;
pub mod protocol;
mod readback;
pub mod release;
pub mod transport;
