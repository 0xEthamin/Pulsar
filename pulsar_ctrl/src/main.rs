//! Bluetooth receiver and user interface firmware for the ESP32.
//!
//! This board owns the radio link, the SBC decode and everything the user
//! touches. It forwards the decoded samples to the processing board untouched,
//! with no filtering, no gain and no limiting.
//!
//! The audio path outranks every other task here.

use esp_idf_svc::sys::link_patches;

fn main()
{
    // The ESP-IDF link step drops these symbols without an explicit reference.
    link_patches();
}
