//! Síntesis de transmisión y reproducción de one-shots de cambio.

pub mod shift_samples;
pub mod transmission;

pub use crate::config::TransmissionConfig;
pub use shift_samples::{GearShiftPlayer, GearShiftSamples};
pub use transmission::{TransmissionInput, TransmissionSynth};
