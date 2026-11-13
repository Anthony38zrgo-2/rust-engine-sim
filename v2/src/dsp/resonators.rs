//! Second-order modal resonators and banks for structural and exhaust cavities.

use std::f32::consts::TAU;

/// Damped second-order resonant mode normalized by pole radius (`1 - r`) so longer
/// decay times preserve steady-state level.
#[derive(Clone, Copy, Debug)]
pub struct Resonator {
    a1: f32,
    a2: f32,
    gain: f32,
    y1: f32,
    y2: f32,
}

impl Resonator {
    /// Builds a modal resonator tuned to `frequency_hz` with exponential decay `decay_seconds`.
    pub fn new(frequency_hz: f32, decay_seconds: f32, gain: f32, sample_rate: f32) -> Self {
        assert!(frequency_hz > 0.0 && frequency_hz < sample_rate * 0.48);
        assert!(decay_seconds > 0.0005 && decay_seconds < 2.0);
        let radius = (-1.0 / (decay_seconds * sample_rate)).exp();
        let omega = TAU * frequency_hz / sample_rate;
        Self {
            a1: 2.0 * radius * omega.cos(),
            a2: -(radius * radius),
            gain: gain * (1.0 - radius),
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Excites the resonator by one sample and returns its response.
    #[inline]
    pub fn process(&mut self, excitation: f32) -> f32 {
        let y = excitation * self.gain + self.a1 * self.y1 + self.a2 * self.y2;
        self.y2 = self.y1;
        self.y1 = if y.abs() < 1.0e-20 { 0.0 } else { y };
        self.y1
    }
}

/// Parallel bank of tuned `Resonator` modes excited from a single structural signal.
#[derive(Clone, Debug)]
pub struct ModalBank {
    modes: Vec<Resonator>,
}

impl ModalBank {
    /// Allocates all modes at initialization time from `(frequency_hz, decay_seconds, gain)` tuples.
    pub fn new(spec: &[(f32, f32, f32)], sample_rate: f32) -> Self {
        Self {
            modes: spec
                .iter()
                .map(|&(frequency_hz, decay_seconds, gain)| {
                    Resonator::new(frequency_hz, decay_seconds, gain, sample_rate)
                })
                .collect(),
        }
    }

    /// Evaluates every mode in the bank for `excitation` and returns the sum.
    #[inline]
    pub fn process(&mut self, excitation: f32) -> f32 {
        self.modes
            .iter_mut()
            .map(|mode| mode.process(excitation))
            .sum()
    }
}
