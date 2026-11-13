//! Second-order modal resonators and banks for structural and exhaust cavities.

use std::f32::consts::{PI, TAU};

use crate::dsp::xorshift64_signed;

/// Frequency threshold above which modal banks become dense enough for subtle load/thermal drift.
const MODAL_DRIFT_MIN_HZ: f32 = 1_200.0;

/// Damped second-order resonant mode normalized by pole radius (`1 - r`) so longer
/// decay times preserve steady-state level.
#[derive(Clone, Copy, Debug)]
pub struct Resonator {
    a1: f32,
    a2: f32,
    gain: f32,
    y1: f32,
    y2: f32,
    frequency_hz: f32,
    base_omega: f32,
    two_radius: f32,
}

impl Resonator {
    /// Builds a modal resonator tuned to `frequency_hz` with exponential decay `decay_seconds`.
    pub fn new(frequency_hz: f32, decay_seconds: f32, gain: f32, sample_rate: f32) -> Self {
        assert!(frequency_hz > 0.0 && frequency_hz < sample_rate * 0.48);
        assert!(decay_seconds > 0.0005 && decay_seconds < 2.0);
        let radius = (-1.0 / (decay_seconds * sample_rate)).exp();
        let omega = TAU * frequency_hz / sample_rate;
        let two_radius = 2.0 * radius;
        Self {
            a1: two_radius * omega.cos(),
            a2: -(radius * radius),
            gain: gain * (1.0 - radius),
            y1: 0.0,
            y2: 0.0,
            frequency_hz,
            base_omega: omega,
            two_radius,
        }
    }

    /// Natural resonant frequency (Hz).
    #[inline]
    pub fn frequency_hz(&self) -> f32 {
        self.frequency_hz
    }

    /// Updates pole angle safely along the constant-radius stability circle (`|z| = r < 1`).
    #[inline]
    pub fn set_frequency_scale(&mut self, scale: f32) {
        let omega = (self.base_omega * scale).clamp(0.0001, PI * 0.96);
        self.a1 = self.two_radius * omega.cos();
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
    drift_states: Vec<f32>,
    drift_rng: u64,
    drift_counter: u32,
    drift_stride: u32,
    drift_alpha: f32,
}

impl ModalBank {
    /// Allocates all modes at initialization time from `(frequency_hz, decay_seconds, gain)` tuples.
    pub fn new(spec: &[(f32, f32, f32)], sample_rate: f32) -> Self {
        let mut seed = 0x5EED_B4A9_u64;
        for &(f, d, g) in spec {
            seed = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(
                (f.to_bits() as u64) ^ ((d.to_bits() as u64) << 16) ^ g.to_bits() as u64,
            );
        }
        let drift_stride = ((sample_rate / 3_000.0).round() as u32).max(1);
        let control_rate = sample_rate / drift_stride as f32;
        Self {
            modes: spec
                .iter()
                .map(|&(frequency_hz, decay_seconds, gain)| {
                    Resonator::new(frequency_hz, decay_seconds, gain, sample_rate)
                })
                .collect(),
            drift_states: vec![0.0; spec.len()],
            drift_rng: seed | 1,
            drift_counter: 0,
            drift_stride,
            drift_alpha: 1.0 - (-TAU * 3.1 / control_rate).exp(),
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

    /// Evaluates all modes while gently drifting poles above `1200 Hz` by load and stochastic walk.
    #[inline]
    pub fn process_modulated(&mut self, excitation: f32, load: f32, drift_depth: f32) -> f32 {
        if self.drift_counter == 0 {
            let clamped_load = load.clamp(0.0, 1.0);
            for (mode, eta) in self.modes.iter_mut().zip(self.drift_states.iter_mut()) {
                if mode.frequency_hz() > MODAL_DRIFT_MIN_HZ {
                    let target = xorshift64_signed(&mut self.drift_rng);
                    *eta += self.drift_alpha * (target - *eta);
                    let scale = 1.0 + drift_depth * (0.010 * clamped_load + 0.007 * *eta);
                    mode.set_frequency_scale(scale);
                }
            }
        }
        self.drift_counter += 1;
        if self.drift_counter >= self.drift_stride {
            self.drift_counter = 0;
        }
        self.process(excitation)
    }
}
