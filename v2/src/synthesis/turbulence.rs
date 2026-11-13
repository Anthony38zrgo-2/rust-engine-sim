//! Deterministic exhaust turbulence envelope and band-pass noise source.

use crate::dsp::{xorshift64_signed, BandPassNoise};

/// Allocation-free turbulent exhaust noise stage driven by blowdown trigger magnitude.
#[derive(Clone, Copy, Debug)]
pub struct TurbulenceGenerator {
    release_alpha: f32,
    filter: BandPassNoise,
    envelope: f32,
    rng: u64,
}

impl TurbulenceGenerator {
    /// Initializes turbulence band-pass filtering and deterministic PRNG state.
    pub fn new(seed: u64, sample_rate: f32) -> Self {
        Self::with_band(seed, sample_rate, 460.0, 2_050.0)
    }

    /// Initializes turbulence band-pass filtering over custom `[low_hz, high_hz]` edges.
    pub fn with_band(seed: u64, sample_rate: f32, low_hz: f32, high_hz: f32) -> Self {
        Self {
            release_alpha: 1.0 - (-1.0 / (0.0038 * sample_rate)).exp(),
            filter: BandPassNoise::new(low_hz, high_hz, sample_rate),
            envelope: 0.0,
            rng: seed ^ 0xA17E_5EED_D15C_A11E,
        }
    }

    /// Advances the turbulence envelope and renders one filtered noise sample.
    #[inline]
    pub fn process(&mut self, trigger: f32, gain: f32, tc_cut: f32) -> f32 {
        let attack = 0.34;
        let release = self.release_alpha;
        if trigger > self.envelope {
            self.envelope += attack * (trigger - self.envelope);
        } else {
            self.envelope += release * (trigger - self.envelope);
        }
        let noise = xorshift64_signed(&mut self.rng);
        let tc_texture = 1.0 + 0.12 * tc_cut;
        self.filter.process(noise) * self.envelope.sqrt() * gain * tc_texture
    }
}
