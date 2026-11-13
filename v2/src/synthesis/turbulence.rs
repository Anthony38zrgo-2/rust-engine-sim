//! Deterministic exhaust turbulence envelope and band-pass noise source.

use crate::dsp::{xorshift64_signed, BandPassNoise, OnePoleLowPass, StateVariableFilter};

/// High-frequency Strouhal jet-shear air-absorption cutoff (Hz).
const JET_AIR_LOWPASS_HZ: f32 = 8_500.0;
/// Core turbulence asymmetric envelope constants in refined mode (~0.8 ms attack / ~11 ms release).
const REFINED_ATTACK_SECONDS: f32 = 0.0008;
const REFINED_RELEASE_SECONDS: f32 = 0.011;
/// Cycle-averaged mean-flow baseline tracker time constant (~55 ms).
const MEAN_FLOW_SECONDS: f32 = 0.055;
/// Slower aerodynamic jet-shear envelope time constants (~3.5 ms attack / ~28 ms release).
const JET_ATTACK_SECONDS: f32 = 0.0035;
const JET_RELEASE_SECONDS: f32 = 0.028;
/// Slow turbulent swirl LFO cutoff (~9.5 Hz) modulating the high-frequency shear band.
const JET_LFO_HZ: f32 = 9.5;

/// Allocation-free turbulent exhaust noise stage driven by blowdown trigger magnitude.
#[derive(Clone, Copy, Debug)]
pub struct TurbulenceGenerator {
    release_alpha: f32,
    refined_attack_alpha: f32,
    refined_release_alpha: f32,
    mean_flow_alpha: f32,
    jet_attack_alpha: f32,
    jet_release_alpha: f32,
    lfo_alpha: f32,
    lfo_gain_norm: f32,
    filter: BandPassNoise,
    jet_svf: StateVariableFilter,
    jet_air_lowpass: OnePoleLowPass,
    refined_dsp: bool,
    jet_base_hz: f32,
    jet_span_hz: f32,
    envelope: f32,
    mean_flow_env: f32,
    jet_envelope: f32,
    lfo_state: f32,
    rng: u64,
    rng_jet: u64,
    rng_lfo: u64,
}

impl TurbulenceGenerator {
    /// Initializes turbulence band-pass filtering and deterministic PRNG state.
    pub fn new(seed: u64, sample_rate: f32) -> Self {
        Self::with_band(seed, sample_rate, 460.0, 2_050.0)
    }

    /// Initializes turbulence band-pass filtering over custom `[low_hz, high_hz]` edges.
    pub fn with_band(seed: u64, sample_rate: f32, low_hz: f32, high_hz: f32) -> Self {
        let time_alpha = |seconds: f32| 1.0 - (-1.0 / (seconds * sample_rate.max(1.0))).exp();
        let lfo_alpha = 1.0 - (-std::f32::consts::TAU * JET_LFO_HZ / sample_rate.max(1.0)).exp();
        let lfo_rms_ratio = (lfo_alpha / (2.0 - lfo_alpha).max(1.0e-6)).sqrt();
        Self {
            release_alpha: time_alpha(0.0038),
            refined_attack_alpha: time_alpha(REFINED_ATTACK_SECONDS),
            refined_release_alpha: time_alpha(REFINED_RELEASE_SECONDS),
            mean_flow_alpha: time_alpha(MEAN_FLOW_SECONDS),
            jet_attack_alpha: time_alpha(JET_ATTACK_SECONDS),
            jet_release_alpha: time_alpha(JET_RELEASE_SECONDS),
            lfo_alpha,
            lfo_gain_norm: (0.70 / lfo_rms_ratio.max(1.0e-4)).min(45.0),
            filter: BandPassNoise::new(low_hz, high_hz, sample_rate),
            jet_svf: StateVariableFilter::new(sample_rate),
            jet_air_lowpass: OnePoleLowPass::lowpass_or_bypass(JET_AIR_LOWPASS_HZ, sample_rate),
            refined_dsp: false,
            jet_base_hz: 2_300.0,
            jet_span_hz: 4_200.0,
            envelope: 0.0,
            mean_flow_env: 0.0,
            jet_envelope: 0.0,
            lfo_state: 0.0,
            rng: seed ^ 0xA17E_5EED_D15C_A11E,
            rng_jet: (seed ^ 0x517C_C1B7_2722_0A95) | 1,
            rng_lfo: (seed ^ 0x9E37_79B9_7F4A_7C15) | 1,
        }
    }

    /// Enables Strouhal-scaled high-band valve/exhaust jet shear noise over `[base_hz, base_hz + span_hz]`.
    pub fn with_refined_jet(mut self, enabled: bool, base_hz: f32, span_hz: f32) -> Self {
        self.refined_dsp = enabled;
        self.jet_base_hz = base_hz;
        self.jet_span_hz = span_hz;
        self
    }

    /// Advances the turbulence envelope and renders one filtered noise sample.
    #[inline]
    pub fn process(&mut self, trigger: f32, gain: f32, tc_cut: f32) -> f32 {
        self.process_with_rpm(trigger, gain, tc_cut, 0.0)
    }

    /// Advances the turbulence envelope and renders core roar plus optional Strouhal-scaled jet hiss.
    #[inline]
    pub fn process_with_rpm(&mut self, trigger: f32, gain: f32, tc_cut: f32, rpm_norm: f32) -> f32 {
        let tc_texture = 1.0 + 0.12 * tc_cut;
        let noise = xorshift64_signed(&mut self.rng);
        if !self.refined_dsp {
            let attack = 0.34;
            let release = self.release_alpha;
            if trigger > self.envelope {
                self.envelope += attack * (trigger - self.envelope);
            } else {
                self.envelope += release * (trigger - self.envelope);
            }
            let core_roar = self.filter.process(noise) * self.envelope.sqrt();
            return core_roar * gain * tc_texture;
        }

        // Cycle-averaged exhaust mass flow prevents firing-rate AM gaps between blowdown events.
        self.mean_flow_env += self.mean_flow_alpha * (trigger - self.mean_flow_env);
        let core_trigger = 0.45 * trigger + 0.55 * self.mean_flow_env;
        let core_rate = if core_trigger > self.envelope {
            self.refined_attack_alpha
        } else {
            self.refined_release_alpha
        };
        self.envelope += core_rate * (core_trigger - self.envelope);

        let jet_trigger = 0.25 * trigger + 0.75 * self.mean_flow_env;
        let jet_rate = if jet_trigger > self.jet_envelope {
            self.jet_attack_alpha
        } else {
            self.jet_release_alpha
        };
        self.jet_envelope += jet_rate * (jet_trigger - self.jet_envelope);

        let lfo_white = xorshift64_signed(&mut self.rng_lfo);
        self.lfo_state += self.lfo_alpha * (lfo_white - self.lfo_state);
        let lfo_norm = (self.lfo_state * self.lfo_gain_norm).clamp(-1.0, 1.0);

        let core_roar = self.filter.process(noise) * self.envelope.max(0.0).sqrt();
        let jet_noise = xorshift64_signed(&mut self.rng_jet);
        let jet_amp = self.jet_envelope.max(0.0).sqrt();
        let strouhal = (0.55 * rpm_norm.clamp(0.0, 1.0)
            + 0.45 * (self.jet_envelope * 1.4).clamp(0.0, 1.0))
        .clamp(0.0, 1.0);
        let jet_fc = (self.jet_base_hz + strouhal * self.jet_span_hz) * (1.0 + 0.06 * lfo_norm);
        let jet_band = self
            .jet_air_lowpass
            .process(self.jet_svf.process_bandpass(jet_noise, jet_fc, 1.15));
        let jet_hiss =
            jet_band * jet_amp * (1.0 + 0.16 * lfo_norm) * (0.28 + 0.72 * strouhal) * 0.30;
        (core_roar + jet_hiss) * gain * tc_texture
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsp::Biquad;

    #[test]
    fn refined_jet_band_adds_high_frequency_turbulence_scaled_by_rpm() {
        let sr = 48_000.0;
        let mut legacy = TurbulenceGenerator::with_band(42, sr, 210.0, 1_350.0);
        let mut refined = TurbulenceGenerator::with_band(42, sr, 210.0, 1_350.0)
            .with_refined_jet(true, 1_850.0, 3_100.0);
        let mut hp_legacy = Biquad::highpass(2_800.0, 0.707, sr);
        let mut hp_refined_low = Biquad::highpass(2_800.0, 0.707, sr);
        let mut hp_refined_high = Biquad::highpass(2_800.0, 0.707, sr);
        let mut refined_low_rpm = refined;

        let mut e_legacy = 0.0f64;
        let mut e_low_rpm = 0.0f64;
        let mut e_high_rpm = 0.0f64;
        for n in 0..24_000 {
            let trig = 0.22 + 0.08 * (std::f32::consts::TAU * 120.0 * n as f32 / sr).sin();
            let s_leg = hp_legacy.process(legacy.process(trig, 1.0, 0.0));
            let s_low =
                hp_refined_low.process(refined_low_rpm.process_with_rpm(trig, 1.0, 0.0, 0.20));
            let s_high = hp_refined_high.process(refined.process_with_rpm(trig, 1.0, 0.0, 0.95));
            if n > 4_000 {
                e_legacy += (s_leg as f64).powi(2);
                e_low_rpm += (s_low as f64).powi(2);
                e_high_rpm += (s_high as f64).powi(2);
            }
        }
        assert!(e_low_rpm > e_legacy * 1.02);
        assert!(e_high_rpm > e_legacy * 2.5);
        assert!(e_high_rpm > e_low_rpm * 2.0);
    }

    #[test]
    fn refined_mean_flow_and_slower_envelope_suppress_periodic_am_buzz() {
        let sr = 48_000.0;
        let mut legacy = TurbulenceGenerator::new(7, sr);
        let mut refined = TurbulenceGenerator::new(7, sr).with_refined_jet(true, 2_300.0, 4_200.0);
        // Exciting with sharp 200 Hz periodic blowdown pulses; measure envelope ripple after warmup
        let mut leg_envelopes = Vec::new();
        let mut ref_envelopes = Vec::new();
        for n in 0..16_000 {
            let trig = if n % 240 < 12 { 1.0 } else { 0.0 };
            let _ = legacy.process(trig, 1.0, 0.0);
            let _ = refined.process_with_rpm(trig, 1.0, 0.0, 0.85);
            if n >= 8_000 {
                leg_envelopes.push(legacy.envelope);
                ref_envelopes.push(refined.jet_envelope);
            }
        }
        let span = |vals: &[f32]| -> f32 {
            let min = vals.iter().copied().fold(f32::INFINITY, f32::min);
            let max = vals.iter().copied().fold(f32::NEG_INFINITY, f32::max);
            let mean = vals.iter().sum::<f32>() / vals.len() as f32;
            (max - min) / mean.max(1.0e-6)
        };
        let leg_ripple = span(&leg_envelopes);
        let jet_ripple = span(&ref_envelopes);
        assert!(
            jet_ripple < leg_ripple * 0.25,
            "jet AM ripple {jet_ripple} should be much smoother than legacy {leg_ripple}"
        );
    }
}
