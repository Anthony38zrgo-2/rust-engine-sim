//! Acoustic exhaust-runner waveguides and 5-in-1 collector junctions.

use std::f32::consts::TAU;

use crate::config::physical::{
    CollectorGeometry, COLLECTOR_END_CORRECTION_RADIUS_FACTOR, GAMMA,
    SPECIFIC_GAS_CONSTANT_J_PER_KG_K,
};
use crate::dsp::{hermite_4pt, Biquad, DcBlocker, ModalBank, OnePoleLowPass};

/// Coldest runner gas temperature considered when sizing the circular delay buffer (K).
const MIN_RUNNER_TEMPERATURE_K: f32 = 200.0;

/// Viscous and thermal wall boundary-layer acoustic attenuation coefficient per runner transit.
pub const RUNNER_WALL_LOSS_COEFF: f32 = 0.32;
/// Incident collector-bound runner wall boundary-layer cutoff (Hz) when refined DSP is enabled.
const RUNNER_INCIDENT_WALL_LOWPASS_HZ: f32 = 9_500.0;
/// Thermal wave-speed slew time constant (~3.5 ms) separating physical T_run from acoustic delay.
const RUNNER_TEMPERATURE_SLEW_SECONDS: f32 = 0.0035;
/// Open-end runner radiation reflection cutoff (Hz) so high frequencies radiate out instead of ringing.
const RUNNER_RADIATION_REFLECTION_CUTOFF_HZ: f32 = 1_650.0;

/// Quadratic confluence dissipation coefficient at the 5-in-1 collector junction.
pub const COLLECTOR_CONFLUENCE_LOSS_COEFF: f32 = 0.08;
/// Inflow coupling coefficient from runner incident pulses to collector chamber pressure.
pub const COLLECTOR_CHAMBER_INFLOW_COEFF: f32 = 0.42;
/// Derived-mode cap for the geometric collector bank.
const COLLECTOR_MAX_MODES: usize = 12;
/// Highest derived collector mode considered before the sample-rate guard.
const COLLECTOR_MAX_MODE_HZ: f32 = 6_000.0;
/// Local oversampling factor used by `Collector` in refined DSP mode.
const COLLECTOR_OVERSAMPLE: usize = 4;
/// Eighth-order Butterworth section Q factors (`1 / (2 * cos((2k + 1) * pi / 16))`)
/// for flat passband and steep image/alias attenuation in 4x local collector oversampling.
const BUTTERWORTH_8_QS: [f32; 4] = [0.509_795_6, 0.601_344_9, 0.899_976_2, 2.562_915_4];

/// Instantaneous acoustic state of the runner waveguide.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RunnerAcousticState {
    pub arrived: f32,
    pub acoustic_pressure: f32,
    pub reflected: f32,
}

/// One-dimensional exhaust primary header waveguide with optional gas-temperature
/// wave-speed tracking and viscous wall reflection loss.
#[derive(Clone, Debug)]
pub struct RunnerWaveguide {
    delay: Vec<f32>,
    cursor: usize,
    reflection: f32,
    feedback_lowpass: f32,
    radiation_reflect_lp_a: OnePoleLowPass,
    radiation_reflect_lp_b: OnePoleLowPass,
    incident_lowpass: OnePoleLowPass,
    acoustic_temperature_k: f32,
    temperature_slew_alpha: f32,
    refined_dsp: bool,
    length_m: f32,
    sample_rate: f32,
    temperature_dependent: bool,
    acoustic_pressure: f32,
    reflected_wave: f32,
    last_arrived: f32,
}

impl RunnerWaveguide {
    /// Allocates circular delay memory once on initialization for fixed or coldest gas speed.
    pub fn new(
        length_m: f32,
        wave_speed_mps: f32,
        reflection: f32,
        sample_rate: f32,
        temperature_dependent: bool,
    ) -> Self {
        let fixed_delay_samples =
            (length_m / wave_speed_mps * sample_rate).round().max(2.0) as usize;
        let buffer_len = if temperature_dependent {
            let min_speed = speed_of_sound_mps(MIN_RUNNER_TEMPERATURE_K);
            (length_m / min_speed * sample_rate).ceil() as usize + 2
        } else {
            fixed_delay_samples
        };
        Self {
            delay: vec![0.0; buffer_len.max(fixed_delay_samples)],
            cursor: 0,
            reflection,
            feedback_lowpass: 0.0,
            radiation_reflect_lp_a: OnePoleLowPass::lowpass_or_bypass(
                RUNNER_RADIATION_REFLECTION_CUTOFF_HZ,
                sample_rate,
            ),
            radiation_reflect_lp_b: OnePoleLowPass::lowpass_or_bypass(
                RUNNER_RADIATION_REFLECTION_CUTOFF_HZ,
                sample_rate,
            ),
            incident_lowpass: OnePoleLowPass::lowpass_or_bypass(
                RUNNER_INCIDENT_WALL_LOWPASS_HZ,
                sample_rate,
            ),
            acoustic_temperature_k: -1.0,
            temperature_slew_alpha: 1.0
                - (-1.0 / (RUNNER_TEMPERATURE_SLEW_SECONDS * sample_rate.max(1.0))).exp(),
            refined_dsp: false,
            length_m,
            sample_rate,
            temperature_dependent,
            acoustic_pressure: 0.0,
            reflected_wave: 0.0,
            last_arrived: 0.0,
        }
    }

    /// Enables 4-point Catmull-Rom cubic interpolation, smoothed acoustic temperature,
    /// open-end radiation rolloff, and incident wall absorption.
    pub fn with_refined_dsp(mut self, enabled: bool) -> Self {
        self.refined_dsp = enabled;
        self
    }

    /// Propagates one sample of exhaust excitation at `temperature_k`.
    #[inline]
    pub fn process(&mut self, input: f32, temperature_k: f32) -> f32 {
        if !self.temperature_dependent {
            return self.process_fixed(input);
        }
        let target_temperature_k = temperature_k.max(MIN_RUNNER_TEMPERATURE_K);
        let effective_temperature_k = if self.refined_dsp {
            if self.acoustic_temperature_k < 0.0 {
                self.acoustic_temperature_k = target_temperature_k;
            } else {
                self.acoustic_temperature_k += self.temperature_slew_alpha
                    * (target_temperature_k - self.acoustic_temperature_k);
            }
            self.acoustic_temperature_k
        } else {
            target_temperature_k
        };
        let speed = speed_of_sound_mps(effective_temperature_k);
        let len = self.delay.len();
        let n = len as f32;
        if self.refined_dsp && len >= 5 {
            let max_delay = (len - 2) as f32;
            let min_delay = 3.0_f32.min(max_delay);
            let delay_float =
                (self.length_m / speed * self.sample_rate).clamp(min_delay, max_delay);
            let mut read = self.cursor as f32 - delay_float;
            if read < 0.0 {
                read += n;
            }
            if read >= n {
                read -= n;
            }
            let i0 = read.floor() as usize;
            let i_m1 = (i0 + len - 1) % len;
            let i1 = (i0 + 1) % len;
            let i2 = (i0 + 2) % len;
            let frac = read - i0 as f32;
            let arrived = hermite_4pt(
                self.delay[i_m1],
                self.delay[i0],
                self.delay[i1],
                self.delay[i2],
                frac,
            );
            return self.advance_wave(input, arrived);
        }
        let max_delay = (len - 1) as f32;
        let delay_float = (self.length_m / speed * self.sample_rate).clamp(2.0, max_delay);
        let mut read = self.cursor as f32 - delay_float;
        if read < 0.0 {
            read += n;
        }
        if read >= n {
            read -= n;
        }
        let i0 = read.floor() as usize;
        let i1 = (i0 + 1) % len;
        let frac = read - read.floor();
        let arrived = self.delay[i0] + frac * (self.delay[i1] - self.delay[i0]);
        self.advance_wave(input, arrived)
    }

    #[inline]
    fn process_fixed(&mut self, input: f32) -> f32 {
        let arrived = self.delay[self.cursor];
        self.advance_wave(input, arrived)
    }

    #[inline]
    fn advance_wave(&mut self, input: f32, arrived: f32) -> f32 {
        let reflected = if self.refined_dsp {
            let radiation_damped = self
                .radiation_reflect_lp_b
                .process(self.radiation_reflect_lp_a.process(arrived));
            self.feedback_lowpass = radiation_damped;
            self.reflection * radiation_damped
        } else {
            self.feedback_lowpass += RUNNER_WALL_LOSS_COEFF * (arrived - self.feedback_lowpass);
            self.reflection * self.feedback_lowpass
        };
        let acoustic_pressure = input + reflected;
        self.delay[self.cursor] = acoustic_pressure;
        self.cursor += 1;
        if self.cursor == self.delay.len() {
            self.cursor = 0;
        }
        let transmitted = if self.refined_dsp {
            self.incident_lowpass.process(arrived)
        } else {
            arrived
        };
        self.acoustic_pressure = acoustic_pressure;
        self.reflected_wave = reflected;
        self.last_arrived = transmitted;
        transmitted
    }

    pub fn delay_samples(&self) -> usize {
        self.delay.len()
    }

    #[inline]
    pub fn acoustic_pressure(&self) -> f32 {
        self.acoustic_pressure
    }

    #[inline]
    pub fn reflected_wave(&self) -> f32 {
        self.reflected_wave
    }

    #[inline]
    pub fn acoustic_state(&self) -> RunnerAcousticState {
        RunnerAcousticState {
            arrived: self.last_arrived,
            acoustic_pressure: self.acoustic_pressure,
            reflected: self.reflected_wave,
        }
    }
}

#[inline]
fn speed_of_sound_mps(temperature_k: f32) -> f32 {
    (GAMMA * SPECIFIC_GAS_CONSTANT_J_PER_KG_K * temperature_k).sqrt()
}

/// Output frame from a 5-in-1 collector junction.
#[derive(Clone, Copy, Debug, Default)]
pub struct CollectorFrame {
    pub pressure: f32,
    pub radiated: f32,
}

/// Matched 8th-order Butterworth low-pass cascade for zero-allocation local oversampling.
#[derive(Clone, Copy, Debug)]
struct ButterworthLp8 {
    stages: [Biquad; 4],
}

impl ButterworthLp8 {
    fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        let clamped_cutoff = cutoff_hz.clamp(10.0, sample_rate * 0.48);
        Self {
            stages: BUTTERWORTH_8_QS.map(|q| Biquad::lowpass(clamped_cutoff, q, sample_rate)),
        }
    }

    #[inline]
    fn process(&mut self, mut signal: f32) -> f32 {
        for stage in &mut self.stages {
            signal = stage.process(signal);
        }
        signal
    }
}

/// Lumped 5-in-1 exhaust collector chamber with modal resonator tailpipe radiation.
#[derive(Clone, Debug)]
pub struct Collector {
    body: ModalBank,
    dc: DcBlocker,
    chamber_pressure: f32,
    chamber_leak: f32,
    sub_inflow: f32,
    sub_leak: f32,
    upsample_lp: ButterworthLp8,
    junction_downsample_lp: ButterworthLp8,
    pressure_downsample_lp: ButterworthLp8,
    refined_dsp: bool,
}

impl Collector {
    fn build(body: ModalBank, sample_rate: f32, chamber_leak: f32) -> Self {
        let sub_leak =
            1.0 - (1.0 - chamber_leak.clamp(0.0, 0.999)).powf(1.0 / COLLECTOR_OVERSAMPLE as f32);
        let sub_inflow = if chamber_leak > 1.0e-6 {
            COLLECTOR_CHAMBER_INFLOW_COEFF * sub_leak / chamber_leak
        } else {
            COLLECTOR_CHAMBER_INFLOW_COEFF / COLLECTOR_OVERSAMPLE as f32
        };
        let os_rate = sample_rate * COLLECTOR_OVERSAMPLE as f32;
        let os_cutoff = sample_rate * 0.40;
        Self {
            body,
            dc: DcBlocker::new(18.0, sample_rate),
            chamber_pressure: 0.0,
            chamber_leak,
            sub_inflow,
            sub_leak,
            upsample_lp: ButterworthLp8::new(os_cutoff, os_rate),
            junction_downsample_lp: ButterworthLp8::new(os_cutoff, os_rate),
            pressure_downsample_lp: ButterworthLp8::new(os_cutoff, os_rate),
            refined_dsp: false,
        }
    }

    /// Legacy empirically tuned 5-in-1 collector bank.
    pub fn new(sample_rate: f32, bank_offset: f32) -> Self {
        let body = ModalBank::new(
            &[
                (164.0 + bank_offset, 0.022, 0.45),
                (328.0 + bank_offset * 1.7, 0.016, 0.60),
                (585.0 + bank_offset * 2.1, 0.008, 0.58),
                (820.0 + bank_offset * 2.4, 0.006, 0.85),
                (1_080.0 + bank_offset * 2.8, 0.005, 0.95),
                (1_390.0 + bank_offset * 2.9, 0.0042, 0.80),
                (1_720.0 + bank_offset * 3.0, 0.0036, 0.36),
                (2_080.0 + bank_offset * 3.0, 0.0030, 0.30),
                (3_250.0 + bank_offset * 4.0, 0.0020, 0.20),
                (5_200.0 + bank_offset * 5.0, 0.0012, 0.10),
            ],
            sample_rate,
        );
        let chamber_leak = 1.0 - (-1.0 / (0.0035 * sample_rate)).exp();
        Self::build(body, sample_rate, chamber_leak)
    }

    /// Collector derived from physical chamber volume and tailpipe dimensions.
    pub fn from_geometry(sample_rate: f32, bank_offset: f32, geometry: &CollectorGeometry) -> Self {
        let speed = geometry.wave_speed_mps();
        let _ = COLLECTOR_END_CORRECTION_RADIUS_FACTOR;
        let length = geometry.effective_outlet_length_m();
        let helmholtz = geometry.helmholtz_hz();
        let loss = geometry.loss_fraction_per_cycle;
        let radius = geometry.outlet_diameter_m * 0.5;
        let ka_one_hz = speed / (TAU * radius).max(1.0e-6);
        let mode_limit = COLLECTOR_MAX_MODE_HZ.min(sample_rate * 0.48);

        let mut frequencies = vec![helmholtz];
        let mut index = 1.0;
        loop {
            let harmonic = (2.0 * index - 1.0) * speed / (4.0 * length);
            if harmonic >= mode_limit || frequencies.len() >= COLLECTOR_MAX_MODES {
                break;
            }
            if (harmonic - helmholtz).abs() > helmholtz * 0.05 {
                frequencies.push(harmonic);
            }
            index += 1.0;
        }
        frequencies.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let spec: Vec<(f32, f32, f32)> = frequencies
            .iter()
            .map(|&frequency| {
                let frequency = (frequency + bank_offset).max(1.0);
                let decay = (2.0 / (loss * frequency)).clamp(0.0006, 1.5);
                let gain = geometry.mode_coupling / (1.0 + (frequency / ka_one_hz).powi(2));
                (frequency, decay, gain)
            })
            .collect();

        let decay_time = (2.0 / (loss * helmholtz)).clamp(0.0006, 1.5);
        let chamber_leak = 1.0 - (-1.0 / (decay_time * sample_rate)).exp();
        Self::build(
            ModalBank::new(&spec, sample_rate),
            sample_rate,
            chamber_leak,
        )
    }

    /// Enables local 4x oversampling around the nonlinear junction confluence and chamber saturation.
    pub fn with_refined_dsp(mut self, enabled: bool) -> Self {
        self.refined_dsp = enabled;
        self
    }

    /// Evaluates junction interference, chamber pressure, and tailpipe radiation for 5 runners.
    #[inline]
    pub fn process_bank(&mut self, runners: &[f32; 5]) -> CollectorFrame {
        let sum: f32 = runners.iter().sum();
        let (junction_input, pressure) = if self.refined_dsp {
            let mut j_out = 0.0;
            let mut p_out = 0.0;
            for step in 0..COLLECTOR_OVERSAMPLE {
                let raw_up = if step == 0 {
                    sum * COLLECTOR_OVERSAMPLE as f32
                } else {
                    0.0
                };
                let x_up = self.upsample_lp.process(raw_up);
                let j_up = x_up - COLLECTOR_CONFLUENCE_LOSS_COEFF * x_up * x_up.abs();
                self.chamber_pressure +=
                    self.sub_inflow * j_up - self.sub_leak * self.chamber_pressure;
                let p_up = self.chamber_pressure / (1.0 + 0.35 * self.chamber_pressure.abs());
                j_out = self.junction_downsample_lp.process(j_up);
                p_out = self.pressure_downsample_lp.process(p_up);
            }
            (j_out, p_out)
        } else {
            let nonlinear_loss = COLLECTOR_CONFLUENCE_LOSS_COEFF * sum * sum.abs();
            let junction_input = sum - nonlinear_loss;
            self.chamber_pressure += COLLECTOR_CHAMBER_INFLOW_COEFF * junction_input
                - self.chamber_leak * self.chamber_pressure;
            let pressure = self.chamber_pressure / (1.0 + 0.35 * self.chamber_pressure.abs());
            (junction_input, pressure)
        };
        let resonant = self.body.process(junction_input + pressure * 0.12);
        CollectorFrame {
            pressure,
            radiated: self
                .dc
                .process(junction_input * 0.62 + pressure * 0.18 + resonant),
        }
    }

    /// Convenience adapter distributing a pre-summed signal evenly across 5 runners.
    #[inline]
    pub fn process(&mut self, header_sum: f32) -> CollectorFrame {
        let runners = [header_sum * 0.2; 5];
        self.process_bank(&runners)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_arrival_matches_runner_geometry() {
        let mut runner = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, false);
        let expected = runner.delay_samples();
        let mut first = None;
        for sample in 0..expected + 4 {
            let y = runner.process(if sample == 0 { 1.0 } else { 0.0 }, 0.0);
            if y != 0.0 && first.is_none() {
                first = Some(sample);
            }
        }
        assert_eq!(first, Some(expected));
    }

    #[test]
    fn fixed_mode_ignores_temperature() {
        let mut a = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, false);
        let mut b = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, false);
        for s in 0..2_000 {
            let x = (s as f32 * 0.011).sin();
            assert_eq!(
                a.process(x, 300.0).to_bits(),
                b.process(x, 2_400.0).to_bits()
            );
        }
    }

    #[test]
    fn warmer_gas_arrives_sooner() {
        let mut hot = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, true);
        let mut cold = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, true);
        let mut hot_first = None;
        let mut cold_first = None;
        for s in 0..400 {
            let pulse = if s == 0 { 1.0 } else { 0.0 };
            let hy = hot.process(pulse, 2_000.0);
            let cy = cold.process(pulse, 350.0);
            if hy != 0.0 && hot_first.is_none() {
                hot_first = Some(s);
            }
            if cy != 0.0 && cold_first.is_none() {
                cold_first = Some(s);
            }
        }
        assert!(hot_first.unwrap() < cold_first.unwrap());
    }

    #[test]
    fn temperature_dependent_is_bit_deterministic() {
        let mut a = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, true);
        let mut b = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, true);
        for s in 0..2_000 {
            let x = (s as f32 * 0.013).sin();
            let t = 800.0 + 400.0 * (s as f32 * 0.002).sin();
            assert_eq!(a.process(x, t).to_bits(), b.process(x, t).to_bits());
        }
    }

    #[test]
    fn seam_interpolation_does_not_panic_across_delay_bounds() {
        let mut wg = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, true);
        for s in 0..10_000 {
            let t = 200.0 + 2000.0 * (s as f32 * 0.005).sin().abs();
            let out = wg.process((s as f32 * 0.05).sin(), t);
            assert!(out.is_finite());
        }
    }

    #[test]
    fn acoustic_state_tracks_pressure_and_reflected_decay() {
        let mut wg = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, true);
        wg.process(1.0, 1000.0);
        assert!((wg.acoustic_pressure() - 1.0).abs() < 1e-6);
        assert_eq!(wg.reflected_wave(), 0.0);

        let mut saw_reflection = false;
        for _ in 0..5_000 {
            wg.process(0.0, 1000.0);
            if wg.reflected_wave().abs() > 1e-4 {
                saw_reflection = true;
            }
        }
        assert!(saw_reflection);
        assert!(wg.acoustic_pressure().abs() < 1e-6);
    }

    #[test]
    fn collector_bank_interference_produces_finite_bounded_pressure() {
        let mut collector = Collector::new(48_000.0, 0.0);
        let pulses = [0.1, -0.05, 0.08, -0.02, 0.04];
        for _ in 0..1_000 {
            let frame = collector.process_bank(&pulses);
            assert!(frame.pressure.is_finite() && frame.radiated.is_finite());
        }
    }

    #[test]
    fn opposing_pulses_interfere_destructively_at_collector_junction() {
        let mut c1 = Collector::new(48_000.0, 0.0);
        let mut c2 = Collector::new(48_000.0, 0.0);
        let f1 = c1.process_bank(&[0.5, 0.5, 0.0, 0.0, 0.0]);
        let f2 = c2.process_bank(&[0.5, -0.5, 0.0, 0.0, 0.0]);
        assert!(f1.pressure > f2.pressure);
        assert!(f1.radiated.abs() > f2.radiated.abs());
    }

    #[test]
    fn geometric_collector_impulse_stays_finite_and_decays() {
        let geometry = CollectorGeometry {
            volume_l: 2.5,
            outlet_length_m: 0.35,
            outlet_diameter_m: 0.09,
            gas_temperature_k: 1_000.0,
            loss_fraction_per_cycle: 0.5,
            mode_coupling: 0.6,
        };
        let mut collector = Collector::from_geometry(48_000.0, -3.5, &geometry);
        let mut peak = 0.0f32;
        let tail_start = 48_000 * 2 - 4_800;
        let mut tail_peak = 0.0f32;
        for sample in 0..(48_000 * 2) {
            let frame =
                collector.process_bank(&[if sample == 0 { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0]);
            assert!(frame.pressure.is_finite() && frame.radiated.is_finite());
            peak = peak.max(frame.radiated.abs());
            if sample >= tail_start {
                tail_peak = tail_peak.max(frame.radiated.abs());
            }
        }
        assert!(peak > 0.0 && tail_peak < peak * 1.0e-3);
    }

    #[test]
    fn refined_waveguide_cubic_hermite_and_wall_loss_reduce_hf_harshness() {
        let mut legacy = RunnerWaveguide::new(0.72, 515.0, -0.38, 48_000.0, true);
        let mut refined =
            RunnerWaveguide::new(0.72, 515.0, -0.38, 48_000.0, true).with_refined_dsp(true);
        // Modulate runner temperature continuously with a high-frequency input pulse train
        let mut legacy_nyq_diff = 0.0f64;
        let mut refined_nyq_diff = 0.0f64;
        let mut prev_l = 0.0f32;
        let mut prev_r = 0.0f32;
        for n in 0..8_000 {
            let temp = 850.0 + 180.0 * (std::f32::consts::TAU * 6.0 * n as f32 / 48_000.0).sin();
            let exc = if n % 120 == 0 { 1.0 } else { 0.0 };
            let out_l = legacy.process(exc, temp);
            let out_r = refined.process(exc, temp);
            assert!(out_r.is_finite());
            if n > 1_000 {
                legacy_nyq_diff += ((out_l - prev_l) as f64).powi(2);
                refined_nyq_diff += ((out_r - prev_r) as f64).powi(2);
            }
            prev_l = out_l;
            prev_r = out_r;
        }
        assert!(refined_nyq_diff < legacy_nyq_diff * 0.75);
    }

    #[test]
    fn refined_collector_oversampling_suppresses_hf_intermodulation_and_preserves_dc() {
        let mut legacy = Collector::new(48_000.0, 0.0);
        let mut refined = Collector::new(48_000.0, 0.0).with_refined_dsp(true);
        // Feed a steady DC pulse first to check equilibrium chamber pressure parity
        let mut dc_l = 0.0f32;
        let mut dc_r = 0.0f32;
        for _ in 0..4_000 {
            dc_l = legacy.process_bank(&[0.2, 0.2, 0.2, 0.2, 0.2]).pressure;
            dc_r = refined.process_bank(&[0.2, 0.2, 0.2, 0.2, 0.2]).pressure;
        }
        assert!((dc_l - dc_r).abs() < 0.02, "DC parity {dc_l} vs {dc_r}");

        // Drive with two hot tones whose folded aliases land at 1.5 kHz (fs - 3*f1) and 2.0 kHz (2*f1 + f2 - fs)
        let mut leg_clean = Collector::new(48_000.0, 0.0);
        let mut ref_clean = Collector::new(48_000.0, 0.0).with_refined_dsp(true);
        let mut leg_c15 = 0.0f64;
        let mut leg_s15 = 0.0f64;
        let mut ref_c15 = 0.0f64;
        let mut ref_s15 = 0.0f64;
        let mut leg_c20 = 0.0f64;
        let mut leg_s20 = 0.0f64;
        let mut ref_c20 = 0.0f64;
        let mut ref_s20 = 0.0f64;
        for n in 0..16_000 {
            let phase1 = TAU * 15_500.0 * n as f32 / 48_000.0;
            let phase2 = TAU * 19_000.0 * n as f32 / 48_000.0;
            let drive = 1.6 * phase1.sin() + 1.6 * phase2.sin();
            let a = leg_clean
                .process_bank(&[drive, 0.0, 0.0, 0.0, 0.0])
                .radiated;
            let b = ref_clean
                .process_bank(&[drive, 0.0, 0.0, 0.0, 0.0])
                .radiated;
            assert!(b.is_finite());
            if n >= 4_000 {
                let (s15, c15) = (std::f64::consts::TAU * 1_500.0 * n as f64 / 48_000.0).sin_cos();
                let (s20, c20) = (std::f64::consts::TAU * 2_000.0 * n as f64 / 48_000.0).sin_cos();
                leg_c15 += a as f64 * c15;
                leg_s15 += a as f64 * s15;
                ref_c15 += b as f64 * c15;
                ref_s15 += b as f64 * s15;
                leg_c20 += a as f64 * c20;
                leg_s20 += a as f64 * s20;
                ref_c20 += b as f64 * c20;
                ref_s20 += b as f64 * s20;
            }
        }
        let leg_alias = (leg_c15 * leg_c15 + leg_s15 * leg_s15).sqrt()
            + (leg_c20 * leg_c20 + leg_s20 * leg_s20).sqrt();
        let ref_alias = (ref_c15 * ref_c15 + ref_s15 * ref_s15).sqrt()
            + (ref_c20 * ref_c20 + ref_s20 * ref_s20).sqrt();
        assert!(
            ref_alias < leg_alias * 0.15,
            "4x collector alias mag {ref_alias} not sharply below 1x {leg_alias}"
        );
    }
}
