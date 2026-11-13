//! Acoustic exhaust-runner waveguide.
//!
//! The single header pipe that runs from a cylinder's exhaust valve to the
//! collector is modelled as a delay line with reflection and loss. It preserves
//! the behaviour captured by the original [`crate::acoustics`] `Header` while
//! living behind the exhaust naming so it can evolve into the stateful runner
//! of PHY-062. PHY-061 makes the propagation temperature-aware: the wave speed
//! is derived from the runner gas temperature (`sqrt(gamma * R * T)`) so the
//! pipe's resonant pitch tracks the real exhaust-gas state, with the fixed
//! 545 m/s path retained as the compatibility mode.

use crate::thermodynamics::exhaust_runner::{GAMMA, SPECIFIC_GAS_CONSTANT_J_PER_KG_K};

/// Coldest runner gas temperature considered when sizing the delay buffer (K).
/// Matches the `GasState` validation floor so the capacity always outlives the
/// physical delay (a colder gas is a slower pipe and therefore a longer delay).
const MIN_RUNNER_TEMPERATURE_K: f32 = 200.0;

/// Viscous and thermal wall boundary-layer acoustic attenuation coefficient per runner transit.
pub const RUNNER_WALL_LOSS_COEFF: f32 = 0.32;

/// Instantaneous acoustic state of the runner waveguide.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RunnerAcousticState {
    /// Forward wave arriving at the collector junction (radiated travelling wave).
    pub arrived: f32,
    /// Superposition acoustic pressure perturbation inside the runner entry.
    pub acoustic_pressure: f32,
    /// Low-pass filtered reflected wave returning upstream toward the valve.
    pub reflected: f32,
}

/// A one-dimensional wave delay line representing a single exhaust runner
/// (primary header pipe).
///
/// - **length** → the physical pipe length (m),
/// - **wave delay** → the round-trip time through the pipe, from `length_m` and the
///   wave speed (fixed `wave_speed_mps`, or `sqrt(gamma*R*T)` when temperature-aware),
/// - **reflection** → mixing of the arriving travelling wave back into the outgoing
///   wave (boundary reflection coefficient),
/// - **loss** → one-pole low-pass on the reflected feedback, a frequency-dependent
///   attenuation that symbolises viscous/thermal wall loss.
///
/// In temperature-aware mode the delay is a fractional number of samples that
/// drifts slowly with the runner temperature, so it is read with linear
/// interpolation from a circular buffer sized for the coldest state. In fixed
/// mode (`temperature_dependent == false`) the buffer is exactly the integer
/// delay length and the original behaviour is reproduced bit-for-bit.
pub struct RunnerWaveguide {
    delay: Vec<f32>,
    cursor: usize,
    reflection: f32,
    feedback_lowpass: f32,
    length_m: f32,
    sample_rate: f32,
    temperature_dependent: bool,
    acoustic_pressure: f32,
    reflected_wave: f32,
    last_arrived: f32,
}

impl RunnerWaveguide {
    pub fn new(
        length_m: f32,
        wave_speed_mps: f32,
        reflection: f32,
        sample_rate: f32,
        temperature_dependent: bool,
    ) -> Self {
        let fixed_delay_samples = (length_m / wave_speed_mps * sample_rate).round().max(2.0) as usize;
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
            length_m,
            sample_rate,
            temperature_dependent,
            acoustic_pressure: 0.0,
            reflected_wave: 0.0,
            last_arrived: 0.0,
        }
    }

    #[inline]
    pub fn process(&mut self, input: f32, temperature_k: f32) -> f32 {
        if !self.temperature_dependent {
            return self.process_fixed(input);
        }
        // Floor the temperature at the buffer-sizing floor so the physical delay
        // never exceeds the capacity that `new` reserved for the coldest gas.
        let speed = speed_of_sound_mps(temperature_k.max(MIN_RUNNER_TEMPERATURE_K));
        let max_delay = (self.delay.len() - 1) as f32;
        let delay_float = (self.length_m / speed * self.sample_rate).clamp(2.0, max_delay);
        // Fractional read position on the circular buffer. A tiny negative
        // `cursor - delay_float` wraps to the buffer tail, but `raw + len` can
        // round to exactly `len` at fp32 precision, so we guard it back under
        // `len`; `i1` wraps to index 0 so the interpolation crosses the seam.
        let n = self.delay.len() as f32;
        let mut read = self.cursor as f32 - delay_float;
        if read < 0.0 {
            read += n;
        }
        if read >= n {
            read -= n;
        }
        let i0 = read.floor() as usize;
        let i1 = (i0 + 1) % self.delay.len();
        let frac = read - read.floor();
        let arrived = self.delay[i0] + frac * (self.delay[i1] - self.delay[i0]);
        self.feedback_lowpass += RUNNER_WALL_LOSS_COEFF * (arrived - self.feedback_lowpass);
        let reflected = self.reflection * self.feedback_lowpass;
        let acoustic_pressure = input + reflected;
        self.delay[self.cursor] = acoustic_pressure;
        self.cursor = (self.cursor + 1) % self.delay.len();
        self.acoustic_pressure = acoustic_pressure;
        self.reflected_wave = reflected;
        self.last_arrived = arrived;
        arrived
    }

    #[inline]
    fn process_fixed(&mut self, input: f32) -> f32 {
        let arrived = self.delay[self.cursor];
        self.feedback_lowpass += RUNNER_WALL_LOSS_COEFF * (arrived - self.feedback_lowpass);
        let reflected = self.reflection * self.feedback_lowpass;
        let acoustic_pressure = input + reflected;
        self.delay[self.cursor] = acoustic_pressure;
        self.cursor += 1;
        if self.cursor == self.delay.len() {
            self.cursor = 0;
        }
        self.acoustic_pressure = acoustic_pressure;
        self.reflected_wave = reflected;
        self.last_arrived = arrived;
        arrived
    }

    pub fn delay_samples(&self) -> usize {
        self.delay.len()
    }

    /// Superposition acoustic pressure perturbation inside the runner entry (Pa proxy).
    #[inline]
    pub fn acoustic_pressure(&self) -> f32 {
        self.acoustic_pressure
    }

    /// Low-pass filtered wave reflected from downstream boundary traveling back upstream.
    #[inline]
    pub fn reflected_wave(&self) -> f32 {
        self.reflected_wave
    }

    /// Complete instantaneous acoustic state of the runner waveguide.
    #[inline]
    pub fn acoustic_state(&self) -> RunnerAcousticState {
        RunnerAcousticState {
            arrived: self.last_arrived,
            acoustic_pressure: self.acoustic_pressure,
            reflected: self.reflected_wave,
        }
    }
}

/// Speed of sound in the exhaust gas at the given temperature (m/s).
#[inline]
fn speed_of_sound_mps(temperature_k: f32) -> f32 {
    (GAMMA * SPECIFIC_GAS_CONSTANT_J_PER_KG_K * temperature_k).sqrt()
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
        assert!(
            hot_first.unwrap() < cold_first.unwrap(),
            "hot {hot_first:?} must arrive before cold {cold_first:?}"
        );
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
        // Sweep temperature widely across boundaries and run through buffer seam crossings
        for s in 0..10_000 {
            let t = 200.0 + 2000.0 * (s as f32 * 0.005).sin().abs();
            let out = wg.process((s as f32 * 0.05).sin(), t);
            assert!(out.is_finite());
        }
    }

    #[test]
    fn acoustic_state_tracks_pressure_and_reflected_decay() {
        let mut wg = RunnerWaveguide::new(0.545, 545.0, -0.3, 48_000.0, true);
        // Step a unit pulse
        wg.process(1.0, 1000.0);
        assert!((wg.acoustic_pressure() - 1.0).abs() < 1e-6);
        assert_eq!(wg.reflected_wave(), 0.0);

        // Run until pulse round-trips and reflections decay
        let mut saw_reflection = false;
        for _ in 0..5_000 {
            wg.process(0.0, 1000.0);
            if wg.reflected_wave().abs() > 1e-4 {
                saw_reflection = true;
            }
        }
        assert!(saw_reflection, "must observe acoustic reflection");
        assert!(
            wg.acoustic_pressure().abs() < 1e-6,
            "acoustic pressure must decay to tail"
        );
    }
}
