const GAMMA: f32 = 1.4;
const SPECIFIC_GAS_CONSTANT_J_PER_KG_K: f32 = 8.314 / 0.0289;
const MIN_RUNNER_TEMPERATURE_K: f32 = 200.0;
pub const RUNNER_WALL_LOSS_COEFF: f32 = 0.32;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RunnerAcousticState {
    pub arrived: f32,
    pub acoustic_pressure: f32,
    pub reflected: f32,
}

#[derive(Clone, Debug)]
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
        let sr = sample_rate.max(1.0);
        let length = length_m.max(0.001);
        let speed = wave_speed_mps.clamp(100.0, 1_500.0);
        let fixed_delay_samples = (length / speed * sr).round().max(2.0) as usize;
        let buffer_len = if temperature_dependent {
            let min_speed = speed_of_sound_mps(MIN_RUNNER_TEMPERATURE_K);
            (length / min_speed * sr).ceil() as usize + 2
        } else {
            fixed_delay_samples
        };
        Self {
            delay: vec![0.0; buffer_len.max(fixed_delay_samples)],
            cursor: 0,
            reflection,
            feedback_lowpass: 0.0,
            length_m: length,
            sample_rate: sr,
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
        let speed = speed_of_sound_mps(temperature_k.max(MIN_RUNNER_TEMPERATURE_K));
        let max_delay = (self.delay.len() - 1) as f32;
        let delay_float = (self.length_m / speed * self.sample_rate).clamp(2.0, max_delay);
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
pub fn speed_of_sound_mps(temperature_k: f32) -> f32 {
    (GAMMA * SPECIFIC_GAS_CONSTANT_J_PER_KG_K * temperature_k.max(1.0)).sqrt()
}

pub fn default_header_reflection() -> f32 {
    -0.34
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
}
