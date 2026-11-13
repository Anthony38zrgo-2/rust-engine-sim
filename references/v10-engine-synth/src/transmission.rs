use crate::acoustics::BandPassNoise;
use crate::runtime::ShiftPhase;

const MAX_OUTPUT: f32 = 8.0;
const TAU: f32 = std::f32::consts::TAU;

#[derive(Clone, Debug)]
pub struct TransmissionConfig {
    pub sample_rate: u32,
    pub output_gain: f32,
    pub gear_ratios: Vec<f32>,
    pub reverse_ratio: f32,
    pub final_drive: f32,
    pub gear_teeth: f32,
    pub final_teeth: f32,
    pub whine_gain: f32,
    pub clack_gain: f32,
    pub rattle_gain: f32,
    pub clutch_gain: f32,
    pub shaft_slew_seconds: f32,
    pub seed: u64,
}

impl Default for TransmissionConfig {
    fn default() -> Self {
        Self {
            sample_rate: 44_100,
            output_gain: 0.0,
            gear_ratios: Vec::new(),
            reverse_ratio: 3.0,
            final_drive: 1.0,
            gear_teeth: 20.0,
            final_teeth: 40.0,
            whine_gain: 1.0,
            clack_gain: 1.0,
            rattle_gain: 1.0,
            clutch_gain: 1.0,
            shaft_slew_seconds: 0.12,
            seed: 0x5EED_7A11_5011_5EED,
        }
    }
}

impl TransmissionConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.sample_rate == 0 {
            return Err("transmission sample rate must be greater than zero".into());
        }
        if !self.output_gain.is_finite() || !(0.0..=2.0).contains(&self.output_gain) {
            return Err(format!("transmission output gain out of range: {}", self.output_gain));
        }
        for (name, gain) in [
            ("whine_gain", self.whine_gain),
            ("clack_gain", self.clack_gain),
            ("rattle_gain", self.rattle_gain),
            ("clutch_gain", self.clutch_gain),
        ] {
            if !gain.is_finite() || !(0.0..=4.0).contains(&gain) {
                return Err(format!("transmission {name} out of range: {gain}"));
            }
        }
        if !self.shaft_slew_seconds.is_finite() || self.shaft_slew_seconds <= 0.0 {
            return Err(format!("invalid shaft slew: {}", self.shaft_slew_seconds));
        }
        for ratio in &self.gear_ratios {
            if !ratio.is_finite() || *ratio <= 0.0 {
                return Err(format!("invalid gear ratio: {ratio}"));
            }
        }
        if !self.reverse_ratio.is_finite() || self.reverse_ratio <= 0.0 {
            return Err(format!("invalid reverse ratio: {}", self.reverse_ratio));
        }
        if !self.final_drive.is_finite() || self.final_drive <= 0.0 {
            return Err(format!("invalid final drive: {}", self.final_drive));
        }
        if !self.gear_teeth.is_finite() || self.gear_teeth <= 0.0 {
            return Err(format!("invalid gear teeth: {}", self.gear_teeth));
        }
        if !self.final_teeth.is_finite() || self.final_teeth <= 0.0 {
            return Err(format!("invalid final teeth: {}", self.final_teeth));
        }
        if self.output_gain > 0.0 && self.gear_ratios.is_empty() {
            return Err("gear ratios are required when the transmission is audible".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TransmissionInput {
    pub rpm: f32,
    pub gear: i8,
    pub clutch: f32,
    pub torque: f32,
    pub throttle: f32,
    pub shift_phase: ShiftPhase,
}

pub struct TransmissionSynth {
    config: TransmissionConfig,
    sample_rate: f32,
    rng: u64,
    shaft_hz: f32,
    gear_phase: f32,
    final_phase: f32,
    rpm_norm: f32,
    engagement: f32,
    last_cut: bool,
    shift_counter: u32,
    shift_recent_s: f32,
    clack_active: bool,
    clack_t: f32,
    clack_amp: f32,
    clack_tau: f32,
    clack_duration: f32,
    clack_scale: f32,
    clack_phase: [f32; 3],
    rattle_env: f32,
    rattle_filter: BandPassNoise,
    slip_filter: BandPassNoise,
    last_torque: f32,
    last_clutch: f32,
    thump_active: bool,
    thump_t: f32,
    thump_amp: f32,
}

impl TransmissionSynth {
    const CLACK_MODES: [(f32, f32); 3] = [(1480.0, 1.0), (2860.0, 0.62), (4310.0, 0.34)];

    pub fn new(config: TransmissionConfig) -> Result<Self, String> {
        config.validate()?;
        let sample_rate = config.sample_rate as f32;
        Ok(Self {
            config,
            sample_rate,
            rng: 0,
            shaft_hz: 0.0,
            gear_phase: 0.0,
            final_phase: 0.0,
            rpm_norm: 0.0,
            engagement: 0.0,
            last_cut: false,
            shift_counter: 0,
            shift_recent_s: 0.0,
            clack_active: false,
            clack_t: 0.0,
            clack_amp: 0.0,
            clack_tau: 0.006,
            clack_duration: 0.0,
            clack_scale: 1.0,
            clack_phase: [0.0; 3],
            rattle_env: 0.0,
            rattle_filter: BandPassNoise::new(1_500.0, 5_000.0, sample_rate),
            slip_filter: BandPassNoise::new(300.0, 1_500.0, sample_rate),
            last_torque: 0.0,
            last_clutch: 0.0,
            thump_active: false,
            thump_t: 0.0,
            thump_amp: 0.0,
        })
        .map(|mut synth| {
            synth.reset();
            synth
        })
    }

    pub fn reset(&mut self) {
        let sample_rate = self.sample_rate;
        self.rng = self.config.seed | 1;
        self.shaft_hz = 0.0;
        self.gear_phase = 0.0;
        self.final_phase = 0.0;
        self.rpm_norm = 0.0;
        self.engagement = 0.0;
        self.last_cut = false;
        self.shift_counter = 0;
        self.shift_recent_s = 0.0;
        self.clack_active = false;
        self.clack_t = 0.0;
        self.clack_amp = 0.0;
        self.rattle_env = 0.0;
        self.rattle_filter = BandPassNoise::new(1_500.0, 5_000.0, sample_rate);
        self.slip_filter = BandPassNoise::new(300.0, 1_500.0, sample_rate);
        self.last_torque = 0.0;
        self.last_clutch = 0.0;
        self.thump_active = false;
        self.thump_t = 0.0;
        self.thump_amp = 0.0;
    }

    pub fn output_gain(&self) -> f32 {
        self.config.output_gain
    }

    pub fn shaft_hz(&self) -> f32 {
        self.shaft_hz
    }

    pub fn gear_mesh_hz(&self) -> f32 {
        self.shaft_hz * self.config.gear_teeth
    }

    pub fn final_mesh_hz(&self) -> f32 {
        self.shaft_hz / self.config.final_drive * self.config.final_teeth
    }

    #[inline]
    pub fn process(&mut self, input: TransmissionInput) -> f32 {
        let dt = 1.0 / self.sample_rate;
        let ratio = self.gear_ratio(input.gear);
        let target_shaft = match ratio {
            Some(ratio) if input.rpm > 0.0 => (input.rpm / 60.0) / ratio,
            _ => 0.0,
        };
        let slew = 1.0 - (-dt / self.config.shaft_slew_seconds).exp();
        self.shaft_hz += slew * (target_shaft - self.shaft_hz);

        let rpm_target = ((input.rpm - 2_000.0) / 10_000.0).clamp(0.0, 1.0);
        let rpm_alpha = 1.0 - (-dt / 0.05).exp();
        self.rpm_norm += rpm_alpha * (rpm_target - self.rpm_norm);

        let engagement_target = match ratio {
            Some(_) => (0.25 + 0.75 * input.clutch.clamp(0.0, 1.0)).min(1.0),
            None => 0.0,
        };
        let engagement_alpha = 1.0 - (-dt / 0.06).exp();
        self.engagement += engagement_alpha * (engagement_target - self.engagement);

        let gear_mesh_hz = self.shaft_hz * self.config.gear_teeth;
        let final_mesh_hz =
            self.shaft_hz / self.config.final_drive * self.config.final_teeth;
        self.gear_phase = wrap_phase(self.gear_phase + TAU * gear_mesh_hz * dt);
        self.final_phase = wrap_phase(self.final_phase + TAU * final_mesh_hz * dt);
        let gear_tone = self.gear_phase.sin()
            + 0.45 * (2.0 * self.gear_phase).sin()
            + 0.22 * (3.0 * self.gear_phase).sin();
        let final_tone = self.final_phase.sin() + 0.5 * (2.0 * self.final_phase).sin();
        let load = 0.30 + 0.70 * input.torque.abs().min(1.0);
        let drive = 0.35 + 0.65 * self.rpm_norm;
        let whine = (gear_tone * 0.7 + final_tone * 0.5)
            * self.engagement
            * load
            * drive
            * self.config.whine_gain
            * 0.35;

        let phase = input.shift_phase as u8;
        let is_cut = phase == 1 || phase == 3;
        if is_cut && !self.last_cut {
            self.shift_counter = self.shift_counter.wrapping_add(1);
            self.shift_recent_s = 1.0;
            self.clack_active = true;
            self.clack_t = 0.0;
            let variation = self.next_unipolar();
            let intensity = (0.35 + 1.4 * self.last_torque.abs()).min(1.6);
            let direction = if phase == 3 { 1.15 } else { 1.0 };
            self.clack_amp = intensity * direction * self.config.clack_gain;
            self.clack_tau = 0.006 + 0.004 * variation;
            self.clack_duration = 0.045;
            self.clack_scale = 0.985 + 0.03 * variation;
            self.clack_phase = [
                0.0,
                TAU * self.next_unipolar(),
                TAU * self.next_unipolar(),
            ];
        }
        self.last_cut = is_cut;
        self.shift_recent_s = (self.shift_recent_s - dt).max(0.0);

        let mut clack = 0.0;
        if self.clack_active {
            self.clack_t += dt;
            if self.clack_t >= self.clack_duration {
                self.clack_active = false;
            } else {
                let envelope = (-self.clack_t / self.clack_tau).exp();
                let mut body = 0.0;
                for (index, (frequency, weight)) in Self::CLACK_MODES.iter().enumerate() {
                    body += weight
                        * (TAU * frequency * self.clack_scale * self.clack_t
                            + self.clack_phase[index])
                            .sin();
                }
                let click = self.next_noise() * (-self.clack_t / 0.0012).exp();
                clack = (body * envelope + click * 0.35) * self.clack_amp;
            }
        }

        let derivative = (input.torque - self.last_torque) / dt;
        let sign_flip = input.torque.signum() != self.last_torque.signum()
            && input.torque.abs().min(self.last_torque.abs()) < 0.35;
        if sign_flip || derivative.abs() > 6.0 {
            self.rattle_env = (self.rattle_env + 0.5).min(1.0);
        }
        self.rattle_env *= (-dt / 0.09).exp();
        let noise = self.next_noise();
        let rattle = self.rattle_filter.process(noise)
            * self.rattle_env
            * (0.25 + 0.75 * input.clutch.clamp(0.0, 1.0))
            * self.rpm_norm
            * self.config.rattle_gain
            * 0.6;

        let slip_level = 4.0 * input.clutch.clamp(0.0, 1.0) * (1.0 - input.clutch.clamp(0.0, 1.0));
        let slip = self.slip_filter.process(noise)
            * slip_level
            * (0.3 + 0.7 * self.rpm_norm)
            * input.throttle.clamp(0.0, 1.0)
            * self.config.clutch_gain
            * 0.5;

        if self.last_clutch < 0.97 && input.clutch >= 0.97 && self.shift_recent_s > 0.0 {
            self.thump_active = true;
            self.thump_t = 0.0;
            self.thump_amp = (0.2 + 0.8 * self.last_torque.abs()).min(1.0);
        }
        let mut thump = 0.0;
        if self.thump_active {
            self.thump_t += dt;
            if self.thump_t > 0.05 {
                self.thump_active = false;
            } else {
                let envelope = (-self.thump_t / 0.012).exp();
                thump = (TAU * 155.0 * self.thump_t).sin() * envelope * self.thump_amp;
            }
        }

        self.last_torque = input.torque;
        self.last_clutch = input.clutch;
        let mix = whine + clack + rattle + slip + thump * self.config.clutch_gain * 0.6;
        (mix * self.config.output_gain).clamp(-MAX_OUTPUT, MAX_OUTPUT)
    }

    fn gear_ratio(&self, gear: i8) -> Option<f32> {
        if gear > 0 {
            self.config.gear_ratios.get(gear as usize - 1).copied()
        } else if gear == -1 {
            Some(self.config.reverse_ratio)
        } else {
            None
        }
    }

    #[inline]
    fn next_noise(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng = x;
        let bits = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40;
        bits as f32 / ((1u64 << 24) - 1) as f32 * 2.0 - 1.0
    }

    #[inline]
    fn next_unipolar(&mut self) -> f32 {
        (self.next_noise() + 1.0) * 0.5
    }
}

#[inline]
fn wrap_phase(phase: f32) -> f32 {
    if phase >= TAU {
        phase - TAU
    } else if phase < 0.0 {
        phase + TAU
    } else {
        phase
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_RATE: u32 = 44_100;

    fn audible_config() -> TransmissionConfig {
        TransmissionConfig {
            output_gain: 1.0,
            gear_ratios: vec![3.0, 2.0, 1.5],
            final_drive: 3.0,
            ..TransmissionConfig::default()
        }
    }

    fn input(rpm: f32, gear: i8, clutch: f32, torque: f32, phase: ShiftPhase) -> TransmissionInput {
        TransmissionInput {
            rpm,
            gear,
            clutch,
            torque,
            throttle: 0.8,
            shift_phase: phase,
        }
    }

    #[test]
    fn silent_when_output_gain_is_zero() {
        let config = TransmissionConfig::default();
        let mut synth = TransmissionSynth::new(config).unwrap();
        let mut peak: f32 = 0.0;
        for index in 0..SAMPLE_RATE {
            let phase = if (10_000..10_500).contains(&index) {
                ShiftPhase::UpshiftCut
            } else {
                ShiftPhase::None
            };
            peak = peak.max(synth.process(input(9_000.0, 3, 1.0, 0.9, phase)).abs());
        }
        assert_eq!(peak, 0.0);
    }

    #[test]
    fn gear_mesh_follows_ratio_and_gear() {
        let mut synth = TransmissionSynth::new(audible_config()).unwrap();
        for _ in 0..SAMPLE_RATE {
            synth.process(input(9_000.0, 1, 1.0, 0.8, ShiftPhase::None));
        }
        let first = synth.gear_mesh_hz();
        assert!((900.0..1_100.0).contains(&first), "mesh {first} Hz");
        for _ in 0..SAMPLE_RATE {
            synth.process(input(9_000.0, 2, 1.0, 0.8, ShiftPhase::None));
        }
        let second = synth.gear_mesh_hz();
        assert!((1_400.0..1_600.0).contains(&second), "mesh {second} Hz");
    }

    #[test]
    fn stationary_or_neutral_has_no_whine_or_clack() {
        let mut synth = TransmissionSynth::new(audible_config()).unwrap();
        let mut peak: f32 = 0.0;
        for _ in 0..SAMPLE_RATE {
            peak = peak.max(synth.process(input(0.0, 0, 0.0, 0.0, ShiftPhase::None)).abs());
        }
        assert!(peak < 1e-4, "stationary peak {peak}");
    }

    #[test]
    fn clack_fires_once_per_cut_edge() {
        let mut synth = TransmissionSynth::new(audible_config()).unwrap();
        let mut onset_peak: f32 = 0.0;
        let mut tail_peak: f32 = 0.0;
        for index in 0..20_000 {
            let phase = if index >= 1_000 {
                ShiftPhase::UpshiftCut
            } else {
                ShiftPhase::None
            };
            let value = synth.process(input(9_000.0, 3, 1.0, 0.9, phase)).abs();
            if (1_000..1_300).contains(&index) {
                onset_peak = onset_peak.max(value);
            }
            if (4_000..4_300).contains(&index) {
                tail_peak = tail_peak.max(value);
            }
        }
        assert!(onset_peak > 0.02, "onset peak {onset_peak}");
        assert!(tail_peak < onset_peak * 0.5, "tail {tail_peak} vs onset {onset_peak}");
    }

    #[test]
    fn process_is_bit_deterministic() {
        let mut a = TransmissionSynth::new(audible_config()).unwrap();
        let mut b = TransmissionSynth::new(audible_config()).unwrap();
        for index in 0..8_192 {
            let phase = match index % 2_048 {
                600..=900 => ShiftPhase::DownshiftCut,
                1_500..=1_700 => ShiftPhase::UpshiftRecovery,
                _ => ShiftPhase::None,
            };
            let gear = if index % 4_096 < 2_048 { 2 } else { 1 };
            let source = input(8_000.0 + index as f32, gear, 0.8, 0.5, phase);
            assert_eq!(a.process(source), b.process(source));
        }
    }

    #[test]
    fn output_stays_bounded_on_extreme_inputs() {
        let mut synth = TransmissionSynth::new(audible_config()).unwrap();
        for index in 0..20_000 {
            let phase = if index % 500 < 50 {
                ShiftPhase::DownshiftCut
            } else if index % 1_000 < 150 {
                ShiftPhase::DownshiftBlip
            } else {
                ShiftPhase::None
            };
            let value = synth.process(TransmissionInput {
                rpm: 18_000.0,
                gear: 1,
                clutch: (index % 100) as f32 / 100.0,
                torque: if index % 200 < 100 { 1.0 } else { -1.0 },
                throttle: 1.0,
                shift_phase: phase,
            });
            assert!(value.is_finite());
            assert!(value.abs() <= MAX_OUTPUT);
        }
    }

    #[test]
    fn audible_transmission_requires_gear_ratios() {
        let config = TransmissionConfig {
            output_gain: 1.0,
            gear_ratios: Vec::new(),
            ..TransmissionConfig::default()
        };
        assert!(TransmissionSynth::new(config).is_err());
    }
}
