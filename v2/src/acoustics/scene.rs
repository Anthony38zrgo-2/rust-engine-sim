//! Multi-path onboard acoustic scene combining airborne, engine-cover, and chassis paths.

use crate::acoustics::cover::EngineCover;
use crate::acoustics::mounts::EngineMountMonocoque;
use crate::acoustics::paths::{AirPath, CylinderMechanicalPath};
use crate::config::AcousticSceneConfig;
use crate::dsp::{Biquad, DcBlocker, OnePoleLowPass};
use crate::physics::CYLINDER_COUNT;
use crate::synthesis::EngineFrame;

const AIR_TILT_HZ: f32 = 2_500.0;

/// Per-sample acoustic scene branches and final onboard mix output.
#[derive(Clone, Copy, Debug, Default)]
pub struct AcousticFrame {
    pub engine_dry: f32,
    pub engine_air: f32,
    pub engine_cover: f32,
    pub engine_mounts: f32,
    pub monocoque_seat: f32,
    pub mount_monocoque: f32,
    pub cylinder_mechanical: [f32; CYLINDER_COUNT],
    pub cylinder_mechanical_sum: f32,
    pub output: f32,
}

/// Complete vehicle cockpit acoustic propagation scene.
#[derive(Clone, Debug)]
pub struct AcousticScene {
    config: AcousticSceneConfig,
    engine_cover: EngineCover,
    mount_monocoque: EngineMountMonocoque,
    cylinder_paths: [CylinderMechanicalPath; CYLINDER_COUNT],
    dry_lowpass: OnePoleLowPass,
    dry_midpass: OnePoleLowPass,
    air_path: AirPath,
    air_tilt: Biquad,
    air_direct_highpass: Biquad,
    onboard_highpass: DcBlocker,
    sample_rate: f32,
    previous_crank_phase: f32,
    firing_frequency_hz: f32,
}

impl AcousticScene {
    /// Validates `config` and preallocates all scene branches for `sample_rate`.
    pub fn new(sample_rate: f32, config: AcousticSceneConfig) -> Result<Self, String> {
        config.validate()?;
        Ok(Self {
            config,
            engine_cover: EngineCover::with_profile(
                sample_rate,
                config.cover_radiation_lowpass_hz,
                config.structural_profile,
            )
            .with_refined_dsp(config.use_refined_dsp),
            mount_monocoque: EngineMountMonocoque::with_profile(
                sample_rate,
                config.structural_profile,
            ),
            cylinder_paths: std::array::from_fn(|index| {
                CylinderMechanicalPath::with_layout(index, sample_rate, config.bank_split)
                    .with_refined_dsp(config.use_refined_dsp)
            }),
            dry_lowpass: OnePoleLowPass::new(360.0, sample_rate),
            dry_midpass: OnePoleLowPass::new(2_650.0, sample_rate),
            air_path: AirPath::new(sample_rate).with_refined_dsp(config.use_refined_dsp),
            air_tilt: Biquad::high_shelf(
                AIR_TILT_HZ,
                std::f32::consts::FRAC_1_SQRT_2,
                config.air_high_tilt_db,
                sample_rate,
            ),
            air_direct_highpass: Biquad::highpass(
                AIR_TILT_HZ,
                std::f32::consts::FRAC_1_SQRT_2,
                sample_rate,
            ),
            onboard_highpass: DcBlocker::new(75.0, sample_rate),
            sample_rate,
            previous_crank_phase: 0.0,
            firing_frequency_hz: 0.0,
        })
    }

    /// Processes one [`EngineFrame`] through airborne, cover, and mount/monocoque paths.
    #[inline]
    pub fn process(&mut self, engine: &EngineFrame) -> AcousticFrame {
        let crank_delta = (engine.crank_phase_deg - self.previous_crank_phase).rem_euclid(720.0);
        self.previous_crank_phase = engine.crank_phase_deg;
        let events_per_rev = self.config.cylinder_count as f32 * 0.5;
        let measured_firing = crank_delta * self.sample_rate / 360.0 * events_per_rev;
        if measured_firing.is_finite() && measured_firing < self.sample_rate * 0.45 {
            self.firing_frequency_hz += 0.04 * (measured_firing - self.firing_frequency_hz);
        }
        let rpm = self.firing_frequency_hz * 60.0 / events_per_rev;
        let high_rpm =
            ((rpm - self.config.high_rpm_start) / self.config.high_rpm_span).clamp(0.0, 1.0);
        let mut cylinder_mechanical = [0.0; CYLINDER_COUNT];
        for (index, path) in self.cylinder_paths[..self.config.cylinder_count]
            .iter_mut()
            .enumerate()
        {
            cylinder_mechanical[index] = if self.config.use_refined_dsp {
                path.process_with_load(
                    engine.cylinder_pressure_derivative[index],
                    engine.cylinder_blowdown[index],
                    engine.cylinder_headers[index],
                    engine.load,
                )
            } else {
                path.process(
                    engine.cylinder_pressure_derivative[index],
                    engine.cylinder_blowdown[index],
                    engine.cylinder_headers[index],
                )
            };
        }
        let mech_scale = if self.config.cylinder_count == CYLINDER_COUNT {
            0.34
        } else {
            0.34 * (CYLINDER_COUNT as f32 / self.config.cylinder_count as f32).sqrt()
        };
        let cylinder_mechanical_sum = cylinder_mechanical.iter().sum::<f32>() * mech_scale;
        let structure_send =
            cylinder_mechanical_sum + engine.transmission * self.config.transmission_mount_gain;
        let transmission_cover_send = engine.transmission * self.config.transmission_cover_gain;
        let engine_cover = self
            .engine_cover
            .process(engine, high_rpm, transmission_cover_send);
        let (engine_mounts, monocoque_seat) = self.mount_monocoque.process(engine, structure_send);
        let mount_monocoque = engine_mounts * 0.58 + monocoque_seat;

        let master = engine.master + engine.transmission;
        let dry_low = self.dry_lowpass.process(master);
        let below_mid = self.dry_midpass.process(master);
        let dry_mid = below_mid - dry_low;
        let dry_high = master - below_mid;
        let filtered_dry = dry_low * self.config.dry_low_gain
            + dry_mid * self.config.dry_mid_gain
            + dry_high * self.config.dry_high_gain * (1.0 - 0.72 * high_rpm);
        let air_input = self.air_tilt.process(filtered_dry);
        let air_direct = self.air_direct_highpass.process(air_input) * self.config.air_direct_gain;
        let engine_air = self.air_path.process(air_input) + air_direct;
        AcousticFrame {
            engine_dry: master,
            engine_air,
            engine_cover,
            engine_mounts,
            monocoque_seat,
            mount_monocoque,
            cylinder_mechanical,
            cylinder_mechanical_sum,
            output: self.onboard_highpass.process(
                (engine_air * self.config.engine_air_gain
                    + engine_cover * self.config.engine_cover_gain
                    + mount_monocoque * self.config.mount_monocoque_gain
                    + engine.gear_shift)
                    * self.config.output_gain,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn silent_engine_stays_silent() {
        let mut scene = AcousticScene::new(48_000.0, AcousticSceneConfig::default()).unwrap();
        for _ in 0..96_000 {
            let frame = scene.process(&EngineFrame::default());
            assert_eq!(frame.engine_air, 0.0);
            assert_eq!(frame.engine_cover, 0.0);
            assert_eq!(frame.mount_monocoque, 0.0);
            assert_eq!(frame.output, 0.0);
        }
    }

    #[test]
    fn filtered_air_path_is_audible() {
        let mut scene = AcousticScene::new(
            48_000.0,
            AcousticSceneConfig {
                dry_low_gain: 1.0,
                dry_mid_gain: 1.0,
                dry_high_gain: 1.0,
                engine_air_gain: 1.0,
                engine_cover_gain: 0.0,
                mount_monocoque_gain: 0.0,
                output_gain: 1.0,
                ..AcousticSceneConfig::default()
            },
        )
        .unwrap();
        let source = EngineFrame {
            master: 0.375,
            ..EngineFrame::default()
        };
        let mut heard = false;
        for _ in 0..400 {
            heard |= scene.process(&source).output != 0.0;
        }
        assert!(heard);
    }

    #[test]
    fn transmission_mount_send_reaches_the_monocoque_path() {
        let config = |gain: f32| AcousticSceneConfig {
            engine_air_gain: 0.0,
            engine_cover_gain: 0.0,
            mount_monocoque_gain: 1.0,
            transmission_mount_gain: gain,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let source = EngineFrame {
            transmission: 0.8,
            ..EngineFrame::default()
        };
        let mut reference = AcousticScene::new(48_000.0, config(0.0)).unwrap();
        let mut candidate = AcousticScene::new(48_000.0, config(0.5)).unwrap();
        let mut reference_peak = 0.0f32;
        let mut candidate_peak = 0.0f32;
        for _ in 0..96_000 {
            reference_peak = reference_peak.max(reference.process(&source).output.abs());
            candidate_peak = candidate_peak.max(candidate.process(&source).output.abs());
        }
        assert_eq!(reference_peak, 0.0);
        assert!(candidate_peak > 1.0e-4);
    }

    #[test]
    fn transmission_cover_send_reaches_the_cover_branch() {
        let config = |gain: f32| AcousticSceneConfig {
            engine_air_gain: 0.0,
            mount_monocoque_gain: 0.0,
            engine_cover_gain: 1.0,
            transmission_cover_gain: gain,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let source = EngineFrame {
            transmission: 0.8,
            ..EngineFrame::default()
        };
        let mut reference = AcousticScene::new(48_000.0, config(0.0)).unwrap();
        let mut candidate = AcousticScene::new(48_000.0, config(0.5)).unwrap();
        let mut reference_peak = 0.0f32;
        let mut candidate_peak = 0.0f32;
        for _ in 0..4_800 {
            reference_peak = reference_peak.max(reference.process(&source).output.abs());
            candidate_peak = candidate_peak.max(candidate.process(&source).output.abs());
        }
        assert_eq!(reference_peak, 0.0);
        assert!(candidate_peak > 1.0e-4);
    }

    #[test]
    fn air_tilt_and_direct_add_high_frequency_detail() {
        let config = |tilt: f32, direct: f32| AcousticSceneConfig {
            engine_cover_gain: 0.0,
            mount_monocoque_gain: 0.0,
            transmission_cover_gain: 0.0,
            engine_air_gain: 1.0,
            air_high_tilt_db: tilt,
            air_direct_gain: direct,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let render = |tilt: f32, direct: f32| {
            let mut scene = AcousticScene::new(48_000.0, config(tilt, direct)).unwrap();
            let mut state = 0x1234_5678u32;
            let mut sum = 0.0f64;
            for _ in 0..96_000 {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let noise = (state as f32 / u32::MAX as f32) * 2.0 - 1.0;
                let frame = EngineFrame {
                    master: 0.3 * noise,
                    ..EngineFrame::default()
                };
                let output = scene.process(&frame).engine_air;
                sum += (output as f64) * (output as f64);
            }
            (sum / 96_000.0).sqrt()
        };
        let reference = render(0.0, 0.0);
        let tilted = render(6.0, 0.0);
        let direct = render(0.0, 0.5);
        assert!(reference > 0.0);
        assert!(tilted > reference * 1.02);
        assert!(direct > reference * 1.02);
    }

    #[test]
    fn engine_air_gain_scales_only_the_air_branch() {
        let base = AcousticSceneConfig {
            engine_cover_gain: 0.0,
            mount_monocoque_gain: 0.0,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let mut reference = AcousticScene::new(48_000.0, base).unwrap();
        let mut louder = AcousticScene::new(
            48_000.0,
            AcousticSceneConfig {
                engine_air_gain: 1.5,
                ..base
            },
        )
        .unwrap();
        let source = EngineFrame {
            master: 0.25,
            ..EngineFrame::default()
        };
        let mut reference_air = 0.0;
        let mut reference_out = 0.0;
        let mut louder_out = 0.0;
        for _ in 0..4_800 {
            let reference_frame = reference.process(&source);
            let louder_frame = louder.process(&source);
            reference_air = reference_frame.engine_air;
            reference_out = reference_frame.output;
            louder_out = louder_frame.output;
        }
        assert!(reference_air.abs() > 1.0e-6);
        assert!((louder_out / reference_out - 1.5).abs() < 1.0e-3);
    }
}
