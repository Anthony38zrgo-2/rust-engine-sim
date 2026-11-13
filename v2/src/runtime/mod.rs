//! Allocation-free real-time runtime facade (`Gf509Runtime`), source dispatch,
//! and audio-bus mixing.
//!
//! Asset loading and reset happen strictly on the control thread. `render_block`
//! performs no heap allocation, file I/O, or logging.

pub mod mixing;
pub mod renderer;
pub mod source;

pub use crate::assets::GF509_HEADROOM_GAIN;
pub use crate::config::{EngineMode, Gf509RuntimeConfig};
pub use crate::control::{ControlInterpolator, RuntimeTelemetry, ShiftPhase, TorqueSign};
pub use mixing::blend_hybrid;
pub use renderer::{AudioRenderer, RenderTrace};
pub use source::{EngineSource, EngineStepTrace};

/// Real-time continuous GF509 engine, gearbox, and acoustic scene runtime facade.
pub struct Gf509Runtime {
    config: Gf509RuntimeConfig,
    renderer: AudioRenderer,
}

impl Gf509Runtime {
    /// Validates `config`, prepares sample banks and gear-shift assets on the
    /// control thread, and initializes all DSP state for allocation-free rendering.
    pub fn new(config: Gf509RuntimeConfig) -> Result<Self, String> {
        let renderer = AudioRenderer::new(&config)?;
        Ok(Self { config, renderer })
    }

    /// Configured output sample rate in Hz.
    pub fn sample_rate(&self) -> u32 {
        self.renderer.sample_rate()
    }

    /// Configured continuous-engine source selector (`Hybrid` or `SampleOnly`).
    pub fn engine_mode(&self) -> EngineMode {
        self.config.engine_mode
    }

    /// Maximum supported block size in `render_block`.
    pub fn max_block_frames(&self) -> usize {
        self.renderer.max_block_frames()
    }

    /// Latest validated target telemetry packet.
    pub fn telemetry(&self) -> RuntimeTelemetry {
        self.renderer.interpolator().target()
    }

    /// Most recently rendered per-sample interpolated telemetry snapshot.
    pub fn rendered_telemetry(&self) -> RuntimeTelemetry {
        self.renderer.interpolator().rendered()
    }

    /// Direct read-only access to the underlying [`AudioRenderer`] and interpolator.
    pub fn renderer(&self) -> &AudioRenderer {
        &self.renderer
    }

    /// Validates `telemetry` and computes constant control-rate linear steps.
    pub fn update_telemetry(&mut self, telemetry: RuntimeTelemetry) -> Result<(), String> {
        self.renderer.update_telemetry(telemetry)
    }

    /// Renders equal-length planar stereo buffers and returns the frame count.
    /// GF509 is currently a centered continuous source, so both channels match.
    pub fn render_block(&mut self, left: &mut [f32], right: &mut [f32]) -> Result<usize, String> {
        self.renderer.render_block(left, right)
    }

    /// Restores deterministic post-construction state while retaining `config`.
    /// May reload assets and must not be called from the audio callback.
    pub fn reset(&mut self) -> Result<(), String> {
        let replacement = Self::new(self.config.clone())?;
        *self = replacement;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::{Path, PathBuf};

    fn fixture_bank(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(name)
    }

    fn running_runtime(max_block_frames: usize) -> Gf509Runtime {
        let config = Gf509RuntimeConfig {
            max_block_frames,
            ..Gf509RuntimeConfig::default()
        };
        let mut runtime = Gf509Runtime::new(config).unwrap();
        runtime
            .update_telemetry(RuntimeTelemetry {
                rpm: 7_499.0,
                throttle: 0.92,
                normalized_engine_load: 0.88,
                normalized_engine_torque: 0.75,
                torque_sign: TorqueSign::Positive,
                rpm_derivative: 1200.0,
                throttle_derivative: 0.5,
                gear: 3,
                shift_phase: ShiftPhase::None,
                clutch_engagement: 1.0,
                tc_cut_ratio: 0.0,
                rev_limiter_active: false,
                dt_seconds: 1.0 / 120.0,
            })
            .unwrap();
        runtime
    }

    fn sampled_telemetry(rpm: f32) -> RuntimeTelemetry {
        RuntimeTelemetry {
            rpm,
            throttle: 0.85,
            normalized_engine_load: 0.8,
            normalized_engine_torque: 0.7,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: 0.0,
            throttle_derivative: 0.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: 1.0 / 120.0,
        }
    }

    fn render_sampled_output(config_mutator: impl FnOnce(&mut Gf509RuntimeConfig)) -> Vec<f32> {
        let mut config = Gf509RuntimeConfig::default();
        config.engine.sample_rate = 48_000;
        config.sample_layer_directory = Some(fixture_bank("bank_schema2"));
        config.engine_mode = EngineMode::SampleOnly;
        config.transmission.sample_rate = 48_000;
        config.transmission.gear_ratios = vec![3.45, 2.75, 2.3, 1.95, 1.68, 1.46];
        config.transmission.final_drive = 4.25;
        config_mutator(&mut config);
        let mut runtime = Gf509Runtime::new(config).unwrap();
        runtime
            .update_telemetry(sampled_telemetry(9_000.0))
            .unwrap();
        let mut left = [0.0f32; 512];
        let mut right = [0.0f32; 512];
        runtime.render_block(&mut left, &mut right).unwrap();
        left.to_vec()
    }

    fn render_average_rms(
        runtime: &mut Gf509Runtime,
        left: &mut [f32; 512],
        right: &mut [f32; 512],
        blocks: usize,
    ) -> f32 {
        let mut energy = 0.0;
        for _ in 0..blocks {
            runtime.render_block(left, right).unwrap();
            energy += left.iter().map(|sample| sample * sample).sum::<f32>();
        }
        (energy / (blocks * left.len()) as f32).sqrt()
    }

    #[test]
    fn transmission_layer_is_summed_into_the_hybrid_mix() {
        let render = |gain: f32| {
            let mut config = Gf509RuntimeConfig::default();
            config.transmission.output_gain = gain;
            config.transmission.gear_ratios = vec![3.45, 2.75, 2.3, 1.95, 1.68, 1.46];
            config.transmission.final_drive = 4.25;
            let mut runtime = Gf509Runtime::new(config).unwrap();
            runtime
                .update_telemetry(sampled_telemetry(9_000.0))
                .unwrap();
            let mut left = vec![0.0; 4_096];
            let mut right = vec![0.0; 4_096];
            runtime.render_block(&mut left, &mut right).unwrap();
            left
        };
        let silent = render(0.0);
        let audible = render(0.9);
        assert!(
            silent
                .iter()
                .zip(audible.iter())
                .any(|(off, on)| (off - on).abs() > 1e-4),
            "transmission must reach the scene output"
        );
    }

    #[test]
    fn control_rate_steps_converge_without_zipper() {
        let mut runtime = running_runtime(1024);
        let mut settle_l = [0.0; 512];
        let mut settle_r = [0.0; 512];
        for _ in 0..4 {
            runtime.render_block(&mut settle_l, &mut settle_r).unwrap();
        }
        let from = runtime.rendered_telemetry();
        let target = RuntimeTelemetry {
            rpm: 12_000.0,
            throttle: 0.4,
            normalized_engine_load: 0.35,
            normalized_engine_torque: -0.2,
            torque_sign: TorqueSign::Negative,
            rpm_derivative: -2000.0,
            throttle_derivative: -2.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 0.6,
            tc_cut_ratio: 0.3,
            rev_limiter_active: false,
            dt_seconds: 1.0 / 120.0,
        };
        runtime.update_telemetry(target).unwrap();
        let steps = runtime.renderer().interpolator().steps();
        let n = runtime.renderer().interpolator().remaining();
        assert!(n > 0);
        let expect = |a: f32, b: f32| (b - a) / n as f32;
        assert!((steps[0] - expect(from.rpm, target.rpm)).abs() < 1e-6);
        assert!((steps[6] - expect(from.clutch_engagement, target.clutch_engagement)).abs() < 1e-6);
        let mut prev_rpm = from.rpm;
        let mut max_step = 0.0f32;
        let mut one_l = [0.0; 1];
        let mut one_r = [0.0; 1];
        for i in 0..n {
            runtime.render_block(&mut one_l, &mut one_r).unwrap();
            assert!(one_l[0].is_finite());
            let rendered = runtime.rendered_telemetry();
            let ideal = from.rpm + (target.rpm - from.rpm) * (i + 1) as f32 / n as f32;
            let tolerance = if i + 1 == n { 1e-6 } else { 2.0 };
            assert!(
                (rendered.rpm - ideal).abs() < tolerance,
                "sample {i}: rpm {} vs ideal {ideal}",
                rendered.rpm
            );
            if target.rpm > from.rpm {
                assert!(rendered.rpm + 1e-3 >= prev_rpm, "rpm stepped back at {i}");
            } else {
                assert!(rendered.rpm - 1e-3 <= prev_rpm, "rpm stepped back at {i}");
            }
            max_step = max_step.max((rendered.rpm - prev_rpm).abs());
            prev_rpm = rendered.rpm;
        }
        let linear = (target.rpm - from.rpm).abs() / n as f32;
        assert!(
            max_step <= linear + 1e-3,
            "max step {max_step} vs linear {linear}"
        );
        let rendered = runtime.rendered_telemetry();
        assert!((rendered.rpm - target.rpm).abs() < 1e-6);
        assert!((rendered.clutch_engagement - target.clutch_engagement).abs() < 1e-6);
        assert!((rendered.tc_cut_ratio - target.tc_cut_ratio).abs() < 1e-6);
    }

    #[test]
    fn variable_blocks_are_finite_and_boundary_continuous() {
        let mut whole = running_runtime(1024);
        let mut split = running_runtime(1024);
        let mut whole_l = [0.0; 1024];
        let mut whole_r = [0.0; 1024];
        whole.render_block(&mut whole_l, &mut whole_r).unwrap();
        let mut joined = Vec::new();
        for frames in [1usize, 17, 255, 3, 512, 236] {
            let mut l = vec![0.0; frames];
            let mut r = vec![0.0; frames];
            split.render_block(&mut l, &mut r).unwrap();
            assert_eq!(l, r);
            assert!(l.iter().all(|sample| sample.is_finite()));
            joined.extend(l);
        }
        assert_eq!(joined, whole_l);
    }

    #[test]
    fn reset_restores_deterministic_output() {
        let mut runtime = running_runtime(128);
        let telemetry = runtime.telemetry();
        let mut first = [0.0; 128];
        let mut right = [0.0; 128];
        runtime.render_block(&mut first, &mut right).unwrap();
        runtime.reset().unwrap();
        runtime.update_telemetry(telemetry).unwrap();
        let mut after = [0.0; 128];
        runtime.render_block(&mut after, &mut right).unwrap();
        assert_eq!(first, after);
    }

    #[test]
    fn packaged_manifest_and_all_three_zones_load() {
        let mut config = Gf509RuntimeConfig::default();
        config.engine.sample_rate = 48_000;
        config.sample_layer_directory = Some(fixture_bank("bank_schema1"));
        let runtime = Gf509Runtime::new(config).unwrap();
        assert_eq!(runtime.sample_rate(), 48_000);
    }

    #[test]
    fn hybrid_blend_carries_the_gesture_to_the_sample_layer() {
        let (scene, sample, physical, sample_weight) = (0.5, 0.5, 1.0, 0.8165);
        let idle = blend_hybrid(scene, Some(sample), physical, sample_weight, 1.0);
        let ducked = blend_hybrid(scene, Some(sample), physical, sample_weight, 0.7);
        assert!(ducked < idle, "gesture must dip the sample contribution");
        let expected = (scene * physical + sample * sample_weight * 0.7) * GF509_HEADROOM_GAIN;
        assert!((ducked - expected).abs() < 1e-6, "{ducked} vs {expected}");
        assert_eq!(
            blend_hybrid(scene, None, physical, sample_weight, 0.7),
            scene
        );
    }

    #[test]
    fn pitch_sweep_stays_finite_and_bounded() {
        let mut config = Gf509RuntimeConfig::default();
        config.engine.sample_rate = 48_000;
        config.max_block_frames = 512;
        config.sample_layer_directory = Some(fixture_bank("bank_schema1"));
        let mut runtime = Gf509Runtime::new(config).unwrap();
        let mut left = [0.0f32; 512];
        let mut right = [0.0f32; 512];
        let mut peak = 0.0f32;
        for block in 0..64 {
            let rpm = if block < 32 {
                4_000.0 + 12_000.0 * (block as f32 / 31.0)
            } else {
                16_000.0 - 12_000.0 * ((block - 32) as f32 / 31.0)
            };
            runtime
                .update_telemetry(RuntimeTelemetry {
                    rpm,
                    throttle: 0.9,
                    normalized_engine_load: 0.85,
                    normalized_engine_torque: 0.8,
                    torque_sign: TorqueSign::Positive,
                    rpm_derivative: 0.0,
                    throttle_derivative: 0.0,
                    gear: 4,
                    shift_phase: ShiftPhase::None,
                    clutch_engagement: 1.0,
                    tc_cut_ratio: 0.0,
                    rev_limiter_active: false,
                    dt_seconds: 1.0 / 120.0,
                })
                .unwrap();
            runtime.render_block(&mut left, &mut right).unwrap();
            assert!(left.iter().all(|sample| sample.is_finite()));
            assert_eq!(left, right);
            peak = peak.max(
                left.iter()
                    .map(|sample| sample.abs())
                    .fold(0.0f32, f32::max),
            );
        }
        assert!(peak > 1e-4, "route must stay audible");
        assert!(peak <= 1.0, "route must stay under PCM ceiling, got {peak}");
    }

    #[test]
    fn sample_only_mode_requires_a_bank() {
        let config = Gf509RuntimeConfig {
            engine_mode: EngineMode::SampleOnly,
            ..Gf509RuntimeConfig::default()
        };
        assert!(Gf509Runtime::new(config).is_err());
    }

    #[test]
    fn sampled_bank_loads_and_renders_finite_over_the_sweep() {
        let mut config = Gf509RuntimeConfig::default();
        config.engine.sample_rate = 48_000;
        config.max_block_frames = 512;
        config.sample_layer_directory = Some(fixture_bank("bank_schema2"));
        config.engine_mode = EngineMode::SampleOnly;
        let mut runtime = Gf509Runtime::new(config).unwrap();
        assert_eq!(runtime.engine_mode(), EngineMode::SampleOnly);
        let mut left = [0.0f32; 512];
        let mut right = [0.0f32; 512];
        let mut peak = 0.0f32;
        for block in 0..64 {
            let rpm = if block < 32 {
                3_000.0 + 15_000.0 * (block as f32 / 31.0)
            } else {
                18_000.0 - 13_000.0 * ((block - 32) as f32 / 31.0)
            };
            runtime.update_telemetry(sampled_telemetry(rpm)).unwrap();
            runtime.render_block(&mut left, &mut right).unwrap();
            assert!(left.iter().all(|sample| sample.is_finite()));
            assert_eq!(left, right);
            peak = peak.max(
                left.iter()
                    .map(|sample| sample.abs())
                    .fold(0.0f32, f32::max),
            );
        }
        assert!(peak > 1e-4, "sampled engine must stay audible");
        assert!(
            peak <= 1.0,
            "sampled engine must stay under PCM ceiling, got {peak}"
        );
    }

    #[test]
    fn sample_only_ignores_the_procedural_engine_seed() {
        let base = Gf509RuntimeConfig {
            sample_layer_directory: Some(fixture_bank("bank_schema2")),
            engine_mode: EngineMode::SampleOnly,
            ..Gf509RuntimeConfig::default()
        };
        let mut first_config = base.clone();
        first_config.engine.sample_rate = 48_000;
        first_config.engine.seed = 0x1111;
        let mut second_config = base;
        second_config.engine.sample_rate = 48_000;
        second_config.engine.seed = 0x2222;
        let mut first = Gf509Runtime::new(first_config).unwrap();
        let mut second = Gf509Runtime::new(second_config).unwrap();
        let mut first_l = [0.0f32; 256];
        let mut first_r = [0.0f32; 256];
        let mut second_l = [0.0f32; 256];
        let mut second_r = [0.0f32; 256];
        for rpm in [4_500.0f32, 6_000.0, 9_000.0, 12_000.0] {
            first.update_telemetry(sampled_telemetry(rpm)).unwrap();
            second.update_telemetry(sampled_telemetry(rpm)).unwrap();
        }
        first.render_block(&mut first_l, &mut first_r).unwrap();
        second.render_block(&mut second_l, &mut second_r).unwrap();
        assert_eq!(first_l, second_l);
    }

    #[test]
    fn sample_only_bypasses_the_acoustic_scene() {
        let neutral = render_sampled_output(|_| {});
        let extreme = render_sampled_output(|config| {
            config.scene.output_gain = 5.0;
            config.scene.engine_air_gain = 1.5;
            config.scene.engine_cover_gain = 1.5;
            config.scene.mount_monocoque_gain = 1.5;
            config.scene.dry_mid_gain = 1.5;
            config.scene.dry_high_gain = 1.5;
        });
        assert_eq!(
            neutral, extreme,
            "sample-only engine must not pass through the acoustic scene"
        );
    }

    #[test]
    fn sample_only_sums_the_procedural_gearbox() {
        let muted_zero = render_sampled_output(|config| {
            config.transmission.output_gain = 0.0;
            config.gear_shift_gain = 0.0;
            config.sampled_gearbox_gain = 0.0;
        });
        let muted_one = render_sampled_output(|config| {
            config.transmission.output_gain = 0.0;
            config.gear_shift_gain = 0.0;
            config.sampled_gearbox_gain = 1.0;
        });
        assert_eq!(
            muted_zero, muted_one,
            "a silent gearbox must leave the bank untouched"
        );
        let audible_zero = render_sampled_output(|config| {
            config.transmission.output_gain = 0.5;
            config.gear_shift_gain = 0.5;
            config.sampled_gearbox_gain = 0.0;
        });
        let audible_one = render_sampled_output(|config| {
            config.transmission.output_gain = 0.5;
            config.gear_shift_gain = 0.5;
            config.sampled_gearbox_gain = 1.0;
        });
        assert_eq!(
            audible_zero, muted_zero,
            "bank output must not depend on gearbox settings"
        );
        assert_ne!(
            audible_zero, audible_one,
            "procedural gearbox must be summed into the sample-only mix"
        );
    }

    #[test]
    fn sample_only_shapes_engine_and_gearbox_buses_independently() {
        let base = render_sampled_output(|config| {
            config.transmission.output_gain = 0.5;
            config.gear_shift_gain = 0.5;
        });
        let engine_shaped = render_sampled_output(|config| {
            config.transmission.output_gain = 0.5;
            config.gear_shift_gain = 0.5;
            config.v10_filter.highpass_hz = 5_000.0;
            config.v10_filter.highpass_slope_db_per_oct = 24.0;
        });
        assert_ne!(base, engine_shaped, "the V10 chain must shape the bank bus");
        let gearbox_shaped = render_sampled_output(|config| {
            config.transmission.output_gain = 0.5;
            config.gear_shift_gain = 0.5;
            config.gearbox_filter.lowpass_hz = 1_500.0;
            config.gearbox_filter.lowpass_slope_db_per_oct = 24.0;
        });
        assert_ne!(
            base, gearbox_shaped,
            "the gearbox chain must shape the gearbox bus"
        );
    }

    #[test]
    fn hybrid_shapes_the_gearbox_independently_of_the_engine() {
        let render = |mutator: &dyn Fn(&mut Gf509RuntimeConfig)| {
            let mut config = Gf509RuntimeConfig::default();
            config.engine.sample_rate = 48_000;
            config.transmission.sample_rate = 48_000;
            config.transmission.output_gain = 0.8;
            config.transmission.gear_ratios = vec![3.45, 2.75, 2.3, 1.95, 1.68, 1.46];
            config.transmission.final_drive = 4.25;
            config.gear_shift_gain = 1.0;
            mutator(&mut config);
            let mut runtime = Gf509Runtime::new(config).unwrap();
            runtime
                .update_telemetry(sampled_telemetry(9_000.0))
                .unwrap();
            let mut left = [0.0f32; 512];
            let mut right = [0.0f32; 512];
            runtime.render_block(&mut left, &mut right).unwrap();
            left.to_vec()
        };
        let base = render(&|_| {});
        let gearbox_shaped = render(&|config| {
            config.gearbox_filter.highpass_hz = 5_000.0;
            config.gearbox_filter.highpass_slope_db_per_oct = 24.0;
        });
        assert_ne!(
            base, gearbox_shaped,
            "the gearbox chain must shape the hybrid gearbox bus"
        );
    }

    #[test]
    fn continuous_mechanics_interpolate_discrete_states_do_not() {
        let mut runtime = running_runtime(1024);
        runtime
            .update_telemetry(RuntimeTelemetry {
                rpm: 8_000.0,
                throttle: 0.8,
                normalized_engine_load: 0.7,
                normalized_engine_torque: -0.4,
                torque_sign: TorqueSign::Negative,
                rpm_derivative: -100.0,
                throttle_derivative: -1.0,
                gear: 4,
                shift_phase: ShiftPhase::UpshiftCut,
                clutch_engagement: 0.2,
                tc_cut_ratio: 0.8,
                rev_limiter_active: true,
                dt_seconds: 1.0 / 120.0,
            })
            .unwrap();
        let mut left = [0.0; 1];
        let mut right = [0.0; 1];
        runtime.render_block(&mut left, &mut right).unwrap();
        let rendered = runtime.rendered_telemetry();
        assert_eq!(rendered.shift_phase, ShiftPhase::UpshiftCut);
        assert!(rendered.clutch_engagement > 0.0 && rendered.clutch_engagement < 0.2);
        assert!(rendered.tc_cut_ratio > 0.0 && rendered.tc_cut_ratio < 0.8);
        assert!(rendered.rev_limiter_active);
    }

    #[test]
    fn cut_and_recovery_are_temporally_smooth_at_fixed_rpm() {
        let mut runtime = running_runtime(1024);
        let steady = RuntimeTelemetry {
            rpm: 10_000.0,
            throttle: 0.9,
            normalized_engine_load: 0.9,
            normalized_engine_torque: 0.9,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: 0.0,
            throttle_derivative: 0.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: 1.0 / 120.0,
        };
        runtime.update_telemetry(steady).unwrap();
        let mut left = [0.0; 512];
        let mut right = [0.0; 512];
        for _ in 0..40 {
            runtime.render_block(&mut left, &mut right).unwrap();
        }
        let steady_rms = render_average_rms(&mut runtime, &mut left, &mut right, 16);
        let steady_peak = left.iter().map(|s| s.abs()).fold(0.0f32, f32::max);
        assert!(steady_rms.is_finite() && steady_rms > 1e-6);

        let mut control = running_runtime(1024);
        control.update_telemetry(steady).unwrap();
        let mut control_left = [0.0; 512];
        let mut control_right = [0.0; 512];
        for _ in 0..40 {
            control
                .render_block(&mut control_left, &mut control_right)
                .unwrap();
        }
        let control_rms =
            render_average_rms(&mut control, &mut control_left, &mut control_right, 16);
        assert!(
            (control_rms - steady_rms).abs() / steady_rms < 0.05,
            "control without event must track steady: steady={steady_rms} control={control_rms}"
        );

        let mut cut = steady;
        cut.shift_phase = ShiftPhase::UpshiftCut;
        runtime.update_telemetry(cut).unwrap();
        let onset_rms = render_average_rms(&mut runtime, &mut left, &mut right, 4);
        assert!(
            onset_rms < steady_rms,
            "cut onset must start falling immediately: steady={steady_rms} onset={onset_rms}"
        );
        for _ in 0..36 {
            runtime.render_block(&mut left, &mut right).unwrap();
        }
        let cut_rms = render_average_rms(&mut runtime, &mut left, &mut right, 16);
        let cut_peak = left.iter().map(|s| s.abs()).fold(0.0f32, f32::max);
        assert!(
            cut_rms.is_finite() && cut_rms < steady_rms * 1.5,
            "cut response must remain bounded: steady={steady_rms} cut={cut_rms}"
        );
        assert!(
            cut_rms < steady_rms * 0.95,
            "cut must attenuate effectively vs steady control: steady={steady_rms} cut={cut_rms}"
        );
        assert!(
            (control_rms - cut_rms).abs() / steady_rms > 0.05,
            "cut must separate from the no-event control: control={control_rms} cut={cut_rms}"
        );
        assert!(left.iter().all(|sample| sample.is_finite()));
        assert!(cut_peak.is_finite() && cut_peak <= steady_peak * 1.5 + 1e-6);

        let mut recovery = cut;
        recovery.shift_phase = ShiftPhase::UpshiftRecovery;
        runtime.update_telemetry(recovery).unwrap();
        let recovery_onset = render_average_rms(&mut runtime, &mut left, &mut right, 4);
        assert!(
            recovery_onset > cut_rms * 0.9,
            "recovery must not collapse below the cut: cut={cut_rms} recovery_onset={recovery_onset}"
        );
        for _ in 0..36 {
            runtime.render_block(&mut left, &mut right).unwrap();
        }
        let recovery_rms = render_average_rms(&mut runtime, &mut left, &mut right, 16);
        assert!(
            recovery_rms.is_finite() && recovery_rms < steady_rms * 1.5,
            "recovery response must remain bounded: steady={steady_rms} recovery={recovery_rms}"
        );
        assert!(
            recovery_rms > cut_rms,
            "recovery must rise from the cut: cut={cut_rms} recovery={recovery_rms}"
        );
        assert!(
            recovery_rms < steady_rms * 1.05,
            "recovery must not overshoot steady: steady={steady_rms} recovery={recovery_rms}"
        );
        assert!(left.iter().all(|sample| sample.is_finite()));

        runtime.update_telemetry(steady).unwrap();
        for _ in 0..40 {
            runtime.render_block(&mut left, &mut right).unwrap();
        }
        let restored = render_average_rms(&mut runtime, &mut left, &mut right, 16);
        assert!(
            (restored - steady_rms).abs() / steady_rms < 0.10,
            "steady must be restored after the event: steady={steady_rms} restored={restored}"
        );
    }

    #[test]
    fn offline_load_scenarios_produce_measurably_different_audio() {
        let make_runtime =
            |throttle: f32, load: f32, torque: f32, t_sign: TorqueSign| -> Gf509Runtime {
                let config = Gf509RuntimeConfig {
                    max_block_frames: 1024,
                    ..Gf509RuntimeConfig::default()
                };
                let mut rt = Gf509Runtime::new(config).unwrap();
                rt.update_telemetry(RuntimeTelemetry {
                    rpm: 15_000.0,
                    throttle,
                    normalized_engine_load: load,
                    normalized_engine_torque: torque,
                    torque_sign: t_sign,
                    rpm_derivative: 0.0,
                    throttle_derivative: 0.0,
                    gear: 5,
                    shift_phase: ShiftPhase::None,
                    clutch_engagement: 1.0,
                    tc_cut_ratio: 0.0,
                    rev_limiter_active: false,
                    dt_seconds: 0.0,
                })
                .unwrap();
                rt
            };

        let mut rt_a = make_runtime(1.0, 1.0, 1.0, TorqueSign::Positive);
        let mut rt_b = make_runtime(1.0, 0.25, 0.25, TorqueSign::Positive);
        let mut rt_c = make_runtime(0.0, 0.55, -0.55, TorqueSign::Negative);
        let mut rt_d = make_runtime(0.0, 0.05, 0.0, TorqueSign::Neutral);

        let render_block = |rt: &mut Gf509Runtime| -> Vec<f32> {
            let mut l = [0.0f32; 1024];
            let mut r = [0.0f32; 1024];
            for _ in 0..5 {
                rt.render_block(&mut l, &mut r).unwrap();
            }
            rt.render_block(&mut l, &mut r).unwrap();
            l.to_vec()
        };

        let out_a = render_block(&mut rt_a);
        let out_b = render_block(&mut rt_b);
        let out_c = render_block(&mut rt_c);
        let out_d = render_block(&mut rt_d);

        let max_diff = |s1: &[f32], s2: &[f32]| -> f32 {
            s1.iter()
                .zip(s2.iter())
                .map(|(a, b)| (a - b).abs())
                .fold(0.0f32, f32::max)
        };

        assert!(max_diff(&out_a, &out_b) > 1e-4);
        assert!(max_diff(&out_a, &out_c) > 1e-4);
        assert!(max_diff(&out_a, &out_d) > 1e-4);
        assert!(max_diff(&out_b, &out_c) > 1e-4);
        assert!(max_diff(&out_c, &out_d) > 1e-4);
    }

    #[test]
    fn render_block_rejects_mismatched_and_oversized_buffers() {
        let mut runtime = running_runtime(64);
        let mut left = [0.0; 32];
        let mut right_short = [0.0; 16];
        assert!(runtime.render_block(&mut left, &mut right_short).is_err());

        let mut oversize_l = [0.0; 65];
        let mut oversize_r = [0.0; 65];
        assert!(runtime
            .render_block(&mut oversize_l, &mut oversize_r)
            .is_err());
    }
}
