//! Static-dispatch engine audio sources (`Hybrid` vs `SampleOnly`) owned by `AudioRenderer`.

use crate::acoustics::{AcousticFrame, AcousticScene};
use crate::config::{EngineConfig, EngineMode, Gf509RuntimeConfig};
use crate::control::{shift_energy_gain, EngineInput, RuntimeTelemetry, ShiftGesture};
use crate::physics::Crankshaft;
use crate::runtime::mixing::{blend_hybrid, GF509_HEADROOM_GAIN};
use crate::samples::{SampleLayerFrame, SampleLayerInput, ThreeZoneSampleLayer};
use crate::synthesis::{EngineFrame, V10Engine};

/// Intermediate per-sample engine, scene, and sample-layer signals captured during traced renders.
#[derive(Clone, Debug, Default)]
pub struct EngineStepTrace {
    pub engine_frame: EngineFrame,
    pub acoustic_frame: AcousticFrame,
    pub has_sample_frame: bool,
    pub sample_frame: SampleLayerFrame,
    pub engine_bus: f32,
}

impl EngineStepTrace {
    /// Allocates trace scratch buffers sized to the configured source count.
    pub fn with_source_capacity(source_count: usize) -> Self {
        Self {
            sample_frame: SampleLayerFrame::with_source_capacity(source_count),
            ..Self::default()
        }
    }
}

#[inline]
fn copy_sample_frame(src: &SampleLayerFrame, dst: &mut SampleLayerFrame) {
    dst.tonal = src.tonal;
    dst.residual = src.residual;
    dst.max_rasp = src.max_rasp;
    dst.off_throttle = src.off_throttle;
    dst.output = src.output;
    dst.zone_weights.clear();
    dst.zone_weights.extend_from_slice(&src.zone_weights);
}

#[inline]
fn clear_sample_frame(dst: &mut SampleLayerFrame) {
    dst.tonal = 0.0;
    dst.residual = 0.0;
    dst.max_rasp = 0.0;
    dst.off_throttle = 0.0;
    dst.output = 0.0;
    dst.zone_weights.clear();
}

/// Procedural + optional sample-layer hybrid engine source.
pub struct HybridEngineSource {
    engine: V10Engine,
    scene: AcousticScene,
    sample_layer: Option<ThreeZoneSampleLayer>,
    sample_frame: SampleLayerFrame,
    physical_blend_weight: f32,
    sample_blend_weight: f32,
}

impl HybridEngineSource {
    fn new(
        config: &Gf509RuntimeConfig,
        engine_config: EngineConfig,
        sample_layer: Option<ThreeZoneSampleLayer>,
    ) -> Result<Self, String> {
        let sample_rate = config.engine.sample_rate as f32;
        let engine = V10Engine::new(engine_config)?;
        let scene = AcousticScene::new(sample_rate, config.scene)?;
        let sample_frame = sample_layer.as_ref().map_or_else(
            SampleLayerFrame::default,
            ThreeZoneSampleLayer::allocate_frame,
        );
        Ok(Self {
            engine,
            scene,
            sample_layer,
            sample_frame,
            physical_blend_weight: config.sample_layer.physical_blend_weight,
            sample_blend_weight: config.sample_layer.sample_blend_weight,
        })
    }

    fn warmup_step(&mut self, input: EngineInput) -> Result<(), String> {
        self.engine.set_input(input)?;
        let frame = self.engine.render_sample();
        self.scene.process(&frame);
        Ok(())
    }

    #[inline]
    fn render_sample_inner(
        &mut self,
        interpolated: RuntimeTelemetry,
        trace: Option<&mut EngineStepTrace>,
    ) -> Result<f32, String> {
        self.engine.set_input(EngineInput {
            rpm: interpolated.rpm,
            throttle: interpolated.throttle,
            load: interpolated.normalized_engine_load,
        })?;
        self.engine.set_mechanical_state(
            interpolated.normalized_engine_torque,
            interpolated.clutch_engagement,
            interpolated.tc_cut_ratio,
            interpolated.rev_limiter_active,
            interpolated.shift_phase,
        );
        let engine_frame = self.engine.render_sample();
        let scene_frame = self.scene.process(&engine_frame);
        let sample_output = if let Some(layer) = self.sample_layer.as_mut() {
            layer.process_into(
                SampleLayerInput {
                    rpm: interpolated.rpm,
                    throttle: interpolated.throttle,
                    load: interpolated.normalized_engine_load,
                    normalized_engine_torque: interpolated.normalized_engine_torque,
                    clutch_engagement: interpolated.clutch_engagement,
                    crank_phase_deg: engine_frame.crank_phase_deg,
                },
                &mut self.sample_frame,
            )?;
            Some(self.sample_frame.output)
        } else {
            None
        };
        let engine_bus = blend_hybrid(
            scene_frame.output,
            sample_output,
            self.physical_blend_weight,
            self.sample_blend_weight,
            self.engine.shift_gesture_gain(),
        );
        if let Some(out) = trace {
            out.engine_frame = engine_frame;
            out.acoustic_frame = scene_frame;
            out.engine_bus = engine_bus;
            if self.sample_layer.is_some() {
                out.has_sample_frame = true;
                copy_sample_frame(&self.sample_frame, &mut out.sample_frame);
            } else {
                out.has_sample_frame = false;
                clear_sample_frame(&mut out.sample_frame);
            }
        }
        Ok(engine_bus)
    }
}

/// Prepared-bank engine source driven by an explicit crankshaft phase accumulator.
pub struct SampleOnlyEngineSource {
    sample_rate_hz: f32,
    sampled_crank: Crankshaft,
    sampled_gesture: ShiftGesture,
    sample_layer: ThreeZoneSampleLayer,
    sample_frame: SampleLayerFrame,
    sampled_engine_gain: f32,
    sampled_gearbox_gain: f32,
}

impl SampleOnlyEngineSource {
    fn new(
        config: &Gf509RuntimeConfig,
        sample_layer: Option<ThreeZoneSampleLayer>,
    ) -> Result<Self, String> {
        let sample_layer =
            sample_layer.ok_or("sample-only engine mode requires a sample-layer bank")?;
        let sample_rate = config.engine.sample_rate;
        let sample_frame = sample_layer.allocate_frame();
        Ok(Self {
            sample_rate_hz: sample_rate as f32,
            sampled_crank: Crankshaft::with_cylinder_count(
                sample_rate,
                config.engine.firing_order,
                config.engine.cylinder_count,
            ),
            sampled_gesture: ShiftGesture::default(),
            sample_layer,
            sample_frame,
            sampled_engine_gain: config.sampled_engine_gain,
            sampled_gearbox_gain: config.sampled_gearbox_gain,
        })
    }

    fn warmup_step(&mut self, input: EngineInput) -> Result<(), String> {
        let events = self.sampled_crank.step(input.rpm);
        self.sample_layer.process_into(
            SampleLayerInput {
                rpm: input.rpm,
                throttle: input.throttle,
                load: input.load,
                normalized_engine_torque: input.load,
                clutch_engagement: 1.0,
                crank_phase_deg: events.crank_phase_deg,
            },
            &mut self.sample_frame,
        )
    }

    #[inline]
    fn render_sample_inner(
        &mut self,
        interpolated: RuntimeTelemetry,
        trace: Option<&mut EngineStepTrace>,
    ) -> Result<f32, String> {
        let events = self.sampled_crank.step(interpolated.rpm);
        let crank_phase_deg = events.crank_phase_deg;
        let shift_phase = interpolated.shift_phase as u8;
        // Gear-shift one-shots own the audible blip cleanly, matching the hybrid path.
        self.sampled_gesture
            .step(shift_phase, self.sample_rate_hz, false);
        self.sample_layer.process_into(
            SampleLayerInput {
                rpm: interpolated.rpm,
                throttle: interpolated.throttle,
                load: interpolated.normalized_engine_load,
                normalized_engine_torque: interpolated.normalized_engine_torque,
                clutch_engagement: interpolated.clutch_engagement,
                crank_phase_deg,
            },
            &mut self.sample_frame,
        )?;
        let gesture = shift_energy_gain(shift_phase) * self.sampled_gesture.gain();
        let engine_bus =
            self.sample_frame.output * gesture * GF509_HEADROOM_GAIN * self.sampled_engine_gain;
        if let Some(out) = trace {
            out.engine_frame = EngineFrame {
                throttle: interpolated.throttle,
                load: interpolated.normalized_engine_load,
                crank_phase_deg,
                master: engine_bus,
                ..EngineFrame::default()
            };
            out.acoustic_frame = AcousticFrame::default();
            out.has_sample_frame = true;
            copy_sample_frame(&self.sample_frame, &mut out.sample_frame);
            out.engine_bus = engine_bus;
        }
        Ok(engine_bus)
    }
}

/// Static-dispatch continuous engine bus generator (`Hybrid` or `SampleOnly`).
pub enum EngineSource {
    Hybrid(Box<HybridEngineSource>),
    SampleOnly(Box<SampleOnlyEngineSource>),
}

impl EngineSource {
    /// Builds the active engine source using already-prepared sample assets.
    pub fn new(
        config: &Gf509RuntimeConfig,
        engine_config: EngineConfig,
        sample_layer: Option<ThreeZoneSampleLayer>,
    ) -> Result<Self, String> {
        match config.engine_mode {
            EngineMode::Hybrid => Ok(Self::Hybrid(Box::new(HybridEngineSource::new(
                config,
                engine_config,
                sample_layer,
            )?))),
            EngineMode::SampleOnly => Ok(Self::SampleOnly(Box::new(SampleOnlyEngineSource::new(
                config,
                sample_layer,
            )?))),
        }
    }

    /// Advances warmup state matching offline `v1` semantics (without sample playback in hybrid
    /// mode or shift/filter progression in sample-only mode).
    pub fn warmup_step(&mut self, input: EngineInput) -> Result<(), String> {
        match self {
            Self::Hybrid(hybrid) => hybrid.warmup_step(input),
            Self::SampleOnly(sampled) => sampled.warmup_step(input),
        }
    }

    /// Renders one mono sample of the continuous engine bus without heap allocation.
    #[inline]
    pub fn render_sample(&mut self, interpolated: RuntimeTelemetry) -> Result<f32, String> {
        match self {
            Self::Hybrid(hybrid) => hybrid.render_sample_inner(interpolated, None),
            Self::SampleOnly(sampled) => sampled.render_sample_inner(interpolated, None),
        }
    }

    /// Renders one mono sample and records intermediate engine/scene/sample stems into `trace`.
    #[inline]
    pub fn render_sample_traced(
        &mut self,
        interpolated: RuntimeTelemetry,
        trace: &mut EngineStepTrace,
    ) -> Result<f32, String> {
        match self {
            Self::Hybrid(hybrid) => hybrid.render_sample_inner(interpolated, Some(trace)),
            Self::SampleOnly(sampled) => sampled.render_sample_inner(interpolated, Some(trace)),
        }
    }

    /// Post-filter multiplier applied to the gearbox bus (`None` in hybrid mode).
    #[inline]
    pub fn gearbox_post_gain(&self) -> Option<f32> {
        match self {
            Self::Hybrid(_) => None,
            Self::SampleOnly(sampled) => Some(sampled.sampled_gearbox_gain),
        }
    }

    /// Returns source weight labels from the active sample layer, or an empty slice.
    pub fn source_labels(&self) -> &[String] {
        match self {
            Self::Hybrid(hybrid) => hybrid
                .sample_layer
                .as_ref()
                .map_or(&[], ThreeZoneSampleLayer::source_labels),
            Self::SampleOnly(sampled) => sampled.sample_layer.source_labels(),
        }
    }

    /// Indicates whether a prepared sample layer is attached to this source.
    pub fn has_sample_layer(&self) -> bool {
        match self {
            Self::Hybrid(hybrid) => hybrid.sample_layer.is_some(),
            Self::SampleOnly(_) => true,
        }
    }
}
