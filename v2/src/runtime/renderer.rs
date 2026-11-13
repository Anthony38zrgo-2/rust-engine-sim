//! Allocation-free real-time audio renderer unifying engine source, gearbox bus,
//! independent spectrum chains, and final presence shelf.

use crate::assets::validate_asset_manifest;
use crate::config::Gf509RuntimeConfig;
use crate::control::{ControlInterpolator, EngineInput, RuntimeTelemetry};
use crate::drivetrain::{GearShiftPlayer, GearShiftSamples, TransmissionInput, TransmissionSynth};
use crate::dsp::{SpectrumChain, UpperMidShelf};
use crate::runtime::source::{EngineSource, EngineStepTrace};
use crate::samples::ThreeZoneSampleLayer;

/// Complete per-sample audio and physical trace captured by [`AudioRenderer::render_direct_traced`].
#[derive(Clone, Debug, Default)]
pub struct RenderTrace {
    pub engine: EngineStepTrace,
    pub transmission: f32,
    pub gear_shift: f32,
    pub gearbox_raw: f32,
    pub v10_filtered: f32,
    pub gearbox_filtered: f32,
    pub hybrid_mix: f32,
}

/// Common single-path audio pipeline driven at control rate via [`ControlInterpolator`]
/// and rendered sample-by-sample without heap allocation.
pub struct AudioRenderer {
    sample_rate: u32,
    max_block_frames: usize,
    controls: ControlInterpolator,
    engine_source: EngineSource,
    v10_filter: SpectrumChain,
    gearbox_filter: SpectrumChain,
    transmission: TransmissionSynth,
    gear_shift: GearShiftPlayer,
    upper_mid_shelf: UpperMidShelf,
}

impl AudioRenderer {
    /// Validates `config`, prepares sample/shift assets on the control thread, and
    /// preallocates all audio-thread DSP state.
    pub fn new(config: &Gf509RuntimeConfig) -> Result<Self, String> {
        config.validate()?;
        let sample_rate = config.engine.sample_rate;
        let sample_rate_hz = sample_rate as f32;

        let (gear_shift_samples, sample_layer) = match config.sample_layer_directory.as_deref() {
            Some(directory) => {
                let shift_samples = GearShiftSamples::load_directory(directory, sample_rate)?;
                validate_asset_manifest(directory, sample_rate)
                    .map_err(|error| error.to_string())?;
                let layer = ThreeZoneSampleLayer::load_directory(
                    sample_rate,
                    directory,
                    config.sample_layer,
                )?;
                (shift_samples, Some(layer))
            }
            None => (None, None),
        };

        let mut engine_config = config.engine.clone();
        if gear_shift_samples.is_some() {
            engine_config.blip_enabled = false;
        }

        let engine_source = EngineSource::new(config, engine_config, sample_layer)?;
        let upper_mid_shelf = UpperMidShelf::new(sample_rate_hz, config.upper_mid_shelf_gain)?;
        let v10_filter = SpectrumChain::new(sample_rate_hz, config.v10_filter)?;
        let gearbox_filter = SpectrumChain::new(sample_rate_hz, config.gearbox_filter)?;
        let transmission = TransmissionSynth::new(config.transmission.clone())?;
        let gear_shift = GearShiftPlayer::new(
            gear_shift_samples,
            config.gear_shift_gain,
            config.gear_shift_reference_rpm,
            sample_rate,
        )?;

        Ok(Self {
            sample_rate,
            max_block_frames: config.max_block_frames,
            controls: ControlInterpolator::default(),
            engine_source,
            v10_filter,
            gearbox_filter,
            transmission,
            gear_shift,
            upper_mid_shelf,
        })
    }

    /// Configured output sample rate in Hz.
    #[inline]
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Maximum supported block frame count in `render_block`.
    #[inline]
    pub fn max_block_frames(&self) -> usize {
        self.max_block_frames
    }

    /// Read-only view of the control-rate interpolator state.
    #[inline]
    pub fn interpolator(&self) -> &ControlInterpolator {
        &self.controls
    }

    /// Validates `telemetry` and latches control-rate linear interpolation steps.
    pub fn update_telemetry(&mut self, telemetry: RuntimeTelemetry) -> Result<(), String> {
        self.controls.update(telemetry, self.sample_rate)?;
        Ok(())
    }

    /// Preallocates a [`RenderTrace`] buffer sized to active sample-layer sources.
    pub fn allocate_trace(&self) -> RenderTrace {
        RenderTrace {
            engine: EngineStepTrace::with_source_capacity(self.engine_source.source_labels().len()),
            ..RenderTrace::default()
        }
    }

    /// Source labels from the attached sample layer, or an empty slice if none.
    #[inline]
    pub fn source_labels(&self) -> &[String] {
        self.engine_source.source_labels()
    }

    /// Whether a prepared sample layer is active inside this renderer.
    #[inline]
    pub fn has_sample_layer(&self) -> bool {
        self.engine_source.has_sample_layer()
    }

    /// Advances offline warmup without triggering sample-layer progression in hybrid mode
    /// or gearbox/shift progression.
    pub fn warmup_step(&mut self, input: EngineInput) -> Result<(), String> {
        self.engine_source.warmup_step(input)
    }

    #[inline]
    fn render_with_controls(
        &mut self,
        interpolated: RuntimeTelemetry,
        mut trace: Option<&mut RenderTrace>,
    ) -> Result<f32, String> {
        let engine_bus = match trace.as_deref_mut() {
            Some(out) => self
                .engine_source
                .render_sample_traced(interpolated, &mut out.engine)?,
            None => self.engine_source.render_sample(interpolated)?,
        };
        let transmission = self.transmission.process(TransmissionInput {
            rpm: interpolated.rpm,
            gear: interpolated.gear,
            clutch: interpolated.clutch_engagement,
            torque: interpolated.normalized_engine_torque,
            throttle: interpolated.throttle,
            shift_phase: interpolated.shift_phase,
        });
        let gear_shift = self
            .gear_shift
            .process(interpolated.shift_phase, interpolated.rpm);
        let gearbox_raw = transmission + gear_shift;
        let v10_filtered = self.v10_filter.process(engine_bus);
        let gearbox_pre_gain = self.gearbox_filter.process(gearbox_raw);
        let gearbox_filtered = match self.engine_source.gearbox_post_gain() {
            Some(gain) => gearbox_pre_gain * gain,
            None => gearbox_pre_gain,
        };
        let hybrid_mix = self
            .upper_mid_shelf
            .process(v10_filtered + gearbox_filtered);
        if let Some(out) = trace {
            out.transmission = transmission;
            out.gear_shift = gear_shift;
            out.gearbox_raw = gearbox_raw;
            out.v10_filtered = v10_filtered;
            out.gearbox_filtered = gearbox_filtered;
            out.hybrid_mix = hybrid_mix;
        }
        Ok(hybrid_mix)
    }

    /// Advances controls and renders one mono sample across engine, gearbox, and buses.
    #[inline]
    pub fn render_sample(&mut self) -> Result<f32, String> {
        let interpolated = self.controls.next_sample();
        self.render_with_controls(interpolated, None)
    }

    /// Renders one mono sample using explicit `telemetry` directly (without control-rate
    /// interpolation lag) and populates `trace` with all intermediate bus stems.
    #[inline]
    pub fn render_direct_traced(
        &mut self,
        telemetry: RuntimeTelemetry,
        trace: &mut RenderTrace,
    ) -> Result<f32, String> {
        self.render_with_controls(telemetry, Some(trace))
    }

    /// Renders equal-length centered stereo buffers and returns the rendered frame count.
    pub fn render_block(&mut self, left: &mut [f32], right: &mut [f32]) -> Result<usize, String> {
        if left.len() != right.len() {
            return Err("left and right buffers must have equal lengths".into());
        }
        if left.len() > self.max_block_frames {
            return Err(format!(
                "block has {} frames; configured maximum is {}",
                left.len(),
                self.max_block_frames
            ));
        }
        for (left_sample, right_sample) in left.iter_mut().zip(right.iter_mut()) {
            let output = self.render_sample()?;
            *left_sample = output;
            *right_sample = output;
        }
        Ok(left.len())
    }
}
