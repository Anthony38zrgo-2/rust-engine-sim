//! Allocation-free block facade for embedding the GF509 continuous engine.
//!
//! Asset loading and reset are control-thread operations. `render_block` owns
//! no temporary buffers and performs no file I/O or logging.

use std::path::PathBuf;
use std::{fs, path::Path};

use crate::{
    shift_energy_gain, AcousticScene, AcousticSceneConfig, Crankshaft, EngineConfig,
    EngineInput, GearShiftPlayer, GearShiftSamples, SampleLayerFrame, SampleLayerInput,
    ShiftGesture, SpectrumChain, SpectrumChainConfig, ThreeZoneSampleLayer,
    ThreeZoneSampleLayerConfig, TransmissionConfig, TransmissionInput, TransmissionSynth,
    UpperMidShelf, V10Engine,
};
use serde::Deserialize;
use sha2::{Digest, Sha256};

pub const GF509_HEADROOM_GAIN: f32 = 0.61;

/// Continuous-engine source selector. `Hybrid` is the shipped GF509 behaviour
/// (procedural engine + sample layer blended before the shelf); `SampleOnly`
/// replaces the engine with the prepared bank and keeps only the gearbox
/// procedural, summed after the acoustic scene.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EngineMode {
    #[default]
    Hybrid,
    SampleOnly,
}

/// Blends the procedural scene with the optional sample layer. The procedural
/// side already carries the synthesized shift gesture (it scales the engine
/// energy); the sample side is multiplied by the same gesture gain so the
/// whole hybrid wobbles together on recovery/blip. Without a sample layer the
/// scene is returned untouched (no double gesture).
#[inline]
pub fn blend_hybrid(
    scene: f32,
    sample: Option<f32>,
    physical_weight: f32,
    sample_weight: f32,
    gesture_gain: f32,
) -> f32 {
    match sample {
        Some(sample) => {
            (scene * physical_weight + sample * sample_weight * gesture_gain) * GF509_HEADROOM_GAIN
        }
        None => scene,
    }
}

#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum TorqueSign {
    Negative = -1,
    #[default]
    Neutral = 0,
    Positive = 1,
}

impl TorqueSign {
    pub fn from_i32(v: i32) -> Result<Self, String> {
        match v {
            -1 => Ok(Self::Negative),
            0 => Ok(Self::Neutral),
            1 => Ok(Self::Positive),
            _ => Err(format!("invalid torque sign: {}", v)),
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ShiftPhase {
    #[default]
    None = 0,
    UpshiftCut = 1,
    UpshiftRecovery = 2,
    DownshiftCut = 3,
    DownshiftBlip = 4,
    DownshiftRecovery = 5,
}

impl ShiftPhase {
    pub fn from_i32(v: i32) -> Result<Self, String> {
        match v {
            0 => Ok(Self::None),
            1 => Ok(Self::UpshiftCut),
            2 => Ok(Self::UpshiftRecovery),
            3 => Ok(Self::DownshiftCut),
            4 => Ok(Self::DownshiftBlip),
            5 => Ok(Self::DownshiftRecovery),
            _ => Err(format!("invalid shift phase: {}", v)),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RuntimeTelemetry {
    pub rpm: f32,
    pub throttle: f32,
    pub normalized_engine_load: f32,
    pub normalized_engine_torque: f32,
    pub torque_sign: TorqueSign,
    pub rpm_derivative: f32,
    pub throttle_derivative: f32,
    pub gear: i8,
    pub shift_phase: ShiftPhase,
    /// Clutch engagement from the physics powertrain, continuously smoothed.
    pub clutch_engagement: f32,
    /// Physical traction-control cut. The load packet already includes this
    /// cut; the synth consumes it only for a dedicated, non-gain modulation.
    pub tc_cut_ratio: f32,
    /// Discrete physical rev limiter state; changed sample-accurately and never
    /// interpolated with a continuous control.
    pub rev_limiter_active: bool,
    pub dt_seconds: f32,
}

impl RuntimeTelemetry {
    pub fn validate(self) -> Result<Self, String> {
        if !self.rpm.is_finite() || !(0.0..=25_000.0).contains(&self.rpm) {
            return Err(format!("rpm out of range: {}", self.rpm));
        }
        if !self.throttle.is_finite() || !(0.0..=1.0).contains(&self.throttle) {
            return Err(format!("throttle out of range: {}", self.throttle));
        }
        if !self.normalized_engine_load.is_finite()
            || !(0.0..=1.0).contains(&self.normalized_engine_load)
        {
            return Err(format!(
                "normalized_engine_load out of range: {}",
                self.normalized_engine_load
            ));
        }
        if !self.normalized_engine_torque.is_finite()
            || !(-1.0..=1.0).contains(&self.normalized_engine_torque)
        {
            return Err(format!(
                "normalized_engine_torque out of range: {}",
                self.normalized_engine_torque
            ));
        }
        if !self.clutch_engagement.is_finite()
            || !(0.0..=1.0).contains(&self.clutch_engagement)
        {
            return Err(format!("clutch_engagement out of range: {}", self.clutch_engagement));
        }
        if !self.tc_cut_ratio.is_finite() || !(0.0..=1.0).contains(&self.tc_cut_ratio) {
            return Err(format!("tc_cut_ratio out of range: {}", self.tc_cut_ratio));
        }
        if !self.rpm_derivative.is_finite() {
            return Err(format!("rpm_derivative not finite: {}", self.rpm_derivative));
        }
        if !self.throttle_derivative.is_finite() {
            return Err(format!(
                "throttle_derivative not finite: {}",
                self.throttle_derivative
            ));
        }
        if !(-1..=12).contains(&self.gear) {
            return Err(format!("gear out of range: {}", self.gear));
        }
        if !self.dt_seconds.is_finite() || !(0.0..=1.0).contains(&self.dt_seconds) {
            return Err(format!("dt_seconds out of range: {}", self.dt_seconds));
        }
        Ok(self)
    }
}

impl Default for RuntimeTelemetry {
    fn default() -> Self {
        Self {
            rpm: 0.0,
            throttle: 0.0,
            normalized_engine_load: 0.0,
            normalized_engine_torque: 0.0,
            torque_sign: TorqueSign::Neutral,
            rpm_derivative: 0.0,
            throttle_derivative: 0.0,
            gear: 0,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 0.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Gf509RuntimeConfig {
    pub engine: EngineConfig,
    pub scene: AcousticSceneConfig,
    pub sample_layer: ThreeZoneSampleLayerConfig,
    /// Prepared sample directory. `None` is the procedural-only INT-01 mode.
    pub sample_layer_directory: Option<PathBuf>,
    pub upper_mid_shelf_gain: f32,
    pub transmission: TransmissionConfig,
    pub gear_shift_gain: f32,
    pub gear_shift_reference_rpm: f32,
    pub max_block_frames: usize,
    /// Hybrid (shipped) or sample-only engine source.
    pub engine_mode: EngineMode,
    /// Bank drive gain for the sample-only engine (level calibration).
    pub sampled_engine_gain: f32,
    /// Post-scene gain on the procedural gearbox in sample-only mode. The
    /// gearbox bypasses the acoustic scene there, so it carries its own level.
    pub sampled_gearbox_gain: f32,
    /// Spectrum chain on the V10 engine bus (procedural + samples). Default is
    /// a bit-transparent bypass.
    pub v10_filter: SpectrumChainConfig,
    /// Spectrum chain on the procedural gearbox bus. Default is a
    /// bit-transparent bypass.
    pub gearbox_filter: SpectrumChainConfig,
}

impl Default for Gf509RuntimeConfig {
    fn default() -> Self {
        Self {
            engine: EngineConfig::default(),
            scene: AcousticSceneConfig::default(),
            sample_layer: ThreeZoneSampleLayerConfig::default(),
            sample_layer_directory: None,
            upper_mid_shelf_gain: 0.0,
            transmission: TransmissionConfig::default(),
            gear_shift_gain: 1.0,
            gear_shift_reference_rpm: 10_000.0,
            max_block_frames: 4096,
            engine_mode: EngineMode::Hybrid,
            sampled_engine_gain: 1.0,
            sampled_gearbox_gain: 1.0,
            v10_filter: SpectrumChainConfig::default(),
            gearbox_filter: SpectrumChainConfig::default(),
        }
    }
}

pub struct Gf509Runtime {
    config: Gf509RuntimeConfig,
    engine: V10Engine,
    scene: AcousticScene,
    sample_layer: Option<ThreeZoneSampleLayer>,
    upper_mid_shelf: UpperMidShelf,
    /// Spectrum chain on the V10 engine bus (procedural + samples).
    v10_filter: SpectrumChain,
    /// Spectrum chain on the procedural gearbox bus.
    gearbox_filter: SpectrumChain,
    transmission: TransmissionSynth,
    gear_shift: GearShiftPlayer,
    /// Sample-only phase authority: a bare crankshaft (no engine model) keeps
    /// loop phase alignment and the scene's firing-frequency tracker fed.
    sampled_crank: Crankshaft,
    /// Sample-only shift gesture (cut + wobble + blip) applied to the bank.
    sampled_gesture: ShiftGesture,
    /// Reused per-sample sample-layer output. Keeps the audio callback
    /// allocation-free by avoiding a fresh `SampleLayerFrame` per sample.
    sample_frame: SampleLayerFrame,
    telemetry: RuntimeTelemetry,
    rendered_telemetry: RuntimeTelemetry,
    interpolation_remaining: usize,
    /// AUD-05: linear interpolation steps for the 8 continuous controls
    /// [rpm, throttle, load, torque, rpm_dot, throttle_dot, clutch, tc].
    /// Computed once per `update_telemetry` (control rate) instead of dividing
    /// by the decreasing remainder on every sample. The exact remainder
    /// division is mathematically the same constant step; precomputing it
    /// removes 8 divisions per sample with only float-rounding differences.
    interp_steps: [f32; 8],
}

impl Gf509Runtime {
    pub fn new(config: Gf509RuntimeConfig) -> Result<Self, String> {
        if config.max_block_frames == 0 {
            return Err("max_block_frames must be greater than zero".into());
        }
        let sample_rate = config.engine.sample_rate;
        let gear_shift_samples = match config.sample_layer_directory.as_deref() {
            Some(directory) => GearShiftSamples::load_directory(directory, sample_rate)?,
            None => None,
        };
        let mut engine_config = config.engine.clone();
        if gear_shift_samples.is_some() {
            engine_config.blip_enabled = false;
        }
        let engine = V10Engine::new(engine_config)?;
        let scene = AcousticScene::new(sample_rate as f32, config.scene)?;
        if let Some(directory) = config.sample_layer_directory.as_deref() {
            validate_asset_manifest(directory, sample_rate)?;
        }
        let sample_layer = config
            .sample_layer_directory
            .as_deref()
            .map(|path| {
                ThreeZoneSampleLayer::load_directory(sample_rate, path, config.sample_layer)
            })
            .transpose()?;
        let upper_mid_shelf = UpperMidShelf::new(sample_rate as f32, config.upper_mid_shelf_gain)?;
        let v10_filter = SpectrumChain::new(sample_rate as f32, config.v10_filter)?;
        let gearbox_filter = SpectrumChain::new(sample_rate as f32, config.gearbox_filter)?;
        let transmission = TransmissionSynth::new(config.transmission.clone())?;
        let gear_shift = GearShiftPlayer::new(
            gear_shift_samples,
            config.gear_shift_gain,
            config.gear_shift_reference_rpm,
            sample_rate,
        )?;
        if config.engine_mode == EngineMode::SampleOnly && sample_layer.is_none() {
            return Err("sample-only engine mode requires a sample-layer bank".into());
        }
        let sampled_crank = Crankshaft::new(sample_rate, config.engine.firing_order);
        Ok(Self {
            config,
            engine,
            scene,
            sample_layer,
            upper_mid_shelf,
            v10_filter,
            gearbox_filter,
            transmission,
            gear_shift,
            sampled_crank,
            sampled_gesture: ShiftGesture::default(),
            sample_frame: SampleLayerFrame::default(),
            telemetry: RuntimeTelemetry::default(),
            rendered_telemetry: RuntimeTelemetry::default(),
            interpolation_remaining: 0,
            interp_steps: [0.0; 8],
        })
    }

    pub fn sample_rate(&self) -> u32 {
        self.config.engine.sample_rate
    }
    pub fn engine_mode(&self) -> EngineMode {
        self.config.engine_mode
    }
    pub fn max_block_frames(&self) -> usize {
        self.config.max_block_frames
    }
    pub fn telemetry(&self) -> RuntimeTelemetry {
        self.telemetry
    }
    pub fn rendered_telemetry(&self) -> RuntimeTelemetry {
        self.rendered_telemetry
    }

    pub fn update_telemetry(&mut self, telemetry: RuntimeTelemetry) -> Result<(), String> {
        self.telemetry = telemetry.validate()?;
        let remaining =
            (telemetry.dt_seconds * self.sample_rate() as f32).round() as usize;
        self.interpolation_remaining = remaining;
        // AUD-05 control-rate step: constant linear increment from the current
        // rendered state to the new target. Applied per sample by addition.
        // Discrete states (sign/gear/phase/limiter) still switch immediately
        // in render_block. Zero-remaining (legacy dt=0) keeps snap behavior.
        if remaining > 0 {
            let from = self.rendered_telemetry;
            let inv = 1.0 / remaining as f32;
            self.interp_steps = [
                (telemetry.rpm - from.rpm) * inv,
                (telemetry.throttle - from.throttle) * inv,
                (telemetry.normalized_engine_load - from.normalized_engine_load) * inv,
                (telemetry.normalized_engine_torque - from.normalized_engine_torque) * inv,
                (telemetry.rpm_derivative - from.rpm_derivative) * inv,
                (telemetry.throttle_derivative - from.throttle_derivative) * inv,
                (telemetry.clutch_engagement - from.clutch_engagement) * inv,
                (telemetry.tc_cut_ratio - from.tc_cut_ratio) * inv,
            ];
        } else {
            self.interp_steps = [0.0; 8];
        }
        Ok(())
    }

    /// Renders equal-length planar stereo buffers and returns their frame count.
    /// GF509 is currently a centered continuous source, so both channels match.
    pub fn render_block(&mut self, left: &mut [f32], right: &mut [f32]) -> Result<usize, String> {
        if left.len() != right.len() {
            return Err("left and right buffers must have equal lengths".into());
        }
        if left.len() > self.config.max_block_frames {
            return Err(format!(
                "block has {} frames; configured maximum is {}",
                left.len(),
                self.config.max_block_frames
            ));
        }
        let target = self.telemetry;
        for (left_sample, right_sample) in left.iter_mut().zip(right.iter_mut()) {
            if self.interpolation_remaining > 0 {
                // AUD-05: precomputed linear steps (control rate); no per-sample
                // division. Snap exactly to target on the final step to avoid
                // float accumulation drift.
                let steps = self.interp_steps;
                self.rendered_telemetry.rpm += steps[0];
                self.rendered_telemetry.throttle += steps[1];
                self.rendered_telemetry.normalized_engine_load += steps[2];
                self.rendered_telemetry.normalized_engine_torque += steps[3];
                self.rendered_telemetry.rpm_derivative += steps[4];
                self.rendered_telemetry.throttle_derivative += steps[5];
                self.rendered_telemetry.clutch_engagement += steps[6];
                self.rendered_telemetry.tc_cut_ratio += steps[7];
                self.interpolation_remaining -= 1;
                if self.interpolation_remaining == 0 {
                    self.rendered_telemetry.rpm = target.rpm;
                    self.rendered_telemetry.throttle = target.throttle;
                    self.rendered_telemetry.normalized_engine_load =
                        target.normalized_engine_load;
                    self.rendered_telemetry.normalized_engine_torque =
                        target.normalized_engine_torque;
                    self.rendered_telemetry.rpm_derivative = target.rpm_derivative;
                    self.rendered_telemetry.throttle_derivative =
                        target.throttle_derivative;
                    self.rendered_telemetry.clutch_engagement = target.clutch_engagement;
                    self.rendered_telemetry.tc_cut_ratio = target.tc_cut_ratio;
                }
            } else {
                self.rendered_telemetry = target;
            }
            self.rendered_telemetry.torque_sign = target.torque_sign;
            self.rendered_telemetry.gear = target.gear;
            self.rendered_telemetry.shift_phase = target.shift_phase;
            self.rendered_telemetry.rev_limiter_active = target.rev_limiter_active;
            self.rendered_telemetry.dt_seconds = target.dt_seconds;
            let interpolated = self.rendered_telemetry;
            let output = match self.config.engine_mode {
                EngineMode::Hybrid => self.render_hybrid_sample(interpolated)?,
                EngineMode::SampleOnly => self.render_sampled_sample(interpolated)?,
            };
            *left_sample = output;
            *right_sample = output;
        }
        Ok(left.len())
    }

    /// Hybrid path: procedural engine + scene, sample layer blended. The
    /// procedural gearbox stays out of the acoustic scene and is summed after
    /// its own spectrum chain, so the V10 and gearbox buses are shaped
    /// independently.
    fn render_hybrid_sample(&mut self, interpolated: RuntimeTelemetry) -> Result<f32, String> {
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
        let sample_frame = if let Some(layer) = self.sample_layer.as_mut() {
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
        let engine_mix = blend_hybrid(
            scene_frame.output,
            sample_frame,
            self.config.sample_layer.physical_blend_weight,
            self.config.sample_layer.sample_blend_weight,
            self.engine.shift_gesture_gain(),
        );
        let gearbox = self.transmission.process(TransmissionInput {
            rpm: interpolated.rpm,
            gear: interpolated.gear,
            clutch: interpolated.clutch_engagement,
            torque: interpolated.normalized_engine_torque,
            throttle: interpolated.throttle,
            shift_phase: interpolated.shift_phase,
        }) + self.gear_shift.process(interpolated.shift_phase, interpolated.rpm);
        let engine_out = self.v10_filter.process(engine_mix);
        let gearbox_out = self.gearbox_filter.process(gearbox);
        Ok(self.upper_mid_shelf.process(engine_out + gearbox_out))
    }

    /// Sample-only variant: the prepared bank is the engine source. The bank
    /// and the procedural gearbox are summed independently, each through its
    /// own spectrum chain (the bank never enters the acoustic scene).
    fn render_sampled_sample(&mut self, interpolated: RuntimeTelemetry) -> Result<f32, String> {
        let sample_rate = self.sample_rate() as f32;
        let events = self.sampled_crank.step(interpolated.rpm);
        let crank_phase_deg = events.crank_phase_deg;
        let shift_phase = interpolated.shift_phase as u8;
        // Gear-shift one-shots own the audible blip, so the procedural blip is
        // disabled exactly as in the hybrid path.
        self.sampled_gesture
            .step(shift_phase, sample_rate, false);
        let layer = self
            .sample_layer
            .as_mut()
            .ok_or("sample-only engine mode requires a sample-layer bank")?;
        layer.process_into(
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
        let drive = self.sample_frame.output
            * gesture
            * GF509_HEADROOM_GAIN
            * self.config.sampled_engine_gain;
        // The sampled engine never enters the acoustic scene: the bank is the
        // whole engine and the procedural gearbox is summed independently.
        let gearbox = self.transmission.process(TransmissionInput {
            rpm: interpolated.rpm,
            gear: interpolated.gear,
            clutch: interpolated.clutch_engagement,
            torque: interpolated.normalized_engine_torque,
            throttle: interpolated.throttle,
            shift_phase: interpolated.shift_phase,
        }) + self.gear_shift.process(interpolated.shift_phase, interpolated.rpm);
        let engine_out = self.v10_filter.process(drive);
        let gearbox_out = self.gearbox_filter.process(gearbox) * self.config.sampled_gearbox_gain;
        Ok(self.upper_mid_shelf.process(engine_out + gearbox_out))
    }

    /// Restores deterministic post-construction state while retaining config.
    /// This may reload assets and must not be called from the audio callback.
    pub fn reset(&mut self) -> Result<(), String> {
        let replacement = Self::new(self.config.clone())?;
        *self = replacement;
        Ok(())
    }
}

#[derive(Deserialize)]
struct AssetManifest {
    schema_version: u32,
    key: String,
    output_gain: f32,
    samples: Vec<AssetEntry>,
}

#[derive(Deserialize)]
struct AssetEntry {
    key: String,
    metadata: String,
    native_rpm: f32,
    sample_rate: u32,
    channels: u16,
    loop_frames: usize,
    tonal: String,
    tonal_sha256: String,
    residual: String,
    residual_sha256: String,
}

fn validate_asset_manifest(directory: &Path, output_sample_rate: u32) -> Result<(), String> {
    let path = directory.join("manifest.json");
    let raw = fs::read_to_string(&path)
        .map_err(|error| format!("cannot read GF509 manifest {}: {error}", path.display()))?;
    let peek: serde_json::Value = serde_json::from_str(&raw)
        .map_err(|error| format!("invalid GF509 manifest {}: {error}", path.display()))?;
    let schema = peek
        .get("schema_version")
        .and_then(|value| value.as_u64())
        .unwrap_or(0);
    if schema == 2 {
        return validate_experimental_manifest(directory, output_sample_rate, &raw);
    }
    let manifest: AssetManifest = serde_json::from_str(&raw)
        .map_err(|error| format!("invalid GF509 manifest {}: {error}", path.display()))?;
    if manifest.schema_version != 1 || manifest.key != "v10_gf509" {
        return Err("GF509 manifest schema/key mismatch".into());
    }
    if (manifest.output_gain - GF509_HEADROOM_GAIN).abs() > f32::EPSILON {
        return Err("GF509 manifest output gain does not match the frozen profile".into());
    }
    if manifest.samples.len() != 3 {
        return Err(format!(
            "GF509 manifest expected 3 samples, found {}",
            manifest.samples.len()
        ));
    }
    for (expected_key, entry) in ["low", "med", "max"].into_iter().zip(&manifest.samples) {
        if entry.key != expected_key
            || !entry.native_rpm.is_finite()
            || entry.sample_rate != output_sample_rate
            || entry.channels != 1
            || entry.loop_frames < 32
        {
            return Err(format!("invalid GF509 manifest entry: {}", entry.key));
        }
        for name in [&entry.metadata, &entry.tonal, &entry.residual] {
            let candidate = Path::new(name);
            if candidate.is_absolute() || candidate.components().count() != 1 {
                return Err(format!("GF509 asset path must be a local filename: {name}"));
            }
        }
        if !directory.join(&entry.metadata).is_file() {
            return Err(format!("missing GF509 metadata: {}", entry.metadata));
        }
        verify_sha256(&directory.join(&entry.tonal), &entry.tonal_sha256)?;
        verify_sha256(&directory.join(&entry.residual), &entry.residual_sha256)?;
    }
    Ok(())
}

#[derive(Deserialize)]
struct ExperimentalVariant {
    group: String,
    position: usize,
    count: usize,
}

#[derive(Deserialize)]
struct ExperimentalEntry {
    key: String,
    role: String,
    metadata: String,
    native_rpm: f32,
    sample_rate: u32,
    channels: u16,
    loop_frames: usize,
    tonal: String,
    tonal_sha256: String,
    residual: String,
    residual_sha256: String,
    variant: Option<ExperimentalVariant>,
}

#[derive(Deserialize)]
struct ExperimentalManifest {
    schema_version: u32,
    key: String,
    output_gain: f32,
    samples: Vec<ExperimentalEntry>,
    off_samples: Vec<ExperimentalEntry>,
}

/// Schema-2 F2002 experimental bank: fail-closed validation. Any load
/// failure must abort with a clear message; silent fallback is forbidden.
fn validate_experimental_manifest(
    directory: &Path,
    output_sample_rate: u32,
    raw: &str,
) -> Result<(), String> {
    let manifest: ExperimentalManifest = serde_json::from_str(raw)
        .map_err(|error| format!("invalid experimental manifest: {error}"))?;
    validate_experimental_header(&manifest)?;

    for entry in manifest.samples.iter().chain(manifest.off_samples.iter()) {
        let expected_role = if manifest
            .samples
            .iter()
            .any(|sample| sample.key == entry.key)
        {
            "on"
        } else {
            "off"
        };
        validate_experimental_entry(entry, directory, output_sample_rate, expected_role)?;
    }

    validate_experimental_variants(&manifest.samples)?;
    validate_experimental_variants(&manifest.off_samples)?;
    Ok(())
}

fn validate_experimental_header(manifest: &ExperimentalManifest) -> Result<(), String> {
    if manifest.schema_version != 2
        || !matches!(
            manifest.key.as_str(),
            "v10_f2002_experimental" | "v10_f2002_sampled"
        )
    {
        return Err("experimental manifest schema/key mismatch".into());
    }
    if (manifest.output_gain - GF509_HEADROOM_GAIN).abs() > f32::EPSILON {
        return Err("experimental manifest output gain does not match 0.61".into());
    }
    if manifest.samples.is_empty() || manifest.off_samples.is_empty() {
        return Err("experimental manifest needs ON and OFF collections".into());
    }
    Ok(())
}

fn validate_experimental_entry(
    entry: &ExperimentalEntry,
    directory: &Path,
    output_sample_rate: u32,
    expected_role: &str,
) -> Result<(), String> {
    if entry.role != expected_role
        || !entry.native_rpm.is_finite()
        || entry.sample_rate != output_sample_rate
        || entry.channels != 1
        || entry.loop_frames < 32
    {
        return Err(format!(
            "invalid experimental manifest entry: {}",
            entry.key
        ));
    }
    for name in [&entry.metadata, &entry.tonal, &entry.residual] {
        let candidate = Path::new(name);
        if candidate.is_absolute() || candidate.components().count() != 1 {
            return Err(format!(
                "experimental asset path must be a local filename: {name}"
            ));
        }
        if !directory.join(name).is_file() {
            return Err(format!("missing experimental asset: {name}"));
        }
    }
    verify_sha256(&directory.join(&entry.tonal), &entry.tonal_sha256)?;
    verify_sha256(&directory.join(&entry.residual), &entry.residual_sha256)?;
    if let Some(variant) = &entry.variant {
        if variant.count < 2 || variant.position >= variant.count || variant.group.is_empty() {
            return Err(format!(
                "invalid variant record for experimental entry: {}",
                entry.key
            ));
        }
    }
    Ok(())
}

/// Variant groups must be complete and consistent within one role collection.
fn validate_experimental_variants(collection: &[ExperimentalEntry]) -> Result<(), String> {
    let mut groups: std::collections::BTreeMap<&str, Vec<(&str, usize, usize)>> =
        std::collections::BTreeMap::new();
    for entry in collection {
        if let Some(variant) = &entry.variant {
            groups.entry(variant.group.as_str()).or_default().push((
                entry.key.as_str(),
                variant.position,
                variant.count,
            ));
        }
    }
    for (group, members) in &groups {
        if members.iter().any(|(_, _, count)| *count != members.len()) {
            return Err(format!("experimental variant group {group} is incomplete"));
        }
        let mut positions: Vec<usize> = members.iter().map(|(_, position, _)| *position).collect();
        positions.sort_unstable();
        if positions != (0..members.len()).collect::<Vec<_>>() {
            return Err(format!(
                "experimental variant group {group} positions invalid"
            ));
        }
    }
    Ok(())
}

fn verify_sha256(path: &Path, expected: &str) -> Result<(), String> {
    let bytes = fs::read(path)
        .map_err(|error| format!("cannot read GF509 asset {}: {error}", path.display()))?;
    let actual = format!("{:x}", Sha256::digest(&bytes));
    if actual != expected {
        return Err(format!("GF509 asset hash mismatch: {}", path.display()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn running_runtime(max_block_frames: usize) -> Gf509Runtime {
        let mut config = Gf509RuntimeConfig::default();
        config.max_block_frames = max_block_frames;
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

    #[test]
    fn transmission_layer_is_summed_into_the_hybrid_mix() {
        let render = |gain: f32| {
            let mut config = Gf509RuntimeConfig::default();
            config.transmission.output_gain = gain;
            config.transmission.gear_ratios = vec![3.45, 2.75, 2.3, 1.95, 1.68, 1.46];
            config.transmission.final_drive = 4.25;
            let mut runtime = Gf509Runtime::new(config).unwrap();
            runtime
                .update_telemetry(RuntimeTelemetry {
                    rpm: 9_000.0,
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
                })
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
        // AUD-05: precomputed linear steps must reach the target exactly,
        // advance monotonically per sample (no step discontinuities), and stay
        // within float-rounding distance of the ideal linear ramp.
        let mut runtime = running_runtime(1024);
        // Settle to the initial target first.
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
        let steps = runtime.interp_steps;
        let n = runtime.interpolation_remaining;
        assert!(n > 0);
        // Steps match the ideal linear ramp from the settled state.
        let expect = |a: f32, b: f32| (b - a) / n as f32;
        assert!((steps[0] - expect(from.rpm, target.rpm)).abs() < 1e-6);
        assert!((steps[6] - expect(from.clutch_engagement, target.clutch_engagement)).abs() < 1e-6);
        // Render sample by sample (1-frame blocks track rendered state).
        let mut prev_rpm = from.rpm;
        let mut max_step = 0.0f32;
        let mut one_l = [0.0; 1];
        let mut one_r = [0.0; 1];
        for i in 0..n {
            runtime.render_block(&mut one_l, &mut one_r).unwrap();
            assert!(one_l[0].is_finite());
            let rendered = runtime.rendered_telemetry();
            let ideal = from.rpm + (target.rpm - from.rpm) * (i + 1) as f32 / n as f32;
            // Snap on the final step keeps exact parity with the target.
            let tolerance = if i + 1 == n { 1e-6 } else { 2.0 };
            assert!((rendered.rpm - ideal).abs() < tolerance,
                "sample {i}: rpm {} vs ideal {ideal}", rendered.rpm);
            // Monotonic approach toward the target (no zipper reversals).
            if target.rpm > from.rpm {
                assert!(rendered.rpm + 1e-3 >= prev_rpm, "rpm stepped back at {i}");
            } else {
                assert!(rendered.rpm - 1e-3 <= prev_rpm, "rpm stepped back at {i}");
            }
            max_step = max_step.max((rendered.rpm - prev_rpm).abs());
            prev_rpm = rendered.rpm;
        }
        // Largest single-sample RPM step is the linear increment (no spikes).
        let linear = (target.rpm - from.rpm).abs() / n as f32;
        assert!(max_step <= linear + 1e-3, "max step {max_step} vs linear {linear}");
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
        config.engine.sample_rate = 44_100;
        config.sample_layer_directory =
            Some(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_gf509"));
        let runtime = Gf509Runtime::new(config).unwrap();
        assert_eq!(runtime.sample_rate(), 44_100);
    }

    #[test]
    fn hybrid_blend_carries_the_gesture_to_the_sample_layer() {
        let (scene, sample, physical, sample_weight) = (0.5, 0.5, 1.0, 0.8165);
        let idle = blend_hybrid(scene, Some(sample), physical, sample_weight, 1.0);
        let ducked = blend_hybrid(scene, Some(sample), physical, sample_weight, 0.7);
        assert!(ducked < idle, "gesture must dip the sample contribution");
        let expected = (scene * physical + sample * sample_weight * 0.7) * GF509_HEADROOM_GAIN;
        assert!((ducked - expected).abs() < 1e-6, "{ducked} vs {expected}");
        // The procedural side already carries the gesture inside the engine;
        // without a sample layer the blend must not apply it twice.
        assert_eq!(
            blend_hybrid(scene, None, physical, sample_weight, 0.7),
            scene
        );
    }

    #[test]
    fn pitch_sweep_stays_finite_and_bounded() {
        // Full-route smoke: an RPM sweep crossing pitch-up ratios must render
        // finite audio under the PCM ceiling.
        let mut config = Gf509RuntimeConfig::default();
        config.engine.sample_rate = 44_100;
        config.max_block_frames = 512;
        config.sample_layer_directory =
            Some(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_gf509"));
        let mut runtime = Gf509Runtime::new(config).unwrap();
        let mut left = [0.0f32; 512];
        let mut right = [0.0f32; 512];
        let mut peak = 0.0f32;
        for block in 0..64 {
            // Sweep up then down through the zone anchors.
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
            peak = peak.max(left.iter().map(|sample| sample.abs()).fold(0.0f32, f32::max));
        }
        assert!(peak > 1e-4, "route must stay audible");
        assert!(peak <= 1.0, "route must stay under PCM ceiling, got {peak}");
    }

    #[test]
    fn data_driven_blend_weights_are_validated() {
        let mut config = ThreeZoneSampleLayerConfig::default();
        assert!(config.validate().is_ok());

        config.physical_blend_weight = 2.5; // > 2.0
        assert!(config.validate().is_err());
        config.physical_blend_weight = 0.5;
        assert!(config.validate().is_ok());
    }

    fn sampled_bank_path() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../audio/v10_f2002_sampled")
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

    #[test]
    fn sample_only_mode_requires_a_bank() {
        let mut config = Gf509RuntimeConfig::default();
        config.engine_mode = EngineMode::SampleOnly;
        assert!(Gf509Runtime::new(config).is_err());
    }

    #[test]
    fn sampled_bank_loads_and_renders_finite_over_the_sweep() {
        let mut config = Gf509RuntimeConfig::default();
        config.engine.sample_rate = 44_100;
        config.max_block_frames = 512;
        config.sample_layer_directory = Some(sampled_bank_path());
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
            peak = peak.max(left.iter().map(|sample| sample.abs()).fold(0.0f32, f32::max));
        }
        assert!(peak > 1e-4, "sampled engine must stay audible");
        assert!(peak <= 1.0, "sampled engine must stay under PCM ceiling, got {peak}");
    }

    #[test]
    fn sample_only_ignores_the_procedural_engine_seed() {
        // The procedural engine must contribute nothing in sample-only mode:
        // two different seeds must render bit-identical output.
        let base = Gf509RuntimeConfig {
            sample_layer_directory: Some(sampled_bank_path()),
            engine_mode: EngineMode::SampleOnly,
            ..Gf509RuntimeConfig::default()
        };
        let mut first_config = base.clone();
        first_config.engine.sample_rate = 44_100;
        first_config.engine.seed = 0x1111;
        let mut second_config = base;
        second_config.engine.sample_rate = 44_100;
        second_config.engine.seed = 0x2222;
        let mut first = Gf509Runtime::new(first_config).unwrap();
        let mut second = Gf509Runtime::new(second_config).unwrap();
        let mut first_l = [0.0f32; 256];
        let mut first_r = [0.0f32; 256];
        let mut second_l = [0.0f32; 256];
        let mut second_r = [0.0f32; 256];
        for rpm in [4_500.0f32, 6_000.0, 9_000.0, 12_000.0] {
            first
                .update_telemetry(sampled_telemetry(rpm))
                .unwrap();
            second
                .update_telemetry(sampled_telemetry(rpm))
                .unwrap();
        }
        first.render_block(&mut first_l, &mut first_r).unwrap();
        second.render_block(&mut second_l, &mut second_r).unwrap();
        assert_eq!(first_l, second_l);
    }

    fn render_sampled_output(config_mutator: impl FnOnce(&mut Gf509RuntimeConfig)) -> Vec<f32> {
        let mut config = Gf509RuntimeConfig::default();
        config.engine.sample_rate = 44_100;
        config.sample_layer_directory = Some(sampled_bank_path());
        config.engine_mode = EngineMode::SampleOnly;
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
        assert_ne!(
            base, engine_shaped,
            "the V10 chain must shape the bank bus"
        );
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
            config.engine.sample_rate = 44_100;
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
    fn runtime_telemetry_validation_rejects_out_of_range() {
        let valid = RuntimeTelemetry {
            rpm: 9000.0,
            throttle: 0.8,
            normalized_engine_load: 0.7,
            normalized_engine_torque: 0.6,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: 500.0,
            throttle_derivative: 1.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: 0.016,
        };
        assert!(valid.validate().is_ok());

        // Negative load rejected
        let mut invalid = valid;
        invalid.normalized_engine_load = -0.1;
        assert!(invalid.validate().is_err());

        // Load > 1.0 rejected
        invalid = valid;
        invalid.normalized_engine_load = 1.05;
        assert!(invalid.validate().is_err());

        // Torque < -1.0 rejected
        invalid = valid;
        invalid.normalized_engine_torque = -1.1;
        assert!(invalid.validate().is_err());

        // Torque > 1.0 rejected
        invalid = valid;
        invalid.normalized_engine_torque = 1.1;
        assert!(invalid.validate().is_err());

        // Non-finite derivatives rejected
        invalid = valid;
        invalid.rpm_derivative = f32::NAN;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.throttle_derivative = f32::INFINITY;
        assert!(invalid.validate().is_err());

        // Gear range
        invalid = valid;
        invalid.gear = -2;
        assert!(invalid.validate().is_err());
        invalid.gear = 13;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.clutch_engagement = -0.1;
        assert!(invalid.validate().is_err());
        invalid = valid;
        invalid.tc_cut_ratio = 1.1;
        assert!(invalid.validate().is_err());
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

        // Control without event: an identical runtime held at steady must stay
        // near steady, proving any later drop comes from the cut (R3).
        let mut control = running_runtime(1024);
        control.update_telemetry(steady).unwrap();
        let mut control_left = [0.0; 512];
        let mut control_right = [0.0; 512];
        for _ in 0..40 {
            control.render_block(&mut control_left, &mut control_right).unwrap();
        }
        let control_rms = render_average_rms(&mut control, &mut control_left, &mut control_right, 16);
        assert!((control_rms - steady_rms).abs() / steady_rms < 0.05,
            "control without event must track steady: steady={steady_rms} control={control_rms}");

        // Cut onset: the first blocks after the event already fall, and the
        // settled cut attenuates effectively (a no-op cut would fail here).
        let mut cut = steady;
        cut.shift_phase = ShiftPhase::UpshiftCut;
        runtime.update_telemetry(cut).unwrap();
        let onset_rms = render_average_rms(&mut runtime, &mut left, &mut right, 4);
        assert!(onset_rms < steady_rms,
            "cut onset must start falling immediately: steady={steady_rms} onset={onset_rms}");
        for _ in 0..36 {
            runtime.render_block(&mut left, &mut right).unwrap();
        }
        let cut_rms = render_average_rms(&mut runtime, &mut left, &mut right, 16);
        let cut_peak = left.iter().map(|s| s.abs()).fold(0.0f32, f32::max);
        assert!(cut_rms.is_finite() && cut_rms < steady_rms * 1.5,
            "cut response must remain bounded: steady={steady_rms} cut={cut_rms}");
        assert!(cut_rms < steady_rms * 0.95,
            "cut must attenuate effectively vs steady control: steady={steady_rms} cut={cut_rms}");
        assert!((control_rms - cut_rms).abs() / steady_rms > 0.05,
            "cut must separate from the no-event control: control={control_rms} cut={cut_rms}");
        assert!(left.iter().all(|sample| sample.is_finite()));
        assert!(cut_peak.is_finite() && cut_peak <= steady_peak * 1.5 + 1e-6);

        // Recovery: rises from the cut toward steady without overshoot.
        let mut recovery = cut;
        recovery.shift_phase = ShiftPhase::UpshiftRecovery;
        runtime.update_telemetry(recovery).unwrap();
        let recovery_onset = render_average_rms(&mut runtime, &mut left, &mut right, 4);
        assert!(recovery_onset > cut_rms * 0.9,
            "recovery must not collapse below the cut: cut={cut_rms} recovery_onset={recovery_onset}");
        for _ in 0..36 {
            runtime.render_block(&mut left, &mut right).unwrap();
        }
        let recovery_rms = render_average_rms(&mut runtime, &mut left, &mut right, 16);
        assert!(recovery_rms.is_finite() && recovery_rms < steady_rms * 1.5,
            "recovery response must remain bounded: steady={steady_rms} recovery={recovery_rms}");
        assert!(recovery_rms > cut_rms,
            "recovery must rise from the cut: cut={cut_rms} recovery={recovery_rms}");
        assert!(recovery_rms < steady_rms * 1.05,
            "recovery must not overshoot steady: steady={steady_rms} recovery={recovery_rms}");
        assert!(left.iter().all(|sample| sample.is_finite()));

        // Return to steady restores the baseline level (duration/recovery closed).
        runtime.update_telemetry(steady).unwrap();
        for _ in 0..40 {
            runtime.render_block(&mut left, &mut right).unwrap();
        }
        let restored = render_average_rms(&mut runtime, &mut left, &mut right, 16);
        assert!((restored - steady_rms).abs() / steady_rms < 0.10,
            "steady must be restored after the event: steady={steady_rms} restored={restored}");
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
    fn torque_sign_and_shift_phase_conversions() {
        assert_eq!(TorqueSign::from_i32(-1).unwrap(), TorqueSign::Negative);
        assert_eq!(TorqueSign::from_i32(0).unwrap(), TorqueSign::Neutral);
        assert_eq!(TorqueSign::from_i32(1).unwrap(), TorqueSign::Positive);
        assert!(TorqueSign::from_i32(2).is_err());

        assert_eq!(ShiftPhase::from_i32(0).unwrap(), ShiftPhase::None);
        assert_eq!(ShiftPhase::from_i32(1).unwrap(), ShiftPhase::UpshiftCut);
        assert_eq!(ShiftPhase::from_i32(2).unwrap(), ShiftPhase::UpshiftRecovery);
        assert_eq!(ShiftPhase::from_i32(3).unwrap(), ShiftPhase::DownshiftCut);
        assert_eq!(ShiftPhase::from_i32(4).unwrap(), ShiftPhase::DownshiftBlip);
        assert_eq!(ShiftPhase::from_i32(5).unwrap(), ShiftPhase::DownshiftRecovery);
        assert!(ShiftPhase::from_i32(6).is_err());
    }

    #[test]
    fn offline_load_scenarios_produce_measurably_different_audio() {
        // Section 14.3 test cases:
        // A: 15000 RPM, throttle 1.0, load 1.0, positive torque
        // B: 15000 RPM, throttle 1.0, load 0.25, positive torque
        // C: 15000 RPM, throttle 0.0, load 0.55, negative torque
        // D: 15000 RPM, throttle 0.0, load 0.05, near-zero torque
        let make_runtime = |throttle: f32, load: f32, torque: f32, t_sign: TorqueSign| -> Gf509Runtime {
            let mut config = Gf509RuntimeConfig::default();
            config.max_block_frames = 1024;
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
            }).unwrap();
            rt
        };

        let mut rt_a = make_runtime(1.0, 1.0, 1.0, TorqueSign::Positive);
        let mut rt_b = make_runtime(1.0, 0.25, 0.25, TorqueSign::Positive);
        let mut rt_c = make_runtime(0.0, 0.55, -0.55, TorqueSign::Negative);
        let mut rt_d = make_runtime(0.0, 0.05, 0.0, TorqueSign::Neutral);

        let render_block = |rt: &mut Gf509Runtime| -> Vec<f32> {
            let mut l = [0.0f32; 1024];
            let mut r = [0.0f32; 1024];
            // Warm up
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
            s1.iter().zip(s2.iter()).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max)
        };

        let diff_ab = max_diff(&out_a, &out_b);
        let diff_ac = max_diff(&out_a, &out_c);
        let diff_ad = max_diff(&out_a, &out_d);
        let diff_bc = max_diff(&out_b, &out_c);
        let diff_cd = max_diff(&out_c, &out_d);

        assert!(diff_ab > 1e-4, "Scenario A and B must differ (same throttle 1.0, different load 1.0 vs 0.25): got {diff_ab}");
        assert!(diff_ac > 1e-4, "Scenario A and C must differ: got {diff_ac}");
        assert!(diff_ad > 1e-4, "Scenario A and D must differ: got {diff_ad}");
        assert!(diff_bc > 1e-4, "Scenario B and C must differ: got {diff_bc}");
        assert!(diff_cd > 1e-4, "Scenario C and D must differ (same throttle 0.0, different load/torque): got {diff_cd}");
    }

    struct ExperimentalDir(std::path::PathBuf);
    impl Drop for ExperimentalDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn experimental_dir(label: &str) -> ExperimentalDir {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("f90_exp_manifest_{label}_{unique}"));
        std::fs::create_dir_all(&dir).unwrap();
        ExperimentalDir(dir)
    }

    fn write_asset(dir: &Path, name: &str, bytes: &[u8]) -> String {
        std::fs::write(dir.join(name), bytes).unwrap();
        format!("{:x}", Sha256::digest(bytes))
    }

    fn experimental_entry(
        dir: &Path,
        key: &str,
        role: &str,
        sample_rate: u32,
    ) -> serde_json::Value {
        let metadata = format!("{key}_meta.json");
        let tonal = format!("{key}_tonal.pcm");
        let residual = format!("{key}_residual.pcm");
        write_asset(dir, &metadata, b"{}");
        let tonal_sha = write_asset(dir, &tonal, format!("{key}-tonal").as_bytes());
        let residual_sha = write_asset(dir, &residual, format!("{key}-residual").as_bytes());
        json!({
            "key": key,
            "role": role,
            "metadata": metadata,
            "native_rpm": 8000.0,
            "sample_rate": sample_rate,
            "channels": 1,
            "loop_frames": 128,
            "tonal": tonal,
            "tonal_sha256": tonal_sha,
            "residual": residual,
            "residual_sha256": residual_sha,
        })
    }

    fn experimental_manifest_value(dir: &Path, sample_rate: u32) -> serde_json::Value {
        json!({
            "schema_version": 2,
            "key": "v10_f2002_experimental",
            "output_gain": GF509_HEADROOM_GAIN,
            "samples": [experimental_entry(dir, "on_low", "on", sample_rate)],
            "off_samples": [experimental_entry(dir, "off_low", "off", sample_rate)],
        })
    }

    /// Writes the manifest next to the assets and validates through the
    /// schema-dispatching entry point.
    fn validate_value(
        dir: &Path,
        sample_rate: u32,
        value: &serde_json::Value,
    ) -> Result<(), String> {
        std::fs::write(dir.join("manifest.json"), value.to_string()).unwrap();
        validate_asset_manifest(dir, sample_rate)
    }

    #[test]
    fn experimental_manifest_accepts_complete_bank() {
        let guard = experimental_dir("valid");
        let value = experimental_manifest_value(&guard.0, 44_100);
        assert!(validate_value(&guard.0, 44_100, &value).is_ok());
    }

    #[test]
    fn experimental_manifest_rejects_header_and_entry_defects() {
        let guard = experimental_dir("defects");
        let dir = &guard.0;
        let valid = experimental_manifest_value(dir, 44_100);

        let mut bad = valid.clone();
        bad["key"] = json!("wrong");
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["output_gain"] = json!(0.5);
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"] = json!([]);
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["role"] = json!("off");
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["native_rpm"] = json!(1e40);
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["sample_rate"] = json!(22_050);
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["channels"] = json!(2);
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["loop_frames"] = json!(8);
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["metadata"] = json!("../escape.json");
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["metadata"] = json!("absent.json");
        assert!(validate_value(dir, 44_100, &bad).is_err());

        let mut bad = valid.clone();
        bad["samples"][0]["tonal_sha256"] = json!("00");
        assert!(validate_value(dir, 44_100, &bad).is_err());
    }

    #[test]
    fn experimental_manifest_variants_must_be_complete_and_ordered() {
        let guard = experimental_dir("variants");
        let dir = &guard.0;

        let mut value = experimental_manifest_value(dir, 44_100);
        value["samples"][0]["variant"] = json!({"group": "rpm", "position": 0, "count": 2});
        let mut second = experimental_entry(dir, "on_high", "on", 44_100);
        second["variant"] = json!({"group": "rpm", "position": 1, "count": 2});
        value["samples"].as_array_mut().unwrap().push(second);
        assert!(validate_value(dir, 44_100, &value).is_ok());

        // A declared count larger than the actual membership is incomplete.
        let mut incomplete = value.clone();
        incomplete["samples"][0]["variant"]["count"] = json!(3);
        assert!(validate_value(dir, 44_100, &incomplete).is_err());

        // Duplicate positions do not form the required 0..n ordering.
        let mut duplicate = value.clone();
        duplicate["samples"][1]["variant"]["position"] = json!(0);
        assert!(validate_value(dir, 44_100, &duplicate).is_err());

        // An empty group name or count below two is an invalid record.
        let mut bad_record = value.clone();
        bad_record["samples"][0]["variant"]["group"] = json!("");
        assert!(validate_value(dir, 44_100, &bad_record).is_err());

        let mut bad_record = value.clone();
        bad_record["samples"][0]["variant"]["count"] = json!(1);
        assert!(validate_value(dir, 44_100, &bad_record).is_err());
    }
}
