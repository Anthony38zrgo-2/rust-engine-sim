//! Offline rendering, sweep/shift scenario scripting, diagnostic stem recording,
//! and provenance metadata generation layered above [`crate::runtime::AudioRenderer`].

pub mod provenance;
pub mod recorder;
pub mod scenario;

use std::path::PathBuf;

use crate::config::{AcousticSceneConfig, Gf509RuntimeConfig};
use crate::runtime::AudioRenderer;

pub use provenance::{git_head, source_dirty, source_fingerprint};
pub use recorder::{OfflineRenderSummary, StemRecorder};
pub use scenario::{
    parse_shift_sequence, shift_phase_at, shift_sequence_label, validate_shift_sequence,
    OfflineScenarioConfig, ShiftDirection, ShiftEvent,
};

/// Full configuration consumed by [`run_offline_render`] and the `v10_render` binary.
#[derive(Clone, Debug)]
pub struct OfflineRenderConfig {
    pub runtime: Gf509RuntimeConfig,
    pub scenario: OfflineScenarioConfig,
    pub gear: i8,
    pub out: PathBuf,
    pub stems_dir: PathBuf,
    pub acoustic_scene: bool,
    pub physical_telemetry_csv: Option<PathBuf>,
    pub physical_master_lowpass_hz: Option<f32>,
    pub exhaust_gain_overridden: bool,
    pub sample_residual_gain_override: Option<f32>,
    pub sample_blend_weight_override: Option<f32>,
    pub physical_blend_weight_override: Option<f32>,
    pub sample_zone_trim_db: Vec<f32>,
    pub scene_gain_overrides_label: String,
}

impl Default for OfflineRenderConfig {
    fn default() -> Self {
        let mut runtime = Gf509RuntimeConfig::default();
        runtime.transmission.sample_rate = runtime.engine.sample_rate;
        Self {
            runtime,
            scenario: OfflineScenarioConfig::default(),
            gear: 4,
            out: PathBuf::from("reports/audio/rust-greenfield/gf310_5000rpm.wav"),
            stems_dir: PathBuf::from("reports/audio/rust-greenfield/gf310_5000rpm_stems"),
            acoustic_scene: false,
            physical_telemetry_csv: None,
            physical_master_lowpass_hz: Some(2_500.0),
            exhaust_gain_overridden: false,
            sample_residual_gain_override: None,
            sample_blend_weight_override: None,
            physical_blend_weight_override: None,
            sample_zone_trim_db: Vec::new(),
            scene_gain_overrides_label: String::new(),
        }
    }
}

/// Applies a single `--scene-gain <branch>=<value>` override onto `scene`.
pub fn apply_scene_gain_override(
    scene: &mut AcousticSceneConfig,
    name: &str,
    gain: f32,
) -> Result<(), String> {
    let slot = match name {
        "dry_low" => &mut scene.dry_low_gain,
        "dry_mid" => &mut scene.dry_mid_gain,
        "dry_high" => &mut scene.dry_high_gain,
        "engine_air" => &mut scene.engine_air_gain,
        "engine_cover" => &mut scene.engine_cover_gain,
        "mount_monocoque" => &mut scene.mount_monocoque_gain,
        "transmission_mount" => &mut scene.transmission_mount_gain,
        "transmission_cover" => &mut scene.transmission_cover_gain,
        "air_tilt_db" => &mut scene.air_high_tilt_db,
        "air_direct" => &mut scene.air_direct_gain,
        "output" => &mut scene.output_gain,
        _ => return Err(format!("unknown scene branch: {name}")),
    };
    *slot = gain;
    Ok(())
}

/// Executes an offline render using [`AudioRenderer`] directly, writing WAV outputs,
/// 61 diagnostic stem WAVs, CSV telemetry, and `.metadata.json`.
pub fn run_offline_render(config: &OfflineRenderConfig) -> Result<OfflineRenderSummary, String> {
    config.scenario.validate()?;
    let mut renderer = AudioRenderer::new(&config.runtime)?;
    let sample_rate = renderer.sample_rate();
    let sample_rate_f32 = sample_rate as f32;
    let dt_seconds = 1.0 / sample_rate_f32;

    let warmup_samples = (config.scenario.warmup * sample_rate_f32).round() as usize;
    let warmup_input = config.scenario.initial_input();
    for _ in 0..warmup_samples {
        renderer.warmup_step(warmup_input)?;
    }

    let total_frames = (config.scenario.seconds * sample_rate_f32).round() as usize;
    let mut trace = renderer.allocate_trace();
    let source_labels = renderer.source_labels().to_vec();
    let has_sample_layer = renderer.has_sample_layer();
    let mut recorder = StemRecorder::new(config, total_frames, &source_labels, has_sample_layer)?;

    for sample in 0..total_frames {
        let time_s = sample as f32 / sample_rate_f32;
        let telemetry = config
            .scenario
            .telemetry_at(time_s, config.gear, dt_seconds);
        let rpm = telemetry.rpm;
        renderer.render_direct_traced(telemetry, &mut trace)?;
        recorder.record_sample(sample, time_s, rpm, &trace)?;
    }

    recorder.finish(config)
}
