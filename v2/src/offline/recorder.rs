//! Stem capture, event/physical CSV telemetry, PCM16 WAV export, and render metadata.

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use serde::Serialize;

use crate::assets::write_mono_pcm16;
use crate::config::EngineMode;
use crate::dsp::ThreePoleLowPass;
use crate::offline::provenance::{git_head, source_dirty, source_fingerprint};
use crate::offline::scenario::shift_sequence_label;
use crate::offline::OfflineRenderConfig;
use crate::physics::CYLINDER_COUNT;
use crate::runtime::{RenderTrace, GF509_HEADROOM_GAIN};

const STEM_NAMES: [&str; 61] = [
    "combustion_source",
    "pressure_derivative",
    "pressure_derivative_a",
    "pressure_derivative_b",
    "pressure_direct",
    "crankcase",
    "block",
    "head",
    "block_head",
    "headers_a",
    "headers_b",
    "collector_a",
    "collector_b",
    "collector_pressure_a",
    "collector_pressure_b",
    "exhaust",
    "turbulence",
    "master_pre_limiter",
    "master",
    "transmission",
    "gear_shift",
    "engine_dry",
    "engine_air",
    "engine_cover",
    "engine_mounts",
    "monocoque_seat",
    "mount_monocoque",
    "cylinder_mechanical_sum",
    "cylinder_mechanical_0",
    "cylinder_mechanical_1",
    "cylinder_mechanical_2",
    "cylinder_mechanical_3",
    "cylinder_mechanical_4",
    "cylinder_mechanical_5",
    "cylinder_mechanical_6",
    "cylinder_mechanical_7",
    "cylinder_mechanical_8",
    "cylinder_mechanical_9",
    "scene_mix",
    "sample_tonal",
    "sample_residual",
    "sample_max_rasp",
    "sample_off_throttle",
    "sample_layer",
    "sampled_engine",
    "gearbox",
    "v10_filtered",
    "gearbox_filtered",
    "hybrid_mix",
    // PHY-140 physical diagnostic stems
    "cylinder_pressure",
    "chamber_temperature",
    "chamber_heat_release",
    "chamber_phase",
    "combustion_heat_release",
    "valve_area",
    "mass_flow",
    "runner_pressure",
    "runner_acoustic_pressure",
    "physical_master",
    "sample_master",
    "hybrid_master",
];

const CYLINDER_STEM_NAMES: [&str; CYLINDER_COUNT] = [
    "cylinder_mechanical_0",
    "cylinder_mechanical_1",
    "cylinder_mechanical_2",
    "cylinder_mechanical_3",
    "cylinder_mechanical_4",
    "cylinder_mechanical_5",
    "cylinder_mechanical_6",
    "cylinder_mechanical_7",
    "cylinder_mechanical_8",
    "cylinder_mechanical_9",
];

/// Accumulated offline render results returned by [`crate::offline::run_offline_render`].
#[derive(Clone, Debug, PartialEq)]
pub struct OfflineRenderSummary {
    pub out: PathBuf,
    pub stems_dir: PathBuf,
    pub metadata_path: PathBuf,
    pub events_path: PathBuf,
    pub event_count: u64,
    pub peak: f32,
    pub rms: f64,
    pub worst_limiter_reduction_db: f32,
    pub frames: usize,
}

/// Stateful recorder collecting all 61 diagnostic stems, firing events, physical
/// cylinder-0 CSV telemetry, and metadata summaries.
pub struct StemRecorder {
    total_frames: usize,
    sample_rate: u32,
    cylinder_count: usize,
    bank_split: usize,
    engine_mode: EngineMode,
    has_sample_layer: bool,
    acoustic_scene: bool,
    stems: BTreeMap<&'static str, Vec<f32>>,
    events_path: PathBuf,
    events_writer: BufWriter<File>,
    phys_writer: Option<BufWriter<File>>,
    physical_master_lp: Option<ThreePoleLowPass>,
    source_labels: Vec<String>,
    source_weight_sums: Vec<f64>,
    sample_tonal_sq: f64,
    sample_residual_sq: f64,
    sample_off_sq: f64,
    peak: f32,
    sum_sq: f64,
    worst_reduction: f32,
    event_count: u64,
}

impl StemRecorder {
    /// Creates output directories, initializes CSV writers, and preallocates all 61 stem buffers.
    pub fn new(
        config: &OfflineRenderConfig,
        total_frames: usize,
        source_labels: &[String],
        has_sample_layer: bool,
    ) -> Result<Self, String> {
        let sample_rate = config.runtime.engine.sample_rate;
        fs::create_dir_all(&config.stems_dir).map_err(|e| e.to_string())?;
        let events_path = config.out.with_extension("events.csv");
        let events_writer = create_csv_writer(
            &events_path,
            "sample,time_s,rpm,crank_phase_deg,cylinder,bank",
        )?;
        let phys_writer = config
            .physical_telemetry_csv
            .as_deref()
            .map(|path| {
                create_csv_writer(
                    path,
                    "sample,time_s,rpm,crank_phase_deg,cylinder_id,chamber_pressure_pa,chamber_temperature_k,heat_release_rate,phase,valve_lift,valve_area_m2,mass_flow_kg_s,runner_pressure_pa,runner_temperature_k,runner_acoustic_pa,collector_pressure_a,collector_pressure_b",
                )
            })
            .transpose()?;
        let physical_master_lp = config.physical_master_lowpass_hz.map(|cutoff| {
            ThreePoleLowPass::new(
                cutoff.clamp(100.0, sample_rate as f32 * 0.48),
                sample_rate as f32,
            )
        });
        let stems = STEM_NAMES
            .iter()
            .map(|&name| (name, Vec::with_capacity(total_frames)))
            .collect();
        Ok(Self {
            total_frames,
            sample_rate,
            cylinder_count: config.runtime.engine.cylinder_count,
            bank_split: config.runtime.engine.bank_split,
            engine_mode: config.runtime.engine_mode,
            has_sample_layer,
            acoustic_scene: config.acoustic_scene,
            stems,
            events_path,
            events_writer,
            phys_writer,
            physical_master_lp,
            source_labels: source_labels.to_vec(),
            source_weight_sums: vec![0.0f64; source_labels.len()],
            sample_tonal_sq: 0.0,
            sample_residual_sq: 0.0,
            sample_off_sq: 0.0,
            peak: 0.0,
            sum_sq: 0.0,
            worst_reduction: 0.0,
            event_count: 0,
        })
    }

    #[inline]
    fn push(&mut self, name: &str, value: f32) {
        self.stems
            .get_mut(name)
            .expect("registered stem name")
            .push(value);
    }

    /// Appends one rendered sample trace, updates RMS/peak statistics, and writes CSV rows.
    pub fn record_sample(
        &mut self,
        sample: usize,
        time_s: f32,
        rpm: f32,
        trace: &RenderTrace,
    ) -> Result<(), String> {
        let frame = &trace.engine.engine_frame;
        let acoustic = &trace.engine.acoustic_frame;
        self.push("combustion_source", frame.combustion_source);
        self.push("pressure_derivative", frame.pressure_derivative);
        self.push("pressure_derivative_a", frame.pressure_derivative_a);
        self.push("pressure_derivative_b", frame.pressure_derivative_b);
        self.push("pressure_direct", frame.pressure_direct);
        self.push("crankcase", frame.crankcase);
        self.push("block", frame.block);
        self.push("head", frame.head);
        self.push("block_head", frame.block_head);
        self.push("headers_a", frame.headers_a);
        self.push("headers_b", frame.headers_b);
        self.push("collector_a", frame.collector_a);
        self.push("collector_b", frame.collector_b);
        self.push("collector_pressure_a", frame.collector_pressure_a);
        self.push("collector_pressure_b", frame.collector_pressure_b);
        self.push("exhaust", frame.exhaust);
        self.push("turbulence", frame.turbulence);
        self.push("master_pre_limiter", frame.master_pre_limiter);
        self.push("master", frame.master);

        self.push("engine_dry", acoustic.engine_dry);
        self.push("engine_air", acoustic.engine_air);
        self.push("engine_cover", acoustic.engine_cover);
        self.push("engine_mounts", acoustic.engine_mounts);
        self.push("monocoque_seat", acoustic.monocoque_seat);
        self.push("mount_monocoque", acoustic.mount_monocoque);
        self.push("cylinder_mechanical_sum", acoustic.cylinder_mechanical_sum);
        for (index, name) in CYLINDER_STEM_NAMES.iter().enumerate() {
            self.push(name, acoustic.cylinder_mechanical[index]);
        }
        self.push("scene_mix", acoustic.output);

        self.push("transmission", trace.transmission);
        self.push("gear_shift", trace.gear_shift);
        self.push("gearbox", trace.gearbox_raw);

        let (sample_tonal, sample_residual, sample_max_rasp, sample_off_throttle, sample_output) =
            if trace.engine.has_sample_frame {
                let sf = &trace.engine.sample_frame;
                for (slot, weight) in self
                    .source_weight_sums
                    .iter_mut()
                    .zip(sf.zone_weights.iter())
                {
                    *slot += *weight as f64;
                }
                self.sample_tonal_sq += (sf.tonal * sf.tonal) as f64;
                self.sample_residual_sq += (sf.residual * sf.residual) as f64;
                self.sample_off_sq += (sf.off_throttle * sf.off_throttle) as f64;
                (
                    sf.tonal,
                    sf.residual,
                    sf.max_rasp,
                    sf.off_throttle,
                    sf.output,
                )
            } else {
                (0.0, 0.0, 0.0, 0.0, 0.0)
            };

        let sample_only = self.engine_mode == EngineMode::SampleOnly;
        self.push(
            "sampled_engine",
            if sample_only { frame.master } else { 0.0 },
        );
        self.push("sample_tonal", sample_tonal);
        self.push("sample_residual", sample_residual);
        self.push("sample_max_rasp", sample_max_rasp);
        self.push("sample_off_throttle", sample_off_throttle);
        self.push("sample_layer", sample_output);
        self.push("v10_filtered", trace.v10_filtered);
        self.push("gearbox_filtered", trace.gearbox_filtered);
        self.push("hybrid_mix", trace.hybrid_mix);

        let inv_cyl = 1.0 / self.cylinder_count as f32;
        let cyl_n = self.cylinder_count;
        self.push(
            "cylinder_pressure",
            frame.cylinder_chamber_pressure_pa[..cyl_n]
                .iter()
                .sum::<f32>()
                * inv_cyl,
        );
        self.push(
            "chamber_temperature",
            frame.cylinder_chamber_temperature_k[..cyl_n]
                .iter()
                .sum::<f32>()
                * inv_cyl,
        );
        self.push(
            "chamber_heat_release",
            frame.cylinder_chamber_heat_release_rate[..cyl_n]
                .iter()
                .sum::<f32>()
                * inv_cyl,
        );
        self.push("chamber_phase", frame.cylinder_chamber_phase[0] as f32);
        self.push("combustion_heat_release", frame.pressure_derivative);
        self.push(
            "valve_area",
            frame.cylinder_valve_area[..cyl_n].iter().sum::<f32>(),
        );
        self.push(
            "mass_flow",
            frame.cylinder_mass_flow[..cyl_n].iter().sum::<f32>(),
        );
        self.push(
            "runner_pressure",
            frame.cylinder_runner_pressure[..cyl_n].iter().sum::<f32>() * inv_cyl,
        );
        self.push(
            "runner_acoustic_pressure",
            frame.cylinder_runner_acoustic_pressure[..cyl_n]
                .iter()
                .sum::<f32>(),
        );

        let filtered_physical_master = if let Some(lp) = self.physical_master_lp.as_mut() {
            lp.process(frame.master)
        } else {
            frame.master
        };
        self.push("physical_master", filtered_physical_master);
        self.push("sample_master", sample_output);
        self.push("hybrid_master", trace.hybrid_mix);

        if let Some(writer) = self.phys_writer.as_mut() {
            writeln!(
                writer,
                "{sample},{time_s:.6},{rpm:.2},{crank_phase_deg:.2},{cyl},{chamber_p:.4e},{chamber_t:.2},{heat:.6e},{phase},{lift:.6},{area:.6e},{flow:.6e},{runner_p:.4e},{runner_t:.2},{acoustic_p:.6e},{col_a:.4e},{col_b:.4e}",
                sample = sample,
                time_s = time_s,
                rpm = rpm,
                crank_phase_deg = frame.crank_phase_deg,
                cyl = 0,
                chamber_p = frame.cylinder_chamber_pressure_pa[0],
                chamber_t = frame.cylinder_chamber_temperature_k[0],
                heat = frame.cylinder_chamber_heat_release_rate[0],
                phase = frame.cylinder_chamber_phase[0],
                lift = frame.cylinder_valve_lift[0],
                area = frame.cylinder_valve_area[0],
                flow = frame.cylinder_mass_flow[0],
                runner_p = frame.cylinder_runner_pressure[0],
                runner_t = frame.cylinder_runner_temperature_k[0],
                acoustic_p = frame.cylinder_runner_acoustic_pressure[0],
                col_a = frame.collector_pressure_a,
                col_b = frame.collector_pressure_b,
            )
            .map_err(|e| e.to_string())?;
        }

        let rendered = if self.has_sample_layer {
            trace.hybrid_mix
        } else if self.acoustic_scene {
            acoustic.output
        } else {
            filtered_physical_master
        };
        self.peak = self.peak.max(rendered.abs());
        self.sum_sq += (rendered * rendered) as f64;
        self.worst_reduction = self.worst_reduction.min(frame.limiter_reduction_db);

        if frame.fired_mask != 0 {
            for cylinder in 0..self.cylinder_count {
                if frame.fired_mask & (1 << cylinder) != 0 {
                    self.event_count += 1;
                    let bank = if cylinder < self.bank_split { "A" } else { "B" };
                    writeln!(
                        self.events_writer,
                        "{sample},{:.9},{:.3},{:.6},{cylinder},{bank}",
                        sample as f64 / self.sample_rate as f64,
                        rpm,
                        frame.crank_phase_deg,
                    )
                    .map_err(|e| e.to_string())?;
                }
            }
        }
        Ok(())
    }

    /// Flushes CSV telemetry, writes primary and stem PCM16 WAVs, and writes `.metadata.json`.
    pub fn finish(mut self, config: &OfflineRenderConfig) -> Result<OfflineRenderSummary, String> {
        self.events_writer.flush().map_err(|e| e.to_string())?;
        if let Some(writer) = self.phys_writer.as_mut() {
            writer.flush().map_err(|e| e.to_string())?;
        }

        let output_stem = if self.has_sample_layer {
            "hybrid_mix"
        } else if self.acoustic_scene {
            "scene_mix"
        } else {
            "master"
        };
        let main_samples = self
            .stems
            .get(output_stem)
            .ok_or_else(|| format!("missing primary output stem {output_stem}"))?;
        write_mono_pcm16(&config.out, self.sample_rate, main_samples).map_err(|e| e.to_string())?;
        for (name, samples) in &self.stems {
            write_mono_pcm16(
                &config.stems_dir.join(format!("{name}.wav")),
                self.sample_rate,
                samples,
            )
            .map_err(|e| e.to_string())?;
        }

        let total_f64 = self.total_frames.max(1) as f64;
        let rms = (self.sum_sq / total_f64).sqrt();
        let metadata_path = config.out.with_extension("metadata.json");
        let metadata = build_metadata_json(
            config,
            self.has_sample_layer,
            self.event_count,
            self.peak,
            rms,
            self.worst_reduction,
            &self.source_labels,
            &self.source_weight_sums,
            (self.sample_tonal_sq / total_f64).sqrt(),
            (self.sample_residual_sq / total_f64).sqrt(),
            (self.sample_off_sq / total_f64).sqrt(),
            total_f64,
        );
        if metadata_path.exists() {
            let _ = fs::remove_file(&metadata_path);
        }
        fs::write(&metadata_path, metadata).map_err(|e| e.to_string())?;

        Ok(OfflineRenderSummary {
            out: config.out.clone(),
            stems_dir: config.stems_dir.clone(),
            metadata_path,
            events_path: self.events_path,
            event_count: self.event_count,
            peak: self.peak,
            rms,
            worst_limiter_reduction_db: self.worst_reduction,
            frames: self.total_frames,
        })
    }
}

fn create_csv_writer(path: &Path, header: &str) -> Result<BufWriter<File>, String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    if path.exists() {
        let _ = fs::remove_file(path);
    }
    let file = File::create(path).map_err(|e| e.to_string())?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "{header}").map_err(|e| e.to_string())?;
    Ok(writer)
}

fn collector_geometry_label(config: &OfflineRenderConfig) -> String {
    let Some(geo) = config.runtime.engine.collector_geometry.as_ref() else {
        return "off".into();
    };
    format!(
        "volume_l={:.4},outlet_length_m={:.4},outlet_diameter_m={:.4},gas_temperature_k={:.1},loss_fraction_per_cycle={:.4},mode_coupling={:.4}",
        geo.volume_l,
        geo.outlet_length_m,
        geo.outlet_diameter_m,
        geo.gas_temperature_k,
        geo.loss_fraction_per_cycle,
        geo.mode_coupling,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_metadata_json(
    config: &OfflineRenderConfig,
    has_sample_layer: bool,
    event_count: u64,
    peak: f32,
    rms: f64,
    worst_reduction: f32,
    source_labels: &[String],
    source_weight_sums: &[f64],
    sample_tonal_rms: f64,
    sample_residual_rms: f64,
    sample_off_rms: f64,
    total_frames: f64,
) -> String {
    let exhaust_gain_label = if config.exhaust_gain_overridden {
        format!("{:.6}", config.runtime.engine.exhaust_gain)
    } else {
        "baseline".into()
    };
    let sample_layer_bank = config
        .runtime
        .sample_layer_directory
        .as_ref()
        .and_then(|path| path.file_name())
        .and_then(|name| name.to_str())
        .unwrap_or("none");
    let sample_layer_source_weights = source_labels
        .iter()
        .zip(source_weight_sums.iter())
        .map(|(label, sum)| format!("{label}={:.6}", sum / total_frames))
        .collect::<Vec<_>>()
        .join(",");
    let sample_layer_tuning = format!(
        "sample_blend_weight={},physical_blend_weight={},zone_trim_db={}",
        config
            .sample_blend_weight_override
            .map(|value| format!("{value}"))
            .unwrap_or_else(|| "baseline".into()),
        config
            .physical_blend_weight_override
            .map(|value| format!("{value}"))
            .unwrap_or_else(|| "baseline".into()),
        if config.sample_zone_trim_db.is_empty() {
            "baseline".into()
        } else {
            config
                .sample_zone_trim_db
                .iter()
                .map(|value| format!("{value}"))
                .collect::<Vec<_>>()
                .join("|")
        }
    );
    let gear_ratios_label = config
        .runtime
        .transmission
        .gear_ratios
        .iter()
        .map(|value| format!("{value}"))
        .collect::<Vec<_>>()
        .join(",");
    let sample_residual_gain = config
        .sample_residual_gain_override
        .map(|gain| format!("{gain}"))
        .unwrap_or_else(|| "load-blended".into());

    #[derive(Serialize)]
    struct RenderMetadataJson<'a> {
        architecture: &'static str,
        git_head: String,
        source_dirty: bool,
        source_fingerprint: String,
        sample_rate: u32,
        seed: u64,
        header_length_scale: f32,
        exhaust_gain: &'a str,
        collector_geometry: String,
        profile: &'static str,
        rpm: f32,
        peak_rpm: f32,
        coast_end_rpm: f32,
        hold_before_lift: bool,
        accel_seconds: f32,
        throttle: f32,
        load: f32,
        duration_s: f32,
        warmup_s: f32,
        shift_sequence: String,
        event_count: u64,
        peak: f32,
        rms: f64,
        worst_limiter_reduction_db: f32,
        legacy_synth_used: bool,
        cpp_used: bool,
        faust_used: bool,
        acoustic_scene_used: bool,
        sample_layer_used: bool,
        sample_rasp_disabled: bool,
        sample_residual_scale: f32,
        sample_residual_gain: &'a str,
        sample_layer_tuning: &'a str,
        scene_gain_overrides: &'a str,
        sample_layer_bank: &'a str,
        engine_mode: &'static str,
        sampled_engine_gain: f32,
        sampled_gearbox_gain: f32,
        sample_layer_source_weights: &'a str,
        sample_tonal_rms: f64,
        sample_residual_rms: f64,
        sample_off_rms: f64,
        hybrid_headroom_gain: f32,
        upper_mid_shelf_gain: f32,
        transmission_gain: f32,
        transmission_gear: i8,
        transmission_gear_ratios: &'a str,
        transmission_final_drive: f32,
        gear_shift_gain: f32,
        gear_shift_reference_rpm: f32,
        chamber_cycle_model: &'static str,
        chamber_phase_contract: &'static str,
        firing_order: Vec<usize>,
        sample_mid_architecture: &'static str,
    }

    let metadata = RenderMetadataJson {
        architecture: "rust-v10-v2",
        git_head: git_head(),
        source_dirty: source_dirty(),
        source_fingerprint: source_fingerprint(),
        sample_rate: config.runtime.engine.sample_rate,
        seed: config.runtime.engine.seed,
        header_length_scale: config.runtime.engine.header_length_scale,
        exhaust_gain: &exhaust_gain_label,
        collector_geometry: collector_geometry_label(config),
        profile: config.scenario.profile_label(),
        rpm: config.scenario.rpm,
        peak_rpm: config.scenario.peak_rpm(),
        coast_end_rpm: config.scenario.metadata_coast_end_rpm(),
        hold_before_lift: config.scenario.hold_before_lift,
        accel_seconds: config.scenario.accel_seconds,
        throttle: config.scenario.throttle,
        load: config.scenario.load,
        duration_s: config.scenario.seconds,
        warmup_s: config.scenario.warmup,
        shift_sequence: shift_sequence_label(&config.scenario.shift_sequence),
        event_count,
        peak,
        rms,
        worst_limiter_reduction_db: worst_reduction,
        legacy_synth_used: false,
        cpp_used: false,
        faust_used: false,
        acoustic_scene_used: config.acoustic_scene,
        sample_layer_used: has_sample_layer,
        sample_rasp_disabled: config.runtime.sample_layer.disable_sample_rasp,
        sample_residual_scale: config.runtime.sample_layer.residual_gain_scale,
        sample_residual_gain: &sample_residual_gain,
        sample_layer_tuning: &sample_layer_tuning,
        scene_gain_overrides: &config.scene_gain_overrides_label,
        sample_layer_bank,
        engine_mode: match config.runtime.engine_mode {
            EngineMode::Hybrid => "hybrid",
            EngineMode::SampleOnly => "sample_only",
        },
        sampled_engine_gain: config.runtime.sampled_engine_gain,
        sampled_gearbox_gain: config.runtime.sampled_gearbox_gain,
        sample_layer_source_weights: &sample_layer_source_weights,
        sample_tonal_rms,
        sample_residual_rms,
        sample_off_rms,
        hybrid_headroom_gain: GF509_HEADROOM_GAIN,
        upper_mid_shelf_gain: config.runtime.upper_mid_shelf_gain,
        transmission_gain: config.runtime.transmission.output_gain,
        transmission_gear: config.gear,
        transmission_gear_ratios: &gear_ratios_label,
        transmission_final_drive: config.runtime.transmission.final_drive,
        gear_shift_gain: config.runtime.gear_shift_gain,
        gear_shift_reference_rpm: config.runtime.gear_shift_reference_rpm,
        chamber_cycle_model: "bounded_720deg_four_stroke_fresh_charge",
        chamber_phase_contract:
            "expansion_0_180,exhaust_180_360,intake_360_540,compression_540_720",
        firing_order: config.runtime.engine.firing_order[..config.runtime.engine.cylinder_count]
            .to_vec(),
        sample_mid_architecture: "zone-specific post gains + deeper max residual compression/saturation + max rasp at +2.5 dB + reduced engine/scene drift + perceptual max crossfade 10000-13750 RPM",
    };
    let mut rendered =
        serde_json::to_string_pretty(&metadata).expect("offline metadata serializes cleanly");
    rendered.push('\n');
    rendered
}
