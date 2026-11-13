use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::f64::consts::TAU;
use std::fs;
use std::path::{Path, PathBuf};
use v10_engine_synth::wav::write_mono_pcm16;
use v10_engine_synth::{
    AcousticScene, AcousticSceneConfig, EngineConfig, EngineInput, EngineMode, Gf509Runtime,
    Gf509RuntimeConfig, RuntimeTelemetry, SampleLayerFrame, SampleLayerInput, ShiftPhase,
    ThreeZoneSampleLayer, ThreeZoneSampleLayerConfig, TorqueSign, TransmissionConfig, V10Engine,
};

const SAMPLE_RATE: u32 = 48_000;
const GF509_HEADROOM_GAIN: f32 = 0.61;
const RESAMPLED_TEST_RATE: u32 = 44_100;

#[derive(Serialize)]
struct PreparedFilesJson {
    loop_tonal: String,
    loop_residual: String,
}

#[derive(Serialize)]
struct PreparedMetadataJson {
    schema_version: u32,
    sample_rate: u32,
    rpm_anchor: f32,
    firing_phase_degrees_at_loop_start: f32,
    loop_length_samples: usize,
    files: PreparedFilesJson,
}

#[derive(Serialize)]
struct Schema1EntryJson {
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

#[derive(Serialize)]
struct Schema1ManifestJson {
    schema_version: u32,
    key: String,
    output_gain: f32,
    samples: Vec<Schema1EntryJson>,
}

#[derive(Serialize)]
struct Schema2VariantJson {
    group: String,
    position: usize,
    count: usize,
}

#[derive(Serialize)]
struct Schema2EntryJson {
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
    #[serde(skip_serializing_if = "Option::is_none")]
    variant: Option<Schema2VariantJson>,
}

#[derive(Serialize)]
struct Schema2ManifestJson {
    schema_version: u32,
    key: String,
    output_gain: f32,
    samples: Vec<Schema2EntryJson>,
    off_samples: Vec<Schema2EntryJson>,
}

#[derive(Serialize, Deserialize)]
struct GoldenScenarioRecord {
    name: String,
    category: String,
    sample_rate: u32,
    channels: u16,
    total_frames: usize,
    raw_f32_path: String,
    raw_f32_sha256: String,
    wav_path: Option<String>,
    wav_sha256: Option<String>,
    rms: f64,
    peak: f32,
    dc_mean: f64,
    finite: bool,
    extra: BTreeMap<String, serde_json::Value>,
}

#[derive(Serialize, Deserialize)]
struct GoldenManifest {
    baseline_crate: String,
    sample_rate: u32,
    fixtures: BTreeMap<String, String>,
    scenarios: Vec<GoldenScenarioRecord>,
}

#[derive(Serialize, Deserialize)]
struct CliGoldenEntry {
    name: String,
    wav_path: String,
    wav_sha256: String,
    sample_rate: u32,
    channels: u16,
    total_frames: usize,
    rms: f64,
    peak: f32,
    finite: bool,
}

#[derive(Serialize, Deserialize)]
struct CliGoldenManifest {
    binary: String,
    entries: Vec<CliGoldenEntry>,
}

struct ZoneSpec<'a> {
    key: &'a str,
    role: &'a str,
    native_rpm: f32,
    loop_frames: usize,
    firing_phase_deg: f32,
    seed: u64,
    variant: Option<(&'a str, usize, usize)>,
}

struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self { state: seed.max(1) }
    }

    fn next_unit_bipolar(&mut self) -> f32 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        let mantissa = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40;
        (mantissa as f32 / ((1u64 << 24) - 1) as f32) * 2.0 - 1.0
    }
}

fn sha256_bytes(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn sha256_file(path: &Path) -> Result<String, String> {
    let bytes = fs::read(path).map_err(|e| format!("failed reading {}: {e}", path.display()))?;
    Ok(sha256_bytes(&bytes))
}

fn synthesize_loop_pair(spec: &ZoneSpec<'_>, sample_rate: u32) -> (Vec<f32>, Vec<f32>) {
    let len = spec.loop_frames;
    let phase_offset = (spec.firing_phase_deg as f64).to_radians();
    // Choose integer fundamental cycle counts close to physical V10 firing frequency
    // so every loop wraps smoothly with zero discontinuity.
    let target_firing_hz = spec.native_rpm as f64 * 5.0 / 60.0;
    let duration_s = len as f64 / sample_rate as f64;
    let firing_cycles = (target_firing_hz * duration_s).round().max(5.0);
    // Keep half-order & bank harmonics aligned with even sub-multiples over the full loop.
    let base_cycles = (firing_cycles / 5.0).round().max(1.0);

    let mut tonal = Vec::with_capacity(len);
    for i in 0..len {
        let t_norm = i as f64 / len as f64;
        let crank_angle = TAU * base_cycles * t_norm + phase_offset;
        // Rich harmonic series: fundamental firing (5x), bank asymmetry (2.5x/1x), 10x, 15x
        let s = 0.20 * (5.0 * crank_angle).sin()
            + 0.085 * (10.0 * crank_angle + 0.35).sin()
            + 0.045 * (15.0 * crank_angle + 0.70).sin()
            + 0.035 * (2.5 * crank_angle + 0.15).sin()
            + 0.025 * (1.0 * crank_angle + 0.90).sin();
        tonal.push(s as f32);
    }

    let mut rng = XorShift64::new(spec.seed);
    let mut raw_res = Vec::with_capacity(len);
    let mut lp = 0.0f32;
    for i in 0..len {
        let white = rng.next_unit_bipolar();
        lp += 0.22 * (white - lp);
        let hp_component = white - lp;
        let mod_env = 0.75 + 0.25 * (TAU * firing_cycles * (i as f64 / len as f64)).cos() as f32;
        raw_res.push((0.55 * lp + 0.45 * hp_component) * mod_env * 0.24);
    }
    // Enforce seamless loop endpoints via tiny edge cross-smoothing across first/last 16 samples
    let mut residual = raw_res.clone();
    let fade = 16usize.min(len / 4);
    for k in 0..fade {
        let alpha = k as f32 / fade as f32;
        let end_idx = len - fade + k;
        let blended = raw_res[end_idx] * (1.0 - alpha) + raw_res[k] * alpha;
        residual[k] = raw_res[k] * alpha + blended * (1.0 - alpha);
    }

    (tonal, residual)
}

fn write_gear_shift_wavs(dir: &Path, sample_rate: u32) -> Result<(), String> {
    let up_len = (sample_rate as usize / 10).max(256); // 100 ms
    let mut up = Vec::with_capacity(up_len);
    let mut rng = XorShift64::new(0x6E42_0001_u64);
    for i in 0..up_len {
        let t = i as f32 / sample_rate as f32;
        let env = (-t / 0.018).exp() * (1.0 - (-t / 0.0015).exp());
        let body = (TAU as f32 * 420.0 * t).sin() * 0.55
            + (TAU as f32 * 1_180.0 * t).sin() * 0.30
            + rng.next_unit_bipolar() * 0.25;
        up.push((body * env * 0.65).clamp(-0.95, 0.95));
    }
    write_mono_pcm16(&dir.join("gearup.wav"), sample_rate, &up)?;

    let dn_len = (sample_rate as usize / 8).max(256); // 125 ms
    let mut dn = Vec::with_capacity(dn_len);
    for i in 0..dn_len {
        let t = i as f32 / sample_rate as f32;
        let env = (-t / 0.028).exp() * (1.0 - (-t / 0.0020).exp());
        let freq = 260.0 + 380.0 * (-t / 0.035).exp();
        let body = (TAU as f32 * freq * t).sin() * 0.60
            + (TAU as f32 * freq * 2.0 * t).sin() * 0.25
            + rng.next_unit_bipolar() * 0.20;
        dn.push((body * env * 0.65).clamp(-0.95, 0.95));
    }
    write_mono_pcm16(&dir.join("geardn.wav"), sample_rate, &dn)?;
    Ok(())
}

fn write_zone_assets(
    dir: &Path,
    spec: &ZoneSpec<'_>,
    metadata_name: &str,
    sample_rate: u32,
) -> Result<(String, String, String, String), String> {
    let tonal_name = format!("{}_tonal.wav", spec.key);
    let residual_name = format!("{}_residual.wav", spec.key);
    let (tonal, residual) = synthesize_loop_pair(spec, sample_rate);

    let tonal_path = dir.join(&tonal_name);
    let residual_path = dir.join(&residual_name);
    write_mono_pcm16(&tonal_path, sample_rate, &tonal)?;
    write_mono_pcm16(&residual_path, sample_rate, &residual)?;

    let tonal_sha = sha256_file(&tonal_path)?;
    let residual_sha = sha256_file(&residual_path)?;

    let meta = PreparedMetadataJson {
        schema_version: 1,
        sample_rate,
        rpm_anchor: spec.native_rpm,
        firing_phase_degrees_at_loop_start: spec.firing_phase_deg,
        loop_length_samples: spec.loop_frames,
        files: PreparedFilesJson {
            loop_tonal: tonal_name.clone(),
            loop_residual: residual_name.clone(),
        },
    };
    let meta_json = serde_json::to_string_pretty(&meta)
        .map_err(|e| format!("serialize metadata failed: {e}"))?;
    fs::write(dir.join(metadata_name), meta_json)
        .map_err(|e| format!("write metadata failed: {e}"))?;

    Ok((tonal_name, tonal_sha, residual_name, residual_sha))
}

fn generate_schema1_bank(dir: &Path) -> Result<(), String> {
    if dir.exists() {
        fs::remove_dir_all(dir).map_err(|e| format!("clean {}: {e}", dir.display()))?;
    }
    fs::create_dir_all(dir).map_err(|e| format!("mkdir {}: {e}", dir.display()))?;

    let on_specs = [
        ZoneSpec {
            key: "low",
            role: "on",
            native_rpm: 4_200.0,
            loop_frames: 4_800,
            firing_phase_deg: 0.0,
            seed: 0x1050_0001,
            variant: None,
        },
        ZoneSpec {
            key: "med",
            role: "on",
            native_rpm: 5_800.0,
            loop_frames: 4_096,
            firing_phase_deg: 18.0,
            seed: 0x1050_0002,
            variant: None,
        },
        ZoneSpec {
            key: "max",
            role: "on",
            native_rpm: 11_500.0,
            loop_frames: 3_600,
            firing_phase_deg: 36.0,
            seed: 0x1050_0003,
            variant: None,
        },
    ];

    let mut manifest_samples = Vec::new();
    for spec in &on_specs {
        let meta_name = format!("{}.sample-layer.json", spec.key);
        let (tonal, tonal_sha, residual, residual_sha) =
            write_zone_assets(dir, spec, &meta_name, SAMPLE_RATE)?;
        manifest_samples.push(Schema1EntryJson {
            key: spec.key.to_string(),
            metadata: meta_name,
            native_rpm: spec.native_rpm,
            sample_rate: SAMPLE_RATE,
            channels: 1,
            loop_frames: spec.loop_frames,
            tonal,
            tonal_sha256: tonal_sha,
            residual,
            residual_sha256: residual_sha,
        });
    }

    let off_spec = ZoneSpec {
        key: "overrun",
        role: "off",
        native_rpm: 6_500.0,
        loop_frames: 4_320,
        firing_phase_deg: 12.0,
        seed: 0x1050_0004,
        variant: None,
    };
    write_zone_assets(dir, &off_spec, "overrun.off-layer.json", SAMPLE_RATE)?;
    write_gear_shift_wavs(dir, SAMPLE_RATE)?;

    let manifest = Schema1ManifestJson {
        schema_version: 1,
        key: "v10_gf509".to_string(),
        output_gain: GF509_HEADROOM_GAIN,
        samples: manifest_samples,
    };
    let raw = serde_json::to_string_pretty(&manifest)
        .map_err(|e| format!("serialize schema1 manifest: {e}"))?;
    fs::write(dir.join("manifest.json"), raw)
        .map_err(|e| format!("write schema1 manifest: {e}"))?;
    Ok(())
}

fn generate_schema2_bank(dir: &Path) -> Result<(), String> {
    if dir.exists() {
        fs::remove_dir_all(dir).map_err(|e| format!("clean {}: {e}", dir.display()))?;
    }
    fs::create_dir_all(dir).map_err(|e| format!("mkdir {}: {e}", dir.display()))?;

    let specs = [
        ZoneSpec {
            key: "on_low",
            role: "on",
            native_rpm: 3_800.0,
            loop_frames: 4_800,
            firing_phase_deg: 0.0,
            seed: 0x2050_0001,
            variant: None,
        },
        ZoneSpec {
            key: "on_mid_a",
            role: "on",
            native_rpm: 6_200.0,
            loop_frames: 4_096,
            firing_phase_deg: 10.0,
            seed: 0x2050_0002,
            variant: Some(("mid_pair", 0, 2)),
        },
        ZoneSpec {
            key: "on_mid_b",
            role: "on",
            native_rpm: 7_400.0,
            loop_frames: 3_840,
            firing_phase_deg: 28.0,
            seed: 0x2050_0003,
            variant: Some(("mid_pair", 1, 2)),
        },
        ZoneSpec {
            key: "on_high",
            role: "on",
            native_rpm: 12_500.0,
            loop_frames: 3_200,
            firing_phase_deg: 44.0,
            seed: 0x2050_0004,
            variant: None,
        },
        ZoneSpec {
            key: "off_low",
            role: "off",
            native_rpm: 4_500.0,
            loop_frames: 4_600,
            firing_phase_deg: 5.0,
            seed: 0x2050_0005,
            variant: None,
        },
        ZoneSpec {
            key: "off_high",
            role: "off",
            native_rpm: 9_500.0,
            loop_frames: 3_600,
            firing_phase_deg: 22.0,
            seed: 0x2050_0006,
            variant: None,
        },
    ];

    let mut samples = Vec::new();
    let mut off_samples = Vec::new();
    for spec in &specs {
        let meta_name = format!("{}.json", spec.key);
        let (tonal, tonal_sha, residual, residual_sha) =
            write_zone_assets(dir, spec, &meta_name, SAMPLE_RATE)?;
        let entry = Schema2EntryJson {
            key: spec.key.to_string(),
            role: spec.role.to_string(),
            metadata: meta_name,
            native_rpm: spec.native_rpm,
            sample_rate: SAMPLE_RATE,
            channels: 1,
            loop_frames: spec.loop_frames,
            tonal,
            tonal_sha256: tonal_sha,
            residual,
            residual_sha256: residual_sha,
            variant: spec
                .variant
                .map(|(group, position, count)| Schema2VariantJson {
                    group: group.to_string(),
                    position,
                    count,
                }),
        };
        if spec.role == "on" {
            samples.push(entry);
        } else {
            off_samples.push(entry);
        }
    }

    write_gear_shift_wavs(dir, SAMPLE_RATE)?;

    let manifest = Schema2ManifestJson {
        schema_version: 2,
        key: "v10_f2002_sampled".to_string(),
        output_gain: GF509_HEADROOM_GAIN,
        samples,
        off_samples,
    };
    let raw = serde_json::to_string_pretty(&manifest)
        .map_err(|e| format!("serialize schema2 manifest: {e}"))?;
    fs::write(dir.join("manifest.json"), raw)
        .map_err(|e| format!("write schema2 manifest: {e}"))?;
    Ok(())
}

fn stats(samples: &[f32]) -> (f64, f32, f64, bool) {
    let mut sum_sq = 0.0f64;
    let mut sum = 0.0f64;
    let mut peak = 0.0f32;
    let mut finite = true;
    for &s in samples {
        if !s.is_finite() {
            finite = false;
            continue;
        }
        let abs = s.abs();
        if abs > peak {
            peak = abs;
        }
        let d = s as f64;
        sum += d;
        sum_sq += d * d;
    }
    let n = samples.len().max(1) as f64;
    ((sum_sq / n).sqrt(), peak, sum / n, finite)
}

fn f32_slice_to_le_bytes(samples: &[f32]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(samples.len() * 4);
    for &s in samples {
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    bytes
}

struct GoldenOutputSpec<'a> {
    name: &'a str,
    category: &'a str,
    sample_rate: u32,
    samples: &'a [f32],
    write_wav: bool,
    extra: BTreeMap<String, serde_json::Value>,
}

fn save_golden_signal(
    golden_dir: &Path,
    v2_root: &Path,
    spec: GoldenOutputSpec<'_>,
) -> Result<GoldenScenarioRecord, String> {
    let raw_dir = golden_dir.join("raw");
    let wav_dir = golden_dir.join("wav");
    fs::create_dir_all(&raw_dir).map_err(|e| format!("mkdir raw: {e}"))?;
    fs::create_dir_all(&wav_dir).map_err(|e| format!("mkdir wav: {e}"))?;

    let raw_path = raw_dir.join(format!("{}.f32", spec.name));
    let raw_bytes = f32_slice_to_le_bytes(spec.samples);
    fs::write(&raw_path, &raw_bytes).map_err(|e| format!("write {}: {e}", raw_path.display()))?;
    let raw_sha = sha256_bytes(&raw_bytes);

    let (wav_rel, wav_sha) = if spec.write_wav {
        let wav_path = wav_dir.join(format!("{}.wav", spec.name));
        write_mono_pcm16(&wav_path, spec.sample_rate, spec.samples)?;
        let sha = sha256_file(&wav_path)?;
        let rel = wav_path
            .strip_prefix(v2_root)
            .map_err(|e| format!("strip prefix: {e}"))?
            .to_string_lossy()
            .replace('\\', "/");
        (Some(rel), Some(sha))
    } else {
        (None, None)
    };

    let raw_rel = raw_path
        .strip_prefix(v2_root)
        .map_err(|e| format!("strip prefix: {e}"))?
        .to_string_lossy()
        .replace('\\', "/");

    let (rms, peak, dc_mean, finite) = stats(spec.samples);
    if !finite {
        return Err(format!(
            "non-finite sample generated in golden scenario {}",
            spec.name
        ));
    }

    Ok(GoldenScenarioRecord {
        name: spec.name.to_string(),
        category: spec.category.to_string(),
        sample_rate: spec.sample_rate,
        channels: 1,
        total_frames: spec.samples.len(),
        raw_f32_path: raw_rel,
        raw_f32_sha256: raw_sha,
        wav_path: wav_rel,
        wav_sha256: wav_sha,
        rms,
        peak,
        dc_mean,
        finite,
        extra: spec.extra,
    })
}

fn build_runtime(
    bank_dir: &Path,
    engine_mode: EngineMode,
    with_transmission: bool,
) -> Result<Gf509Runtime, String> {
    let transmission = if with_transmission {
        active_transmission_config(SAMPLE_RATE)
    } else {
        TransmissionConfig::default()
    };
    Gf509Runtime::new(Gf509RuntimeConfig {
        sample_layer_directory: Some(bank_dir.to_path_buf()),
        engine_mode,
        max_block_frames: 1024,
        transmission,
        ..Gf509RuntimeConfig::default()
    })
}

fn active_transmission_config(sample_rate: u32) -> TransmissionConfig {
    TransmissionConfig {
        sample_rate,
        output_gain: 0.22,
        gear_ratios: vec![3.10, 2.35, 1.85, 1.50, 1.25, 1.06],
        ..TransmissionConfig::default()
    }
}

fn capture_steady_5000rpm_engine(
    golden_dir: &Path,
    v2_root: &Path,
) -> Result<GoldenScenarioRecord, String> {
    let mut engine = V10Engine::new(EngineConfig::default())?;
    let mut scene = AcousticScene::new(SAMPLE_RATE as f32, AcousticSceneConfig::default())?;
    engine.set_input(EngineInput {
        rpm: 5_000.0,
        throttle: 0.72,
        load: 0.78,
    })?;

    let warmup_frames = SAMPLE_RATE as usize / 4; // 0.25s
    for _ in 0..warmup_frames {
        let frame = engine.render_sample();
        let _ = scene.process(&frame);
    }

    let render_frames = SAMPLE_RATE as usize; // 1.0s
    let mut scene_out = Vec::with_capacity(render_frames);
    let mut engine_master = Vec::with_capacity(render_frames);
    let mut engine_exhaust = Vec::with_capacity(render_frames);
    let mut engine_block_head = Vec::with_capacity(render_frames);
    let mut engine_turbulence = Vec::with_capacity(render_frames);
    let mut fired_events = 0u64;
    let mut max_limiter_db = 0.0f32;
    let mut final_crank_phase = 0.0f32;

    for _ in 0..render_frames {
        let frame = engine.render_sample();
        let acoustic = scene.process(&frame);
        engine_master.push(frame.master);
        engine_exhaust.push(frame.exhaust);
        engine_block_head.push(frame.block_head);
        engine_turbulence.push(frame.turbulence);
        scene_out.push(acoustic.output);
        fired_events += frame.fired_mask.count_ones() as u64;
        if frame.limiter_reduction_db > max_limiter_db {
            max_limiter_db = frame.limiter_reduction_db;
        }
        final_crank_phase = frame.crank_phase_deg;
    }

    let mut extra = BTreeMap::new();
    extra.insert("fired_events".into(), serde_json::json!(fired_events));
    extra.insert(
        "final_crank_phase_deg".into(),
        serde_json::json!(final_crank_phase),
    );
    extra.insert(
        "max_limiter_reduction_db".into(),
        serde_json::json!(max_limiter_db),
    );
    extra.insert(
        "engine_master_sha256".into(),
        serde_json::json!(sha256_bytes(&f32_slice_to_le_bytes(&engine_master))),
    );
    extra.insert(
        "engine_master_rms".into(),
        serde_json::json!(stats(&engine_master).0),
    );
    extra.insert(
        "engine_exhaust_rms".into(),
        serde_json::json!(stats(&engine_exhaust).0),
    );
    extra.insert(
        "engine_block_head_rms".into(),
        serde_json::json!(stats(&engine_block_head).0),
    );
    extra.insert(
        "engine_turbulence_rms".into(),
        serde_json::json!(stats(&engine_turbulence).0),
    );

    save_golden_signal(
        golden_dir,
        v2_root,
        GoldenOutputSpec {
            name: "steady_5000rpm_engine",
            category: "engine_scene",
            sample_rate: SAMPLE_RATE,
            samples: &scene_out,
            write_wav: true,
            extra,
        },
    )
}

fn capture_steady_5000rpm_runtime_hybrid(
    golden_dir: &Path,
    v2_root: &Path,
    schema1_dir: &Path,
) -> Result<GoldenScenarioRecord, String> {
    let mut runtime = build_runtime(schema1_dir, EngineMode::Hybrid, false)?;

    let block_size = 400usize; // 1/120 s at 48 kHz
    runtime.update_telemetry(RuntimeTelemetry {
        rpm: 5_000.0,
        throttle: 0.72,
        normalized_engine_load: 0.78,
        normalized_engine_torque: 0.72,
        torque_sign: TorqueSign::Positive,
        rpm_derivative: 0.0,
        throttle_derivative: 0.0,
        gear: 3,
        shift_phase: ShiftPhase::None,
        clutch_engagement: 1.0,
        tc_cut_ratio: 0.0,
        rev_limiter_active: false,
        dt_seconds: block_size as f32 / SAMPLE_RATE as f32,
    })?;

    let mut left = vec![0.0f32; block_size];
    let mut right = vec![0.0f32; block_size];
    for _ in 0..30 {
        runtime.render_block(&mut left, &mut right)?;
    }

    let mut out = Vec::with_capacity(SAMPLE_RATE as usize);
    let mut stereo_identical = true;
    for _ in 0..120 {
        runtime.render_block(&mut left, &mut right)?;
        if left
            .iter()
            .zip(right.iter())
            .any(|(l, r)| l.to_bits() != r.to_bits())
        {
            stereo_identical = false;
        }
        out.extend_from_slice(&left);
    }

    let mut extra = BTreeMap::new();
    extra.insert(
        "stereo_left_equals_right".into(),
        serde_json::json!(stereo_identical),
    );
    save_golden_signal(
        golden_dir,
        v2_root,
        GoldenOutputSpec {
            name: "steady_5000rpm_runtime_hybrid",
            category: "runtime_hybrid_schema1",
            sample_rate: SAMPLE_RATE,
            samples: &out,
            write_wav: true,
            extra,
        },
    )
}

fn capture_rpm_sweep_hybrid_schema1(
    golden_dir: &Path,
    v2_root: &Path,
    schema1_dir: &Path,
) -> Result<GoldenScenarioRecord, String> {
    let mut rt_whole = build_runtime(schema1_dir, EngineMode::Hybrid, false)?;
    let mut rt_split = build_runtime(schema1_dir, EngineMode::Hybrid, false)?;
    let step_frames = 400usize;
    let total_steps = 180usize; // 1.5s
    let dt = step_frames as f32 / SAMPLE_RATE as f32;
    let mut out_whole = Vec::with_capacity(step_frames * total_steps);
    let mut out_split = Vec::with_capacity(step_frames * total_steps);
    let split_sizes = [127usize, 273usize]; // sums to 400

    for step in 0..total_steps {
        let u = step as f32 / (total_steps - 1) as f32;
        let rpm = 4_200.0 + (18_500.0 - 4_200.0) * u;
        let telemetry = RuntimeTelemetry {
            rpm,
            throttle: 0.94,
            normalized_engine_load: 0.88,
            normalized_engine_torque: 0.85,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: (18_500.0 - 4_200.0) / 1.5,
            throttle_derivative: 0.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: rpm > 18_000.0,
            dt_seconds: dt,
        };
        rt_whole.update_telemetry(telemetry)?;
        rt_split.update_telemetry(telemetry)?;

        let mut lw = vec![0.0f32; step_frames];
        let mut rw = vec![0.0f32; step_frames];
        rt_whole.render_block(&mut lw, &mut rw)?;
        out_whole.extend_from_slice(&lw);

        for &sz in &split_sizes {
            let mut ls = vec![0.0f32; sz];
            let mut rs = vec![0.0f32; sz];
            rt_split.render_block(&mut ls, &mut rs)?;
            out_split.extend_from_slice(&ls);
        }
    }

    let bit_identical = out_whole.len() == out_split.len()
        && out_whole
            .iter()
            .zip(out_split.iter())
            .all(|(a, b)| a.to_bits() == b.to_bits());
    if !bit_identical {
        return Err("rpm_sweep_hybrid_schema1 split-block output diverged from whole-block".into());
    }

    let mut extra = BTreeMap::new();
    extra.insert(
        "block_split_bit_identical".into(),
        serde_json::json!(bit_identical),
    );
    save_golden_signal(
        golden_dir,
        v2_root,
        GoldenOutputSpec {
            name: "rpm_sweep_hybrid_schema1",
            category: "runtime_hybrid_schema1",
            sample_rate: SAMPLE_RATE,
            samples: &out_whole,
            write_wav: true,
            extra,
        },
    )
}

fn capture_coast_lift_hybrid_schema2(
    golden_dir: &Path,
    v2_root: &Path,
    schema2_dir: &Path,
) -> Result<GoldenScenarioRecord, String> {
    let mut runtime = build_runtime(schema2_dir, EngineMode::Hybrid, true)?;

    let step_frames = 400usize;
    let total_steps = 180usize; // 1.5s: 0..60 accel, 60..84 hold, 84..180 coast/lift
    let dt = step_frames as f32 / SAMPLE_RATE as f32;
    let mut out = Vec::with_capacity(step_frames * total_steps);
    let mut left = vec![0.0f32; step_frames];
    let mut right = vec![0.0f32; step_frames];

    for step in 0..total_steps {
        let (rpm, throttle, load, torque, sign, rpm_deriv, thr_deriv) = if step < 60 {
            let u = step as f32 / 60.0;
            (
                9_000.0 + 6_000.0 * u,
                0.96,
                0.92,
                0.86,
                TorqueSign::Positive,
                12_000.0,
                0.0,
            )
        } else if step < 84 {
            (15_000.0, 0.96, 0.90, 0.82, TorqueSign::Positive, 0.0, 0.0)
        } else {
            let u = (step - 84) as f32 / 96.0;
            (
                15_000.0 - 9_000.0 * u,
                0.0,
                0.08,
                -0.55,
                TorqueSign::Negative,
                -11_250.0,
                if step == 84 { -30.0 } else { 0.0 },
            )
        };
        runtime.update_telemetry(RuntimeTelemetry {
            rpm,
            throttle,
            normalized_engine_load: load,
            normalized_engine_torque: torque,
            torque_sign: sign,
            rpm_derivative: rpm_deriv,
            throttle_derivative: thr_deriv,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: dt,
        })?;
        runtime.render_block(&mut left, &mut right)?;
        out.extend_from_slice(&left);
    }

    save_golden_signal(
        golden_dir,
        v2_root,
        GoldenOutputSpec {
            name: "coast_lift_hybrid_schema2",
            category: "runtime_hybrid_schema2",
            sample_rate: SAMPLE_RATE,
            samples: &out,
            write_wav: true,
            extra: BTreeMap::new(),
        },
    )
}

fn capture_shift_up_down_hybrid(
    golden_dir: &Path,
    v2_root: &Path,
    schema1_dir: &Path,
) -> Result<GoldenScenarioRecord, String> {
    let mut runtime = build_runtime(schema1_dir, EngineMode::Hybrid, true)?;

    let step_frames = 400usize;
    let total_steps = 192usize; // 1.6s
    let dt = step_frames as f32 / SAMPLE_RATE as f32;
    let mut out = Vec::with_capacity(step_frames * total_steps);
    let mut left = vec![0.0f32; step_frames];
    let mut right = vec![0.0f32; step_frames];

    for step in 0..total_steps {
        let (rpm, throttle, load, torque, sign, gear, phase, clutch, tc_cut, limiter) = match step {
            0..=35 => (
                12_000.0 + (step as f32 / 36.0) * 4_500.0,
                0.98,
                0.95,
                0.92,
                TorqueSign::Positive,
                3,
                ShiftPhase::None,
                1.0,
                if step >= 28 { 0.35 } else { 0.0 },
                step >= 32,
            ),
            36..=44 => (
                16_000.0,
                0.98,
                0.15,
                -0.10,
                TorqueSign::Negative,
                3,
                ShiftPhase::UpshiftCut,
                0.35,
                0.0,
                false,
            ),
            45..=56 => (
                13_200.0,
                0.96,
                0.85,
                0.80,
                TorqueSign::Positive,
                4,
                ShiftPhase::UpshiftRecovery,
                0.90,
                0.0,
                false,
            ),
            57..=107 => (
                13_200.0 - ((step - 57) as f32 / 51.0) * 4_000.0,
                0.05,
                0.10,
                -0.48,
                TorqueSign::Negative,
                4,
                ShiftPhase::None,
                1.0,
                0.0,
                false,
            ),
            108..=114 => (
                9_200.0,
                0.0,
                0.08,
                -0.40,
                TorqueSign::Negative,
                4,
                ShiftPhase::DownshiftCut,
                0.20,
                0.0,
                false,
            ),
            115..=124 => (
                12_800.0,
                0.55,
                0.30,
                0.15,
                TorqueSign::Positive,
                4,
                ShiftPhase::DownshiftBlip,
                0.25,
                0.0,
                false,
            ),
            125..=140 => (
                11_800.0,
                0.10,
                0.15,
                -0.45,
                TorqueSign::Negative,
                3,
                ShiftPhase::DownshiftRecovery,
                0.95,
                0.0,
                false,
            ),
            _ => (
                11_800.0 - ((step - 141) as f32 / 51.0) * 2_800.0,
                0.10,
                0.12,
                -0.35,
                TorqueSign::Negative,
                3,
                ShiftPhase::None,
                1.0,
                0.0,
                false,
            ),
        };
        runtime.update_telemetry(RuntimeTelemetry {
            rpm,
            throttle,
            normalized_engine_load: load,
            normalized_engine_torque: torque,
            torque_sign: sign,
            rpm_derivative: 0.0,
            throttle_derivative: 0.0,
            gear,
            shift_phase: phase,
            clutch_engagement: clutch,
            tc_cut_ratio: tc_cut,
            rev_limiter_active: limiter,
            dt_seconds: dt,
        })?;
        runtime.render_block(&mut left, &mut right)?;
        out.extend_from_slice(&left);
    }

    save_golden_signal(
        golden_dir,
        v2_root,
        GoldenOutputSpec {
            name: "shift_up_down_hybrid",
            category: "runtime_shifts",
            sample_rate: SAMPLE_RATE,
            samples: &out,
            write_wav: true,
            extra: BTreeMap::new(),
        },
    )
}

fn capture_sample_only_runtime(
    golden_dir: &Path,
    v2_root: &Path,
    name: &str,
    bank_dir: &Path,
) -> Result<GoldenScenarioRecord, String> {
    let mut runtime = build_runtime(bank_dir, EngineMode::SampleOnly, true)?;

    let step_frames = 400usize;
    let total_steps = 96usize; // 0.8s
    let dt = step_frames as f32 / SAMPLE_RATE as f32;
    let mut out = Vec::with_capacity(step_frames * total_steps);
    let mut left = vec![0.0f32; step_frames];
    let mut right = vec![0.0f32; step_frames];

    for step in 0..total_steps {
        let (rpm, throttle, load, torque, sign) = if step < 60 {
            let u = step as f32 / 60.0;
            (
                4_000.0 + 11_000.0 * u,
                0.90,
                0.85,
                0.82,
                TorqueSign::Positive,
            )
        } else {
            let u = (step - 60) as f32 / 36.0;
            (
                15_000.0 - 7_500.0 * u,
                0.0,
                0.10,
                -0.50,
                TorqueSign::Negative,
            )
        };
        runtime.update_telemetry(RuntimeTelemetry {
            rpm,
            throttle,
            normalized_engine_load: load,
            normalized_engine_torque: torque,
            torque_sign: sign,
            rpm_derivative: 0.0,
            throttle_derivative: 0.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: dt,
        })?;
        runtime.render_block(&mut left, &mut right)?;
        out.extend_from_slice(&left);
    }

    save_golden_signal(
        golden_dir,
        v2_root,
        GoldenOutputSpec {
            name,
            category: "runtime_sample_only",
            sample_rate: SAMPLE_RATE,
            samples: &out,
            write_wav: true,
            extra: BTreeMap::new(),
        },
    )
}

fn capture_sample_layer_direct(
    golden_dir: &Path,
    v2_root: &Path,
    name: &str,
    bank_dir: &Path,
    output_sample_rate: u32,
) -> Result<GoldenScenarioRecord, String> {
    let mut layer = ThreeZoneSampleLayer::load_directory(
        output_sample_rate,
        bank_dir,
        ThreeZoneSampleLayerConfig::default(),
    )?;
    let frames = (output_sample_rate as usize) / 2; // 0.5s
    let mut out = Vec::with_capacity(frames);
    let mut frame = SampleLayerFrame::default();
    let mut phase_deg = 0.0f32;

    for i in 0..frames {
        let u = i as f32 / (frames - 1) as f32;
        let (rpm, throttle, load, torque) = if u < 0.65 {
            let k = u / 0.65;
            (3_800.0 + 11_700.0 * k, 0.92, 0.88, 0.86)
        } else {
            let k = (u - 0.65) / 0.35;
            (15_500.0 - 9_500.0 * k, 0.02, 0.12, -0.52)
        };
        let crank_step = rpm * 6.0 / output_sample_rate as f32;
        phase_deg = (phase_deg + crank_step).rem_euclid(720.0);
        layer.process_into(
            SampleLayerInput {
                rpm,
                throttle,
                load,
                normalized_engine_torque: torque,
                clutch_engagement: 1.0,
                crank_phase_deg: phase_deg,
            },
            &mut frame,
        )?;
        out.push(frame.output);
    }

    let mut extra = BTreeMap::new();
    extra.insert(
        "suspended_zone_samples".into(),
        serde_json::json!(layer.suspended_zone_samples()),
    );
    extra.insert(
        "source_labels".into(),
        serde_json::json!(layer.source_labels()),
    );

    save_golden_signal(
        golden_dir,
        v2_root,
        GoldenOutputSpec {
            name,
            category: "sample_layer_direct",
            sample_rate: output_sample_rate,
            samples: &out,
            write_wav: false,
            extra,
        },
    )
}

fn read_wav_pcm16_samples(path: &Path) -> Result<(u32, u16, Vec<f32>), String> {
    let bytes = fs::read(path).map_err(|e| format!("read wav {}: {e}", path.display()))?;
    if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return Err(format!("invalid RIFF/WAVE header in {}", path.display()));
    }
    let mut pos = 12usize;
    let mut sample_rate = 0u32;
    let mut channels = 0u16;
    let mut bits_per_sample = 0u16;
    let mut data_slice: Option<&[u8]> = None;
    while pos + 8 <= bytes.len() {
        let id = &bytes[pos..pos + 4];
        let size = u32::from_le_bytes(bytes[pos + 4..pos + 8].try_into().unwrap()) as usize;
        let payload_start = pos + 8;
        let payload_end = payload_start.saturating_add(size).min(bytes.len());
        if id == b"fmt " && size >= 16 {
            channels = u16::from_le_bytes(
                bytes[payload_start + 2..payload_start + 4]
                    .try_into()
                    .unwrap(),
            );
            sample_rate = u32::from_le_bytes(
                bytes[payload_start + 4..payload_start + 8]
                    .try_into()
                    .unwrap(),
            );
            bits_per_sample = u16::from_le_bytes(
                bytes[payload_start + 14..payload_start + 16]
                    .try_into()
                    .unwrap(),
            );
        } else if id == b"data" {
            data_slice = Some(&bytes[payload_start..payload_end]);
        }
        pos = payload_end + (size % 2);
    }
    if bits_per_sample != 16 {
        return Err(format!(
            "expected 16-bit PCM in {}, got {bits_per_sample}",
            path.display()
        ));
    }
    let data = data_slice.ok_or_else(|| format!("missing data chunk in {}", path.display()))?;
    let mut samples = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
        samples.push(s as f32 / i16::MAX as f32);
    }
    let total_frames = if channels > 0 {
        samples.len() / channels as usize
    } else {
        0
    };
    let _ = total_frames;
    Ok((sample_rate, channels, samples))
}

fn hash_cli_outputs(v2_root: &Path, golden_dir: &Path) -> Result<(), String> {
    let cli_dir = golden_dir.join("cli");
    if !cli_dir.is_dir() {
        return Err(format!("CLI golden dir missing: {}", cli_dir.display()));
    }
    let mut wavs: Vec<PathBuf> = fs::read_dir(&cli_dir)
        .map_err(|e| format!("read_dir {}: {e}", cli_dir.display()))?
        .filter_map(|e| e.ok().map(|v| v.path()))
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    wavs.sort();

    let mut entries = Vec::new();
    for path in wavs {
        let sha = sha256_file(&path)?;
        let (sr, ch, samples) = read_wav_pcm16_samples(&path)?;
        let (rms, peak, _, finite) = stats(&samples);
        let rel = path
            .strip_prefix(v2_root)
            .map_err(|e| format!("strip_prefix: {e}"))?
            .to_string_lossy()
            .replace('\\', "/");
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .to_string();
        let total_frames = samples.len() / (ch.max(1) as usize);
        entries.push(CliGoldenEntry {
            name,
            wav_path: rel,
            wav_sha256: sha,
            sample_rate: sr,
            channels: ch,
            total_frames,
            rms,
            peak,
            finite,
        });
    }

    let manifest = CliGoldenManifest {
        binary: "v10_render".to_string(),
        entries,
    };
    let json = serde_json::to_string_pretty(&manifest)
        .map_err(|e| format!("serialize cli manifest: {e}"))?;
    let out_path = golden_dir.join("cli_manifest.json");
    fs::write(&out_path, json).map_err(|e| format!("write {}: {e}", out_path.display()))?;
    println!(
        "Wrote {} ({} CLI WAV entries)",
        out_path.display(),
        manifest.entries.len()
    );
    Ok(())
}

fn run_generate(v2_root: &Path) -> Result<(), String> {
    let fixtures_dir = v2_root.join("tests/fixtures");
    let golden_dir = v2_root.join("tests/golden");
    let schema1_dir = fixtures_dir.join("bank_schema1");
    let schema2_dir = fixtures_dir.join("bank_schema2");

    generate_schema1_bank(&schema1_dir)?;
    generate_schema2_bank(&schema2_dir)?;
    println!("Generated fixtures at {}", fixtures_dir.display());

    let scenarios = vec![
        capture_steady_5000rpm_engine(&golden_dir, v2_root)?,
        capture_steady_5000rpm_runtime_hybrid(&golden_dir, v2_root, &schema1_dir)?,
        capture_rpm_sweep_hybrid_schema1(&golden_dir, v2_root, &schema1_dir)?,
        capture_coast_lift_hybrid_schema2(&golden_dir, v2_root, &schema2_dir)?,
        capture_shift_up_down_hybrid(&golden_dir, v2_root, &schema1_dir)?,
        capture_sample_only_runtime(&golden_dir, v2_root, "sample_only_schema1", &schema1_dir)?,
        capture_sample_only_runtime(&golden_dir, v2_root, "sample_only_schema2", &schema2_dir)?,
        capture_sample_layer_direct(
            &golden_dir,
            v2_root,
            "sample_layer_direct_schema1",
            &schema1_dir,
            SAMPLE_RATE,
        )?,
        capture_sample_layer_direct(
            &golden_dir,
            v2_root,
            "sample_layer_direct_schema2",
            &schema2_dir,
            SAMPLE_RATE,
        )?,
        capture_sample_layer_direct(
            &golden_dir,
            v2_root,
            "sample_layer_resample_44k",
            &schema1_dir,
            RESAMPLED_TEST_RATE,
        )?,
    ];

    let mut fixtures_map = BTreeMap::new();
    fixtures_map.insert(
        "bank_schema1_manifest_sha256".into(),
        sha256_file(&schema1_dir.join("manifest.json"))?,
    );
    fixtures_map.insert(
        "bank_schema2_manifest_sha256".into(),
        sha256_file(&schema2_dir.join("manifest.json"))?,
    );

    let manifest = GoldenManifest {
        baseline_crate: "references/v10-engine-synth".to_string(),
        sample_rate: SAMPLE_RATE,
        fixtures: fixtures_map,
        scenarios,
    };
    let manifest_json = serde_json::to_string_pretty(&manifest)
        .map_err(|e| format!("serialize golden manifest: {e}"))?;
    let manifest_path = golden_dir.join("golden_manifest.json");
    fs::write(&manifest_path, manifest_json)
        .map_err(|e| format!("write {}: {e}", manifest_path.display()))?;

    for s in &manifest.scenarios {
        println!(
            "  scenario={:<30} frames={:<6} rms={:.6} peak={:.6} sha256={}",
            s.name, s.total_frames, s.rms, s.peak, s.raw_f32_sha256
        );
    }
    println!("Wrote {}", manifest_path.display());
    Ok(())
}

fn main() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let v2_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("v1-baseline must live under v2/tools/v1-baseline")
        .to_path_buf();

    let mode = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "generate".to_string());
    let res = match mode.as_str() {
        "generate" | "--generate" => run_generate(&v2_root),
        "hash-cli" | "--hash-cli" => hash_cli_outputs(&v2_root, &v2_root.join("tests/golden")),
        other => Err(format!(
            "unknown mode '{other}'; expected 'generate' or 'hash-cli'"
        )),
    };
    if let Err(err) = res {
        eprintln!("v1-baseline error: {err}");
        std::process::exit(1);
    }
}
