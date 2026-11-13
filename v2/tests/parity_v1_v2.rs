use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Deserialize)]
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

#[derive(Deserialize)]
struct GoldenManifest {
    baseline_crate: String,
    sample_rate: u32,
    fixtures: BTreeMap<String, String>,
    scenarios: Vec<GoldenScenarioRecord>,
}

#[derive(Deserialize)]
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

#[derive(Deserialize)]
struct CliGoldenManifest {
    binary: String,
    entries: Vec<CliGoldenEntry>,
}

fn v2_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn sha256_bytes(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn sha256_file(path: &Path) -> String {
    let bytes = fs::read(path).unwrap_or_else(|e| panic!("failed reading {}: {e}", path.display()));
    sha256_bytes(&bytes)
}

fn decode_f32_le(bytes: &[u8]) -> Vec<f32> {
    assert_eq!(
        bytes.len() % 4,
        0,
        "f32 buffer length must be multiple of 4"
    );
    bytes
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

#[test]
fn golden_master_binaries_and_hashes_are_intact_and_finite() {
    let root = v2_root();
    let manifest_path = root.join("tests/golden/golden_manifest.json");
    let raw = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|e| panic!("missing {}: {e}", manifest_path.display()));
    let manifest: GoldenManifest = serde_json::from_str(&raw).expect("valid golden_manifest.json");

    assert_eq!(manifest.baseline_crate, "references/v10-engine-synth");
    assert_eq!(manifest.sample_rate, 48_000);
    assert_eq!(manifest.scenarios.len(), 10);

    let expected_schema1_sha = manifest
        .fixtures
        .get("bank_schema1_manifest_sha256")
        .expect("schema1 hash present");
    let expected_schema2_sha = manifest
        .fixtures
        .get("bank_schema2_manifest_sha256")
        .expect("schema2 hash present");
    assert_eq!(
        &sha256_file(&root.join("tests/fixtures/bank_schema1/manifest.json")),
        expected_schema1_sha
    );
    assert_eq!(
        &sha256_file(&root.join("tests/fixtures/bank_schema2/manifest.json")),
        expected_schema2_sha
    );

    let mut seen_names = Vec::new();
    for scenario in &manifest.scenarios {
        assert!(
            scenario.finite,
            "scenario {} marked non-finite",
            scenario.name
        );
        assert!(
            scenario.total_frames > 0,
            "scenario {} empty",
            scenario.name
        );
        assert!(!scenario.category.is_empty());
        assert_eq!(scenario.channels, 1);
        let expected_sr = if scenario.name == "sample_layer_resample_44k" {
            44_100
        } else {
            48_000
        };
        assert_eq!(
            scenario.sample_rate, expected_sr,
            "unexpected sample_rate for {}",
            scenario.name
        );

        let raw_path = root.join(&scenario.raw_f32_path);
        let bytes =
            fs::read(&raw_path).unwrap_or_else(|e| panic!("missing {}: {e}", raw_path.display()));
        assert_eq!(
            sha256_bytes(&bytes),
            scenario.raw_f32_sha256,
            "sha256 mismatch for {}",
            scenario.name
        );

        let samples = decode_f32_le(&bytes);
        assert_eq!(samples.len(), scenario.total_frames);

        let mut sum_sq = 0.0f64;
        let mut sum = 0.0f64;
        let mut peak = 0.0f32;
        for &s in &samples {
            assert!(s.is_finite(), "non-finite sample in {}", scenario.name);
            peak = peak.max(s.abs());
            sum += s as f64;
            sum_sq += (s as f64) * (s as f64);
        }
        let rms = (sum_sq / samples.len() as f64).sqrt();
        let dc = sum / samples.len() as f64;
        assert!(
            rms > 1e-4,
            "scenario {} should produce audible signal, got rms={rms}",
            scenario.name
        );
        assert!(
            (rms - scenario.rms).abs() < 1e-9,
            "rms mismatch in {}",
            scenario.name
        );
        assert!(
            (peak - scenario.peak).abs() < 1e-6,
            "peak mismatch in {}",
            scenario.name
        );
        assert!(
            (dc - scenario.dc_mean).abs() < 1e-9,
            "dc mismatch in {}",
            scenario.name
        );

        if let (Some(wav_rel), Some(wav_sha)) = (&scenario.wav_path, &scenario.wav_sha256) {
            let wav_path = root.join(wav_rel);
            assert_eq!(
                &sha256_file(&wav_path),
                wav_sha,
                "wav sha256 mismatch for {}",
                scenario.name
            );
        }

        if scenario.name == "rpm_sweep_hybrid_schema1" {
            assert_eq!(
                scenario.extra.get("block_split_bit_identical"),
                Some(&serde_json::json!(true)),
                "split-block boundary invariance must hold"
            );
        }
        if scenario.name == "steady_5000rpm_runtime_hybrid" {
            assert_eq!(
                scenario.extra.get("stereo_left_equals_right"),
                Some(&serde_json::json!(true))
            );
        }
        seen_names.push(scenario.name.as_str());
    }

    for required in [
        "steady_5000rpm_engine",
        "steady_5000rpm_runtime_hybrid",
        "rpm_sweep_hybrid_schema1",
        "coast_lift_hybrid_schema2",
        "shift_up_down_hybrid",
        "sample_only_schema1",
        "sample_only_schema2",
        "sample_layer_direct_schema1",
        "sample_layer_direct_schema2",
        "sample_layer_resample_44k",
    ] {
        assert!(
            seen_names.contains(&required),
            "missing scenario {required}"
        );
    }
}

#[test]
fn cli_v10_render_golden_wavs_match_recorded_hashes() {
    let root = v2_root();
    let cli_manifest_path = root.join("tests/golden/cli_manifest.json");
    let raw = fs::read_to_string(&cli_manifest_path)
        .unwrap_or_else(|e| panic!("missing {}: {e}", cli_manifest_path.display()));
    let manifest: CliGoldenManifest = serde_json::from_str(&raw).expect("valid cli_manifest.json");

    assert_eq!(manifest.binary, "v10_render");
    assert_eq!(manifest.entries.len(), 4);

    for entry in &manifest.entries {
        assert!(entry.finite);
        assert_eq!(entry.sample_rate, 48_000);
        assert_eq!(entry.channels, 1);
        assert!(entry.total_frames > 0);
        assert!(entry.rms > 1e-4 && entry.peak > 1e-3 && entry.peak <= 1.0);
        let wav_path = root.join(&entry.wav_path);
        assert_eq!(
            sha256_file(&wav_path),
            entry.wav_sha256,
            "CLI wav hash mismatch for {}",
            entry.name
        );
    }
}

#[test]
fn v2_parity_steady_5000rpm_engine_matches_v1_golden() {
    use v10_engine_synth::{
        AcousticScene, AcousticSceneConfig, EngineConfig, EngineInput, V10Engine,
    };

    let root = v2_root();
    let manifest_path = root.join("tests/golden/golden_manifest.json");
    let raw = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|e| panic!("missing {}: {e}", manifest_path.display()));
    let manifest: GoldenManifest = serde_json::from_str(&raw).expect("valid golden_manifest.json");

    let scenario = manifest
        .scenarios
        .iter()
        .find(|s| s.name == "steady_5000rpm_engine")
        .expect("missing steady_5000rpm_engine scenario");
    assert_eq!(scenario.sample_rate, 48_000);
    assert_eq!(scenario.channels, 1);

    let expected_bytes = fs::read(root.join(&scenario.raw_f32_path))
        .unwrap_or_else(|e| panic!("failed reading {}: {e}", scenario.raw_f32_path));
    let expected_scene = decode_f32_le(&expected_bytes);
    assert_eq!(expected_scene.len(), scenario.total_frames);

    let sample_rate = scenario.sample_rate;
    let mut engine = V10Engine::new(EngineConfig::default()).expect("engine init");
    let mut scene =
        AcousticScene::new(sample_rate as f32, AcousticSceneConfig::default()).expect("scene init");
    engine
        .set_input(EngineInput {
            rpm: 5_000.0,
            throttle: 0.72,
            load: 0.78,
        })
        .expect("set_input");

    let warmup_frames = sample_rate as usize / 4;
    for _ in 0..warmup_frames {
        let frame = engine.render_sample();
        let _ = scene.process(&frame);
    }

    let render_frames = sample_rate as usize;
    assert_eq!(render_frames, expected_scene.len());
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

    let mut bit_mismatches = 0usize;
    let mut max_abs_diff = 0.0f32;
    let mut max_diff_idx = 0usize;
    for (idx, (&a, &b)) in scene_out.iter().zip(expected_scene.iter()).enumerate() {
        assert!(a.is_finite(), "non-finite scene output at {idx}");
        let diff = (a - b).abs();
        if diff > max_abs_diff {
            max_abs_diff = diff;
            max_diff_idx = idx;
        }
        if a.to_bits() != b.to_bits() {
            bit_mismatches += 1;
        }
    }
    assert_eq!(
        bit_mismatches, 0,
        "steady_5000rpm_engine scene output diverged from v1 golden: bit_mismatches={bit_mismatches}/{render_frames}, max_abs_diff={max_abs_diff} at idx={max_diff_idx} (v2={}, v1={})",
        scene_out[max_diff_idx],
        expected_scene[max_diff_idx]
    );

    let mut master_bytes = Vec::with_capacity(engine_master.len() * 4);
    for &sample in &engine_master {
        master_bytes.extend_from_slice(&sample.to_le_bytes());
    }
    let expected_master_sha = scenario
        .extra
        .get("engine_master_sha256")
        .and_then(|v| v.as_str())
        .expect("engine_master_sha256 in extra");
    assert_eq!(
        sha256_bytes(&master_bytes),
        expected_master_sha,
        "engine.master stream sha256 diverged from v1 golden"
    );

    let expected_fired = scenario
        .extra
        .get("fired_events")
        .and_then(|v| v.as_u64())
        .expect("fired_events in extra");
    assert_eq!(fired_events, expected_fired);

    let expected_phase = scenario
        .extra
        .get("final_crank_phase_deg")
        .and_then(|v| v.as_f64())
        .expect("final_crank_phase_deg in extra") as f32;
    assert_eq!(final_crank_phase.to_bits(), expected_phase.to_bits());

    let expected_limiter = scenario
        .extra
        .get("max_limiter_reduction_db")
        .and_then(|v| v.as_f64())
        .expect("max_limiter_reduction_db in extra") as f32;
    assert_eq!(max_limiter_db.to_bits(), expected_limiter.to_bits());

    let rms = |samples: &[f32]| -> f64 {
        let sum_sq: f64 = samples.iter().map(|&s| (s as f64) * (s as f64)).sum();
        (sum_sq / samples.len() as f64).sqrt()
    };
    for (key, samples) in [
        ("engine_master_rms", &engine_master),
        ("engine_exhaust_rms", &engine_exhaust),
        ("engine_block_head_rms", &engine_block_head),
        ("engine_turbulence_rms", &engine_turbulence),
    ] {
        let expected_rms = scenario
            .extra
            .get(key)
            .and_then(|v| v.as_f64())
            .unwrap_or_else(|| panic!("missing {key} in golden extra"));
        let actual_rms = rms(samples);
        assert!(
            (actual_rms - expected_rms).abs() <= 1.0e-12,
            "{key} mismatch: v2={actual_rms} v1={expected_rms}"
        );
    }
}

#[test]
fn v2_parity_sample_layer_matches_v1_golden() {
    use v10_engine_synth::{SampleLayerInput, ThreeZoneSampleLayer, ThreeZoneSampleLayerConfig};

    let root = v2_root();
    let manifest_path = root.join("tests/golden/golden_manifest.json");
    let raw = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|e| panic!("missing {}: {e}", manifest_path.display()));
    let manifest: GoldenManifest = serde_json::from_str(&raw).expect("valid golden_manifest.json");

    let cases = [
        (
            "sample_layer_direct_schema1",
            "tests/fixtures/bank_schema1",
            48_000u32,
        ),
        (
            "sample_layer_direct_schema2",
            "tests/fixtures/bank_schema2",
            48_000u32,
        ),
        (
            "sample_layer_resample_44k",
            "tests/fixtures/bank_schema1",
            44_100u32,
        ),
    ];

    for (scenario_name, bank_rel, output_sample_rate) in cases {
        let scenario = manifest
            .scenarios
            .iter()
            .find(|s| s.name == scenario_name)
            .unwrap_or_else(|| panic!("missing golden scenario {scenario_name}"));
        assert_eq!(scenario.sample_rate, output_sample_rate);
        assert_eq!(scenario.channels, 1);

        let expected_bytes = fs::read(root.join(&scenario.raw_f32_path))
            .unwrap_or_else(|e| panic!("failed reading golden raw {}: {e}", scenario.raw_f32_path));
        let expected = decode_f32_le(&expected_bytes);
        assert_eq!(expected.len(), scenario.total_frames);

        let mut layer = ThreeZoneSampleLayer::load_directory(
            output_sample_rate,
            &root.join(bank_rel),
            ThreeZoneSampleLayerConfig::default(),
        )
        .unwrap_or_else(|e| panic!("load_directory failed for {scenario_name}: {e}"));

        let frames = (output_sample_rate as usize) / 2;
        assert_eq!(frames, expected.len());
        let mut frame = layer.allocate_frame();
        let mut actual = Vec::with_capacity(frames);
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
            layer
                .process_into(
                    SampleLayerInput {
                        rpm,
                        throttle,
                        load,
                        normalized_engine_torque: torque,
                        clutch_engagement: 1.0,
                        crank_phase_deg: phase_deg,
                    },
                    &mut frame,
                )
                .unwrap_or_else(|e| panic!("process_into failed at {i} for {scenario_name}: {e}"));
            actual.push(frame.output);
        }

        let mut max_abs_diff = 0.0f32;
        let mut max_diff_idx = 0usize;
        let mut bit_mismatches = 0usize;
        for (idx, (&a, &b)) in actual.iter().zip(expected.iter()).enumerate() {
            assert!(
                a.is_finite(),
                "non-finite output at {idx} for {scenario_name}"
            );
            let diff = (a - b).abs();
            if diff > max_abs_diff {
                max_abs_diff = diff;
                max_diff_idx = idx;
            }
            if a.to_bits() != b.to_bits() {
                bit_mismatches += 1;
            }
        }

        assert_eq!(
            bit_mismatches, 0,
            "{scenario_name} diverged from v1 golden: bit_mismatches={bit_mismatches}/{frames}, max_abs_diff={max_abs_diff} at idx={max_diff_idx} (v2={}, v1={})",
            actual[max_diff_idx],
            expected[max_diff_idx]
        );

        let expected_suspended = scenario
            .extra
            .get("suspended_zone_samples")
            .and_then(|v| v.as_u64())
            .expect("suspended_zone_samples in golden extra");
        assert_eq!(
            layer.suspended_zone_samples(),
            expected_suspended,
            "suspended_zone_samples mismatch for {scenario_name}"
        );

        let expected_labels: Vec<String> = serde_json::from_value(
            scenario
                .extra
                .get("source_labels")
                .expect("source_labels in golden extra")
                .clone(),
        )
        .expect("valid source_labels array");
        assert_eq!(
            layer.source_labels(),
            expected_labels.as_slice(),
            "source_labels mismatch for {scenario_name}"
        );
    }
}

fn assert_matches_raw_golden(
    manifest: &GoldenManifest,
    root: &Path,
    scenario_name: &str,
    actual: &[f32],
) {
    let scenario = manifest
        .scenarios
        .iter()
        .find(|s| s.name == scenario_name)
        .unwrap_or_else(|| panic!("missing golden scenario {scenario_name}"));
    assert_eq!(scenario.sample_rate, 48_000);
    assert_eq!(scenario.channels, 1);
    assert_eq!(actual.len(), scenario.total_frames);

    let expected_bytes = fs::read(root.join(&scenario.raw_f32_path))
        .unwrap_or_else(|e| panic!("failed reading golden raw {}: {e}", scenario.raw_f32_path));
    let expected = decode_f32_le(&expected_bytes);
    assert_eq!(expected.len(), actual.len());

    let mut max_abs_diff = 0.0f32;
    let mut max_diff_idx = 0usize;
    let mut bit_mismatches = 0usize;
    let mut actual_bytes = Vec::with_capacity(actual.len() * 4);
    for (idx, (&a, &b)) in actual.iter().zip(expected.iter()).enumerate() {
        assert!(
            a.is_finite(),
            "non-finite sample at {idx} for {scenario_name}"
        );
        actual_bytes.extend_from_slice(&a.to_le_bytes());
        let diff = (a - b).abs();
        if diff > max_abs_diff {
            max_abs_diff = diff;
            max_diff_idx = idx;
        }
        if a.to_bits() != b.to_bits() {
            bit_mismatches += 1;
        }
    }

    assert_eq!(
        bit_mismatches, 0,
        "{scenario_name} diverged from v1 golden: bit_mismatches={bit_mismatches}/{}, max_abs_diff={max_abs_diff} at idx={max_diff_idx} (v2={}, v1={})",
        actual.len(),
        actual[max_diff_idx],
        expected[max_diff_idx]
    );
    assert_eq!(
        sha256_bytes(&actual_bytes),
        scenario.raw_f32_sha256,
        "{scenario_name} sha256 diverged from v1 golden"
    );
}

#[test]
fn v2_parity_runtime_hybrid_and_shifts_match_v1_golden() {
    use v10_engine_synth::{
        EngineMode, Gf509Runtime, Gf509RuntimeConfig, RuntimeTelemetry, ShiftPhase, TorqueSign,
        TransmissionConfig,
    };

    const SAMPLE_RATE: u32 = 48_000;
    let root = v2_root();
    let manifest_path = root.join("tests/golden/golden_manifest.json");
    let raw = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|e| panic!("missing {}: {e}", manifest_path.display()));
    let manifest: GoldenManifest = serde_json::from_str(&raw).expect("valid golden_manifest.json");

    let schema1_dir = root.join("tests/fixtures/bank_schema1");
    let schema2_dir = root.join("tests/fixtures/bank_schema2");

    let build_runtime = |bank_dir: &Path, engine_mode: EngineMode, with_tx: bool| -> Gf509Runtime {
        let transmission = if with_tx {
            TransmissionConfig {
                sample_rate: SAMPLE_RATE,
                output_gain: 0.22,
                gear_ratios: vec![3.10, 2.35, 1.85, 1.50, 1.25, 1.06],
                ..TransmissionConfig::default()
            }
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
        .expect("runtime init")
    };

    // 1. steady_5000rpm_runtime_hybrid
    {
        let mut runtime = build_runtime(&schema1_dir, EngineMode::Hybrid, false);
        let block_size = 400usize;
        runtime
            .update_telemetry(RuntimeTelemetry {
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
            })
            .expect("update_telemetry");
        let mut left = vec![0.0f32; block_size];
        let mut right = vec![0.0f32; block_size];
        for _ in 0..30 {
            runtime.render_block(&mut left, &mut right).expect("warmup");
        }
        let mut out = Vec::with_capacity(SAMPLE_RATE as usize);
        for _ in 0..120 {
            runtime.render_block(&mut left, &mut right).expect("render");
            assert!(
                left.iter()
                    .zip(right.iter())
                    .all(|(l, r)| l.to_bits() == r.to_bits()),
                "steady_5000rpm_runtime_hybrid stereo channels diverged"
            );
            out.extend_from_slice(&left);
        }
        assert_matches_raw_golden(&manifest, &root, "steady_5000rpm_runtime_hybrid", &out);
    }

    // 2. rpm_sweep_hybrid_schema1 (+ block-split invariance)
    {
        let mut rt_whole = build_runtime(&schema1_dir, EngineMode::Hybrid, false);
        let mut rt_split = build_runtime(&schema1_dir, EngineMode::Hybrid, false);
        let step_frames = 400usize;
        let total_steps = 180usize;
        let dt = step_frames as f32 / SAMPLE_RATE as f32;
        let mut out_whole = Vec::with_capacity(step_frames * total_steps);
        let mut out_split = Vec::with_capacity(step_frames * total_steps);
        let split_sizes = [127usize, 273usize];
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
            rt_whole.update_telemetry(telemetry).expect("whole update");
            rt_split.update_telemetry(telemetry).expect("split update");

            let mut lw = vec![0.0f32; step_frames];
            let mut rw = vec![0.0f32; step_frames];
            rt_whole
                .render_block(&mut lw, &mut rw)
                .expect("whole render");
            out_whole.extend_from_slice(&lw);

            for &sz in &split_sizes {
                let mut ls = vec![0.0f32; sz];
                let mut rs = vec![0.0f32; sz];
                rt_split
                    .render_block(&mut ls, &mut rs)
                    .expect("split render");
                out_split.extend_from_slice(&ls);
            }
        }
        assert!(
            out_whole.len() == out_split.len()
                && out_whole
                    .iter()
                    .zip(out_split.iter())
                    .all(|(a, b)| a.to_bits() == b.to_bits()),
            "rpm_sweep_hybrid_schema1 split-block output diverged from whole-block"
        );
        assert_matches_raw_golden(&manifest, &root, "rpm_sweep_hybrid_schema1", &out_whole);
    }

    // 3. coast_lift_hybrid_schema2
    {
        let mut runtime = build_runtime(&schema2_dir, EngineMode::Hybrid, true);
        let step_frames = 400usize;
        let total_steps = 180usize;
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
            runtime
                .update_telemetry(RuntimeTelemetry {
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
                })
                .expect("update_telemetry");
            runtime.render_block(&mut left, &mut right).expect("render");
            out.extend_from_slice(&left);
        }
        assert_matches_raw_golden(&manifest, &root, "coast_lift_hybrid_schema2", &out);
    }

    // 4. shift_up_down_hybrid
    {
        let mut runtime = build_runtime(&schema1_dir, EngineMode::Hybrid, true);
        let step_frames = 400usize;
        let total_steps = 192usize;
        let dt = step_frames as f32 / SAMPLE_RATE as f32;
        let mut out = Vec::with_capacity(step_frames * total_steps);
        let mut left = vec![0.0f32; step_frames];
        let mut right = vec![0.0f32; step_frames];
        for step in 0..total_steps {
            let (rpm, throttle, load, torque, sign, gear, phase, clutch, tc_cut, limiter) =
                match step {
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
            runtime
                .update_telemetry(RuntimeTelemetry {
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
                })
                .expect("update_telemetry");
            runtime.render_block(&mut left, &mut right).expect("render");
            out.extend_from_slice(&left);
        }
        assert_matches_raw_golden(&manifest, &root, "shift_up_down_hybrid", &out);
    }

    // 5 & 6. sample_only_schema1 & sample_only_schema2
    for (scenario_name, bank_dir) in [
        ("sample_only_schema1", schema1_dir.as_path()),
        ("sample_only_schema2", schema2_dir.as_path()),
    ] {
        let mut runtime = build_runtime(bank_dir, EngineMode::SampleOnly, true);
        let step_frames = 400usize;
        let total_steps = 96usize;
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
            runtime
                .update_telemetry(RuntimeTelemetry {
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
                })
                .expect("update_telemetry");
            runtime.render_block(&mut left, &mut right).expect("render");
            out.extend_from_slice(&left);
        }
        assert_matches_raw_golden(&manifest, &root, scenario_name, &out);
    }
}

#[test]
fn v2_parity_cli_renders_match_v1_golden() {
    use std::process::Command;

    struct TempDirGuard(PathBuf);
    impl Drop for TempDirGuard {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    let root = v2_root();
    let manifest_path = root.join("tests/golden/cli_manifest.json");
    let raw = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|e| panic!("missing {}: {e}", manifest_path.display()));
    let manifest: CliGoldenManifest = serde_json::from_str(&raw).expect("valid cli_manifest.json");
    assert_eq!(manifest.binary, "v10_render");
    assert_eq!(manifest.entries.len(), 4);

    let bin = env!("CARGO_BIN_EXE_v10_render");
    let unique = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let work_dir = std::env::temp_dir().join(format!("v10_v2_cli_parity_{unique}"));
    fs::create_dir_all(&work_dir).expect("create temp parity dir");
    let _guard = TempDirGuard(work_dir.clone());

    let schema1_dir = root.join("tests/fixtures/bank_schema1");
    let schema2_dir = root.join("tests/fixtures/bank_schema2");
    let schema1_str = schema1_dir.to_string_lossy().to_string();
    let schema2_str = schema2_dir.to_string_lossy().to_string();

    for entry in &manifest.entries {
        let out_wav = work_dir.join(format!("{}.wav", entry.name));
        let stems_dir = work_dir.join(format!("{}_stems", entry.name));
        let mut args: Vec<String> = match entry.name.as_str() {
            "cli_5000rpm_procedural" => vec![
                "--rpm",
                "5000",
                "--seconds",
                "0.5",
                "--warmup",
                "0.2",
                "--accel-seconds",
                "0.25",
                "--acoustic-scene",
            ]
            .into_iter()
            .map(str::to_string)
            .collect(),
            "cli_coast_lift_schema2" => vec![
                "--rpm".into(),
                "9000".into(),
                "--sweep-end-rpm".into(),
                "15000".into(),
                "--hold-before-lift".into(),
                "--seconds".into(),
                "1.0".into(),
                "--warmup".into(),
                "0.1".into(),
                "--accel-seconds".into(),
                "0.4".into(),
                "--acoustic-scene".into(),
                "--sample-layer-dir".into(),
                schema2_str.clone(),
            ],
            "cli_sample_only_schema1" => vec![
                "--engine-mode".into(),
                "sample-only".into(),
                "--rpm".into(),
                "4200".into(),
                "--sweep-end-rpm".into(),
                "14000".into(),
                "--coast-end-rpm".into(),
                "6000".into(),
                "--seconds".into(),
                "0.8".into(),
                "--warmup".into(),
                "0.1".into(),
                "--accel-seconds".into(),
                "0.5".into(),
                "--sample-layer-dir".into(),
                schema1_str.clone(),
            ],
            "cli_sweep_shift_schema1" => vec![
                "--rpm".into(),
                "4500".into(),
                "--sweep-end-rpm".into(),
                "16500".into(),
                "--coast-end-rpm".into(),
                "7000".into(),
                "--seconds".into(),
                "1.2".into(),
                "--warmup".into(),
                "0.1".into(),
                "--accel-seconds".into(),
                "0.6".into(),
                "--shift-sequence".into(),
                "u@0.350,d@0.850".into(),
                "--acoustic-scene".into(),
                "--sample-layer-dir".into(),
                schema1_str.clone(),
                "--transmission-gain".into(),
                "0.2".into(),
                "--gear".into(),
                "3".into(),
                "--gear-ratios".into(),
                "3.1,2.35,1.85,1.5,1.25,1.06".into(),
            ],
            other => panic!("unexpected CLI golden entry: {other}"),
        };
        args.push("--out".into());
        args.push(out_wav.to_string_lossy().to_string());
        args.push("--stems-dir".into());
        args.push(stems_dir.to_string_lossy().to_string());

        let output = Command::new(bin)
            .args(&args)
            .output()
            .unwrap_or_else(|e| panic!("failed spawning v10_render for {}: {e}", entry.name));
        assert!(
            output.status.success(),
            "v10_render failed for {}:\nstdout:{}\nstderr:{}",
            entry.name,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );

        let actual_wav = fs::read(&out_wav)
            .unwrap_or_else(|e| panic!("missing rendered WAV {}: {e}", out_wav.display()));
        let expected_wav_path = root.join(&entry.wav_path);
        let expected_wav = fs::read(&expected_wav_path)
            .unwrap_or_else(|e| panic!("missing golden WAV {}: {e}", expected_wav_path.display()));
        assert_eq!(
            sha256_bytes(&actual_wav),
            entry.wav_sha256,
            "WAV SHA-256 mismatch for {}",
            entry.name
        );
        assert_eq!(
            actual_wav, expected_wav,
            "WAV bytes differ from v1 golden for {}",
            entry.name
        );

        let decoded = v10_engine_synth::assets::read_mono_pcm16(&out_wav).expect("decode wav");
        assert_eq!(decoded.sample_rate, entry.sample_rate);
        assert_eq!(decoded.samples.len(), entry.total_frames);
        assert!(decoded.samples.iter().all(|s| s.is_finite()));

        let golden_events_path = root.join(format!("tests/golden/cli/{}.events.csv", entry.name));
        let actual_events_path = out_wav.with_extension("events.csv");
        assert_eq!(
            fs::read_to_string(&actual_events_path).expect("read rendered events csv"),
            fs::read_to_string(&golden_events_path).expect("read golden events csv"),
            "events.csv mismatch for {}",
            entry.name
        );

        let metadata_raw =
            fs::read_to_string(out_wav.with_extension("metadata.json")).expect("read metadata");
        let metadata: serde_json::Value =
            serde_json::from_str(&metadata_raw).expect("valid rendered metadata JSON");
        assert!(metadata["sample_rate"].as_u64() == Some(u64::from(entry.sample_rate)));

        let stem_count = fs::read_dir(&stems_dir)
            .expect("read stems dir")
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().and_then(|s| s.to_str()) == Some("wav"))
            .count();
        assert_eq!(stem_count, 61, "expected 61 stem WAVs for {}", entry.name);
    }
}
