#![forbid(unsafe_code)]

use std::env;
use std::path::PathBuf;

use v10_engine_synth::config::{
    CollectorGeometry, EngineMode, Gf509RuntimeConfig, ZONE_TRIM_SLOTS,
};
use v10_engine_synth::control::EngineInput;
use v10_engine_synth::dsp::SpectrumChainConfig;
use v10_engine_synth::offline::{
    apply_scene_gain_override, parse_shift_sequence, run_offline_render, OfflineRenderConfig,
};

fn parse_value<T: std::str::FromStr>(
    args: &[String],
    index: &mut usize,
    name: &str,
) -> Result<T, String> {
    *index += 1;
    args.get(*index)
        .ok_or_else(|| format!("missing value for {name}"))?
        .parse::<T>()
        .map_err(|_| format!("invalid value for {name}"))
}

fn select_runtime_preset(args: &[String]) -> Result<Gf509RuntimeConfig, String> {
    let mut preset = "v10-f1";
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--engine" {
            i += 1;
            preset = args
                .get(i)
                .ok_or_else(|| "missing value for --engine".to_string())?
                .as_str();
        }
        i += 1;
    }
    let mut runtime = match preset {
        "v10-f1" | "v10_f1" | "v10" => Gf509RuntimeConfig::default(),
        "pinto-rs2000" | "pinto_rs2000" | "escort-rs2000" | "rs2000" => {
            Gf509RuntimeConfig::ford_pinto_rs2000(48_000)
        }
        other => return Err(format!("unknown --engine preset: {other}")),
    };
    runtime.transmission.sample_rate = runtime.engine.sample_rate;
    Ok(runtime)
}

fn parse_args() -> Result<OfflineRenderConfig, String> {
    let raw: Vec<String> = env::args().skip(1).collect();
    let mut config = OfflineRenderConfig {
        runtime: select_runtime_preset(&raw)?,
        ..OfflineRenderConfig::default()
    };
    let mut exhaust_gain_override: Option<f32> = None;
    let mut collector_enabled = false;
    let mut collector =
        config
            .runtime
            .engine
            .collector_geometry
            .clone()
            .unwrap_or(CollectorGeometry {
                volume_l: 2.5,
                outlet_length_m: 0.35,
                outlet_diameter_m: 0.09,
                gas_temperature_k: 1_000.0,
                loss_fraction_per_cycle: 0.5,
                mode_coupling: 0.6,
            });
    let mut scene_gains: Vec<(String, f32)> = Vec::new();
    let mut cover_lowpass_hz: Option<f32> = None;
    let mut v10_filter = SpectrumChainConfig::default();
    let mut gearbox_filter = SpectrumChainConfig::default();

    let mut i = 0;
    while i < raw.len() {
        match raw[i].as_str() {
            "--rpm" => config.scenario.rpm = parse_value(&raw, &mut i, "--rpm")?,
            "--seconds" => config.scenario.seconds = parse_value(&raw, &mut i, "--seconds")?,
            "--warmup" => config.scenario.warmup = parse_value(&raw, &mut i, "--warmup")?,
            "--throttle" => config.scenario.throttle = parse_value(&raw, &mut i, "--throttle")?,
            "--load" => config.scenario.load = parse_value(&raw, &mut i, "--load")?,
            "--sample-rate" => {
                config.runtime.engine.sample_rate = parse_value(&raw, &mut i, "--sample-rate")?;
            }
            "--seed" => config.runtime.engine.seed = parse_value(&raw, &mut i, "--seed")?,
            "--header-length-scale" => {
                config.runtime.engine.header_length_scale =
                    parse_value(&raw, &mut i, "--header-length-scale")?;
            }
            "--exhaust-gain" => {
                exhaust_gain_override = Some(parse_value(&raw, &mut i, "--exhaust-gain")?);
            }
            "--collector-geometry" => collector_enabled = true,
            "--collector-volume-l" => {
                collector.volume_l = parse_value(&raw, &mut i, "--collector-volume-l")?;
            }
            "--collector-outlet-length-m" => {
                collector.outlet_length_m =
                    parse_value(&raw, &mut i, "--collector-outlet-length-m")?;
            }
            "--collector-outlet-diameter-m" => {
                collector.outlet_diameter_m =
                    parse_value(&raw, &mut i, "--collector-outlet-diameter-m")?;
            }
            "--collector-gas-temperature-k" => {
                collector.gas_temperature_k =
                    parse_value(&raw, &mut i, "--collector-gas-temperature-k")?;
            }
            "--collector-loss-fraction" => {
                collector.loss_fraction_per_cycle =
                    parse_value(&raw, &mut i, "--collector-loss-fraction")?;
            }
            "--collector-mode-coupling" => {
                collector.mode_coupling = parse_value(&raw, &mut i, "--collector-mode-coupling")?;
            }
            "--out" => config.out = PathBuf::from(parse_value::<String>(&raw, &mut i, "--out")?),
            "--stems-dir" => {
                config.stems_dir =
                    PathBuf::from(parse_value::<String>(&raw, &mut i, "--stems-dir")?);
            }
            "--acoustic-scene" => config.acoustic_scene = true,
            "--sample-layer-dir" => {
                config.runtime.sample_layer_directory = Some(PathBuf::from(parse_value::<String>(
                    &raw,
                    &mut i,
                    "--sample-layer-dir",
                )?));
            }
            "--sweep-end-rpm" => {
                config.scenario.sweep_end_rpm = Some(parse_value(&raw, &mut i, "--sweep-end-rpm")?);
            }
            "--hold-before-lift" => config.scenario.hold_before_lift = true,
            "--accel-seconds" => {
                config.scenario.accel_seconds = parse_value(&raw, &mut i, "--accel-seconds")?;
            }
            "--coast-end-rpm" => {
                config.scenario.coast_end_rpm = parse_value(&raw, &mut i, "--coast-end-rpm")?;
            }
            "--physical-telemetry-csv" => {
                config.physical_telemetry_csv = Some(PathBuf::from(parse_value::<String>(
                    &raw,
                    &mut i,
                    "--physical-telemetry-csv",
                )?));
            }
            "--physical-master-lowpass-hz" => {
                config.physical_master_lowpass_hz =
                    Some(parse_value(&raw, &mut i, "--physical-master-lowpass-hz")?);
            }
            "--no-physical-master-lowpass" => {
                config.physical_master_lowpass_hz = None;
            }
            "--no-sample-rasp" => config.runtime.sample_layer.disable_sample_rasp = true,
            "--sample-residual-scale" => {
                config.runtime.sample_layer.residual_gain_scale =
                    parse_value(&raw, &mut i, "--sample-residual-scale")?;
            }
            "--sample-residual-gain" => {
                config.sample_residual_gain_override =
                    Some(parse_value(&raw, &mut i, "--sample-residual-gain")?);
            }
            "--sample-blend-weight" => {
                config.sample_blend_weight_override =
                    Some(parse_value(&raw, &mut i, "--sample-blend-weight")?);
            }
            "--physical-blend-weight" => {
                config.physical_blend_weight_override =
                    Some(parse_value(&raw, &mut i, "--physical-blend-weight")?);
            }
            "--sample-zone-trim-db" => {
                let spec = parse_value::<String>(&raw, &mut i, "--sample-zone-trim-db")?;
                config.sample_zone_trim_db = spec
                    .split(',')
                    .map(|part| {
                        part.trim()
                            .parse::<f32>()
                            .map_err(|_| format!("invalid zone tonal trim: {part}"))
                    })
                    .collect::<Result<Vec<f32>, String>>()?;
            }
            "--cover-lowpass-hz" => {
                cover_lowpass_hz = Some(parse_value(&raw, &mut i, "--cover-lowpass-hz")?);
            }
            "--upper-mid-shelf-gain" => {
                config.runtime.upper_mid_shelf_gain =
                    parse_value(&raw, &mut i, "--upper-mid-shelf-gain")?;
            }
            "--gear" => config.gear = parse_value(&raw, &mut i, "--gear")?,
            "--transmission-gain" => {
                config.runtime.transmission.output_gain =
                    parse_value(&raw, &mut i, "--transmission-gain")?;
            }
            "--transmission-whine" => {
                config.runtime.transmission.whine_gain =
                    parse_value(&raw, &mut i, "--transmission-whine")?;
            }
            "--transmission-clack" => {
                config.runtime.transmission.clack_gain =
                    parse_value(&raw, &mut i, "--transmission-clack")?;
            }
            "--transmission-rattle" => {
                config.runtime.transmission.rattle_gain =
                    parse_value(&raw, &mut i, "--transmission-rattle")?;
            }
            "--transmission-clutch" => {
                config.runtime.transmission.clutch_gain =
                    parse_value(&raw, &mut i, "--transmission-clutch")?;
            }
            "--gear-ratios" => {
                let spec = parse_value::<String>(&raw, &mut i, "--gear-ratios")?;
                let mut values = Vec::new();
                for chunk in spec
                    .split(',')
                    .map(str::trim)
                    .filter(|part| !part.is_empty())
                {
                    values.push(
                        chunk
                            .parse::<f32>()
                            .map_err(|_| format!("invalid gear ratio: {chunk}"))?,
                    );
                }
                config.runtime.transmission.gear_ratios = values;
            }
            "--final-drive" => {
                config.runtime.transmission.final_drive =
                    parse_value(&raw, &mut i, "--final-drive")?;
            }
            "--reverse-ratio" => {
                config.runtime.transmission.reverse_ratio =
                    parse_value(&raw, &mut i, "--reverse-ratio")?;
            }
            "--gear-teeth" => {
                config.runtime.transmission.gear_teeth = parse_value(&raw, &mut i, "--gear-teeth")?;
            }
            "--final-teeth" => {
                config.runtime.transmission.final_teeth =
                    parse_value(&raw, &mut i, "--final-teeth")?;
            }
            "--gear-shift-gain" => {
                config.runtime.gear_shift_gain = parse_value(&raw, &mut i, "--gear-shift-gain")?;
            }
            "--gear-shift-reference-rpm" => {
                config.runtime.gear_shift_reference_rpm =
                    parse_value(&raw, &mut i, "--gear-shift-reference-rpm")?;
            }
            "--scene-gain" => {
                let spec = parse_value::<String>(&raw, &mut i, "--scene-gain")?;
                let (name, value) = spec.split_once('=').ok_or_else(|| {
                    "expected --scene-gain <branch>=<value>, e.g. --scene-gain metal=0.9"
                        .to_string()
                })?;
                let gain: f32 = value
                    .parse()
                    .map_err(|_| format!("invalid scene gain value: {value}"))?;
                scene_gains.push((name.to_string(), gain));
            }
            "--shift-sequence" => {
                let spec = parse_value::<String>(&raw, &mut i, "--shift-sequence")?;
                config.scenario.shift_sequence = parse_shift_sequence(&spec)?;
            }
            "--engine" => {
                let _ = parse_value::<String>(&raw, &mut i, "--engine")?;
            }
            "--engine-mode" => {
                let value = parse_value::<String>(&raw, &mut i, "--engine-mode")?;
                config.runtime.engine_mode = match value.as_str() {
                    "hybrid" => EngineMode::Hybrid,
                    "sample-only" | "sample_only" => EngineMode::SampleOnly,
                    other => return Err(format!("invalid --engine-mode: {other}")),
                };
            }
            "--sampled-gearbox-gain" => {
                config.runtime.sampled_gearbox_gain =
                    parse_value(&raw, &mut i, "--sampled-gearbox-gain")?;
            }
            "--sampled-engine-gain" => {
                config.runtime.sampled_engine_gain =
                    parse_value(&raw, &mut i, "--sampled-engine-gain")?;
            }
            "--v10-highpass-hz" => {
                v10_filter.highpass_hz = parse_value(&raw, &mut i, "--v10-highpass-hz")?;
            }
            "--v10-highpass-slope" => {
                v10_filter.highpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--v10-highpass-slope")?;
            }
            "--v10-lowpass-hz" => {
                v10_filter.lowpass_hz = parse_value(&raw, &mut i, "--v10-lowpass-hz")?;
            }
            "--v10-lowpass-slope" => {
                v10_filter.lowpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--v10-lowpass-slope")?;
            }
            "--v10-peak-hz" => {
                v10_filter.peak_hz = parse_value(&raw, &mut i, "--v10-peak-hz")?;
            }
            "--v10-peak-gain-db" => {
                v10_filter.peak_gain_db = parse_value(&raw, &mut i, "--v10-peak-gain-db")?;
            }
            "--v10-peak-q" => {
                v10_filter.peak_q = parse_value(&raw, &mut i, "--v10-peak-q")?;
            }
            "--v10-peak-drive" => {
                v10_filter.peak_drive = parse_value(&raw, &mut i, "--v10-peak-drive")?;
            }
            "--v10-peak-drive-mix" => {
                v10_filter.peak_drive_mix = parse_value(&raw, &mut i, "--v10-peak-drive-mix")?;
            }
            "--gearbox-highpass-hz" => {
                gearbox_filter.highpass_hz = parse_value(&raw, &mut i, "--gearbox-highpass-hz")?;
            }
            "--gearbox-highpass-slope" => {
                gearbox_filter.highpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--gearbox-highpass-slope")?;
            }
            "--gearbox-lowpass-hz" => {
                gearbox_filter.lowpass_hz = parse_value(&raw, &mut i, "--gearbox-lowpass-hz")?;
            }
            "--gearbox-lowpass-slope" => {
                gearbox_filter.lowpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--gearbox-lowpass-slope")?;
            }
            unknown => return Err(format!("unknown argument: {unknown}")),
        }
        i += 1;
    }

    if let Some(gain) = exhaust_gain_override {
        config.runtime.engine.exhaust_gain = gain;
        config.exhaust_gain_overridden = true;
    }
    if collector_enabled {
        config.runtime.engine.collector_geometry = Some(collector);
    }
    config.runtime.transmission.sample_rate = config.runtime.engine.sample_rate;

    if let Some(hz) = cover_lowpass_hz {
        config.runtime.scene.cover_radiation_lowpass_hz = hz;
    }
    for (name, gain) in &scene_gains {
        apply_scene_gain_override(&mut config.runtime.scene, name, *gain)?;
    }
    let mut scene_overrides = scene_gains
        .iter()
        .map(|(name, gain)| format!("{name}={gain}"))
        .collect::<Vec<_>>()
        .join(",");
    if let Some(hz) = cover_lowpass_hz {
        scene_overrides.push_str(&format!(",cover_lowpass_hz={hz}"));
    }
    config.scene_gain_overrides_label = scene_overrides;

    if let Some(residual) = config.sample_residual_gain_override {
        config.runtime.sample_layer.residual_gain_closed = residual;
        config.runtime.sample_layer.residual_gain_loaded = residual;
    }
    if let Some(weight) = config.sample_blend_weight_override {
        config.runtime.sample_layer.sample_blend_weight = weight;
    }
    if let Some(weight) = config.physical_blend_weight_override {
        config.runtime.sample_layer.physical_blend_weight = weight;
    }
    let mut zone_trim_db = [0.0f32; ZONE_TRIM_SLOTS];
    for (slot, value) in zone_trim_db
        .iter_mut()
        .zip(config.sample_zone_trim_db.iter())
    {
        *slot = *value;
    }
    config.runtime.sample_layer.zone_trim_db = zone_trim_db;
    config.runtime.v10_filter = v10_filter;
    config.runtime.gearbox_filter = gearbox_filter;

    config.scenario.validate()?;
    EngineInput {
        rpm: config.scenario.rpm,
        throttle: config.scenario.throttle,
        load: config.scenario.load,
    }
    .validate()?;
    if config.runtime.engine_mode == EngineMode::SampleOnly
        && config.runtime.sample_layer_directory.is_none()
    {
        return Err("--engine-mode sample-only requires --sample-layer-dir".into());
    }
    if !config.runtime.sampled_gearbox_gain.is_finite() || config.runtime.sampled_gearbox_gain < 0.0
    {
        return Err("--sampled-gearbox-gain out of range".into());
    }
    if !config.runtime.sampled_engine_gain.is_finite() || config.runtime.sampled_engine_gain < 0.0 {
        return Err("--sampled-engine-gain out of range".into());
    }

    Ok(config)
}

fn run() -> Result<(), String> {
    let config = parse_args()?;
    let summary = run_offline_render(&config)?;
    println!("wav={}", summary.out.display());
    println!("stems={}", summary.stems_dir.display());
    println!(
        "events={} peak={:.6} rms={:.6} limiter_db={:.3}",
        summary.event_count, summary.peak, summary.rms, summary.worst_limiter_reduction_db
    );
    Ok(())
}

fn main() {
    if let Err(err) = run() {
        eprintln!("{err}");
        std::process::exit(1);
    }
}
