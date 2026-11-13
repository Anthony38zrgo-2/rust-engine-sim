use std::collections::BTreeMap;
use std::env;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use v10_engine_synth::runtime::blend_hybrid;
use v10_engine_synth::wav::write_mono_pcm16;
use v10_engine_synth::{
    shift_energy_gain, AcousticFrame, AcousticScene, AcousticSceneConfig, CollectorGeometry,
    Crankshaft, EngineConfig, EngineFrame, EngineInput, EngineMode, GearShiftPlayer,
    GearShiftSamples, SampleLayerInput, ShiftGesture, ShiftPhase, SpectrumChain,
    SpectrumChainConfig, ThreeZoneSampleLayer, ThreeZoneSampleLayerConfig, TransmissionConfig,
    TransmissionInput, TransmissionSynth, UpperMidShelf, V10Engine,
};

const HYBRID_HEADROOM_GAIN: f32 = 0.61;

/// Audition timings for `--shift-sequence`. Mirrors the acoustic telemetry
/// adapter: cut, optional downshift blip, then recovery.
const SHIFT_CUT_SECONDS: f32 = 0.250;
const SHIFT_BLIP_SECONDS: f32 = 0.120;
const SHIFT_RECOVERY_SECONDS: f32 = 0.040;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ShiftDirection {
    Up,
    Down,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ShiftEvent {
    time_s: f32,
    direction: ShiftDirection,
}

/// Parses `--shift-sequence "u@2.5,d@7.2"`; events must be strictly ascending.
fn parse_shift_sequence(spec: &str) -> Result<Vec<ShiftEvent>, String> {
    let mut events: Vec<ShiftEvent> = Vec::new();
    for part in spec
        .split(',')
        .map(str::trim)
        .filter(|part| !part.is_empty())
    {
        let (token, time) = part
            .split_once('@')
            .ok_or_else(|| format!("expected <u|d>@<seconds>: {part}"))?;
        let direction = match token.trim() {
            "u" => ShiftDirection::Up,
            "d" => ShiftDirection::Down,
            other => return Err(format!("unknown shift direction: {other}")),
        };
        let time_s: f32 = time
            .trim()
            .parse()
            .map_err(|_| format!("invalid shift time: {part}"))?;
        if !time_s.is_finite() || time_s < 0.0 {
            return Err(format!("shift time out of range: {part}"));
        }
        if events.last().is_some_and(|last| time_s <= last.time_s) {
            return Err(format!("shift times must be strictly ascending: {part}"));
        }
        events.push(ShiftEvent { time_s, direction });
    }
    Ok(events)
}

fn validate_shift_sequence(events: &[ShiftEvent], seconds: f32) -> Result<(), String> {
    if let Some(last) = events.last() {
        if last.time_s >= seconds {
            return Err("--shift-sequence events must start within --seconds".into());
        }
    }
    Ok(())
}

/// Phase for the latest event that has started; later events take precedence.
fn shift_phase_at(events: &[ShiftEvent], time_s: f32) -> ShiftPhase {
    let Some(event) = events.iter().rev().find(|event| time_s >= event.time_s) else {
        return ShiftPhase::None;
    };
    let elapsed = time_s - event.time_s;
    if elapsed < SHIFT_CUT_SECONDS {
        return match event.direction {
            ShiftDirection::Up => ShiftPhase::UpshiftCut,
            ShiftDirection::Down => ShiftPhase::DownshiftCut,
        };
    }
    let blip_end = SHIFT_CUT_SECONDS
        + if event.direction == ShiftDirection::Down {
            SHIFT_BLIP_SECONDS
        } else {
            0.0
        };
    if elapsed < blip_end {
        return ShiftPhase::DownshiftBlip;
    }
    if elapsed < blip_end + SHIFT_RECOVERY_SECONDS {
        return match event.direction {
            ShiftDirection::Up => ShiftPhase::UpshiftRecovery,
            ShiftDirection::Down => ShiftPhase::DownshiftRecovery,
        };
    }
    ShiftPhase::None
}

fn shift_sequence_label(events: &[ShiftEvent]) -> String {
    if events.is_empty() {
        return "none".into();
    }
    events
        .iter()
        .map(|event| {
            let token = match event.direction {
                ShiftDirection::Up => "u",
                ShiftDirection::Down => "d",
            };
            format!("{token}@{:.3}", event.time_s)
        })
        .collect::<Vec<_>>()
        .join(",")
}

struct OnePoleLowPass {
    alpha: f32,
    state: f32,
}

impl OnePoleLowPass {
    fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        Self {
            alpha: 1.0 - (-std::f32::consts::TAU * cutoff_hz / sample_rate).exp(),
            state: 0.0,
        }
    }

    #[inline]
    fn process(&mut self, input: f32) -> f32 {
        self.state += self.alpha * (input - self.state);
        self.state
    }
}

/// 3rd-order (-18 dB/octave) all-pole lowpass filter cascade for natural, smooth, non-resonant rolloff.
struct ThreePoleLowPass {
    p1: OnePoleLowPass,
    p2: OnePoleLowPass,
    p3: OnePoleLowPass,
}

impl ThreePoleLowPass {
    fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        Self {
            p1: OnePoleLowPass::new(cutoff_hz, sample_rate),
            p2: OnePoleLowPass::new(cutoff_hz, sample_rate),
            p3: OnePoleLowPass::new(cutoff_hz, sample_rate),
        }
    }

    #[inline]
    fn process(&mut self, input: f32) -> f32 {
        self.p3.process(self.p2.process(self.p1.process(input)))
    }
}

struct Args {
    rpm: f32,
    seconds: f32,
    warmup: f32,
    throttle: f32,
    load: f32,
    sample_rate: u32,
    seed: u64,
    header_length_scale: f32,
    exhaust_gain: Option<f32>,
    collector_geometry: bool,
    collector_volume_l: f32,
    collector_outlet_length_m: f32,
    collector_outlet_diameter_m: f32,
    collector_gas_temperature_k: f32,
    collector_loss_fraction: f32,
    collector_mode_coupling: f32,
    out: PathBuf,
    stems_dir: PathBuf,
    acoustic_scene: bool,
    sample_layer_dir: Option<PathBuf>,
    sweep_end_rpm: Option<f32>,
    hold_before_lift: bool,
    accel_seconds: f32,
    coast_end_rpm: f32,
    physical_telemetry_csv: Option<PathBuf>,
    physical_master_lowpass_hz: Option<f32>,
    no_sample_rasp: bool,
    sample_residual_scale: f32,
    sample_residual_gain: Option<f32>,
    sample_blend_weight: Option<f32>,
    physical_blend_weight: Option<f32>,
    sample_zone_trim_db: Vec<f32>,
    scene_gains: Vec<(String, f32)>,
    cover_lowpass_hz: Option<f32>,
    upper_mid_shelf_gain: f32,
    gear: i8,
    transmission_gain: f32,
    transmission_whine_gain: f32,
    transmission_clack_gain: f32,
    transmission_rattle_gain: f32,
    transmission_clutch_gain: f32,
    gear_ratios: Vec<f32>,
    final_drive: f32,
    reverse_ratio: f32,
    gear_teeth: f32,
    final_teeth: f32,
    gear_shift_gain: f32,
    gear_shift_reference_rpm: f32,
    shift_sequence: Vec<ShiftEvent>,
    engine_mode: EngineMode,
    sampled_engine_gain: f32,
    sampled_gearbox_gain: f32,
    v10_filter: SpectrumChainConfig,
    gearbox_filter: SpectrumChainConfig,
}

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

fn parse_args() -> Result<Args, String> {
    let raw: Vec<String> = env::args().skip(1).collect();
    let mut parsed = Args {
        rpm: 5_000.0,
        seconds: 6.0,
        warmup: 1.0,
        throttle: 0.72,
        load: 0.78,
        sample_rate: 48_000,
        seed: 0xF090_0010,
        header_length_scale: 1.0,
        exhaust_gain: None,
        collector_geometry: false,
        collector_volume_l: 2.5,
        collector_outlet_length_m: 0.35,
        collector_outlet_diameter_m: 0.09,
        collector_gas_temperature_k: 1_000.0,
        collector_loss_fraction: 0.5,
        collector_mode_coupling: 0.6,
        out: PathBuf::from("reports/audio/rust-greenfield/gf310_5000rpm.wav"),
        stems_dir: PathBuf::from("reports/audio/rust-greenfield/gf310_5000rpm_stems"),
        acoustic_scene: false,
        sample_layer_dir: None,
        sweep_end_rpm: None,
        hold_before_lift: false,
        accel_seconds: 7.0,
        coast_end_rpm: 6_500.0,
        physical_telemetry_csv: None,
        physical_master_lowpass_hz: Some(2_500.0),
        no_sample_rasp: false,
        sample_residual_scale: 1.0,
        sample_residual_gain: None,
        sample_blend_weight: None,
        physical_blend_weight: None,
        sample_zone_trim_db: Vec::new(),
        scene_gains: Vec::new(),
        cover_lowpass_hz: None,
        upper_mid_shelf_gain: 0.0,
        gear: 4,
        transmission_gain: 0.0,
        transmission_whine_gain: 1.0,
        transmission_clack_gain: 1.0,
        transmission_rattle_gain: 1.0,
        transmission_clutch_gain: 1.0,
        gear_ratios: Vec::new(),
        final_drive: 1.0,
        reverse_ratio: 3.0,
        gear_teeth: 20.0,
        final_teeth: 40.0,
        gear_shift_gain: 1.0,
        gear_shift_reference_rpm: 10_000.0,
        shift_sequence: Vec::new(),
        engine_mode: EngineMode::Hybrid,
        sampled_engine_gain: 1.0,
        sampled_gearbox_gain: 1.0,
        v10_filter: SpectrumChainConfig::default(),
        gearbox_filter: SpectrumChainConfig::default(),
    };
    let mut i = 0;
    while i < raw.len() {
        match raw[i].as_str() {
            "--rpm" => parsed.rpm = parse_value(&raw, &mut i, "--rpm")?,
            "--seconds" => parsed.seconds = parse_value(&raw, &mut i, "--seconds")?,
            "--warmup" => parsed.warmup = parse_value(&raw, &mut i, "--warmup")?,
            "--throttle" => parsed.throttle = parse_value(&raw, &mut i, "--throttle")?,
            "--load" => parsed.load = parse_value(&raw, &mut i, "--load")?,
            "--sample-rate" => parsed.sample_rate = parse_value(&raw, &mut i, "--sample-rate")?,
            "--seed" => parsed.seed = parse_value(&raw, &mut i, "--seed")?,
            "--header-length-scale" => {
                parsed.header_length_scale =
                    parse_value(&raw, &mut i, "--header-length-scale")?
            }
            "--exhaust-gain" => {
                parsed.exhaust_gain = Some(parse_value(&raw, &mut i, "--exhaust-gain")?)
            }
            "--collector-geometry" => parsed.collector_geometry = true,
            "--collector-volume-l" => {
                parsed.collector_volume_l = parse_value(&raw, &mut i, "--collector-volume-l")?
            }
            "--collector-outlet-length-m" => {
                parsed.collector_outlet_length_m =
                    parse_value(&raw, &mut i, "--collector-outlet-length-m")?
            }
            "--collector-outlet-diameter-m" => {
                parsed.collector_outlet_diameter_m =
                    parse_value(&raw, &mut i, "--collector-outlet-diameter-m")?
            }
            "--collector-gas-temperature-k" => {
                parsed.collector_gas_temperature_k =
                    parse_value(&raw, &mut i, "--collector-gas-temperature-k")?
            }
            "--collector-loss-fraction" => {
                parsed.collector_loss_fraction =
                    parse_value(&raw, &mut i, "--collector-loss-fraction")?
            }
            "--collector-mode-coupling" => {
                parsed.collector_mode_coupling =
                    parse_value(&raw, &mut i, "--collector-mode-coupling")?
            }
            "--out" => parsed.out = PathBuf::from(parse_value::<String>(&raw, &mut i, "--out")?),
            "--stems-dir" => {
                parsed.stems_dir =
                    PathBuf::from(parse_value::<String>(&raw, &mut i, "--stems-dir")?)
            }
            "--acoustic-scene" => parsed.acoustic_scene = true,
            "--sample-layer-dir" => {
                parsed.sample_layer_dir = Some(PathBuf::from(parse_value::<String>(
                    &raw,
                    &mut i,
                    "--sample-layer-dir",
                )?))
            }
            "--sweep-end-rpm" => {
                parsed.sweep_end_rpm = Some(parse_value(&raw, &mut i, "--sweep-end-rpm")?)
            }
            "--hold-before-lift" => parsed.hold_before_lift = true,
            "--accel-seconds" => {
                parsed.accel_seconds = parse_value(&raw, &mut i, "--accel-seconds")?
            }
            "--coast-end-rpm" => {
                parsed.coast_end_rpm = parse_value(&raw, &mut i, "--coast-end-rpm")?
            }
            "--physical-telemetry-csv" => {
                parsed.physical_telemetry_csv = Some(PathBuf::from(parse_value::<String>(
                    &raw,
                    &mut i,
                    "--physical-telemetry-csv",
                )?))
            }
            "--physical-master-lowpass-hz" => {
                parsed.physical_master_lowpass_hz =
                    Some(parse_value(&raw, &mut i, "--physical-master-lowpass-hz")?)
            }
            "--no-physical-master-lowpass" => {
                parsed.physical_master_lowpass_hz = None;
            }
            "--no-sample-rasp" => parsed.no_sample_rasp = true,
            "--sample-residual-scale" => {
                parsed.sample_residual_scale =
                    parse_value(&raw, &mut i, "--sample-residual-scale")?
            }
            "--sample-residual-gain" => {
                parsed.sample_residual_gain =
                    Some(parse_value(&raw, &mut i, "--sample-residual-gain")?)
            }
            "--sample-blend-weight" => {
                parsed.sample_blend_weight =
                    Some(parse_value(&raw, &mut i, "--sample-blend-weight")?)
            }
            "--physical-blend-weight" => {
                parsed.physical_blend_weight =
                    Some(parse_value(&raw, &mut i, "--physical-blend-weight")?)
            }
            "--sample-zone-trim-db" => {
                let spec = parse_value::<String>(&raw, &mut i, "--sample-zone-trim-db")?;
                parsed.sample_zone_trim_db = spec
                    .split(',')
                    .map(|part| {
                        part.trim()
                            .parse::<f32>()
                            .map_err(|_| format!("invalid zone tonal trim: {part}"))
                    })
                    .collect::<Result<Vec<f32>, String>>()?;
            }
            "--cover-lowpass-hz" => {
                parsed.cover_lowpass_hz = Some(parse_value(&raw, &mut i, "--cover-lowpass-hz")?)
            }
            "--upper-mid-shelf-gain" => {
                parsed.upper_mid_shelf_gain =
                    parse_value(&raw, &mut i, "--upper-mid-shelf-gain")?
            }
            "--gear" => parsed.gear = parse_value(&raw, &mut i, "--gear")?,
            "--transmission-gain" => {
                parsed.transmission_gain = parse_value(&raw, &mut i, "--transmission-gain")?
            }
            "--transmission-whine" => {
                parsed.transmission_whine_gain =
                    parse_value(&raw, &mut i, "--transmission-whine")?
            }
            "--transmission-clack" => {
                parsed.transmission_clack_gain =
                    parse_value(&raw, &mut i, "--transmission-clack")?
            }
            "--transmission-rattle" => {
                parsed.transmission_rattle_gain =
                    parse_value(&raw, &mut i, "--transmission-rattle")?
            }
            "--transmission-clutch" => {
                parsed.transmission_clutch_gain =
                    parse_value(&raw, &mut i, "--transmission-clutch")?
            }
            "--gear-ratios" => {
                let spec = parse_value::<String>(&raw, &mut i, "--gear-ratios")?;
                let mut values = Vec::new();
                for chunk in spec.split(',').map(str::trim).filter(|part| !part.is_empty()) {
                    values.push(
                        chunk
                            .parse::<f32>()
                            .map_err(|_| format!("invalid gear ratio: {chunk}"))?,
                    );
                }
                parsed.gear_ratios = values;
            }
            "--final-drive" => {
                parsed.final_drive = parse_value(&raw, &mut i, "--final-drive")?
            }
            "--reverse-ratio" => {
                parsed.reverse_ratio = parse_value(&raw, &mut i, "--reverse-ratio")?
            }
            "--gear-teeth" => {
                parsed.gear_teeth = parse_value(&raw, &mut i, "--gear-teeth")?
            }
            "--final-teeth" => {
                parsed.final_teeth = parse_value(&raw, &mut i, "--final-teeth")?
            }
            "--gear-shift-gain" => {
                parsed.gear_shift_gain = parse_value(&raw, &mut i, "--gear-shift-gain")?
            }
            "--gear-shift-reference-rpm" => {
                parsed.gear_shift_reference_rpm =
                    parse_value(&raw, &mut i, "--gear-shift-reference-rpm")?
            }
            "--scene-gain" => {
                let spec = parse_value::<String>(&raw, &mut i, "--scene-gain")?;
                let (name, value) = spec.split_once('=').ok_or_else(|| {
                    "expected --scene-gain <branch>=<value>, e.g. --scene-gain metal=0.9".to_string()
                })?;
                let gain: f32 = value
                    .parse()
                    .map_err(|_| format!("invalid scene gain value: {value}"))?;
                parsed.scene_gains.push((name.to_string(), gain));
            }
            "--shift-sequence" => {
                let spec = parse_value::<String>(&raw, &mut i, "--shift-sequence")?;
                parsed.shift_sequence = parse_shift_sequence(&spec)?;
            }
            "--engine-mode" => {
                let value = parse_value::<String>(&raw, &mut i, "--engine-mode")?;
                parsed.engine_mode = match value.as_str() {
                    "hybrid" => EngineMode::Hybrid,
                    "sample-only" | "sample_only" => EngineMode::SampleOnly,
                    other => return Err(format!("invalid --engine-mode: {other}")),
                };
            }
            "--sampled-gearbox-gain" => {
                parsed.sampled_gearbox_gain =
                    parse_value(&raw, &mut i, "--sampled-gearbox-gain")?;
            }
            "--sampled-engine-gain" => {
                parsed.sampled_engine_gain =
                    parse_value(&raw, &mut i, "--sampled-engine-gain")?;
            }
            "--v10-highpass-hz" => {
                parsed.v10_filter.highpass_hz =
                    parse_value(&raw, &mut i, "--v10-highpass-hz")?
            }
            "--v10-highpass-slope" => {
                parsed.v10_filter.highpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--v10-highpass-slope")?
            }
            "--v10-lowpass-hz" => {
                parsed.v10_filter.lowpass_hz = parse_value(&raw, &mut i, "--v10-lowpass-hz")?
            }
            "--v10-lowpass-slope" => {
                parsed.v10_filter.lowpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--v10-lowpass-slope")?
            }
            "--v10-peak-hz" => {
                parsed.v10_filter.peak_hz = parse_value(&raw, &mut i, "--v10-peak-hz")?
            }
            "--v10-peak-gain-db" => {
                parsed.v10_filter.peak_gain_db =
                    parse_value(&raw, &mut i, "--v10-peak-gain-db")?
            }
            "--v10-peak-q" => {
                parsed.v10_filter.peak_q = parse_value(&raw, &mut i, "--v10-peak-q")?
            }
            "--v10-peak-drive" => {
                parsed.v10_filter.peak_drive =
                    parse_value(&raw, &mut i, "--v10-peak-drive")?
            }
            "--v10-peak-drive-mix" => {
                parsed.v10_filter.peak_drive_mix =
                    parse_value(&raw, &mut i, "--v10-peak-drive-mix")?
            }
            "--gearbox-highpass-hz" => {
                parsed.gearbox_filter.highpass_hz =
                    parse_value(&raw, &mut i, "--gearbox-highpass-hz")?
            }
            "--gearbox-highpass-slope" => {
                parsed.gearbox_filter.highpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--gearbox-highpass-slope")?
            }
            "--gearbox-lowpass-hz" => {
                parsed.gearbox_filter.lowpass_hz =
                    parse_value(&raw, &mut i, "--gearbox-lowpass-hz")?
            }
            "--gearbox-lowpass-slope" => {
                parsed.gearbox_filter.lowpass_slope_db_per_oct =
                    parse_value(&raw, &mut i, "--gearbox-lowpass-slope")?
            }
            unknown => return Err(format!("unknown argument: {unknown}")),
        }
        i += 1;
    }
    if !parsed.seconds.is_finite() || !(0.1..=120.0).contains(&parsed.seconds) {
        return Err("--seconds must be in 0.1..120".into());
    }
    if !parsed.warmup.is_finite() || !(0.0..=10.0).contains(&parsed.warmup) {
        return Err("--warmup must be in 0..10".into());
    }
    if !parsed.accel_seconds.is_finite()
        || parsed.accel_seconds <= 0.0
        || parsed.accel_seconds >= parsed.seconds
    {
        return Err("--accel-seconds must be inside the render duration".into());
    }
    EngineInput {
        rpm: parsed.rpm,
        throttle: parsed.throttle,
        load: parsed.load,
    }
    .validate()?;
    if let Some(end_rpm) = parsed.sweep_end_rpm {
        EngineInput {
            rpm: end_rpm,
            throttle: parsed.throttle,
            load: parsed.load,
        }
        .validate()?;
        EngineInput {
            rpm: parsed.coast_end_rpm,
            throttle: 0.0,
            load: 0.0,
        }
        .validate()?;
    }
    if parsed.hold_before_lift && parsed.sweep_end_rpm.is_none() {
        return Err("--hold-before-lift requires --sweep-end-rpm".into());
    }
    validate_shift_sequence(&parsed.shift_sequence, parsed.seconds)?;
    if parsed.engine_mode == EngineMode::SampleOnly && parsed.sample_layer_dir.is_none() {
        return Err("--engine-mode sample-only requires --sample-layer-dir".into());
    }
    if !parsed.sampled_gearbox_gain.is_finite() || parsed.sampled_gearbox_gain < 0.0 {
        return Err("--sampled-gearbox-gain out of range".into());
    }
    if !parsed.sampled_engine_gain.is_finite() || parsed.sampled_engine_gain < 0.0 {
        return Err("--sampled-engine-gain out of range".into());
    }
    Ok(parsed)
}

fn render_input(args: &Args, time_s: f32) -> EngineInput {
    let Some(end_rpm) = args.sweep_end_rpm else {
        return EngineInput {
            rpm: args.rpm,
            throttle: args.throttle,
            load: args.load,
        };
    };
    if args.hold_before_lift {
        let coast_end_rpm = end_rpm;
        if time_s < args.accel_seconds {
            return EngineInput {
                rpm: args.rpm,
                throttle: args.throttle,
                load: args.load,
            };
        }
        let coast_duration = (args.seconds - args.accel_seconds).max(1.0e-6);
        let t = ((time_s - args.accel_seconds) / coast_duration).clamp(0.0, 1.0);
        let rpm_decay = 1.0 - (1.0 - t).powf(1.55);
        let lift = (-t / 0.035).exp();
        return EngineInput {
            rpm: args.rpm + (coast_end_rpm - args.rpm) * rpm_decay,
            throttle: 0.035 + (args.throttle - 0.035) * lift,
            load: 0.10 + (args.load - 0.10) * (-t / 0.12).exp(),
        };
    }
    if time_s < args.accel_seconds {
        let t = (time_s / args.accel_seconds).clamp(0.0, 1.0);
        // Smoothstep avoids an acceleration discontinuity at either endpoint.
        let shaped = t * t * (3.0 - 2.0 * t);
        EngineInput {
            rpm: args.rpm + (end_rpm - args.rpm) * shaped,
            throttle: args.throttle,
            load: args.load,
        }
    } else {
        let coast_duration = (args.seconds - args.accel_seconds).max(1.0e-6);
        let t = ((time_s - args.accel_seconds) / coast_duration).clamp(0.0, 1.0);
        let rpm_decay = 1.0 - (1.0 - t).powf(1.55);
        let lift = (-t / 0.035).exp();
        EngineInput {
            rpm: end_rpm + (args.coast_end_rpm - end_rpm) * rpm_decay,
            throttle: 0.035 + (args.throttle - 0.035) * lift,
            load: 0.10 + (args.load - 0.10) * (-t / 0.12).exp(),
        }
    }
}

fn exhaust_gain_label(args: &Args) -> String {
    match args.exhaust_gain {
        Some(gain) => format!("{gain:.6}"),
        None => "baseline".into(),
    }
}

fn collector_geometry_label(args: &Args) -> String {
    if !args.collector_geometry {
        return "off".into();
    }
    format!(
        "volume_l={:.4},outlet_length_m={:.4},outlet_diameter_m={:.4},gas_temperature_k={:.1},loss_fraction_per_cycle={:.4},mode_coupling={:.4}",
        args.collector_volume_l,
        args.collector_outlet_length_m,
        args.collector_outlet_diameter_m,
        args.collector_gas_temperature_k,
        args.collector_loss_fraction,
        args.collector_mode_coupling,
    )
}

fn git_head() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned())
        .unwrap_or_else(|| "unknown".into())
}

const SOURCE_PATHS: [&str; 15] = [
    "game/crates/v10-engine-synth/src/acoustics.rs",
    "game/crates/v10-engine-synth/src/config.rs",
    "game/crates/v10-engine-synth/src/crank.rs",
    "game/crates/v10-engine-synth/src/cylinder.rs",
    "game/crates/v10-engine-synth/src/engine.rs",
    "game/crates/v10-engine-synth/src/geometry.rs",
    "game/crates/v10-engine-synth/src/thermodynamics/combustion.rs",
    "game/crates/v10-engine-synth/src/thermodynamics/gas.rs",
    "game/crates/v10-engine-synth/src/thermodynamics/polytrope.rs",
    "game/crates/v10-engine-synth/src/thermodynamics/exhaust_runner.rs",
    "game/crates/v10-engine-synth/src/thermodynamics/exhaust_valve.rs",
    "game/crates/v10-engine-synth/src/sample_layer.rs",
    "game/crates/v10-engine-synth/src/scene.rs",
    "game/crates/v10-engine-synth/src/runtime.rs",
    "game/crates/v10-engine-synth/src/bin/v10_render.rs",
];

fn source_dirty() -> bool {
    let dirty = |cached: bool| {
        let mut command = std::process::Command::new("git");
        command.args(["diff", "--quiet"]);
        if cached {
            command.arg("--cached");
        }
        command.arg("--").args(SOURCE_PATHS);
        command
            .status()
            .map(|status| !status.success())
            .unwrap_or(true)
    };
    dirty(false) || dirty(true)
}

fn source_fingerprint() -> String {
    // Stable FNV-1a over path names and working-tree bytes. This is provenance,
    // not a cryptographic signature.
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for path in SOURCE_PATHS {
        for byte in path.bytes().chain([0]) {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
        match fs::read(path) {
            Ok(bytes) => {
                for byte in bytes {
                    hash ^= byte as u64;
                    hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
                }
            }
            Err(_) => return "unavailable".into(),
        }
    }
    format!("fnv1a64:{hash:016x}")
}

fn main() {
    if let Err(error) = run() {
        eprintln!("v10_render: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let mut config = EngineConfig {
        sample_rate: args.sample_rate,
        seed: args.seed,
        header_length_scale: args.header_length_scale,
        collector_geometry: args.collector_geometry.then(|| CollectorGeometry {
            volume_l: args.collector_volume_l,
            outlet_length_m: args.collector_outlet_length_m,
            outlet_diameter_m: args.collector_outlet_diameter_m,
            gas_temperature_k: args.collector_gas_temperature_k,
            loss_fraction_per_cycle: args.collector_loss_fraction,
            mode_coupling: args.collector_mode_coupling,
        }),
        ..EngineConfig::default()
    };
    let gear_shift_samples = match args.sample_layer_dir.as_deref() {
        Some(directory) => GearShiftSamples::load_directory(directory, args.sample_rate)?,
        None => None,
    };
    if gear_shift_samples.is_some() {
        config.blip_enabled = false;
    }
    let mut gear_shift_player = GearShiftPlayer::new(
        gear_shift_samples,
        args.gear_shift_gain,
        args.gear_shift_reference_rpm,
        args.sample_rate,
    )?;
    if let Some(gain) = args.exhaust_gain {
        config.exhaust_gain = gain;
    }
    let mut engine = V10Engine::new(config.clone())?;
    let mut scene_config = AcousticSceneConfig::default();
    if let Some(hz) = args.cover_lowpass_hz {
        scene_config.cover_radiation_lowpass_hz = hz;
    }
    for (name, gain) in &args.scene_gains {
        let slot = match name.as_str() {
            "dry_low" => &mut scene_config.dry_low_gain,
            "dry_mid" => &mut scene_config.dry_mid_gain,
            "dry_high" => &mut scene_config.dry_high_gain,
            "engine_air" => &mut scene_config.engine_air_gain,
            "engine_cover" => &mut scene_config.engine_cover_gain,
            "mount_monocoque" => &mut scene_config.mount_monocoque_gain,
            "transmission_mount" => &mut scene_config.transmission_mount_gain,
            "transmission_cover" => &mut scene_config.transmission_cover_gain,
            "air_tilt_db" => &mut scene_config.air_high_tilt_db,
            "air_direct" => &mut scene_config.air_direct_gain,
            "output" => &mut scene_config.output_gain,
            _ => return Err(format!("unknown scene branch: {name}")),
        };
        *slot = *gain;
    }
    let mut scene_gain_overrides = args
        .scene_gains
        .iter()
        .map(|(name, gain)| format!("{name}={gain}"))
        .collect::<Vec<_>>()
        .join(",");
    if let Some(hz) = args.cover_lowpass_hz {
        scene_gain_overrides.push_str(&format!(",cover_lowpass_hz={hz}"));
    }
    let sample_layer_tuning = format!(
        "sample_blend_weight={},physical_blend_weight={},zone_trim_db={}",
        args.sample_blend_weight
            .map(|value| format!("{value}"))
            .unwrap_or_else(|| "baseline".to_string()),
        args.physical_blend_weight
            .map(|value| format!("{value}"))
            .unwrap_or_else(|| "baseline".to_string()),
        if args.sample_zone_trim_db.is_empty() {
            "baseline".to_string()
        } else {
            args.sample_zone_trim_db
                .iter()
                .map(|value| format!("{value}"))
                .collect::<Vec<_>>()
                .join("|")
        }
    );
    let mut scene = AcousticScene::new(args.sample_rate as f32, scene_config)?;
    let mut upper_mid_shelf =
        UpperMidShelf::new(args.sample_rate as f32, args.upper_mid_shelf_gain)?;
    let mut transmission = TransmissionSynth::new(TransmissionConfig {
        sample_rate: args.sample_rate,
        output_gain: args.transmission_gain,
        gear_ratios: args.gear_ratios.clone(),
        reverse_ratio: args.reverse_ratio,
        final_drive: args.final_drive,
        gear_teeth: args.gear_teeth,
        final_teeth: args.final_teeth,
        whine_gain: args.transmission_whine_gain,
        clack_gain: args.transmission_clack_gain,
        rattle_gain: args.transmission_rattle_gain,
        clutch_gain: args.transmission_clutch_gain,
        ..TransmissionConfig::default()
    })?;
    let mut zone_trim_db = [0.0f32; v10_engine_synth::ZONE_TRIM_SLOTS];
    for (slot, value) in zone_trim_db.iter_mut().zip(args.sample_zone_trim_db.iter()) {
        *slot = *value;
    }
    let mut sample_layer = args
        .sample_layer_dir
        .as_ref()
        .map(|directory| {
            ThreeZoneSampleLayer::load_directory(
                args.sample_rate,
                directory,
                ThreeZoneSampleLayerConfig {
                    disable_sample_rasp: args.no_sample_rasp,
                    residual_gain_scale: args.sample_residual_scale,
                    residual_gain_closed: args.sample_residual_gain.unwrap_or(
                        ThreeZoneSampleLayerConfig::default().residual_gain_closed,
                    ),
                    residual_gain_loaded: args.sample_residual_gain.unwrap_or(
                        ThreeZoneSampleLayerConfig::default().residual_gain_loaded,
                    ),
                    sample_blend_weight: args
                        .sample_blend_weight
                        .unwrap_or(ThreeZoneSampleLayerConfig::default().sample_blend_weight),
                    physical_blend_weight: args.physical_blend_weight.unwrap_or(
                        ThreeZoneSampleLayerConfig::default().physical_blend_weight,
                    ),
                    zone_trim_db,
                    ..ThreeZoneSampleLayerConfig::default()
                },
            )
        })
        .transpose()?;
    engine.set_input(EngineInput {
        rpm: args.rpm,
        throttle: args.throttle,
        load: args.load,
    })?;
    let sample_only = args.engine_mode == EngineMode::SampleOnly;
    let mut sampled_crank = Crankshaft::new(args.sample_rate, EngineConfig::default().firing_order);
    let mut sampled_gesture = ShiftGesture::default();
    let sample_blend_weight = args
        .sample_blend_weight
        .unwrap_or(ThreeZoneSampleLayerConfig::default().sample_blend_weight);
    let physical_blend_weight = args.physical_blend_weight.unwrap_or(
        ThreeZoneSampleLayerConfig::default().physical_blend_weight,
    );
    let mut v10_chain = SpectrumChain::new(args.sample_rate as f32, args.v10_filter)?;
    let mut gearbox_chain = SpectrumChain::new(args.sample_rate as f32, args.gearbox_filter)?;

    let warmup_samples = (args.warmup * args.sample_rate as f32).round() as usize;
    for _ in 0..warmup_samples {
        if sample_only {
            // The sampled engine bypasses the acoustic scene entirely: warmup
            // only advances the crank phase and the bank's phase alignment.
            let events = sampled_crank.step(args.rpm);
            sample_layer
                .as_mut()
                .ok_or("sample-only warmup requires a sample layer")?
                .process(SampleLayerInput {
                    rpm: args.rpm,
                    throttle: args.throttle,
                    load: args.load,
                    normalized_engine_torque: args.load,
                    clutch_engagement: 1.0,
                    crank_phase_deg: events.crank_phase_deg,
                })?;
        } else {
            let frame = engine.render_sample();
            scene.process(&frame);
        }
    }

    let total = (args.seconds * args.sample_rate as f32).round() as usize;
    let names = [
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
    let mut stems: BTreeMap<&str, Vec<f32>> = names
        .iter()
        .map(|&name| (name, Vec::with_capacity(total)))
        .collect();
    fs::create_dir_all(&args.stems_dir).map_err(|e| e.to_string())?;
    let telemetry_path = args.out.with_extension("events.csv");
    if let Some(parent) = telemetry_path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    if telemetry_path.exists() {
        let _ = fs::remove_file(&telemetry_path);
    }
    let mut telemetry = BufWriter::new(File::create(&telemetry_path).map_err(|e| e.to_string())?);
    writeln!(telemetry, "sample,time_s,rpm,crank_phase_deg,cylinder,bank")
        .map_err(|e| e.to_string())?;

    let mut phys_telemetry = args
        .physical_telemetry_csv
        .as_ref()
        .map(|path| {
            if let Some(parent) = path.parent() {
                let _ = fs::create_dir_all(parent);
            }
            if path.exists() {
                let _ = fs::remove_file(path);
            }
            let file = File::create(path).map_err(|e| e.to_string())?;
            let mut writer = BufWriter::new(file);
            writeln!(
                writer,
                "sample,time_s,rpm,crank_phase_deg,cylinder_id,chamber_pressure_pa,chamber_temperature_k,heat_release_rate,phase,valve_lift,valve_area_m2,mass_flow_kg_s,runner_pressure_pa,runner_temperature_k,runner_acoustic_pa,collector_pressure_a,collector_pressure_b"
            ).map_err(|e| e.to_string())?;
            Ok::<_, String>(writer)
        })
        .transpose()?;

    let mut peak = 0.0f32;
    let mut sum_sq = 0.0f64;
    // F2K evidence: per-source accumulated weights plus sampled-layer
    // component energy, all from this same render.
    let source_labels: Vec<String> = sample_layer
        .as_ref()
        .map(|layer| layer.source_labels().to_vec())
        .unwrap_or_default();
    let mut source_weight_sums = vec![0.0f64; source_labels.len()];
    let mut sample_tonal_sq = 0.0f64;
    let mut sample_residual_sq = 0.0f64;
    let mut sample_off_sq = 0.0f64;
    let mut worst_reduction = 0.0f32;
    let mut event_count = 0u64;
    let mut physical_master_lp = args.physical_master_lowpass_hz.map(|cutoff| {
        ThreePoleLowPass::new(
            cutoff.clamp(100.0, args.sample_rate as f32 * 0.48),
            args.sample_rate as f32,
        )
    });
    for sample in 0..total {
        let time_s = sample as f32 / args.sample_rate as f32;
        let current_input = render_input(&args, time_s);
        // Mechanical engagement is identical with and without a shift
        // sequence: only the phase differs, so A/B isolates the gesture.
        let retention_torque = if current_input.throttle > 0.30 {
            current_input.load
        } else {
            -((0.30 - current_input.throttle) / 0.30) * 0.6
        };
        let shift_phase = shift_phase_at(&args.shift_sequence, time_s);
        let transmission_signal = transmission.process(TransmissionInput {
            rpm: current_input.rpm,
            gear: args.gear,
            clutch: 1.0,
            torque: retention_torque,
            throttle: current_input.throttle,
            shift_phase,
        });
        let gear_shift_signal = gear_shift_player.process(shift_phase, current_input.rpm);
        let mut frame: EngineFrame = if sample_only {
            let events = sampled_crank.step(current_input.rpm);
            sampled_gesture.step(shift_phase as u8, args.sample_rate as f32, false);
            EngineFrame {
                throttle: current_input.throttle,
                load: current_input.load,
                crank_phase_deg: events.crank_phase_deg,
                ..EngineFrame::default()
            }
        } else {
            engine.set_input(current_input)?;
            engine.set_mechanical_state(retention_torque, 1.0, 0.0, false, shift_phase);
            engine.render_sample()
        };
        let sampled = sample_layer
            .as_mut()
            .map(|layer| {
                // Imposed evaluation trajectory (not physics braking): the
                // sweep prescribes RPM/throttle/load offline, so retention
                // signals are derived from the same prescribed controls.
                // Accel (throttle open) carries positive torque equal to the
                // prescribed load; coast (throttle closing) carries negative
                // torque proportional to the lift depth. Clutch stays engaged
                // through the lift-and-coast. No lift read-position jump.
                let retention_torque = if current_input.throttle > 0.30 {
                    current_input.load
                } else {
                    -((0.30 - current_input.throttle) / 0.30) * 0.6
                };
                layer.process(SampleLayerInput {
                    rpm: current_input.rpm,
                    throttle: current_input.throttle,
                    load: current_input.load,
                    normalized_engine_torque: retention_torque,
                    clutch_engagement: 1.0,
                    crank_phase_deg: frame.crank_phase_deg,
                })
            })
            .transpose()?;
        if sample_only {
            // The prepared bank is the engine: it drives the mix directly and
            // the gearbox is summed after it, never through the scene.
            let gesture = shift_energy_gain(shift_phase as u8) * sampled_gesture.gain();
            let sampled_engine = sampled.as_ref().map_or(0.0, |f| f.output)
                * gesture
                * HYBRID_HEADROOM_GAIN
                * args.sampled_engine_gain;
            frame.master = sampled_engine;
        }
        let acoustic = if sample_only {
            AcousticFrame::default()
        } else {
            scene.process(&frame)
        };
        stems
            .get_mut("combustion_source")
            .unwrap()
            .push(frame.combustion_source);
        stems
            .get_mut("pressure_derivative")
            .unwrap()
            .push(frame.pressure_derivative);
        stems
            .get_mut("pressure_derivative_a")
            .unwrap()
            .push(frame.pressure_derivative_a);
        stems
            .get_mut("pressure_derivative_b")
            .unwrap()
            .push(frame.pressure_derivative_b);
        stems
            .get_mut("pressure_direct")
            .unwrap()
            .push(frame.pressure_direct);
        stems.get_mut("crankcase").unwrap().push(frame.crankcase);
        stems.get_mut("block").unwrap().push(frame.block);
        stems.get_mut("head").unwrap().push(frame.head);
        stems.get_mut("block_head").unwrap().push(frame.block_head);
        stems.get_mut("headers_a").unwrap().push(frame.headers_a);
        stems.get_mut("headers_b").unwrap().push(frame.headers_b);
        stems
            .get_mut("collector_a")
            .unwrap()
            .push(frame.collector_a);
        stems
            .get_mut("collector_b")
            .unwrap()
            .push(frame.collector_b);
        stems
            .get_mut("collector_pressure_a")
            .unwrap()
            .push(frame.collector_pressure_a);
        stems
            .get_mut("collector_pressure_b")
            .unwrap()
            .push(frame.collector_pressure_b);
        stems.get_mut("exhaust").unwrap().push(frame.exhaust);
        stems.get_mut("turbulence").unwrap().push(frame.turbulence);
        stems
            .get_mut("master_pre_limiter")
            .unwrap()
            .push(frame.master_pre_limiter);
        stems.get_mut("master").unwrap().push(frame.master);
        stems
            .get_mut("engine_dry")
            .unwrap()
            .push(acoustic.engine_dry);
        stems
            .get_mut("engine_air")
            .unwrap()
            .push(acoustic.engine_air);
        stems
            .get_mut("engine_cover")
            .unwrap()
            .push(acoustic.engine_cover);
        stems
            .get_mut("engine_mounts")
            .unwrap()
            .push(acoustic.engine_mounts);
        stems
            .get_mut("monocoque_seat")
            .unwrap()
            .push(acoustic.monocoque_seat);
        stems
            .get_mut("mount_monocoque")
            .unwrap()
            .push(acoustic.mount_monocoque);
        stems
            .get_mut("cylinder_mechanical_sum")
            .unwrap()
            .push(acoustic.cylinder_mechanical_sum);
        const CYLINDER_STEMS: [&str; 10] = [
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
        for (index, name) in CYLINDER_STEMS.iter().enumerate() {
            stems
                .get_mut(name)
                .unwrap()
                .push(acoustic.cylinder_mechanical[index]);
        }
        stems.get_mut("scene_mix").unwrap().push(acoustic.output);
        stems
            .get_mut("transmission")
            .unwrap()
            .push(transmission_signal);
        stems
            .get_mut("gear_shift")
            .unwrap()
            .push(gear_shift_signal);
        stems
            .get_mut("gearbox")
            .unwrap()
            .push(transmission_signal + gear_shift_signal);
        let sample_tonal = sampled.as_ref().map_or(0.0, |frame| frame.tonal);
        let sample_residual = sampled.as_ref().map_or(0.0, |frame| frame.residual);
        let sample_max_rasp = sampled.as_ref().map_or(0.0, |frame| frame.max_rasp);
        let sample_off_throttle = sampled.as_ref().map_or(0.0, |frame| frame.off_throttle);
        let sample_output = sampled.as_ref().map_or(0.0, |frame| frame.output);
        if let Some(frame) = sampled {
            for (slot, weight) in source_weight_sums.iter_mut().zip(frame.zone_weights.iter()) {
                *slot += *weight as f64;
            }
            sample_tonal_sq += (frame.tonal * frame.tonal) as f64;
            sample_residual_sq += (frame.residual * frame.residual) as f64;
            sample_off_sq += (frame.off_throttle * frame.off_throttle) as f64;
        }
        let gearbox_signal = transmission_signal + gear_shift_signal;
        let (v10_bus, gearbox_level) = if sample_only {
            (frame.master, args.sampled_gearbox_gain)
        } else {
            // Hybrid headroom is static and transparent. A limiter here would
            // hide gain errors and make the sample layer part of the sound
            // design. The blend mirrors Gf509Runtime: the sample layer follows
            // the same synthesized shift gesture as the procedural side.
            let blended = blend_hybrid(
                acoustic.output,
                sample_layer.is_some().then_some(sample_output),
                physical_blend_weight,
                sample_blend_weight,
                engine.shift_gesture_gain(),
            );
            (blended, 1.0)
        };
        let v10_filtered = v10_chain.process(v10_bus);
        let gearbox_filtered = gearbox_chain.process(gearbox_signal) * gearbox_level;
        let hybrid = upper_mid_shelf.process(v10_filtered + gearbox_filtered);
        stems
            .get_mut("sampled_engine")
            .unwrap()
            .push(if sample_only { frame.master } else { 0.0 });
        stems.get_mut("sample_tonal").unwrap().push(sample_tonal);
        stems
            .get_mut("sample_residual")
            .unwrap()
            .push(sample_residual);
        stems
            .get_mut("sample_max_rasp")
            .unwrap()
            .push(sample_max_rasp);
        stems
            .get_mut("sample_off_throttle")
            .unwrap()
            .push(sample_off_throttle);
        stems.get_mut("sample_layer").unwrap().push(sample_output);
        stems.get_mut("v10_filtered").unwrap().push(v10_filtered);
        stems
            .get_mut("gearbox_filtered")
            .unwrap()
            .push(gearbox_filtered);
        stems.get_mut("hybrid_mix").unwrap().push(hybrid);
        stems
            .get_mut("cylinder_pressure")
            .unwrap()
            .push(frame.cylinder_chamber_pressure_pa.iter().sum::<f32>() / 10.0);
        stems
            .get_mut("chamber_temperature")
            .unwrap()
            .push(frame.cylinder_chamber_temperature_k.iter().sum::<f32>() / 10.0);
        stems
            .get_mut("chamber_heat_release")
            .unwrap()
            .push(frame.cylinder_chamber_heat_release_rate.iter().sum::<f32>() / 10.0);
        stems
            .get_mut("chamber_phase")
            .unwrap()
            .push(frame.cylinder_chamber_phase[0] as f32);
        stems
            .get_mut("combustion_heat_release")
            .unwrap()
            .push(frame.pressure_derivative);
        stems
            .get_mut("valve_area")
            .unwrap()
            .push(frame.cylinder_valve_area.iter().sum::<f32>());
        stems
            .get_mut("mass_flow")
            .unwrap()
            .push(frame.cylinder_mass_flow.iter().sum::<f32>());
        stems
            .get_mut("runner_pressure")
            .unwrap()
            .push(frame.cylinder_runner_pressure.iter().sum::<f32>() * 0.1);
        stems
            .get_mut("runner_acoustic_pressure")
            .unwrap()
            .push(frame.cylinder_runner_acoustic_pressure.iter().sum::<f32>());
        let filtered_physical_master = if let Some(lp) = physical_master_lp.as_mut() {
            lp.process(frame.master)
        } else {
            frame.master
        };
        stems
            .get_mut("physical_master")
            .unwrap()
            .push(filtered_physical_master);
        stems.get_mut("sample_master").unwrap().push(sample_output);
        stems.get_mut("hybrid_master").unwrap().push(hybrid);

        if let Some(writer) = phys_telemetry.as_mut() {
            writeln!(
                writer,
                "{sample},{time_s:.6},{rpm:.2},{crank_phase_deg:.2},{cyl},{chamber_p:.4e},{chamber_t:.2},{heat:.6e},{phase},{lift:.6},{area:.6e},{flow:.6e},{runner_p:.4e},{runner_t:.2},{acoustic_p:.6e},{col_a:.4e},{col_b:.4e}",
                sample = sample,
                time_s = time_s,
                rpm = current_input.rpm,
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
            ).map_err(|e| e.to_string())?;
        }

        let rendered = if sample_layer.is_some() {
            hybrid
        } else if args.acoustic_scene {
            acoustic.output
        } else {
            filtered_physical_master
        };
        peak = peak.max(rendered.abs());
        sum_sq += (rendered * rendered) as f64;
        worst_reduction = worst_reduction.min(frame.limiter_reduction_db);
        if frame.fired_mask != 0 {
            for cylinder in 0..10 {
                if frame.fired_mask & (1 << cylinder) != 0 {
                    event_count += 1;
                    let bank = if cylinder < 5 { "A" } else { "B" };
                    writeln!(
                        telemetry,
                        "{sample},{:.9},{:.3},{:.6},{cylinder},{bank}",
                        sample as f64 / args.sample_rate as f64,
                        current_input.rpm,
                        frame.crank_phase_deg,
                    )
                    .map_err(|e| e.to_string())?;
                }
            }
        }
    }
    telemetry.flush().map_err(|e| e.to_string())?;

    let output_stem = if sample_layer.is_some() {
        "hybrid_mix"
    } else if args.acoustic_scene {
        "scene_mix"
    } else {
        "master"
    };
    write_mono_pcm16(&args.out, args.sample_rate, stems.get(output_stem).unwrap())?;
    for (name, samples) in &stems {
        write_mono_pcm16(
            &args.stems_dir.join(format!("{name}.wav")),
            args.sample_rate,
            samples,
        )?;
    }

    let rms = (sum_sq / total as f64).sqrt();
    let sample_layer_bank = args
        .sample_layer_dir
        .as_ref()
        .and_then(|path| path.file_name())
        .and_then(|name| name.to_str())
        .unwrap_or("none")
        .to_string();
    let sample_layer_source_weights = source_labels
        .iter()
        .zip(source_weight_sums.iter())
        .map(|(label, sum)| format!("{label}={:.6}", sum / total as f64))
        .collect::<Vec<_>>()
        .join(",");
    let sample_tonal_rms = (sample_tonal_sq / total as f64).sqrt();
    let sample_residual_rms = (sample_residual_sq / total as f64).sqrt();
    let sample_off_rms = (sample_off_sq / total as f64).sqrt();
    let metadata_path = args.out.with_extension("metadata.json");
    let profile = if args.hold_before_lift {
        "lift_coast_hold"
    } else if args.sweep_end_rpm.is_some() {
        "sweep_lift_coast"
    } else {
        "steady"
    };
    let peak_rpm = if args.hold_before_lift {
        args.rpm.max(args.sweep_end_rpm.unwrap_or(args.rpm))
    } else {
        args.sweep_end_rpm.unwrap_or(args.rpm)
    };
    let metadata_coast_end_rpm = if args.hold_before_lift {
        args.sweep_end_rpm.unwrap_or(args.coast_end_rpm)
    } else {
        args.coast_end_rpm
    };
    let metadata = format!(
        concat!(
            "{{\n",
            "  \"architecture\": \"rust-greenfield-v10\",\n",
            "  \"git_head\": \"{}\",\n",
            "  \"source_dirty\": {},\n",
            "  \"source_fingerprint\": \"{}\",\n",
            "  \"sample_rate\": {},\n",
            "  \"seed\": {},\n",
            "  \"header_length_scale\": {:.6},\n",
            "  \"exhaust_gain\": \"{}\",\n",
            "  \"collector_geometry\": \"{}\",\n",
            "  \"profile\": \"{}\",\n",
            "  \"rpm\": {:.3},\n",
            "  \"peak_rpm\": {:.3},\n",
            "  \"coast_end_rpm\": {:.3},\n",
            "  \"hold_before_lift\": {},\n",
            "  \"accel_seconds\": {:.6},\n",
            "  \"throttle\": {:.6},\n",
            "  \"load\": {:.6},\n",
            "  \"duration_s\": {:.6},\n",
            "  \"warmup_s\": {:.6},\n",
            "  \"shift_sequence\": \"{}\",\n",
            "  \"event_count\": {},\n",
            "  \"peak\": {:.9},\n",
            "  \"rms\": {:.9},\n",
            "  \"worst_limiter_reduction_db\": {:.6},\n",
            "  \"legacy_synth_used\": false,\n",
            "  \"cpp_used\": false,\n",
            "  \"faust_used\": false,\n",
            "  \"acoustic_scene_used\": {},\n",
            "  \"sample_layer_used\": {},\n",
            "  \"sample_rasp_disabled\": {},\n",
            "  \"sample_residual_scale\": {},\n",
            "  \"sample_residual_gain\": \"{}\",\n",
            "  \"sample_layer_tuning\": \"{}\",\n",
            "  \"scene_gain_overrides\": \"{}\",\n",
            "  \"sample_layer_bank\": \"{}\",\n",
            "  \"engine_mode\": \"{}\",\n",
            "  \"sampled_engine_gain\": {:.6},\n",
            "  \"sampled_gearbox_gain\": {:.6},\n",
            "  \"sample_layer_source_weights\": \"{}\",\n",
            "  \"sample_tonal_rms\": {:.9},\n",
            "  \"sample_residual_rms\": {:.9},\n",
            "  \"sample_off_rms\": {:.9},\n",
            "  \"hybrid_headroom_gain\": {:.6},\n",
            "  \"upper_mid_shelf_gain\": {:.6},\n",
            "  \"transmission_gain\": {:.6},\n",
            "  \"transmission_gear\": {},\n",
            "  \"transmission_gear_ratios\": \"{}\",\n",
            "  \"transmission_final_drive\": {:.6},\n",
            "  \"gear_shift_gain\": {:.6},\n",
            "  \"gear_shift_reference_rpm\": {:.6},\n",
            "  \"chamber_cycle_model\": \"bounded_720deg_four_stroke_fresh_charge\",\n",
            "  \"chamber_phase_contract\": \"expansion_0_180,exhaust_180_360,intake_360_540,compression_540_720\",\n",
            "  \"firing_order\": [0,5,1,6,2,7,3,8,4,9],\n",
            "  \"sample_mid_architecture\": \"zone-specific post gains + deeper max residual compression/saturation + max rasp at +2.5 dB + reduced engine/scene drift + perceptual max crossfade 10000-13750 RPM\"\n",
            "}}\n"
        ),
        git_head(),
        source_dirty(),
        source_fingerprint(),
        args.sample_rate,
        args.seed,
        args.header_length_scale,
        exhaust_gain_label(&args),
        collector_geometry_label(&args),
        profile,
        args.rpm,
        peak_rpm,
        metadata_coast_end_rpm,
        args.hold_before_lift,
        args.accel_seconds,
        args.throttle,
        args.load,
        args.seconds,
        args.warmup,
        shift_sequence_label(&args.shift_sequence),
        event_count,
        peak,
        rms,
        worst_reduction,
        args.acoustic_scene,
        sample_layer.is_some(),
        args.no_sample_rasp,
        args.sample_residual_scale,
        args.sample_residual_gain
            .map(|gain| format!("{gain}"))
            .unwrap_or_else(|| "load-blended".to_string()),
        sample_layer_tuning,
        scene_gain_overrides,
        sample_layer_bank,
        if sample_only { "sample_only" } else { "hybrid" },
        args.sampled_engine_gain,
        args.sampled_gearbox_gain,
        sample_layer_source_weights,
        sample_tonal_rms,
        sample_residual_rms,
        sample_off_rms,
        args.upper_mid_shelf_gain,
        args.transmission_gain,
        args.gear,
        args.gear_ratios
            .iter()
            .map(|value| format!("{value}"))
            .collect::<Vec<_>>()
            .join(","),
args.final_drive,
        args.gear_shift_gain,
        args.gear_shift_reference_rpm,
        HYBRID_HEADROOM_GAIN,
    );
    if metadata_path.exists() {
        let _ = fs::remove_file(&metadata_path);
    }
    fs::write(&metadata_path, metadata).map_err(|e| e.to_string())?;
    println!("wav={}", args.out.display());
    println!("stems={}", args.stems_dir.display());
    println!("events={event_count} peak={peak:.6} rms={rms:.6} limiter_db={worst_reduction:.3}");
    Ok(())
}

#[allow(dead_code)]
fn _assert_path(_: &Path) {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_shift_sequence_events() {
        let events = parse_shift_sequence("u@2.5, d@7.2").unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[0],
            ShiftEvent {
                time_s: 2.5,
                direction: ShiftDirection::Up
            }
        );
        assert_eq!(
            events[1],
            ShiftEvent {
                time_s: 7.2,
                direction: ShiftDirection::Down
            }
        );
    }

    #[test]
    fn rejects_malformed_shift_sequence() {
        assert!(parse_shift_sequence("u2.5").is_err());
        assert!(parse_shift_sequence("x@1.0").is_err());
        assert!(parse_shift_sequence("u@nope").is_err());
        assert!(parse_shift_sequence("u@-1.0").is_err());
        assert!(parse_shift_sequence("u@3.0,u@2.0").is_err());
        assert!(parse_shift_sequence("u@2.0,d@2.0").is_err());
    }

    #[test]
    fn shift_events_must_fit_inside_the_render_duration() {
        let events = parse_shift_sequence("u@2.5").unwrap();
        assert!(validate_shift_sequence(&events, 6.0).is_ok());
        assert!(validate_shift_sequence(&events, 2.5).is_err());
        assert!(validate_shift_sequence(&[], 2.5).is_ok());
    }

    #[test]
    fn phase_schedule_matches_cut_blip_recovery() {
        let events = parse_shift_sequence("u@1.0,d@5.0").unwrap();
        assert_eq!(shift_phase_at(&events, 0.5), ShiftPhase::None);
        assert_eq!(shift_phase_at(&events, 1.0), ShiftPhase::UpshiftCut);
        assert_eq!(shift_phase_at(&events, 1.24), ShiftPhase::UpshiftCut);
        assert_eq!(shift_phase_at(&events, 1.26), ShiftPhase::UpshiftRecovery);
        assert_eq!(shift_phase_at(&events, 1.30), ShiftPhase::None);
        assert_eq!(shift_phase_at(&events, 5.1), ShiftPhase::DownshiftCut);
        assert_eq!(shift_phase_at(&events, 5.24), ShiftPhase::DownshiftCut);
        assert_eq!(shift_phase_at(&events, 5.26), ShiftPhase::DownshiftBlip);
        assert_eq!(shift_phase_at(&events, 5.39), ShiftPhase::DownshiftRecovery);
        assert_eq!(shift_phase_at(&events, 5.50), ShiftPhase::None);
    }

    #[test]
    fn shift_sequence_label_round_trips() {
        assert_eq!(shift_sequence_label(&[]), "none");
        let events = parse_shift_sequence("u@2.5,d@7.25").unwrap();
        assert_eq!(shift_sequence_label(&events), "u@2.500,d@7.250");
    }
}
