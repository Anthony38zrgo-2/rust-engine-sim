//! Offline renderer: JSON engine → simulation → DSP → WAV.

use es_audio_io::{read_impulse_response_i16, write_wav_i16_mono};
use es_config::{load_path, EngineFile};
use es_dsp::scene::SceneConfig;
use es_dsp::{
    render_offline_to, render_pulse_channels, render_scene, AudioParameters, Biquad, PulseChannel,
    SceneChannel, SceneRenderParams, SceneStems,
};
use es_sim::{Engine, Event, ScenarioEvent};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!(
            "usage: render <engine.json> [--out <file.wav>] [--duration <s>] [--assets <dir>]"
        );
        return ExitCode::from(2);
    }

    let engine_path = &args[1];
    let mut out_override: Option<PathBuf> = None;
    let mut duration_override: Option<f64> = None;
    let mut acoustic_override: Option<f64> = None;
    let mut stems_dir: Option<PathBuf> = None;
    let mut assets = PathBuf::from("assets");

    let mut i = 2;
    while i < args.len() {
        match args[i].as_str() {
            "--out" => {
                i += 1;
                out_override = args.get(i).map(PathBuf::from);
            }
            "--duration" => {
                i += 1;
                duration_override = args.get(i).and_then(|s| s.parse().ok());
            }
            "--acoustic-rate" => {
                i += 1;
                acoustic_override = args.get(i).and_then(|s| s.parse().ok());
            }
            "--stems" => {
                i += 1;
                stems_dir = args.get(i).map(PathBuf::from);
            }
            "--assets" => {
                i += 1;
                if let Some(a) = args.get(i) {
                    assets = PathBuf::from(a);
                }
            }
            other => {
                eprintln!("unknown flag: {other}");
                return ExitCode::from(2);
            }
        }
        i += 1;
    }

    match run(
        engine_path,
        out_override.as_deref(),
        duration_override,
        acoustic_override,
        stems_dir.as_deref(),
        &assets,
    ) {
        Ok(msg) => {
            println!("{msg}");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(
    engine_path: &str,
    out_override: Option<&Path>,
    duration_override: Option<f64>,
    acoustic_override: Option<f64>,
    stems_dir: Option<&Path>,
    assets: &Path,
) -> Result<String, String> {
    let cfg: EngineFile = load_path(engine_path)?;
    let build = cfg.to_build();
    let name = build.meta.name.clone();
    let sim_fs = build.meta.simulation_frequency;

    let mut engine = Engine::build(build);

    let duration = duration_override.unwrap_or(cfg.scenario.duration_s);
    if duration <= 0.0 {
        return Err("duration must be > 0".into());
    }

    // Build scenario events from JSON or default spin-up
    let events: Vec<ScenarioEvent> = if cfg.scenario.events.is_empty() {
        default_scenario(sim_fs, engine.meta.redline)
    } else {
        let mut evs: Vec<ScenarioEvent> = cfg
            .scenario
            .events
            .iter()
            .map(|e| {
                let event = match e.action.as_str() {
                    "throttle" => Event::SetThrottle(e.value),
                    "starter" => Event::SetStarter(e.value != 0.0),
                    "dyno" => Event::SetDyno(e.value != 0.0),
                    "dyno_speed" => Event::SetDynoSpeed(es_units::rpm(e.value)),
                    "dyno_hold" => Event::SetDynoHold(e.value != 0.0),
                    "ignition" => Event::SetIgnition(e.value != 0.0),
                    _ => Event::SetThrottle(0.0),
                };
                ScenarioEvent {
                    time: e.time_s,
                    event,
                }
            })
            .collect();
        evs.sort_by(|a, b| a.time.total_cmp(&b.time));
        evs
    };

    eprintln!(
        "[{name}] simulating {duration:.2}s @ {sim_fs} Hz, {} channels...",
        engine.exhausts.len().max(1)
    );

    let out = engine.run_offline(duration, &events);

    let audio_rate = cfg.audio.sample_rate;
    let acoustic_rate = acoustic_override
        .or(cfg.audio.acoustic_sample_rate)
        .unwrap_or_else(|| audio_rate.max(48_000.0));
    let params = AudioParameters {
        volume: cfg.audio.volume,
        convolution: cfg.audio.convolution,
        hf_mix: cfg.audio.hf_mix.unwrap_or(cfg.audio.d_f_f_mix),
        hf_reference_hz: cfg.audio.hf_reference_hz.unwrap_or(1_000.0),
        input_sample_noise: cfg.audio.input_sample_noise,
        input_sample_noise_frequency_cutoff: cfg.audio.input_sample_noise_frequency_cutoff,
        flow_noise: cfg.audio.flow_noise.unwrap_or(cfg.audio.air_noise),
        flow_noise_frequency_cutoff: cfg
            .audio
            .flow_noise_frequency_cutoff
            .unwrap_or(cfg.audio.air_noise_frequency_cutoff),
        mechanical_noise: cfg.audio.mechanical_noise.unwrap_or(0.0),
        mechanical_noise_frequency_cutoff: cfg
            .audio
            .mechanical_noise_frequency_cutoff
            .unwrap_or(8_000.0),
        input_antialias_frequency_cutoff: cfg.audio.input_antialias_frequency_cutoff,
        leveler_enabled: cfg.audio.leveler_enabled.unwrap_or(true),
        leveler_target: cfg.audio.leveler_target.unwrap_or(30_000.0),
        leveler_max_gain: cfg.audio.leveler_max_gain.unwrap_or(1.9),
        leveler_min_gain: 1e-5,
        leveler_attack: cfg.audio.leveler_attack.unwrap_or(0.001),
        leveler_release: cfg.audio.leveler_release.unwrap_or(0.05),
    };
    let normalize_peak = cfg
        .audio
        .normalize_peak_dbfs
        .map(|db| 10f64.powf(db as f64 / 20.0) as f32 * i16::MAX as f32);

    // Load IRs per exhaust system (with per-IR amplitude scale)
    let mut irs: Vec<Option<(u32, Vec<i16>, f32)>> = vec![None; out.audio_channels.len()];
    for (i, ex) in cfg.exhausts.iter().enumerate() {
        if i >= irs.len() {
            break;
        }
        if let Some(rel) = &ex.impulse_response {
            let path = resolve_ir(assets, rel);
            match read_impulse_response_i16(&path) {
                Ok((sr, samples)) => {
                    let resampled = resample_i16(&samples, sr, acoustic_rate as u32);
                    eprintln!(
                        "  IR ch{i}: {} ({} samples @ {} Hz -> {} samples @ {} Hz, vol {})",
                        path.display(),
                        samples.len(),
                        sr,
                        resampled.len(),
                        acoustic_rate as u32,
                        ex.impulse_response_volume
                    );
                    irs[i] = Some((
                        acoustic_rate as u32,
                        resampled,
                        ex.impulse_response_volume as f32,
                    ));
                }
                Err(e) => eprintln!("  warn: IR load failed {path:?}: {e}"),
            }
        }
    }

    let bank_gain = cfg.audio.bank_gain.unwrap_or([1.0, 1.0]);
    let pulse_channels: Vec<PulseChannel<'_>> = out
        .cylinder_audio
        .iter()
        .enumerate()
        .map(|(i, src)| PulseChannel {
            base: &src.base,
            dyn_p: &src.dyn_p,
            att3: &src.att3,
            intake: &src.intake,
            delay_s: out.cylinder_delay_s[i],
            gain: out.cylinder_audio_gain[i] * bank_gain[out.cylinder_bank[i].min(1)],
            intake_gain: cfg.audio.intake_gain.unwrap_or(0.0),
            exhaust: out.cylinder_exhaust[i],
        })
        .collect();

    eprintln!(
        "DSP: {} sim samples @ {sim_fs} Hz → acoustic {acoustic_rate} Hz → {audio_rate} Hz...",
        out.cylinder_audio
            .first()
            .map(|c| c.base.len())
            .unwrap_or(0)
    );

    let mut channels = render_pulse_channels(
        &pulse_channels,
        sim_fs,
        acoustic_rate,
        out.audio_channels.len(),
    );
    if !cfg.audio.resonances.is_empty() {
        for ch in channels.iter_mut() {
            let mut filters: Vec<Biquad> = cfg
                .audio
                .resonances
                .iter()
                .map(|r| {
                    Biquad::peaking(
                        r.frequency_hz as f32,
                        r.q as f32,
                        r.gain_db as f32,
                        acoustic_rate as f32,
                    )
                })
                .collect();
            for x in ch.iter_mut() {
                let mut v = *x as f32;
                for f in filters.iter_mut() {
                    v = f.process(v);
                }
                *x = v as f64;
            }
        }
    }
    let n_samples = out
        .cylinder_audio
        .first()
        .map(|c| c.base.len())
        .unwrap_or(0);
    let n_exhausts = out.audio_channels.len().max(1);
    let mut envelopes: Vec<Vec<f64>> = vec![vec![0.0; n_samples]; n_exhausts];
    for (i, src) in out.cylinder_audio.iter().enumerate() {
        let ex = out.cylinder_exhaust[i].min(n_exhausts - 1);
        let gain = out.cylinder_audio_gain[i];
        for k in 0..n_samples {
            let p = gain * src.att3[k] * (src.base[k] + 0.1 * src.dyn_p[k]);
            envelopes[ex][k] += p.abs();
        }
    }
    let smoothing = 1.0 - (-2.0 * std::f64::consts::PI * 80.0 / sim_fs).exp();
    let mut global_max = 0.0f64;
    for env in &mut envelopes {
        let mut y = 0.0;
        for v in env.iter_mut() {
            y += (*v - y) * smoothing;
            *v = y;
            if y > global_max {
                global_max = y;
            }
        }
    }
    if global_max > 0.0 {
        for env in &mut envelopes {
            for v in env.iter_mut() {
                *v /= global_max;
            }
        }
    }
    let scene_cfg = scene_config(&cfg.audio.scene);
    let mut scene_stems = SceneStems::default();
    let mut mix_channels: Vec<Vec<f64>> = channels.clone();
    let mut mix_irs: Vec<Option<(u32, Vec<i16>, f32)>> = irs.clone();
    if scene_cfg.enabled {
        let master: Vec<f64> = if let Some(first) = channels.first() {
            (0..first.len())
                .map(|i| channels.iter().map(|c| c[i]).sum())
                .collect()
        } else {
            Vec::new()
        };
        let turbulence: Vec<f64> = if n_samples > 0 && !envelopes.is_empty() {
            (0..n_samples)
                .map(|k| envelopes.iter().map(|e| e[k]).sum::<f64>() / envelopes.len() as f64)
                .collect()
        } else {
            Vec::new()
        };
        let waveguide = cfg.audio.header_waveguide.as_ref();
        let waveguide_enabled = waveguide
            .and_then(|w| w.enabled)
            .unwrap_or(waveguide.is_some());
        let scene_channels: Vec<SceneChannel<'_>> = out
            .cylinder_audio
            .iter()
            .enumerate()
            .map(|(i, src)| SceneChannel {
                base: &src.base,
                derivative: &src.derivative,
                blowdown: &src.blowdown,
                flow: &src.flow,
                runner_temp: &src.runner_temp,
                delay_s: out.cylinder_delay_s[i],
                bank: out.cylinder_bank[i],
                signature: 1.0,
                header_length_m: out.cylinder_primary_length[i],
            })
            .collect();
        let scene_params = SceneRenderParams {
            config: scene_cfg,
            bank_gain: [bank_gain[0] as f32, bank_gain[1] as f32],
            collector_pressure: &out.collector_pressure,
            master: &master,
            rpm: &out.rpm,
            turbulence: &turbulence,
            flow_excitation_gain: waveguide
                .and_then(|w| w.excitation_gain)
                .unwrap_or(if waveguide_enabled { 1.0 } else { 0.0 }),
            header_reflection: waveguide.and_then(|w| w.reflection).unwrap_or(-0.34) as f32,
            header_temperature_dependent: waveguide
                .and_then(|w| w.temperature_dependent)
                .unwrap_or(true),
            header_gain: waveguide.and_then(|w| w.header_gain).unwrap_or(0.0),
            load: scene_cfg.load,
            throttle: scene_cfg.throttle,
        };
        scene_stems = render_scene(&scene_channels, sim_fs, acoustic_rate, &scene_params);
        let stem_peak = |v: &[f32]| v.iter().fold(0.0f32, |a, b| a.max(b.abs()));
        eprintln!(
            "Scene: enabled (air={:.2} cover={:.2} mount={:.2} cyl={:.2} wg={}) peaks air={:.2e} cover={:.2e} mount={:.2e} cyl={:.2e} header={:.2e} out={:.2e}",
            scene_cfg.engine_air_gain,
            scene_cfg.engine_cover_gain,
            scene_cfg.mount_monocoque_gain,
            scene_cfg.cylinder_gain,
            if waveguide_enabled { "on" } else { "off" },
            stem_peak(&scene_stems.air),
            stem_peak(&scene_stems.cover),
            stem_peak(&scene_stems.mount),
            stem_peak(&scene_stems.cylinder),
            stem_peak(&scene_stems.header),
            stem_peak(&scene_stems.output),
        );
        if scene_stems.output.iter().any(|v| *v != 0.0) {
            mix_channels.push(scene_stems.output.iter().map(|&v| v as f64).collect());
            mix_irs.push(None);
        }
        if scene_params.header_gain != 0.0 && scene_stems.header.iter().any(|v| *v != 0.0) {
            mix_channels.push(scene_stems.header.iter().map(|&v| v as f64).collect());
            mix_irs.push(None);
        }
    }
    let samples = render_offline_to(
        &mix_channels,
        acoustic_rate,
        acoustic_rate,
        audio_rate,
        &params,
        &mix_irs,
        &envelopes,
        normalize_peak,
    );

    let out_path: PathBuf = match out_override {
        Some(p) => p.to_path_buf(),
        None => {
            let mut p = PathBuf::from(&cfg.audio.output);
            if p.is_relative() {
                p = PathBuf::from("output").join(p);
            }
            p
        }
    };
    if let Some(parent) = out_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }

    let peak = samples.iter().map(|&s| (s as i32).abs()).max().unwrap_or(0);
    let nonzero = samples.iter().filter(|&&s| s != 0).count();
    let clipped = samples
        .iter()
        .filter(|&&s| s == i16::MIN || s == i16::MAX)
        .count();
    let first_clip = samples
        .iter()
        .position(|&s| s == i16::MIN || s == i16::MAX)
        .unwrap_or(usize::MAX);
    let sum2: f64 = samples.iter().map(|&s| (s as f64) * (s as f64)).sum();
    let rms = (sum2 / samples.len().max(1) as f64).sqrt();
    let crest = if rms > 0.0 { peak as f64 / rms } else { 0.0 };
    let plateau_level = (params.leveler_target as f64 * params.volume as f64).abs();
    let limited = if params.leveler_enabled {
        samples
            .iter()
            .filter(|&&s| (s as f64).abs() >= plateau_level * 0.99)
            .count()
    } else {
        0
    };
    let mut max_flat_run = 0usize;
    let mut flat_run = 0usize;
    let mut prev: Option<i16> = None;
    for &s in &samples {
        match prev {
            Some(p) if p == s => {
                flat_run += 1;
                if flat_run > max_flat_run {
                    max_flat_run = flat_run;
                }
            }
            _ => flat_run = 1,
        }
        prev = Some(s);
    }

    write_wav_i16_mono(&out_path, audio_rate as u32, &samples)?;

    let mut stems_written = 0usize;
    if let Some(dir) = stems_dir {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        for (i, ch) in channels.iter().enumerate() {
            let single = [ch.clone()];
            let stem = render_offline_to(
                &single,
                acoustic_rate,
                acoustic_rate,
                audio_rate,
                &params,
                &[None],
                &[],
                normalize_peak,
            );
            write_wav_i16_mono(
                dir.join(format!("exhaust_{i}.wav")),
                audio_rate as u32,
                &stem,
            )?;
            stems_written += 1;
        }
        write_wav_i16_mono(dir.join("mix.wav"), audio_rate as u32, &samples)?;
        stems_written += 1;
        if scene_cfg.enabled {
            let scene_stem_list: [(&str, &Vec<f32>); 6] = [
                ("scene_air", &scene_stems.air),
                ("scene_cover", &scene_stems.cover),
                ("scene_mount", &scene_stems.mount),
                ("scene_cylinder", &scene_stems.cylinder),
                ("scene_header", &scene_stems.header),
                ("scene_mix", &scene_stems.output),
            ];
            for (stem_name, series) in scene_stem_list {
                if series.is_empty() {
                    continue;
                }
                let single: Vec<f64> = series.iter().map(|&v| v as f64).collect();
                let stem = render_offline_to(
                    &[single],
                    acoustic_rate,
                    acoustic_rate,
                    audio_rate,
                    &params,
                    &[None],
                    &[],
                    normalize_peak,
                );
                write_wav_i16_mono(
                    dir.join(format!("{stem_name}.wav")),
                    audio_rate as u32,
                    &stem,
                )?;
                stems_written += 1;
            }
        }
    }

    let rpm_first = out.rpm.first().copied().unwrap_or(0.0);
    let rpm_last = out.rpm.last().copied().unwrap_or(0.0);
    let rpm_peak = out.rpm.iter().cloned().fold(0.0, f64::max);

    Ok(format!(
        "wrote {} ({} samples @ {} Hz, acoustic {acoustic_rate} Hz, peak={peak}, rms={rms:.0}, crest={crest:.2}, plateau={:.2}% clipped={clipped} first_clip={first_clip}, flat_run={max_flat_run}, stems={stems_written}, nonzero={nonzero}, rpm {rpm_first:.0}→{rpm_last:.0}, peak_rpm {rpm_peak:.0})",
        out_path.display(),
        samples.len(),
        audio_rate as u32,
        100.0 * limited as f64 / samples.len().max(1) as f64
    ))
}

fn scene_config(file: &Option<es_config::SceneFile>) -> SceneConfig {
    let d = SceneConfig::default();
    let Some(f) = file else {
        return d;
    };
    SceneConfig {
        enabled: f.enabled.unwrap_or(true),
        engine_air_gain: f.engine_air_gain.unwrap_or(d.engine_air_gain),
        engine_cover_gain: f.engine_cover_gain.unwrap_or(d.engine_cover_gain),
        mount_monocoque_gain: f.mount_monocoque_gain.unwrap_or(d.mount_monocoque_gain),
        cylinder_gain: f.cylinder_gain.unwrap_or(d.cylinder_gain),
        dry_low_gain: f.dry_low_gain.unwrap_or(d.dry_low_gain),
        dry_mid_gain: f.dry_mid_gain.unwrap_or(d.dry_mid_gain),
        dry_high_gain: f.dry_high_gain.unwrap_or(d.dry_high_gain),
        output_gain: f.output_gain.unwrap_or(d.output_gain),
        air_high_tilt_db: f.air_high_tilt_db.unwrap_or(d.air_high_tilt_db),
        air_direct_gain: f.air_direct_gain.unwrap_or(d.air_direct_gain),
        cover_radiation_lowpass_hz: f
            .cover_radiation_lowpass_hz
            .unwrap_or(d.cover_radiation_lowpass_hz),
        high_rpm_start: f.high_rpm_start.unwrap_or(d.high_rpm_start),
        high_rpm_span: f.high_rpm_span.unwrap_or(d.high_rpm_span),
        load: f.load.unwrap_or(d.load),
        throttle: f.throttle.unwrap_or(d.throttle),
        excitation_scale: d.excitation_scale,
    }
}

fn resample_i16(samples: &[i16], from: u32, to: u32) -> Vec<i16> {
    if from == to || samples.is_empty() {
        return samples.to_vec();
    }
    let ratio = to as f64 / from as f64;
    let out_len = (samples.len() as f64 * ratio).round() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let x = i as f64 / ratio;
        let i0 = x.floor() as usize;
        let frac = x - i0 as f64;
        let s0 = samples[i0.min(samples.len() - 1)] as f64;
        let s1 = samples[(i0 + 1).min(samples.len() - 1)] as f64;
        out.push((s0 * (1.0 - frac) + s1 * frac).round() as i16);
    }
    out
}

fn resolve_ir(assets: &Path, rel: &str) -> PathBuf {
    let p = Path::new(rel);
    if p.is_absolute() {
        return p.to_path_buf();
    }
    // Try assets-relative, then bare name under sound-library
    let candidates = [
        assets.join(rel),
        assets.join("sound-library").join(rel),
        assets.join("sound-library").join("smooth").join(rel),
        PathBuf::from(rel),
    ];
    for c in candidates {
        if c.exists() {
            return c;
        }
    }
    assets.join(rel)
}

/// Default: start, run starter briefly, release, full throttle rev to redline-ish.
fn default_scenario(_fs: f64, redline: f64) -> Vec<ScenarioEvent> {
    let mut evs = vec![
        ScenarioEvent {
            time: 0.0,
            event: Event::SetStarter(true),
        },
        ScenarioEvent {
            time: 0.0,
            event: Event::SetIgnition(true),
        },
        ScenarioEvent {
            time: 0.0,
            event: Event::SetThrottle(0.12),
        },
        ScenarioEvent {
            time: 0.4,
            event: Event::SetStarter(false),
        },
        ScenarioEvent {
            time: 0.5,
            event: Event::SetThrottle(1.0),
        },
        ScenarioEvent {
            time: 2.5,
            event: Event::SetThrottle(0.0),
        },
        ScenarioEvent {
            time: 3.0,
            event: Event::SetThrottle(0.0),
        },
    ];
    let _ = redline;
    evs.sort_by(|a, b| a.time.total_cmp(&b.time));
    evs
}
