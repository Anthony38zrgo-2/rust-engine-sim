//! Offline renderer: JSON engine → simulation → DSP → WAV.

use es_audio_io::{read_impulse_response_i16, write_wav_i16_mono};
use es_config::{load_path, EngineFile};
use es_dsp::{render_offline, AudioParameters};
use es_sim::{Engine, Event, ScenarioEvent};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: render <engine.json> [--out <file.wav>] [--duration <s>] [--assets <dir>]");
        return ExitCode::from(2);
    }

    let engine_path = &args[1];
    let mut out_override: Option<PathBuf> = None;
    let mut duration_override: Option<f64> = None;
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

    match run(engine_path, out_override.as_deref(), duration_override, &assets) {
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
    assets: &Path,
) -> Result<String, String> {
    let cfg: EngineFile = load_path(engine_path)?;
    let build = cfg.to_build();
    let name = build.meta.name.clone();
    let sim_fs = build.meta.simulation_frequency;

    let mut engine = Engine::build(build);

    let duration = duration_override.unwrap_or_else(|| cfg.scenario.duration_s);
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

    // Temporary diagnostics
    {
        let max_p = engine
            .chambers
            .iter()
            .map(|c| c.firing_pressure())
            .fold(0.0f64, f64::max);
        let _cur_p: Vec<f64> = engine.chambers.iter().map(|c| c.system.pressure()).collect();
        let lit: Vec<bool> = engine.chambers.iter().map(|c| c.lit).collect();
        let p_fuel: Vec<f64> = engine.chambers.iter().map(|c| c.system.mix().p_fuel).collect();
        let p_o2: Vec<f64> = engine.chambers.iter().map(|c| c.system.mix().p_o2).collect();
        let intake_pf = engine.intakes[0].system.mix().p_fuel;
        let intake_p = engine.intakes[0].system.pressure();
        let omega = engine.omega();
        let mut max_il = 0.0f64;
        let mut max_el = 0.0f64;
        for k in 0..720 {
            let crank = k as f64 * std::f64::consts::PI / 360.0;
            for j in 0..engine.chambers.len().min(3) {
                let il = engine.valve_lifts(j, crank).0;
                let el = engine.valve_lifts(j, crank).1;
                max_il = max_il.max(il);
                max_el = max_el.max(el);
            }
        }
        eprintln!(
            "DIAG max_firing_p={max_p:.0} lit_any={} intake_pf={intake_pf:.4} intake_p={intake_p:.0} omega={omega:.1} max_il={max_il:.4} max_el={max_el:.4} p_fuel0={:?} p_o2_0={:.3}",
            lit.iter().any(|&x| x),
            &p_fuel[..3.min(p_fuel.len())],
            p_o2.first().copied().unwrap_or(0.0),
        );
        eprintln!(
            "DIAG peak_p={:.0} min_v={:.4e} max_v={:.4e} v_end={:.4e} burnt_fuel={:.4e} n_burnt={:?}",
            engine.diag_peak_p,
            engine.diag_min_v,
            engine.diag_max_v,
            c0_vol(&engine),
            engine.diag_burnt_fuel,
            engine.chambers.iter().map(|c| c.n_burnt_fuel).collect::<Vec<_>>(),
        );
        // Spark-time mix (approx): dump first chamber mix + molecular_afr equivalence window
        let c0 = &engine.chambers[0];
        let m = c0.system.mix();
        let afr = if m.p_fuel > 0.0 { m.p_o2 / m.p_fuel } else { 0.0 };
        // default fuel molecular_afr = 12.5
        let eq = afr / 12.5;
        eprintln!(
            "DIAG end_ch0 p={:.0} T={:.0} V={:.3e} n={:.3e} p_fuel={:.4e} p_o2={:.4} afr={:.2} eq={:.3} lit={}",
            c0.system.pressure(),
            c0.system.temperature(),
            c0.system.volume(),
            c0.system.n(),
            m.p_fuel,
            m.p_o2,
            afr,
            eq,
            c0.lit,
        );
        let n = out.rpm.len();
        if n > 10 {
            let samples: Vec<String> = (0..10)
                .map(|i| format!("{:.0}", out.rpm[i * n / 10]))
                .collect();
            eprintln!("DIAG rpm_tens={}", samples.join(","));
        }
    }

    let audio_rate = cfg.audio.sample_rate;
    let params = AudioParameters {
        volume: cfg.audio.volume,
        convolution: cfg.audio.convolution,
        d_f_f_mix: cfg.audio.d_f_f_mix,
        input_sample_noise: cfg.audio.input_sample_noise,
        input_sample_noise_frequency_cutoff: cfg.audio.input_sample_noise_frequency_cutoff,
        air_noise: cfg.audio.air_noise,
        air_noise_frequency_cutoff: cfg.audio.air_noise_frequency_cutoff,
        leveler_target: cfg
            .audio
            .leveler_target
            .unwrap_or(30_000.0),
        leveler_max_gain: cfg.audio.leveler_max_gain.unwrap_or(1.9),
        leveler_min_gain: 1e-5,
    };

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
                    eprintln!(
                        "  IR ch{i}: {} ({} samples @ {} Hz, vol {})",
                        path.display(),
                        samples.len(),
                        sr,
                        ex.impulse_response_volume
                    );
                    irs[i] = Some((sr, samples, ex.impulse_response_volume as f32));
                }
                Err(e) => eprintln!("  warn: IR load failed {path:?}: {e}"),
            }
        }
    }

    eprintln!(
        "DSP: {} sim samples → audio_rate {audio_rate} Hz...",
        out.audio_channels.first().map(|c| c.len()).unwrap_or(0)
    );

    let samples = render_offline(&out.audio_channels, sim_fs, audio_rate, &params, &irs);

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

    let peak = samples
        .iter()
        .map(|&s| (s as i32).abs())
        .max()
        .unwrap_or(0);
    let nonzero = samples.iter().filter(|&&s| s != 0).count();
    let clipped = samples
        .iter()
        .filter(|&&s| s == i16::MIN || s == i16::MAX)
        .count();
    let first_clip = samples
        .iter()
        .position(|&s| s == i16::MIN || s == i16::MAX)
        .unwrap_or(usize::MAX);

    write_wav_i16_mono(&out_path, audio_rate as u32, &samples)?;

    let rpm_first = out.rpm.first().copied().unwrap_or(0.0);
    let rpm_last = out.rpm.last().copied().unwrap_or(0.0);

    Ok(format!(
        "wrote {} ({} samples @ {} Hz, peak={peak}, clipped={clipped} first_clip={first_clip}, nonzero={nonzero}, rpm {rpm_first:.0}→{rpm_last:.0})",
        out_path.display(),
        samples.len(),
        audio_rate as u32
    ))
}

fn c0_vol(engine: &Engine) -> f64 {
    engine.chambers.first().map(|c| c.system.volume()).unwrap_or(0.0)
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
        ScenarioEvent { time: 0.0, event: Event::SetStarter(true) },
        ScenarioEvent { time: 0.0, event: Event::SetIgnition(true) },
        ScenarioEvent { time: 0.0, event: Event::SetThrottle(0.12) },
        ScenarioEvent { time: 0.4, event: Event::SetStarter(false) },
        ScenarioEvent { time: 0.5, event: Event::SetThrottle(1.0) },
        ScenarioEvent { time: 2.5, event: Event::SetThrottle(0.0) },
        ScenarioEvent { time: 3.0, event: Event::SetThrottle(0.0) },
    ];
    let _ = redline;
    evs.sort_by(|a, b| a.time.total_cmp(&b.time));
    evs
}
