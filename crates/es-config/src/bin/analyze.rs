//! Diagnostic: analyze a WAV for clipping, leveler plateaus and engine orders.
//! Usage: cargo run -p es-config --bin analyze -- <file.wav> [options]

use es_audio_io::read_wav_mono_i16;
use rustfft::num_complex::Complex;
use rustfft::FftPlanner;
use serde::Serialize;

#[derive(Serialize, Clone, Copy)]
struct OrderRow {
    name: &'static str,
    frequency_hz: f64,
    magnitude_db: f64,
    rel_vs_firing_db: f64,
}

#[derive(Serialize, Clone)]
struct WindowReport {
    timestamp_s: f64,
    rpm: f64,
    orders: Vec<OrderRow>,
    spectral_centroid_hz: f64,
    spectral_rolloff_hz: f64,
    rms: f64,
    peak: f64,
    crest_factor: f64,
    clip_count: usize,
    plateau_count: usize,
    max_flat_run: usize,
    bank_alarm: bool,
}

#[derive(Serialize)]
struct GlobalStats {
    peak: f64,
    rms: f64,
    crest_factor: f64,
    clip_count: usize,
    plateau_count: usize,
    max_flat_run: usize,
}

#[derive(Serialize)]
struct Report {
    file: String,
    sample_rate: u32,
    duration_s: f64,
    rpm_start: f64,
    rpm_end: f64,
    rpm_t0_s: f64,
    rpm_t1_s: f64,
    cylinder_count: usize,
    cycle_type: u32,
    window: usize,
    threshold_db: f64,
    bank_alarm_count: usize,
    windows: Vec<WindowReport>,
    global: GlobalStats,
}

struct Config {
    path: String,
    rpm_start: f64,
    rpm_end: f64,
    duration: f64,
    rpm_t0: f64,
    rpm_t1: f64,
    cylinders: usize,
    cycle: u32,
    window: usize,
    threshold_db: f64,
    leveler_target: Option<f64>,
    probes: Vec<f64>,
    json_out: Option<String>,
    md_out: Option<String>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            path: String::new(),
            rpm_start: 0.0,
            rpm_end: 0.0,
            duration: 0.0,
            rpm_t0: 0.0,
            rpm_t1: f64::NAN,
            cylinders: 10,
            cycle: 4,
            window: 4096,
            threshold_db: 3.0,
            leveler_target: None,
            probes: vec![10_000.0, 14_000.0, 18_000.0],
            json_out: None,
            md_out: None,
        }
    }
}

fn parse_args() -> Result<Config, String> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        return Err("usage: analyze <file.wav> [--rpm-start N] [--rpm-end N] [--duration S] [--rpm-t0 S] [--rpm-t1 S] [--cylinders N] [--cycle 4] [--window N] [--threshold-db N] [--leveler-target N] [--probe r1,r2] [--json out.json] [--md out.md]".into());
    }
    let mut cfg = Config::default();
    cfg.path = args[1].clone();
    let mut i = 2;
    while i < args.len() {
        let next_f = |i: &mut usize| -> Result<f64, String> {
            *i += 1;
            args.get(*i)
                .and_then(|s| s.parse::<f64>().ok())
                .ok_or_else(|| format!("missing value for {}", args[*i - 1]))
        };
        match args[i].as_str() {
            "--rpm-start" => cfg.rpm_start = next_f(&mut i)?,
            "--rpm-end" => cfg.rpm_end = next_f(&mut i)?,
            "--duration" => cfg.duration = next_f(&mut i)?,
            "--rpm-t0" => cfg.rpm_t0 = next_f(&mut i)?,
            "--rpm-t1" => cfg.rpm_t1 = next_f(&mut i)?,
            "--cylinders" => cfg.cylinders = next_f(&mut i)? as usize,
            "--cycle" => cfg.cycle = next_f(&mut i)? as u32,
            "--window" => cfg.window = next_f(&mut i)? as usize,
            "--threshold-db" => cfg.threshold_db = next_f(&mut i)?,
            "--leveler-target" => cfg.leveler_target = Some(next_f(&mut i)?),
            "--probe" => {
                i += 1;
                let raw = args.get(i).ok_or("missing value for --probe")?;
                cfg.probes = raw
                    .split(',')
                    .filter_map(|s| s.trim().parse::<f64>().ok())
                    .collect();
            }
            "--json" => {
                i += 1;
                cfg.json_out = Some(args.get(i).ok_or("missing value for --json")?.clone());
            }
            "--md" => {
                i += 1;
                cfg.md_out = Some(args.get(i).ok_or("missing value for --md")?.clone());
            }
            other => return Err(format!("unknown flag: {other}")),
        }
        i += 1;
    }
    if cfg.window < 64 {
        return Err("window must be >= 64".into());
    }
    Ok(cfg)
}

fn rpm_at(t: f64, cfg: &Config) -> f64 {
    let t1 = if cfg.rpm_t1.is_nan() {
        cfg.duration
    } else {
        cfg.rpm_t1
    };
    if cfg.rpm_end <= cfg.rpm_start || t1 <= cfg.rpm_t0 {
        return cfg.rpm_start;
    }
    let u = ((t - cfg.rpm_t0) / (t1 - cfg.rpm_t0)).clamp(0.0, 1.0);
    cfg.rpm_start + (cfg.rpm_end - cfg.rpm_start) * u
}

fn order_frequencies(rpm: f64, cfg: &Config) -> [(&'static str, f64); 9] {
    let crank = rpm / 60.0;
    let firing = crank * cfg.cylinders as f64 / (cfg.cycle as f64 / 2.0);
    let bank = firing / 2.0;
    [
        ("crank", crank),
        ("bank", bank),
        ("firing", firing),
        ("firing2", 2.0 * firing),
        ("firing3", 3.0 * firing),
        ("firing4", 4.0 * firing),
        ("firing5", 5.0 * firing),
        ("firing6", 6.0 * firing),
        ("firing7", 7.0 * firing),
    ]
}

fn band_peak(bins: &[f64], bin_hz: f64, target: f64) -> f64 {
    if target <= 0.0 {
        return 0.0;
    }
    let half = (3.0 * bin_hz).max(1.0);
    let lo = ((target - half) / bin_hz).floor().max(1.0) as usize;
    let hi = (((target + half) / bin_hz).ceil() as usize).min(bins.len() - 1);
    if hi < lo {
        return 0.0;
    }
    bins[lo..=hi].iter().cloned().fold(0.0, f64::max)
}

fn db(x: f64) -> f64 {
    20.0 * (x + 1e-12).log10()
}

fn analyze_window(
    samples: &[i16],
    start: usize,
    window: usize,
    sr: f64,
    cfg: &Config,
) -> WindowReport {
    let end = (start + window).min(samples.len());
    let n = end - start;
    let ts = (start + n / 2) as f64 / sr;
    let rpm = rpm_at(ts, cfg);

    let mut buf: Vec<Complex<f64>> = Vec::with_capacity(window);
    let mut peak = 0.0f64;
    let mut sum2 = 0.0f64;
    let mut clip_count = 0usize;
    let mut plateau_count = 0usize;
    let mut max_flat_run = 0usize;
    let mut flat_run = 0usize;
    let mut prev: Option<i16> = None;
    for i in 0..window {
        let s = if start + i < samples.len() {
            samples[start + i]
        } else {
            0
        };
        let v = s as f64;
        let a = v.abs();
        if a > peak {
            peak = a;
        }
        sum2 += v * v;
        if a >= 32767.0 {
            clip_count += 1;
        }
        if let Some(t) = cfg.leveler_target {
            if a >= t * 0.99 {
                plateau_count += 1;
            }
        }
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
        let w = if n > 1 {
            0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / (n as f64 - 1.0)).cos()
        } else {
            1.0
        };
        buf.push(Complex::new(v / 32768.0 * w, 0.0));
    }

    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(window);
    fft.process(&mut buf);

    let half = window / 2;
    let bin_hz = sr / window as f64;
    let bins: Vec<f64> = buf[..half].iter().map(|c| c.norm()).collect();

    let orders_freq = order_frequencies(rpm, cfg);
    let firing_mag = band_peak(&bins, bin_hz, orders_freq[2].1);
    let firing_db = db(firing_mag);
    let orders: Vec<OrderRow> = orders_freq
        .iter()
        .map(|&(name, f)| {
            let m = db(band_peak(&bins, bin_hz, f));
            OrderRow {
                name,
                frequency_hz: f,
                magnitude_db: m,
                rel_vs_firing_db: m - firing_db,
            }
        })
        .collect();

    let total_mag: f64 = bins.iter().sum();
    let centroid = if total_mag > 0.0 {
        bins.iter()
            .enumerate()
            .map(|(i, m)| i as f64 * bin_hz * m)
            .sum::<f64>()
            / total_mag
    } else {
        0.0
    };
    let total_energy: f64 = bins.iter().map(|m| m * m).sum();
    let mut acc = 0.0;
    let mut rolloff = 0.0;
    for (i, m) in bins.iter().enumerate() {
        acc += m * m;
        if total_energy > 0.0 && acc >= 0.85 * total_energy {
            rolloff = i as f64 * bin_hz;
            break;
        }
    }

    let rms = (sum2 / n.max(1) as f64).sqrt();
    let crest = if rms > 0.0 { peak / rms } else { 0.0 };
    let bank_rel = orders[1].rel_vs_firing_db;

    WindowReport {
        timestamp_s: ts,
        rpm,
        orders,
        spectral_centroid_hz: centroid,
        spectral_rolloff_hz: rolloff,
        rms,
        peak,
        crest_factor: crest,
        clip_count,
        plateau_count,
        max_flat_run,
        bank_alarm: bank_rel > cfg.threshold_db,
    }
}

fn main() {
    let cfg = match parse_args() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    let (sr, samples) = match read_wav_mono_i16(&cfg.path) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("read error: {e}");
            std::process::exit(1);
        }
    };
    let duration = if cfg.duration > 0.0 {
        cfg.duration
    } else {
        samples.len() as f64 / sr as f64
    };

    let hop = (cfg.window / 2).max(1);
    let mut windows = Vec::new();
    let mut start = 0usize;
    while start + cfg.window <= samples.len() {
        windows.push(analyze_window(&samples, start, cfg.window, sr as f64, &cfg));
        start += hop;
    }
    if windows.is_empty() && !samples.is_empty() {
        windows.push(analyze_window(&samples, 0, cfg.window, sr as f64, &cfg));
    }

    let mut peak = 0.0f64;
    let mut sum2 = 0.0f64;
    let mut clip_count = 0usize;
    let mut plateau_count = 0usize;
    let mut max_flat_run = 0usize;
    for &s in &samples {
        let a = (s as f64).abs();
        if a > peak {
            peak = a;
        }
        sum2 += (s as f64) * (s as f64);
        if a >= 32767.0 {
            clip_count += 1;
        }
        if let Some(t) = cfg.leveler_target {
            if a >= t * 0.99 {
                plateau_count += 1;
            }
        }
    }
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
    let rms = (sum2 / samples.len().max(1) as f64).sqrt();
    let crest = if rms > 0.0 { peak / rms } else { 0.0 };
    let bank_alarm_count = windows.iter().filter(|w| w.bank_alarm).count();

    let report = Report {
        file: cfg.path.clone(),
        sample_rate: sr,
        duration_s: duration,
        rpm_start: cfg.rpm_start,
        rpm_end: cfg.rpm_end,
        rpm_t0_s: cfg.rpm_t0,
        rpm_t1_s: if cfg.rpm_t1.is_nan() {
            duration
        } else {
            cfg.rpm_t1
        },
        cylinder_count: cfg.cylinders,
        cycle_type: cfg.cycle,
        window: cfg.window,
        threshold_db: cfg.threshold_db,
        bank_alarm_count,
        windows: windows.clone(),
        global: GlobalStats {
            peak,
            rms,
            crest_factor: crest,
            clip_count,
            plateau_count,
            max_flat_run,
        },
    };

    println!(
        "{}: {} samples @ {} Hz ({:.2}s) | peak={:.0} rms={:.0} crest={:.2} clips={} plateau={} flat_run={}",
        report.file,
        samples.len(),
        sr,
        duration,
        peak,
        rms,
        crest,
        clip_count,
        plateau_count,
        max_flat_run
    );
    println!(
        "{:>8} {:>7} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>7}",
        "t(s)", "rpm", "crank", "bank", "firing", "2xf", "3xf", "4xf", "alarm"
    );
    for w in &windows {
        let rel = |idx: usize| w.orders[idx].rel_vs_firing_db;
        println!(
            "{:>8.2} {:>7.0} {:>9.1} {:>9.1} {:>9.1} {:>9.1} {:>9.1} {:>9.1} {:>7}",
            w.timestamp_s,
            w.rpm,
            rel(0),
            rel(1),
            rel(2),
            rel(3),
            rel(4),
            rel(5),
            if w.bank_alarm { "BANK" } else { "" }
        );
    }

    println!();
    for probe in &cfg.probes {
        if let Some(w) = windows.iter().min_by(|a, b| {
            (a.rpm - probe)
                .abs()
                .partial_cmp(&(b.rpm - probe).abs())
                .unwrap()
        }) {
            println!(
                "probe {:>6.0} rpm (t={:.2}s, est {:>6.0} rpm):",
                probe, w.timestamp_s, w.rpm
            );
            for o in &w.orders {
                println!(
                    "    {:>8} {:>9.1} Hz  {:>9.2} dB  rel {:>7.2} dB",
                    o.name, o.frequency_hz, o.magnitude_db, o.rel_vs_firing_db
                );
            }
            println!(
                "    centroid {:.0} Hz  rolloff85 {:.0} Hz  crest {:.2}",
                w.spectral_centroid_hz, w.spectral_rolloff_hz, w.crest_factor
            );
        }
    }
    if bank_alarm_count > 0 {
        println!(
            "ALARM: bank order above firing by > {:.1} dB in {} windows",
            cfg.threshold_db, bank_alarm_count
        );
    }

    if let Some(path) = &cfg.json_out {
        let text = serde_json::to_string_pretty(&report).unwrap();
        if let Err(e) = std::fs::write(path, text) {
            eprintln!("json write error: {e}");
        } else {
            println!("wrote {path}");
        }
    }

    if let Some(path) = &cfg.md_out {
        let mut md = String::new();
        md.push_str(&format!(
            "# Order analysis: {}\n\nSample rate {} Hz, window {}, threshold {:.1} dB, bank alarms {}\n\n",
            report.file, sr, cfg.window, cfg.threshold_db, bank_alarm_count
        ));
        md.push_str("| rpm | crank rel dB | bank rel dB | firing dB | 2xf rel | 3xf rel | 4xf rel | 5xf rel | 6xf rel | centroid Hz | rolloff Hz |\n");
        md.push_str("|---|---|---|---|---|---|---|---|---|---|---|\n");
        let mut probe_windows: Vec<&WindowReport> = Vec::new();
        for probe in &cfg.probes {
            if let Some(w) = windows.iter().min_by(|a, b| {
                (a.rpm - probe)
                    .abs()
                    .partial_cmp(&(b.rpm - probe).abs())
                    .unwrap()
            }) {
                if !probe_windows.iter().any(|x| x.timestamp_s == w.timestamp_s) {
                    probe_windows.push(w);
                }
            }
        }
        for w in &probe_windows {
            md.push_str(&format!(
                "| {:.0} | {:+.2} | {:+.2} | {:.2} | {:+.2} | {:+.2} | {:+.2} | {:+.2} | {:+.2} | {:.0} | {:.0} |\n",
                w.rpm,
                w.orders[0].rel_vs_firing_db,
                w.orders[1].rel_vs_firing_db,
                w.orders[2].magnitude_db,
                w.orders[3].rel_vs_firing_db,
                w.orders[4].rel_vs_firing_db,
                w.orders[5].rel_vs_firing_db,
                w.orders[6].rel_vs_firing_db,
                w.orders[7].rel_vs_firing_db,
                w.spectral_centroid_hz,
                w.spectral_rolloff_hz,
            ));
        }
        md.push_str(&format!(
            "\nGlobal: peak {:.0}, RMS {:.0}, crest {:.2}, clips {}, plateau {}, max flat run {}\n",
            peak, rms, crest, clip_count, plateau_count, max_flat_run
        ));
        if let Err(e) = std::fs::write(path, md) {
            eprintln!("md write error: {e}");
        } else {
            println!("wrote {path}");
        }
    }
}
