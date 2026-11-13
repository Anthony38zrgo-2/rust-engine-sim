//! PHY-052 measurement probe (throwaway).
//!
//! Runs the engine to a steady point, captures each candidate physical
//! excitation signal per cylinder, and replays each candidate (peak-normalised
//! per cylinder) through the real header -> collector -> radiation chain to
//! compare the emitted exhaust waveform and per-order spectrum. This is the
//! evidence behind the PHY-052 decision note; it is not part of the library.

use std::f32::consts::TAU;

use v10_engine_synth::acoustics::Collector;
use v10_engine_synth::config::EngineConfig;
use v10_engine_synth::engine::{EngineInput, V10Engine};
use v10_engine_synth::exhaust::runner::RunnerWaveguide;
use v10_engine_synth::crank::CYLINDER_COUNT;

const CASE_RPM: f32 = 7_499.0;
const CASE_THROTTLE: f32 = 0.72;
const CASE_LOAD: f32 = 0.66;
const CYCLES_CAPTURED: usize = 14;
const CYCLES_SKIPPED: usize = 2;

fn goertzel_mag(samples: &[f32], freq_hz: f32, sr: f32) -> f64 {
    let w = TAU * freq_hz / sr;
    let cosine = w.cos();
    let coeff = 2.0 * cosine;
    let mut s1 = 0.0f64;
    let mut s2 = 0.0f64;
    for &x in samples {
        let s0 = x as f64 + coeff as f64 * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    let re = s1 - s2 * cosine as f64;
    let im = s2 * w.sin() as f64;
    (re * re + im * im).sqrt() / samples.len() as f64
}

fn main() {
    let config = EngineConfig::default();
    let sr = config.sample_rate as f32;
    let mut engine = V10Engine::new(config.clone()).unwrap();
    engine
        .set_input(EngineInput {
            rpm: CASE_RPM,
            throttle: CASE_THROTTLE,
            load: CASE_LOAD,
        })
        .unwrap();

    let deg_per_sample = CASE_RPM * 6.0 / sr;
    let samples_per_cycle = (720.0 / deg_per_sample).round() as usize;
    let capture = CYCLES_CAPTURED * samples_per_cycle;

    // Warm the physical model (energy smoothing, runner states) to a steady
    // point before capturing diagnostic frames.
    for _ in 0..(sr as usize * 2) {
        engine.render_sample();
    }

    let mut proxy = vec![[0.0f32; CYLINDER_COUNT]; capture];
    let mut mass_flow = vec![[0.0f32; CYLINDER_COUNT]; capture];
    let mut runner_p = vec![[0.0f32; CYLINDER_COUNT]; capture];
    let mut live_excitation = vec![[0.0f32; CYLINDER_COUNT]; capture];
    for sample in 0..capture {
        let frame = engine.render_sample();
        for c in 0..CYLINDER_COUNT {
            proxy[sample][c] = frame.cylinder_blowdown[c];
            mass_flow[sample][c] = frame.cylinder_mass_flow[c];
            runner_p[sample][c] = frame.cylinder_runner_pressure[c];
            live_excitation[sample][c] = frame.cylinder_exhaust_excitation[c];
        }
    }

    // Per-cylinder derivative candidate (t -> t-1).
    let mut dmass_flow = vec![[0.0f32; CYLINDER_COUNT]; capture];
    let mut drunner_p = vec![[0.0f32; CYLINDER_COUNT]; capture];
    for sample in 1..capture {
        for c in 0..CYLINDER_COUNT {
            dmass_flow[sample][c] = mass_flow[sample][c] - mass_flow[sample - 1][c];
            drunner_p[sample][c] = runner_p[sample][c] - runner_p[sample - 1][c];
        }
    }

    let candidates: Vec<(&str, &Vec<[f32; CYLINDER_COUNT]>)> = vec![
        ("live_excitation (routed)", &live_excitation),
        ("proxy d(p*lift^1.35)", &proxy),
        ("mass_flow", &mass_flow),
        ("d_mass_flow", &dmass_flow),
        ("runner_pressure", &runner_p),
        ("d_runner_pressure", &drunner_p),
    ];

    let measure_start = CYCLES_SKIPPED * samples_per_cycle;
    let measure_len = capture - measure_start;

    println!("crate: v10-engine-synth  rpm={CASE_RPM}  thr={CASE_THROTTLE}  load={CASE_LOAD}");
    println!(
        "firing fundamental = {:.2} Hz ; measure window = {measure_len} samples",
        CASE_RPM / 60.0 * 2.5
    );
    println!();

    for (name, signal) in &candidates {
        // Replay this candidate through the real acoustic chain.
        let mut headers: Vec<RunnerWaveguide> = (0..CYLINDER_COUNT)
            .map(|c| RunnerWaveguide::new(config.header_lengths_m[c], 545.0, -0.34, sr, false))
            .collect();
        let mut col_a = Collector::new(sr, -3.5);
        let mut col_b = Collector::new(sr, 3.5);
        let mut radiated = vec![0.0f32; capture];

        // Normalise each cylinder to unit peak so every cylinder contributes
        // equally and we isolate the excitation's spectral character.
        let mut peak = [0.0f32; CYLINDER_COUNT];
        for sample in 0..capture {
            for c in 0..CYLINDER_COUNT {
                peak[c] = peak[c].max(signal[sample][c].abs());
            }
        }
        for sample in 0..capture {
            let mut header_a = 0.0f32;
            let mut header_b = 0.0f32;
            for c in 0..CYLINDER_COUNT {
                let norm = if peak[c] > 0.0 {
                    signal[sample][c] / peak[c]
                } else {
                    0.0
                };
                let h = headers[c].process(norm, 0.0);
                if c < 5 {
                    header_a += h;
                } else {
                    header_b += h;
                }
            }
            let out_a = col_a.process(header_a);
            let out_b = col_b.process(header_b);
            radiated[sample] = out_a.radiated + out_b.radiated;
        }

        let meas = &radiated[measure_start..];
        let mut rms = 0.0f64;
        let mut pk = 0.0f32;
        for &x in meas {
            rms += (x * x) as f64;
            pk = pk.max(x.abs());
        }
        rms = (rms / meas.len() as f64).sqrt();

        let fundamental = CASE_RPM / 60.0 * 2.5;
        let mut order_mag = [0.0f64; 16];
        let mut tonal_energy = 0.0f64;
        for (i, m) in order_mag.iter_mut().enumerate() {
            *m = goertzel_mag(meas, fundamental * (i as f32 + 1.0), sr);
            tonal_energy += *m * *m;
        }
        let total = meas.iter().map(|&x| (x * x) as f64).sum::<f64>();
        let mut centroid = 0.0f64;
        let mut tone_sum = 0.0f64;
        for (i, m) in order_mag.iter().enumerate() {
            let order = i as f64 + 1.0;
            centroid += order * *m;
            tone_sum += *m;
        }
        centroid = if tone_sum > 0.0 { centroid / tone_sum } else { 0.0 };

        let tone_frac = tonal_energy / total.max(1.0e-12);
        let crest = if rms > 0.0 { pk as f64 / rms } else { 0.0 };

        println!("=== {} ===", name);
        println!(
            "  rms={:.6}  peak={:.5}  crest={:.2}  centroid_order={:.2}  tonal_frac={:.3}  broadband_frac={:.3}",
            rms, pk, crest, centroid, tone_frac, (1.0 - tone_frac)
        );
        let top: Vec<String> = order_mag
            .iter()
            .enumerate()
            .filter(|(_, m)| **m > 1.0e-5)
            .map(|(i, m)| format!("O{}={:.4}", i + 1, m))
            .collect();
        println!("  orders: {}", top.join("  "));
        println!();
    }

    // Raw (un-normalised) level matching for the PHY-060 header re-feed. Reports
    // the excitation amplitude the runner actually sees so a fixed gain can be
    // chosen to keep the radiated level comparable to today's proxy.
    println!("=== raw per-cylinder peaks ===");
    for (name, signal) in &candidates {
        let mut pk = [0.0f32; CYLINDER_COUNT];
        for sample in 0..capture {
            for c in 0..CYLINDER_COUNT {
                pk[c] = pk[c].max(signal[sample][c].abs());
            }
        }
        let max_pk = pk.iter().cloned().fold(0.0f32, f32::max);
        println!(
            "  {name:24} max_peak={max_pk:.6}  series={:?}",
            &pk[..]
        );
    }
}
