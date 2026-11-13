//! PHY-061 measurement probe (throwaway).
//!
//! Runs the engine to a steady point twice: once with temperature-dependent
//! exhaust wave speed (the new default) and once with the fixed 545 m/s
//! compatibility mode. Both emit through the identical chain, so the only
//! difference is the runner pipe's propagation speed. Measures the emitted
//! exhaust spectrum, the runner gas temperature / resulting wave speed, and
//! any level change. Evidence behind the PHY-061 decision note; not part of
//! the library.

use std::f32::consts::TAU;

use v10_engine_synth::config::EngineConfig;
use v10_engine_synth::engine::{EngineFrame, EngineInput, V10Engine};
use v10_engine_synth::crank::CYLINDER_COUNT;

/// Exhaust-gas physical constants mirroring the library runner boundary.
const GAMMA: f32 = 1.4;
const R_EXHAUST: f32 = 8.314 / 0.0289;

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

fn speed_of_sound(temperature_k: f32) -> f32 {
    (GAMMA * R_EXHAUST * temperature_k.max(1.0)).sqrt()
}

/// Capture the exhaust bus (radiated) over a steady window and summarise it.
fn capture_bus(engine: &mut V10Engine, capture: usize) -> (Vec<f32>, f32, f32) {
    let mut exhaust = vec![0.0f32; capture];
    let mut temp_sum = 0.0f64;
    let mut temp_n = 0u64;
    let mut temp_min = f32::MAX;
    let mut temp_max = f32::MIN;
    let mut runner_speed = 0.0f64;
    let mut speed_n = 0u64;
    for sample in 0..capture {
        let frame: EngineFrame = engine.render_sample();
        exhaust[sample] = frame.exhaust;
        for c in 0..CYLINDER_COUNT {
            let t = frame.cylinder_runner_temperature_k[c];
            temp_sum += t as f64;
            runner_speed += speed_of_sound(t) as f64;
            temp_min = temp_min.min(t);
            temp_max = temp_max.max(t);
            temp_n += 1;
            speed_n += 1;
        }
    }
    let mean_temp = temp_sum / temp_n.max(1) as f64;
    let mean_speed = runner_speed / speed_n.max(1) as f64;
    println!(
        "  runner_T min={temp_min:.0} K max={temp_max:.0} K mean={mean_temp:.0} K -> c_min={:.0} c_max={:.0} m/s",
        speed_of_sound(temp_min),
        speed_of_sound(temp_max)
    );
    (exhaust, mean_temp as f32, mean_speed as f32)
}

fn summarise(name: &str, exhaust: &[f32], measure_start: usize, sr: f32) {
    let meas = &exhaust[measure_start..];
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

    println!("=== {name} ===");
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

fn main() {
    let sr = EngineConfig::default().sample_rate as f32;
    let deg_per_sample = CASE_RPM * 6.0 / sr;
    let samples_per_cycle = (720.0 / deg_per_sample).round() as usize;
    let capture = CYCLES_CAPTURED * samples_per_cycle;
    let measure_start = CYCLES_SKIPPED * samples_per_cycle;
    let measure_len = capture - measure_start;

    let mut temp_on = EngineConfig::default();
    temp_on.use_temperature_dependent_wave_speed = true;
    let mut temp_off = temp_on.clone();
    temp_off.use_temperature_dependent_wave_speed = false;

    let mut engine = V10Engine::new(temp_on).unwrap();
    engine
        .set_input(EngineInput {
            rpm: CASE_RPM,
            throttle: CASE_THROTTLE,
            load: CASE_LOAD,
        })
        .unwrap();
    for _ in 0..(sr as usize * 2) {
        engine.render_sample();
    }
    let (on, temp, speed) = capture_bus(&mut engine, capture);
    println!(
        "temperature-dependent wave speed: mean runner T = {temp:.0} K -> mean c = {speed:.0} m/s (fixed compat = 545 m/s)"
    );
    println!(
        "firing fundamental = {:.2} Hz ; measure window = {measure_len} samples",
        CASE_RPM / 60.0 * 2.5
    );
    println!();
    summarise("temperature-dependent (default ON)", &on, measure_start, sr);

    let mut engine = V10Engine::new(temp_off).unwrap();
    engine
        .set_input(EngineInput {
            rpm: CASE_RPM,
            throttle: CASE_THROTTLE,
            load: CASE_LOAD,
        })
        .unwrap();
    for _ in 0..(sr as usize * 2) {
        engine.render_sample();
    }
    let (off, _, _) = capture_bus(&mut engine, capture);
    summarise("fixed 545 m/s (compatibility)", &off, measure_start, sr);

    let on_rms = (on[measure_start..].iter().map(|&x| (x * x) as f64).sum::<f64>() / measure_len as f64).sqrt();
    let off_rms = (off[measure_start..].iter().map(|&x| (x * x) as f64).sum::<f64>() / measure_len as f64).sqrt();
    println!(
        "level delta (temperature-dependent vs fixed): {on_rms:.6} / {off_rms:.6} = {:.3} dB",
        20.0 * (on_rms / off_rms.max(1.0e-12)).log10()
    );
}
