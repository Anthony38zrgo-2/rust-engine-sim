//! Coast/accel CPU bench: GF509 bank vs F2002 experimental bank.
//!
//! Usage: v10_coast_bench <gf509-dir> <experimental-dir>
//! Reports wall time per 256-frame block (p50/p99/max + % of realtime
//! budget) split by accel/coast phase, at 44100 and 48000 Hz.

use std::hint::black_box;
use std::path::PathBuf;
use std::time::Instant;

use v10_engine_synth::{
    Gf509Runtime, Gf509RuntimeConfig, RuntimeTelemetry, ShiftPhase, TorqueSign,
};

const BLOCK: usize = 256;
const WARMUP_S: f32 = 1.0;
const ACCEL_S: f32 = 7.0;
const COAST_S: f32 = 3.0;

fn telemetry(time_s: f32, dt: f32) -> RuntimeTelemetry {
    if time_s < ACCEL_S {
        let t = time_s / ACCEL_S;
        let shaped = t * t * (3.0 - 2.0 * t);
        RuntimeTelemetry {
            rpm: 5_000.0 + 13_000.0 * shaped,
            throttle: 0.95,
            normalized_engine_load: 0.90,
            normalized_engine_torque: 0.85,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: 13_000.0 / ACCEL_S,
            throttle_derivative: 0.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: dt,
        }
    } else {
        let t = time_s - ACCEL_S;
        let decay = 1.0 - (1.0 - t / COAST_S).powf(1.55);
        RuntimeTelemetry {
            rpm: 18_000.0 - 13_000.0 * decay,
            throttle: 0.035,
            normalized_engine_load: 0.10,
            normalized_engine_torque: -0.53,
            torque_sign: TorqueSign::Negative,
            rpm_derivative: -13_000.0 / COAST_S,
            throttle_derivative: 0.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: dt,
        }
    }
}

fn bench(name: &str, dir: Option<&str>, sample_rate: u32) -> Result<(), String> {
    let dt = BLOCK as f32 / sample_rate as f32;
    let mut config = Gf509RuntimeConfig::default();
    config.engine.sample_rate = sample_rate;
    config.max_block_frames = BLOCK;
    config.sample_layer_directory = dir.map(PathBuf::from);
    if name == "f2002nooff" {
        config.sample_layer.off_throttle_gain = 0.0;
    }
    if name == "f2002m0" {
        config.sample_layer.suspend_margin_rpm = 0.0;
    }
    if name == "f2002m150" {
        config.sample_layer.suspend_margin_rpm = 150.0;
    }
    let mut runtime = Gf509Runtime::new(config)?;
    let mut left = [0.0f32; BLOCK];
    let mut right = [0.0f32; BLOCK];
    // Warmup (excluded).
    let warmup_blocks = (WARMUP_S / dt) as usize;
    for block in 0..warmup_blocks {
        let time_s = block as f32 * dt;
        runtime.update_telemetry(telemetry(time_s.min(ACCEL_S - dt), dt))?;
        runtime.render_block(black_box(&mut left), black_box(&mut right))?;
    }
    let mut accel_us: Vec<f64> = Vec::new();
    let mut coast_us: Vec<f64> = Vec::new();
    let total_blocks = ((ACCEL_S + COAST_S) / dt) as usize;
    for block in 0..total_blocks {
        let time_s = block as f32 * dt;
        runtime.update_telemetry(telemetry(time_s, dt))?;
        let start = Instant::now();
        runtime.render_block(black_box(&mut left), black_box(&mut right))?;
        black_box(left[0]);
        let elapsed_us = start.elapsed().as_secs_f64() * 1.0e6;
        if time_s < ACCEL_S {
            accel_us.push(elapsed_us);
        } else {
            coast_us.push(elapsed_us);
        }
    }
    let budget_us = dt as f64 * 1.0e6;
    for (phase, mut samples) in [("accel", accel_us), ("coast", coast_us)] {
        samples.sort_by(|a, b| a.total_cmp(b));
        let last = samples.len() - 1;
        let pick = |q: f64| samples[((q * samples.len() as f64) as usize).min(last)];
        println!(
            "{name} sr={sample_rate} {phase}: n={} p50={:.1}us ({:.1}%RT) p99={:.1}us ({:.1}%RT) max={:.1}us ({:.1}%RT)",
            samples.len(),
            pick(0.50),
            pick(0.50) / budget_us * 100.0,
            pick(0.99),
            pick(0.99) / budget_us * 100.0,
            samples[samples.len() - 1],
            samples[samples.len() - 1] / budget_us * 100.0,
        );
    }
    Ok(())
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        return Err("usage: v10_coast_bench <gf509-dir> <experimental-dir>".into());
    }
    // Note: the bank validator requires manifest rate == output rate and
    // the game runs at 44100, so 48000 is not a supported configuration.
    for sample_rate in [44_100u32] {
        bench("proc", None, sample_rate)?;
        bench("gf509", Some(&args[1]), sample_rate)?;
        bench("f2002", Some(&args[2]), sample_rate)?;
        bench("f2002nooff", Some(&args[2]), sample_rate)?;
        bench("f2002m0", Some(&args[2]), sample_rate)?;
        bench("f2002m150", Some(&args[2]), sample_rate)?;
    }
    Ok(())
}
