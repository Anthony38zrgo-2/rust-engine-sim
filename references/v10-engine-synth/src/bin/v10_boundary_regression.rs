use std::fs;
use std::path::PathBuf;

use serde::Serialize;
use v10_engine_synth::{
    EngineConfig, EngineFrame, EngineInput, Gf509Runtime, Gf509RuntimeConfig,
    RuntimeTelemetry, ShiftPhase, TorqueSign, V10Engine,
};

const FIRING_ORDER: [usize; 10] = [0, 5, 1, 6, 2, 7, 3, 8, 4, 9];

#[derive(Default, Serialize)]
struct ExcitationMetric {
    peak: f32,
    rms: f32,
    max_abs_step: f32,
    rms_step: f32,
    max_abs_second_difference: f32,
}

#[derive(Default, Serialize)]
struct ScenarioReport {
    name: String,
    sample_rate: u32,
    samples: usize,
    finite: bool,
    bounded: bool,
    firing_events: u64,
    expected_firing_order: Vec<usize>,
    observed_firing_order: Vec<usize>,
    firing_order_ok: bool,
    max_phase_step_error_deg: f32,
    max_frame_abs: f32,
    excitations: serde_json::Value,
}

#[derive(Default, Serialize)]
struct RuntimeMatrixEntry {
    sample_rate: u32,
    block_size: usize,
    samples: usize,
    finite: bool,
    peak: f32,
    rms: f32,
}

#[derive(Default, Serialize)]
struct BoundaryReport {
    schema_version: String,
    contract: String,
    generated_from: String,
    scenarios: Vec<ScenarioReport>,
    runtime_matrix: Vec<RuntimeMatrixEntry>,
    semantic_pass: bool,
    numerical_pass: bool,
    runtime_matrix_pass: bool,
    human_listening_gate: String,
}

#[derive(Default)]
struct MetricAccumulator {
    previous: f32,
    previous_delta: f32,
    initialized: bool,
    sum_sq: f64,
    sum_delta_sq: f64,
    peak: f32,
    max_abs_step: f32,
    max_abs_second_difference: f32,
}

impl MetricAccumulator {
    fn push(&mut self, value: f32) {
        self.peak = self.peak.max(value.abs());
        self.sum_sq += value as f64 * value as f64;
        if self.initialized {
            let delta = value - self.previous;
            self.max_abs_step = self.max_abs_step.max(delta.abs());
            self.sum_delta_sq += delta as f64 * delta as f64;
            self.max_abs_second_difference = self
                .max_abs_second_difference
                .max((delta - self.previous_delta).abs());
            self.previous_delta = delta;
        } else {
            self.initialized = true;
        }
        self.previous = value;
    }

    fn finish(self, samples: usize) -> ExcitationMetric {
        let n = samples.max(1) as f64;
        let step_n = samples.saturating_sub(1).max(1) as f64;
        ExcitationMetric {
            peak: self.peak,
            rms: (self.sum_sq / n).sqrt() as f32,
            max_abs_step: self.max_abs_step,
            rms_step: (self.sum_delta_sq / step_n).sqrt() as f32,
            max_abs_second_difference: self.max_abs_second_difference,
        }
    }
}

#[derive(Default)]
struct ScenarioAccumulators {
    combustion_source: MetricAccumulator,
    chamber_pressure: MetricAccumulator,
    chamber_temperature: MetricAccumulator,
    heat_release: MetricAccumulator,
    exhaust_excitation: MetricAccumulator,
    header_excitation: MetricAccumulator,
    runner_acoustic: MetricAccumulator,
    exhaust: MetricAccumulator,
    master: MetricAccumulator,
    samples: usize,
    finite: bool,
    bounded: bool,
    max_frame_abs: f32,
    firing_events: u64,
    observed_firing_order: Vec<usize>,
    firing_order_ok: bool,
    max_phase_step_error_deg: f32,
    previous_phase: Option<f32>,
}

impl ScenarioAccumulators {
    fn push(&mut self, frame: &EngineFrame, rpm: f32, sample_rate: u32) {
        self.samples += 1;
        let mut values = Vec::with_capacity(1 + 10 * 7);
        values.push(frame.combustion_source);
        values.extend(frame.cylinder_chamber_pressure_pa);
        values.extend(frame.cylinder_chamber_temperature_k);
        values.extend(frame.cylinder_chamber_heat_release_rate);
        values.extend(frame.cylinder_exhaust_excitation);
        values.extend(frame.cylinder_headers);
        values.extend(frame.cylinder_runner_acoustic_pressure);
        values.extend([frame.exhaust, frame.master_pre_limiter, frame.master]);
        for value in values {
            self.finite &= value.is_finite();
            self.max_frame_abs = self.max_frame_abs.max(value.abs());
        }
        self.bounded &= frame.cylinder_chamber_pressure_pa.iter().all(|v| v.abs() <= 25.0e6)
            && frame.cylinder_chamber_temperature_k.iter().all(|v| v.abs() <= 8_000.0)
            && frame.cylinder_mass_flow.iter().all(|v| v.abs() <= 20.0)
            && frame.cylinder_exhaust_excitation.iter().all(|v| v.abs() <= 10.0)
            && frame.master.abs() <= 2.0;

        let chamber_pressure = mean(&frame.cylinder_chamber_pressure_pa);
        let chamber_temperature = mean(&frame.cylinder_chamber_temperature_k);
        let heat_release = mean(&frame.cylinder_chamber_heat_release_rate);
        let exhaust_excitation = mean(&frame.cylinder_exhaust_excitation);
        let header_excitation = mean(&frame.cylinder_headers);
        let runner_acoustic = mean(&frame.cylinder_runner_acoustic_pressure);
        self.combustion_source.push(frame.combustion_source);
        self.chamber_pressure.push(chamber_pressure);
        self.chamber_temperature.push(chamber_temperature);
        self.heat_release.push(heat_release);
        self.exhaust_excitation.push(exhaust_excitation);
        self.header_excitation.push(header_excitation);
        self.runner_acoustic.push(runner_acoustic);
        self.exhaust.push(frame.exhaust);
        self.master.push(frame.master);

        let expected_step = rpm.max(0.0) as f64 * 6.0 / sample_rate as f64;
        if let Some(previous) = self.previous_phase {
            let observed_step = (frame.crank_phase_deg - previous).rem_euclid(720.0) as f64;
            self.max_phase_step_error_deg = self
                .max_phase_step_error_deg
                .max((observed_step - expected_step).abs() as f32);
        }
        self.previous_phase = Some(frame.crank_phase_deg);

        for cylinder in 0..10 {
            if frame.fired_mask & (1 << cylinder) != 0 {
                self.firing_order_ok &=
                    cylinder == FIRING_ORDER[(self.firing_events as usize) % FIRING_ORDER.len()];
                self.firing_events += 1;
                if self.observed_firing_order.len() < 20 {
                    self.observed_firing_order.push(cylinder);
                }
            }
        }
    }

    fn finish(self, name: &str, sample_rate: u32) -> ScenarioReport {
        let expected = (0..self.observed_firing_order.len())
            .map(|index| FIRING_ORDER[index % FIRING_ORDER.len()])
            .collect::<Vec<_>>();
        let firing_order_ok = self.firing_order_ok && self.observed_firing_order == expected;
        let excitations = serde_json::json!({
            "combustion_source": self.combustion_source.finish(self.samples),
            "chamber_pressure_pa": self.chamber_pressure.finish(self.samples),
            "chamber_temperature_k": self.chamber_temperature.finish(self.samples),
            "heat_release_rate": self.heat_release.finish(self.samples),
            "exhaust_excitation": self.exhaust_excitation.finish(self.samples),
            "runner_header": self.header_excitation.finish(self.samples),
            "runner_acoustic_pressure": self.runner_acoustic.finish(self.samples),
            "collector_exhaust": self.exhaust.finish(self.samples),
            "master": self.master.finish(self.samples),
        });
        ScenarioReport {
            name: name.to_string(),
            sample_rate,
            samples: self.samples,
            finite: self.finite,
            bounded: self.bounded,
            firing_events: self.firing_events,
            expected_firing_order: expected,
            observed_firing_order: self.observed_firing_order,
            firing_order_ok,
            max_phase_step_error_deg: self.max_phase_step_error_deg,
            max_frame_abs: self.max_frame_abs,
            excitations,
        }
    }
}

fn mean(values: &[f32; 10]) -> f32 {
    values.iter().copied().sum::<f32>() / values.len() as f32
}

fn scenario_input(name: &str, sample: usize, sample_rate: u32) -> EngineInput {
    let t = sample as f32 / sample_rate as f32;
    match name {
        "motored_low_energy" => EngineInput { rpm: 4_000.0, throttle: 0.0, load: 0.0 },
        "high_energy" => EngineInput { rpm: 12_000.0, throttle: 1.0, load: 1.0 },
        "changing_rpm_load" => {
            let phase = (t / 3.0).clamp(0.0, 1.0);
            EngineInput { rpm: 2_000.0 + phase * 13_000.0, throttle: 0.15 + phase * 0.80, load: 0.10 + phase * 0.85 }
        }
        "start_stop" => {
            let (rpm, throttle, load) = if t < 0.35 {
                (0.0, 0.0, 0.0)
            } else if t < 1.35 {
                let p = (t - 0.35) / 1.0;
                (1_200.0 + p * 8_800.0, p * 0.75, p * 0.60)
            } else if t < 2.05 {
                (10_000.0, 0.85, 0.75)
            } else if t < 2.60 {
                let p = (t - 2.05) / 0.55;
                (10_000.0 - p * 8_800.0, 0.05, 0.02)
            } else if t < 2.95 {
                (1_200.0, 0.0, 0.0)
            } else {
                let p = ((t - 2.95) / 1.05).clamp(0.0, 1.0);
                (1_200.0 + p * 10_800.0, 0.20 + p * 0.70, 0.15 + p * 0.75)
            };
            EngineInput { rpm, throttle, load }
        }
        _ => EngineInput { rpm: 9_000.0, throttle: 0.72, load: 0.66 },
    }
}

fn run_scenario(name: &str, sample_rate: u32, seconds: f32) -> ScenarioReport {
    let mut config = EngineConfig::default();
    config.sample_rate = sample_rate;
    config.seed = 0x710A_0070_u64;
    let mut engine = V10Engine::new(config).expect("valid boundary regression config");
    let samples = (seconds * sample_rate as f32) as usize;
    let mut acc = ScenarioAccumulators {
        finite: true,
        bounded: true,
        firing_order_ok: true,
        ..Default::default()
    };
    for sample in 0..samples {
        let input = scenario_input(name, sample, sample_rate);
        engine.set_input(input).expect("scenario input validates");
        let frame = engine.render_sample();
        acc.push(&frame, input.rpm, sample_rate);
    }
    acc.finish(name, sample_rate)
}

fn runtime_matrix_entry(sample_rate: u32, block_size: usize) -> RuntimeMatrixEntry {
    let mut config = EngineConfig::default();
    config.sample_rate = sample_rate;
    config.seed = 0x710A_0070_u64;
    let mut runtime = Gf509Runtime::new(Gf509RuntimeConfig {
        engine: config,
        max_block_frames: block_size,
        ..Gf509RuntimeConfig::default()
    })
    .expect("valid runtime matrix config");
    let blocks = (sample_rate as usize / block_size).max(1);
    let mut left = vec![0.0f32; block_size];
    let mut right = vec![0.0f32; block_size];
    let mut sum_sq = 0.0f64;
    let mut peak = 0.0f32;
    let mut finite = true;
    for block in 0..blocks {
        let p = block as f32 / blocks as f32;
        runtime.update_telemetry(RuntimeTelemetry {
            rpm: 1_800.0 + 10_800.0 * p,
            throttle: 0.10 + 0.80 * p,
            normalized_engine_load: 0.10 + 0.80 * p,
            normalized_engine_torque: 0.10 + 0.70 * p,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: 10_800.0,
            throttle_derivative: 0.8,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: block_size as f32 / sample_rate as f32,
        }).expect("runtime telemetry validates");
        runtime.render_block(&mut left, &mut right).expect("runtime block renders");
        for sample in &left {
            finite &= sample.is_finite();
            peak = peak.max(sample.abs());
            sum_sq += *sample as f64 * *sample as f64;
        }
    }
    let samples = blocks * block_size;
    RuntimeMatrixEntry { sample_rate, block_size, samples, finite, peak, rms: (sum_sq / samples.max(1) as f64).sqrt() as f32 }
}

fn output_path() -> PathBuf {
    std::env::args_os().nth(1).map(PathBuf::from).unwrap_or_else(|| {
        PathBuf::from("reports/audio-v10/v10-007-011/boundary-regression.json")
    })
}

fn main() {
    let output = output_path();
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent).expect("create boundary report directory");
    }
    let scenarios = [
        ("consecutive_720_cycles", 4.0f32),
        ("motored_low_energy", 2.0f32),
        ("high_energy", 2.0f32),
        ("changing_rpm_load", 3.0f32),
        ("start_stop", 4.0f32),
    ];
    let reports = scenarios.iter().map(|(name, seconds)| run_scenario(name, 48_000, *seconds)).collect::<Vec<_>>();
    let runtime_matrix = [44_100u32, 48_000u32].into_iter().flat_map(|sample_rate| {
        [128usize, 256, 512].into_iter().map(move |block_size| runtime_matrix_entry(sample_rate, block_size))
    }).collect::<Vec<_>>();
    let semantic_pass = reports.iter().all(|report| report.firing_order_ok || report.firing_events == 0);
    let numerical_pass = reports.iter().all(|report| report.finite && report.bounded && report.max_phase_step_error_deg < 1.0e-3);
    let runtime_matrix_pass = runtime_matrix.iter().all(|entry| entry.finite && entry.peak <= 1.0);
    let report = BoundaryReport {
        schema_version: "v10-007-boundary-regression-v1".into(),
        contract: "720-degree fresh-charge chamber with downstream runner excitation".into(),
        generated_from: "v10-engine-synth::V10Engine + Gf509Runtime".into(),
        scenarios: reports,
        runtime_matrix,
        semantic_pass,
        numerical_pass,
        runtime_matrix_pass,
        human_listening_gate: "PENDING_HUMAN_LISTENING".into(),
    };
    fs::write(&output, serde_json::to_vec_pretty(&report).expect("serialize boundary report"))
        .expect("write boundary report");
    println!("{}", output.display());
    println!("semantic_pass={semantic_pass}; numerical_pass={numerical_pass}; runtime_matrix_pass={runtime_matrix_pass}");
}
