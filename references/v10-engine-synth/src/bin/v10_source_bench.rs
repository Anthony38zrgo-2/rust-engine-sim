//! Repeatable V10 source-kernel timing for V10-012.
//!
//! This intentionally measures the corrected physical source through the same
//! `V10Engine::render_sample` path used by the renderer. It is a timing
//! instrument, not a production runtime path.

use std::hint::black_box;
use std::time::Instant;

use v10_engine_synth::{EngineConfig, EngineInput, SliderCrank, V10Engine};

const SAMPLE_RATE: u32 = 48_000;
const WARMUP_SAMPLES: usize = 48_000;
const MEASURE_SAMPLES: usize = 192_000;
const GEOMETRY_SAMPLES: usize = 2_000_000;

#[derive(Clone, Copy)]
struct Scenario {
    name: &'static str,
    rpm: f32,
    throttle: f32,
    load: f32,
}

const SCENARIOS: [Scenario; 4] = [
    Scenario {
        name: "steady_9000rpm",
        rpm: 9_000.0,
        throttle: 0.72,
        load: 0.78,
    },
    Scenario {
        name: "steady_18000rpm",
        rpm: 18_000.0,
        throttle: 0.88,
        load: 0.92,
    },
    Scenario {
        name: "transient_5000_to_18000",
        rpm: 5_000.0,
        throttle: 0.72,
        load: 0.78,
    },
    Scenario {
        name: "coast_18000_to_1800",
        rpm: 18_000.0,
        throttle: 0.0,
        load: 0.0,
    },
];

fn config() -> EngineConfig {
    let mut config = EngineConfig::default();
    config.sample_rate = SAMPLE_RATE;
    config.seed = 0xF090_0012;
    config
}

fn input_for(scenario: Scenario, sample: usize) -> EngineInput {
    let progress = sample as f32 / MEASURE_SAMPLES as f32;
    let rpm = match scenario.name {
        "transient_5000_to_18000" => 5_000.0 + 13_000.0 * progress,
        "coast_18000_to_1800" => 18_000.0 - 16_200.0 * progress,
        _ => scenario.rpm,
    };
    EngineInput {
        rpm,
        throttle: scenario.throttle,
        load: scenario.load,
    }
}

fn run(scenario: Scenario) -> (f64, u64, bool) {
    let mut engine = V10Engine::new(config()).expect("valid benchmark config");
    engine
        .set_input(input_for(scenario, 0))
        .expect("valid benchmark input");
    for sample in 0..WARMUP_SAMPLES {
        engine
            .set_input(input_for(scenario, sample))
            .expect("valid benchmark input");
        black_box(engine.render_sample());
    }

    let start = Instant::now();
    let mut checksum = 0u64;
    let mut finite = true;
    for sample in 0..MEASURE_SAMPLES {
        engine
            .set_input(input_for(scenario, sample))
            .expect("valid benchmark input");
        let frame = black_box(engine.render_sample());
        finite &= frame.master.is_finite()
            && frame.combustion_source.is_finite()
            && frame
                .cylinder_pressure
                .iter()
                .all(|value| value.is_finite());
        checksum = checksum
            .wrapping_add(u64::from(frame.master.to_bits()))
            .rotate_left(7);
    }
    let elapsed = start.elapsed();
    (
        elapsed.as_secs_f64() * 1_000_000_000.0 / MEASURE_SAMPLES as f64,
        checksum,
        finite,
    )
}

fn run_geometry() -> (f64, u64, bool) {
    let geometry = SliderCrank::new(0.096, 0.042, 0.135, 12.5);
    let start = Instant::now();
    let mut checksum = 0u64;
    let mut finite = true;
    for sample in 0..GEOMETRY_SAMPLES {
        let angle = (sample as f32 * 0.137).rem_euclid(720.0);
        let volume = black_box(geometry.instantaneous_volume_m3(angle));
        finite &= volume.is_finite() && volume > 0.0;
        checksum = checksum
            .wrapping_add(u64::from(volume.to_bits()))
            .rotate_left(5);
    }
    let elapsed = start.elapsed();
    (
        elapsed.as_secs_f64() * 1_000_000_000.0 / GEOMETRY_SAMPLES as f64,
        checksum,
        finite,
    )
}

fn main() {
    println!("{{");
    println!("  \"sample_rate\": {SAMPLE_RATE},");
    println!("  \"warmup_samples\": {WARMUP_SAMPLES},");
    println!("  \"measure_samples\": {MEASURE_SAMPLES},");
    println!("  \"geometry_samples\": {GEOMETRY_SAMPLES},");
    let (geometry_ns_per_call, geometry_checksum, geometry_finite) = run_geometry();
    println!(
        "  \"geometry\": {{\"ns_per_call\":{:.3},\"checksum\":{},\"finite\":{}}},",
        geometry_ns_per_call, geometry_checksum, geometry_finite
    );
    println!("  \"scenarios\": [");
    for (index, scenario) in SCENARIOS.iter().copied().enumerate() {
        let (ns_per_sample, checksum, finite) = run(scenario);
        let comma = if index + 1 == SCENARIOS.len() {
            ""
        } else {
            ","
        };
        println!(
            "    {{\"name\":\"{}\",\"ns_per_sample\":{:.3},\"checksum\":{},\"finite\":{}}}{}",
            scenario.name, ns_per_sample, checksum, finite, comma
        );
    }
    println!("  ]");
    println!("}}");
}
