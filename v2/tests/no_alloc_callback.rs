use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;

struct CountingAllocator;

struct ThreadAllocState {
    active: bool,
    allocs: usize,
    reallocs: usize,
    deallocs: usize,
    bytes: usize,
}

impl ThreadAllocState {
    const fn new() -> Self {
        Self {
            active: false,
            allocs: 0,
            reallocs: 0,
            deallocs: 0,
            bytes: 0,
        }
    }
}

thread_local! {
    static TLS_STATE: Cell<ThreadAllocState> = const { Cell::new(ThreadAllocState::new()) };
}

fn record_alloc(size: usize) {
    let _ = TLS_STATE.try_with(|cell| {
        let mut st = cell.replace(ThreadAllocState::new());
        if st.active {
            st.allocs += 1;
            st.bytes += size;
        }
        cell.set(st);
    });
}

fn record_dealloc() {
    let _ = TLS_STATE.try_with(|cell| {
        let mut st = cell.replace(ThreadAllocState::new());
        if st.active {
            st.deallocs += 1;
        }
        cell.set(st);
    });
}

fn record_realloc(new_size: usize) {
    let _ = TLS_STATE.try_with(|cell| {
        let mut st = cell.replace(ThreadAllocState::new());
        if st.active {
            st.reallocs += 1;
            st.bytes += new_size;
        }
        cell.set(st);
    });
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record_alloc(layout.size());
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        record_dealloc();
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        record_realloc(new_size);
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: CountingAllocator = CountingAllocator;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct AllocCounts {
    allocs: usize,
    reallocs: usize,
    deallocs: usize,
    bytes: usize,
}

fn measure_allocs<F: FnOnce() -> R, R>(f: F) -> (R, AllocCounts) {
    TLS_STATE.with(|cell| {
        let st = cell.replace(ThreadAllocState::new());
        assert!(!st.active, "nested measure_allocs not supported");
        cell.set(ThreadAllocState {
            active: true,
            allocs: 0,
            reallocs: 0,
            deallocs: 0,
            bytes: 0,
        });
    });

    struct ResetGuard;
    impl Drop for ResetGuard {
        fn drop(&mut self) {
            TLS_STATE.with(|cell| cell.set(ThreadAllocState::new()));
        }
    }
    let _reset = ResetGuard;

    let result = f();

    let delta = TLS_STATE.with(|cell| {
        let st = cell.replace(ThreadAllocState::new());
        AllocCounts {
            allocs: st.allocs,
            reallocs: st.reallocs,
            deallocs: st.deallocs,
            bytes: st.bytes,
        }
    });
    (result, delta)
}

#[test]
fn counting_allocator_allows_preallocated_hot_loop_with_zero_allocs() {
    let mut left = [0.0f32; 512];
    let mut right = [0.0f32; 512];
    // Warm up TLS slot on this thread before measurement.
    let (_, _) = measure_allocs(|| {});

    let (sum, counts) = measure_allocs(|| {
        let mut acc = 0.0f32;
        for block in 0..32 {
            for i in 0..left.len() {
                let sample = ((block * 512 + i) as f32 * 0.001).sin() * 0.25;
                left[i] = sample;
                right[i] = sample;
                acc += sample;
            }
            black_box(&left);
            black_box(&right);
        }
        acc
    });

    assert!(sum.is_finite());
    assert_eq!(counts.allocs, 0, "unexpected heap allocs: {counts:?}");
    assert_eq!(counts.reallocs, 0, "unexpected heap reallocs: {counts:?}");
    assert_eq!(counts.deallocs, 0, "unexpected heap deallocs: {counts:?}");
    assert_eq!(counts.bytes, 0, "unexpected allocated bytes: {counts:?}");
}

#[test]
fn counting_allocator_detects_heap_allocations_inside_guarded_scope() {
    let (len, counts) = measure_allocs(|| {
        let mut buf: Vec<f32> = Vec::with_capacity(64);
        buf.push(1.0);
        buf.resize(256, 0.5);
        black_box(&buf).len()
    });
    assert_eq!(len, 256);
    assert!(
        counts.allocs >= 1 && counts.bytes >= 64 * std::mem::size_of::<f32>(),
        "allocator instrumentation failed to catch heap activity: {counts:?}"
    );
}

#[test]
fn v2_samples_playback_performs_zero_heap_allocations_schema1_and_schema2() {
    use std::path::PathBuf;
    use std::sync::Arc;
    use v10_engine_synth::{
        PreparedSampleBank, SampleBankPlayer, SampleLayerInput, ThreeZoneSampleLayerConfig,
    };

    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let configs = [
        ("tests/fixtures/bank_schema1", 48_000u32),
        ("tests/fixtures/bank_schema1", 44_100u32),
        ("tests/fixtures/bank_schema2", 48_000u32),
    ];

    // Warm up TLS slot on this thread before measurement.
    let (_, _) = measure_allocs(|| {});

    for (bank_rel, output_sample_rate) in configs {
        let bank = Arc::new(
            PreparedSampleBank::load_directory(
                output_sample_rate,
                &root.join(bank_rel),
                ThreeZoneSampleLayerConfig::default(),
            )
            .unwrap_or_else(|e| panic!("failed preparing {bank_rel}@{output_sample_rate}: {e}")),
        );
        let mut player = SampleBankPlayer::new(Arc::clone(&bank));
        let mut frame = player.allocate_frame();
        let total_frames = (output_sample_rate as usize) / 2;

        let ((acc, last_zone_sum), counts) = measure_allocs(|| {
            let mut sum = 0.0f32;
            let mut phase_deg = 0.0f32;
            for i in 0..total_frames {
                let u = i as f32 / (total_frames - 1) as f32;
                let (rpm, throttle, load, torque) = if u < 0.65 {
                    let k = u / 0.65;
                    (3_800.0 + 11_700.0 * k, 0.92, 0.88, 0.86)
                } else {
                    let k = (u - 0.65) / 0.35;
                    (15_500.0 - 9_500.0 * k, 0.02, 0.12, -0.52)
                };
                let crank_step = rpm * 6.0 / output_sample_rate as f32;
                phase_deg = (phase_deg + crank_step).rem_euclid(720.0);
                player
                    .process_into(
                        SampleLayerInput {
                            rpm,
                            throttle,
                            load,
                            normalized_engine_torque: torque,
                            clutch_engagement: 1.0,
                            crank_phase_deg: phase_deg,
                        },
                        &mut frame,
                    )
                    .expect("process_into succeeds");
                sum += frame.output;
                black_box(frame.output);
                black_box(&frame.zone_weights);
            }
            let zsum: f32 = frame.zone_weights.iter().copied().sum();
            (sum, zsum)
        });

        assert!(acc.is_finite() && last_zone_sum.is_finite());
        assert_eq!(
            counts.allocs, 0,
            "{bank_rel}@{output_sample_rate} unexpected heap allocs: {counts:?}"
        );
        assert_eq!(
            counts.reallocs, 0,
            "{bank_rel}@{output_sample_rate} unexpected heap reallocs: {counts:?}"
        );
        assert_eq!(
            counts.deallocs, 0,
            "{bank_rel}@{output_sample_rate} unexpected heap deallocs: {counts:?}"
        );
        assert_eq!(
            counts.bytes, 0,
            "{bank_rel}@{output_sample_rate} unexpected allocated bytes: {counts:?}"
        );
    }
}

#[test]
fn v2_procedural_engine_scene_and_drivetrain_perform_zero_heap_allocations() {
    use std::path::PathBuf;
    use v10_engine_synth::{
        AcousticScene, AcousticSceneConfig, EngineConfig, EngineInput, GearShiftPlayer,
        GearShiftSamples, ShiftPhase, TransmissionConfig, TransmissionInput, TransmissionSynth,
        V10Engine,
    };

    let sample_rate = 48_000u32;
    let mut engine = V10Engine::new(EngineConfig {
        sample_rate,
        ..EngineConfig::default()
    })
    .expect("engine init");
    let mut scene =
        AcousticScene::new(sample_rate as f32, AcousticSceneConfig::default()).expect("scene init");
    let mut tx = TransmissionSynth::new(TransmissionConfig {
        sample_rate,
        output_gain: 0.25,
        gear_ratios: vec![3.10, 2.35, 1.85, 1.50, 1.25, 1.06],
        ..TransmissionConfig::default()
    })
    .expect("transmission init");
    let shift_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/bank_schema1");
    let shift_samples =
        GearShiftSamples::load_directory(&shift_dir, sample_rate).expect("shift samples load");
    let mut shift_player =
        GearShiftPlayer::new(shift_samples, 0.8, 10_000.0, sample_rate).expect("shift player init");

    // Warm up TLS slot on this thread before measurement.
    let (_, _) = measure_allocs(|| {});

    let (sum, counts) = measure_allocs(|| {
        let mut acc = 0.0f32;
        for sample in 0..24_000usize {
            let phase = match sample {
                4_000..=5_999 => ShiftPhase::UpshiftCut,
                6_000..=7_999 => ShiftPhase::UpshiftRecovery,
                14_000..=15_999 => ShiftPhase::DownshiftCut,
                16_000..=17_999 => ShiftPhase::DownshiftBlip,
                _ => ShiftPhase::None,
            };
            let rpm = 5_000.0 + (sample as f32 / 24_000.0) * 9_000.0;
            engine
                .set_input(EngineInput {
                    rpm,
                    throttle: 0.85,
                    load: 0.80,
                })
                .expect("set_input");
            engine.set_mechanical_state(0.75, 1.0, 0.0, false, phase);
            let mut engine_frame = engine.render_sample();
            engine_frame.transmission = tx.process(TransmissionInput {
                rpm,
                gear: 3,
                clutch: 1.0,
                torque: 0.75,
                throttle: 0.85,
                shift_phase: phase,
            });
            engine_frame.gear_shift = shift_player.process(phase, rpm);
            let acoustic = scene.process(&engine_frame);
            acc += acoustic.output;
            black_box(acoustic.output);
        }
        acc
    });

    assert!(sum.is_finite());
    assert_eq!(counts.allocs, 0, "unexpected heap allocs: {counts:?}");
    assert_eq!(counts.reallocs, 0, "unexpected heap reallocs: {counts:?}");
    assert_eq!(counts.deallocs, 0, "unexpected heap deallocs: {counts:?}");
    assert_eq!(counts.bytes, 0, "unexpected allocated bytes: {counts:?}");
}

#[test]
fn v2_audio_callback_render_block_performs_zero_heap_allocations() {
    use std::path::PathBuf;
    use v10_engine_synth::{
        EngineMode, Gf509Runtime, Gf509RuntimeConfig, RuntimeTelemetry, ShiftPhase,
        SpectrumChainConfig, TorqueSign, TransmissionConfig,
    };

    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let sample_rate = 48_000u32;
    let block_frames = 400usize;
    let dt = block_frames as f32 / sample_rate as f32;

    let cases = [
        (
            "hybrid_schema1",
            EngineMode::Hybrid,
            "tests/fixtures/bank_schema1",
        ),
        (
            "hybrid_schema2",
            EngineMode::Hybrid,
            "tests/fixtures/bank_schema2",
        ),
        (
            "sample_only_schema1",
            EngineMode::SampleOnly,
            "tests/fixtures/bank_schema1",
        ),
        (
            "sample_only_schema2",
            EngineMode::SampleOnly,
            "tests/fixtures/bank_schema2",
        ),
    ];

    // Warm up TLS slot on this thread before measurement.
    let (_, _) = measure_allocs(|| {});

    for (label, engine_mode, bank_rel) in cases {
        let mut config = Gf509RuntimeConfig {
            sample_layer_directory: Some(root.join(bank_rel)),
            engine_mode,
            max_block_frames: block_frames,
            upper_mid_shelf_gain: 0.35,
            transmission: TransmissionConfig {
                sample_rate,
                output_gain: 0.22,
                gear_ratios: vec![3.10, 2.35, 1.85, 1.50, 1.25, 1.06],
                ..TransmissionConfig::default()
            },
            v10_filter: SpectrumChainConfig {
                highpass_hz: 45.0,
                highpass_slope_db_per_oct: 12.0,
                peak_hz: 1_800.0,
                peak_gain_db: 2.0,
                peak_q: 1.1,
                peak_drive: 1.5,
                peak_drive_mix: 0.3,
                ..SpectrumChainConfig::default()
            },
            gearbox_filter: SpectrumChainConfig {
                lowpass_hz: 9_500.0,
                lowpass_slope_db_per_oct: 12.0,
                ..SpectrumChainConfig::default()
            },
            ..Gf509RuntimeConfig::default()
        };
        config.engine.sample_rate = sample_rate;
        let mut runtime =
            Gf509Runtime::new(config).unwrap_or_else(|e| panic!("{label} init failed: {e}"));

        let mut left = vec![0.0f32; block_frames];
        let mut right = vec![0.0f32; block_frames];

        let (acc, counts) = measure_allocs(|| {
            let mut sum = 0.0f32;
            for step in 0..80usize {
                let (rpm, throttle, load, torque, sign, gear, phase, clutch, tc_cut, limiter) =
                    match step {
                        0..=24 => (
                            5_000.0 + step as f32 * 450.0,
                            0.95,
                            0.90,
                            0.85,
                            TorqueSign::Positive,
                            3,
                            ShiftPhase::None,
                            1.0,
                            0.0,
                            false,
                        ),
                        25..=32 => (
                            15_800.0,
                            0.95,
                            0.15,
                            -0.10,
                            TorqueSign::Negative,
                            3,
                            ShiftPhase::UpshiftCut,
                            0.35,
                            0.25,
                            true,
                        ),
                        33..=44 => (
                            13_200.0,
                            0.95,
                            0.85,
                            0.80,
                            TorqueSign::Positive,
                            4,
                            ShiftPhase::UpshiftRecovery,
                            0.90,
                            0.0,
                            false,
                        ),
                        45..=52 => (
                            9_500.0,
                            0.0,
                            0.08,
                            -0.45,
                            TorqueSign::Negative,
                            4,
                            ShiftPhase::DownshiftCut,
                            0.20,
                            0.0,
                            false,
                        ),
                        53..=62 => (
                            12_600.0,
                            0.55,
                            0.30,
                            0.15,
                            TorqueSign::Positive,
                            4,
                            ShiftPhase::DownshiftBlip,
                            0.25,
                            0.0,
                            false,
                        ),
                        _ => (
                            11_800.0 - (step - 63) as f32 * 220.0,
                            0.08,
                            0.12,
                            -0.38,
                            TorqueSign::Negative,
                            3,
                            ShiftPhase::DownshiftRecovery,
                            0.95,
                            0.0,
                            false,
                        ),
                    };
                runtime
                    .update_telemetry(RuntimeTelemetry {
                        rpm,
                        throttle,
                        normalized_engine_load: load,
                        normalized_engine_torque: torque,
                        torque_sign: sign,
                        rpm_derivative: 0.0,
                        throttle_derivative: 0.0,
                        gear,
                        shift_phase: phase,
                        clutch_engagement: clutch,
                        tc_cut_ratio: tc_cut,
                        rev_limiter_active: limiter,
                        dt_seconds: dt,
                    })
                    .expect("update_telemetry");
                let n = runtime
                    .render_block(&mut left, &mut right)
                    .expect("render_block");
                assert_eq!(n, block_frames);
                sum += black_box(&left).iter().copied().sum::<f32>();
                black_box(&right);
            }
            sum
        });

        assert!(acc.is_finite(), "{label} rendered non-finite output");
        assert_eq!(counts.allocs, 0, "{label} unexpected allocs: {counts:?}");
        assert_eq!(
            counts.reallocs, 0,
            "{label} unexpected reallocs: {counts:?}"
        );
        assert_eq!(
            counts.deallocs, 0,
            "{label} unexpected deallocs: {counts:?}"
        );
        assert_eq!(counts.bytes, 0, "{label} unexpected bytes: {counts:?}");
    }
}

#[test]
fn ford_pinto_rs2000_callback_performs_zero_heap_allocations() {
    use v10_engine_synth::{
        Gf509Runtime, Gf509RuntimeConfig, RuntimeTelemetry, ShiftPhase, TorqueSign,
    };

    let sample_rate = 48_000u32;
    let block_frames = 400usize;
    let dt = block_frames as f32 / sample_rate as f32;
    let mut config = Gf509RuntimeConfig::ford_pinto_rs2000(sample_rate);
    config.max_block_frames = block_frames;
    config.transmission.output_gain = 0.12;
    let mut runtime = Gf509Runtime::new(config).expect("runtime init");
    let mut left = vec![0.0f32; block_frames];
    let mut right = vec![0.0f32; block_frames];

    let (_, _) = measure_allocs(|| {});
    let (acc, counts) = measure_allocs(|| {
        let mut sum = 0.0f32;
        for step in 0..90 {
            let (rpm, throttle, load) = if step < 50 {
                (1_500.0 + step as f32 * 114.0, 0.92, 0.90)
            } else {
                (7_200.0 - (step - 50) as f32 * 140.0, 0.0, 0.0)
            };
            runtime
                .update_telemetry(RuntimeTelemetry {
                    rpm,
                    throttle,
                    normalized_engine_load: load,
                    normalized_engine_torque: if throttle > 0.1 { 0.85 } else { -0.25 },
                    torque_sign: if throttle > 0.1 {
                        TorqueSign::Positive
                    } else {
                        TorqueSign::Negative
                    },
                    rpm_derivative: 0.0,
                    throttle_derivative: 0.0,
                    gear: 3,
                    shift_phase: ShiftPhase::None,
                    clutch_engagement: 1.0,
                    tc_cut_ratio: 0.0,
                    rev_limiter_active: false,
                    dt_seconds: dt,
                })
                .expect("update_telemetry");
            let n = runtime
                .render_block(&mut left, &mut right)
                .expect("render_block");
            assert_eq!(n, block_frames);
            sum += black_box(&left).iter().copied().sum::<f32>();
            black_box(&right);
        }
        sum
    });

    assert!(acc.is_finite());
    assert_eq!(counts.allocs, 0, "unexpected allocs: {counts:?}");
    assert_eq!(counts.reallocs, 0, "unexpected reallocs: {counts:?}");
    assert_eq!(counts.deallocs, 0, "unexpected deallocs: {counts:?}");
    assert_eq!(counts.bytes, 0, "unexpected bytes: {counts:?}");
}
