//! DSP filters ported from engine-sim's synthesizer chain:
//! jitter → DC-block → derivative(hf) → air-noise mix → convolution IR
//! → antialias → leveler → volume → i16.

pub mod modal;
pub mod scene;
pub mod waveguide;

use std::f32::consts::PI as PI_F32;
use std::f64::consts::PI as PI_F64;

// ---------------------------------------------------------------------------
// Audio parameters (mirrors Synthesizer::AudioParameters)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct AudioParameters {
    pub volume: f32,
    pub convolution: f32,
    pub hf_mix: f32,
    pub hf_reference_hz: f32,
    pub input_sample_noise: f32,
    pub input_sample_noise_frequency_cutoff: f32,
    pub flow_noise: f32,
    pub flow_noise_frequency_cutoff: f32,
    pub mechanical_noise: f32,
    pub mechanical_noise_frequency_cutoff: f32,
    /// Input antialias cutoff, bounded to 45% of the simulation rate.
    pub input_antialias_frequency_cutoff: f32,
    pub leveler_enabled: bool,
    pub leveler_target: f32,
    pub leveler_max_gain: f32,
    pub leveler_min_gain: f32,
    pub leveler_attack: f32,
    pub leveler_release: f32,
}

impl Default for AudioParameters {
    fn default() -> Self {
        Self {
            volume: 1.0,
            convolution: 1.0,
            hf_mix: 0.01,
            hf_reference_hz: 1_000.0,
            input_sample_noise: 0.5,
            input_sample_noise_frequency_cutoff: 10_000.0,
            flow_noise: 1.0,
            flow_noise_frequency_cutoff: 2_000.0,
            mechanical_noise: 0.0,
            mechanical_noise_frequency_cutoff: 8_000.0,
            input_antialias_frequency_cutoff: 1_900.0,
            leveler_enabled: true,
            leveler_target: 30_000.0,
            leveler_max_gain: 1.9,
            leveler_min_gain: 1e-5,
            leveler_attack: 0.001,
            leveler_release: 0.05,
        }
    }
}

// ---------------------------------------------------------------------------
// Simple one-pole low-pass (RC)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default)]
pub struct LowPassFilter {
    y: f32,
    rc: f32,
    dt: f32,
}

impl LowPassFilter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_cutoff_frequency(&mut self, f: f32) {
        self.rc = 1.0 / (f * 2.0 * PI_F32);
    }

    pub fn set_dt(&mut self, dt: f32) {
        self.dt = dt;
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        let alpha = self.dt / (self.rc + self.dt);
        self.y = alpha * sample + (1.0 - alpha) * self.y;
        self.y
    }
}

// ---------------------------------------------------------------------------
// 4th-order Butterworth low-pass (matches C++ template, f32 specialization)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct ButterworthLowPassFilter {
    y: [f64; 4],
    x: [f64; 4],
    a: [f64; 5],
    f_4: f64,
    /// Index of oldest sample (next write position in C++ ring after removeBeginning).
    head: usize,
}

impl Default for ButterworthLowPassFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl ButterworthLowPassFilter {
    pub fn new() -> Self {
        Self {
            y: [0.0; 4],
            x: [0.0; 4],
            a: [0.0; 5],
            f_4: 0.0,
            head: 0,
        }
    }

    pub fn set_cutoff_frequency(&mut self, f_c: f32, sample_rate: f32) {
        // Coefficients computed in f64: at very low normalized cutoffs the
        // pole cluster sits within ~1e-3 of the unit circle and f32 rounding
        // of the recursion coefficients can push a pole outside it (the f32
        // recursion then diverges to ±Inf).
        let f_c = f_c as f64;
        let sample_rate = sample_rate as f64;
        let f = (PI_F64 * f_c / sample_rate).tan();
        let f_2 = f * f;
        let f_3 = f_2 * f;
        let f_4 = f_2 * f_2;
        let m = -2.0 * (5.0 * PI_F64 / 8.0).cos();
        let n = -2.0 * (7.0 * PI_F64 / 8.0).cos();

        let a0 = 1.0 + (m + n) * f + (2.0 + n * m) * f_2 + (m + n) * f_3 + f_4;
        self.a[0] = a0;
        self.a[1] = (-4.0 - 2.0 * (n + m) * f + 2.0 * (m + n) * f_3 + 4.0 * f_4) / a0;
        self.a[2] = (6.0 - 2.0 * (2.0 + m * n) * f_2 + 6.0 * f_4) / a0;
        self.a[3] = (-4.0 + 2.0 * (m + n) * f - 2.0 * (m + n) * f_3 + 4.0 * f_4) / a0;
        self.a[4] = (1.0 - (n + m) * f + (2.0 + m * n) * f_2 - (m + n) * f_3 + f_4) / a0;
        self.f_4 = f_4;
    }

    /// Most recent at index `head-1` (mod 4); matches C++ read(0)=newest.
    fn y_at(&self, ago: usize) -> f64 {
        // ago=0 → newest
        self.y[(self.head + 4 - 1 - ago) % 4]
    }

    fn x_at(&self, ago: usize) -> f64 {
        self.x[(self.head + 4 - 1 - ago) % 4]
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        // C++: n = m_f_4 / m_a[0] * (...), a[1..4] already normalized by a0.
        // The recursion runs in f64 so pole clusters near z=1 (very low
        // normalized cutoffs) stay inside the unit circle.
        let sample = sample as f64;
        let n = (self.f_4 / self.a[0])
            * (sample
                + 4.0 * self.x_at(0)
                + 6.0 * self.x_at(1)
                + 4.0 * self.x_at(2)
                + self.x_at(3));
        let d = -self.a[1] * self.y_at(0)
            - self.a[2] * self.y_at(1)
            - self.a[3] * self.y_at(2)
            - self.a[4] * self.y_at(3);
        let y = n + d;

        self.x[self.head] = sample;
        self.y[self.head] = y;
        self.head = (self.head + 1) % 4;
        y as f32
    }
}

// ---------------------------------------------------------------------------
// Peaking biquad (collector / outlet resonances)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    pub fn peaking(frequency_hz: f32, q: f32, gain_db: f32, sample_rate: f32) -> Self {
        let sr = sample_rate.max(1.0);
        let f = frequency_hz.clamp(1.0, 0.49 * sr);
        let q = q.max(0.01);
        let amp = 10f32.powf(gain_db / 40.0);
        let w0 = 2.0 * PI_F32 * f / sr;
        let alpha = w0.sin() / (2.0 * q);
        let cos_w0 = w0.cos();
        let a0 = 1.0 + alpha / amp;
        Self {
            b0: (1.0 + alpha * amp) / a0,
            b1: (-2.0 * cos_w0) / a0,
            b2: (1.0 - alpha * amp) / a0,
            a1: (-2.0 * cos_w0) / a0,
            a2: (1.0 - alpha / amp) / a0,
            ..Default::default()
        }
    }

    pub fn high_shelf(frequency_hz: f32, q: f32, gain_db: f32, sample_rate: f32) -> Self {
        let sr = sample_rate.max(1.0);
        let f = frequency_hz.clamp(1.0, 0.49 * sr);
        let q = q.max(0.01);
        let amp = 10f32.powf(gain_db / 40.0);
        let w0 = 2.0 * PI_F32 * f / sr;
        let alpha = w0.sin() / (2.0 * q);
        let cos_w0 = w0.cos();
        let beta = 2.0 * amp.sqrt() * alpha;
        let a0 = (amp + 1.0) - (amp - 1.0) * cos_w0 + beta;
        Self {
            b0: amp * ((amp + 1.0) + (amp - 1.0) * cos_w0 + beta) / a0,
            b1: -2.0 * amp * ((amp - 1.0) + (amp + 1.0) * cos_w0) / a0,
            b2: amp * ((amp + 1.0) + (amp - 1.0) * cos_w0 - beta) / a0,
            a1: 2.0 * ((amp - 1.0) - (amp + 1.0) * cos_w0) / a0,
            a2: ((amp + 1.0) - (amp - 1.0) * cos_w0 - beta) / a0,
            ..Default::default()
        }
    }

    pub fn highpass(frequency_hz: f32, q: f32, sample_rate: f32) -> Self {
        let sr = sample_rate.max(1.0);
        let f = frequency_hz.clamp(1.0, 0.49 * sr);
        let q = q.max(0.01);
        let w0 = 2.0 * PI_F32 * f / sr;
        let alpha = w0.sin() / (2.0 * q);
        let cos_w0 = w0.cos();
        let a0 = 1.0 + alpha;
        Self {
            b0: (1.0 + cos_w0) / 2.0 / a0,
            b1: -(1.0 + cos_w0) / a0,
            b2: (1.0 + cos_w0) / 2.0 / a0,
            a1: (-2.0 * cos_w0) / a0,
            a2: (1.0 - alpha) / a0,
            ..Default::default()
        }
    }

    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = if y.abs() < 1.0e-20 { 0.0 } else { y };
        y
    }
}

// ---------------------------------------------------------------------------
// Derivative filter
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default)]
pub struct DerivativeFilter {
    pub dt: f32,
    previous: f32,
}

impl DerivativeFilter {
    pub fn new(dt: f32) -> Self {
        Self { dt, previous: 0.0 }
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        let d = (sample - self.previous) / self.dt;
        self.previous = sample;
        d
    }
}

// ---------------------------------------------------------------------------
// Convolution filter
//
// Short impulse responses (<= DIRECT_CONV_THRESHOLD taps) use the original
// direct circular-buffer FIR. Longer ones switch to blockwise overlap-save
// FFT convolution (O(n log n) instead of O(n) per sample).
//
// The FFT path buffers `block` input samples and emits the corresponding
// `block` outputs once the block completes, so the wet signal is delayed by
// `latency()` samples. Use `latency()` to delay the dry path identically
// before mixing (see `Synthesizer::render_sample`).
// ---------------------------------------------------------------------------

use rustfft::num_complex::Complex;
use rustfft::{Fft, FftPlanner};
use std::collections::VecDeque;
use std::sync::Arc;

const DIRECT_CONV_THRESHOLD: usize = 128;
const FFT_BLOCK_SIZE: usize = 4096;

#[derive(Clone, Default)]
pub struct ConvolutionFilter {
    ir_raw: Vec<f32>,
    // direct path
    shift_register: Vec<f32>,
    shift_offset: usize,
    // fft path
    fft: Option<Arc<dyn Fft<f32>>>,
    ifft: Option<Arc<dyn Fft<f32>>>,
    ir_spectrum: Vec<Complex<f32>>,
    ir_spectrum_len: usize,
    history: Vec<f32>,
    pending: Vec<f32>,
    out_queue: VecDeque<f32>,
    block: usize,
    n_fft: usize,
}

impl ConvolutionFilter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn initialize(&mut self, samples: usize) {
        self.ir_raw = vec![0.0; samples];
        self.ir_spectrum_len = 0;
        self.shift_register = Vec::new();
        self.shift_offset = 0;
        self.out_queue.clear();
        self.pending.clear();
        if samples > DIRECT_CONV_THRESHOLD {
            let block = FFT_BLOCK_SIZE;
            let n_fft = (samples + block - 1).next_power_of_two();
            let mut planner = FftPlanner::<f32>::new();
            self.fft = Some(planner.plan_fft_forward(n_fft));
            self.ifft = Some(planner.plan_fft_inverse(n_fft));
            self.block = block;
            self.n_fft = n_fft;
            self.history = vec![0.0; n_fft - block];
        } else {
            self.fft = None;
            self.ifft = None;
            self.shift_register = vec![0.0; samples];
        }
    }

    pub fn impulse_response_mut(&mut self) -> &mut [f32] {
        &mut self.ir_raw
    }

    pub fn sample_count(&self) -> usize {
        self.ir_raw.len()
    }

    /// Sample delay introduced by the filter (FFT path only; 0 for the
    /// direct path and for passthrough with an empty IR).
    pub fn latency(&self) -> usize {
        if self.ir_raw.len() > DIRECT_CONV_THRESHOLD {
            self.block.saturating_sub(1)
        } else {
            0
        }
    }

    /// (Re)build the IR spectrum from the raw taps when they changed.
    fn ensure_ir_spectrum(&mut self) {
        let n_fft = self.n_fft;
        if n_fft == 0 || self.ir_spectrum_len == n_fft {
            return;
        }
        let mut buf: Vec<Complex<f32>> =
            self.ir_raw.iter().map(|&t| Complex::new(t, 0.0)).collect();
        buf.resize(n_fft, Complex::new(0.0, 0.0));
        self.fft.as_ref().unwrap().process(&mut buf);
        self.ir_spectrum = buf;
        self.ir_spectrum_len = n_fft;
    }

    fn run_block(&mut self) {
        let block = self.block;
        let n_fft = self.n_fft;
        let keep = n_fft - block;
        self.ensure_ir_spectrum();

        // Window = previous context ++ pending samples.
        let mut input = vec![0.0f32; n_fft];
        input[..keep].copy_from_slice(&self.history);
        input[keep..].copy_from_slice(&self.pending);
        // Retain the last `keep` samples as context for the next block.
        self.history.copy_from_slice(&input[block..]);
        self.pending.clear();

        let mut buf: Vec<Complex<f32>> = input.into_iter().map(|v| Complex::new(v, 0.0)).collect();
        self.fft.as_ref().unwrap().process(&mut buf);
        for (s, ir) in buf.iter_mut().zip(self.ir_spectrum.iter()) {
            *s *= *ir;
        }
        self.ifft.as_ref().unwrap().process(&mut buf);
        let inv = 1.0 / n_fft as f32;
        self.out_queue
            .extend(buf[n_fft - block..].iter().map(|s| s.re * inv));
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        let n = self.ir_raw.len();
        if n == 0 {
            return sample;
        }
        if n <= DIRECT_CONV_THRESHOLD {
            self.shift_register[self.shift_offset] = sample;

            let mut result = 0.0f32;
            for i in 0..(n - self.shift_offset) {
                result += self.ir_raw[i] * self.shift_register[i + self.shift_offset];
            }
            for i in (n - self.shift_offset)..n {
                result += self.ir_raw[i] * self.shift_register[i - (n - self.shift_offset)];
            }

            self.shift_offset = (self.shift_offset + n - 1) % n;
            result
        } else {
            self.pending.push(sample);
            if self.pending.len() >= self.block {
                self.run_block();
            }
            self.out_queue.pop_front().unwrap_or(0.0)
        }
    }
}

// ---------------------------------------------------------------------------
// Jitter filter (subsample delay with noise)
// ---------------------------------------------------------------------------

pub struct JitterFilter {
    noise_filter: ButterworthLowPassFilter,
    jitter_scale: f32,
    max_jitter: usize,
    offset: usize,
    history: Vec<f32>,
    rng_state: u64,
}

impl JitterFilter {
    pub fn new(max_jitter: usize, noise_cutoff: f32, audio_rate: f32) -> Self {
        let mut noise_filter = ButterworthLowPassFilter::new();
        noise_filter.set_cutoff_frequency(noise_cutoff, audio_rate);
        let max_jitter = max_jitter.max(2);
        Self {
            noise_filter,
            jitter_scale: 1.0,
            max_jitter,
            offset: 0,
            history: vec![0.0; max_jitter],
            rng_state: 0x1234_5678_9abc_def0,
        }
    }

    pub fn set_jitter_scale(&mut self, s: f32) {
        self.jitter_scale = s;
    }

    pub fn next_random(&mut self) -> f32 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        (x as f64 / u64::MAX as f64) as f32
    }

    pub fn process(&mut self, sample: f32, jitter_scale: f32) -> f32 {
        self.history[self.offset] = sample;
        self.offset += 1;
        if self.offset >= self.max_jitter {
            self.offset = 0;
        }

        let dist = self.next_random() * (self.max_jitter - 1) as f32;
        let s = self
            .noise_filter
            .process(dist * self.jitter_scale * jitter_scale);
        let s_i_0 = s.floor().clamp(0.0, (self.max_jitter - 1) as f32);
        let s_i_1 = s.ceil().clamp(0.0, (self.max_jitter - 1) as f32);
        let s_frac = s - s_i_0;

        let i_0 = s_i_0 as usize + self.offset;
        let i_1 = s_i_1 as usize + self.offset;
        let v0 = self.history[i_0 % self.max_jitter];
        let v1 = self.history[i_1 % self.max_jitter];

        v1 * s_frac + v0 * (1.0 - s_frac)
    }
}

// ---------------------------------------------------------------------------
// Leveling filter (adaptive gain to target peak)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct LevelingFilter {
    peak: f32,
    attenuation: f32,
    pub enabled: bool,
    pub target: f32,
    pub max_level: f32,
    pub min_level: f32,
    pub attack: f32,
    pub release: f32,
    dt: f32,
}

impl Default for LevelingFilter {
    fn default() -> Self {
        Self {
            peak: 30_000.0,
            attenuation: 1.0,
            enabled: true,
            target: 30_000.0,
            max_level: 1.9,
            min_level: 1e-5,
            attack: 0.001,
            release: 0.05,
            dt: 1.0 / 44_100.0,
        }
    }
}

impl LevelingFilter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_dt(&mut self, dt: f32) {
        self.dt = dt;
    }

    fn coefficient(&self, time: f32) -> f32 {
        if time <= 0.0 {
            1.0
        } else {
            1.0 - (-self.dt / time).exp()
        }
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        if !self.enabled {
            return sample;
        }
        self.peak *= 0.9995;
        let abs = sample.abs();
        if abs > self.peak {
            self.peak = abs;
        }
        if self.peak <= 1e-9 {
            return sample;
        }

        let desired = (self.target / self.peak).clamp(self.min_level, self.max_level);
        let coef = if desired < self.attenuation {
            self.coefficient(self.attack)
        } else {
            self.coefficient(self.release)
        };
        self.attenuation += (desired - self.attenuation) * coef;
        sample * self.attenuation
    }

    pub fn attenuation(&self) -> f32 {
        self.attenuation
    }
}

// ---------------------------------------------------------------------------
// Delay filter
// ---------------------------------------------------------------------------

// NOTE: the engine uses a fixed-latency ring-buffer delay (`DelayLine` in
// es-sim) for exhaust pulse staging. The old duplicate `DelayFilter` here
// was removed; nothing else used it.

// ---------------------------------------------------------------------------
// Full per-sample synthesizer (offline, no threads)
// ---------------------------------------------------------------------------

struct ChannelFilters {
    convolution: ConvolutionFilter,
    derivative: DerivativeFilter,
    jitter: JitterFilter,
    flow_noise_lp: ButterworthLowPassFilter,
    mechanical_noise_lp: ButterworthLowPassFilter,
    input_dc: LowPassFilter,
    input_antialias: ButterworthLowPassFilter,
    /// Dry-signal delay compensating the FFT convolution block latency.
    dry_delay: Vec<f32>,
    dry_idx: usize,
}

pub struct Synthesizer {
    pub params: AudioParameters,
    channels: Vec<ChannelFilters>,
    leveler: LevelingFilter,
    antialiasing: ButterworthLowPassFilter,
    #[allow(dead_code)]
    sample_rate: f64,
    /// TEMP diagnostics: max magnitudes seen per render (env-gated).
    dbg_max_input: f32,
    dbg_max_f_in: f32,
    dbg_max_f: f32,
    dbg_max_fp: f32,
    dbg_max_v_in: f32,
    dbg_max_signal: f32,
    dbg_max_leveled: f32,
    dbg_max_antialias: f32,
    dbg_max_r: f32,
    dbg_count: usize,
    dbg_r_idx: usize,
    dbg_v_idx: usize,
    dbg_v_fp: f32,
    dbg_v_f: f32,
    dbg_v_r: f32,
}

impl Synthesizer {
    pub fn new(
        channel_count: usize,
        input_rate: f64,
        audio_rate: f64,
        params: AudioParameters,
    ) -> Self {
        let mut channels = Vec::with_capacity(channel_count);
        if std::env::var("ES_DSP_DEBUG").is_ok() {
            eprintln!(
                "Synth params: flow_noise={} flow_fc={} jitter={} jitter_fc={}",
                params.flow_noise,
                params.flow_noise_frequency_cutoff,
                params.input_sample_noise,
                params.input_sample_noise_frequency_cutoff,
            );
        }
        for _ in 0..channel_count {
            let mut flow = ButterworthLowPassFilter::new();
            flow.set_cutoff_frequency(params.flow_noise_frequency_cutoff, audio_rate as f32);
            let mut mech = ButterworthLowPassFilter::new();
            mech.set_cutoff_frequency(params.mechanical_noise_frequency_cutoff, audio_rate as f32);
            let mut dc = LowPassFilter::new();
            dc.set_cutoff_frequency(10.0);
            dc.set_dt(1.0 / audio_rate as f32);
            let mut aa_in = ButterworthLowPassFilter::new();
            let input_cutoff = params
                .input_antialias_frequency_cutoff
                .min((input_rate * 0.45) as f32);
            aa_in.set_cutoff_frequency(input_cutoff, audio_rate as f32);
            channels.push(ChannelFilters {
                convolution: ConvolutionFilter::new(),
                derivative: DerivativeFilter::new(1.0 / audio_rate as f32),
                jitter: JitterFilter::new(
                    10,
                    params.input_sample_noise_frequency_cutoff,
                    audio_rate as f32,
                ),
                flow_noise_lp: flow,
                mechanical_noise_lp: mech,
                input_dc: dc,
                input_antialias: aa_in,
                dry_delay: Vec::new(),
                dry_idx: 0,
            });
        }

        let mut antialiasing = ButterworthLowPassFilter::new();
        antialiasing.set_cutoff_frequency((audio_rate * 0.45) as f32, audio_rate as f32);

        let mut leveler = LevelingFilter::new();
        leveler.enabled = params.leveler_enabled;
        leveler.target = params.leveler_target;
        leveler.max_level = params.leveler_max_gain;
        leveler.min_level = params.leveler_min_gain;
        leveler.attack = params.leveler_attack;
        leveler.release = params.leveler_release;
        leveler.set_dt(1.0 / audio_rate as f32);

        Self {
            params,
            channels,
            leveler,
            antialiasing,
            sample_rate: audio_rate,
            dbg_max_input: 0.0,
            dbg_max_f_in: 0.0,
            dbg_max_f: 0.0,
            dbg_max_fp: 0.0,
            dbg_max_v_in: 0.0,
            dbg_max_signal: 0.0,
            dbg_max_leveled: 0.0,
            dbg_max_antialias: 0.0,
            dbg_max_r: 0.0,
            dbg_count: 0,
            dbg_r_idx: 0,
            dbg_v_idx: 0,
            dbg_v_fp: 0.0,
            dbg_v_f: 0.0,
            dbg_v_r: 0.0,
        }
    }

    /// Load impulse response for channel (i16 samples, like C++).
    pub fn set_impulse_response(&mut self, channel: usize, ir: &[i16], volume: f32) {
        if channel >= self.channels.len() {
            return;
        }
        let mut clipped = 0usize;
        for (i, s) in ir.iter().enumerate() {
            if (*s as i32).abs() > 100 {
                clipped = i + 1;
            }
        }
        let n = clipped.min(10_000);
        if n == 0 {
            return;
        }
        let ch = &mut self.channels[channel];
        ch.convolution.initialize(n);
        let ir_f = ch.convolution.impulse_response_mut();
        for i in 0..n {
            ir_f[i] = volume * ir[i] as f32 / i16::MAX as f32;
        }
        // Compensate the block latency of the FFT path so the dry/wet mix
        // stays phase-aligned.
        let lat = ch.convolution.latency();
        ch.dry_delay = vec![0.0; lat];
        ch.dry_idx = 0;
    }

    /// Render one audio-rate sample from already-resampled input samples per channel.
    pub fn render_sample(&mut self, input: &[f32], flow_env: &[f32]) -> i16 {
        quantize_i16(self.render_sample_f32(input, flow_env))
    }

    pub fn render_sample_f32(&mut self, input: &[f32], flow_env: &[f32]) -> f32 {
        let p = self.params;
        let mut signal = 0.0f32;
        let n_ch = self.channels.len().min(input.len());
        self.dbg_count += 1;

        for (i, ch) in self.channels.iter_mut().enumerate().take(n_ch) {
            if input[i].abs() > self.dbg_max_input {
                self.dbg_max_input = input[i].abs();
            }
            // Reference `Synthesizer::writeInput` applies the 1900 Hz input
            // antialiasing filter to every resampled sample.
            let input_i = ch.input_antialias.process(input[i]);
            if input_i.abs() > self.dbg_max_f_in {
                self.dbg_max_f_in = input_i.abs();
            }
            let jittered = ch.jitter.process(input_i, p.input_sample_noise);
            let f_in = jittered;
            let f_dc = ch.input_dc.process(f_in);
            let f = f_in - f_dc;
            let f_p = ch.derivative.process(f_in);
            let hf = f_p / (2.0 * PI_F32 * p.hf_reference_hz.max(1.0));
            if f.abs() > self.dbg_max_f {
                self.dbg_max_f = f.abs();
            }
            if f_p.abs() > self.dbg_max_fp {
                self.dbg_max_fp = f_p.abs();
            }

            let noise = 2.0 * ch.jitter.next_random() - 1.0;
            let flow = ch.flow_noise_lp.process(noise);
            let mechanical = ch
                .mechanical_noise_lp
                .process(2.0 * ch.jitter.next_random() - 1.0);
            if flow.abs() > self.dbg_max_r {
                self.dbg_max_r = flow.abs();
                self.dbg_r_idx = self.dbg_count;
            }
            let envelope = flow_env.get(i).copied().unwrap_or(1.0);

            let mut v_in = f * (1.0 - p.hf_mix)
                + hf * p.hf_mix
                + flow * p.flow_noise * envelope
                + mechanical * p.mechanical_noise;
            if v_in.is_subnormal() {
                v_in = 0.0;
            }
            if v_in.abs() > self.dbg_max_v_in {
                self.dbg_max_v_in = v_in.abs();
                self.dbg_v_idx = self.dbg_count;
                self.dbg_v_fp = f_p;
                self.dbg_v_f = f;
                self.dbg_v_r = flow;
            }

            let wet = ch.convolution.process(v_in);
            // Delay the dry path by the FFT block latency to match the wet path.
            let dry = {
                if ch.dry_delay.is_empty() {
                    v_in
                } else {
                    let out = ch.dry_delay[ch.dry_idx];
                    ch.dry_delay[ch.dry_idx] = v_in;
                    ch.dry_idx = (ch.dry_idx + 1) % ch.dry_delay.len();
                    out
                }
            };
            let v = p.convolution * wet + (1.0 - p.convolution) * dry;
            signal += v;
        }

        if signal.abs() > self.dbg_max_signal {
            self.dbg_max_signal = signal.abs();
        }
        let signal = self.antialiasing.process(signal);
        if signal.abs() > self.dbg_max_antialias {
            self.dbg_max_antialias = signal.abs();
        }
        let leveled = self.leveler.process(signal) * p.volume;
        if leveled.abs() > self.dbg_max_leveled {
            self.dbg_max_leveled = leveled.abs();
        }
        leveled
    }

    pub fn leveler_gain(&self) -> f32 {
        self.leveler.attenuation()
    }
}

// ---------------------------------------------------------------------------
// Cylinder pulse reconstruction (acoustic rate)
// ---------------------------------------------------------------------------

/// Per-cylinder pulse source sampled at the simulation rate.
///
/// `base` is (runner pressure - atmosphere), `dyn_p` the dynamic pressure term
/// and `att3` the rpm attenuation (cubed) times its scale factor.
pub struct PulseChannel<'a> {
    pub base: &'a [f64],
    pub dyn_p: &'a [f64],
    pub att3: &'a [f64],
    pub intake: &'a [f64],
    pub delay_s: f64,
    pub gain: f64,
    pub intake_gain: f64,
    pub exhaust: usize,
}

fn cubic_sample(series: &[f64], x: f64) -> f64 {
    if series.is_empty() || x < 0.0 {
        return 0.0;
    }
    let last = series.len() - 1;
    if x >= last as f64 {
        return series[last];
    }
    let i = x.floor() as isize;
    let f = x - i as f64;
    let get = |k: isize| -> f64 {
        if k < 0 {
            0.0
        } else {
            series[(k as usize).min(last)]
        }
    };
    let p0 = get(i - 1);
    let p1 = get(i);
    let p2 = get(i + 1);
    let p3 = get(i + 2);
    0.5 * ((2.0 * p1)
        + (-p0 + p2) * f
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * f * f
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * f * f * f)
}

/// Rebuild per-exhaust pulse channels at `out_rate` from sim-rate cylinder
/// sources, interpolating the physical variables and applying each cylinder's
/// acoustic delay at sub-sample resolution.
pub fn render_pulse_channels(
    channels: &[PulseChannel<'_>],
    in_rate: f64,
    out_rate: f64,
    n_exhausts: usize,
) -> Vec<Vec<f64>> {
    let n_out = n_exhausts.max(1);
    if channels.is_empty() || channels[0].base.is_empty() {
        return vec![Vec::new(); n_out];
    }
    let in_len = channels[0].base.len();
    let duration = in_len as f64 / in_rate;
    let out_len = (duration * out_rate).round() as usize;
    let mut out: Vec<Vec<f64>> = (0..n_out).map(|_| vec![0.0; out_len]).collect();
    for i in 0..out_len {
        let t = i as f64 / out_rate;
        for ch in channels {
            if ch.exhaust >= n_out {
                continue;
            }
            let x = (t - ch.delay_s) * in_rate;
            let base = cubic_sample(ch.base, x);
            let dyn_p = cubic_sample(ch.dyn_p, x);
            let att3 = cubic_sample(ch.att3, x);
            let intake = cubic_sample(ch.intake, x);
            let pulse = ch.gain * att3 * (base + 0.1 * dyn_p) + ch.intake_gain * intake;
            out[ch.exhaust][i] += pulse;
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Structural scene reconstruction
// ---------------------------------------------------------------------------

pub struct SceneChannel<'a> {
    pub base: &'a [f64],
    pub derivative: &'a [f64],
    pub blowdown: &'a [f64],
    pub flow: &'a [f64],
    pub runner_temp: &'a [f64],
    pub delay_s: f64,
    pub bank: usize,
    pub signature: f64,
    pub header_length_m: f64,
}

pub struct SceneRenderParams<'a> {
    pub config: crate::scene::SceneConfig,
    pub bank_gain: [f32; 2],
    pub collector_pressure: &'a [Vec<f64>],
    pub master: &'a [f64],
    pub rpm: &'a [f64],
    pub turbulence: &'a [f64],
    pub flow_excitation_gain: f64,
    pub header_reflection: f32,
    pub header_temperature_dependent: bool,
    pub header_gain: f64,
    pub load: f32,
    pub throttle: f32,
}

#[derive(Clone, Debug, Default)]
pub struct SceneStems {
    pub air: Vec<f32>,
    pub cover: Vec<f32>,
    pub mount: Vec<f32>,
    pub cylinder: Vec<f32>,
    pub header: Vec<f32>,
    pub output: Vec<f32>,
}

pub fn render_scene(
    channels: &[SceneChannel<'_>],
    in_rate: f64,
    out_rate: f64,
    params: &SceneRenderParams<'_>,
) -> SceneStems {
    use crate::scene::{SceneConfig, SceneInput, StructuralScene, CYLINDER_COUNT};
    use crate::waveguide::RunnerWaveguide;

    let in_len = channels.first().map(|c| c.base.len()).unwrap_or(0);
    if in_len == 0 || channels.is_empty() {
        return SceneStems::default();
    }
    let duration = in_len as f64 / in_rate;
    let out_len = (duration * out_rate).round() as usize;
    let mut stems = SceneStems {
        air: vec![0.0; out_len],
        cover: vec![0.0; out_len],
        mount: vec![0.0; out_len],
        cylinder: vec![0.0; out_len],
        header: vec![0.0; out_len],
        output: vec![0.0; out_len],
    };
    let mut scene = match StructuralScene::new(out_rate as f32, params.config) {
        Ok(s) => s,
        Err(_) => match StructuralScene::new(out_rate as f32, SceneConfig::default()) {
            Ok(s) => s,
            Err(_) => return stems,
        },
    };
    let mut waveguides: Vec<RunnerWaveguide> = channels
        .iter()
        .map(|c| {
            RunnerWaveguide::new(
                c.header_length_m as f32,
                545.0,
                params.header_reflection,
                out_rate as f32,
                params.header_temperature_dependent,
            )
        })
        .collect();
    let mut prev_flow = vec![0.0f64; channels.len()];
    let max_abs = |series: &[f64]| series.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    let mut base_max = 0.0f64;
    let mut deriv_max = 0.0f64;
    let mut blowdown_max = 0.0f64;
    let mut flow_delta_max = 0.0f64;
    let mut collector_max = 0.0f64;
    for ch in channels {
        base_max = base_max.max(max_abs(ch.base));
        deriv_max = deriv_max.max(max_abs(ch.derivative));
        blowdown_max = blowdown_max.max(max_abs(ch.blowdown));
        let d_max = ch
            .flow
            .windows(2)
            .fold(0.0f64, |a, w| a.max((w[1] - w[0]).abs()))
            * in_rate;
        flow_delta_max = flow_delta_max.max(d_max);
    }
    for series in params.collector_pressure {
        collector_max = collector_max.max(max_abs(series));
    }
    let master_max = max_abs(params.master);
    let master_scale = if master_max > 0.0 {
        1.0 / master_max
    } else {
        0.0
    };
    let scene_scale = if master_max > 0.0 { master_max } else { 1.0 };
    let excitation_scale = params.config.excitation_scale.max(0.0) as f64;
    let base_scale = if base_max > 0.0 {
        excitation_scale / base_max
    } else {
        0.0
    };
    let deriv_scale = if deriv_max > 0.0 {
        excitation_scale / deriv_max
    } else {
        0.0
    };
    let blowdown_scale = if blowdown_max > 0.0 {
        excitation_scale / blowdown_max
    } else {
        0.0
    };
    let flow_delta_scale = if flow_delta_max > 0.0 {
        excitation_scale / flow_delta_max
    } else {
        0.0
    };
    let collector_scale = if collector_max > 0.0 {
        excitation_scale / collector_max
    } else {
        0.0
    };

    for i in 0..out_len {
        let t = i as f64 / out_rate;
        let x = t * in_rate;
        let mut input = SceneInput::default();
        input.load = params.load;
        input.throttle = params.throttle;
        input.master = (params.master.get(i).copied().unwrap_or(0.0) * master_scale) as f32;
        input.turbulence = cubic_sample(params.turbulence, x) as f32;
        let rpm = cubic_sample(params.rpm, x);
        input.high_rpm = ((rpm - params.config.high_rpm_start as f64)
            / params.config.high_rpm_span.max(1.0) as f64)
            .clamp(0.0, 1.0) as f32;
        let mut pressure = 0.0f64;
        let mut derivative = 0.0f64;
        let mut header_sum = 0.0f64;
        for (ci, ch) in channels.iter().enumerate().take(CYLINDER_COUNT) {
            let xs = (t - ch.delay_s) * in_rate;
            let base = cubic_sample(ch.base, xs) * base_scale;
            let deriv = cubic_sample(ch.derivative, xs) * deriv_scale;
            let blowdown = cubic_sample(ch.blowdown, xs) * blowdown_scale;
            let flow = cubic_sample(ch.flow, xs);
            let temp = cubic_sample(ch.runner_temp, xs);
            let bank_gain = params.bank_gain[ch.bank.min(1)] as f64;
            pressure += bank_gain * base;
            derivative += bank_gain * deriv;
            input.cylinder_derivative[ci] = deriv as f32;
            input.cylinder_blowdown[ci] = blowdown as f32;
            if let Some(wg) = waveguides.get_mut(ci) {
                let d_flow = (flow - prev_flow[ci]) * in_rate * flow_delta_scale;
                prev_flow[ci] = flow;
                let excitation = d_flow * params.flow_excitation_gain * ch.signature;
                let header = wg.process(excitation as f32, temp as f32);
                input.cylinder_header[ci] = header;
                header_sum += header as f64 * params.header_gain;
            }
        }
        input.pressure = pressure as f32;
        input.derivative = derivative as f32;
        for bank in 0..2 {
            let series = params.collector_pressure.get(bank);
            let value = series
                .map(|s| cubic_sample(s, x) * collector_scale)
                .unwrap_or(0.0);
            input.collector[bank] = value as f32;
        }
        let frame = scene.process(&input);
        stems.air[i] = frame.engine_air * scene_scale as f32;
        stems.cover[i] = frame.engine_cover * scene_scale as f32;
        stems.mount[i] = frame.mount_monocoque * scene_scale as f32;
        stems.cylinder[i] = frame.cylinder_mechanical_sum * scene_scale as f32;
        stems.header[i] = header_sum as f32 * scene_scale as f32;
        stems.output[i] = frame.output * scene_scale as f32;
    }
    stems
}

// ---------------------------------------------------------------------------
// Offline multi-channel render
// ---------------------------------------------------------------------------

fn quantize_i16(v: f32) -> i16 {
    let r = (v as f64).round();
    r.clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

/// Render offline from `input_rate` channels to `output_rate` i16 samples.
///
/// The synthesizer runs at `dsp_rate`; when `dsp_rate > output_rate` the
/// output is band-limited before decimation.
pub fn render_offline_to(
    channels_in: &[Vec<f64>],
    input_rate: f64,
    dsp_rate: f64,
    output_rate: f64,
    params: &AudioParameters,
    irs: &[Option<(u32, Vec<i16>, f32)>],
    flow_env: &[Vec<f64>],
    normalize_peak: Option<f32>,
) -> Vec<i16> {
    let n_ch = channels_in.len();
    if n_ch == 0 || channels_in[0].is_empty() {
        return Vec::new();
    }
    let mut synth = Synthesizer::new(n_ch, input_rate, dsp_rate, *params);
    for (i, ir) in irs.iter().enumerate() {
        if let Some((_, samples, volume)) = ir {
            synth.set_impulse_response(i, samples, *volume);
        }
    }

    let ratio = dsp_rate / input_rate;
    let dsp_len = (channels_in[0].len() as f64 * ratio).round() as usize;
    let mut decim = if dsp_rate > output_rate + 1e-9 {
        let mut f = ButterworthLowPassFilter::new();
        let cutoff = (output_rate * 0.45).min(dsp_rate * 0.45) as f32;
        f.set_cutoff_frequency(cutoff, dsp_rate as f32);
        Some(f)
    } else {
        None
    };
    let mut inputs = vec![0.0f32; n_ch];
    let mut envs = vec![0.0f32; flow_env.len().min(n_ch)];
    let mut outs: Vec<f32> = Vec::with_capacity(dsp_len);
    let mut phase = 0.0f64;

    for i in 0..dsp_len {
        let target_input = i as f64 / ratio;
        let i0 = target_input.floor() as usize;
        let frac = target_input - i0 as f64;

        for c in 0..n_ch {
            let len = channels_in[c].len();
            let s0 = channels_in[c][i0.min(len - 1)];
            let s1 = channels_in[c][(i0 + 1).min(len - 1)];
            inputs[c] = (s0 * (1.0 - frac) + s1 * frac) as f32;
        }
        for c in 0..envs.len() {
            let len = flow_env[c].len();
            if len == 0 {
                envs[c] = 1.0;
                continue;
            }
            let s0 = flow_env[c][i0.min(len - 1)];
            let s1 = flow_env[c][(i0 + 1).min(len - 1)];
            envs[c] = (s0 * (1.0 - frac) + s1 * frac) as f32;
        }

        let y = synth.render_sample_f32(&inputs, &envs);
        let y = match decim.as_mut() {
            Some(f) => f.process(y),
            None => y,
        };
        phase += output_rate;
        while phase >= dsp_rate - 1e-9 {
            phase -= dsp_rate;
            outs.push(y);
        }
    }

    let scale = match normalize_peak {
        Some(target) => {
            let peak = outs.iter().fold(0.0f32, |a, b| a.max(b.abs()));
            if peak > 1e-9 {
                target / peak
            } else {
                1.0
            }
        }
        None => 1.0,
    };
    let mut out: Vec<i16> = Vec::with_capacity(outs.len());
    for y in outs {
        out.push(quantize_i16(y * scale));
    }

    #[allow(clippy::let_and_return)]
    let _dbg = (
        synth.dbg_max_input,
        synth.dbg_max_f_in,
        synth.dbg_max_f,
        synth.dbg_max_fp,
        synth.dbg_max_v_in,
        synth.dbg_max_signal,
        synth.dbg_max_antialias,
        synth.dbg_max_leveled,
    );
    if std::env::var("ES_DSP_DEBUG").is_ok() {
        eprintln!(
            "DSP diag: max input={:.3e} f_in={:.3e} f={:.3e} f_p={:.3e} v_in={:.3e}@{} (fp={:.3e} f={:.3e} rmix={:.3e}) r_max={:.3e}@{} signal={:.3e} antialias={:.3e} leveled={:.3e}",
            synth.dbg_max_input,
            synth.dbg_max_f_in,
            synth.dbg_max_f,
            synth.dbg_max_fp,
            synth.dbg_max_v_in,
            synth.dbg_v_idx,
            synth.dbg_v_fp,
            synth.dbg_v_f,
            synth.dbg_v_r,
            synth.dbg_max_r,
            synth.dbg_r_idx,
            synth.dbg_max_signal,
            synth.dbg_max_antialias,
            synth.dbg_max_leveled,
        );
    }

    out
}

/// Render offline from sim-rate multi-channel input to audio-rate i16 samples.
///
/// Each channel in `channels_in` is a time series at `input_rate`.
/// Channels are summed after per-channel DSP (matching Synthesizer multi-channel).
pub fn render_offline(
    channels_in: &[Vec<f64>],
    input_rate: f64,
    audio_rate: f64,
    params: &AudioParameters,
    irs: &[Option<(u32, Vec<i16>, f32)>],
) -> Vec<i16> {
    render_offline_to(
        channels_in,
        input_rate,
        audio_rate,
        audio_rate,
        params,
        irs,
        &[],
        None,
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn low_pass_attenuates_dc_gradually() {
        let mut lp = LowPassFilter::new();
        lp.set_cutoff_frequency(100.0);
        lp.set_dt(1.0 / 44_100.0);
        let mut y = 0.0;
        for _ in 0..10_000 {
            y = lp.process(1.0);
        }
        assert!(y > 0.5, "y={y}");
        assert!(y < 1.01, "y={y}");
    }

    #[test]
    fn leveler_pulls_toward_target() {
        let mut lev = LevelingFilter::new();
        lev.target = 30_000.0;
        lev.max_level = 1.9;
        lev.min_level = 1e-5;
        let mut y = 0.0;
        for _ in 0..5000 {
            y = lev.process(1000.0);
        }
        assert!(y > 1000.0, "y={y}");
    }

    #[test]
    fn convolution_delta_passes_impulse() {
        let mut c = ConvolutionFilter::new();
        c.initialize(4);
        c.impulse_response_mut()[0] = 1.0;
        let y = c.process(1.0);
        assert!((y - 1.0).abs() < 1e-6, "y={y}");
    }

    #[test]
    fn derivative_of_step_is_large_then_zero() {
        let mut d = DerivativeFilter::new(1.0 / 44_100.0);
        let d0 = d.process(0.0);
        let d1 = d.process(1.0);
        let d2 = d.process(1.0);
        assert!(d0.abs() < 1e-6);
        assert!(d1 > 1000.0, "d1={d1}");
        assert!(d2.abs() < 1e-3, "d2={d2}");
    }

    #[test]
    fn butterworth_stable_at_low_cutoffs() {
        // Very low normalized cutoffs place the pole cluster within ~1e-3 of
        // the unit circle; f32 coefficient rounding once made these diverge.
        for &fc in &[10.0f32, 20.0, 100.0, 3500.0] {
            let mut f = ButterworthLowPassFilter::new();
            f.set_cutoff_frequency(fc, 44100.0);
            let mut seed = 12345u64;
            let mut max_out = 0.0f32;
            for _ in 0..100_000 {
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let noise = ((seed >> 33) as f64 / (u64::MAX >> 33) as f64) * 2.0 - 1.0;
                let y = f.process(noise as f32);
                if y.abs() > max_out {
                    max_out = y.abs();
                }
            }
            assert!(
                max_out.is_finite() && max_out < 10.0,
                "fc={fc} unstable: {max_out}"
            );
        }
    }

    #[test]
    fn leveler_handles_pathological_input() {
        let mut lvl = LevelingFilter::new();
        lvl.target = 30_000.0;
        lvl.min_level = 1e-5;
        lvl.max_level = 1.9;
        let mut max_out = 0.0f32;
        for i in 0..5000 {
            let x = if i < 10 { 1e12 } else { (i as f32) * 3.0 };
            let y = lvl.process(x);
            assert!(y.is_finite(), "leveler produced non-finite output");
            if y.abs() > max_out {
                max_out = y.abs();
            }
        }
        assert!(max_out.is_finite());
        assert!(
            (1e-6..=2.0).contains(&lvl.attenuation()),
            "gain={}",
            lvl.attenuation()
        );
    }

    #[test]
    fn leveler_bypasses_when_disabled() {
        let mut lvl = LevelingFilter::new();
        lvl.enabled = false;
        for i in 0..1000 {
            let x = (i as f32) * 0.5 - 250.0;
            let y = lvl.process(x);
            assert!((y - x).abs() < 1e-6, "bypass modified sample: {y} != {x}");
        }
        assert!((lvl.attenuation() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn hf_branch_is_frequency_normalized() {
        let sr = 48_000.0f32;
        let reference = 1_000.0f32;
        let expected = [
            (500.0f32, 0.5f32),
            (1_000.0, 1.0),
            (2_000.0, 2.0),
            (4_000.0, 4.0),
            (8_000.0, 8.0),
        ];
        for &(freq, ratio) in &expected {
            let mut d = DerivativeFilter::new(1.0 / sr);
            let n = 4800;
            let mut sum2 = 0.0f64;
            for i in 0..n {
                let t = i as f32 / sr;
                let x = (2.0 * PI_F32 * freq * t).sin();
                let hf = d.process(x) / (2.0 * PI_F32 * reference);
                if i > 100 {
                    sum2 += (hf as f64) * (hf as f64);
                }
            }
            let rms = (sum2 / (n - 100) as f64).sqrt();
            let expect = (ratio as f64) / 2.0f64.sqrt();
            assert!(
                (rms - expect).abs() < 0.05 * expect + 0.01,
                "freq={freq} rms={rms} expect={expect}"
            );
        }
    }

    #[test]
    fn butterworth_passes_dc() {
        let mut b = ButterworthLowPassFilter::new();
        b.set_cutoff_frequency(1000.0, 44_100.0);
        let mut y = 0.0;
        for _ in 0..2000 {
            y = b.process(1.0);
        }
        assert!((y - 1.0).abs() < 0.1, "y={y}");
    }

    #[test]
    fn resample_inline_length() {
        let input = vec![0.0; 1000];
        let out = render_offline(
            &[input],
            10_000.0,
            44_100.0,
            &AudioParameters::default(),
            &[None],
        );
        assert_eq!(out.len(), 4410);
    }

    #[test]
    fn synthesizer_renders_nonzero() {
        let params = AudioParameters::default();
        let mut s = Synthesizer::new(1, 10_000.0, 44_100.0, params);
        let mut out = Vec::new();
        for i in 0..4410 {
            let t = i as f64 / 44_100.0;
            let x = (2.0 * std::f64::consts::PI * 100.0 * t).sin() * 10_000.0;
            out.push(s.render_sample(&[x as f32], &[]));
        }
        let peak = out.iter().map(|&s| (s as i32).abs()).max().unwrap();
        assert!(peak > 0, "peak={peak}");
        assert!(out.iter().any(|s| *s != 0));
    }

    #[test]
    fn render_offline_produces_samples() {
        let params = AudioParameters::default();
        let input: Vec<f64> = (0..1000).map(|i| (i as f64 * 0.1).sin() * 5000.0).collect();
        let out = render_offline(&[input], 10_000.0, 44_100.0, &params, &[None]);
        assert_eq!(out.len(), 4410);
        assert!(out.iter().any(|s| *s != 0));
    }

    #[test]
    fn fft_convolution_matches_direct() {
        // IR length > DIRECT_CONV_THRESHOLD forces the FFT path.
        let n = 200;
        let mut c = ConvolutionFilter::new();
        c.initialize(n);
        {
            let ir = c.impulse_response_mut();
            for (i, t) in ir.iter_mut().enumerate() {
                *t = ((i * 37 % 101) as f32 - 50.0) / 100.0;
            }
        }
        let ir: Vec<f32> = c.impulse_response_mut().to_vec();
        let input: Vec<f32> = (0..8192)
            .map(|i| ((i * 13 % 997) as f32 - 498.0) / 997.0)
            .collect();

        let mut out = Vec::with_capacity(input.len());
        for &s in &input {
            out.push(c.process(s));
        }

        // The wet stream is delayed by `latency` samples; before that it is 0.
        let latency = c.latency();
        assert_eq!(latency, FFT_BLOCK_SIZE - 1);
        assert!(out[..latency].iter().all(|&v| v == 0.0));

        for i in 0..(input.len() - latency) {
            let mut expect = 0.0f32;
            for j in 0..n {
                if i >= j {
                    expect += ir[j] * input[i - j];
                }
            }
            let got = out[i + latency];
            assert!((got - expect).abs() < 1e-3, "i={i} got={got} exp={expect}");
        }
    }

    fn bin_magnitude(signal: &[f64], rate: f64, freq: f64) -> f64 {
        let n = signal.len().next_power_of_two();
        let mut planner = FftPlanner::<f64>::new();
        let fft = planner.plan_fft_forward(n);
        let mut buf: Vec<Complex<f64>> = Vec::with_capacity(n);
        for i in 0..n {
            let w = if i < signal.len() {
                0.5 - 0.5
                    * (2.0 * std::f64::consts::PI * i as f64 / (signal.len() - 1) as f64).cos()
            } else {
                0.0
            };
            let v = if i < signal.len() { signal[i] } else { 0.0 };
            buf.push(Complex::new(v * w, 0.0));
        }
        fft.process(&mut buf);
        let center = (freq * n as f64 / rate).round() as isize;
        let mut m = 0.0f64;
        for k in (center - 2).max(1)..=(center + 2).min(n as isize / 2 - 1) {
            m = m.max(buf[k as usize].norm());
        }
        m
    }

    fn pulse_train(
        rate: f64,
        rpm: f64,
        n: usize,
        interval_deg: f64,
        offset_samples: f64,
    ) -> Vec<f64> {
        let revs_per_s = rpm / 60.0;
        let interval = (interval_deg / 360.0) / revs_per_s * rate;
        let mut x = vec![0.0; n];
        let mut pos = offset_samples;
        while (pos as usize) < n {
            x[pos as usize] += 1.0;
            pos += interval;
        }
        x
    }

    #[test]
    fn v10_ideal_train_dominated_by_firing_order() {
        let rate = 48_000.0;
        let rpm = 18_000.0;
        let x = pulse_train(rate, rpm, 48_000, 72.0, 0.0);
        let firing = bin_magnitude(&x, rate, rpm / 12.0);
        let bank = bin_magnitude(&x, rate, rpm / 24.0);
        let crank = bin_magnitude(&x, rate, rpm / 60.0);
        assert!(firing > 0.0);
        assert!(
            20.0 * (bank / firing).log10() < -20.0,
            "bank should vanish in a uniform train"
        );
        assert!(20.0 * (crank / firing).log10() < -20.0);
    }

    #[test]
    fn banks_offset_half_period_cancel_bank_order() {
        let rate = 48_000.0;
        let rpm = 18_000.0;
        let firing = rpm / 12.0;
        let bank_interval_samples = 2.0 * rate / firing;
        let bank_a = pulse_train(rate, rpm, 48_000, 144.0, 0.0);
        let bank_b = pulse_train(rate, rpm, 48_000, 144.0, bank_interval_samples / 2.0);
        let sum: Vec<f64> = bank_a.iter().zip(&bank_b).map(|(a, b)| a + b).collect();
        let firing_mag = bin_magnitude(&sum, rate, rpm / 12.0);
        let bank_mag = bin_magnitude(&sum, rate, rpm / 24.0);
        assert!(
            20.0 * (bank_mag / firing_mag).log10() < -20.0,
            "equal banks must cancel the bank order"
        );
    }

    #[test]
    fn known_delay_changes_bank_versus_firing_ratio() {
        let rate = 48_000.0;
        let rpm = 18_000.0;
        let firing = rpm / 12.0;
        let bank_interval_samples = 2.0 * rate / firing;
        let delay = bank_interval_samples / 2.0;
        let bank_a = pulse_train(rate, rpm, 48_000, 144.0, 0.0);
        let bank_b = pulse_train(
            rate,
            rpm,
            48_000,
            144.0,
            bank_interval_samples / 2.0 + delay,
        );
        let sum: Vec<f64> = bank_a.iter().zip(&bank_b).map(|(a, b)| a + b).collect();
        let firing_mag = bin_magnitude(&sum, rate, rpm / 12.0);
        let bank_mag = bin_magnitude(&sum, rate, rpm / 24.0);
        let ratio_db = 20.0 * (bank_mag / firing_mag).log10();
        assert!(
            ratio_db > -3.0,
            "a half-period bank delay must revive the bank order (got {ratio_db:.1} dB)"
        );
    }

    #[test]
    fn resample_96_to_48_keeps_harmonics_without_aliases() {
        let mut params = AudioParameters::default();
        params.leveler_enabled = false;
        params.hf_mix = 0.0;
        params.flow_noise = 0.0;
        params.mechanical_noise = 0.0;
        params.input_sample_noise = 0.0;
        params.convolution = 0.0;
        params.volume = 1.0;
        params.input_antialias_frequency_cutoff = 40_000.0;
        let rate = 96_000.0;
        let n = 96_000usize;
        let freqs = [1_000.0, 5_000.0, 10_000.0, 15_000.0, 18_000.0];
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / rate;
                freqs
                    .iter()
                    .map(|f| (2.0 * std::f64::consts::PI * f * t).sin() * 5_000.0)
                    .sum()
            })
            .collect();
        let out = render_offline_to(&[input], rate, rate, 48_000.0, &params, &[None], &[], None);
        let signal: Vec<f64> = out.iter().map(|&s| s as f64).collect();
        for f in freqs {
            let mag = bin_magnitude(&signal, 48_000.0, f);
            assert!(mag > 100.0, "harmonic {f} lost (mag={mag})");
        }
        for alias in [1_500.0, 3_000.0, 7_000.0, 21_000.0] {
            let mag = bin_magnitude(&signal, 48_000.0, alias);
            let ref_mag = bin_magnitude(&signal, 48_000.0, 5_000.0);
            assert!(
                mag < 0.05 * ref_mag,
                "alias energy at {alias} Hz: {mag} vs {ref_mag}"
            );
        }
    }

    #[test]
    fn disabled_leveler_leaves_no_digital_plateaus() {
        let mut params = AudioParameters::default();
        params.leveler_enabled = false;
        params.hf_mix = 0.0;
        params.flow_noise = 0.0;
        params.mechanical_noise = 0.0;
        params.input_sample_noise = 0.0;
        params.convolution = 0.0;
        params.input_sample_noise = 0.0;
        let rate = 48_000.0;
        let input: Vec<f64> = (0..48_000)
            .map(|i| {
                let t = i as f64 / rate;
                (2.0 * std::f64::consts::PI * 150.0 * t).sin() * 3_000_000.0
            })
            .collect();
        let out = render_offline_to(
            &[input],
            rate,
            rate,
            rate,
            &params,
            &[None],
            &[],
            Some(29_000.0),
        );
        let mut max_run = 0usize;
        let mut run = 0usize;
        let mut prev = None;
        for &s in out.iter().skip(1_000) {
            match prev {
                Some(p) if p == s => {
                    run += 1;
                    max_run = max_run.max(run);
                }
                _ => run = 1,
            }
            prev = Some(s);
        }
        assert!(max_run < 8, "digital plateau detected: run={max_run}");
        let peak = out.iter().map(|&s| (s as i32).abs()).max().unwrap();
        assert!((peak - 29_000).abs() < 50, "normalization off: peak={peak}");
    }

    #[test]
    fn fft_convolution_with_ir_sets_latency_and_dry_path() {
        // Short IR: no latency, direct path.
        let mut c = ConvolutionFilter::new();
        c.initialize(4);
        c.impulse_response_mut()[0] = 1.0;
        assert_eq!(c.latency(), 0);
        // Empty IR: passthrough.
        let mut c2 = ConvolutionFilter::new();
        c2.initialize(0);
        assert_eq!(c2.process(0.5), 0.5);
    }

    #[test]
    fn render_scene_is_finite_and_audible_with_waveguide() {
        let n = 4_800usize;
        let base: Vec<f64> = (0..n).map(|i| if i == 0 { 1.0e5 } else { 0.0 }).collect();
        let derivative = base.clone();
        let blowdown = base.clone();
        let flow: Vec<f64> = (0..n).map(|i| (i as f64 * 0.02).sin()).collect();
        let temp = vec![800.0; n];
        let channels = [SceneChannel {
            base: &base,
            derivative: &derivative,
            blowdown: &blowdown,
            flow: &flow,
            runner_temp: &temp,
            delay_s: 0.0,
            bank: 0,
            signature: 1.0,
            header_length_m: 0.508,
        }];
        let master: Vec<f64> = (0..n).map(|i| if i == 0 { 1.0e6 } else { 0.0 }).collect();
        let rpm = vec![18_000.0; n];
        let turbulence = vec![0.0; n];
        let collector = vec![vec![0.0; n], vec![0.0; n]];
        let mut config = crate::scene::SceneConfig::default();
        config.enabled = true;
        let params = SceneRenderParams {
            config,
            bank_gain: [1.0, 1.0],
            collector_pressure: &collector,
            master: &master,
            rpm: &rpm,
            turbulence: &turbulence,
            flow_excitation_gain: 1.0,
            header_reflection: -0.34,
            header_temperature_dependent: true,
            header_gain: 0.5,
            load: 0.78,
            throttle: 0.72,
        };
        let stems = render_scene(&channels, 48_000.0, 48_000.0, &params);
        assert_eq!(stems.output.len(), n);
        assert!(stems.output.iter().all(|v| v.is_finite()));
        assert!(stems.output.iter().any(|v| v.abs() > 0.0));
        assert!(
            stems.header.iter().any(|v| v.abs() > 0.0),
            "waveguide produced no header output"
        );
        assert!(stems.air.iter().any(|v| v.abs() > 0.0));
        assert!(stems.cover.iter().any(|v| v.abs() > 0.0));
        assert!(stems.mount.iter().any(|v| v.abs() > 0.0));
    }

    #[test]
    fn render_scene_silent_input_stays_silent() {
        let n = 2_400usize;
        let zeros = vec![0.0f64; n];
        let channels = [SceneChannel {
            base: &zeros,
            derivative: &zeros,
            blowdown: &zeros,
            flow: &zeros,
            runner_temp: &zeros,
            delay_s: 0.0,
            bank: 0,
            signature: 1.0,
            header_length_m: 0.508,
        }];
        let collector = vec![zeros.clone(), zeros.clone()];
        let mut config = crate::scene::SceneConfig::default();
        config.enabled = true;
        let params = SceneRenderParams {
            config,
            bank_gain: [1.0, 1.0],
            collector_pressure: &collector,
            master: &zeros,
            rpm: &zeros,
            turbulence: &zeros,
            flow_excitation_gain: 1.0,
            header_reflection: -0.34,
            header_temperature_dependent: true,
            header_gain: 0.5,
            load: 0.78,
            throttle: 0.72,
        };
        let stems = render_scene(&channels, 48_000.0, 48_000.0, &params);
        assert!(stems.output.iter().all(|v| *v == 0.0));
        assert!(stems.header.iter().all(|v| *v == 0.0));
    }
}
