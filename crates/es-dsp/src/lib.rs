//! DSP filters ported from engine-sim's synthesizer chain:
//! jitter → DC-block → derivative(hf) → air-noise mix → convolution IR
//! → antialias → leveler → volume → i16.

use std::f32::consts::PI as PI_F32;
use std::f64::consts::PI as PI_F64;

// ---------------------------------------------------------------------------
// Audio parameters (mirrors Synthesizer::AudioParameters)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct AudioParameters {
    pub volume: f32,
    pub convolution: f32,
    pub d_f_f_mix: f32,
    pub input_sample_noise: f32,
    pub input_sample_noise_frequency_cutoff: f32,
    pub air_noise: f32,
    pub air_noise_frequency_cutoff: f32,
    /// Input antialias cutoff, bounded to 45% of the simulation rate.
    pub input_antialias_frequency_cutoff: f32,
    pub leveler_target: f32,
    pub leveler_max_gain: f32,
    pub leveler_min_gain: f32,
}

impl Default for AudioParameters {
    fn default() -> Self {
        Self {
            volume: 1.0,
            convolution: 1.0,
            d_f_f_mix: 0.01,
            input_sample_noise: 0.5,
            input_sample_noise_frequency_cutoff: 10_000.0,
            air_noise: 1.0,
            air_noise_frequency_cutoff: 2_000.0,
            input_antialias_frequency_cutoff: 1_900.0,
            leveler_target: 30_000.0,
            leveler_max_gain: 1.9,
            leveler_min_gain: 1e-5,
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
            * (sample + 4.0 * self.x_at(0) + 6.0 * self.x_at(1) + 4.0 * self.x_at(2) + self.x_at(3));
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
        let mut buf: Vec<Complex<f32>> = self.ir_raw.iter().map(|&t| Complex::new(t, 0.0)).collect();
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
        let s = self.noise_filter.process(dist * self.jitter_scale * jitter_scale);
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
    pub target: f32,
    pub max_level: f32,
    pub min_level: f32,
}

impl Default for LevelingFilter {
    fn default() -> Self {
        Self {
            peak: 30_000.0,
            attenuation: 1.0,
            target: 30_000.0,
            max_level: 1.0,
            min_level: 0.0,
        }
    }
}

impl LevelingFilter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        self.peak *= 0.999;
        let abs = sample.abs();
        if abs > self.peak {
            self.peak = abs;
        }
        if self.peak == 0.0 {
            return 0.0;
        }

        let raw = (self.target / self.peak).clamp(self.min_level, self.max_level);
        // Instant attack + smooth release. The reference smooths both ways,
        // which lets fast transients (combustion pulses) clip before the gain
        // catches up. Attacking immediately keeps |out| <= target because
        // peak >= |sample| by construction.
        self.attenuation = if raw < self.attenuation {
            raw
        } else {
            0.9 * self.attenuation + 0.1 * raw
        };
        // Hard safety clamp: when the gain clamps (min/max level) the
        // target/peak ratio no longer bounds the output, so a pathological
        // input could otherwise reach ±Inf and hard-clip the final WAV.
        (sample * self.attenuation).clamp(-self.target, self.target)
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
    air_noise_lp: ButterworthLowPassFilter,
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
                "Synth params: air_noise={} air_fc={} jitter={} jitter_fc={}",
                params.air_noise,
                params.air_noise_frequency_cutoff,
                params.input_sample_noise,
                params.input_sample_noise_frequency_cutoff,
            );
        }
        for _ in 0..channel_count {
            let mut air = ButterworthLowPassFilter::new();
            air.set_cutoff_frequency(params.air_noise_frequency_cutoff, audio_rate as f32);
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
                air_noise_lp: air,
                input_dc: dc,
                input_antialias: aa_in,
                dry_delay: Vec::new(),
                dry_idx: 0,
            });
        }

        let mut antialiasing = ButterworthLowPassFilter::new();
        antialiasing.set_cutoff_frequency((audio_rate * 0.45) as f32, audio_rate as f32);

        let mut leveler = LevelingFilter::new();
        leveler.target = params.leveler_target;
        leveler.max_level = params.leveler_max_gain;
        leveler.min_level = params.leveler_min_gain;

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
    pub fn render_sample(&mut self, input: &[f32]) -> i16 {
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
            if f.abs() > self.dbg_max_f {
                self.dbg_max_f = f.abs();
            }
            if f_p.abs() > self.dbg_max_fp {
                self.dbg_max_fp = f_p.abs();
            }

            let noise = 2.0 * ch.jitter.next_random() - 1.0;
            let r = ch.air_noise_lp.process(noise);
            let r_mixed = p.air_noise * r + (1.0 - p.air_noise);
            if r.abs() > self.dbg_max_r {
                self.dbg_max_r = r.abs();
                self.dbg_r_idx = self.dbg_count;
            }

            let mut v_in = f_p * p.d_f_f_mix + f * r_mixed * (1.0 - p.d_f_f_mix);
            if v_in.is_subnormal() {
                v_in = 0.0;
            }
            if v_in.abs() > self.dbg_max_v_in {
                self.dbg_max_v_in = v_in.abs();
                self.dbg_v_idx = self.dbg_count;
                self.dbg_v_fp = f_p;
                self.dbg_v_f = f;
                self.dbg_v_r = r_mixed;
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
        let r_int = (leveled as f64).round();
        r_int.clamp(i16::MIN as f64, i16::MAX as f64) as i16
    }

    pub fn leveler_gain(&self) -> f32 {
        self.leveler.attenuation()
    }
}

// ---------------------------------------------------------------------------
// Offline multi-channel render
// ---------------------------------------------------------------------------

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
    let n_ch = channels_in.len();
    if n_ch == 0 || channels_in[0].is_empty() {
        return Vec::new();
    }
    let mut synth = Synthesizer::new(n_ch, input_rate, audio_rate, *params);
    for (i, ir) in irs.iter().enumerate() {
        if let Some((_, samples, volume)) = ir {
            synth.set_impulse_response(i, samples, *volume);
        }
    }

    let ratio = audio_rate / input_rate;
    let out_len = (channels_in[0].len() as f64 * ratio).round() as usize;
    let mut out = Vec::with_capacity(out_len);
    let mut inputs = vec![0.0f32; n_ch];

    for out_idx in 0..out_len {
        let target_input = out_idx as f64 / ratio;
        let i0 = target_input.floor() as usize;
        let frac = target_input - i0 as f64;

        for c in 0..n_ch {
            let len = channels_in[c].len();
            let s0 = channels_in[c][i0.min(len - 1)];
            let s1 = channels_in[c][(i0 + 1).min(len - 1)];
            inputs[c] = (s0 * (1.0 - frac) + s1 * frac) as f32;
        }

        out.push(synth.render_sample(&inputs));
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
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
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
    fn leveler_output_never_exceeds_target() {
        // Even a pathological (astronomically hot) input must be clamped to
        // the leveler target: previously the min/max gain clamp let such
        // inputs reach ±Inf, hard-clipping the rendered WAV at the i16 rails.
        let mut lvl = LevelingFilter::new();
        lvl.target = 30_000.0;
        lvl.min_level = 1e-5;
        lvl.max_level = 1.9;
        let mut max_out = 0.0f32;
        for i in 0..5000 {
            let x = if i < 10 { 1e12 } else { (i as f32) * 3.0 };
            let y = lvl.process(x);
            if y.abs() > max_out {
                max_out = y.abs();
            }
        }
        assert!(max_out.is_finite() && max_out <= 30_000.0, "max={max_out}");
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
        let out = render_offline(&[input], 10_000.0, 44_100.0, &AudioParameters::default(), &[None]);
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
            out.push(s.render_sample(&[x as f32]));
        }
        let peak = out.iter().map(|&s| (s as i32).abs()).max().unwrap();
        assert!(peak > 0, "peak={peak}");
        assert!(out.iter().any(|s| *s != 0));
    }

    #[test]
    fn render_offline_produces_samples() {
        let params = AudioParameters::default();
        let input: Vec<f64> = (0..1000)
            .map(|i| (i as f64 * 0.1).sin() * 5000.0)
            .collect();
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
            assert!(
                (got - expect).abs() < 1e-3,
                "i={i} got={got} exp={expect}"
            );
        }
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
}
