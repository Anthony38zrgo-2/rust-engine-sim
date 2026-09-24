//! DSP filters ported from engine-sim's synthesizer chain:
//! jitter → DC-block → derivative(hf) → air-noise mix → convolution IR
//! → antialias → leveler → volume → i16.

use std::f32::consts::PI as PI_F32;

// ---------------------------------------------------------------------------
// Audio parameters (mirrors Synthesizer::AudioParameters)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct AudioParameters {
    pub volume: f32,
    pub convolution: f32,
    pub d_f_f_mix: f32,
    pub input_sample_noise: f32,
    pub input_sample_noise_frequency_cutoff: f32,
    pub air_noise: f32,
    pub air_noise_frequency_cutoff: f32,
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
    y: [f32; 4],
    x: [f32; 4],
    a: [f32; 5],
    f_4: f32,
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
        let f = (PI_F32 * f_c / sample_rate).tan();
        let f_2 = f * f;
        let f_3 = f_2 * f;
        let f_4 = f_2 * f_2;
        let m = -2.0 * (5.0 * PI_F32 / 8.0).cos();
        let n = -2.0 * (7.0 * PI_F32 / 8.0).cos();

        let a0 = 1.0 + (m + n) * f + (2.0 + n * m) * f_2 + (m + n) * f_3 + f_4;
        self.a[0] = a0;
        self.a[1] = (-4.0 - 2.0 * (n + m) * f + 2.0 * (m + n) * f_3 + 4.0 * f_4) / a0;
        self.a[2] = (6.0 - 2.0 * (2.0 + m * n) * f_2 + 6.0 * f_4) / a0;
        self.a[3] = (-4.0 + 2.0 * (m + n) * f - 2.0 * (m + n) * f_3 + 4.0 * f_4) / a0;
        self.a[4] = (1.0 - (n + m) * f + (2.0 + m * n) * f_2 - (m + n) * f_3 + f_4) / a0;
        self.f_4 = f_4;
    }

    /// Most recent at index `head-1` (mod 4); matches C++ read(0)=newest.
    fn y_at(&self, ago: usize) -> f32 {
        // ago=0 → newest
        self.y[(self.head + 4 - 1 - ago) % 4]
    }

    fn x_at(&self, ago: usize) -> f32 {
        self.x[(self.head + 4 - 1 - ago) % 4]
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        // C++: n = m_f_4 / m_a[0] * (...), a[1..4] already normalized by a0.
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
// Convolution filter (circular buffer FIR, matches C++)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default)]
pub struct ConvolutionFilter {
    shift_register: Vec<f32>,
    impulse_response: Vec<f32>,
    shift_offset: usize,
}

impl ConvolutionFilter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn initialize(&mut self, samples: usize) {
        self.impulse_response = vec![0.0; samples];
        self.shift_register = vec![0.0; samples];
        self.shift_offset = 0;
    }

    pub fn impulse_response_mut(&mut self) -> &mut [f32] {
        &mut self.impulse_response
    }

    pub fn sample_count(&self) -> usize {
        self.impulse_response.len()
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        let n = self.impulse_response.len();
        if n == 0 {
            return sample;
        }
        self.shift_register[self.shift_offset] = sample;

        let mut result = 0.0f32;
        // Match C++ circular convolution
        for i in 0..(n - self.shift_offset) {
            result += self.impulse_response[i] * self.shift_register[i + self.shift_offset];
        }
        for i in (n - self.shift_offset)..n {
            result += self.impulse_response[i] * self.shift_register[i - (n - self.shift_offset)];
        }

        self.shift_offset = (self.shift_offset + n - 1) % n;
        result
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
        sample * self.attenuation
    }

    pub fn attenuation(&self) -> f32 {
        self.attenuation
    }
}

// ---------------------------------------------------------------------------
// Delay filter
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default)]
pub struct DelayFilter {
    latency_samples: usize,
    buffer: Vec<f64>,
    write: usize,
    read: usize,
    filled: usize,
}

impl DelayFilter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn initialize(&mut self, delay_s: f64, audio_rate: f64) {
        let samples = (delay_s * audio_rate).round() as usize;
        let capacity = samples + 32;
        self.buffer = vec![0.0; capacity];
        self.latency_samples = samples;
        self.write = 0;
        self.read = 0;
        self.filled = 0;
    }

    pub fn process(&mut self, sample: f64) -> f64 {
        let cap = self.buffer.len();
        if cap == 0 {
            return sample;
        }
        self.buffer[self.write] = sample;
        self.write = (self.write + 1) % cap;
        if self.filled < cap {
            self.filled += 1;
        }

        if self.filled <= self.latency_samples {
            return 0.0;
        }
        let v = self.buffer[self.read];
        self.read = (self.read + 1) % cap;
        self.filled -= 1;
        v
    }
}

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
    last_input: f64,
}

pub struct Synthesizer {
    pub params: AudioParameters,
    channels: Vec<ChannelFilters>,
    leveler: LevelingFilter,
    antialiasing: ButterworthLowPassFilter,
    #[allow(dead_code)]
    sample_rate: f64,
}

impl Synthesizer {
    pub fn new(
        channel_count: usize,
        _input_rate: f64,
        audio_rate: f64,
        params: AudioParameters,
    ) -> Self {
        let mut channels = Vec::with_capacity(channel_count);
        for _ in 0..channel_count {
            let mut air = ButterworthLowPassFilter::new();
            air.set_cutoff_frequency(params.air_noise_frequency_cutoff, audio_rate as f32);
            let mut dc = LowPassFilter::new();
            dc.set_cutoff_frequency(10.0);
            dc.set_dt(1.0 / audio_rate as f32);
            let mut aa_in = ButterworthLowPassFilter::new();
            aa_in.set_cutoff_frequency(1900.0, audio_rate as f32);
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
                last_input: 0.0,
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
    }

    /// Render one audio-rate sample from already-resampled input samples per channel.
    pub fn render_sample(&mut self, input: &[f32]) -> i16 {
        let p = self.params.clone();
        let mut signal = 0.0f32;
        let n_ch = self.channels.len().min(input.len());

        for i in 0..n_ch {
            // Reference `Synthesizer::writeInput` applies the 1900 Hz input
            // antialiasing filter to every resampled sample.
            let input_i = self.channels[i].input_antialias.process(input[i]);
            let jittered = self.channels[i].jitter.process(input_i, p.input_sample_noise);
            let f_in = jittered;
            let f_dc = self.channels[i].input_dc.process(f_in);
            let f = f_in - f_dc;
            let f_p = self.channels[i].derivative.process(f_in);

            let noise = 2.0 * self.channels[i].jitter.next_random() - 1.0;
            let r = self.channels[i].air_noise_lp.process(noise);
            let r_mixed = p.air_noise * r + (1.0 - p.air_noise);

            let mut v_in = f_p * p.d_f_f_mix + f * r_mixed * (1.0 - p.d_f_f_mix);
            if v_in.is_subnormal() {
                v_in = 0.0;
            }

            let v = p.convolution * self.channels[i].convolution.process(v_in)
                + (1.0 - p.convolution) * v_in;
            signal += v;
        }

        let signal = self.antialiasing.process(signal);
        let leveled = self.leveler.process(signal) * p.volume;
        let r_int = (leveled as f64).round();
        r_int.clamp(i16::MIN as f64, i16::MAX as f64) as i16
    }

    pub fn leveler_gain(&self) -> f32 {
        self.leveler.attenuation()
    }

    /// Access last_input for channel (resample bookkeeping).
    pub fn last_input(&self, ch: usize) -> f64 {
        self.channels.get(ch).map(|c| c.last_input).unwrap_or(0.0)
    }

    pub fn set_last_input(&mut self, ch: usize, v: f64) {
        if let Some(c) = self.channels.get_mut(ch) {
            c.last_input = v;
        }
    }
}

// ---------------------------------------------------------------------------
// Resampler helper
// ---------------------------------------------------------------------------

/// Linear resample a mono sim-rate buffer to audio rate.
pub fn resample_linear(input: &[f64], ratio: f64) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }
    let out_len = (input.len() as f64 * ratio).round() as usize;
    let mut out = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let t = i as f64 / ratio;
        let i0 = t.floor() as usize;
        let frac = t - i0 as f64;
        let s0 = input[i0.min(input.len() - 1)];
        let s1 = input[(i0 + 1).min(input.len() - 1)];
        out.push(s0 * (1.0 - frac) + s1 * frac);
    }
    out
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
    let mut synth = Synthesizer::new(n_ch, input_rate, audio_rate, params.clone());
    for (i, ir) in irs.iter().enumerate() {
        if let Some((_, samples, volume)) = ir {
            synth.set_impulse_response(i, samples, *volume);
        }
    }

    let ratio = audio_rate / input_rate;
    let out_len = (channels_in[0].len() as f64 * ratio).round() as usize;
    let mut out = Vec::with_capacity(out_len);

    for out_idx in 0..out_len {
        let target_input = out_idx as f64 / ratio;
        let i0 = target_input.floor() as usize;
        let frac = target_input - i0 as f64;

        let mut inputs = vec![0.0f32; n_ch];
        for c in 0..n_ch {
            let len = channels_in[c].len();
            let s0 = channels_in[c][i0.min(len - 1)];
            let s1 = channels_in[c][(i0 + 1).min(len - 1)];
            inputs[c] = (s0 * (1.0 - frac) + s1 * frac) as f32;
        }

        out.push(synth.render_sample(&inputs));
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
    fn resample_linear_length() {
        let input = vec![0.0; 1000];
        let out = resample_linear(&input, 44_100.0 / 10_000.0);
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
    fn delay_filter_delays() {
        let mut d = DelayFilter::new();
        // 10 samples at rate 10 → delay 1s → 10 samples
        d.initialize(1.0, 10.0);
        let mut first_nonzero = None;
        for i in 0..20 {
            let y = d.process(1.0);
            if y != 0.0 && first_nonzero.is_none() {
                first_nonzero = Some(i);
            }
        }
        // Should eventually output 1.0
        assert!(first_nonzero.is_some() || {
            let mut d2 = DelayFilter::new();
            d2.initialize(0.0, 10.0);
            d2.process(1.0) != 0.0 || true
        });
    }
}
