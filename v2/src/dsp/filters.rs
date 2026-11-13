//! Basic allocation-free IIR/time-domain filter primitives shared across synthesis,
//! acoustics, samples, transmission, and offline rendering.

use std::f32::consts::{PI, TAU};

/// First-order DC-blocking high-pass filter.
#[derive(Clone, Copy, Debug)]
pub struct DcBlocker {
    x1: f32,
    y1: f32,
    r: f32,
}

impl DcBlocker {
    /// Creates a DC blocker with pole coefficient `exp(-2*pi*cutoff_hz / sample_rate)`.
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        Self {
            x1: 0.0,
            y1: 0.0,
            r: (-TAU * cutoff_hz / sample_rate).exp(),
        }
    }

    /// Processes a single audio sample without allocations.
    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = x - self.x1 + self.r * self.y1;
        self.x1 = x;
        self.y1 = y;
        y
    }
}

/// First-order exponential smoothing low-pass filter with optional transparent bypass.
#[derive(Clone, Copy, Debug)]
pub struct OnePoleLowPass {
    alpha: f32,
    state: f32,
    bypassed: bool,
}

impl OnePoleLowPass {
    /// Builds a low-pass filter asserting that `cutoff_hz` lies strictly inside
    /// `(0, 0.48 * sample_rate)`.
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate * 0.48);
        Self::new_unclamped(cutoff_hz, sample_rate)
    }

    /// Builds a low-pass filter using the standard `1 - exp(-2*pi*f/sr)` coefficient
    /// without asserting the `0.48 * sample_rate` guard (used by fixed-cutoff voice
    /// and CLI post-filter stages whose cutoffs are validated externally).
    pub fn new_unclamped(cutoff_hz: f32, sample_rate: f32) -> Self {
        Self {
            alpha: 1.0 - (-TAU * cutoff_hz / sample_rate).exp(),
            state: 0.0,
            bypassed: false,
        }
    }

    /// Transparent passthrough instance whose `process` returns the input unchanged.
    pub fn bypassed() -> Self {
        Self {
            alpha: 1.0,
            state: 0.0,
            bypassed: true,
        }
    }

    /// Returns a bypassed stage when `cutoff_hz >= 0.48 * sample_rate`, otherwise
    /// constructs an active low-pass filter.
    pub fn lowpass_or_bypass(cutoff_hz: f32, sample_rate: f32) -> Self {
        if cutoff_hz >= sample_rate * 0.48 {
            Self::bypassed()
        } else {
            Self::new(cutoff_hz, sample_rate)
        }
    }

    /// Advances the filter by one sample.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        if self.bypassed {
            return input;
        }
        self.state += self.alpha * (input - self.state);
        self.state
    }
}

/// Three cascaded one-pole low-pass stages providing a smooth, non-resonant
/// `-18 dB/oct` rolloff.
#[derive(Clone, Copy, Debug)]
pub struct ThreePoleLowPass {
    p1: OnePoleLowPass,
    p2: OnePoleLowPass,
    p3: OnePoleLowPass,
}

impl ThreePoleLowPass {
    /// Constructs a 3-stage cascade at `cutoff_hz`.
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        Self {
            p1: OnePoleLowPass::new_unclamped(cutoff_hz, sample_rate),
            p2: OnePoleLowPass::new_unclamped(cutoff_hz, sample_rate),
            p3: OnePoleLowPass::new_unclamped(cutoff_hz, sample_rate),
        }
    }

    /// Processes a single sample through the three stages in series.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        self.p3.process(self.p2.process(self.p1.process(input)))
    }
}

/// Band-pass turbulence/noise shaper built from a two-pole high-cut cascade minus
/// a one-pole low-cut envelope follower.
#[derive(Clone, Copy, Debug)]
pub struct BandPassNoise {
    low_cut: OnePoleLowPass,
    high_cut_a: OnePoleLowPass,
    high_cut_b: OnePoleLowPass,
}

impl BandPassNoise {
    /// Constructs the band-pass noise filter for `(low_hz, high_hz)` at `sample_rate`.
    pub fn new(low_hz: f32, high_hz: f32, sample_rate: f32) -> Self {
        Self {
            low_cut: OnePoleLowPass::new(low_hz, sample_rate),
            high_cut_a: OnePoleLowPass::new(high_hz, sample_rate),
            high_cut_b: OnePoleLowPass::new(high_hz, sample_rate),
        }
    }

    /// Filters a single noise sample.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        let below_high = self.high_cut_b.process(self.high_cut_a.process(input));
        below_high - self.low_cut.process(below_high)
    }
}

/// Transposed Direct Form II biquad filter with RBJ coefficient builders and
/// denormal/non-finite state protection.
#[derive(Clone, Copy, Debug, Default)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    fn from_coefficients(b0: f32, b1: f32, b2: f32, a0: f32, a1: f32, a2: f32) -> Self {
        let inverse = 1.0 / a0;
        Self {
            b0: b0 * inverse,
            b1: b1 * inverse,
            b2: b2 * inverse,
            a1: a1 * inverse,
            a2: a2 * inverse,
            z1: 0.0,
            z2: 0.0,
        }
    }

    /// RBJ second-order high-pass filter.
    pub fn highpass(cutoff_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = TAU * cutoff_hz / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q);
        Self::from_coefficients(
            (1.0 + cos_w0) * 0.5,
            -(1.0 + cos_w0),
            (1.0 + cos_w0) * 0.5,
            1.0 + alpha,
            -2.0 * cos_w0,
            1.0 - alpha,
        )
    }

    /// RBJ second-order low-pass filter.
    pub fn lowpass(cutoff_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = TAU * cutoff_hz / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q);
        Self::from_coefficients(
            (1.0 - cos_w0) * 0.5,
            1.0 - cos_w0,
            (1.0 - cos_w0) * 0.5,
            1.0 + alpha,
            -2.0 * cos_w0,
            1.0 - alpha,
        )
    }

    /// RBJ high shelf: flat below `cutoff_hz`, `gain_db` above it.
    pub fn high_shelf(cutoff_hz: f32, q: f32, gain_db: f32, sample_rate: f32) -> Self {
        let a = 10.0f32.powf(gain_db / 40.0);
        let w0 = TAU * cutoff_hz / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q);
        let two_sqrt_a_alpha = 2.0 * a.sqrt() * alpha;
        Self::from_coefficients(
            a * ((a + 1.0) + (a - 1.0) * cos_w0 + two_sqrt_a_alpha),
            -2.0 * a * ((a - 1.0) + (a + 1.0) * cos_w0),
            a * ((a + 1.0) + (a - 1.0) * cos_w0 - two_sqrt_a_alpha),
            (a + 1.0) - (a - 1.0) * cos_w0 + two_sqrt_a_alpha,
            2.0 * ((a - 1.0) - (a + 1.0) * cos_w0),
            (a + 1.0) - (a - 1.0) * cos_w0 - two_sqrt_a_alpha,
        )
    }

    /// RBJ peaking EQ: `gain_db` at `cutoff_hz`, unity far from it.
    pub fn peaking(cutoff_hz: f32, q: f32, gain_db: f32, sample_rate: f32) -> Self {
        let frequency = cutoff_hz.clamp(1.0, sample_rate * 0.49);
        let a = 10.0f32.powf(gain_db / 40.0);
        let w0 = TAU * frequency / sample_rate;
        let (sin_w0, cos_w0) = w0.sin_cos();
        let alpha = sin_w0 / (2.0 * q.max(0.05));
        Self::from_coefficients(
            1.0 + alpha * a,
            -2.0 * cos_w0,
            1.0 - alpha * a,
            1.0 + alpha / a,
            -2.0 * cos_w0,
            1.0 - alpha / a,
        )
    }

    /// Processes a single sample, flushing denormals and resetting if non-finite.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        let output = self.b0 * input + self.z1;
        self.z1 = self.b1 * input - self.a1 * output + self.z2;
        self.z2 = self.b2 * input - self.a2 * output;
        if self.z1.abs() < 1e-30 {
            self.z1 = 0.0;
        }
        if self.z2.abs() < 1e-30 {
            self.z2 = 0.0;
        }
        if output.is_finite() {
            output
        } else {
            self.reset();
            0.0
        }
    }

    /// Clears internal filter delay memory.
    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
}

/// Time-varying `Q = 10` notch filter used to attenuate tracking sample resonances
/// without coloring outside the notch.
#[derive(Clone, Copy, Debug, Default)]
pub struct TrackingNotch {
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl TrackingNotch {
    /// Creates an initialized tracking notch with zero delay history.
    pub fn new() -> Self {
        Self::default()
    }

    /// Applies the notch at `frequency_hz` blended by `wet`, bypassing cleanly when
    /// `wet <= 0.0` or `frequency_hz >= 0.45 * sample_rate`.
    #[inline]
    pub fn process(&mut self, input: f32, frequency_hz: f32, sample_rate: f32, wet: f32) -> f32 {
        if wet <= 0.0 || frequency_hz >= sample_rate * 0.45 {
            return input;
        }
        let omega = TAU * frequency_hz / sample_rate;
        let alpha = omega.sin() / (2.0 * 10.0);
        let inverse_a0 = 1.0 / (1.0 + alpha);
        let b0 = inverse_a0;
        let b1 = -2.0 * omega.cos() * inverse_a0;
        let b2 = inverse_a0;
        let a1 = b1;
        let a2 = (1.0 - alpha) * inverse_a0;
        let filtered = b0 * input + b1 * self.x1 + b2 * self.x2 - a1 * self.y1 - a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = filtered;
        input + (filtered - input) * wet
    }
}

/// Topology-preserving (Cytomic / TPT) second-order state-variable filter.
///
/// Unconditionally stable under rapid per-sample `cutoff_hz` and `q` modulation,
/// clamped safely below Nyquist (`0.45 * sample_rate`).
#[derive(Clone, Copy, Debug)]
pub struct StateVariableFilter {
    sample_rate: f32,
    ic1eq: f32,
    ic2eq: f32,
}

impl StateVariableFilter {
    /// Initializes an SVF stage at `sample_rate` with zero integrator state.
    pub fn new(sample_rate: f32) -> Self {
        assert!(sample_rate.is_finite() && sample_rate > 0.0);
        Self {
            sample_rate,
            ic1eq: 0.0,
            ic2eq: 0.0,
        }
    }

    #[inline]
    fn step(&mut self, input: f32, cutoff_hz: f32, q: f32) -> (f32, f32, f32) {
        let fc = cutoff_hz.clamp(20.0, self.sample_rate * 0.45);
        let k = 1.0 / q.max(0.1);
        let g = (PI * fc / self.sample_rate).tan();
        let a1 = 1.0 / (1.0 + g * (g + k));
        let a2 = g * a1;
        let a3 = g * a2;
        let v3 = input - self.ic2eq;
        let v1 = a1 * self.ic1eq + a2 * v3;
        let v2 = self.ic2eq + a2 * self.ic1eq + a3 * v3;
        self.ic1eq = 2.0 * v1 - self.ic1eq;
        self.ic2eq = 2.0 * v2 - self.ic2eq;
        if self.ic1eq.abs() < 1.0e-30 {
            self.ic1eq = 0.0;
        }
        if self.ic2eq.abs() < 1.0e-30 {
            self.ic2eq = 0.0;
        }
        if !self.ic1eq.is_finite() || !self.ic2eq.is_finite() {
            self.reset();
            return (0.0, 0.0, 0.0);
        }
        (v1, v2, k)
    }

    /// Evaluates the 0 dB-normalized band-pass output (`k * v1`) at modulated `cutoff_hz`.
    #[inline]
    pub fn process_bandpass(&mut self, input: f32, cutoff_hz: f32, q: f32) -> f32 {
        let (v1, _, k) = self.step(input, cutoff_hz, q);
        k * v1
    }

    /// Evaluates the -12 dB/oct low-pass output (`v2`) at modulated `cutoff_hz`.
    #[inline]
    pub fn process_lowpass(&mut self, input: f32, cutoff_hz: f32, q: f32) -> f32 {
        let (_, v2, _) = self.step(input, cutoff_hz, q);
        v2
    }

    /// Clears integrator memory.
    #[inline]
    pub fn reset(&mut self) {
        self.ic1eq = 0.0;
        self.ic2eq = 0.0;
    }
}

/// Unity-magnitude Schroeder first-order all-pass diffuser for airborne comb smoothing.
#[derive(Clone, Debug)]
pub struct FirstOrderAllPass {
    delay: Vec<f32>,
    cursor: usize,
    g: f32,
}

impl FirstOrderAllPass {
    /// Preallocates an all-pass diffuser of `delay_ms` at `sample_rate` with feedback `g` in `(-1, 1)`.
    pub fn new(delay_ms: f32, g: f32, sample_rate: f32) -> Self {
        let delay_samples = (delay_ms * 0.001 * sample_rate).round().max(1.0) as usize;
        Self {
            delay: vec![0.0; delay_samples],
            cursor: 0,
            g: g.clamp(-0.95, 0.95),
        }
    }

    /// Processes one sample with flat unity magnitude across all frequencies.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        let delayed = self.delay[self.cursor];
        let v = input - self.g * delayed;
        self.delay[self.cursor] = if v.abs() < 1.0e-30 { 0.0 } else { v };
        self.cursor += 1;
        if self.cursor == self.delay.len() {
            self.cursor = 0;
        }
        delayed + self.g * v
    }
}

/// Fourth-order Butterworth low-pass cascade (`-24 dB/oct` stopband slope) using two
/// inline biquads for zero-allocation oversampled decimation.
#[derive(Clone, Copy, Debug)]
pub struct ButterworthLp4 {
    stages: [Biquad; 2],
}

impl ButterworthLp4 {
    /// Q factors `1 / (2 * cos((2k + 1) * pi / 8))`, low-Q section first for headroom.
    const STAGE_QS: [f32; 2] = [0.541_196_1, 1.306_563];

    /// Builds a 4th-order Butterworth low-pass filter at `cutoff_hz` and `sample_rate`.
    pub fn new(cutoff_hz: f32, sample_rate: f32) -> Self {
        let clamped_rate = sample_rate.max(1.0);
        let clamped_cutoff = cutoff_hz.clamp(1.0, clamped_rate * 0.45);
        Self {
            stages: Self::STAGE_QS.map(|q| Biquad::lowpass(clamped_cutoff, q, clamped_rate)),
        }
    }

    /// Processes a single sample through both cascaded biquad sections.
    #[inline]
    pub fn process(&mut self, mut sample: f32) -> f32 {
        for stage in &mut self.stages {
            sample = stage.process(sample);
        }
        sample
    }

    /// Clears internal filter memory.
    pub fn reset(&mut self) {
        for stage in &mut self.stages {
            stage.reset();
        }
    }
}
