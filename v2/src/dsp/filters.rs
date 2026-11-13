//! Basic allocation-free IIR/time-domain filter primitives shared across synthesis,
//! acoustics, samples, transmission, and offline rendering.

use std::f32::consts::TAU;

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
