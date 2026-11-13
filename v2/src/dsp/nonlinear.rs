//! Allocation-free smooth nonlinearities, antiderivative anti-aliasing (`ADAA1`),
//! and cubic spline interpolation primitives.

/// Polynomial C1-continuous smooth minimum approximating `a.min(b)` with transition
/// radius `radius` (`k >= 0`).
///
/// Outside `|b - a| < radius`, returns `a.min(b)` exactly; across the transition band
/// both value and first derivative remain continuous (`C1`).
#[inline]
pub fn smooth_min(a: f32, b: f32, radius: f32) -> f32 {
    if radius <= 0.0 {
        return a.min(b);
    }
    let h = (0.5 + 0.5 * (b - a) / radius).clamp(0.0, 1.0);
    b * (1.0 - h) + a * h - radius * h * (1.0 - h)
}

/// Four-point Catmull-Rom cubic Hermite interpolation at fractional offset `t in [0, 1]`
/// between `y0` (`t = 0`) and `y1` (`t = 1`), using neighboring samples `y_m1` and `y2`.
#[inline]
pub fn hermite_4pt(y_m1: f32, y0: f32, y1: f32, y2: f32, t: f32) -> f32 {
    let c0 = y0;
    let c1 = 0.5 * (y1 - y_m1);
    let c2 = y_m1 - 2.5 * y0 + 2.0 * y1 - 0.5 * y2;
    let c3 = 0.5 * (y2 - y_m1) + 1.5 * (y0 - y1);
    ((c3 * t + c2) * t + c1) * t + c0
}

/// First-order Antiderivative Anti-Aliasing (`ADAA1`) evaluator for `tanh(drive * x) / norm`.
///
/// Uses analytical integration in `f64` (`F(x) = (|kx| + ln(1 + exp(-2|kx|)) - ln(2)) / (k * norm)`)
/// so the backward difference `(F(x_n) - F(x_{n-1})) / (x_n - x_{n-1})` suppresses foldover
/// harmonics at `1x` sample rate without heap allocations or oversampling latency.
#[derive(Clone, Copy, Debug)]
pub struct TanhAdaa1 {
    drive: f64,
    inv_norm: f64,
    inv_drive_norm: f64,
    x1: f64,
    f1: f64,
}

impl TanhAdaa1 {
    /// Threshold below which the analytical midpoint tangent replaces the difference quotient.
    const ILL_CONDITIONED_DX: f64 = 1.0e-5;

    /// Builds an ADAA1 saturator equivalent to `(x * drive).tanh() / norm`.
    pub fn new(drive: f32, norm: f32) -> Self {
        assert!(drive.is_finite() && drive > 0.0);
        assert!(norm.is_finite() && norm > 0.0);
        let drive_f64 = drive as f64;
        let norm_f64 = norm as f64;
        Self {
            drive: drive_f64,
            inv_norm: 1.0 / norm_f64,
            inv_drive_norm: 1.0 / (drive_f64 * norm_f64),
            x1: 0.0,
            f1: 0.0,
        }
    }

    #[inline]
    fn antiderivative(x: f64, drive: f64, inv_drive_norm: f64) -> f64 {
        let ax = (drive * x).abs();
        (ax + (-2.0 * ax).exp().ln_1p() - std::f64::consts::LN_2) * inv_drive_norm
    }

    /// Evaluates one ADAA1-filtered `tanh(drive * x) / norm` sample without allocations.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        let x = input as f64;
        if !x.is_finite() {
            self.reset();
            return 0.0;
        }
        let dx = x - self.x1;
        let f_curr = Self::antiderivative(x, self.drive, self.inv_drive_norm);
        let y = if dx.abs() >= Self::ILL_CONDITIONED_DX {
            (f_curr - self.f1) / dx
        } else {
            let mid = 0.5 * (x + self.x1);
            (self.drive * mid).tanh() * self.inv_norm
        };
        self.x1 = x;
        self.f1 = f_curr;
        y as f32
    }

    /// Resets memory back to zero state (silence-safe).
    #[inline]
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.f1 = 0.0;
    }
}
