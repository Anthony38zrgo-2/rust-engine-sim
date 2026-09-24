//! Tabulated 1D functions with sorted samples.
//!
//! Replaces the original C++ `Function` class. Supports:
//! - linear interpolation (default, cam lobe profiles)
//! - monotone cubic (PCHIP) interpolation (torque/VE curves, no overshoot)

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Interpolation {
    Linear,
    /// Piecewise cubic Hermite, monotone (Fritsch–Carlson). No overshoot.
    MonotoneCubic,
}

#[derive(Clone, Debug)]
pub struct Function {
    x: Vec<f64>,
    y: Vec<f64>,
    input_scale: f64,
    output_scale: f64,
    interpolation: Interpolation,
}

impl Default for Function {
    fn default() -> Self {
        Self::new()
    }
}

impl Function {
    pub fn new() -> Self {
        Self {
            x: Vec::new(),
            y: Vec::new(),
            input_scale: 1.0,
            output_scale: 1.0,
            interpolation: Interpolation::Linear,
        }
    }

    pub fn with_interpolation(interp: Interpolation) -> Self {
        Self {
            interpolation: interp,
            ..Self::new()
        }
    }

    /// Build from unsorted `(x, y)` pairs; sorted by `x` on construction.
    pub fn from_samples(samples: impl IntoIterator<Item = (f64, f64)>, interp: Interpolation) -> Self {
        let mut f = Self::with_interpolation(interp);
        for (x, y) in samples {
            f.add_sample(x, y);
        }
        f
    }

    pub fn set_input_scale(&mut self, s: f64) {
        self.input_scale = s;
    }

    pub fn set_output_scale(&mut self, s: f64) {
        self.output_scale = s;
    }

    pub fn set_interpolation(&mut self, interp: Interpolation) {
        self.interpolation = interp;
    }

    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    pub fn len(&self) -> usize {
        self.x.len()
    }

    /// Insert sample keeping `x` sorted. Replaces sample at equal `x`.
    pub fn add_sample(&mut self, x: f64, y: f64) {
        let idx = match self.x.binary_search_by(|probe| {
            probe.partial_cmp(&x).unwrap_or(std::cmp::Ordering::Equal)
        }) {
            Ok(i) => {
                self.y[i] = y;
                return;
            }
            Err(i) => i,
        };
        self.x.insert(idx, x);
        self.y.insert(idx, y);
    }

    pub fn domain(&self) -> (f64, f64) {
        match (self.x.first().copied(), self.x.last().copied()) {
            (Some(a), Some(b)) => (a, b),
            _ => (0.0, 0.0),
        }
    }

    pub fn range(&self) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for &y in &self.y {
            min = min.min(y);
            max = max.max(y);
        }
        if self.y.is_empty() {
            (0.0, 0.0)
        } else {
            (min, max)
        }
    }

    /// Sample the function. Clamps outside the domain.
    pub fn sample(&self, x: f64) -> f64 {
        if self.x.is_empty() {
            return 0.0;
        }
        let xs = x * self.input_scale;
        if xs <= self.x[0] {
            return self.y[0] * self.output_scale;
        }
        let last = self.x.len() - 1;
        if xs >= self.x[last] {
            return self.y[last] * self.output_scale;
        }

        let i = self.segment_index(xs);
        let v = match self.interpolation {
            Interpolation::Linear => {
                let (x0, x1) = (self.x[i], self.x[i + 1]);
                let (y0, y1) = (self.y[i], self.y[i + 1]);
                let t = (xs - x0) / (x1 - x0);
                y0 + t * (y1 - y0)
            }
            Interpolation::MonotoneCubic => self.pchip_sample(xs, i),
        };
        v * self.output_scale
    }

    /// Periodic sample: wraps `x` into `[x0, x0 + period)` using domain start.
    pub fn sample_periodic(&self, x: f64, period: f64) -> f64 {
        if self.x.is_empty() || period <= 0.0 {
            return self.sample(x);
        }
        let x0 = self.x[0];
        let mut xs = (x - x0) % period;
        if xs < 0.0 {
            xs += period;
        }
        self.sample(x0 + xs)
    }

    fn segment_index(&self, xs: f64) -> usize {
        // Largest i such that x[i] <= xs, with x[i+1] > xs
        let mut lo = 0;
        let mut hi = self.x.len() - 1;
        while lo + 1 < hi {
            let mid = (lo + hi) / 2;
            if self.x[mid] <= xs {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    fn pchip_sample(&self, xs: f64, i: usize) -> f64 {
        let n = self.x.len();
        let m = pchip_tangents(&self.x, &self.y);
        let (x0, x1) = (self.x[i], self.x[i + 1]);
        let (y0, y1) = (self.y[i], self.y[i + 1]);
        let h = x1 - x0;
        let t = (xs - x0) / h;
        let t2 = t * t;
        let t3 = t2 * t;
        let m0 = m[i.min(n - 1)];
        let m1 = m[(i + 1).min(n - 1)];
        // Hermite basis
        (2.0 * t3 - 3.0 * t2 + 1.0) * y0
            + (t3 - 2.0 * t2 + t) * h * m0
            + (-2.0 * t3 + 3.0 * t2) * y1
            + (t3 - t2) * h * m1
    }
}

/// Fritsch–Carlson monotone tangents for all points.
fn pchip_tangents(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut m = vec![0.0; n];
    if n < 2 {
        return m;
    }
    if n == 2 {
        let d = (y[1] - y[0]) / (x[1] - x[0]);
        return vec![d, d];
    }

    let mut delta = vec![0.0; n - 1];
    for i in 0..n - 1 {
        delta[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
    }

    for i in 1..n - 1 {
        if delta[i - 1] * delta[i] <= 0.0 {
            m[i] = 0.0;
        } else {
            let h0 = x[i] - x[i - 1];
            let h1 = x[i + 1] - x[i];
            let w1 = 2.0 * h1 + h0;
            let w2 = h1 + 2.0 * h0;
            m[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
        }
    }

    // Endpoints: secant slope, clamped to preserve monotonicity of end intervals
    m[0] = delta[0];
    m[n - 1] = delta[n - 2];
    m
}

/// Harmonic cam lobe profile generator, C++-compatible semantics.
///
/// Mirrors `GenerateHarmonicCamLobeNode` from the reference implementation:
///
/// ```text
/// angle   = duration_at_ref / 4          (cam half-angle at reference lift)
/// s       = (2 * ref_lift / lift)^(1/gamma) - 1
/// k       = acos(s) / angle
/// extents = pi / k
/// lift(x) = lift * (0.5 + 0.5 * cos(k * x))^gamma   for |x| <= extents, else 0
/// ```
///
/// The profile is centered at x = 0 (peak) so that the cam peak occurs when
/// `(crank + advance) / 2 + lobe_centerline == 0 (mod 2π)`, matching the
/// reference `Camshaft::valveLift` convention. `duration_at_ref_deg` is the
/// full opening duration (crank degrees) measured at `ref_lift_m`.
///
/// `Function::sample` clamps outside the domain, and the endpoints are
/// zero-lift, so the base circle is reproduced automatically.
pub fn harmonic_lobe_profile_at_ref(
    duration_at_ref_deg: f64,
    ref_lift_m: f64,
    lift_m: f64,
    gamma: f64,
    steps: usize,
) -> Function {
    let mut f = Function::new();
    if lift_m <= 0.0 || gamma <= 0.0 || duration_at_ref_deg <= 0.0 {
        return f;
    }
    let gamma = gamma.max(1e-3);
    let angle = (duration_at_ref_deg.to_radians() * 0.25).max(1e-9);
    let s = ((2.0 * ref_lift_m / lift_m).powf(1.0 / gamma) - 1.0).clamp(-1.0, 1.0);
    let k = (s.acos() / angle).max(1e-9);
    let extents = std::f64::consts::PI / k;

    let total = steps.max(8);
    for i in 0..=total {
        let x = -extents + 2.0 * extents * (i as f64 / total as f64);
        let lift = if x.abs() >= extents {
            0.0
        } else {
            lift_m * (0.5 + 0.5 * (k * x).cos()).powf(gamma)
        };
        f.add_sample(x, lift);
    }
    f
}

/// Harmonic lobe profile specified by its **full zero-lift duration** in crank
/// degrees (intuitive JSON semantics). Converts to the reference-lift duration
/// and delegates to [`harmonic_lobe_profile_at_ref`].
pub fn harmonic_lobe_profile(
    full_duration_deg: f64,
    ref_lift_m: f64,
    lift_m: f64,
    gamma: f64,
    steps: usize,
) -> Function {
    if lift_m <= 0.0 || gamma <= 0.0 || full_duration_deg <= 0.0 {
        return Function::new();
    }
    let gamma = gamma.max(1e-3);
    let s = ((2.0 * ref_lift_m / lift_m).powf(1.0 / gamma) - 1.0).clamp(-1.0, 1.0);
    // k = pi / half_full and angle = acos(s) / k  =>  duration_at_ref = acos(s) * full / pi
    let duration_at_ref_deg = s.acos() * full_duration_deg / std::f64::consts::PI;
    harmonic_lobe_profile_at_ref(duration_at_ref_deg, ref_lift_m, lift_m, gamma, steps)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_interpolation_midpoint() {
        let f = Function::from_samples([(0.0, 0.0), (10.0, 100.0)], Interpolation::Linear);
        assert!((f.sample(5.0) - 50.0).abs() < 1e-9);
    }

    #[test]
    fn clamps_outside_domain() {
        let f = Function::from_samples([(0.0, 1.0), (1.0, 2.0)], Interpolation::Linear);
        assert!((f.sample(-5.0) - 1.0).abs() < 1e-12);
        assert!((f.sample(5.0) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn monotone_cubic_no_overshoot() {
        // Rising then flat — must not exceed max y
        let f = Function::from_samples(
            [(0.0, 0.0), (1.0, 1.0), (2.0, 1.0), (3.0, 1.0)],
            Interpolation::MonotoneCubic,
        );
        for i in 0..=300 {
            let x = i as f64 * 0.01;
            let y = f.sample(x);
            assert!((-1e-9..=1.0 + 1e-9).contains(&y), "y={y} at x={x}");
        }
    }

    #[test]
    fn torque_curve_rs24_shape() {
        let pts = [
            (6000.0, 240.0),
            (8000.0, 285.0),
            (10000.0, 315.0),
            (12000.0, 335.0),
            (14000.0, 348.0),
            (15000.0, 351.0),
            (16000.0, 349.0),
            (17000.0, 345.0),
            (18000.0, 339.0),
            (18500.0, 335.0),
            (19000.0, 330.0),
        ];
        let f = Function::from_samples(pts, Interpolation::MonotoneCubic);
        assert!((f.sample(15000.0) - 351.0).abs() < 1e-6);
        assert!(f.sample(13000.0) > 315.0 && f.sample(13000.0) < 348.0);
        // No overshoot beyond peak
        for rpm in (6000..=19000).step_by(100) {
            assert!(f.sample(rpm as f64) <= 351.0 + 1e-6);
        }
    }

    #[test]
    fn sort_insert_replaces_duplicate() {
        let mut f = Function::new();
        f.add_sample(1.0, 10.0);
        f.add_sample(0.0, 5.0);
        f.add_sample(1.0, 20.0);
        assert_eq!(f.len(), 2);
        assert!((f.sample(1.0) - 20.0).abs() < 1e-12);
    }
}
