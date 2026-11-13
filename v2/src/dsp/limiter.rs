//! Master output soft-safety limiter matching the GF509 knee and ceiling.

pub const MASTER_LIMITER_KNEE: f32 = 0.92;
pub const MASTER_LIMITER_HEADROOM: f32 = 0.075;
pub const MASTER_LIMITER_CEILING: f32 = 0.995;

/// Applies the transparent safety limiter above [`MASTER_LIMITER_KNEE`].
/// Returns `(limited_sample, reduction_db)`. Below the knee, output equals `pre`
/// identically and gain reduction is `0.0`.
#[inline]
pub fn limit_master(pre: f32) -> (f32, f32) {
    let abs = pre.abs();
    if abs <= MASTER_LIMITER_KNEE {
        (pre, 0.0)
    } else {
        let excess = abs - MASTER_LIMITER_KNEE;
        let limited_abs = MASTER_LIMITER_KNEE
            + MASTER_LIMITER_HEADROOM * (excess / MASTER_LIMITER_HEADROOM).tanh();
        let limited = pre.signum() * limited_abs.min(MASTER_LIMITER_CEILING);
        let reduction_db = 20.0 * (limited.abs() / abs.max(1.0e-12)).log10();
        (limited, reduction_db)
    }
}

/// First-order Antiderivative Anti-Aliasing (`ADAA1`) wrapper for [`limit_master`].
///
/// Identical to `limit_master` when staying inside the linear region `[-0.92, 0.92]`,
/// and suppresses Nyquist foldover harmonics whenever transient peaks cross the knee.
#[derive(Clone, Copy, Debug, Default)]
pub struct MasterLimiterAdaa1 {
    x1: f64,
    f1: f64,
}

impl MasterLimiterAdaa1 {
    const ILL_CONDITIONED_DX: f64 = 1.0e-5;

    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    fn antiderivative(x: f64) -> f64 {
        let k = MASTER_LIMITER_KNEE as f64;
        let h = MASTER_LIMITER_HEADROOM as f64;
        let u = x.abs();
        if u <= k {
            0.5 * u * u
        } else {
            let d = u - k;
            let excess_scaled = d / h;
            0.5 * k * k
                + k * d
                + h * (d + h * ((-2.0 * excess_scaled).exp().ln_1p() - std::f64::consts::LN_2))
        }
    }

    /// Evaluates one ADAA1-filtered safety-limited sample and reports instantaneous gain reduction (dB).
    #[inline]
    pub fn process(&mut self, pre: f32) -> (f32, f32) {
        let x = pre as f64;
        if !x.is_finite() {
            self.x1 = 0.0;
            self.f1 = 0.0;
            return (0.0, 0.0);
        }
        let f_curr = Self::antiderivative(x);
        let dx = x - self.x1;
        let y = if dx.abs() >= Self::ILL_CONDITIONED_DX {
            (f_curr - self.f1) / dx
        } else {
            let mid = 0.5 * (x + self.x1);
            let abs = mid.abs();
            let k = MASTER_LIMITER_KNEE as f64;
            let h = MASTER_LIMITER_HEADROOM as f64;
            if abs <= k {
                mid
            } else {
                mid.signum() * (k + h * ((abs - k) / h).tanh())
            }
        };
        self.x1 = x;
        self.f1 = f_curr;
        let limited = (y as f32).clamp(-MASTER_LIMITER_CEILING, MASTER_LIMITER_CEILING);
        let reduction_db = if pre.abs() > MASTER_LIMITER_KNEE {
            20.0 * (limited.abs() / pre.abs().max(1.0e-12)).log10()
        } else {
            0.0
        };
        (limited, reduction_db)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn master_limiter_adaa1_matches_linear_region_and_reduces_hf_alias() {
        let mut adaa = MasterLimiterAdaa1::new();
        for x in [-0.9, -0.4, 0.0, 0.35, 0.91] {
            // Settle 2 samples to traverse ADAA1's half-sample linear region delay (x_n == x_{n-1}).
            let _ = adaa.process(x);
            let (y, db) = adaa.process(x);
            let (expected, expected_db) = limit_master(x);
            assert!(
                (y - expected).abs() < 1.0e-5,
                "linear x={x}: {y} vs {expected}"
            );
            assert_eq!(db, expected_db);
        }

        let mut adaa_sat = MasterLimiterAdaa1::new();
        let mut naive_hf_diff = 0.0f64;
        let mut adaa_hf_diff = 0.0f64;
        let mut prev_n = 0.0f32;
        let mut prev_a = 0.0f32;
        for n in 0..8_000 {
            let x = 2.4 * (std::f32::consts::TAU * 13_700.0 * n as f32 / 48_000.0).sin();
            let (yn, _) = limit_master(x);
            let (ya, _) = adaa_sat.process(x);
            assert!(ya.is_finite());
            assert!(ya.abs() <= MASTER_LIMITER_CEILING + 1.0e-6);
            if n > 500 {
                naive_hf_diff += (yn as f64 - prev_n as f64).powi(2);
                adaa_hf_diff += (ya as f64 - prev_a as f64).powi(2);
            }
            prev_n = yn;
            prev_a = ya;
        }
        assert!(adaa_hf_diff < naive_hf_diff * 0.9);
    }
}
