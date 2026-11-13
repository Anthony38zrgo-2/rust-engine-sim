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
