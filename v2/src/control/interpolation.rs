//! Control-rate to audio-rate linear telemetry interpolator.

use crate::control::types::RuntimeTelemetry;

/// Precomputes constant linear per-sample increments at control rate (`update`)
/// for the 8 continuous telemetry fields while passing discrete states
/// (`torque_sign`, `gear`, `shift_phase`, `rev_limiter_active`, `dt_seconds`)
/// through immediately at the sample boundary.
#[derive(Clone, Copy, Debug, Default)]
pub struct ControlInterpolator {
    target: RuntimeTelemetry,
    rendered: RuntimeTelemetry,
    remaining: usize,
    steps: [f32; 8],
}

impl ControlInterpolator {
    /// Validates `telemetry` and precomputes constant per-sample steps over `dt_seconds`.
    pub fn update(
        &mut self,
        telemetry: RuntimeTelemetry,
        sample_rate: u32,
    ) -> Result<RuntimeTelemetry, String> {
        let validated = telemetry.validate()?;
        self.target = validated;
        let remaining = (validated.dt_seconds * sample_rate as f32).round() as usize;
        self.remaining = remaining;
        if remaining > 0 {
            let from = self.rendered;
            let inv = 1.0 / remaining as f32;
            self.steps = [
                (validated.rpm - from.rpm) * inv,
                (validated.throttle - from.throttle) * inv,
                (validated.normalized_engine_load - from.normalized_engine_load) * inv,
                (validated.normalized_engine_torque - from.normalized_engine_torque) * inv,
                (validated.rpm_derivative - from.rpm_derivative) * inv,
                (validated.throttle_derivative - from.throttle_derivative) * inv,
                (validated.clutch_engagement - from.clutch_engagement) * inv,
                (validated.tc_cut_ratio - from.tc_cut_ratio) * inv,
            ];
        } else {
            self.steps = [0.0; 8];
        }
        Ok(self.target)
    }

    /// Advances interpolation by one audio sample, snapping cleanly on the final step
    /// to prevent floating-point drift.
    #[inline]
    pub fn next_sample(&mut self) -> RuntimeTelemetry {
        let target = self.target;
        if self.remaining > 0 {
            let steps = self.steps;
            self.rendered.rpm += steps[0];
            self.rendered.throttle += steps[1];
            self.rendered.normalized_engine_load += steps[2];
            self.rendered.normalized_engine_torque += steps[3];
            self.rendered.rpm_derivative += steps[4];
            self.rendered.throttle_derivative += steps[5];
            self.rendered.clutch_engagement += steps[6];
            self.rendered.tc_cut_ratio += steps[7];
            self.remaining -= 1;
            if self.remaining == 0 {
                self.rendered.rpm = target.rpm;
                self.rendered.throttle = target.throttle;
                self.rendered.normalized_engine_load = target.normalized_engine_load;
                self.rendered.normalized_engine_torque = target.normalized_engine_torque;
                self.rendered.rpm_derivative = target.rpm_derivative;
                self.rendered.throttle_derivative = target.throttle_derivative;
                self.rendered.clutch_engagement = target.clutch_engagement;
                self.rendered.tc_cut_ratio = target.tc_cut_ratio;
            }
        } else {
            self.rendered = target;
        }
        self.rendered.torque_sign = target.torque_sign;
        self.rendered.gear = target.gear;
        self.rendered.shift_phase = target.shift_phase;
        self.rendered.rev_limiter_active = target.rev_limiter_active;
        self.rendered.dt_seconds = target.dt_seconds;
        self.rendered
    }

    /// Latest validated control target.
    #[inline]
    pub fn target(&self) -> RuntimeTelemetry {
        self.target
    }

    /// Most recently rendered per-sample telemetry snapshot.
    #[inline]
    pub fn rendered(&self) -> RuntimeTelemetry {
        self.rendered
    }

    /// Number of samples left in the current interpolation ramp.
    #[inline]
    pub fn remaining(&self) -> usize {
        self.remaining
    }

    /// Precomputed per-sample increments for the 8 continuous fields.
    #[inline]
    pub fn steps(&self) -> [f32; 8] {
        self.steps
    }
}
