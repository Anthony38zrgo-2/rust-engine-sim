//! Equal-power zone and variant crossfade curves using preallocated scratch buffers.

use std::f32::consts::FRAC_PI_2;

pub const ZONE_COUNT: usize = 3;

/// Sanitized control inputs for a single sample-layer render step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SampleLayerInput {
    pub rpm: f32,
    pub throttle: f32,
    pub load: f32,
    /// Signed engine torque normalized to the physical maximum.
    pub normalized_engine_torque: f32,
    /// Physical clutch engagement; gates retention-only off audio.
    pub clutch_engagement: f32,
    pub crank_phase_deg: f32,
}

impl SampleLayerInput {
    /// Validates finite control ranges (`rpm` 0..=25_000, normalized fields in bounds).
    pub fn validate(self) -> Result<Self, String> {
        if !self.rpm.is_finite() || !(0.0..=25_000.0).contains(&self.rpm) {
            return Err(format!("sample-layer rpm out of range: {}", self.rpm));
        }
        if !self.throttle.is_finite() || !(0.0..=1.0).contains(&self.throttle) {
            return Err(format!(
                "sample-layer throttle out of range: {}",
                self.throttle
            ));
        }
        if !self.load.is_finite() || !(0.0..=1.0).contains(&self.load) {
            return Err(format!("sample-layer load out of range: {}", self.load));
        }
        if !self.normalized_engine_torque.is_finite()
            || !(-1.0..=1.0).contains(&self.normalized_engine_torque)
        {
            return Err(format!(
                "sample-layer normalized torque out of range: {}",
                self.normalized_engine_torque
            ));
        }
        if !self.clutch_engagement.is_finite() || !(0.0..=1.0).contains(&self.clutch_engagement) {
            return Err(format!(
                "sample-layer clutch engagement out of range: {}",
                self.clutch_engagement
            ));
        }
        if !self.crank_phase_deg.is_finite() {
            return Err("sample-layer crank phase must be finite".into());
        }
        Ok(self)
    }
}

/// Reusable scratch buffers preallocated once when constructing a [`crate::samples::SampleBankPlayer`].
#[derive(Clone, Debug)]
pub struct CrossfadeScratch {
    pub on_weights: Vec<f32>,
    pub off_zone_weights: Vec<f32>,
    pub variant_weights: Vec<f32>,
}

impl CrossfadeScratch {
    /// Reserves exact capacity needed for all ON zones, OFF zones, and largest variant group.
    pub fn with_capacities(
        on_zone_count: usize,
        off_zone_count: usize,
        max_variant_group_len: usize,
    ) -> Self {
        Self {
            on_weights: vec![0.0; on_zone_count],
            off_zone_weights: vec![0.0; off_zone_count],
            variant_weights: vec![0.0; max_variant_group_len.max(1)],
        }
    }
}

/// Off-throttle overrun & clutch-gated retention weight.
#[inline]
pub fn off_throttle_weight(input: SampleLayerInput, gain: f32) -> f32 {
    let overrun = ((0.30 - input.throttle) / 0.30).clamp(0.0, 1.0);
    let low_load = ((0.40 - input.load) / 0.40).clamp(0.0, 1.0);
    let retention = (-input.normalized_engine_torque).max(0.0) * input.clutch_engagement;
    let low_load_engaged = low_load * input.clutch_engagement;
    overrun * low_load_engaged.max(retention) * gain
}

/// Crossfade boundaries `[lo, hi]` for `N` sorted zone anchors.
pub fn zone_boundaries(anchors: &[f32]) -> Vec<(f32, f32)> {
    let count = anchors.len();
    let mut bounds = Vec::with_capacity(count);
    for index in 0..count {
        let lo = if index == 0 {
            if count > 1 {
                anchors[0] - (anchors[1] - anchors[0]) * 0.5
            } else {
                f32::NEG_INFINITY
            }
        } else {
            (anchors[index - 1] + anchors[index]) * 0.5
        };
        let hi = if index + 1 == count {
            if count > 1 {
                anchors[count - 1] + (anchors[count - 1] - anchors[count - 2]) * 0.5
            } else {
                f32::INFINITY
            }
        } else {
            (anchors[index] + anchors[index + 1]) * 0.5
        };
        bounds.push((lo, hi));
    }
    bounds
}

/// Generalized neighbor equal-power crossfade written directly into preallocated `out`.
#[inline]
pub fn multi_zone_weights_into(rpm: f32, anchors: &[f32], out: &mut Vec<f32>) {
    let count = anchors.len();
    out.clear();
    out.resize(count, 0.0);
    if count == 0 {
        return;
    }
    if count == 1 || rpm <= anchors[0] {
        out[0] = 1.0;
        return;
    }
    if rpm >= anchors[count - 1] {
        out[count - 1] = 1.0;
        return;
    }
    for index in 0..count - 1 {
        if rpm >= anchors[index] && rpm < anchors[index + 1] {
            let span = (anchors[index + 1] - anchors[index]).max(1.0e-3);
            let t = ((rpm - anchors[index]) / span).clamp(0.0, 1.0);
            let theta = t * FRAC_PI_2;
            out[index] = theta.cos();
            out[index + 1] = theta.sin();
            return;
        }
    }
    out[count - 1] = 1.0;
}

/// Equal-power variant-chain interpolation inside one zone over `variant_span`, written into `out`.
#[inline]
pub fn variant_weights_into(
    rpm: f32,
    member_count: usize,
    variant_span: (f32, f32),
    out: &mut Vec<f32>,
) {
    out.clear();
    if member_count <= 1 {
        out.resize(member_count, 1.0);
        return;
    }
    out.resize(member_count, 0.0);
    let (lo, hi) = variant_span;
    let span = (hi - lo).max(1.0e-3);
    let t = ((rpm - lo) / span).clamp(0.0, 1.0) * (member_count - 1) as f32;
    let segment = (t.floor() as usize).min(member_count - 2);
    let frac = t - segment as f32;
    let theta = frac * FRAC_PI_2;
    out[segment] = theta.cos();
    out[segment + 1] = theta.sin();
}

/// Legacy schema-1 3-zone equal-power crossfade curve.
#[inline]
pub fn legacy_zone_weights(
    rpm: f32,
    anchors: [f32; ZONE_COUNT],
    max_fade_start_rpm: f32,
    max_full_rpm: f32,
) -> [f32; ZONE_COUNT] {
    if rpm <= anchors[0] {
        return [1.0, 0.0, 0.0];
    }
    if rpm < anchors[1] {
        let t = ((rpm - anchors[0]) / (anchors[1] - anchors[0])).clamp(0.0, 1.0);
        let theta = t * FRAC_PI_2;
        return [theta.cos(), theta.sin(), 0.0];
    }
    if rpm < max_fade_start_rpm {
        return [0.0, 1.0, 0.0];
    }
    if rpm < max_full_rpm {
        let t = ((rpm - max_fade_start_rpm) / (max_full_rpm - max_fade_start_rpm)).clamp(0.0, 1.0);
        let theta = t * FRAC_PI_2;
        return [0.0, theta.cos(), theta.sin()];
    }
    [0.0, 0.0, 1.0]
}
