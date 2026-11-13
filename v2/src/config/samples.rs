//! Multi-zone sample-layer configuration and trim validation.

/// Per-zone trim slots sized with headroom for multi-zone schema-2 banks.
pub const ZONE_TRIM_SLOTS: usize = 16;

/// Configuration governing tonal/residual load curves, RPM crossfade windows,
/// hybrid blend weights, and per-zone trim eq.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThreeZoneSampleLayerConfig {
    pub tonal_gain_closed: f32,
    pub tonal_gain_loaded: f32,
    pub residual_gain_closed: f32,
    pub residual_gain_loaded: f32,
    pub max_fade_start_rpm: f32,
    pub max_full_rpm: f32,
    /// Physical simulation blend weight (default 1.0).
    pub physical_blend_weight: f32,
    /// Sample layer blend weight (default 1.0).
    pub sample_blend_weight: f32,
    /// Off-throttle overrun layer blend gain (default 0.20).
    pub off_throttle_gain: f32,
    /// Skip sinc resampling + mid DSP for inaudible zones while advancing phase cursors.
    pub suspend_inaudible_zones: bool,
    /// Deep-silence margin in RPM around crossfade regions before suspension engages.
    pub suspend_margin_rpm: f32,
    /// Resample through the precomputed windowed-sinc table (`AUD-07a`).
    pub use_tabled_sinc: bool,
    /// Diagnostic mute for the top-zone +2.5 dB rasp boost and stem.
    pub disable_sample_rasp: bool,
    /// Diagnostic multiplier on load-blended residual gain.
    pub residual_gain_scale: f32,
    /// Per-zone tonal trim in dB (`-12.0..=12.0`), indexed by ON zone order.
    pub zone_trim_db: [f32; ZONE_TRIM_SLOTS],
}

impl Default for ThreeZoneSampleLayerConfig {
    fn default() -> Self {
        Self {
            tonal_gain_closed: 0.05,
            tonal_gain_loaded: 0.22,
            residual_gain_closed: 0.12,
            residual_gain_loaded: 0.40,
            max_fade_start_rpm: 5_970.0,
            max_full_rpm: 8_725.0,
            physical_blend_weight: 1.0,
            sample_blend_weight: 1.0,
            off_throttle_gain: 0.20,
            suspend_inaudible_zones: true,
            suspend_margin_rpm: 500.0,
            use_tabled_sinc: true,
            disable_sample_rasp: false,
            residual_gain_scale: 1.0,
            zone_trim_db: [0.0; ZONE_TRIM_SLOTS],
        }
    }
}

impl ThreeZoneSampleLayerConfig {
    /// Validates gains, suspend margins, trims, and RPM crossfade boundaries.
    pub fn validate(self) -> Result<Self, String> {
        for (name, value) in [
            ("tonal_gain_closed", self.tonal_gain_closed),
            ("tonal_gain_loaded", self.tonal_gain_loaded),
            ("residual_gain_closed", self.residual_gain_closed),
            ("residual_gain_loaded", self.residual_gain_loaded),
            ("physical_blend_weight", self.physical_blend_weight),
            ("sample_blend_weight", self.sample_blend_weight),
            ("off_throttle_gain", self.off_throttle_gain),
        ] {
            if !value.is_finite() || !(0.0..=2.0).contains(&value) {
                return Err(format!("{name} outside 0..2.0: {value}"));
            }
        }
        if !self.suspend_margin_rpm.is_finite()
            || !(0.0..=5_000.0).contains(&self.suspend_margin_rpm)
        {
            return Err(format!(
                "suspend_margin_rpm outside 0..5000: {}",
                self.suspend_margin_rpm
            ));
        }
        if !self.residual_gain_scale.is_finite() || !(0.0..=2.0).contains(&self.residual_gain_scale)
        {
            return Err(format!(
                "residual_gain_scale outside 0..2.0: {}",
                self.residual_gain_scale
            ));
        }
        for (index, trim) in self.zone_trim_db.iter().enumerate() {
            if !trim.is_finite() || !(-12.0..=12.0).contains(trim) {
                return Err(format!("zone_trim_db[{index}] outside -12..12 dB: {trim}"));
            }
        }
        if !self.max_fade_start_rpm.is_finite()
            || !self.max_full_rpm.is_finite()
            || self.max_fade_start_rpm < 1_000.0
            || self.max_full_rpm > 25_000.0
            || self.max_full_rpm - self.max_fade_start_rpm < 500.0
        {
            return Err("invalid max-zone RPM crossfade window".into());
        }
        Ok(self)
    }
}
