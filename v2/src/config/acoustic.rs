//! Structural engine mix and vehicle acoustic scene configuration.

use crate::config::physical::{EngineStructuralProfile, CYLINDER_COUNT};

/// Standalone acoustic mix and structural gain configuration (`PHY-130`).
#[derive(Clone, Debug, PartialEq)]
pub struct AcousticMixConfig {
    pub pressure_direct_gain: f32,
    pub crankcase_gain: f32,
    pub block_gain: f32,
    pub head_gain: f32,
    pub exhaust_gain: f32,
    pub turbulence_gain: f32,
    pub master_gain: f32,
}

impl Default for AcousticMixConfig {
    fn default() -> Self {
        Self {
            pressure_direct_gain: 0.025,
            crankcase_gain: 0.008,
            block_gain: 0.56,
            head_gain: 0.48,
            exhaust_gain: 2.20,
            turbulence_gain: 0.40,
            master_gain: 7.0,
        }
    }
}

impl AcousticMixConfig {
    /// Acoustic mix gains calibrated for the 1,993 cc Ford Pinto OHC L4 (Escort Mk1 RS2000).
    pub fn ford_pinto_rs2000() -> Self {
        Self {
            pressure_direct_gain: 0.016,
            crankcase_gain: 0.012,
            block_gain: 0.55,
            head_gain: 0.95,
            exhaust_gain: 3.40,
            turbulence_gain: 0.78,
            master_gain: 7.0,
        }
    }

    /// Validates structural and master mix gains against frozen safety bounds.
    pub fn validate(&self) -> Result<(), String> {
        if !(0.0..=2.0).contains(&self.pressure_direct_gain)
            || !(0.0..=2.0).contains(&self.crankcase_gain)
            || !(0.0..=2.0).contains(&self.block_gain)
            || !(0.0..=2.0).contains(&self.head_gain)
            || !(0.0..=4.0).contains(&self.exhaust_gain)
            || !(0.0..=2.0).contains(&self.turbulence_gain)
            || !(0.0..=20.0).contains(&self.master_gain)
        {
            return Err("mix gain outside declared safety bounds".into());
        }
        Ok(())
    }
}

/// Configuration for the multi-path chassis/air/engine-cover acoustic scene.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AcousticSceneConfig {
    pub dry_low_gain: f32,
    pub dry_mid_gain: f32,
    pub dry_high_gain: f32,
    /// Gain on the air-propagated dry branch in the scene mix (default 1.0).
    pub engine_air_gain: f32,
    pub engine_cover_gain: f32,
    pub mount_monocoque_gain: f32,
    /// Parallel send of the transmission into the mount/monocoque structural path.
    pub transmission_mount_gain: f32,
    /// Parallel send of the transmission into the engine-cover skin.
    pub transmission_cover_gain: f32,
    /// High-shelf lift (dB) above `2500 Hz` applied to the air-path input.
    pub air_high_tilt_db: f32,
    /// Fraction of the high-tilted air input mixed straight into the air output.
    pub air_direct_gain: f32,
    pub output_gain: f32,
    /// One-pole lowpass on engine-cover radiation (Hz); `>= 0.48 * sample_rate` bypasses.
    pub cover_radiation_lowpass_hz: f32,
    /// Active engine cylinder count (`1..=CYLINDER_COUNT`).
    pub cylinder_count: usize,
    /// Number of Bank A cylinders routed to the first exhaust collector (`1..=cylinder_count`).
    pub bank_split: usize,
    /// Structural panel/mount acoustic profile.
    pub structural_profile: EngineStructuralProfile,
    /// RPM threshold where high-RPM cover/skin character weighting begins ramping in.
    pub high_rpm_start: f32,
    /// RPM span over which high-RPM weighting ramps from `0.0` to `1.0`.
    pub high_rpm_span: f32,
}

impl Default for AcousticSceneConfig {
    fn default() -> Self {
        Self {
            dry_low_gain: 0.18,
            dry_mid_gain: 0.46,
            dry_high_gain: 0.14,
            engine_air_gain: 1.0,
            engine_cover_gain: 0.42,
            mount_monocoque_gain: 0.06,
            transmission_mount_gain: 0.0,
            transmission_cover_gain: 0.0,
            air_high_tilt_db: 0.0,
            air_direct_gain: 0.0,
            output_gain: 2.90,
            cover_radiation_lowpass_hz: 6_400.0,
            cylinder_count: CYLINDER_COUNT,
            bank_split: 5,
            structural_profile: EngineStructuralProfile::F1V10,
            high_rpm_start: 8_500.0,
            high_rpm_span: 6_000.0,
        }
    }
}

impl AcousticSceneConfig {
    /// Acoustic scene tuned for the Ford Escort Mk1 RS2000 (steel unibody/bonnet, 4-cyl OHC).
    pub fn ford_pinto_rs2000() -> Self {
        Self {
            dry_low_gain: 0.19,
            dry_mid_gain: 0.48,
            dry_high_gain: 0.14,
            engine_air_gain: 1.15,
            engine_cover_gain: 0.42,
            mount_monocoque_gain: 0.075,
            transmission_mount_gain: 0.0,
            transmission_cover_gain: 0.0,
            air_high_tilt_db: -0.5,
            air_direct_gain: 0.0,
            output_gain: 2.95,
            cover_radiation_lowpass_hz: 4_800.0,
            cylinder_count: 4,
            bank_split: 4,
            structural_profile: EngineStructuralProfile::CastIronSohcL4,
            high_rpm_start: 3_400.0,
            high_rpm_span: 3_200.0,
        }
    }

    /// Validates every acoustic scene gain, tilt, and radiation cutoff bound.
    pub fn validate(&self) -> Result<(), String> {
        if !(0.0..=1.5).contains(&self.dry_low_gain)
            || !(0.0..=1.5).contains(&self.dry_mid_gain)
            || !(0.0..=1.5).contains(&self.dry_high_gain)
            || !(0.0..=1.5).contains(&self.engine_air_gain)
            || !(0.0..=1.5).contains(&self.engine_cover_gain)
            || !(0.0..=1.5).contains(&self.mount_monocoque_gain)
            || !(0.0..=1.5).contains(&self.transmission_mount_gain)
            || !(0.0..=2.0).contains(&self.transmission_cover_gain)
            || !(-6.0..=12.0).contains(&self.air_high_tilt_db)
            || !(0.0..=1.0).contains(&self.air_direct_gain)
            || !(0.25..=5.0).contains(&self.output_gain)
            || !(100.0..=1_000_000.0).contains(&self.cover_radiation_lowpass_hz)
        {
            return Err("acoustic scene gain outside supported range".into());
        }
        if !(1..=CYLINDER_COUNT).contains(&self.cylinder_count)
            || !(1..=self.cylinder_count).contains(&self.bank_split)
            || !self.high_rpm_start.is_finite()
            || self.high_rpm_start < 500.0
            || !self.high_rpm_span.is_finite()
            || self.high_rpm_span < 500.0
        {
            return Err(
                "acoustic scene engine layout or RPM range outside supported bounds".into(),
            );
        }
        Ok(())
    }
}
