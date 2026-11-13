//! Unified configuration schemas, derived engine geometry, and invariant validation.

pub mod acoustic;
pub mod physical;
pub mod runtime;
pub mod samples;

pub use acoustic::{AcousticMixConfig, AcousticSceneConfig};
pub use physical::{
    CollectorGeometry, EngineStructuralProfile, PhysicalEngineConfig,
    COLLECTOR_END_CORRECTION_RADIUS_FACTOR, CYLINDER_COUNT, GAMMA, GAS_CONSTANT_J_PER_MOL_K,
    MOLAR_MASS_EXHAUST_KG_MOL, SPECIFIC_GAS_CONSTANT_J_PER_KG_K,
};
pub use runtime::{EngineMode, Gf509RuntimeConfig, TransmissionConfig};
pub use samples::{ThreeZoneSampleLayerConfig, ZONE_TRIM_SLOTS};

/// Flat compatibility facade combining physical geometry/combustion/exhaust
/// configuration (`PhysicalEngineConfig`) and mix gains (`AcousticMixConfig`).
#[derive(Clone, Debug, PartialEq)]
pub struct EngineConfig {
    pub sample_rate: u32,
    pub seed: u64,
    pub bore_m: f32,
    pub stroke_m: f32,
    pub rod_length_m: f32,
    pub compression_ratio: f32,
    pub bank_angle_deg: f32,
    pub cylinder_count: usize,
    pub bank_split: usize,
    pub coast_max_rpm: f32,
    pub structural_profile: EngineStructuralProfile,
    pub firing_order: [usize; CYLINDER_COUNT],
    pub combustion_start_deg: f32,
    pub combustion_rise_deg: f32,
    pub combustion_shape_factor: f32,
    pub combustion_efficiency: f32,
    pub fuel_energy_per_cycle: f32,
    /// Chamber pressure (Pa) mapping to an acoustic source of 1.0.
    pub combustion_pressure_reference_pa: f32,
    /// Selects thermodynamic pressure path (`true`) or legacy handcrafted shape (`false`).
    pub use_physical_pressure: bool,
    pub expansion_decay_deg: f32,
    /// Exhaust valve opening angle (deg, EVO).
    pub exhaust_open_deg: f32,
    pub exhaust_opening_ramp_deg: f32,
    /// Exhaust valve duration (deg); `EVC = exhaust_open_deg + exhaust_duration_deg`.
    pub exhaust_duration_deg: f32,
    pub exhaust_closing_ramp_deg: f32,
    /// Poppet exhaust-valve seat diameter (m).
    pub exhaust_valve_diameter_m: f32,
    /// Maximum exhaust-valve lift (m).
    pub exhaust_max_lift_m: f32,
    /// Discharge coefficient relating effective curtain area to geometric curtain area.
    pub exhaust_discharge_coefficient: f32,
    /// Opening-ramp shape exponent (`> 0`). `1.0` is a raised-cosine ramp.
    pub exhaust_opening_shape: f32,
    /// Closing-ramp shape exponent (`> 0`). `1.0` is a raised-cosine ramp.
    pub exhaust_closing_shape: f32,
    pub cycle_variation: f32,
    pub cylinder_spread: f32,
    pub cylinder_signature: [f32; CYLINDER_COUNT],
    pub header_lengths_m: [f32; CYLINDER_COUNT],
    /// Uniform multiplier applied to `header_lengths_m`.
    pub header_length_scale: f32,
    pub exhaust_wave_speed_mps: f32,
    pub header_reflection: f32,
    /// Derives wave speed from runner gas temperature (`sqrt(gamma * R * T)`).
    pub use_temperature_dependent_wave_speed: bool,
    /// Uses physical runner mass-flow delta as header excitation.
    pub use_physical_exhaust_excitation: bool,
    /// Optional physical 5-in-1 collector geometry (`EXH-03`).
    pub collector_geometry: Option<CollectorGeometry>,
    /// Level-matching gain from normalized runner mass-flow delta to acoustic excitation.
    pub exhaust_excitation_gain: f32,
    /// Enables C1 smooth physics, Hermite waveguides, dual-band turbulence, and modal drift.
    pub use_refined_dsp: bool,
    pub pressure_direct_gain: f32,
    pub crankcase_gain: f32,
    pub block_gain: f32,
    pub head_gain: f32,
    pub exhaust_gain: f32,
    pub turbulence_gain: f32,
    pub master_gain: f32,
    /// Synthesized downshift blip flare (phase 4). Disabled when playing `geardn.wav`.
    pub blip_enabled: bool,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self::from_subsystems(
            48_000,
            0xF090_0010,
            PhysicalEngineConfig::default(),
            AcousticMixConfig::default(),
        )
    }
}

impl EngineConfig {
    /// Preset configuration for the 1,993 cc Ford Pinto OHC inline-4 in the Escort Mk1 RS2000.
    pub fn ford_pinto_rs2000(sample_rate: u32) -> Self {
        Self::from_subsystems(
            sample_rate,
            0xE5C0_2000,
            PhysicalEngineConfig::ford_pinto_rs2000(),
            AcousticMixConfig::ford_pinto_rs2000(),
        )
    }

    /// Validates sample rate, physical configuration, and acoustic mix bounds.
    pub fn validate(&self) -> Result<(), String> {
        self.physical().validate(self.sample_rate)?;
        self.mix().validate()
    }

    /// Distance from crankshaft axis to crankpin at TDC (`stroke_m * 0.5`).
    pub fn crank_radius_m(&self) -> f32 {
        self.stroke_m * 0.5
    }

    /// Piston crown area derived from bore.
    pub fn bore_area_m2(&self) -> f32 {
        let bore = self.bore_m as f64;
        (std::f64::consts::PI * 0.25 * bore * bore) as f32
    }

    /// Per-cylinder volume displaced between TDC and BDC.
    pub fn swept_volume_m3(&self) -> f32 {
        self.bore_area_m2() * self.stroke_m
    }

    /// Combustion chamber volume at TDC.
    pub fn clearance_volume_m3(&self) -> f32 {
        self.swept_volume_m3() / (self.compression_ratio - 1.0)
    }

    /// Per-cylinder displacement in liters.
    pub fn cylinder_displacement_liters(&self) -> f32 {
        self.swept_volume_m3() * 1_000.0
    }

    /// Total engine displacement in liters across active `cylinder_count` cylinders.
    pub fn engine_displacement_liters(&self) -> f32 {
        self.cylinder_displacement_liters() * self.cylinder_count as f32
    }

    /// Header lengths after applying `header_length_scale`.
    pub fn effective_header_lengths_m(&self) -> [f32; CYLINDER_COUNT] {
        self.header_lengths_m
            .map(|length| length * self.header_length_scale)
    }

    /// Extracts the dedicated physical engine configuration (`PHY-130`).
    pub fn physical(&self) -> PhysicalEngineConfig {
        PhysicalEngineConfig {
            bore_m: self.bore_m,
            stroke_m: self.stroke_m,
            rod_length_m: self.rod_length_m,
            compression_ratio: self.compression_ratio,
            bank_angle_deg: self.bank_angle_deg,
            cylinder_count: self.cylinder_count,
            bank_split: self.bank_split,
            coast_max_rpm: self.coast_max_rpm,
            structural_profile: self.structural_profile,
            firing_order: self.firing_order,
            combustion_start_deg: self.combustion_start_deg,
            combustion_rise_deg: self.combustion_rise_deg,
            combustion_shape_factor: self.combustion_shape_factor,
            combustion_efficiency: self.combustion_efficiency,
            fuel_energy_per_cycle: self.fuel_energy_per_cycle,
            combustion_pressure_reference_pa: self.combustion_pressure_reference_pa,
            use_physical_pressure: self.use_physical_pressure,
            expansion_decay_deg: self.expansion_decay_deg,
            exhaust_open_deg: self.exhaust_open_deg,
            exhaust_opening_ramp_deg: self.exhaust_opening_ramp_deg,
            exhaust_duration_deg: self.exhaust_duration_deg,
            exhaust_closing_ramp_deg: self.exhaust_closing_ramp_deg,
            exhaust_valve_diameter_m: self.exhaust_valve_diameter_m,
            exhaust_max_lift_m: self.exhaust_max_lift_m,
            exhaust_discharge_coefficient: self.exhaust_discharge_coefficient,
            exhaust_opening_shape: self.exhaust_opening_shape,
            exhaust_closing_shape: self.exhaust_closing_shape,
            cycle_variation: self.cycle_variation,
            cylinder_spread: self.cylinder_spread,
            cylinder_signature: self.cylinder_signature,
            header_lengths_m: self.header_lengths_m,
            header_length_scale: self.header_length_scale,
            exhaust_wave_speed_mps: self.exhaust_wave_speed_mps,
            header_reflection: self.header_reflection,
            use_temperature_dependent_wave_speed: self.use_temperature_dependent_wave_speed,
            use_physical_exhaust_excitation: self.use_physical_exhaust_excitation,
            collector_geometry: self.collector_geometry.clone(),
            exhaust_excitation_gain: self.exhaust_excitation_gain,
            use_refined_dsp: self.use_refined_dsp,
        }
    }

    /// Extracts the dedicated acoustic mix configuration (`PHY-130`).
    pub fn mix(&self) -> AcousticMixConfig {
        AcousticMixConfig {
            pressure_direct_gain: self.pressure_direct_gain,
            crankcase_gain: self.crankcase_gain,
            block_gain: self.block_gain,
            head_gain: self.head_gain,
            exhaust_gain: self.exhaust_gain,
            turbulence_gain: self.turbulence_gain,
            master_gain: self.master_gain,
        }
    }

    /// Constructs a unified `EngineConfig` from separate physical and mix configurations (`PHY-130`).
    pub fn from_subsystems(
        sample_rate: u32,
        seed: u64,
        physical: PhysicalEngineConfig,
        mix: AcousticMixConfig,
    ) -> Self {
        Self {
            sample_rate,
            seed,
            bore_m: physical.bore_m,
            stroke_m: physical.stroke_m,
            rod_length_m: physical.rod_length_m,
            compression_ratio: physical.compression_ratio,
            bank_angle_deg: physical.bank_angle_deg,
            cylinder_count: physical.cylinder_count,
            bank_split: physical.bank_split,
            coast_max_rpm: physical.coast_max_rpm,
            structural_profile: physical.structural_profile,
            firing_order: physical.firing_order,
            combustion_start_deg: physical.combustion_start_deg,
            combustion_rise_deg: physical.combustion_rise_deg,
            combustion_shape_factor: physical.combustion_shape_factor,
            combustion_efficiency: physical.combustion_efficiency,
            fuel_energy_per_cycle: physical.fuel_energy_per_cycle,
            combustion_pressure_reference_pa: physical.combustion_pressure_reference_pa,
            use_physical_pressure: physical.use_physical_pressure,
            expansion_decay_deg: physical.expansion_decay_deg,
            exhaust_open_deg: physical.exhaust_open_deg,
            exhaust_opening_ramp_deg: physical.exhaust_opening_ramp_deg,
            exhaust_duration_deg: physical.exhaust_duration_deg,
            exhaust_closing_ramp_deg: physical.exhaust_closing_ramp_deg,
            exhaust_valve_diameter_m: physical.exhaust_valve_diameter_m,
            exhaust_max_lift_m: physical.exhaust_max_lift_m,
            exhaust_discharge_coefficient: physical.exhaust_discharge_coefficient,
            exhaust_opening_shape: physical.exhaust_opening_shape,
            exhaust_closing_shape: physical.exhaust_closing_shape,
            cycle_variation: physical.cycle_variation,
            cylinder_spread: physical.cylinder_spread,
            cylinder_signature: physical.cylinder_signature,
            header_lengths_m: physical.header_lengths_m,
            header_length_scale: physical.header_length_scale,
            exhaust_wave_speed_mps: physical.exhaust_wave_speed_mps,
            header_reflection: physical.header_reflection,
            use_temperature_dependent_wave_speed: physical.use_temperature_dependent_wave_speed,
            use_physical_exhaust_excitation: physical.use_physical_exhaust_excitation,
            collector_geometry: physical.collector_geometry,
            exhaust_excitation_gain: physical.exhaust_excitation_gain,
            use_refined_dsp: physical.use_refined_dsp,
            pressure_direct_gain: mix.pressure_direct_gain,
            crankcase_gain: mix.crankcase_gain,
            block_gain: mix.block_gain,
            head_gain: mix.head_gain,
            exhaust_gain: mix.exhaust_gain,
            turbulence_gain: mix.turbulence_gain,
            master_gain: mix.master_gain,
            blip_enabled: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collector_geometry() -> CollectorGeometry {
        CollectorGeometry {
            volume_l: 2.5,
            outlet_length_m: 0.35,
            outlet_diameter_m: 0.09,
            gas_temperature_k: 1_000.0,
            loss_fraction_per_cycle: 0.5,
            mode_coupling: 0.6,
        }
    }

    #[test]
    fn physical_and_mix_subsystems_roundtrip_identically() {
        let base = EngineConfig::default();
        let phys = base.physical();
        let mix = base.mix();
        let roundtrip =
            EngineConfig::from_subsystems(base.sample_rate, base.seed, phys.clone(), mix.clone());
        assert_eq!(roundtrip.physical(), phys);
        assert_eq!(roundtrip.mix(), mix);
        assert_eq!(roundtrip, base);
        assert!(roundtrip.validate().is_ok());
        assert!((base.engine_displacement_liters() - 3.04).abs() < 0.05);
    }

    #[test]
    fn header_length_scale_scales_every_primary_and_rejects_out_of_range() {
        let mut config = EngineConfig::default();
        assert_eq!(config.header_length_scale, 1.0);
        assert_eq!(config.effective_header_lengths_m(), config.header_lengths_m);

        config.header_length_scale = 1.5;
        let scaled = config.effective_header_lengths_m();
        for (scaled_length, declared) in scaled.iter().zip(config.header_lengths_m.iter()) {
            assert!((scaled_length / declared - 1.5).abs() < 1.0e-6);
        }
        assert!(config.validate().is_ok());

        config.header_length_scale = 0.0;
        assert!(config.validate().is_err());
        config.header_length_scale = 2.5;
        assert!(config.validate().is_err());
        config.header_length_scale = 2.0;
        config.header_lengths_m[0] = 1.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn collector_geometry_defaults_and_helmholtz_scaling_behave_physically() {
        let mut config = EngineConfig::default();
        assert!(config.collector_geometry.is_none());
        assert!(config.validate().is_ok());

        let base = collector_geometry();
        let fundamental = base.helmholtz_hz();
        let mut larger = base.clone();
        larger.volume_l = 5.0;
        let mut longer = base.clone();
        longer.outlet_length_m = 0.70;
        let mut hotter = base.clone();
        hotter.gas_temperature_k = 1_400.0;
        let mut wider = base.clone();
        wider.outlet_diameter_m = 0.15;
        assert!(larger.helmholtz_hz() < fundamental);
        assert!(longer.helmholtz_hz() < fundamental);
        assert!(hotter.helmholtz_hz() > fundamental);
        assert!(wider.helmholtz_hz() > fundamental);

        config.collector_geometry = Some(base);
        assert!(config.validate().is_ok());

        let mut invalid = collector_geometry();
        invalid.volume_l = 0.0;
        config.collector_geometry = Some(invalid);
        assert!(config.validate().is_err());

        let mut invalid = collector_geometry();
        invalid.loss_fraction_per_cycle = 0.0;
        config.collector_geometry = Some(invalid);
        assert!(config.validate().is_err());
    }

    #[test]
    fn zone_tonal_trims_transmission_scene_and_runtime_configs_validate() {
        let sample_cfg = ThreeZoneSampleLayerConfig::default();
        assert_eq!(sample_cfg.zone_trim_db, [0.0; ZONE_TRIM_SLOTS]);
        assert!(sample_cfg.validate().is_ok());

        let mut invalid_sample = sample_cfg;
        invalid_sample.zone_trim_db[3] = 13.0;
        assert!(invalid_sample.validate().is_err());

        let mut valid_sample = sample_cfg;
        valid_sample.zone_trim_db[0] = 2.0;
        valid_sample.zone_trim_db[5] = -6.0;
        assert!(valid_sample.validate().is_ok());

        assert!(AcousticSceneConfig::default().validate().is_ok());
        let bad_scene = AcousticSceneConfig {
            output_gain: 10.0,
            ..AcousticSceneConfig::default()
        };
        assert!(bad_scene.validate().is_err());

        assert!(TransmissionConfig::default().validate().is_ok());
        let mut bad_tx = TransmissionConfig {
            output_gain: 0.5,
            ..TransmissionConfig::default()
        };
        assert!(bad_tx.validate().is_err());
        bad_tx.gear_ratios = vec![3.1];
        assert!(bad_tx.validate().is_ok());

        let mut rt = Gf509RuntimeConfig::default();
        assert!(rt.validate().is_ok());
        rt.engine_mode = EngineMode::SampleOnly;
        assert!(rt.validate().is_err());
        rt.sample_layer_directory = Some("bank".into());
        assert!(rt.validate().is_ok());
    }

    #[test]
    fn ford_pinto_rs2000_preset_has_1993cc_four_cylinder_layout_and_validates() {
        let engine = EngineConfig::ford_pinto_rs2000(48_000);
        assert!(engine.validate().is_ok());
        assert_eq!(engine.cylinder_count, 4);
        assert_eq!(engine.bank_split, 4);
        assert_eq!(engine.bank_angle_deg, 0.0);
        assert_eq!(&engine.firing_order[..4], &[0, 2, 3, 1]);
        assert!((engine.engine_displacement_liters() - 1.993).abs() < 0.002);

        let runtime = Gf509RuntimeConfig::ford_pinto_rs2000(48_000);
        assert!(runtime.validate().is_ok());
        assert_eq!(runtime.scene.cylinder_count, 4);
        assert_eq!(runtime.scene.bank_split, 4);

        let mut bad = engine.clone();
        bad.firing_order[1] = 0;
        assert!(bad.validate().is_err());

        let mut bad_split = engine;
        bad_split.bank_split = 5;
        assert!(bad_split.validate().is_err());
    }
}
