//! Physical geometry, combustion, exhaust-runner, and collector configuration.

use std::f32::consts::{PI, TAU};

/// Ten-cylinder four-stroke V10 engine layout.
pub const CYLINDER_COUNT: usize = 10;

/// Structural acoustic character used by structural engine and bay resonators.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EngineStructuralProfile {
    /// High-stiffness F1 aluminum/magnesium V10 engine and carbon-monocoque bay.
    #[default]
    F1V10,
    /// Cast-iron Ford Pinto SOHC inline-4 block, alloy 8V head, steel bay/chassis.
    CastIronSohcL4,
}

/// Exhaust gas thermodynamic constants shared between collector sizing and
/// temperature-dependent runner wave propagation.
pub const MOLAR_MASS_EXHAUST_KG_MOL: f32 = 0.0289;
pub const GAS_CONSTANT_J_PER_MOL_K: f32 = 8.314;
pub const GAMMA: f32 = 1.4;
pub const SPECIFIC_GAS_CONSTANT_J_PER_KG_K: f32 =
    GAS_CONSTANT_J_PER_MOL_K / MOLAR_MASS_EXHAUST_KG_MOL;

/// Unflanged-pipe radiation mass end correction as a fraction of tailpipe radius.
pub const COLLECTOR_END_CORRECTION_RADIUS_FACTOR: f32 = 0.61;

/// Declared 5-in-1 collector geometry for the reduced-order physical collector
/// (`EXH-03`). Treats each bank junction as a lumped chamber of `volume_l`
/// breathing through a straight tailpipe of `outlet_length_m` and `outlet_diameter_m`.
#[derive(Clone, Debug, PartialEq)]
pub struct CollectorGeometry {
    /// Lumped chamber volume per bank (L).
    pub volume_l: f32,
    /// Straight tailpipe length from junction to open end (m).
    pub outlet_length_m: f32,
    /// Tailpipe diameter (m).
    pub outlet_diameter_m: f32,
    /// Design gas temperature for collector wave speed (K).
    pub gas_temperature_k: f32,
    /// Fraction of modal energy lost per cycle (`tau = 2 / (loss * f)`).
    pub loss_fraction_per_cycle: f32,
    /// Modal excitation coupling into radiated collector mix.
    pub mode_coupling: f32,
}

impl CollectorGeometry {
    /// Validates physical geometry bounds and confirms Helmholtz resonance sits
    /// inside the audio band `[1.0, 0.48 * sample_rate]`.
    pub fn validate(&self, sample_rate: u32) -> Result<(), String> {
        if !self.volume_l.is_finite() || !(0.5..=10.0).contains(&self.volume_l) {
            return Err("collector volume outside declared range".into());
        }
        if !self.outlet_length_m.is_finite() || !(0.05..=1.5).contains(&self.outlet_length_m) {
            return Err("collector outlet length outside declared range".into());
        }
        if !self.outlet_diameter_m.is_finite() || !(0.02..=0.30).contains(&self.outlet_diameter_m) {
            return Err("collector outlet diameter outside declared range".into());
        }
        if !self.gas_temperature_k.is_finite()
            || !(300.0..=1_800.0).contains(&self.gas_temperature_k)
        {
            return Err("collector gas temperature outside declared range".into());
        }
        if !self.loss_fraction_per_cycle.is_finite()
            || !(0.05..=0.9).contains(&self.loss_fraction_per_cycle)
        {
            return Err("collector loss fraction outside declared range".into());
        }
        if !self.mode_coupling.is_finite() || !(0.0..=2.0).contains(&self.mode_coupling) {
            return Err("collector mode coupling outside declared range".into());
        }
        let helmholtz = self.helmholtz_hz();
        if !helmholtz.is_finite() || !(1.0..=sample_rate as f32 * 0.48).contains(&helmholtz) {
            return Err("collector Helmholtz resonance outside the audio band".into());
        }
        Ok(())
    }

    /// Speed of sound at the declared collector gas temperature (m/s).
    pub fn wave_speed_mps(&self) -> f32 {
        (GAMMA * SPECIFIC_GAS_CONSTANT_J_PER_KG_K * self.gas_temperature_k).sqrt()
    }

    /// Effective tailpipe length including the unflanged-end radiation mass correction `0.61 * r` (m).
    pub fn effective_outlet_length_m(&self) -> f32 {
        let radius = self.outlet_diameter_m * 0.5;
        self.outlet_length_m + COLLECTOR_END_CORRECTION_RADIUS_FACTOR * radius
    }

    /// Helmholtz resonance of the chamber volume breathing through the tailpipe mass (Hz).
    pub fn helmholtz_hz(&self) -> f32 {
        let volume_m3 = self.volume_l * 1.0e-3;
        let radius = self.outlet_diameter_m * 0.5;
        let area = PI * radius * radius;
        let length = self.effective_outlet_length_m();
        self.wave_speed_mps() / TAU * (area / (volume_m3 * length)).sqrt()
    }
}

/// Standalone physical engine geometry, combustion, and gas boundary configuration (`PHY-130`).
#[derive(Clone, Debug, PartialEq)]
pub struct PhysicalEngineConfig {
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
    pub combustion_pressure_reference_pa: f32,
    pub use_physical_pressure: bool,
    pub expansion_decay_deg: f32,
    pub exhaust_open_deg: f32,
    pub exhaust_opening_ramp_deg: f32,
    pub exhaust_duration_deg: f32,
    pub exhaust_closing_ramp_deg: f32,
    pub exhaust_valve_diameter_m: f32,
    pub exhaust_max_lift_m: f32,
    pub exhaust_discharge_coefficient: f32,
    pub exhaust_opening_shape: f32,
    pub exhaust_closing_shape: f32,
    pub cycle_variation: f32,
    pub cylinder_spread: f32,
    pub cylinder_signature: [f32; CYLINDER_COUNT],
    pub header_lengths_m: [f32; CYLINDER_COUNT],
    pub header_length_scale: f32,
    pub exhaust_wave_speed_mps: f32,
    pub header_reflection: f32,
    pub use_temperature_dependent_wave_speed: bool,
    pub use_physical_exhaust_excitation: bool,
    pub collector_geometry: Option<CollectorGeometry>,
    pub exhaust_excitation_gain: f32,
    pub use_refined_dsp: bool,
}

impl Default for PhysicalEngineConfig {
    fn default() -> Self {
        Self {
            bore_m: 0.096,
            stroke_m: 0.042,
            rod_length_m: 0.135,
            compression_ratio: 12.5,
            bank_angle_deg: 72.0,
            cylinder_count: CYLINDER_COUNT,
            bank_split: 5,
            coast_max_rpm: 15_000.0,
            structural_profile: EngineStructuralProfile::F1V10,
            firing_order: [0, 5, 1, 6, 2, 7, 3, 8, 4, 9],
            combustion_start_deg: 0.0,
            combustion_rise_deg: 24.0,
            combustion_shape_factor: 3.0,
            combustion_efficiency: 0.96,
            fuel_energy_per_cycle: 600.0,
            combustion_pressure_reference_pa: 17_000_000.0,
            use_physical_pressure: true,
            expansion_decay_deg: 112.0,
            exhaust_open_deg: 116.0,
            exhaust_opening_ramp_deg: 5.0,
            exhaust_duration_deg: 142.0,
            exhaust_closing_ramp_deg: 24.0,
            exhaust_valve_diameter_m: 0.03456,
            exhaust_max_lift_m: 0.010368,
            exhaust_discharge_coefficient: 0.62,
            exhaust_opening_shape: 1.0,
            exhaust_closing_shape: 1.0,
            cycle_variation: 0.004,
            cylinder_spread: 0.012,
            cylinder_signature: [1.10, 0.94, 1.04, 0.90, 1.07, 0.96, 1.12, 0.92, 1.02, 0.97],
            header_lengths_m: [
                0.535, 0.557, 0.548, 0.571, 0.562, 0.541, 0.566, 0.552, 0.578, 0.559,
            ],
            header_length_scale: 1.0,
            exhaust_wave_speed_mps: 545.0,
            header_reflection: -0.34,
            use_temperature_dependent_wave_speed: true,
            use_physical_exhaust_excitation: true,
            collector_geometry: None,
            exhaust_excitation_gain: 0.096,
            use_refined_dsp: false,
        }
    }
}

impl PhysicalEngineConfig {
    /// Physical configuration of the 1,993 cc Ford Pinto OHC (TL20) inline-4 for the Escort Mk1 RS2000.
    ///
    /// Bore 90.8 mm x stroke 76.95 mm (1,993.1 cc), 126.87 mm conrod, 9.2:1 compression ratio,
    /// firing order 1-3-4-2 (`[0, 2, 3, 1]`), 36 mm exhaust valve, and single 4-in-1 collector.
    pub fn ford_pinto_rs2000() -> Self {
        Self {
            bore_m: 0.0908,
            stroke_m: 0.07695,
            rod_length_m: 0.12687,
            compression_ratio: 9.2,
            bank_angle_deg: 0.0,
            cylinder_count: 4,
            bank_split: 4,
            coast_max_rpm: 7_000.0,
            structural_profile: EngineStructuralProfile::CastIronSohcL4,
            firing_order: [0, 2, 3, 1, 4, 5, 6, 7, 8, 9],
            combustion_start_deg: 0.0,
            combustion_rise_deg: 42.0,
            combustion_shape_factor: 2.2,
            combustion_efficiency: 0.90,
            fuel_energy_per_cycle: 1_150.0,
            combustion_pressure_reference_pa: 9_500_000.0,
            use_physical_pressure: true,
            expansion_decay_deg: 118.0,
            exhaust_open_deg: 126.0,
            exhaust_opening_ramp_deg: 14.0,
            exhaust_duration_deg: 172.0,
            exhaust_closing_ramp_deg: 32.0,
            exhaust_valve_diameter_m: 0.036,
            exhaust_max_lift_m: 0.0105,
            exhaust_discharge_coefficient: 0.66,
            exhaust_opening_shape: 1.25,
            exhaust_closing_shape: 1.35,
            cycle_variation: 0.018,
            cylinder_spread: 0.035,
            cylinder_signature: [1.06, 0.96, 1.03, 0.95, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            header_lengths_m: [0.72, 0.75, 0.76, 0.71, 0.54, 0.54, 0.54, 0.54, 0.54, 0.54],
            header_length_scale: 1.0,
            exhaust_wave_speed_mps: 515.0,
            header_reflection: -0.38,
            use_temperature_dependent_wave_speed: true,
            use_physical_exhaust_excitation: true,
            collector_geometry: Some(CollectorGeometry {
                volume_l: 1.65,
                outlet_length_m: 0.56,
                outlet_diameter_m: 0.052,
                gas_temperature_k: 880.0,
                loss_fraction_per_cycle: 0.48,
                mode_coupling: 0.72,
            }),
            exhaust_excitation_gain: 0.135,
            use_refined_dsp: true,
        }
    }

    /// Header lengths after applying `header_length_scale`.
    pub fn effective_header_lengths_m(&self) -> [f32; CYLINDER_COUNT] {
        self.header_lengths_m
            .map(|length| length * self.header_length_scale)
    }

    /// Validates physical geometry, combustion parameters, exhaust timing/runners, and collector.
    pub fn validate(&self, sample_rate: u32) -> Result<(), String> {
        self.validate_geometry(sample_rate)?;
        self.validate_combustion()?;
        self.validate_exhaust_timing()?;
        self.validate_exhaust_wave()?;
        if let Some(geometry) = &self.collector_geometry {
            geometry.validate(sample_rate)?;
        }
        Ok(())
    }

    fn validate_geometry(&self, sample_rate: u32) -> Result<(), String> {
        if !(8_000..=192_000).contains(&sample_rate) {
            return Err(format!("sample_rate out of range: {sample_rate}"));
        }
        if !self.bore_m.is_finite() || !(0.05..=0.20).contains(&self.bore_m) {
            return Err("bore_m outside supported range".into());
        }
        if !self.stroke_m.is_finite() || !(0.030..=0.200).contains(&self.stroke_m) {
            return Err("stroke_m outside supported range".into());
        }
        let crank_radius_m = self.stroke_m * 0.5;
        if !self.rod_length_m.is_finite() || !(crank_radius_m..=0.50).contains(&self.rod_length_m) {
            return Err("rod_length_m too short for crank radius or outside range".into());
        }
        if !self.compression_ratio.is_finite() || !(6.0..=20.0).contains(&self.compression_ratio) {
            return Err("compression_ratio outside supported range".into());
        }
        if !self.bank_angle_deg.is_finite() || !(0.0..=180.0).contains(&self.bank_angle_deg) {
            return Err("bank_angle_deg outside supported range".into());
        }
        if !(1..=CYLINDER_COUNT).contains(&self.cylinder_count) {
            return Err("cylinder_count outside supported range".into());
        }
        if !(1..=self.cylinder_count).contains(&self.bank_split)
            || self.bank_split > 5
            || (self.cylinder_count - self.bank_split) > 5
        {
            return Err("bank_split outside supported 5-runner collector capacity".into());
        }
        if !self.coast_max_rpm.is_finite() || !(2_000.0..=25_000.0).contains(&self.coast_max_rpm) {
            return Err("coast_max_rpm outside supported range".into());
        }
        let mut seen = [false; CYLINDER_COUNT];
        for &cylinder in &self.firing_order[..self.cylinder_count] {
            if cylinder >= self.cylinder_count || seen[cylinder] {
                return Err(
                    "firing_order must contain every active cylinder index exactly once".into(),
                );
            }
            seen[cylinder] = true;
        }
        Ok(())
    }

    fn validate_combustion(&self) -> Result<(), String> {
        if !(20.0..=160.0).contains(&self.combustion_rise_deg) {
            return Err("combustion_rise_deg outside physical experiment bounds".into());
        }
        if !self.combustion_shape_factor.is_finite()
            || !(1.0..=10.0).contains(&self.combustion_shape_factor)
        {
            return Err("combustion_shape_factor outside supported range".into());
        }
        if !self.combustion_efficiency.is_finite()
            || !(0.0..=1.0).contains(&self.combustion_efficiency)
        {
            return Err("combustion_efficiency outside supported range".into());
        }
        if !self.fuel_energy_per_cycle.is_finite() || self.fuel_energy_per_cycle <= 0.0 {
            return Err("fuel_energy_per_cycle must be positive".into());
        }
        if !self.combustion_pressure_reference_pa.is_finite()
            || self.combustion_pressure_reference_pa <= 0.0
        {
            return Err("combustion_pressure_reference_pa must be positive".into());
        }
        if !(30.0..=260.0).contains(&self.expansion_decay_deg) {
            return Err("expansion_decay_deg outside physical experiment bounds".into());
        }
        Ok(())
    }

    fn validate_exhaust_timing(&self) -> Result<(), String> {
        if !(60.0..=240.0).contains(&self.exhaust_open_deg) {
            return Err("exhaust_open_deg outside physical experiment bounds".into());
        }
        if !(1.0..=30.0).contains(&self.exhaust_opening_ramp_deg) {
            return Err("exhaust_opening_ramp_deg outside physical experiment bounds".into());
        }
        if !(30.0..=240.0).contains(&self.exhaust_duration_deg) {
            return Err("exhaust_duration_deg outside physical experiment bounds".into());
        }
        if !(1.0..=60.0).contains(&self.exhaust_closing_ramp_deg) {
            return Err("exhaust_closing_ramp_deg outside physical experiment bounds".into());
        }
        if self.exhaust_opening_ramp_deg + self.exhaust_closing_ramp_deg
            >= self.exhaust_duration_deg
        {
            return Err("exhaust opening + closing ramps must fit within the lift window".into());
        }
        if !self.exhaust_valve_diameter_m.is_finite()
            || !(0.005..=0.10).contains(&self.exhaust_valve_diameter_m)
        {
            return Err("exhaust_valve_diameter_m outside supported range".into());
        }
        if !self.exhaust_max_lift_m.is_finite()
            || !(0.001..=0.020).contains(&self.exhaust_max_lift_m)
        {
            return Err("exhaust_max_lift_m outside supported range".into());
        }
        if !self.exhaust_discharge_coefficient.is_finite()
            || !(0.10..=0.95).contains(&self.exhaust_discharge_coefficient)
        {
            return Err("exhaust_discharge_coefficient outside supported range".into());
        }
        for (name, shape) in [
            ("exhaust_opening_shape", self.exhaust_opening_shape),
            ("exhaust_closing_shape", self.exhaust_closing_shape),
        ] {
            if !shape.is_finite() || shape <= 0.0 || shape > 10.0 {
                return Err(format!("{name} outside supported range (must be > 0)"));
            }
        }
        Ok(())
    }

    fn validate_exhaust_wave(&self) -> Result<(), String> {
        if !(0.0..=0.15).contains(&self.cycle_variation)
            || !(0.0..=0.10).contains(&self.cylinder_spread)
        {
            return Err("combustion variation exceeds safe bounds".into());
        }
        if self.cylinder_signature[..self.cylinder_count]
            .iter()
            .any(|&x| !x.is_finite() || !(0.75..=1.25).contains(&x))
        {
            return Err("cylinder signature outside supported range".into());
        }
        if self.exhaust_wave_speed_mps < 250.0 || self.exhaust_wave_speed_mps > 900.0 {
            return Err("exhaust wave speed outside supported range".into());
        }
        if self.header_lengths_m[..self.cylinder_count]
            .iter()
            .any(|&x| !(0.2..=1.5).contains(&x))
        {
            return Err("header length outside supported range".into());
        }
        if !self.header_length_scale.is_finite() || !(0.4..=2.0).contains(&self.header_length_scale)
        {
            return Err("header length scale outside supported range".into());
        }
        if self.effective_header_lengths_m()[..self.cylinder_count]
            .iter()
            .any(|&x| !(0.2..=1.5).contains(&x))
        {
            return Err("effective header length outside supported range".into());
        }
        if self.header_reflection.abs() >= 0.75 {
            return Err("header reflection must remain safely below unity".into());
        }
        if !self.exhaust_excitation_gain.is_finite()
            || !(0.0..=2.0).contains(&self.exhaust_excitation_gain)
        {
            return Err("exhaust excitation gain outside declared safety bounds".into());
        }
        Ok(())
    }
}
