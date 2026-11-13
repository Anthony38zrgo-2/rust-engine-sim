//! Lightweight ideal-gas state for a single cylinder charge.
//!
//! This is intentionally a coarse thermodynamic model: no species chemistry,
//! no dissociation, and no per-sample allocation. It tracks only the bulk P/T
//! state needed to drive pressure and mass flow later in the physical chain.

/// Specific gas constant for dry air, J/(kg·K).
pub const R_SPECIFIC_AIR: f32 = 287.05;
/// Adiabatic index (ratio of specific heats) for air.
pub const GAMMA: f32 = 1.4;
/// Specific heat at constant volume, J/(kg·K): `R / (gamma - 1)`.
pub const CV_AIR: f32 = R_SPECIFIC_AIR / (GAMMA - 1.0);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GasState {
    pub pressure_pa: f32,
    pub temperature_k: f32,
    pub mass_kg: f32,
    pub volume_m3: f32,
}

impl GasState {
    pub fn new(pressure_pa: f32, temperature_k: f32, mass_kg: f32, volume_m3: f32) -> Self {
        Self {
            pressure_pa,
            temperature_k,
            mass_kg,
            volume_m3,
        }
    }

    /// Build from mass, temperature and known volume using the ideal gas law.
    pub fn from_mass_temperature_volume(
        mass_kg: f32,
        temperature_k: f32,
        volume_m3: f32,
    ) -> Self {
        Self {
            pressure_pa: ideal_gas_pressure(mass_kg, temperature_k, volume_m3),
            temperature_k,
            mass_kg,
            volume_m3,
        }
    }

    pub fn density_kg_m3(&self) -> f32 {
        self.mass_kg / self.volume_m3
    }

    /// Specific internal energy per unit mass, J/kg.
    pub fn specific_internal_energy_j_kg(&self) -> f32 {
        CV_AIR * self.temperature_k
    }

    /// Total internal energy of the charge, J.
    pub fn internal_energy_j(&self) -> f32 {
        self.mass_kg * self.specific_internal_energy_j_kg()
    }

    /// Speed of sound in the charge, m/s.
    pub fn speed_of_sound_mps(&self) -> f32 {
        (GAMMA * R_SPECIFIC_AIR * self.temperature_k).sqrt()
    }

    /// Validate that the state is finite and physically plausible.
    pub fn validate(&self) -> Result<(), String> {
        let all_finite = self.pressure_pa.is_finite()
            && self.temperature_k.is_finite()
            && self.mass_kg.is_finite()
            && self.volume_m3.is_finite();
        if !all_finite {
            return Err("gas state contains a non-finite quantity".into());
        }
        if self.pressure_pa <= 0.0 || self.pressure_pa > 2.0e7 {
            return Err(format!("pressure_pa out of range: {}", self.pressure_pa));
        }
        if !(200.0..=3500.0).contains(&self.temperature_k) {
            return Err(format!(
                "temperature_k out of range: {}",
                self.temperature_k
            ));
        }
        if self.mass_kg < 0.0 {
            return Err(format!("mass_kg cannot be negative: {}", self.mass_kg));
        }
        if self.volume_m3 <= 0.0 {
            return Err(format!("volume_m3 must be positive: {}", self.volume_m3));
        }
        Ok(())
    }

    pub fn is_valid(&self) -> bool {
        self.validate().is_ok()
    }
}

/// Ideal gas law: `P = m * R * T / V`.
#[inline]
pub fn ideal_gas_pressure(mass_kg: f32, temperature_k: f32, volume_m3: f32) -> f32 {
    mass_kg * R_SPECIFIC_AIR * temperature_k / volume_m3
}

#[cfg(test)]
mod tests {
    use super::*;

    fn air_state() -> GasState {
        GasState::new(101_325.0, 300.0, 0.001, 0.000_85)
    }

    #[test]
    fn constructor_stores_state_and_derives_quantities() {
        let s = air_state();
        assert_eq!(s.pressure_pa, 101_325.0);
        assert_eq!(s.temperature_k, 300.0);
        assert_eq!(s.mass_kg, 0.001);
        let expected_pressure = ideal_gas_pressure(0.001, 300.0, 0.000_85);
        assert!((s.pressure_pa - expected_pressure).abs() > 1.0e-3, "inputs are not an ideal-gas pair");
        assert!(s.is_valid());
    }

    #[test]
    fn from_mass_temperature_volume_derives_pressure() {
        let s = GasState::from_mass_temperature_volume(0.001, 300.0, 0.000_85);
        assert!(s.is_valid());
        let ana = ideal_gas_pressure(0.001, 300.0, 0.000_85);
        assert!((s.pressure_pa - ana).abs() < 1.0e-3);
    }

    #[test]
    fn density_is_mass_over_volume() {
        let s = air_state();
        assert!((s.density_kg_m3() - (0.001 / 0.000_85)).abs() < 1.0e-6);
    }

    #[test]
    fn speed_of_sound_is_positive_and_increases_with_temperature() {
        let cold = air_state();
        let hot = GasState::new(101_325.0, 1_200.0, 0.001, 0.000_85);
        let c_cold = cold.speed_of_sound_mps();
        let c_hot = hot.speed_of_sound_mps();
        assert!(c_cold.is_finite() && c_cold > 0.0);
        assert!(c_hot > c_cold);
        let expected_cold = (GAMMA * R_SPECIFIC_AIR * 300.0).sqrt();
        assert!((c_cold - expected_cold).abs() < 1.0e-3);
    }

    #[test]
    fn internal_energy_scales_with_temperature() {
        let s = air_state();
        let per_kg = s.specific_internal_energy_j_kg();
        let total = s.internal_energy_j();
        assert!((per_kg - CV_AIR * 300.0).abs() < 1.0e-3);
        assert!((total - 0.001 * CV_AIR * 300.0).abs() < 1.0e-3);
    }

    #[test]
    fn representative_states_are_valid() {
        // Idle charge.
        assert!(GasState::new(101_325.0, 300.0, 0.001, 0.001).is_valid());
        // Compression end, realistic V10.
        assert!(GasState::new(1_800_000.0, 700.0, 0.001, 0.000_1).is_valid());
        // Post-combustion peak.
        assert!(GasState::new(9_000_000.0, 2_800.0, 0.001, 0.000_12).is_valid());
    }

    #[test]
    fn invalid_states_are_rejected() {
        assert!(!GasState::new(101_325.0, 300.0, 0.001, 0.0).is_valid()); // zero volume
        assert!(!GasState::new(-1.0, 300.0, 0.001, 0.001).is_valid()); // -ve pressure
        assert!(!GasState::new(f32::NAN, 300.0, 0.001, 0.001).is_valid()); // NaN
        assert!(!GasState::new(101_325.0, 50.0, 0.001, 0.001).is_valid()); // cold
        assert!(!GasState::new(101_325.0, 300.0, -0.001, 0.001).is_valid()); // -ve mass
    }
}
