//! Polytropic compression and expansion of a cylinder charge.
//!
//! Uses `P * V^n = constant` and `T * V^(n - 1) = constant` with `n = GAMMA`
//! by default for adiabatic dry air.

use crate::physics::thermodynamics::gas::{GasState, GAMMA};

/// Polytropic relation evaluating cylinder charge states across volume changes.
#[derive(Clone, Copy, Debug)]
pub struct PolytropicProcess {
    gamma: f32,
}

impl PolytropicProcess {
    /// Builds a process with polytropic index `gamma` (must be finite and `> 1.0`).
    pub fn new(gamma: f32) -> Result<Self, String> {
        if !gamma.is_finite() || gamma <= 1.0 {
            return Err("polytropic index must be finite and greater than 1".into());
        }
        Ok(Self { gamma })
    }

    /// Default adiabatic (isentropic) process for air.
    pub fn adiabatic() -> Self {
        Self { gamma: GAMMA }
    }

    pub fn gamma(&self) -> f32 {
        self.gamma
    }

    /// Pressure after moving from `volume_ref` (at `pressure_ref`) to `volume`.
    #[inline]
    pub fn pressure_at(&self, pressure_ref: f32, volume_ref: f32, volume: f32) -> f32 {
        pressure_ref * (volume_ref / volume).powf(self.gamma)
    }

    /// Temperature after moving from `volume_ref` (at `temperature_ref`) to `volume`.
    #[inline]
    pub fn temperature_at(&self, temperature_ref: f32, volume_ref: f32, volume: f32) -> f32 {
        temperature_ref * (volume_ref / volume).powf(self.gamma - 1.0)
    }

    /// Produces the charge state at `volume` from a reference state, conserving mass.
    #[inline]
    pub fn state_at(&self, reference: &GasState, volume: f32) -> GasState {
        GasState::new(
            self.pressure_at(reference.pressure_pa, reference.volume_m3, volume),
            self.temperature_at(reference.temperature_k, reference.volume_m3, volume),
            reference.mass_kg,
            volume,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::geometry::SliderCrank;
    use crate::physics::thermodynamics::gas::R_SPECIFIC_AIR;

    const BORE_M: f32 = 0.096;
    const STROKE_M: f32 = 0.042;
    const ROD_M: f32 = 0.135;
    const CR: f32 = 12.5;
    const REF_PA: f32 = 101_325.0;
    const REF_K: f32 = 300.0;

    fn anatomy() -> SliderCrank {
        SliderCrank::new(BORE_M, STROKE_M, ROD_M, CR)
    }

    fn intake_charge(crank: &SliderCrank) -> GasState {
        let v_bdc = crank.instantaneous_volume_m3(180.0);
        let mass = REF_PA * v_bdc / (R_SPECIFIC_AIR * REF_K);
        GasState::from_mass_temperature_volume(mass, REF_K, v_bdc)
    }

    #[test]
    fn invalid_polytropic_index_is_rejected() {
        assert!(PolytropicProcess::new(1.0).is_err());
        assert!(PolytropicProcess::new(0.5).is_err());
        assert!(PolytropicProcess::new(f32::NAN).is_err());
        assert!(PolytropicProcess::new(1.4).is_ok());
    }

    #[test]
    fn compression_from_bdc_raises_pressure_and_temperature() {
        let crank = anatomy();
        let charge = intake_charge(&crank);
        let process = PolytropicProcess::adiabatic();
        let v_tdc = crank.instantaneous_volume_m3(360.0);
        let compressed = process.state_at(&charge, v_tdc);

        assert!(compressed.volume_m3 < charge.volume_m3);
        assert!(compressed.pressure_pa > charge.pressure_pa);
        assert!(compressed.temperature_k > charge.temperature_k);
        assert_eq!(compressed.mass_kg, charge.mass_kg);
        assert!(compressed.is_valid());
    }

    #[test]
    fn expansion_from_tdc_lowers_pressure_and_temperature() {
        let crank = anatomy();
        let charge = intake_charge(&crank);
        let process = PolytropicProcess::adiabatic();
        let v_tdc = crank.instantaneous_volume_m3(360.0);
        let compressed = process.state_at(&charge, v_tdc);
        let v_bdc = crank.instantaneous_volume_m3(540.0);
        let expanded = process.state_at(&compressed, v_bdc);

        assert!(expanded.volume_m3 > compressed.volume_m3);
        assert!(expanded.pressure_pa < compressed.pressure_pa);
        assert!(expanded.temperature_k < compressed.temperature_k);
        assert_eq!(expanded.mass_kg, compressed.mass_kg);
        assert!(expanded.is_valid());
    }

    #[test]
    fn state_is_continuous_and_monotonic_over_compression_stroke() {
        let crank = anatomy();
        let charge = intake_charge(&crank);
        let process = PolytropicProcess::adiabatic();

        let mut prev_v = f32::MAX;
        let mut prev_p = f32::MIN;
        let mut prev_t = f32::MIN;
        for i in 1..=180 {
            let deg = 180.0 + i as f32;
            let state = process.state_at(&charge, crank.instantaneous_volume_m3(deg));
            assert!(state.is_valid());
            assert!(state.volume_m3 < prev_v);
            assert!(state.pressure_pa > prev_p);
            assert!(state.temperature_k > prev_t);
            prev_v = state.volume_m3;
            prev_p = state.pressure_pa;
            prev_t = state.temperature_k;
        }
    }

    #[test]
    fn pressure_times_volume_gamma_is_invariant() {
        let crank = anatomy();
        let charge = intake_charge(&crank);
        let process = PolytropicProcess::adiabatic();
        let gamma = process.gamma();

        let base_u = charge.pressure_pa * charge.volume_m3.powf(gamma);
        for deg in [180.0, 240.0, 300.0, 360.0] {
            let state = process.state_at(&charge, crank.instantaneous_volume_m3(deg));
            let u = state.pressure_pa * state.volume_m3.powf(gamma);
            assert!(((u - base_u).abs() / base_u).abs() < 1.0e-4);
        }
    }

    #[test]
    fn compression_ratio_state_matches_geometry() {
        let crank = anatomy();
        let charge = intake_charge(&crank);
        let process = PolytropicProcess::adiabatic();
        let v_tdc = crank.instantaneous_volume_m3(360.0);
        let compressed = process.state_at(&charge, v_tdc);
        let expected = (CR as f64).powf((process.gamma() - 1.0) as f64);
        let actual = (compressed.temperature_k / charge.temperature_k) as f64;
        assert!((actual - expected).abs() < 1.0e-3);
    }
}
