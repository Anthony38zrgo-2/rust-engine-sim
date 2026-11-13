//! Lumped exhaust-runner gas boundary driving compressible valve discharge.

use crate::config::physical::{
    GAMMA, GAS_CONSTANT_J_PER_MOL_K, MOLAR_MASS_EXHAUST_KG_MOL, SPECIFIC_GAS_CONSTANT_J_PER_KG_K,
};
use crate::config::EngineConfig;

/// Near-port runner volume as a fraction of the acoustic header volume.
const RUNNER_VOLUME_FRACTION_OF_HEADER: f32 = 0.1;
/// Downstream atmosphere reference pressure (Pa).
const OUTLET_PRESSURE_PA: f32 = 101_325.0;
/// Downstream exhaust reference temperature (K).
const OUTLET_TEMPERATURE_K: f32 = 1_000.0;
/// Relaxation time constant toward the outlet boundary (s).
const OUTLET_BLEED_TIME_S: f32 = 0.004;

/// Instantaneous exhaust-runner gas pressure and temperature.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunnerState {
    pub pressure_pa: f32,
    pub temperature_k: f32,
}

/// Stateful lumped exhaust-runner gas volume immediately downstream of the valve.
#[derive(Clone, Copy, Debug)]
pub struct ExhaustRunner {
    volume_m3: f32,
    n_mol: f32,
    temperature_k: f32,
}

impl ExhaustRunner {
    /// Initializes runner gas volume at outlet pressure and temperature for cylinder `index`.
    pub fn new(config: &EngineConfig, index: usize) -> Self {
        let header_length_m = config.effective_header_lengths_m()[index];
        let volume_m3 = config.bore_area_m2() * header_length_m * RUNNER_VOLUME_FRACTION_OF_HEADER;
        Self {
            volume_m3,
            n_mol: OUTLET_PRESSURE_PA * volume_m3
                / (GAS_CONSTANT_J_PER_MOL_K * OUTLET_TEMPERATURE_K),
            temperature_k: OUTLET_TEMPERATURE_K,
        }
    }

    /// Returns the instantaneous runner gas state.
    #[inline]
    pub fn state(&self) -> RunnerState {
        RunnerState {
            pressure_pa: self.pressure(),
            temperature_k: self.temperature_k,
        }
    }

    #[inline]
    fn pressure(&self) -> f32 {
        if self.temperature_k > 0.0 {
            self.n_mol * GAS_CONSTANT_J_PER_MOL_K * self.temperature_k / self.volume_m3
        } else {
            0.0
        }
    }

    /// Evaluates compressible choked/subsonic mass flow (`kg/s`) across `effective_area_m2`.
    #[inline]
    pub fn mass_flow(
        &self,
        cylinder_pressure_pa: f32,
        cylinder_temperature_k: f32,
        effective_area_m2: f32,
    ) -> f32 {
        if effective_area_m2 <= 0.0 {
            return 0.0;
        }
        let runner_pressure = self.pressure();
        let runner_temperature = self.temperature_k;
        let (up_p, up_t, down_p, direction) = if cylinder_pressure_pa >= runner_pressure {
            (
                cylinder_pressure_pa,
                cylinder_temperature_k,
                runner_pressure,
                1.0,
            )
        } else {
            (
                runner_pressure,
                runner_temperature,
                cylinder_pressure_pa,
                -1.0,
            )
        };
        if up_p <= 0.0 || up_t <= 0.0 {
            return 0.0;
        }
        let p_ratio = (down_p / up_p).clamp(0.0, 1.0);
        let flux_per_m2 = up_p
            * (GAMMA / (SPECIFIC_GAS_CONSTANT_J_PER_KG_K * up_t)).sqrt()
            * compressible_flow_factor(p_ratio);
        direction * effective_area_m2 * flux_per_m2
    }

    /// Advances runner gas moles and temperature by `dt_s` with outlet bleed relaxation.
    #[inline]
    pub fn step(&mut self, mass_flow_kg_s: f32, dt_s: f32, inlet_temperature_k: f32) {
        if dt_s <= 0.0 {
            return;
        }
        let dm_mol = mass_flow_kg_s * dt_s / MOLAR_MASS_EXHAUST_KG_MOL;
        let source_temperature_k = if mass_flow_kg_s >= 0.0 {
            inlet_temperature_k
        } else {
            self.temperature_k
        };
        let n_new = self.n_mol + dm_mol;
        if n_new > 0.0 {
            self.temperature_k =
                (self.n_mol * self.temperature_k + dm_mol * source_temperature_k) / n_new;
        }
        self.n_mol = n_new.max(0.0);
        let equilibrium_mol = if self.temperature_k > 0.0 {
            OUTLET_PRESSURE_PA * self.volume_m3 / (GAS_CONSTANT_J_PER_MOL_K * self.temperature_k)
        } else {
            0.0
        };
        if self.n_mol > equilibrium_mol {
            let dn = ((self.n_mol - equilibrium_mol) * (dt_s / OUTLET_BLEED_TIME_S))
                .min(self.n_mol - equilibrium_mol);
            self.n_mol -= dn;
        }
    }
}

#[inline]
fn compressible_flow_factor(p_ratio: f32) -> f32 {
    let choked_limit = (2.0 / (GAMMA + 1.0)).powf(GAMMA / (GAMMA - 1.0));
    if p_ratio <= choked_limit {
        (2.0 / (GAMMA + 1.0)).powf((GAMMA + 1.0) / (2.0 * (GAMMA - 1.0)))
    } else {
        let s = p_ratio.powf(1.0 / GAMMA);
        let term = (2.0 / (GAMMA - 1.0)) * (1.0 - p_ratio.powf((GAMMA - 1.0) / GAMMA));
        s * term.max(0.0).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn runner() -> ExhaustRunner {
        ExhaustRunner::new(&EngineConfig::default(), 0)
    }

    #[test]
    fn compressible_factor_reaches_known_choked_coefficient() {
        let choked_limit = (2.0 / (GAMMA + 1.0)).powf(GAMMA / (GAMMA - 1.0));
        let expected = (2.0 / (GAMMA + 1.0)).powf((GAMMA + 1.0) / (2.0 * (GAMMA - 1.0)));
        assert!((compressible_flow_factor(choked_limit) - expected).abs() < 1.0e-6);
        assert!((compressible_flow_factor(0.0) - expected).abs() < 1.0e-6);
        assert!(expected > 0.0 && expected < 1.0);
    }

    #[test]
    fn compressible_factor_is_zero_at_equal_pressure_and_monotonic() {
        assert!((compressible_flow_factor(1.0)).abs() < 1.0e-6);
        let mut previous = compressible_flow_factor(0.01);
        for ratio in (2..=99).map(|v| v as f32 / 100.0) {
            let factor = compressible_flow_factor(ratio);
            assert!(factor <= previous + 1.0e-6 && factor >= 0.0);
            previous = factor;
        }
    }

    #[test]
    fn starts_at_the_outlet_reference() {
        let r = runner();
        let s = r.state();
        assert!((s.pressure_pa - OUTLET_PRESSURE_PA).abs() < 1.0);
        assert!((s.temperature_k - OUTLET_TEMPERATURE_K).abs() < 1.0);
    }

    #[test]
    fn mass_flow_is_zero_when_valve_closed() {
        let r = runner();
        assert_eq!(r.mass_flow(8_000_000.0, 1_900.0, 0.0), 0.0);
    }

    #[test]
    fn forward_flow_raises_runner_pressure_within_bounds() {
        let mut r = runner();
        let area = 0.0005;
        let inlet = 1_500.0;
        let mut last_pressure = r.state().pressure_pa;
        for _ in 0..2_000 {
            let flow = r.mass_flow(8_000_000.0, 1_900.0, area);
            assert!(flow >= 0.0);
            r.step(flow, 1.0 / 48_000.0, inlet);
            last_pressure = r.state().pressure_pa;
            assert!(last_pressure.is_finite());
        }
        assert!(last_pressure > OUTLET_PRESSURE_PA * 1.5);
    }

    #[test]
    fn flow_amounts_to_a_finite_positive_mass_flux() {
        let r = runner();
        let flow = r.mass_flow(8_000_000.0, 1_900.0, 0.0005);
        assert!(flow.is_finite() && flow > 1.0 && flow < 100.0);
    }

    #[test]
    fn reverse_flow_occurs_when_runner_is_pressurized_above_cylinder() {
        let mut r = runner();
        for _ in 0..1_000 {
            let flow = r.mass_flow(8_000_000.0, 1_900.0, 0.0005);
            r.step(flow, 1.0 / 48_000.0, 1_500.0);
        }
        let flow = r.mass_flow(120_000.0, 1_000.0, 0.0005);
        assert!(flow < 0.0);
    }

    #[test]
    fn runner_relaxes_toward_outlet_when_empty() {
        let mut r = runner();
        let mut last = r.state().pressure_pa;
        for _ in 0..2_000 {
            let flow = r.mass_flow(8_000_000.0, 1_900.0, 0.0005);
            r.step(flow, 1.0 / 48_000.0, 1_500.0);
            last = r.state().pressure_pa;
        }
        let pressurized = last;
        assert!(pressurized > OUTLET_PRESSURE_PA * 1.5);
        for _ in 0..20_000 {
            r.step(0.0, 1.0 / 48_000.0, 1_500.0);
        }
        let relaxed = r.state().pressure_pa;
        assert!(relaxed < pressurized * 0.5);
    }

    #[test]
    fn longer_primary_volume_damps_equal_blowdown_more() {
        let config = EngineConfig {
            header_length_scale: 1.5,
            ..EngineConfig::default()
        };
        let mut base = runner();
        let mut scaled = ExhaustRunner::new(&config, 0);
        let dt = 1.0 / 48_000.0;
        for _ in 0..200 {
            base.step(0.01, dt, 1_500.0);
            scaled.step(0.01, dt, 1_500.0);
        }
        assert!(base.state().pressure_pa > scaled.state().pressure_pa);
    }
}
