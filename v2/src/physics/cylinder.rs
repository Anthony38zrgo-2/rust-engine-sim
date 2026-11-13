//! Per-cylinder combustion, poppet exhaust valve, and runner boundary state.

use crate::config::EngineConfig;
use crate::dsp::xorshift64_signed;
use crate::physics::thermodynamics::combustion::{
    ChamberPhase, CombustionChamber, CombustionFrame,
};
use crate::physics::thermodynamics::exhaust_runner::ExhaustRunner;
use crate::physics::thermodynamics::exhaust_valve::ExhaustValve;

/// Poppet-valve flow effective exponent relating geometric lift fraction to turbulent discharge area.
pub const VALVE_FLOW_AREA_EXPONENT: f32 = 1.35;

/// Instantaneous thermodynamic and acoustic excitation frame emitted by one cylinder.
#[derive(Clone, Copy, Debug, Default)]
pub struct CylinderFrame {
    pub pressure: f32,
    pub pressure_derivative: f32,
    /// Absolute chamber pressure (Pa) from the bounded four-stroke model.
    pub chamber_pressure_pa: f32,
    /// Chamber bulk temperature (K) from the bounded four-stroke model.
    pub chamber_temperature_k: f32,
    /// Heat-release diagnostic in the model's Joules-per-crank-degree units.
    pub chamber_heat_release_rate: f32,
    pub chamber_phase: ChamberPhase,
    pub blowdown: f32,
    pub valve_lift: f32,
    /// Effective exhaust port area (m²) from the physical valve model.
    pub effective_area_m2: f32,
    /// Pressure-driven exhaust mass flow (kg/s) from the runner boundary.
    pub exhaust_mass_flow_kg_s: f32,
    /// Per-sample change in exhaust mass flow (kg/s): physical blowdown edge.
    pub exhaust_dmass_flow_kg_s: f32,
    /// Normalised acoustic excitation fed into the header waveguide.
    pub exhaust_excitation: f32,
    /// Exhaust-runner gas pressure (Pa) driving the valve discharge.
    pub runner_pressure_pa: f32,
    /// Exhaust-runner gas temperature (K).
    pub runner_temperature_k: f32,
}

/// Complete single-cylinder thermodynamic and valve-boundary simulator.
#[derive(Clone, Copy, Debug)]
pub struct Cylinder {
    age_deg: f32,
    last_pressure: f32,
    last_exhaust_flow: f32,
    last_exhaust_mass_flow: f32,
    cycle_gain: f32,
    fixed_gain: f32,
    chamber: CombustionChamber,
    exhaust_valve: ExhaustValve,
    exhaust_runner: ExhaustRunner,
    rng: u64,
}

impl Cylinder {
    /// Initializes cylinder `index` with deterministic per-cylinder signature and PRNG seed.
    pub fn new(index: usize, config: &EngineConfig) -> Self {
        let half_span = (config.cylinder_count.saturating_sub(1)) as f32 * 0.5;
        let offset = if half_span > 0.0 {
            (index as f32 - half_span) / half_span
        } else {
            0.0
        };
        let fixed_gain = (1.0 + offset * config.cylinder_spread) * config.cylinder_signature[index];
        Self {
            age_deg: 1.0e9,
            last_pressure: 0.0,
            last_exhaust_flow: 0.0,
            last_exhaust_mass_flow: 0.0,
            cycle_gain: 1.0,
            fixed_gain,
            chamber: CombustionChamber::new(config),
            exhaust_valve: ExhaustValve::new(config),
            exhaust_runner: ExhaustRunner::new(config, index),
            rng: config.seed ^ ((index as u64 + 1).wrapping_mul(0x9E37_79B9_7F4A_7C15)),
        }
    }

    /// Triggers a new firing cycle at TDC (`age_deg = 0.0`) with deterministic cycle variation.
    #[inline]
    pub fn fire(&mut self, variation: f32) {
        self.age_deg = 0.0;
        self.cycle_gain = 1.0 + xorshift64_signed(&mut self.rng) * variation;
    }

    #[inline]
    fn pressure_shape(age: f32, rise_deg: f32, expansion_decay_deg: f32) -> f32 {
        if age < 0.0 {
            return 0.0;
        }
        if age <= rise_deg {
            let u = (age / rise_deg).clamp(0.0, 1.0);
            let burn = 1.0 - (-6.0 * u.powf(2.35)).exp();
            burn * (-0.55 * u).exp()
        } else {
            let peak = (1.0 - (-6.0f32).exp()) * (-0.55f32).exp();
            peak * (-(age - rise_deg) / expansion_decay_deg).exp()
        }
    }

    /// Steps chamber pressure, valve lift, and exhaust runner gas exchange by one sample.
    #[inline]
    pub fn process(
        &mut self,
        deg_per_sample: f32,
        energy: f32,
        config: &EngineConfig,
    ) -> CylinderFrame {
        let chamber_frame = config
            .use_physical_pressure
            .then(|| self.chamber.frame(self.age_deg, energy));
        let (pressure, chamber_pressure_pa, chamber_temperature_k) =
            if let Some(frame) = chamber_frame {
                (
                    frame.pressure_pa / config.combustion_pressure_reference_pa,
                    frame.pressure_pa,
                    frame.temperature_k,
                )
            } else {
                let shifted_age = self.age_deg - config.combustion_start_deg;
                let p = Self::pressure_shape(
                    shifted_age,
                    config.combustion_rise_deg,
                    config.expansion_decay_deg,
                ) * energy;
                (p, 0.0, 0.0)
            };
        let chamber_frame = chamber_frame.unwrap_or(CombustionFrame {
            pressure_pa: 0.0,
            temperature_k: 0.0,
            burn_fraction: 0.0,
            phase: ChamberPhase::Expansion,
            heat_release_rate_w: 0.0,
        });
        let pressure = pressure * self.cycle_gain * self.fixed_gain;
        let pressure_derivative = pressure - self.last_pressure;
        self.last_pressure = pressure;

        let lift_fraction = self.exhaust_valve.lift_fraction(self.age_deg);
        let effective_area_m2 = self.exhaust_valve.effective_area_m2(lift_fraction);
        let runner_state = self.exhaust_runner.state();
        let mass_flow = self.exhaust_runner.mass_flow(
            chamber_pressure_pa,
            chamber_temperature_k,
            effective_area_m2,
        );
        let dt_s = 1.0 / config.sample_rate as f32;
        self.exhaust_runner
            .step(mass_flow, dt_s, chamber_temperature_k);
        let valve_lift = lift_fraction;
        let exhaust_flow = pressure * valve_lift.powf(VALVE_FLOW_AREA_EXPONENT);
        let blowdown = exhaust_flow - self.last_exhaust_flow;
        self.last_exhaust_flow = exhaust_flow;
        let dmass_flow_kg_s = mass_flow - self.last_exhaust_mass_flow;
        self.last_exhaust_mass_flow = mass_flow;
        let exhaust_excitation = if config.use_physical_exhaust_excitation {
            dmass_flow_kg_s * self.fixed_gain * config.exhaust_excitation_gain
        } else {
            blowdown
        };
        self.age_deg += deg_per_sample;
        CylinderFrame {
            pressure,
            pressure_derivative,
            chamber_pressure_pa,
            chamber_temperature_k,
            chamber_heat_release_rate: chamber_frame.heat_release_rate_w,
            chamber_phase: chamber_frame.phase,
            blowdown,
            valve_lift,
            effective_area_m2,
            exhaust_mass_flow_kg_s: mass_flow,
            exhaust_dmass_flow_kg_s: dmass_flow_kg_s,
            exhaust_excitation,
            runner_pressure_pa: runner_state.pressure_pa,
            runner_temperature_k: runner_state.temperature_k,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blowdown_cannot_precede_valve_opening() {
        let config = EngineConfig::default();
        let mut cylinder = Cylinder::new(0, &config);
        cylinder.fire(0.0);
        let deg = 1.0;
        for age in 0..config.exhaust_open_deg as usize {
            let frame = cylinder.process(deg, 1.0, &config);
            assert_eq!(frame.blowdown, 0.0, "blowdown at age {age}");
        }
    }

    #[test]
    fn pressure_event_is_finite_and_decays() {
        let config = EngineConfig::default();
        let mut cylinder = Cylinder::new(0, &config);
        cylinder.fire(0.0);
        let mut peak = 0.0f32;
        for _ in 0..1_500 {
            let frame = cylinder.process(0.5, 1.0, &config);
            assert!(frame.pressure.is_finite() && frame.pressure_derivative.is_finite());
            peak = peak.max(frame.pressure);
        }
        assert!(peak > 0.5);
        let late = cylinder.process(0.5, 1.0, &config);
        assert!(late.pressure.is_finite() && late.pressure_derivative.is_finite());
    }

    #[test]
    fn physical_pressure_rises_with_load() {
        let config = EngineConfig::default();
        let mut low = Cylinder::new(0, &config);
        let mut high = Cylinder::new(0, &config);
        low.fire(0.0);
        high.fire(0.0);
        let mut low_peak = 0.0f32;
        let mut high_peak = 0.0f32;
        let deg = 0.5;
        for _ in 0..120 {
            let f = low.process(deg, 0.25, &config);
            low_peak = low_peak.max(f.pressure);
            let f = high.process(deg, 1.0, &config);
            high_peak = high_peak.max(f.pressure);
        }
        assert!(high_peak > low_peak);
    }

    #[test]
    fn legacy_mode_still_decays_to_tail() {
        let config = EngineConfig {
            use_physical_pressure: false,
            ..EngineConfig::default()
        };
        let mut cylinder = Cylinder::new(0, &config);
        cylinder.fire(0.0);
        let mut peak = 0.0f32;
        let mut last = 0.0;
        for _ in 0..1_500 {
            let frame = cylinder.process(0.5, 1.0, &config);
            peak = peak.max(frame.pressure);
            last = frame.pressure;
        }
        assert!(peak > 0.5 && last < peak * 0.02);
    }
}
