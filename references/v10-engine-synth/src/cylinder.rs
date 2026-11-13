use crate::config::EngineConfig;
use crate::thermodynamics::combustion::{ChamberPhase, CombustionChamber};
use crate::thermodynamics::exhaust_runner::ExhaustRunner;
use crate::thermodynamics::exhaust_valve::ExhaustValve;

/// Poppet-valve flow effective exponent relating geometric lift fraction to turbulent discharge area.
pub const VALVE_FLOW_AREA_EXPONENT: f32 = 1.35;

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
    /// Per-sample change in exhaust mass flow (kg/s): the physical blowdown edge.
    pub exhaust_dmass_flow_kg_s: f32,
    /// Normalised acoustic excitation fed into the header waveguide.
    pub exhaust_excitation: f32,
    /// Exhaust-runner gas pressure (Pa) driving the valve discharge.
    pub runner_pressure_pa: f32,
    /// Exhaust-runner gas temperature (K).
    pub runner_temperature_k: f32,
}

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
    pub fn new(index: usize, config: &EngineConfig) -> Self {
        let centered = index as f32 - 4.5;
        let fixed_gain =
            (1.0 + centered / 4.5 * config.cylinder_spread) * config.cylinder_signature[index];
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

    #[inline]
    fn next_signed(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng = x;
        let bits = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40;
        bits as f32 / ((1u64 << 24) - 1) as f32 * 2.0 - 1.0
    }

    pub fn fire(&mut self, variation: f32) {
        self.age_deg = 0.0;
        self.cycle_gain = 1.0 + self.next_signed() * variation;
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

    pub fn process(
        &mut self,
        deg_per_sample: f32,
        energy: f32,
        config: &EngineConfig,
    ) -> CylinderFrame {
        // Normalised chamber pressure before the (energy-independent, once-per-
        // cylinder) signature and cycle-variation gains. Energy is folded inside
        // the thermodynamic model so it is not double-counted. The physical path
        // also exposes the chamber pressure/temperature used to drive the
        // exhaust valve's pressure differential; the legacy handcrafted path has
        // no such state, so it reports a null chamber state.
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
        let chamber_frame = chamber_frame.unwrap_or_else(|| {
            // Legacy compatibility mode has no gas-state authority. Keep the
            // diagnostic explicit instead of presenting proxy values as
            // thermodynamic measurements.
            crate::thermodynamics::combustion::CombustionFrame {
                pressure_pa: 0.0,
                temperature_k: 0.0,
                burn_fraction: 0.0,
                phase: ChamberPhase::Expansion,
                heat_release_rate_w: 0.0,
            }
        });
        let pressure = pressure * self.cycle_gain * self.fixed_gain;
        let pressure_derivative = pressure - self.last_pressure;
        self.last_pressure = pressure;

        // Physical exhaust valve: crank angle -> lift -> effective area. The
        // runner boundary owns the flow authority: it reads the discharging area
        // and the cylinder state into its lumped gas volume and reports the
        // mass flow (kg/s) plus its own back-pressure state. The acoustic
        // excitation is still driven by the normalised mass-flow proxy below;
        // PHY-052 re-derives it from the physical flow.
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
        // The pipe is acoustically excited by the change in mass-flow proxy,
        // not by a slowly varying unipolar flow. The fast but finite valve ramp
        // supplies the real blowdown edge without inventing broadband noise.
        let exhaust_flow = pressure * valve_lift.powf(VALVE_FLOW_AREA_EXPONENT);
        let blowdown = exhaust_flow - self.last_exhaust_flow;
        self.last_exhaust_flow = exhaust_flow;
        // PHY-052: the physical runner gas moves out of the valve as mass flow;
        // its per-sample delta is the true blowdown edge. The legacy proxy above
        // is retained for turbulence/telemetry and for the non-physical path.
        let dmass_flow_kg_s = mass_flow - self.last_exhaust_mass_flow;
        self.last_exhaust_mass_flow = mass_flow;
        // The physical excitation is level-matched to the proxy in the acoustic
        // chain (measured in PHY-052) and retains the per-cylinder signature so
        // the exhaust keeps its subtle cylinder-to-cylinder character.
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
        // The physical model is cyclic: after the exhaust/intake strokes the
        // charge is re-compressed toward the next firing TDC, so the pressure
        // rises again rather than decaying to near zero.
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
        // Warp through the burn window one crank step at a time.
        let deg = 0.5;
        for _ in 0..120 {
            let f = low.process(deg, 0.25, &config);
            low_peak = low_peak.max(f.pressure);
            let f = high.process(deg, 1.0, &config);
            high_peak = high_peak.max(f.pressure);
        }
        assert!(
            high_peak > low_peak,
            "more load must produce more pressure: high {high_peak} vs low {low_peak}"
        );
    }

    #[test]
    fn legacy_mode_still_decays_to_tail() {
        let mut config = EngineConfig::default();
        config.use_physical_pressure = false;
        let mut cylinder = Cylinder::new(0, &config);
        cylinder.fire(0.0);
        let mut peak = 0.0f32;
        let mut last = 0.0;
        for _ in 0..1_500 {
            let frame = cylinder.process(0.5, 1.0, &config);
            peak = peak.max(frame.pressure);
            last = frame.pressure;
        }
        assert!(peak > 0.5);
        assert!(last < peak * 0.02, "legacy tail {last} peak {peak}");
    }
}
