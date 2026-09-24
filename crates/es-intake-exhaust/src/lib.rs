//! Intake plenum and exhaust collector systems.

use es_gas::{GasSystem, Mix};
use es_units::{self as units, PI};

// ---------------------------------------------------------------------------
// Intake
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct IntakeParams {
    pub volume: f64,
    pub cross_section_area: f64,
    pub input_flow_k: f64,
    pub idle_flow_k: f64,
    pub runner_flow_rate: f64,
    pub molecular_afr: f64,
    pub idle_throttle_plate_position: f64,
    pub runner_length: f64,
    pub velocity_decay: f64,
}

impl Default for IntakeParams {
    fn default() -> Self {
        Self {
            volume: units::ATM * 0.001, // placeholder, real value from config
            cross_section_area: 0.001,
            input_flow_k: 0.0,
            idle_flow_k: 0.0,
            runner_flow_rate: 0.0,
            molecular_afr: 25.0 / 2.0,
            idle_throttle_plate_position: 0.975,
            runner_length: units::INCH * 4.0,
            velocity_decay: 0.5,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Intake {
    pub system: GasSystem,
    pub atmosphere: GasSystem,
    pub throttle: f64,
    pub flow: f64,
    pub flow_rate: f64,
    pub total_fuel_injected: f64,
    params: IntakeParams,
}

impl Intake {
    pub fn new(p: IntakeParams) -> Self {
        let width = p.cross_section_area.sqrt();
        let mut system = GasSystem::new();
        system.initialize(units::ATM, p.volume, units::celsius(25.0), Mix::AIR, 5);
        system.set_geometry(width, p.volume / p.cross_section_area, 1.0, 0.0);

        let mut atmosphere = GasSystem::new();
        atmosphere.initialize(units::ATM, 1000.0, units::celsius(25.0), Mix::AIR, 5);
        atmosphere.set_geometry(100.0, 100.0, 1.0, 0.0);

        Self {
            system,
            atmosphere,
            throttle: 1.0,
            flow: 0.0,
            flow_rate: 0.0,
            total_fuel_injected: 0.0,
            params: p,
        }
    }

    pub fn runner_flow_rate(&self) -> f64 {
        self.params.runner_flow_rate
    }

    pub fn throttle_plate_position(&self) -> f64 {
        self.params.idle_throttle_plate_position * self.throttle
    }

    pub fn runner_length(&self) -> f64 {
        self.params.runner_length
    }

    pub fn cross_section_area(&self) -> f64 {
        self.params.cross_section_area
    }

    pub fn velocity_decay(&self) -> f64 {
        self.params.velocity_decay
    }

    pub fn process(&mut self, dt: f64) {
        let ideal_afr = 0.8 * self.params.molecular_afr * 4.0;
        let mix = self.system.mix();
        let current_afr_denom = mix.p_fuel;
        let _ = (ideal_afr, current_afr_denom);

        let p_air = ideal_afr / (1.0 + ideal_afr);
        let fuel_air_mix = Mix {
            p_fuel: 1.0 - p_air,
            p_inert: p_air * 0.75,
            p_o2: p_air * 0.25,
        };

        let idle_afr = 2.0;
        let p_idle_air = idle_afr / (1.0 + idle_afr);
        let fuel_mix = Mix {
            p_fuel: 1.0 - p_idle_air,
            p_inert: p_idle_air * 0.75,
            p_o2: p_idle_air * 0.25,
        };

        let throttle = self.throttle_plate_position();
        let flow_attenuation = (throttle * PI / 2.0).cos();

        // Atmosphere → plenum (throttle)
        self.atmosphere
            .reset(units::ATM, units::celsius(25.0), fuel_air_mix);
        {
            let mut params = es_gas::FlowParams {
                k_flow: flow_attenuation * self.params.input_flow_k,
                dt,
                direction: (0.0, -1.0),
                cross_section_0: 10.0,
                cross_section_1: self.params.cross_section_area,
                system_0: &mut self.atmosphere,
                system_1: &mut self.system,
            };
            // C++ calls m_system.flow(flowParams) with system_0=atm, system_1=plenum
            // but method is on m_system - flow uses system_0→system_1 direction via pressures
            self.flow = es_gas::GasSystem::flow_between(&mut params);
        }

        // Idle circuit
        self.atmosphere
            .reset(units::ATM, units::celsius(25.0), fuel_mix);
        let idle_flow = {
            let mut params = es_gas::FlowParams {
                k_flow: self.params.idle_flow_k,
                dt,
                direction: (0.0, -1.0),
                cross_section_0: 10.0,
                cross_section_1: self.params.cross_section_area,
                system_0: &mut self.atmosphere,
                system_1: &mut self.system,
            };
            es_gas::GasSystem::flow_between(&mut params)
        };

        self.system.dissipate_excess_velocity();
        self.system.update_velocity(dt, self.params.velocity_decay);

        // Fuel accounting: positive flow = fuel entering plenum from atm (C++ m_flow > 0)
        if self.flow > 0.0 {
            self.total_fuel_injected += fuel_air_mix.p_fuel * self.flow;
        }
        if idle_flow > 0.0 {
            self.total_fuel_injected += fuel_mix.p_fuel * idle_flow;
        }
    }
}

// ---------------------------------------------------------------------------
// Exhaust
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct ExhaustParams {
    pub length: f64,
    pub collector_cross_section: f64,
    pub outlet_flow_rate: f64,
    pub primary_tube_length: f64,
    pub primary_flow_rate: f64,
    pub velocity_decay: f64,
    pub audio_volume: f64,
    /// Path to impulse response WAV (relative to assets)
    pub impulse_response: Option<String>,
}

#[derive(Clone, Debug)]
pub struct ExhaustSystem {
    pub atmosphere: GasSystem,
    pub system: GasSystem,
    pub index: usize,
    pub flow: f64,
    params: ExhaustParams,
}

impl ExhaustSystem {
    pub fn new(index: usize, p: ExhaustParams) -> Self {
        let width = p.collector_cross_section.sqrt();
        let volume = p.collector_cross_section * p.length;
        let mut system = GasSystem::new();
        system.initialize(units::ATM, volume, units::celsius(25.0), Mix::AIR, 5);
        system.set_geometry(p.length, width, 1.0, 0.0);

        let mut atmosphere = GasSystem::new();
        atmosphere.initialize(units::ATM, 1000.0, units::celsius(25.0), Mix::AIR, 5);
        atmosphere.set_geometry(10.0, 10.0, 1.0, 0.0);

        Self {
            atmosphere,
            system,
            index,
            flow: 0.0,
            params: p,
        }
    }

    pub fn length(&self) -> f64 {
        self.params.length
    }

    pub fn audio_volume(&self) -> f64 {
        self.params.audio_volume
    }

    pub fn primary_flow_rate(&self) -> f64 {
        self.params.primary_flow_rate
    }

    pub fn collector_cross_section(&self) -> f64 {
        self.params.collector_cross_section
    }

    pub fn primary_tube_length(&self) -> f64 {
        self.params.primary_tube_length
    }

    pub fn velocity_decay(&self) -> f64 {
        self.params.velocity_decay
    }

    pub fn impulse_response(&self) -> Option<&str> {
        self.params.impulse_response.as_deref()
    }

    pub fn process(&mut self, dt: f64) {
        let air_mix = Mix {
            p_fuel: 0.0,
            p_inert: 1.0,
            p_o2: 0.0,
        };
        self.atmosphere
            .reset(units::ATM, units::celsius(25.0), air_mix);

        {
            let mut params = es_gas::FlowParams {
                k_flow: self.params.outlet_flow_rate,
                dt,
                direction: (1.0, 0.0),
                cross_section_0: self.params.collector_cross_section,
                cross_section_1: 10.0,
                system_0: &mut self.atmosphere,
                system_1: &mut self.system,
            };
            // C++: flowParams.system_0 = atmosphere, system_1 = m_system,
            // then m_system.flow(flowParams) — flow() static uses system_0→1
            self.flow = es_gas::GasSystem::flow_between(&mut params);
        }

        self.system.dissipate_excess_velocity();
        self.system.update_velocity(dt, self.params.velocity_decay);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intake_initializes_at_atm() {
        let mut intake = Intake::new(IntakeParams {
            volume: 0.001,
            ..Default::default()
        });
        assert!((intake.system.pressure() - units::ATM).abs() < 1.0);
        intake.process(1e-5);
        assert!(intake.system.pressure().is_finite());
    }

    #[test]
    fn exhaust_holds_volume() {
        let mut ex = ExhaustSystem::new(
            0,
            ExhaustParams {
                length: 0.5,
                collector_cross_section: 0.002,
                outlet_flow_rate: 0.0,
                primary_tube_length: 0.3,
                primary_flow_rate: 0.0,
                velocity_decay: 1.0,
                audio_volume: 1.0,
                impulse_response: None,
            },
        );
        let v0 = ex.system.volume();
        ex.process(1e-5);
        assert!((ex.system.volume() - v0).abs() < 1e-12);
    }
}
