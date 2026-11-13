//! Per-sample procedural V10 frame carrying engine, cylinder, and injected vehicle stems.

use crate::physics::CYLINDER_COUNT;

/// Complete per-sample output of [`crate::synthesis::V10Engine`], consumed directly
/// by [`crate::acoustics::AcousticScene`].
#[derive(Clone, Copy, Debug, Default)]
pub struct EngineFrame {
    pub throttle: f32,
    pub load: f32,
    pub combustion_source: f32,
    pub pressure_derivative: f32,
    pub pressure_derivative_a: f32,
    pub pressure_derivative_b: f32,
    pub cylinder_pressure: [f32; CYLINDER_COUNT],
    pub cylinder_pressure_derivative: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_pressure_pa: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_temperature_k: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_heat_release_rate: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_phase: [u8; CYLINDER_COUNT],
    pub cylinder_blowdown: [f32; CYLINDER_COUNT],
    pub cylinder_headers: [f32; CYLINDER_COUNT],
    pub cylinder_valve_lift: [f32; CYLINDER_COUNT],
    pub cylinder_valve_area: [f32; CYLINDER_COUNT],
    /// Per-cylinder physical exhaust mass flow (kg/s) from the runner boundary.
    pub cylinder_mass_flow: [f32; CYLINDER_COUNT],
    /// Per-cylinder exhaust-runner gas pressure (Pa).
    pub cylinder_runner_pressure: [f32; CYLINDER_COUNT],
    /// Per-cylinder dynamic acoustic pressure perturbation in the runner waveguide.
    pub cylinder_runner_acoustic_pressure: [f32; CYLINDER_COUNT],
    /// Per-cylinder exhaust-runner gas temperature (K).
    pub cylinder_runner_temperature_k: [f32; CYLINDER_COUNT],
    /// Per-cylinder normalised header excitation fed into the runner waveguide.
    pub cylinder_exhaust_excitation: [f32; CYLINDER_COUNT],
    pub pressure_direct: f32,
    pub crankcase: f32,
    pub block: f32,
    pub head: f32,
    pub block_head: f32,
    pub headers_a: f32,
    pub headers_b: f32,
    pub collector_a: f32,
    pub collector_b: f32,
    pub collector_pressure_a: f32,
    pub collector_pressure_b: f32,
    pub exhaust: f32,
    pub turbulence: f32,
    pub master_pre_limiter: f32,
    pub master: f32,
    /// Structure-borne transmission and gearbox radiation injected by `Gf509Runtime`.
    pub transmission: f32,
    /// Gear up/down one-shot blip samples injected by `Gf509Runtime`.
    pub gear_shift: f32,
    pub fired_mask: u16,
    pub crank_phase_deg: f32,
    pub limiter_reduction_db: f32,
}
