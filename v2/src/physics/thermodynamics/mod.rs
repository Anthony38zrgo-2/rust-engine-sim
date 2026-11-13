//! Modelos termodinámicos de cámara, válvulas y runners de escape.

pub mod combustion;
pub mod exhaust_runner;
pub mod exhaust_valve;
pub mod gas;
pub mod polytrope;

pub use combustion::{
    ChamberPhase, CombustionChamber, CombustionFrame, INTAKE_PRESSURE_PA, INTAKE_TEMPERATURE_K,
    WIEBE_A, WIEBE_M,
};
pub use exhaust_runner::{ExhaustRunner, RunnerState};
pub use exhaust_valve::ExhaustValve;
pub use gas::{ideal_gas_pressure, GasState, CV_AIR, GAMMA, R_SPECIFIC_AIR};
pub use polytrope::PolytropicProcess;
