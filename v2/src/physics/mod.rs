//! Simulación mecánica y termodinámica del motor (sin WAV ni buses de salida).

pub mod crank;
pub mod cylinder;
pub mod geometry;
pub mod thermodynamics;

pub use crank::{Crankshaft, FireEvents, CYCLE_DEG, CYLINDER_COUNT};
pub use cylinder::{Cylinder, CylinderFrame, VALVE_FLOW_AREA_EXPONENT};
pub use geometry::{SliderCrank, VOLUME_TABLE_SAMPLES};
pub use thermodynamics::{
    ChamberPhase, CombustionChamber, CombustionFrame, ExhaustRunner, ExhaustValve, GasState,
    PolytropicProcess, RunnerState,
};
