//! Orquestación acústica procedural del motor (`V10Engine`, waveguides, colectores, ruido).

pub mod exhaust;
pub mod frame;
pub mod procedural;
pub mod structural;
pub mod turbulence;

pub use exhaust::{
    Collector, CollectorFrame, RunnerAcousticState, RunnerWaveguide,
    COLLECTOR_CHAMBER_INFLOW_COEFF, COLLECTOR_CONFLUENCE_LOSS_COEFF, RUNNER_WALL_LOSS_COEFF,
};
pub use frame::EngineFrame;
pub use procedural::V10Engine;
pub use structural::{BlockHead, StructureFrame};
pub use turbulence::TurbulenceGenerator;
