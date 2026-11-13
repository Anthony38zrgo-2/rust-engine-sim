#![forbid(unsafe_code)]

pub mod acoustics;
pub mod config;
pub mod crank;
pub mod cylinder;
pub mod engine;
pub mod exhaust;
pub mod gear_shift;
pub mod geometry;
pub mod runtime;
pub mod sample_layer;
pub mod scene;
pub mod thermodynamics;
pub mod tone;
pub mod transmission;
pub mod wav;

pub use config::{CollectorGeometry, EngineConfig};
pub use crank::Crankshaft;
pub use engine::{shift_energy_gain, EngineFrame, EngineInput, ShiftGesture, V10Engine};
pub use gear_shift::{GearShiftPlayer, GearShiftSamples};
pub use geometry::SliderCrank;
pub use runtime::{
    EngineMode, Gf509Runtime, Gf509RuntimeConfig, RuntimeTelemetry, ShiftPhase, TorqueSign,
};
pub use sample_layer::{
    SampleLayerFrame, SampleLayerInput, ThreeZoneSampleLayer, ThreeZoneSampleLayerConfig,
    ZONE_TRIM_SLOTS,
};
pub use scene::{
    AcousticFrame, AcousticScene, AcousticSceneConfig, EngineCover, EngineMountMonocoque,
};
pub use tone::{SpectrumChain, SpectrumChainConfig, UpperMidShelf};
pub use transmission::{TransmissionConfig, TransmissionInput, TransmissionSynth};
pub use thermodynamics::{
    ChamberPhase, CombustionChamber, CombustionFrame, ExhaustRunner, ExhaustValve, GasState,
    PolytropicProcess, RunnerState,
};
