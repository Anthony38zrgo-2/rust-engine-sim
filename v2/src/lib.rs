#![forbid(unsafe_code)]

//! v10-engine-synth (v2): arquitectura modular por responsabilidades únicas.
//!
//! La fachada pública conserva los reexports compatibles de `v1` mientras los
//! módulos internos separan estrictamente hilo de control/carga y hilo de audio.

pub mod acoustics;
pub mod assets;
pub mod config;
pub mod control;
pub mod drivetrain;
pub mod dsp;
pub mod offline;
pub mod physics;
pub mod runtime;
pub mod samples;
pub mod synthesis;

pub use acoustics::{AcousticFrame, AcousticScene, EngineCover, EngineMountMonocoque};
pub use assets::{AssetError, BankManifest, GF509_HEADROOM_GAIN};
pub use config::{
    AcousticMixConfig, AcousticSceneConfig, CollectorGeometry, EngineConfig, EngineMode,
    EngineStructuralProfile, Gf509RuntimeConfig, PhysicalEngineConfig, ThreeZoneSampleLayerConfig,
    TransmissionConfig, ZONE_TRIM_SLOTS,
};
pub use control::{
    shift_energy_gain, EngineInput, RuntimeTelemetry, ShiftGesture, ShiftPhase, TorqueSign,
};
pub use drivetrain::{GearShiftPlayer, GearShiftSamples, TransmissionInput, TransmissionSynth};
pub use dsp::{SpectrumChain, SpectrumChainConfig, UpperMidShelf};
pub use physics::{
    ChamberPhase, CombustionChamber, CombustionFrame, Crankshaft, Cylinder, CylinderFrame,
    ExhaustRunner, ExhaustValve, FireEvents, GasState, PolytropicProcess, RunnerState, SliderCrank,
    CYLINDER_COUNT,
};
pub use runtime::{blend_hybrid, AudioRenderer, EngineSource, Gf509Runtime};
pub use samples::{
    PreparedSampleBank, SampleBankPlayer, SampleLayerFrame, SampleLayerInput, ThreeZoneSampleLayer,
};
pub use synthesis::{
    Collector, CollectorFrame, EngineFrame, RunnerAcousticState, RunnerWaveguide, V10Engine,
};
