//! Rutas de radiación estructural y acústica hacia el oyente (`AcousticScene`).

pub mod cover;
pub mod mounts;
pub mod paths;
pub mod scene;

pub use crate::config::AcousticSceneConfig;
pub use cover::EngineCover;
pub use mounts::EngineMountMonocoque;
pub use paths::{AirPath, CylinderMechanicalPath};
pub use scene::{AcousticFrame, AcousticScene};
