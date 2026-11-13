//! Runtime mode, drivetrain transmission, and top-level GF509 runtime configurations.

use std::path::PathBuf;

use crate::config::acoustic::AcousticSceneConfig;
use crate::config::samples::ThreeZoneSampleLayerConfig;
use crate::config::EngineConfig;
use crate::dsp::spectrum::{SpectrumChainConfig, UpperMidShelf};

/// Continuous-engine source selector. `Hybrid` is the shipped GF509 behaviour
/// (procedural engine + sample layer blended before the shelf); `SampleOnly`
/// replaces the engine with the prepared bank and keeps only the gearbox
/// procedural, summed after the acoustic scene.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EngineMode {
    #[default]
    Hybrid,
    SampleOnly,
}

/// Configuration for procedural transmission whine, clack, rattle, and clutch noise.
#[derive(Clone, Debug, PartialEq)]
pub struct TransmissionConfig {
    pub sample_rate: u32,
    pub output_gain: f32,
    pub gear_ratios: Vec<f32>,
    pub reverse_ratio: f32,
    pub final_drive: f32,
    pub gear_teeth: f32,
    pub final_teeth: f32,
    pub whine_gain: f32,
    pub clack_gain: f32,
    pub rattle_gain: f32,
    pub clutch_gain: f32,
    pub shaft_slew_seconds: f32,
    pub seed: u64,
    /// Enables tooth-mesh phase jitter and contact micro-modulation.
    pub use_refined_dsp: bool,
}

impl Default for TransmissionConfig {
    fn default() -> Self {
        Self {
            sample_rate: 44_100,
            output_gain: 0.0,
            gear_ratios: Vec::new(),
            reverse_ratio: 3.0,
            final_drive: 1.0,
            gear_teeth: 20.0,
            final_teeth: 40.0,
            whine_gain: 1.0,
            clack_gain: 1.0,
            rattle_gain: 1.0,
            clutch_gain: 1.0,
            shaft_slew_seconds: 0.12,
            seed: 0x5EED_7A11_5011_5EED,
            use_refined_dsp: false,
        }
    }
}

impl TransmissionConfig {
    /// Ford Type E 4-speed close/clubman gearbox + 3.54:1 English axle for the Escort Mk1 RS2000.
    pub fn escort_mk1_rs2000(sample_rate: u32) -> Self {
        Self {
            sample_rate,
            output_gain: 0.0,
            gear_ratios: vec![3.65, 1.97, 1.37, 1.00],
            reverse_ratio: 3.66,
            final_drive: 3.54,
            gear_teeth: 22.0,
            final_teeth: 39.0,
            whine_gain: 0.08,
            clack_gain: 0.25,
            rattle_gain: 0.35,
            clutch_gain: 0.40,
            shaft_slew_seconds: 0.04,
            seed: 0xE5C0_7E45,
            use_refined_dsp: true,
        }
    }

    /// Validates transmission ratios, tooth counts, gains, and sample rate.
    pub fn validate(&self) -> Result<(), String> {
        if self.sample_rate == 0 {
            return Err("transmission sample rate must be greater than zero".into());
        }
        if !self.output_gain.is_finite() || !(0.0..=2.0).contains(&self.output_gain) {
            return Err(format!(
                "transmission output gain out of range: {}",
                self.output_gain
            ));
        }
        for (name, gain) in [
            ("whine_gain", self.whine_gain),
            ("clack_gain", self.clack_gain),
            ("rattle_gain", self.rattle_gain),
            ("clutch_gain", self.clutch_gain),
        ] {
            if !gain.is_finite() || !(0.0..=4.0).contains(&gain) {
                return Err(format!("transmission {name} out of range: {gain}"));
            }
        }
        if !self.shaft_slew_seconds.is_finite() || self.shaft_slew_seconds <= 0.0 {
            return Err(format!("invalid shaft slew: {}", self.shaft_slew_seconds));
        }
        for ratio in &self.gear_ratios {
            if !ratio.is_finite() || *ratio <= 0.0 {
                return Err(format!("invalid gear ratio: {ratio}"));
            }
        }
        if !self.reverse_ratio.is_finite() || self.reverse_ratio <= 0.0 {
            return Err(format!("invalid reverse ratio: {}", self.reverse_ratio));
        }
        if !self.final_drive.is_finite() || self.final_drive <= 0.0 {
            return Err(format!("invalid final drive: {}", self.final_drive));
        }
        if !self.gear_teeth.is_finite() || self.gear_teeth <= 0.0 {
            return Err(format!("invalid gear teeth: {}", self.gear_teeth));
        }
        if !self.final_teeth.is_finite() || self.final_teeth <= 0.0 {
            return Err(format!("invalid final teeth: {}", self.final_teeth));
        }
        if self.output_gain > 0.0 && self.gear_ratios.is_empty() {
            return Err("gear ratios are required when the transmission is audible".into());
        }
        Ok(())
    }
}

/// Complete runtime configuration assembling engine, acoustic scene, sample bank,
/// transmission, spectrum filters, and operating mode.
#[derive(Clone, Debug)]
pub struct Gf509RuntimeConfig {
    pub engine: EngineConfig,
    pub scene: AcousticSceneConfig,
    pub sample_layer: ThreeZoneSampleLayerConfig,
    /// Prepared sample directory. `None` runs procedural-only `INT-01` mode.
    pub sample_layer_directory: Option<PathBuf>,
    pub upper_mid_shelf_gain: f32,
    pub transmission: TransmissionConfig,
    pub gear_shift_gain: f32,
    pub gear_shift_reference_rpm: f32,
    pub max_block_frames: usize,
    pub engine_mode: EngineMode,
    pub sampled_engine_gain: f32,
    pub sampled_gearbox_gain: f32,
    pub v10_filter: SpectrumChainConfig,
    pub gearbox_filter: SpectrumChainConfig,
}

impl Default for Gf509RuntimeConfig {
    fn default() -> Self {
        Self {
            engine: EngineConfig::default(),
            scene: AcousticSceneConfig::default(),
            sample_layer: ThreeZoneSampleLayerConfig::default(),
            sample_layer_directory: None,
            upper_mid_shelf_gain: 0.0,
            transmission: TransmissionConfig::default(),
            gear_shift_gain: 1.0,
            gear_shift_reference_rpm: 10_000.0,
            max_block_frames: 4096,
            engine_mode: EngineMode::Hybrid,
            sampled_engine_gain: 1.0,
            sampled_gearbox_gain: 1.0,
            v10_filter: SpectrumChainConfig::default(),
            gearbox_filter: SpectrumChainConfig::default(),
        }
    }
}

impl Gf509RuntimeConfig {
    /// Complete runtime configuration preset for the Ford Escort Mk1 RS2000 Pinto 2.0 OHC.
    pub fn ford_pinto_rs2000(sample_rate: u32) -> Self {
        Self {
            engine: EngineConfig::ford_pinto_rs2000(sample_rate),
            scene: AcousticSceneConfig::ford_pinto_rs2000(),
            sample_layer: ThreeZoneSampleLayerConfig::default(),
            sample_layer_directory: None,
            upper_mid_shelf_gain: 0.0,
            transmission: TransmissionConfig::escort_mk1_rs2000(sample_rate),
            gear_shift_gain: 0.45,
            gear_shift_reference_rpm: 5_200.0,
            max_block_frames: 4096,
            engine_mode: EngineMode::Hybrid,
            sampled_engine_gain: 1.0,
            sampled_gearbox_gain: 1.0,
            v10_filter: SpectrumChainConfig {
                lowpass_hz: 12_500.0,
                lowpass_slope_db_per_oct: 12.0,
                ..SpectrumChainConfig::default()
            },
            gearbox_filter: SpectrumChainConfig::default(),
        }
    }

    /// Validates all runtime sub-configurations before asset loading or DSP construction.
    pub fn validate(&self) -> Result<(), String> {
        if self.max_block_frames == 0 {
            return Err("max_block_frames must be greater than zero".into());
        }
        self.engine.validate()?;
        self.scene.validate()?;
        self.sample_layer.validate()?;
        if !self.upper_mid_shelf_gain.is_finite()
            || !(UpperMidShelf::MIN_GAIN..=UpperMidShelf::MAX_GAIN)
                .contains(&self.upper_mid_shelf_gain)
        {
            return Err(format!(
                "upper-mid shelf gain out of range: {}",
                self.upper_mid_shelf_gain
            ));
        }
        self.transmission.validate()?;
        if !self.gear_shift_gain.is_finite() || self.gear_shift_gain < 0.0 {
            return Err(format!("invalid gear_shift_gain: {}", self.gear_shift_gain));
        }
        if !self.gear_shift_reference_rpm.is_finite() || self.gear_shift_reference_rpm <= 0.0 {
            return Err(format!(
                "invalid gear_shift_reference_rpm: {}",
                self.gear_shift_reference_rpm
            ));
        }
        if !self.sampled_engine_gain.is_finite() || self.sampled_engine_gain < 0.0 {
            return Err(format!(
                "invalid sampled_engine_gain: {}",
                self.sampled_engine_gain
            ));
        }
        if !self.sampled_gearbox_gain.is_finite() || self.sampled_gearbox_gain < 0.0 {
            return Err(format!(
                "invalid sampled_gearbox_gain: {}",
                self.sampled_gearbox_gain
            ));
        }
        if self.engine_mode == EngineMode::SampleOnly && self.sample_layer_directory.is_none() {
            return Err("sample-only engine mode requires a sample-layer bank".into());
        }
        Ok(())
    }
}
