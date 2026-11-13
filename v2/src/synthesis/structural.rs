//! Block, crankcase, and cylinder-head structural radiation synthesizer.

use crate::config::EngineStructuralProfile;
use crate::dsp::{DcBlocker, ModalBank, OnePoleLowPass, TanhAdaa1};

/// Structural radiation outputs per audio sample.
#[derive(Clone, Copy, Debug, Default)]
pub struct StructureFrame {
    pub pressure_direct: f32,
    pub crankcase: f32,
    pub block: f32,
    pub head: f32,
}

/// Damped modal synthesizer modelling crankcase, block, and head acoustic radiation.
#[derive(Clone, Debug)]
pub struct BlockHead {
    pressure_dc: DcBlocker,
    pressure_lp_a: OnePoleLowPass,
    pressure_lp_b: OnePoleLowPass,
    pressure_adaa: TanhAdaa1,
    crankcase: ModalBank,
    block: ModalBank,
    head: ModalBank,
}

impl BlockHead {
    /// Allocates modal resonators and smoothing filters for `sample_rate`.
    pub fn new(sample_rate: f32) -> Self {
        Self::with_profile(sample_rate, EngineStructuralProfile::F1V10)
    }

    /// Allocates block, crankcase, and head resonators tailored to `profile`.
    pub fn with_profile(sample_rate: f32, profile: EngineStructuralProfile) -> Self {
        type ModeList = &'static [(f32, f32, f32)];
        let (crank_modes, block_modes, head_modes, lp_hz): (ModeList, ModeList, ModeList, f32) =
            match profile {
                EngineStructuralProfile::F1V10 => (
                    &[
                        (86.0, 0.012, 0.92),
                        (132.0, 0.010, 0.84),
                        (196.0, 0.008, 0.72),
                        (278.0, 0.006, 0.58),
                    ],
                    &[
                        (335.0, 0.0055, 0.46),
                        (470.0, 0.0048, 0.48),
                        (620.0, 0.0042, 0.56),
                        (790.0, 0.0038, 0.58),
                        (980.0, 0.0033, 0.52),
                        (1_225.0, 0.0029, 0.46),
                    ],
                    &[
                        (760.0, 0.0035, 0.48),
                        (940.0, 0.0031, 0.52),
                        (1_160.0, 0.0028, 0.50),
                        (1_410.0, 0.0025, 0.46),
                        (1_720.0, 0.0022, 0.41),
                        (2_080.0, 0.0019, 0.34),
                        (2_520.0, 0.0016, 0.23),
                        (3_150.0, 0.0013, 0.12),
                    ],
                    720.0,
                ),
                EngineStructuralProfile::CastIronSohcL4 => (
                    &[
                        (68.0, 0.010, 0.38),
                        (108.0, 0.0085, 0.34),
                        (158.0, 0.0072, 0.28),
                        (226.0, 0.0058, 0.22),
                    ],
                    &[
                        (255.0, 0.0085, 0.52),
                        (365.0, 0.0072, 0.55),
                        (495.0, 0.0060, 0.54),
                        (650.0, 0.0049, 0.48),
                        (830.0, 0.0039, 0.41),
                        (1_040.0, 0.0031, 0.32),
                    ],
                    &[
                        (520.0, 0.0048, 0.54),
                        (710.0, 0.0042, 0.58),
                        (920.0, 0.0036, 0.56),
                        (1_180.0, 0.0030, 0.48),
                        (1_480.0, 0.0025, 0.40),
                        (1_840.0, 0.0020, 0.30),
                        (2_240.0, 0.0017, 0.20),
                        (2_720.0, 0.0013, 0.12),
                    ],
                    520.0,
                ),
            };
        Self {
            pressure_dc: DcBlocker::new(28.0, sample_rate),
            pressure_lp_a: OnePoleLowPass::new(lp_hz, sample_rate),
            pressure_lp_b: OnePoleLowPass::new(lp_hz, sample_rate),
            pressure_adaa: TanhAdaa1::new(1.8, 1.8),
            crankcase: ModalBank::new(crank_modes, sample_rate),
            block: ModalBank::new(block_modes, sample_rate),
            head: ModalBank::new(head_modes, sample_rate),
        }
    }

    /// Evaluates direct combustion body and structural modal banks without heap allocations.
    #[inline]
    pub fn process(&mut self, pressure: f32, derivative: f32) -> StructureFrame {
        let pressure_ac = self.pressure_dc.process(pressure);
        let pressure_low = self
            .pressure_lp_b
            .process(self.pressure_lp_a.process(pressure_ac));
        let pressure_direct = (pressure_low * 1.8).tanh() / 1.8;
        StructureFrame {
            pressure_direct,
            crankcase: self.crankcase.process(pressure_ac),
            block: self.block.process(derivative),
            head: self.head.process(derivative),
        }
    }

    /// Evaluates structural banks with ADAA1 direct saturation and load/stochastic modal drift.
    #[inline]
    pub fn process_modulated(
        &mut self,
        pressure: f32,
        derivative: f32,
        load: f32,
    ) -> StructureFrame {
        let pressure_ac = self.pressure_dc.process(pressure);
        let pressure_low = self
            .pressure_lp_b
            .process(self.pressure_lp_a.process(pressure_ac));
        let pressure_direct = self.pressure_adaa.process(pressure_low);
        StructureFrame {
            pressure_direct,
            crankcase: self.crankcase.process_modulated(pressure_ac, load, 1.0),
            block: self.block.process_modulated(derivative, load, 1.0),
            head: self.head.process_modulated(derivative, load, 1.0),
        }
    }
}
