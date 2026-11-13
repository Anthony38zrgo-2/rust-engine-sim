//! Carbon/panel engine-cover acoustic skin radiation model.

use crate::acoustics::paths::advance_delay;
use crate::config::EngineStructuralProfile;
use crate::dsp::{DcBlocker, ModalBank, OnePoleLowPass, TanhAdaa1};
use crate::synthesis::EngineFrame;

/// Radiating engine-cover panel and upper skin modal resonator network.
#[derive(Clone, Debug)]
pub struct EngineCover {
    broad_panels: ModalBank,
    upper_skin: ModalBank,
    propagation_delay: Vec<f32>,
    delay_cursor: usize,
    dc: DcBlocker,
    radiation_lowpass: OnePoleLowPass,
    adaa: TanhAdaa1,
    refined_dsp: bool,
}

impl EngineCover {
    /// Preallocates panel modal banks and radiation filters at `sample_rate`.
    pub fn new(sample_rate: f32, radiation_lowpass_hz: f32) -> Self {
        Self::with_profile(
            sample_rate,
            radiation_lowpass_hz,
            EngineStructuralProfile::F1V10,
        )
    }

    /// Preallocates engine-bay/cover panel modal banks tailored to `profile`.
    pub fn with_profile(
        sample_rate: f32,
        radiation_lowpass_hz: f32,
        profile: EngineStructuralProfile,
    ) -> Self {
        let delay_samples = (0.001309 * sample_rate).round().max(1.0) as usize;
        type ModeList = &'static [(f32, f32, f32)];
        let (broad_spec, upper_spec, dc_hz): (ModeList, ModeList, f32) = match profile {
            EngineStructuralProfile::F1V10 => (
                &[
                    (684.0, 0.0130, 0.25),
                    (817.0, 0.0110, -0.23),
                    (1_036.0, 0.0092, 0.22),
                    (1_291.0, 0.0077, -0.20),
                    (1_603.0, 0.0063, 0.18),
                    (1_982.0, 0.0051, -0.16),
                ],
                &[
                    (2_438.0, 0.0042, 0.14),
                    (2_997.0, 0.0034, -0.12),
                    (3_611.0, 0.0027, 0.095),
                    (4_283.0, 0.0021, -0.070),
                    (5_071.0, 0.0016, 0.045),
                ],
                420.0,
            ),
            EngineStructuralProfile::CastIronSohcL4 => (
                &[
                    (340.0, 0.0160, 0.26),
                    (455.0, 0.0135, -0.24),
                    (610.0, 0.0110, 0.22),
                    (795.0, 0.0088, -0.19),
                    (1_040.0, 0.0070, 0.16),
                    (1_320.0, 0.0054, -0.14),
                ],
                &[
                    (1_640.0, 0.0042, 0.12),
                    (2_050.0, 0.0033, -0.10),
                    (2_540.0, 0.0025, 0.080),
                    (3_120.0, 0.0019, -0.055),
                    (3_810.0, 0.0014, 0.035),
                ],
                190.0,
            ),
        };
        Self {
            broad_panels: ModalBank::new(broad_spec, sample_rate),
            upper_skin: ModalBank::new(upper_spec, sample_rate),
            propagation_delay: vec![0.0; delay_samples + 1],
            delay_cursor: 0,
            dc: DcBlocker::new(dc_hz, sample_rate),
            radiation_lowpass: OnePoleLowPass::lowpass_or_bypass(radiation_lowpass_hz, sample_rate),
            adaa: TanhAdaa1::new(5.5, 2.1),
            refined_dsp: false,
        }
    }

    /// Enables ADAA1 cover saturation (`drive = 5.5`) and high-mode thermal/load frequency drift.
    pub fn with_refined_dsp(mut self, enabled: bool) -> Self {
        self.refined_dsp = enabled;
        self
    }

    /// Evaluates engine-cover radiation from head/block/turbulence and optional gearbox send.
    #[inline]
    pub fn process(&mut self, frame: &EngineFrame, high_rpm: f32, transmission_send: f32) -> f32 {
        let excitation =
            frame.head * 0.58 + frame.block * 0.16 + frame.turbulence * 0.12 + transmission_send;
        let arrived = advance_delay(
            &mut self.propagation_delay,
            &mut self.delay_cursor,
            excitation,
        );

        let (broad, skin) = if self.refined_dsp {
            (
                self.broad_panels
                    .process_modulated(arrived, frame.load, 1.0)
                    * (1.0 + 0.28 * high_rpm),
                self.upper_skin.process_modulated(
                    arrived + frame.pressure_derivative * 0.08,
                    frame.load,
                    1.0,
                ) * (1.0 - 0.68 * high_rpm),
            )
        } else {
            (
                self.broad_panels.process(arrived) * (1.0 + 0.28 * high_rpm),
                self.upper_skin
                    .process(arrived + frame.pressure_derivative * 0.08)
                    * (1.0 - 0.68 * high_rpm),
            )
        };
        let radiated = self
            .dc
            .process(self.radiation_lowpass.process(broad + skin * 1.35));
        if self.refined_dsp {
            self.adaa.process(radiated)
        } else {
            (radiated * 21.0).tanh() / 3.15
        }
    }
}
