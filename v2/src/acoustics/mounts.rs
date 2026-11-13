//! Structure-borne engine mount and monocoque cockpit seat transmission path.

use crate::acoustics::paths::advance_delay;
use crate::config::EngineStructuralProfile;
use crate::dsp::{DcBlocker, ModalBank, OnePoleLowPass};
use crate::synthesis::EngineFrame;

/// Coupled two-stage mount and chassis monocoque structural resonator.
#[derive(Clone, Debug)]
pub struct EngineMountMonocoque {
    mount_modes: ModalBank,
    monocoque_modes: ModalBank,
    mount_delay: Vec<f32>,
    monocoque_delay: Vec<f32>,
    mount_cursor: usize,
    monocoque_cursor: usize,
    mount_dc: DcBlocker,
    monocoque_dc: DcBlocker,
    mount_lowpass: OnePoleLowPass,
    monocoque_lowpass: OnePoleLowPass,
}

impl EngineMountMonocoque {
    /// Preallocates mount and chassis monocoque modal banks and delay buffers.
    pub fn new(sample_rate: f32) -> Self {
        Self::with_profile(sample_rate, EngineStructuralProfile::F1V10)
    }

    /// Preallocates mount and body/chassis modal banks tailored to `profile`.
    pub fn with_profile(sample_rate: f32, profile: EngineStructuralProfile) -> Self {
        let delay = |milliseconds: f32| {
            vec![0.0; (milliseconds * 0.001 * sample_rate).round().max(1.0) as usize + 1]
        };
        type ModeList = &'static [(f32, f32, f32)];
        let (mount_modes_spec, monocoque_modes_spec, mount_lp_hz, mono_lp_hz): (
            ModeList,
            ModeList,
            f32,
            f32,
        ) = match profile {
            EngineStructuralProfile::F1V10 => (
                &[
                    (148.0, 0.022, 0.10),
                    (193.0, 0.020, -0.12),
                    (247.0, 0.018, 0.31),
                    (318.0, 0.016, -0.34),
                    (402.0, 0.013, 0.30),
                    (515.0, 0.010, -0.25),
                ],
                &[
                    (171.0, 0.021, -0.09),
                    (226.0, 0.019, 0.13),
                    (291.0, 0.017, -0.30),
                    (367.0, 0.014, 0.32),
                    (454.0, 0.012, -0.28),
                    (548.0, 0.009, 0.22),
                ],
                920.0,
                760.0,
            ),
            EngineStructuralProfile::CastIronSohcL4 => (
                &[
                    (78.0, 0.028, 0.18),
                    (116.0, 0.024, -0.22),
                    (164.0, 0.020, 0.32),
                    (228.0, 0.016, -0.28),
                    (305.0, 0.013, 0.22),
                    (410.0, 0.010, -0.16),
                ],
                &[
                    (64.0, 0.026, -0.14),
                    (96.0, 0.022, 0.22),
                    (142.0, 0.019, -0.31),
                    (204.0, 0.015, 0.29),
                    (276.0, 0.012, -0.23),
                    (365.0, 0.009, 0.16),
                ],
                680.0,
                540.0,
            ),
        };
        Self {
            mount_modes: ModalBank::new(mount_modes_spec, sample_rate),
            monocoque_modes: ModalBank::new(monocoque_modes_spec, sample_rate),
            mount_delay: delay(0.18),
            monocoque_delay: delay(0.82),
            mount_cursor: 0,
            monocoque_cursor: 0,
            mount_dc: DcBlocker::new(48.0, sample_rate),
            monocoque_dc: DcBlocker::new(42.0, sample_rate),
            mount_lowpass: OnePoleLowPass::new(mount_lp_hz, sample_rate),
            monocoque_lowpass: OnePoleLowPass::new(mono_lp_hz, sample_rate),
        }
    }

    /// Evaluates `(engine_mounts, monocoque_seat)` structural response for one sample.
    #[inline]
    pub fn process(&mut self, frame: &EngineFrame, cylinder_structure: f32) -> (f32, f32) {
        let torque_transfer = 0.42 + 0.58 * frame.load * (0.30 + 0.70 * frame.throttle);
        let block_force = (frame.pressure_direct * 0.52
            + frame.block_head * 0.68
            + frame.block * 0.32
            + (frame.collector_pressure_a + frame.collector_pressure_b) * 0.14
            + frame.pressure_derivative * 0.035
            + cylinder_structure * 0.24)
            * torque_transfer;
        let mount_arrival =
            advance_delay(&mut self.mount_delay, &mut self.mount_cursor, block_force);
        let mount_resonance = self.mount_modes.process(mount_arrival);
        let engine_mounts = self.mount_lowpass.process(
            self.mount_dc
                .process(mount_arrival * 0.10 + mount_resonance),
        );

        let chassis_force = advance_delay(
            &mut self.monocoque_delay,
            &mut self.monocoque_cursor,
            engine_mounts * 0.84 + frame.block_head * 0.055,
        );
        let shell = self.monocoque_modes.process(chassis_force);
        let monocoque_seat = self
            .monocoque_lowpass
            .process(self.monocoque_dc.process(chassis_force * 0.08 + shell));

        (
            (engine_mounts * 13.0).tanh() / 3.0,
            (monocoque_seat * 15.0).tanh() / 3.2,
        )
    }
}
