use crate::acoustics::{DcBlocker, ModalBank, OnePoleLowPass};
use crate::engine::EngineFrame;
use crate::tone::Biquad;

use crate::crank::CYLINDER_COUNT;

const AIR_TILT_HZ: f32 = 2_500.0;

#[derive(Clone, Copy, Debug)]
pub struct AcousticSceneConfig {
    pub dry_low_gain: f32,
    pub dry_mid_gain: f32,
    pub dry_high_gain: f32,
    /// Gain on the air-propagated dry branch in the scene mix (default 1.0).
    pub engine_air_gain: f32,
    pub engine_cover_gain: f32,
    pub mount_monocoque_gain: f32,
    /// Parallel send of the transmission (whine/clack/rattle) into the
    /// mount/monocoque structural path. 0.0 keeps the transmission air-only.
    pub transmission_mount_gain: f32,
    /// Parallel send of the transmission into the engine-cover skin, the only
    /// scene branch broadband enough to radiate the gearbox band.
    pub transmission_cover_gain: f32,
    /// High-shelf lift (dB) above `air_tilt_hz` applied to the air-path input.
    /// 0.0 keeps the air branch untouched.
    pub air_high_tilt_db: f32,
    /// Fraction of the high-tilted air input mixed straight into the air
    /// output, bypassing the comb taps. 0.0 keeps the pure comb.
    pub air_direct_gain: f32,
    pub output_gain: f32,
    /// One-pole lowpass on the engine-cover radiation (Hz). A value at or above
    /// `0.48 * sample_rate` disables (bypasses) the filter.
    pub cover_radiation_lowpass_hz: f32,
}

/// Build a one-pole lowpass, or a transparent passthrough when the requested
/// cutoff reaches the Nyquist guard: an explicit, reversible way to disable a
/// filter stage for diagnostics.
fn lowpass_or_bypass(cutoff_hz: f32, sample_rate: f32) -> OnePoleLowPass {
    if cutoff_hz >= sample_rate * 0.48 {
        OnePoleLowPass::bypassed()
    } else {
        OnePoleLowPass::new(cutoff_hz, sample_rate)
    }
}

impl Default for AcousticSceneConfig {
    fn default() -> Self {
        Self {
            dry_low_gain: 0.18,
            dry_mid_gain: 0.46,
            dry_high_gain: 0.14,
            engine_air_gain: 1.0,
            engine_cover_gain: 0.42,
            mount_monocoque_gain: 0.06,
            transmission_mount_gain: 0.0,
            transmission_cover_gain: 0.0,
            air_high_tilt_db: 0.0,
            air_direct_gain: 0.0,
            output_gain: 2.90,
            cover_radiation_lowpass_hz: 6_400.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AcousticFrame {
    pub engine_dry: f32,
    pub engine_air: f32,
    pub engine_cover: f32,
    pub engine_mounts: f32,
    pub monocoque_seat: f32,
    pub mount_monocoque: f32,
    pub cylinder_mechanical: [f32; CYLINDER_COUNT],
    pub cylinder_mechanical_sum: f32,
    pub output: f32,
}

/// One short structural route per cylinder. These paths retain cylinder
/// identity until after propagation; collectors and the large vehicle
/// structures remain shared.
pub struct CylinderMechanicalPath {
    signature: ModalBank,
    propagation_delay: Vec<f32>,
    delay_cursor: usize,
    dc: DcBlocker,
    radiation_lowpass: OnePoleLowPass,
    polarity: f32,
}

impl CylinderMechanicalPath {
    fn new(index: usize, sample_rate: f32) -> Self {
        let bank = if index < 5 { 0.0 } else { 1.0 };
        let position = (index % 5) as f32;
        let spread = position - 2.0;
        let base = 236.0 + spread * 11.5 + bank * 8.0;
        let delay_ms = 0.16 + position * 0.085 + bank * 0.055;
        Self {
            signature: ModalBank::new(
                &[
                    (base, 0.0095 + position * 0.00035, 0.115),
                    (base * (1.47 + bank * 0.018), 0.0072, -0.090),
                    (base * (2.08 - position * 0.012), 0.0050, 0.062),
                ],
                sample_rate,
            ),
            propagation_delay: vec![
                0.0;
                (delay_ms * 0.001 * sample_rate).round().max(1.0) as usize
            ],
            delay_cursor: 0,
            dc: DcBlocker::new(72.0 + position * 4.0, sample_rate),
            radiation_lowpass: OnePoleLowPass::new(1_350.0 + position * 85.0, sample_rate),
            polarity: if (index + index / 5).is_multiple_of(2) {
                1.0
            } else {
                -1.0
            },
        }
    }

    #[inline]
    fn process(&mut self, derivative: f32, blowdown: f32, header: f32) -> f32 {
        let excitation = derivative * 0.72 + blowdown * 0.22 + header * 0.16;
        let arrived = self.propagation_delay[self.delay_cursor];
        self.propagation_delay[self.delay_cursor] = excitation;
        self.delay_cursor += 1;
        if self.delay_cursor == self.propagation_delay.len() {
            self.delay_cursor = 0;
        }
        let resonant = self.signature.process(arrived * self.polarity);
        let radiated = self
            .radiation_lowpass
            .process(self.dc.process(arrived * 0.018 + resonant));
        (radiated * 10.0).tanh() / 4.0
    }
}

pub struct EngineCover {
    broad_panels: ModalBank,
    upper_skin: ModalBank,
    propagation_delay: Vec<f32>,
    delay_cursor: usize,
    dc: DcBlocker,
    radiation_lowpass: OnePoleLowPass,
}

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
    pub fn new(sample_rate: f32) -> Self {
        let delay = |milliseconds: f32| {
            vec![0.0; (milliseconds * 0.001 * sample_rate).round().max(1.0) as usize + 1]
        };
        Self {
            mount_modes: ModalBank::new(
                &[
                    (148.0, 0.022, 0.10),
                    (193.0, 0.020, -0.12),
                    (247.0, 0.018, 0.31),
                    (318.0, 0.016, -0.34),
                    (402.0, 0.013, 0.30),
                    (515.0, 0.010, -0.25),
                ],
                sample_rate,
            ),
            monocoque_modes: ModalBank::new(
                &[
                    (171.0, 0.021, -0.09),
                    (226.0, 0.019, 0.13),
                    (291.0, 0.017, -0.30),
                    (367.0, 0.014, 0.32),
                    (454.0, 0.012, -0.28),
                    (548.0, 0.009, 0.22),
                ],
                sample_rate,
            ),
            mount_delay: delay(0.18),
            monocoque_delay: delay(0.82),
            mount_cursor: 0,
            monocoque_cursor: 0,
            mount_dc: DcBlocker::new(48.0, sample_rate),
            monocoque_dc: DcBlocker::new(42.0, sample_rate),
            mount_lowpass: OnePoleLowPass::new(920.0, sample_rate),
            monocoque_lowpass: OnePoleLowPass::new(760.0, sample_rate),
        }
    }

    #[inline]
    fn delayed(delay: &mut [f32], cursor: &mut usize, input: f32) -> f32 {
        let arrived = delay[*cursor];
        delay[*cursor] = input;
        *cursor += 1;
        if *cursor == delay.len() {
            *cursor = 0;
        }
        arrived
    }

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
            Self::delayed(&mut self.mount_delay, &mut self.mount_cursor, block_force);
        let mount_resonance = self.mount_modes.process(mount_arrival);
        let engine_mounts = self.mount_lowpass.process(
            self.mount_dc
                .process(mount_arrival * 0.10 + mount_resonance),
        );

        let chassis_force = Self::delayed(
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

impl EngineCover {
    pub fn new(sample_rate: f32, radiation_lowpass_hz: f32) -> Self {
        // Panel radiation faces the cockpit microphone ~0.45 m away.
        let delay_samples = (0.001309 * sample_rate).round().max(1.0) as usize;
        Self {
            broad_panels: ModalBank::new(
                &[
                    (684.0, 0.0130, 0.25),
                    (817.0, 0.0110, -0.23),
                    (1_036.0, 0.0092, 0.22),
                    (1_291.0, 0.0077, -0.20),
                    (1_603.0, 0.0063, 0.18),
                    (1_982.0, 0.0051, -0.16),
                ],
                sample_rate,
            ),
            upper_skin: ModalBank::new(
                &[
                    (2_438.0, 0.0042, 0.14),
                    (2_997.0, 0.0034, -0.12),
                    (3_611.0, 0.0027, 0.095),
                    (4_283.0, 0.0021, -0.070),
                    (5_071.0, 0.0016, 0.045),
                ],
                sample_rate,
            ),
            propagation_delay: vec![0.0; delay_samples + 1],
            delay_cursor: 0,
            dc: DcBlocker::new(420.0, sample_rate),
            radiation_lowpass: lowpass_or_bypass(radiation_lowpass_hz, sample_rate),
        }
    }

    #[inline]
    pub fn process(&mut self, frame: &EngineFrame, high_rpm: f32, transmission_send: f32) -> f32 {
        // The cover is a lightly coupled skin. It receives structure from the
        // heads plus airborne pressure from the engine bay, plus the optional
        // gearbox send (the only branch broadband enough to radiate it).
        let excitation =
            frame.head * 0.58 + frame.block * 0.16 + frame.turbulence * 0.12 + transmission_send;
        let arrived = self.propagation_delay[self.delay_cursor];
        self.propagation_delay[self.delay_cursor] = excitation;
        self.delay_cursor += 1;
        if self.delay_cursor == self.propagation_delay.len() {
            self.delay_cursor = 0;
        }

        let broad = self.broad_panels.process(arrived) * (1.0 + 0.28 * high_rpm);
        let skin = self
            .upper_skin
            .process(arrived + frame.pressure_derivative * 0.08)
            * (1.0 - 0.68 * high_rpm);
        let radiated = self
            .dc
            .process(self.radiation_lowpass.process(broad + skin * 1.35));
        (radiated * 21.0).tanh() / 3.15
    }
}

struct AirPath {
    delay: Vec<f32>,
    cursor: usize,
    taps: [(usize, f32); 3],
}

impl AirPath {
    fn new(sample_rate: f32) -> Self {
        let tap = |milliseconds: f32| (milliseconds * 0.001 * sample_rate).round() as usize;
        let taps = [(tap(2.11), 0.56), (tap(3.91), 0.28), (tap(6.31), 0.13)];
        let maximum = taps.iter().map(|&(delay, _)| delay).max().unwrap_or(0);
        Self {
            delay: vec![0.0; maximum + 1],
            cursor: 0,
            taps,
        }
    }

    #[inline]
    fn process(&mut self, input: f32) -> f32 {
        self.delay[self.cursor] = input;
        let mut output = 0.0;
        for &(delay, gain) in &self.taps {
            let index = (self.cursor + self.delay.len() - delay) % self.delay.len();
            output += self.delay[index] * gain;
        }
        self.cursor += 1;
        if self.cursor == self.delay.len() {
            self.cursor = 0;
        }
        output
    }
}

pub struct AcousticScene {
    config: AcousticSceneConfig,
    engine_cover: EngineCover,
    mount_monocoque: EngineMountMonocoque,
    cylinder_paths: [CylinderMechanicalPath; CYLINDER_COUNT],
    dry_lowpass: OnePoleLowPass,
    dry_midpass: OnePoleLowPass,
    air_path: AirPath,
    air_tilt: Biquad,
    air_direct_highpass: Biquad,
    onboard_highpass: DcBlocker,
    sample_rate: f32,
    previous_crank_phase: f32,
    firing_frequency_hz: f32,
}

impl AcousticScene {
    pub fn new(sample_rate: f32, config: AcousticSceneConfig) -> Result<Self, String> {
        if !(0.0..=1.5).contains(&config.dry_low_gain)
            || !(0.0..=1.5).contains(&config.dry_mid_gain)
            || !(0.0..=1.5).contains(&config.dry_high_gain)
            || !(0.0..=1.5).contains(&config.engine_air_gain)
            || !(0.0..=1.5).contains(&config.engine_cover_gain)
            || !(0.0..=1.5).contains(&config.mount_monocoque_gain)
            || !(0.0..=1.5).contains(&config.transmission_mount_gain)
            || !(0.0..=2.0).contains(&config.transmission_cover_gain)
            || !(-6.0..=12.0).contains(&config.air_high_tilt_db)
            || !(0.0..=1.0).contains(&config.air_direct_gain)
            || !(0.25..=5.0).contains(&config.output_gain)
            || !(100.0..=1_000_000.0).contains(&config.cover_radiation_lowpass_hz)
        {
            return Err("acoustic scene gain outside supported range".into());
        }
        Ok(Self {
            config,
            engine_cover: EngineCover::new(sample_rate, config.cover_radiation_lowpass_hz),
            mount_monocoque: EngineMountMonocoque::new(sample_rate),
            cylinder_paths: std::array::from_fn(|index| {
                CylinderMechanicalPath::new(index, sample_rate)
            }),
            dry_lowpass: OnePoleLowPass::new(360.0, sample_rate),
            dry_midpass: OnePoleLowPass::new(2_650.0, sample_rate),
            air_path: AirPath::new(sample_rate),
            air_tilt: Biquad::high_shelf(
                AIR_TILT_HZ,
                std::f32::consts::FRAC_1_SQRT_2,
                config.air_high_tilt_db,
                sample_rate,
            ),
            air_direct_highpass: Biquad::highpass(
                AIR_TILT_HZ,
                std::f32::consts::FRAC_1_SQRT_2,
                sample_rate,
            ),
            onboard_highpass: DcBlocker::new(75.0, sample_rate),
            sample_rate,
            previous_crank_phase: 0.0,
            firing_frequency_hz: 0.0,
        })
    }

    #[inline]
    pub fn process(&mut self, engine: &EngineFrame) -> AcousticFrame {
        let crank_delta = (engine.crank_phase_deg - self.previous_crank_phase).rem_euclid(720.0);
        self.previous_crank_phase = engine.crank_phase_deg;
        let measured_firing = crank_delta * self.sample_rate / 360.0 * 5.0;
        if measured_firing.is_finite() && measured_firing < self.sample_rate * 0.45 {
            self.firing_frequency_hz += 0.04 * (measured_firing - self.firing_frequency_hz);
        }
        // HR-2: the low-body capture of the gearbox/empty casing and the
        // overbright second-harmonic shell are speed-sensitive, so the whole
        // redistribution rides one continuous high-RPM ramp (8500 -> 14500).
        let rpm = self.firing_frequency_hz * 60.0 / 5.0;
        let high_rpm = ((rpm - 8_500.0) / 6_000.0).clamp(0.0, 1.0);
        let mut cylinder_mechanical = [0.0; CYLINDER_COUNT];
        for (index, path) in self.cylinder_paths.iter_mut().enumerate() {
            cylinder_mechanical[index] = path.process(
                engine.cylinder_pressure_derivative[index],
                engine.cylinder_blowdown[index],
                engine.cylinder_headers[index],
            );
        }
        let cylinder_mechanical_sum = cylinder_mechanical.iter().sum::<f32>() * 0.34;
        let structure_send =
            cylinder_mechanical_sum + engine.transmission * self.config.transmission_mount_gain;
        let transmission_cover_send =
            engine.transmission * self.config.transmission_cover_gain;
        let engine_cover = self
            .engine_cover
            .process(engine, high_rpm, transmission_cover_send);
        let (engine_mounts, monocoque_seat) = self
            .mount_monocoque
            .process(engine, structure_send);
        let mount_monocoque = engine_mounts * 0.58 + monocoque_seat;

        let master = engine.master + engine.transmission;
        let dry_low = self.dry_lowpass.process(master);
        let below_mid = self.dry_midpass.process(master);
        let dry_mid = below_mid - dry_low;
        let dry_high = master - below_mid;
        // The high-frequency air is rolled off as the intake charge pulse
        // shortens at high engine speed.
        let filtered_dry = dry_low * self.config.dry_low_gain
            + dry_mid * self.config.dry_mid_gain
            + dry_high * self.config.dry_high_gain * (1.0 - 0.72 * high_rpm);
        let air_input = self.air_tilt.process(filtered_dry);
        let air_direct = self.air_direct_highpass.process(air_input) * self.config.air_direct_gain;
        let engine_air = self.air_path.process(air_input) + air_direct;
        AcousticFrame {
            engine_dry: master,
            engine_air,
            engine_cover,
            engine_mounts,
            monocoque_seat,
            mount_monocoque,
            cylinder_mechanical,
            cylinder_mechanical_sum,
            output: self.onboard_highpass.process(
                (engine_air * self.config.engine_air_gain
                    + engine_cover * self.config.engine_cover_gain
                    + mount_monocoque * self.config.mount_monocoque_gain
                    + engine.gear_shift)
                    * self.config.output_gain,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn silent_engine_stays_silent() {
        let mut scene = AcousticScene::new(48_000.0, AcousticSceneConfig::default()).unwrap();
        for _ in 0..96_000 {
            let frame = scene.process(&EngineFrame::default());
            assert_eq!(frame.engine_air, 0.0);
            assert_eq!(frame.engine_cover, 0.0);
            assert_eq!(frame.mount_monocoque, 0.0);
            assert_eq!(frame.output, 0.0);
        }
    }

    #[test]
    fn filtered_air_path_is_audible() {
        let mut scene = AcousticScene::new(
            48_000.0,
            AcousticSceneConfig {
                dry_low_gain: 1.0,
                dry_mid_gain: 1.0,
                dry_high_gain: 1.0,
                engine_air_gain: 1.0,
                engine_cover_gain: 0.0,
                mount_monocoque_gain: 0.0,
                output_gain: 1.0,
                ..AcousticSceneConfig::default()
            },
        )
        .unwrap();
        let source = EngineFrame {
            master: 0.375,
            ..EngineFrame::default()
        };
        // Air propagation deliberately adds latency; after its longest tap the
        // all-passband dry signal is reconstructed without a direct shortcut.
        let mut heard = false;
        for _ in 0..400 {
            let output = scene.process(&source).output;
            heard |= output != 0.0;
        }
        assert!(heard);
    }

    #[test]
    fn transmission_mount_send_reaches_the_monocoque_path() {
        let config = |gain: f32| AcousticSceneConfig {
            engine_air_gain: 0.0,
            engine_cover_gain: 0.0,
            mount_monocoque_gain: 1.0,
            transmission_mount_gain: gain,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let source = EngineFrame {
            transmission: 0.8,
            ..EngineFrame::default()
        };
        let mut reference = AcousticScene::new(48_000.0, config(0.0)).unwrap();
        let mut candidate = AcousticScene::new(48_000.0, config(0.5)).unwrap();
        let mut reference_peak = 0.0f32;
        let mut candidate_peak = 0.0f32;
        for _ in 0..96_000 {
            reference_peak = reference_peak.max(reference.process(&source).output.abs());
            candidate_peak = candidate_peak.max(candidate.process(&source).output.abs());
        }
        assert_eq!(reference_peak, 0.0, "air send off must isolate the mount path");
        assert!(
            candidate_peak > 1.0e-4,
            "transmission must reach the mount path, peak {candidate_peak}"
        );
    }

    #[test]
    fn transmission_cover_send_reaches_the_cover_branch() {
        let config = |gain: f32| AcousticSceneConfig {
            engine_air_gain: 0.0,
            mount_monocoque_gain: 0.0,
            engine_cover_gain: 1.0,
            transmission_cover_gain: gain,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let source = EngineFrame {
            transmission: 0.8,
            ..EngineFrame::default()
        };
        let mut reference = AcousticScene::new(48_000.0, config(0.0)).unwrap();
        let mut candidate = AcousticScene::new(48_000.0, config(0.5)).unwrap();
        let mut reference_peak = 0.0f32;
        let mut candidate_peak = 0.0f32;
        for _ in 0..4_800 {
            reference_peak = reference_peak.max(reference.process(&source).output.abs());
            candidate_peak = candidate_peak.max(candidate.process(&source).output.abs());
        }
        assert_eq!(reference_peak, 0.0, "cover send off must isolate the branch");
        assert!(
            candidate_peak > 1.0e-4,
            "transmission must reach the cover branch, peak {candidate_peak}"
        );
    }

    #[test]
    fn air_tilt_and_direct_add_high_frequency_detail() {
        let config = |tilt: f32, direct: f32| AcousticSceneConfig {
            engine_cover_gain: 0.0,
            mount_monocoque_gain: 0.0,
            transmission_cover_gain: 0.0,
            engine_air_gain: 1.0,
            air_high_tilt_db: tilt,
            air_direct_gain: direct,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let render = |tilt: f32, direct: f32| {
            let mut scene = AcousticScene::new(48_000.0, config(tilt, direct)).unwrap();
            let mut state = 0x1234_5678u32;
            let mut sum = 0.0f64;
            for _ in 0..96_000 {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let noise = (state as f32 / u32::MAX as f32) * 2.0 - 1.0;
                let mut frame = EngineFrame::default();
                frame.master = 0.3 * noise;
                let output = scene.process(&frame).engine_air;
                sum += (output as f64) * (output as f64);
            }
            (sum / 96_000.0).sqrt()
        };
        let reference = render(0.0, 0.0);
        let tilted = render(6.0, 0.0);
        let direct = render(0.0, 0.5);
        assert!(reference > 0.0);
        assert!(tilted > reference * 1.02, "tilt must add air detail");
        assert!(direct > reference * 1.02, "direct must add air detail");
    }

    #[test]
    fn engine_air_gain_scales_only_the_air_branch() {
        let base = AcousticSceneConfig {
            engine_cover_gain: 0.0,
            mount_monocoque_gain: 0.0,
            output_gain: 1.0,
            ..AcousticSceneConfig::default()
        };
        let mut reference = AcousticScene::new(48_000.0, base).unwrap();
        let mut louder = AcousticScene::new(
            48_000.0,
            AcousticSceneConfig {
                engine_air_gain: 1.5,
                ..base
            },
        )
        .unwrap();
        let source = EngineFrame {
            master: 0.25,
            ..EngineFrame::default()
        };
        let mut reference_air = 0.0;
        let mut reference_out = 0.0;
        let mut louder_out = 0.0;
        for _ in 0..4_800 {
            let reference_frame = reference.process(&source);
            let louder_frame = louder.process(&source);
            reference_air = reference_frame.engine_air;
            reference_out = reference_frame.output;
            louder_out = louder_frame.output;
        }
        assert!(reference_air.abs() > 1.0e-6);
        assert!((louder_out / reference_out - 1.5).abs() < 1.0e-3);
    }
}
