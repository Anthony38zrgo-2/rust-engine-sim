use crate::modal::{DcBlocker, ModalBank, OnePoleLowPass};
use crate::Biquad;

pub const CYLINDER_COUNT: usize = 10;
const AIR_TILT_HZ: f32 = 2_500.0;

#[derive(Clone, Copy, Debug)]
pub struct SceneConfig {
    pub enabled: bool,
    pub engine_air_gain: f32,
    pub engine_cover_gain: f32,
    pub mount_monocoque_gain: f32,
    pub cylinder_gain: f32,
    pub dry_low_gain: f32,
    pub dry_mid_gain: f32,
    pub dry_high_gain: f32,
    pub output_gain: f32,
    pub air_high_tilt_db: f32,
    pub air_direct_gain: f32,
    pub cover_radiation_lowpass_hz: f32,
    pub high_rpm_start: f32,
    pub high_rpm_span: f32,
    pub load: f32,
    pub throttle: f32,
    pub excitation_scale: f32,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            engine_air_gain: 1.0,
            engine_cover_gain: 0.42,
            mount_monocoque_gain: 0.06,
            cylinder_gain: 0.34,
            dry_low_gain: 0.18,
            dry_mid_gain: 0.46,
            dry_high_gain: 0.14,
            output_gain: 2.90,
            air_high_tilt_db: 0.0,
            air_direct_gain: 0.0,
            cover_radiation_lowpass_hz: 6_400.0,
            high_rpm_start: 8_500.0,
            high_rpm_span: 6_000.0,
            load: 0.78,
            throttle: 0.72,
            excitation_scale: 0.1,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SceneInput {
    pub master: f32,
    pub pressure: f32,
    pub derivative: f32,
    pub collector: [f32; 2],
    pub turbulence: f32,
    pub high_rpm: f32,
    pub load: f32,
    pub throttle: f32,
    pub cylinder_derivative: [f32; CYLINDER_COUNT],
    pub cylinder_blowdown: [f32; CYLINDER_COUNT],
    pub cylinder_header: [f32; CYLINDER_COUNT],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SceneFrame {
    pub engine_air: f32,
    pub engine_cover: f32,
    pub engine_mounts: f32,
    pub monocoque_seat: f32,
    pub mount_monocoque: f32,
    pub cylinder_mechanical_sum: f32,
    pub output: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StructureFrame {
    pub pressure_direct: f32,
    pub crankcase: f32,
    pub block: f32,
    pub head: f32,
}

pub struct BlockHead {
    pressure_dc: DcBlocker,
    pressure_lp_a: OnePoleLowPass,
    pressure_lp_b: OnePoleLowPass,
    crankcase: ModalBank,
    block: ModalBank,
    head: ModalBank,
}

impl BlockHead {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            pressure_dc: DcBlocker::new(28.0, sample_rate),
            pressure_lp_a: OnePoleLowPass::new(720.0, sample_rate),
            pressure_lp_b: OnePoleLowPass::new(720.0, sample_rate),
            crankcase: ModalBank::new(
                &[
                    (86.0, 0.012, 0.92),
                    (132.0, 0.010, 0.84),
                    (196.0, 0.008, 0.72),
                    (278.0, 0.006, 0.58),
                ],
                sample_rate,
            ),
            block: ModalBank::new(
                &[
                    (335.0, 0.0055, 0.46),
                    (470.0, 0.0048, 0.48),
                    (620.0, 0.0042, 0.56),
                    (790.0, 0.0038, 0.58),
                    (980.0, 0.0033, 0.52),
                    (1_225.0, 0.0029, 0.46),
                ],
                sample_rate,
            ),
            head: ModalBank::new(
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
                sample_rate,
            ),
        }
    }

    #[inline]
    pub fn process(&mut self, pressure: f32, derivative: f32) -> StructureFrame {
        let pressure_ac = self.pressure_dc.process(pressure);
        let pressure_low = self
            .pressure_lp_b
            .process(self.pressure_lp_a.process(pressure_ac));
        StructureFrame {
            pressure_direct: (pressure_low * 1.8).tanh() / 1.8,
            crankcase: self.crankcase.process(pressure_ac),
            block: self.block.process(derivative),
            head: self.head.process(derivative),
        }
    }
}

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
            polarity: if (index + index / 5) % 2 == 0 {
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

impl EngineCover {
    pub fn new(sample_rate: f32, radiation_lowpass_hz: f32) -> Self {
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
            radiation_lowpass: if radiation_lowpass_hz >= sample_rate * 0.48 {
                OnePoleLowPass::bypassed()
            } else {
                OnePoleLowPass::new(radiation_lowpass_hz, sample_rate)
            },
        }
    }

    #[inline]
    pub fn process(
        &mut self,
        head: f32,
        block: f32,
        turbulence: f32,
        pressure_derivative: f32,
        high_rpm: f32,
    ) -> f32 {
        let excitation = head * 0.58 + block * 0.16 + turbulence * 0.12;
        let arrived = self.propagation_delay[self.delay_cursor];
        self.propagation_delay[self.delay_cursor] = excitation;
        self.delay_cursor += 1;
        if self.delay_cursor == self.propagation_delay.len() {
            self.delay_cursor = 0;
        }

        let broad = self.broad_panels.process(arrived) * (1.0 + 0.28 * high_rpm);
        let skin = self
            .upper_skin
            .process(arrived + pressure_derivative * 0.08)
            * (1.0 - 0.68 * high_rpm);
        let radiated = self
            .dc
            .process(self.radiation_lowpass.process(broad + skin * 1.35));
        (radiated * 21.0).tanh() / 3.15
    }
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
    pub fn process(
        &mut self,
        structure: &StructureFrame,
        collector: [f32; 2],
        pressure_derivative: f32,
        cylinder_structure: f32,
        load: f32,
        throttle: f32,
    ) -> (f32, f32) {
        let torque_transfer = 0.42 + 0.58 * load * (0.30 + 0.70 * throttle);
        let block_force = (structure.pressure_direct * 0.52
            + structure.crankcase * 0.68
            + structure.block * 0.32
            + (collector[0] + collector[1]) * 0.14
            + pressure_derivative * 0.035
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
            engine_mounts * 0.84 + structure.head * 0.055,
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

pub struct StructuralScene {
    config: SceneConfig,
    block_head: BlockHead,
    engine_cover: EngineCover,
    mount_monocoque: EngineMountMonocoque,
    cylinder_paths: [CylinderMechanicalPath; CYLINDER_COUNT],
    dry_lowpass: OnePoleLowPass,
    dry_midpass: OnePoleLowPass,
    air_path: AirPath,
    air_tilt: Biquad,
    air_direct_highpass: Biquad,
    onboard_highpass: DcBlocker,
}

impl StructuralScene {
    pub fn new(sample_rate: f32, config: SceneConfig) -> Result<Self, String> {
        let in_range = |v: f32, lo: f32, hi: f32| v.is_finite() && (lo..=hi).contains(&v);
        if !in_range(config.engine_air_gain, 0.0, 2.0)
            || !in_range(config.engine_cover_gain, 0.0, 2.0)
            || !in_range(config.mount_monocoque_gain, 0.0, 2.0)
            || !in_range(config.cylinder_gain, 0.0, 2.0)
            || !in_range(config.dry_low_gain, 0.0, 1.5)
            || !in_range(config.dry_mid_gain, 0.0, 1.5)
            || !in_range(config.dry_high_gain, 0.0, 1.5)
            || !in_range(config.air_high_tilt_db, -6.0, 12.0)
            || !in_range(config.air_direct_gain, 0.0, 1.0)
            || !in_range(config.output_gain, 0.25, 5.0)
            || !in_range(config.cover_radiation_lowpass_hz, 100.0, 1_000_000.0)
        {
            return Err("acoustic scene gain outside supported range".into());
        }
        Ok(Self {
            config,
            block_head: BlockHead::new(sample_rate),
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
        })
    }

    pub fn config(&self) -> SceneConfig {
        self.config
    }

    pub fn set_config(&mut self, config: SceneConfig) {
        self.config = config;
    }

    #[inline]
    pub fn process(&mut self, input: &SceneInput) -> SceneFrame {
        let structure = self.block_head.process(input.pressure, input.derivative);
        let mut cylinder_mechanical = [0.0f32; CYLINDER_COUNT];
        for (index, path) in self.cylinder_paths.iter_mut().enumerate() {
            cylinder_mechanical[index] = path.process(
                input.cylinder_derivative[index],
                input.cylinder_blowdown[index],
                input.cylinder_header[index],
            );
        }
        let cylinder_mechanical_sum =
            cylinder_mechanical.iter().sum::<f32>() * self.config.cylinder_gain;
        let engine_cover = self.engine_cover.process(
            structure.head,
            structure.block,
            input.turbulence,
            input.derivative,
            input.high_rpm,
        );
        let (engine_mounts, monocoque_seat) = self.mount_monocoque.process(
            &structure,
            input.collector,
            input.derivative,
            cylinder_mechanical_sum,
            input.load,
            input.throttle,
        );
        let mount_monocoque = engine_mounts * 0.58 + monocoque_seat;

        let master = input.master;
        let dry_low = self.dry_lowpass.process(master);
        let below_mid = self.dry_midpass.process(master);
        let dry_mid = below_mid - dry_low;
        let dry_high = master - below_mid;
        let filtered_dry = dry_low * self.config.dry_low_gain
            + dry_mid * self.config.dry_mid_gain
            + dry_high * self.config.dry_high_gain * (1.0 - 0.72 * input.high_rpm);
        let air_input = self.air_tilt.process(filtered_dry);
        let air_direct = self.air_direct_highpass.process(air_input) * self.config.air_direct_gain;
        let engine_air = self.air_path.process(air_input) + air_direct;

        SceneFrame {
            engine_air,
            engine_cover,
            engine_mounts,
            monocoque_seat,
            mount_monocoque,
            cylinder_mechanical_sum,
            output: self.onboard_highpass.process(
                (engine_air * self.config.engine_air_gain
                    + engine_cover * self.config.engine_cover_gain
                    + mount_monocoque * self.config.mount_monocoque_gain)
                    * self.config.output_gain,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn silent_input_stays_silent() {
        let mut scene = StructuralScene::new(48_000.0, SceneConfig::default()).unwrap();
        for _ in 0..96_000 {
            let frame = scene.process(&SceneInput::default());
            assert_eq!(frame.engine_air, 0.0);
            assert_eq!(frame.engine_cover, 0.0);
            assert_eq!(frame.mount_monocoque, 0.0);
            assert_eq!(frame.output, 0.0);
        }
    }

    #[test]
    fn cylinder_polarity_alternates() {
        let a = CylinderMechanicalPath::new(0, 48_000.0).polarity;
        let b = CylinderMechanicalPath::new(1, 48_000.0).polarity;
        let c = CylinderMechanicalPath::new(2, 48_000.0).polarity;
        assert_eq!(a, 1.0);
        assert_eq!(b, -1.0);
        assert_eq!(c, 1.0);
    }

    #[test]
    fn cylinder_paths_do_not_cancel_pairwise() {
        let sr = 48_000.0;
        let mut a = CylinderMechanicalPath::new(0, sr);
        let mut b = CylinderMechanicalPath::new(1, sr);
        let mut peak_a = 0.0f32;
        let mut peak_b = 0.0f32;
        for s in 0..2_400 {
            let x = if s == 0 { 1.0 } else { 0.0 };
            peak_a = peak_a.max(a.process(x, 0.0, 0.0).abs());
            peak_b = peak_b.max(b.process(x, 0.0, 0.0).abs());
        }
        assert!(peak_a > 1e-4 && peak_b > 1e-4);
    }

    #[test]
    fn cover_and_mount_respond_to_excitation() {
        let mut scene = StructuralScene::new(48_000.0, SceneConfig::default()).unwrap();
        let mut input = SceneInput::default();
        input.master = 1.0;
        input.pressure = 1.0;
        input.derivative = 1.0;
        let mut peak_cover = 0.0f32;
        let mut peak_mount = 0.0f32;
        for s in 0..48_000 {
            input.derivative = if s == 0 { 1.0 } else { 0.0 };
            let frame = scene.process(&input);
            peak_cover = peak_cover.max(frame.engine_cover.abs());
            peak_mount = peak_mount.max(frame.mount_monocoque.abs());
        }
        assert!(peak_cover > 1e-3, "cover peak={peak_cover}");
        assert!(peak_mount > 1e-4, "mount peak={peak_mount}");
    }

    #[test]
    fn air_route_applies_comb_taps() {
        let mut scene = StructuralScene::new(48_000.0, SceneConfig::default()).unwrap();
        let mut input = SceneInput::default();
        let mut first_nonzero = None;
        for s in 0..48_000 {
            input.master = if s == 0 { 1.0 } else { 0.0 };
            let frame = scene.process(&input);
            if frame.engine_air != 0.0 && first_nonzero.is_none() {
                first_nonzero = Some(s);
            }
        }
        let expected = (2.11f32 * 0.001 * 48_000.0).round() as usize;
        assert_eq!(first_nonzero, Some(expected));
    }

    #[test]
    fn invalid_config_is_rejected() {
        let mut config = SceneConfig::default();
        config.output_gain = 100.0;
        assert!(StructuralScene::new(48_000.0, config).is_err());
    }
}
