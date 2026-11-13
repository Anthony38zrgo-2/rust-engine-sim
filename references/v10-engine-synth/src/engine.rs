use crate::acoustics::{BandPassNoise, BlockHead, Collector, DcBlocker};
use crate::config::EngineConfig;
use crate::crank::{Crankshaft, CYLINDER_COUNT};
use crate::cylinder::Cylinder;
use crate::exhaust::runner::RunnerWaveguide;

const COAST_IDLE_RPM: f32 = 1_000.0;
const COAST_MAX_RPM: f32 = 15_000.0;
const COAST_BASE_ENERGY: f32 = 0.12;
const COAST_RPM_ENERGY: f32 = 0.20;
const ENERGY_ATTACK_SECONDS: f32 = 0.018;
const ENERGY_RELEASE_SECONDS: f32 = 0.413;
/// Shift gesture: damped amplitude wobble synthesized on recovery. Applied
/// after the energy follower (not inside its target) so the asymmetric attack/
/// release smoothing cannot rectify the oscillation. Bounded and dissipative.
/// 10 Hz over 175 ms (1.75 cycles ending exactly at unity), sag to 0.70 with
/// the positive rebound hard-capped at 1.15.
const SHIFT_WOBBLE_DEPTH: f32 = 0.30;
const SHIFT_WOBBLE_SECONDS: f32 = 0.175;
const SHIFT_WOBBLE_TAU: f32 = 0.070;
const SHIFT_WOBBLE_CYCLES: f32 = 1.75;
const SHIFT_WOBBLE_MIN: f32 = 0.60;
const SHIFT_WOBBLE_MAX: f32 = 1.15;
/// Downshift blip: short energy flare over the generated `DownshiftBlip`
/// phase. Linear decay, hard-bounded.
const SHIFT_BLIP_SECONDS: f32 = 0.120;
const SHIFT_BLIP_GAIN: f32 = 0.40;
const SHIFT_GAIN_MAX: f32 = 1.45;

#[derive(Clone, Copy, Debug)]
pub struct EngineInput {
    pub rpm: f32,
    pub throttle: f32,
    pub load: f32,
}

#[derive(Clone, Copy, Debug, Default)]
struct MechanicalState {
    /// Signed engine torque normalized to the physical maximum.
    torque: f32,
    clutch: f32,
    /// Physical TC cut has already been applied to the delivered load. It is
    /// retained here for acoustic texture only, never applied a second time.
    tc_cut: f32,
    limiter_active: bool,
    shift_phase: u8,
}

impl EngineInput {
    pub fn validate(self) -> Result<Self, String> {
        if !self.rpm.is_finite() || !(0.0..=25_000.0).contains(&self.rpm) {
            return Err(format!("rpm out of range: {}", self.rpm));
        }
        if !self.throttle.is_finite() || !(0.0..=1.0).contains(&self.throttle) {
            return Err(format!("throttle out of range: {}", self.throttle));
        }
        if !self.load.is_finite() || !(0.0..=1.0).contains(&self.load) {
            return Err(format!("load out of range: {}", self.load));
        }
        Ok(self)
    }
}

/// Per-sample state for the synthesized shift gesture (wobble + blip).
/// Driven by the discrete mechanical `shift_phase`; phase edges start an
/// envelope, and the envelope keeps running after the phase returns to `None`
/// so the wobble carries the shift gesture. Allocation-free and deterministic.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShiftGesture {
    last_phase: u8,
    wobble_active: bool,
    wobble_t: f32,
    blip_active: bool,
    blip_t: f32,
}

/// Discrete energy multiplier carried by the physical shift phases. Mirrors
/// the mapping inside `target_acoustic_energy_with_state` so the sample-only
/// runtime can apply the same cut/recovery gesture to a sampled engine.
#[inline]
pub fn shift_energy_gain(phase: u8) -> f32 {
    match phase {
        1 | 3 => 0.12,
        2 | 5 => 0.72,
        _ => 1.0,
    }
}

impl ShiftGesture {
    /// Advances rising-edge detection and the envelope timers by one sample.
    #[inline]
    pub fn step(&mut self, phase: u8, sample_rate: f32, blip_enabled: bool) {
        let is_cut = phase == 1 || phase == 3;
        // A new cut restarts the gesture: the previous wobble/blip is over.
        if is_cut {
            self.wobble_active = false;
            self.blip_active = false;
        }
        if phase == 4 && self.last_phase != 4 && blip_enabled {
            self.blip_active = true;
            self.blip_t = 0.0;
        }
        if (phase == 2 || phase == 5) && self.last_phase != phase {
            self.wobble_active = true;
            self.wobble_t = 0.0;
        }
        let dt = 1.0 / sample_rate;
        if self.wobble_active {
            self.wobble_t += dt;
            if self.wobble_t >= SHIFT_WOBBLE_SECONDS {
                self.wobble_active = false;
            }
        }
        if self.blip_active {
            self.blip_t += dt;
            if self.blip_t >= SHIFT_BLIP_SECONDS {
                self.blip_active = false;
            }
        }
        self.last_phase = phase;
    }

    /// Bounded amplitude multiplier for the current gesture state. `1.0` when
    /// idle; sag -> rebound -> settle on recovery; energy flare on blip.
    #[inline]
    pub fn gain(&self) -> f32 {
        let wobble = if self.wobble_active {
            let decay = (-self.wobble_t / SHIFT_WOBBLE_TAU).exp();
            let omega = std::f32::consts::TAU * SHIFT_WOBBLE_CYCLES / SHIFT_WOBBLE_SECONDS;
            (1.0 - SHIFT_WOBBLE_DEPTH * decay * (omega * self.wobble_t).cos())
                .clamp(SHIFT_WOBBLE_MIN, SHIFT_WOBBLE_MAX)
        } else {
            1.0
        };
        let blip = if self.blip_active {
            (1.0 + SHIFT_BLIP_GAIN * (1.0 - self.blip_t / SHIFT_BLIP_SECONDS))
                .clamp(1.0, 1.0 + SHIFT_BLIP_GAIN)
        } else {
            1.0
        };
        (wobble * blip).clamp(0.0, SHIFT_GAIN_MAX)
    }
}

#[inline]
#[cfg_attr(not(test), allow(dead_code))]
fn target_acoustic_energy(input: EngineInput) -> f32 {
    target_acoustic_energy_with_state(input, MechanicalState::default())
}

#[inline]
fn target_acoustic_energy_with_state(input: EngineInput, mechanical: MechanicalState) -> f32 {
    let rpm_norm =
        ((input.rpm - COAST_IDLE_RPM) / (COAST_MAX_RPM - COAST_IDLE_RPM)).clamp(0.0, 1.0);
    let coast_energy = COAST_BASE_ENERGY + COAST_RPM_ENERGY * rpm_norm;
    let powered_energy = 0.25 * input.throttle + 0.63 * input.load * (0.35 + 0.65 * input.throttle);
    // A shift cut and the physical limiter suppress combustion energy. Their
    // state is discrete and arrives from physics. The synthesized recovery
    // wobble and downshift blip are applied after the energy follower (see
    // `ShiftGesture`), never inside this target.
    let cut_gain = shift_energy_gain(mechanical.shift_phase);
    let limiter_gain = if mechanical.limiter_active { 0.78 } else { 1.0 };
    // Negative torque remains audible during retention/free-rev. The clutch
    // scales this drag contribution without changing the delivered load.
    let retention_energy = (-mechanical.torque).max(0.0) * mechanical.clutch * 0.08;
    (coast_energy + powered_energy * cut_gain * limiter_gain + retention_energy).clamp(0.0, 1.0)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EngineFrame {
    pub throttle: f32,
    pub load: f32,
    pub combustion_source: f32,
    pub pressure_derivative: f32,
    pub pressure_derivative_a: f32,
    pub pressure_derivative_b: f32,
    pub cylinder_pressure: [f32; CYLINDER_COUNT],
    pub cylinder_pressure_derivative: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_pressure_pa: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_temperature_k: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_heat_release_rate: [f32; CYLINDER_COUNT],
    pub cylinder_chamber_phase: [u8; CYLINDER_COUNT],
    pub cylinder_blowdown: [f32; CYLINDER_COUNT],
    pub cylinder_headers: [f32; CYLINDER_COUNT],
    pub cylinder_valve_lift: [f32; CYLINDER_COUNT],
    pub cylinder_valve_area: [f32; CYLINDER_COUNT],
    /// Per-cylinder physical exhaust mass flow (kg/s) from the runner boundary.
    pub cylinder_mass_flow: [f32; CYLINDER_COUNT],
    /// Per-cylinder exhaust-runner gas pressure (Pa).
    pub cylinder_runner_pressure: [f32; CYLINDER_COUNT],
    /// Per-cylinder dynamic acoustic pressure perturbation in the runner waveguide (PHY-062).
    pub cylinder_runner_acoustic_pressure: [f32; CYLINDER_COUNT],
    /// Per-cylinder exhaust-runner gas temperature (K), the PHY-061 wave-speed source.
    pub cylinder_runner_temperature_k: [f32; CYLINDER_COUNT],
    /// Per-cylinder normalised header excitation fed into the runner waveguide.
    pub cylinder_exhaust_excitation: [f32; CYLINDER_COUNT],
    pub pressure_direct: f32,
    pub crankcase: f32,
    pub block: f32,
    pub head: f32,
    pub block_head: f32,
    pub headers_a: f32,
    pub headers_b: f32,
    pub collector_a: f32,
    pub collector_b: f32,
    pub collector_pressure_a: f32,
    pub collector_pressure_b: f32,
    pub exhaust: f32,
    pub turbulence: f32,
    pub master_pre_limiter: f32,
    pub master: f32,
    /// Structure-borne transmission and gearbox radiation injected by
    /// `Gf509Runtime`; summed into the scene dry paths by `AcousticScene`.
    pub transmission: f32,
    /// Gear up/down one-shot blip samples injected by `Gf509Runtime`.
    pub gear_shift: f32,
    pub fired_mask: u16,
    pub crank_phase_deg: f32,
    pub limiter_reduction_db: f32,
}

pub struct V10Engine {
    config: EngineConfig,
    crank: Crankshaft,
    cylinders: [Cylinder; CYLINDER_COUNT],
    block_head: BlockHead,
    headers: [RunnerWaveguide; CYLINDER_COUNT],
    collectors: [Collector; 2],
    source_dc: DcBlocker,
    master_dc: DcBlocker,
    input: EngineInput,
    mechanical: MechanicalState,
    smoothed_energy: f32,
    /// AUD-05: one-pole energy follower coefficients. Depend only on the
    /// sample rate, so they are precomputed at construction instead of
    /// evaluating `exp` once per rendered sample (bit-identical formula).
    energy_attack_alpha: f32,
    energy_release_alpha: f32,
    /// AUD-05: turbulence envelope release coefficient (sample-rate only).
    turbulence_release_alpha: f32,
    turbulence_filter: BandPassNoise,
    turbulence_envelope: f32,
    noise_rng: u64,
    sample_clock: u64,
    gesture: ShiftGesture,
    blip_enabled: bool,
}

impl V10Engine {
    pub fn new(config: EngineConfig) -> Result<Self, String> {
        config.validate()?;
        let sample_rate = config.sample_rate as f32;
        Ok(Self {
            crank: Crankshaft::new(config.sample_rate, config.firing_order),
            cylinders: std::array::from_fn(|index| Cylinder::new(index, &config)),
            block_head: BlockHead::new(sample_rate),
            headers: std::array::from_fn(|index| {
                RunnerWaveguide::new(
                    config.effective_header_lengths_m()[index],
                    config.exhaust_wave_speed_mps,
                    config.header_reflection,
                    sample_rate,
                    config.use_temperature_dependent_wave_speed,
                )
            }),
            collectors: match &config.collector_geometry {
                Some(geometry) => [
                    Collector::from_geometry(sample_rate, -3.5, geometry),
                    Collector::from_geometry(sample_rate, 3.5, geometry),
                ],
                None => [
                    Collector::new(sample_rate, -3.5),
                    Collector::new(sample_rate, 3.5),
                ],
            },
            source_dc: DcBlocker::new(16.0, sample_rate),
            master_dc: DcBlocker::new(16.0, sample_rate),
            input: EngineInput {
                rpm: 0.0,
                throttle: 0.0,
                load: 0.0,
            },
            mechanical: MechanicalState::default(),
            smoothed_energy: 0.0,
            energy_attack_alpha: 1.0 - (-1.0 / (ENERGY_ATTACK_SECONDS * sample_rate)).exp(),
            energy_release_alpha: 1.0 - (-1.0 / (ENERGY_RELEASE_SECONDS * sample_rate)).exp(),
            turbulence_release_alpha: 1.0 - (-1.0 / (0.0038 * sample_rate)).exp(),
            turbulence_filter: BandPassNoise::new(460.0, 2_050.0, sample_rate),
            turbulence_envelope: 0.0,
            noise_rng: config.seed ^ 0xA17E_5EED_D15C_A11E,
            sample_clock: 0,
            gesture: ShiftGesture::default(),
            blip_enabled: config.blip_enabled,
            config,
        })
    }

    pub fn set_input(&mut self, input: EngineInput) -> Result<(), String> {
        self.input = input.validate()?;
        Ok(())
    }

    /// Supplies physical controls that shape synthesis. Continuous values are
    /// interpolated by `Gf509Runtime`; discrete phase/limiter states are applied
    /// immediately at the sample boundary.
    #[inline]
    pub fn set_mechanical_state(
        &mut self,
        torque: f32,
        clutch: f32,
        tc_cut: f32,
        limiter_active: bool,
        shift_phase: crate::runtime::ShiftPhase,
    ) {
        self.mechanical = MechanicalState {
            torque: torque.clamp(-1.0, 1.0),
            clutch: clutch.clamp(0.0, 1.0),
            tc_cut: tc_cut.clamp(0.0, 1.0),
            limiter_active,
            shift_phase: shift_phase as u8,
        };
    }

    /// Current synthesized shift gesture gain (wobble * blip), exactly the
    /// multiplier applied to the procedural energy. `1.0` when idle. The cut
    /// is not part of this gain; it lives in the acoustic energy target. The
    /// GF509 runtime uses this to carry the gesture to the sample layer so the
    /// whole hybrid mix wobbles together.
    #[inline]
    pub fn shift_gesture_gain(&self) -> f32 {
        self.gesture.gain()
    }

    #[inline]
    pub fn render_sample(&mut self) -> EngineFrame {
        let sample_rate = self.config.sample_rate as f32;
        let target_energy = target_acoustic_energy_with_state(self.input, self.mechanical);
        // AUD-05: coefficients depend only on direction + sample rate.
        let alpha = if target_energy > self.smoothed_energy {
            self.energy_attack_alpha
        } else {
            self.energy_release_alpha
        };
        self.smoothed_energy += (target_energy - self.smoothed_energy) * alpha;
        // Synthesized shift gesture, applied after the follower so the wobble
        // and blip retain their exact bounded shape.
        self.gesture.step(self.mechanical.shift_phase, sample_rate, self.blip_enabled);
        let shift_gain = self.gesture.gain();
        // Combustion quality wanders slowly, rather than drawing a new random
        // gain every mechanical cycle. Two incommensurate rates avoid a loop.
        let time = self.sample_clock as f32 / sample_rate;
        let slow_drift = 1.0
            + 0.010 * (std::f32::consts::TAU * 0.83 * time).sin()
            + 0.005 * (std::f32::consts::TAU * 1.17 * time + 1.3).sin();

        let events = self.crank.step(self.input.rpm);
        let deg_per_sample = self.input.rpm.max(0.0) * 6.0 / sample_rate;
        let mut pressure_a = 0.0;
        let mut pressure_b = 0.0;
        let mut derivative_a = 0.0;
        let mut derivative_b = 0.0;
        let mut runners_a = [0.0f32; 5];
        let mut runners_b = [0.0f32; 5];
        let mut header_a = 0.0;
        let mut header_b = 0.0;
        let mut turbulence_trigger = 0.0f32;
        let mut cylinder_pressure = [0.0; CYLINDER_COUNT];
        let mut cylinder_pressure_derivative = [0.0; CYLINDER_COUNT];
        let mut cylinder_chamber_pressure_pa = [0.0; CYLINDER_COUNT];
        let mut cylinder_chamber_temperature_k = [0.0; CYLINDER_COUNT];
        let mut cylinder_chamber_heat_release_rate = [0.0; CYLINDER_COUNT];
        let mut cylinder_chamber_phase = [0u8; CYLINDER_COUNT];
        let mut cylinder_blowdown = [0.0; CYLINDER_COUNT];
        let mut cylinder_headers = [0.0; CYLINDER_COUNT];
        let mut cylinder_valve_lift = [0.0; CYLINDER_COUNT];
        let mut cylinder_valve_area = [0.0; CYLINDER_COUNT];
        let mut cylinder_mass_flow = [0.0; CYLINDER_COUNT];
        let mut cylinder_runner_pressure = [0.0; CYLINDER_COUNT];
        let mut cylinder_runner_acoustic_pressure = [0.0; CYLINDER_COUNT];
        let mut cylinder_runner_temperature_k = [0.0; CYLINDER_COUNT];
        let mut cylinder_exhaust_excitation = [0.0; CYLINDER_COUNT];

        for index in 0..CYLINDER_COUNT {
            if events.fired(index) {
                self.cylinders[index].fire(self.config.cycle_variation);
            }
            let cylinder = self.cylinders[index].process(
                deg_per_sample,
                self.smoothed_energy * shift_gain * slow_drift,
                &self.config,
            );
            if index < 5 {
                pressure_a += cylinder.pressure;
                derivative_a += cylinder.pressure_derivative;
            } else {
                pressure_b += cylinder.pressure;
                derivative_b += cylinder.pressure_derivative;
            }
            let header = self.headers[index]
                .process(cylinder.exhaust_excitation, cylinder.runner_temperature_k);
            cylinder_pressure[index] = cylinder.pressure;
            cylinder_pressure_derivative[index] = cylinder.pressure_derivative;
            cylinder_chamber_pressure_pa[index] = cylinder.chamber_pressure_pa;
            cylinder_chamber_temperature_k[index] = cylinder.chamber_temperature_k;
            cylinder_chamber_heat_release_rate[index] = cylinder.chamber_heat_release_rate;
            cylinder_chamber_phase[index] = cylinder.chamber_phase as u8;
            cylinder_blowdown[index] = cylinder.blowdown;
            cylinder_headers[index] = header;
            cylinder_valve_lift[index] = cylinder.valve_lift;
            cylinder_valve_area[index] = cylinder.effective_area_m2;
            cylinder_mass_flow[index] = cylinder.exhaust_mass_flow_kg_s;
            cylinder_runner_pressure[index] = cylinder.runner_pressure_pa;
            cylinder_runner_acoustic_pressure[index] = self.headers[index].acoustic_pressure();
            cylinder_runner_temperature_k[index] = cylinder.runner_temperature_k;
            cylinder_exhaust_excitation[index] = cylinder.exhaust_excitation;
            turbulence_trigger += cylinder.blowdown.abs();
            if index < 5 {
                runners_a[index] = header;
                header_a += header;
            } else {
                runners_b[index - 5] = header;
                header_b += header;
            }
        }

        let pressure = pressure_a + pressure_b;
        let derivative = derivative_a + derivative_b;
        // Bank acoustic asymmetry: models crank journal stagger and asymmetric acoustic
        // transmission to the cockpit/T-cam observation point, exciting the Order 0.5
        // (f0 / 2) 5-cylinder bank pulsation observed at -5.2 dB in the Renault R24 onboard.
        const BANK_A_ACOUSTIC_WEIGHT: f32 = 1.20;
        const BANK_B_ACOUSTIC_WEIGHT: f32 = 0.80;
        let structural_derivative = self
            .source_dc
            .process(derivative_a * BANK_A_ACOUSTIC_WEIGHT + derivative_b * BANK_B_ACOUSTIC_WEIGHT);
        let structural_pressure = pressure_a * 1.10 + pressure_b * 0.90;
        let structure = self
            .block_head
            .process(structural_pressure, structural_derivative);
        let block_sum = structure.pressure_direct * self.config.pressure_direct_gain
            + structure.crankcase * self.config.crankcase_gain
            + structure.block * self.config.block_gain
            + structure.head * self.config.head_gain;
        let collector_a = self.collectors[0].process_bank(&runners_a);
        let collector_b = self.collectors[1].process_bank(&runners_b);
        let exhaust = (collector_a.radiated * BANK_A_ACOUSTIC_WEIGHT
            + collector_b.radiated * BANK_B_ACOUSTIC_WEIGHT)
            * self.config.exhaust_gain;
        let attack = 0.34;
        let release = self.turbulence_release_alpha;
        if turbulence_trigger > self.turbulence_envelope {
            self.turbulence_envelope += attack * (turbulence_trigger - self.turbulence_envelope);
        } else {
            self.turbulence_envelope += release * (turbulence_trigger - self.turbulence_envelope);
        }
        let mut x = self.noise_rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.noise_rng = x;
        let noise_bits = x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40;
        let noise = noise_bits as f32 / ((1u64 << 24) - 1) as f32 * 2.0 - 1.0;
        // TC is already represented in `input.load`; this small modulation
        // conveys the physical cut as texture without reducing load twice.
        let tc_texture = 1.0 + 0.12 * self.mechanical.tc_cut;
        let turbulence = self.turbulence_filter.process(noise)
            * self.turbulence_envelope.sqrt()
            * self.config.turbulence_gain
            * tc_texture;
        let pre = self
            .master_dc
            .process((block_sum + exhaust + turbulence) * self.config.master_gain);
        self.sample_clock = self.sample_clock.wrapping_add(1);

        // Safety only. At nominal settings the candidate must stay below this
        // knee; any material reduction is reported and fails the perceptual gate.
        let abs = pre.abs();
        let (master, reduction_db) = if abs <= 0.92 {
            (pre, 0.0)
        } else {
            let excess = abs - 0.92;
            let limited_abs = 0.92 + 0.075 * (excess / 0.075).tanh();
            let limited = pre.signum() * limited_abs.min(0.995);
            let reduction = 20.0 * (limited.abs() / abs.max(1.0e-12)).log10();
            (limited, reduction)
        };

        debug_assert!(master.is_finite());
        EngineFrame {
            throttle: self.input.throttle,
            load: self.input.load,
            combustion_source: pressure,
            pressure_derivative: derivative,
            pressure_derivative_a: derivative_a,
            pressure_derivative_b: derivative_b,
            cylinder_pressure,
            cylinder_pressure_derivative,
            cylinder_chamber_pressure_pa,
            cylinder_chamber_temperature_k,
            cylinder_chamber_heat_release_rate,
            cylinder_chamber_phase,
            cylinder_blowdown,
            cylinder_headers,
            cylinder_valve_lift,
            cylinder_valve_area,
            cylinder_mass_flow,
            cylinder_runner_pressure,
            cylinder_runner_acoustic_pressure,
            cylinder_runner_temperature_k,
            cylinder_exhaust_excitation,
            pressure_direct: structure.pressure_direct,
            crankcase: structure.crankcase,
            block: structure.block,
            head: structure.head,
            block_head: block_sum,
            headers_a: header_a,
            headers_b: header_b,
            collector_a: collector_a.radiated,
            collector_b: collector_b.radiated,
            collector_pressure_a: collector_a.pressure,
            collector_pressure_b: collector_b.pressure,
            exhaust,
            turbulence,
            master_pre_limiter: pre,
            master,
            transmission: 0.0,
            gear_shift: 0.0,
            fired_mask: events.mask,
            crank_phase_deg: events.crank_phase_deg,
            limiter_reduction_db: reduction_db,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coast_energy_retains_rpm_dependent_engine_drag() {
        let closed = |rpm| {
            target_acoustic_energy(EngineInput {
                rpm,
                throttle: 0.0,
                load: 0.0,
            })
        };

        assert!((closed(1_000.0) - 0.12).abs() < 1.0e-6);
        assert!((closed(8_000.0) - 0.22).abs() < 1.0e-6);
        assert!((closed(15_000.0) - 0.32).abs() < 1.0e-6);
    }

    #[test]
    fn lift_off_energy_release_uses_413_ms_time_constant() {
        let config = EngineConfig::default();
        let sample_rate = config.sample_rate as usize;
        let mut engine = V10Engine::new(config).unwrap();
        engine.smoothed_energy = 1.0;
        engine
            .set_input(EngineInput {
                rpm: 15_000.0,
                throttle: 0.0,
                load: 0.0,
            })
            .unwrap();

        for _ in 0..(sample_rate * 413 / 1_000) {
            engine.render_sample();
        }

        let expected_after_one_tau = 0.32 + (1.0 - 0.32) / std::f32::consts::E;
        assert!((engine.smoothed_energy - expected_after_one_tau).abs() < 2.0e-4);
    }

    #[test]
    fn identical_seed_and_controls_are_bit_deterministic() {
        let config = EngineConfig::default();
        let mut a = V10Engine::new(config.clone()).unwrap();
        let mut b = V10Engine::new(config).unwrap();
        let input = EngineInput {
            rpm: 5_000.0,
            throttle: 0.72,
            load: 0.78,
        };
        a.set_input(input).unwrap();
        b.set_input(input).unwrap();
        for _ in 0..96_000 {
            assert_eq!(
                a.render_sample().master.to_bits(),
                b.render_sample().master.to_bits()
            );
        }
    }

    #[test]
    fn nominal_5000_rpm_is_finite_nonzero_and_not_limiter_driven() {
        let config = EngineConfig::default();
        let mut engine = V10Engine::new(config).unwrap();
        engine
            .set_input(EngineInput {
                rpm: 5_000.0,
                throttle: 0.72,
                load: 0.78,
            })
            .unwrap();
        let mut energy = 0.0f64;
        let mut peak = 0.0f32;
        let mut worst_reduction = 0.0f32;
        for _ in 0..96_000 {
            let frame = engine.render_sample();
            assert!(frame.master.is_finite());
            energy += (frame.master * frame.master) as f64;
            peak = peak.max(frame.master.abs());
            worst_reduction = worst_reduction.min(frame.limiter_reduction_db);
        }
        assert!(energy > 1.0e-6);
        assert!(peak < 1.0);
        assert!(
            worst_reduction > -0.5,
            "limiter reduction {worst_reduction} dB"
        );
    }

    #[test]
    fn ten_cylinder_signals_remain_independent_before_bank_sum() {
        let baseline = EngineConfig::default();
        let mut altered = baseline.clone();
        altered.cylinder_signature[3] = 0.75;
        let mut engine = V10Engine::new(baseline).unwrap();
        let mut isolated = V10Engine::new(altered).unwrap();
        let input = EngineInput {
            rpm: 7_499.0,
            throttle: 0.72,
            load: 0.66,
        };
        engine.set_input(input).unwrap();
        isolated.set_input(input).unwrap();
        let mut energy = [0.0f64; CYLINDER_COUNT];
        let mut altered_target = false;
        for _ in 0..48_000 {
            let frame = engine.render_sample();
            let isolated_frame = isolated.render_sample();
            let bank_a: f32 = frame.cylinder_pressure_derivative[..5].iter().sum();
            let bank_b: f32 = frame.cylinder_pressure_derivative[5..].iter().sum();
            assert!((bank_a - frame.pressure_derivative_a).abs() < 1.0e-6);
            assert!((bank_b - frame.pressure_derivative_b).abs() < 1.0e-6);
            for (total, sample) in energy
                .iter_mut()
                .zip(frame.cylinder_pressure_derivative.iter())
            {
                *total += (*sample * *sample) as f64;
            }
            for index in 0..CYLINDER_COUNT {
                if index == 3 {
                    altered_target |= frame.cylinder_pressure_derivative[index].to_bits()
                        != isolated_frame.cylinder_pressure_derivative[index].to_bits();
                } else {
                    assert_eq!(
                        frame.cylinder_pressure_derivative[index].to_bits(),
                        isolated_frame.cylinder_pressure_derivative[index].to_bits(),
                        "cylinder {index} changed when only cylinder 3 was altered"
                    );
                    assert_eq!(
                        frame.cylinder_headers[index].to_bits(),
                        isolated_frame.cylinder_headers[index].to_bits(),
                        "header {index} changed when only cylinder 3 was altered"
                    );
                }
            }
        }
        assert!(energy.iter().all(|value| *value > 1.0e-8), "{energy:?}");
        assert!(altered_target, "altered cylinder never diverged");
    }

    #[test]
    fn invalid_control_fails_before_render() {
        let mut engine = V10Engine::new(EngineConfig::default()).unwrap();
        assert!(engine
            .set_input(EngineInput {
                rpm: f32::NAN,
                throttle: 0.5,
                load: 0.5
            })
            .is_err());
        assert!(engine
            .set_input(EngineInput {
                rpm: 5_000.0,
                throttle: 1.2,
                load: 0.5
            })
            .is_err());
    }

    use crate::runtime::ShiftPhase;

    fn shift_test_engine() -> V10Engine {
        let mut engine = V10Engine::new(EngineConfig::default()).unwrap();
        engine
            .set_input(EngineInput {
                rpm: 9_000.0,
                throttle: 0.9,
                load: 0.9,
            })
            .unwrap();
        engine
    }

    #[test]
    fn shift_cut_keeps_the_static_target_levels() {
        let input = EngineInput {
            rpm: 8_000.0,
            throttle: 1.0,
            load: 1.0,
        };
        let base = target_acoustic_energy_with_state(input, MechanicalState::default());
        let cut = target_acoustic_energy_with_state(
            input,
            MechanicalState {
                shift_phase: ShiftPhase::UpshiftCut as u8,
                ..MechanicalState::default()
            },
        );
        let recovery = target_acoustic_energy_with_state(
            input,
            MechanicalState {
                shift_phase: ShiftPhase::UpshiftRecovery as u8,
                ..MechanicalState::default()
            },
        );
        assert!(cut < base * 0.35, "cut={cut} base={base}");
        assert!(recovery > cut && recovery < base, "recovery={recovery}");
    }

    #[test]
    fn shift_wobble_sags_rebounds_is_bounded_and_settles() {
        let sample_rate = EngineConfig::default().sample_rate as f32;
        let mut engine = shift_test_engine();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftCut);
        for _ in 0..2_048 {
            engine.render_sample();
        }
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftRecovery);
        let total = (SHIFT_WOBBLE_SECONDS * 2.0 * sample_rate) as usize;
        let mut first_active = None;
        let mut min = f32::MAX;
        let mut max = 0.0f32;
        for _ in 0..total {
            engine.render_sample();
            if engine.gesture.wobble_active {
                let gain = engine.shift_gesture_gain();
                first_active.get_or_insert(gain);
                min = min.min(gain);
                max = max.max(gain);
            }
        }
        let first = first_active.expect("recovery must start the wobble");
        assert!(
            first <= 0.72,
            "recovery must open with the sag, got {first}"
        );
        assert!(min >= SHIFT_WOBBLE_MIN - 1e-6, "wobble min {min}");
        assert!(
            max > 1.10 && max <= SHIFT_WOBBLE_MAX + 1e-6,
            "wobble must rebound under the hard bound, got {max}"
        );
        assert!(!engine.gesture.wobble_active, "wobble must end");
        assert_eq!(engine.shift_gesture_gain(), 1.0);
    }

    #[test]
    fn shift_gesture_gain_is_unity_idle_and_excludes_the_cut() {
        let mut engine = shift_test_engine();
        assert_eq!(engine.shift_gesture_gain(), 1.0);
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftCut);
        engine.render_sample();
        assert_eq!(
            engine.shift_gesture_gain(),
            1.0,
            "the cut lives in the energy target, not in the gesture gain"
        );
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftRecovery);
        engine.render_sample();
        let gain = engine.shift_gesture_gain();
        assert!(gain < 0.75, "gesture must open on recovery, got {gain}");
    }

    #[test]
    fn downshift_blip_flares_then_decays_to_unity() {
        let sample_rate = EngineConfig::default().sample_rate as f32;
        let mut engine = shift_test_engine();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftCut);
        for _ in 0..512 {
            engine.render_sample();
        }
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftBlip);
        engine.render_sample();
        let first = engine.shift_gesture_gain();
        assert!(first > 1.35, "blip must flare on phase 4, got {first}");
        let total = (SHIFT_BLIP_SECONDS * 2.0 * sample_rate) as usize;
        let mut max = first;
        for _ in 1..total {
            engine.render_sample();
            max = max.max(engine.shift_gesture_gain());
        }
        assert!(max <= 1.0 + SHIFT_BLIP_GAIN + 1e-6, "blip max {max}");
        assert!(!engine.gesture.blip_active, "blip must end");
        assert_eq!(engine.shift_gesture_gain(), 1.0);
    }

    #[test]
    fn blip_is_suppressed_when_disabled() {
        let mut config = EngineConfig::default();
        config.blip_enabled = false;
        let mut engine = V10Engine::new(config).unwrap();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftCut);
        for _ in 0..512 {
            engine.render_sample();
        }
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftBlip);
        engine.render_sample();
        assert_eq!(
            engine.shift_gesture_gain(),
            1.0,
            "blip must stay flat when disabled"
        );
        assert!(!engine.gesture.blip_active);
    }

    #[test]
    fn overlapping_shift_gestures_stay_hard_bounded() {
        let sample_rate = EngineConfig::default().sample_rate as f32;
        let mut engine = shift_test_engine();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftBlip);
        engine.render_sample();
        for _ in 0..(0.05 * sample_rate) as usize {
            engine.render_sample();
        }
        // Defensive overlap: telemetry sequences blip then recovery, but a
        // direct caller may stack them; the combined gain must stay bounded.
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftRecovery);
        let mut max = 0.0f32;
        for _ in 0..(0.3 * sample_rate) as usize {
            engine.render_sample();
            max = max.max(engine.gesture.gain());
        }
        assert!(max <= SHIFT_GAIN_MAX + 1e-6, "combined gain {max}");
    }

    #[test]
    fn new_cut_cancels_an_active_gesture() {
        let mut engine = shift_test_engine();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftCut);
        engine.render_sample();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftRecovery);
        engine.render_sample();
        assert!(engine.gesture.wobble_active);
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftCut);
        engine.render_sample();
        assert!(!engine.gesture.wobble_active && !engine.gesture.blip_active);
        assert_eq!(engine.gesture.gain(), 1.0);
    }

    #[test]
    fn shift_gesture_sequence_is_bit_deterministic() {
        let config = EngineConfig::default();
        let mut a = V10Engine::new(config.clone()).unwrap();
        let mut b = V10Engine::new(config).unwrap();
        let input = EngineInput {
            rpm: 9_000.0,
            throttle: 0.9,
            load: 0.9,
        };
        a.set_input(input).unwrap();
        b.set_input(input).unwrap();
        for sample in 0..40_000 {
            let phase = match sample {
                0..=1_999 => ShiftPhase::None,
                2_000..=5_999 => ShiftPhase::UpshiftCut,
                6_000..=7_999 => ShiftPhase::UpshiftRecovery,
                8_000..=11_999 => ShiftPhase::None,
                12_000..=15_999 => ShiftPhase::DownshiftCut,
                16_000..=17_999 => ShiftPhase::DownshiftBlip,
                _ => ShiftPhase::None,
            };
            a.set_mechanical_state(0.8, 1.0, 0.0, false, phase);
            b.set_mechanical_state(0.8, 1.0, 0.0, false, phase);
            assert_eq!(
                a.render_sample().master.to_bits(),
                b.render_sample().master.to_bits(),
                "sample {sample} diverged"
            );
        }
    }
}
