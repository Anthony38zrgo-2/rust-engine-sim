//! Procedural V10 engine orchestrator coordinating the 7 per-sample synthesis stages.

use crate::config::{EngineConfig, EngineStructuralProfile};
use crate::control::{
    target_acoustic_energy_with_range, EnergyFollower, EngineInput, MechanicalState, ShiftGesture,
    ShiftPhase, COAST_IDLE_RPM,
};
use crate::dsp::{limit_master, DcBlocker, MasterLimiterAdaa1};
use crate::physics::{Crankshaft, Cylinder, CYLINDER_COUNT};
use crate::synthesis::exhaust::{Collector, CollectorFrame, RunnerWaveguide};
use crate::synthesis::frame::EngineFrame;
use crate::synthesis::structural::BlockHead;
use crate::synthesis::turbulence::TurbulenceGenerator;

/// Bank acoustic asymmetry weights reproducing odd-order V10 bank stagger.
const BANK_A_ACOUSTIC_WEIGHT: f32 = 1.20;
const BANK_B_ACOUSTIC_WEIGHT: f32 = 0.80;

/// Stateful, allocation-free per-sample procedural V10 engine synthesizer.
pub struct V10Engine {
    config: EngineConfig,
    crank: Crankshaft,
    cylinders: [Cylinder; CYLINDER_COUNT],
    block_head: BlockHead,
    headers: [RunnerWaveguide; CYLINDER_COUNT],
    collectors: [Collector; 2],
    source_dc: DcBlocker,
    master_dc: DcBlocker,
    master_limiter: MasterLimiterAdaa1,
    input: EngineInput,
    mechanical: MechanicalState,
    energy: EnergyFollower,
    turbulence: TurbulenceGenerator,
    sample_clock: u64,
    gesture: ShiftGesture,
    blip_enabled: bool,
}

impl V10Engine {
    /// Validates `config` and preallocates all physical cylinders, waveguides, and filters.
    pub fn new(config: EngineConfig) -> Result<Self, String> {
        config.validate()?;
        let sample_rate = config.sample_rate as f32;
        let (offset_a, offset_b) = if config.bank_split == config.cylinder_count {
            (0.0, 0.0)
        } else {
            (-3.5, 3.5)
        };
        Ok(Self {
            crank: Crankshaft::with_cylinder_count(
                config.sample_rate,
                config.firing_order,
                config.cylinder_count,
            ),
            cylinders: std::array::from_fn(|index| Cylinder::new(index, &config)),
            block_head: BlockHead::with_profile(sample_rate, config.structural_profile),
            headers: std::array::from_fn(|index| {
                RunnerWaveguide::new(
                    config.effective_header_lengths_m()[index],
                    config.exhaust_wave_speed_mps,
                    config.header_reflection,
                    sample_rate,
                    config.use_temperature_dependent_wave_speed,
                )
                .with_refined_dsp(config.use_refined_dsp)
            }),
            collectors: match &config.collector_geometry {
                Some(geometry) => [
                    Collector::from_geometry(sample_rate, offset_a, geometry)
                        .with_refined_dsp(config.use_refined_dsp),
                    Collector::from_geometry(sample_rate, offset_b, geometry)
                        .with_refined_dsp(config.use_refined_dsp),
                ],
                None => [
                    Collector::new(sample_rate, offset_a).with_refined_dsp(config.use_refined_dsp),
                    Collector::new(sample_rate, offset_b).with_refined_dsp(config.use_refined_dsp),
                ],
            },
            source_dc: DcBlocker::new(16.0, sample_rate),
            master_dc: DcBlocker::new(16.0, sample_rate),
            master_limiter: MasterLimiterAdaa1::new(),
            input: EngineInput {
                rpm: 0.0,
                throttle: 0.0,
                load: 0.0,
            },
            mechanical: MechanicalState::default(),
            energy: EnergyFollower::new(sample_rate),
            turbulence: match config.structural_profile {
                EngineStructuralProfile::F1V10 => TurbulenceGenerator::new(
                    config.seed,
                    sample_rate,
                )
                .with_refined_jet(config.use_refined_dsp, 2_300.0, 4_200.0),
                EngineStructuralProfile::CastIronSohcL4 => {
                    TurbulenceGenerator::with_band(config.seed, sample_rate, 210.0, 1_350.0)
                        .with_refined_jet(config.use_refined_dsp, 1_850.0, 3_100.0)
                }
            },
            sample_clock: 0,
            gesture: ShiftGesture::default(),
            blip_enabled: config.blip_enabled,
            config,
        })
    }

    /// Validates and latches primary engine control inputs.
    pub fn set_input(&mut self, input: EngineInput) -> Result<(), String> {
        self.input = input.validate()?;
        Ok(())
    }

    /// Supplies smoothed/discrete drivetrain mechanical controls shaping synthesis.
    #[inline]
    pub fn set_mechanical_state(
        &mut self,
        torque: f32,
        clutch: f32,
        tc_cut: f32,
        limiter_active: bool,
        shift_phase: ShiftPhase,
    ) {
        self.mechanical =
            MechanicalState::from_controls(torque, clutch, tc_cut, limiter_active, shift_phase);
    }

    /// Current synthesized shift gesture gain (`wobble * blip`, `1.0` when idle).
    #[inline]
    pub fn shift_gesture_gain(&self) -> f32 {
        self.gesture.gain()
    }

    /// Renders a single sample preserving the exact 7-stage evaluation order of `v1`.
    #[inline]
    pub fn render_sample(&mut self) -> EngineFrame {
        let sample_rate = self.config.sample_rate as f32;

        // Stage 1: Acoustic energy target, asymmetric follower, shift gesture, and crank step.
        let target_energy = target_acoustic_energy_with_range(
            self.input,
            self.mechanical,
            COAST_IDLE_RPM,
            self.config.coast_max_rpm,
        );
        let smoothed_energy = self.energy.step(target_energy);
        self.gesture
            .step(self.mechanical.shift_phase, sample_rate, self.blip_enabled);
        let shift_gain = self.gesture.gain();
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

        // Stages 2 & 3: Cylinder firing/thermodynamics and primary runner waveguide propagation.
        let active_cylinders = self.config.cylinder_count;
        let bank_split = self.config.bank_split;
        let has_bank_b = bank_split < active_cylinders;
        let cylinder_energy = smoothed_energy * shift_gain * slow_drift;
        for index in 0..active_cylinders {
            if events.fired(index) {
                if self.config.use_refined_dsp {
                    self.cylinders[index].fire_at(
                        self.config.cycle_variation,
                        self.crank.fire_overshoot_deg(index),
                    );
                } else {
                    self.cylinders[index].fire(self.config.cycle_variation);
                }
            }
            let cylinder =
                self.cylinders[index].process(deg_per_sample, cylinder_energy, &self.config);
            if index < bank_split {
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
            if index < bank_split {
                runners_a[index] = header;
                header_a += header;
            } else {
                runners_b[index - bank_split] = header;
                header_b += header;
            }
        }

        // Stage 4: Asymmetric bank weighting and block/head/crankcase modal structure.
        let pressure = pressure_a + pressure_b;
        let derivative = derivative_a + derivative_b;
        let (structural_derivative_raw, structural_pressure) = if has_bank_b {
            (
                derivative_a * BANK_A_ACOUSTIC_WEIGHT + derivative_b * BANK_B_ACOUSTIC_WEIGHT,
                pressure_a * 1.10 + pressure_b * 0.90,
            )
        } else {
            (derivative_a, pressure_a)
        };
        let structural_derivative = self.source_dc.process(structural_derivative_raw);
        let structure = if self.config.use_refined_dsp {
            self.block_head.process_modulated(
                structural_pressure,
                structural_derivative,
                self.input.load,
            )
        } else {
            self.block_head
                .process(structural_pressure, structural_derivative)
        };
        let block_sum = structure.pressure_direct * self.config.pressure_direct_gain
            + structure.crankcase * self.config.crankcase_gain
            + structure.block * self.config.block_gain
            + structure.head * self.config.head_gain;

        // Stage 5: Collector junctions with bank asymmetry (or single collector when inline).
        let collector_a = self.collectors[0].process_bank(&runners_a);
        let collector_b = if has_bank_b {
            self.collectors[1].process_bank(&runners_b)
        } else {
            CollectorFrame::default()
        };
        let exhaust = if has_bank_b {
            (collector_a.radiated * BANK_A_ACOUSTIC_WEIGHT
                + collector_b.radiated * BANK_B_ACOUSTIC_WEIGHT)
                * self.config.exhaust_gain
        } else {
            collector_a.radiated * 1.35 * self.config.exhaust_gain
        };

        // Stage 6: Blowdown-triggered turbulent exhaust noise.
        let rpm_norm = ((self.input.rpm - 1_500.0)
            / (self.config.coast_max_rpm - 1_500.0).max(500.0))
        .clamp(0.0, 1.0);
        let turbulence = if self.config.use_refined_dsp {
            self.turbulence.process_with_rpm(
                turbulence_trigger,
                self.config.turbulence_gain,
                self.mechanical.tc_cut,
                rpm_norm,
            )
        } else {
            self.turbulence.process(
                turbulence_trigger,
                self.config.turbulence_gain,
                self.mechanical.tc_cut,
            )
        };

        // Stage 7: Master sum, DC blocking, sample clock advance, and safety limiter.
        let pre = self
            .master_dc
            .process((block_sum + exhaust + turbulence) * self.config.master_gain);
        self.sample_clock = self.sample_clock.wrapping_add(1);
        let (master, reduction_db) = if self.config.use_refined_dsp {
            self.master_limiter.process(pre)
        } else {
            limit_master(pre)
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
    use crate::control::{
        SHIFT_BLIP_GAIN, SHIFT_BLIP_SECONDS, SHIFT_GAIN_MAX, SHIFT_WOBBLE_MAX, SHIFT_WOBBLE_MIN,
        SHIFT_WOBBLE_SECONDS,
    };

    #[test]
    fn lift_off_energy_release_uses_413_ms_time_constant() {
        let config = EngineConfig::default();
        let sample_rate = config.sample_rate as usize;
        let mut engine = V10Engine::new(config).unwrap();
        engine.energy.set_value(1.0);
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
        assert!((engine.energy.value() - expected_after_one_tau).abs() < 2.0e-4);
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
        assert!(energy > 1.0e-6 && peak < 1.0 && worst_reduction > -0.5);
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
                    );
                    assert_eq!(
                        frame.cylinder_headers[index].to_bits(),
                        isolated_frame.cylinder_headers[index].to_bits(),
                    );
                }
            }
        }
        assert!(energy.iter().all(|value| *value > 1.0e-8));
        assert!(altered_target);
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
            if engine.gesture.is_wobble_active() {
                let gain = engine.shift_gesture_gain();
                first_active.get_or_insert(gain);
                min = min.min(gain);
                max = max.max(gain);
            }
        }
        let first = first_active.expect("recovery must start the wobble");
        assert!(first <= 0.72);
        assert!(min >= SHIFT_WOBBLE_MIN - 1e-6);
        assert!(max > 1.10 && max <= SHIFT_WOBBLE_MAX + 1e-6);
        assert!(!engine.gesture.is_wobble_active());
        assert_eq!(engine.shift_gesture_gain(), 1.0);
    }

    #[test]
    fn shift_gesture_gain_is_unity_idle_and_excludes_the_cut() {
        let mut engine = shift_test_engine();
        assert_eq!(engine.shift_gesture_gain(), 1.0);
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftCut);
        engine.render_sample();
        assert_eq!(engine.shift_gesture_gain(), 1.0);
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftRecovery);
        engine.render_sample();
        assert!(engine.shift_gesture_gain() < 0.75);
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
        assert!(first > 1.35);
        let total = (SHIFT_BLIP_SECONDS * 2.0 * sample_rate) as usize;
        let mut max = first;
        for _ in 1..total {
            engine.render_sample();
            max = max.max(engine.shift_gesture_gain());
        }
        assert!(max <= 1.0 + SHIFT_BLIP_GAIN + 1e-6);
        assert!(!engine.gesture.is_blip_active());
        assert_eq!(engine.shift_gesture_gain(), 1.0);
    }

    #[test]
    fn blip_is_suppressed_when_disabled() {
        let config = EngineConfig {
            blip_enabled: false,
            ..EngineConfig::default()
        };
        let mut engine = V10Engine::new(config).unwrap();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftCut);
        for _ in 0..512 {
            engine.render_sample();
        }
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftBlip);
        engine.render_sample();
        assert_eq!(engine.shift_gesture_gain(), 1.0);
        assert!(!engine.gesture.is_blip_active());
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
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::DownshiftRecovery);
        let mut max = 0.0f32;
        for _ in 0..(0.3 * sample_rate) as usize {
            engine.render_sample();
            max = max.max(engine.gesture.gain());
        }
        assert!(max <= SHIFT_GAIN_MAX + 1e-6);
    }

    #[test]
    fn new_cut_cancels_an_active_gesture() {
        let mut engine = shift_test_engine();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftCut);
        engine.render_sample();
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftRecovery);
        engine.render_sample();
        assert!(engine.gesture.is_wobble_active());
        engine.set_mechanical_state(0.8, 1.0, 0.0, false, ShiftPhase::UpshiftCut);
        engine.render_sample();
        assert!(!engine.gesture.is_wobble_active() && !engine.gesture.is_blip_active());
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

    #[test]
    fn ford_pinto_rs2000_sweep_is_finite_with_single_collector() {
        let mut engine = V10Engine::new(EngineConfig::ford_pinto_rs2000(48_000)).unwrap();
        let total = 48_000;
        let mut sum_sq = 0.0f64;
        let mut peak = 0.0f32;
        let mut worst_reduction = 0.0f32;
        for sample in 0..total {
            let progress = sample as f32 / (total - 1) as f32;
            let rpm = 1_500.0 + 5_700.0 * progress;
            engine
                .set_input(EngineInput {
                    rpm,
                    throttle: 0.90,
                    load: 0.88,
                })
                .unwrap();
            let frame = engine.render_sample();
            assert!(frame.master.is_finite());
            assert_eq!(frame.collector_b, 0.0);
            for inactive in &frame.cylinder_pressure[4..] {
                assert_eq!(*inactive, 0.0);
            }
            peak = peak.max(frame.master.abs());
            worst_reduction = worst_reduction.min(frame.limiter_reduction_db);
            sum_sq += (frame.master * frame.master) as f64;
        }
        let rms = (sum_sq / total as f64).sqrt();
        assert!(rms > 0.01 && rms < 0.5, "unexpected Pinto RMS: {rms}");
        assert!(peak < 0.95, "unexpected Pinto peak: {peak}");
        assert_eq!(
            worst_reduction, 0.0,
            "Pinto sweep unexpectedly hit limiter: {worst_reduction}"
        );
    }
}
