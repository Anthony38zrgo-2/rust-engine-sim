//! Control plane types, telemetry validation, interpolation, energy target/follower,
//! and shift gesture envelopes.

pub mod energy;
pub mod interpolation;
pub mod shift_gesture;
pub mod types;

pub use energy::{
    target_acoustic_energy, target_acoustic_energy_with_range, target_acoustic_energy_with_state,
    EnergyFollower, COAST_BASE_ENERGY, COAST_IDLE_RPM, COAST_MAX_RPM, COAST_RPM_ENERGY,
    ENERGY_ATTACK_SECONDS, ENERGY_RELEASE_SECONDS,
};
pub use interpolation::ControlInterpolator;
pub use shift_gesture::{
    shift_energy_gain, ShiftGesture, SHIFT_BLIP_GAIN, SHIFT_BLIP_SECONDS, SHIFT_GAIN_MAX,
    SHIFT_WOBBLE_CYCLES, SHIFT_WOBBLE_DEPTH, SHIFT_WOBBLE_MAX, SHIFT_WOBBLE_MIN,
    SHIFT_WOBBLE_SECONDS, SHIFT_WOBBLE_TAU,
};
pub use types::{EngineInput, MechanicalState, RuntimeTelemetry, ShiftPhase, TorqueSign};

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_telemetry() -> RuntimeTelemetry {
        RuntimeTelemetry {
            rpm: 9_000.0,
            throttle: 0.8,
            normalized_engine_load: 0.7,
            normalized_engine_torque: 0.6,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: 500.0,
            throttle_derivative: 1.0,
            gear: 4,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: 0.016,
        }
    }

    #[test]
    fn runtime_telemetry_validation_enforces_field_bounds() {
        let valid = valid_telemetry();
        assert!(valid.validate().is_ok());

        let mut invalid = valid;
        invalid.rpm = 25_001.0;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.normalized_engine_load = -0.1;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.normalized_engine_load = 1.05;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.normalized_engine_torque = -1.1;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.normalized_engine_torque = 1.1;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.rpm_derivative = f32::NAN;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.throttle_derivative = f32::INFINITY;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.gear = -2;
        assert!(invalid.validate().is_err());
        invalid.gear = 13;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.clutch_engagement = -0.1;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.tc_cut_ratio = 1.1;
        assert!(invalid.validate().is_err());

        invalid = valid;
        invalid.dt_seconds = 1.2;
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn torque_sign_and_shift_phase_conversions() {
        assert_eq!(TorqueSign::from_i32(-1).unwrap(), TorqueSign::Negative);
        assert_eq!(TorqueSign::from_i32(0).unwrap(), TorqueSign::Neutral);
        assert_eq!(TorqueSign::from_i32(1).unwrap(), TorqueSign::Positive);
        assert!(TorqueSign::from_i32(2).is_err());

        assert_eq!(ShiftPhase::from_i32(0).unwrap(), ShiftPhase::None);
        assert_eq!(ShiftPhase::from_i32(1).unwrap(), ShiftPhase::UpshiftCut);
        assert_eq!(
            ShiftPhase::from_i32(2).unwrap(),
            ShiftPhase::UpshiftRecovery
        );
        assert_eq!(ShiftPhase::from_i32(3).unwrap(), ShiftPhase::DownshiftCut);
        assert_eq!(ShiftPhase::from_i32(4).unwrap(), ShiftPhase::DownshiftBlip);
        assert_eq!(
            ShiftPhase::from_i32(5).unwrap(),
            ShiftPhase::DownshiftRecovery
        );
        assert!(ShiftPhase::from_i32(6).is_err());
    }

    #[test]
    fn control_rate_steps_converge_without_zipper_and_switch_discrete_fields() {
        let sample_rate = 48_000;
        let mut interp = ControlInterpolator::default();
        let start = RuntimeTelemetry {
            rpm: 7_499.0,
            throttle: 0.92,
            normalized_engine_load: 0.88,
            normalized_engine_torque: 0.75,
            torque_sign: TorqueSign::Positive,
            rpm_derivative: 1200.0,
            throttle_derivative: 0.5,
            gear: 3,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: 0.0,
        };
        interp.update(start, sample_rate).unwrap();
        let from = interp.next_sample();

        let target = RuntimeTelemetry {
            rpm: 12_000.0,
            throttle: 0.4,
            normalized_engine_load: 0.35,
            normalized_engine_torque: -0.2,
            torque_sign: TorqueSign::Negative,
            rpm_derivative: -2000.0,
            throttle_derivative: -2.0,
            gear: 4,
            shift_phase: ShiftPhase::UpshiftCut,
            clutch_engagement: 0.6,
            tc_cut_ratio: 0.3,
            rev_limiter_active: true,
            dt_seconds: 1.0 / 120.0,
        };
        interp.update(target, sample_rate).unwrap();
        let n = interp.remaining();
        let steps = interp.steps();
        assert!(n > 0);
        let expect = |a: f32, b: f32| (b - a) / n as f32;
        assert!((steps[0] - expect(from.rpm, target.rpm)).abs() < 1e-6);
        assert!((steps[6] - expect(from.clutch_engagement, target.clutch_engagement)).abs() < 1e-6);

        let mut prev_rpm = from.rpm;
        let mut max_step = 0.0f32;
        for i in 0..n {
            let rendered = interp.next_sample();
            assert_eq!(rendered.shift_phase, ShiftPhase::UpshiftCut);
            assert_eq!(rendered.gear, 4);
            assert_eq!(rendered.torque_sign, TorqueSign::Negative);
            assert!(rendered.rev_limiter_active);
            let ideal = from.rpm + (target.rpm - from.rpm) * (i + 1) as f32 / n as f32;
            let tolerance = if i + 1 == n { 1e-6 } else { 2.0 };
            assert!((rendered.rpm - ideal).abs() < tolerance);
            assert!(rendered.rpm + 1e-3 >= prev_rpm);
            max_step = max_step.max((rendered.rpm - prev_rpm).abs());
            prev_rpm = rendered.rpm;
        }
        let linear = (target.rpm - from.rpm).abs() / n as f32;
        assert!(max_step <= linear + 1e-3);
        let final_state = interp.rendered();
        assert!((final_state.rpm - target.rpm).abs() < 1e-6);
        assert!((final_state.clutch_engagement - target.clutch_engagement).abs() < 1e-6);
        assert!((final_state.tc_cut_ratio - target.tc_cut_ratio).abs() < 1e-6);
    }

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
        let sample_rate = 48_000usize;
        let mut follower = EnergyFollower::new(sample_rate as f32);
        follower.set_value(1.0);
        let target = target_acoustic_energy(EngineInput {
            rpm: 15_000.0,
            throttle: 0.0,
            load: 0.0,
        });
        for _ in 0..(sample_rate * 413 / 1_000) {
            follower.step(target);
        }
        let expected_after_one_tau = 0.32 + (1.0 - 0.32) / std::f32::consts::E;
        assert!((follower.value() - expected_after_one_tau).abs() < 2.0e-4);
    }

    #[test]
    fn shift_gesture_envelope_invariants_hold() {
        let sample_rate = 48_000.0;
        let mut gesture = ShiftGesture::default();
        assert_eq!(gesture.gain(), 1.0);

        // Cut lives in energy target, not gesture gain.
        gesture.step(ShiftPhase::UpshiftCut as u8, sample_rate, true);
        assert_eq!(gesture.gain(), 1.0);

        // Recovery opens wobble.
        gesture.step(ShiftPhase::UpshiftRecovery as u8, sample_rate, true);
        assert!(gesture.is_wobble_active());
        assert!(gesture.gain() < 0.75);

        // New cut cancels active gesture.
        gesture.step(ShiftPhase::UpshiftCut as u8, sample_rate, true);
        assert!(!gesture.is_wobble_active() && !gesture.is_blip_active());
        assert_eq!(gesture.gain(), 1.0);

        // Downshift blip flares when enabled and stays 1.0 when disabled.
        gesture.step(ShiftPhase::DownshiftBlip as u8, sample_rate, true);
        let first = gesture.gain();
        assert!(first > 1.35);
        for _ in 0..(SHIFT_BLIP_SECONDS * 2.0 * sample_rate) as usize {
            gesture.step(ShiftPhase::DownshiftBlip as u8, sample_rate, true);
        }
        assert!(!gesture.is_blip_active());
        assert_eq!(gesture.gain(), 1.0);

        let mut suppressed = ShiftGesture::default();
        suppressed.step(ShiftPhase::DownshiftBlip as u8, sample_rate, false);
        assert!(!suppressed.is_blip_active());
        assert_eq!(suppressed.gain(), 1.0);

        // Overlapping blip and recovery stays hard bounded.
        let mut overlap = ShiftGesture::default();
        overlap.step(ShiftPhase::DownshiftBlip as u8, sample_rate, true);
        for _ in 0..(0.05 * sample_rate) as usize {
            overlap.step(ShiftPhase::DownshiftBlip as u8, sample_rate, true);
        }
        overlap.step(ShiftPhase::DownshiftRecovery as u8, sample_rate, true);
        let mut max_gain = 0.0f32;
        for _ in 0..(0.3 * sample_rate) as usize {
            overlap.step(ShiftPhase::DownshiftRecovery as u8, sample_rate, true);
            max_gain = max_gain.max(overlap.gain());
        }
        assert!(max_gain <= SHIFT_GAIN_MAX + 1e-6);
    }
}
