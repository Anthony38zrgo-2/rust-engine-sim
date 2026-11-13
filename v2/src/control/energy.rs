//! Acoustic energy target mapping and asymmetric attack/release follower.

use crate::control::shift_gesture::shift_energy_gain;
use crate::control::types::{EngineInput, MechanicalState};

pub const COAST_IDLE_RPM: f32 = 1_000.0;
pub const COAST_MAX_RPM: f32 = 15_000.0;
pub const COAST_BASE_ENERGY: f32 = 0.12;
pub const COAST_RPM_ENERGY: f32 = 0.20;
pub const ENERGY_ATTACK_SECONDS: f32 = 0.018;
pub const ENERGY_RELEASE_SECONDS: f32 = 0.413;

/// Computes target acoustic energy using default mechanical state.
#[inline]
pub fn target_acoustic_energy(input: EngineInput) -> f32 {
    target_acoustic_energy_with_state(input, MechanicalState::default())
}

/// Maps primary engine controls, discrete/continuous mechanical state, and engine
/// RPM scaling bounds to a normalized `[0.0, 1.0]` combustion energy target.
#[inline]
pub fn target_acoustic_energy_with_range(
    input: EngineInput,
    mechanical: MechanicalState,
    idle_rpm: f32,
    max_rpm: f32,
) -> f32 {
    let span = (max_rpm - idle_rpm).max(1.0);
    let rpm_norm = ((input.rpm - idle_rpm) / span).clamp(0.0, 1.0);
    let coast_energy = COAST_BASE_ENERGY + COAST_RPM_ENERGY * rpm_norm;
    let powered_energy = 0.25 * input.throttle + 0.63 * input.load * (0.35 + 0.65 * input.throttle);
    let cut_gain = shift_energy_gain(mechanical.shift_phase);
    let limiter_gain = if mechanical.limiter_active { 0.78 } else { 1.0 };
    // Negative torque remains audible during overrun/coast while clutch is engaged.
    let retention_energy = (-mechanical.torque).max(0.0) * mechanical.clutch * 0.08;
    (coast_energy + powered_energy * cut_gain * limiter_gain + retention_energy).clamp(0.0, 1.0)
}

/// Maps primary engine controls and discrete/continuous mechanical state to a
/// normalized `[0.0, 1.0]` combustion energy target.
#[inline]
pub fn target_acoustic_energy_with_state(input: EngineInput, mechanical: MechanicalState) -> f32 {
    target_acoustic_energy_with_range(input, mechanical, COAST_IDLE_RPM, COAST_MAX_RPM)
}

/// Asymmetric one-pole envelope follower separating fast throttle attack (18 ms)
/// from slow lift-off decay (413 ms).
#[derive(Clone, Copy, Debug)]
pub struct EnergyFollower {
    attack_alpha: f32,
    release_alpha: f32,
    value: f32,
}

impl EnergyFollower {
    /// Constructs an energy follower initialized to zero with coefficients derived
    /// solely from direction and `sample_rate`.
    pub fn new(sample_rate: f32) -> Self {
        assert!(sample_rate.is_finite() && sample_rate > 0.0);
        Self {
            attack_alpha: 1.0 - (-1.0 / (ENERGY_ATTACK_SECONDS * sample_rate)).exp(),
            release_alpha: 1.0 - (-1.0 / (ENERGY_RELEASE_SECONDS * sample_rate)).exp(),
            value: 0.0,
        }
    }

    /// Advances the smoothed energy by one audio sample toward `target`.
    #[inline]
    pub fn step(&mut self, target: f32) -> f32 {
        let alpha = if target > self.value {
            self.attack_alpha
        } else {
            self.release_alpha
        };
        self.value += (target - self.value) * alpha;
        self.value
    }

    /// Returns the current smoothed energy level.
    #[inline]
    pub fn value(&self) -> f32 {
        self.value
    }

    /// Overrides the internal follower state (e.g., for deterministic test warm-start).
    #[inline]
    pub fn set_value(&mut self, value: f32) {
        self.value = value;
    }
}
