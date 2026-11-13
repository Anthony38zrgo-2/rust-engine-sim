//! Core control-plane value types and validation for telemetry and engine inputs.

/// Instantaneous primary operating input to the V10 synthesizer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineInput {
    pub rpm: f32,
    pub throttle: f32,
    pub load: f32,
}

impl EngineInput {
    /// Validates finite control bounds (`rpm` 0..=25_000, `throttle` and `load` 0..=1).
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

/// Sign of delivered engine torque from vehicle physics.
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum TorqueSign {
    Negative = -1,
    #[default]
    Neutral = 0,
    Positive = 1,
}

impl TorqueSign {
    /// Converts a wire integer (`-1`, `0`, `1`) into `TorqueSign`.
    pub fn from_i32(value: i32) -> Result<Self, String> {
        match value {
            -1 => Ok(Self::Negative),
            0 => Ok(Self::Neutral),
            1 => Ok(Self::Positive),
            _ => Err(format!("invalid torque sign: {value}")),
        }
    }
}

/// Discrete mechanical gearbox/shift phase signaled by drivetrain physics.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ShiftPhase {
    #[default]
    None = 0,
    UpshiftCut = 1,
    UpshiftRecovery = 2,
    DownshiftCut = 3,
    DownshiftBlip = 4,
    DownshiftRecovery = 5,
}

impl ShiftPhase {
    /// Converts a wire integer (`0..=5`) into `ShiftPhase`.
    pub fn from_i32(value: i32) -> Result<Self, String> {
        match value {
            0 => Ok(Self::None),
            1 => Ok(Self::UpshiftCut),
            2 => Ok(Self::UpshiftRecovery),
            3 => Ok(Self::DownshiftCut),
            4 => Ok(Self::DownshiftBlip),
            5 => Ok(Self::DownshiftRecovery),
            _ => Err(format!("invalid shift phase: {value}")),
        }
    }
}

/// Complete telemetry packet passed at control rate to `Gf509Runtime`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RuntimeTelemetry {
    pub rpm: f32,
    pub throttle: f32,
    pub normalized_engine_load: f32,
    pub normalized_engine_torque: f32,
    pub torque_sign: TorqueSign,
    pub rpm_derivative: f32,
    pub throttle_derivative: f32,
    pub gear: i8,
    pub shift_phase: ShiftPhase,
    /// Clutch engagement from the physics powertrain, continuously smoothed.
    pub clutch_engagement: f32,
    /// Physical traction-control cut. The load packet already includes this
    /// cut; the synth consumes it only for a dedicated, non-gain modulation.
    pub tc_cut_ratio: f32,
    /// Discrete physical rev limiter state; changed sample-accurately and never
    /// interpolated with a continuous control.
    pub rev_limiter_active: bool,
    pub dt_seconds: f32,
}

impl RuntimeTelemetry {
    /// Validates all fields against supported operating bounds.
    pub fn validate(self) -> Result<Self, String> {
        if !self.rpm.is_finite() || !(0.0..=25_000.0).contains(&self.rpm) {
            return Err(format!("rpm out of range: {}", self.rpm));
        }
        if !self.throttle.is_finite() || !(0.0..=1.0).contains(&self.throttle) {
            return Err(format!("throttle out of range: {}", self.throttle));
        }
        if !self.normalized_engine_load.is_finite()
            || !(0.0..=1.0).contains(&self.normalized_engine_load)
        {
            return Err(format!(
                "normalized_engine_load out of range: {}",
                self.normalized_engine_load
            ));
        }
        if !self.normalized_engine_torque.is_finite()
            || !(-1.0..=1.0).contains(&self.normalized_engine_torque)
        {
            return Err(format!(
                "normalized_engine_torque out of range: {}",
                self.normalized_engine_torque
            ));
        }
        if !self.clutch_engagement.is_finite() || !(0.0..=1.0).contains(&self.clutch_engagement) {
            return Err(format!(
                "clutch_engagement out of range: {}",
                self.clutch_engagement
            ));
        }
        if !self.tc_cut_ratio.is_finite() || !(0.0..=1.0).contains(&self.tc_cut_ratio) {
            return Err(format!("tc_cut_ratio out of range: {}", self.tc_cut_ratio));
        }
        if !self.rpm_derivative.is_finite() {
            return Err(format!(
                "rpm_derivative not finite: {}",
                self.rpm_derivative
            ));
        }
        if !self.throttle_derivative.is_finite() {
            return Err(format!(
                "throttle_derivative not finite: {}",
                self.throttle_derivative
            ));
        }
        if !(-1..=12).contains(&self.gear) {
            return Err(format!("gear out of range: {}", self.gear));
        }
        if !self.dt_seconds.is_finite() || !(0.0..=1.0).contains(&self.dt_seconds) {
            return Err(format!("dt_seconds out of range: {}", self.dt_seconds));
        }
        Ok(self)
    }
}

impl Default for RuntimeTelemetry {
    fn default() -> Self {
        Self {
            rpm: 0.0,
            throttle: 0.0,
            normalized_engine_load: 0.0,
            normalized_engine_torque: 0.0,
            torque_sign: TorqueSign::Neutral,
            rpm_derivative: 0.0,
            throttle_derivative: 0.0,
            gear: 0,
            shift_phase: ShiftPhase::None,
            clutch_engagement: 0.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds: 0.0,
        }
    }
}

/// Sanitized mechanical state applied inside per-sample synthesis.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MechanicalState {
    /// Signed engine torque normalized to the physical maximum.
    pub torque: f32,
    pub clutch: f32,
    /// Physical TC cut retained for acoustic turbulence texture only.
    pub tc_cut: f32,
    pub limiter_active: bool,
    pub shift_phase: u8,
}

impl MechanicalState {
    /// Clamps normalized mechanical inputs while preserving discrete flags and shift phase.
    pub fn from_controls(
        torque: f32,
        clutch: f32,
        tc_cut: f32,
        limiter_active: bool,
        shift_phase: ShiftPhase,
    ) -> Self {
        Self {
            torque: torque.clamp(-1.0, 1.0),
            clutch: clutch.clamp(0.0, 1.0),
            tc_cut: tc_cut.clamp(0.0, 1.0),
            limiter_active,
            shift_phase: shift_phase as u8,
        }
    }

    /// Extracts mechanical synthesis state from validated runtime telemetry.
    pub fn from_telemetry(telemetry: RuntimeTelemetry) -> Self {
        Self::from_controls(
            telemetry.normalized_engine_torque,
            telemetry.clutch_engagement,
            telemetry.tc_cut_ratio,
            telemetry.rev_limiter_active,
            telemetry.shift_phase,
        )
    }
}
