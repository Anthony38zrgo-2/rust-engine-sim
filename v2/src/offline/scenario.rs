//! Offline sweep, lift-coast, and shift-sequence trajectory helpers.

use crate::control::{EngineInput, RuntimeTelemetry, ShiftPhase, TorqueSign};

/// Audition timings for `--shift-sequence`: cut, optional downshift blip, then recovery.
const SHIFT_CUT_SECONDS: f32 = 0.250;
const SHIFT_BLIP_SECONDS: f32 = 0.120;
const SHIFT_RECOVERY_SECONDS: f32 = 0.040;

/// Discrete gear shift direction parsed from `--shift-sequence`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShiftDirection {
    Up,
    Down,
}

/// Timed shift trigger `(time_s, direction)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShiftEvent {
    pub time_s: f32,
    pub direction: ShiftDirection,
}

/// Time-varying offline audition profile configuration.
#[derive(Clone, Debug, PartialEq)]
pub struct OfflineScenarioConfig {
    pub rpm: f32,
    pub seconds: f32,
    pub warmup: f32,
    pub throttle: f32,
    pub load: f32,
    pub sweep_end_rpm: Option<f32>,
    pub hold_before_lift: bool,
    pub accel_seconds: f32,
    pub coast_end_rpm: f32,
    pub shift_sequence: Vec<ShiftEvent>,
}

impl Default for OfflineScenarioConfig {
    fn default() -> Self {
        Self {
            rpm: 5_000.0,
            seconds: 6.0,
            warmup: 1.0,
            throttle: 0.72,
            load: 0.78,
            sweep_end_rpm: None,
            hold_before_lift: false,
            accel_seconds: 7.0,
            coast_end_rpm: 6_500.0,
            shift_sequence: Vec::new(),
        }
    }
}

impl OfflineScenarioConfig {
    /// Validates render durations, RPM/throttle/load bounds, and shift-event ordering.
    pub fn validate(&self) -> Result<(), String> {
        if !self.seconds.is_finite() || !(0.1..=120.0).contains(&self.seconds) {
            return Err("--seconds must be in 0.1..120".into());
        }
        if !self.warmup.is_finite() || !(0.0..=10.0).contains(&self.warmup) {
            return Err("--warmup must be in 0..10".into());
        }
        if !self.accel_seconds.is_finite()
            || self.accel_seconds <= 0.0
            || self.accel_seconds >= self.seconds
        {
            return Err("--accel-seconds must be inside the render duration".into());
        }
        EngineInput {
            rpm: self.rpm,
            throttle: self.throttle,
            load: self.load,
        }
        .validate()?;
        if let Some(end_rpm) = self.sweep_end_rpm {
            EngineInput {
                rpm: end_rpm,
                throttle: self.throttle,
                load: self.load,
            }
            .validate()?;
            EngineInput {
                rpm: self.coast_end_rpm,
                throttle: 0.0,
                load: 0.0,
            }
            .validate()?;
        }
        if self.hold_before_lift && self.sweep_end_rpm.is_none() {
            return Err("--hold-before-lift requires --sweep-end-rpm".into());
        }
        validate_shift_sequence(&self.shift_sequence, self.seconds)?;
        Ok(())
    }

    /// Initial steady input used during warmup before time `t = 0`.
    pub fn initial_input(&self) -> EngineInput {
        EngineInput {
            rpm: self.rpm,
            throttle: self.throttle,
            load: self.load,
        }
    }

    /// Instantaneous prescribed `(rpm, throttle, load)` at `time_s`.
    pub fn input_at(&self, time_s: f32) -> EngineInput {
        let Some(end_rpm) = self.sweep_end_rpm else {
            return self.initial_input();
        };
        if self.hold_before_lift {
            let coast_end_rpm = end_rpm;
            if time_s < self.accel_seconds {
                return self.initial_input();
            }
            let coast_duration = (self.seconds - self.accel_seconds).max(1.0e-6);
            let t = ((time_s - self.accel_seconds) / coast_duration).clamp(0.0, 1.0);
            let rpm_decay = 1.0 - (1.0 - t).powf(1.55);
            let lift = (-t / 0.035).exp();
            return EngineInput {
                rpm: self.rpm + (coast_end_rpm - self.rpm) * rpm_decay,
                throttle: 0.035 + (self.throttle - 0.035) * lift,
                load: 0.10 + (self.load - 0.10) * (-t / 0.12).exp(),
            };
        }
        if time_s < self.accel_seconds {
            let t = (time_s / self.accel_seconds).clamp(0.0, 1.0);
            // Smoothstep avoids an acceleration discontinuity at either endpoint.
            let shaped = t * t * (3.0 - 2.0 * t);
            EngineInput {
                rpm: self.rpm + (end_rpm - self.rpm) * shaped,
                throttle: self.throttle,
                load: self.load,
            }
        } else {
            let coast_duration = (self.seconds - self.accel_seconds).max(1.0e-6);
            let t = ((time_s - self.accel_seconds) / coast_duration).clamp(0.0, 1.0);
            let rpm_decay = 1.0 - (1.0 - t).powf(1.55);
            let lift = (-t / 0.035).exp();
            EngineInput {
                rpm: end_rpm + (self.coast_end_rpm - end_rpm) * rpm_decay,
                throttle: 0.035 + (self.throttle - 0.035) * lift,
                load: 0.10 + (self.load - 0.10) * (-t / 0.12).exp(),
            }
        }
    }

    /// Complete prescribed per-sample runtime telemetry matching `v1` `v10_render` curves.
    pub fn telemetry_at(&self, time_s: f32, gear: i8, dt_seconds: f32) -> RuntimeTelemetry {
        let input = self.input_at(time_s);
        let retention_torque = if input.throttle > 0.30 {
            input.load
        } else {
            -((0.30 - input.throttle) / 0.30) * 0.6
        };
        let torque_sign = if retention_torque > 0.0 {
            TorqueSign::Positive
        } else if retention_torque < 0.0 {
            TorqueSign::Negative
        } else {
            TorqueSign::Neutral
        };
        RuntimeTelemetry {
            rpm: input.rpm,
            throttle: input.throttle,
            normalized_engine_load: input.load,
            normalized_engine_torque: retention_torque,
            torque_sign,
            rpm_derivative: 0.0,
            throttle_derivative: 0.0,
            gear,
            shift_phase: shift_phase_at(&self.shift_sequence, time_s),
            clutch_engagement: 1.0,
            tc_cut_ratio: 0.0,
            rev_limiter_active: false,
            dt_seconds,
        }
    }

    /// Profile identifier written to render metadata JSON.
    pub fn profile_label(&self) -> &'static str {
        if self.hold_before_lift {
            "lift_coast_hold"
        } else if self.sweep_end_rpm.is_some() {
            "sweep_lift_coast"
        } else {
            "steady"
        }
    }

    /// Peak RPM reported in render metadata JSON.
    pub fn peak_rpm(&self) -> f32 {
        if self.hold_before_lift {
            self.rpm.max(self.sweep_end_rpm.unwrap_or(self.rpm))
        } else {
            self.sweep_end_rpm.unwrap_or(self.rpm)
        }
    }

    /// Final coast RPM reported in render metadata JSON.
    pub fn metadata_coast_end_rpm(&self) -> f32 {
        if self.hold_before_lift {
            self.sweep_end_rpm.unwrap_or(self.coast_end_rpm)
        } else {
            self.coast_end_rpm
        }
    }
}

/// Parses `--shift-sequence "u@2.5,d@7.2"`; events must be strictly ascending.
pub fn parse_shift_sequence(spec: &str) -> Result<Vec<ShiftEvent>, String> {
    let mut events: Vec<ShiftEvent> = Vec::new();
    for part in spec
        .split(',')
        .map(str::trim)
        .filter(|part| !part.is_empty())
    {
        let (token, time) = part
            .split_once('@')
            .ok_or_else(|| format!("expected <u|d>@<seconds>: {part}"))?;
        let direction = match token.trim() {
            "u" => ShiftDirection::Up,
            "d" => ShiftDirection::Down,
            other => return Err(format!("unknown shift direction: {other}")),
        };
        let time_s: f32 = time
            .trim()
            .parse()
            .map_err(|_| format!("invalid shift time: {part}"))?;
        if !time_s.is_finite() || time_s < 0.0 {
            return Err(format!("shift time out of range: {part}"));
        }
        if events.last().is_some_and(|last| time_s <= last.time_s) {
            return Err(format!("shift times must be strictly ascending: {part}"));
        }
        events.push(ShiftEvent { time_s, direction });
    }
    Ok(events)
}

/// Ensures every scheduled shift starts strictly before `seconds`.
pub fn validate_shift_sequence(events: &[ShiftEvent], seconds: f32) -> Result<(), String> {
    if let Some(last) = events.last() {
        if last.time_s >= seconds {
            return Err("--shift-sequence events must start within --seconds".into());
        }
    }
    Ok(())
}

/// Shift phase for the latest event started at or before `time_s`.
pub fn shift_phase_at(events: &[ShiftEvent], time_s: f32) -> ShiftPhase {
    let Some(event) = events.iter().rev().find(|event| time_s >= event.time_s) else {
        return ShiftPhase::None;
    };
    let elapsed = time_s - event.time_s;
    if elapsed < SHIFT_CUT_SECONDS {
        return match event.direction {
            ShiftDirection::Up => ShiftPhase::UpshiftCut,
            ShiftDirection::Down => ShiftPhase::DownshiftCut,
        };
    }
    let blip_end = SHIFT_CUT_SECONDS
        + if event.direction == ShiftDirection::Down {
            SHIFT_BLIP_SECONDS
        } else {
            0.0
        };
    if elapsed < blip_end {
        return ShiftPhase::DownshiftBlip;
    }
    if elapsed < blip_end + SHIFT_RECOVERY_SECONDS {
        return match event.direction {
            ShiftDirection::Up => ShiftPhase::UpshiftRecovery,
            ShiftDirection::Down => ShiftPhase::DownshiftRecovery,
        };
    }
    ShiftPhase::None
}

/// Formats shift sequence events for metadata JSON (`"none"` when empty).
pub fn shift_sequence_label(events: &[ShiftEvent]) -> String {
    if events.is_empty() {
        return "none".into();
    }
    events
        .iter()
        .map(|event| {
            let token = match event.direction {
                ShiftDirection::Up => "u",
                ShiftDirection::Down => "d",
            };
            format!("{token}@{:.3}", event.time_s)
        })
        .collect::<Vec<_>>()
        .join(",")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_shift_sequence_events() {
        let events = parse_shift_sequence("u@2.5, d@7.2").unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[0],
            ShiftEvent {
                time_s: 2.5,
                direction: ShiftDirection::Up,
            }
        );
        assert_eq!(
            events[1],
            ShiftEvent {
                time_s: 7.2,
                direction: ShiftDirection::Down,
            }
        );
    }

    #[test]
    fn rejects_malformed_shift_sequence() {
        assert!(parse_shift_sequence("u2.5").is_err());
        assert!(parse_shift_sequence("x@1.0").is_err());
        assert!(parse_shift_sequence("u@nope").is_err());
        assert!(parse_shift_sequence("u@-1.0").is_err());
        assert!(parse_shift_sequence("u@3.0,u@2.0").is_err());
        assert!(parse_shift_sequence("u@2.0,d@2.0").is_err());
    }

    #[test]
    fn shift_events_must_fit_inside_the_render_duration() {
        let events = parse_shift_sequence("u@2.5").unwrap();
        assert!(validate_shift_sequence(&events, 6.0).is_ok());
        assert!(validate_shift_sequence(&events, 2.5).is_err());
        assert!(validate_shift_sequence(&[], 2.5).is_ok());
    }

    #[test]
    fn phase_schedule_matches_cut_blip_recovery() {
        let events = parse_shift_sequence("u@1.0,d@5.0").unwrap();
        assert_eq!(shift_phase_at(&events, 0.5), ShiftPhase::None);
        assert_eq!(shift_phase_at(&events, 1.0), ShiftPhase::UpshiftCut);
        assert_eq!(shift_phase_at(&events, 1.24), ShiftPhase::UpshiftCut);
        assert_eq!(shift_phase_at(&events, 1.26), ShiftPhase::UpshiftRecovery);
        assert_eq!(shift_phase_at(&events, 1.30), ShiftPhase::None);
        assert_eq!(shift_phase_at(&events, 5.1), ShiftPhase::DownshiftCut);
        assert_eq!(shift_phase_at(&events, 5.24), ShiftPhase::DownshiftCut);
        assert_eq!(shift_phase_at(&events, 5.26), ShiftPhase::DownshiftBlip);
        assert_eq!(shift_phase_at(&events, 5.39), ShiftPhase::DownshiftRecovery);
        assert_eq!(shift_phase_at(&events, 5.50), ShiftPhase::None);
    }

    #[test]
    fn shift_sequence_label_round_trips() {
        assert_eq!(shift_sequence_label(&[]), "none");
        let events = parse_shift_sequence("u@2.5,d@7.25").unwrap();
        assert_eq!(shift_sequence_label(&events), "u@2.500,d@7.250");
    }
}
