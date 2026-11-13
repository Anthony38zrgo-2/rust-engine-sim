//! Allocation-free shift recovery wobble and downshift blip envelope generator.

/// Shift gesture: damped amplitude wobble synthesized on recovery. Applied
/// after the energy follower so asymmetric attack/release smoothing cannot
/// rectify the oscillation. 10 Hz over 175 ms (1.75 cycles ending at unity),
/// sag to 0.70 with positive rebound capped at 1.15.
pub const SHIFT_WOBBLE_DEPTH: f32 = 0.30;
pub const SHIFT_WOBBLE_SECONDS: f32 = 0.175;
pub const SHIFT_WOBBLE_TAU: f32 = 0.070;
pub const SHIFT_WOBBLE_CYCLES: f32 = 1.75;
pub const SHIFT_WOBBLE_MIN: f32 = 0.60;
pub const SHIFT_WOBBLE_MAX: f32 = 1.15;

/// Downshift blip: short energy flare over the `DownshiftBlip` phase.
pub const SHIFT_BLIP_SECONDS: f32 = 0.120;
pub const SHIFT_BLIP_GAIN: f32 = 0.40;
pub const SHIFT_GAIN_MAX: f32 = 1.45;

/// Discrete energy multiplier carried by the physical shift phases. Mirrors the
/// mapping inside `target_acoustic_energy_with_state` so procedural and sample-only
/// runtimes share the same cut/recovery attenuation curve.
#[inline]
pub fn shift_energy_gain(phase: u8) -> f32 {
    match phase {
        1 | 3 => 0.12,
        2 | 5 => 0.72,
        _ => 1.0,
    }
}

/// Per-sample state for the synthesized shift gesture (wobble + blip).
/// Driven by discrete mechanical `shift_phase`; phase edges start an envelope
/// that continues running after the phase returns to `None`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ShiftGesture {
    last_phase: u8,
    wobble_active: bool,
    wobble_t: f32,
    blip_active: bool,
    blip_t: f32,
}

impl ShiftGesture {
    /// Advances rising-edge detection and the envelope timers by one sample.
    #[inline]
    pub fn step(&mut self, phase: u8, sample_rate: f32, blip_enabled: bool) {
        let is_cut = phase == 1 || phase == 3;
        // A new cut restarts the gesture: previous wobble/blip envelopes end cleanly.
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

    /// Bounded amplitude multiplier for the current gesture state (`1.0` when idle).
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

    /// Reports whether the recovery wobble timer is currently running.
    #[inline]
    pub fn is_wobble_active(&self) -> bool {
        self.wobble_active
    }

    /// Reports whether the downshift blip timer is currently running.
    #[inline]
    pub fn is_blip_active(&self) -> bool {
        self.blip_active
    }
}
