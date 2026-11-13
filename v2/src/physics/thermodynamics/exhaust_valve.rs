//! Single poppet exhaust-valve lift profile and effective curtain/throat area.

use crate::config::EngineConfig;

/// Port throat diameter as a fraction of the valve seat diameter.
const PORT_THROAT_FRACTION_OF_VALVE_DIAMETER: f32 = 0.9;

/// Physical model of a single poppet exhaust valve (`crank angle -> lift -> effective area`).
#[derive(Clone, Copy, Debug)]
pub struct ExhaustValve {
    valve_diameter_m: f32,
    max_lift_m: f32,
    discharge_coefficient: f32,
    exhaust_open_deg: f32,
    exhaust_opening_ramp_deg: f32,
    exhaust_duration_deg: f32,
    exhaust_closing_ramp_deg: f32,
    exhaust_opening_shape: f32,
    exhaust_closing_shape: f32,
}

#[inline]
fn shaped_cosine_ramp(progress: f32, cos_sign: f32, exponent: f32) -> f32 {
    let v = progress.clamp(0.0, 1.0);
    let ramp = 0.5 * (1.0 + cos_sign * (std::f32::consts::PI * v).cos());
    ramp.powf(exponent)
}

impl ExhaustValve {
    /// Builds exhaust valve geometry and timing from [`EngineConfig`].
    pub fn new(config: &EngineConfig) -> Self {
        Self {
            valve_diameter_m: config.exhaust_valve_diameter_m,
            max_lift_m: config.exhaust_max_lift_m,
            discharge_coefficient: config.exhaust_discharge_coefficient,
            exhaust_open_deg: config.exhaust_open_deg,
            exhaust_opening_ramp_deg: config.exhaust_opening_ramp_deg,
            exhaust_duration_deg: config.exhaust_duration_deg,
            exhaust_closing_ramp_deg: config.exhaust_closing_ramp_deg,
            exhaust_opening_shape: config.exhaust_opening_shape,
            exhaust_closing_shape: config.exhaust_closing_shape,
        }
    }

    /// Continuous normalised lift in `[0, 1]` as a function of crank angle since firing.
    #[inline]
    pub fn lift_fraction(&self, age_deg: f32) -> f32 {
        let valve_age = age_deg - self.exhaust_open_deg;
        if !(0.0..self.exhaust_duration_deg).contains(&valve_age) {
            return 0.0;
        }
        let plateau = self.exhaust_duration_deg
            - self.exhaust_opening_ramp_deg
            - self.exhaust_closing_ramp_deg;
        if valve_age < self.exhaust_opening_ramp_deg {
            shaped_cosine_ramp(
                valve_age / self.exhaust_opening_ramp_deg,
                -1.0,
                self.exhaust_opening_shape,
            )
        } else if valve_age < self.exhaust_opening_ramp_deg + plateau {
            1.0
        } else {
            let closing_progress = (valve_age - self.exhaust_opening_ramp_deg - plateau)
                / self.exhaust_closing_ramp_deg;
            shaped_cosine_ramp(closing_progress, 1.0, self.exhaust_closing_shape)
        }
    }

    /// Effective discharge flow area (`m²`) capped by port throat capacity.
    #[inline]
    pub fn effective_area_m2(&self, lift_fraction: f32) -> f32 {
        let lift_m = lift_fraction.clamp(0.0, 1.0) * self.max_lift_m;
        let curtain_area = std::f32::consts::PI * self.valve_diameter_m * lift_m;
        let throat_diameter_m = self.valve_diameter_m * PORT_THROAT_FRACTION_OF_VALVE_DIAMETER;
        let throat_area = std::f32::consts::PI * 0.25 * throat_diameter_m * throat_diameter_m;
        curtain_area.min(throat_area) * self.discharge_coefficient
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valve() -> ExhaustValve {
        ExhaustValve::new(&EngineConfig::default())
    }

    #[test]
    fn lift_is_zero_before_open_and_after_close() {
        let v = valve();
        assert_eq!(v.lift_fraction(v.exhaust_open_deg - 1.0), 0.0);
        assert_eq!(v.lift_fraction(v.exhaust_open_deg), 0.0);
        assert_eq!(
            v.lift_fraction(v.exhaust_open_deg + v.exhaust_duration_deg),
            0.0
        );
        assert_eq!(
            v.lift_fraction(v.exhaust_open_deg + v.exhaust_duration_deg + 1.0),
            0.0
        );
    }

    #[test]
    fn lift_is_continuous_at_open_and_close_and_bounded() {
        let v = valve();
        let open = v.exhaust_open_deg;
        let close = open + v.exhaust_duration_deg;
        assert_eq!(v.lift_fraction(open - 0.5), 0.0);
        assert_eq!(v.lift_fraction(open), 0.0);
        assert_eq!(v.lift_fraction(close), 0.0);
        assert_eq!(v.lift_fraction(close + 0.5), 0.0);
        let first_step = v.lift_fraction(open + 1.0);
        assert!(first_step > 0.0 && first_step < 0.5);
        for step in 0..=240 {
            let lift = v.lift_fraction(open + step as f32);
            assert!((0.0..=1.0).contains(&lift), "lift out of bounds: {lift}");
        }
    }

    #[test]
    fn effective_area_is_zero_at_zero_lift_and_monotonic() {
        let v = valve();
        assert_eq!(v.effective_area_m2(0.0), 0.0);
        let low = v.effective_area_m2(0.4);
        let mid = v.effective_area_m2(0.6);
        let high = v.effective_area_m2(1.0);
        assert!(low > 0.0 && mid > low && high >= mid);
    }

    #[test]
    fn effective_area_saturates_at_the_port_throat() {
        let v = valve();
        let throat_area = v.effective_area_m2(1.0);
        assert!((v.effective_area_m2(0.7) - throat_area).abs() < 1.0e-7);
        assert!(v.effective_area_m2(0.4) < throat_area);
        let uncapped_curtain =
            std::f32::consts::PI * v.valve_diameter_m * v.max_lift_m * v.discharge_coefficient;
        assert!(throat_area < uncapped_curtain);
    }

    #[test]
    fn shape_exponents_are_plumbed_into_the_curve() {
        let config = EngineConfig {
            exhaust_opening_shape: 2.0,
            exhaust_closing_shape: 2.0,
            ..EngineConfig::default()
        };
        let v = ExhaustValve::new(&config);
        let default = ExhaustValve::new(&EngineConfig::default());
        assert_ne!(
            v.lift_fraction(config.exhaust_open_deg + 2.5),
            default.lift_fraction(config.exhaust_open_deg + 2.5),
        );
        let mid = config.exhaust_open_deg + config.exhaust_duration_deg * 0.5;
        assert_eq!(v.lift_fraction(mid), 1.0);
    }

    #[test]
    fn aggressive_shapes_never_produce_a_discontinuity() {
        let config = EngineConfig {
            exhaust_opening_shape: 0.4,
            exhaust_closing_shape: 8.0,
            ..EngineConfig::default()
        };
        let v = ExhaustValve::new(&config);
        let open = config.exhaust_open_deg;
        let close = open + config.exhaust_duration_deg;
        assert_eq!(v.lift_fraction(open - 1.0), 0.0);
        assert_eq!(v.lift_fraction(open), 0.0);
        assert_eq!(v.lift_fraction(close), 0.0);
        assert_eq!(v.lift_fraction(close + 1.0), 0.0);
        for step in 0..=240 {
            let lift = v.lift_fraction(open + step as f32);
            assert!((0.0..=1.0).contains(&lift), "lift out of bounds: {lift}");
        }
    }
}
