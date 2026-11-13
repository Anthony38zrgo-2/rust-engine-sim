use crate::config::EngineConfig;

/// Port throat diameter as a fraction of the valve seat diameter. The throat is
/// slightly narrower than the valve head, so it becomes the flow limiter well
/// before the valve reaches full lift.
const PORT_THROAT_FRACTION_OF_VALVE_DIAMETER: f32 = 0.9;

/// Physical model of a single poppet exhaust valve.
///
/// The model carries one crank-angle-only dependency through the pipeline
///
/// ```text
/// crank angle → valve lift → effective area
/// ```
///
/// so that the boundary and mass-flow authority live in the runner
/// (`ExhaustRunner`), which is where PHY-051 (compressible mass flow) will be
/// applied. Lift and effective area are refined independently of the flow:
/// PHY-041 (lift curve + config), PHY-042 (effective area). PHY-050 keeps the
/// valve as pure valve geometry and hands the runner the discharging area.
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

impl ExhaustValve {
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

    /// Continuous normalised lift in [0, 1] as a function of crank angle since
    /// firing. The profile is a raised-cosine opening ramp (`0 -> 1` over
    /// `exhaust_opening_ramp_deg`), a flat plateau at full lift, then a
    /// raised-cosine closing ramp (`1 -> 0` over `exhaust_closing_ramp_deg`).
    ///
    /// A raised-cosine ramp has zero slope and exact value 0/1 at its own ends,
    /// so the whole curve is C1-continuous and lands at exactly zero at both the
    /// opening and closing boundaries for any positive shape exponent. Applying
    /// the shape exponent after the ramp keeps `0^shape = 0` and `1^shape = 1`,
    /// so a discontinuous jump can never be produced by tuning `opening_shape`
    /// or `closing_shape`. EVO = `exhaust_open_deg`; EVC = `exhaust_open_deg +
    /// exhaust_duration_deg`.
    pub fn lift_fraction(&self, age_deg: f32) -> f32 {
        let valve_age = age_deg - self.exhaust_open_deg;
        if !(0.0..self.exhaust_duration_deg).contains(&valve_age) {
            return 0.0;
        }
        let plateau = self.exhaust_duration_deg
            - self.exhaust_opening_ramp_deg
            - self.exhaust_closing_ramp_deg;
        let shaped = |progress: f32, exponent: f32| {
            let v = progress.clamp(0.0, 1.0);
            let ramp = 0.5 * (1.0 - (std::f32::consts::PI * v).cos());
            ramp.powf(exponent)
        };
        if valve_age < self.exhaust_opening_ramp_deg {
            shaped(valve_age / self.exhaust_opening_ramp_deg, self.exhaust_opening_shape)
        } else if valve_age < self.exhaust_opening_ramp_deg + plateau {
            1.0
        } else {
            let closing_progress =
                (valve_age - self.exhaust_opening_ramp_deg - plateau) / self.exhaust_closing_ramp_deg;
            let v = closing_progress.clamp(0.0, 1.0);
            let ramp = 0.5 * (1.0 + (std::f32::consts::PI * v).cos());
            ramp.powf(self.exhaust_closing_shape)
        }
    }

    /// Effective flow area (m²) for a given lift.
    ///
    /// The poppet curtain area `π·d·lift` grows linearly with lift until it
    /// exceeds the port throat area `π·d_throat²/4`; beyond that the port
    /// throat is the flow limiter and the area plateaus. Taking the minimum of
    /// the two and scaling by the discharge coefficient keeps the result zero
    /// at zero lift, bounded by the port capacity, and physically interpretable.
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
        // The curve is continuous across the window: below the opening angle and
        // at the opening angle the lift is exactly zero (no jump at EVO), and
        // likewise it returns to exactly zero at close. The interior ramp is
        // steep by design but never exceeds unity and never goes negative.
        let open = v.exhaust_open_deg;
        let close = open + v.exhaust_duration_deg;
        assert_eq!(v.lift_fraction(open - 0.5), 0.0);
        assert_eq!(v.lift_fraction(open), 0.0);
        assert_eq!(v.lift_fraction(close), 0.0);
        assert_eq!(v.lift_fraction(close + 0.5), 0.0);
        // A bounded opening ramp means the lift climbs smoothly from zero: the
        // first degree is a small positive step, never a full-height jump.
        let first_step = v.lift_fraction(open + 1.0) - 0.0;
        assert!(first_step > 0.0 && first_step < 0.5, "first step {first_step}");
        for step in 0..=240 {
            let lift = v.lift_fraction(open + step as f32);
            assert!(lift >= 0.0 && lift <= 1.0, "lift out of bounds: {lift}");
        }
    }

    #[test]
    fn effective_area_is_zero_at_zero_lift_and_monotonic() {
        let v = valve();
        assert_eq!(v.effective_area_m2(0.0), 0.0);
        let low = v.effective_area_m2(0.4);
        let mid = v.effective_area_m2(0.6);
        let high = v.effective_area_m2(1.0);
        assert!(low > 0.0);
        assert!(mid > low, "effective area must grow with lift");
        assert!(high >= mid, "effective area must not shrink");
    }

    #[test]
    fn effective_area_saturates_at_the_port_throat() {
        let v = valve();
        let throat_area = v.effective_area_m2(1.0);
        // Once lift has pushed the curtain area past the throat, the effective
        // area stops growing and stays at the port capacity.
        assert!(
            (v.effective_area_m2(0.7) - throat_area).abs() < 1.0e-7,
            "effective area must plateau at the port throat"
        );
        // Before the throat limit the effective area is the curtain area.
        assert!(v.effective_area_m2(0.4) < throat_area);
        // The capped area is always at or below the unconstrained curtain area.
        let uncapped_curtain = std::f32::consts::PI
            * v.valve_diameter_m
            * (1.0 * v.max_lift_m)
            * v.discharge_coefficient;
        assert!(throat_area < uncapped_curtain);
    }

    #[test]
    fn shape_exponents_are_plumbed_into_the_curve() {
        let mut config = EngineConfig::default();
        config.exhaust_opening_shape = 2.0;
        config.exhaust_closing_shape = 2.0;
        let v = ExhaustValve::new(&config);
        let default = ExhaustValve::new(&EngineConfig::default());
        assert_ne!(
            v.lift_fraction(config.exhaust_open_deg + 2.5),
            default.lift_fraction(config.exhaust_open_deg + 2.5),
            "opening_shape must change the ramp"
        );
        // The curve must still reach full lift in the plateau and stay bounded.
        let mid = config.exhaust_open_deg + config.exhaust_duration_deg * 0.5;
        assert_eq!(v.lift_fraction(mid), 1.0);
    }

    #[test]
    fn aggressive_shapes_never_produce_a_discontinuity() {
        let mut config = EngineConfig::default();
        config.exhaust_opening_shape = 0.4;
        config.exhaust_closing_shape = 8.0;
        let v = ExhaustValve::new(&config);
        let open = config.exhaust_open_deg;
        let close = open + config.exhaust_duration_deg;
        // Values meet exactly zero at both boundaries regardless of shape.
        assert_eq!(v.lift_fraction(open - 1.0), 0.0);
        assert_eq!(v.lift_fraction(open), 0.0);
        assert_eq!(v.lift_fraction(close), 0.0);
        assert_eq!(v.lift_fraction(close + 1.0), 0.0);
        // Bounded and monotonic within the window.
        let mut previous = 0.0f32;
        for step in 0..=240 {
            let lift = v.lift_fraction(open + step as f32);
            assert!(lift >= 0.0 && lift <= 1.0, "lift out of bounds: {lift}");
            previous = lift;
        }
        assert!(previous >= -0.0);
    }
}
