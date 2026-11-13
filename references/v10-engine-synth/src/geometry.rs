//! Analytic slider-crank geometry and instantaneous cylinder volume.
//!
//! Pure deterministic functions only: no allocation and no per-cycle state.
//! Crank angle is measured in degrees, `0°` applying at the compression TDC.

use crate::config::EngineConfig;

/// Number of intervals in the 720° volume phase table.
///
/// The table is per configured slider-crank and stores 1025 `f32` values. Ten
/// cylinders therefore reserve about 40 KiB, paid once at construction, while
/// the audio path replaces two transcendental functions and a square root with
/// a bounded linear interpolation.
pub const VOLUME_TABLE_SAMPLES: usize = 1_024;

#[derive(Clone, Copy, Debug)]
pub struct SliderCrank {
    bore_m: f32,
    crank_radius_m: f32,
    rod_length_m: f32,
    clearance_volume_m3: f32,
    swept_volume_m3: f32,
    area_m2: f32,
    volume_table_m3: [f32; VOLUME_TABLE_SAMPLES + 1],
}

impl SliderCrank {
    /// Build a slider-crank from base geometry. The caller is expected to have
    /// validated the config; values that violate the slider-crank constraint
    /// (`rod_length >= crank_radius`) are clamped to a non-degenerate geometry.
    pub fn new(bore_m: f32, stroke_m: f32, rod_length_m: f32, compression_ratio: f32) -> Self {
        let crank_radius_m = stroke_m * 0.5;
        let rod_length_m = rod_length_m.max(crank_radius_m);
        let bore = bore_m as f64;
        let area_m2 = std::f64::consts::PI * 0.25 * bore * bore;
        let swept_volume_m3 = (area_m2 * stroke_m as f64) as f32;
        let clearance_volume_m3 = swept_volume_m3 / (compression_ratio - 1.0);
        let mut geometry = Self {
            bore_m,
            crank_radius_m,
            rod_length_m,
            clearance_volume_m3,
            swept_volume_m3,
            area_m2: area_m2 as f32,
            volume_table_m3: [0.0; VOLUME_TABLE_SAMPLES + 1],
        };
        for index in 0..=VOLUME_TABLE_SAMPLES {
            let angle_deg = index as f32 * (720.0 / VOLUME_TABLE_SAMPLES as f32);
            let value = geometry.instantaneous_volume_m3_exact(angle_deg);
            geometry.volume_table_m3[index] = value;
        }
        geometry
    }

    /// Build from an `EngineConfig`, which is expected to already be validated.
    pub fn from_config(config: &EngineConfig) -> Self {
        Self::new(
            config.bore_m,
            config.stroke_m,
            config.rod_length_m,
            config.compression_ratio,
        )
    }

    pub fn bore_m(&self) -> f32 {
        self.bore_m
    }

    pub fn crank_radius_m(&self) -> f32 {
        self.crank_radius_m
    }

    pub fn rod_length_m(&self) -> f32 {
        self.rod_length_m
    }

    pub fn clearance_volume_m3(&self) -> f32 {
        self.clearance_volume_m3
    }

    pub fn swept_volume_m3(&self) -> f32 {
        self.swept_volume_m3
    }

    pub fn area_m2(&self) -> f32 {
        self.area_m2
    }

    /// Piston travel from TDC along the cylinder axis, in metres.
    ///
    /// At TDC (`0°`) this is `0`; at BDC (`180°`) it equals the stroke.
    pub fn displacement_m(&self, crank_angle_deg: f32) -> f32 {
        self.disp(tau(crank_angle_deg)) as f32
    }

    /// Distance from the crankshaft axis to the piston crown, in metres.
    pub fn piston_position_m(&self, crank_angle_deg: f32) -> f32 {
        let r = self.crank_radius_m as f64;
        let l = self.rod_length_m as f64;
        let top = r + l;
        (top - self.disp(tau(crank_angle_deg))) as f32
    }

    /// Instantaneous chamber volume above the piston, in cubic metres.
    ///
    /// `V = clearance + A * (stroke_variation)`, where the swept term is derived
    /// analytically from the slider-crank kinematics.
    pub fn instantaneous_volume_m3(&self, crank_angle_deg: f32) -> f32 {
        let phase_deg = crank_angle_deg.rem_euclid(720.0);
        let position = phase_deg * VOLUME_TABLE_SAMPLES as f32 / 720.0;
        let index = (position as usize).min(VOLUME_TABLE_SAMPLES - 1);
        let fraction = position - index as f32;
        let lower = self.volume_table_m3[index];
        let upper = self.volume_table_m3[index + 1];
        lower + (upper - lower) * fraction
    }

    /// Exact analytic volume retained as a reference for bounded-equivalence
    /// tests. The real-time path uses `instantaneous_volume_m3` above.
    pub(crate) fn instantaneous_volume_m3_exact(&self, crank_angle_deg: f32) -> f32 {
        let area = self.area_m2 as f64;
        (self.clearance_volume_m3 as f64 + area * self.disp(tau(crank_angle_deg))) as f32
    }

    /// Piston displacement from TDC, given crank angle in radians.
    fn disp(&self, theta: f64) -> f64 {
        let r = self.crank_radius_m as f64;
        let l = self.rod_length_m as f64;
        let sin = theta.sin();
        let cos = theta.cos();
        let sqrt = (l * l - r * r * sin * sin).max(0.0).sqrt();
        r * (1.0 - cos) + l - sqrt
    }
}

/// Convert a crank angle in degrees (any sign, may exceed 720°) to radians.
fn tau(deg: f32) -> f64 {
    (deg as f64).to_radians()
}

#[cfg(test)]
mod tests {
    use super::*;

    const BORE_M: f32 = 0.096;
    const STROKE_M: f32 = 0.042;
    const ROD_M: f32 = 0.135;
    const CR: f32 = 12.5;

    fn sample() -> SliderCrank {
        SliderCrank::new(BORE_M, STROKE_M, ROD_M, CR)
    }

    #[test]
    fn tdc_is_minimum_volume_equal_to_clearance() {
        let s = sample();
        let v_tdc = s.instantaneous_volume_m3(0.0);
        let v_after = s.instantaneous_volume_m3(10.0);
        assert!(v_tdc.is_finite() && v_tdc > 0.0);
        assert_eq!(v_tdc, s.clearance_volume_m3());
        assert!(v_after > v_tdc, "volume must increase after TDC");
    }

    #[test]
    fn bdc_is_maximum_volume() {
        let s = sample();
        let v_bdc = s.instantaneous_volume_m3(180.0);
        let v_neighbour = s.instantaneous_volume_m3(172.0);
        assert!(v_bdc > v_neighbour, "volume must be maximum at BDC");
    }

    #[test]
    fn swept_volume_matches_analytic_geometry() {
        let s = sample();
        let swept = s.instantaneous_volume_m3(180.0) - s.instantaneous_volume_m3(0.0);
        let expected =
            std::f64::consts::PI * 0.25 * BORE_M as f64 * BORE_M as f64 * STROKE_M as f64;
        assert!(
            (swept as f64 - expected).abs() < 1e-8,
            "swept {swept} expected {expected}"
        );
    }

    #[test]
    fn compression_ratio_matches_config_within_tolerance() {
        let s = sample();
        let v_max = s.instantaneous_volume_m3(180.0);
        let v_min = s.instantaneous_volume_m3(0.0);
        let ratio = v_max / v_min;
        assert!((ratio - CR).abs() < 0.01, "ratio {ratio}");
    }

    #[test]
    fn geometry_is_finite_and_positive_across_cycle() {
        let s = sample();
        let mut max_v = f32::MIN;
        let mut min_v = f32::MAX;
        for i in 0..=7200 {
            let deg = i as f32 * 0.1;
            let v = s.instantaneous_volume_m3(deg);
            let p = s.piston_position_m(deg);
            assert!(v.is_finite() && v > 0.0, "volume at {deg}°: {v}");
            assert!(p.is_finite() && p > 0.0, "position at {deg}°: {p}");
            max_v = max_v.max(v);
            min_v = min_v.min(v);
        }
        assert!(max_v > min_v, "expected ordered volume extremes");
        assert!(min_v > 0.0);
    }

    #[test]
    fn phase_table_error_stays_within_declared_bound() {
        let s = sample();
        let mut max_error = 0.0f32;
        for i in 0..=20_000 {
            let angle = i as f32 * 0.037;
            let exact = s.instantaneous_volume_m3_exact(angle);
            let table = s.instantaneous_volume_m3(angle);
            max_error = max_error.max((table - exact).abs());
        }
        assert!(
            max_error <= 1.0e-8,
            "phase-table volume error {max_error:e} m³ exceeds 1.0e-8 m³"
        );
    }

    #[test]
    fn phase_table_wraps_and_supports_geometry_extremes() {
        let geometries = [
            SliderCrank::new(0.075, 0.030, 0.100, 8.0),
            SliderCrank::new(0.110, 0.055, 0.190, 18.0),
            SliderCrank::new(0.090, 0.050, 0.010, 10.0),
        ];
        for geometry in geometries {
            assert_eq!(
                geometry.instantaneous_volume_m3(0.0),
                geometry.instantaneous_volume_m3(720.0)
            );
            for i in 0..=4_000 {
                let angle = i as f32 * 0.181;
                let exact = geometry.instantaneous_volume_m3_exact(angle);
                let table = geometry.instantaneous_volume_m3(angle);
                assert!(exact.is_finite() && exact > 0.0);
                assert!(table.is_finite() && table > 0.0);
                assert!(
                    (table - exact).abs() <= 1.0e-8,
                    "angle {angle}: {table} vs {exact}"
                );
            }
        }
    }

    #[test]
    fn piston_position_descends_toward_bdc() {
        let s = sample();
        let top = s.piston_position_m(0.0);
        let bottom = s.piston_position_m(180.0);
        assert!(top > bottom, "crown must move away from head toward BDC");
        let travel = top - bottom;
        let expected = STROKE_M;
        assert!((travel - expected).abs() < 1e-6, "travel {travel}");
    }

    #[test]
    fn rod_length_short_geometries_are_clamped_to_non_degenerate() {
        let s = SliderCrank::new(0.09, 0.05, 0.01, 10.0);
        for i in 0..=720 {
            let v = s.instantaneous_volume_m3(i as f32);
            assert!(v.is_finite() && v > 0.0);
        }
    }
}
