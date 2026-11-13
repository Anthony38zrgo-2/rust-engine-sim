//! Wiebe-style combustion heat release driving the cylinder P/T gas state.
//!
//! This is a simplified single-zone burn model: instead of a flame-front
//! tracking the fuel mixture (which engine-sim does with a full gas network,
//! rejected in PHY-003), the burned mass fraction follows a Wiebe function and
//! the released heat is added to the post-compression internal energy. Pressure
//! then follows from the ideal gas law at the instantaneous chamber volume.
//!
//! The model is a pure function of crank angle: no per-sample allocation and no
//! hidden state beyond what a cylinder already tracks (age since firing).

use crate::config::EngineConfig;
use crate::geometry::SliderCrank;
use crate::thermodynamics::gas::{ideal_gas_pressure, GasState, CV_AIR, R_SPECIFIC_AIR};
use crate::thermodynamics::polytrope::PolytropicProcess;

const FOUR_STROKE_CYCLE_DEG: f32 = 720.0;
const EXPANSION_END_DEG: f32 = 180.0;
const EXHAUST_END_DEG: f32 = 360.0;
const INTAKE_END_DEG: f32 = 540.0;
const EXHAUST_PRESSURE_PA: f32 = 101_325.0;
const EXHAUST_TEMPERATURE_K: f32 = 1_000.0;
const INTAKE_MANIFOLD_PRESSURE_PA: f32 = 101_325.0;

/// Wiebe completeness factor `a`; `X_end = 1 - exp(-a)`.
pub const WIEBE_A: f32 = 5.0;
/// Wiebe shape factor `m`.
pub const WIEBE_M: f32 = 3.0;
/// Intake manifold pressure at the start of compression, Pa.
pub const INTAKE_PRESSURE_PA: f32 = 101_325.0;
/// Intake manifold temperature, K.
pub const INTAKE_TEMPERATURE_K: f32 = 300.0;

/// Explicit four-stroke phase after the firing TDC at angle zero.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ChamberPhase {
    Expansion = 0,
    Exhaust = 1,
    Intake = 2,
    Compression = 3,
}

impl Default for ChamberPhase {
    fn default() -> Self {
        Self::Expansion
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CombustionFrame {
    pub pressure_pa: f32,
    pub temperature_k: f32,
    pub burn_fraction: f32,
    pub phase: ChamberPhase,
    pub heat_release_rate_w: f32,
}

/// A bounded single-zone combustion chamber driven by a 720-degree cycle.
///
/// The model is deliberately a small real-time approximation, not a CFD or a
/// complete mass/energy solver.  The charge is renewed at the start of the
/// compression stroke; heat release is only present in the firing/expansion
/// phase; exhaust and intake use bounded prescribed boundary states.  This
/// prevents the old model from carrying burned energy into a second hot
/// compression while keeping the pressure source deterministic and cheap.
#[derive(Clone, Copy, Debug)]
pub struct CombustionChamber {
    ignition_deg: f32,
    burn_duration_deg: f32,
    shape_factor: f32,
    efficiency: f32,
    fuel_energy_j: f32,
    charge: GasState,
    geometry: SliderCrank,
    polytrope: PolytropicProcess,
}

impl CombustionChamber {
    /// Build from configuration. The charge is derived from intake conditions
    /// at BDC (maximum volume) so that the polytropic compression reference is
    /// a proper ideal-gas state.
    pub fn new(config: &EngineConfig) -> Self {
        let geometry = SliderCrank::from_config(config);
        let v_bdc = geometry.instantaneous_volume_m3(180.0);
        let mass = INTAKE_PRESSURE_PA * v_bdc / (R_SPECIFIC_AIR * INTAKE_TEMPERATURE_K);
        let charge = GasState::new(INTAKE_PRESSURE_PA, INTAKE_TEMPERATURE_K, mass, v_bdc);
        Self {
            ignition_deg: config.combustion_start_deg,
            burn_duration_deg: config.combustion_rise_deg,
            shape_factor: config.combustion_shape_factor,
            efficiency: config.combustion_efficiency,
            fuel_energy_j: config.fuel_energy_per_cycle,
            charge,
            geometry,
            polytrope: PolytropicProcess::adiabatic(),
        }
    }

    /// Cumulative Wiebe burned-mass fraction at `deg` past ignition.
    pub fn burn_fraction(&self, deg_past_ignition: f32) -> f32 {
        if deg_past_ignition <= 0.0 || self.burn_duration_deg <= 0.0 {
            return 0.0;
        }
        // Clamp to the well-defined burn window to avoid overflow on exp.
        let u = (deg_past_ignition / self.burn_duration_deg).clamp(0.0, 1.0);
        1.0 - (-WIEBE_A * u.powf(self.shape_factor + 1.0)).exp()
    }

    /// Wiebe heat-release rate (`dQ/ddeg`), in Joules per degree.
    pub fn heat_release_rate_per_deg(&self, deg_past_ignition: f32, energy: f32) -> f32 {
        if deg_past_ignition <= 0.0 || self.burn_duration_deg <= 0.0 {
            return 0.0;
        }
        let u = (deg_past_ignition / self.burn_duration_deg).clamp(0.0, 1.0);
        if u >= 1.0 {
            return 0.0;
        }
        let dfrac = self.d_burn_fraction_d_deg(u);
        self.fuel_energy_j * self.efficiency * energy * dfrac
    }

    fn d_burn_fraction_d_deg(&self, u: f32) -> f32 {
        let m1 = self.shape_factor + 1.0;
        let outer = (-WIEBE_A * u.powf(m1)).exp();
        WIEBE_A * m1 / self.burn_duration_deg * u.powf(self.shape_factor) * outer
    }

    #[inline]
    fn smoothstep(value: f32) -> f32 {
        let t = value.clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    }

    /// Cold charge state at a phase angle.  The charge is always referenced to
    /// the fresh intake state at BDC, so compression after intake cannot reuse
    /// the burned state from the previous expansion stroke.
    fn cold_charge_state(&self, angle_deg: f32) -> GasState {
        let volume = self.geometry.instantaneous_volume_m3(angle_deg);
        self.polytrope.state_at(&self.charge, volume)
    }

    /// Combustion/expansion state while the exhaust valve is still closed.
    fn expansion_state(&self, angle_deg: f32, energy: f32) -> (GasState, f32, f32) {
        let cold = self.cold_charge_state(angle_deg);
        let deg_past_ignition = angle_deg - self.ignition_deg;
        let burn_fraction = self.burn_fraction(deg_past_ignition);
        let heat_j = self.fuel_energy_j * self.efficiency * energy * burn_fraction;
        let temperature_k = cold.temperature_k + heat_j / (self.charge.mass_kg * CV_AIR);
        let pressure_pa = ideal_gas_pressure(self.charge.mass_kg, temperature_k, cold.volume_m3);
        (
            GasState::new(
                pressure_pa,
                temperature_k,
                self.charge.mass_kg,
                cold.volume_m3,
            ),
            burn_fraction,
            self.heat_release_rate_per_deg(deg_past_ignition, energy),
        )
    }

    /// Combustion state at `age_deg` past firing (TDC at `age = 0`).
    ///
    /// Angle zero and 720 degrees are the same firing TDC.  The four phases
    /// are explicit: 0..180 expansion/combustion, 180..360 exhaust, 360..540
    /// intake, and 540..720 compression.  Exhaust/intake are bounded boundary
    /// approximations; they intentionally do not claim full thermodynamic
    /// conservation.  The important invariant is that fresh charge is used
    /// for every compression stroke and the cycle closes continuously.
    ///
    /// Pressure is ideal-gas pressure for the compression/combustion envelope;
    /// the exhaust and intake boundary states remain finite and bounded.
    pub fn frame(&self, age_deg: f32, energy: f32) -> CombustionFrame {
        let angle = age_deg.rem_euclid(FOUR_STROKE_CYCLE_DEG);
        if angle < EXPANSION_END_DEG {
            let (state, burn_fraction, heat_release_rate_w) = self.expansion_state(angle, energy);
            return CombustionFrame {
                pressure_pa: state.pressure_pa,
                temperature_k: state.temperature_k,
                burn_fraction,
                phase: ChamberPhase::Expansion,
                heat_release_rate_w,
            };
        }

        if angle < EXHAUST_END_DEG {
            let (at_bdc, _, _) = self.expansion_state(EXPANSION_END_DEG, energy);
            let exhaust_progress = Self::smoothstep(
                (angle - EXPANSION_END_DEG) / (EXHAUST_END_DEG - EXPANSION_END_DEG),
            );
            return CombustionFrame {
                pressure_pa: at_bdc.pressure_pa
                    + (EXHAUST_PRESSURE_PA - at_bdc.pressure_pa) * exhaust_progress,
                temperature_k: at_bdc.temperature_k
                    + (EXHAUST_TEMPERATURE_K - at_bdc.temperature_k) * exhaust_progress,
                burn_fraction: self.burn_fraction(EXPANSION_END_DEG - self.ignition_deg),
                phase: ChamberPhase::Exhaust,
                heat_release_rate_w: 0.0,
            };
        }

        if angle < INTAKE_END_DEG {
            let intake_progress =
                Self::smoothstep((angle - EXHAUST_END_DEG) / (INTAKE_END_DEG - EXHAUST_END_DEG));
            return CombustionFrame {
                pressure_pa: EXHAUST_PRESSURE_PA
                    + (INTAKE_MANIFOLD_PRESSURE_PA - EXHAUST_PRESSURE_PA) * intake_progress,
                temperature_k: EXHAUST_TEMPERATURE_K
                    + (INTAKE_TEMPERATURE_K - EXHAUST_TEMPERATURE_K) * intake_progress,
                burn_fraction: 0.0,
                phase: ChamberPhase::Intake,
                heat_release_rate_w: 0.0,
            };
        }

        let state = self.cold_charge_state(angle);
        CombustionFrame {
            pressure_pa: state.pressure_pa,
            temperature_k: state.temperature_k,
            burn_fraction: 0.0,
            phase: ChamberPhase::Compression,
            heat_release_rate_w: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chamber() -> CombustionChamber {
        CombustionChamber::new(&EngineConfig::default())
    }

    fn exact_cold_charge_state(chamber: &CombustionChamber, angle_deg: f32) -> GasState {
        let volume = chamber.geometry.instantaneous_volume_m3_exact(angle_deg);
        chamber.polytrope.state_at(&chamber.charge, volume)
    }

    fn exact_expansion_state(
        chamber: &CombustionChamber,
        angle_deg: f32,
        energy: f32,
    ) -> (GasState, f32, f32) {
        let cold = exact_cold_charge_state(chamber, angle_deg);
        let deg_past_ignition = angle_deg - chamber.ignition_deg;
        let burn_fraction = chamber.burn_fraction(deg_past_ignition);
        let heat_j = chamber.fuel_energy_j * chamber.efficiency * energy * burn_fraction;
        let temperature_k = cold.temperature_k + heat_j / (chamber.charge.mass_kg * CV_AIR);
        let pressure_pa = ideal_gas_pressure(chamber.charge.mass_kg, temperature_k, cold.volume_m3);
        (
            GasState::new(
                pressure_pa,
                temperature_k,
                chamber.charge.mass_kg,
                cold.volume_m3,
            ),
            burn_fraction,
            chamber.heat_release_rate_per_deg(deg_past_ignition, energy),
        )
    }

    fn exact_frame(chamber: &CombustionChamber, age_deg: f32, energy: f32) -> CombustionFrame {
        let angle = age_deg.rem_euclid(FOUR_STROKE_CYCLE_DEG);
        if angle < EXPANSION_END_DEG {
            let (state, burn_fraction, heat_release_rate_w) =
                exact_expansion_state(chamber, angle, energy);
            return CombustionFrame {
                pressure_pa: state.pressure_pa,
                temperature_k: state.temperature_k,
                burn_fraction,
                phase: ChamberPhase::Expansion,
                heat_release_rate_w,
            };
        }
        if angle < EXHAUST_END_DEG {
            let (at_bdc, _, _) = exact_expansion_state(chamber, EXPANSION_END_DEG, energy);
            let exhaust_progress = CombustionChamber::smoothstep(
                (angle - EXPANSION_END_DEG) / (EXHAUST_END_DEG - EXPANSION_END_DEG),
            );
            return CombustionFrame {
                pressure_pa: at_bdc.pressure_pa
                    + (EXHAUST_PRESSURE_PA - at_bdc.pressure_pa) * exhaust_progress,
                temperature_k: at_bdc.temperature_k
                    + (EXHAUST_TEMPERATURE_K - at_bdc.temperature_k) * exhaust_progress,
                burn_fraction: chamber.burn_fraction(EXPANSION_END_DEG - chamber.ignition_deg),
                phase: ChamberPhase::Exhaust,
                heat_release_rate_w: 0.0,
            };
        }
        if angle < INTAKE_END_DEG {
            let intake_progress = CombustionChamber::smoothstep(
                (angle - EXHAUST_END_DEG) / (INTAKE_END_DEG - EXHAUST_END_DEG),
            );
            return CombustionFrame {
                pressure_pa: EXHAUST_PRESSURE_PA
                    + (INTAKE_MANIFOLD_PRESSURE_PA - EXHAUST_PRESSURE_PA) * intake_progress,
                temperature_k: EXHAUST_TEMPERATURE_K
                    + (INTAKE_TEMPERATURE_K - EXHAUST_TEMPERATURE_K) * intake_progress,
                burn_fraction: 0.0,
                phase: ChamberPhase::Intake,
                heat_release_rate_w: 0.0,
            };
        }
        let state = exact_cold_charge_state(chamber, angle);
        CombustionFrame {
            pressure_pa: state.pressure_pa,
            temperature_k: state.temperature_k,
            burn_fraction: 0.0,
            phase: ChamberPhase::Compression,
            heat_release_rate_w: 0.0,
        }
    }

    #[test]
    fn burn_fraction_starts_zero_and_reaches_completeness() {
        let c = chamber();
        assert_eq!(c.burn_fraction(0.0), 0.0);
        assert_eq!(c.burn_fraction(-5.0), 0.0);
        let end = c.burn_fraction(c.burn_duration_deg);
        assert!((end - (1.0 - (-WIEBE_A).exp())).abs() < 1.0e-4);
        assert!(end > 0.98);
    }

    #[test]
    fn heat_release_raises_pressure_and_temperature_above_compression() {
        let c = chamber();
        let low = c.frame(10.0, 0.3);
        let high = c.frame(10.0, 1.0);
        let comp = c
            .polytrope
            .state_at(&c.charge, c.geometry.instantaneous_volume_m3(10.0));
        // Compression-only baseline at the same angle.
        assert!(high.pressure_pa >= comp.pressure_pa);
        assert!(high.temperature_k > comp.temperature_k);
        assert!(
            high.pressure_pa > low.pressure_pa,
            "more energy -> more pressure"
        );
    }

    #[test]
    fn heat_release_rate_is_crank_angle_coherent() {
        let c = chamber();
        let mut peak = f32::MIN;
        let mut peak_deg = f32::MIN;
        for deg in (0..=c.burn_duration_deg as usize).map(|d| d as f32) {
            let rate = c.heat_release_rate_per_deg(deg, 1.0);
            assert!(rate.is_finite() && rate >= 0.0);
            if rate > peak {
                peak = rate;
                peak_deg = deg;
            }
            assert!(rate >= -1.0e-6, "rate must not go negative near {deg}");
        }
        assert!(peak > 0.0);
        assert!(peak_deg > 0.0, "heat release must ramp up after ignition");
    }

    #[test]
    fn pressure_is_bounded_and_finite_over_full_cycle() {
        let c = chamber();
        let mut peak = 0.0f32;
        let mut min = f32::MAX;
        for i in 0..=720 {
            let deg = i as f32;
            let f = c.frame(deg, 1.0);
            assert!(f.pressure_pa.is_finite() && f.pressure_pa > 0.0);
            assert!(f.temperature_k.is_finite() && f.temperature_k > 0.0);
            peak = peak.max(f.pressure_pa);
            min = min.min(f.pressure_pa);
        }
        assert!(peak > min);
        assert!(
            peak < 5.0e7,
            "peak pressure must stay within physical bounds"
        );
    }

    #[test]
    fn fresh_charge_prevents_second_hot_compression() {
        let c = chamber();
        let firing_tdc = c.frame(0.0, 1.0);
        let exhaust_tdc = c.frame(360.0, 1.0);
        let next_firing_tdc = c.frame(720.0, 1.0);

        assert_eq!(firing_tdc.phase, ChamberPhase::Expansion);
        assert_eq!(exhaust_tdc.phase, ChamberPhase::Intake);
        assert!(
            exhaust_tdc.pressure_pa < firing_tdc.pressure_pa * 0.1,
            "exhaust TDC must not recompress the burned charge: {} vs {}",
            exhaust_tdc.pressure_pa,
            firing_tdc.pressure_pa
        );
        assert!(
            (next_firing_tdc.pressure_pa - firing_tdc.pressure_pa).abs() < 1.0,
            "cycle boundary must reuse fresh compression state"
        );
    }

    #[test]
    fn motored_cycle_has_compression_without_heat_release() {
        let c = chamber();
        let motored = c.frame(0.0, 0.0);
        let loaded = c.frame(0.0, 1.0);
        assert_eq!(motored.burn_fraction, 0.0);
        assert_eq!(motored.heat_release_rate_w, 0.0);
        assert!(motored.pressure_pa > EXHAUST_PRESSURE_PA);
        assert!(loaded.pressure_pa >= motored.pressure_pa);
    }

    #[test]
    fn no_heat_before_ignition_matches_pure_compression() {
        let c = chamber();
        let pre = c.frame(-20.0, 1.0);
        let comp = c
            .polytrope
            .state_at(&c.charge, c.geometry.instantaneous_volume_m3(-20.0));
        assert!((pre.temperature_k - comp.temperature_k).abs() < 1.0e-2);
        assert!((pre.burn_fraction).abs() < 1.0e-6);
    }

    #[test]
    fn phase_table_matches_corrected_chamber_at_steady_and_transient_points() {
        let c = chamber();
        let points = [
            (0.0, 0.0),
            (8.3, 0.25),
            (27.7, 0.82),
            (91.4, 1.0),
            (179.7, 0.64),
            (180.1, 0.64),
            (271.3, 0.64),
            (359.9, 0.64),
            (421.7, 0.18),
            (539.8, 0.18),
            (612.2, 0.18),
            (719.9, 0.92),
            (720.0, 0.92),
            (1_439.9, 0.37),
        ];
        for (angle, energy) in points {
            let optimized = c.frame(angle, energy);
            let reference = exact_frame(&c, angle, energy);
            assert_eq!(optimized.phase, reference.phase, "phase at {angle}°");
            assert_eq!(
                optimized.burn_fraction, reference.burn_fraction,
                "burn at {angle}°"
            );
            assert_eq!(optimized.heat_release_rate_w, reference.heat_release_rate_w);
            let pressure_relative = (optimized.pressure_pa - reference.pressure_pa).abs()
                / reference.pressure_pa.max(1.0);
            let temperature_relative = (optimized.temperature_k - reference.temperature_k).abs()
                / reference.temperature_k.max(1.0);
            assert!(
                pressure_relative <= 2.0e-3,
                "pressure relative error {pressure_relative:e} at {angle}°"
            );
            assert!(
                temperature_relative <= 2.0e-4,
                "temperature relative error {temperature_relative:e} at {angle}°"
            );
        }
    }

    #[test]
    fn corrected_chamber_energy_response_is_affine_but_not_promoted_to_a_table() {
        let c = chamber();
        for angle in [0.0, 8.3, 91.4, 179.7, 180.1, 271.3, 421.7, 612.2, 719.9] {
            let zero = c.frame(angle, 0.0);
            let half = c.frame(angle, 0.5);
            let full = c.frame(angle, 1.0);
            let expected_pressure = (zero.pressure_pa + full.pressure_pa) * 0.5;
            let expected_temperature = (zero.temperature_k + full.temperature_k) * 0.5;
            assert!(
                (half.pressure_pa - expected_pressure).abs()
                    <= expected_pressure.abs().max(1.0) * 2.0e-6,
                "pressure energy response is not affine at {angle}°"
            );
            assert!(
                (half.temperature_k - expected_temperature).abs()
                    <= expected_temperature.abs().max(1.0) * 2.0e-6,
                "temperature energy response is not affine at {angle}°"
            );
            assert_eq!(half.phase, full.phase);
        }
    }

    #[test]
    #[ignore]
    fn print_peak_magnitudes() {
        let c = chamber();
        for energy in [0.0f32, 0.25, 0.5, 0.76, 1.0] {
            let mut peak = 0.0f32;
            let mut peak_deg = 0.0f32;
            let mut tpeak = 0.0f32;
            for i in -200..=300 {
                let deg = i as f32;
                let f = c.frame(deg, energy);
                if f.pressure_pa > peak {
                    peak = f.pressure_pa;
                    peak_deg = deg;
                    tpeak = f.temperature_k;
                }
            }
            eprintln!(
                "energy={energy:.2} peak={:.3} MPa at {peak_deg:.0}deg T={tpeak:.0}K",
                peak / 1.0e6
            );
        }
    }
}
