//! Per-cylinder combustion, poppet exhaust valve, and runner boundary state.

use crate::config::EngineConfig;
use crate::dsp::{xorshift64_signed, ButterworthLp4};
use crate::physics::thermodynamics::combustion::{
    ChamberPhase, CombustionChamber, CombustionFrame,
};
use crate::physics::thermodynamics::exhaust_runner::ExhaustRunner;
use crate::physics::thermodynamics::exhaust_valve::ExhaustValve;

/// Poppet-valve flow effective exponent relating geometric lift fraction to turbulent discharge area.
pub const VALVE_FLOW_AREA_EXPONENT: f32 = 1.35;
/// 2x-oversampled 4th-order Butterworth decimation cutoffs used by refined DSP:
/// suppresses the flat `8–24 kHz` shelf and folds >24 kHz energy down by >28 dB.
const REFINED_DP_DECIMATION_LOWPASS_HZ: f32 = 9_500.0;
const REFINED_EXHAUST_DECIMATION_LOWPASS_HZ: f32 = 10_500.0;

/// Cycle-to-cycle stochastic gain slew time constant (~1 ms) when refined DSP is enabled.
const REFINED_CYCLE_GAIN_SLEW_SECONDS: f32 = 0.001;
/// Mean and symmetric scatter (deg) for cycle ignition delay in refined DSP mode (`[0.0, 0.6]°`).
const REFINED_IGNITION_OFFSET_MEAN_DEG: f32 = 0.30;
const REFINED_IGNITION_JITTER_DEG: f32 = 0.30;
/// Symmetric fractional scatter (`±5%`) applied to cycle Wiebe burn duration in refined DSP mode.
const REFINED_BURN_DURATION_JITTER_FRACTION: f32 = 0.05;

/// Instantaneous thermodynamic and acoustic excitation frame emitted by one cylinder.
#[derive(Clone, Copy, Debug, Default)]
pub struct CylinderFrame {
    pub pressure: f32,
    pub pressure_derivative: f32,
    /// Absolute chamber pressure (Pa) from the bounded four-stroke model.
    pub chamber_pressure_pa: f32,
    /// Chamber bulk temperature (K) from the bounded four-stroke model.
    pub chamber_temperature_k: f32,
    /// Heat-release diagnostic in the model's Joules-per-crank-degree units.
    pub chamber_heat_release_rate: f32,
    pub chamber_phase: ChamberPhase,
    pub blowdown: f32,
    pub valve_lift: f32,
    /// Effective exhaust port area (m²) from the physical valve model.
    pub effective_area_m2: f32,
    /// Pressure-driven exhaust mass flow (kg/s) from the runner boundary.
    pub exhaust_mass_flow_kg_s: f32,
    /// Per-sample change in exhaust mass flow (kg/s): physical blowdown edge.
    pub exhaust_dmass_flow_kg_s: f32,
    /// Normalised acoustic excitation fed into the header waveguide.
    pub exhaust_excitation: f32,
    /// Exhaust-runner gas pressure (Pa) driving the valve discharge.
    pub runner_pressure_pa: f32,
    /// Exhaust-runner gas temperature (K).
    pub runner_temperature_k: f32,
}

/// Complete single-cylinder thermodynamic and valve-boundary simulator.
#[derive(Clone, Copy, Debug)]
pub struct Cylinder {
    age_deg: f32,
    last_pressure: f32,
    last_exhaust_flow: f32,
    last_exhaust_mass_flow: f32,
    cycle_gain: f32,
    cycle_gain_target: f32,
    cycle_slew_alpha_2x: f32,
    ignition_offset_deg: f32,
    burn_duration_scale: f32,
    fixed_gain: f32,
    chamber: CombustionChamber,
    exhaust_valve: ExhaustValve,
    exhaust_runner: ExhaustRunner,
    dp_decimator: ButterworthLp4,
    exhaust_decimator: ButterworthLp4,
    rng: u64,
    rng_jitter: u64,
}

impl Cylinder {
    /// Initializes cylinder `index` with deterministic per-cylinder signature and PRNG seed.
    pub fn new(index: usize, config: &EngineConfig) -> Self {
        let half_span = (config.cylinder_count.saturating_sub(1)) as f32 * 0.5;
        let offset = if half_span > 0.0 {
            (index as f32 - half_span) / half_span
        } else {
            0.0
        };
        let fixed_gain = (1.0 + offset * config.cylinder_spread) * config.cylinder_signature[index];
        let sample_rate = config.sample_rate as f32;
        let oversampled_rate = sample_rate * 2.0;
        let seed = config.seed ^ ((index as u64 + 1).wrapping_mul(0x9E37_79B9_7F4A_7C15));
        Self {
            age_deg: 1.0e9,
            last_pressure: 0.0,
            last_exhaust_flow: 0.0,
            last_exhaust_mass_flow: 0.0,
            cycle_gain: 1.0,
            cycle_gain_target: 1.0,
            cycle_slew_alpha_2x: 1.0
                - (-1.0 / (REFINED_CYCLE_GAIN_SLEW_SECONDS * oversampled_rate)).exp(),
            ignition_offset_deg: 0.0,
            burn_duration_scale: 1.0,
            fixed_gain,
            chamber: CombustionChamber::new(config),
            exhaust_valve: ExhaustValve::new(config),
            exhaust_runner: ExhaustRunner::new(config, index),
            dp_decimator: ButterworthLp4::new(REFINED_DP_DECIMATION_LOWPASS_HZ, oversampled_rate),
            exhaust_decimator: ButterworthLp4::new(
                REFINED_EXHAUST_DECIMATION_LOWPASS_HZ,
                oversampled_rate,
            ),
            rng: seed,
            rng_jitter: (seed ^ 0xD1B5_4A32_D192_ED03) | 1,
        }
    }

    /// Triggers a new firing cycle at TDC (`age_deg = 0.0`) with deterministic cycle variation.
    #[inline]
    pub fn fire(&mut self, variation: f32) {
        self.fire_at(variation, 0.0);
    }

    /// Triggers a new firing cycle starting at sub-sample residual angle `initial_age_deg`.
    #[inline]
    pub fn fire_at(&mut self, variation: f32, initial_age_deg: f32) {
        self.age_deg = initial_age_deg.max(0.0);
        self.cycle_gain_target = 1.0 + xorshift64_signed(&mut self.rng) * variation;
        self.ignition_offset_deg = (REFINED_IGNITION_OFFSET_MEAN_DEG
            + xorshift64_signed(&mut self.rng_jitter) * REFINED_IGNITION_JITTER_DEG)
            .max(0.0);
        self.burn_duration_scale =
            1.0 + xorshift64_signed(&mut self.rng_jitter) * REFINED_BURN_DURATION_JITTER_FRACTION;
    }

    #[inline]
    fn pressure_shape(age: f32, rise_deg: f32, expansion_decay_deg: f32) -> f32 {
        if age < 0.0 {
            return 0.0;
        }
        if age <= rise_deg {
            let u = (age / rise_deg).clamp(0.0, 1.0);
            let burn = 1.0 - (-6.0 * u.powf(2.35)).exp();
            burn * (-0.55 * u).exp()
        } else {
            let peak = (1.0 - (-6.0f32).exp()) * (-0.55f32).exp();
            peak * (-(age - rise_deg) / expansion_decay_deg).exp()
        }
    }

    /// Steps chamber pressure, valve lift, and exhaust runner gas exchange by one sample.
    #[inline]
    pub fn process(
        &mut self,
        deg_per_sample: f32,
        energy: f32,
        config: &EngineConfig,
    ) -> CylinderFrame {
        if config.use_refined_dsp {
            self.process_refined(deg_per_sample, energy, config)
        } else {
            self.process_legacy(deg_per_sample, energy, config)
        }
    }

    #[inline]
    fn process_legacy(
        &mut self,
        deg_per_sample: f32,
        energy: f32,
        config: &EngineConfig,
    ) -> CylinderFrame {
        self.cycle_gain = self.cycle_gain_target;

        let chamber_frame = config
            .use_physical_pressure
            .then(|| self.chamber.frame(self.age_deg, energy));
        let (pressure, chamber_pressure_pa, chamber_temperature_k) =
            if let Some(frame) = chamber_frame {
                (
                    frame.pressure_pa / config.combustion_pressure_reference_pa * self.cycle_gain,
                    frame.pressure_pa,
                    frame.temperature_k,
                )
            } else {
                let shifted_age = self.age_deg - config.combustion_start_deg;
                let p = Self::pressure_shape(
                    shifted_age,
                    config.combustion_rise_deg,
                    config.expansion_decay_deg,
                ) * energy
                    * self.cycle_gain;
                (p, 0.0, 0.0)
            };
        let chamber_frame = chamber_frame.unwrap_or(CombustionFrame {
            pressure_pa: 0.0,
            temperature_k: 0.0,
            burn_fraction: 0.0,
            phase: ChamberPhase::Expansion,
            heat_release_rate_w: 0.0,
        });
        let pressure = pressure * self.fixed_gain;
        let pressure_derivative = pressure - self.last_pressure;
        self.last_pressure = pressure;

        let lift_fraction = self.exhaust_valve.lift_fraction(self.age_deg);
        let effective_area_m2 = self.exhaust_valve.effective_area_m2(lift_fraction);
        let runner_state = self.exhaust_runner.state();
        let mass_flow = self.exhaust_runner.mass_flow(
            chamber_pressure_pa,
            chamber_temperature_k,
            effective_area_m2,
        );
        let dt_s = 1.0 / config.sample_rate as f32;
        self.exhaust_runner
            .step(mass_flow, dt_s, chamber_temperature_k);
        let valve_lift = lift_fraction;
        let exhaust_flow = pressure * valve_lift.powf(VALVE_FLOW_AREA_EXPONENT);
        let blowdown = exhaust_flow - self.last_exhaust_flow;
        self.last_exhaust_flow = exhaust_flow;
        let dmass_flow_kg_s = mass_flow - self.last_exhaust_mass_flow;
        self.last_exhaust_mass_flow = mass_flow;
        let exhaust_excitation = if config.use_physical_exhaust_excitation {
            dmass_flow_kg_s * self.fixed_gain * config.exhaust_excitation_gain
        } else {
            blowdown
        };
        self.age_deg += deg_per_sample;
        CylinderFrame {
            pressure,
            pressure_derivative,
            chamber_pressure_pa,
            chamber_temperature_k,
            chamber_heat_release_rate: chamber_frame.heat_release_rate_w,
            chamber_phase: chamber_frame.phase,
            blowdown,
            valve_lift,
            effective_area_m2,
            exhaust_mass_flow_kg_s: mass_flow,
            exhaust_dmass_flow_kg_s: dmass_flow_kg_s,
            exhaust_excitation,
            runner_pressure_pa: runner_state.pressure_pa,
            runner_temperature_k: runner_state.temperature_k,
        }
    }

    #[inline]
    fn process_refined(
        &mut self,
        deg_per_sample: f32,
        energy: f32,
        config: &EngineConfig,
    ) -> CylinderFrame {
        let half_deg = 0.5 * deg_per_sample;
        let half_dt_s = 0.5 / config.sample_rate as f32;
        let prev_exhaust_flow = self.last_exhaust_flow;
        let prev_mass_flow = self.last_exhaust_mass_flow;

        let mut pressure = 0.0f32;
        let mut pressure_derivative = 0.0f32;
        let mut chamber_frame = CombustionFrame {
            pressure_pa: 0.0,
            temperature_k: 0.0,
            burn_fraction: 0.0,
            phase: ChamberPhase::Expansion,
            heat_release_rate_w: 0.0,
        };
        let mut chamber_pressure_pa = 0.0f32;
        let mut chamber_temperature_k = 0.0f32;
        let mut lift_fraction = 0.0f32;
        let mut effective_area_m2 = 0.0f32;
        let mut runner_state = self.exhaust_runner.state();
        let mut mass_flow = 0.0f32;
        let mut exhaust_flow = 0.0f32;
        let mut exhaust_excitation = 0.0f32;

        for substep in 0..2 {
            self.cycle_gain +=
                self.cycle_slew_alpha_2x * (self.cycle_gain_target - self.cycle_gain);
            let sub_age = self.age_deg + substep as f32 * half_deg;

            let raw_dp_1x = if config.use_physical_pressure {
                let (frame, dp_ddeg) = self.chamber.frame_refined_with_derivative(
                    sub_age,
                    energy * self.cycle_gain,
                    self.ignition_offset_deg,
                    self.burn_duration_scale,
                );
                let norm_scale = self.fixed_gain / config.combustion_pressure_reference_pa;
                pressure = frame.pressure_pa * norm_scale;
                chamber_pressure_pa = frame.pressure_pa;
                chamber_temperature_k = frame.temperature_k;
                chamber_frame = frame;
                dp_ddeg * norm_scale * deg_per_sample
            } else {
                let shifted_age = sub_age - config.combustion_start_deg;
                pressure = Self::pressure_shape(
                    shifted_age,
                    config.combustion_rise_deg,
                    config.expansion_decay_deg,
                ) * energy
                    * self.cycle_gain
                    * self.fixed_gain;
                chamber_pressure_pa = 0.0;
                chamber_temperature_k = 0.0;
                (pressure - self.last_pressure) * 2.0
            };
            self.last_pressure = pressure;
            pressure_derivative = self.dp_decimator.process(raw_dp_1x);

            lift_fraction = self.exhaust_valve.lift_fraction(sub_age);
            effective_area_m2 = self.exhaust_valve.effective_area_m2_smooth(lift_fraction);
            runner_state = self.exhaust_runner.state();
            mass_flow = self.exhaust_runner.mass_flow(
                chamber_pressure_pa,
                chamber_temperature_k,
                effective_area_m2,
            );
            self.exhaust_runner
                .step(mass_flow, half_dt_s, chamber_temperature_k);

            exhaust_flow = pressure * lift_fraction.powf(VALVE_FLOW_AREA_EXPONENT);
            let blowdown_1x = (exhaust_flow - self.last_exhaust_flow) * 2.0;
            self.last_exhaust_flow = exhaust_flow;
            let dmass_1x = (mass_flow - self.last_exhaust_mass_flow) * 2.0;
            self.last_exhaust_mass_flow = mass_flow;
            let raw_exh_exc = if config.use_physical_exhaust_excitation {
                dmass_1x * self.fixed_gain * config.exhaust_excitation_gain
            } else {
                blowdown_1x
            };
            exhaust_excitation = self.exhaust_decimator.process(raw_exh_exc);
        }

        self.age_deg += deg_per_sample;
        CylinderFrame {
            pressure,
            pressure_derivative,
            chamber_pressure_pa,
            chamber_temperature_k,
            chamber_heat_release_rate: chamber_frame.heat_release_rate_w,
            chamber_phase: chamber_frame.phase,
            blowdown: exhaust_flow - prev_exhaust_flow,
            valve_lift: lift_fraction,
            effective_area_m2,
            exhaust_mass_flow_kg_s: mass_flow,
            exhaust_dmass_flow_kg_s: mass_flow - prev_mass_flow,
            exhaust_excitation,
            runner_pressure_pa: runner_state.pressure_pa,
            runner_temperature_k: runner_state.temperature_k,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blowdown_cannot_precede_valve_opening() {
        let config = EngineConfig::default();
        let mut cylinder = Cylinder::new(0, &config);
        cylinder.fire(0.0);
        let deg = 1.0;
        for age in 0..config.exhaust_open_deg as usize {
            let frame = cylinder.process(deg, 1.0, &config);
            assert_eq!(frame.blowdown, 0.0, "blowdown at age {age}");
        }
    }

    #[test]
    fn pressure_event_is_finite_and_decays() {
        let config = EngineConfig::default();
        let mut cylinder = Cylinder::new(0, &config);
        cylinder.fire(0.0);
        let mut peak = 0.0f32;
        for _ in 0..1_500 {
            let frame = cylinder.process(0.5, 1.0, &config);
            assert!(frame.pressure.is_finite() && frame.pressure_derivative.is_finite());
            peak = peak.max(frame.pressure);
        }
        assert!(peak > 0.5);
        let late = cylinder.process(0.5, 1.0, &config);
        assert!(late.pressure.is_finite() && late.pressure_derivative.is_finite());
    }

    #[test]
    fn physical_pressure_rises_with_load() {
        let config = EngineConfig::default();
        let mut low = Cylinder::new(0, &config);
        let mut high = Cylinder::new(0, &config);
        low.fire(0.0);
        high.fire(0.0);
        let mut low_peak = 0.0f32;
        let mut high_peak = 0.0f32;
        let deg = 0.5;
        for _ in 0..120 {
            let f = low.process(deg, 0.25, &config);
            low_peak = low_peak.max(f.pressure);
            let f = high.process(deg, 1.0, &config);
            high_peak = high_peak.max(f.pressure);
        }
        assert!(high_peak > low_peak);
    }

    #[test]
    fn legacy_mode_still_decays_to_tail() {
        let config = EngineConfig {
            use_physical_pressure: false,
            ..EngineConfig::default()
        };
        let mut cylinder = Cylinder::new(0, &config);
        cylinder.fire(0.0);
        let mut peak = 0.0f32;
        let mut last = 0.0;
        for _ in 0..1_500 {
            let frame = cylinder.process(0.5, 1.0, &config);
            peak = peak.max(frame.pressure);
            last = frame.pressure;
        }
        assert!(peak > 0.5 && last < peak * 0.02);
    }

    #[test]
    fn refined_cycle_gain_leaves_compression_tdc_continuous_at_fire() {
        let legacy_cfg = EngineConfig {
            cycle_variation: 0.25,
            use_refined_dsp: false,
            ..EngineConfig::ford_pinto_rs2000(48_000)
        };
        let refined_cfg = EngineConfig {
            cycle_variation: 0.25,
            use_refined_dsp: true,
            ..EngineConfig::ford_pinto_rs2000(48_000)
        };
        let mut legacy = Cylinder::new(0, &legacy_cfg);
        let mut refined = Cylinder::new(0, &refined_cfg);
        let deg_per_sample = 4_500.0 * 6.0 / 48_000.0;
        legacy.fire_at(legacy_cfg.cycle_variation, 0.0);
        refined.fire_at(refined_cfg.cycle_variation, 0.0);

        let mut legacy_fire_jump = 0.0f32;
        let mut refined_fire_jump = 0.0f32;
        let mut prev_leg = 0.0f32;
        let mut prev_ref = 0.0f32;
        for step in 0..4_000 {
            let firing = legacy.age_deg >= 720.0;
            if firing {
                legacy.fire_at(legacy_cfg.cycle_variation, legacy.age_deg - 720.0);
                refined.fire_at(refined_cfg.cycle_variation, refined.age_deg - 720.0);
            }
            let fl = legacy.process(deg_per_sample, 1.0, &legacy_cfg);
            let fr = refined.process(deg_per_sample, 1.0, &refined_cfg);
            if firing && step > 100 {
                legacy_fire_jump = legacy_fire_jump.max((fl.pressure - prev_leg).abs());
                refined_fire_jump = refined_fire_jump.max((fr.pressure - prev_ref).abs());
            }
            prev_leg = fl.pressure;
            prev_ref = fr.pressure;
        }
        assert!(
            refined_fire_jump < legacy_fire_jump * 0.15,
            "refined jump {refined_fire_jump} not sharply lower than legacy {legacy_fire_jump}"
        );
    }

    #[test]
    fn refined_2x_analytic_dp_eliminates_hf_shelf_while_preserving_passband() {
        let legacy_cfg = EngineConfig {
            cycle_variation: 0.0,
            use_refined_dsp: false,
            ..EngineConfig::ford_pinto_rs2000(48_000)
        };
        let refined_cfg = EngineConfig {
            cycle_variation: 0.0,
            use_refined_dsp: true,
            ..EngineConfig::ford_pinto_rs2000(48_000)
        };
        let mut legacy = Cylinder::new(0, &legacy_cfg);
        let mut refined = Cylinder::new(0, &refined_cfg);
        let deg_per_sample = 6_800.0 * 6.0 / 48_000.0;
        legacy.fire_at(0.0, 0.0);
        refined.fire_at(0.0, 0.0);

        let mut leg_dp = Vec::with_capacity(8_192);
        let mut ref_dp = Vec::with_capacity(8_192);
        for _ in 0..8_192 {
            if legacy.age_deg >= 720.0 {
                legacy.fire_at(0.0, legacy.age_deg - 720.0);
                refined.fire_at(0.0, refined.age_deg - 720.0);
            }
            leg_dp.push(
                legacy
                    .process(deg_per_sample, 0.9, &legacy_cfg)
                    .pressure_derivative,
            );
            ref_dp.push(
                refined
                    .process(deg_per_sample, 0.9, &refined_cfg)
                    .pressure_derivative,
            );
        }

        let band_power = |buf: &[f32], f_lo: f64, f_hi: f64| {
            let skip = 2_048;
            let n = buf.len() - skip;
            let sr = 48_000.0f64;
            let mut sum = 0.0f64;
            let k_lo = (f_lo * n as f64 / sr).ceil() as usize;
            let k_hi = (f_hi * n as f64 / sr).floor() as usize;
            for k in k_lo..=k_hi {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for (i, &s) in buf[skip..].iter().enumerate() {
                    let w = 0.5 * (1.0 - (std::f64::consts::TAU * i as f64 / (n - 1) as f64).cos());
                    let phi = std::f64::consts::TAU * k as f64 * i as f64 / n as f64;
                    let v = s as f64 * w;
                    re += v * phi.cos();
                    im -= v * phi.sin();
                }
                sum += re * re + im * im;
            }
            sum
        };

        let leg_mid = band_power(&leg_dp, 1_000.0, 5_000.0);
        let ref_mid = band_power(&ref_dp, 1_000.0, 5_000.0);
        let leg_hf = band_power(&leg_dp, 14_000.0, 23_000.0);
        let ref_hf = band_power(&ref_dp, 14_000.0, 23_000.0);

        let mid_ratio = (ref_mid / leg_mid).sqrt();
        assert!(
            (0.85..=1.15).contains(&mid_ratio),
            "1-5 kHz passband gain mismatch: ratio={mid_ratio}"
        );
        let leg_hf_ratio = leg_hf / leg_mid;
        let ref_hf_ratio = ref_hf / ref_mid;
        assert!(
            ref_hf_ratio < leg_hf_ratio * 0.01,
            "14-23 kHz HF shelf not suppressed: legacy={leg_hf_ratio}, refined={ref_hf_ratio}"
        );
    }
}
