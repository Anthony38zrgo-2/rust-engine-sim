//! JSON engine configuration → `EngineBuild`.
//!
//! Suffix conventions: `_mm`, `_rpm`, `_deg`, `_nm`, `_cc`, `_kg`, `_m`.

use es_combustion::{
    default_turbulence_to_flame_speed_ratio, Fuel, FuelParams,
};
use es_function::{harmonic_lobe_profile, Function, Interpolation};
use es_mechanics::{CamshaftParams, CylinderHeadParams};
use es_sim::{
    BankConfig, CrankConfig, CylinderConfig, EngineBuild, EngineMeta, IgnitionConfig,
};
use es_units::{self as units, rpm};
use serde::{Deserialize, Serialize};
use std::path::Path;

// ---------------------------------------------------------------------------
// JSON schema
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EngineFile {
    pub name: String,
    #[serde(default)]
    pub meta: MetaFile,
    pub crank: CrankFile,
    pub banks: Vec<BankFile>,
    pub cylinders: Vec<CylinderFile>,
    pub intake: IntakeFile,
    pub exhausts: Vec<ExhaustFile>,
    pub heads: Vec<HeadFile>,
    pub ignition: IgnitionFile,
    #[serde(default)]
    pub cams: CamsFile,
    #[serde(default)]
    pub fuel: FuelFile,
    #[serde(default)]
    pub fluids: FluidsFile,
    #[serde(default)]
    pub scenario: ScenarioFile,
    #[serde(default)]
    pub audio: AudioFile,
}

/// Fuel / combustion properties. Defaults mirror the reference `.mr` `fuel` node.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FuelFile {
    #[serde(default = "d_fuel_name")]
    pub name: String,
    /// Molar mass [g/mol]
    #[serde(default = "d_fuel_molecular_mass_g")]
    pub molecular_mass_g: f64,
    /// Lower heating value [kJ/g]
    #[serde(default = "d_fuel_energy_kj_per_g")]
    pub energy_density_kj_per_g: f64,
    #[serde(default = "d_fuel_density_kg_per_l")]
    pub density_kg_per_l: f64,
    #[serde(default = "d_fuel_afr")]
    pub molecular_afr: f64,
    #[serde(default = "d_fuel_efficiency")]
    pub max_burning_efficiency: f64,
    #[serde(default = "d_fuel_randomness")]
    pub burning_efficiency_randomness: f64,
    #[serde(default = "d_fuel_low_att")]
    pub low_efficiency_attenuation: f64,
    #[serde(default = "d_fuel_turb")]
    pub max_turbulence_effect: f64,
    #[serde(default = "d_fuel_dilution")]
    pub max_dilution_effect: f64,
    /// turbulence/S_L → flame-speed multiplier samples. Empty = reference default.
    #[serde(default)]
    pub turbulence_to_flame_speed_ratio: Vec<[f64; 2]>,
}

fn d_fuel_name() -> String {
    "Gasoline [Default]".into()
}
fn d_fuel_molecular_mass_g() -> f64 {
    100.0
}
fn d_fuel_energy_kj_per_g() -> f64 {
    48.1
}
fn d_fuel_density_kg_per_l() -> f64 {
    0.755
}
fn d_fuel_afr() -> f64 {
    25.0 / 2.0
}
fn d_fuel_efficiency() -> f64 {
    0.8
}
fn d_fuel_randomness() -> f64 {
    0.5
}
fn d_fuel_low_att() -> f64 {
    0.6
}
fn d_fuel_turb() -> f64 {
    2.0
}
fn d_fuel_dilution() -> f64 {
    10.0
}

impl Default for FuelFile {
    fn default() -> Self {
        Self {
            name: d_fuel_name(),
            molecular_mass_g: d_fuel_molecular_mass_g(),
            energy_density_kj_per_g: d_fuel_energy_kj_per_g(),
            density_kg_per_l: d_fuel_density_kg_per_l(),
            molecular_afr: d_fuel_afr(),
            max_burning_efficiency: d_fuel_efficiency(),
            burning_efficiency_randomness: d_fuel_randomness(),
            low_efficiency_attenuation: d_fuel_low_att(),
            max_turbulence_effect: d_fuel_turb(),
            max_dilution_effect: d_fuel_dilution(),
            turbulence_to_flame_speed_ratio: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MetaFile {
    #[serde(default = "d_starter_torque_ft_lb")]
    pub starter_torque_ft_lb: f64,
    #[serde(default = "d_starter_speed_rpm")]
    pub starter_speed_rpm: f64,
    #[serde(default = "d_redline_rpm")]
    pub redline_rpm: f64,
    #[serde(default = "d_dyno_min_rpm")]
    pub dyno_min_rpm: f64,
    #[serde(default = "d_dyno_max_rpm")]
    pub dyno_max_rpm: f64,
    #[serde(default = "d_dyno_hold_rpm")]
    pub dyno_hold_rpm: f64,
    #[serde(default = "d_sim_freq")]
    pub simulation_frequency: f64,
    #[serde(default = "d_fluid_steps")]
    pub fluid_simulation_steps: usize,
}

fn d_starter_torque_ft_lb() -> f64 {
    90.0
}
fn d_starter_speed_rpm() -> f64 {
    200.0
}
fn d_redline_rpm() -> f64 {
    6500.0
}
fn d_dyno_min_rpm() -> f64 {
    1000.0
}
fn d_dyno_max_rpm() -> f64 {
    6500.0
}
fn d_dyno_hold_rpm() -> f64 {
    100.0
}
fn d_sim_freq() -> f64 {
    10_000.0
}
fn d_fluid_steps() -> usize {
    8
}

impl Default for MetaFile {
    fn default() -> Self {
        Self {
            starter_torque_ft_lb: d_starter_torque_ft_lb(),
            starter_speed_rpm: d_starter_speed_rpm(),
            redline_rpm: d_redline_rpm(),
            dyno_min_rpm: d_dyno_min_rpm(),
            dyno_max_rpm: d_dyno_max_rpm(),
            dyno_hold_rpm: d_dyno_hold_rpm(),
            simulation_frequency: d_sim_freq(),
            fluid_simulation_steps: d_fluid_steps(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CrankFile {
    pub mass_kg: f64,
    pub flywheel_mass_kg: f64,
    pub moment_of_inertia_kg_m2: f64,
    pub stroke_mm: f64,
    #[serde(default)]
    pub tdc_deg: f64,
    #[serde(default = "d_crank_friction_nm")]
    pub friction_torque_nm: f64,
    pub rod_journals: usize,
    /// Crankpin angles [deg]
    pub journal_angles_deg: Vec<f64>,
}

fn d_crank_friction_nm() -> f64 {
    5.0
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BankFile {
    pub angle_deg: f64,
    pub bore_mm: f64,
    /// Optional; default = stroke/2 + rod_len + compression_height + clearance
    #[serde(default)]
    pub deck_height_mm: Option<f64>,
    #[serde(default)]
    pub position_x_mm: f64,
    #[serde(default)]
    pub position_y_mm: f64,
    /// Exhaust system index for this bank
    #[serde(default)]
    pub exhaust_system: Option<usize>,
    /// Header primary length [mm]
    #[serde(default)]
    pub primary_length_mm: f64,
    #[serde(default = "d_attenuation")]
    pub sound_attenuation: f64,
}

fn d_attenuation() -> f64 {
    1.0
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CylinderFile {
    pub bank: usize,
    pub bank_cylinder: usize,
    pub rod_length_mm: f64,
    #[serde(default)]
    pub rod_center_of_mass_mm: f64,
    #[serde(default = "d_rod_mass_g")]
    pub rod_mass_g: f64,
    #[serde(default)]
    pub rod_inertia_kg_m2: Option<f64>,
    #[serde(default = "d_piston_mass_g")]
    pub piston_mass_g: f64,
    pub compression_height_mm: f64,
    #[serde(default)]
    pub wrist_pin_position_mm: f64,
    #[serde(default)]
    pub blowby_flow_coefficient: f64,
    #[serde(default)]
    pub journal: usize,
}

fn d_rod_mass_g() -> f64 {
    300.0
}
fn d_piston_mass_g() -> f64 {
    300.0
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IntakeFile {
    #[serde(default = "d_plenum_cc")]
    pub plenum_volume_cc: f64,
    #[serde(default = "d_plenum_cs_cm2")]
    pub cross_section_area_cm2: f64,
    #[serde(default = "d_flow_k")]
    pub input_flow_k: f64,
    #[serde(default)]
    pub idle_flow_k: f64,
    #[serde(default = "d_flow_k")]
    pub runner_flow_k: f64,
    #[serde(default = "d_afr")]
    pub molecular_afr: f64,
    #[serde(default = "d_idle_plate")]
    pub idle_throttle_plate_position: f64,
    #[serde(default = "d_runner_mm")]
    pub runner_length_mm: f64,
    #[serde(default = "d_decay")]
    pub velocity_decay: f64,
}

fn d_plenum_cc() -> f64 {
    1000.0
}
fn d_plenum_cs_cm2() -> f64 {
    20.0
}
fn d_flow_k() -> f64 {
    // ~k_carb(500 scfm)
    es_gas::GasSystem::k_carb(500.0)
}
fn d_afr() -> f64 {
    12.5
}
fn d_idle_plate() -> f64 {
    0.99
}
fn d_runner_mm() -> f64 {
    100.0
}
fn d_decay() -> f64 {
    0.5
}

impl Default for IntakeFile {
    fn default() -> Self {
        Self {
            plenum_volume_cc: d_plenum_cc(),
            cross_section_area_cm2: d_plenum_cs_cm2(),
            input_flow_k: d_flow_k(),
            idle_flow_k: 0.0,
            runner_flow_k: d_flow_k(),
            molecular_afr: d_afr(),
            idle_throttle_plate_position: d_idle_plate(),
            runner_length_mm: d_runner_mm(),
            velocity_decay: d_decay(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExhaustFile {
    #[serde(default = "d_ex_length_mm")]
    pub length_mm: f64,
    #[serde(default = "d_ex_cs_cm2")]
    pub collector_cross_section_cm2: f64,
    #[serde(default = "d_flow_k")]
    pub outlet_flow_k: f64,
    #[serde(default = "d_primary_mm")]
    pub primary_tube_length_mm: f64,
    #[serde(default = "d_flow_k")]
    pub primary_flow_k: f64,
    #[serde(default = "d_ex_decay")]
    pub velocity_decay: f64,
    #[serde(default = "d_audio_vol")]
    pub audio_volume: f64,
    /// Path relative to assets root
    #[serde(default)]
    pub impulse_response: Option<String>,
    /// Impulse-response amplitude scale (reference IR library uses ~0.001–0.01).
    #[serde(default = "d_ir_volume")]
    pub impulse_response_volume: f64,
}

fn d_ir_volume() -> f64 {
    0.001
}

fn d_ex_length_mm() -> f64 {
    2500.0
}
fn d_ex_cs_cm2() -> f64 {
    50.0
}
fn d_primary_mm() -> f64 {
    300.0
}
fn d_ex_decay() -> f64 {
    1.0
}
fn d_audio_vol() -> f64 {
    1.0
}

impl Default for ExhaustFile {
    fn default() -> Self {
        Self {
            length_mm: d_ex_length_mm(),
            collector_cross_section_cm2: d_ex_cs_cm2(),
            outlet_flow_k: d_flow_k(),
            primary_tube_length_mm: d_primary_mm(),
            primary_flow_k: d_flow_k(),
            velocity_decay: d_ex_decay(),
            audio_volume: d_audio_vol(),
            impulse_response: None,
            impulse_response_volume: d_ir_volume(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HeadFile {
    pub bank: usize,
    #[serde(default = "d_chamber_cc")]
    pub combustion_chamber_volume_cc: f64,
    #[serde(default = "d_runner_vol_cc")]
    pub intake_runner_volume_cc: f64,
    #[serde(default = "d_runner_cs_cm2")]
    pub intake_runner_cross_section_cm2: f64,
    #[serde(default = "d_runner_vol_cc")]
    pub exhaust_runner_volume_cc: f64,
    #[serde(default = "d_ex_runner_cs_cm2")]
    pub exhaust_runner_cross_section_cm2: f64,
    pub cylinder_count: usize,
    /// Flow samples as [lift_thou, flow] pairs
    #[serde(default)]
    pub intake_flow: Vec<[f64; 2]>,
    #[serde(default)]
    pub exhaust_flow: Vec<[f64; 2]>,
    /// Per-cylinder exhaust system override
    #[serde(default)]
    pub cylinder_exhaust: Vec<Option<usize>>,
    #[serde(default)]
    pub cylinder_primary_mm: Vec<f64>,
    #[serde(default)]
    pub cylinder_attenuation: Vec<f64>,
}

fn d_chamber_cc() -> f64 {
    45.0
}
fn d_runner_vol_cc() -> f64 {
    100.0
}
fn d_runner_cs_cm2() -> f64 {
    10.0
}
fn d_ex_runner_cs_cm2() -> f64 {
    8.0
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IgnitionFile {
    /// (cylinder, crank_deg_in_cycle 0..720)
    pub firing_order: Vec<(usize, f64)>,
    /// [[rpm, advance_deg], ...]
    pub timing_curve_rpm_deg: Vec<[f64; 2]>,
    pub rev_limit_rpm: f64,
    #[serde(default = "d_limiter_s")]
    pub limiter_duration_s: f64,
}

fn d_limiter_s() -> f64 {
    0.05
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CamsFile {
    #[serde(default = "d_intake_duration")]
    pub intake_duration_deg: f64,
    #[serde(default = "d_exhaust_duration")]
    pub exhaust_duration_deg: f64,
    #[serde(default = "d_intake_lift_mm")]
    pub intake_lift_mm: f64,
    #[serde(default = "d_exhaust_lift_mm")]
    pub exhaust_lift_mm: f64,
    #[serde(default = "d_gamma")]
    pub gamma: f64,
    #[serde(default = "d_base_radius_mm")]
    pub base_radius_mm: f64,
    #[serde(default)]
    pub intake_advance_deg: f64,
    #[serde(default = "d_ex_adv")]
    pub exhaust_advance_deg: f64,
    /// Intake lobe centerline (crank degrees after fire angle).
    /// 450° = peak at mid-intake stroke.
    #[serde(default = "d_intake_centerline")]
    pub intake_centerline_deg: f64,
    /// Exhaust lobe centerline (crank degrees after fire angle).
    /// 270° = peak at mid-exhaust stroke.
    #[serde(default = "d_exhaust_centerline")]
    pub exhaust_centerline_deg: f64,
}

fn d_intake_duration() -> f64 {
    250.0
}
fn d_exhaust_duration() -> f64 {
    255.0
}
fn d_intake_lift_mm() -> f64 {
    10.0
}
fn d_exhaust_lift_mm() -> f64 {
    9.5
}
fn d_gamma() -> f64 {
    2.0
}
fn d_base_radius_mm() -> f64 {
    15.0
}
fn d_ex_adv() -> f64 {
    0.0
}

fn d_intake_centerline() -> f64 {
    450.0
}
fn d_exhaust_centerline() -> f64 {
    270.0
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FluidsFile {
    #[serde(default = "d_start_p")]
    pub starting_pressure_pa: f64,
    #[serde(default = "d_start_t_c")]
    pub starting_temperature_c: f64,
    #[serde(default = "d_crankcase")]
    pub crankcase_pressure_pa: f64,
}

fn d_start_p() -> f64 {
    units::ATM
}
fn d_start_t_c() -> f64 {
    25.0
}
fn d_crankcase() -> f64 {
    units::ATM * 0.3
}

impl Default for FluidsFile {
    fn default() -> Self {
        Self {
            starting_pressure_pa: d_start_p(),
            starting_temperature_c: d_start_t_c(),
            crankcase_pressure_pa: d_crankcase(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScenarioEventFile {
    pub time_s: f64,
    pub action: String,
    #[serde(default)]
    pub value: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScenarioFile {
    pub duration_s: f64,
    #[serde(default)]
    pub events: Vec<ScenarioEventFile>,
}

impl Default for ScenarioFile {
    fn default() -> Self {
        Self {
            duration_s: 3.0,
            events: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AudioFile {
    #[serde(default = "d_audio_rate")]
    pub sample_rate: f64,
    #[serde(default = "d_out_path")]
    pub output: String,
    #[serde(default)]
    pub volume: f32,
    #[serde(default = "d_conv")]
    pub convolution: f32,
    #[serde(default = "d_dff")]
    pub d_f_f_mix: f32,
    #[serde(default = "d_jitter")]
    pub input_sample_noise: f32,
    #[serde(default = "d_air")]
    pub air_noise: f32,
    #[serde(default = "d_air_fc")]
    pub air_noise_frequency_cutoff: f32,
    #[serde(default = "d_in_fc")]
    pub input_sample_noise_frequency_cutoff: f32,
    #[serde(default)]
    pub leveler_target: Option<f32>,
    #[serde(default)]
    pub leveler_max_gain: Option<f32>,
}

fn d_audio_rate() -> f64 {
    44_100.0
}
fn d_out_path() -> String {
    "output.wav".into()
}
fn d_conv() -> f32 {
    1.0
}
fn d_dff() -> f32 {
    0.01
}
fn d_jitter() -> f32 {
    0.5
}
fn d_air() -> f32 {
    1.0
}
fn d_air_fc() -> f32 {
    2000.0
}
fn d_in_fc() -> f32 {
    10_000.0
}

impl Default for AudioFile {
    fn default() -> Self {
        Self {
            sample_rate: d_audio_rate(),
            output: d_out_path(),
            volume: 1.0,
            convolution: d_conv(),
            d_f_f_mix: d_dff(),
            input_sample_noise: d_jitter(),
            air_noise: d_air(),
            air_noise_frequency_cutoff: d_air_fc(),
            input_sample_noise_frequency_cutoff: d_in_fc(),
            leveler_target: None,
            leveler_max_gain: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Load + convert
// ---------------------------------------------------------------------------

pub fn load_path<P: AsRef<Path>>(path: P) -> Result<EngineFile, String> {
    let text = std::fs::read_to_string(path.as_ref()).map_err(|e| e.to_string())?;
    serde_json::from_str(&text).map_err(|e| e.to_string())
}

pub fn load_str(text: &str) -> Result<EngineFile, String> {
    serde_json::from_str(text).map_err(|e| e.to_string())
}

fn flow_function(samples: &[[f64; 2]], default: bool) -> Function {
    // Port-flow samples are [lift_thou, flow_scfm] as in original add_flow_sample → k_28inH2O(flow).
    // Values already in orifice-k range (tiny) are left as-is.
    let to_k = |y: f64| {
        if y == 0.0 {
            0.0
        } else if y > 1.0 {
            // treat as scfm (bench flow)
            es_gas::GasSystem::k_28in_h2o(y)
        } else {
            y
        }
    };
    if samples.is_empty() {
        if default {
            // Default: rises with lift (thou → scfm → k_28inH2O)
            return Function::from_samples(
                [
                    (0.0, 0.0),
                    (0.001, es_gas::GasSystem::k_28in_h2o(50.0)),
                    (0.002, es_gas::GasSystem::k_28in_h2o(100.0)),
                    (0.003, es_gas::GasSystem::k_28in_h2o(150.0)),
                    (0.005, es_gas::GasSystem::k_28in_h2o(250.0)),
                ],
                Interpolation::Linear,
            );
        }
        return Function::from_samples([(0.0, 0.0)], Interpolation::Linear);
    }
    // Convert thou → m for x if values look like thou (0..500), else assume meters
    let mut pts = Vec::with_capacity(samples.len());
    for s in samples {
        let (x, y) = (s[0], s[1]);
        let x_m = if x > 0.05 {
            // treat as thou
            units::inch(x / 1000.0)
        } else {
            x
        };
        pts.push((x_m, to_k(y)));
    }
    Function::from_samples(pts, Interpolation::Linear)
}

fn default_rod_inertia(mass_kg: f64, length_m: f64) -> f64 {
    // slender rod approximation
    mass_kg * length_m * length_m / 12.0
}

impl EngineFile {
    pub fn to_build(&self) -> EngineBuild {
        let meta = EngineMeta {
            name: self.name.clone(),
            starter_torque: units::ft_lb(self.meta.starter_torque_ft_lb),
            starter_speed: rpm(self.meta.starter_speed_rpm),
            redline: rpm(self.meta.redline_rpm),
            dyno_min_speed: rpm(self.meta.dyno_min_rpm),
            dyno_max_speed: rpm(self.meta.dyno_max_rpm),
            dyno_hold_step: rpm(self.meta.dyno_hold_rpm),
            simulation_frequency: self.meta.simulation_frequency,
            fluid_simulation_steps: self.meta.fluid_simulation_steps.max(1),
        };

        let stroke_m = units::mm(self.crank.stroke_mm);
        let throw_m = stroke_m * 0.5;

        // Derive deck height from first cylinder if needed
        let default_deck = {
            let c0 = &self.cylinders[0];
            let rod = units::mm(c0.rod_length_mm);
            let ch = units::mm(c0.compression_height_mm);
            // Matches reference .mr: deck = stroke/2 + rod + compression_height
            // (clearance volume is only combustion_chamber_volume at TDC).
            rod + stroke_m * 0.5 + ch
        };

        let banks: Vec<BankConfig> = self
            .banks
            .iter()
            .map(|b| {
                let deck = b
                    .deck_height_mm
                    .map(units::mm)
                    .unwrap_or(default_deck);
                BankConfig {
                    angle: units::deg(b.angle_deg),
                    bore: units::mm(b.bore_mm),
                    deck_height: deck,
                    position_x: units::mm(b.position_x_mm),
                    position_y: units::mm(b.position_y_mm),
                    cylinders: vec![],
                }
            })
            .collect();

        // Group cylinder indices per bank
        let mut banks = banks;
        for (i, c) in self.cylinders.iter().enumerate() {
            if let Some(bank) = banks.get_mut(c.bank) {
                bank.cylinders.push(i);
            }
        }

        let cylinders: Vec<CylinderConfig> = self
            .cylinders
            .iter()
            .map(|c| {
                let rod_len = units::mm(c.rod_length_mm);
                let rod_mass = c.rod_mass_g / 1000.0;
                CylinderConfig {
                    bank: c.bank,
                    bank_cylinder: c.bank_cylinder,
                    rod_length: rod_len,
                    rod_center_of_mass: units::mm(c.rod_center_of_mass_mm),
                    rod_mass,
                    rod_inertia: c
                        .rod_inertia_kg_m2
                        .unwrap_or_else(|| default_rod_inertia(rod_mass, rod_len)),
                    piston_mass: c.piston_mass_g / 1000.0,
                    compression_height: units::mm(c.compression_height_mm),
                    wrist_pin_position: units::mm(c.wrist_pin_position_mm),
                    blowby_flow_coefficient: c.blowby_flow_coefficient,
                    displacement: 0.0,
                    journal: c.journal,
                }
            })
            .collect();

        let crank = CrankConfig {
            mass: self.crank.mass_kg,
            flywheel_mass: self.crank.flywheel_mass_kg,
            moment_of_inertia: self.crank.moment_of_inertia_kg_m2,
            crank_throw: throw_m,
            stroke: stroke_m,
            pos_x: 0.0,
            pos_y: 0.0,
            tdc: units::deg(self.crank.tdc_deg),
            friction_torque: self.crank.friction_torque_nm,
            rod_journals: self.crank.rod_journals,
            journal_angles_deg: self.crank.journal_angles_deg.clone(),
        };

        let intake = self.intake.clone().into();
        let exhausts: Vec<_> = self.exhausts.iter().map(|e| e.clone().into()).collect();

        let heads: Vec<CylinderHeadParams> = self
            .heads
            .iter()
            .map(|h| CylinderHeadParams {
                bank: h.bank,
                exhaust_port_flow: flow_function(&h.exhaust_flow, true),
                intake_port_flow: flow_function(&h.intake_flow, true),
                combustion_chamber_volume: units::cc(h.combustion_chamber_volume_cc),
                intake_runner_volume: units::cc(h.intake_runner_volume_cc),
                intake_runner_cross_section: h.intake_runner_cross_section_cm2 * 1e-4,
                exhaust_runner_volume: units::cc(h.exhaust_runner_volume_cc),
                exhaust_runner_cross_section: h.exhaust_runner_cross_section_cm2 * 1e-4,
                cylinder_count: h.cylinder_count,
            })
            .collect();

        let chamber_flow: Vec<(f64, f64, f64, f64, f64, f64)> = self
            .cylinders
            .iter()
            .map(|c| {
                let head = self.heads.get(c.bank).or_else(|| self.heads.first()).unwrap();
                let intake_cs = head.intake_runner_cross_section_cm2 * 1e-4;
                let exhaust_cs = head.exhaust_runner_cross_section_cm2 * 1e-4;
                let runner_len = units::mm(self.intake.runner_length_mm.max(1.0));
                let primary_len = self
                    .exhausts
                    .first()
                    .map(|e| units::mm(e.primary_tube_length_mm))
                    .unwrap_or(0.3);
                let primary_flow_k = self
                    .exhausts
                    .first()
                    .map(|e| e.primary_flow_k)
                    .unwrap_or_else(d_flow_k);
                (
                    self.intake.runner_flow_k,
                    primary_flow_k,
                    runner_len,
                    primary_len,
                    intake_cs,
                    exhaust_cs,
                )
            })
            .collect();

        let timing_curve = Function::from_samples(
            self.ignition
                .timing_curve_rpm_deg
                .iter()
                .map(|s| (rpm(s[0]), units::deg(s[1]))),
            Interpolation::Linear,
        );

        let ignition = IgnitionConfig {
            firing_order: self
                .ignition
                .firing_order
                .iter()
                .map(|&(c, a)| (c, units::deg(a)))
                .collect(),
            timing_curve,
            rev_limit: rpm(self.ignition.rev_limit_rpm),
            limiter_duration: self.ignition.limiter_duration_s,
        };

        let intake_lobe = harmonic_lobe_profile(
            self.cams.intake_duration_deg,
            units::mm(self.cams.intake_lift_mm) * 0.50,
            units::mm(self.cams.intake_lift_mm),
            self.cams.gamma,
            128,
        );
        let exhaust_lobe = harmonic_lobe_profile(
            self.cams.exhaust_duration_deg,
            units::mm(self.cams.exhaust_lift_mm) * 0.50,
            units::mm(self.cams.exhaust_lift_mm),
            self.cams.gamma,
            128,
        );

        let n_cyl = self.cylinders.len();
        let cam_assignment: Vec<(usize, usize, usize)> =
            (0..n_cyl).map(|i| (i, i, i)).collect();

        let intake_cam_params = CamshaftParams {
            lobes: n_cyl,
            advance: units::deg(self.cams.intake_advance_deg),
            crankshaft: 0,
            lobe_profile: intake_lobe,
            base_radius: units::mm(self.cams.base_radius_mm),
        };
        let exhaust_cam_params = CamshaftParams {
            lobes: n_cyl,
            advance: units::deg(self.cams.exhaust_advance_deg),
            crankshaft: 0,
            lobe_profile: exhaust_lobe,
            base_radius: units::mm(self.cams.base_radius_mm),
        };

        let bank_exhaust: Vec<(usize, f64)> = self
            .banks
            .iter()
            .enumerate()
            .map(|(i, b)| {
                let sys = b.exhaust_system.unwrap_or(i.min(self.exhausts.len().saturating_sub(1)));
                (sys, units::mm(b.primary_length_mm))
            })
            .collect();

        let bank_sound_attenuation: Vec<f64> =
            self.banks.iter().map(|b| b.sound_attenuation).collect();

        // Fuel / combustion model (configurable, defaults mirror the .mr fuel node).
        let fuel = Fuel::new(FuelParams {
            name: self.fuel.name.clone(),
            molecular_mass: self.fuel.molecular_mass_g / 1000.0,
            energy_density: self.fuel.energy_density_kj_per_g * 1e6,
            density: self.fuel.density_kg_per_l,
            molecular_afr: self.fuel.molecular_afr,
            burning_efficiency_randomness: self.fuel.burning_efficiency_randomness,
            low_efficiency_attenuation: self.fuel.low_efficiency_attenuation,
            max_burning_efficiency: self.fuel.max_burning_efficiency,
            max_turbulence_effect: self.fuel.max_turbulence_effect,
            max_dilution_effect: self.fuel.max_dilution_effect,
            turbulence_to_flame_speed_ratio: if self
                .fuel
                .turbulence_to_flame_speed_ratio
                .is_empty()
            {
                default_turbulence_to_flame_speed_ratio()
            } else {
                Function::from_samples(
                    self.fuel
                        .turbulence_to_flame_speed_ratio
                        .iter()
                        .map(|s| (s[0], s[1])),
                    Interpolation::Linear,
                )
            },
        });

        // Override per-cylinder exhaust if specified on head
        // (EngineBuild only has bank-level; per-cyl overrides applied post-build in render)

        EngineBuild {
            meta,
            crank,
            banks,
            cylinders,
            intake,
            exhausts,
            heads,
            chamber_flow,
            ignition,
            fuel,
            intake_cam_params,
            exhaust_cam_params,
            intake_centerline: units::deg(self.cams.intake_centerline_deg),
            exhaust_centerline: units::deg(self.cams.exhaust_centerline_deg),
            cam_assignment,
            // Reference `engine_node.h`: turbulence = 0.5 * mean piston speed.
            mean_piston_speed_to_turbulence: Function::from_samples(
                [(0.0, 0.0), (30.0, 15.0)],
                Interpolation::Linear,
            ),
            starting_pressure: self.fluids.starting_pressure_pa,
            starting_temperature: units::celsius(self.fluids.starting_temperature_c),
            crankcase_pressure: self.fluids.crankcase_pressure_pa,
            bank_exhaust,
            bank_sound_attenuation,
        }
    }
}

impl From<IntakeFile> for es_intake_exhaust::IntakeParams {
    fn from(f: IntakeFile) -> Self {
        es_intake_exhaust::IntakeParams {
            volume: units::cc(f.plenum_volume_cc),
            cross_section_area: f.cross_section_area_cm2 * 1e-4,
            input_flow_k: f.input_flow_k,
            idle_flow_k: f.idle_flow_k,
            runner_flow_rate: f.runner_flow_k,
            molecular_afr: f.molecular_afr,
            idle_throttle_plate_position: f.idle_throttle_plate_position,
            runner_length: units::mm(f.runner_length_mm),
            velocity_decay: f.velocity_decay,
        }
    }
}

impl From<ExhaustFile> for es_intake_exhaust::ExhaustParams {
    fn from(f: ExhaustFile) -> Self {
        es_intake_exhaust::ExhaustParams {
            length: units::mm(f.length_mm),
            collector_cross_section: f.collector_cross_section_cm2 * 1e-4,
            outlet_flow_rate: f.outlet_flow_k,
            primary_tube_length: units::mm(f.primary_tube_length_mm),
            primary_flow_rate: f.primary_flow_k,
            velocity_decay: f.velocity_decay,
            audio_volume: f.audio_volume,
            impulse_response: f.impulse_response,
        }
    }
}

// Helper trait removed — keep module clean

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_minimal_json() {
        let json = r#"{
            "name": "test",
            "crank": {
                "mass_kg": 5.0,
                "flywheel_mass_kg": 2.0,
                "moment_of_inertia_kg_m2": 0.2,
                "stroke_mm": 86.0,
                "rod_journals": 1,
                "journal_angles_deg": [0.0]
            },
            "banks": [{"angle_deg": 0.0, "bore_mm": 86.0}],
            "cylinders": [{
                "bank": 0,
                "bank_cylinder": 0,
                "rod_length_mm": 140.0,
                "compression_height_mm": 20.0
            }],
            "intake": {},
            "exhausts": [{}],
            "heads": [{"bank": 0, "cylinder_count": 1}],
            "ignition": {
                "firing_order": [[0, 0.0]],
                "timing_curve_rpm_deg": [[0, 0], [4000, 24]],
                "rev_limit_rpm": 5500
            }
        }"#;
        let f = load_str(json).unwrap();
        let build = f.to_build();
        assert_eq!(build.cylinders.len(), 1);
        assert_eq!(build.banks.len(), 1);
        assert!(build.crank.crank_throw > 0.0);
        // deck height critical for positive volume
        assert!(build.banks[0].deck_height > build.crank.crank_throw);
    }
}
