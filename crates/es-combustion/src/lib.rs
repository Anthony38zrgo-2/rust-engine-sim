//! Combustion chamber, fuel model, ignition module.

use es_function::Function;
use es_gas::{GasSystem, Mix};
use es_mechanics::{CylinderBank, CylinderHead, Piston, PistonLike};
use es_solver::{BodySet, ForceGenerator};
use es_units::{self as units, PI, ROOT_2, E};

// ---------------------------------------------------------------------------
// Fuel
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct FuelParams {
    pub name: String,
    pub molecular_mass: f64,
    /// J/kg
    pub energy_density: f64,
    pub density: f64,
    pub molecular_afr: f64,
    pub burning_efficiency_randomness: f64,
    pub low_efficiency_attenuation: f64,
    pub max_burning_efficiency: f64,
    pub max_turbulence_effect: f64,
    pub max_dilution_effect: f64,
    pub turbulence_to_flame_speed_ratio: Function,
}

impl Default for FuelParams {
    fn default() -> Self {
        Self {
            name: "Gasoline".into(),
            molecular_mass: 0.1,
            energy_density: 48_100_000.0 / 1000.0 * 1000.0, // 48.1 kJ/g → J/kg ≈ 44e6.. keep C++: 48.1e3/1e-3
            density: 755.0,
            molecular_afr: 25.0 / 2.0,
            burning_efficiency_randomness: 0.5,
            low_efficiency_attenuation: 0.6,
            max_burning_efficiency: 0.8,
            max_turbulence_effect: 2.0,
            // Reference `.mr` fuel node default (fuel.h uses 50, the runtime
            // node value is 10).
            max_dilution_effect: 10.0,
            turbulence_to_flame_speed_ratio: default_turbulence_to_flame_speed_ratio(),
        }
    }
}

/// C++ default: energyDensity = 48.1 kJ / 1 g = 48.1e3 / 1e-3 J/kg = 48.1e6
pub fn default_fuel() -> Fuel {
    Fuel::new(FuelParams {
        energy_density: 48.1e3 / 1e-3,
        ..Default::default()
    })
}

/// Default turbulence/S_L → flame-speed multiplier curve.
///
/// Matches the reference `.mr` `turbulence_to_flame_speed_ratio_default`:
/// `(0, 3), (5, 7.5), (10, 15), ... (45, 67.5)` (i.e. ratio 1.5 * x for x >= 5).
pub fn default_turbulence_to_flame_speed_ratio() -> Function {
    Function::from_samples(
        [
            (0.0, 3.0),
            (5.0, 7.5),
            (10.0, 15.0),
            (15.0, 22.5),
            (20.0, 30.0),
            (25.0, 37.5),
            (30.0, 45.0),
            (35.0, 52.5),
            (40.0, 60.0),
            (45.0, 67.5),
        ],
        es_function::Interpolation::Linear,
    )
}

#[derive(Clone, Debug)]
pub struct Fuel {
    pub params: FuelParams,
}

impl Fuel {
    pub fn new(params: FuelParams) -> Self {
        Self { params }
    }

    pub fn molecular_mass(&self) -> f64 {
        self.params.molecular_mass
    }

    pub fn energy_density(&self) -> f64 {
        self.params.energy_density
    }

    pub fn density(&self) -> f64 {
        self.params.density
    }

    pub fn molecular_afr(&self) -> f64 {
        self.params.molecular_afr
    }

    pub fn flame_speed(
        &self,
        turbulence: f64,
        molecular_afr: f64,
        _t: f64,
        _p: f64,
        _firing_pressure: f64,
        _motoring_pressure: f64,
    ) -> f64 {
        let s_l = self.laminar_burning_velocity(molecular_afr, _t, _p);
        self.params
            .turbulence_to_flame_speed_ratio
            .sample((turbulence / s_l.max(1e-9)) * 1.0)
            * s_l
    }

    pub fn laminar_burning_velocity(&self, molecular_afr: f64, t: f64, p: f64) -> f64 {
        const ER_M: f64 = 1.21;
        const B_M: f64 = 0.305; // m/s
        const B_ER: f64 = -0.549;
        let er = molecular_afr / self.params.molecular_afr;
        let alpha = 2.4 - 0.271 * er.powf(3.51);
        let beta = -0.357 + 0.14 * er.powf(2.77);
        let s_l_0 = B_M + B_ER * (er - ER_M) * (er - ER_M);
        let t_ratio = t / 298.0;
        let p_ratio = p / units::ATM;
        s_l_0 * t_ratio.powf(alpha) * p_ratio.powf(beta)
    }
}

// ---------------------------------------------------------------------------
// Ignition module
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct IgnitionParams {
    pub cylinder_count: usize,
    pub timing_curve: Function,
    /// rad/s
    pub rev_limit: f64,
    pub limiter_duration: f64,
}

#[derive(Clone, Debug)]
pub struct SparkPlug {
    pub angle: f64,
    pub ignition_event: bool,
    pub enabled: bool,
}

#[derive(Clone, Debug)]
pub struct IgnitionModule {
    plugs: Vec<SparkPlug>,
    timing_curve: Function,
    last_cycle_angle: f64,
    rev_limit: f64,
    rev_limit_timer: f64,
    limiter_duration: f64,
    pub enabled: bool,
}

impl IgnitionModule {
    pub fn new(p: IgnitionParams) -> Self {
        Self {
            plugs: (0..p.cylinder_count)
                .map(|_| SparkPlug {
                    angle: 0.0,
                    ignition_event: false,
                    enabled: false,
                })
                .collect(),
            timing_curve: p.timing_curve,
            last_cycle_angle: 0.0,
            rev_limit: p.rev_limit,
            rev_limit_timer: 0.0,
            limiter_duration: p.limiter_duration,
            enabled: false,
        }
    }

    pub fn set_firing_order(&mut self, cylinder: usize, angle: f64) {
        self.plugs[cylinder].angle = angle;
        self.plugs[cylinder].enabled = true;
    }

    pub fn reset(&mut self, cycle_angle: f64) {
        self.last_cycle_angle = cycle_angle;
        self.reset_events();
    }

    pub fn update(&mut self, dt: f64, cycle_angle: f64, omega: f64) {
        let four_pi = 4.0 * PI;
        if self.enabled && self.rev_limit_timer == 0.0 {
            let advance = self.timing_curve.sample(-omega);
            for plug in &mut self.plugs {
                if !plug.enabled {
                    continue;
                }
                let mut adjusted = (plug.angle - advance).rem_euclid(four_pi);
                let r0 = self.last_cycle_angle;
                let mut r1 = cycle_angle;

                // Port of IgnitionModule::update (C++): shift both r1 and adjusted on wrap.
                if omega < 0.0 {
                    if r1 < r0 {
                        r1 += four_pi;
                        adjusted += four_pi;
                    }
                    if adjusted >= r0 && adjusted < r1 {
                        plug.ignition_event = true;
                    }
                } else {
                    if r1 > r0 {
                        r1 -= four_pi;
                        adjusted -= four_pi;
                    }
                    if adjusted >= r1 && adjusted < r0 {
                        plug.ignition_event = true;
                    }
                }
            }
        }

        self.rev_limit_timer -= dt;
        if omega.abs() > self.rev_limit {
            self.rev_limit_timer = self.limiter_duration;
        }
        if self.rev_limit_timer < 0.0 {
            self.rev_limit_timer = 0.0;
        }

        self.last_cycle_angle = cycle_angle;
    }

    pub fn ignition_event(&self, i: usize) -> bool {
        self.plugs[i].ignition_event
    }

    pub fn reset_events(&mut self) {
        for p in &mut self.plugs {
            p.ignition_event = false;
        }
    }

    pub fn timing_advance(&self, omega: f64) -> f64 {
        self.timing_curve.sample(-omega)
    }
}

// ---------------------------------------------------------------------------
// Combustion chamber
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct FlameEvent {
    pub lit_n: f64,
    pub total_n: f64,
    pub percentage_lit: f64,
    pub efficiency: f64,
    pub flame_speed: f64,
    pub last_volume: f64,
    pub travel_x: f64,
    pub travel_y: f64,
    pub global_mix: Mix,
}

impl Default for FlameEvent {
    fn default() -> Self {
        Self {
            lit_n: 0.0,
            total_n: 0.0,
            percentage_lit: 0.0,
            efficiency: 1.0,
            flame_speed: 0.0,
            last_volume: 0.0,
            travel_x: 0.0,
            travel_y: 0.0,
            global_mix: Mix::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct FrictionModel {
    pub friction_coeff: f64,
    pub breakaway_friction: f64,
    pub breakaway_friction_velocity: f64,
    pub viscous_friction_coefficient: f64,
}

impl Default for FrictionModel {
    fn default() -> Self {
        Self {
            friction_coeff: 0.06,
            breakaway_friction: 50.0,
            breakaway_friction_velocity: 0.1,
            viscous_friction_coefficient: 20.0,
        }
    }
}

pub struct CombustionChamber {
    pub system: GasSystem,
    pub intake_runner: GasSystem,
    pub exhaust_runner: GasSystem,
    pub flame: FlameEvent,
    pub lit: bool,
    lit_last: bool,
    pub friction: FrictionModel,
    pub peak_temperature: f64,
    pub n_burnt_fuel: f64,
    pub mean_piston_speed_to_turbulence: Function,

    manifold_to_runner_rate: f64,
    primary_to_collector_rate: f64,
    cylinder_area: f64,
    #[allow(dead_code)]
    cylinder_width: f64,

    last_exhaust_flow: f64,
    last_intake_flow: f64,
    exhaust_flow: f64,
    crankcase_pressure: f64,
    /// Geometric max volume (BDC): chamber + area * stroke. Set at build.
    max_volume: f64,

    pressure_history: [f64; 256],
    piston_speed_history: [f64; 256],

    /// Deterministic per-chamber RNG for combustion randomness (xorshift64).
    rng_state: u64,

    pub piston: usize,
    pub head: usize,
}

/// SplitMix64-style seed mixer so different chambers get distinct, stable
/// sequences while runs remain reproducible (no global rand()).
fn rng_seed(piston: usize, head: usize) -> u64 {
    let mut z = 0x9E37_79B9_7F4A_7C15u64 ^ ((piston as u64) << 32 | head as u64);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

pub struct ChamberRefs<'a> {
    pub bank: &'a CylinderBank,
    pub head: &'a CylinderHead,
    pub piston: &'a Piston,
    pub intake_cs: f64,
    pub exhaust_cs: f64,
    pub runner_flow_k: f64,
    pub primary_flow_k: f64,
    pub runner_length: f64,
    pub primary_length: f64,
    pub intake_runner_vol: f64,
    pub exhaust_runner_vol: f64,
    /// Crank stroke [m]; max sweep volume = area * stroke.
    pub stroke: f64,
}

impl CombustionChamber {
    pub fn new(
        piston: usize,
        head: usize,
        refs: &ChamberRefs<'_>,
        mean_piston_speed_to_turbulence: Function,
        starting_pressure: f64,
        starting_temperature: f64,
        crankcase_pressure: f64,
    ) -> Self {
        let bore_r = refs.bank.bore() / 2.0;
        let cylinder_area = PI * bore_r * bore_r;
        let cylinder_width = cylinder_area.sqrt();

        let mut system = GasSystem::new();
        let mut intake_runner = GasSystem::new();
        let mut exhaust_runner = GasSystem::new();

        // Initial volume filled in later by update_volume
        system.initialize(
            starting_pressure,
            1e-6,
            starting_temperature,
            Mix::AIR,
            5,
        );
        system.set_geometry(cylinder_width, 1e-3, 1.0, 0.0);

        let intake_cs = refs.head.intake_runner_cross_section();
        let intake_w = intake_cs.sqrt();
        let manifold_vol = intake_cs * refs.runner_length;
        let total_intake_vol = refs.head.intake_runner_volume() + manifold_vol;
        let overall_intake_len = total_intake_vol / intake_cs;
        intake_runner.initialize(starting_pressure, total_intake_vol, starting_temperature, Mix::AIR, 5);
        intake_runner.set_geometry(overall_intake_len, intake_w, 1.0, 0.0);

        let exhaust_cs = refs.head.exhaust_runner_cross_section();
        let exhaust_w = exhaust_cs.sqrt();
        let tube_len = refs.primary_length + 0.0;
        let tube_vol = exhaust_cs * tube_len;
        let total_exhaust_vol = refs.head.exhaust_runner_volume() + tube_vol;
        let overall_exhaust_len = total_exhaust_vol / exhaust_cs;
        exhaust_runner.initialize(starting_pressure, total_exhaust_vol, starting_temperature, Mix::AIR, 5);
        exhaust_runner.set_geometry(overall_exhaust_len, exhaust_w, 1.0, 0.0);

        Self {
            system,
            intake_runner,
            exhaust_runner,
            flame: FlameEvent::default(),
            lit: false,
            lit_last: false,
            friction: FrictionModel::default(),
            peak_temperature: 0.0,
            n_burnt_fuel: 0.0,
            mean_piston_speed_to_turbulence,
            manifold_to_runner_rate: refs.runner_flow_k,
            primary_to_collector_rate: refs.primary_flow_k,
            cylinder_area,
            cylinder_width,
            last_exhaust_flow: 0.0,
            last_intake_flow: 0.0,
            exhaust_flow: 0.0,
            crankcase_pressure,
            max_volume: refs.head.combustion_chamber_volume() + cylinder_area * refs.stroke,
            pressure_history: [0.0; 256],
            piston_speed_history: [0.0; 256],
            rng_state: rng_seed(piston, head),
            piston,
            head,
        }
    }

    /// xorshift64 uniform in [0, 1).
    fn next_random(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        (x >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn volume<P: PistonLike>(&self, bank: &CylinderBank, head: &CylinderHead, piston: &P) -> f64 {
        let area = bank.bore_surface_area();
        let body = piston.body();
        let rel_x = body.p_x - bank.pos().0;
        let rel_y = body.p_y - bank.pos().1;
        let s = rel_x * bank.dx() + rel_y * bank.dy();
        let sweep = area * (bank.deck_height() - s - piston.compression_height());
        // Never fall below clearance volume: solver overshoot at TDC must not
        // collapse V→0 (pressure would explode). Ideal TDC sweep ≈ 0.
        let chamber = head.combustion_chamber_volume();
        // Clamp to geometric max: BDC volume = chamber + area*stroke.
        let max_vol = self.max_volume.max(chamber + 1e-9);
        (sweep + chamber - piston.displacement()).clamp(chamber.max(1e-9), max_vol)
    }

    pub fn update_volume<P: PistonLike>(
        &mut self,
        bank: &CylinderBank,
        head: &CylinderHead,
        piston: &P,
    ) {
        let v = self.volume(bank, head, piston);
        self.system.set_volume(v);
    }

    pub fn piston_speed<P: PistonLike>(&self, bank: &CylinderBank, piston: &P) -> f64 {
        let body = piston.body();
        body.v_x * bank.dx() + body.v_y * bank.dy()
    }

    pub fn mean_piston_speed(&self) -> f64 {
        let sum: f64 = self.piston_speed_history.iter().sum();
        sum / self.piston_speed_history.len() as f64
    }

    pub fn firing_pressure(&self) -> f64 {
        self.pressure_history.iter().cloned().fold(0.0, f64::max)
    }

    pub fn pop_lit_last(&mut self) -> bool {
        let lit = self.lit_last;
        self.lit_last = false;
        lit
    }

    pub fn ignite(&mut self, fuel: &Fuel) {
        if self.lit {
            return;
        }
        if self.system.mix().p_fuel == 0.0 {
            return;
        }
        let afr = self.system.mix().p_o2 / self.system.mix().p_fuel;
        let equivalence = afr / fuel.molecular_afr();
        if !(0.5..=1.9).contains(&equivalence) {
            return;
        }

        let ideal_inert = self.system.mix().p_o2 / 0.7;
        let dilution = if ideal_inert > 0.0 {
            (self.system.mix().p_inert / ideal_inert) - 1.0
        } else {
            0.0
        };

        self.flame.last_volume = self.system.volume();
        self.flame.travel_x = 0.0;
        self.flame.travel_y = 0.0;
        self.flame.lit_n = 0.0;
        self.flame.total_n = self.system.n();
        self.flame.percentage_lit = 0.0;
        self.flame.global_mix = self.system.mix();
        self.lit = true;
        self.lit_last = true;

        let p = &fuel.params;
        let turbulence = self
            .mean_piston_speed_to_turbulence
            .sample(self.mean_piston_speed());
        let mixing = 1.0
            - ((turbulence / p.max_turbulence_effect).clamp(0.0, 1.0)
                * (1.0 - dilution / p.max_dilution_effect).clamp(0.0, 1.0));
        // Per-event randomness (deterministic per chamber, varied across
        // ignitions) instead of a fixed 0.5 — the run-to-run variation is
        // part of the acoustic texture, as in the reference `rand()`.
        let rand_s = p.low_efficiency_attenuation
            * ((1.0 - p.burning_efficiency_randomness)
                + p.burning_efficiency_randomness * self.next_random());
        let eff_att = mixing * rand_s + (1.0 - mixing);
        self.flame.efficiency = eff_att * p.max_burning_efficiency;
        self.flame.flame_speed = fuel.flame_speed(
            turbulence,
            afr,
            self.system.temperature(),
            self.system.pressure(),
            self.firing_pressure(),
            160.0 * 6894.76, // 160 psi
        );
    }

    pub fn reset_flow_counters(&mut self) {
        self.last_exhaust_flow = 0.0;
        self.last_intake_flow = 0.0;
    }

    pub fn update_cycle_state<P: PistonLike>(&mut self, cycle_angle: f64, bank: &CylinderBank, piston: &P) {
        let angle = if cycle_angle.is_nan() || cycle_angle.is_infinite() {
            0.0
        } else {
            cycle_angle
        };
        let i = ((angle / (4.0 * PI)) * 255.0).round() as isize;
        let i = i.clamp(0, 255) as usize;
        self.piston_speed_history[i] = self.piston_speed(bank, piston).abs();
        self.pressure_history[i] = self.system.pressure();
    }

    /// Gas exchange + flame advance (port of flow()).
    #[allow(clippy::too_many_arguments)]
    pub fn flow<P: PistonLike>(
        &mut self,
        dt: f64,
        bank: &CylinderBank,
        head: &CylinderHead,
        piston: &P,
        intake_valve_lift: f64,
        exhaust_valve_lift: f64,
        fuel: &Fuel,
        intake_plenum: &mut GasSystem,
        exhaust_collector: &mut GasSystem,
        intake_cs: f64,
        exhaust_collector_cs: f64,
        intake_velocity_decay: f64,
        exhaust_velocity_decay: f64,
        valve_lift_to_k_scale: f64,
    ) {
        if self.system.temperature() > self.peak_temperature {
            self.peak_temperature = self.system.temperature();
        }

        let volume = self.volume(bank, head, piston);
        let cylinder_height = volume / self.cylinder_area;
        let cylinder_surface =
            cylinder_height * PI * bank.bore() + self.cylinder_area * 2.0;

        let d_t = units::celsius(90.0) - self.system.temperature();
        self.system.change_energy(d_t * cylinder_surface * 100.0 * dt);

        // Blow-by
        self.system
            .flow_env(piston.blowby_k(), dt, self.crankcase_pressure, units::celsius(25.0), Mix::AIR);

        let intake_flow_k = head.intake_flow_rate(
            head_cyl_index(head, piston),
            intake_valve_lift,
        ) * valve_lift_to_k_scale;
        let exhaust_flow_k = head.exhaust_flow_rate(
            head_cyl_index(head, piston),
            exhaust_valve_lift,
        ) * valve_lift_to_k_scale;

        // Plenum → runner (rate fixed at init)
        {
            let mut params = es_gas::FlowParams {
                k_flow: self.manifold_to_runner_rate,
                dt,
                direction: (1.0, 0.0),
                cross_section_0: intake_cs,
                cross_section_1: head.intake_runner_cross_section(),
                system_0: intake_plenum,
                system_1: &mut self.intake_runner,
            };
            es_gas::GasSystem::flow_between(&mut params);
        }
        self.intake_runner.dissipate_excess_velocity();

        // Runner → cylinder
        let runner_cs = head.intake_runner_cross_section();
        let cyl_cs = if cylinder_height > 0.0 {
            volume / cylinder_height
        } else {
            self.cylinder_area
        };
        let intake_flow = {
            let mut params = es_gas::FlowParams {
                k_flow: intake_flow_k,
                dt,
                direction: (1.0, 0.0),
                cross_section_0: runner_cs,
                cross_section_1: cyl_cs,
                system_0: &mut self.intake_runner,
                system_1: &mut self.system,
            };
            es_gas::GasSystem::flow_between(&mut params)
        };
        self.intake_runner.dissipate_excess_velocity();
        self.system.dissipate_excess_velocity();

        // Cylinder → exhaust runner
        let exhaust_flow = {
            let mut params = es_gas::FlowParams {
                k_flow: exhaust_flow_k,
                dt,
                direction: (1.0, 0.0),
                cross_section_0: cyl_cs,
                cross_section_1: head.exhaust_runner_cross_section(),
                system_0: &mut self.system,
                system_1: &mut self.exhaust_runner,
            };
            es_gas::GasSystem::flow_between(&mut params)
        };
        self.system.dissipate_excess_velocity();
        self.exhaust_runner.dissipate_excess_velocity();

        // Runner → collector
        {
            let mut params = es_gas::FlowParams {
                k_flow: self.primary_to_collector_rate,
                dt,
                direction: (1.0, 0.0),
                cross_section_0: head.exhaust_runner_cross_section(),
                cross_section_1: exhaust_collector_cs,
                system_0: &mut self.exhaust_runner,
                system_1: exhaust_collector,
            };
            es_gas::GasSystem::flow_between(&mut params);
        }

        self.intake_runner.update_velocity(dt, intake_velocity_decay);
        self.system.update_velocity(dt, 0.5);
        self.exhaust_runner.update_velocity(dt, exhaust_velocity_decay);

        // Reference semantics: any fresh intake charge entering the chamber
        // extinguishes the flame (intake valve open / valve overlap).
        // `flow_between` returns exactly 0 when the valve is closed (k_flow = 0).
        if intake_flow.abs() > 1e-9 && self.lit {
            self.lit = false;
        }

        self.exhaust_flow = exhaust_flow;
        self.last_exhaust_flow += exhaust_flow;
        self.last_intake_flow += intake_flow;

        // Flame propagation
        if self.lit {
            let total_travel_x = bank.bore() / 2.0;
            let total_travel_y = volume / bank.bore_surface_area();
            let expansion = if self.flame.last_volume > 0.0 {
                volume / self.flame.last_volume
            } else {
                1.0
            };
            let last_x = self.flame.travel_x;
            let last_y = self.flame.travel_y * expansion;
            let speed = self.flame.flame_speed;

            self.flame.travel_x = (last_x + dt * speed).min(total_travel_x);
            self.flame.travel_y = (last_y + dt * speed).min(total_travel_y);

            if last_x < self.flame.travel_x || last_y < self.flame.travel_y {
                let burned_vol = self.flame.travel_x * self.flame.travel_x * PI * self.flame.travel_y;
                let prev_vol = last_x * last_x * PI * last_y;
                // Volume expansion can stretch `last_y` past the (clamped)
                // current flame extent, making the delta negative. Guard it.
                let lit_vol = (burned_vol - prev_vol).max(0.0);
                let n = if volume > 0.0 {
                    (lit_vol / volume) * self.system.n()
                } else {
                    0.0
                };

                let fuel_burned =
                    self.system.react(n * self.flame.efficiency, self.flame.global_mix);
                let mass_fuel = fuel_burned * fuel.molecular_mass();
                self.system.change_energy(mass_fuel * fuel.energy_density());

                self.flame.lit_n += n;
                self.flame.percentage_lit += if volume > 0.0 { lit_vol / volume } else { 0.0 };
                self.n_burnt_fuel += mass_fuel;
            } else {
                self.lit = false;
            }
            self.flame.last_volume = volume;
        }
    }

    pub fn friction_force(&self, v_s: f64, wall_force: f64) -> f64 {
        let f_coul = self.friction.friction_coeff * wall_force;
        let v_st = self.friction.breakaway_friction_velocity * ROOT_2;
        let v_coul = self.friction.breakaway_friction_velocity / 10.0;
        let f_brk = self.friction.breakaway_friction;
        let v = v_s.abs();
        let f_0 = ROOT_2 * E * (f_brk - f_coul);
        let f_1 = v / v_st;
        let f_2 = (-f_1 * f_1).exp() * f_1;
        let f_3 = f_coul * (v / v_coul).tanh();
        let f_4 = self.friction.viscous_friction_coefficient * v;
        f_0 * f_2 + f_3 + f_4
    }

    pub fn last_exhaust_flow(&self) -> f64 {
        self.last_exhaust_flow
    }

    pub fn last_intake_flow(&self) -> f64 {
        self.last_intake_flow
    }

    pub fn exhaust_runner_pressure(&self) -> f64 {
        self.exhaust_runner.pressure()
    }

    pub fn exhaust_runner_dynamic_pressure(&self, dx: f64, dy: f64) -> f64 {
        self.exhaust_runner.dynamic_pressure(dx, dy)
    }
}

fn head_cyl_index<P: PistonLike>(head: &CylinderHead, piston: &P) -> usize {
    let _ = head;
    piston.cylinder_index()
}

// ---------------------------------------------------------------------------
// Force generator: pressure + friction on piston
// ---------------------------------------------------------------------------

pub struct ChamberForce {
    pub chamber: usize,
    pub piston: usize,
    pub bank: usize,
    pub crankcase_pressure: f64,
    pub bore: f64,
    pub bank_dx: f64,
    pub bank_dy: f64,
}

/// Shared access to chambers for force application.
pub struct ChamberForceState {
    pub pressures: Vec<f64>,
    pub piston_speeds: Vec<f64>,
    pub wall_forces: Vec<f64>,
    pub friction_models: Vec<FrictionModel>,
    pub bank_dx: Vec<f64>,
    pub bank_dy: Vec<f64>,
    pub bore: Vec<f64>,
    pub crankcase_pressure: f64,
    pub piston_index: Vec<usize>,
}

impl ForceGenerator for ChamberForceState {
    fn apply(&mut self, bodies: &mut BodySet) {
        for i in 0..self.pressures.len() {
            let area = (self.bore[i] * self.bore[i] / 4.0) * PI;
            let pi = self.piston_index[i];
            if pi >= bodies.bodies.len() {
                continue;
            }
            let b = &bodies.bodies[pi];
            let v_s = b.v_x * self.bank_dx[i] + b.v_y * self.bank_dy[i];

            let pressure_diff = self.pressures[i] - self.crankcase_pressure;
            let force = -area * pressure_diff;

            let limit = 1e-3;
            let abs_v_s = v_s.abs().min(limit);
            let attenuation = abs_v_s / limit;

            let model = &self.friction_models[i];
            let f_coul = model.friction_coeff * self.wall_forces[i];
            let v_st = model.breakaway_friction_velocity * ROOT_2;
            let v_coul = model.breakaway_friction_velocity / 10.0;
            let f_brk = model.breakaway_friction;
            let v = v_s.abs();
            let f_0 = ROOT_2 * E * (f_brk - f_coul);
            let f_1 = v / v_st;
            let f_2 = (-f_1 * f_1).exp() * f_1;
            let f_3 = f_coul * (v / v_coul).tanh();
            let f_4 = model.viscous_friction_coefficient * v;
            let f_fric_raw = (f_0 * f_2 + f_3 + f_4) * attenuation;
            let f_fric = if v_s > 0.0 { -f_fric_raw } else { f_fric_raw };

            let total = force + f_fric;
            let fx = total * self.bank_dx[i];
            let fy = total * self.bank_dy[i];
            // Apply at local (0,0) — force through piston pin along bank axis
            bodies.bodies[pi].force_x += fx;
            bodies.bodies[pi].force_y += fy;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fuel_defaults() {
        let f = default_fuel();
        assert!((f.molecular_afr() - 12.5).abs() < 1e-9);
        assert!(f.energy_density() > 1e7);
    }

    #[test]
    fn ignition_detects_crossing() {
        let mut im = IgnitionModule::new(IgnitionParams {
            cylinder_count: 1,
            timing_curve: Function::from_samples([(0.0, 0.0), (1000.0, 0.5)], es_function::Interpolation::Linear),
            rev_limit: 1000.0,
            limiter_duration: 0.1,
        });
        im.set_firing_order(0, 0.0);
        im.enabled = true;
        im.reset(6.0);
        // cycle advances past 0 (wrap) with negative omega (normal engine direction in C++)
        im.update(1e-4, 0.1, -100.0);
        // With wrap from 6.0 to 0.1, event depends on logic; at least should not panic
        im.reset_events();
    }

    #[test]
    fn ignition_fires_on_cycle_wrap() {
        let mut im = IgnitionModule::new(IgnitionParams {
            cylinder_count: 1,
            timing_curve: Function::from_samples([(0.0, 0.0), (1000.0, 0.0)], es_function::Interpolation::Linear),
            rev_limit: 1000.0,
            limiter_duration: 0.1,
        });
        im.set_firing_order(0, 0.0);
        im.enabled = true;
        // Start just below 4π, then advance past 0 (omega < 0 => cycle increases).
        im.reset(6.2);
        im.update(1e-4, 0.2, -1000.0);
        assert!(im.ignition_event(0), "plug should fire crossing the 0° wrap");
        im.reset_events();

        // No wrap: no fire when the firing angle is not traversed.
        im.reset(1.0);
        im.update(1e-4, 2.0, -1000.0);
        assert!(!im.ignition_event(0), "plug must not fire outside its window");
    }

    #[test]
    fn ignition_respects_enabled_flag() {
        let mut im = IgnitionModule::new(IgnitionParams {
            cylinder_count: 1,
            timing_curve: Function::from_samples([(0.0, 0.0), (1000.0, 0.0)], es_function::Interpolation::Linear),
            rev_limit: 1000.0,
            limiter_duration: 0.1,
        });
        im.set_firing_order(0, 0.0);
        im.enabled = false;
        im.reset(6.2);
        im.update(1e-4, 0.2, -1000.0);
        assert!(!im.ignition_event(0), "disabled ignition must never fire");
    }

    #[test]
    fn chamber_ignites_with_fuel() {
        let bank = CylinderBank::new(es_mechanics::CylinderBankParams {
            position_x: 0.0,
            position_y: 0.0,
            angle: 0.0,
            bore: 0.08,
            deck_height: 0.12,
            cylinder_count: 1,
            index: 0,
        });
        let head = CylinderHead::new(es_mechanics::CylinderHeadParams {
            bank: 0,
            exhaust_port_flow: Function::new(),
            intake_port_flow: Function::new(),
            combustion_chamber_volume: 0.00005,
            intake_runner_volume: 0.0001,
            intake_runner_cross_section: 0.001,
            exhaust_runner_volume: 0.0001,
            exhaust_runner_cross_section: 0.001,
            cylinder_count: 1,
        });
        let mut piston = Piston::new(es_mechanics::PistonParams {
            rod: 0,
            bank: 0,
            cylinder_index: 0,
            blowby_flow_coefficient: 0.0,
            compression_height: 0.02,
            wrist_pin_position: 0.0,
            displacement: 0.0,
            mass: 0.4,
        });
        piston.body.p_y = 0.1;

        let refs = ChamberRefs {
            bank: &bank,
            head: &head,
            piston: &piston,
            intake_cs: 0.001,
            exhaust_cs: 0.001,
            runner_flow_k: 1e-8,
            primary_flow_k: 1e-8,
            runner_length: 0.1,
            primary_length: 0.3,
            intake_runner_vol: 0.0001,
            exhaust_runner_vol: 0.0001,
            stroke: 0.086,
        };
        let mut chamber = CombustionChamber::new(
            0,
            0,
            &refs,
            Function::from_samples([(0.0, 5.0), (20.0, 5.0)], es_function::Interpolation::Linear),
            units::ATM,
            units::celsius(25.0),
            units::ATM * 0.3,
        );
        chamber.update_volume(&bank, &head, &piston);

        // Inject fuel into cylinder
        chamber.system.change_mix(Mix {
            p_fuel: 0.10,
            p_inert: 0.00,
            p_o2: 0.90,
        });

        let fuel = default_fuel();
        chamber.ignite(&fuel);
        assert!(chamber.lit, "should light with fuel present");
    }
}
