//! 0-D gas system: ideal gas state, compressible orifice flow with choke,
//! bulk momentum, and a simplified hydrocarbon reaction.
//!
//! Port of `GasSystem` from the original engine-sim C++ codebase.

use es_units::{self as units, R};

/// Mixture mole fractions (p_fuel + p_inert + p_o2 = 1).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mix {
    pub p_fuel: f64,
    pub p_inert: f64,
    pub p_o2: f64,
}

impl Default for Mix {
    fn default() -> Self {
        Self {
            p_fuel: 0.0,
            p_inert: 1.0,
            p_o2: 0.0,
        }
    }
}

impl Mix {
    pub const AIR: Mix = Mix {
        p_fuel: 0.0,
        p_inert: 0.79,
        p_o2: 0.21,
    };

    pub fn normalize(&mut self) {
        let s = self.p_fuel + self.p_inert + self.p_o2;
        if s > 0.0 {
            self.p_fuel /= s;
            self.p_inert /= s;
            self.p_o2 /= s;
        }
    }
}

#[derive(Clone, Debug)]
struct State {
    n_mol: f64,
    e_k: f64,
    v: f64,
    momentum: [f64; 2],
    mix: Mix,
}

impl Default for State {
    fn default() -> Self {
        Self {
            n_mol: 0.0,
            e_k: 0.0,
            v: 0.0,
            momentum: [0.0, 0.0],
            mix: Mix::default(),
        }
    }
}

/// Geometry used only for bulk-velocity wall-friction updates.
#[derive(Clone, Copy, Debug, Default)]
pub struct Geometry {
    pub width: f64,
    pub height: f64,
    pub dx: f64,
    pub dy: f64,
}

/// Two-system flow parameters (orifice between control volumes).
pub struct FlowParams<'a> {
    pub k_flow: f64,
    pub dt: f64,
    pub direction: (f64, f64),
    pub cross_section_0: f64,
    pub cross_section_1: f64,
    pub system_0: &'a mut GasSystem,
    pub system_1: &'a mut GasSystem,
}

#[derive(Clone, Debug)]
pub struct GasSystem {
    state: State,
    dof: i32,
    choked_flow_limit: f64,
    choked_flow_factor: f64,
    geometry: Geometry,
}

impl Default for GasSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl GasSystem {
    pub fn new() -> Self {
        Self {
            state: State::default(),
            dof: 5,
            choked_flow_limit: choked_flow_limit(5),
            choked_flow_factor: choked_flow_rate(5),
            geometry: Geometry::default(),
        }
    }

    pub fn set_geometry(&mut self, width: f64, height: f64, dx: f64, dy: f64) {
        self.geometry = Geometry {
            width,
            height,
            dx,
            dy,
        };
    }

    pub fn initialize(&mut self, p: f64, v: f64, t: f64, mix: Mix, degrees_of_freedom: i32) {
        self.dof = degrees_of_freedom;
        self.state.n_mol = p * v / (R * t);
        self.state.v = v;
        self.state.e_k = t * (0.5 * degrees_of_freedom as f64 * self.state.n_mol * R);
        self.state.mix = mix;
        self.state.momentum = [0.0, 0.0];
        self.choked_flow_limit = choked_flow_limit(degrees_of_freedom);
        self.choked_flow_factor = choked_flow_rate(degrees_of_freedom);
    }

    pub fn reset(&mut self, p: f64, t: f64, mix: Mix) {
        self.state.n_mol = p * self.volume() / (R * t);
        self.state.e_k = t * (0.5 * self.dof as f64 * self.state.n_mol * R);
        self.state.mix = mix;
        self.state.momentum = [0.0, 0.0];
    }

    pub fn set_volume(&mut self, v: f64) {
        self.change_volume(v - self.state.v);
    }

    pub fn set_n(&mut self, n: f64) {
        let ek_per = if self.state.n_mol != 0.0 {
            self.state.e_k / self.state.n_mol
        } else {
            0.0
        };
        self.state.e_k = ek_per * n;
        self.state.n_mol = n;
    }

    pub fn change_volume(&mut self, dv: f64) {
        let v = self.volume();
        let l = (v + dv).max(0.0).powf(1.0 / 3.0);
        let surface = l * l;
        let dl = if surface > 0.0 { -dv / surface } else { 0.0 };
        let w = dl * self.pressure() * surface;
        self.state.v += dv;
        self.state.e_k += w;
    }

    pub fn change_pressure(&mut self, dp: f64) {
        self.state.e_k += dp * self.volume() * self.dof as f64 * 0.5;
    }

    pub fn change_temperature(&mut self, dt: f64) {
        self.state.e_k += dt * 0.5 * self.dof as f64 * self.n() * R;
    }

    pub fn change_temperature_n(&mut self, dt: f64, n: f64) {
        self.state.e_k += dt * 0.5 * self.dof as f64 * n * R;
    }

    pub fn change_energy(&mut self, de: f64) {
        if !de.is_finite() {
            return;
        }
        self.state.e_k += de;
        if !self.state.e_k.is_finite() {
            self.state.e_k = 0.0;
        }
    }

    pub fn change_mix(&mut self, mix: Mix) {
        self.state.mix = mix;
    }

    pub fn inject_fuel(&mut self, n: f64) {
        let n_total = self.n();
        if n_total <= 0.0 {
            return;
        }
        let n_fuel = self.n_fuel() + n;
        self.state.mix.p_fuel = n_fuel / n_total;
    }

    /// Burn `n` moles of the given local mix. Returns moles of fuel actually burned.
    pub fn react(&mut self, n: f64, mix: Mix) -> f64 {
        let l_n_fuel = mix.p_fuel * n;
        let l_n_o2 = mix.p_o2 * n;

        let system_n_fuel = self.n_fuel();
        let system_n_o2 = self.n_o2();
        let system_n_inert = self.n_inert();
        let system_n = self.n();

        // 25[O2] + 2[C8H16] -> 16[CO2] + 18[H2O]
        const IDEAL_O2_RATIO: f64 = 25.0 / 2.0;
        const IDEAL_FUEL_RATIO: f64 = 2.0 / 25.0;
        const OUTPUT_INPUT_RATIO: f64 = (16.0 + 18.0) / (25.0 + 2.0);

        let ideal_fuel_n = IDEAL_FUEL_RATIO * l_n_o2;
        let ideal_o2_n = IDEAL_O2_RATIO * l_n_fuel;

        let a_n_fuel = system_n_fuel.min(l_n_fuel).min(ideal_fuel_n);
        let a_n_o2 = system_n_o2.min(l_n_o2).min(ideal_o2_n);

        let reactants_n = a_n_fuel + a_n_o2;
        let products_n = OUTPUT_INPUT_RATIO * reactants_n;
        let dn = products_n - reactants_n;

        self.state.n_mol += dn;

        let new_fuel = system_n_fuel - a_n_fuel;
        let new_o2 = system_n_o2 - a_n_o2;
        let new_inert = system_n_inert + products_n;
        let new_n = system_n + dn;

        if new_n != 0.0 {
            self.state.mix.p_fuel = new_fuel / new_n;
            self.state.mix.p_inert = new_inert / new_n;
            self.state.mix.p_o2 = new_o2 / new_n;
        } else {
            self.state.mix = Mix {
                p_fuel: 0.0,
                p_inert: 0.0,
                p_o2: 0.0,
            };
        }

        a_n_fuel
    }

    /// Orifice constant that yields `target_flow_rate` at the given bench condition.
    pub fn flow_constant(
        target_flow_rate: f64,
        p: f64,
        pressure_drop: f64,
        t: f64,
        hcr: f64,
    ) -> f64 {
        let t0 = t;
        let p0 = p;
        let p_t = p - pressure_drop;

        let choked = (2.0 / (hcr + 1.0)).powf(hcr / (hcr - 1.0));
        let p_ratio = p_t / p0;

        let mut flow_rate = if p_ratio <= choked {
            (hcr.sqrt()) * (2.0 / (hcr + 1.0)).powf((hcr + 1.0) / (2.0 * (hcr - 1.0)))
        } else {
            let mut fr = (2.0 * hcr) / (hcr - 1.0);
            fr *= 1.0 - p_ratio.powf((hcr - 1.0) / hcr);
            fr = fr.sqrt();
            fr * p_ratio.powf(1.0 / hcr)
        };

        flow_rate *= p0 / (R * t0).sqrt();
        target_flow_rate / flow_rate
    }

    pub fn k_28in_h2o(flow_rate_scfm: f64) -> f64 {
        Self::flow_constant(
            units::scfm(flow_rate_scfm),
            units::ATM,
            28.0 * units::IN_H2O,
            units::celsius(25.0),
            heat_capacity_ratio(5),
        )
    }

    pub fn k_carb(flow_rate_scfm: f64) -> f64 {
        Self::flow_constant(
            units::scfm(flow_rate_scfm),
            units::ATM,
            1.5 * units::IN_HG,
            units::celsius(25.0),
            heat_capacity_ratio(5),
        )
    }

    /// Compressible orifice mass-flow rate in mol/s (signed by pressure gradient).
    pub fn flow_rate(
        k_flow: f64,
        p0: f64,
        p1: f64,
        t0: f64,
        t1: f64,
        hcr: f64,
        choked_limit: f64,
        choked_factor_cached: f64,
    ) -> f64 {
        // Robustness guards: never let a non-finite or non-positive state
        // propagate NaN through the flow network.
        if k_flow <= 0.0
            || !k_flow.is_finite()
            || !p0.is_finite()
            || !p1.is_finite()
            || !t0.is_finite()
            || !t1.is_finite()
            || p0 <= 0.0
            || p1 <= 0.0
            || t0 <= 0.0
            || t1 <= 0.0
        {
            return 0.0;
        }

        let (direction, t_0, p_0, p_t) = if p0 > p1 {
            (1.0, t0, p0, p1)
        } else {
            (-1.0, t1, p1, p0)
        };

        let p_ratio = p_t / p_0;
        let mut flow_rate = if p_ratio <= choked_limit {
            choked_factor_cached / (R * t_0).sqrt()
        } else {
            let s = p_ratio.powf(1.0 / hcr);
            let fr = (2.0 * hcr) / (hcr - 1.0);
            let fr = fr * s * (s - p_ratio);
            (fr.max(0.0) / (R * t_0)).sqrt()
        };

        flow_rate *= direction * p_0;
        flow_rate * k_flow
    }

    pub fn lose_n(&mut self, dn: f64, e_k_per_mol: f64) -> f64 {
        self.state.e_k -= e_k_per_mol * dn;
        self.state.n_mol -= dn;
        if self.state.n_mol < 0.0 {
            self.state.n_mol = 0.0;
        }
        dn
    }

    pub fn gain_n(&mut self, dn: f64, e_k_per_mol: f64, mix: Mix) -> f64 {
        let next_n = self.state.n_mol + dn;
        let current_n = self.state.n_mol;

        self.state.e_k += dn * e_k_per_mol;
        self.state.n_mol = next_n;

        if next_n != 0.0 {
            self.state.mix.p_fuel =
                (self.state.mix.p_fuel * current_n + dn * mix.p_fuel) / next_n;
            self.state.mix.p_inert =
                (self.state.mix.p_inert * current_n + dn * mix.p_inert) / next_n;
            self.state.mix.p_o2 = (self.state.mix.p_o2 * current_n + dn * mix.p_o2) / next_n;
        } else {
            self.state.mix = Mix {
                p_fuel: 0.0,
                p_inert: 0.0,
                p_o2: 0.0,
            };
        }

        -dn
    }

    pub fn dissipate_excess_velocity(&mut self) {
        let v_x = self.velocity_x();
        let v_y = self.velocity_y();
        let v_sq = v_x * v_x + v_y * v_y;
        let c = self.sound_speed();
        let c_sq = c * c;

        if c_sq >= v_sq || v_sq == 0.0 {
            return;
        }

        let k = (c_sq / v_sq).sqrt();
        self.state.momentum[0] *= k;
        self.state.momentum[1] *= k;
        self.state.e_k += 0.5 * self.mass() * (v_sq - c_sq);
        if self.state.e_k < 0.0 {
            self.state.e_k = 0.0;
        }
    }

    pub fn update_velocity(&mut self, dt: f64, beta: f64) {
        if self.n() == 0.0 {
            return;
        }
        let g = self.geometry;
        if g.width == 0.0 || g.height == 0.0 {
            return;
        }

        let depth = self.volume() / (g.width * g.height);

        let p0 = self.dynamic_pressure(g.dx, g.dy);
        let p1 = self.dynamic_pressure(-g.dx, -g.dy);
        let p2 = self.dynamic_pressure(g.dy, g.dx);
        let p3 = self.dynamic_pressure(-g.dy, -g.dx);

        let p_sa_0 = p0 * (g.height * depth);
        let p_sa_1 = p1 * (g.height * depth);
        let p_sa_2 = p2 * (g.width * depth);
        let p_sa_3 = p3 * (g.width * depth);

        let d_momentum_x = p_sa_0 * g.dx + p_sa_2 * g.dy - p_sa_1 * g.dx - p_sa_3 * g.dy;
        let d_momentum_y = p_sa_0 * g.dy + p_sa_2 * g.dx - p_sa_1 * g.dy - p_sa_3 * g.dx;

        let m = self.mass();
        let inv_m = 1.0 / m;
        let v0_x = self.state.momentum[0] * inv_m;
        let v0_y = self.state.momentum[1] * inv_m;

        self.state.momentum[0] -= d_momentum_x * dt * beta;
        self.state.momentum[1] -= d_momentum_y * dt * beta;

        let v1_x = self.state.momentum[0] * inv_m;
        let v1_y = self.state.momentum[1] * inv_m;

        self.state.e_k -= 0.5 * m * (v1_x * v1_x - v0_x * v0_x);
        self.state.e_k -= 0.5 * m * (v1_y * v1_y - v0_y * v0_y);
        if self.state.e_k < 0.0 {
            self.state.e_k = 0.0;
        }
    }

    pub fn dissipate_velocity(&mut self, dt: f64, time_constant: f64) {
        if self.n() == 0.0 {
            return;
        }
        let inv_mass = 1.0 / self.mass();
        let vx = self.state.momentum[0] * inv_mass;
        let vy = self.state.momentum[1] * inv_mass;
        let v_sq = vx * vx + vy * vy;

        let s = dt / (dt + time_constant);
        self.state.momentum[0] *= 1.0 - s;
        self.state.momentum[1] *= 1.0 - s;

        let nvx = self.state.momentum[0] * inv_mass;
        let nvy = self.state.momentum[1] * inv_mass;
        let n_v_sq = nvx * nvx + nvy * nvy;

        self.state.e_k += 0.5 * self.mass() * (v_sq - n_v_sq);
    }

    /// Flow between two systems. Returns net moles transferred from 0 to 1.
    pub fn flow_between(params: &mut FlowParams<'_>) -> f64 {
        let (dx, dy) = params.direction;
        // Split borrows carefully: compute pressures first
        let p0 = {
            let s = &*params.system_0;
            s.pressure() + s.dynamic_pressure(dx, dy)
        };
        let p1 = {
            let s = &*params.system_1;
            s.pressure() + s.dynamic_pressure(-dx, -dy)
        };

        let (
            source_ptr,
            sink_ptr,
            source_pressure,
            sink_pressure,
            source_cs,
            sink_cs,
            direction,
            sx,
            sy,
        ) = if p0 > p1 {
            (
                params.system_0 as *mut GasSystem,
                params.system_1 as *mut GasSystem,
                p0,
                p1,
                params.cross_section_0,
                params.cross_section_1,
                1.0,
                dx,
                dy,
            )
        } else {
            (
                params.system_1 as *mut GasSystem,
                params.system_0 as *mut GasSystem,
                p1,
                p0,
                params.cross_section_1,
                params.cross_section_0,
                -1.0,
                -dx,
                -dy,
            )
        };

        // SAFETY: source and sink are distinct objects (caller guarantees system_0 != system_1).
        let source: &mut GasSystem = unsafe { &mut *source_ptr };
        let sink: &mut GasSystem = unsafe { &mut *sink_ptr };

        let mut flow = params.dt
            * Self::flow_rate(
                params.k_flow,
                source_pressure,
                sink_pressure,
                source.temperature(),
                sink.temperature(),
                source.heat_capacity_ratio(),
                source.choked_flow_limit,
                source.choked_flow_factor,
            );

        flow = flow.clamp(0.0, 0.9 * source.n());
        if flow == 0.0 || source.n() == 0.0 {
            return flow * direction;
        }

        let fraction = flow / source.n();
        let fraction_volume = fraction * source.volume();
        let fraction_mass = fraction * source.mass();

        let bulk_src0 = source.bulk_kinetic_energy();
        let bulk_sink0 = sink.bulk_kinetic_energy();

        let e_k_per_mol = source.kinetic_energy_per_mol();
        let mix = source.state.mix;
        sink.gain_n(flow, e_k_per_mol, mix);
        source.lose_n(flow, e_k_per_mol);

        let dp_x = source.state.momentum[0] * fraction;
        let dp_y = source.state.momentum[1] * fraction;
        source.state.momentum[0] -= dp_x;
        source.state.momentum[1] -= dp_y;
        sink.state.momentum[0] += dp_x;
        sink.state.momentum[1] += dp_y;

        let bulk_src1 = source.bulk_kinetic_energy();
        let bulk_sink1 = sink.bulk_kinetic_energy();
        sink.state.e_k -= (bulk_src1 + bulk_sink1) - (bulk_src0 + bulk_sink0);

        let source_mass = source.mass();
        let inv_source_mass = if source_mass != 0.0 {
            1.0 / source_mass
        } else {
            0.0
        };
        let sink_mass = sink.mass();
        let inv_sink_mass = if sink_mass != 0.0 {
            1.0 / sink_mass
        } else {
            0.0
        };

        let c_source = source.sound_speed();
        let c_sink = sink.sound_speed();

        let src_i_x = source.state.momentum[0];
        let src_i_y = source.state.momentum[1];
        let snk_i_x = sink.state.momentum[0];
        let snk_i_y = sink.state.momentum[1];

        if sink_cs != 0.0 {
            let v = ((fraction_volume / sink_cs) / params.dt).clamp(0.0, c_sink);
            sink.state.momentum[0] += v * sx * fraction_mass;
            sink.state.momentum[1] += v * sy * fraction_mass;
        }
        if source_cs != 0.0 && source_mass != 0.0 {
            let v = ((fraction_volume / source_cs) / params.dt).clamp(0.0, c_source);
            source.state.momentum[0] += v * sx * fraction_mass;
            source.state.momentum[1] += v * sy * fraction_mass;
        }

        if source_mass != 0.0 {
            let v0x = src_i_x * inv_source_mass;
            let v0y = src_i_y * inv_source_mass;
            let v1x = source.state.momentum[0] * inv_source_mass;
            let v1y = source.state.momentum[1] * inv_source_mass;
            source.state.e_k -= 0.5 * source_mass * (v1x * v1x - v0x * v0x);
            source.state.e_k -= 0.5 * source_mass * (v1y * v1y - v0y * v0y);
        }
        if sink_mass > 0.0 {
            let v0x = snk_i_x * inv_sink_mass;
            let v0y = snk_i_y * inv_sink_mass;
            let v1x = sink.state.momentum[0] * inv_sink_mass;
            let v1y = sink.state.momentum[1] * inv_sink_mass;
            sink.state.e_k -= 0.5 * sink_mass * (v1x * v1x - v0x * v0x);
            sink.state.e_k -= 0.5 * sink_mass * (v1y * v1y - v0y * v0y);
        }

        if sink.state.e_k < 0.0 {
            sink.state.e_k = 0.0;
        }
        if source.state.e_k < 0.0 {
            source.state.e_k = 0.0;
        }

        flow * direction
    }

    /// Flow between this system and an environment at (P_env, T_env).
    /// Positive = leaving the system.
    pub fn flow_env(&mut self, k_flow: f64, dt: f64, p_env: f64, t_env: f64, mix: Mix) -> f64 {
        let max_flow = self.pressure_equilibrium_max_flow_env(p_env, t_env);
        let mut flow = dt
            * Self::flow_rate(
                k_flow,
                self.pressure(),
                p_env,
                self.temperature(),
                t_env,
                self.heat_capacity_ratio(),
                self.choked_flow_limit,
                self.choked_flow_factor,
            );

        if flow.abs() > max_flow.abs() {
            flow = max_flow;
        }

        if flow < 0.0 {
            let bulk0 = self.bulk_kinetic_energy();
            self.gain_n(
                -flow,
                kinetic_energy_per_mol(t_env, self.dof),
                mix,
            );
            let bulk1 = self.bulk_kinetic_energy();
            self.state.e_k += bulk1 - bulk0;
        } else {
            let starting_n = self.n();
            self.lose_n(flow, self.kinetic_energy_per_mol());
            if starting_n > 0.0 {
                let frac = flow / starting_n;
                self.state.momentum[0] -= frac * self.state.momentum[0];
                self.state.momentum[1] -= frac * self.state.momentum[1];
            }
        }

        flow
    }

    pub fn pressure_equilibrium_max_flow(&self, b: &GasSystem) -> f64 {
        if self.pressure() > b.pressure() {
            let max_flow = (b.volume() * self.state.e_k - self.volume() * b.state.e_k)
                / (b.volume() * self.kinetic_energy_per_mol()
                    + self.volume() * b.kinetic_energy_per_mol());
            0.0_f64.min(max_flow.min(self.n()))
        } else {
            let max_flow = (b.volume() * self.state.e_k - self.volume() * b.state.e_k)
                / (b.volume() * b.kinetic_energy_per_mol()
                    + self.volume() * b.kinetic_energy_per_mol());
            0.0_f64.max(max_flow).min(0.0).max(-b.n())
        }
    }

    pub fn pressure_equilibrium_max_flow_env(&self, p_env: f64, t_env: f64) -> f64 {
        if self.pressure() > p_env {
            -(p_env * (0.5 * self.dof as f64 * self.volume()) - self.state.e_k)
                / self.kinetic_energy_per_mol()
        } else {
            let e_k_per_mol_env = 0.5 * t_env * R * self.dof as f64;
            -(p_env * (0.5 * self.dof as f64 * self.volume()) - self.state.e_k) / e_k_per_mol_env
        }
    }

    pub fn heat_capacity_ratio(&self) -> f64 {
        heat_capacity_ratio(self.dof)
    }

    pub fn n(&self) -> f64 {
        self.state.n_mol
    }

    pub fn kinetic_energy(&self) -> f64 {
        self.state.e_k
    }

    pub fn kinetic_energy_for_n(&self, n: f64) -> f64 {
        if self.state.n_mol == 0.0 {
            return 0.0;
        }
        (self.state.e_k / self.state.n_mol) * n
    }

    pub fn kinetic_energy_per_mol(&self) -> f64 {
        if self.state.n_mol == 0.0 {
            return 0.0;
        }
        self.state.e_k / self.state.n_mol
    }

    pub fn total_energy(&self) -> f64 {
        if self.n() == 0.0 {
            return 0.0;
        }
        let m = self.mass();
        let inv_mass = 1.0 / m;
        let vx = self.state.momentum[0] * inv_mass;
        let vy = self.state.momentum[1] * inv_mass;
        self.state.e_k + 0.5 * m * (vx * vx + vy * vy)
    }

    pub fn bulk_kinetic_energy(&self) -> f64 {
        let m = self.mass();
        if m == 0.0 {
            return 0.0;
        }
        let vx = self.state.momentum[0] / m;
        let vy = self.state.momentum[1] / m;
        0.5 * m * (vx * vx + vy * vy)
    }

    pub fn dynamic_pressure(&self, dx: f64, dy: f64) -> f64 {
        if self.n() == 0.0 || self.state.e_k == 0.0 {
            return 0.0;
        }
        let inv_mass = 1.0 / self.mass();
        let v = inv_mass * (dx * self.state.momentum[0] + dy * self.state.momentum[1]);
        if v <= 0.0 {
            return 0.0;
        }

        let hcr = self.heat_capacity_ratio();
        let static_p = self.pressure();
        let density = self.approximate_density();
        let c_sq = static_p * hcr / density;
        if c_sq <= 0.0 {
            return 0.0;
        }
        let mach_sq = v * v / c_sq;
        let x = 1.0 + ((hcr - 1.0) / 2.0) * mach_sq;

        let x_d = match self.dof {
            3 => x * x * x * x * x,
            5 => {
                let x2 = x * x;
                let x3 = x2 * x;
                x3 * x3 * x
            }
            _ => x,
        };

        static_p * (x_d.sqrt() - 1.0)
    }

    pub fn approximate_density(&self) -> f64 {
        (units::AIR_MOLECULAR_MASS * self.n()) / self.volume().max(1e-30)
    }

    pub fn mass(&self) -> f64 {
        units::AIR_MOLECULAR_MASS * self.n()
    }

    pub fn pressure(&self) -> f64 {
        let v = self.volume();
        if v != 0.0 {
            self.state.e_k / (0.5 * self.dof as f64 * v)
        } else {
            0.0
        }
    }

    pub fn temperature(&self) -> f64 {
        if self.n() == 0.0 {
            0.0
        } else {
            self.state.e_k / (0.5 * self.dof as f64 * self.n() * R)
        }
    }

    pub fn velocity_x(&self) -> f64 {
        if self.n() == 0.0 {
            0.0
        } else {
            self.state.momentum[0] / self.mass()
        }
    }

    pub fn velocity_y(&self) -> f64 {
        if self.n() == 0.0 {
            0.0
        } else {
            self.state.momentum[1] / self.mass()
        }
    }

    pub fn volume(&self) -> f64 {
        self.state.v
    }

    pub fn n_fuel(&self) -> f64 {
        self.state.mix.p_fuel * self.n()
    }

    pub fn n_inert(&self) -> f64 {
        self.state.mix.p_inert * self.n()
    }

    pub fn n_o2(&self) -> f64 {
        self.state.mix.p_o2 * self.n()
    }

    pub fn mix(&self) -> Mix {
        self.state.mix
    }

    pub fn degrees_of_freedom(&self) -> i32 {
        self.dof
    }

    pub fn sound_speed(&self) -> f64 {
        if self.n() == 0.0 || self.state.e_k == 0.0 {
            return 0.0;
        }
        let hcr = self.heat_capacity_ratio();
        let static_p = self.pressure();
        let density = self.approximate_density();
        if density <= 0.0 {
            return 0.0;
        }
        (static_p * hcr / density).max(0.0).sqrt()
    }
}

pub const fn kinetic_energy_per_mol(t: f64, degrees_of_freedom: i32) -> f64 {
    0.5 * t * R * degrees_of_freedom as f64
}

pub const fn heat_capacity_ratio(degrees_of_freedom: i32) -> f64 {
    1.0 + (2.0 / degrees_of_freedom as f64)
}

pub fn choked_flow_limit(degrees_of_freedom: i32) -> f64 {
    let hcr = heat_capacity_ratio(degrees_of_freedom);
    (2.0 / (hcr + 1.0)).powf(hcr / (hcr - 1.0))
}

pub fn choked_flow_rate(degrees_of_freedom: i32) -> f64 {
    let hcr = heat_capacity_ratio(degrees_of_freedom);
    hcr.sqrt() * (2.0 / (hcr + 1.0)).powf((hcr + 1.0) / (2.0 * (hcr - 1.0)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ideal_gas_pressure_temperature() {
        let mut g = GasSystem::new();
        let p = units::ATM;
        let v = 0.001;
        let t = 300.0;
        g.initialize(p, v, t, Mix::AIR, 5);
        assert!((g.pressure() - p).abs() / p < 1e-9);
        assert!((g.temperature() - t).abs() < 1e-6);
        assert!(g.n() > 0.0);
    }

    #[test]
    fn adiabatic_compression_raises_pressure() {
        let mut g = GasSystem::new();
        g.initialize(units::ATM, 0.001, 300.0, Mix::AIR, 5);
        let p0 = g.pressure();
        g.set_volume(0.0005);
        assert!(g.pressure() > p0 * 1.5, "p0={p0} p1={}", g.pressure());
        // Temperature also rises (work done on gas)
        assert!(g.temperature() > 300.0);
    }

    #[test]
    fn choke_limit_diatomic() {
        // γ=1.4 → choked pressure ratio ≈ 0.528
        let lim = choked_flow_limit(5);
        assert!((lim - 0.52828).abs() < 1e-3, "lim={lim}");
    }

    #[test]
    fn flow_constant_zero_flow() {
        assert_eq!(GasSystem::k_carb(0.0), 0.0);
    }

    #[test]
    fn high_to_low_flow_is_positive_env() {
        let mut g = GasSystem::new();
        g.initialize(2.0 * units::ATM, 0.001, 300.0, Mix::AIR, 5);
        let k = GasSystem::k_carb(100.0);
        let flow = g.flow_env(k, 1e-5, units::ATM, 300.0, Mix::AIR);
        // Pressure higher than env → flow leaves (positive in C++ convention)
        assert!(flow > 0.0, "flow={flow}");
        let n_after = g.n();
        assert!(n_after < 2.0 * units::ATM * 0.001 / (R * 300.0));
    }

    #[test]
    fn react_conserves_and_burns_fuel() {
        let mut g = GasSystem::new();
        // Fuel-rich mixture in cylinder
        let mix = Mix {
            p_fuel: 0.05,
            p_inert: 0.70,
            p_o2: 0.25,
        };
        g.initialize(units::ATM, 0.0003, 800.0, mix, 5);
        let n0 = g.n();
        let fuel0 = g.n_fuel();
        let burned = g.react(g.n(), mix);
        assert!(burned > 0.0);
        assert!(g.n_fuel() < fuel0);
        assert!(g.n() > 0.0);
        let _ = n0;
        // Heat must be added externally for pressure rise; reaction alone changes composition
        assert!(g.n_inert() > n0 * 0.70);
    }

    #[test]
    fn two_system_flow_conserves_moles() {
        let mut a = GasSystem::new();
        let mut b = GasSystem::new();
        a.initialize(2.0 * units::ATM, 0.001, 300.0, Mix::AIR, 5);
        b.initialize(units::ATM, 0.001, 300.0, Mix::AIR, 5);
        let n_total = a.n() + b.n();
        let k = GasSystem::k_carb(50.0);

        // Manual: use raw flow computation between two systems via raw pointer pattern
        {
            let params = FlowParams {
                k_flow: k,
                dt: 1e-5,
                direction: (1.0, 0.0),
                cross_section_0: 0.0,
                cross_section_1: 0.0,
                system_0: &mut a,
                system_1: &mut b,
            };
            let mut params = params;
            GasSystem::flow_between(&mut params);
        }

        let n_total2 = a.n() + b.n();
        assert!(
            (n_total - n_total2).abs() / n_total < 1e-9,
            "moles not conserved: {n_total} vs {n_total2}"
        );
        assert!(b.n() > units::ATM * 0.001 / (R * 300.0));
    }

    #[test]
    fn energy_not_negative_after_flow() {
        let mut a = GasSystem::new();
        let mut b = GasSystem::new();
        a.initialize(5.0 * units::ATM, 1e-4, 500.0, Mix::AIR, 5);
        b.initialize(units::ATM, 1e-4, 300.0, Mix::AIR, 5);
        let k = GasSystem::k_carb(200.0);
        for _ in 0..100 {
            let mut params = FlowParams {
                k_flow: k,
                dt: 1e-5,
                direction: (1.0, 0.0),
                cross_section_0: 1e-4,
                cross_section_1: 1e-4,
                system_0: &mut a,
                system_1: &mut b,
            };
            GasSystem::flow_between(&mut params);
        }
        assert!(a.kinetic_energy() >= 0.0);
        assert!(b.kinetic_energy() >= 0.0);
    }
}
