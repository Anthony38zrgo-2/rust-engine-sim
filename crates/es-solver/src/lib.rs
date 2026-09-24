//! 2D rigid-body dynamics with sequential-impulse constraint solving.
//!
//! Inspired by `atg_scs` (simple-2d-constraint-solver) used by engine-sim:
//! fixed-position crank, line (cylinder wall), link (rod ends), clutch
//! (multi-crank), rotation friction, plus external force generators.

use std::collections::HashSet;

#[derive(Clone, Debug, Default)]
pub struct RigidBody {
    pub index: usize,
    pub p_x: f64,
    pub p_y: f64,
    /// Rotation angle [rad]
    pub theta: f64,
    pub v_x: f64,
    pub v_y: f64,
    /// Angular velocity [rad/s] (alias of `omega`; matches `v_theta` in C++)
    pub v_theta: f64,
    pub m: f64,
    pub i: f64,
    pub inv_m: f64,
    pub inv_i: f64,
    /// Accumulated external force this step (world frame, applied at COM)
    pub force_x: f64,
    pub force_y: f64,
    /// Accumulated external torque
    pub torque: f64,
}

impl RigidBody {
    pub fn new() -> Self {
        Self {
            inv_m: 0.0,
            inv_i: 0.0,
            ..Default::default()
        }
    }

    pub fn set_mass_properties(&mut self, m: f64, i: f64) {
        self.m = m;
        self.i = i;
        self.inv_m = if m > 0.0 { 1.0 / m } else { 0.0 };
        self.inv_i = if i > 0.0 { 1.0 / i } else { 0.0 };
    }

    pub fn omega(&self) -> f64 {
        self.v_theta
    }

    pub fn local_to_world(&self, lx: f64, ly: f64) -> (f64, f64) {
        // NOTE: f64::sin_cos returns (sin, cos).
        let (s, c) = self.theta.sin_cos();
        (
            self.p_x + c * lx - s * ly,
            self.p_y + s * lx + c * ly,
        )
    }

    pub fn clear_forces(&mut self) {
        self.force_x = 0.0;
        self.force_y = 0.0;
        self.torque = 0.0;
    }

    /// Apply force at world-space point.
    pub fn apply_force_at(&mut self, fx: f64, fy: f64, px: f64, py: f64) {
        self.force_x += fx;
        self.force_y += fy;
        self.torque += (px - self.p_x) * fy - (py - self.p_y) * fx;
    }

    /// Apply force at body-local point.
    pub fn apply_force_local(&mut self, fx: f64, fy: f64, lx: f64, ly: f64) {
        let (wx, wy) = self.local_to_world(lx, ly);
        self.apply_force_at(fx, fy, wx, wy);
    }
}

pub trait ForceGenerator {
    fn apply(&mut self, bodies: &mut BodySet);
}

// ---------------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------------

const KS_DEFAULT: f64 = 5000.0;
const KD_DEFAULT: f64 = 10.0;
/// Match C++ OptimizedNsv bias_factor=1.0 (full positional correction per step).
/// Sequential impulse needs more iterations than a global linear solve.
const SOLVER_ITERATIONS: usize = 32;
/// Baumgarte beta: 1.0 = fully correct position error in one step (C++ bias_factor).
const BIAS_BETA: f64 = 1.0;

pub trait Constraint {
    fn prepare(&mut self, bodies: &BodySet, dt: f64);
    /// One sequential-impulse iteration.
    fn solve(&mut self, bodies: &mut BodySet);
    /// Record reaction force magnitude for diagnostics (wall force etc.).
    fn reaction(&self) -> f64 {
        0.0
    }
    /// Speed-control (starter/dyno) enable; ignored by geometric constraints.
    fn set_speed_control(&mut self, _enabled: bool, _hold: bool) {}
    /// Optional target angular velocity for speed-control constraints.
    fn set_rotation_speed(&mut self, _speed: f64) {}
}

/// Point on body pinned to a world position.
pub struct FixedPositionConstraint {
    pub body: usize,
    pub local: (f64, f64),
    pub world: (f64, f64),
    pub ks: f64,
    pub kd: f64,
    // solver state
    r: (f64, f64),
    jacobian_mass_x: f64,
    jacobian_mass_y: f64,
    bias: f64,
    error: (f64, f64),
    impulse: f64,
}

impl FixedPositionConstraint {
    pub fn new(body: usize, world: (f64, f64)) -> Self {
        Self {
            body,
            local: (0.0, 0.0),
            world,
            ks: KS_DEFAULT,
            kd: KD_DEFAULT,
            r: (0.0, 0.0),
            jacobian_mass_x: 0.0,
            jacobian_mass_y: 0.0,
            bias: 0.0,
            error: (0.0, 0.0),
            impulse: 0.0,
        }
    }
}

impl Constraint for FixedPositionConstraint {
    fn prepare(&mut self, bodies: &BodySet, dt: f64) {
        let b = &bodies.bodies[self.body];
        let (wx, wy) = b.local_to_world(self.local.0, self.local.1);
        self.r = (wx - b.p_x, wy - b.p_y);

        let im_x = b.inv_m + b.inv_i * self.r.1 * self.r.1;
        let im_y = b.inv_m + b.inv_i * self.r.0 * self.r.0;
        self.jacobian_mass_x = if im_x > 0.0 { 1.0 / im_x } else { 0.0 };
        self.jacobian_mass_y = if im_y > 0.0 { 1.0 / im_y } else { 0.0 };

        let c_x = wx - self.world.0;
        let c_y = wy - self.world.1;
        self.bias = (BIAS_BETA / dt).max(0.0);
        self.error = (c_x, c_y);
        self.impulse = 0.0;
    }

    fn solve(&mut self, bodies: &mut BodySet) {
        let b = &mut bodies.bodies[self.body];
        let (pvx, pvy) = point_velocity(b, self.r);
        let target_vx = -self.bias * self.error.0;
        let target_vy = -self.bias * self.error.1;
        let lambda_x = -(pvx - target_vx) * self.jacobian_mass_x;
        let lambda_y = -(pvy - target_vy) * self.jacobian_mass_y;
        apply_point_impulse(b, lambda_x, lambda_y, self.r);
        self.impulse += (lambda_x * lambda_x + lambda_y * lambda_y).sqrt();
        // Refresh position error for next iteration (pseudo position projection)
        let (wx, wy) = b.local_to_world(self.local.0, self.local.1);
        self.error = (wx - self.world.0, wy - self.world.1);
    }

    fn reaction(&self) -> f64 {
        self.impulse
    }
}

fn point_velocity(b: &RigidBody, r: (f64, f64)) -> (f64, f64) {
    (
        b.v_x - b.v_theta * r.1,
        b.v_y + b.v_theta * r.0,
    )
}

fn apply_point_impulse(b: &mut RigidBody, ix: f64, iy: f64, r: (f64, f64)) {
    b.v_x += ix * b.inv_m;
    b.v_y += iy * b.inv_m;
    b.v_theta += b.inv_i * (r.0 * iy - r.1 * ix);
}

/// Point on body constrained to a line (infinite) through `p0` with direction `(dx,dy)`.
pub struct LineConstraint {
    pub body: usize,
    pub local: (f64, f64),
    pub p0: (f64, f64),
    pub d: (f64, f64),
    pub ks: f64,
    pub kd: f64,
    r: (f64, f64),
    n: (f64, f64), // line normal
    effective_mass: f64,
    bias: f64,
    error: f64,
    impulse: f64,
}

impl LineConstraint {
    pub fn new(body: usize, d: (f64, f64), p0: (f64, f64)) -> Self {
        let len = (d.0 * d.0 + d.1 * d.1).sqrt();
        let d = if len > 0.0 { (d.0 / len, d.1 / len) } else { (1.0, 0.0) };
        Self {
            body,
            local: (0.0, 0.0),
            p0,
            d,
            ks: KS_DEFAULT,
            kd: KD_DEFAULT,
            r: (0.0, 0.0),
            n: (d.1, -d.0),
            effective_mass: 0.0,
            bias: 0.0,
            error: 0.0,
            impulse: 0.0,
        }
    }
}

impl Constraint for LineConstraint {
    fn prepare(&mut self, bodies: &BodySet, dt: f64) {
        let b = &bodies.bodies[self.body];
        let (wx, wy) = b.local_to_world(self.local.0, self.local.1);
        self.r = (wx - b.p_x, wy - b.p_y);
        // signed distance to line
        let rel_x = wx - self.p0.0;
        let rel_y = wy - self.p0.1;
        self.error = rel_x * self.n.0 + rel_y * self.n.1;

        let rn = self.r.0 * self.n.0 + self.r.1 * self.n.1;
        let im = b.inv_m + b.inv_i * rn * rn;
        self.effective_mass = if im > 0.0 { 1.0 / im } else { 0.0 };
        self.bias = (BIAS_BETA / dt).max(0.0);
        self.impulse = 0.0;
    }

    fn solve(&mut self, bodies: &mut BodySet) {
        let b = &mut bodies.bodies[self.body];
        let (pvx, pvy) = point_velocity(b, self.r);
        let v_n = pvx * self.n.0 + pvy * self.n.1;
        let target = -self.bias * self.error;
        let lambda = -(v_n - target) * self.effective_mass;
        apply_point_impulse(b, lambda * self.n.0, lambda * self.n.1, self.r);
        self.impulse += lambda.abs();
    }

    /// Accumulated impulse magnitude across solver iterations (N·s). The engine's
    /// friction model consumes this raw magnitude (reference convention, the
    /// friction constants are calibrated to it), so callers must not rescale.
    fn reaction(&self) -> f64 {
        self.impulse
    }
}

/// Two points on two bodies must coincide.
pub struct LinkConstraint {
    pub body1: usize,
    pub body2: usize,
    pub local1: (f64, f64),
    pub local2: (f64, f64),
    pub ks: f64,
    pub kd: f64,
    r1: (f64, f64),
    r2: (f64, f64),
    effective_mass: f64,
    bias: f64,
    error: (f64, f64),
    impulse: f64,
}

impl LinkConstraint {
    pub fn new(body1: usize, body2: usize) -> Self {
        Self {
            body1,
            body2,
            local1: (0.0, 0.0),
            local2: (0.0, 0.0),
            ks: KS_DEFAULT,
            kd: KD_DEFAULT,
            r1: (0.0, 0.0),
            r2: (0.0, 0.0),
            effective_mass: 0.0,
            bias: 0.0,
            error: (0.0, 0.0),
            impulse: 0.0,
        }
    }

    fn solve_axis(&mut self, bodies: &mut BodySet, axis: (f64, f64), err: f64, bias: f64) {
        let (i1, i2) = (self.body1, self.body2);
        let (r1, r2) = (self.r1, self.r2);
        {
            let b1 = &bodies.bodies[i1];
            let b2 = &bodies.bodies[i2];
            let a1 = r1.0 * axis.1 - r1.1 * axis.0;
            let a2 = r2.0 * axis.1 - r2.1 * axis.0;
            let im = b1.inv_m + b2.inv_m + b1.inv_i * a1 * a1 + b2.inv_i * a2 * a2;
            self.effective_mass = if im > 0.0 { 1.0 / im } else { 0.0 };
            self.bias = bias;
            self.error = (err, 0.0);
        }

        let (v1x, v1y, v2x, v2y, inv_m1, inv_i1, inv_m2, inv_i2) = {
            let b1 = &bodies.bodies[i1];
            let b2 = &bodies.bodies[i2];
            let (vx1, vy1) = point_velocity(b1, r1);
            let (vx2, vy2) = point_velocity(b2, r2);
            (vx1, vy1, vx2, vy2, b1.inv_m, b1.inv_i, b2.inv_m, b2.inv_i)
        };

        let v_rel = (v1x - v2x) * axis.0 + (v1y - v2y) * axis.1;
        let target = -bias * err;
        let lambda = -(v_rel - target) * self.effective_mass;

        let b1 = &mut bodies.bodies[i1];
        b1.v_x += lambda * axis.0 * inv_m1;
        b1.v_y += lambda * axis.1 * inv_m1;
        b1.v_theta += inv_i1 * (r1.0 * (lambda * axis.1) - r1.1 * (lambda * axis.0));

        let b2 = &mut bodies.bodies[i2];
        b2.v_x -= lambda * axis.0 * inv_m2;
        b2.v_y -= lambda * axis.1 * inv_m2;
        b2.v_theta -= inv_i2 * (r2.0 * (lambda * axis.1) - r2.1 * (lambda * axis.0));

        self.impulse += lambda.abs();
    }
}

impl Constraint for LinkConstraint {
    fn prepare(&mut self, bodies: &BodySet, dt: f64) {
        let b1 = &bodies.bodies[self.body1];
        let b2 = &bodies.bodies[self.body2];
        let (w1x, w1y) = b1.local_to_world(self.local1.0, self.local1.1);
        let (w2x, w2y) = b2.local_to_world(self.local2.0, self.local2.1);
        self.r1 = (w1x - b1.p_x, w1y - b1.p_y);
        self.r2 = (w2x - b2.p_x, w2y - b2.p_y);
        self.error = (w1x - w2x, w1y - w2y);
        self.bias = (BIAS_BETA / dt).max(0.0);
        self.impulse = 0.0;
    }

    fn solve(&mut self, bodies: &mut BodySet) {
        let beta = self.bias;
        let ex = self.error.0;
        let ey = self.error.1;
        // Solve x then y axes
        self.solve_axis(bodies, (1.0, 0.0), ex, beta);
        self.solve_axis(bodies, (0.0, 1.0), ey, beta);
    }

    fn reaction(&self) -> f64 {
        self.impulse
    }
}

/// Equalize angular velocities of two bodies (rigid coupling of cranks).
pub struct ClutchConstraint {
    pub body1: usize,
    pub body2: usize,
    pub ks: f64,
    pub kd: f64,
    bias: f64,
    error: f64,
    effective_mass: f64,
    impulse: f64,
}

impl ClutchConstraint {
    pub fn new(body1: usize, body2: usize) -> Self {
        Self {
            body1,
            body2,
            ks: KS_DEFAULT,
            kd: KD_DEFAULT,
            bias: 0.0,
            error: 0.0,
            effective_mass: 0.0,
            impulse: 0.0,
        }
    }
}

impl Constraint for ClutchConstraint {
    fn prepare(&mut self, bodies: &BodySet, dt: f64) {
        let b1 = &bodies.bodies[self.body1];
        let b2 = &bodies.bodies[self.body2];
        // Also keep angles locked (position error on theta)
        self.error = b1.theta - b2.theta;
        let im = b1.inv_i + b2.inv_i;
        self.effective_mass = if im > 0.0 { 1.0 / im } else { 0.0 };
        self.bias = (BIAS_BETA / dt).max(0.0);
        self.impulse = 0.0;
    }

    fn solve(&mut self, bodies: &mut BodySet) {
        let (i1, i2) = (self.body1, self.body2);
        let (bias, err, em) = (self.bias, self.error, self.effective_mass);
        let (inv_i1, inv_i2) = {
            let b1 = &bodies.bodies[i1];
            let b2 = &bodies.bodies[i2];
            (b1.inv_i, b2.inv_i)
        };
        let domega = bodies.bodies[i1].v_theta - bodies.bodies[i2].v_theta;
        let target = -bias * err;
        let lambda = -(domega - target) * em;
        bodies.bodies[i1].v_theta += lambda * inv_i1;
        bodies.bodies[i2].v_theta -= lambda * inv_i2;
        self.impulse += lambda.abs();
    }

    fn reaction(&self) -> f64 {
        self.impulse
    }
}

/// Coulomb friction on rotation: torque in `[-max_torque, max_torque]`
/// resisting motion (used for crank friction, starter, dyno via external).
/// Applied once per step in `prepare` (impulse = torque * dt), not per iteration.
pub struct RotationFrictionConstraint {
    pub body: usize,
    pub max_torque: f64,
    impulse: f64,
}

impl RotationFrictionConstraint {
    pub fn new(body: usize, max_torque: f64) -> Self {
        Self {
            body,
            max_torque,
            impulse: 0.0,
        }
    }
}

impl Constraint for RotationFrictionConstraint {
    fn prepare(&mut self, bodies: &BodySet, dt: f64) {
        self.impulse = 0.0;
        if self.max_torque <= 0.0 || dt <= 0.0 {
            return;
        }
        let b = &bodies.bodies[self.body];
        if b.inv_i == 0.0 || b.v_theta == 0.0 {
            return;
        }
        // Coulomb friction impulse opposing rotation, limited by max_torque * dt
        let max_impulse = self.max_torque * dt;
        let omega = b.v_theta;
        // Impulse that would stop rotation: I * omega, limited by friction
        let stop_impulse = omega * b.i;
        let friction_impulse = stop_impulse.clamp(-max_impulse, max_impulse);
        self.impulse = friction_impulse;
    }

    fn solve(&mut self, bodies: &mut BodySet) {
        // Apply the friction impulse once (first solve call), then zero it
        if self.impulse == 0.0 {
            return;
        }
        let b = &mut bodies.bodies[self.body];
        if b.inv_i == 0.0 {
            self.impulse = 0.0;
            return;
        }
        b.v_theta -= self.impulse * b.inv_i;
        self.impulse = 0.0;
    }

    fn reaction(&self) -> f64 {
        self.impulse.abs()
    }
}

// ---------------------------------------------------------------------------
// World / system
// ---------------------------------------------------------------------------

pub struct BodySet {
    pub bodies: Vec<RigidBody>,
}

impl BodySet {
    pub fn new() -> Self {
        Self { bodies: Vec::new() }
    }

    pub fn add(&mut self, mut body: RigidBody) -> usize {
        let idx = self.bodies.len();
        body.index = idx;
        if body.m > 0.0 {
            body.set_mass_properties(body.m, body.i);
        }
        self.bodies.push(body);
        idx
    }
}

impl Default for BodySet {
    fn default() -> Self {
        Self::new()
    }
}

pub struct RigidBodySystem {
    pub bodies: BodySet,
    constraints: Vec<Box<dyn Constraint>>,
    force_generators: Vec<Box<dyn ForceGenerator>>,
    pub iterations: usize,
}

impl RigidBodySystem {
    pub fn new() -> Self {
        Self {
            bodies: BodySet::new(),
            constraints: Vec::new(),
            force_generators: Vec::new(),
            iterations: SOLVER_ITERATIONS,
        }
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        self.bodies.add(body)
    }

    pub fn add_constraint(&mut self, c: Box<dyn Constraint>) {
        self.constraints.push(c);
    }

    pub fn add_force_generator(&mut self, g: Box<dyn ForceGenerator>) {
        self.force_generators.push(g);
    }

    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }

    pub fn constraint_reaction_at(&self, index: usize) -> Option<f64> {
        self.constraints.get(index).map(|c| c.reaction())
    }

    /// Pop the last force generator (for generators that must be reclaimed each step).
    pub fn force_generators_pop_last(&mut self) -> Option<Box<dyn ForceGenerator>> {
        self.force_generators.pop()
    }

    /// Apply enable flags to starter/dyno speed-control constraints by index.
    pub fn set_speed_controls(
        &mut self,
        starter_idx: usize,
        dyno_idx: usize,
        starter_enabled: bool,
        dyno_enabled: bool,
        dyno_hold: bool,
        dyno_speed: f64,
    ) {
        if let Some(c) = self.constraints.get_mut(starter_idx) {
            c.set_speed_control(starter_enabled, false);
        }
        if let Some(c) = self.constraints.get_mut(dyno_idx) {
            c.set_speed_control(dyno_enabled, dyno_hold);
            c.set_rotation_speed(dyno_speed);
        }
    }

    /// Full step: forces → integrate velocities → constraints → integrate positions.
    pub fn process(&mut self, dt: f64) {
        // Clear and gather external forces
        for b in &mut self.bodies.bodies {
            b.clear_forces();
        }
        // Force generators need body indices; they mutate bodies directly.
        // Use split borrow via indices collected first.
        let n_gen = self.force_generators.len();
        for i in 0..n_gen {
            // SAFETY: swap out generator temporarily to avoid double borrow of self
            let mut gen = std::mem::replace(
                &mut self.force_generators[i],
                Box::new(NoopForce),
            );
            gen.apply(&mut self.bodies);
            self.force_generators[i] = gen;
        }

        // Integrate velocities (semi-implicit)
        for b in &mut self.bodies.bodies {
            if b.inv_m > 0.0 {
                b.v_x += b.force_x * b.inv_m * dt;
                b.v_y += b.force_y * b.inv_m * dt;
            }
            if b.inv_i > 0.0 {
                b.v_theta += b.torque * b.inv_i * dt;
            }
        }

        // Prepare constraints
        for c in &mut self.constraints {
            c.prepare(&self.bodies, dt);
        }

        // Iterations
        for _ in 0..self.iterations {
            for c in &mut self.constraints {
                c.solve(&mut self.bodies);
            }
        }

        // Integrate positions
        for b in &mut self.bodies.bodies {
            b.p_x += b.v_x * dt;
            b.p_y += b.v_y * dt;
            b.theta += b.v_theta * dt;
        }
    }

    pub fn reset(&mut self) {
        self.bodies.bodies.clear();
        self.constraints.clear();
        self.force_generators.clear();
    }
}

impl Default for RigidBodySystem {
    fn default() -> Self {
        Self::new()
    }
}

struct NoopForce;

impl ForceGenerator for NoopForce {
    fn apply(&mut self, _bodies: &mut BodySet) {}
}

/// Query reaction of constraint at index (for wall force diagnostics).
pub fn constraint_reaction(system: &RigidBodySystem, index: usize) -> f64 {
    system
        .constraints
        .get(index)
        .map(|c| c.reaction())
        .unwrap_or(0.0)
}

/// Utility: track wall force for a specific LineConstraint by index.
pub struct ConstraintIndexMap {
    pub map: HashSet<usize>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_fall() {
        let mut sys = RigidBodySystem::new();
        let mut b = RigidBody::new();
        b.set_mass_properties(1.0, 1.0);
        b.p_y = 10.0;
        let idx = sys.add_body(b);

        struct Gravity;
        impl ForceGenerator for Gravity {
            fn apply(&mut self, bodies: &mut BodySet) {
                for b in &mut bodies.bodies {
                    b.force_y -= 9.81 * b.m;
                }
            }
        }
        sys.add_force_generator(Box::new(Gravity));

        let dt = 0.01;
        for _ in 0..100 {
            sys.process(dt);
        }
        let b = &sys.bodies.bodies[idx];
        // y ≈ 10 - 0.5*9.81*1^2 = 5.095, v = -9.81
        assert!((b.p_y - 5.095).abs() < 0.05, "y={}", b.p_y);
        assert!((b.v_y + 9.81).abs() < 0.1, "vy={}", b.v_y);
    }

    #[test]
    fn fixed_position_holds() {
        let mut sys = RigidBodySystem::new();
        let mut b = RigidBody::new();
        b.set_mass_properties(10.0, 1.0);
        b.p_x = 1.0;
        b.p_y = 0.5;
        let idx = sys.add_body(b);
        sys.add_constraint(Box::new(FixedPositionConstraint::new(idx, (1.0, 0.5))));

        // Apply a force trying to move it
        struct Push;
        impl ForceGenerator for Push {
            fn apply(&mut self, bodies: &mut BodySet) {
                for b in &mut bodies.bodies {
                    b.force_x += 100.0;
                }
            }
        }
        sys.add_force_generator(Box::new(Push));

        for _ in 0..500 {
            sys.process(1.0 / 1000.0);
        }
        let b = &sys.bodies.bodies[idx];
        assert!((b.p_x - 1.0).abs() < 0.05, "x drifted to {}", b.p_x);
        assert!((b.p_y - 0.5).abs() < 0.05, "y drifted to {}", b.p_y);
    }

    #[test]
    fn link_keeps_distance() {
        let mut sys = RigidBodySystem::new();
        let mut a = RigidBody::new();
        a.set_mass_properties(1.0, 1.0);
        let ai = sys.add_body(a);
        let mut b = RigidBody::new();
        b.set_mass_properties(1.0, 1.0);
        b.p_x = 1.0;
        let bi = sys.add_body(b);

        let mut link = LinkConstraint::new(ai, bi);
        link.local2 = (0.0, 0.0);
        sys.add_constraint(Box::new(link));

        // Pull body B
        struct Pull;
        impl ForceGenerator for Pull {
            fn apply(&mut self, bodies: &mut BodySet) {
                if let Some(b) = bodies.bodies.get_mut(1) {
                    b.force_x += 50.0;
                }
            }
        }
        sys.add_force_generator(Box::new(Pull));

        for _ in 0..1000 {
            sys.process(1.0 / 1000.0);
        }
        let d = {
            let a = &sys.bodies.bodies[ai];
            let b = &sys.bodies.bodies[bi];
            ((a.p_x - b.p_x).powi(2) + (a.p_y - b.p_y).powi(2)).sqrt()
        };
        assert!((d - 1.0).abs() < 0.1, "distance={d}");
    }
}
