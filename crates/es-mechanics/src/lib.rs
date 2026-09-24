//! Engine mechanical parts: crankshaft, piston, connecting rod, cylinder bank,
//! cylinder head, camshaft, valvetrain. Ports of the original C++ classes.

use es_function::Function;
use es_solver::RigidBody;
use es_units::PI;

// ---------------------------------------------------------------------------
// Crankshaft
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct CrankshaftParams {
    pub mass: f64,
    pub flywheel_mass: f64,
    pub moment_of_inertia: f64,
    /// Half stroke [m]
    pub crank_throw: f64,
    pub pos_x: f64,
    pub pos_y: f64,
    /// TDC offset [rad]
    pub tdc: f64,
    pub friction_torque: f64,
    pub rod_journals: usize,
}

#[derive(Clone, Debug)]
pub struct Crankshaft {
    pub body: RigidBody,
    rod_journal_angles: Vec<f64>,
    tdc: f64,
    throw: f64,
    mass: f64,
    inertia: f64,
    flywheel_mass: f64,
    pos_x: f64,
    pos_y: f64,
    friction_torque: f64,
}

impl Crankshaft {
    pub fn new(p: CrankshaftParams) -> Self {
        let mut body = RigidBody::new();
        body.set_mass_properties(p.mass + p.flywheel_mass, p.moment_of_inertia);
        body.p_x = p.pos_x;
        body.p_y = p.pos_y;
        body.theta = 0.0;
        Self {
            body,
            rod_journal_angles: vec![0.0; p.rod_journals],
            tdc: p.tdc,
            throw: p.crank_throw,
            mass: p.mass,
            inertia: p.moment_of_inertia,
            flywheel_mass: p.flywheel_mass,
            pos_x: p.pos_x,
            pos_y: p.pos_y,
            friction_torque: p.friction_torque,
        }
    }

    pub fn rod_journal_count(&self) -> usize {
        self.rod_journal_angles.len()
    }

    pub fn set_rod_journal_angle(&mut self, i: usize, angle: f64) {
        if i < self.rod_journal_angles.len() {
            self.rod_journal_angles[i] = angle;
        }
    }

    pub fn rod_journal_angle(&self, i: usize) -> f64 {
        self.rod_journal_angles[i]
    }

    pub fn rod_journal_position_local(&self, i: usize) -> (f64, f64) {
        let theta = self.rod_journal_angles[i];
        (theta.cos() * self.throw, theta.sin() * self.throw)
    }

    pub fn rod_journal_position_global(&self, i: usize) -> (f64, f64) {
        let (lx, ly) = self.rod_journal_position_local(i);
        (lx + self.body.p_x, ly + self.body.p_y)
    }

    pub fn reset_angle(&mut self) {
        // Wrap to [-4π, 4π] without changing fractional part much
        let four_pi = 4.0 * PI;
        self.body.theta = self.body.theta.rem_euclid(four_pi);
    }

    pub fn angle(&self) -> f64 {
        self.body.theta - self.tdc
    }

    pub fn cycle_angle(&self) -> f64 {
        let wrapped = (-self.angle()).rem_euclid(4.0 * PI);
        wrapped
    }

    pub fn tdc(&self) -> f64 {
        self.tdc
    }

    pub fn throw(&self) -> f64 {
        self.throw
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn moment_of_inertia(&self) -> f64 {
        self.inertia
    }

    pub fn flywheel_mass(&self) -> f64 {
        self.flywheel_mass
    }

    pub fn pos(&self) -> (f64, f64) {
        (self.pos_x, self.pos_y)
    }

    pub fn friction_torque(&self) -> f64 {
        self.friction_torque
    }
}

// ---------------------------------------------------------------------------
// Connecting rod
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct ConnectingRodParams {
    pub mass: f64,
    pub moment_of_inertia: f64,
    pub center_of_mass: f64,
    pub length: f64,
    pub journal: usize,
    pub crankshaft: Option<usize>,
    pub slave_throw: f64,
}

#[derive(Clone, Debug)]
pub struct ConnectingRod {
    pub body: RigidBody,
    center_of_mass: f64,
    length: f64,
    mass: f64,
    inertia: f64,
    journal: usize,
    pub crankshaft: Option<usize>,
    pub master: Option<usize>,
    #[allow(dead_code)]
    slave_throw: f64,
    #[allow(dead_code)]
    rod_journal_angles: Vec<f64>,
}

impl ConnectingRod {
    pub fn new(p: ConnectingRodParams) -> Self {
        let mut body = RigidBody::new();
        body.set_mass_properties(p.mass, p.moment_of_inertia);
        Self {
            body,
            center_of_mass: p.center_of_mass,
            length: p.length,
            mass: p.mass,
            inertia: p.moment_of_inertia,
            journal: p.journal,
            crankshaft: p.crankshaft,
            master: None,
            slave_throw: p.slave_throw,
            rod_journal_angles: Vec::new(),
        }
    }

    pub fn big_end_local(&self) -> f64 {
        -(self.length / 2.0) + self.center_of_mass
    }

    pub fn little_end_local(&self) -> f64 {
        (self.length / 2.0) - self.center_of_mass
    }

    pub fn journal(&self) -> usize {
        self.journal
    }

    pub fn length(&self) -> f64 {
        self.length
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn moment_of_inertia(&self) -> f64 {
        self.inertia
    }

    pub fn center_of_mass(&self) -> f64 {
        self.center_of_mass
    }

    pub fn master_rod(&self) -> Option<usize> {
        self.master
    }
}

// ---------------------------------------------------------------------------
// Piston
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct PistonParams {
    pub rod: usize,
    pub bank: usize,
    pub cylinder_index: usize,
    pub blowby_flow_coefficient: f64,
    pub compression_height: f64,
    pub wrist_pin_position: f64,
    pub displacement: f64,
    pub mass: f64,
}

#[derive(Clone, Debug)]
pub struct Piston {
    pub body: RigidBody,
    pub rod: usize,
    pub bank: usize,
    pub cylinder_index: usize,
    blowby_k: f64,
    compression_height: f64,
    wrist_pin: f64,
    displacement: f64,
    mass: f64,
    /// Wall reaction magnitude (updated each step from LineConstraint)
    pub wall_force: f64,
}

impl Piston {
    pub fn new(p: PistonParams) -> Self {
        let mut body = RigidBody::new();
        body.set_mass_properties(p.mass, 1.0);
        Self {
            body,
            rod: p.rod,
            bank: p.bank,
            cylinder_index: p.cylinder_index,
            blowby_k: p.blowby_flow_coefficient,
            compression_height: p.compression_height,
            wrist_pin: p.wrist_pin_position,
            displacement: p.displacement,
            mass: p.mass,
            wall_force: 0.0,
        }
    }

    pub fn relative_x(&self, bank_x: f64) -> f64 {
        self.body.p_x - bank_x
    }

    pub fn relative_y(&self, bank_y: f64) -> f64 {
        self.body.p_y - bank_y
    }

    pub fn compression_height(&self) -> f64 {
        self.compression_height
    }

    pub fn wrist_pin_location(&self) -> f64 {
        self.wrist_pin
    }

    pub fn blowby_k(&self) -> f64 {
        self.blowby_k
    }

    pub fn mass_val(&self) -> f64 {
        self.mass
    }

    pub fn displacement(&self) -> f64 {
        self.displacement
    }
}

/// Geometry view of a piston usable by combustion without full ownership.
pub trait PistonLike {
    fn body(&self) -> &es_solver::RigidBody;
    fn blowby_k(&self) -> f64;
    fn compression_height(&self) -> f64;
    fn wrist_pin_location(&self) -> f64;
    fn displacement(&self) -> f64;
    fn cylinder_index(&self) -> usize;
    fn bank_index(&self) -> usize;
    fn rod_index(&self) -> usize;
}

impl PistonLike for Piston {
    fn body(&self) -> &es_solver::RigidBody {
        &self.body
    }
    fn blowby_k(&self) -> f64 {
        self.blowby_k
    }
    fn compression_height(&self) -> f64 {
        self.compression_height
    }
    fn wrist_pin_location(&self) -> f64 {
        self.wrist_pin
    }
    fn displacement(&self) -> f64 {
        self.displacement
    }
    fn cylinder_index(&self) -> usize {
        self.cylinder_index
    }
    fn bank_index(&self) -> usize {
        self.bank
    }
    fn rod_index(&self) -> usize {
        self.rod
    }
}

// ---------------------------------------------------------------------------
// Cylinder bank
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct CylinderBankParams {
    pub position_x: f64,
    pub position_y: f64,
    pub angle: f64,
    pub bore: f64,
    pub deck_height: f64,
    pub cylinder_count: usize,
    pub index: usize,
}

#[derive(Clone, Debug)]
pub struct CylinderBank {
    angle: f64,
    bore: f64,
    deck_height: f64,
    cylinder_count: usize,
    index: usize,
    dx: f64,
    dy: f64,
    x: f64,
    y: f64,
}

impl CylinderBank {
    pub fn new(p: CylinderBankParams) -> Self {
        let dx = (p.angle + PI / 2.0).cos();
        let dy = (p.angle + PI / 2.0).sin();
        Self {
            angle: p.angle,
            bore: p.bore,
            deck_height: p.deck_height,
            cylinder_count: p.cylinder_count,
            index: p.index,
            dx,
            dy,
            x: p.position_x,
            y: p.position_y,
        }
    }

    pub fn angle(&self) -> f64 {
        self.angle
    }

    pub fn bore(&self) -> f64 {
        self.bore
    }

    pub fn deck_height(&self) -> f64 {
        self.deck_height
    }

    pub fn cylinder_count(&self) -> usize {
        self.cylinder_count
    }

    pub fn index(&self) -> usize {
        self.index
    }

    pub fn dx(&self) -> f64 {
        self.dx
    }

    pub fn dy(&self) -> f64 {
        self.dy
    }

    pub fn pos(&self) -> (f64, f64) {
        (self.x, self.y)
    }

    pub fn bore_surface_area(&self) -> f64 {
        PI * self.bore * self.bore / 4.0
    }

    pub fn position_above_deck(&self, h: f64) -> (f64, f64) {
        (
            self.dx * (self.deck_height + h) + self.x,
            self.dy * (self.deck_height + h) + self.y,
        )
    }
}

// ---------------------------------------------------------------------------
// Camshaft
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct CamshaftParams {
    pub lobes: usize,
    /// Advance in camshaft radians
    pub advance: f64,
    pub crankshaft: usize,
    pub lobe_profile: Function,
    pub base_radius: f64,
}

#[derive(Clone, Debug)]
pub struct Camshaft {
    crankshaft: usize,
    lobe_profile: Function,
    lobe_angles: Vec<f64>,
    advance: f64,
    base_radius: f64,
}

impl Camshaft {
    pub fn new(p: CamshaftParams) -> Self {
        Self {
            crankshaft: p.crankshaft,
            lobe_profile: p.lobe_profile,
            lobe_angles: vec![0.0; p.lobes],
            advance: p.advance,
            base_radius: p.base_radius,
        }
    }

    pub fn valve_lift(&self, lobe: usize, crank_angle: f64) -> f64 {
        self.sample_lobe(self.angle(crank_angle) + self.lobe_angles[lobe])
    }

    /// Sample the lobe profile at cam angle `theta`.
    ///
    /// Matches the reference `Camshaft::sampleLobe`: wrap into `[-π, π)` and
    /// sample the (zero-centered) profile directly. `Function::sample` clamps
    /// outside the lobe extents, yielding zero lift on the base circle.
    pub fn sample_lobe(&self, theta: f64) -> f64 {
        let mut t = theta % (2.0 * PI);
        if t < 0.0 {
            t += 2.0 * PI;
        }
        if t >= PI {
            t -= 2.0 * PI;
        }
        self.lobe_profile.sample(t)
    }

    pub fn set_lobe_centerline(&mut self, lobe: usize, crank_angle: f64) {
        self.lobe_angles[lobe] = crank_angle / 2.0;
    }

    pub fn lobe_centerline(&self, lobe: usize) -> f64 {
        self.lobe_angles[lobe]
    }

    pub fn angle(&self, crank_angle: f64) -> f64 {
        let angle = ((crank_angle + self.advance) * 0.5).rem_euclid(2.0 * PI);
        angle
    }

    pub fn crankshaft(&self) -> usize {
        self.crankshaft
    }

    pub fn base_radius(&self) -> f64 {
        self.base_radius
    }

    pub fn advance(&self) -> f64 {
        self.advance
    }
}

// ---------------------------------------------------------------------------
// Valvetrain
// ---------------------------------------------------------------------------

pub trait Valvetrain {
    fn intake_valve_lift(&self, cylinder: usize, crank_angle: f64) -> f64;
    fn exhaust_valve_lift(&self, cylinder: usize, crank_angle: f64) -> f64;
}

/// Standard DOHC/SOHC: one intake cam, one exhaust cam shared by cylinders.
#[derive(Clone, Debug)]
pub struct StandardValvetrain {
    pub intake_camshaft: usize,
    pub exhaust_camshaft: usize,
}

impl Valvetrain for StandardValvetrain {
    fn intake_valve_lift(&self, cylinder: usize, crank_angle: f64) -> f64 {
        // Camshaft lookup needs actual cam; caller provides via Engine wiring.
        // This trait is implemented on EngineAssembly which holds cams.
        let _ = (cylinder, crank_angle);
        0.0
    }

    fn exhaust_valve_lift(&self, cylinder: usize, crank_angle: f64) -> f64 {
        let _ = (cylinder, crank_angle);
        0.0
    }
}

/// Valvetrain that resolves lifts through a cam table holder.
pub struct ValvetrainRefs<'a> {
    pub cams: &'a [Camshaft],
    pub intake_cam: usize,
    pub exhaust_cam: usize,
    /// Map cylinder → lobe index on each cam
    pub intake_lobe: Vec<usize>,
    pub exhaust_lobe: Vec<usize>,
}

impl<'a> ValvetrainRefs<'a> {
    pub fn intake_lift(&self, cylinder: usize, crank_angle: f64) -> f64 {
        let cam = &self.cams[self.intake_cam];
        cam.valve_lift(self.intake_lobe[cylinder], crank_angle)
    }

    pub fn exhaust_lift(&self, cylinder: usize, crank_angle: f64) -> f64 {
        let cam = &self.cams[self.exhaust_cam];
        cam.valve_lift(self.exhaust_lobe[cylinder], crank_angle)
    }
}

// ---------------------------------------------------------------------------
// Cylinder head
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct CylinderHeadParams {
    pub bank: usize,
    pub exhaust_port_flow: Function,
    pub intake_port_flow: Function,
    pub combustion_chamber_volume: f64,
    pub intake_runner_volume: f64,
    pub intake_runner_cross_section: f64,
    pub exhaust_runner_volume: f64,
    pub exhaust_runner_cross_section: f64,
    pub cylinder_count: usize,
}

#[derive(Clone, Debug)]
pub struct HeadCylinder {
    pub exhaust_system: Option<usize>,
    pub intake: Option<usize>,
    pub sound_attenuation: f64,
    pub header_primary_length: f64,
}

#[derive(Clone, Debug)]
pub struct CylinderHead {
    pub bank: usize,
    cylinders: Vec<HeadCylinder>,
    exhaust_port_flow: Function,
    intake_port_flow: Function,
    intake_runner_volume: f64,
    intake_runner_cross_section: f64,
    exhaust_runner_volume: f64,
    exhaust_runner_cross_section: f64,
    combustion_chamber_volume: f64,
}

impl CylinderHead {
    pub fn new(p: CylinderHeadParams) -> Self {
        Self {
            bank: p.bank,
            cylinders: (0..p.cylinder_count)
                .map(|_| HeadCylinder {
                    exhaust_system: None,
                    intake: None,
                    sound_attenuation: 1.0,
                    header_primary_length: 0.0,
                })
                .collect(),
            exhaust_port_flow: p.exhaust_port_flow,
            intake_port_flow: p.intake_port_flow,
            intake_runner_volume: p.intake_runner_volume,
            intake_runner_cross_section: p.intake_runner_cross_section,
            exhaust_runner_volume: p.exhaust_runner_volume,
            exhaust_runner_cross_section: p.exhaust_runner_cross_section,
            combustion_chamber_volume: p.combustion_chamber_volume,
        }
    }

    pub fn intake_flow_rate(&self, _cylinder: usize, valve_lift: f64) -> f64 {
        self.intake_port_flow.sample(valve_lift)
    }

    pub fn exhaust_flow_rate(&self, _cylinder: usize, valve_lift: f64) -> f64 {
        self.exhaust_port_flow.sample(valve_lift)
    }

    pub fn exhaust_system(&self, cylinder: usize) -> Option<usize> {
        self.cylinders[cylinder].exhaust_system
    }

    pub fn set_all_exhaust_systems(&mut self, system: usize) {
        for c in &mut self.cylinders {
            c.exhaust_system = Some(system);
        }
    }

    pub fn set_exhaust_system(&mut self, i: usize, system: usize) {
        self.cylinders[i].exhaust_system = Some(system);
    }

    pub fn intake(&self, cylinder: usize) -> Option<usize> {
        self.cylinders[cylinder].intake
    }

    pub fn set_all_intakes(&mut self, intake: usize) {
        for c in &mut self.cylinders {
            c.intake = Some(intake);
        }
    }

    pub fn set_intake(&mut self, i: usize, intake: usize) {
        self.cylinders[i].intake = Some(intake);
    }

    pub fn sound_attenuation(&self, cylinder: usize) -> f64 {
        self.cylinders[cylinder].sound_attenuation
    }

    pub fn header_primary_length(&self, cylinder: usize) -> f64 {
        self.cylinders[cylinder].header_primary_length
    }

    pub fn combustion_chamber_volume(&self) -> f64 {
        self.combustion_chamber_volume
    }

    pub fn intake_runner_volume(&self) -> f64 {
        self.intake_runner_volume
    }

    pub fn intake_runner_cross_section(&self) -> f64 {
        self.intake_runner_cross_section
    }

    pub fn exhaust_runner_volume(&self) -> f64 {
        self.exhaust_runner_volume
    }

    pub fn exhaust_runner_cross_section(&self) -> f64 {
        self.exhaust_runner_cross_section
    }

    pub fn cylinder_count(&self) -> usize {
        self.cylinders.len()
    }

    pub fn set_header_primary_length(&mut self, cylinder: usize, length_m: f64) {
        if let Some(c) = self.cylinders.get_mut(cylinder) {
            c.header_primary_length = length_m;
        }
    }

    pub fn set_sound_attenuation(&mut self, cylinder: usize, att: f64) {
        if let Some(c) = self.cylinders.get_mut(cylinder) {
            c.sound_attenuation = att;
        }
    }
}

// ---------------------------------------------------------------------------
// Placement helpers (kinematics)
// ---------------------------------------------------------------------------

/// Place a rod given crank angle; returns journal world pos, rod theta, piston s along bank.
pub fn place_rod_kinematics(
    bank: &CylinderBank,
    rod: &ConnectingRod,
    journal_local: (f64, f64),
    crank_pos: (f64, f64),
    crank_theta: f64,
) -> Option<(f64, f64, f64, f64)> {
    let (dx0, dy0) = (crank_theta.cos(), crank_theta.sin());
    let p_x = crank_pos.0 + (dx0 * journal_local.0 - dy0 * journal_local.1);
    let p_y = crank_pos.1 + (dy0 * journal_local.0 + dx0 * journal_local.1);

    let (bx, by) = bank.pos();
    let (bdx, bdy) = (bank.dx(), bank.dy());
    let a = bdx * bdx + bdy * bdy;
    let b = -2.0 * bdx * (p_x - bx) - 2.0 * bdy * (p_y - by);
    let c = (p_x - bx) * (p_x - bx) + (p_y - by) * (p_y - by) - rod.length() * rod.length();

    let det = b * b - 4.0 * a * c;
    if det < 0.0 {
        return None;
    }
    let sqrt_det = det.sqrt();
    let s0 = (-b + sqrt_det) / (2.0 * a);
    let s1 = (-b - sqrt_det) / (2.0 * a);
    let s = s0.max(s1);
    if s < 0.0 {
        return None;
    }

    let e_x = s * bdx + bx;
    let e_y = s * bdy + by;

    let theta = if (e_y - p_y) > 0.0 {
        ((e_x - p_x) / rod.length()).clamp(-1.0, 1.0).acos()
    } else {
        2.0 * PI - ((e_x - p_x) / rod.length()).clamp(-1.0, 1.0).acos()
    };

    Some((p_x, p_y, theta - PI / 2.0, s))
}

#[cfg(test)]
mod tests {
    use super::*;
    use es_units::{mm, deg};

    #[test]
    fn crank_journal_position() {
        let crank = Crankshaft::new(CrankshaftParams {
            mass: 1.0,
            flywheel_mass: 1.0,
            moment_of_inertia: 0.05,
            crank_throw: 0.02,
            pos_x: 0.0,
            pos_y: 0.0,
            tdc: 0.0,
            friction_torque: 1.0,
            rod_journals: 1,
        });
        let (x, y) = crank.rod_journal_position_local(0);
        // angle 0 → (throw, 0)
        assert!((x - 0.02).abs() < 1e-12);
        assert!(y.abs() < 1e-12);
    }

    #[test]
    fn bank_orientation_v10() {
        let bank0 = CylinderBank::new(CylinderBankParams {
            position_x: 0.0,
            position_y: 0.0,
            angle: deg(-36.0), // half of 72° split, relative convention
            bore: mm(96.0),
            deck_height: 0.1,
            cylinder_count: 5,
            index: 0,
        });
        // direction is angle + 90°
        let expected = (deg(-36.0) + PI / 2.0).cos();
        assert!((bank0.dx() - expected).abs() < 1e-12);
    }

    #[test]
    fn place_rod_geometry() {
        let bank = CylinderBank::new(CylinderBankParams {
            position_x: 0.0,
            position_y: 0.0,
            angle: 0.0, // cylinders along +y (dx=cos(π/2)=0, dy=1)
            bore: 0.08,
            deck_height: 0.15,
            cylinder_count: 1,
            index: 0,
        });
        let rod = ConnectingRod::new(ConnectingRodParams {
            mass: 0.4,
            moment_of_inertia: 0.002,
            center_of_mass: 0.0,
            length: 0.1,
            journal: 0,
            crankshaft: Some(0),
            slave_throw: 0.0,
        });
        // Crank at 0°: journal at (throw, 0) = (0.03, 0)
        let result = place_rod_kinematics(
            &bank,
            &rod,
            (0.03, 0.0),
            (0.0, 0.0),
            0.0,
        );
        assert!(result.is_some());
        let (_, _, _, s) = result.unwrap();
        // Piston should be somewhere along bank axis with positive s
        assert!(s > 0.05 && s < 0.25, "s={s}");
    }
}
