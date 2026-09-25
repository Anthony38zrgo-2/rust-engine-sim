//! Engine assembly and offline simulation loop.

use std::cell::RefCell;
use std::rc::Rc;

use es_combustion::{
    ChamberForceState, ChamberRefs, CombustionChamber, FrictionModel, IgnitionModule,
    IgnitionParams,
};
use es_function::Function;
use es_intake_exhaust::{ExhaustParams, ExhaustSystem, Intake, IntakeParams};
use es_mechanics::{
    Camshaft, CamshaftParams, ConnectingRod, ConnectingRodParams, Crankshaft, CrankshaftParams,
    CylinderBank, CylinderBankParams, CylinderHead, CylinderHeadParams, Piston, PistonLike,
    PistonParams,
};
use es_solver::{
    BodySet, Constraint, FixedPositionConstraint, ForceGenerator, LineConstraint, LinkConstraint,
    RigidBody, RigidBodySystem, RotationFrictionConstraint,
};
use es_units::{self as units, rpm, PI};

// ---------------------------------------------------------------------------
// Speed-control constraint (starter / dyno)
// ---------------------------------------------------------------------------

/// Track a target angular velocity with a torque limit, matching the reference
/// `StarterMotor` / `Dynamometer` constraint semantics.
///
/// The reference solver solves the velocity constraint `J·v + v_bias = 0`
/// simultaneously with the linkage, with the Lagrange multiplier clamped to
/// `limits * dt` (torque limits scaled to impulses). We reproduce that as a
/// single impulse per physics step:
///
/// ```text
/// lambda = clamp((target - omega) * I, lo * dt, hi * dt)
/// omega += lambda / I
/// ```
///
/// applied exactly once (first solver iteration), never per iteration.
pub struct SpeedControlConstraint {
    pub body: usize,
    /// Signed target angular velocity [rad/s].
    pub rotation_speed: f64,
    /// Torque limit [N·m] (scaled by dt to get the impulse limit).
    pub max_torque: f64,
    /// Dynamometer mode: drives |omega| toward `rotation_speed` and brakes
    /// when `hold` is set. Starter mode only pushes toward the signed speed.
    pub dyno_mode: bool,
    enabled: bool,
    hold: bool,
    target: f64,
    lo_impulse: f64,
    hi_impulse: f64,
    dt: f64,
    impulse: f64,
    last_torque: f64,
}

impl SpeedControlConstraint {
    pub fn new(body: usize) -> Self {
        Self {
            body,
            rotation_speed: 0.0,
            max_torque: 0.0,
            dyno_mode: false,
            enabled: false,
            hold: false,
            target: 0.0,
            lo_impulse: 0.0,
            hi_impulse: 0.0,
            dt: 1e-4,
            impulse: 0.0,
            last_torque: 0.0,
        }
    }

    pub fn torque(&self) -> f64 {
        self.last_torque
    }
}

impl Constraint for SpeedControlConstraint {
    fn prepare(&mut self, bodies: &BodySet, dt: f64) {
        self.dt = dt.max(1e-12);
        self.impulse = 0.0;
        self.last_torque = 0.0;

        let b = &bodies.bodies[self.body];
        let max_impulse = self.max_torque * self.dt;

        if self.dyno_mode {
            // Reference Dynamometer: bias follows the current spin direction;
            // `hold` gates braking, `enabled` gates the driving side.
            // Positive impulses speed up positive rotation (and slow negative),
            // negative impulses do the opposite.
            if b.v_theta < 0.0 {
                self.target = -self.rotation_speed;
                // Drive toward the (negative) target: negative impulses.
                self.lo_impulse = if self.enabled { -max_impulse } else { 0.0 };
                // Hold: brake back toward target with positive impulses.
                self.hi_impulse = if self.hold && self.enabled {
                    max_impulse
                } else {
                    0.0
                };
            } else {
                self.target = self.rotation_speed;
                // Hold: brake back toward target with negative impulses.
                self.lo_impulse = if self.hold && self.enabled {
                    -max_impulse
                } else {
                    0.0
                };
                // Drive toward the (positive) target: positive impulses.
                self.hi_impulse = if self.enabled { max_impulse } else { 0.0 };
            }
        } else {
            // Reference StarterMotor: v_bias = -rotation_speed => target = speed.
            // Limits are one-sided: the starter can only push toward its own
            // signed direction (it cannot brake an overshooting engine).
            self.target = self.rotation_speed;
            if self.rotation_speed < 0.0 {
                self.lo_impulse = if self.enabled { -max_impulse } else { 0.0 };
                self.hi_impulse = 0.0;
            } else if self.rotation_speed > 0.0 {
                self.lo_impulse = 0.0;
                self.hi_impulse = if self.enabled { max_impulse } else { 0.0 };
            } else {
                let m = if self.enabled { max_impulse } else { 0.0 };
                self.lo_impulse = -m;
                self.hi_impulse = m;
            }
        }
    }

    fn solve(&mut self, bodies: &mut BodySet) {
        let b = &mut bodies.bodies[self.body];
        if b.inv_i == 0.0 {
            return;
        }
        // Impulse that would reach the target velocity right now.
        let desired = (self.target - b.v_theta) / b.inv_i;
        // Accumulate across iterations but keep the total within the
        // torque-limited budget for this step (matches reference `limits * dt`).
        let new_impulse = (self.impulse + desired).clamp(self.lo_impulse, self.hi_impulse);
        let delta = new_impulse - self.impulse;
        if delta != 0.0 {
            b.v_theta += delta * b.inv_i;
            self.impulse = new_impulse;
        }
        self.last_torque = self.impulse / self.dt;
    }

    fn reaction(&self) -> f64 {
        self.impulse
    }

    fn set_speed_control(&mut self, enabled: bool, hold: bool) {
        self.enabled = enabled;
        self.hold = hold;
    }

    fn set_rotation_speed(&mut self, speed: f64) {
        self.rotation_speed = speed;
    }

    fn solver_priority(&self) -> i32 {
        -1
    }
}

// ---------------------------------------------------------------------------
// Audio signal source
// ---------------------------------------------------------------------------

/// Raw engine audio input channels (one per exhaust system).
pub trait AudioSignalSource {
    fn channel_count(&self) -> usize;
    /// Last-step channel samples (system index → sample).
    fn channels(&self) -> &[f64];
    fn sample_rate(&self) -> f64;
}

// ---------------------------------------------------------------------------
// Engine description
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct EngineMeta {
    pub name: String,
    pub starter_torque: f64,
    pub starter_speed: f64,
    pub redline: f64,
    pub dyno_min_speed: f64,
    pub dyno_max_speed: f64,
    pub dyno_hold_step: f64,
    pub simulation_frequency: f64,
    pub fluid_simulation_steps: usize,
}

impl Default for EngineMeta {
    fn default() -> Self {
        Self {
            name: "engine".into(),
            starter_torque: units::ft_lb(90.0),
            starter_speed: rpm(200.0),
            redline: rpm(6500.0),
            dyno_min_speed: rpm(1000.0),
            dyno_max_speed: rpm(6500.0),
            dyno_hold_step: rpm(100.0),
            simulation_frequency: 10_000.0,
            fluid_simulation_steps: 8,
        }
    }
}

#[derive(Clone, Debug)]
pub struct BankConfig {
    pub angle: f64,
    pub bore: f64,
    pub deck_height: f64,
    pub position_x: f64,
    pub position_y: f64,
    pub cylinders: Vec<usize>,
}

#[derive(Clone, Debug)]
pub struct CylinderConfig {
    pub bank: usize,
    pub bank_cylinder: usize,
    pub rod_length: f64,
    pub rod_center_of_mass: f64,
    pub rod_mass: f64,
    pub rod_inertia: f64,
    pub piston_mass: f64,
    pub compression_height: f64,
    pub wrist_pin_position: f64,
    pub blowby_flow_coefficient: f64,
    pub displacement: f64,
    pub journal: usize,
}

#[derive(Clone, Debug)]
pub struct CrankConfig {
    pub mass: f64,
    pub flywheel_mass: f64,
    pub moment_of_inertia: f64,
    pub crank_throw: f64,
    /// Full stroke [m] (= 2 * crank_throw). Used for chamber max volume.
    pub stroke: f64,
    pub pos_x: f64,
    pub pos_y: f64,
    pub tdc: f64,
    pub friction_torque: f64,
    pub rod_journals: usize,
    /// Crankpin angle per journal [deg]
    pub journal_angles_deg: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct IgnitionConfig {
    pub firing_order: Vec<(usize, f64)>,
    pub timing_curve: Function,
    pub rev_limit: f64,
    pub limiter_duration: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct AudioPathParams {
    pub equal_bank_delay: bool,
    pub include_output_path_delay: bool,
    pub listener_distance: f64,
    pub speed_of_sound: f64,
}

impl Default for AudioPathParams {
    fn default() -> Self {
        Self {
            equal_bank_delay: false,
            include_output_path_delay: false,
            listener_distance: 0.0,
            speed_of_sound: 343.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Assembled engine
// ---------------------------------------------------------------------------

pub struct Engine {
    pub meta: EngineMeta,
    pub crankshafts: Vec<Crankshaft>,
    pub banks: Vec<CylinderBank>,
    pub heads: Vec<CylinderHead>,
    pub pistons: Vec<Piston>,
    pub rods: Vec<ConnectingRod>,
    pub chambers: Vec<CombustionChamber>,
    pub intakes: Vec<Intake>,
    pub exhausts: Vec<ExhaustSystem>,
    pub cams: Vec<Camshaft>,
    pub ignition: IgnitionModule,
    pub fuel: es_combustion::Fuel,
    pub throttle: f64,

    crank_body: usize,
    rod_body: Vec<usize>,
    piston_body: Vec<usize>,
    wall_constraint: Vec<usize>,
    starter_idx: usize,
    dyno_idx: usize,

    system: RigidBodySystem,
    force_state: Rc<RefCell<ChamberForceState>>,

    intake_cam: usize,
    exhaust_cam: usize,
    intake_lobe: Vec<usize>,
    exhaust_lobe: Vec<usize>,

    exhaust_flow_buffer: Vec<f64>,
    audio_source: Vec<CylinderAudioSeries>,
    audio_gain: Vec<f64>,
    audio_last_base: Vec<f64>,
    audio_last_flow: Vec<f64>,
    collector_pressure: Vec<Vec<f64>>,
    /// Exhaust system index per cylinder (audio staging).
    exhaust_system_index: Vec<usize>,
    /// Per-cylinder exhaust pulse delay (header propagation plus optional
    /// output path and listener distance, divided by the speed of sound).
    exhaust_delay: Vec<DelayLine>,
    exhaust_delay_s: Vec<f64>,
    time: f64,

    pub starter_enabled: bool,
    pub dyno_enabled: bool,
    pub dyno_hold: bool,
    pub dyno_speed: f64,
}

/// Fixed-latency ring-buffer delay line (reference `DelayFilter`).
struct DelayLine {
    latency_samples: usize,
    buffer: Vec<f64>,
    write: usize,
    read: usize,
    filled: usize,
}

impl DelayLine {
    fn new(delay_s: f64, rate: f64) -> Self {
        let latency_samples = (delay_s * rate).round().max(0.0) as usize;
        let capacity = latency_samples + 32;
        Self {
            latency_samples,
            buffer: vec![0.0; capacity],
            write: 0,
            read: 0,
            filled: 0,
        }
    }

    fn process(&mut self, sample: f64) -> f64 {
        let cap = self.buffer.len();
        if cap == 0 {
            return sample;
        }
        self.buffer[self.write] = sample;
        self.write = (self.write + 1) % cap;
        if self.filled < cap {
            self.filled += 1;
        }
        if self.filled <= self.latency_samples {
            return 0.0;
        }
        let v = self.buffer[self.read];
        self.read = (self.read + 1) % cap;
        self.filled -= 1;
        v
    }
}

/// Shared view of chamber pressures used as force generator.
struct ChamberForceProxy {
    state: Rc<RefCell<ChamberForceState>>,
}

impl ForceGenerator for ChamberForceProxy {
    fn apply(&mut self, bodies: &mut BodySet) {
        let mut st = self.state.borrow_mut();
        st.apply(bodies);
    }
}

pub struct EngineBuild {
    pub meta: EngineMeta,
    pub crank: CrankConfig,
    pub banks: Vec<BankConfig>,
    pub cylinders: Vec<CylinderConfig>,
    pub intake: IntakeParams,
    pub exhausts: Vec<ExhaustParams>,
    pub heads: Vec<CylinderHeadParams>,
    /// Per cylinder: (runner_flow_k, primary_flow_k, runner_length, primary_length, intake_cs, exhaust_cs)
    pub chamber_flow: Vec<(f64, f64, f64, f64, f64, f64)>,
    pub ignition: IgnitionConfig,
    pub fuel: es_combustion::Fuel,
    pub audio_path: AudioPathParams,
    pub intake_cam_params: CamshaftParams,
    pub exhaust_cam_params: CamshaftParams,
    /// Intake lobe centerline in crank degrees relative to each cylinder's
    /// fire angle. The intake peak occurs at `cycle = fire + intake_centerline`
    /// (450° = mid-intake stroke).
    pub intake_centerline: f64,
    /// Exhaust lobe centerline in crank degrees relative to fire angle
    /// (270° = mid-exhaust stroke).
    pub exhaust_centerline: f64,
    /// (cylinder, intake_lobe, exhaust_lobe)
    pub cam_assignment: Vec<(usize, usize, usize)>,
    pub mean_piston_speed_to_turbulence: Function,
    pub starting_pressure: f64,
    pub starting_temperature: f64,
    pub crankcase_pressure: f64,
    /// Per bank: (exhaust_system_index, primary_length_m)
    pub bank_exhaust: Vec<(usize, f64)>,
    /// Per bank sound attenuation multiplier
    pub bank_sound_attenuation: Vec<f64>,
    /// Per head, per cylinder exhaust system override (None = bank default).
    pub head_cylinder_exhaust: Vec<Vec<Option<usize>>>,
    /// Per head, per cylinder header primary length [m] (empty = bank default).
    pub head_cylinder_primary_mm: Vec<Vec<f64>>,
    /// Per head, per cylinder sound attenuation (empty = bank default).
    pub head_cylinder_attenuation: Vec<Vec<f64>>,
}

impl Engine {
    pub fn build(b: EngineBuild) -> Self {
        let meta = b.meta.clone();
        let mut system = RigidBodySystem::new();

        // Crankshaft
        let crank = Crankshaft::new(CrankshaftParams {
            mass: b.crank.mass,
            flywheel_mass: b.crank.flywheel_mass,
            moment_of_inertia: b.crank.moment_of_inertia,
            crank_throw: b.crank.crank_throw,
            pos_x: b.crank.pos_x,
            pos_y: b.crank.pos_y,
            tdc: b.crank.tdc,
            friction_torque: b.crank.friction_torque,
            rod_journals: b.crank.rod_journals,
        });
        let mut crank = crank;
        for (i, &deg) in b.crank.journal_angles_deg.iter().enumerate() {
            crank.set_rod_journal_angle(i, units::deg(deg));
        }
        let crank_body = system.add_body(crank.body.clone());
        system.add_constraint(Box::new(FixedPositionConstraint::new(
            crank_body,
            (b.crank.pos_x, b.crank.pos_y),
        )));
        system.add_constraint(Box::new(RotationFrictionConstraint::new(
            crank_body,
            b.crank.friction_torque,
        )));

        // Banks
        let banks: Vec<CylinderBank> = b
            .banks
            .iter()
            .enumerate()
            .map(|(i, cfg)| {
                CylinderBank::new(CylinderBankParams {
                    position_x: cfg.position_x,
                    position_y: cfg.position_y,
                    angle: cfg.angle,
                    bore: cfg.bore,
                    deck_height: cfg.deck_height,
                    cylinder_count: cfg.cylinders.len(),
                    index: i,
                })
            })
            .collect();

        // Heads
        let mut heads: Vec<CylinderHead> =
            b.heads.clone().into_iter().map(CylinderHead::new).collect();

        // Rods + pistons
        let mut rods = Vec::with_capacity(b.cylinders.len());
        let mut pistons = Vec::with_capacity(b.cylinders.len());
        for cc in &b.cylinders {
            rods.push(ConnectingRod::new(ConnectingRodParams {
                mass: cc.rod_mass,
                moment_of_inertia: cc.rod_inertia,
                center_of_mass: cc.rod_center_of_mass,
                length: cc.rod_length,
                journal: cc.journal,
                crankshaft: Some(0),
                slave_throw: 0.0,
            }));
            pistons.push(Piston::new(PistonParams {
                rod: rods.len() - 1,
                bank: cc.bank,
                cylinder_index: cc.bank_cylinder,
                blowby_flow_coefficient: cc.blowby_flow_coefficient,
                compression_height: cc.compression_height,
                wrist_pin_position: cc.wrist_pin_position,
                displacement: cc.displacement,
                mass: cc.piston_mass,
            }));
        }

        let mut rod_body = Vec::with_capacity(rods.len());
        for rod in &mut rods {
            rod_body.push(system.add_body(rod.body.clone()));
        }
        let mut piston_body = Vec::with_capacity(pistons.len());
        for piston in &mut pistons {
            piston_body.push(system.add_body(piston.body.clone()));
        }

        // Constraints: links + walls
        let mut wall_constraint = Vec::new();
        for (i, cc) in b.cylinders.iter().enumerate() {
            let bank = &banks[cc.bank];

            let mut link1 = LinkConstraint::new(rod_body[i], piston_body[i]);
            link1.local1 = (0.0, rods[i].little_end_local());
            link1.local2 = (0.0, pistons[i].wrist_pin_location());
            system.add_constraint(Box::new(link1));

            let journal_local = crank.rod_journal_position_local(cc.journal);
            let mut link2 = LinkConstraint::new(rod_body[i], crank_body);
            link2.local1 = (0.0, rods[i].big_end_local());
            link2.local2 = journal_local;
            system.add_constraint(Box::new(link2));

            let wall = LineConstraint::new(piston_body[i], (bank.dx(), bank.dy()), bank.pos());
            system.add_constraint(Box::new(wall));
            wall_constraint.push(system.constraint_count() - 1);
        }

        // Starter + dyno
        let mut starter = SpeedControlConstraint::new(crank_body);
        starter.rotation_speed = -meta.starter_speed;
        starter.max_torque = meta.starter_torque;
        system.add_constraint(Box::new(starter));
        let starter_idx = system.constraint_count() - 1;

        let mut dyno = SpeedControlConstraint::new(crank_body);
        dyno.rotation_speed = 0.0;
        dyno.max_torque = units::ft_lb(10_000.0);
        dyno.dyno_mode = true;
        system.add_constraint(Box::new(dyno));
        let dyno_idx = system.constraint_count() - 1;

        // Intake / exhaust
        let n_exhausts = b.exhausts.len();
        let intakes = vec![Intake::new(b.intake)];
        let exhausts: Vec<ExhaustSystem> = b
            .exhausts
            .into_iter()
            .enumerate()
            .map(|(i, p)| ExhaustSystem::new(i, p))
            .collect();

        // Chambers
        let mut chambers = Vec::with_capacity(b.cylinders.len());
        for (i, cc) in b.cylinders.iter().enumerate() {
            let bank = &banks[cc.bank];
            let head = &heads[cc.bank];
            let piston = &pistons[i];
            let (
                runner_flow_k,
                primary_flow_k,
                runner_length,
                primary_length,
                intake_cs,
                exhaust_cs,
            ) = b.chamber_flow[i];
            let refs = ChamberRefs {
                bank,
                head,
                piston,
                intake_cs,
                exhaust_cs,
                runner_flow_k,
                primary_flow_k,
                runner_length,
                primary_length,
                intake_runner_vol: head.intake_runner_volume(),
                exhaust_runner_vol: head.exhaust_runner_volume(),
                stroke: b.crank.stroke,
            };
            chambers.push(CombustionChamber::new(
                i,
                cc.bank,
                &refs,
                b.mean_piston_speed_to_turbulence.clone(),
                b.starting_pressure,
                b.starting_temperature,
                b.crankcase_pressure,
            ));
        }

        // Ignition
        let mut ignition = IgnitionModule::new(IgnitionParams {
            cylinder_count: b.cylinders.len(),
            timing_curve: b.ignition.timing_curve.clone(),
            rev_limit: b.ignition.rev_limit,
            limiter_duration: b.ignition.limiter_duration,
        });
        for (cyl, angle) in &b.ignition.firing_order {
            ignition.set_firing_order(*cyl, *angle);
        }

        // Cams
        let mut cams = vec![
            Camshaft::new(b.intake_cam_params),
            Camshaft::new(b.exhaust_cam_params),
        ];
        let n_cyl = b.cylinders.len();
        let mut intake_lobe = vec![0usize; n_cyl];
        let mut exhaust_lobe = vec![0usize; n_cyl];
        for (cyl, il, el) in &b.cam_assignment {
            intake_lobe[*cyl] = *il;
            exhaust_lobe[*cyl] = *el;
        }
        // Lobe centerlines from firing order (reference Camshaft convention:
        // peak when (crank + advance) / 2 + centerline / 2 == 0 (mod 2π),
        // i.e. peak crank angle == -advance - centerline (mod 4π)).
        //
        // `cycle_angle = -crank_angle`, so a lobe centerline of `fire + X`
        // peaks at `cycle = fire + X`. Configurable per cam (crank degrees).
        for &(cyl, fire_rad) in &b.ignition.firing_order {
            if cyl >= n_cyl {
                continue;
            }
            let il = intake_lobe[cyl];
            let el = exhaust_lobe[cyl];
            cams[0].set_lobe_centerline(il, fire_rad + b.intake_centerline);
            cams[1].set_lobe_centerline(el, fire_rad + b.exhaust_centerline);
        }
        // Wire exhaust systems / headers onto heads
        for (i, head) in heads.iter_mut().enumerate() {
            let default_sys = if n_exhausts == 0 {
                0
            } else {
                i.min(n_exhausts - 1)
            };
            head.set_all_exhaust_systems(default_sys);
            if let Some(&(sys, primary_m)) = b.bank_exhaust.get(i) {
                head.set_all_exhaust_systems(sys);
                for c in 0..head.cylinder_count() {
                    head.set_header_primary_length(c, primary_m);
                }
            }
            if let Some(&att) = b.bank_sound_attenuation.get(i) {
                for c in 0..head.cylinder_count() {
                    head.set_sound_attenuation(c, att);
                }
            }
            // Per-cylinder overrides (JSON `cylinder_exhaust` etc.)
            if let Some(overrides) = b.head_cylinder_exhaust.get(i) {
                for (c, sys) in overrides.iter().enumerate() {
                    if let Some(sys) = sys {
                        head.set_exhaust_system(c, *sys);
                    }
                }
            }
            if let Some(primaries) = b.head_cylinder_primary_mm.get(i) {
                for (c, len_m) in primaries.iter().enumerate() {
                    if *len_m > 0.0 {
                        head.set_header_primary_length(c, *len_m);
                    }
                }
            }
            if let Some(atts) = b.head_cylinder_attenuation.get(i) {
                for (c, att) in atts.iter().enumerate() {
                    if *att > 0.0 {
                        head.set_sound_attenuation(c, *att);
                    }
                }
            }
            head.set_all_intakes(0);
        }

        // Force state
        let force_state = Rc::new(RefCell::new(ChamberForceState {
            pressures: vec![b.starting_pressure; n_cyl],
            piston_speeds: vec![0.0; n_cyl],
            wall_forces: vec![0.0; n_cyl],
            friction_models: vec![FrictionModel::default(); n_cyl],
            bank_dx: b.cylinders.iter().map(|c| banks[c.bank].dx()).collect(),
            bank_dy: b.cylinders.iter().map(|c| banks[c.bank].dy()).collect(),
            bore: b.cylinders.iter().map(|c| banks[c.bank].bore()).collect(),
            crankcase_pressure: b.crankcase_pressure,
            piston_index: piston_body.clone(),
        }));
        system.add_force_generator(Box::new(ChamberForceProxy {
            state: force_state.clone(),
        }));

        let exhaust_flow_buffer = vec![0.0; exhausts.len()];

        // Audio staging: per-cylinder exhaust system index and pulse delay.
        // The acoustic phase uses primary/header propagation only; the output
        // path length is not applied as a pure delay unless explicitly enabled.
        let n_cyl = pistons.len();
        let mut exhaust_system_index = Vec::with_capacity(n_cyl);
        let mut exhaust_delay = Vec::with_capacity(n_cyl);
        let delay_rate = meta.simulation_frequency;
        let speed_of_sound = b.audio_path.speed_of_sound.max(1.0);
        let n_chambers = n_cyl.max(1) as f64;
        let mut delays = Vec::with_capacity(n_cyl);
        let mut audio_gain = Vec::with_capacity(n_cyl);
        for piston in &pistons {
            let bank_idx = piston.bank;
            let bank_cyl = piston.cylinder_index;
            let head = &heads[bank_idx];
            let ex_idx = head.exhaust_system(bank_cyl).unwrap_or(0);
            let exhaust = &exhausts[ex_idx];
            let mut delay_s = head.header_primary_length(bank_cyl) / speed_of_sound;
            if b.audio_path.include_output_path_delay {
                delay_s += exhaust.length() / speed_of_sound;
            }
            delay_s += b.audio_path.listener_distance / speed_of_sound;
            let losses = exhaust.header_loss_gain()
                * exhaust.collector_loss_gain()
                * exhaust.exhaust_output_gain();
            audio_gain.push(
                head.sound_attenuation(bank_cyl) * exhaust.audio_volume() / n_chambers * losses,
            );
            exhaust_system_index.push(ex_idx);
            delays.push(delay_s);
        }
        if b.audio_path.equal_bank_delay {
            let max_delay = delays.iter().cloned().fold(0.0, f64::max);
            for d in &mut delays {
                *d = max_delay;
            }
        }
        for d in &delays {
            exhaust_delay.push(DelayLine::new(*d, delay_rate));
        }
        let exhaust_delay_s = delays;
        let n_exhausts = exhausts.len().max(1);

        Self {
            meta,
            crankshafts: vec![crank],
            banks,
            heads,
            pistons,
            rods,
            chambers,
            intakes,
            exhausts,
            cams,
            ignition,
            fuel: b.fuel,
            throttle: 0.0,
            crank_body,
            rod_body,
            piston_body,
            wall_constraint,
            starter_idx,
            dyno_idx,
            system,
            force_state,
            intake_cam: 0,
            exhaust_cam: 1,
            intake_lobe,
            exhaust_lobe,
            exhaust_flow_buffer,
            audio_source: (0..n_cyl).map(|_| CylinderAudioSeries::default()).collect(),
            audio_gain,
            audio_last_base: vec![0.0; n_cyl],
            audio_last_flow: vec![0.0; n_cyl],
            collector_pressure: vec![Vec::new(); n_exhausts],
            exhaust_system_index,
            exhaust_delay,
            exhaust_delay_s,
            time: 0.0,
            starter_enabled: false,
            dyno_enabled: false,
            dyno_hold: false,
            dyno_speed: 0.0,
        }
    }

    pub fn rpm(&self) -> f64 {
        units::to_rpm(self.system.bodies.bodies[self.crank_body].v_theta).abs()
    }

    pub fn omega(&self) -> f64 {
        self.crankshafts[0].body.v_theta
    }

    pub fn cylinder_exhaust_system(&self, i: usize) -> usize {
        self.exhaust_system_index[i]
    }

    pub fn cylinder_primary_length(&self, i: usize) -> f64 {
        let bank = self.pistons[i].bank;
        let cyl = self.pistons[i].cylinder_index;
        self.heads[bank].header_primary_length(cyl)
    }

    pub fn cylinder_delay_seconds(&self, i: usize) -> f64 {
        self.exhaust_delay_s[i]
    }

    /// Intake and exhaust valve lift for cylinder `j` at crank angle (radians).
    pub fn valve_lifts(&self, j: usize, crank_angle: f64) -> (f64, f64) {
        let il = self.cams[self.intake_cam].valve_lift(self.intake_lobe[j], crank_angle);
        let el = self.cams[self.exhaust_cam].valve_lift(self.exhaust_lobe[j], crank_angle);
        (il, el)
    }

    /// Set throttle plate position in [0,1] (1 = fully closed, matching
    /// `Intake::throttle_plate_position` = idle_position * throttle).
    pub fn set_throttle(&mut self, t: f64) {
        self.throttle = t.clamp(0.0, 1.0);
        for intake in &mut self.intakes {
            intake.throttle = self.throttle;
        }
    }

    /// Place all pistons/rods for current crank angle and initialize chamber volumes.
    pub fn place_cylinders(&mut self) {
        for i in 0..self.pistons.len() {
            self.place_cylinder(i);
        }
        for i in 0..self.chambers.len() {
            let vol = {
                let bank = &self.banks[self.pistons[i].bank];
                let head = &self.heads[self.pistons[i].bank];
                let piston = &self.pistons[i];
                self.chambers[i].volume(bank, head, piston)
            };
            self.chambers[i].system.initialize(
                units::ATM,
                vol,
                units::celsius(25.0),
                es_gas::Mix::AIR,
                5,
            );
        }
    }

    fn place_cylinder(&mut self, i: usize) {
        let bank_idx = self.pistons[i].bank;
        let journal = self.rods[i].journal();
        let rod_len = self.rods[i].length();
        let crank_pos = self.crankshafts[0].pos();
        let theta = self.crankshafts[0].body.theta;
        let journal_local = self.crankshafts[0].rod_journal_position_local(journal);
        let bank = self.banks[bank_idx].clone();

        let (jx, jy) = {
            let (lx, ly) = journal_local;
            // NOTE: f64::sin_cos returns (sin, cos).
            let (s, c) = theta.sin_cos();
            (crank_pos.0 + c * lx - s * ly, crank_pos.1 + s * lx + c * ly)
        };

        let a = bank.dx() * bank.dx() + bank.dy() * bank.dy();
        let b = -2.0 * bank.dx() * (jx - bank.pos().0) - 2.0 * bank.dy() * (jy - bank.pos().1);
        let c = (jx - bank.pos().0).powi(2) + (jy - bank.pos().1).powi(2) - rod_len * rod_len;
        let det = b * b - 4.0 * a * c;
        if det < 0.0 {
            return;
        }
        let sqrt_det = det.sqrt();
        let s = ((-b + sqrt_det) / (2.0 * a)).max((-b - sqrt_det) / (2.0 * a));
        if s < 0.0 {
            return;
        }
        let e_x = s * bank.dx() + bank.pos().0;
        let e_y = s * bank.dy() + bank.pos().1;

        let theta_rod = if (e_y - jy) > 0.0 {
            ((e_x - jx) / rod_len).clamp(-1.0, 1.0).acos()
        } else {
            2.0 * PI - ((e_x - jx) / rod_len).clamp(-1.0, 1.0).acos()
        };
        {
            let rod = &mut self.rods[i];
            rod.body.theta = theta_rod - PI / 2.0;
            rod.body.p_x = 0.0;
            rod.body.p_y = 0.0;
        }
        let big_local = {
            let th = self.rods[i].body.theta;
            let ly = self.rods[i].big_end_local();
            // NOTE: f64::sin_cos returns (sin, cos).
            let (s, c) = th.sin_cos();
            (c * 0.0 - s * ly, s * 0.0 + c * ly)
        };
        self.rods[i].body.p_x += jx - big_local.0;
        self.rods[i].body.p_y += jy - big_local.1;

        {
            let piston = &mut self.pistons[i];
            piston.body.p_x = e_x;
            piston.body.p_y = e_y;
            piston.body.theta = bank.angle() + PI;
        }

        self.system.bodies.bodies[self.rod_body[i]] = self.rods[i].body.clone();
        self.system.bodies.bodies[self.piston_body[i]] = self.pistons[i].body.clone();
        self.crankshafts[0].body = self.system.bodies.bodies[self.crank_body].clone();
    }

    fn update_force_state(&mut self) {
        let mut st = self.force_state.borrow_mut();
        for i in 0..self.chambers.len() {
            st.pressures[i] = self.chambers[i].system.pressure();
            let pi = self.piston_body[i];
            let b = &self.system.bodies.bodies[pi];
            let bank = self.pistons[i].bank;
            st.piston_speeds[i] = b.v_x * st.bank_dx[bank] + b.v_y * st.bank_dy[bank];
            st.wall_forces[i] = self.pistons[i].wall_force;
        }
    }

    /// One physics step of duration `dt` (simulation frequency).
    pub fn step(&mut self, dt: f64) {
        self.time += dt;

        // 1) Ignition. `enabled` is controlled by scenario events; do not
        // force it here or SetIgnition(false) becomes a no-op.
        let omega = self.omega();
        let cycle_angle = self.crankshafts[0].cycle_angle();
        self.ignition.update(dt, cycle_angle, omega);

        for i in 0..self.chambers.len() {
            if self.ignition.ignition_event(i) {
                let fuel = self.fuel.clone();
                self.chambers[i].ignite(&fuel);
            }
            let ca = self.crankshafts[0].cycle_angle();
            let bank = self.banks[self.pistons[i].bank].clone();
            let pg = self.piston_geometry(i);
            self.chambers[i].update_cycle_state(ca, &bank, &pg);
        }
        self.ignition.reset_events();

        // 2) Reset flow counters
        for c in &mut self.chambers {
            c.reset_flow_counters();
        }

        // 3) Fluid substeps
        let fluid_dt = dt / self.meta.fluid_simulation_steps as f64;
        for _ in 0..self.meta.fluid_simulation_steps {
            for j in 0..self.exhausts.len() {
                self.exhausts[j].process(fluid_dt);
            }
            for j in 0..self.intakes.len() {
                self.intakes[j].process(fluid_dt);
            }
            for j in 0..self.chambers.len() {
                self.chamber_flow_step(j, fluid_dt);
            }
        }

        // 4) Speed control flags
        self.system.set_speed_controls(
            self.starter_idx,
            self.dyno_idx,
            self.starter_enabled,
            self.dyno_enabled,
            self.dyno_hold,
            self.dyno_speed,
        );

        // 5) Forces + solve
        self.update_force_state();
        self.system.process(dt);

        // 6) Sync bodies back to part structs. The wall reaction is the raw
        // accumulated solver impulse magnitude (N·s). The friction model in
        // CombustionChamber::applyForce is empirically calibrated to this
        // convention (matching the reference), so keep it un-scaled.
        self.crankshafts[0].body = self.system.bodies.bodies[self.crank_body].clone();
        for i in 0..self.rods.len() {
            self.rods[i].body = self.system.bodies.bodies[self.rod_body[i]].clone();
            self.pistons[i].body = self.system.bodies.bodies[self.piston_body[i]].clone();
            if let Some(r) = self.system.constraint_reaction_at(self.wall_constraint[i]) {
                self.pistons[i].wall_force = r;
            }
        }

        // 7) Update chamber volumes + audio staging
        for j in 0..self.chambers.len() {
            let bank = self.banks[self.pistons[j].bank].clone();
            let head = self.heads[self.pistons[j].bank].clone();
            let pg = self.piston_geometry(j);
            self.chambers[j].update_volume(&bank, &head, &pg);
        }

        self.stage_exhaust_audio();
    }

    fn piston_geometry(&self, i: usize) -> PistonGeometry {
        PistonGeometry::from(&self.pistons[i])
    }

    fn chamber_flow_step(&mut self, j: usize, dt: f64) {
        let crank_angle = self.crankshafts[0].angle();
        let bank_idx = self.pistons[j].bank;
        let bank_cyl = self.pistons[j].cylinder_index;

        let intake_lift = self.cams[self.intake_cam].valve_lift(self.intake_lobe[j], crank_angle);
        let exhaust_lift =
            self.cams[self.exhaust_cam].valve_lift(self.exhaust_lobe[j], crank_angle);

        let fuel = self.fuel.clone();
        let bank = self.banks[bank_idx].clone();
        let head = self.heads[bank_idx].clone();
        let pg = self.piston_geometry(j);

        let intake_cs = head.intake_runner_cross_section().max(1e-6);
        let ex_idx = head.exhaust_system(bank_cyl).unwrap_or(0);

        let (ch_pre, ch_rest) = self.chambers.split_at_mut(j);
        let chamber = &mut ch_rest[0];
        let _ = ch_pre;
        let intake = &mut self.intakes[0];
        let (ex_pre, ex_rest) = self.exhausts.split_at_mut(ex_idx);
        let exhaust = &mut ex_rest[0];
        let _ = ex_pre;

        let collector_cs = exhaust.collector_cross_section();
        let intake_decay = intake.velocity_decay();
        let exhaust_decay = exhaust.velocity_decay();

        chamber.flow(
            dt,
            &bank,
            &head,
            &pg,
            intake_lift,
            exhaust_lift,
            &fuel,
            &mut intake.system,
            &mut exhaust.system,
            intake_cs,
            collector_cs,
            intake_decay,
            exhaust_decay,
            1.0,
        );
    }

    fn stage_exhaust_audio(&mut self) {
        for x in &mut self.exhaust_flow_buffer {
            *x = 0.0;
        }
        let omega = self.omega();
        for i in 0..self.chambers.len() {
            let ex_idx = self.exhaust_system_index[i];
            let chamber = &self.chambers[i];
            let attenuation = (omega.abs() / 40.0).min(1.0);
            let attenuation_3 = attenuation * attenuation * attenuation;
            let atm = units::ATM;
            let dyn_p = chamber.exhaust_runner_dynamic_pressure(1.0, 0.0)
                + chamber.exhaust_runner_dynamic_pressure(-1.0, 0.0);
            let base = chamber.exhaust_runner_pressure() - atm;
            let intake = chamber.intake_runner.pressure() - atm;
            let flow = chamber.last_exhaust_flow();
            let runner_temp = chamber.exhaust_runner.temperature();
            let derivative = base - self.audio_last_base[i];
            let blowdown = flow - self.audio_last_flow[i];
            self.audio_last_base[i] = base;
            self.audio_last_flow[i] = flow;
            let att3 = attenuation_3 * 1600.0;
            {
                let src = &mut self.audio_source[i];
                src.base.push(base);
                src.dyn_p.push(dyn_p);
                src.att3.push(att3);
                src.intake.push(intake);
                src.derivative.push(derivative);
                src.blowdown.push(blowdown);
                src.flow.push(flow);
                src.runner_temp.push(runner_temp);
            }
            let pulse = self.audio_gain[i] * att3 * (base + 0.1 * dyn_p);
            let delayed = self.exhaust_delay[i].process(pulse);
            self.exhaust_flow_buffer[ex_idx] += delayed;
        }
        for (ex, series) in self.collector_pressure.iter_mut().enumerate() {
            series.push(self.exhausts[ex].system.pressure() - units::ATM);
        }
    }
}

// ---------------------------------------------------------------------------
// Geometry view of a piston
// ---------------------------------------------------------------------------

/// Clone of piston geometry for chamber calculations without ownership conflicts.
#[derive(Clone, Debug)]
pub struct PistonGeometry {
    pub body: RigidBody,
    pub rod: usize,
    pub bank: usize,
    pub cylinder_index: usize,
    blowby_k: f64,
    compression_height: f64,
    wrist_pin: f64,
    displacement: f64,
    #[allow(dead_code)]
    wall_force: f64,
}

impl From<&Piston> for PistonGeometry {
    fn from(p: &Piston) -> Self {
        Self {
            body: p.body.clone(),
            rod: p.rod,
            bank: p.bank,
            cylinder_index: p.cylinder_index,
            blowby_k: p.blowby_k(),
            compression_height: p.compression_height(),
            wrist_pin: p.wrist_pin_location(),
            displacement: p.displacement(),
            wall_force: p.wall_force,
        }
    }
}

impl PistonLike for PistonGeometry {
    fn body(&self) -> &RigidBody {
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
// Scenario events
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub enum Event {
    SetThrottle(f64),
    SetStarter(bool),
    SetDyno(bool),
    SetDynoSpeed(f64),
    SetDynoHold(bool),
    SetIgnition(bool),
}

pub struct ScenarioEvent {
    pub time: f64,
    pub event: Event,
}

// ---------------------------------------------------------------------------
// Offline run loop
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default)]
pub struct CylinderAudioSeries {
    pub base: Vec<f64>,
    pub dyn_p: Vec<f64>,
    pub att3: Vec<f64>,
    pub intake: Vec<f64>,
    pub derivative: Vec<f64>,
    pub blowdown: Vec<f64>,
    pub flow: Vec<f64>,
    pub runner_temp: Vec<f64>,
}

pub struct SimOutput {
    pub rpm: Vec<f64>,
    pub exhaust_pressure: Vec<f64>,
    pub audio_channels: Vec<Vec<f64>>,
    pub cylinder_audio: Vec<CylinderAudioSeries>,
    pub cylinder_delay_s: Vec<f64>,
    pub cylinder_audio_gain: Vec<f64>,
    pub cylinder_exhaust: Vec<usize>,
    pub cylinder_bank: Vec<usize>,
    pub cylinder_primary_length: Vec<f64>,
    pub collector_pressure: Vec<Vec<f64>>,
    pub sample_rate: f64,
}

impl AudioSignalSource for Engine {
    fn channel_count(&self) -> usize {
        self.exhaust_flow_buffer.len()
    }
    fn channels(&self) -> &[f64] {
        &self.exhaust_flow_buffer
    }
    fn sample_rate(&self) -> f64 {
        self.meta.simulation_frequency
    }
}

impl Engine {
    /// Run offline simulation for `duration_s` seconds at `meta.simulation_frequency`.
    pub fn run_offline(&mut self, duration_s: f64, events: &[ScenarioEvent]) -> SimOutput {
        let fs = self.meta.simulation_frequency;
        let dt = 1.0 / fs;
        let steps = (duration_s * fs).round() as usize;

        let mut rpm_series = Vec::with_capacity(steps);
        let mut p_series = Vec::with_capacity(steps);
        let mut audio: Vec<Vec<f64>> = vec![Vec::with_capacity(steps); self.exhausts.len()];

        self.place_cylinders();
        self.ignition.reset(self.crankshafts[0].cycle_angle());
        self.ignition.enabled = true;
        for src in &mut self.audio_source {
            src.base.reserve(steps);
            src.dyn_p.reserve(steps);
            src.att3.reserve(steps);
            src.intake.reserve(steps);
            src.derivative.reserve(steps);
            src.blowdown.reserve(steps);
            src.flow.reserve(steps);
            src.runner_temp.reserve(steps);
        }
        for series in &mut self.collector_pressure {
            series.reserve(steps);
        }

        let mut ev_i = 0;
        for step in 0..steps {
            let t = step as f64 * dt;
            while ev_i < events.len() && events[ev_i].time <= t {
                match events[ev_i].event {
                    // Scenario `throttle` values follow the engine-sim JSON
                    // convention (1 = wide open); internally the throttle is
                    // a plate position where 1 = closed, hence `1 - v`.
                    Event::SetThrottle(v) => self.set_throttle(1.0 - v),
                    Event::SetStarter(v) => self.starter_enabled = v,
                    Event::SetDyno(v) => self.dyno_enabled = v,
                    Event::SetDynoSpeed(v) => self.dyno_speed = v,
                    Event::SetDynoHold(v) => self.dyno_hold = v,
                    Event::SetIgnition(v) => self.ignition.enabled = v,
                }
                ev_i += 1;
            }

            self.step(dt);

            rpm_series.push(self.rpm());
            let mean_p: f64 = self
                .exhausts
                .iter()
                .map(|e| e.system.pressure())
                .sum::<f64>()
                / self.exhausts.len().max(1) as f64;
            p_series.push(mean_p);
            for (ch, sample) in audio.iter_mut().zip(self.exhaust_flow_buffer.iter()) {
                ch.push(*sample);
            }
        }

        SimOutput {
            rpm: rpm_series,
            exhaust_pressure: p_series,
            audio_channels: audio,
            cylinder_audio: std::mem::take(&mut self.audio_source),
            cylinder_delay_s: self.exhaust_delay_s.clone(),
            cylinder_audio_gain: self.audio_gain.clone(),
            cylinder_exhaust: self.exhaust_system_index.clone(),
            cylinder_bank: (0..self.pistons.len())
                .map(|i| self.pistons[i].bank)
                .collect(),
            cylinder_primary_length: (0..self.pistons.len())
                .map(|i| self.cylinder_primary_length(i))
                .collect(),
            collector_pressure: std::mem::take(&mut self.collector_pressure),
            sample_rate: fs,
        }
    }
}

// ---------------------------------------------------------------------------
// Test helper: simple single-cylinder engine
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn single_cylinder_engine() -> Engine {
        use es_function::Interpolation;
        let bore = 0.086;
        let stroke = 0.086;
        let rod_len = 0.14;
        let deck = rod_len + stroke / 2.0 + 0.03;

        // Realistic port-flow constants (k ≈ 0.001–0.007 for a small engine),
        // matching the magnitude the rs24_v10.json config uses. The previous
        // values (1e-7) were ~10 000x too restrictive to run at all.
        let intake_flow = Function::from_samples(
            [(0.0, 0.0), (0.005, 0.004), (0.010, 0.007)],
            Interpolation::Linear,
        );
        let exhaust_flow = Function::from_samples(
            [(0.0, 0.0), (0.005, 0.004), (0.010, 0.007)],
            Interpolation::Linear,
        );

        // ref lift = 5 mm, max lift = 10 mm (the previous call passed 0.0 lift,
        // producing an empty profile — the valves never opened).
        let lobe = es_function::harmonic_lobe_profile(180.0, 0.005, 0.01, 1.0, 64);

        Engine::build(EngineBuild {
            meta: EngineMeta {
                name: "test-single".into(),
                starter_torque: units::ft_lb(40.0),
                starter_speed: rpm(300.0),
                redline: rpm(6000.0),
                dyno_min_speed: rpm(800.0),
                dyno_max_speed: rpm(6000.0),
                dyno_hold_step: rpm(100.0),
                simulation_frequency: 5_000.0,
                fluid_simulation_steps: 4,
            },
            crank: CrankConfig {
                mass: 5.0,
                flywheel_mass: 2.0,
                moment_of_inertia: 0.2,
                crank_throw: stroke / 2.0,
                stroke,
                pos_x: 0.0,
                pos_y: 0.0,
                tdc: 0.0,
                friction_torque: 4.0,
                rod_journals: 1,
                journal_angles_deg: vec![0.0],
            },
            banks: vec![BankConfig {
                angle: 0.0,
                bore,
                deck_height: deck,
                position_x: 0.0,
                position_y: 0.0,
                cylinders: vec![0],
            }],
            cylinders: vec![CylinderConfig {
                bank: 0,
                bank_cylinder: 0,
                rod_length: rod_len,
                rod_center_of_mass: 0.0,
                rod_mass: 0.4,
                rod_inertia: 0.003,
                piston_mass: 0.35,
                compression_height: 0.02,
                wrist_pin_position: 0.0,
                blowby_flow_coefficient: 1e-9,
                displacement: 0.0,
                journal: 0,
            }],
            intake: IntakeParams {
                volume: 0.0005,
                cross_section_area: 0.001,
                input_flow_k: 0.02,
                idle_flow_k: 5e-8,
                runner_flow_rate: 0.006,
                molecular_afr: 12.5,
                idle_throttle_plate_position: 0.9,
                runner_length: 0.1,
                velocity_decay: 0.5,
            },
            exhausts: vec![ExhaustParams {
                length: 0.5,
                collector_cross_section: 0.002,
                outlet_flow_rate: 0.02,
                primary_tube_length: 0.3,
                primary_flow_rate: 0.03,
                velocity_decay: 1.0,
                audio_volume: 1.0,
                impulse_response: None,
                header_loss_gain: 1.0,
                collector_loss_gain: 1.0,
                exhaust_output_gain: 1.0,
            }],
            heads: vec![CylinderHeadParams {
                bank: 0,
                exhaust_port_flow: exhaust_flow,
                intake_port_flow: intake_flow,
                combustion_chamber_volume: 4.5e-5,
                intake_runner_volume: 0.0002,
                intake_runner_cross_section: 0.001,
                exhaust_runner_volume: 0.0002,
                exhaust_runner_cross_section: 0.001,
                cylinder_count: 1,
            }],
            chamber_flow: vec![(0.006, 0.03, 0.1, 0.3, 0.001, 0.001)],
            ignition: IgnitionConfig {
                firing_order: vec![(0, 0.0)],
                timing_curve: Function::from_samples(
                    [(0.0, 0.0), (rpm(4000.0), 0.4)],
                    Interpolation::Linear,
                ),
                rev_limit: rpm(5500.0),
                limiter_duration: 0.05,
            },
            fuel: es_combustion::default_fuel(),
            audio_path: AudioPathParams::default(),
            intake_cam_params: CamshaftParams {
                lobes: 1,
                advance: 0.0,
                crankshaft: 0,
                lobe_profile: lobe.clone(),
                base_radius: 0.01,
            },
            exhaust_cam_params: CamshaftParams {
                lobes: 1,
                advance: 0.0,
                crankshaft: 0,
                lobe_profile: lobe,
                base_radius: 0.01,
            },
            intake_centerline: units::deg(450.0),
            exhaust_centerline: units::deg(270.0),
            cam_assignment: vec![(0, 0, 0)],
            mean_piston_speed_to_turbulence: Function::from_samples(
                [(0.0, 4.0), (30.0, 6.0)],
                Interpolation::Linear,
            ),
            starting_pressure: units::ATM,
            starting_temperature: units::celsius(25.0),
            // Reference engine-sim uses atmospheric crankcase pressure.
            crankcase_pressure: units::ATM,
            bank_exhaust: vec![(0, 0.3)],
            bank_sound_attenuation: vec![1.0],
            head_cylinder_exhaust: vec![Vec::new()],
            head_cylinder_primary_mm: vec![Vec::new()],
            head_cylinder_attenuation: vec![Vec::new()],
        })
    }

    #[test]
    fn mechanism_stays_bounded_at_speed() {
        let mut e = single_cylinder_engine();
        let events = vec![ScenarioEvent {
            time: 0.0,
            event: Event::SetStarter(true),
        }];
        let out = e.run_offline(5.0, &events);
        assert!(out.rpm.iter().all(|r| r.is_finite() && *r < 1000.0));
        let rod = e.rods[0].length();
        let throw = e.crankshafts[0].throw();
        let max_norm = rod + throw + 0.01;
        for (i, p) in e.pistons.iter().enumerate() {
            let norm = (p.body.p_x * p.body.p_x + p.body.p_y * p.body.p_y).sqrt();
            assert!(
                norm <= max_norm,
                "piston {i} drifted outside linkage: norm={norm} max={max_norm}"
            );
        }
    }

    #[test]
    fn mechanism_stays_bounded_at_high_rpm() {
        let mut e = single_cylinder_engine();
        e.meta.simulation_frequency = 20_000.0;
        let events = vec![
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDyno(true),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDynoSpeed(rpm(15_000.0)),
            },
        ];
        let out = e.run_offline(3.0, &events);
        let last = *out.rpm.last().unwrap();
        assert!(last > 14_000.0, "dyno did not hold high rpm: {last}");
        let rod = e.rods[0].length();
        let throw = e.crankshafts[0].throw();
        let max_norm = rod + throw + 0.01;
        for (i, p) in e.pistons.iter().enumerate() {
            let norm = (p.body.p_x * p.body.p_x + p.body.p_y * p.body.p_y).sqrt();
            assert!(
                norm <= max_norm,
                "piston {i} drifted at high rpm: norm={norm} max={max_norm}"
            );
        }
    }

    #[test]
    fn engine_builds_and_places() {
        let mut e = single_cylinder_engine();
        e.place_cylinders();
        assert_eq!(e.pistons.len(), 1);
        assert!(
            e.pistons[0].body.p_y.abs() > 0.01,
            "piston y={}",
            e.pistons[0].body.p_y
        );
        assert!(e.chambers[0].system.volume() > 1e-6);
        assert!(e.rpm().is_finite());
    }

    #[test]
    fn starter_spins_engine() {
        let mut e = single_cylinder_engine();
        let events = vec![
            ScenarioEvent {
                time: 0.0,
                event: Event::SetStarter(true),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetIgnition(true),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetThrottle(0.15),
            },
        ];
        let out = e.run_offline(0.5, &events);
        let last_rpm = *out.rpm.last().unwrap();
        let max_rpm = out.rpm.iter().cloned().fold(0.0, f64::max);
        // Starter must track its target speed (~300 rpm) without runaway.
        // The one-sided starter cannot brake, so gas-spring pumping allows a
        // modest overshoot but never the historical multi-thousand rpm blowup.
        assert!(max_rpm < 600.0, "starter runaway: max rpm {max_rpm}");
        assert!(
            (last_rpm - 300.0).abs() < 100.0,
            "expected starter near 300 rpm, got {last_rpm}"
        );
        assert_eq!(out.rpm.len(), out.audio_channels[0].len());
        assert!(out.exhaust_pressure.iter().all(|p| p.is_finite()));
    }

    #[test]
    fn dyno_drives_to_target_speed() {
        let mut e = single_cylinder_engine();
        let events = vec![
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDyno(true),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDynoSpeed(rpm(2000.0)),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDynoHold(false),
            },
        ];
        let out = e.run_offline(0.6, &events);
        let last = *out.rpm.last().unwrap();
        assert!(
            (last - 2000.0).abs() < 150.0,
            "dyno did not reach target: {last} rpm"
        );
    }

    #[test]
    fn dyno_hold_brakes_overspeed() {
        let mut e = single_cylinder_engine();
        let events = vec![
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDyno(true),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDynoSpeed(rpm(3000.0)),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetDynoHold(false),
            },
            // Slow the target down with `hold` enabled: the dyno must brake.
            ScenarioEvent {
                time: 0.4,
                event: Event::SetDynoSpeed(rpm(1500.0)),
            },
            ScenarioEvent {
                time: 0.4,
                event: Event::SetDynoHold(true),
            },
        ];
        let out = e.run_offline(1.0, &events);
        let last = *out.rpm.last().unwrap();
        assert!(last < 2000.0, "dyno hold failed to brake: {last} rpm");
        assert!(
            (last - 1500.0).abs() < 250.0,
            "dyno hold did not settle near target: {last} rpm"
        );
    }

    #[test]
    fn ignition_disabled_prevents_combustion() {
        let mut e = single_cylinder_engine();
        let events = vec![
            ScenarioEvent {
                time: 0.0,
                event: Event::SetStarter(true),
            },
            // Explicitly OFF: `step()` must not re-enable it.
            ScenarioEvent {
                time: 0.0,
                event: Event::SetIgnition(false),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetThrottle(0.15),
            },
        ];
        e.run_offline(0.3, &events);
        let burnt: f64 = e.chambers.iter().map(|c| c.n_burnt_fuel).sum();
        assert_eq!(burnt, 0.0, "ignition off but fuel was burned ({burnt})");
    }

    #[test]
    fn full_throttle_combusts_and_revs() {
        let mut e = single_cylinder_engine();
        let events = vec![
            ScenarioEvent {
                time: 0.0,
                event: Event::SetStarter(true),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetIgnition(true),
            },
            ScenarioEvent {
                time: 0.0,
                event: Event::SetThrottle(1.0),
            }, // wide open
            ScenarioEvent {
                time: 0.5,
                event: Event::SetStarter(false),
            },
        ];
        let out = e.run_offline(2.0, &events);
        let max_rpm = out.rpm.iter().cloned().fold(0.0, f64::max);
        let burnt: f64 = e.chambers.iter().map(|c| c.n_burnt_fuel).sum();
        // Combustion must actually burn fuel and add torque beyond the ~300 rpm
        // the starter alone would hold (the previous WOT behaviour, at 394 rpm,
        // is a symptom of the documented residual-dilution/tuning issue).
        assert!(burnt > 0.0, "no fuel burned at wide open throttle");
        assert!(
            max_rpm > 340.0,
            "combustion should push rpm past starter idle, max {max_rpm}"
        );
    }
}
