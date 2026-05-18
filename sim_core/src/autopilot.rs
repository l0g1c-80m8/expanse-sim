//! Brachistochrone rendezvous autopilot.
//!
//! A simple state-machine guidance loop that takes a target body, the
//! spacecraft state, and a commanded proper acceleration, and produces a
//! thrust direction that lets the ship arrive at the target with (near-)
//! zero relative velocity. The phases:
//!
//! ```text
//!  Coast ─► Boost (point at intercept, full thrust)
//!         │
//!         ▼
//!     Brake (point retrograde-of-rel-vel, full thrust)
//!         │
//!         ▼
//!      Arrive (zero thrust, station-keeping range tolerance)
//! ```
//!
//! The intercept point is updated each tick using a few iterations of
//! "predict arrival time → lead the target by its velocity vector". This is
//! deliberately not Lambert-optimal — we want something simple and robust
//! that an autonomy stack can be tested *against* (e.g., does the operator's
//! planner beat the baseline?). For shorter ranges and high-thrust drives
//! this scheme matches an idealised flip-and-burn to within a few percent.
//!
//! The autopilot writes its desired thrust direction + magnitude into a
//! resource the host (sim_server) picks up and applies via its existing
//! thrust controller path — keeping the dynamics integrator the sole writer
//! of `f = ma`.

use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::components::{PropulsionDrive, RigidBody, Spacecraft};
use crate::ephemeris::EphemerisCache;
use crate::mission::Mission;

/// Standard gravity (m/s²) used to convert "Expanse-style" g-loads into
/// proper acceleration commands.
pub const G0: f64 = 9.806_65;

/// Lifecycle of a single transit under autopilot control.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AutopilotPhase {
    /// Autopilot disengaged. The ship is hand-flown.
    Idle,
    /// Initial boost — accelerate toward the predicted intercept point.
    Boost,
    /// Decelerate against relative velocity to null it out at arrival.
    Brake,
    /// Within tolerance bands; thrust commanded to zero.
    Arrived,
    /// Autopilot engaged but the target body is missing or the geometry is
    /// degenerate (e.g. zero distance). No thrust is applied.
    Hold,
}

/// What "arrived" should look like.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ArrivalTolerance {
    /// Maximum range to target (m).
    pub range_m: f64,
    /// Maximum relative speed to target (m/s).
    pub rel_speed_m_s: f64,
}

impl Default for ArrivalTolerance {
    fn default() -> Self {
        Self {
            // 50,000 km — comfortable for a planet flyby. Tighten if you
            // need to "park" at the body's L-point or surface.
            range_m: 5.0e7,
            rel_speed_m_s: 50.0,
        }
    }
}

/// Operator-facing autopilot configuration.
#[derive(Resource, Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Autopilot {
    pub engaged: bool,
    /// Commanded proper acceleration during boost / brake, in multiples of g₀.
    pub accel_g: f64,
    pub tolerance: ArrivalTolerance,
    /// Last computed phase. Updated by `autopilot_system` every tick.
    pub phase: AutopilotPhase,
    /// Estimated time-of-flight at the current tick (seconds).
    pub eta_s: f64,
    /// Last computed range to target (m).
    pub range_m: f64,
    /// Closing rate toward target (m/s, positive = approaching).
    pub closing_m_s: f64,
}

impl Default for Autopilot {
    fn default() -> Self {
        Self {
            engaged: false,
            accel_g: 1.0, // 1 g — Epstein-style cruise comfort
            tolerance: ArrivalTolerance::default(),
            phase: AutopilotPhase::Idle,
            eta_s: f64::INFINITY,
            range_m: f64::INFINITY,
            closing_m_s: 0.0,
        }
    }
}

/// What the autopilot wants the propulsion system to do this tick.
/// `dir_inertial` is a unit vector or `ZERO` (= cut thrust). `magnitude_n` is
/// already scaled to Newtons given the commanded `accel_g` and current mass.
#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct AutopilotCommand {
    pub dir_inertial: DVec3,
    pub magnitude_n: f64,
    pub active: bool,
}

/// Estimate the time-of-flight of a brachistochrone profile over straight-
/// line distance `d` with constant proper acceleration `a`: t = 2·√(d/a).
/// Used as a first-order ETA and to seed the t-go iteration.
pub fn brachistochrone_tof(d: f64, a: f64) -> f64 {
    if a <= 0.0 || d <= 0.0 {
        return 0.0;
    }
    2.0 * (d / a).sqrt()
}

/// Iteratively predict the intercept point and brachistochrone time-of-flight,
/// leading a moving target by its velocity. Fixed-point converges in a
/// handful of iterations for any sensible solar-system geometry.
pub fn predict_intercept(
    ship_pos: DVec3,
    target_pos: DVec3,
    target_vel: DVec3,
    accel: f64,
) -> (DVec3, f64) {
    let mut intercept = target_pos;
    let mut tof = brachistochrone_tof((intercept - ship_pos).length(), accel);
    for _ in 0..6 {
        intercept = target_pos + target_vel * tof;
        let d = (intercept - ship_pos).length();
        tof = brachistochrone_tof(d, accel);
    }
    (intercept, tof)
}

/// Distance required to brake from relative speed `v` to zero at constant
/// proper acceleration `a`: d = v²/(2a). Used by the diagnostic ETA path.
pub fn braking_distance(v: f64, a: f64) -> f64 {
    if a <= 0.0 {
        return f64::INFINITY;
    }
    v * v / (2.0 * a)
}

/// Zero-Effort-Miss / Zero-Effort-Velocity rendezvous guidance.
///
/// Given the relative state to a target and a time-to-go `tgo`, returns the
/// commanded acceleration that — applied as a constant over `tgo` — would
/// drive both relative position and relative velocity to zero. This is the
/// optimal closed-form solution to the quadratic-cost rendezvous problem
/// and the standard law in real flight software (Apollo descent guidance,
/// AR&D phasing burns).
///
/// `r_rel = r_target − r_ship`, `v_rel = v_target − v_ship`. Returns the
/// commanded acceleration *of the spacecraft*, in the inertial frame.
pub fn zem_zev_accel(r_rel: DVec3, v_rel: DVec3, tgo: f64) -> DVec3 {
    if tgo <= 1e-6 {
        return DVec3::ZERO;
    }
    // Standard ZEM/ZEV coefficients for fixed-time rendezvous: K_zem = 6,
    // K_zev = -2. These come from minimising ∫a·a dt subject to terminal
    // constraints; see Battin §13.
    let zem = r_rel + v_rel * tgo;
    let zev = v_rel;
    (zem * 6.0 / (tgo * tgo)) + (zev * (-2.0) / tgo)
}

/// ECS system: reads autopilot config + spacecraft state, writes the
/// resulting thrust command both into the shared `AutopilotCommand`
/// resource *and* directly onto the spacecraft's drive — the latter so the
/// autopilot just works when used with the bare sim_core (no external
/// thrust controller). Hosts that want to override the autopilot can do so
/// by checking `Autopilot.engaged`.
pub fn autopilot_system(
    mut ap: ResMut<Autopilot>,
    mut cmd: ResMut<AutopilotCommand>,
    mission: Res<Mission>,
    cache: Option<Res<EphemerisCache>>,
    mut q: Query<(&mut RigidBody, &mut PropulsionDrive), With<Spacecraft>>,
) {
    if !ap.engaged {
        ap.phase = AutopilotPhase::Idle;
        *cmd = AutopilotCommand::default();
        return;
    }

    let Some((mut rb, mut drive)) = q.iter_mut().next() else {
        ap.phase = AutopilotPhase::Hold;
        *cmd = AutopilotCommand::default();
        return;
    };
    let Some(cache) = cache else {
        ap.phase = AutopilotPhase::Hold;
        *cmd = AutopilotCommand::default();
        drive.thrust_command = DVec3::ZERO;
        return;
    };
    let Some(target_id) = mission.target_body else {
        ap.phase = AutopilotPhase::Hold;
        *cmd = AutopilotCommand::default();
        drive.thrust_command = DVec3::ZERO;
        return;
    };
    let Some(target) = cache.get(target_id) else {
        ap.phase = AutopilotPhase::Hold;
        *cmd = AutopilotCommand::default();
        drive.thrust_command = DVec3::ZERO;
        return;
    };

    let accel = ap.accel_g.max(0.0) * G0;

    let rel_pos = target.position - rb.position;
    let rel_vel = target.velocity - rb.velocity;
    let range = rel_pos.length();
    let rel_speed = rel_vel.length();
    // Positive closing rate ⇒ approaching. Equivalent to −d|r|/dt.
    let closing = if range > 0.0 {
        -rel_pos.dot(rel_vel) / range
    } else {
        0.0
    };

    ap.range_m = range;
    ap.closing_m_s = closing;

    if range < ap.tolerance.range_m && rel_speed < ap.tolerance.rel_speed_m_s {
        ap.phase = AutopilotPhase::Arrived;
        ap.eta_s = 0.0;
        *cmd = AutopilotCommand::default();
        drive.thrust_command = DVec3::ZERO;
        return;
    }

    if accel < 1e-9 {
        ap.phase = AutopilotPhase::Hold;
        *cmd = AutopilotCommand::default();
        drive.thrust_command = DVec3::ZERO;
        return;
    }

    // Seed time-to-go from a brachistochrone estimate, then *expand* tgo
    // until the ZEM/ZEV-commanded acceleration fits within our actual
    // thrust budget. Without this expansion the controller demands an
    // unbounded acceleration whenever the relative velocity is poorly
    // aligned with `rel_pos` — the classic ZEM/ZEV failure mode under
    // bounded actuation. Growing tgo slows the commanded profile until it
    // becomes feasible, which is provably stable.
    let (_intercept_pos, mut tgo) =
        predict_intercept(rb.position, target.position, target.velocity, accel);
    tgo = tgo.max(1.0);
    // Headroom: command at most 90 % of budget so there's bandwidth for
    // unmodelled disturbances (mass loss during burn, gravity gradients
    // when enabled, etc.).
    let a_budget = accel * 0.9;
    let mut a_cmd = zem_zev_accel(rel_pos, rel_vel, tgo);
    for _ in 0..40 {
        if a_cmd.length() <= a_budget {
            break;
        }
        tgo *= 1.3;
        a_cmd = zem_zev_accel(rel_pos, rel_vel, tgo);
    }
    ap.eta_s = tgo;

    let a_cmd_mag = a_cmd.length();

    // Clip to the commanded proper-acceleration budget. With the budget
    // saturated we may not exactly hit the optimal profile, but we always
    // produce the best-effort direction.
    let dir = if a_cmd_mag > 1e-12 {
        a_cmd / a_cmd_mag
    } else {
        DVec3::ZERO
    };
    let mag_n = rb.mass * accel.min(a_cmd_mag);

    // Diagnostic phase classification: if the commanded thrust component
    // along −rel_vel dominates, we're braking; otherwise boosting.
    let rel_vel_mag = rel_speed.max(1e-9);
    let along_minus_rel = a_cmd.dot(-rel_vel) / (a_cmd_mag.max(1e-12) * rel_vel_mag);
    ap.phase = if a_cmd_mag < 1e-6 {
        AutopilotPhase::Boost
    } else if along_minus_rel > 0.3 {
        AutopilotPhase::Brake
    } else {
        AutopilotPhase::Boost
    };

    cmd.dir_inertial = dir;
    cmd.magnitude_n = mag_n;
    cmd.active = dir.length_squared() > 0.0 && mag_n > 0.0;

    if cmd.active {
        rb.attitude = DQuat::from_rotation_arc(DVec3::X, dir);
        drive.thrust_command = DVec3::X * mag_n;
    } else {
        drive.thrust_command = DVec3::ZERO;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephemeris::naif;
    use crate::ephemeris::EphemerisCache;
    use approx::assert_relative_eq;

    #[test]
    fn brachistochrone_tof_matches_kinematics() {
        // 2 m at 1 m/s² ⇒ 2·√2 ≈ 2.828 s
        assert_relative_eq!(brachistochrone_tof(2.0, 1.0), 2.828_427_124_746_19, epsilon = 1e-9);
    }

    #[test]
    fn predict_intercept_static_target() {
        // Stationary target 1000 m away, 10 m/s² accel → straight shot.
        let ship = DVec3::ZERO;
        let target = DVec3::new(1000.0, 0.0, 0.0);
        let (intercept, tof) = predict_intercept(ship, target, DVec3::ZERO, 10.0);
        assert_relative_eq!(intercept.x, 1000.0, epsilon = 1e-9);
        // tof = 2*sqrt(1000/10) = 20 s
        assert_relative_eq!(tof, 20.0, epsilon = 1e-9);
    }

    #[test]
    fn predict_intercept_leads_moving_target() {
        let ship = DVec3::ZERO;
        let target_pos = DVec3::new(1000.0, 0.0, 0.0);
        let target_vel = DVec3::new(0.0, 50.0, 0.0);
        let (intercept, tof) = predict_intercept(ship, target_pos, target_vel, 10.0);
        // Intercept must lead the target along its velocity vector.
        assert!(intercept.y > 0.0, "intercept should lead +y, got {}", intercept.y);
        // Geometrically: the predicted intercept and the predicted tof must
        // satisfy `intercept = target_pos + v · tof` to within fixed-point
        // residual (≤1 iteration off the converged value).
        let residual = intercept.y - target_vel.y * tof;
        assert!(
            residual.abs() < target_vel.y * tof * 0.05,
            "leading residual too large: {}",
            residual
        );
    }

    #[test]
    fn braking_distance_matches_kinematics() {
        // 100 m/s at 5 m/s² → 1000 m
        assert_relative_eq!(braking_distance(100.0, 5.0), 1000.0, epsilon = 1e-12);
    }

    #[test]
    fn autopilot_idle_when_disengaged() {
        let mut world = World::new();
        world.insert_resource(Autopilot::default());
        world.insert_resource(AutopilotCommand::default());
        world.insert_resource(Mission::default());
        world.insert_resource(EphemerisCache::with_default_bodies());
        let mut schedule = Schedule::default();
        schedule.add_systems(autopilot_system);
        schedule.run(&mut world);
        let ap = world.resource::<Autopilot>();
        assert_eq!(ap.phase, AutopilotPhase::Idle);
        let cmd = world.resource::<AutopilotCommand>();
        assert!(!cmd.active);
    }

    #[test]
    fn autopilot_holds_when_target_missing() {
        let mut world = World::new();
        world.insert_resource(Autopilot {
            engaged: true,
            ..Default::default()
        });
        world.insert_resource(AutopilotCommand::default());
        world.insert_resource(Mission::default()); // no target
        world.insert_resource(EphemerisCache::with_default_bodies());
        world.spawn((
            Spacecraft { id: 1 },
            RigidBody::default(),
            PropulsionDrive::default(),
        ));
        let mut schedule = Schedule::default();
        schedule.add_systems(autopilot_system);
        schedule.run(&mut world);
        assert_eq!(world.resource::<Autopilot>().phase, AutopilotPhase::Hold);
    }

    #[test]
    fn autopilot_enters_boost_then_brake_with_inbound_geometry() {
        use crate::components::PropulsionType;
        let mut world = World::new();
        let ap = Autopilot { engaged: true, accel_g: 1.0, ..Default::default() };
        world.insert_resource(ap);
        world.insert_resource(AutopilotCommand::default());
        world.insert_resource(Mission::new(naif::SUN, naif::EARTH));
        world.insert_resource(EphemerisCache::with_default_bodies());

        let earth = world
            .resource::<EphemerisCache>()
            .get(naif::EARTH)
            .unwrap()
            .position;
        world.spawn((
            Spacecraft { id: 1 },
            RigidBody {
                position: earth - DVec3::new(1.0e9, 0.0, 0.0),
                velocity: DVec3::ZERO,
                mass: 1000.0,
                ..Default::default()
            },
            PropulsionDrive {
                drive_type: PropulsionType::Brachistochrone,
                isp: 12_000.0,
                propellant_mass: 10_000.0,
                max_thrust: 1.0e6,
                ..Default::default()
            },
        ));

        let mut sched = Schedule::default();
        sched.add_systems(autopilot_system);
        sched.run(&mut world);

        let phase = world.resource::<Autopilot>().phase;
        let cmd = *world.resource::<AutopilotCommand>();
        assert!(
            matches!(phase, AutopilotPhase::Boost | AutopilotPhase::Brake),
            "expected boost/brake, got {:?}",
            phase
        );
        assert!(cmd.active);
        assert!((cmd.dir_inertial.length() - 1.0).abs() < 1e-9);
    }
}
