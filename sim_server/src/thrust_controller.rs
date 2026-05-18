//! Operator-facing thrust controller.
//!
//! Translates a high-level mode + magnitude (e.g. "burn prograde at 50 % of
//! max thrust") into a body-frame thrust command on the spacecraft's drive.
//! Each tick the system re-evaluates the desired inertial direction, slews
//! the attitude to align body +x with it (instant slew — a real attitude
//! controller is a future addition), and writes the body-frame command.
//!
//! The raw `set_thrust` WebSocket command remains available as an autonomy /
//! debugging escape hatch; this controller only runs when `mode != Off`.

use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use sim_core::{
    Autopilot, EphemerisCache, Mission, PropulsionDrive, RigidBody, Spacecraft,
};

/// High-level thrust direction selector. The body-bound variants resolve
/// against the ephemeris cache each tick, so even a fast-moving Mars stays
/// targeted correctly.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum ThrustMode {
    /// Engine off. Raw `set_thrust` commands are honoured as written.
    #[default]
    Off,
    /// Push along the spacecraft's body +x axis. The user has manual control
    /// over attitude (via `set_attitude_rate`); this mode does not slew.
    BodyX,
    /// Burn along the inertial velocity vector — raises orbital energy.
    Prograde,
    /// Burn opposite the velocity vector — lowers orbital energy.
    Retrograde,
    /// Point at the mission target body and burn.
    TowardTarget,
    /// Burn away from the mission target.
    AwayFromTarget,
    /// Point at the mission source body and burn.
    TowardSource,
    AwayFromSource,
    /// Point at a specific NAIF body (`set_thrust_mode { mode: "toward_body", body }`).
    TowardBody(i32),
    AwayFromBody(i32),
}

#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct ThrustController {
    pub mode: ThrustMode,
    /// Commanded thrust magnitude (Newtons). Capped by the drive's
    /// `max_thrust` downstream.
    pub magnitude_n: f64,
}

impl ThrustController {
    pub fn parse_mode(mode: &str, body: Option<i32>) -> ThrustMode {
        match mode {
            "off" => ThrustMode::Off,
            "body_x" | "body" => ThrustMode::BodyX,
            "prograde" => ThrustMode::Prograde,
            "retrograde" => ThrustMode::Retrograde,
            "toward_target" => ThrustMode::TowardTarget,
            "away_from_target" => ThrustMode::AwayFromTarget,
            "toward_source" => ThrustMode::TowardSource,
            "away_from_source" => ThrustMode::AwayFromSource,
            "toward_body" => ThrustMode::TowardBody(body.unwrap_or(0)),
            "away_from_body" => ThrustMode::AwayFromBody(body.unwrap_or(0)),
            _ => ThrustMode::Off,
        }
    }

    pub fn mode_str(&self) -> &'static str {
        match self.mode {
            ThrustMode::Off => "off",
            ThrustMode::BodyX => "body_x",
            ThrustMode::Prograde => "prograde",
            ThrustMode::Retrograde => "retrograde",
            ThrustMode::TowardTarget => "toward_target",
            ThrustMode::AwayFromTarget => "away_from_target",
            ThrustMode::TowardSource => "toward_source",
            ThrustMode::AwayFromSource => "away_from_source",
            ThrustMode::TowardBody(_) => "toward_body",
            ThrustMode::AwayFromBody(_) => "away_from_body",
        }
    }
}

fn body_direction(
    cache: &EphemerisCache,
    body_id: i32,
    from: DVec3,
    invert: bool,
) -> Option<DVec3> {
    let state = cache.get(body_id)?;
    let mut d = state.position - from;
    if d.length_squared() < 1.0 {
        return None;
    }
    if invert {
        d = -d;
    }
    Some(d.normalize())
}

/// ECS system that resolves the operator's mode + magnitude into a body-frame
/// thrust command and aligns attitude with the requested direction.
///
/// When the autopilot is engaged it writes the drive directly (see
/// `sim_core::autopilot_system`) — we skip in that case so the autopilot has
/// final say.
pub fn thrust_controller_system(
    ctrl: Res<ThrustController>,
    autopilot: Res<Autopilot>,
    mission: Res<Mission>,
    cache: Option<Res<EphemerisCache>>,
    mut q: Query<(&mut RigidBody, &mut PropulsionDrive), With<Spacecraft>>,
) {
    if autopilot.engaged {
        return;
    }

    if matches!(ctrl.mode, ThrustMode::Off) {
        return;
    }
    let Some((mut rb, mut drive)) = q.iter_mut().next() else {
        return;
    };

    let dir = match ctrl.mode {
        ThrustMode::Off => return,
        ThrustMode::BodyX => {
            // Don't slew — the user has manual attitude control.
            drive.thrust_command = DVec3::X * ctrl.magnitude_n;
            return;
        }
        ThrustMode::Prograde => {
            if rb.velocity.length_squared() < 1.0 {
                return;
            }
            rb.velocity.normalize()
        }
        ThrustMode::Retrograde => {
            if rb.velocity.length_squared() < 1.0 {
                return;
            }
            -rb.velocity.normalize()
        }
        ThrustMode::TowardTarget => {
            let Some(c) = cache.as_deref() else { return };
            let Some(id) = mission.target_body else { return };
            let Some(d) = body_direction(c, id, rb.position, false) else { return };
            d
        }
        ThrustMode::AwayFromTarget => {
            let Some(c) = cache.as_deref() else { return };
            let Some(id) = mission.target_body else { return };
            let Some(d) = body_direction(c, id, rb.position, true) else { return };
            d
        }
        ThrustMode::TowardSource => {
            let Some(c) = cache.as_deref() else { return };
            let Some(id) = mission.source_body else { return };
            let Some(d) = body_direction(c, id, rb.position, false) else { return };
            d
        }
        ThrustMode::AwayFromSource => {
            let Some(c) = cache.as_deref() else { return };
            let Some(id) = mission.source_body else { return };
            let Some(d) = body_direction(c, id, rb.position, true) else { return };
            d
        }
        ThrustMode::TowardBody(id) => {
            let Some(c) = cache.as_deref() else { return };
            let Some(d) = body_direction(c, id, rb.position, false) else { return };
            d
        }
        ThrustMode::AwayFromBody(id) => {
            let Some(c) = cache.as_deref() else { return };
            let Some(d) = body_direction(c, id, rb.position, true) else { return };
            d
        }
    };

    // Slew the attitude so body +x aligns with `dir`. Instantaneous — adequate
    // until a real attitude-control loop lives in sim_core.
    rb.attitude = DQuat::from_rotation_arc(DVec3::X, dir);
    drive.thrust_command = DVec3::X * ctrl.magnitude_n;
}
