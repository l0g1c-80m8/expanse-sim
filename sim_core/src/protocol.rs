//! Wire-format protocol shared by every front-end of the simulator.
//!
//! Both `sim_server` (tokio + WebSocket) and `sim_wasm` (wasm-bindgen +
//! Web Worker) use the same `TelemetryFrame` / `ControlCommand` JSON shapes
//! and the same `snapshot` / `apply_command` implementations defined here.
//! The dashboard's TypeScript types in `web_dashboard/src/lib/telemetry.ts`
//! mirror these structs.

use bevy_ecs::prelude::*;
use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::autopilot::{Autopilot, AutopilotPhase};
use crate::clock::SimClock;
use crate::components::{
    CommandedWrench, PropulsionDrive, PropulsionType, RadiationModel, RigidBody, Spacecraft,
};
use crate::ephemeris::{naif, EphemerisCache, AU, MU_SUN};
use crate::mission::Mission;
use crate::mode::{SimMode, SimModeState};
use crate::nav::{NavEstimate, NavFilter};
use crate::sensors::{LatestSensorPack, SensorConfig, SensorPack};
use crate::thrust_controller::{ThrustController, ThrustMode};
use crate::{ExpanseSim, SimConfig, SimTime};

// ────────────────────────────────────────────────────────────────────────
// JSON wire types
// ────────────────────────────────────────────────────────────────────────

/// Player / dashboard command pushed over the transport.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type")]
pub enum ControlCommand {
    #[serde(rename = "set_warp")]
    SetWarp { warp: f64 },
    #[serde(rename = "set_paused")]
    SetPaused { paused: bool },
    #[serde(rename = "set_thrust")]
    SetThrust { thrust: [f64; 3] },
    #[serde(rename = "set_drive")]
    SetDrive { drive: String },
    #[serde(rename = "set_attitude_rate")]
    SetAttitudeRate { angular_velocity: [f64; 3] },
    #[serde(rename = "set_mission")]
    SetMission {
        source: Option<i32>,
        target: Option<i32>,
    },
    /// Re-spawn the spacecraft on the source body's orbit.
    #[serde(rename = "stage_at_source")]
    StageAtSource,
    #[serde(rename = "set_thrust_mode")]
    SetThrustMode { mode: String, body: Option<i32> },
    #[serde(rename = "set_thrust_magnitude")]
    SetThrustMagnitude { magnitude: f64 },
    /// Engage / disengage the brachistochrone rendezvous autopilot.
    #[serde(rename = "set_autopilot")]
    SetAutopilot {
        engaged: bool,
        accel_g: Option<f64>,
    },
    #[serde(rename = "set_sensor_config")]
    SetSensorConfig {
        enabled: Option<bool>,
        range_relative_sigma: Option<f64>,
        range_absolute_sigma_m: Option<f64>,
        range_rate_sigma_m_s: Option<f64>,
        bearing_sigma_rad: Option<f64>,
        accel_sigma_m_s2: Option<f64>,
        gyro_sigma_rad_s: Option<f64>,
        star_tracker_sigma_rad: Option<f64>,
        light_time_delay: Option<bool>,
    },
    #[serde(rename = "set_mode")]
    SetMode { mode: String },
    /// One-shot mission kickoff: set mode → stage → engage in one transaction.
    /// `warp` optionally bumps the time-warp at engagement so the operator
    /// doesn't have to watch a 3-day transit at 60× warp.
    #[serde(rename = "start_mission")]
    StartMission {
        source: Option<i32>,
        target: Option<i32>,
        accel_g: Option<f64>,
        warp: Option<f64>,
    },
    #[serde(rename = "set_nav_filter")]
    SetNavFilter {
        enabled: bool,
        init_sigma_r_m: Option<f64>,
        init_sigma_v_m_s: Option<f64>,
    },
    #[serde(rename = "reset")]
    Reset,
}

/// One broadcasted frame of telemetry.
#[derive(Debug, Clone, Serialize)]
pub struct TelemetryFrame {
    pub sim_time: f64,
    pub warp: f64,
    pub paused: bool,
    pub bodies: Vec<BodySnapshot>,
    pub spacecraft: Option<SpacecraftSnapshot>,
    pub mission: MissionSnapshot,
    pub thrust_controller: ThrustControllerSnapshot,
    pub autopilot: AutopilotSnapshot,
    pub sensors: SensorPack,
    pub nav: NavEstimate,
    pub mode: String,
    pub tick: u64,
    /// Effective sim-seconds advanced per wall-second over the last second.
    pub effective_warp: f64,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct MissionSnapshot {
    pub source: Option<i32>,
    pub target: Option<i32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ThrustControllerSnapshot {
    pub mode: String,
    pub body: Option<i32>,
    pub magnitude: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct AutopilotSnapshot {
    pub engaged: bool,
    pub phase: String,
    pub accel_g: f64,
    pub range_m: f64,
    pub closing_m_s: f64,
    pub eta_s: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct BodySnapshot {
    pub id: i32,
    pub name: String,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub radius: f64,
    /// Gravitational parameter μ = G·M (m³/s²). Exposed so the dashboard can
    /// run a client-side forecast for the predicted-trajectory overlay.
    pub mu: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct SpacecraftSnapshot {
    pub id: u32,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude: [f64; 4], // x,y,z,w
    pub angular_velocity: [f64; 3],
    pub mass: f64,
    pub propellant_mass: f64,
    pub thrust_command: [f64; 3],
    pub drive: String,
    pub isp: f64,
    pub max_thrust: f64,
}

/// Parameters needed by the `Reset` command path to rebuild the simulator.
/// Lives outside the sim's own resources because the runner owns these
/// configuration knobs (dt, initial warp, gravity toggle).
#[derive(Debug, Clone, Copy)]
pub struct ResetParams {
    pub dt: f64,
    pub initial_warp: f64,
    pub apply_gravity: bool,
}

impl Default for ResetParams {
    fn default() -> Self {
        Self {
            dt: 0.01,
            initial_warp: 1.0,
            apply_gravity: true,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────
// Default scenario setup
// ────────────────────────────────────────────────────────────────────────

/// Build a fresh `ExpanseSim` with the same resources / schedule that both
/// frontends use: Mission, ThrustController, default Earth → Mars mission,
/// and the operator-thrust system wired before propulsion.
pub fn build_default_sim(params: ResetParams) -> ExpanseSim {
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: params.dt,
        warp: params.initial_warp,
        apply_gravity: params.apply_gravity,
        load_default_bodies: true,
        ..Default::default()
    });
    sim.world.insert_resource(Mission::new(naif::EARTH, naif::MARS));
    sim.world.insert_resource(ThrustController::default());
    sim.schedule.add_systems(
        crate::thrust_controller::thrust_controller_system
            .before(crate::propulsion::propulsion_system),
    );
    sim
}

/// Spawn the demo Rocinante on a slightly inclined heliocentric orbit
/// between Earth and Mars. Returns the Entity for the runner to track.
pub fn spawn_default_spacecraft(sim: &mut ExpanseSim) -> Entity {
    let r = 1.3 * AU;
    let v_circ = (MU_SUN / r).sqrt();
    sim.world
        .spawn((
            Spacecraft { id: 1 },
            RigidBody {
                position: DVec3::new(r * 0.7, r * 0.7, 0.04 * AU),
                velocity: DVec3::new(-v_circ * 0.7, v_circ * 0.7, 0.0),
                attitude: DQuat::IDENTITY,
                angular_velocity: DVec3::ZERO,
                mass: 250_000.0,
                inertia: DMat3::IDENTITY * 5_000_000.0,
            },
            CommandedWrench::default(),
            PropulsionDrive {
                drive_type: PropulsionType::Brachistochrone,
                isp: 12_000.0,
                propellant_mass: 200_000.0,
                max_thrust: 5.0e7,
                ..Default::default()
            },
            // ~50 m² with cR=1.6: physically tiny on the brachistochrone,
            // visible to the EKF as a small systematic over long coast phases.
            RadiationModel { area_m2: 50.0, cr: 1.6 },
        ))
        .id()
}

// ────────────────────────────────────────────────────────────────────────
// Snapshot — read the world into a JSON-serializable frame
// ────────────────────────────────────────────────────────────────────────

fn phase_str(p: AutopilotPhase) -> String {
    match p {
        AutopilotPhase::Idle => "idle",
        AutopilotPhase::Boost => "boost",
        AutopilotPhase::Brake => "brake",
        AutopilotPhase::Arrived => "arrived",
        AutopilotPhase::Hold => "hold",
    }
    .into()
}

pub fn snapshot(
    sim: &ExpanseSim,
    spacecraft: Entity,
    tick: u64,
    effective_warp: f64,
) -> TelemetryFrame {
    let sim_time = sim.world.resource::<SimTime>().time;
    let clock = *sim.world.resource::<SimClock>();

    let bodies = sim
        .world
        .get_resource::<EphemerisCache>()
        .map(|c| {
            c.all_states()
                .into_iter()
                .map(|(b, s)| BodySnapshot {
                    id: b.id,
                    name: b.name.to_string(),
                    position: s.position.to_array(),
                    velocity: s.velocity.to_array(),
                    radius: b.radius,
                    mu: b.mu,
                })
                .collect()
        })
        .unwrap_or_default();

    let spacecraft_snap = sim.world.get_entity(spacecraft).ok().and_then(|e| {
        let rb = e.get::<RigidBody>()?;
        let drive = e.get::<PropulsionDrive>().cloned().unwrap_or_default();
        let sc = e.get::<Spacecraft>()?;
        Some(SpacecraftSnapshot {
            id: sc.id,
            position: rb.position.to_array(),
            velocity: rb.velocity.to_array(),
            attitude: [rb.attitude.x, rb.attitude.y, rb.attitude.z, rb.attitude.w],
            angular_velocity: rb.angular_velocity.to_array(),
            mass: rb.mass,
            propellant_mass: drive.propellant_mass,
            thrust_command: drive.thrust_command.to_array(),
            drive: match drive.drive_type {
                PropulsionType::Conventional => "conventional".into(),
                PropulsionType::Brachistochrone => "brachistochrone".into(),
            },
            isp: drive.isp,
            max_thrust: drive.max_thrust,
        })
    });

    let mission = sim
        .world
        .get_resource::<Mission>()
        .map(|m| MissionSnapshot {
            source: m.source_body,
            target: m.target_body,
        })
        .unwrap_or_default();

    let thrust_controller = sim
        .world
        .get_resource::<ThrustController>()
        .map(|c| ThrustControllerSnapshot {
            mode: c.mode_str().to_string(),
            body: c.target_body(),
            magnitude: c.magnitude_n,
        })
        .unwrap_or(ThrustControllerSnapshot {
            mode: "off".into(),
            body: None,
            magnitude: 0.0,
        });

    let autopilot = sim
        .world
        .get_resource::<Autopilot>()
        .map(|a| AutopilotSnapshot {
            engaged: a.engaged,
            phase: phase_str(a.phase),
            accel_g: a.accel_g,
            range_m: a.range_m,
            closing_m_s: a.closing_m_s,
            eta_s: a.eta_s,
        })
        .unwrap_or(AutopilotSnapshot {
            engaged: false,
            phase: "idle".into(),
            accel_g: 0.0,
            range_m: f64::INFINITY,
            closing_m_s: 0.0,
            eta_s: f64::INFINITY,
        });

    let sensors = sim
        .world
        .get_resource::<LatestSensorPack>()
        .map(|p| p.0.clone())
        .unwrap_or_default();

    let nav = sim
        .world
        .get_resource::<NavEstimate>()
        .cloned()
        .unwrap_or_default();

    let mode = sim
        .world
        .get_resource::<SimModeState>()
        .map(|m| match m.mode {
            SimMode::Sandbox => "sandbox",
            SimMode::Mission => "mission",
        })
        .unwrap_or("sandbox")
        .to_string();

    TelemetryFrame {
        sim_time,
        warp: clock.warp,
        paused: clock.paused,
        bodies,
        spacecraft: spacecraft_snap,
        mission,
        thrust_controller,
        autopilot,
        sensors,
        nav,
        mode,
        tick,
        effective_warp,
    }
}

// ────────────────────────────────────────────────────────────────────────
// apply_command — mutate the world from a JSON command
// ────────────────────────────────────────────────────────────────────────

/// Apply a single `ControlCommand` to the simulator. The Entity reference
/// is `&mut` because the `Reset` command rebuilds the sim and respawns the
/// spacecraft, returning a new Entity for the caller to track.
pub fn apply_command(
    sim: &mut ExpanseSim,
    spacecraft_entity: &mut Entity,
    cmd: ControlCommand,
    reset_params: ResetParams,
) {
    match cmd {
        ControlCommand::SetWarp { warp } => sim.set_warp(warp),
        ControlCommand::SetPaused { paused } => sim.set_paused(paused),
        ControlCommand::SetThrust { thrust } => {
            if let Ok(mut entity) = sim.world.get_entity_mut(*spacecraft_entity) {
                if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
                    drive.thrust_command = DVec3::from_array(thrust);
                }
            }
        }
        ControlCommand::SetDrive { drive } => {
            if let Ok(mut entity) = sim.world.get_entity_mut(*spacecraft_entity) {
                if let Some(mut d) = entity.get_mut::<PropulsionDrive>() {
                    d.drive_type = match drive.as_str() {
                        "brachistochrone" | "brach" | "epstein" => {
                            PropulsionType::Brachistochrone
                        }
                        _ => PropulsionType::Conventional,
                    };
                }
            }
        }
        ControlCommand::SetAttitudeRate { angular_velocity } => {
            if let Ok(mut entity) = sim.world.get_entity_mut(*spacecraft_entity) {
                if let Some(mut rb) = entity.get_mut::<RigidBody>() {
                    rb.angular_velocity = DVec3::from_array(angular_velocity);
                }
            }
        }
        ControlCommand::SetThrustMode { mode, body } => {
            let parsed = ThrustController::parse_mode(&mode, body);
            if let Some(mut ctrl) = sim.world.get_resource_mut::<ThrustController>() {
                ctrl.mode = parsed;
            }
            if matches!(parsed, ThrustMode::Off) {
                if let Ok(mut entity) = sim.world.get_entity_mut(*spacecraft_entity) {
                    if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
                        drive.thrust_command = DVec3::ZERO;
                    }
                }
            }
        }
        ControlCommand::SetThrustMagnitude { magnitude } => {
            if let Some(mut ctrl) = sim.world.get_resource_mut::<ThrustController>() {
                ctrl.magnitude_n = magnitude.max(0.0);
            }
        }
        ControlCommand::SetMission { source, target } => {
            if let Some(mut mission) = sim.world.get_resource_mut::<Mission>() {
                mission.source_body = source;
                mission.target_body = target;
            }
        }
        ControlCommand::StageAtSource => {
            stage_at_source(sim, *spacecraft_entity);
        }
        ControlCommand::SetAutopilot { engaged, accel_g } => {
            if let Some(mut ap) = sim.world.get_resource_mut::<Autopilot>() {
                ap.engaged = engaged;
                if let Some(g) = accel_g {
                    ap.accel_g = g.max(0.0);
                }
                if !engaged {
                    ap.phase = AutopilotPhase::Idle;
                }
            }
            if !engaged {
                if let Ok(mut entity) = sim.world.get_entity_mut(*spacecraft_entity) {
                    if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
                        drive.thrust_command = DVec3::ZERO;
                    }
                }
            }
        }
        ControlCommand::SetSensorConfig {
            enabled,
            range_relative_sigma,
            range_absolute_sigma_m,
            range_rate_sigma_m_s,
            bearing_sigma_rad,
            accel_sigma_m_s2,
            gyro_sigma_rad_s,
            star_tracker_sigma_rad,
            light_time_delay,
        } => {
            if let Some(mut cfg) = sim.world.get_resource_mut::<SensorConfig>() {
                if let Some(v) = enabled { cfg.enabled = v; }
                if let Some(v) = range_relative_sigma { cfg.range_relative_sigma = v.max(0.0); }
                if let Some(v) = range_absolute_sigma_m { cfg.range_absolute_sigma_m = v.max(0.0); }
                if let Some(v) = range_rate_sigma_m_s { cfg.range_rate_sigma_m_s = v.max(0.0); }
                if let Some(v) = bearing_sigma_rad { cfg.bearing_sigma_rad = v.max(0.0); }
                if let Some(v) = accel_sigma_m_s2 { cfg.accel_sigma_m_s2 = v.max(0.0); }
                if let Some(v) = gyro_sigma_rad_s { cfg.gyro_sigma_rad_s = v.max(0.0); }
                if let Some(v) = star_tracker_sigma_rad {
                    cfg.star_tracker_sigma_rad = v.max(0.0);
                }
                if let Some(v) = light_time_delay { cfg.light_time_delay = v; }
            }
        }
        ControlCommand::SetMode { mode } => {
            let new_mode = match mode.as_str() {
                "mission" => SimMode::Mission,
                _ => SimMode::Sandbox,
            };
            let was = sim
                .world
                .get_resource::<SimModeState>()
                .map(|m| m.mode)
                .unwrap_or(SimMode::Sandbox);
            if let Some(mut state) = sim.world.get_resource_mut::<SimModeState>() {
                state.mode = new_mode;
            }
            if was != new_mode && matches!(new_mode, SimMode::Sandbox) {
                // Leaving Mission: cut autopilot + thrust so the ship coasts.
                if let Some(mut ap) = sim.world.get_resource_mut::<Autopilot>() {
                    ap.engaged = false;
                }
                if let Ok(mut entity) = sim.world.get_entity_mut(*spacecraft_entity) {
                    if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
                        drive.thrust_command = DVec3::ZERO;
                    }
                }
            }
        }
        ControlCommand::SetNavFilter {
            enabled,
            init_sigma_r_m,
            init_sigma_v_m_s,
        } => {
            if let Some(mut nf) = sim.world.get_resource_mut::<NavFilter>() {
                let want_reset = !nf.enabled && enabled;
                nf.enabled = enabled;
                if let Some(s) = init_sigma_r_m { nf.init_sigma_r_m = s.max(0.0); }
                if let Some(s) = init_sigma_v_m_s { nf.init_sigma_v_m_s = s.max(0.0); }
                if want_reset {
                    nf.initialized = false;
                }
            }
        }
        ControlCommand::StartMission { source, target, accel_g, warp } => {
            if source.is_some() || target.is_some() {
                if let Some(mut m) = sim.world.get_resource_mut::<Mission>() {
                    if let Some(s) = source { m.source_body = Some(s); }
                    if let Some(t) = target { m.target_body = Some(t); }
                }
            }
            if let Some(mut state) = sim.world.get_resource_mut::<SimModeState>() {
                state.mode = SimMode::Mission;
            }
            stage_at_source(sim, *spacecraft_entity);
            if let Some(mut ap) = sim.world.get_resource_mut::<Autopilot>() {
                ap.engaged = true;
                if let Some(g) = accel_g {
                    ap.accel_g = g.max(0.0);
                }
            }
            if let Some(w) = warp {
                sim.set_warp(w.max(0.0));
            }
        }
        ControlCommand::Reset => {
            *sim = build_default_sim(reset_params);
            *spacecraft_entity = spawn_default_spacecraft(sim);
        }
    }
}

fn stage_at_source(sim: &mut ExpanseSim, spacecraft_entity: Entity) {
    let source = sim
        .world
        .get_resource::<Mission>()
        .and_then(|m| m.source_body);
    let Some(source_id) = source else { return };
    let state = sim
        .world
        .get_resource::<EphemerisCache>()
        .and_then(|c| c.get(source_id));
    let Some(state) = state else { return };
    if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
        if let Some(mut rb) = entity.get_mut::<RigidBody>() {
            rb.position = state.position;
            rb.velocity = state.velocity;
            rb.attitude = DQuat::IDENTITY;
            rb.angular_velocity = DVec3::ZERO;
        }
        if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
            drive.thrust_command = DVec3::ZERO;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn new_sim_with_ship() -> (ExpanseSim, Entity) {
        let mut sim = build_default_sim(ResetParams::default());
        let id = spawn_default_spacecraft(&mut sim);
        (sim, id)
    }

    #[test]
    fn snapshot_includes_all_default_bodies() {
        let (sim, id) = new_sim_with_ship();
        let frame = snapshot(&sim, id, 0, 0.0);
        // Default roster: Sun + 8 planets + Luna + Phobos + Deimos = 12.
        assert_eq!(frame.bodies.len(), 12);
        assert!(frame.spacecraft.is_some());
        assert_eq!(frame.mode, "sandbox");
        assert_eq!(frame.mission.source, Some(naif::EARTH));
        assert_eq!(frame.mission.target, Some(naif::MARS));
    }

    #[test]
    fn apply_set_warp_round_trips() {
        let (mut sim, mut id) = new_sim_with_ship();
        let params = ResetParams::default();
        apply_command(
            &mut sim,
            &mut id,
            ControlCommand::SetWarp { warp: 250.0 },
            params,
        );
        let frame = snapshot(&sim, id, 0, 0.0);
        assert!((frame.warp - 250.0).abs() < 1e-9);
    }

    #[test]
    fn start_mission_engages_autopilot() {
        let (mut sim, mut id) = new_sim_with_ship();
        let params = ResetParams::default();
        apply_command(
            &mut sim,
            &mut id,
            ControlCommand::StartMission {
                source: Some(naif::EARTH),
                target: Some(naif::MARS),
                accel_g: Some(2.0),
                warp: None,
            },
            params,
        );
        let frame = snapshot(&sim, id, 0, 0.0);
        assert_eq!(frame.mode, "mission");
        assert!(frame.autopilot.engaged);
        assert!((frame.autopilot.accel_g - 2.0).abs() < 1e-9);
    }

    #[test]
    fn reset_rebuilds_sim_and_respawns_spacecraft() {
        let (mut sim, mut id) = new_sim_with_ship();
        let params = ResetParams::default();
        // First, switch mode to mission so we can verify reset restores sandbox.
        apply_command(
            &mut sim,
            &mut id,
            ControlCommand::SetMode { mode: "mission".into() },
            params,
        );
        apply_command(&mut sim, &mut id, ControlCommand::Reset, params);
        let frame = snapshot(&sim, id, 0, 0.0);
        assert_eq!(frame.mode, "sandbox");
        assert!(frame.spacecraft.is_some());
        assert!(!frame.autopilot.engaged);
    }
}
