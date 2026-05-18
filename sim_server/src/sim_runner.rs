//! Simulation driver that runs `sim_core::ExpanseSim` in a dedicated
//! tokio task.
//!
//! The driver:
//!
//! * keeps a wall-clock vs sim-clock budget so warp behaves consistently;
//! * drains the command channel between ticks so player input lands
//!   deterministically before the next physics step;
//! * publishes a `TelemetryFrame` on every Nth tick via a `broadcast`
//!   channel that the WebSocket handler subscribes to.

use std::sync::Arc;
use std::time::{Duration, Instant};

use bevy_ecs::prelude::*;
use glam::{DMat3, DQuat, DVec3};
use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use sim_core::ephemeris::{naif, AU, MU_SUN};
use sim_core::{
    CommandedWrench, EphemerisCache, ExpanseSim, Mission, PropulsionDrive, PropulsionType,
    RigidBody, SimClock, SimConfig, Spacecraft,
};

use crate::thrust_controller::{thrust_controller_system, ThrustController};
use tokio::sync::{broadcast, mpsc};


#[derive(Debug, Clone)]
pub struct SimSettings {
    pub dt: f64,
    pub initial_warp: f64,
    pub apply_gravity: bool,
    pub telemetry_stride: u32,
    pub loop_interval: Duration,
}

/// Player / dashboard commands forwarded from the WebSocket handler.
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
    /// Set the mission's source and/or target body. `None` clears that endpoint.
    #[serde(rename = "set_mission")]
    SetMission {
        source: Option<i32>,
        target: Option<i32>,
    },
    /// Re-spawn the spacecraft on the source body's orbit. Convenience used
    /// by the dashboard's "Stage at source" button after picking a mission.
    #[serde(rename = "stage_at_source")]
    StageAtSource,
    /// Operator thrust direction mode (e.g. `"prograde"`, `"toward_target"`).
    /// Optional `body` parameter is used by `toward_body` / `away_from_body`.
    #[serde(rename = "set_thrust_mode")]
    SetThrustMode { mode: String, body: Option<i32> },
    /// Operator thrust magnitude in Newtons. Capped by the drive's max thrust.
    #[serde(rename = "set_thrust_magnitude")]
    SetThrustMagnitude { magnitude: f64 },
    /// Engage / disengage the brachistochrone rendezvous autopilot. When
    /// engaged it consumes the current `mission.target_body` and drives the
    /// ship to a soft arrival.
    #[serde(rename = "set_autopilot")]
    SetAutopilot {
        engaged: bool,
        accel_g: Option<f64>,
    },
    /// Configure the synthetic sensor model.
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
    },
    #[serde(rename = "reset")]
    Reset,
}

/// One broadcasted frame of telemetry. Designed to keep wire size low —
/// planet positions are repeated each frame so the dashboard can drop frames
/// freely without ever rendering a stale planet snapshot.
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
    pub sensors: sim_core::SensorPack,
    pub tick: u64,
    /// Effective sim-seconds advanced per wall-second over the last second.
    /// Useful for spotting cases where the requested warp exceeds the
    /// throughput cap and sim time is falling behind.
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

/// Shared handle injected into Axum routes.
#[derive(Clone)]
pub struct AppState {
    pub telemetry_rx: broadcast::Sender<TelemetryFrame>,
    pub command_tx: mpsc::Sender<ControlCommand>,
    pub latest: Arc<Mutex<Option<TelemetryFrame>>>,
}

pub fn spawn_simulation(settings: SimSettings) -> AppState {
    let (telemetry_tx, _) = broadcast::channel::<TelemetryFrame>(64);
    let (command_tx, mut command_rx) = mpsc::channel::<ControlCommand>(128);
    let latest = Arc::new(Mutex::new(None));

    let telemetry_pub = telemetry_tx.clone();
    let latest_pub = latest.clone();

    tokio::task::spawn_blocking(move || {
        let mut sim = build_initial_sim(&settings);
        // Pre-seed a default Earth → Mars mission so the dashboard renders
        // something the moment it connects. The operator can change either
        // endpoint via the mission panel.
        sim.world.insert_resource(Mission::new(naif::EARTH, naif::MARS));
        sim.world.insert_resource(ThrustController::default());
        // Insert the operator-thrust system before propulsion so its writes
        // are picked up in the same tick.
        sim.schedule
            .add_systems(thrust_controller_system.before(sim_core::propulsion_system));

        let mut spacecraft_id = spawn_default_spacecraft(&mut sim);
        let mut tick: u64 = 0;
        let mut budget: f64 = 0.0;
        let mut last = Instant::now();

        // Effective-warp tracking — a 1-second rolling window over advanced
        // sim seconds, so the dashboard can show real throughput vs requested
        // warp and the user can spot the throughput cap kicking in.
        let mut last_warp_sample = Instant::now();
        let mut sim_seconds_in_window: f64 = 0.0;
        let mut effective_warp: f64 = 0.0;

        loop {
            let now = Instant::now();
            let wall = (now - last).as_secs_f64();
            last = now;

            while let Ok(cmd) = command_rx.try_recv() {
                apply_command(&mut sim, spacecraft_id, cmd, &settings, &mut spacecraft_id);
            }

            let clock = *sim.world.resource::<SimClock>();
            let dt = sim.world.resource::<sim_core::SimTime>().dt;
            if !clock.paused {
                budget += wall * clock.warp;
            }

            // Cap the per-iteration tick count so a long stall doesn't trigger
            // a runaway catch-up burst. 50k @ 10 ms loop ≈ 5 M ticks/sec,
            // enough to deliver ~250 000× warp at dt = 0.05.
            let mut ticks_to_run = (budget / dt).floor() as u32;
            ticks_to_run = ticks_to_run.min(50_000);
            for _ in 0..ticks_to_run {
                sim.tick();
                tick += 1;
                sim_seconds_in_window += dt;
                if tick % settings.telemetry_stride as u64 == 0 {
                    let frame = snapshot(&sim, spacecraft_id, tick, effective_warp);
                    *latest_pub.lock() = Some(frame.clone());
                    let _ = telemetry_pub.send(frame);
                }
            }
            budget -= ticks_to_run as f64 * dt;

            // Bleed accumulated budget if the cap was hit — the operator
            // wants "best we can do" not "infinitely behind".
            if budget > dt * 50_000.0 {
                budget = dt * 50_000.0;
            }

            // Sample effective warp once per wall-second.
            let elapsed = last_warp_sample.elapsed().as_secs_f64();
            if elapsed >= 1.0 {
                effective_warp = sim_seconds_in_window / elapsed;
                sim_seconds_in_window = 0.0;
                last_warp_sample = Instant::now();
            }

            if ticks_to_run == 0 {
                let frame = snapshot(&sim, spacecraft_id, tick, effective_warp);
                *latest_pub.lock() = Some(frame.clone());
                let _ = telemetry_pub.send(frame);
            }

            // Adaptive pacing: only sleep when there's no pending budget. If
            // we have more work, yield to the scheduler and come back
            // immediately so high-warp throughput isn't capped by the loop
            // interval.
            if clock.paused || budget < dt {
                std::thread::sleep(settings.loop_interval);
            } else {
                std::thread::yield_now();
            }
        }
    });

    AppState {
        telemetry_rx: telemetry_tx,
        command_tx,
        latest,
    }
}

fn build_initial_sim(settings: &SimSettings) -> ExpanseSim {
    ExpanseSim::with_config(SimConfig {
        dt: settings.dt,
        warp: settings.initial_warp,
        apply_gravity: settings.apply_gravity,
        load_default_bodies: true,
        ..Default::default()
    })
}

fn spawn_default_spacecraft(sim: &mut ExpanseSim) -> Entity {
    // Place the demo Rocinante between Earth and Mars on a slightly inclined
    // heliocentric orbit so the dashboard can clearly distinguish it from the
    // planet markers at default zoom.
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
        ))
        .id()
}

fn snapshot(
    sim: &ExpanseSim,
    spacecraft: Entity,
    tick: u64,
    effective_warp: f64,
) -> TelemetryFrame {
    let sim_time = sim.world.resource::<sim_core::SimTime>().time;
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

    let spacecraft = sim.world.get_entity(spacecraft).ok().and_then(|e| {
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
        .map(|c| {
            let body = match c.mode {
                crate::thrust_controller::ThrustMode::TowardBody(id)
                | crate::thrust_controller::ThrustMode::AwayFromBody(id) => Some(id),
                _ => None,
            };
            ThrustControllerSnapshot {
                mode: c.mode_str().to_string(),
                body,
                magnitude: c.magnitude_n,
            }
        })
        .unwrap_or(ThrustControllerSnapshot {
            mode: "off".into(),
            body: None,
            magnitude: 0.0,
        });

    let autopilot = sim
        .world
        .get_resource::<sim_core::Autopilot>()
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
        .get_resource::<sim_core::LatestSensorPack>()
        .map(|p| p.0.clone())
        .unwrap_or_default();

    TelemetryFrame {
        sim_time,
        warp: clock.warp,
        paused: clock.paused,
        bodies,
        spacecraft,
        mission,
        thrust_controller,
        autopilot,
        sensors,
        tick,
        effective_warp,
    }
}

fn phase_str(p: sim_core::AutopilotPhase) -> String {
    match p {
        sim_core::AutopilotPhase::Idle => "idle",
        sim_core::AutopilotPhase::Boost => "boost",
        sim_core::AutopilotPhase::Brake => "brake",
        sim_core::AutopilotPhase::Arrived => "arrived",
        sim_core::AutopilotPhase::Hold => "hold",
    }
    .into()
}

fn apply_command(
    sim: &mut ExpanseSim,
    spacecraft_entity: Entity,
    cmd: ControlCommand,
    settings: &SimSettings,
    spacecraft_out: &mut Entity,
) {
    match cmd {
        ControlCommand::SetWarp { warp } => sim.set_warp(warp),
        ControlCommand::SetPaused { paused } => sim.set_paused(paused),
        ControlCommand::SetThrust { thrust } => {
            if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
                if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
                    drive.thrust_command = DVec3::from_array(thrust);
                }
            }
        }
        ControlCommand::SetDrive { drive } => {
            if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
                if let Some(mut d) = entity.get_mut::<PropulsionDrive>() {
                    d.drive_type = match drive.as_str() {
                        "brachistochrone" | "brach" | "epstein" => PropulsionType::Brachistochrone,
                        _ => PropulsionType::Conventional,
                    };
                }
            }
        }
        ControlCommand::SetAttitudeRate { angular_velocity } => {
            if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
                if let Some(mut rb) = entity.get_mut::<RigidBody>() {
                    rb.angular_velocity = DVec3::from_array(angular_velocity);
                }
            }
        }
        ControlCommand::SetThrustMode { mode, body } => {
            let parsed = ThrustController::parse_mode(&mode, body);
            if let Some(mut ctrl) = sim.world.get_resource_mut::<ThrustController>() {
                ctrl.mode = parsed;
                // Switching to "off" should also stop any in-flight burn that
                // a previous mode wrote into the drive.
                if matches!(parsed, crate::thrust_controller::ThrustMode::Off) {
                    drop(ctrl);
                    if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
                        if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
                            drive.thrust_command = DVec3::ZERO;
                        }
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
            let source = sim.world.get_resource::<Mission>().and_then(|m| m.source_body);
            if let Some(source_id) = source {
                if let Some(cache) = sim.world.get_resource::<EphemerisCache>() {
                    if let Some(state) = cache.get(source_id) {
                        if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
                            if let Some(mut rb) = entity.get_mut::<RigidBody>() {
                                // Sit at the source body's position with its
                                // own heliocentric velocity — the autonomy
                                // stack picks it up from there.
                                rb.position = state.position;
                                rb.velocity = state.velocity;
                                rb.attitude = glam::DQuat::IDENTITY;
                                rb.angular_velocity = DVec3::ZERO;
                            }
                            if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
                                drive.thrust_command = DVec3::ZERO;
                            }
                        }
                    }
                }
            }
        }
        ControlCommand::SetAutopilot { engaged, accel_g } => {
            if let Some(mut ap) = sim.world.get_resource_mut::<sim_core::Autopilot>() {
                ap.engaged = engaged;
                if let Some(g) = accel_g {
                    ap.accel_g = g.max(0.0);
                }
                if !engaged {
                    ap.phase = sim_core::AutopilotPhase::Idle;
                }
            }
            if !engaged {
                // Cut any in-flight thrust the autopilot may have left.
                if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
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
        } => {
            if let Some(mut cfg) = sim.world.get_resource_mut::<sim_core::SensorConfig>() {
                if let Some(v) = enabled { cfg.enabled = v; }
                if let Some(v) = range_relative_sigma { cfg.range_relative_sigma = v.max(0.0); }
                if let Some(v) = range_absolute_sigma_m { cfg.range_absolute_sigma_m = v.max(0.0); }
                if let Some(v) = range_rate_sigma_m_s { cfg.range_rate_sigma_m_s = v.max(0.0); }
                if let Some(v) = bearing_sigma_rad { cfg.bearing_sigma_rad = v.max(0.0); }
                if let Some(v) = accel_sigma_m_s2 { cfg.accel_sigma_m_s2 = v.max(0.0); }
                if let Some(v) = gyro_sigma_rad_s { cfg.gyro_sigma_rad_s = v.max(0.0); }
                if let Some(v) = star_tracker_sigma_rad { cfg.star_tracker_sigma_rad = v.max(0.0); }
            }
        }
        ControlCommand::Reset => {
            *sim = build_initial_sim(settings);
            sim.world.insert_resource(Mission::new(naif::EARTH, naif::MARS));
            sim.world.insert_resource(ThrustController::default());
            sim.schedule.add_systems(
                thrust_controller_system.before(sim_core::propulsion_system),
            );
            *spacecraft_out = spawn_default_spacecraft(sim);
        }
    }
}
