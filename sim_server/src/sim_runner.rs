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
use sim_core::ephemeris::{AU, MU_SUN};
use sim_core::{
    CommandedWrench, EphemerisCache, ExpanseSim, PropulsionDrive, PropulsionType, RigidBody,
    SimClock, SimConfig, Spacecraft,
};
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
    pub tick: u64,
}

#[derive(Debug, Clone, Serialize)]
pub struct BodySnapshot {
    pub id: i32,
    pub name: String,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub radius: f64,
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
        let mut spacecraft_id = spawn_default_spacecraft(&mut sim);
        let mut tick: u64 = 0;
        // Wall-clock budget accumulator: we may need to retire many sim ticks
        // per loop iteration when warp is high.
        let mut budget: f64 = 0.0;
        let mut last = Instant::now();

        loop {
            let now = Instant::now();
            let wall = (now - last).as_secs_f64();
            last = now;

            // Drain pending control commands before stepping.
            while let Ok(cmd) = command_rx.try_recv() {
                apply_command(&mut sim, spacecraft_id, cmd, &settings, &mut spacecraft_id);
            }

            let clock = *sim.world.resource::<SimClock>();
            let dt = sim.world.resource::<sim_core::SimTime>().dt;
            if !clock.paused {
                budget += wall * clock.warp;
            }

            // Cap the per-iteration tick count so a long stall doesn't trigger
            // a runaway catch-up burst that hogs the executor.
            let mut ticks_to_run = (budget / dt).floor() as u32;
            ticks_to_run = ticks_to_run.min(5_000);
            for _ in 0..ticks_to_run {
                sim.tick();
                tick += 1;
                if tick % settings.telemetry_stride as u64 == 0 {
                    let frame = snapshot(&sim, spacecraft_id, tick);
                    *latest_pub.lock() = Some(frame.clone());
                    let _ = telemetry_pub.send(frame);
                }
            }
            budget -= ticks_to_run as f64 * dt;

            // Even paused / idle clients should see fresh frames so the
            // dashboard's body positions don't go stale.
            if ticks_to_run == 0 {
                let frame = snapshot(&sim, spacecraft_id, tick);
                *latest_pub.lock() = Some(frame.clone());
                let _ = telemetry_pub.send(frame);
            }

            std::thread::sleep(settings.loop_interval);
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
    // Place the demo Rocinante in a near-Earth heliocentric orbit so the
    // dashboard has something interesting to render at zoom level 1.
    let r = AU + 1.5e9;
    let v_circ = (MU_SUN / r).sqrt();
    sim.world
        .spawn((
            Spacecraft { id: 1 },
            RigidBody {
                position: DVec3::new(r, 0.0, 0.0),
                velocity: DVec3::new(0.0, v_circ, 0.0),
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

fn snapshot(sim: &ExpanseSim, spacecraft: Entity, tick: u64) -> TelemetryFrame {
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

    TelemetryFrame {
        sim_time,
        warp: clock.warp,
        paused: clock.paused,
        bodies,
        spacecraft,
        tick,
    }
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
        ControlCommand::Reset => {
            *sim = build_initial_sim(settings);
            *spacecraft_out = spawn_default_spacecraft(sim);
        }
    }
}
