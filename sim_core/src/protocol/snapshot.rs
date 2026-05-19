//! Read the ECS world into a JSON-serialisable telemetry frame.

use bevy_ecs::prelude::*;

use crate::autopilot::{Autopilot, AutopilotPhase};
use crate::clock::SimClock;
use crate::components::{PropulsionDrive, PropulsionType, RigidBody, Spacecraft};
use crate::ephemeris::EphemerisCache;
use crate::mission::Mission;
use crate::mode::{SimMode, SimModeState};
use crate::nav::NavEstimate;
use crate::sensors::LatestSensorPack;
use crate::thrust_controller::ThrustController;
use crate::{ExpanseSim, SimTime};

use super::types::{
    AutopilotSnapshot, BodySnapshot, MissionSnapshot, SpacecraftSnapshot, TelemetryFrame,
    ThrustControllerSnapshot,
};

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
