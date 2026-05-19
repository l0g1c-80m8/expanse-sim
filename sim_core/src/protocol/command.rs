//! Mutate the ECS world from a JSON-encoded `ControlCommand`.
//!
//! The `Reset` command rebuilds the simulator from scratch and respawns
//! the spacecraft, returning a new Entity through the `&mut Entity` handle
//! the caller passes in.

use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};

use crate::autopilot::{Autopilot, AutopilotPhase};
use crate::components::{PropulsionDrive, PropulsionType, RigidBody};
use crate::ephemeris::{park_orbit_state, EphemerisCache};
use crate::mission::Mission;
use crate::mode::{SimMode, SimModeState};
use crate::nav::NavFilter;
use crate::sensors::SensorConfig;
use crate::thrust_controller::{ThrustController, ThrustMode};
use crate::{ExpanseSim, SimConfig};

use super::setup::{build_default_sim, spawn_default_spacecraft};
use super::types::ControlCommand;

/// Apply a single `ControlCommand` to the simulator. The Entity reference
/// is `&mut` because the `Reset` command rebuilds the sim and respawns the
/// spacecraft, returning a new Entity for the caller to track.
pub fn apply_command(
    sim: &mut ExpanseSim,
    spacecraft_entity: &mut Entity,
    cmd: ControlCommand,
    reset_cfg: SimConfig,
) {
    match cmd {
        ControlCommand::SetWarp { warp } => sim.set_warp(warp),
        ControlCommand::SetPaused { paused } => sim.set_paused(paused),
        ControlCommand::SetThrust { thrust } => {
            with_drive(sim, *spacecraft_entity, |drive| {
                drive.thrust_command = DVec3::from_array(thrust);
            });
        }
        ControlCommand::SetDrive { drive } => {
            with_drive(sim, *spacecraft_entity, |d| {
                d.drive_type = match drive.as_str() {
                    "brachistochrone" | "brach" | "epstein" => PropulsionType::Brachistochrone,
                    _ => PropulsionType::Conventional,
                };
            });
        }
        ControlCommand::SetAttitudeRate { angular_velocity } => {
            with_rigidbody(sim, *spacecraft_entity, |rb| {
                rb.angular_velocity = DVec3::from_array(angular_velocity);
            });
        }
        ControlCommand::SetThrustMode { mode, body } => {
            let parsed = ThrustController::parse_mode(&mode, body);
            if let Some(mut ctrl) = sim.world.get_resource_mut::<ThrustController>() {
                ctrl.mode = parsed;
            }
            if matches!(parsed, ThrustMode::Off) {
                with_drive(sim, *spacecraft_entity, |drive| {
                    drive.thrust_command = DVec3::ZERO;
                });
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
        ControlCommand::StageAtSource => stage_at_source(sim, *spacecraft_entity),
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
                with_drive(sim, *spacecraft_entity, |drive| {
                    drive.thrust_command = DVec3::ZERO;
                });
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
                with_drive(sim, *spacecraft_entity, |drive| {
                    drive.thrust_command = DVec3::ZERO;
                });
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
            if (source.is_some() || target.is_some())
                && let Some(mut m) = sim.world.get_resource_mut::<Mission>() {
                    if let Some(s) = source { m.source_body = Some(s); }
                    if let Some(t) = target { m.target_body = Some(t); }
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
            *sim = build_default_sim(reset_cfg);
            *spacecraft_entity = spawn_default_spacecraft(sim);
        }
    }
}

/// Stage the spacecraft in a circular parking orbit around the mission's
/// current source body.
pub fn stage_at_source(sim: &mut ExpanseSim, spacecraft_entity: Entity) {
    let source = sim
        .world
        .get_resource::<Mission>()
        .and_then(|m| m.source_body);
    let Some(source_id) = source else { return };
    let cache = sim.world.get_resource::<EphemerisCache>();
    let Some(cache) = cache else { return };
    let Some(state) = cache.get(source_id) else { return };
    let params = cache.bodies().into_iter().find(|b| b.id == source_id);

    // Stage in a circular parking orbit around the source body when we have
    // its gravitational parameters. For synthetic / massless test bodies we
    // fall through to the body-centre behaviour the integration tests assume.
    let staged = params
        .and_then(|p| park_orbit_state(state, p))
        .unwrap_or(state);

    if let Ok(mut entity) = sim.world.get_entity_mut(spacecraft_entity) {
        if let Some(mut rb) = entity.get_mut::<RigidBody>() {
            rb.position = staged.position;
            rb.velocity = staged.velocity;
            rb.attitude = DQuat::IDENTITY;
            rb.angular_velocity = DVec3::ZERO;
        }
        if let Some(mut drive) = entity.get_mut::<PropulsionDrive>() {
            drive.thrust_command = DVec3::ZERO;
        }
    }
}

// ── small ECS access helpers — keep the match arms readable ───────────────

fn with_drive<F>(sim: &mut ExpanseSim, entity: Entity, f: F)
where
    F: FnOnce(&mut PropulsionDrive),
{
    if let Ok(mut e) = sim.world.get_entity_mut(entity)
        && let Some(mut drive) = e.get_mut::<PropulsionDrive>() {
            f(&mut drive);
        }
}

fn with_rigidbody<F>(sim: &mut ExpanseSim, entity: Entity, f: F)
where
    F: FnOnce(&mut RigidBody),
{
    if let Ok(mut e) = sim.world.get_entity_mut(entity)
        && let Some(mut rb) = e.get_mut::<RigidBody>() {
            f(&mut rb);
        }
}
