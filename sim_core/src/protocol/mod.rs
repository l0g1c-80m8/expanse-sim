//! Wire-format protocol shared by every frontend of the simulator.
//!
//! Both `sim_server` (tokio + WebSocket) and `sim_wasm` (wasm-bindgen +
//! Web Worker) use the same `TelemetryFrame` / `ControlCommand` JSON shapes
//! and the same `snapshot` / `apply_command` implementations defined here.
//! The dashboard's TypeScript types in `web_dashboard/src/lib/telemetry.ts`
//! mirror these structs.
//!
//! Submodule layout:
//!
//! * [`types`]    — JSON wire types (TelemetryFrame, ControlCommand, …)
//! * [`setup`]    — `build_default_sim` / `spawn_default_spacecraft`
//! * [`snapshot`] — read the ECS world into a `TelemetryFrame`
//! * [`command`]  — apply a `ControlCommand` to the ECS world

pub mod command;
pub mod setup;
pub mod snapshot;
pub mod types;

pub use command::{apply_command, stage_at_source};
pub use setup::{build_default_sim, spawn_default_spacecraft};
pub use snapshot::snapshot;
pub use types::{
    AutopilotSnapshot, BodySnapshot, ControlCommand, MissionSnapshot, SpacecraftSnapshot,
    TelemetryFrame, ThrustControllerSnapshot,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephemeris::naif;
    use crate::{ExpanseSim, SimConfig};
    use bevy_ecs::prelude::Entity;

    fn new_sim_with_ship() -> (ExpanseSim, Entity) {
        let mut sim = build_default_sim(SimConfig::default());
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
        let cfg = SimConfig::default();
        apply_command(
            &mut sim,
            &mut id,
            ControlCommand::SetWarp { warp: 250.0 },
            cfg,
        );
        let frame = snapshot(&sim, id, 0, 0.0);
        assert!((frame.warp - 250.0).abs() < 1e-9);
    }

    #[test]
    fn start_mission_engages_autopilot() {
        let (mut sim, mut id) = new_sim_with_ship();
        let cfg = SimConfig::default();
        apply_command(
            &mut sim,
            &mut id,
            ControlCommand::StartMission {
                source: Some(naif::EARTH),
                target: Some(naif::MARS),
                accel_g: Some(2.0),
                warp: None,
            },
            cfg,
        );
        let frame = snapshot(&sim, id, 0, 0.0);
        assert_eq!(frame.mode, "mission");
        assert!(frame.autopilot.engaged);
        assert!((frame.autopilot.accel_g - 2.0).abs() < 1e-9);
    }

    #[test]
    fn stage_at_source_parks_ship_in_earth_orbit() {
        let (mut sim, mut id) = new_sim_with_ship();
        let cfg = SimConfig::default();
        apply_command(
            &mut sim,
            &mut id,
            ControlCommand::SetMission {
                source: Some(naif::EARTH),
                target: Some(naif::MARS),
            },
            cfg,
        );
        apply_command(&mut sim, &mut id, ControlCommand::StageAtSource, cfg);

        let frame = snapshot(&sim, id, 0, 0.0);
        let ship_pos = frame
            .spacecraft
            .as_ref()
            .map(|s| glam::DVec3::from_array(s.position))
            .expect("spacecraft snapshot");
        let earth = frame
            .bodies
            .iter()
            .find(|b| b.id == naif::EARTH)
            .expect("earth in roster");
        let earth_pos = glam::DVec3::from_array(earth.position);
        let altitude = (ship_pos - earth_pos).length() - earth.radius;
        assert!(
            altitude > 5.0e4 && altitude < 1.0e8,
            "expected park-orbit altitude in (50 km, 100 Mm), got {:.0} m",
            altitude
        );

        let ship_vel = frame
            .spacecraft
            .as_ref()
            .map(|s| glam::DVec3::from_array(s.velocity))
            .unwrap();
        let earth_vel = glam::DVec3::from_array(earth.velocity);
        let rel_v = (ship_vel - earth_vel).length();
        assert!(
            rel_v > 1_000.0 && rel_v < 20_000.0,
            "expected ~1–20 km/s relative orbital velocity, got {:.1} m/s",
            rel_v
        );
    }

    #[test]
    fn reset_rebuilds_sim_and_respawns_spacecraft() {
        let (mut sim, mut id) = new_sim_with_ship();
        let cfg = SimConfig::default();
        apply_command(
            &mut sim,
            &mut id,
            ControlCommand::SetMode { mode: "mission".into() },
            cfg,
        );
        apply_command(&mut sim, &mut id, ControlCommand::Reset, cfg);
        let frame = snapshot(&sim, id, 0, 0.0);
        assert_eq!(frame.mode, "sandbox");
        assert!(frame.spacecraft.is_some());
        assert!(!frame.autopilot.engaged);
    }
}
