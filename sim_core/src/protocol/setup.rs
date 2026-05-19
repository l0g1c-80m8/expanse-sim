//! Default scenario construction — used by both the WebSocket server and
//! the in-browser WASM runner to spin up an identical sim instance.

use bevy_ecs::prelude::*;
use glam::{DMat3, DQuat, DVec3};

use crate::components::{
    CommandedWrench, PropulsionDrive, PropulsionType, RadiationModel, RigidBody, Spacecraft,
};
use crate::ephemeris::{naif, AU, MU_SUN};
use crate::mission::Mission;
use crate::thrust_controller::{thrust_controller_system, ThrustController};
use crate::{ExpanseSim, SimConfig};

/// Build a fresh `ExpanseSim` with the same resources / schedule that both
/// frontends use: Mission, ThrustController, default Earth → Mars mission,
/// and the operator-thrust system wired before propulsion.
pub fn build_default_sim(cfg: SimConfig) -> ExpanseSim {
    let mut sim = ExpanseSim::with_config(cfg);
    sim.world
        .insert_resource(Mission::new(naif::EARTH, naif::MARS));
    sim.world.insert_resource(ThrustController::default());
    sim.schedule.add_systems(
        thrust_controller_system.before(crate::propulsion::propulsion_system),
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
