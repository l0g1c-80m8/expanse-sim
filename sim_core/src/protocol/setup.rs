//! Default scenario construction — used by both the WebSocket server and
//! the in-browser WASM runner to spin up an identical sim instance.

use bevy_ecs::prelude::*;
use glam::{DMat3, DQuat, DVec3};

use crate::components::{
    CommandedWrench, PropulsionDrive, PropulsionType, RadiationModel, RigidBody, Spacecraft,
};
use crate::ephemeris::{naif, park_orbit_state, AU, MU_SUN};
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

/// Spawn the demo Rocinante in a low Earth parking orbit so the scene opens
/// on a coherent "ship at the source body" view. Falls back to a free-space
/// orbit between Earth and Mars if the ephemeris isn't loaded (synthetic test
/// configs). Returns the Entity for the runner to track.
pub fn spawn_default_spacecraft(sim: &mut ExpanseSim) -> Entity {
    let (position, velocity) = default_spacecraft_state(sim);
    sim.world
        .spawn((
            Spacecraft { id: 1 },
            RigidBody {
                position,
                velocity,
                attitude: DQuat::IDENTITY,
                angular_velocity: DVec3::ZERO,
                mass: 250_000.0,
                inertia: DMat3::IDENTITY * 5_000_000.0,
            },
            CommandedWrench::default(),
            PropulsionDrive {
                drive_type: PropulsionType::Brachistochrone,
                // Expanse-style fusion drives have absurdly high effective
                // exhaust velocities — the old 12 000 s Isp drained the tank
                // in minutes of 1 g burn, so the autopilot never finished a
                // solar-system transit. 1e7 s is in the right ballpark for
                // a torch drive: a full Earth → Mars brachistochrone at 1 g
                // uses well under 1 % of the propellant budget.
                isp: 1.0e7,
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

/// Pick the spawn (position, velocity) — Earth parking orbit if the
/// ephemeris cache is loaded, otherwise the legacy free-space placement so
/// tests / minimal configs still see a sane starting state.
fn default_spacecraft_state(sim: &ExpanseSim) -> (DVec3, DVec3) {
    if let Some(cache) = sim.world.get_resource::<crate::ephemeris::EphemerisCache>()
        && let Some(earth_state) = cache.get(naif::EARTH)
        && let Some(earth_params) = cache.bodies().into_iter().find(|b| b.id == naif::EARTH)
        && let Some(parked) = park_orbit_state(earth_state, earth_params)
    {
        return (parked.position, parked.velocity);
    }
    let r = 1.3 * AU;
    let v_circ = (MU_SUN / r).sqrt();
    (
        DVec3::new(r * 0.7, r * 0.7, 0.04 * AU),
        DVec3::new(-v_circ * 0.7, v_circ * 0.7, 0.0),
    )
}
