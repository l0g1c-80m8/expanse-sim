//! End-to-end integration tests for the ExpanseSim physics plant.

use approx::assert_relative_eq;
use bevy_ecs::prelude::*;
use glam::{DMat3, DQuat, DVec3};
use sim_core::ephemeris::{naif, AU, MU_SUN};
use sim_core::prelude::*;

fn spawn_default_spacecraft(sim: &mut ExpanseSim) -> Entity {
    let r = AU;
    let v_circ = (MU_SUN / r).sqrt();
    sim.world
        .spawn((
            Spacecraft { id: 1 },
            RigidBody {
                position: DVec3::new(r, 0.0, 0.0),
                velocity: DVec3::new(0.0, v_circ, 0.0),
                attitude: DQuat::IDENTITY,
                angular_velocity: DVec3::ZERO,
                mass: 50_000.0,
                inertia: DMat3::IDENTITY * 1_000_000.0,
            },
            CommandedWrench::default(),
            PropulsionDrive {
                drive_type: PropulsionType::Conventional,
                isp: 320.0,
                propellant_mass: 10_000.0,
                max_thrust: 1.5e6,
                ..Default::default()
            },
        ))
        .id()
}

#[test]
fn fresh_sim_ticks_advance_clock_deterministically() {
    let mut sim = ExpanseSim::with_config(SimConfig {
        load_default_bodies: false,
        apply_gravity: false,
        ..Default::default()
    });
    sim.tick_n(1000);
    assert_relative_eq!(sim.sim_time(), 10.0, epsilon = 1e-9);
}

#[test]
fn warp_does_not_change_dt() {
    let mut sim = ExpanseSim::with_config(SimConfig {
        load_default_bodies: false,
        apply_gravity: false,
        ..Default::default()
    });
    sim.set_warp(1000.0);
    sim.tick_n(100);
    // 100 ticks × 0.01 dt = 1.0 s of sim time, regardless of warp.
    assert_relative_eq!(sim.sim_time(), 1.0, epsilon = 1e-9);
}

#[test]
fn spacecraft_in_circular_orbit_stays_in_orbit() {
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: 10.0,
        load_default_bodies: true,
        apply_gravity: true,
        ..Default::default()
    });
    let id = spawn_default_spacecraft(&mut sim);

    // Run ~1 day of sim time at 10 s/tick.
    sim.tick_n(8640);

    let rb = sim.world.entity(id).get::<RigidBody>().unwrap();
    let r = rb.position.length();
    // Should still be in roughly a 1-AU orbit (within 0.5%).
    assert!(
        (r - AU).abs() / AU < 5e-3,
        "orbital radius drifted: r/AU = {}",
        r / AU
    );
}

#[test]
fn brachistochrone_burn_increases_speed_and_drops_mass() {
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: 1.0,
        load_default_bodies: false,
        apply_gravity: false,
        ..Default::default()
    });
    let id = sim
        .world
        .spawn((
            Spacecraft { id: 1 },
            RigidBody {
                mass: 100_000.0,
                inertia: DMat3::IDENTITY * 1.0e6,
                ..Default::default()
            },
            CommandedWrench::default(),
            PropulsionDrive {
                drive_type: PropulsionType::Brachistochrone,
                thrust_command: DVec3::new(1.0e6, 0.0, 0.0),
                isp: 12_000.0,
                propellant_mass: 50_000.0,
                max_thrust: 5.0e6,
            },
        ))
        .id();

    let v0 = sim.world.entity(id).get::<RigidBody>().unwrap().velocity.length();
    let m0 = sim.world.entity(id).get::<RigidBody>().unwrap().mass;
    sim.tick_n(60); // 60 seconds of continuous burn
    let rb = sim.world.entity(id).get::<RigidBody>().unwrap();

    assert!(rb.velocity.length() > v0, "should accelerate");
    assert!(rb.mass < m0, "should burn propellant");
    assert!(
        rb.velocity.x > 0.0,
        "body-frame thrust on +x with identity attitude → +x velocity"
    );
}

#[test]
fn body_frame_thrust_follows_attitude() {
    // Spacecraft pointing along +x burns thrust along +x in body frame.
    // After rotating attitude 90° about z, the same body-frame thrust should
    // accelerate the craft along +y in the inertial frame.
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: 0.1,
        load_default_bodies: false,
        apply_gravity: false,
        ..Default::default()
    });
    let id = sim
        .world
        .spawn((
            Spacecraft { id: 1 },
            RigidBody {
                attitude: DQuat::from_axis_angle(DVec3::Z, std::f64::consts::FRAC_PI_2),
                mass: 1000.0,
                ..Default::default()
            },
            CommandedWrench::default(),
            PropulsionDrive {
                drive_type: PropulsionType::Conventional,
                thrust_command: DVec3::new(1000.0, 0.0, 0.0),
                isp: 300.0,
                propellant_mass: 1000.0,
                max_thrust: 5000.0,
            },
        ))
        .id();
    sim.tick_n(10);
    let rb = sim.world.entity(id).get::<RigidBody>().unwrap();
    assert!(rb.velocity.y > 0.0, "+x body thrust @ 90° z-rotation should push +y inertial");
    assert!(rb.velocity.x.abs() < rb.velocity.y * 1e-3);
}

#[test]
fn ephemeris_planet_orbits_around_sun() {
    let sim = ExpanseSim::new();
    let cache = sim.world.resource::<sim_core::prelude::EphemerisCache>().clone();
    let earth = cache.get(naif::EARTH).unwrap();
    assert!(
        (earth.position.length() - AU).abs() / AU < 0.1,
        "earth must sit near 1 AU at epoch"
    );
}

#[test]
fn idle_sim_advances_planet_states_over_time() {
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: 86_400.0, // 1-day ticks for an ephemeris-only run
        load_default_bodies: true,
        apply_gravity: false,
        ..Default::default()
    });
    let cache = sim.world.resource::<sim_core::prelude::EphemerisCache>().clone();
    let earth_t0 = cache.get(naif::EARTH).unwrap();
    sim.tick_n(30); // 30 days
    let earth_t1 = sim.world.resource::<sim_core::prelude::EphemerisCache>().get(naif::EARTH).unwrap();
    let moved = (earth_t1.position - earth_t0.position).length();
    // Earth covers a chunk of an arc in 30 days — way more than 1e8 m.
    assert!(moved > 1.0e10, "earth should move > 10⁹ m over 30 days, got {}", moved);
}
