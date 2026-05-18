//! End-to-end test: under autopilot control, a brachistochrone-class drive
//! actually arrives at the mission target with near-zero relative velocity.
//!
//! No gravity gradients in this test (we use a no-default-bodies world and
//! force-disable gravity). The point is to validate the guidance loop, not
//! the orbital mechanics — those are covered elsewhere.

use glam::{DMat3, DQuat, DVec3};
use sim_core::ephemeris::{BodyParams, BodyState, EphemerisCache};
use sim_core::{
    Autopilot, AutopilotPhase, CommandedWrench, ExpanseSim, Mission, PropulsionDrive,
    PropulsionType, RigidBody, SimConfig, Spacecraft,
};

const TARGET_ID: i32 = 9_001;

fn install_synthetic_target(sim: &mut ExpanseSim, position: DVec3, velocity: DVec3) {
    let cache = sim.world.resource::<EphemerisCache>().clone();
    // Replace the default roster with just our synthetic body so the
    // autopilot only sees what the test asks for.
    cache.set_bodies(vec![]);
    cache.set_state(
        BodyParams {
            id: TARGET_ID,
            name: "Target",
            mu: 0.0,
            radius: 1.0e6,
            kepler: None,
        },
        BodyState { position, velocity },
    );
    sim.world.insert_resource(Mission {
        source_body: None,
        target_body: Some(TARGET_ID),
    });
}

#[test]
fn autopilot_arrives_at_static_target_with_low_relative_velocity() {
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: 0.5,
        load_default_bodies: true,
        apply_gravity: false,
        ..Default::default()
    });
    install_synthetic_target(
        &mut sim,
        DVec3::new(1.0e7, 0.0, 0.0),
        DVec3::ZERO,
    );

    // Spawn ship at origin, at rest, with a high-thrust drive.
    sim.world.spawn((
        Spacecraft { id: 1 },
        RigidBody {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            attitude: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
            mass: 1000.0,
            inertia: DMat3::IDENTITY * 1.0e3,
        },
        CommandedWrench::default(),
        PropulsionDrive {
            drive_type: PropulsionType::Brachistochrone,
            isp: 12_000.0,
            propellant_mass: 50_000.0,
            // Roughly 5g of acceleration headroom at 1000 kg.
            max_thrust: 1.0e6,
            ..Default::default()
        },
    ));

    // Engage autopilot at 1 g.
    {
        let mut ap = sim.world.resource_mut::<Autopilot>();
        ap.engaged = true;
        ap.accel_g = 1.0;
        ap.tolerance.range_m = 5.0e5; // 500 km
        ap.tolerance.rel_speed_m_s = 5.0; // 5 m/s
    }

    // The brachistochrone over 1e7 m at 1 g (~9.8 m/s²) is t = 2√(d/a) ≈
    // 2·√(1.02e6) ≈ 2020 s. Step long enough to comfortably cover both phases.
    let max_steps = 10_000;
    let mut arrived = false;
    for _ in 0..max_steps {
        sim.tick();
        let phase = sim.world.resource::<Autopilot>().phase;
        if matches!(phase, AutopilotPhase::Arrived) {
            arrived = true;
            break;
        }
    }
    assert!(arrived, "autopilot never reported arrival");

    let ap = sim.world.resource::<Autopilot>();
    assert!(
        ap.range_m < 5.0e5,
        "final range too large: {} m",
        ap.range_m
    );
    // Closing rate at arrival should be tiny — the brake phase nulls it out.
    assert!(
        ap.closing_m_s.abs() < 50.0,
        "rel speed too large at arrival: {} m/s",
        ap.closing_m_s
    );
}

#[test]
fn autopilot_catches_moving_target() {
    // Target drifting transverse to the boresight — the autopilot must lead
    // it (via the predict_intercept iteration) rather than aim where it is now.
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: 0.5,
        load_default_bodies: true,
        apply_gravity: false,
        ..Default::default()
    });
    install_synthetic_target(
        &mut sim,
        DVec3::new(1.0e7, 0.0, 0.0),
        DVec3::new(0.0, 100.0, 0.0), // 100 m/s transverse drift
    );
    // Keep the target moving each tick — refresh the cache state.
    let cache = sim.world.resource::<EphemerisCache>().clone();

    sim.world.spawn((
        Spacecraft { id: 1 },
        RigidBody {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            attitude: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
            mass: 1000.0,
            inertia: DMat3::IDENTITY * 1.0e3,
        },
        CommandedWrench::default(),
        PropulsionDrive {
            drive_type: PropulsionType::Brachistochrone,
            isp: 12_000.0,
            propellant_mass: 50_000.0,
            max_thrust: 1.0e6,
            ..Default::default()
        },
    ));
    {
        let mut ap = sim.world.resource_mut::<Autopilot>();
        ap.engaged = true;
        ap.accel_g = 1.0;
        ap.tolerance.range_m = 1.0e6; // 1000 km — soft arrival
        ap.tolerance.rel_speed_m_s = 50.0;
    }

    let mut arrived = false;
    for step in 0..10_000 {
        // Advance the synthetic target's position manually (cache refresh
        // sees its kepler=None and preserves last-written state, so we have
        // to integrate it ourselves).
        let t = (step as f64) * 0.5;
        cache.set_state(
            BodyParams {
                id: TARGET_ID,
                name: "Target",
                mu: 0.0,
                radius: 1.0e6,
                kepler: None,
            },
            BodyState {
                position: DVec3::new(1.0e7, 100.0 * t, 0.0),
                velocity: DVec3::new(0.0, 100.0, 0.0),
            },
        );
        sim.tick();
        if matches!(
            sim.world.resource::<Autopilot>().phase,
            AutopilotPhase::Arrived
        ) {
            arrived = true;
            break;
        }
    }
    assert!(arrived, "autopilot failed to catch moving target");
}

#[test]
fn autopilot_disengages_cleanly() {
    let mut sim = ExpanseSim::with_config(SimConfig {
        dt: 0.5,
        load_default_bodies: true,
        apply_gravity: false,
        ..Default::default()
    });
    install_synthetic_target(
        &mut sim,
        DVec3::new(1.0e7, 0.0, 0.0),
        DVec3::ZERO,
    );
    sim.world.spawn((
        Spacecraft { id: 1 },
        RigidBody {
            mass: 1000.0,
            inertia: DMat3::IDENTITY * 1.0e3,
            ..Default::default()
        },
        CommandedWrench::default(),
        PropulsionDrive {
            drive_type: PropulsionType::Brachistochrone,
            isp: 12_000.0,
            propellant_mass: 50_000.0,
            max_thrust: 1.0e6,
            ..Default::default()
        },
    ));
    sim.world.resource_mut::<Autopilot>().engaged = true;
    sim.world.resource_mut::<Autopilot>().accel_g = 1.0;
    // Take a few ticks of boost.
    for _ in 0..10 {
        sim.tick();
    }
    sim.world.resource_mut::<Autopilot>().engaged = false;
    // After a couple more ticks the autopilot must report idle.
    for _ in 0..3 {
        sim.tick();
    }
    let ap = sim.world.resource::<Autopilot>();
    assert_eq!(ap.phase, AutopilotPhase::Idle);
}
