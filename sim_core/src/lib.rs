//! Expanse-Sim core: a deterministic, data-driven, 6-DOF spacecraft simulator
//! intended as a backend for ROS 2 autonomy stacks.
//!
//! Architecture, top-down:
//!
//! ```text
//! ┌──────────────── ExpanseSim ────────────────┐
//! │  bevy_ecs World + fixed-step Schedule       │
//! │                                             │
//! │  ephemeris_refresh ─► propulsion ─► dynamics│
//! │                                  │          │
//! │                                  └► thermal │
//! │                                  └► lockstep│
//! └─────────────────────────────────────────────┘
//!         │                       ▲
//!  ZeroMQ │  (telemetry)          │ wrench
//!         ▼                       │
//!     Autonomy stack (ROS 2)  ────┘
//! ```
//!
//! The simulator's tick is independent of wall-clock time: callers (the
//! server binary, an embed user, or a test) decide how many ticks to retire
//! per wall-second. `SimClock` provides the wall→sim conversion when needed.

// ── module roster ─────────────────────────────────────────────────────────
// Modules are grouped roughly by concern:
//   environment   ephemeris, mission, mode
//   physics       components, dynamics, propulsion, clock, sensors
//   guidance      autopilot, nav, thrust_controller
//   integration   ipc (autonomy bridges), protocol (wire format)
//   options       thermal (feature-gated)

pub mod autopilot;
pub mod clock;
pub mod components;
pub mod dynamics;
pub mod ephemeris;
pub mod ipc;
pub mod mission;
pub mod mode;
pub mod nav;
pub mod propulsion;
pub mod protocol;
pub mod sensors;
#[cfg(feature = "thermodynamics")]
pub mod thermal;
pub mod thrust_controller;

pub mod prelude;

// ── core types kept at crate root for ergonomics ──────────────────────────
//
// Everything else lives under its module (e.g. `sim_core::autopilot::Autopilot`).
// Use `sim_core::prelude::*` if you want the kitchen-sink import.

pub use clock::{SimClock, SimTime};
pub use components::RigidBody;

use autopilot::{autopilot_system, Autopilot, AutopilotCommand};
use bevy_ecs::prelude::*;
use dynamics::{dynamics_system, DynamicsConfig};
use ephemeris::{ephemeris_refresh_system, EphemerisCache};
use ipc::{lockstep_sync_system, AutonomyBridge};
use mission::Mission;
use mode::SimModeState;
use nav::{nav_filter_system, NavEstimate, NavFilter};
use propulsion::propulsion_system;
use sensors::{sensor_system, LatestSensorPack, SensorConfig};
#[cfg(feature = "thermodynamics")]
use thermal::thermodynamics_system;

/// The central physics plant. Holds the ECS world and the deterministic
/// schedule of systems. One tick advances `SimTime.time` by exactly
/// `SimTime.dt`, regardless of wall-clock pacing.
pub struct ExpanseSim {
    pub world: World,
    pub schedule: Schedule,
}

impl ExpanseSim {
    /// Build a simulator with default resources (ephemeris cache pre-loaded
    /// with the 9-body solar system, autonomy bridge disabled).
    pub fn new() -> Self {
        Self::with_config(SimConfig::default())
    }

    pub fn with_config(cfg: SimConfig) -> Self {
        let mut world = World::new();
        world.insert_resource(SimTime::new(cfg.dt));
        world.insert_resource(SimClock {
            warp: cfg.warp,
            paused: false,
            epoch_j2000: cfg.epoch_j2000,
        });
        world.insert_resource(DynamicsConfig {
            apply_gravity: cfg.apply_gravity,
            ..Default::default()
        });
        if cfg.load_default_bodies {
            world.insert_resource(EphemerisCache::with_default_bodies());
        }
        world.insert_resource(AutonomyBridge::default());
        world.insert_resource(Mission::default());
        world.insert_resource(SimModeState::default());
        world.insert_resource(Autopilot::default());
        world.insert_resource(AutopilotCommand::default());
        world.insert_resource(SensorConfig::default());
        world.insert_resource(LatestSensorPack::default());
        world.insert_resource(NavFilter::default());
        world.insert_resource(NavEstimate::default());

        let mut schedule = Schedule::default();
        // Order: ephemeris refresh → autopilot (reads post-refresh body
        // states) → propulsion → dynamics → sensors (post-step) → thermal
        // → lockstep.
        if cfg.load_default_bodies {
            schedule.add_systems(ephemeris_refresh_system);
        }
        schedule.add_systems(
            autopilot_system
                .after(ephemeris_refresh_system)
                .before(propulsion_system),
        );
        schedule.add_systems(
            (propulsion_system, dynamics_system)
                .chain()
                .after(autopilot_system),
        );
        schedule.add_systems(sensor_system.after(dynamics_system));
        schedule.add_systems(nav_filter_system.after(sensor_system));
        #[cfg(feature = "thermodynamics")]
        schedule.add_systems(thermodynamics_system.after(propulsion_system));
        schedule.add_systems(lockstep_sync_system.after(nav_filter_system));

        Self { world, schedule }
    }

    /// Advance the simulation by one fixed `dt` tick.
    pub fn tick(&mut self) {
        self.schedule.run(&mut self.world);
        self.world.resource_mut::<SimTime>().advance();
    }

    /// Advance the simulation by `n` ticks. Useful for tests and warp.
    pub fn tick_n(&mut self, n: u64) {
        for _ in 0..n {
            self.tick();
        }
    }

    pub fn sim_time(&self) -> f64 {
        self.world.resource::<SimTime>().time
    }

    pub fn set_warp(&mut self, warp: f64) {
        self.world.resource_mut::<SimClock>().warp = warp.max(0.0);
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.world.resource_mut::<SimClock>().paused = paused;
    }
}

impl Default for ExpanseSim {
    fn default() -> Self {
        Self::new()
    }
}

/// Construction parameters for `ExpanseSim`.
#[derive(Debug, Clone, Copy)]
pub struct SimConfig {
    pub dt: f64,
    pub warp: f64,
    pub epoch_j2000: f64,
    pub apply_gravity: bool,
    pub load_default_bodies: bool,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            dt: 0.01,
            warp: 1.0,
            epoch_j2000: 0.0,
            apply_gravity: true,
            load_default_bodies: true,
        }
    }
}
