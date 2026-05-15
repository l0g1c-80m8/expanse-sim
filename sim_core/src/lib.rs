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

pub mod clock;
pub mod components;
pub mod dynamics;
pub mod ephemeris;
pub mod ipc;
pub mod propulsion;
#[cfg(feature = "thermodynamics")]
pub mod thermal;

pub use clock::{SimClock, SimTime};
pub use components::{
    CommandedWrench, PropulsionDrive, PropulsionType, RigidBody, Spacecraft,
};
pub use dynamics::{dynamics_system, DynamicsConfig};
pub use ephemeris::{ephemeris_refresh_system, BodyParams, BodyState, EphemerisCache};
pub use ipc::{AutonomyBridge, lockstep_sync_system, CommandPacket, TickTelemetry};
pub use propulsion::propulsion_system;

#[cfg(feature = "thermodynamics")]
pub use components::Thermodynamics;
#[cfg(feature = "thermodynamics")]
pub use thermal::thermodynamics_system;

use bevy_ecs::prelude::*;

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

        let mut schedule = Schedule::default();
        // Order matters: refresh planet states first, then update propellant
        // (which may rescale inertia), then integrate, then optional thermal,
        // then the lockstep exchange so autonomy sees post-tick state.
        if cfg.load_default_bodies {
            schedule.add_systems(ephemeris_refresh_system);
        }
        schedule.add_systems(
            (propulsion_system, dynamics_system)
                .chain()
                .after(ephemeris_refresh_system),
        );
        #[cfg(feature = "thermodynamics")]
        schedule.add_systems(thermodynamics_system.after(propulsion_system));
        schedule.add_systems(lockstep_sync_system.after(dynamics_system));

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
