//! Convenience re-exports for `use sim_core::prelude::*;`.
//!
//! Intended for tests, demos, and external crates that don't want to
//! remember each module path. The crate-root only re-exports a tiny
//! ergonomic surface (`SimTime`, `SimClock`, `RigidBody`); everything
//! else is reached through the prelude or its own module.

// ECS handle (host code that holds entity IDs needs this).
pub use bevy_ecs::prelude::Entity;

// Core driver
pub use crate::{ExpanseSim, SimConfig, SimTime, SimClock};

// Components
pub use crate::components::{
    CommandedWrench, PropulsionDrive, PropulsionType, RadiationModel, RigidBody, Spacecraft,
};
#[cfg(feature = "thermodynamics")]
pub use crate::components::Thermodynamics;

// Environment
pub use crate::ephemeris::{
    park_orbit_state, BodyParams, BodyState, EphemerisCache,
};
pub use crate::mission::Mission;
pub use crate::mode::{SimMode, SimModeState};

// Physics
pub use crate::dynamics::{
    dynamics_system, srp_force, DynamicsConfig, SRP_AT_1AU_N_M2,
};
pub use crate::propulsion::propulsion_system;
pub use crate::sensors::{
    sensor_system, LatestSensorPack, SensorConfig, SensorPack,
};
#[cfg(feature = "thermodynamics")]
pub use crate::thermal::thermodynamics_system;

// Guidance
pub use crate::autopilot::{
    autopilot_system, ArrivalTolerance, Autopilot, AutopilotCommand, AutopilotPhase,
};
pub use crate::nav::{nav_filter_system, NavEstimate, NavFilter};
pub use crate::thrust_controller::{
    thrust_controller_system, ThrustController, ThrustMode,
};

// IPC
pub use crate::ipc::{
    lockstep_sync_system, AutonomyBridge, CommandPacket, TickTelemetry,
};

// Wire protocol
pub use crate::protocol::{
    apply_command, build_default_sim, snapshot, spawn_default_spacecraft, stage_at_source,
    AutopilotSnapshot, BodySnapshot, ControlCommand, MissionSnapshot, SpacecraftSnapshot,
    TelemetryFrame, ThrustControllerSnapshot,
};
