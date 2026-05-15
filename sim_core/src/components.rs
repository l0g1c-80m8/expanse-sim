//! Core ECS components shared across the simulator.

use bevy_ecs::prelude::*;
use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

/// 13-DOF rigid-body state: position (3) + quaternion attitude (4) +
/// velocity (3) + angular velocity (3) = 13 state variables, plus mass and
/// an inertia tensor that may vary as propellant is consumed.
///
/// Double precision is mandatory: a single-precision float can only resolve
/// ~10 m at Saturn distances, which is unacceptable for an autonomy harness.
#[derive(Component, Debug, Clone, Serialize, Deserialize)]
pub struct RigidBody {
    /// Position in the inertial reference frame (meters, J2000-aligned).
    pub position: DVec3,
    /// Velocity vector (m/s, inertial).
    pub velocity: DVec3,
    /// Attitude quaternion: rotates body-frame vectors into the inertial frame.
    pub attitude: DQuat,
    /// Angular velocity (rad/s) expressed in the body frame.
    pub angular_velocity: DVec3,
    /// Total mass (kg). Updated by propulsion systems as propellant is expended.
    pub mass: f64,
    /// Inertia tensor (kg·m²) in the body frame. Scales with mass as propellant burns.
    pub inertia: DMat3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            attitude: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
            mass: 1.0,
            inertia: DMat3::IDENTITY,
        }
    }
}

impl RigidBody {
    /// Rotate a body-frame vector into the inertial frame using the current attitude.
    pub fn body_to_inertial(&self, v_body: DVec3) -> DVec3 {
        self.attitude * v_body
    }

    /// Rotate an inertial-frame vector into the body frame.
    pub fn inertial_to_body(&self, v_inertial: DVec3) -> DVec3 {
        self.attitude.inverse() * v_inertial
    }
}

/// Identifies which dynamical model governs a propulsion stack.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PropulsionType {
    /// Low-thrust / impulsive chemical or ion engines. Mass is roughly constant
    /// over a single tick; suitable for Hohmann-style mission profiles.
    Conventional,
    /// High-thrust, high-Isp continuous drive (Epstein-style). Mass changes
    /// significantly within a single tick, and the inertia tensor must be
    /// re-evaluated.
    Brachistochrone,
}

/// A propulsion drive attached to a rigid body. `thrust_command` is the
/// commanded thrust vector in the **body frame**; the propulsion system
/// rotates it through the rigid body's attitude before applying the force.
#[derive(Component, Debug, Clone, Serialize, Deserialize)]
pub struct PropulsionDrive {
    pub drive_type: PropulsionType,
    /// Commanded thrust vector in the body frame (Newtons).
    pub thrust_command: DVec3,
    /// Specific impulse (seconds).
    pub isp: f64,
    /// Remaining propellant mass (kg).
    pub propellant_mass: f64,
    /// Maximum thrust magnitude the drive can produce (Newtons).
    pub max_thrust: f64,
}

impl Default for PropulsionDrive {
    fn default() -> Self {
        Self {
            drive_type: PropulsionType::Conventional,
            thrust_command: DVec3::ZERO,
            isp: 300.0,
            propellant_mass: 0.0,
            max_thrust: 0.0,
        }
    }
}

impl PropulsionDrive {
    /// Returns the realisable thrust vector after clamping to `max_thrust`.
    pub fn realised_thrust(&self) -> DVec3 {
        let mag = self.thrust_command.length();
        if mag <= self.max_thrust || mag == 0.0 {
            self.thrust_command
        } else {
            self.thrust_command * (self.max_thrust / mag)
        }
    }
}

/// External wrench (force + torque) commanded by the autonomy stack each tick.
/// Both vectors live in the **inertial frame**. The dynamics system consumes
/// and clears this every tick so stale commands do not accumulate silently.
#[derive(Component, Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct CommandedWrench {
    pub force: DVec3,
    pub torque: DVec3,
}

/// Marks an entity as the active spacecraft tracked by the dashboard. The
/// server uses it to choose what to broadcast as primary telemetry.
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct Spacecraft {
    pub id: u32,
}

/// Optional thermodynamic model for an engine / hull pair.
#[cfg(feature = "thermodynamics")]
#[derive(Component, Debug, Clone, Serialize, Deserialize)]
pub struct Thermodynamics {
    pub temperature: f64,
    pub heat_capacity: f64,
    pub max_temperature: f64,
    pub radiative_area: f64,
    pub emissivity: f64,
}

#[cfg(feature = "thermodynamics")]
impl Default for Thermodynamics {
    fn default() -> Self {
        Self {
            temperature: 293.15,
            heat_capacity: 50_000.0,
            max_temperature: 1500.0,
            radiative_area: 20.0,
            emissivity: 0.85,
        }
    }
}
