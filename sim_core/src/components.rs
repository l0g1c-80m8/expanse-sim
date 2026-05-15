use bevy_ecs::prelude::*;
use glam::{DVec3, DQuat, DMat3};

/// The primary rigid body state of any simulated entity.
/// Uses double precision (f64) for interplanetary distances.
#[derive(Component, Debug, Clone)]
pub struct RigidBody {
    /// Position in the reference frame (meters)
    pub position: DVec3,
    /// Velocity vector (meters / second)
    pub velocity: DVec3,
    /// Attitude quaternion representing rotation from the reference frame
    pub attitude: DQuat,
    /// Angular rates (radians / second)
    pub angular_velocity: DVec3,
    /// Total mass of the object (kg)
    pub mass: f64,
    /// Inertia tensor (3x3 matrix)
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

#[derive(Debug, Clone, PartialEq)]
pub enum PropulsionType {
    Conventional,
    Brachistochrone,
}

#[derive(Component, Debug, Clone)]
pub struct PropulsionDrive {
    pub drive_type: PropulsionType,
    /// Commanded thrust vector in body frame (Newtons)
    pub thrust: DVec3, 
    /// Specific Impulse (seconds)
    pub isp: f64,
    /// Remaining propellant mass (kg)
    pub propellant_mass: f64,
    /// Maximum thrust capability (Newtons)
    pub max_thrust: f64,
}

#[cfg(feature = "thermodynamics")]
#[derive(Component, Debug, Clone)]
pub struct Thermodynamics {
    /// Core temperature (Kelvin)
    pub temperature: f64,
    /// Heat capacity (J/K)
    pub heat_capacity: f64,
    /// Maximum safe temperature (Kelvin)
    pub max_temperature: f64,
}
