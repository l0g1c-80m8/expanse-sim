use bevy_ecs::prelude::*;
use glam::DVec3;
use crate::components::{RigidBody, PropulsionDrive, PropulsionType};
#[cfg(feature = "thermodynamics")]
use crate::components::Thermodynamics;

/// Resource tracking the simulation time and step
#[derive(Resource, Debug)]
pub struct SimTime {
    pub time: f64,
    pub dt: f64,
}

impl Default for SimTime {
    fn default() -> Self {
        Self { time: 0.0, dt: 0.01 }
    }
}

/// Computes propulsion dynamics and updates rigid body states
pub fn propulsion_system(
    sim_time: Res<SimTime>,
    mut query: Query<(&mut RigidBody, &mut PropulsionDrive)>
) {
    let dt = sim_time.dt;
    // Standard gravity acceleration constant
    let g0 = 9.80665; 

    for (mut rb, mut drive) in query.iter_mut() {
        if drive.thrust.length_squared() > 0.0 && drive.propellant_mass > 0.0 {
            // Calculate magnitude of thrust applied
            let thrust_mag = drive.thrust.length().min(drive.max_thrust);
            let thrust_dir = drive.thrust.normalize();

            // Calculate mass flow rate: m_dot = T / (Isp * g0)
            let m_dot = thrust_mag / (drive.isp * g0);
            let mass_expended = m_dot * dt;
            
            // Ensure we don't burn more fuel than we have
            let actual_mass_expended = mass_expended.min(drive.propellant_mass);
            let actual_thrust = (actual_mass_expended / dt) * (drive.isp * g0);
            
            drive.propellant_mass -= actual_mass_expended;
            
            match drive.drive_type {
                PropulsionType::Conventional => {
                    // Constant mass during tick for simplicity, instantaneous delta V applied as force
                    let accel = (thrust_dir * actual_thrust) / rb.mass;
                    rb.velocity += accel * dt;
                },
                PropulsionType::Brachistochrone => {
                    // Huge mass reduction, time-variant mass integration
                    rb.mass -= actual_mass_expended;
                    
                    // Simple Tsiolkovsky integration over dt
                    // DeltaV = Isp * g0 * ln(m0 / m1)
                    if rb.mass > 0.0 {
                        let m0 = rb.mass + actual_mass_expended;
                        let m1 = rb.mass;
                        let delta_v = drive.isp * g0 * (m0 / m1).ln();
                        rb.velocity += thrust_dir * delta_v;
                        
                        // Recompute inertia tensor I(t) based on mass distribution
                        // For now, scale inertia linearly with mass
                        let mass_ratio = m1 / m0;
                        rb.inertia *= mass_ratio;
                    }
                }
            }
        }
    }
}

#[cfg(feature = "thermodynamics")]
pub fn thermodynamics_system(
    sim_time: Res<SimTime>,
    mut query: Query<(&mut Thermodynamics, &PropulsionDrive)>
) {
    let dt = sim_time.dt;
    for (mut thermo, drive) in query.iter_mut() {
        if drive.thrust.length_squared() > 0.0 {
            // Simple heat generation model proportional to thrust
            let heat_generated = drive.thrust.length() * 0.1 * dt; 
            let temp_increase = heat_generated / thermo.heat_capacity;
            thermo.temperature += temp_increase;
        } else {
            // Radiative cooling
            let cooling_rate = 0.01 * dt; // Arbitrary cooling factor
            thermo.temperature = (thermo.temperature - cooling_rate).max(2.7); // CMB temp
        }
    }
}

/// RK4 integrator for rigid body kinematics
pub fn rigid_body_kinematics_system(
    sim_time: Res<SimTime>,
    mut query: Query<&mut RigidBody>
) {
    let dt = sim_time.dt;
    for mut rb in query.iter_mut() {
        // Semi-implicit Euler integration for position and attitude
        rb.position += rb.velocity * dt;
        
        let omega = rb.angular_velocity;
        if omega.length_squared() > 1e-8 {
            let angle = omega.length() * dt;
            let axis = omega.normalize();
            let dq = glam::DQuat::from_axis_angle(axis, angle);
            rb.attitude = (rb.attitude * dq).normalize();
        }
    }
}
