//! 6-DOF rigid-body integration.
//!
//! Implements a classical fourth-order Runge-Kutta integrator for the
//! translational state and a quaternion-aware integrator for the rotational
//! state. Forces and torques arrive each tick via `CommandedWrench` (set by
//! the autonomy bridge) and `PropulsionDrive` (set by the onboard controller).

use bevy_ecs::prelude::*;
use glam::{DMat3, DQuat, DVec3};

use crate::clock::SimTime;
use crate::components::{CommandedWrench, PropulsionDrive, RigidBody};
use crate::ephemeris::EphemerisCache;

/// Configuration controlling which forces are summed into the equations of
/// motion. Anything not listed here lives in `CommandedWrench`.
#[derive(Resource, Debug, Clone, Copy)]
pub struct DynamicsConfig {
    /// If true, the dynamics system applies the gravity gradient from the
    /// ephemeris cache to each rigid body. Disable for unit tests that check
    /// pure inertial motion.
    pub apply_gravity: bool,
    /// Standard gravitational parameter for the central body (Sun, m³/s²) used
    /// when no ephemeris cache is registered. Defaults to GM_sun.
    pub fallback_mu: f64,
}

impl Default for DynamicsConfig {
    fn default() -> Self {
        Self {
            apply_gravity: true,
            fallback_mu: 1.327_124_400_18e20, // GM_sun, m³/s²
        }
    }
}

/// Net gravitational acceleration (m/s²) on a probe at `position` (meters,
/// inertial) at sim-time `t`. Uses the ephemeris cache if present, otherwise
/// falls back to a point-mass Sun at the origin.
fn gravity_at(
    position: DVec3,
    t: f64,
    cache: Option<&EphemerisCache>,
    fallback_mu: f64,
) -> DVec3 {
    if let Some(cache) = cache {
        return cache.gravity_at(position, t);
    }
    let r = position;
    let r2 = r.length_squared();
    if r2 < 1.0 {
        return DVec3::ZERO;
    }
    let r_mag = r2.sqrt();
    -r * (fallback_mu / (r2 * r_mag))
}

/// Net acceleration applied to a rigid body for a given snapshot of its state.
fn acceleration(
    position: DVec3,
    rb: &RigidBody,
    cmd: CommandedWrench,
    prop_force_inertial: DVec3,
    t: f64,
    cache: Option<&EphemerisCache>,
    cfg: &DynamicsConfig,
) -> DVec3 {
    let mut a = DVec3::ZERO;
    if cfg.apply_gravity {
        a += gravity_at(position, t, cache, cfg.fallback_mu);
    }
    a += (cmd.force + prop_force_inertial) / rb.mass.max(1e-12);
    a
}

/// Classical RK4 step for the translational state of a single rigid body.
///
/// The propulsion force is treated as constant across the substeps — at
/// sub-millisecond tick rates this is a tight approximation, and it keeps
/// the inner loop side-effect free.
fn rk4_translational(
    rb: &mut RigidBody,
    cmd: CommandedWrench,
    prop_force_inertial: DVec3,
    t: f64,
    dt: f64,
    cache: Option<&EphemerisCache>,
    cfg: &DynamicsConfig,
) {
    let p0 = rb.position;
    let v0 = rb.velocity;

    let a1 = acceleration(p0, rb, cmd, prop_force_inertial, t, cache, cfg);
    let k1_p = v0;
    let k1_v = a1;

    let p2 = p0 + k1_p * (dt * 0.5);
    let v2 = v0 + k1_v * (dt * 0.5);
    let a2 = acceleration(p2, rb, cmd, prop_force_inertial, t + dt * 0.5, cache, cfg);
    let k2_p = v2;
    let k2_v = a2;

    let p3 = p0 + k2_p * (dt * 0.5);
    let v3 = v0 + k2_v * (dt * 0.5);
    let a3 = acceleration(p3, rb, cmd, prop_force_inertial, t + dt * 0.5, cache, cfg);
    let k3_p = v3;
    let k3_v = a3;

    let p4 = p0 + k3_p * dt;
    let v4 = v0 + k3_v * dt;
    let a4 = acceleration(p4, rb, cmd, prop_force_inertial, t + dt, cache, cfg);
    let k4_p = v4;
    let k4_v = a4;

    rb.position = p0 + (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p) * (dt / 6.0);
    rb.velocity = v0 + (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) * (dt / 6.0);
}

/// Integrate the rotational state using Euler's rigid-body equation
/// `I ω̇ = τ − ω × (I ω)` with a midpoint step on ω and a quaternion
/// exponential map on the attitude.
fn integrate_rotational(rb: &mut RigidBody, torque_body: DVec3, dt: f64) {
    // Convert inertial torque to body frame for the inertia tensor maths.
    let inv_i = rb.inertia.inverse();
    let omega = rb.angular_velocity;
    let omega_dot = inv_i * (torque_body - omega.cross(rb.inertia * omega));
    rb.angular_velocity = omega + omega_dot * dt;

    // Quaternion exponential map: q ← q ⊗ exp(½ ω dt)
    let half_dt = 0.5 * dt;
    let phi = rb.angular_velocity * half_dt;
    let angle = phi.length();
    let dq = if angle < 1e-12 {
        DQuat::from_xyzw(phi.x, phi.y, phi.z, 1.0).normalize()
    } else {
        let s = angle.sin() / angle;
        DQuat::from_xyzw(phi.x * s, phi.y * s, phi.z * s, angle.cos())
    };
    rb.attitude = (rb.attitude * dq).normalize();
}

/// The 6-DOF integration system.
///
/// Reads commanded wrenches (which include propulsion forces written by
/// `propulsion_system` earlier in the schedule) and steps the rigid-body
/// state forward by `SimTime.dt` using RK4 for translation and an
/// Euler-equation step for rotation.
pub fn dynamics_system(
    sim_time: Res<SimTime>,
    cfg: Res<DynamicsConfig>,
    cache: Option<Res<EphemerisCache>>,
    mut query: Query<(&mut RigidBody, &mut CommandedWrench, Option<&PropulsionDrive>)>,
) {
    let t = sim_time.time;
    let dt = sim_time.dt;
    let cache_ref = cache.as_deref();

    for (mut rb, mut wrench, drive) in query.iter_mut() {
        // The propulsion system has already pushed its inertial force into
        // CommandedWrench, but Brachistochrone needs no separate channel — it
        // burns through the same path. We split it out here only to allow the
        // RK4 substeps to use a steady force.
        let prop_force_inertial = drive
            .map(|d| {
                // For both conventional and brachistochrone, the realised body-frame
                // thrust is rotated through current attitude. The propulsion system
                // updates propellant_mass / mass, so this is just the force half.
                rb.body_to_inertial(d.realised_thrust())
            })
            .unwrap_or(DVec3::ZERO);

        // The CommandedWrench.force from autonomy is layered on top.
        let cmd_only = CommandedWrench {
            force: wrench.force,
            torque: wrench.torque,
        };

        rk4_translational(&mut rb, cmd_only, prop_force_inertial, t, dt, cache_ref, &cfg);

        // Rotational dynamics: external torque (cmd.torque) lives in the inertial
        // frame; rotate into the body frame for Euler's equation.
        let torque_body = rb.inertial_to_body(wrench.torque);
        integrate_rotational(&mut rb, torque_body, dt);

        // Consume the wrench so stale commands don't accumulate if the
        // autonomy client misses a tick. The lockstep bridge guarantees a
        // fresh write whenever it's enabled.
        *wrench = CommandedWrench::default();
    }
}

/// Helper used by tests and by the lockstep bridge to peek at inertia/mass
/// invariants after a tick.
pub fn kinetic_energy(rb: &RigidBody) -> f64 {
    let v2 = rb.velocity.length_squared();
    let omega = rb.angular_velocity;
    let rot = omega.dot(rb.inertia * omega);
    0.5 * rb.mass * v2 + 0.5 * rot
}

/// Specific orbital energy ε = v²/2 − μ/r — invariant for a Keplerian orbit.
pub fn specific_orbital_energy(rb: &RigidBody, mu: f64) -> f64 {
    let r = rb.position.length();
    if r < 1e-12 {
        return f64::INFINITY;
    }
    rb.velocity.length_squared() * 0.5 - mu / r
}

/// Apply the inertia-tensor scaling for a mass change. Mass-distribution-free
/// scaling: I' = I · (m'/m). Adequate for early prototyping; replace with a
/// real geometry-based tensor once the spacecraft loadout is parameterised.
pub fn scale_inertia_for_mass(inertia: DMat3, old_mass: f64, new_mass: f64) -> DMat3 {
    if old_mass <= 0.0 {
        return inertia;
    }
    inertia * (new_mass / old_mass)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::CommandedWrench;
    use approx::assert_relative_eq;

    fn unit_body() -> RigidBody {
        RigidBody {
            position: DVec3::new(1.0e8, 0.0, 0.0),
            velocity: DVec3::new(0.0, 1.0e3, 0.0),
            attitude: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
            mass: 1000.0,
            inertia: DMat3::IDENTITY,
        }
    }

    #[test]
    fn rk4_with_no_force_is_uniform_motion() {
        let mut rb = unit_body();
        let cfg = DynamicsConfig {
            apply_gravity: false,
            ..Default::default()
        };
        let dt = 0.1;
        let initial_pos = rb.position;
        let initial_vel = rb.velocity;
        for _ in 0..100 {
            rk4_translational(
                &mut rb,
                CommandedWrench::default(),
                DVec3::ZERO,
                0.0,
                dt,
                None,
                &cfg,
            );
        }
        assert_relative_eq!(rb.velocity.x, initial_vel.x, epsilon = 1e-9);
        assert_relative_eq!(rb.velocity.y, initial_vel.y, epsilon = 1e-9);
        assert_relative_eq!(rb.position.x, initial_pos.x + initial_vel.x * 10.0, epsilon = 1e-6);
        assert_relative_eq!(rb.position.y, initial_pos.y + initial_vel.y * 10.0, epsilon = 1e-6);
    }

    #[test]
    fn rk4_constant_force_matches_analytic_kinematics() {
        // a = F/m, p(t) = p0 + v0 t + ½ a t²
        let mut rb = unit_body();
        rb.velocity = DVec3::ZERO;
        rb.position = DVec3::ZERO;
        let cfg = DynamicsConfig {
            apply_gravity: false,
            ..Default::default()
        };
        let dt = 0.01;
        let force = DVec3::new(10.0, 0.0, 0.0);
        let total_steps = 1000; // 10 s
        for _ in 0..total_steps {
            rk4_translational(
                &mut rb,
                CommandedWrench { force, torque: DVec3::ZERO },
                DVec3::ZERO,
                0.0,
                dt,
                None,
                &cfg,
            );
        }
        let t_total = dt * total_steps as f64;
        let a = force.x / rb.mass;
        let expected_x = 0.5 * a * t_total * t_total;
        let expected_vx = a * t_total;
        assert_relative_eq!(rb.position.x, expected_x, epsilon = 1e-6);
        assert_relative_eq!(rb.velocity.x, expected_vx, epsilon = 1e-9);
    }

    #[test]
    fn keplerian_circular_orbit_conserves_energy() {
        // Circular orbit at 1 AU around the Sun with the gravitational
        // potential the dynamics system uses by default.
        let mu: f64 = 1.327_124_400_18e20;
        let r: f64 = 1.495_978_707e11;
        let v_circ = (mu / r).sqrt();
        let mut rb = RigidBody {
            position: DVec3::new(r, 0.0, 0.0),
            velocity: DVec3::new(0.0, v_circ, 0.0),
            mass: 1000.0,
            ..Default::default()
        };
        let cfg = DynamicsConfig::default();
        let dt = 60.0; // 1-minute steps
        let mut t = 0.0;
        let e0 = specific_orbital_energy(&rb, mu);
        // Quarter orbit; period ≈ 1 year, so ~91 days = 7_862_400 s
        let n_steps = 131_040; // ~91 days
        for _ in 0..n_steps {
            rk4_translational(
                &mut rb,
                CommandedWrench::default(),
                DVec3::ZERO,
                t,
                dt,
                None,
                &cfg,
            );
            t += dt;
        }
        let e1 = specific_orbital_energy(&rb, mu);
        // Energy drift over a quarter orbit should be well under 0.1 %.
        let drift = ((e1 - e0) / e0).abs();
        assert!(drift < 1e-3, "energy drift too large: {}", drift);
    }

    #[test]
    fn free_rotation_preserves_quaternion_norm() {
        let mut rb = RigidBody {
            angular_velocity: DVec3::new(0.5, 0.1, -0.2),
            ..Default::default()
        };
        for _ in 0..10_000 {
            integrate_rotational(&mut rb, DVec3::ZERO, 0.01);
        }
        let norm = rb.attitude.length();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn torque_changes_angular_velocity() {
        let mut rb = RigidBody::default();
        let omega0 = rb.angular_velocity;
        for _ in 0..1000 {
            integrate_rotational(&mut rb, DVec3::new(1.0, 0.0, 0.0), 0.01);
        }
        // I = identity, so ω ≈ τ·t = 10 rad/s (small numerical drift from coupling terms)
        assert!(rb.angular_velocity.x > 9.5 && rb.angular_velocity.x < 10.5);
        assert!(rb.angular_velocity.y.abs() < 0.5);
        assert_ne!(rb.angular_velocity, omega0);
    }

    #[test]
    fn scale_inertia_is_linear_in_mass() {
        let i0 = DMat3::IDENTITY * 100.0;
        let i1 = scale_inertia_for_mass(i0, 1000.0, 500.0);
        assert_relative_eq!(i1.x_axis.x, 50.0, epsilon = 1e-12);
        assert_relative_eq!(i1.y_axis.y, 50.0, epsilon = 1e-12);
        assert_relative_eq!(i1.z_axis.z, 50.0, epsilon = 1e-12);
    }
}
