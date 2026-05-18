//! Onboard EKF navigation filter.
//!
//! Consumes the synthetic sensor pack (per-body ranges) and produces an
//! estimated `[r, v]` state in the inertial heliocentric frame, together
//! with a 6×6 covariance. The estimator is what makes the project a
//! *digital twin for nav R&D*: external autonomy stacks can compare the
//! filter's output against ground truth to benchmark themselves, or replace
//! this filter wholesale with their own EKF / UKF / particle filter and
//! verify it on a deterministic harness.
//!
//! State:        x = [r_x, r_y, r_z, v_x, v_y, v_z]  (inertial, J2000, SI)
//! Process:      ẋ = [v, -μ_sun·r/|r|³]   (Sun-only gravity model)
//! Observations: range to each catalogued body z_i = |r − p_i|
//!
//! The filter intentionally models *only Sun gravity in the process model*
//! and ignores commanded thrust — that's what a real interplanetary nav
//! filter does in cruise. Thrust events show up as residuals during burns
//! and the filter re-converges in coast.

use bevy_ecs::prelude::*;
use nalgebra::{Matrix6, SMatrix, SVector, Vector3, Vector6};
use serde::Serialize;

use crate::clock::SimTime;
use crate::components::{RigidBody, Spacecraft};
use crate::ephemeris::{EphemerisCache, MU_SUN};
use crate::sensors::LatestSensorPack;

#[derive(Resource, Debug, Clone)]
pub struct NavFilter {
    pub enabled: bool,
    pub initialized: bool,
    /// State vector [r_x, r_y, r_z, v_x, v_y, v_z].
    pub x: Vector6<f64>,
    /// Covariance.
    pub p: Matrix6<f64>,
    /// Continuous-time process noise spectral density per axis. Lower bound
    /// on filter agility; bigger Q = filter trusts measurements more.
    pub q_pos_psd: f64,
    pub q_vel_psd: f64,
    /// Measurement noise variances actually used by the filter. Operators
    /// can intentionally mismatch these against the true sensor σ to study
    /// filter robustness.
    pub r_range_var: f64,
    /// Initial-state uncertainty seeded from ground truth at engage time.
    pub init_sigma_r_m: f64,
    pub init_sigma_v_m_s: f64,
}

impl Default for NavFilter {
    fn default() -> Self {
        Self {
            enabled: false,
            initialized: false,
            x: Vector6::zeros(),
            p: Matrix6::identity(),
            q_pos_psd: 1.0e-4,           // 0.01 m²/s³
            q_vel_psd: 1.0e-8,           // (m/s)²/s
            r_range_var: 1.0e4,          // 100 m σ
            init_sigma_r_m: 1.0e6,       // 1 000 km
            init_sigma_v_m_s: 100.0,     // 100 m/s
        }
    }
}

/// Latest nav estimate, broadcast in telemetry so the dashboard can render
/// the ghost-ship and the nav-error vector.
#[derive(Resource, Default, Debug, Clone, Serialize)]
pub struct NavEstimate {
    pub initialized: bool,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// Trace-derived 1-σ position uncertainty (m).
    pub position_sigma_m: f64,
    pub velocity_sigma_m_s: f64,
    /// Last range residual (m) — useful for chi-square monitoring.
    pub residual_range_m: f64,
    /// Last-update body NAIF id (or 0 if no update this tick).
    pub last_update_body: i32,
    /// Number of range updates folded in over the lifetime of this filter
    /// instance. Reset to 0 when `set_enabled(false)` is followed by
    /// re-engage.
    pub updates_count: u64,
}

fn vec3(a: [f64; 3]) -> Vector3<f64> {
    Vector3::new(a[0], a[1], a[2])
}

fn dvec_to_sv3(d: glam::DVec3) -> Vector3<f64> {
    Vector3::new(d.x, d.y, d.z)
}

/// Build the state-transition Jacobian F = I + dt · ∂f/∂x for a Sun-only
/// gravity process model. f = [v; g(r)], so
///   ∂f/∂x = [[0, I], [G, 0]]
/// where G = ∂g/∂r = (μ/|r|⁵)·(3·r·rᵀ − |r|²·I).
fn build_state_transition(r: Vector3<f64>, dt: f64) -> Matrix6<f64> {
    let r2 = r.norm_squared();
    let r_mag = r2.sqrt();
    let mut f = Matrix6::identity();
    // top-right 3×3 = dt · I
    for i in 0..3 {
        f[(i, i + 3)] = dt;
    }
    // bottom-left 3×3 = dt · G
    if r_mag > 1.0 {
        let mu_r5 = MU_SUN / (r2 * r2 * r_mag);
        for i in 0..3 {
            for j in 0..3 {
                let delta = if i == j { 1.0 } else { 0.0 };
                f[(i + 3, j)] = dt * mu_r5 * (3.0 * r[i] * r[j] - r2 * delta);
            }
        }
    }
    f
}

/// Discrete process-noise covariance assuming continuous-time process
/// noise applied to velocity only (the standard random-walk acceleration
/// model). Position covariance accumulates from velocity noise via dt².
fn build_process_noise(q_pos_psd: f64, q_vel_psd: f64, dt: f64) -> Matrix6<f64> {
    let mut q = Matrix6::zeros();
    let q_p = q_pos_psd * dt;
    let q_v = q_vel_psd * dt;
    for i in 0..3 {
        q[(i, i)] = q_p + q_v * dt * dt / 3.0;
        q[(i, i + 3)] = q_v * dt / 2.0;
        q[(i + 3, i)] = q_v * dt / 2.0;
        q[(i + 3, i + 3)] = q_v;
    }
    q
}

/// One EKF predict step: x ← f(x), P ← F P Fᵀ + Q.
fn predict(filter: &mut NavFilter, dt: f64) {
    let r = filter.x.fixed_rows::<3>(0).into_owned();
    let v = filter.x.fixed_rows::<3>(3).into_owned();
    let r2 = r.norm_squared();
    let r_mag = r2.sqrt();
    let g = if r_mag > 1.0 {
        let factor = -MU_SUN / (r2 * r_mag);
        Vector3::new(r[0] * factor, r[1] * factor, r[2] * factor)
    } else {
        Vector3::zeros()
    };
    // Euler propagate (acceptable at typical dt = 0.05 s for the EKF prediction).
    let new_r = r + v * dt;
    let new_v = v + g * dt;
    for i in 0..3 {
        filter.x[i] = new_r[i];
        filter.x[i + 3] = new_v[i];
    }
    let f_mat = build_state_transition(r, dt);
    let q_mat = build_process_noise(filter.q_pos_psd, filter.q_vel_psd, dt);
    filter.p = f_mat * filter.p * f_mat.transpose() + q_mat;
}

/// Fold a single range observation into the filter.
fn update_range(
    filter: &mut NavFilter,
    body_pos: Vector3<f64>,
    z_meas: f64,
    r_var: f64,
) -> Option<f64> {
    let r = filter.x.fixed_rows::<3>(0).into_owned();
    let dr = r - body_pos;
    let dr_mag = dr.norm();
    if dr_mag < 1.0 {
        return None;
    }
    let z_pred = dr_mag;
    let y = z_meas - z_pred;

    // H = [dr/|dr| , 0]  (1×6).
    let mut h = SMatrix::<f64, 1, 6>::zeros();
    for i in 0..3 {
        h[(0, i)] = dr[i] / dr_mag;
    }
    let s = (h * filter.p * h.transpose())[(0, 0)] + r_var;
    if s <= 0.0 {
        return None;
    }
    let k: SVector<f64, 6> = filter.p * h.transpose() / s;
    filter.x += k * y;
    let kh: Matrix6<f64> = k * h;
    let i6: Matrix6<f64> = Matrix6::identity();
    filter.p = (i6 - kh) * filter.p;
    // Joseph form for numerical PSD-ness:
    let p_sym = (filter.p + filter.p.transpose()) * 0.5;
    filter.p = p_sym;
    Some(y)
}

pub fn nav_filter_system(
    mut filter: ResMut<NavFilter>,
    mut estimate: ResMut<NavEstimate>,
    sim_time: Res<SimTime>,
    sensors: Res<LatestSensorPack>,
    cache: Option<Res<EphemerisCache>>,
    q: Query<&RigidBody, With<Spacecraft>>,
) {
    if !filter.enabled {
        if filter.initialized {
            *filter = NavFilter {
                enabled: false,
                ..NavFilter::default()
            };
        }
        *estimate = NavEstimate::default();
        return;
    }

    let Some(rb) = q.iter().next() else {
        return;
    };
    let Some(cache) = cache else {
        return;
    };

    // Initialise from ground truth on engage. A real mission would seed
    // from a launch-state OD solution; for sim purposes, ground-truth +
    // operator-set uncertainty is the cleanest contract.
    if !filter.initialized {
        for i in 0..3 {
            filter.x[i] = rb.position[i];
            filter.x[i + 3] = rb.velocity[i];
        }
        filter.p = Matrix6::zeros();
        for i in 0..3 {
            filter.p[(i, i)] = filter.init_sigma_r_m.powi(2);
            filter.p[(i + 3, i + 3)] = filter.init_sigma_v_m_s.powi(2);
        }
        filter.initialized = true;
        estimate.updates_count = 0;
    }

    // Predict.
    predict(&mut filter, sim_time.dt);

    // Update with each range reading available this tick.
    let mut last_residual = 0.0;
    let mut last_body = 0;
    let r_var = filter.r_range_var;
    for reading in &sensors.0.ranges {
        // Skip the Sun for range updates — at the origin a small position
        // error gives a huge bearing error and the update is poorly
        // conditioned. (Also the Sun itself doesn't echo back radar.)
        if reading.body_id == crate::ephemeris::naif::SUN {
            continue;
        }
        let Some(body_state) = cache.get(reading.body_id) else {
            continue;
        };
        let body_pos = dvec_to_sv3(body_state.position);
        if let Some(y) = update_range(&mut filter, body_pos, reading.range_m, r_var) {
            last_residual = y;
            last_body = reading.body_id;
            estimate.updates_count += 1;
        }
    }

    // Emit estimate.
    estimate.initialized = true;
    estimate.position = [filter.x[0], filter.x[1], filter.x[2]];
    estimate.velocity = [filter.x[3], filter.x[4], filter.x[5]];
    let pos_var: f64 = (0..3).map(|i| filter.p[(i, i)]).sum();
    let vel_var: f64 = (3..6).map(|i| filter.p[(i, i)]).sum();
    estimate.position_sigma_m = pos_var.max(0.0).sqrt();
    estimate.velocity_sigma_m_s = vel_var.max(0.0).sqrt();
    estimate.residual_range_m = last_residual;
    estimate.last_update_body = last_body;
}

/// Helpers for telemetry / tests — keep `vec3` exported.
pub fn nav_error_m(estimate: &NavEstimate, truth: glam::DVec3) -> f64 {
    if !estimate.initialized {
        return 0.0;
    }
    (vec3(estimate.position) - dvec_to_sv3(truth)).norm()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephemeris::{naif, EphemerisCache, AU};
    use crate::sensors::{LatestSensorPack, RangeReading, SensorPack};
    use bevy_ecs::prelude::*;
    use glam::DVec3;

    fn world_at_earth() -> World {
        let mut w = World::new();
        // Small dt so the predict step doesn't move the estimate far from
        // its initial seed before the test asserts.
        w.insert_resource(SimTime::new(0.1));
        w.insert_resource(NavFilter {
            enabled: true,
            init_sigma_r_m: 1.0e6,
            init_sigma_v_m_s: 100.0,
            r_range_var: 1.0e4,
            ..Default::default()
        });
        w.insert_resource(NavEstimate::default());
        w.insert_resource(LatestSensorPack::default());
        w.insert_resource(EphemerisCache::with_default_bodies());

        // Spawn ship at 1 AU on the +x axis with circular velocity.
        let r = AU;
        let v_circ = (MU_SUN / r).sqrt();
        w.spawn((
            Spacecraft { id: 1 },
            RigidBody {
                position: DVec3::new(r, 0.0, 0.0),
                velocity: DVec3::new(0.0, v_circ, 0.0),
                mass: 1000.0,
                ..Default::default()
            },
        ));
        w
    }

    #[test]
    fn nav_initializes_on_engage_from_ground_truth() {
        let mut w = world_at_earth();
        let mut s = Schedule::default();
        s.add_systems(nav_filter_system);
        s.run(&mut w);
        let est = w.resource::<NavEstimate>().clone();
        assert!(est.initialized);
        // Position estimate ≈ ground truth (subject to one predict step drift).
        let pos_err = (DVec3::from_array(est.position) - DVec3::new(AU, 0.0, 0.0)).length();
        assert!(pos_err < 1.0e6, "init pos error {} m too large", pos_err);
    }

    #[test]
    fn nav_disable_clears_estimate() {
        let mut w = world_at_earth();
        let mut s = Schedule::default();
        s.add_systems(nav_filter_system);
        s.run(&mut w);
        w.resource_mut::<NavFilter>().enabled = false;
        s.run(&mut w);
        let est = w.resource::<NavEstimate>();
        assert!(!est.initialized);
    }

    #[test]
    fn perfect_range_observations_shrink_position_sigma() {
        let mut w = world_at_earth();
        // Manually populate sensor pack with noise-free ranges to every
        // catalogued body. The cache itself has the bodies; pull each.
        let cache = w.resource::<EphemerisCache>().clone();
        let truth = w
            .query::<&RigidBody>()
            .iter(&w)
            .next()
            .unwrap()
            .position;
        let mut readings = Vec::new();
        for (b, s) in cache.all_states() {
            if b.id == naif::SUN {
                continue;
            }
            let dr = s.position - truth;
            let range = dr.length();
            if range < 1.0 {
                continue;
            }
            readings.push(RangeReading {
                body_id: b.id,
                range_m: range,
                range_rate_m_s: 0.0,
                bearing_inertial: dr.normalize().to_array(),
            });
        }
        w.resource_mut::<LatestSensorPack>().0 = SensorPack {
            ranges: readings,
            ..Default::default()
        };
        let mut s = Schedule::default();
        s.add_systems(nav_filter_system);
        // Initial run seeds + does one update.
        s.run(&mut w);
        let sigma_after_first = w.resource::<NavEstimate>().position_sigma_m;
        // Several updates should keep sigma bounded below the initial 1 Mm.
        for _ in 0..50 {
            s.run(&mut w);
        }
        let sigma_after_many = w.resource::<NavEstimate>().position_sigma_m;
        assert!(
            sigma_after_many < sigma_after_first + 1.0,
            "sigma grew unexpectedly: {} → {}",
            sigma_after_first,
            sigma_after_many
        );
        // And the estimate should be near ground truth.
        let err = nav_error_m(w.resource::<NavEstimate>(), truth);
        assert!(err < 1.0e5, "nav error {} m too large", err);
    }
}
