//! Synthetic sensor model for navigation / localization testing.
//!
//! The goal of this module is to make `sim_core` usable as a **digital twin**
//! for autonomy research: given the ground-truth state, generate the noisy
//! measurements an onboard nav stack would actually consume. Each
//! measurement is bundled into a `SensorReading` and broadcast alongside
//! telemetry so an external estimator can be driven without ever seeing the
//! ground truth directly.
//!
//! Modelled measurements:
//!
//! * **Range** — scalar distance to each catalogued body (m). White Gaussian
//!   noise with σ proportional to range.
//! * **Range rate** — closing speed (m/s, +closing). σ in m/s.
//! * **Bearing** — unit line-of-sight from spacecraft to the body in the
//!   *inertial* frame. Angular noise applied as a small random rotation of
//!   the LOS unit vector.
//! * **IMU** — body-frame specific force (commanded thrust / mass + random
//!   walk noise) and body-frame angular rate, mirroring an ADIS-class IMU.
//! * **Attitude** — quaternion from a star-tracker, with small isotropic
//!   pointing noise.
//!
//! Noise is deterministic per-tick via a small xorshift PRNG seeded by sim
//! time + body id. Reproducibility matters: a test that "this nav filter
//! achieves σ_pos < 50 km over 1 day" needs the exact same noise on every
//! replay.

use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use serde::Serialize;

use crate::clock::SimTime;
use crate::components::{CommandedWrench, PropulsionDrive, RigidBody, Spacecraft};
use crate::ephemeris::EphemerisCache;

/// Per-body radar / lidar range measurement.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct RangeReading {
    pub body_id: i32,
    pub range_m: f64,
    pub range_rate_m_s: f64,
    pub bearing_inertial: [f64; 3],
}

/// IMU sample. Frame: body. Specific force convention: SI m/s², gravity-free
/// (no gravity included — that's the autonomy stack's job to add back in
/// from the ephemeris model).
#[derive(Debug, Clone, Copy, Serialize)]
pub struct ImuReading {
    pub specific_force_body: [f64; 3],
    pub angular_rate_body: [f64; 3],
}

/// Star-tracker attitude estimate.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct AttitudeReading {
    pub q_xyzw: [f64; 4],
}

/// Complete sensor pack for a single tick.
#[derive(Debug, Clone, Default, Serialize)]
pub struct SensorPack {
    pub ranges: Vec<RangeReading>,
    pub imu: Option<ImuReading>,
    pub attitude: Option<AttitudeReading>,
}

/// Operator-tunable noise model. All sigmas are 1-sigma. Set everything to
/// zero to get noise-free ground-truth-mirror measurements (useful for
/// initial autonomy bring-up).
#[derive(Resource, Debug, Clone, Copy, Serialize)]
pub struct SensorConfig {
    /// Range σ as a fraction of true range. 1 part in 10⁶ ≈ space-DSN class.
    pub range_relative_sigma: f64,
    /// Floor on range noise (m), in case relative σ gets unrealistically small.
    pub range_absolute_sigma_m: f64,
    /// Range-rate σ (m/s).
    pub range_rate_sigma_m_s: f64,
    /// Bearing σ (radians).
    pub bearing_sigma_rad: f64,
    /// Accelerometer σ (m/s²).
    pub accel_sigma_m_s2: f64,
    /// Gyro σ (rad/s).
    pub gyro_sigma_rad_s: f64,
    /// Star-tracker σ (radians) per axis.
    pub star_tracker_sigma_rad: f64,
    /// Enable / disable. When false the system runs but writes zero-σ
    /// readings — useful for "ground truth" replay.
    pub enabled: bool,
    /// Apply one-way light-time delay to range / range-rate readings. When
    /// true, the body's position is propagated linearly *backwards* by
    /// `range / c` before the range / Doppler observation is computed.
    /// Off by default so existing nav-filter tests (which assume zero
    /// delay) keep passing; flip on for realistic DSN-style observations.
    pub light_time_delay: bool,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            range_relative_sigma: 1.0e-5,
            range_absolute_sigma_m: 10.0,
            range_rate_sigma_m_s: 0.05,
            bearing_sigma_rad: 1.0e-5, // ~2 arcseconds
            accel_sigma_m_s2: 1.0e-4,
            gyro_sigma_rad_s: 1.0e-6,
            star_tracker_sigma_rad: 1.0e-5,
            enabled: true,
            light_time_delay: false,
        }
    }
}

/// Output resource consumed by the host (sim_server) for telemetry.
#[derive(Resource, Debug, Default)]
pub struct LatestSensorPack(pub SensorPack);

/// Compact, deterministic PRNG. Seeded from (sim_time, body_id) so replays
/// hit the same noise samples.
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed },
        }
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }
    fn next_f01(&mut self) -> f64 {
        // 53-bit precision uniform [0,1)
        (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
    }
    /// Standard normal via Box–Muller. Both draws consumed for two samples.
    fn next_pair_normal(&mut self) -> (f64, f64) {
        let u1 = (self.next_f01()).max(1.0e-300);
        let u2 = self.next_f01();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * std::f64::consts::PI * u2;
        (r * theta.cos(), r * theta.sin())
    }
    fn next_normal(&mut self) -> f64 {
        self.next_pair_normal().0
    }
}

fn seed_for(sim_time: f64, salt: u64) -> u64 {
    // Bit-tweaked mix — collisions don't matter for noise but we want decent
    // sample diversity tick to tick.
    let t = sim_time.to_bits();
    let mut s = t.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    s ^= salt.wrapping_add(0x94D0_49BB_1331_11EB);
    s = s.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    s ^ (s >> 31)
}

/// Rotate `v` by a small random rotation with isotropic 1-σ `sigma` rad.
fn jitter_unit_vec(v: DVec3, sigma: f64, rng: &mut XorShift64) -> DVec3 {
    if sigma <= 0.0 || v.length_squared() < 1e-30 {
        return v;
    }
    let (n1, n2) = rng.next_pair_normal();
    let dtheta = (n1.hypot(n2)) * sigma;
    // Random rotation axis perpendicular to v.
    let helper = if v.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
    let axis = v.cross(helper).normalize();
    let q = DQuat::from_axis_angle(axis, dtheta);
    q * v
}

/// Builds and stores the per-tick sensor pack. Run *after* dynamics so the
/// readings reflect the post-step state.
pub fn sensor_system(
    cfg: Res<SensorConfig>,
    cache: Option<Res<EphemerisCache>>,
    sim_time: Res<SimTime>,
    q: Query<(&RigidBody, Option<&CommandedWrench>, Option<&PropulsionDrive>), With<Spacecraft>>,
    mut out: ResMut<LatestSensorPack>,
) {
    let Some((rb, wrench, drive)) = q.iter().next() else {
        out.0 = SensorPack::default();
        return;
    };
    let Some(cache) = cache else {
        out.0 = SensorPack::default();
        return;
    };

    let mut pack = SensorPack::default();

    // Speed of light (m/s). Used by the optional light-time-delay path.
    const C_M_S: f64 = 299_792_458.0;

    for (b, state) in cache.all_states() {
        // If light-time delay is on, the observation we see at sim-time `t`
        // came from where the body actually was `range/c` seconds ago.
        // First pass: compute the current geometric range; second pass:
        // shift the body's position back along its velocity by τ = range/c
        // and recompute. One iteration is enough — light-time τ << the
        // body's orbital period, so a linear back-step is accurate to
        // O((vτ)²/c²) — i.e., bounded by (30 km/s · 22 min / c)² ≈ 10⁻¹⁰.
        let (eff_position, eff_velocity) = if cfg.light_time_delay {
            let rough_range = (state.position - rb.position).length();
            let tau = rough_range / C_M_S;
            (state.position - state.velocity * tau, state.velocity)
        } else {
            (state.position, state.velocity)
        };

        let rel = eff_position - rb.position;
        let range = rel.length();
        if range < 1.0 {
            continue;
        }
        let los = rel / range;
        let rel_v = eff_velocity - rb.velocity;
        let closing = -rel_v.dot(los); // +closing ⇒ approaching

        let mut rng = XorShift64::new(seed_for(sim_time.time, b.id as u64));
        let (r_noise_m, dr_noise) = if cfg.enabled {
            let r_sigma = (range * cfg.range_relative_sigma).max(cfg.range_absolute_sigma_m);
            let (a, c) = rng.next_pair_normal();
            (a * r_sigma, c * cfg.range_rate_sigma_m_s)
        } else {
            (0.0, 0.0)
        };
        let bearing = if cfg.enabled {
            jitter_unit_vec(los, cfg.bearing_sigma_rad, &mut rng)
        } else {
            los
        };
        pack.ranges.push(RangeReading {
            body_id: b.id,
            range_m: (range + r_noise_m).max(0.0),
            range_rate_m_s: closing + dr_noise,
            bearing_inertial: bearing.to_array(),
        });
    }

    // IMU specific force: thrust + commanded wrench, divided by mass. We do
    // *not* include gravity — that's what makes "specific force" useful for
    // strapdown inertial nav: the gravity model lives in the estimator.
    let mut force_body = DVec3::ZERO;
    if let Some(d) = drive {
        force_body += d.realised_thrust();
    }
    if let Some(w) = wrench {
        force_body += rb.inertial_to_body(w.force);
    }
    let mut a_body = force_body / rb.mass.max(1e-12);
    let mut omega = rb.angular_velocity;
    if cfg.enabled {
        let mut rng = XorShift64::new(seed_for(sim_time.time, 0xA5A5_FAFA));
        for ax in [&mut a_body.x, &mut a_body.y, &mut a_body.z] {
            *ax += rng.next_normal() * cfg.accel_sigma_m_s2;
        }
        for ax in [&mut omega.x, &mut omega.y, &mut omega.z] {
            *ax += rng.next_normal() * cfg.gyro_sigma_rad_s;
        }
    }
    pack.imu = Some(ImuReading {
        specific_force_body: a_body.to_array(),
        angular_rate_body: omega.to_array(),
    });

    // Star tracker: jitter the attitude quaternion's rotation axis.
    let q_true = rb.attitude;
    let q_meas = if cfg.enabled {
        let mut rng = XorShift64::new(seed_for(sim_time.time, 0xB10C_5C12));
        // Sample a small rotation: angle ~ N(0, σ), axis uniform.
        let (n1, n2) = rng.next_pair_normal();
        let angle = (n1.hypot(n2)) * cfg.star_tracker_sigma_rad;
        let axis = DVec3::new(rng.next_normal(), rng.next_normal(), rng.next_normal())
            .try_normalize()
            .unwrap_or(DVec3::X);
        let dq = DQuat::from_axis_angle(axis, angle);
        (dq * q_true).normalize()
    } else {
        q_true
    };
    pack.attitude = Some(AttitudeReading {
        q_xyzw: [q_meas.x, q_meas.y, q_meas.z, q_meas.w],
    });

    out.0 = pack;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ephemeris::{naif, EphemerisCache};
    use approx::assert_relative_eq;

    fn world_with_ship() -> World {
        let mut w = World::new();
        w.insert_resource(SimTime::default());
        w.insert_resource(SensorConfig {
            enabled: false,
            ..Default::default()
        });
        w.insert_resource(EphemerisCache::with_default_bodies());
        w.insert_resource(LatestSensorPack::default());
        w.spawn((
            Spacecraft { id: 1 },
            RigidBody {
                position: DVec3::new(1.5e11, 0.0, 0.0),
                velocity: DVec3::ZERO,
                mass: 1000.0,
                ..Default::default()
            },
            CommandedWrench::default(),
        ));
        w
    }

    #[test]
    fn noiseless_range_matches_geometry() {
        let mut w = world_with_ship();
        let mut s = Schedule::default();
        s.add_systems(sensor_system);
        s.run(&mut w);
        let pack = &w.resource::<LatestSensorPack>().0;
        // Earth at ~1 AU on ecliptic. Our ship sits at (1.5e11, 0, 0). Range
        // to Earth from there should be small (within Earth's orbital radius).
        let earth = pack.ranges.iter().find(|r| r.body_id == naif::EARTH).unwrap();
        assert!(
            earth.range_m > 0.0 && earth.range_m < 2.0 * crate::ephemeris::AU,
            "earth range out of bounds: {}",
            earth.range_m
        );
    }

    #[test]
    fn noisy_range_is_within_bounds() {
        let mut w = world_with_ship();
        {
            let mut cfg = w.resource_mut::<SensorConfig>();
            cfg.enabled = true;
            cfg.range_relative_sigma = 0.0;
            cfg.range_absolute_sigma_m = 1000.0;
        }
        let mut s = Schedule::default();
        s.add_systems(sensor_system);
        s.run(&mut w);
        let pack = &w.resource::<LatestSensorPack>().0;
        let earth = pack.ranges.iter().find(|r| r.body_id == naif::EARTH).unwrap();
        // True range minus a 6-sigma envelope must still be well above zero.
        assert!(earth.range_m > 1e10);
    }

    #[test]
    fn imu_specific_force_excludes_gravity() {
        // With no thrust and zero commanded wrench, specific force = 0
        // regardless of position in the gravity well.
        let mut w = world_with_ship();
        let mut s = Schedule::default();
        s.add_systems(sensor_system);
        s.run(&mut w);
        let imu = w.resource::<LatestSensorPack>().0.imu.unwrap();
        for c in imu.specific_force_body {
            assert!(c.abs() < 1e-9, "specific force should be zero: {}", c);
        }
    }

    #[test]
    fn determinism_replay_under_noise() {
        // Use a fresh schedule per world — bevy_ecs's schedule caches the
        // owning WorldId, so one cannot legally re-run a schedule against a
        // different world.
        let mut w1 = world_with_ship();
        let mut w2 = world_with_ship();
        for w in [&mut w1, &mut w2] {
            w.resource_mut::<SensorConfig>().enabled = true;
        }
        let mut s1 = Schedule::default();
        s1.add_systems(sensor_system);
        let mut s2 = Schedule::default();
        s2.add_systems(sensor_system);
        s1.run(&mut w1);
        s2.run(&mut w2);
        let a = &w1.resource::<LatestSensorPack>().0;
        let b = &w2.resource::<LatestSensorPack>().0;
        assert!(!a.ranges.is_empty());
        for (ra, rb) in a.ranges.iter().zip(b.ranges.iter()) {
            assert_relative_eq!(ra.range_m, rb.range_m, epsilon = 0.0);
        }
    }

    #[test]
    fn light_time_delay_shifts_observed_range_toward_past_body_position() {
        // With light-time delay on, the observed range to a body should be
        // the distance to where the body WAS one light-time ago — for a
        // body moving away from us, that's a shorter range than the
        // geometric "now" range.
        let mut w_off = world_with_ship();
        let mut w_on = world_with_ship();
        w_on.resource_mut::<SensorConfig>().light_time_delay = true;

        // Place a fast-moving synthetic body well clear of the default
        // roster. We pin its state directly so we know the answer.
        for w in [&mut w_off, &mut w_on] {
            let cache = w.resource::<EphemerisCache>().clone();
            cache.set_state(
                crate::ephemeris::BodyParams {
                    id: 9999,
                    name: "Probe",
                    mu: 0.0,
                    radius: 1.0,
                    kepler: None,
                    parent_body: None,
                },
                crate::ephemeris::BodyState {
                    // 1 AU "ahead" of the ship's +x; moving at +30 km/s in +x
                    // (i.e. away from the ship, so light-time path shortens).
                    position: DVec3::new(1.5e11 + crate::ephemeris::AU, 0.0, 0.0),
                    velocity: DVec3::new(30_000.0, 0.0, 0.0),
                },
            );
        }
        let mut sched_off = Schedule::default();
        sched_off.add_systems(sensor_system);
        let mut sched_on = Schedule::default();
        sched_on.add_systems(sensor_system);
        sched_off.run(&mut w_off);
        sched_on.run(&mut w_on);

        let r_off = w_off
            .resource::<LatestSensorPack>()
            .0
            .ranges
            .iter()
            .find(|r| r.body_id == 9999)
            .unwrap()
            .range_m;
        let r_on = w_on
            .resource::<LatestSensorPack>()
            .0
            .ranges
            .iter()
            .find(|r| r.body_id == 9999)
            .unwrap()
            .range_m;

        // Expected light-time correction: τ ≈ r/c ≈ 1 AU / c ≈ 499 s.
        // Body moves +30 km/s × 499 s ≈ 1.5e7 m in the +x direction over
        // that interval — but we observe where it WAS, so the corrected
        // range is shorter than the geometric "now" range by ≈ that delta.
        let delta = r_off - r_on;
        assert!(
            delta > 1.0e7 && delta < 2.0e7,
            "expected light-time correction in 10–20 Mm range, got Δ={} m",
            delta,
        );
    }

    #[test]
    fn jitter_unit_vec_preserves_unit_norm() {
        let mut rng = XorShift64::new(42);
        let v = DVec3::new(1.0, 0.0, 0.0);
        for _ in 0..1000 {
            let j = jitter_unit_vec(v, 0.01, &mut rng);
            assert_relative_eq!(j.length(), 1.0, epsilon = 1e-12);
        }
    }
}
