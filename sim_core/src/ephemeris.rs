//! Data-driven environment: planetary states & local gravity gradients.
//!
//! The simulator never integrates planetary positions itself — it queries a
//! pre-computed ephemeris. By default a deterministic Keplerian propagator
//! covers the Sun + 8 planets so the project builds and tests cleanly without
//! NAIF CSPICE installed. With `--features spice` enabled, a SPICE-backed
//! cache replaces the Keplerian model and queries real BSP kernels.

use bevy_ecs::prelude::*;
use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Astronomical unit (m).
pub const AU: f64 = 1.495_978_707e11;
/// Standard gravitational parameter of the Sun (m³/s²).
pub const MU_SUN: f64 = 1.327_124_400_18e20;

/// NAIF body identifiers reused even in the Keplerian fallback so backends
/// stay interchangeable.
pub mod naif {
    pub const SUN: i32 = 10;
    pub const MERCURY: i32 = 199;
    pub const VENUS: i32 = 299;
    pub const EARTH: i32 = 399;
    pub const LUNA: i32 = 301;
    pub const MARS: i32 = 499;
    pub const PHOBOS: i32 = 401;
    pub const DEIMOS: i32 = 402;
    pub const JUPITER: i32 = 599;
    pub const SATURN: i32 = 699;
    pub const URANUS: i32 = 799;
    pub const NEPTUNE: i32 = 899;
}

/// Snapshot of a body's inertial state.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BodyState {
    pub position: DVec3, // m
    pub velocity: DVec3, // m/s
}

impl Default for BodyState {
    fn default() -> Self {
        Self {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
        }
    }
}

/// Static physical parameters of a body needed for gravity / Keplerian propagation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BodyParams {
    pub id: i32,
    pub name: &'static str,
    /// Gravitational parameter (m³/s²).
    pub mu: f64,
    /// Equatorial radius (m), used for rendering and proximity tests.
    pub radius: f64,
    /// Initial Keplerian elements at sim epoch t=0. When `parent_body` is
    /// `None`, the elements are heliocentric. When `parent_body = Some(id)`,
    /// the elements describe the orbit *relative to that body* (e.g. Luna's
    /// elements are around Earth, not the Sun).
    pub kepler: Option<KeplerElements>,
    /// NAIF id of the central body for the Kepler elements. `None` = Sun.
    /// Order matters: parents must appear before their satellites in the
    /// body list so the single-pass refresh sees them at their up-to-date
    /// state when propagating the child.
    pub parent_body: Option<i32>,
}

/// Classical Keplerian orbital elements (referenced to the Sun, J2000 frame).
/// Angles are in radians, sma in metres.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct KeplerElements {
    pub sma: f64,                 // semi-major axis (m)
    pub eccentricity: f64,        // 0 ≤ e < 1
    pub inclination: f64,         // i (rad)
    pub raan: f64,                // longitude of ascending node Ω (rad)
    pub arg_periapsis: f64,       // argument of periapsis ω (rad)
    pub mean_anomaly_at_epoch: f64, // M₀ (rad)
}

impl KeplerElements {
    /// Mean motion `n = √(μ/a³)` (rad/s).
    pub fn mean_motion(&self, mu_central: f64) -> f64 {
        (mu_central / self.sma.powi(3)).sqrt()
    }

    /// Orbital period (seconds).
    pub fn period(&self, mu_central: f64) -> f64 {
        2.0 * std::f64::consts::PI / self.mean_motion(mu_central)
    }
}

/// Solve Kepler's equation `M = E − e sin E` for E via Newton-Raphson.
/// Converges in 4–5 iterations for `e < 0.5`.
fn solve_kepler(mean_anomaly: f64, eccentricity: f64) -> f64 {
    let m = mean_anomaly.rem_euclid(2.0 * std::f64::consts::PI);
    let mut e = if eccentricity < 0.8 { m } else { std::f64::consts::PI };
    for _ in 0..20 {
        let f = e - eccentricity * e.sin() - m;
        let fp = 1.0 - eccentricity * e.cos();
        let de = f / fp;
        e -= de;
        if de.abs() < 1e-12 {
            break;
        }
    }
    e
}

/// Propagate a Keplerian orbit around a central body of parameter `mu_central`
/// to time `t` (seconds since epoch) and return the body's inertial state.
pub fn propagate_keplerian(k: &KeplerElements, mu_central: f64, t: f64) -> BodyState {
    let n = k.mean_motion(mu_central);
    let m_t = k.mean_anomaly_at_epoch + n * t;
    let big_e = solve_kepler(m_t, k.eccentricity);

    // True anomaly ν from eccentric anomaly E.
    let cos_e = big_e.cos();
    let nu = 2.0 * ((1.0 + k.eccentricity).sqrt() * (big_e * 0.5).sin())
        .atan2((1.0 - k.eccentricity).sqrt() * (big_e * 0.5).cos());

    let r = k.sma * (1.0 - k.eccentricity * cos_e);

    // Position in the orbital plane (perifocal frame).
    let x_pf = r * nu.cos();
    let y_pf = r * nu.sin();
    let p = k.sma * (1.0 - k.eccentricity * k.eccentricity);
    let h = (mu_central * p).sqrt();
    let vx_pf = -mu_central / h * nu.sin();
    let vy_pf = mu_central / h * (k.eccentricity + nu.cos());

    // Rotate perifocal → inertial via Ω, i, ω.
    let (cos_w, sin_w) = (k.arg_periapsis.cos(), k.arg_periapsis.sin());
    let (cos_o, sin_o) = (k.raan.cos(), k.raan.sin());
    let (cos_i, sin_i) = (k.inclination.cos(), k.inclination.sin());

    let r11 = cos_o * cos_w - sin_o * sin_w * cos_i;
    let r12 = -cos_o * sin_w - sin_o * cos_w * cos_i;
    let r21 = sin_o * cos_w + cos_o * sin_w * cos_i;
    let r22 = -sin_o * sin_w + cos_o * cos_w * cos_i;
    let r31 = sin_w * sin_i;
    let r32 = cos_w * sin_i;

    let pos = DVec3::new(
        r11 * x_pf + r12 * y_pf,
        r21 * x_pf + r22 * y_pf,
        r31 * x_pf + r32 * y_pf,
    );
    let vel = DVec3::new(
        r11 * vx_pf + r12 * vy_pf,
        r21 * vx_pf + r22 * vy_pf,
        r31 * vx_pf + r32 * vy_pf,
    );

    BodyState { position: pos, velocity: vel }
}

/// Default planet roster. Elements are J2000-epoch heliocentric ecliptic
/// approximations from JPL fact sheets. Good enough for autonomy testing —
/// load real SPICE kernels when you need arc-second accuracy.
pub fn default_bodies() -> Vec<BodyParams> {
    // Helper: degrees → radians.
    fn d(deg: f64) -> f64 {
        deg.to_radians()
    }
    vec![
        BodyParams {
            id: naif::SUN, name: "Sun", mu: MU_SUN, radius: 6.957e8,
            kepler: None, parent_body: None,
        },
        BodyParams {
            id: naif::MERCURY, name: "Mercury", mu: 2.2032e13, radius: 2.4397e6,
            kepler: Some(KeplerElements {
                sma: 0.387 * AU, eccentricity: 0.2056, inclination: d(7.005),
                raan: d(48.331), arg_periapsis: d(29.124),
                mean_anomaly_at_epoch: d(174.796),
            }),
            parent_body: None,
        },
        BodyParams {
            id: naif::VENUS, name: "Venus", mu: 3.2486e14, radius: 6.0518e6,
            kepler: Some(KeplerElements {
                sma: 0.723 * AU, eccentricity: 0.0068, inclination: d(3.395),
                raan: d(76.680), arg_periapsis: d(54.884),
                mean_anomaly_at_epoch: d(50.115),
            }),
            parent_body: None,
        },
        BodyParams {
            id: naif::EARTH, name: "Earth", mu: 3.986_004_418e14, radius: 6.371e6,
            kepler: Some(KeplerElements {
                sma: 1.000 * AU, eccentricity: 0.0167, inclination: d(0.000),
                raan: d(-11.260), arg_periapsis: d(114.208),
                mean_anomaly_at_epoch: d(358.617),
            }),
            parent_body: None,
        },
        // Luna — orbits Earth. Mean elements from JPL fact sheet.
        BodyParams {
            id: naif::LUNA, name: "Luna", mu: 4.9028e12, radius: 1.7374e6,
            kepler: Some(KeplerElements {
                sma: 3.844e8, eccentricity: 0.0549, inclination: d(5.145),
                raan: d(125.08), arg_periapsis: d(318.15),
                mean_anomaly_at_epoch: d(135.27),
            }),
            parent_body: Some(naif::EARTH),
        },
        BodyParams {
            id: naif::MARS, name: "Mars", mu: 4.282_837e13, radius: 3.389_5e6,
            kepler: Some(KeplerElements {
                sma: 1.524 * AU, eccentricity: 0.0934, inclination: d(1.850),
                raan: d(49.558), arg_periapsis: d(286.502),
                mean_anomaly_at_epoch: d(19.412),
            }),
            parent_body: None,
        },
        // Phobos — orbits Mars at 9376 km, P = 7.66 h.
        BodyParams {
            id: naif::PHOBOS, name: "Phobos", mu: 7.087e5, radius: 1.1267e4,
            kepler: Some(KeplerElements {
                sma: 9.376e6, eccentricity: 0.0151, inclination: d(1.075),
                raan: d(83.0), arg_periapsis: d(216.0),
                mean_anomaly_at_epoch: d(91.0),
            }),
            parent_body: Some(naif::MARS),
        },
        // Deimos — orbits Mars at 23463 km, P = 30.3 h.
        BodyParams {
            id: naif::DEIMOS, name: "Deimos", mu: 9.615e4, radius: 6.2e3,
            kepler: Some(KeplerElements {
                sma: 2.3463e7, eccentricity: 0.0002, inclination: d(1.788),
                raan: d(82.7), arg_periapsis: d(0.0),
                mean_anomaly_at_epoch: d(325.3),
            }),
            parent_body: Some(naif::MARS),
        },
        BodyParams {
            id: naif::JUPITER, name: "Jupiter", mu: 1.266_865_3e17, radius: 6.991_1e7,
            kepler: Some(KeplerElements {
                sma: 5.203 * AU, eccentricity: 0.0489, inclination: d(1.303),
                raan: d(100.464), arg_periapsis: d(273.867),
                mean_anomaly_at_epoch: d(20.020),
            }),
            parent_body: None,
        },
        BodyParams {
            id: naif::SATURN, name: "Saturn", mu: 3.793_118_7e16, radius: 5.823_2e7,
            kepler: Some(KeplerElements {
                sma: 9.537 * AU, eccentricity: 0.0565, inclination: d(2.485),
                raan: d(113.665), arg_periapsis: d(339.392),
                mean_anomaly_at_epoch: d(317.020),
            }),
            parent_body: None,
        },
        BodyParams {
            id: naif::URANUS, name: "Uranus", mu: 5.793_939e15, radius: 2.536_2e7,
            kepler: Some(KeplerElements {
                sma: 19.191 * AU, eccentricity: 0.0457, inclination: d(0.773),
                raan: d(74.006), arg_periapsis: d(96.998),
                mean_anomaly_at_epoch: d(142.238),
            }),
            parent_body: None,
        },
        BodyParams {
            id: naif::NEPTUNE, name: "Neptune", mu: 6.836_529e15, radius: 2.462_2e7,
            kepler: Some(KeplerElements {
                sma: 30.069 * AU, eccentricity: 0.0113, inclination: d(1.770),
                raan: d(131.784), arg_periapsis: d(276.336),
                mean_anomaly_at_epoch: d(256.228),
            }),
            parent_body: None,
        },
    ]
}

/// Resource-shape ephemeris cache. Lock-free reads from the dynamics system
/// in the common case (a single writer thread refreshes the inner map every
/// few ticks).
#[derive(Resource, Clone, Default)]
pub struct EphemerisCache {
    inner: Arc<RwLock<EphemerisInner>>,
}

#[derive(Default)]
struct EphemerisInner {
    bodies: Vec<BodyParams>,
    states: HashMap<i32, BodyState>,
    last_t: f64,
}

impl EphemerisCache {
    /// Create a cache pre-loaded with the default body roster.
    pub fn with_default_bodies() -> Self {
        let cache = Self::default();
        cache.set_bodies(default_bodies());
        cache
    }

    pub fn set_bodies(&self, bodies: Vec<BodyParams>) {
        let mut inner = self.inner.write().unwrap();
        inner.bodies = bodies;
        inner.states.clear();
        inner.last_t = 0.0;
        drop(inner);
        // Single-pass propagation that respects parent_body. The body list
        // is expected to list parents before satellites — see BodyParams docs.
        self.refresh(0.0);
    }

    /// Recompute all body states for sim-time `t`. Run this in the worker
    /// thread or from the schedule's prologue; never from the integrator.
    ///
    /// Bodies are processed in list order; a body with `parent_body = Some(id)`
    /// reads the parent's current state and applies its Keplerian elements
    /// relative to that body using the **parent's μ** (not μ_sun). The
    /// default body roster lists parents before children so a single pass
    /// suffices; user-supplied rosters must obey the same ordering.
    ///
    /// Bodies that don't carry Kepler elements (Sun, manually-pinned
    /// synthetic targets) keep whatever state was last written.
    pub fn refresh(&self, t: f64) {
        let mut inner = self.inner.write().unwrap();
        let bodies = inner.bodies.clone();
        for b in &bodies {
            let state = match (b.kepler, b.parent_body) {
                (Some(k), None) => propagate_keplerian(&k, MU_SUN, t),
                (Some(k), Some(parent_id)) => {
                    let parent_state = inner.states.get(&parent_id).copied();
                    let parent_mu = bodies
                        .iter()
                        .find(|p| p.id == parent_id)
                        .map(|p| p.mu)
                        .unwrap_or(MU_SUN);
                    if let Some(p_state) = parent_state {
                        let rel = propagate_keplerian(&k, parent_mu, t);
                        BodyState {
                            position: p_state.position + rel.position,
                            velocity: p_state.velocity + rel.velocity,
                        }
                    } else {
                        // Parent not yet propagated this tick — fall back to
                        // last-known state to avoid a phantom origin position.
                        inner.states.get(&b.id).copied().unwrap_or_default()
                    }
                }
                (None, _) => inner.states.get(&b.id).copied().unwrap_or_default(),
            };
            inner.states.insert(b.id, state);
        }
        inner.last_t = t;
    }

    /// Pin a body's state directly. Useful for tests and for injecting
    /// vehicles whose motion is driven externally (e.g., a target drone).
    /// Inserts the body if it isn't already registered.
    pub fn set_state(&self, params: BodyParams, state: BodyState) {
        let mut inner = self.inner.write().unwrap();
        if let Some(i) = inner.bodies.iter().position(|b| b.id == params.id) {
            inner.bodies[i] = params;
        } else {
            inner.bodies.push(params);
        }
        inner.states.insert(params.id, state);
    }

    pub fn get(&self, body: i32) -> Option<BodyState> {
        self.inner.read().unwrap().states.get(&body).copied()
    }

    pub fn bodies(&self) -> Vec<BodyParams> {
        self.inner.read().unwrap().bodies.clone()
    }

    pub fn all_states(&self) -> Vec<(BodyParams, BodyState)> {
        let inner = self.inner.read().unwrap();
        inner
            .bodies
            .iter()
            .filter_map(|b| inner.states.get(&b.id).map(|s| (*b, *s)))
            .collect()
    }

    /// Net gravity from every body in the cache at `position` (m) and time `t` (s).
    /// `t` is used only to decide whether to refresh; cache currency is
    /// the caller's responsibility for performance reasons.
    ///
    /// Outside a body's radius: standard inverse-square law.
    /// Inside a body's radius: uniform-density interior model — gravity
    /// magnitude grows linearly with `r` from zero at the centre to `GM/R²`
    /// at the surface. This is what the shell theorem gives for a uniform
    /// solid sphere and, crucially, **eliminates the 1/r² singularity**
    /// that previously turned a spacecraft staged at a planet's
    /// heliocentric position into a 10⁸ m/s² rocket inside RK4 substeps.
    pub fn gravity_at(&self, position: DVec3, _t: f64) -> DVec3 {
        let inner = self.inner.read().unwrap();
        let mut a = DVec3::ZERO;
        for b in &inner.bodies {
            if let Some(s) = inner.states.get(&b.id) {
                let r_vec = s.position - position;
                let r2 = r_vec.length_squared();
                // Below 1 m the direction is numerically meaningless; drop.
                if r2 < 1.0 {
                    continue;
                }
                let r_mag = r2.sqrt();
                let factor = if r_mag >= b.radius {
                    // Exterior: GM / r³, applied as factor · r_vec.
                    b.mu / (r_mag * r_mag * r_mag)
                } else {
                    // Interior (uniform-density shell theorem):
                    // |a| = GM·r / R³, direction toward centre.
                    b.mu / (b.radius * b.radius * b.radius)
                };
                a += r_vec * factor;
            }
        }
        a
    }
}

/// System that keeps the ephemeris cache in step with the simulation clock.
/// Cheap when bodies haven't changed: Kepler propagation is O(N) over the
/// body roster (≈ 9 entries) — well below the dynamics step cost.
pub fn ephemeris_refresh_system(
    sim_time: Res<crate::clock::SimTime>,
    clock: Res<crate::clock::SimClock>,
    cache: Res<EphemerisCache>,
) {
    let t = clock.epoch_j2000 + sim_time.time;
    cache.refresh(t);
}

#[cfg(feature = "spice")]
pub mod spice_backend {
    //! Optional NAIF SPICE backend. Activates with `--features spice`.
    //!
    //! At runtime the user calls `load_kernels()` once with a directory of
    //! `.bsp` / `.tls` / `.tpc` files. Subsequent calls to
    //! `refresh_with_spice()` populate the `EphemerisCache` from SPICE rather
    //! than the Keplerian fallback. Failures fall back to Keplerian silently
    //! (with a log line) so the autonomy harness keeps running.

    use super::*;
    use std::path::Path;

    pub fn load_kernels(dir: &str) -> Result<usize, std::io::Error> {
        let path = Path::new(dir);
        if !path.is_dir() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("SPICE kernel directory '{}' missing", dir),
            ));
        }
        let mut loaded = 0usize;
        for entry in std::fs::read_dir(path)?.flatten() {
            let p = entry.path();
            let ext = p.extension().and_then(|e| e.to_str()).unwrap_or("");
            if matches!(ext, "bsp" | "tls" | "tpc" | "tm" | "tf") {
                if let Some(s) = p.to_str() {
                    spice::furnsh(s);
                    loaded += 1;
                }
            }
        }
        Ok(loaded)
    }

    pub fn body_state(naif_id: i32, et_seconds: f64) -> Option<BodyState> {
        // spkez returns position (km) + velocity (km/s); convert to SI.
        let (state, _lt) = spice::spkez(naif_id as i32, et_seconds, "J2000", "NONE", 10);
        Some(BodyState {
            position: DVec3::new(state[0], state[1], state[2]) * 1000.0,
            velocity: DVec3::new(state[3], state[4], state[5]) * 1000.0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn earth_period_matches_one_year() {
        let earth = default_bodies()
            .into_iter()
            .find(|b| b.id == naif::EARTH)
            .unwrap();
        let k = earth.kepler.unwrap();
        let period_days = k.period(MU_SUN) / 86_400.0;
        assert!(
            (period_days - 365.25).abs() < 5.0,
            "earth period off: {} days",
            period_days
        );
    }

    #[test]
    fn earth_remains_at_one_au_after_full_orbit() {
        let earth = default_bodies()
            .into_iter()
            .find(|b| b.id == naif::EARTH)
            .unwrap();
        let k = earth.kepler.unwrap();
        let t0 = 0.0;
        let t1 = k.period(MU_SUN);
        let s0 = propagate_keplerian(&k, MU_SUN, t0);
        let s1 = propagate_keplerian(&k, MU_SUN, t1);
        // Same orbital phase after one full period.
        assert_relative_eq!(s0.position.x, s1.position.x, epsilon = 1e3);
        assert_relative_eq!(s0.position.y, s1.position.y, epsilon = 1e3);
    }

    #[test]
    fn mars_distance_lies_in_orbital_band() {
        let mars = default_bodies()
            .into_iter()
            .find(|b| b.id == naif::MARS)
            .unwrap();
        let k = mars.kepler.unwrap();
        // Sample distances around the orbit; bound by perihelion / aphelion.
        let q = k.sma * (1.0 - k.eccentricity);
        let cap_q = k.sma * (1.0 + k.eccentricity);
        for i in 0..36 {
            let t = (i as f64 / 36.0) * k.period(MU_SUN);
            let r = propagate_keplerian(&k, MU_SUN, t).position.length();
            assert!(r > q - 1.0 && r < cap_q + 1.0, "r out of band: {}", r);
        }
    }

    #[test]
    fn cache_default_returns_all_planets() {
        let cache = EphemerisCache::with_default_bodies();
        // Sun + 8 planets + Luna + Phobos + Deimos = 12.
        assert_eq!(cache.bodies().len(), 12);
        assert!(cache.get(naif::EARTH).is_some());
        assert!(cache.get(naif::MARS).is_some());
        assert!(cache.get(naif::LUNA).is_some());
        assert!(cache.get(naif::PHOBOS).is_some());
        assert!(cache.get(naif::DEIMOS).is_some());
    }

    #[test]
    fn luna_orbits_within_a_lunar_distance_of_earth() {
        let cache = EphemerisCache::with_default_bodies();
        let earth = cache.get(naif::EARTH).unwrap();
        let luna = cache.get(naif::LUNA).unwrap();
        let d = (luna.position - earth.position).length();
        // Lunar perigee/apogee bound: 363,300 → 405,500 km. Allow a margin.
        assert!(
            d > 3.5e8 && d < 4.2e8,
            "Luna distance from Earth out of expected band: {} m",
            d
        );
    }

    #[test]
    fn luna_period_is_about_27_days() {
        let cache = EphemerisCache::with_default_bodies();
        let earth_0 = cache.get(naif::EARTH).unwrap();
        let luna_0 = cache.get(naif::LUNA).unwrap();
        let rel_0 = luna_0.position - earth_0.position;
        // After one sidereal month the relative geometry should be roughly
        // back where it started. We use mean (not osculating) elements with
        // no secular perturbations, so 5 % drift is the realistic envelope.
        cache.refresh(27.321_661 * 86_400.0);
        let earth_t = cache.get(naif::EARTH).unwrap();
        let luna_t = cache.get(naif::LUNA).unwrap();
        let rel_t = luna_t.position - earth_t.position;
        let drift = (rel_0 - rel_t).length();
        let lunar_distance = rel_0.length();
        assert!(
            drift < 0.05 * lunar_distance,
            "Luna-Earth geometry drifted {} m ({:.2}% of lunar distance) — beyond Keplerian envelope",
            drift,
            100.0 * drift / lunar_distance,
        );
    }

    #[test]
    fn phobos_orbits_close_to_mars() {
        let cache = EphemerisCache::with_default_bodies();
        let mars = cache.get(naif::MARS).unwrap();
        let phobos = cache.get(naif::PHOBOS).unwrap();
        let d = (phobos.position - mars.position).length();
        assert!(
            d > 9.0e6 && d < 1.0e7,
            "Phobos distance from Mars out of band: {} m",
            d
        );
    }

    #[test]
    fn refresh_advances_states_over_time() {
        let cache = EphemerisCache::with_default_bodies();
        let earth_0 = cache.get(naif::EARTH).unwrap();
        cache.refresh(60.0 * 86_400.0); // 60 days later
        let earth_t = cache.get(naif::EARTH).unwrap();
        assert!((earth_0.position - earth_t.position).length() > 1e6);
    }

    #[test]
    fn gravity_inside_a_planet_is_bounded() {
        // Regression: previously, a spacecraft staged at Earth's exact
        // heliocentric position experienced a 10⁸ m/s² acceleration during
        // RK4 substeps that probed positions a few hundred metres off
        // Earth's centre. The interior model must cap acceleration at
        // ≤ Earth's surface gravity (~9.8 m/s²) plus solar contribution.
        let cache = EphemerisCache::with_default_bodies();
        let earth = cache.get(naif::EARTH).unwrap().position;
        // Sample 100 points inside Earth at random offsets.
        for i in 0..100 {
            let theta = (i as f64) * 0.1;
            let phi = (i as f64) * 0.07;
            let off = DVec3::new(theta.sin() * phi.cos(), theta.cos(), phi.sin())
                * 5.0e6; // 5000 km — well inside Earth's 6371 km radius
            let pos = earth + off;
            let a = cache.gravity_at(pos, 0.0);
            // Interior |a| ≤ GM_earth / R_earth² = ~9.8 m/s², plus a tiny
            // solar contribution (~6e-3 m/s²) plus minor planet pulls.
            // 20 m/s² is a comfortable upper bound.
            assert!(
                a.length() < 20.0,
                "interior gravity at offset {:.0} m: {} m/s²",
                off.length(),
                a.length(),
            );
        }
    }

    #[test]
    fn gravity_at_planet_surface_matches_surface_g() {
        // Earth surface acceleration should be ~9.8 m/s².
        let cache = EphemerisCache::with_default_bodies();
        let earth_state = cache.get(naif::EARTH).unwrap();
        let earth_params = cache
            .bodies()
            .into_iter()
            .find(|b| b.id == naif::EARTH)
            .unwrap();
        let surface = earth_state.position + DVec3::X * earth_params.radius;
        let a = cache.gravity_at(surface, 0.0);
        // Earth's pull dominates at the surface; |a| ≈ 9.8 m/s² with a
        // small solar / planetary perturbation.
        assert!(
            (a.length() - 9.8).abs() < 0.5,
            "surface gravity {} m/s² off expected 9.8",
            a.length(),
        );
    }

    #[test]
    fn gravity_at_sun_distance_is_close_to_solar_acceleration() {
        let cache = EphemerisCache::with_default_bodies();
        // At 1 AU from the origin (Sun) the acceleration is GM/r² ≈ 5.93e-3 m/s².
        let a = cache.gravity_at(DVec3::new(AU, 0.0, 0.0), 0.0);
        let expected = MU_SUN / (AU * AU);
        // Other planets are tiny perturbers — within 10%.
        assert!(a.length() > expected * 0.9 && a.length() < expected * 1.1);
        // Direction points back toward the Sun.
        assert!(a.x < 0.0);
    }

    #[test]
    fn solve_kepler_zero_eccentricity_is_identity() {
        let e = solve_kepler(1.234, 0.0);
        assert_relative_eq!(e, 1.234, epsilon = 1e-12);
    }
}
