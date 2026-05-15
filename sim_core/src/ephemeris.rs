use bevy_ecs::prelude::*;
use glam::DVec3;
use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

/// Represents a celestial body's state at a specific point in time.
#[derive(Debug, Clone, Copy)]
pub struct BodyState {
    pub position: DVec3, // km
    pub velocity: DVec3, // km/s
}

/// A double-buffered cache that stores interpolated states ahead of `sim_time`.
/// This prevents O(N) SPICE C-library bottleneck during the hot physics loop.
#[derive(Resource, Clone)]
pub struct EphemerisCache {
    /// Maps a NAIF body ID to a queue or spline of pre-computed states.
    /// For simplicity in Phase 2, we just store a hashmap of current target states.
    /// A true spline would require `(t0, t1, p0, p1, v0, v1)`.
    pub current_states: Arc<RwLock<HashMap<i32, BodyState>>>,
}

impl Default for EphemerisCache {
    fn default() -> Self {
        Self {
            current_states: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

/// Initializes the NAIF SPICE subsystem and loads essential kernels.
pub fn initialize_spice(kernel_dir: &str) {
    let path = Path::new(kernel_dir);
    if !path.exists() {
        println!("WARNING: SPICE kernel directory '{}' not found. Ephemeris will be zeroed.", kernel_dir);
        return;
    }

    // Ideally, we load a meta-kernel here. 
    // spice::furnsh("kernels/meta.tm");
    // For now, we search for basic kernels like .bsp and .tls
    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            let p = entry.path();
            if let Some(ext) = p.extension() {
                let ext_str = ext.to_string_lossy();
                if ext_str == "bsp" || ext_str == "tls" || ext_str == "tpc" {
                    if let Some(p_str) = p.to_str() {
                        spice::furnsh(p_str);
                        println!("Loaded SPICE kernel: {}", p_str);
                    }
                }
            }
        }
    }
}

/// Spawns a background worker thread that continually queries SPICE and updates the cache.
pub fn spawn_ephemeris_worker(cache: EphemerisCache, target_bodies: Vec<i32>) {
    thread::spawn(move || {
        loop {
            // In a real system, this would query SPICE based on `sim_time` + lookahead.
            // spice::spkpos("TARGET", et, "J2000", "NONE", "OBSERVER");

            let mut states = cache.current_states.write().unwrap();
            for &body in &target_bodies {
                // Dummy data until actual ET is integrated with `SimTime`.
                states.insert(body, BodyState {
                    position: DVec3::ZERO,
                    velocity: DVec3::ZERO,
                });
            }
            drop(states); // release lock

            // Sleep to avoid thrashing CPU. 
            // The worker only needs to compute occasionally and fill the spline buffer.
            thread::sleep(Duration::from_millis(100));
        }
    });
}
