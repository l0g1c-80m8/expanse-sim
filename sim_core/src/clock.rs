//! Simulation clock — decouples sim-time from wall-clock and provides
//! deterministic time-warp.
//!
//! Determinism contract: ROS 2 control nodes connected via the lockstep
//! bridge see a constant `dt` every tick regardless of the wall-clock pacing
//! used by the driving thread. Time-warp only changes how often ticks happen
//! in real time, never the value of `dt` itself. This is what lets a PID/MPC
//! tuned at 1× still behave identically at 1000×.

use bevy_ecs::prelude::*;
use serde::{Deserialize, Serialize};

/// The simulator's authoritative time state.
#[derive(Resource, Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SimTime {
    /// Elapsed simulation time (seconds since epoch start).
    pub time: f64,
    /// Fixed integration step (seconds). Keep this constant for determinism.
    pub dt: f64,
}

impl Default for SimTime {
    fn default() -> Self {
        Self { time: 0.0, dt: 0.01 }
    }
}

impl SimTime {
    pub fn new(dt: f64) -> Self {
        assert!(dt > 0.0, "integration step must be positive");
        Self { time: 0.0, dt }
    }

    pub fn advance(&mut self) {
        self.time += self.dt;
    }
}

/// Wall-clock to sim-clock conversion policy.
///
/// `warp` multiplies wall-clock seconds to produce sim-clock seconds — at
/// warp 10, a wall-second advances the simulation by 10 sim-seconds (so the
/// driver must execute 10× as many fixed ticks per wall-second).
///
/// The clock never affects `SimTime.dt`; it only governs how many ticks the
/// driver should retire per wall-clock interval.
#[derive(Resource, Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SimClock {
    pub warp: f64,
    pub paused: bool,
    /// Sim-time epoch (J2000 seconds) at which `SimTime.time = 0`. SPICE-style
    /// ephemeris queries use `epoch_j2000 + sim_time.time`.
    pub epoch_j2000: f64,
}

impl Default for SimClock {
    fn default() -> Self {
        Self {
            warp: 1.0,
            paused: false,
            epoch_j2000: 0.0,
        }
    }
}

impl SimClock {
    /// Number of fixed ticks to retire for a given wall-clock interval.
    ///
    /// Returned as a float so callers can carry remainder forward — accumulating
    /// the fractional tick budget avoids drift across long runs.
    pub fn ticks_for_wall_seconds(&self, wall_seconds: f64, dt: f64) -> f64 {
        if self.paused {
            return 0.0;
        }
        (wall_seconds * self.warp) / dt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_warp_is_realtime() {
        let clock = SimClock::default();
        // 1 wall-second @ 1× warp with dt=0.01 → 100 ticks
        assert!((clock.ticks_for_wall_seconds(1.0, 0.01) - 100.0).abs() < 1e-9);
    }

    #[test]
    fn warp_scales_tick_budget_linearly() {
        let mut clock = SimClock::default();
        clock.warp = 1000.0;
        // 1 wall-second @ 1000× → 100_000 ticks
        assert!((clock.ticks_for_wall_seconds(1.0, 0.01) - 100_000.0).abs() < 1e-6);
    }

    #[test]
    fn paused_clock_yields_zero_ticks() {
        let mut clock = SimClock::default();
        clock.paused = true;
        assert_eq!(clock.ticks_for_wall_seconds(1.0, 0.01), 0.0);
    }

    #[test]
    fn dt_is_invariant_under_warp() {
        // Determinism guarantee: same dt regardless of warp.
        let mut t = SimTime::new(0.005);
        let mut clock = SimClock::default();
        clock.warp = 250.0;
        let dt_before = t.dt;
        // Advance many ticks
        for _ in 0..1000 {
            t.advance();
        }
        assert_eq!(t.dt, dt_before);
        assert!((t.time - 5.0).abs() < 1e-9);
    }
}
