//! WebAssembly wrapper around `sim_core` for the browser dashboard.
//!
//! The shape of this crate is intentionally minimal: it exposes a single
//! [`SimWasm`] handle that owns an [`ExpanseSim`] + the spacecraft entity,
//! and forwards everything to `sim_core::protocol::{snapshot, apply_command}`
//! so the wire format stays identical to the WebSocket server.
//!
//! Build:
//! ```bash
//! wasm-pack build sim_wasm \
//!     --release \
//!     --target web \
//!     --out-dir ../web_dashboard/public/wasm
//! ```
//!
//! Use from JS:
//! ```js
//! import init, { SimWasm } from '/wasm/sim_wasm.js';
//! await init();
//! const sim = new SimWasm();
//! sim.tick();
//! const frameJson = sim.snapshot_json(0, 0.0);
//! sim.apply_command_json('{"type":"set_warp","warp":1000}');
//! ```

use sim_core::prelude::Entity;
use sim_core::protocol::{
    apply_command, build_default_sim, snapshot, spawn_default_spacecraft, ControlCommand,
};
use sim_core::{ExpanseSim, SimConfig};
use wasm_bindgen::prelude::*;

/// Installable once per worker so Rust panics surface in the JS console
/// with readable backtraces instead of `unreachable executed`.
#[wasm_bindgen(start)]
pub fn _wasm_start() {
    console_error_panic_hook::set_once();
}

/// Browser-facing simulator handle. Lives on the JS heap (boxed), owns
/// the bevy_ecs World and the spacecraft Entity, and serialises every
/// snapshot to JSON so the existing dashboard TypeScript types apply
/// unchanged.
#[wasm_bindgen]
pub struct SimWasm {
    sim: ExpanseSim,
    spacecraft: Entity,
    sim_cfg: SimConfig,
}

#[wasm_bindgen]
impl SimWasm {
    /// Construct a new simulator with the default Earth → Mars mission.
    #[wasm_bindgen(constructor)]
    pub fn new(dt: f64, initial_warp: f64, apply_gravity: bool) -> SimWasm {
        let sim_cfg = SimConfig {
            dt,
            warp: initial_warp,
            apply_gravity,
            load_default_bodies: true,
            ..Default::default()
        };
        let mut sim = build_default_sim(sim_cfg);
        let spacecraft = spawn_default_spacecraft(&mut sim);
        SimWasm { sim, spacecraft, sim_cfg }
    }

    /// Convenience constructor matching the dashboard's default knobs.
    ///
    /// dt = 0.5 s is the chosen baseline: small enough that RK4 still
    /// resolves a parking-orbit revolution in ~30 substeps (good visual
    /// continuity), large enough that high-warp playback isn't bottlenecked
    /// by per-tick fixed costs (ECS schedule, gravity loop). At dt=0.05 the
    /// wall-time-per-sim-day ceiling on a laptop was ~minute-scale, which
    /// made interplanetary transits feel "stuck" even though the autopilot
    /// was thrusting correctly.
    #[wasm_bindgen(js_name = newDefault)]
    pub fn new_default() -> SimWasm {
        Self::new(0.5, 60.0, true)
    }

    /// Advance the simulator by exactly one fixed `dt` tick.
    pub fn tick(&mut self) {
        self.sim.tick();
    }

    /// Advance the simulator by `n` ticks. Cheaper than calling `tick()` n
    /// times across the JS / WASM boundary.
    #[wasm_bindgen(js_name = tickN)]
    pub fn tick_n(&mut self, n: u32) {
        for _ in 0..n {
            self.sim.tick();
        }
    }

    /// Build a JSON-encoded `TelemetryFrame`. `tick` and `effective_warp`
    /// are provided by the caller (the worker tracks them) since they are
    /// driver-side bookkeeping, not part of the simulator's own state.
    #[wasm_bindgen(js_name = snapshotJson)]
    pub fn snapshot_json(&self, tick: u64, effective_warp: f64) -> String {
        let frame = snapshot(&self.sim, self.spacecraft, tick, effective_warp);
        serde_json::to_string(&frame).unwrap_or_else(|_| "{}".into())
    }

    /// Apply a single command. Returns `Ok(())` on success, or an `Error`
    /// JS value with a description on parse failure.
    #[wasm_bindgen(js_name = applyCommandJson)]
    pub fn apply_command_json(&mut self, cmd_json: &str) -> Result<(), JsValue> {
        let cmd: ControlCommand = serde_json::from_str(cmd_json)
            .map_err(|e| JsValue::from_str(&format!("invalid command JSON: {e}")))?;
        apply_command(&mut self.sim, &mut self.spacecraft, cmd, self.sim_cfg);
        Ok(())
    }

    /// Current sim time in seconds (clock-decoupled).
    #[wasm_bindgen(js_name = simTime)]
    pub fn sim_time(&self) -> f64 {
        self.sim.sim_time()
    }

    /// Current dt (integration step, seconds).
    pub fn dt(&self) -> f64 {
        self.sim.world.resource::<sim_core::SimTime>().dt
    }

    /// Current warp factor.
    pub fn warp(&self) -> f64 {
        self.sim.world.resource::<sim_core::SimClock>().warp
    }

    /// Whether the sim is paused.
    pub fn paused(&self) -> bool {
        self.sim.world.resource::<sim_core::SimClock>().paused
    }
}
