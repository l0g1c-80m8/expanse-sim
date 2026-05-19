//! Simulation driver that runs `sim_core::ExpanseSim` in a dedicated
//! tokio task. The protocol layer (TelemetryFrame, ControlCommand,
//! snapshot, apply_command) lives in `sim_core::protocol` so the same
//! wire format is used by the WASM frontend.
//!
//! Responsibilities of this module:
//! * own the tokio task that ticks the simulator
//! * track wall-clock vs sim-clock budget for time-warp
//! * fan out telemetry to all connected WebSocket clients via a broadcast
//!   channel
//! * drain commands from a mpsc channel and forward them to
//!   `sim_core::protocol::apply_command`

use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use sim_core::protocol::{
    apply_command, build_default_sim, snapshot, spawn_default_spacecraft, ControlCommand,
    TelemetryFrame,
};
use sim_core::{SimClock, SimConfig, SimTime};
use tokio::sync::{broadcast, mpsc};

/// Operator-facing settings provided at startup by `sim_server`'s CLI.
/// The dynamics knobs (dt, warp, gravity, default bodies) are projected
/// into `SimConfig` for the protocol layer's `build_default_sim`;
/// `telemetry_stride` and `loop_interval` are runner-specific.
#[derive(Debug, Clone)]
pub struct SimSettings {
    pub dt: f64,
    pub initial_warp: f64,
    pub apply_gravity: bool,
    pub telemetry_stride: u32,
    pub loop_interval: Duration,
}

impl SimSettings {
    fn sim_config(&self) -> SimConfig {
        SimConfig {
            dt: self.dt,
            warp: self.initial_warp,
            apply_gravity: self.apply_gravity,
            load_default_bodies: true,
            ..Default::default()
        }
    }
}

/// Shared handle injected into Axum routes.
#[derive(Clone)]
pub struct AppState {
    pub telemetry_rx: broadcast::Sender<TelemetryFrame>,
    pub command_tx: mpsc::Sender<ControlCommand>,
    pub latest: Arc<Mutex<Option<TelemetryFrame>>>,
}

pub fn spawn_simulation(settings: SimSettings) -> AppState {
    let (telemetry_tx, _) = broadcast::channel::<TelemetryFrame>(64);
    let (command_tx, mut command_rx) = mpsc::channel::<ControlCommand>(128);
    let latest = Arc::new(Mutex::new(None));

    let telemetry_pub = telemetry_tx.clone();
    let latest_pub = latest.clone();

    tokio::task::spawn_blocking(move || {
        let sim_cfg = settings.sim_config();
        let mut sim = build_default_sim(sim_cfg);
        let mut spacecraft_id = spawn_default_spacecraft(&mut sim);
        let mut tick: u64 = 0;
        let mut budget: f64 = 0.0;
        let mut last = Instant::now();

        // Effective-warp tracking — a 1-second rolling window over advanced
        // sim seconds, so the dashboard can show real throughput vs requested
        // warp and the user can spot the throughput cap kicking in.
        let mut last_warp_sample = Instant::now();
        let mut sim_seconds_in_window: f64 = 0.0;
        let mut effective_warp: f64 = 0.0;

        loop {
            let now = Instant::now();
            let wall = (now - last).as_secs_f64();
            last = now;

            while let Ok(cmd) = command_rx.try_recv() {
                apply_command(&mut sim, &mut spacecraft_id, cmd, sim_cfg);
            }

            let clock = *sim.world.resource::<SimClock>();
            let dt = sim.world.resource::<SimTime>().dt;
            if !clock.paused {
                budget += wall * clock.warp;
            }

            // Cap the per-iteration tick count so a long stall doesn't trigger
            // a runaway catch-up burst. 50k @ 10 ms loop ≈ 5 M ticks/sec,
            // enough to deliver ~250 000× warp at dt = 0.05.
            let mut ticks_to_run = (budget / dt).floor() as u32;
            ticks_to_run = ticks_to_run.min(50_000);
            for _ in 0..ticks_to_run {
                sim.tick();
                tick += 1;
                sim_seconds_in_window += dt;
                if tick % settings.telemetry_stride as u64 == 0 {
                    let frame = snapshot(&sim, spacecraft_id, tick, effective_warp);
                    *latest_pub.lock() = Some(frame.clone());
                    let _ = telemetry_pub.send(frame);
                }
            }
            budget -= ticks_to_run as f64 * dt;

            // Bleed accumulated budget if the cap was hit — the operator
            // wants "best we can do" not "infinitely behind".
            if budget > dt * 50_000.0 {
                budget = dt * 50_000.0;
            }

            // Sample effective warp once per wall-second.
            let elapsed = last_warp_sample.elapsed().as_secs_f64();
            if elapsed >= 1.0 {
                effective_warp = sim_seconds_in_window / elapsed;
                sim_seconds_in_window = 0.0;
                last_warp_sample = Instant::now();
            }

            if ticks_to_run == 0 {
                let frame = snapshot(&sim, spacecraft_id, tick, effective_warp);
                *latest_pub.lock() = Some(frame.clone());
                let _ = telemetry_pub.send(frame);
            }

            // Adaptive pacing: only sleep when there's no pending budget. If
            // we have more work, yield to the scheduler and come back
            // immediately so high-warp throughput isn't capped by the loop
            // interval.
            if clock.paused || budget < dt {
                std::thread::sleep(settings.loop_interval);
            } else {
                std::thread::yield_now();
            }
        }
    });

    AppState {
        telemetry_rx: telemetry_tx,
        command_tx,
        latest,
    }
}
