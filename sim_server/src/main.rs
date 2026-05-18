//! Web/IPC bridge for `sim_core`.
//!
//! The simulator runs in a dedicated tokio task at a fixed integration step,
//! independent from any HTTP request. Telemetry is broadcast to all connected
//! WebSocket clients every tick (with a downsample factor at high warp to
//! keep the wire from saturating). Each client can also push commands —
//! play/pause, set warp, change thrust — which are forwarded into the
//! simulation via a bounded MPSC channel.
//!
//! Threading model:
//!
//! ```text
//!  ┌──────────┐  Command  ┌────────────┐  Telemetry  ┌────────────┐
//!  │ WS client├──────────►│ sim driver ├────────────►│ broadcast  │
//!  └──────────┘   (mpsc)  └────────────┘   (tokio    │  channel   │
//!                                          broadcast)└────────────┘
//! ```

mod sim_runner;
mod thrust_controller;
mod ws;

use std::net::SocketAddr;

use axum::{routing::get, Router};
use clap::Parser;
use tower_http::cors::{Any, CorsLayer};

use crate::sim_runner::{spawn_simulation, AppState, SimSettings};

#[derive(Parser, Debug, Clone)]
#[command(name = "sim_server")]
#[command(about = "Expanse-Sim web bridge: serves telemetry over WebSocket")]
struct Cli {
    /// Address to bind the HTTP / WS server on.
    #[arg(long, default_value = "127.0.0.1:8080")]
    bind: SocketAddr,

    /// Fixed integration step (seconds).
    #[arg(long, default_value_t = 0.05)]
    dt: f64,

    /// Initial time-warp multiplier.
    #[arg(long, default_value_t = 60.0)]
    warp: f64,

    /// Disable gravity (useful for debugging propulsion in isolation).
    #[arg(long, default_value_t = false)]
    no_gravity: bool,

    /// How many simulation ticks per telemetry broadcast frame. Larger values
    /// reduce wire load at very high warp (e.g. `--telemetry-stride 50` at
    /// warp 1000× sends one telemetry frame per 2.5 sim-seconds).
    #[arg(long, default_value_t = 1)]
    telemetry_stride: u32,

    /// Wall-clock pacing budget per loop iteration (microseconds). Lower values
    /// keep the sim more responsive but burn more CPU.
    #[arg(long, default_value_t = 10_000)]
    loop_interval_us: u64,
}

#[tokio::main]
async fn main() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,sim_server=debug".into()),
        )
        .try_init();

    let cli = Cli::parse();
    tracing::info!(?cli, "starting sim_server");

    let settings = SimSettings {
        dt: cli.dt,
        initial_warp: cli.warp,
        apply_gravity: !cli.no_gravity,
        telemetry_stride: cli.telemetry_stride.max(1),
        loop_interval: std::time::Duration::from_micros(cli.loop_interval_us),
    };

    let state = spawn_simulation(settings);

    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    let app = Router::new()
        .route("/", get(|| async { "expanse-sim server — connect to /ws" }))
        .route("/health", get(|| async { "ok" }))
        .route("/snapshot", get(ws::snapshot_handler))
        .route("/ws", get(ws::ws_handler))
        .with_state(state)
        .layer(cors);

    tracing::info!(addr = %cli.bind, "binding HTTP/WS server");
    let listener = tokio::net::TcpListener::bind(cli.bind)
        .await
        .expect("failed to bind");
    axum::serve(listener, app)
        .await
        .expect("axum server crashed");
}

#[allow(dead_code)]
fn _smoke(_s: AppState) {}
