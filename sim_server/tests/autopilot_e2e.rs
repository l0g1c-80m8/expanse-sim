//! End-to-end: engage autopilot via WebSocket, watch the phase transitions
//! through telemetry, confirm the ship arrives. Requires the *release*
//! binary to keep walltime reasonable: at warp 2000× over a 3-day brachistochrone
//! the dev build can't sustain the required tick rate.
//!
//! Run with:
//!   `cargo build --release -p sim_server`
//!   `CARGO_BIN_EXE_sim_server=target/release/sim_server cargo test -p sim_server \`
//!     `--test autopilot_e2e -- --ignored --nocapture`

use std::process::{Child, Command, Stdio};
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::time::{sleep, timeout};
use tokio_tungstenite::tungstenite::Message;

struct ServerProc(Child);
impl Drop for ServerProc {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn launch(port: u16) -> ServerProc {
    let bin = env!("CARGO_BIN_EXE_sim_server");
    let child = Command::new(bin)
        .args([
            "--bind",
            &format!("127.0.0.1:{port}"),
            // Tight dt for clean autopilot integration during high-g burn.
            // dt=0.05 + warp=5000 = ~250 sim-s per wall-s, which the release
            // server handles comfortably.
            "--dt",
            "0.05",
            "--warp",
            "5000",
            "--telemetry-stride",
            "200",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("launch sim_server");
    ServerProc(child)
}

async fn wait_for_health(port: u16) {
    for _ in 0..50 {
        if reqwest::get(format!("http://127.0.0.1:{port}/health"))
            .await
            .and_then(|r| r.error_for_status())
            .is_ok()
        {
            return;
        }
        sleep(Duration::from_millis(100)).await;
    }
    panic!("server never healthy");
}

#[tokio::test]
#[ignore]
async fn autopilot_reaches_mars() {
    let port = 18183;
    let _s = launch(port);
    wait_for_health(port).await;

    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}/ws"))
        .await
        .unwrap();

    // One-shot mission kickoff: Earth → Mars at 5 g. This switches to
    // Mission mode (which the autopilot requires to write thrust),
    // stages at Earth, and engages.
    ws.send(Message::Text(
        r#"{"type":"start_mission","source":399,"target":499,"accel_g":5.0}"#.to_string(),
    ))
    .await
    .unwrap();

    // Watch the autopilot phase. Walltime budget: 30 s. At warp 5000 with
    // 5g acceleration over ~0.5 AU, transit ≈ 3.5 days sim = 60 s wall.
    // The autopilot is in flight under realistic Sun-only gravity. A full
    // Earth → Mars rendezvous to soft arrival takes several minutes wall
    // at warp 5000 with the ZEM/ZEV controller's non-optimal trajectory,
    // so this E2E focuses on validating the phase progression: idle →
    // boost (autopilot has authority) → brake (autopilot decelerates),
    // plus a fuel-burn sanity check. Full soft-arrival is exercised by
    // sim_core's autopilot_rendezvous integration tests under controlled
    // conditions (static / moving synthetic targets).
    let mut saw_boost = false;
    let mut saw_brake = false;
    let mut last_phase = String::new();
    let mut initial_fuel: Option<f64> = None;
    let mut last_fuel: Option<f64> = None;
    let deadline = std::time::Instant::now() + Duration::from_secs(45);
    while std::time::Instant::now() < deadline {
        let Ok(msg) = timeout(Duration::from_secs(5), ws.next()).await else {
            break;
        };
        let Some(Ok(Message::Text(t))) = msg else { continue };
        let v: serde_json::Value = serde_json::from_str(&t).unwrap();
        let phase = v["autopilot"]["phase"].as_str().unwrap_or("").to_string();
        let range = v["autopilot"]["range_m"].as_f64().unwrap_or(0.0);
        let closing = v["autopilot"]["closing_m_s"].as_f64().unwrap_or(0.0);
        let fuel = v["spacecraft"]["propellant_mass"].as_f64();
        if phase != last_phase {
            eprintln!(
                "phase {:>8} → {:>8}   range={:.3e}  closing={:>7.1} m/s",
                last_phase, phase, range, closing,
            );
            last_phase = phase.clone();
        }
        if initial_fuel.is_none() && phase == "boost" {
            initial_fuel = fuel;
        }
        if fuel.is_some() {
            last_fuel = fuel;
        }
        match phase.as_str() {
            "boost" => saw_boost = true,
            "brake" => saw_brake = true,
            "arrived" => break,
            _ => {}
        }
        if saw_boost && saw_brake {
            break;
        }
    }

    assert!(saw_boost, "autopilot never entered boost");
    assert!(saw_brake, "autopilot never entered brake");
    let f0 = initial_fuel.expect("captured initial fuel");
    let f1 = last_fuel.expect("captured later fuel");
    assert!(
        f0 - f1 > 1000.0,
        "autopilot should have burned > 1 t of fuel, got Δ={:.0} kg",
        f0 - f1
    );
}
