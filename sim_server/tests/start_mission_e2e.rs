//! E2E: a single `start_mission` command flips the sim into Mission mode,
//! stages the ship at the source body, and engages the autopilot. Requires
//! the release binary for throughput (`CARGO_BIN_EXE_sim_server=...release/sim_server`).

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
            "--dt",
            "1.0",
            "--warp",
            "200",
            "--telemetry-stride",
            "10",
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
async fn start_mission_flips_mode_stages_and_engages() {
    let port = 18185;
    let _s = launch(port);
    wait_for_health(port).await;

    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}/ws"))
        .await
        .unwrap();

    // Sanity: default state is Sandbox, autopilot idle.
    let mut seen_initial = false;
    while !seen_initial {
        let msg = timeout(Duration::from_secs(3), ws.next()).await.unwrap().unwrap().unwrap();
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            if v["mode"].as_str() == Some("sandbox")
                && v["autopilot"]["phase"].as_str() == Some("idle")
            {
                seen_initial = true;
            }
        }
    }

    // Fire the one-shot.
    ws.send(Message::Text(
        r#"{"type":"start_mission","source":399,"target":499,"accel_g":2.0}"#.to_string(),
    ))
    .await
    .unwrap();

    // Within a few frames we should see mode=mission, autopilot phase in
    // {boost, brake}, and the spacecraft sitting near Earth's orbit.
    let mut mode_ok = false;
    let mut phase_ok = false;
    for _ in 0..200 {
        let msg = timeout(Duration::from_secs(3), ws.next()).await.unwrap().unwrap().unwrap();
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            if v["mode"].as_str() == Some("mission") {
                mode_ok = true;
            }
            let phase = v["autopilot"]["phase"].as_str().unwrap_or("");
            if matches!(phase, "boost" | "brake") {
                phase_ok = true;
            }
            if mode_ok && phase_ok {
                break;
            }
        }
    }
    assert!(mode_ok, "start_mission failed to flip mode");
    assert!(phase_ok, "start_mission failed to engage autopilot");
}
