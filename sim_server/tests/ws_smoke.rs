//! End-to-end smoke test: launches the sim_server binary, opens a WebSocket,
//! checks telemetry arrives and that control commands flow through.
//!
//! Ignored by default so `cargo test` on machines without a built binary
//! doesn't fail. Run with: `cargo test -p sim_server --test ws_smoke -- --ignored`.

use std::process::{Child, Command, Stdio};
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::time::{sleep, timeout};
use tokio_tungstenite::tungstenite::Message;

struct ServerProc {
    child: Child,
}
impl Drop for ServerProc {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn launch_server(port: u16) -> ServerProc {
    let bin = env!("CARGO_BIN_EXE_sim_server");
    let child = Command::new(bin)
        .args([
            "--bind",
            &format!("127.0.0.1:{port}"),
            "--dt",
            "0.05",
            "--warp",
            "60",
            "--telemetry-stride",
            "5",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("failed to launch sim_server");
    ServerProc { child }
}

async fn wait_for_health(port: u16) {
    for _ in 0..40 {
        if reqwest::get(format!("http://127.0.0.1:{port}/health"))
            .await
            .and_then(|r| r.error_for_status())
            .is_ok()
        {
            return;
        }
        sleep(Duration::from_millis(100)).await;
    }
    panic!("server never became healthy on port {port}");
}

#[tokio::test]
#[ignore]
async fn websocket_streams_telemetry_and_accepts_commands() {
    let port = 18181;
    let _server = launch_server(port);
    wait_for_health(port).await;

    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}/ws"))
        .await
        .expect("ws connect");

    // Pull at least 5 frames within 3 seconds.
    let mut frames = 0;
    let mut last_warp = 0.0_f64;
    while frames < 5 {
        let msg = timeout(Duration::from_secs(3), ws.next())
            .await
            .expect("ws frame timeout")
            .expect("ws stream ended")
            .expect("ws error");
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            assert!(v["bodies"].as_array().unwrap().len() >= 8);
            last_warp = v["warp"].as_f64().unwrap();
            frames += 1;
        }
    }
    assert!((last_warp - 60.0).abs() < 1e-6);

    // Push a set_warp command and verify the next few frames reflect it.
    ws.send(Message::Text(
        r#"{"type":"set_warp","warp":250}"#.to_string(),
    ))
    .await
    .unwrap();

    let mut saw_new_warp = false;
    for _ in 0..40 {
        let msg = timeout(Duration::from_secs(2), ws.next()).await.unwrap().unwrap().unwrap();
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            if (v["warp"].as_f64().unwrap() - 250.0).abs() < 1e-6 {
                saw_new_warp = true;
                break;
            }
        }
    }
    assert!(saw_new_warp, "warp command never reflected in telemetry");

    // Push a thrust and confirm propellant starts dropping (over a window
    // long enough to ensure tick budget retired some sim time).
    ws.send(Message::Text(
        r#"{"type":"set_thrust","thrust":[1.0e7, 0.0, 0.0]}"#.to_string(),
    ))
    .await
    .unwrap();

    let mut first_propellant: Option<f64> = None;
    let mut later_propellant: Option<f64> = None;
    for _ in 0..30 {
        let msg = timeout(Duration::from_secs(2), ws.next()).await.unwrap().unwrap().unwrap();
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            let p = v["spacecraft"]["propellant_mass"].as_f64();
            if first_propellant.is_none() {
                first_propellant = p;
            }
            later_propellant = p;
        }
    }
    let p0 = first_propellant.expect("had first frame");
    let p1 = later_propellant.expect("had later frame");
    assert!(
        p1 < p0,
        "propellant should burn down once thrust is commanded (p0={p0}, p1={p1})"
    );
}
