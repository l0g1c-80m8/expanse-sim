//! Verifies the new operator-thrust path end-to-end: set a thrust mode +
//! magnitude over the WebSocket, watch the propellant burn down with the
//! ship pointing at the chosen direction.

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

fn launch_server(port: u16) -> ServerProc {
    let bin = env!("CARGO_BIN_EXE_sim_server");
    let child = Command::new(bin)
        .args([
            "--bind",
            &format!("127.0.0.1:{port}"),
            "--dt",
            "0.05",
            "--warp",
            "1000",
            "--telemetry-stride",
            "5",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("failed to launch sim_server");
    ServerProc(child)
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
    panic!("server never healthy");
}

#[tokio::test]
#[ignore]
async fn thrust_mode_prograde_burns_propellant() {
    let port = 18182;
    let _s = launch_server(port);
    wait_for_health(port).await;

    let (mut ws, _) =
        tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{port}/ws"))
            .await
            .unwrap();

    // Mission: Earth → Mars (default), set thrust mode = toward_target,
    // 80 % magnitude.
    ws.send(Message::Text(
        r#"{"type":"set_thrust_mode","mode":"toward_target"}"#.to_string(),
    ))
    .await
    .unwrap();
    ws.send(Message::Text(
        r#"{"type":"set_thrust_magnitude","magnitude":4.0e7}"#.to_string(),
    ))
    .await
    .unwrap();

    // Snapshot propellant before and after a couple of seconds of wall time.
    let mut first: Option<f64> = None;
    let mut mode_seen: Option<String> = None;
    let mut later: Option<f64> = None;
    for _ in 0..120 {
        let msg = timeout(Duration::from_secs(3), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            mode_seen = v["thrust_controller"]["mode"].as_str().map(String::from);
            let p = v["spacecraft"]["propellant_mass"].as_f64();
            if first.is_none() {
                first = p;
            }
            later = p;
        }
    }
    let p0 = first.unwrap();
    let p1 = later.unwrap();
    assert_eq!(mode_seen.as_deref(), Some("toward_target"));
    assert!(p1 < p0, "propellant should drop with thrust on (p0={p0}, p1={p1})");

    // Cut and confirm propellant stops dropping.
    ws.send(Message::Text(
        r#"{"type":"set_thrust_mode","mode":"off"}"#.to_string(),
    ))
    .await
    .unwrap();
    ws.send(Message::Text(
        r#"{"type":"set_thrust_magnitude","magnitude":0.0}"#.to_string(),
    ))
    .await
    .unwrap();

    // Wait until we see a frame with mode=off — earlier frames may still be
    // draining the broadcast queue from before the cut was applied.
    let mut first_after_cut: Option<f64> = None;
    while first_after_cut.is_none() {
        let msg = timeout(Duration::from_secs(3), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            if v["thrust_controller"]["mode"].as_str() == Some("off") {
                first_after_cut = v["spacecraft"]["propellant_mass"].as_f64();
            }
        }
    }
    // Read many more frames; pick the last one. Propellant should be
    // essentially flat across that interval.
    let mut last_after_cut: Option<f64> = None;
    for _ in 0..100 {
        let msg = timeout(Duration::from_secs(3), ws.next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        if let Message::Text(t) = msg {
            let v: serde_json::Value = serde_json::from_str(&t).unwrap();
            last_after_cut = v["spacecraft"]["propellant_mass"].as_f64();
        }
    }
    let after = first_after_cut.unwrap();
    let final_ = last_after_cut.unwrap();
    let drop_after_cut = (after - final_) / after.max(1.0);
    assert!(
        drop_after_cut.abs() < 0.005,
        "propellant should hold steady after cut: {} → {} ({}%)",
        after,
        final_,
        drop_after_cut * 100.0,
    );
}
