//! WebSocket endpoint and one-shot snapshot endpoint.

use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
    Json,
};
use futures_util::{sink::SinkExt, stream::StreamExt};

use crate::sim_runner::{AppState, ControlCommand};

pub async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

async fn handle_socket(socket: WebSocket, state: AppState) {
    let (mut sender, mut receiver) = socket.split();
    let mut rx = state.telemetry_rx.subscribe();

    // Push the latest snapshot immediately so the dashboard renders before
    // the first broadcast frame arrives.
    let initial = state.latest.lock().clone();
    if let Some(initial) = initial {
        if let Ok(text) = serde_json::to_string(&initial) {
            let _ = sender.send(Message::Text(text)).await;
        }
    }

    let send_task = tokio::spawn(async move {
        while let Ok(frame) = rx.recv().await {
            let payload = match serde_json::to_string(&frame) {
                Ok(s) => s,
                Err(_) => continue,
            };
            if sender.send(Message::Text(payload)).await.is_err() {
                break;
            }
        }
    });

    let cmd_tx = state.command_tx.clone();
    let recv_task = tokio::spawn(async move {
        while let Some(Ok(msg)) = receiver.next().await {
            match msg {
                Message::Text(text) => {
                    if let Ok(cmd) = serde_json::from_str::<ControlCommand>(&text) {
                        let _ = cmd_tx.send(cmd).await;
                    } else {
                        tracing::debug!(text = %text, "unrecognised command");
                    }
                }
                Message::Close(_) => break,
                _ => {}
            }
        }
    });

    let _ = tokio::join!(send_task, recv_task);
}

/// One-shot HTTP snapshot — convenient for the dashboard's first render
/// before the WebSocket connects, and for curl-based smoke tests.
pub async fn snapshot_handler(State(state): State<AppState>) -> impl IntoResponse {
    Json(state.latest.lock().clone())
}
