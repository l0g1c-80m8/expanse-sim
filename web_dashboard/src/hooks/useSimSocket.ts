'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import type { ControlCommand, TelemetryFrame } from '@/lib/telemetry';

export type ConnectionStatus = 'idle' | 'connecting' | 'open' | 'closed' | 'error';

export interface UseSimSocketResult {
  frame: TelemetryFrame | null;
  status: ConnectionStatus;
  /** Wall-seconds since the last status transition — useful for UI hints. */
  statusElapsedSec: number;
  /** Number of consecutive failed connect attempts since the last `open`. */
  failedAttempts: number;
  send: (cmd: ControlCommand) => void;
  reconnect: () => void;
}

/** Hard cap on a single connect attempt before we declare it failed. Some
 * browsers (Firefox in particular) sit on a ws:// connect for the full TCP
 * timeout (~75 s) when there's no listener, which leaves the UI stuck.
 */
const CONNECT_TIMEOUT_MS = 4_000;

/** Backoff schedule (ms) between reconnect attempts — capped at 8 s. */
const BACKOFF_MS = [500, 1_000, 2_000, 4_000, 8_000];

/**
 * Hook that opens a WebSocket to the Rust sim_server, keeps the most recent
 * telemetry frame in state, and exposes a `send()` for control commands.
 *
 * Robustness contract:
 *   - Exactly one live socket at a time. Stale `onclose` handlers from
 *     superseded sockets are gated on a `socketEpoch` counter.
 *   - Connect attempts time out after `CONNECT_TIMEOUT_MS` so the UI never
 *     hangs on "connecting" when there's no listener.
 *   - Reconnect backoff is exponential up to 8 s.
 *   - Survives StrictMode double-mount.
 */
export function useSimSocket(url: string): UseSimSocketResult {
  const [frame, setFrame] = useState<TelemetryFrame | null>(null);
  const [status, setStatus] = useState<ConnectionStatus>('idle');
  const [statusEpoch, setStatusEpoch] = useState(0);
  const [failedAttempts, setFailedAttempts] = useState(0);
  const [statusElapsedSec, setStatusElapsedSec] = useState(0);

  const socketRef = useRef<WebSocket | null>(null);
  const socketEpoch = useRef(0);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const connectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const failedAttemptsRef = useRef(0);

  const transition = useCallback((next: ConnectionStatus) => {
    setStatus(next);
    setStatusEpoch((e) => e + 1);
  }, []);

  const clearTimers = useCallback(() => {
    if (reconnectTimer.current) {
      clearTimeout(reconnectTimer.current);
      reconnectTimer.current = null;
    }
    if (connectTimer.current) {
      clearTimeout(connectTimer.current);
      connectTimer.current = null;
    }
  }, []);

  const scheduleReconnect = useCallback(() => {
    if (reconnectTimer.current) return;
    const idx = Math.min(failedAttemptsRef.current, BACKOFF_MS.length - 1);
    const delay = BACKOFF_MS[idx];
    reconnectTimer.current = setTimeout(() => {
      reconnectTimer.current = null;
      // Re-enter the effect chain by triggering a state change; rather
      // than calling connect() directly (which races with React), we
      // bump the epoch to retrigger the URL effect.
      setStatusEpoch((e) => e + 1);
    }, delay);
  }, []);

  // Main connect effect — runs whenever the URL or the status epoch changes
  // (latter is bumped by scheduleReconnect to trigger a retry).
  useEffect(() => {
    if (typeof window === 'undefined') return;
    // Empty URL = disabled. Callers (useSimTransport) use this to suppress
    // the WS branch when WASM is active without yanking the hook order.
    if (!url) {
      setStatus('idle');
      return;
    }
    // Don't connect again while a socket is still open / connecting.
    if (
      socketRef.current &&
      (socketRef.current.readyState === WebSocket.OPEN ||
        socketRef.current.readyState === WebSocket.CONNECTING)
    ) {
      return;
    }

    const epoch = ++socketEpoch.current;
    let ws: WebSocket;
    try {
      transition('connecting');
      ws = new WebSocket(url);
    } catch {
      failedAttemptsRef.current += 1;
      setFailedAttempts(failedAttemptsRef.current);
      transition('error');
      scheduleReconnect();
      return;
    }
    socketRef.current = ws;

    // Hard timeout: if onopen doesn't fire within CONNECT_TIMEOUT_MS,
    // force-close and treat as a failed attempt.
    connectTimer.current = setTimeout(() => {
      if (socketEpoch.current !== epoch) return;
      if (ws.readyState === WebSocket.CONNECTING) {
        try { ws.close(); } catch {}
        // onclose will run and trigger reconnect.
      }
    }, CONNECT_TIMEOUT_MS);

    ws.onopen = () => {
      if (socketEpoch.current !== epoch) return;
      if (connectTimer.current) {
        clearTimeout(connectTimer.current);
        connectTimer.current = null;
      }
      failedAttemptsRef.current = 0;
      setFailedAttempts(0);
      transition('open');
    };
    ws.onmessage = (evt) => {
      if (socketEpoch.current !== epoch) return;
      try {
        const data: TelemetryFrame = JSON.parse(evt.data);
        setFrame(data);
      } catch {
        /* drop malformed frame */
      }
    };
    ws.onerror = () => {
      if (socketEpoch.current !== epoch) return;
      transition('error');
    };
    ws.onclose = () => {
      if (socketEpoch.current !== epoch) return;
      if (connectTimer.current) {
        clearTimeout(connectTimer.current);
        connectTimer.current = null;
      }
      socketRef.current = null;
      failedAttemptsRef.current += 1;
      setFailedAttempts(failedAttemptsRef.current);
      transition('closed');
      scheduleReconnect();
    };

    return () => {
      socketEpoch.current += 1; // invalidate any pending callbacks
      clearTimers();
      try {
        ws.close();
      } catch {}
      socketRef.current = null;
    };
  }, [url, statusEpoch, transition, scheduleReconnect, clearTimers]);

  // 1-Hz tick to drive the "stuck connecting?" UI hint.
  useEffect(() => {
    setStatusElapsedSec(0);
    const start = Date.now();
    const id = setInterval(() => {
      setStatusElapsedSec(Math.floor((Date.now() - start) / 1000));
    }, 500);
    return () => clearInterval(id);
  }, [status]);

  const send = useCallback((cmd: ControlCommand) => {
    const ws = socketRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(cmd));
    }
  }, []);

  const reconnect = useCallback(() => {
    clearTimers();
    if (socketRef.current) {
      try { socketRef.current.close(); } catch {}
      socketRef.current = null;
    }
    failedAttemptsRef.current = 0;
    setFailedAttempts(0);
    // Bump the epoch so the connect effect runs again immediately.
    setStatusEpoch((e) => e + 1);
  }, [clearTimers]);

  return { frame, status, statusElapsedSec, failedAttempts, send, reconnect };
}
