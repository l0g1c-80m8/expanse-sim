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

/**
 * Hard cap on a single connect attempt before we declare it failed. Some
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
 * Single effect keyed on `url`. All retry / timeout / socket-lifecycle
 * state lives in the effect's local closure so React state never appears
 * in the dep array — that was the source of an infinite-update loop:
 * `setStatus` would re-run the effect, which would close + reopen the
 * socket, which would fire onclose, which would `setStatus` again...
 *
 * The empty URL is a deliberate "disabled" sentinel used by useSimTransport
 * to keep this hook quiet while the WASM transport is active.
 */
export function useSimSocket(url: string): UseSimSocketResult {
  const [frame, setFrame] = useState<TelemetryFrame | null>(null);
  const [status, setStatus] = useState<ConnectionStatus>('idle');
  const [statusElapsedSec, setStatusElapsedSec] = useState(0);
  const [failedAttempts, setFailedAttempts] = useState(0);

  const socketRef = useRef<WebSocket | null>(null);
  // The latest `connect()` closure, kept in a ref so the imperative
  // `reconnect()` API can trigger it without remounting the effect.
  const reconnectFnRef = useRef<(() => void) | null>(null);

  useEffect(() => {
    if (typeof window === 'undefined') return;
    if (!url) {
      // Disabled sentinel (useSimTransport passes '' for the inactive
      // branch). Sit idle and skip everything below.
      setStatus('idle');
      socketRef.current = null;
      reconnectFnRef.current = null;
      return;
    }

    let cancelled = false;
    let socket: WebSocket | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let connectTimer: ReturnType<typeof setTimeout> | null = null;
    let attempt = 0;

    const clearLocalTimers = () => {
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
      if (connectTimer) {
        clearTimeout(connectTimer);
        connectTimer = null;
      }
    };

    const scheduleRetry = () => {
      if (cancelled || reconnectTimer) return;
      const delay = BACKOFF_MS[Math.min(attempt, BACKOFF_MS.length - 1)];
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect();
      }, delay);
    };

    const connect = () => {
      if (cancelled) return;
      // Don't double-open. If a socket is alive, leave it alone.
      if (
        socket &&
        (socket.readyState === WebSocket.OPEN ||
          socket.readyState === WebSocket.CONNECTING)
      ) {
        return;
      }
      clearLocalTimers();

      let ws: WebSocket;
      try {
        setStatus('connecting');
        ws = new WebSocket(url);
      } catch {
        attempt += 1;
        setFailedAttempts(attempt);
        setStatus('error');
        scheduleRetry();
        return;
      }
      socket = ws;
      socketRef.current = ws;

      // Hard timeout: if onopen doesn't fire within CONNECT_TIMEOUT_MS,
      // force-close. The onclose handler will schedule the next retry.
      connectTimer = setTimeout(() => {
        if (cancelled) return;
        if (ws.readyState === WebSocket.CONNECTING) {
          try {
            ws.close();
          } catch {
            /* noop */
          }
        }
      }, CONNECT_TIMEOUT_MS);

      ws.onopen = () => {
        if (cancelled) return;
        if (connectTimer) {
          clearTimeout(connectTimer);
          connectTimer = null;
        }
        attempt = 0;
        setFailedAttempts(0);
        setStatus('open');
      };
      ws.onmessage = (evt) => {
        if (cancelled) return;
        try {
          const data: TelemetryFrame = JSON.parse(evt.data);
          setFrame(data);
        } catch {
          /* drop malformed frame */
        }
      };
      ws.onerror = () => {
        if (cancelled) return;
        setStatus('error');
      };
      ws.onclose = () => {
        if (cancelled) return;
        if (connectTimer) {
          clearTimeout(connectTimer);
          connectTimer = null;
        }
        if (socketRef.current === ws) socketRef.current = null;
        socket = null;
        attempt += 1;
        setFailedAttempts(attempt);
        setStatus('closed');
        scheduleRetry();
      };
    };

    reconnectFnRef.current = () => {
      // Force-close any live socket, reset backoff, and re-enter connect()
      // immediately. Surfaced to consumers via the public `reconnect()`.
      clearLocalTimers();
      attempt = 0;
      setFailedAttempts(0);
      if (socket) {
        try {
          socket.close();
        } catch {
          /* noop */
        }
        socket = null;
      }
      connect();
    };

    connect();

    return () => {
      cancelled = true;
      clearLocalTimers();
      reconnectFnRef.current = null;
      if (socket) {
        try {
          socket.close();
        } catch {
          /* noop */
        }
        socket = null;
      }
      socketRef.current = null;
    };
  }, [url]);

  // Reset the elapsed-time counter whenever status transitions. Independent
  // of the connect effect so it can react to user-facing status without
  // poking the socket lifecycle.
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
    reconnectFnRef.current?.();
  }, []);

  return { frame, status, statusElapsedSec, failedAttempts, send, reconnect };
}
