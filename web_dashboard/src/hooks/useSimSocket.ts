'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import type { ControlCommand, TelemetryFrame } from '@/lib/telemetry';

export type ConnectionStatus = 'idle' | 'connecting' | 'open' | 'closed' | 'error';

export interface UseSimSocketResult {
  frame: TelemetryFrame | null;
  status: ConnectionStatus;
  send: (cmd: ControlCommand) => void;
  reconnect: () => void;
}

/**
 * Hook that opens a WebSocket to the Rust sim_server, keeps the most recent
 * telemetry frame in state, and exposes a `send()` for control commands.
 *
 * Reconnection: backoff at 1 s on close/error. Survives StrictMode double-
 * mount because we only open the socket once per mount.
 */
export function useSimSocket(url: string): UseSimSocketResult {
  const [frame, setFrame] = useState<TelemetryFrame | null>(null);
  const [status, setStatus] = useState<ConnectionStatus>('idle');
  const socketRef = useRef<WebSocket | null>(null);
  const reconnectKey = useRef(0);

  const connect = useCallback(() => {
    if (typeof window === 'undefined') return;
    try {
      setStatus('connecting');
      const ws = new WebSocket(url);
      socketRef.current = ws;
      ws.onopen = () => setStatus('open');
      ws.onmessage = (evt) => {
        try {
          const data: TelemetryFrame = JSON.parse(evt.data);
          setFrame(data);
        } catch {
          /* drop malformed frame */
        }
      };
      ws.onclose = () => {
        setStatus('closed');
        socketRef.current = null;
        const k = ++reconnectKey.current;
        setTimeout(() => {
          if (reconnectKey.current === k) connect();
        }, 1000);
      };
      ws.onerror = () => setStatus('error');
    } catch {
      setStatus('error');
    }
  }, [url]);

  useEffect(() => {
    connect();
    return () => {
      reconnectKey.current++;
      socketRef.current?.close();
      socketRef.current = null;
    };
  }, [connect]);

  const send = useCallback((cmd: ControlCommand) => {
    const ws = socketRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(cmd));
    }
  }, []);

  const reconnect = useCallback(() => {
    socketRef.current?.close();
    socketRef.current = null;
    connect();
  }, [connect]);

  return { frame, status, send, reconnect };
}
