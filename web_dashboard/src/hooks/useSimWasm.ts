'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import type { ControlCommand, TelemetryFrame } from '@/lib/telemetry';
import type { ConnectionStatus, UseSimSocketResult } from './useSimSocket';

/**
 * Returns the same shape as {@link useSimSocket}, but runs the simulator in
 * a Web Worker via the WASM build. Lets the dashboard's published Pages
 * deployment work without any backend.
 *
 * `basePath` should match `NEXT_PUBLIC_BASE_PATH` (e.g. `/expanse-sim`); the
 * worker uses it to resolve the WASM module URL.
 */
const DISABLED = '__disabled__';

export function useSimWasm(basePath: string = ''): UseSimSocketResult {
  const enabled = basePath !== DISABLED;
  const [frame, setFrame] = useState<TelemetryFrame | null>(null);
  const [status, setStatus] = useState<ConnectionStatus>('idle');
  const [statusElapsedSec, setStatusElapsedSec] = useState(0);
  const [failedAttempts, setFailedAttempts] = useState(0);
  const workerRef = useRef<Worker | null>(null);

  useEffect(() => {
    if (typeof window === 'undefined') return;
    if (!enabled) {
      setStatus('idle');
      return;
    }
    let cancelled = false;

    setStatus('connecting');

    // The worker is bundled as a module by Turbopack via `new URL(..., import.meta.url)`.
    const worker = new Worker(
      new URL('../workers/sim_worker.ts', import.meta.url),
      { type: 'module' },
    );
    workerRef.current = worker;

    worker.onmessage = (evt: MessageEvent) => {
      if (cancelled) return;
      const data = evt.data as
        | { type: 'ready' }
        | { type: 'frame'; frame: TelemetryFrame }
        | { type: 'error'; message: string };

      switch (data.type) {
        case 'ready':
          setStatus('open');
          setFailedAttempts(0);
          worker.postMessage({ type: 'start' });
          break;
        case 'frame':
          setFrame(data.frame);
          break;
        case 'error':
          setStatus('error');
          setFailedAttempts((n) => n + 1);
          // eslint-disable-next-line no-console
          console.error('sim_worker:', data.message);
          break;
      }
    };

    worker.onerror = (e) => {
      if (cancelled) return;
      setStatus('error');
      setFailedAttempts((n) => n + 1);
      // eslint-disable-next-line no-console
      console.error('sim_worker uncaught:', e);
    };

    worker.postMessage({ type: 'init', basePath });

    return () => {
      cancelled = true;
      worker.postMessage({ type: 'stop' });
      worker.terminate();
      workerRef.current = null;
    };
  }, [basePath, enabled]);

  // Elapsed seconds since last status change — same shape as useSimSocket.
  useEffect(() => {
    setStatusElapsedSec(0);
    const start = Date.now();
    const id = setInterval(() => {
      setStatusElapsedSec(Math.floor((Date.now() - start) / 1000));
    }, 500);
    return () => clearInterval(id);
  }, [status]);

  const send = useCallback((cmd: ControlCommand) => {
    workerRef.current?.postMessage({ type: 'command', command: cmd });
  }, []);

  const reconnect = useCallback(() => {
    // The hook re-runs on `basePath` change; without that we just nudge the
    // worker via stop+start which restarts the loop but reuses the live
    // SimWasm instance. For a hard reset, use the `Reset` ControlCommand.
    workerRef.current?.postMessage({ type: 'stop' });
    workerRef.current?.postMessage({ type: 'start' });
  }, []);

  return {
    frame,
    status,
    statusElapsedSec,
    failedAttempts,
    send,
    reconnect,
  };
}
