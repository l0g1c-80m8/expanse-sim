'use client';

import { useSimSocket, type UseSimSocketResult } from './useSimSocket';
import { useSimWasm } from './useSimWasm';

export type TransportMode = 'wasm' | 'ws';

export interface UseSimTransportResult extends UseSimSocketResult {
  mode: TransportMode;
}

/**
 * Picks between the WASM in-browser simulator and the remote WebSocket
 * transport. Both branches expose the same `UseSimSocketResult` shape so the
 * rest of the dashboard never has to care which is running.
 *
 * React's rule-of-hooks requires unconditional hook calls — so both hooks
 * run every render. The unused branch is cheap: the disabled WebSocket
 * never connects (we pass an empty URL, which `useSimSocket` interprets as
 * "do nothing"), and the disabled WASM hook never spawns its worker.
 *
 * To keep things simple we don't try to short-circuit — instead we always
 * spin up the active transport and return its result, and `null` out the
 * inactive one entirely by passing `enabled = false`.
 */
export function useSimTransport(
  mode: TransportMode,
  wsUrl: string,
  basePath: string,
): UseSimTransportResult {
  // Hook order must be stable. We call both, but only attach the
  // appropriate one's behaviour to props that meaningfully trigger work.
  // Empty URL disables useSimSocket; `__disabled__` sentinel disables
  // useSimWasm (so it doesn't try to download the WASM module from a
  // nonsense path). Whichever hook is inactive idles silently.
  const wsResult = useSimSocket(mode === 'ws' ? wsUrl : '');
  const wasmResult = useSimWasm(mode === 'wasm' ? basePath : '__disabled__');

  const active = mode === 'ws' ? wsResult : wasmResult;
  return { ...active, mode };
}
