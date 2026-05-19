'use client';

import type { TransportMode } from '@/hooks/useSimTransport';

/**
 * Header chip pair that flips the dashboard between the in-browser WASM
 * simulator and a remote WebSocket server. Choice is persisted by the
 * parent via `localStorage`.
 */
export function TransportToggle({
  mode,
  onChange,
}: {
  mode: TransportMode;
  onChange: (m: TransportMode) => void;
}) {
  const baseChip =
    'px-2 py-1 rounded-full text-[10px] font-mono uppercase tracking-wider border transition-colors';
  return (
    <div
      className="inline-flex items-center gap-1 px-1 py-0.5 rounded-full bg-black/30 border border-white/15"
      title="Where the simulator runs: in your browser (WASM) or on a remote server (WS)."
    >
      <button
        className={`${baseChip} ${
          mode === 'wasm'
            ? 'bg-violet-500/30 text-violet-100 border-violet-500/50'
            : 'border-transparent text-slate-400 hover:text-slate-200'
        }`}
        onClick={() => onChange('wasm')}
      >
        WASM
      </button>
      <button
        className={`${baseChip} ${
          mode === 'ws'
            ? 'bg-blue-500/30 text-blue-100 border-blue-500/50'
            : 'border-transparent text-slate-400 hover:text-slate-200'
        }`}
        onClick={() => onChange('ws')}
      >
        Server
      </button>
    </div>
  );
}
