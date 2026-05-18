'use client';

import { useState, useEffect } from 'react';
import { Pause, Play, RotateCcw, Zap, Square } from 'lucide-react';
import type {
  ControlCommand,
  SpacecraftSnapshot,
  ThrustControllerSnapshot,
  ThrustModeId,
} from '@/lib/telemetry';

interface Props {
  paused: boolean;
  warp: number;
  spacecraft: SpacecraftSnapshot | null;
  thrustController: ThrustControllerSnapshot | null;
  send: (cmd: ControlCommand) => void;
}

const WARPS = [1, 10, 100, 1_000, 10_000, 100_000];

const THRUST_MODES: { id: ThrustModeId; label: string; hint?: string }[] = [
  { id: 'off', label: 'Off', hint: 'engine cut' },
  { id: 'prograde', label: 'Prograde', hint: 'along velocity — raises orbit' },
  { id: 'retrograde', label: 'Retrograde', hint: 'opposite velocity — lowers orbit' },
  { id: 'toward_target', label: 'Toward target', hint: 'point at mission target' },
  { id: 'away_from_target', label: 'Away from target' },
  { id: 'toward_source', label: 'Toward source' },
  { id: 'away_from_source', label: 'Away from source' },
  { id: 'body_x', label: 'Body +X (manual)', hint: 'no slew — manual attitude' },
];

export function ControlsPanel({
  paused,
  warp,
  spacecraft,
  thrustController,
  send,
}: Props) {
  const maxThrust = spacecraft?.max_thrust ?? 5e7;
  const currentMode = thrustController?.mode ?? 'off';
  const currentMag = thrustController?.magnitude ?? 0;
  const [pct, setPct] = useState(0);
  const [warpInput, setWarpInput] = useState<string>(String(warp));

  // Sync local slider to server state whenever the server tells us the
  // magnitude changed (e.g. on reset or via a different client).
  useEffect(() => {
    const serverPct = maxThrust > 0 ? (currentMag / maxThrust) * 100 : 0;
    if (Math.abs(serverPct - pct) > 1) setPct(serverPct);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [currentMag, maxThrust]);

  useEffect(() => {
    setWarpInput((prev) => (Number(prev) === warp ? prev : String(warp)));
  }, [warp]);

  const applyMagnitude = (pctNext: number) => {
    setPct(pctNext);
    send({
      type: 'set_thrust_magnitude',
      magnitude: (pctNext / 100) * maxThrust,
    });
  };

  const applyMode = (mode: ThrustModeId) => {
    send({ type: 'set_thrust_mode', mode });
  };

  const commitWarpInput = () => {
    const n = Number(warpInput);
    if (Number.isFinite(n) && n >= 0) {
      send({ type: 'set_warp', warp: n });
    } else {
      setWarpInput(String(warp));
    }
  };

  return (
    <div className="pointer-events-auto flex flex-col gap-3 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-3 shadow-xl max-w-3xl">
      {/* Top row: transport + warp */}
      <div className="flex items-center gap-3 flex-wrap">
        <button
          onClick={() => send({ type: 'set_paused', paused: !paused })}
          className="p-2.5 bg-blue-600 hover:bg-blue-500 rounded-xl shadow-lg shadow-blue-500/30 transition-colors"
          aria-label={paused ? 'play' : 'pause'}
          title="Space"
        >
          {paused ? <Play className="w-4 h-4 fill-current" /> : <Pause className="w-4 h-4" />}
        </button>

        <button
          onClick={() => {
            send({ type: 'reset' });
            setPct(0);
          }}
          className="p-2.5 bg-white/10 hover:bg-white/15 rounded-xl transition-colors"
          aria-label="reset"
          title="R"
        >
          <RotateCcw className="w-4 h-4" />
        </button>

        <button
          onClick={() => {
            send({ type: 'set_thrust_mode', mode: 'off' });
            applyMagnitude(0);
          }}
          className="px-3 py-2 bg-red-500/20 hover:bg-red-500/30 border border-red-500/40 text-red-200 rounded-xl text-xs uppercase tracking-wider transition-colors flex items-center gap-1.5"
          title="X — emergency engine cut"
        >
          <Square className="w-3.5 h-3.5 fill-current" /> Cut
        </button>

        <div className="w-px h-7 bg-white/15 mx-1" />

        <div className="flex items-center gap-1 flex-wrap">
          {WARPS.map((w) => (
            <button
              key={w}
              onClick={() => send({ type: 'set_warp', warp: w })}
              className={`px-2.5 py-1.5 rounded-lg font-mono text-xs transition-colors ${
                Math.abs(warp - w) / Math.max(warp, 1) < 0.05
                  ? 'bg-indigo-500/30 text-indigo-200 border border-indigo-500/50'
                  : 'text-slate-400 hover:bg-white/10 border border-transparent'
              }`}
            >
              {w >= 1000 ? `${w / 1000}k` : `${w}`}×
            </button>
          ))}
          <input
            type="number"
            min={0}
            step={1}
            value={warpInput}
            onChange={(e) => setWarpInput(e.target.value)}
            onBlur={commitWarpInput}
            onKeyDown={(e) => {
              if (e.key === 'Enter') (e.target as HTMLInputElement).blur();
            }}
            className="w-20 bg-black/30 border border-white/10 rounded-lg px-2 py-1.5 text-xs font-mono"
            title="Custom warp (Enter to apply)"
          />
        </div>
      </div>

      {/* Bottom row: thrust mode + magnitude + drive type */}
      <div className="flex items-center gap-3 flex-wrap">
        <Zap className="w-4 h-4 text-orange-300" />
        <select
          value={currentMode}
          onChange={(e) => applyMode(e.target.value as ThrustModeId)}
          className="bg-black/40 border border-orange-500/30 text-slate-200 rounded-md px-2 py-1.5 text-xs focus:outline-none focus:ring-2 focus:ring-orange-500/40"
          title={
            THRUST_MODES.find((m) => m.id === currentMode)?.hint ?? 'thrust mode'
          }
        >
          {THRUST_MODES.map((m) => (
            <option key={m.id} value={m.id} className="bg-slate-900">
              {m.label}
            </option>
          ))}
        </select>

        <input
          type="range"
          min={0}
          max={100}
          step={1}
          value={pct}
          onChange={(e) => applyMagnitude(Number(e.target.value))}
          className="w-40 accent-orange-400"
          aria-label="thrust magnitude"
          disabled={currentMode === 'off'}
        />
        <span className="text-xs font-mono text-slate-300 w-20 text-right">
          {pct.toFixed(0)}% · {thrustPretty((pct / 100) * maxThrust)}
        </span>

        <div className="w-px h-7 bg-white/15 mx-1" />

        <select
          value={spacecraft?.drive ?? 'brachistochrone'}
          onChange={(e) =>
            send({
              type: 'set_drive',
              drive: e.target.value as 'conventional' | 'brachistochrone',
            })
          }
          className="bg-white/10 border border-white/15 rounded-lg px-2 py-1.5 text-xs text-slate-200 cursor-pointer"
          title="drive type"
        >
          <option className="bg-slate-900" value="brachistochrone">
            Brachistochrone
          </option>
          <option className="bg-slate-900" value="conventional">
            Conventional
          </option>
        </select>
      </div>

      <div className="text-[10px] text-slate-500 font-mono">
        keys: <kbd className="px-1 bg-white/10 rounded">space</kbd> pause ·{' '}
        <kbd className="px-1 bg-white/10 rounded">[</kbd>/
        <kbd className="px-1 bg-white/10 rounded">]</kbd> warp ·{' '}
        <kbd className="px-1 bg-white/10 rounded">x</kbd> cut ·{' '}
        <kbd className="px-1 bg-white/10 rounded">f</kbd> focus ship ·{' '}
        <kbd className="px-1 bg-white/10 rounded">r</kbd> reset
      </div>
    </div>
  );
}

function thrustPretty(n: number): string {
  if (n >= 1e6) return `${(n / 1e6).toFixed(2)} MN`;
  if (n >= 1e3) return `${(n / 1e3).toFixed(1)} kN`;
  return `${n.toFixed(0)} N`;
}
