'use client';

import { useState } from 'react';
import { Pause, Play, RotateCcw, Zap } from 'lucide-react';
import type { ControlCommand, SpacecraftSnapshot } from '@/lib/telemetry';

interface Props {
  paused: boolean;
  warp: number;
  spacecraft: SpacecraftSnapshot | null;
  send: (cmd: ControlCommand) => void;
}

const WARPS = [1, 10, 100, 1000, 10_000, 100_000];

export function ControlsPanel({ paused, warp, spacecraft, send }: Props) {
  const [thrustPct, setThrustPct] = useState(0);
  const maxThrust = spacecraft?.max_thrust ?? 1e7;

  const applyThrust = (pct: number) => {
    setThrustPct(pct);
    // Apply thrust along body +x; the dynamics engine rotates through
    // attitude so the resulting acceleration follows where the ship points.
    const f = (pct / 100) * maxThrust;
    send({ type: 'set_thrust', thrust: [f, 0, 0] });
  };

  return (
    <div className="pointer-events-auto flex items-center gap-3 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl px-4 py-3 shadow-xl">
      <button
        onClick={() => send({ type: 'set_paused', paused: !paused })}
        className="p-3 bg-blue-600 hover:bg-blue-500 rounded-xl shadow-lg shadow-blue-500/30 transition-colors"
        aria-label={paused ? 'play' : 'pause'}
      >
        {paused ? <Play className="w-5 h-5 fill-current" /> : <Pause className="w-5 h-5" />}
      </button>

      <button
        onClick={() => {
          send({ type: 'reset' });
          setThrustPct(0);
        }}
        className="p-3 bg-white/10 hover:bg-white/15 rounded-xl transition-colors"
        aria-label="reset"
      >
        <RotateCcw className="w-5 h-5" />
      </button>

      <div className="w-px h-8 bg-white/15 mx-1" />

      <div className="flex items-center gap-1">
        {WARPS.map((w) => (
          <button
            key={w}
            onClick={() => send({ type: 'set_warp', warp: w })}
            className={`px-3 py-2 rounded-lg font-mono text-xs transition-colors ${
              Math.abs(warp - w) / Math.max(warp, 1) < 0.05
                ? 'bg-indigo-500/30 text-indigo-200 border border-indigo-500/50'
                : 'text-slate-400 hover:bg-white/10 border border-transparent'
            }`}
          >
            {w >= 1000 ? `${w / 1000}k×` : `${w}×`}
          </button>
        ))}
      </div>

      <div className="w-px h-8 bg-white/15 mx-1" />

      <div className="flex items-center gap-2">
        <Zap className="w-4 h-4 text-orange-300" />
        <input
          type="range"
          min={0}
          max={100}
          step={1}
          value={thrustPct}
          onChange={(e) => applyThrust(Number(e.target.value))}
          className="w-32 accent-orange-400"
          aria-label="thrust"
        />
        <span className="text-xs font-mono text-slate-300 w-12 text-right">{thrustPct}%</span>
      </div>

      <div className="w-px h-8 bg-white/15 mx-1" />

      <select
        value={spacecraft?.drive ?? 'brachistochrone'}
        onChange={(e) =>
          send({
            type: 'set_drive',
            drive: e.target.value as 'conventional' | 'brachistochrone',
          })
        }
        className="bg-white/10 border border-white/15 rounded-lg px-2 py-1.5 text-xs text-slate-200 cursor-pointer"
      >
        <option className="bg-slate-900" value="brachistochrone">
          Brachistochrone
        </option>
        <option className="bg-slate-900" value="conventional">
          Conventional
        </option>
      </select>
    </div>
  );
}
