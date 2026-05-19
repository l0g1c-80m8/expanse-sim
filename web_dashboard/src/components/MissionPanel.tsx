'use client';

import { useState } from 'react';
import { Play, Rocket, Target as TargetIcon } from 'lucide-react';
import type {
  BodySnapshot,
  ControlCommand,
  MissionSnapshot,
} from '@/lib/telemetry';

interface Props {
  mission: MissionSnapshot;
  bodies: BodySnapshot[];
  send: (cmd: ControlCommand) => void;
}

const TRANSIT_WARPS = [100, 1_000, 10_000, 100_000];

export function MissionPanel({ mission, bodies, send }: Props) {
  // Sun isn't a useful source/target — exclude it from the picker.
  const choices = bodies.filter((b) => b.name !== 'Sun');
  const [accelG, setAccelG] = useState(1.0);
  // High default so a 3-day Earth-Mars transit at 1g fits in ~26 seconds
  // of wall time instead of 12 hours at the dashboard's idle 60× warp.
  const [transitWarp, setTransitWarp] = useState(10_000);

  const update = (
    source: number | null,
    target: number | null,
  ) => send({ type: 'set_mission', source, target });

  const ready = mission.source != null && mission.target != null;

  return (
    <aside className="pointer-events-auto w-80 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-4 text-xs font-mono space-y-3 shadow-xl">
      <header className="flex items-center justify-between">
        <span className="text-[10px] uppercase tracking-widest text-slate-400">
          Mission
        </span>
        <span className="text-[10px] text-slate-500">
          source → target
        </span>
      </header>

      <div className="space-y-2">
        <BodyPicker
          icon={<Rocket className="w-3.5 h-3.5 text-emerald-300" />}
          label="source"
          selected={mission.source}
          choices={choices}
          onChange={(v) => update(v, mission.target)}
          accent="emerald"
        />
        <BodyPicker
          icon={<TargetIcon className="w-3.5 h-3.5 text-rose-300" />}
          label="target"
          selected={mission.target}
          choices={choices}
          onChange={(v) => update(mission.source, v)}
          accent="rose"
        />
      </div>

      <button
        onClick={() => send({ type: 'stage_at_source' })}
        disabled={mission.source == null}
        className="w-full bg-emerald-500/10 hover:bg-emerald-500/20 disabled:opacity-30 disabled:cursor-not-allowed border border-emerald-500/30 text-emerald-200 rounded-lg py-1.5 text-[10px] uppercase tracking-wider transition-colors"
        title="Reposition the spacecraft to the source body's current orbit"
      >
        Stage at source
      </button>

      <div className="border-t border-white/10 pt-3 space-y-2">
        <label className="flex items-center gap-2">
          <span className="text-slate-400 w-12">accel</span>
          <input
            type="range"
            min={0.1}
            max={5.0}
            step={0.1}
            value={accelG}
            onChange={(e) => setAccelG(Number(e.target.value))}
            className="flex-1 accent-cyan-400"
          />
          <span className="text-cyan-200 w-12 text-right">
            {accelG.toFixed(1)} g
          </span>
        </label>

        <div className="flex items-center gap-2">
          <span className="text-slate-400 w-12">warp</span>
          <div className="flex-1 flex gap-1">
            {TRANSIT_WARPS.map((w) => (
              <button
                key={w}
                onClick={() => setTransitWarp(w)}
                className={`flex-1 py-1 rounded text-[10px] font-mono transition-colors ${
                  transitWarp === w
                    ? 'bg-cyan-500/25 text-cyan-100 border border-cyan-500/40'
                    : 'border border-transparent text-slate-400 hover:bg-white/5'
                }`}
              >
                {w >= 1000 ? `${w / 1000}k×` : `${w}×`}
              </button>
            ))}
          </div>
        </div>

        <button
          onClick={() =>
            send({
              type: 'start_mission',
              source: mission.source ?? undefined,
              target: mission.target ?? undefined,
              accel_g: accelG,
              warp: transitWarp,
            })
          }
          disabled={!ready}
          className="w-full flex items-center justify-center gap-2 bg-cyan-500/25 hover:bg-cyan-500/35 disabled:opacity-30 disabled:cursor-not-allowed border border-cyan-500/50 text-cyan-100 rounded-lg py-2 text-xs uppercase tracking-wider transition-colors"
          title="Switch to Mission mode, stage at source, engage autopilot, and bump warp in one shot"
        >
          <Play className="w-3.5 h-3.5 fill-current" />
          Plan &amp; Run
        </button>
      </div>

      {ready && (
        <TransitEstimate
          source={bodies.find((b) => b.id === mission.source)}
          target={bodies.find((b) => b.id === mission.target)}
        />
      )}
    </aside>
  );
}

function BodyPicker({
  icon,
  label,
  selected,
  choices,
  onChange,
  accent,
}: {
  icon: React.ReactNode;
  label: string;
  selected: number | null;
  choices: BodySnapshot[];
  onChange: (id: number | null) => void;
  accent: 'emerald' | 'rose';
}) {
  const ring =
    accent === 'emerald'
      ? 'focus:ring-emerald-500/40 border-emerald-500/30'
      : 'focus:ring-rose-500/40 border-rose-500/30';
  return (
    <label className="flex items-center gap-2">
      {icon}
      <span className="text-slate-400 w-12">{label}</span>
      <select
        value={selected ?? ''}
        onChange={(e) =>
          onChange(e.target.value === '' ? null : Number(e.target.value))
        }
        className={`flex-1 bg-black/40 border ${ring} text-slate-200 rounded-md px-2 py-1 text-xs focus:outline-none focus:ring-2`}
      >
        <option value="" className="bg-slate-900">
          —
        </option>
        {choices.map((b) => (
          <option key={b.id} value={b.id} className="bg-slate-900">
            {b.name}
          </option>
        ))}
      </select>
    </label>
  );
}

const AU = 1.495_978_707e11;
const MU_SUN = 1.327_124_400_18e20;

/**
 * Quick Hohmann + brachistochrone transit-time estimates.
 *
 * Hohmann: t = π · √( (r1 + r2)³ / (8 · μ) )
 * Brach (constant 1 g):  t = 2 · √( d / a )  where a = 9.81 m/s², d is the
 *   straight-line current distance — useful as a "back of envelope" order-
 *   of-magnitude figure even though it ignores gravity gradients.
 */
function TransitEstimate({
  source,
  target,
}: {
  source: BodySnapshot | undefined;
  target: BodySnapshot | undefined;
}) {
  if (!source || !target) return null;
  const r1 = Math.hypot(...source.position);
  const r2 = Math.hypot(...target.position);
  const hohmann = Math.PI * Math.sqrt(Math.pow(r1 + r2, 3) / (8 * MU_SUN));
  const d = Math.hypot(
    source.position[0] - target.position[0],
    source.position[1] - target.position[1],
    source.position[2] - target.position[2],
  );
  const brach1g = 2 * Math.sqrt(d / 9.81);

  return (
    <section className="border-t border-white/10 pt-3 space-y-1 text-[11px]">
      <div className="text-[10px] uppercase tracking-widest text-slate-500">
        Order-of-magnitude
      </div>
      <div className="flex justify-between">
        <span className="text-slate-400">distance</span>
        <span className="text-slate-200">{(d / AU).toFixed(2)} AU</span>
      </div>
      <div className="flex justify-between">
        <span className="text-slate-400">Hohmann</span>
        <span className="text-amber-300">{fmtDuration(hohmann)}</span>
      </div>
      <div className="flex justify-between">
        <span className="text-slate-400">brach @ 1 g</span>
        <span className="text-cyan-300">{fmtDuration(brach1g)}</span>
      </div>
    </section>
  );
}

function fmtDuration(seconds: number): string {
  if (!Number.isFinite(seconds)) return '—';
  const d = seconds / 86_400;
  if (d >= 365) return `${(d / 365.25).toFixed(2)} yr`;
  if (d >= 1) return `${d.toFixed(1)} d`;
  const h = seconds / 3600;
  if (h >= 1) return `${h.toFixed(1)} h`;
  return `${(seconds / 60).toFixed(1)} min`;
}
