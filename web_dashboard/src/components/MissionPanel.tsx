'use client';

import { useState } from 'react';
import { MousePointer2, Play, Target as TargetIcon } from 'lucide-react';
import type {
  BodySnapshot,
  ControlCommand,
  MissionSnapshot,
  SpacecraftSnapshot,
} from '@/lib/telemetry';

interface Props {
  mission: MissionSnapshot;
  bodies: BodySnapshot[];
  spacecraft: SpacecraftSnapshot | null;
  send: (cmd: ControlCommand) => void;
}

const TRANSIT_WARPS = [100, 1_000, 10_000, 100_000];

export function MissionPanel({ mission, bodies, spacecraft, send }: Props) {
  // Sun isn't a useful target — exclude it from the picker.
  const choices = bodies.filter((b) => b.name !== 'Sun');
  const [accelG, setAccelG] = useState(1.0);
  // High default so a 3-day Earth-Mars transit at 1g fits in ~26 seconds
  // of wall time instead of 12 hours at the dashboard's idle 60× warp.
  const [transitWarp, setTransitWarp] = useState(10_000);

  const setTarget = (target: number | null) =>
    send({ type: 'set_mission', source: mission.source, target });

  const target = bodies.find((b) => b.id === mission.target);
  const ready = mission.target != null;
  // Nearest body to the ship — useful as a "you are here" readout now that
  // the source dropdown is gone. We only show it when telemetry is live.
  const nearest = spacecraft ? nearestBody(spacecraft, bodies) : null;

  return (
    <aside className="pointer-events-auto w-80 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-4 text-xs font-mono space-y-3 shadow-xl">
      <header className="flex items-center justify-between">
        <span className="text-[10px] uppercase tracking-widest text-slate-400">
          Mission
        </span>
        <span className="text-[10px] text-slate-500">target body</span>
      </header>

      <div className="flex items-start gap-2 text-[10px] text-slate-400 bg-white/5 border border-white/10 rounded-lg p-2">
        <MousePointer2 className="w-3 h-3 mt-0.5 shrink-0 text-cyan-300" />
        <span>
          Click any planet or moon in the scene to set it as the target — the
          autopilot retargets immediately while engaged.
        </span>
      </div>

      <BodyPicker
        icon={<TargetIcon className="w-3.5 h-3.5 text-rose-300" />}
        label="target"
        selected={mission.target}
        choices={choices}
        onChange={setTarget}
      />

      {nearest && (
        <div className="flex justify-between text-[10px] text-slate-500">
          <span>you are here</span>
          <span className="text-slate-300">
            {nearest.name} · {nearest.distanceKm < 1000
              ? `${nearest.distanceKm.toFixed(0)} km`
              : nearest.distanceKm < 1.0e6
              ? `${(nearest.distanceKm / 1000).toFixed(2)} Mm`
              : `${(nearest.distanceKm / 1.495_978_707e8).toFixed(3)} AU`}
          </span>
        </div>
      )}

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
              target: mission.target ?? undefined,
              accel_g: accelG,
              warp: transitWarp,
            })
          }
          disabled={!ready}
          className="w-full flex items-center justify-center gap-2 bg-cyan-500/25 hover:bg-cyan-500/35 disabled:opacity-30 disabled:cursor-not-allowed border border-cyan-500/50 text-cyan-100 rounded-lg py-2 text-xs uppercase tracking-wider transition-colors"
          title="Switch to Mission mode, engage autopilot toward the selected target from the ship's current state, and bump warp"
        >
          <Play className="w-3.5 h-3.5 fill-current" />
          Plan &amp; Run
        </button>
        <button
          onClick={() => send({ type: 'stage_at_source' })}
          disabled={mission.source == null}
          className="w-full bg-white/5 hover:bg-white/10 disabled:opacity-30 disabled:cursor-not-allowed border border-white/10 text-slate-300 rounded-lg py-1 text-[10px] uppercase tracking-wider transition-colors"
          title="Optional: teleport the spacecraft back to the source body's parking orbit for a clean start"
        >
          Restage at source
        </button>
      </div>

      {ready && spacecraft && target && (
        <TransitEstimate spacecraft={spacecraft} target={target} />
      )}
    </aside>
  );
}

function nearestBody(
  spacecraft: SpacecraftSnapshot,
  bodies: BodySnapshot[],
): { name: string; distanceKm: number } | null {
  let bestName: string | null = null;
  let bestD = Infinity;
  for (const b of bodies) {
    if (b.name === 'Sun') continue;
    const dx = b.position[0] - spacecraft.position[0];
    const dy = b.position[1] - spacecraft.position[1];
    const dz = b.position[2] - spacecraft.position[2];
    const d = Math.hypot(dx, dy, dz);
    if (d < bestD) {
      bestD = d;
      bestName = b.name;
    }
  }
  if (!bestName) return null;
  return { name: bestName, distanceKm: bestD / 1000 };
}

function BodyPicker({
  icon,
  label,
  selected,
  choices,
  onChange,
}: {
  icon: React.ReactNode;
  label: string;
  selected: number | null;
  choices: BodySnapshot[];
  onChange: (id: number | null) => void;
}) {
  return (
    <label className="flex items-center gap-2">
      {icon}
      <span className="text-slate-400 w-12">{label}</span>
      <select
        value={selected ?? ''}
        onChange={(e) =>
          onChange(e.target.value === '' ? null : Number(e.target.value))
        }
        className="flex-1 bg-black/40 border border-rose-500/30 text-slate-200 rounded-md px-2 py-1 text-xs focus:outline-none focus:ring-2 focus:ring-rose-500/40"
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

/**
 * Order-of-magnitude transit estimates from the *ship's current state* to
 * the target body. The brachistochrone TOF (2·√(d/a)) is the headline
 * figure since that's what the autopilot actually flies. We also surface
 * the closing speed so the operator can sanity-check whether the geometry
 * is favourable or whether they're chasing a body that's racing away.
 */
function TransitEstimate({
  spacecraft,
  target,
}: {
  spacecraft: SpacecraftSnapshot;
  target: BodySnapshot;
}) {
  const dx = target.position[0] - spacecraft.position[0];
  const dy = target.position[1] - spacecraft.position[1];
  const dz = target.position[2] - spacecraft.position[2];
  const d = Math.hypot(dx, dy, dz);
  const brach1g = 2 * Math.sqrt(d / 9.81);
  // Range-rate = −d|r|/dt; positive = approaching.
  const dvx = target.velocity[0] - spacecraft.velocity[0];
  const dvy = target.velocity[1] - spacecraft.velocity[1];
  const dvz = target.velocity[2] - spacecraft.velocity[2];
  const closingMs = d > 0 ? -(dx * dvx + dy * dvy + dz * dvz) / d : 0;

  return (
    <section className="border-t border-white/10 pt-3 space-y-1 text-[11px]">
      <div className="text-[10px] uppercase tracking-widest text-slate-500">
        From ship → target
      </div>
      <div className="flex justify-between">
        <span className="text-slate-400">distance</span>
        <span className="text-slate-200">{(d / AU).toFixed(2)} AU</span>
      </div>
      <div className="flex justify-between">
        <span className="text-slate-400">brach @ 1 g</span>
        <span className="text-cyan-300">{fmtDuration(brach1g)}</span>
      </div>
      <div className="flex justify-between">
        <span className="text-slate-400">closing</span>
        <span
          className={closingMs >= 0 ? 'text-emerald-300' : 'text-rose-300'}
        >
          {(closingMs / 1000).toFixed(2)} km/s
        </span>
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
