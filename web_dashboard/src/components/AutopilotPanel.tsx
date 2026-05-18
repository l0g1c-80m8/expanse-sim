'use client';

import { useState } from 'react';
import { Cpu, Gauge } from 'lucide-react';
import type {
  AutopilotPhase,
  AutopilotSnapshot,
  ControlCommand,
} from '@/lib/telemetry';

interface Props {
  autopilot: AutopilotSnapshot;
  send: (cmd: ControlCommand) => void;
}

const PHASE_COLORS: Record<AutopilotPhase, string> = {
  idle: 'bg-slate-500/20 text-slate-300 border-slate-500/40',
  boost: 'bg-amber-500/20 text-amber-200 border-amber-500/40',
  brake: 'bg-rose-500/20 text-rose-200 border-rose-500/40',
  arrived: 'bg-emerald-500/20 text-emerald-200 border-emerald-500/40',
  hold: 'bg-zinc-500/20 text-zinc-300 border-zinc-500/40',
};

const PHASE_LABELS: Record<AutopilotPhase, string> = {
  idle: 'IDLE',
  boost: '◀ BOOST ▶',
  brake: '▼ BRAKE ▼',
  arrived: '✓ ARRIVED',
  hold: '— HOLD',
};

export function AutopilotPanel({ autopilot, send }: Props) {
  const [accelG, setAccelG] = useState(autopilot.accel_g || 1.0);

  const toggle = () => {
    send({
      type: 'set_autopilot',
      engaged: !autopilot.engaged,
      accel_g: accelG,
    });
  };

  return (
    <aside className="pointer-events-auto w-80 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-4 text-xs font-mono space-y-3 shadow-xl">
      <header className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Cpu className="w-3.5 h-3.5 text-cyan-300" />
          <span className="text-[10px] uppercase tracking-widest text-slate-400">
            Autopilot
          </span>
        </div>
        <span
          className={`px-2 py-0.5 rounded text-[10px] border font-semibold ${
            PHASE_COLORS[autopilot.phase]
          }`}
        >
          {PHASE_LABELS[autopilot.phase]}
        </span>
      </header>

      <button
        onClick={toggle}
        className={`w-full py-2 rounded-lg text-xs uppercase tracking-wider transition-colors ${
          autopilot.engaged
            ? 'bg-rose-500/20 hover:bg-rose-500/30 border border-rose-500/40 text-rose-200'
            : 'bg-emerald-500/20 hover:bg-emerald-500/30 border border-emerald-500/40 text-emerald-200'
        }`}
      >
        {autopilot.engaged ? 'Disengage' : 'Engage rendezvous'}
      </button>

      <div className="space-y-1.5">
        <label className="flex items-center gap-2">
          <Gauge className="w-3.5 h-3.5 text-amber-300" />
          <span className="text-slate-400 w-16">accel</span>
          <input
            type="range"
            min={0.1}
            max={5.0}
            step={0.1}
            value={accelG}
            onChange={(e) => {
              const v = Number(e.target.value);
              setAccelG(v);
              if (autopilot.engaged) {
                send({ type: 'set_autopilot', engaged: true, accel_g: v });
              }
            }}
            className="flex-1 accent-amber-400"
          />
          <span className="text-amber-200 w-12 text-right">
            {accelG.toFixed(1)} g
          </span>
        </label>
      </div>

      <section className="space-y-1 border-t border-white/10 pt-3">
        <Row k="range" v={fmtDist(autopilot.range_m)} />
        <Row
          k="closing"
          v={`${(autopilot.closing_m_s / 1000).toFixed(2)} km/s`}
          accent={autopilot.closing_m_s > 0 ? 'text-emerald-300' : 'text-rose-300'}
        />
        <Row k="eta" v={fmtDuration(autopilot.eta_s)} />
      </section>

      <p className="text-[10px] text-slate-500 leading-tight">
        Engages a ZEM/ZEV guidance law that nulls both relative position and
        velocity at the mission target. Accel = commanded proper acceleration
        during boost &amp; brake phases.
      </p>
    </aside>
  );
}

function Row({
  k,
  v,
  accent,
}: {
  k: string;
  v: string;
  accent?: string;
}) {
  return (
    <div className="flex justify-between gap-3">
      <span className="text-slate-400">{k}</span>
      <span className={accent ?? 'text-slate-200'}>{v}</span>
    </div>
  );
}

function fmtDist(m: number): string {
  if (!Number.isFinite(m)) return '—';
  const AU = 1.495_978_707e11;
  if (m > 0.05 * AU) return `${(m / AU).toFixed(2)} AU`;
  if (m > 1.0e6) return `${(m / 1e6).toFixed(1)} Mm`;
  if (m > 1.0e3) return `${(m / 1e3).toFixed(1)} km`;
  return `${m.toFixed(0)} m`;
}

function fmtDuration(s: number): string {
  if (!Number.isFinite(s)) return '—';
  if (s > 365 * 86_400) return `${(s / (365.25 * 86_400)).toFixed(1)} yr`;
  if (s > 86_400) return `${(s / 86_400).toFixed(1)} d`;
  if (s > 3600) return `${(s / 3600).toFixed(1)} h`;
  if (s > 60) return `${(s / 60).toFixed(1)} min`;
  return `${s.toFixed(0)} s`;
}
