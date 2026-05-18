'use client';

import { Eye } from 'lucide-react';
import type { BodySnapshot } from '@/lib/telemetry';
import type { ViewSettings } from '@/lib/viewSettings';

interface Props {
  settings: ViewSettings;
  update: (patch: Partial<ViewSettings>) => void;
  bodies: BodySnapshot[];
  cameraFocus: number | 'ship' | 'sun' | null;
  onCameraFocus: (target: number | 'ship' | 'sun' | null) => void;
  spacecraftAvailable: boolean;
}

const HORIZON_PRESETS: { label: string; seconds: number }[] = [
  { label: '1 d', seconds: 86_400 },
  { label: '7 d', seconds: 7 * 86_400 },
  { label: '1 mo', seconds: 30 * 86_400 },
  { label: '6 mo', seconds: 6 * 30 * 86_400 },
  { label: '1 yr', seconds: 365 * 86_400 },
  { label: '5 yr', seconds: 5 * 365 * 86_400 },
];

export function ViewPanel({
  settings,
  update,
  bodies,
  cameraFocus,
  onCameraFocus,
  spacecraftAvailable,
}: Props) {
  return (
    <aside className="pointer-events-auto w-72 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-4 text-xs font-mono space-y-3 shadow-xl">
      <header className="flex items-center gap-2">
        <Eye className="w-3.5 h-3.5 text-slate-400" />
        <span className="text-[10px] uppercase tracking-widest text-slate-400">
          View
        </span>
      </header>

      <section className="space-y-1.5">
        <Toggle
          label="Orbits"
          value={settings.showOrbits}
          onChange={(v) => update({ showOrbits: v })}
        />
        <Toggle
          label="Labels"
          value={settings.showLabels}
          onChange={(v) => update({ showLabels: v })}
        />
        <Toggle
          label="Grid"
          value={settings.showGrid}
          onChange={(v) => update({ showGrid: v })}
        />
        <Toggle
          label="Stars"
          value={settings.showStars}
          onChange={(v) => update({ showStars: v })}
        />
        <Toggle
          label="Predicted trajectory"
          value={settings.showTrajectory}
          onChange={(v) => update({ showTrajectory: v })}
        />
      </section>

      {settings.showTrajectory && (
        <section className="space-y-1.5 border-t border-white/10 pt-3">
          <div className="text-[10px] uppercase tracking-widest text-slate-500">
            Forecast horizon
          </div>
          <div className="flex flex-wrap gap-1">
            {HORIZON_PRESETS.map((p) => (
              <button
                key={p.label}
                onClick={() => update({ trajectoryHorizonSec: p.seconds })}
                className={`px-2 py-1 rounded text-[10px] transition-colors ${
                  Math.abs(settings.trajectoryHorizonSec - p.seconds) <
                  p.seconds * 0.05
                    ? 'bg-cyan-500/20 text-cyan-200 border border-cyan-500/40'
                    : 'bg-white/5 text-slate-400 hover:bg-white/10 border border-transparent'
                }`}
              >
                {p.label}
              </button>
            ))}
          </div>
        </section>
      )}

      <section className="space-y-1.5 border-t border-white/10 pt-3">
        <div className="text-[10px] uppercase tracking-widest text-slate-500">
          Camera focus
        </div>
        <select
          value={
            cameraFocus === null
              ? ''
              : cameraFocus === 'ship'
              ? 'ship'
              : cameraFocus === 'sun'
              ? 'sun'
              : String(cameraFocus)
          }
          onChange={(e) => {
            const v = e.target.value;
            if (v === '') onCameraFocus(null);
            else if (v === 'ship') onCameraFocus('ship');
            else if (v === 'sun') onCameraFocus('sun');
            else onCameraFocus(Number(v));
          }}
          className="w-full bg-black/40 border border-white/10 text-slate-200 rounded-md px-2 py-1.5 text-xs focus:outline-none focus:ring-2 focus:ring-blue-500/40"
        >
          <option value="" className="bg-slate-900">
            (free orbit)
          </option>
          <option
            value="ship"
            className="bg-slate-900"
            disabled={!spacecraftAvailable}
          >
            Rocinante
          </option>
          {bodies.map((b) => (
            <option key={b.id} value={b.id} className="bg-slate-900">
              {b.name}
            </option>
          ))}
        </select>
      </section>
    </aside>
  );
}

function Toggle({
  label,
  value,
  onChange,
}: {
  label: string;
  value: boolean;
  onChange: (v: boolean) => void;
}) {
  return (
    <label className="flex items-center justify-between cursor-pointer select-none">
      <span className="text-slate-300">{label}</span>
      <button
        onClick={() => onChange(!value)}
        className={`relative w-9 h-5 rounded-full transition-colors ${
          value ? 'bg-blue-500/70' : 'bg-white/10'
        }`}
        aria-pressed={value}
        type="button"
      >
        <span
          className={`absolute top-0.5 w-4 h-4 rounded-full bg-white transition-transform ${
            value ? 'translate-x-4' : 'translate-x-0.5'
          }`}
        />
      </button>
    </label>
  );
}
