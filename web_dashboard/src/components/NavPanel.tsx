'use client';

import { Crosshair } from 'lucide-react';
import type {
  ControlCommand,
  NavEstimate,
  SpacecraftSnapshot,
} from '@/lib/telemetry';

interface Props {
  nav: NavEstimate;
  truth: SpacecraftSnapshot | null;
  send: (cmd: ControlCommand) => void;
}

export function NavPanel({ nav, truth, send }: Props) {
  const error =
    nav.initialized && truth
      ? Math.hypot(
          truth.position[0] - nav.position[0],
          truth.position[1] - nav.position[1],
          truth.position[2] - nav.position[2],
        )
      : 0;
  const velError =
    nav.initialized && truth
      ? Math.hypot(
          truth.velocity[0] - nav.velocity[0],
          truth.velocity[1] - nav.velocity[1],
          truth.velocity[2] - nav.velocity[2],
        )
      : 0;
  // Compare actual nav error against 1-σ. Good filters keep error around 1-σ
  // — if it persistently sits above 3-σ the model and reality have diverged.
  const errorSigma = nav.position_sigma_m > 0 ? error / nav.position_sigma_m : 0;

  return (
    <aside className="pointer-events-auto w-80 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-4 text-xs font-mono space-y-3 shadow-xl">
      <header className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Crosshair className="w-3.5 h-3.5 text-violet-300" />
          <span className="text-[10px] uppercase tracking-widest text-slate-400">
            Onboard Nav (EKF)
          </span>
        </div>
        <span
          className={`text-[10px] px-2 py-0.5 rounded border ${
            nav.initialized
              ? 'bg-violet-500/20 text-violet-200 border-violet-500/40'
              : 'bg-slate-600/20 text-slate-400 border-slate-600/40'
          }`}
        >
          {nav.initialized ? `${nav.updates_count} fix` : 'OFF'}
        </span>
      </header>

      <div className="flex gap-2">
        <button
          onClick={() => send({ type: 'set_nav_filter', enabled: !nav.initialized })}
          className={`flex-1 py-1.5 rounded-lg text-[10px] uppercase tracking-wider border transition-colors ${
            nav.initialized
              ? 'bg-rose-500/20 hover:bg-rose-500/30 border-rose-500/40 text-rose-200'
              : 'bg-violet-500/20 hover:bg-violet-500/30 border-violet-500/40 text-violet-200'
          }`}
        >
          {nav.initialized ? 'Disable' : 'Engage filter'}
        </button>
      </div>

      {nav.initialized ? (
        <>
          <section className="space-y-1 border-t border-white/10 pt-3">
            <div className="text-[10px] uppercase tracking-widest text-slate-500">
              Uncertainty (1-σ)
            </div>
            <Row k="σ_r" v={fmtDist(nav.position_sigma_m)} accent="text-violet-200" />
            <Row k="σ_v" v={`${nav.velocity_sigma_m_s.toFixed(2)} m/s`} accent="text-violet-200" />
            <Row k="last z" v={`${fmtScaledMeters(nav.residual_range_m)}`} />
          </section>

          {truth && (
            <section className="space-y-1 border-t border-white/10 pt-3">
              <div className="text-[10px] uppercase tracking-widest text-slate-500">
                Estimator error vs ground truth
              </div>
              <Row
                k="|Δr|"
                v={fmtDist(error)}
                accent={
                  errorSigma > 3
                    ? 'text-rose-300'
                    : errorSigma > 1
                    ? 'text-amber-300'
                    : 'text-emerald-300'
                }
              />
              <Row k="|Δv|" v={`${velError.toFixed(2)} m/s`} />
              <Row
                k="|Δr| / σ"
                v={`${errorSigma.toFixed(2)}`}
                accent={errorSigma > 3 ? 'text-rose-300' : 'text-slate-200'}
              />
            </section>
          )}
        </>
      ) : (
        <p className="text-[10px] text-slate-500 leading-tight border-t border-white/10 pt-3">
          A 6-state EKF over inertial position + velocity. Process model: Sun
          gravity only (cruise-style). Measurements: per-body range from the
          synthetic sensor pack. Initializes from ground truth + the
          uncertainty you set on engage; thereafter operates purely on
          sensor data.
        </p>
      )}
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
  if (m > 0.05 * AU) return `${(m / AU).toFixed(3)} AU`;
  if (m > 1.0e6) return `${(m / 1e6).toFixed(2)} Mm`;
  if (m > 1.0e3) return `${(m / 1e3).toFixed(1)} km`;
  return `${m.toFixed(0)} m`;
}

function fmtScaledMeters(m: number): string {
  if (!Number.isFinite(m)) return '—';
  const abs = Math.abs(m);
  const sign = m < 0 ? '−' : '';
  if (abs > 1.0e6) return `${sign}${(abs / 1e6).toFixed(2)} Mm`;
  if (abs > 1.0e3) return `${sign}${(abs / 1e3).toFixed(1)} km`;
  return `${sign}${abs.toFixed(0)} m`;
}
