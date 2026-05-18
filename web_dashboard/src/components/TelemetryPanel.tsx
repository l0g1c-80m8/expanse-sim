'use client';

import type { SpacecraftSnapshot, TelemetryFrame } from '@/lib/telemetry';

interface Props {
  frame: TelemetryFrame | null;
  status: string;
}

const fmt = (n: number, digits = 2) =>
  Number.isFinite(n) ? n.toFixed(digits) : '—';

const sci = (n: number, digits = 3) =>
  Number.isFinite(n) ? n.toExponential(digits) : '—';

function vecMag(v: [number, number, number]) {
  return Math.hypot(v[0], v[1], v[2]);
}

export function TelemetryPanel({ frame, status }: Props) {
  return (
    <aside className="pointer-events-auto w-80 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-4 text-xs font-mono space-y-3 shadow-xl">
      <header className="flex items-center justify-between">
        <span className="text-[10px] uppercase tracking-widest text-slate-400">
          Telemetry
        </span>
        <span
          className={`px-2 py-0.5 rounded-full text-[10px] ${
            status === 'open'
              ? 'bg-emerald-500/20 text-emerald-300 border border-emerald-500/40'
              : status === 'connecting'
              ? 'bg-amber-500/20 text-amber-300 border border-amber-500/40'
              : 'bg-red-500/20 text-red-300 border border-red-500/40'
          }`}
        >
          {status}
        </span>
      </header>

      {frame ? <SystemBlock frame={frame} /> : <Pending />}
      {frame?.spacecraft ? (
        <SpacecraftBlock sc={frame.spacecraft} />
      ) : (
        <div className="text-slate-500">no spacecraft</div>
      )}
    </aside>
  );
}

function Pending() {
  return <div className="text-slate-500">awaiting telemetry…</div>;
}

function SystemBlock({ frame }: { frame: TelemetryFrame }) {
  // Flag when the requested warp can't actually be delivered — once the
  // server's tick budget is saturated, effective_warp falls below warp.
  const overshoot =
    frame.warp > 0 && frame.effective_warp < frame.warp * 0.9 && !frame.paused;
  return (
    <section className="space-y-1">
      <Row k="sim-t" v={`${fmt(frame.sim_time, 1)} s`} />
      <Row k="warp req" v={`${fmt(frame.warp, 1)}×`} />
      <Row
        k="warp eff"
        v={`${fmt(frame.effective_warp, 1)}×`}
        accent={overshoot ? 'text-amber-300' : false}
      />
      <Row k="state" v={frame.paused ? 'PAUSED' : 'RUNNING'} />
      <Row k="tick" v={frame.tick.toString()} />
      <Row k="bodies" v={frame.bodies.length.toString()} />
    </section>
  );
}

function SpacecraftBlock({ sc }: { sc: SpacecraftSnapshot }) {
  const speed = vecMag(sc.velocity);
  const thrust = vecMag(sc.thrust_command);
  return (
    <section className="space-y-1 border-t border-white/10 pt-3">
      <div className="text-[10px] uppercase tracking-widest text-slate-400">
        Rocinante #{sc.id}
      </div>
      <Row k="drive" v={sc.drive} accent />
      <Row k="mass" v={`${fmt(sc.mass / 1000, 2)} t`} />
      <Row k="fuel" v={`${fmt(sc.propellant_mass / 1000, 2)} t`} />
      <Row k="speed" v={`${fmt(speed / 1000, 2)} km/s`} />
      <Row k="thrust" v={`${sci(thrust, 2)} N`} />
      <Row k="isp" v={`${fmt(sc.isp, 0)} s`} />
      <Row k="ω" v={`${sci(vecMag(sc.angular_velocity), 2)} rad/s`} />
      <Row k="|q|" v={fmt(Math.hypot(...sc.attitude), 4)} />
    </section>
  );
}

function Row({
  k,
  v,
  accent = false,
}: {
  k: string;
  v: string;
  accent?: boolean | string;
}) {
  const cls =
    typeof accent === 'string'
      ? accent
      : accent
      ? 'text-cyan-300'
      : 'text-slate-200';
  return (
    <div className="flex justify-between gap-3">
      <span className="text-slate-400">{k}</span>
      <span className={cls}>{v}</span>
    </div>
  );
}
