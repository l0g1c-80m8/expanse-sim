'use client';

import type { SpacecraftSnapshot, TelemetryFrame } from '@/lib/telemetry';

interface Props {
  frame: TelemetryFrame | null;
  status: string;
  statusElapsedSec?: number;
  failedAttempts?: number;
  wsUrl?: string;
  onReconnect?: () => void;
  transport?: 'wasm' | 'ws';
}

const fmt = (n: number, digits = 2) =>
  Number.isFinite(n) ? n.toFixed(digits) : '—';

const sci = (n: number, digits = 3) =>
  Number.isFinite(n) ? n.toExponential(digits) : '—';

function vecMag(v: [number, number, number]) {
  return Math.hypot(v[0], v[1], v[2]);
}

export function TelemetryPanel({
  frame,
  status,
  statusElapsedSec = 0,
  failedAttempts = 0,
  wsUrl,
  onReconnect,
  transport = 'ws',
}: Props) {
  const showNotRunning =
    (status === 'connecting' && statusElapsedSec >= 3) ||
    status === 'closed' ||
    status === 'error' ||
    failedAttempts >= 1;

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
          {status === 'connecting' && statusElapsedSec > 0
            ? ` ${statusElapsedSec}s`
            : ''}
        </span>
      </header>

      {showNotRunning && !frame && (
        <NotConnectedHint
          wsUrl={wsUrl}
          onReconnect={onReconnect}
          transport={transport}
        />
      )}

      {frame ? <SystemBlock frame={frame} /> : !showNotRunning && <Pending />}
      {frame?.spacecraft ? (
        <SpacecraftBlock sc={frame.spacecraft} />
      ) : frame ? (
        <div className="text-slate-500">no spacecraft</div>
      ) : null}
    </aside>
  );
}

function NotConnectedHint({
  wsUrl,
  onReconnect,
  transport,
}: {
  wsUrl?: string;
  onReconnect?: () => void;
  transport: 'wasm' | 'ws';
}) {
  if (transport === 'wasm') {
    return (
      <section className="space-y-2 border-y border-amber-500/20 -mx-4 px-4 py-3 bg-amber-500/5">
        <div className="text-amber-200 text-[11px] leading-snug">
          Loading the WebAssembly simulator…
        </div>
        <div className="text-[10px] text-slate-400 leading-snug">
          The browser is downloading and instantiating the WASM module.
          First load is ~700 KB; cached on subsequent visits.
        </div>
        {onReconnect && (
          <button
            onClick={onReconnect}
            className="w-full mt-1 py-1.5 rounded text-[10px] uppercase tracking-wider bg-amber-500/20 hover:bg-amber-500/30 border border-amber-500/40 text-amber-100 transition-colors"
          >
            Retry now
          </button>
        )}
      </section>
    );
  }
  return (
    <section className="space-y-2 border-y border-amber-500/20 -mx-4 px-4 py-3 bg-amber-500/5">
      <div className="text-amber-200 text-[11px] leading-snug">
        Can&apos;t reach the simulator. Is{' '}
        <code className="text-amber-100">sim_server</code> running?
      </div>
      <code className="block text-[10px] text-slate-400 truncate">
        {wsUrl ?? 'ws://127.0.0.1:8080/ws'}
      </code>
      <div className="text-[10px] text-slate-400 leading-snug">
        Start it with:
        <br />
        <code className="text-slate-300">
          cargo run --release -p sim_server
        </code>
        <br />
        Or switch to <span className="text-violet-200">WASM</span> mode in the
        header to run in-browser.
      </div>
      {onReconnect && (
        <button
          onClick={onReconnect}
          className="w-full mt-1 py-1.5 rounded text-[10px] uppercase tracking-wider bg-amber-500/20 hover:bg-amber-500/30 border border-amber-500/40 text-amber-100 transition-colors"
        >
          Retry now
        </button>
      )}
    </section>
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
