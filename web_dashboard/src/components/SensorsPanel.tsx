'use client';

import { useState } from 'react';
import { Radio } from 'lucide-react';
import type {
  BodySnapshot,
  ControlCommand,
  SensorPack,
} from '@/lib/telemetry';

interface Props {
  sensors: SensorPack;
  bodies: BodySnapshot[];
  send: (cmd: ControlCommand) => void;
}

const NOISE_PRESETS: { label: string; cfg: Record<string, number> }[] = [
  {
    label: 'Off',
    cfg: {
      range_relative_sigma: 0,
      range_absolute_sigma_m: 0,
      range_rate_sigma_m_s: 0,
      bearing_sigma_rad: 0,
      accel_sigma_m_s2: 0,
      gyro_sigma_rad_s: 0,
      star_tracker_sigma_rad: 0,
    },
  },
  {
    label: 'DSN-class',
    cfg: {
      range_relative_sigma: 1e-6,
      range_absolute_sigma_m: 1,
      range_rate_sigma_m_s: 0.001,
      bearing_sigma_rad: 1e-6,
      accel_sigma_m_s2: 1e-5,
      gyro_sigma_rad_s: 1e-7,
      star_tracker_sigma_rad: 1e-6,
    },
  },
  {
    label: 'Spacecraft',
    cfg: {
      range_relative_sigma: 1e-5,
      range_absolute_sigma_m: 10,
      range_rate_sigma_m_s: 0.05,
      bearing_sigma_rad: 1e-5,
      accel_sigma_m_s2: 1e-4,
      gyro_sigma_rad_s: 1e-6,
      star_tracker_sigma_rad: 1e-5,
    },
  },
  {
    label: 'Stressed',
    cfg: {
      range_relative_sigma: 1e-3,
      range_absolute_sigma_m: 1000,
      range_rate_sigma_m_s: 1.0,
      bearing_sigma_rad: 1e-3,
      accel_sigma_m_s2: 0.01,
      gyro_sigma_rad_s: 1e-4,
      star_tracker_sigma_rad: 1e-3,
    },
  },
];

export function SensorsPanel({ sensors, bodies, send }: Props) {
  const [selectedBody, setSelectedBody] = useState<number | null>(
    bodies.find((b) => b.name === 'Mars')?.id ?? null,
  );
  const [open, setOpen] = useState(false);
  const [lightTimeDelay, setLightTimeDelay] = useState(false);

  const reading =
    selectedBody != null
      ? sensors.ranges.find((r) => r.body_id === selectedBody)
      : null;
  const bodyName =
    selectedBody != null
      ? bodies.find((b) => b.id === selectedBody)?.name ?? '—'
      : '—';

  return (
    <aside className="pointer-events-auto w-80 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl p-4 text-xs font-mono space-y-3 shadow-xl">
      <header className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Radio className="w-3.5 h-3.5 text-fuchsia-300" />
          <span className="text-[10px] uppercase tracking-widest text-slate-400">
            Sensors
          </span>
        </div>
        <button
          onClick={() => setOpen((o) => !o)}
          className="text-[10px] text-slate-400 hover:text-slate-200 transition-colors"
        >
          {open ? 'collapse' : 'noise…'}
        </button>
      </header>

      {open && (
        <section className="space-y-1.5 border-b border-white/10 pb-3">
          <div className="text-[10px] uppercase tracking-widest text-slate-500">
            Noise profile
          </div>
          <div className="flex flex-wrap gap-1">
            {NOISE_PRESETS.map((p) => (
              <button
                key={p.label}
                onClick={() =>
                  send({
                    type: 'set_sensor_config',
                    enabled: p.label !== 'Off' ? true : true,
                    ...p.cfg,
                  })
                }
                className="px-2 py-1 rounded text-[10px] bg-white/5 text-slate-300 hover:bg-fuchsia-500/20 hover:text-fuchsia-200 border border-transparent hover:border-fuchsia-500/40 transition-colors"
              >
                {p.label}
              </button>
            ))}
          </div>
          <label className="flex items-center gap-2 pt-1">
            <input
              type="checkbox"
              checked={lightTimeDelay}
              onChange={(e) => {
                setLightTimeDelay(e.target.checked);
                send({
                  type: 'set_sensor_config',
                  light_time_delay: e.target.checked,
                });
              }}
              className="accent-fuchsia-400"
            />
            <span className="text-slate-300 text-[10px]">
              Light-time delay
            </span>
            <span
              className="text-slate-500 text-[9px]"
              title="Range / range-rate reflect where the body was r/c seconds ago — the real DSN observation. Important for EKF benchmarking at outer-planet ranges."
            >
              ⓘ
            </span>
          </label>
        </section>
      )}

      <select
        value={selectedBody ?? ''}
        onChange={(e) =>
          setSelectedBody(e.target.value ? Number(e.target.value) : null)
        }
        className="w-full bg-black/40 border border-white/10 text-slate-200 rounded-md px-2 py-1.5 text-xs focus:outline-none focus:ring-2 focus:ring-fuchsia-500/40"
      >
        <option value="" className="bg-slate-900">
          (pick a body)
        </option>
        {bodies.map((b) => (
          <option key={b.id} value={b.id} className="bg-slate-900">
            {b.name}
          </option>
        ))}
      </select>

      <section className="space-y-1">
        <div className="text-[10px] uppercase tracking-widest text-slate-500">
          {bodyName} radar
        </div>
        {reading ? (
          <>
            <Row k="range" v={fmtDist(reading.range_m)} />
            <Row
              k="ṙ"
              v={`${(reading.range_rate_m_s / 1000).toFixed(3)} km/s`}
              accent={
                reading.range_rate_m_s > 0
                  ? 'text-emerald-300'
                  : 'text-rose-300'
              }
            />
            <Row
              k="bearing"
              v={`(${reading.bearing_inertial
                .map((c) => c.toFixed(3))
                .join(', ')})`}
            />
          </>
        ) : (
          <div className="text-slate-500">no reading</div>
        )}
      </section>

      {sensors.imu && (
        <section className="space-y-1 border-t border-white/10 pt-3">
          <div className="text-[10px] uppercase tracking-widest text-slate-500">
            IMU (body frame)
          </div>
          <Row
            k="|a|"
            v={`${vecMag(sensors.imu.specific_force_body).toExponential(2)} m/s²`}
          />
          <Row
            k="|ω|"
            v={`${vecMag(sensors.imu.angular_rate_body).toExponential(2)} rad/s`}
          />
        </section>
      )}

      {sensors.attitude && (
        <section className="space-y-1 border-t border-white/10 pt-3">
          <div className="text-[10px] uppercase tracking-widest text-slate-500">
            Star tracker
          </div>
          <Row
            k="q"
            v={sensors.attitude.q_xyzw.map((c) => c.toFixed(3)).join(', ')}
          />
        </section>
      )}

      <p className="text-[10px] text-slate-500 leading-tight">
        Synthetic measurements for nav / EKF benchmarking. Gaussian noise,
        deterministic per (sim_time, body_id).
      </p>
    </aside>
  );
}

function vecMag(v: [number, number, number]) {
  return Math.hypot(v[0], v[1], v[2]);
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
  if (m > 1.0e6) return `${(m / 1e6).toFixed(1)} Mm`;
  if (m > 1.0e3) return `${(m / 1e3).toFixed(1)} km`;
  return `${m.toFixed(0)} m`;
}
