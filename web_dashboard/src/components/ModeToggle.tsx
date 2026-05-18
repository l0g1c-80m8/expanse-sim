'use client';

import { Compass, Rocket } from 'lucide-react';
import type { ControlCommand, SimMode } from '@/lib/telemetry';

interface Props {
  mode: SimMode;
  send: (cmd: ControlCommand) => void;
}

/**
 * Top-level operating mode toggle.
 *
 * Sandbox = live solar system, gravity-only on the ship, manual flight or
 *           hands-off observation. Useful for orbital mechanics study and
 *           nav-filter benchmarking without thrust confounds.
 *
 * Mission = scripted transit. The autopilot has authority; operator thrust
 *           commands are suppressed so the planned trajectory plays out.
 */
export function ModeToggle({ mode, send }: Props) {
  return (
    <div className="inline-flex items-center bg-black/30 border border-white/15 rounded-full p-1 text-[10px] font-mono uppercase tracking-wider">
      <Button
        active={mode === 'sandbox'}
        onClick={() => send({ type: 'set_mode', mode: 'sandbox' })}
        icon={<Compass className="w-3 h-3" />}
        label="Sandbox"
        accent="emerald"
      />
      <Button
        active={mode === 'mission'}
        onClick={() => send({ type: 'set_mode', mode: 'mission' })}
        icon={<Rocket className="w-3 h-3" />}
        label="Mission"
        accent="cyan"
      />
    </div>
  );
}

function Button({
  active,
  onClick,
  icon,
  label,
  accent,
}: {
  active: boolean;
  onClick: () => void;
  icon: React.ReactNode;
  label: string;
  accent: 'emerald' | 'cyan';
}) {
  const activeBg = accent === 'emerald' ? 'bg-emerald-500/30 text-emerald-200 border-emerald-500/50' : 'bg-cyan-500/30 text-cyan-200 border-cyan-500/50';
  return (
    <button
      onClick={onClick}
      className={`flex items-center gap-1.5 px-2.5 py-1 rounded-full border transition-colors ${
        active ? activeBg : 'border-transparent text-slate-400 hover:text-slate-200'
      }`}
    >
      {icon}
      {label}
    </button>
  );
}
