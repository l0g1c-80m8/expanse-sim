'use client';

import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stars, Grid } from '@react-three/drei';
import { useRef, useState } from 'react';
import { Crosshair, Settings } from 'lucide-react';

import { SolarSystem } from '@/components/SolarSystem';
import { TelemetryPanel } from '@/components/TelemetryPanel';
import { ControlsPanel } from '@/components/ControlsPanel';
import { MissionPanel } from '@/components/MissionPanel';
import { ViewPanel } from '@/components/ViewPanel';
import { AutopilotPanel } from '@/components/AutopilotPanel';
import { SensorsPanel } from '@/components/SensorsPanel';
import { NavPanel } from '@/components/NavPanel';
import { ModeToggle } from '@/components/ModeToggle';
import { TransportToggle } from '@/components/TransportToggle';
import {
  FollowCamera,
  type FocusTarget,
} from '@/components/FollowCamera';
import { useKeyboardShortcuts } from '@/hooks/useKeyboardShortcuts';
import { useBreadcrumbs } from '@/hooks/useBreadcrumbs';
import { useSimTransport, type TransportMode } from '@/hooks/useSimTransport';
import { useViewSettings } from '@/lib/viewSettings';
import { metersToSceneUnits } from '@/lib/telemetry';
import { formatSimTime } from '@/lib/format';

// Default WS endpoint. Hosted deployments (e.g. GitHub Pages) can bake in a
// different default at build time via NEXT_PUBLIC_DEFAULT_WS_URL; users can
// still override it through the settings panel (value persists in
// localStorage).
const DEFAULT_WS =
  process.env.NEXT_PUBLIC_DEFAULT_WS_URL ?? 'ws://127.0.0.1:8080/ws';
// Default to WASM so the hosted page works without any backend. Local devs
// can flip to WS via the header toggle (and the choice persists).
const DEFAULT_TRANSPORT: TransportMode = 'wasm';
const BASE_PATH = process.env.NEXT_PUBLIC_BASE_PATH ?? '';
const WARP_LADDER = [0, 1, 10, 100, 1_000, 10_000, 100_000];


export default function Dashboard() {
  const [wsUrl, setWsUrl] = useState<string>(() => {
    if (typeof window === 'undefined') return DEFAULT_WS;
    return window.localStorage.getItem('expanse_ws_url') ?? DEFAULT_WS;
  });
  const [transport, setTransport] = useState<TransportMode>(() => {
    if (typeof window === 'undefined') return DEFAULT_TRANSPORT;
    return (window.localStorage.getItem('expanse_transport') as TransportMode | null) ??
      DEFAULT_TRANSPORT;
  });
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [cameraFocus, setCameraFocus] = useState<FocusTarget>(null);
  const [view, updateView] = useViewSettings();

  const { frame, status, statusElapsedSec, failedAttempts, send, reconnect } =
    useSimTransport(transport, wsUrl, BASE_PATH);

  // Capture mass-at-engage so the Δv-budget block can compute spent Δv via
  // Tsiolkovsky. Resets when the autopilot disengages or arrives.
  const initialMassRef = useRef<number | null>(null);
  const lastEngagedRef = useRef<boolean>(false);
  if (frame?.autopilot) {
    const eng = frame.autopilot.engaged;
    if (eng && !lastEngagedRef.current && frame.spacecraft) {
      initialMassRef.current = frame.spacecraft.mass;
    }
    if (!eng) initialMassRef.current = null;
    lastEngagedRef.current = eng;
  }

  const updateTransport = (next: TransportMode) => {
    setTransport(next);
    if (typeof window !== 'undefined') {
      window.localStorage.setItem('expanse_transport', next);
    }
  };

  const breadcrumbs = useBreadcrumbs(
    frame?.spacecraft?.position ?? null,
    frame?.tick ?? 0,
    600,
  );

  const focusShip = () => setCameraFocus('ship');

  // Keyboard shortcuts → server actions.
  useKeyboardShortcuts({
    onTogglePause: () =>
      send({ type: 'set_paused', paused: !(frame?.paused ?? true) }),
    onWarpUp: () => stepWarp(frame?.warp ?? 1, +1, send),
    onWarpDown: () => stepWarp(frame?.warp ?? 1, -1, send),
    onCutEngine: () => {
      send({ type: 'set_thrust_mode', mode: 'off' });
      send({ type: 'set_thrust_magnitude', magnitude: 0 });
    },
    onFocusShip: focusShip,
    onReset: () => send({ type: 'reset' }),
  });

  return (
    <main className="relative w-screen h-screen bg-black overflow-hidden text-slate-200 font-sans">
      <div className="absolute inset-0 z-0">
        <Canvas
          camera={{ position: [4, 5, 6], fov: 50, near: 0.001, far: 10_000 }}
          gl={{ logarithmicDepthBuffer: true, antialias: true }}
        >
          <color attach="background" args={['#020617']} />
          <ambientLight intensity={0.18} />
          <pointLight position={[0, 0, 0]} intensity={3} decay={0} color="#fcd34d" />
          {view.showStars && (
            <Stars
              radius={500}
              depth={100}
              count={6000}
              factor={4}
              saturation={0}
              fade
              speed={0.3}
            />
          )}

          {view.showGrid && (
            <Grid
              args={[40, 40]}
              cellSize={1}
              cellThickness={0.6}
              cellColor="#1e293b"
              sectionSize={5}
              sectionThickness={1}
              sectionColor="#334155"
              fadeDistance={50}
              fadeStrength={1.5}
              infiniteGrid
              position={[0, -0.001, 0]}
            />
          )}

          {frame ? (
            <SolarSystem
              bodies={frame.bodies}
              spacecraft={frame.spacecraft}
              mission={frame.mission}
              view={view}
              nav={frame.nav}
              breadcrumbs={breadcrumbs}
              onBodyClick={(id) => {
                // The Sun's id is 10; treat as the 'sun' shorthand so the
                // CameraFocus helper sets a sensible default zoom.
                setCameraFocus(id === 10 ? 'sun' : id);
              }}
            />
          ) : (
            <Placeholder />
          )}

          <OrbitControls
            makeDefault
            enableDamping
            dampingFactor={0.08}
            minDistance={0.05}
            maxDistance={200}
          />

          <FollowCamera
            target={cameraFocus}
            bodies={frame?.bodies ?? []}
            spacecraftPosition={
              frame?.spacecraft
                ? metersToSceneUnits(frame.spacecraft.position)
                : null
            }
          />
        </Canvas>
      </div>

      <div className="absolute inset-0 z-10 pointer-events-none flex flex-col justify-between p-6 gap-4">
        <header className="flex justify-between items-start gap-4">
          <div className="pointer-events-auto flex flex-col gap-3">
            <div className="flex items-center gap-3 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl px-5 py-3 shadow-xl">
              <h1 className="text-xl font-bold tracking-wider bg-clip-text text-transparent bg-gradient-to-r from-blue-300 to-indigo-300">
                EXPANSE SIM
              </h1>
              <span
                className="font-mono text-xs px-2.5 py-1 rounded-full bg-white/10"
                title={frame ? `${frame.sim_time.toFixed(2)} s` : ''}
              >
                T+ {frame ? formatSimTime(frame.sim_time) : '—'}
              </span>
              {frame && <ModeToggle mode={frame.mode} send={send} />}
              <TransportToggle mode={transport} onChange={updateTransport} />
              <button
                onClick={focusShip}
                disabled={!frame?.spacecraft}
                className="p-2 hover:bg-white/10 rounded-lg transition-colors disabled:opacity-30 disabled:cursor-not-allowed"
                title="Focus camera on Rocinante (F)"
                aria-label="focus ship"
              >
                <Crosshair className="w-4 h-4" />
              </button>
              <button
                onClick={() => setSettingsOpen((s) => !s)}
                className="p-2 hover:bg-white/10 rounded-lg transition-colors"
                aria-label="settings"
              >
                <Settings className="w-4 h-4" />
              </button>
            </div>
            <ViewPanel
              settings={view}
              update={updateView}
              bodies={frame?.bodies ?? []}
              cameraFocus={cameraFocus}
              onCameraFocus={setCameraFocus}
              spacecraftAvailable={Boolean(frame?.spacecraft)}
            />
          </div>

          <div className="flex flex-col gap-3 items-end max-h-[calc(100vh-3rem)] overflow-y-auto no-scrollbar">
            <TelemetryPanel
              frame={frame}
              status={status}
              statusElapsedSec={statusElapsedSec}
              failedAttempts={failedAttempts}
              wsUrl={wsUrl}
              onReconnect={reconnect}
              transport={transport}
            />
            {frame && (
              <>
                <MissionPanel
                  mission={frame.mission}
                  bodies={frame.bodies}
                  send={send}
                />
                <AutopilotPanel
                  autopilot={frame.autopilot}
                  spacecraft={frame.spacecraft}
                  initialMass={initialMassRef.current}
                  send={send}
                />
                <NavPanel nav={frame.nav} truth={frame.spacecraft} send={send} />
                <SensorsPanel
                  sensors={frame.sensors}
                  bodies={frame.bodies}
                  send={send}
                />
              </>
            )}
          </div>
        </header>

        {settingsOpen && (
          <div className="pointer-events-auto bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl px-4 py-3 shadow-xl flex items-center gap-3 self-start">
            <label className="text-xs text-slate-400 font-mono uppercase tracking-wider">
              ws url
            </label>
            <input
              value={wsUrl}
              onChange={(e) => {
                setWsUrl(e.target.value);
                if (typeof window !== 'undefined') {
                  window.localStorage.setItem('expanse_ws_url', e.target.value);
                }
              }}
              className="bg-black/30 border border-white/10 rounded-lg px-3 py-1.5 text-xs font-mono w-80"
            />
          </div>
        )}

        <footer className="flex justify-center">
          <ControlsPanel
            paused={frame?.paused ?? true}
            warp={frame?.warp ?? 1}
            spacecraft={frame?.spacecraft ?? null}
            thrustController={frame?.thrust_controller ?? null}
            send={send}
          />
        </footer>
      </div>
    </main>
  );
}

/**
 * Step warp up or down the preset ladder. Local helper used by the
 * keyboard-shortcut handlers; kept in this file because the ladder values
 * are page-local config.
 */
function stepWarp(
  current: number,
  direction: 1 | -1,
  send: (cmd: { type: 'set_warp'; warp: number }) => void,
) {
  const idx = WARP_LADDER.findIndex((w) => w >= current);
  const cur = idx === -1 ? WARP_LADDER.length - 1 : idx;
  const next = Math.min(Math.max(cur + direction, 0), WARP_LADDER.length - 1);
  send({ type: 'set_warp', warp: WARP_LADDER[next] });
}

/** Wireframe sphere shown in the 3D scene before the first telemetry frame. */
function Placeholder() {
  return (
    <mesh>
      <sphereGeometry args={[0.1, 16, 16]} />
      <meshBasicMaterial color="#374151" wireframe />
    </mesh>
  );
}
