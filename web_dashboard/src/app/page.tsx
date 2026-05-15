'use client';

import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stars } from '@react-three/drei';
import { useState } from 'react';
import { Settings } from 'lucide-react';

import { SolarSystem } from '@/components/SolarSystem';
import { TelemetryPanel } from '@/components/TelemetryPanel';
import { ControlsPanel } from '@/components/ControlsPanel';
import { useSimSocket } from '@/hooks/useSimSocket';

const DEFAULT_WS = 'ws://127.0.0.1:8080/ws';

export default function Dashboard() {
  // The WS URL is configurable so the same static export can target a
  // different host (e.g. a co-located container). Persisted in localStorage
  // so URL edits survive a refresh.
  const [wsUrl, setWsUrl] = useState<string>(() => {
    if (typeof window === 'undefined') return DEFAULT_WS;
    return window.localStorage.getItem('expanse_ws_url') ?? DEFAULT_WS;
  });
  const [settingsOpen, setSettingsOpen] = useState(false);

  const { frame, status, send } = useSimSocket(wsUrl);

  return (
    <main className="relative w-screen h-screen bg-black overflow-hidden text-slate-200 font-sans">
      <div className="absolute inset-0 z-0">
        <Canvas
          camera={{ position: [0, 4, 4], fov: 55, near: 0.001, far: 1e6 }}
          gl={{ logarithmicDepthBuffer: true, antialias: true }}
        >
          <ambientLight intensity={0.12} />
          <pointLight position={[0, 0, 0]} intensity={3.5} decay={0} color="#fcd34d" />
          <Stars radius={300} depth={50} count={5000} factor={3} saturation={0} fade speed={0.5} />

          {frame ? (
            <SolarSystem bodies={frame.bodies} spacecraft={frame.spacecraft} />
          ) : (
            <Placeholder />
          )}

          <OrbitControls makeDefault enableDamping dampingFactor={0.08} maxDistance={200} />
        </Canvas>
      </div>

      <div className="absolute inset-0 z-10 pointer-events-none flex flex-col justify-between p-6 gap-4">
        <header className="flex justify-between items-start gap-4">
          <div className="pointer-events-auto flex items-center gap-4 bg-white/5 backdrop-blur-md border border-white/10 rounded-2xl px-5 py-3 shadow-xl">
            <h1 className="text-xl font-bold tracking-wider bg-clip-text text-transparent bg-gradient-to-r from-blue-300 to-indigo-300">
              EXPANSE SIM
            </h1>
            <span className="font-mono text-xs px-2.5 py-1 rounded-full bg-white/10">
              T+ {frame ? frame.sim_time.toFixed(2) : '—'} s
            </span>
            <button
              onClick={() => setSettingsOpen((s) => !s)}
              className="p-2 hover:bg-white/10 rounded-lg transition-colors"
              aria-label="settings"
            >
              <Settings className="w-4 h-4" />
            </button>
          </div>

          <TelemetryPanel frame={frame} status={status} />
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
            send={send}
          />
        </footer>
      </div>
    </main>
  );
}

function Placeholder() {
  return (
    <mesh>
      <sphereGeometry args={[0.1, 16, 16]} />
      <meshBasicMaterial color="#374151" wireframe />
    </mesh>
  );
}
