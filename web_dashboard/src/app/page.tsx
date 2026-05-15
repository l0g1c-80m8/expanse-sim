"use client";

import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stars } from '@react-three/drei';
import { useState, useEffect } from 'react';
import { Play, Pause, FastForward, Settings } from 'lucide-react';

export default function Dashboard() {
  const [simTime, setSimTime] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [warpSpeed, setWarpSpeed] = useState(1);

  // In a real app, this would connect to the Rust backend via WebSockets
  // const ws = new WebSocket("ws://localhost:5555");
  
  useEffect(() => {
    let interval: NodeJS.Timeout;
    if (isPlaying) {
      interval = setInterval(() => {
        setSimTime((prev) => prev + 0.01 * warpSpeed);
      }, 10);
    }
    return () => clearInterval(interval);
  }, [isPlaying, warpSpeed]);

  return (
    <div className="relative w-screen h-screen bg-black overflow-hidden text-slate-200 font-sans">
      {/* 3D Visualization Layer */}
      <div className="absolute inset-0 z-0">
        <Canvas camera={{ position: [0, 50, 50], fov: 60 }} gl={{ logarithmicDepthBuffer: true }}>
          <ambientLight intensity={0.1} />
          <pointLight position={[0, 0, 0]} intensity={2} color="#fcd34d" />
          <Stars radius={300} depth={50} count={5000} factor={4} saturation={0} fade speed={1} />
          
          {/* Mock Sun */}
          <mesh position={[0, 0, 0]}>
            <sphereGeometry args={[5, 32, 32]} />
            <meshBasicMaterial color="#fcd34d" />
          </mesh>

          {/* Mock Planet */}
          <mesh position={[20 * Math.cos(simTime), 0, 20 * Math.sin(simTime)]}>
            <sphereGeometry args={[1, 32, 32]} />
            <meshStandardMaterial color="#3b82f6" roughness={0.7} />
          </mesh>

          <OrbitControls makeDefault />
        </Canvas>
      </div>

      {/* UI Overlay */}
      <div className="absolute inset-0 z-10 pointer-events-none flex flex-col justify-between p-6">
        {/* Header Bar */}
        <header className="flex justify-between items-center pointer-events-auto backdrop-blur-md bg-white/5 p-4 rounded-2xl border border-white/10 shadow-xl">
          <div className="flex items-center gap-4">
            <h1 className="text-xl font-bold tracking-wider text-transparent bg-clip-text bg-gradient-to-r from-blue-400 to-indigo-400">EXPANSE SIM</h1>
            <span className="px-3 py-1 bg-white/10 rounded-full text-xs font-mono">T+ {simTime.toFixed(2)}s</span>
          </div>
          <button className="p-2 hover:bg-white/10 rounded-lg transition-colors">
            <Settings className="w-5 h-5" />
          </button>
        </header>

        {/* Bottom Control Bar */}
        <div className="flex justify-center pointer-events-auto">
          <div className="flex items-center gap-4 backdrop-blur-md bg-white/5 p-3 rounded-2xl border border-white/10 shadow-xl">
            <button 
              onClick={() => setIsPlaying(!isPlaying)}
              className="p-3 bg-blue-600 hover:bg-blue-500 rounded-xl transition-colors shadow-lg shadow-blue-500/20"
            >
              {isPlaying ? <Pause className="w-6 h-6" /> : <Play className="w-6 h-6 fill-current" />}
            </button>
            
            <div className="w-px h-8 bg-white/20 mx-2" />
            
            {[1, 10, 100, 1000].map((speed) => (
              <button 
                key={speed}
                onClick={() => setWarpSpeed(speed)}
                className={`px-4 py-2 rounded-lg font-mono text-sm transition-colors ${warpSpeed === speed ? 'bg-indigo-500/30 text-indigo-300 border border-indigo-500/50' : 'hover:bg-white/10 text-slate-400 border border-transparent'}`}
              >
                {speed}x
              </button>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
