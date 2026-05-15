'use client';

import { useMemo } from 'react';
import { Sphere } from '@react-three/drei';
import { metersToSceneUnits, radiusToSceneUnits } from '@/lib/telemetry';
import type { BodySnapshot, SpacecraftSnapshot } from '@/lib/telemetry';

const COLORS: Record<string, string> = {
  Sun: '#fcd34d',
  Mercury: '#9ca3af',
  Venus: '#f5d0a9',
  Earth: '#3b82f6',
  Mars: '#ef4444',
  Jupiter: '#fbbf24',
  Saturn: '#facc15',
  Uranus: '#67e8f9',
  Neptune: '#60a5fa',
};

interface Props {
  bodies: BodySnapshot[];
  spacecraft: SpacecraftSnapshot | null;
}

export function SolarSystem({ bodies, spacecraft }: Props) {
  const bodyMeshes = useMemo(() => {
    return bodies.map((b) => {
      const pos = metersToSceneUnits(b.position);
      const isSun = b.name === 'Sun';
      // Sun gets a smaller exaggeration so it doesn't swallow Mercury / Venus.
      const r = isSun ? radiusToSceneUnits(b.radius, 6) : radiusToSceneUnits(b.radius, 1500);
      const color = COLORS[b.name] ?? '#cbd5e1';
      return (
        <group key={b.id} position={pos}>
          <Sphere args={[Math.max(r, 0.02), 32, 32]}>
            {isSun ? (
              <meshBasicMaterial color={color} />
            ) : (
              <meshStandardMaterial color={color} roughness={0.7} metalness={0.15} />
            )}
          </Sphere>
        </group>
      );
    });
  }, [bodies]);

  const ship = useMemo(() => {
    if (!spacecraft) return null;
    const p = metersToSceneUnits(spacecraft.position);
    return (
      <group position={p}>
        <Sphere args={[0.04, 16, 16]}>
          <meshStandardMaterial color="#22d3ee" emissive="#0ea5e9" emissiveIntensity={0.4} />
        </Sphere>
        {/* Thrust vector indicator */}
        <ThrustArrow spacecraft={spacecraft} />
      </group>
    );
  }, [spacecraft]);

  return (
    <>
      {bodyMeshes}
      {ship}
    </>
  );
}

function ThrustArrow({ spacecraft }: { spacecraft: SpacecraftSnapshot }) {
  const [tx, ty, tz] = spacecraft.thrust_command;
  const mag = Math.hypot(tx, ty, tz);
  if (mag < 1e-6) return null;
  const scale = Math.min(0.5, Math.log10(1 + mag) / 12);
  const dir = [tx / mag, ty / mag, tz / mag] as const;
  return (
    <mesh position={[dir[0] * scale * 0.6, dir[1] * scale * 0.6, dir[2] * scale * 0.6]}>
      <coneGeometry args={[0.02, scale, 12]} />
      <meshStandardMaterial color="#f97316" emissive="#fb923c" emissiveIntensity={0.6} />
    </mesh>
  );
}
