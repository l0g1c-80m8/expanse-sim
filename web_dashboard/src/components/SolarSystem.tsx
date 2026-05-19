'use client';

import { useMemo, useState } from 'react';
import { Html, Line } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import {
  distanceAU,
  metersToSceneUnits,
  visualRadius,
  AU,
  type BodySnapshot,
  type MissionSnapshot,
  type NavEstimate,
  type SpacecraftSnapshot,
} from '@/lib/telemetry';
import { forecastTrajectory } from '@/lib/trajectory';
import type { ViewSettings } from '@/lib/viewSettings';

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

const LABEL_COLORS: Record<string, string> = {
  Sun: 'text-amber-300',
  Mercury: 'text-slate-400',
  Venus: 'text-amber-200',
  Earth: 'text-blue-300',
  Luna: 'text-slate-300',
  Mars: 'text-red-300',
  Phobos: 'text-orange-200',
  Deimos: 'text-orange-200',
  Jupiter: 'text-amber-300',
  Saturn: 'text-yellow-300',
  Uranus: 'text-cyan-300',
  Neptune: 'text-blue-300',
};

/**
 * Moons of the default body roster. Heliocentric orbit rings don't make
 * sense for these (they orbit a planet, not the Sun) and their labels
 * collide with the parent's at solar-system zooms.
 */
const MOON_PARENTS: Record<string, string> = {
  Luna: 'Earth',
  Phobos: 'Mars',
  Deimos: 'Mars',
};

const isMoon = (name: string) => Object.prototype.hasOwnProperty.call(MOON_PARENTS, name);

interface Props {
  bodies: BodySnapshot[];
  spacecraft: SpacecraftSnapshot | null;
  mission?: MissionSnapshot;
  view: ViewSettings;
  nav?: NavEstimate | null;
  breadcrumbs?: Float32Array;
  onBodyClick?: (bodyId: number) => void;
}

export function SolarSystem({
  bodies,
  spacecraft,
  mission,
  view,
  nav,
  breadcrumbs,
  onBodyClick,
}: Props) {
  const source =
    mission?.source != null
      ? bodies.find((b) => b.id === mission.source) ?? null
      : null;
  const target =
    mission?.target != null
      ? bodies.find((b) => b.id === mission.target) ?? null
      : null;

  return (
    <>
      {bodies.map((b) => (
        <Body
          key={b.id}
          body={b}
          showOrbit={view.showOrbits}
          showLabel={view.showLabels}
          highlight={
            source?.id === b.id
              ? 'source'
              : target?.id === b.id
              ? 'target'
              : null
          }
          onClick={onBodyClick ? () => onBodyClick(b.id) : undefined}
        />
      ))}

      {source && target && <TransitLine source={source} target={target} />}

      {breadcrumbs && breadcrumbs.length >= 6 && <Breadcrumbs points={breadcrumbs} />}

      {spacecraft && (
        <Ship
          spacecraft={spacecraft}
          showLabel={view.showLabels}
          showTrajectory={view.showTrajectory}
          trajectoryHorizonSec={view.trajectoryHorizonSec}
        />
      )}

      {nav?.initialized && spacecraft && (
        <GhostShip nav={nav} truth={spacecraft} />
      )}
    </>
  );
}

function Breadcrumbs({ points }: { points: Float32Array }) {
  // Convert each metres-triple into a Vector3 tuple in scene coords. drei's
  // <Line> uses meshline under the hood so `lineWidth` actually works
  // (raw three.js `<line>` ignores width on most platforms).
  const scenePoints = useMemo<[number, number, number][]>(() => {
    if (points.length < 6) return [];
    const out: [number, number, number][] = [];
    for (let i = 0; i < points.length; i += 3) {
      out.push([points[i] / AU, points[i + 2] / AU, -points[i + 1] / AU]);
    }
    return out;
  }, [points]);
  if (scenePoints.length < 2) return null;
  return (
    <Line
      points={scenePoints}
      color="#fb923c"
      lineWidth={2.5}
      transparent
      opacity={0.8}
      depthWrite={false}
    />
  );
}

function GhostShip({
  nav,
  truth,
}: {
  nav: NavEstimate;
  truth: SpacecraftSnapshot;
}) {
  const pos = metersToSceneUnits(nav.position);
  const truthPos = metersToSceneUnits(truth.position);
  // 1-σ position uncertainty as a wireframe sphere around the estimate.
  // Clamp the visual radius so a wildly diverged filter doesn't swallow
  // the whole scene.
  const sigmaScene = Math.min(nav.position_sigma_m / AU, 0.5);

  return (
    <group>
      <group position={pos}>
        <mesh>
          <sphereGeometry args={[0.018, 16, 16]} />
          <meshStandardMaterial
            color="#a78bfa"
            emissive="#7c3aed"
            emissiveIntensity={0.5}
            transparent
            opacity={0.55}
          />
        </mesh>
        {sigmaScene > 0.01 && (
          <mesh>
            <sphereGeometry args={[sigmaScene, 24, 24]} />
            <meshBasicMaterial color="#a78bfa" transparent opacity={0.08} wireframe />
          </mesh>
        )}
      </group>
      <Line
        points={[pos, truthPos]}
        color="#a78bfa"
        lineWidth={2}
        transparent
        opacity={0.9}
        depthWrite={false}
      />
    </group>
  );
}

function Body({
  body,
  showOrbit,
  showLabel,
  highlight,
  onClick,
}: {
  body: BodySnapshot;
  showOrbit: boolean;
  showLabel: boolean;
  highlight: 'source' | 'target' | null;
  onClick?: () => void;
}) {
  const pos = metersToSceneUnits(body.position);
  const radius = visualRadius(body.name);
  const color = COLORS[body.name] ?? '#cbd5e1';
  const isSun = body.name === 'Sun';
  const moon = isMoon(body.name);
  const distance = distanceAU(body.position);
  const highlightColor =
    highlight === 'source' ? '#34d399' : highlight === 'target' ? '#fb7185' : null;

  // Hide moon labels when the camera is too far away for the moon to be
  // angularly distinct from its parent. The threshold is a multiple of the
  // body's visual radius — close in, you see "Luna"; zoomed out to the
  // solar system, it's hidden so the planet's label stays readable.
  const [labelVisible, setLabelVisible] = useState(!moon);
  useFrame(({ camera }) => {
    if (!moon) return;
    const dx = camera.position.x - pos[0];
    const dy = camera.position.y - pos[1];
    const dz = camera.position.z - pos[2];
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    // Show the moon label when camera is within ~0.05 AU (Earth–Luna ≈
    // 0.003 AU; this gives a comfortable zoom-in window).
    const next = dist < 0.05;
    if (next !== labelVisible) setLabelVisible(next);
  });

  return (
    <group>
      {/* Skip orbit rings for moons — they orbit a planet, not the Sun, so
         a ring at their heliocentric radius is misleading clutter. */}
      {showOrbit && !isSun && !moon && <OrbitRing radius={distance} color={color} />}

      <group position={pos}>
        {highlightColor && (
          <mesh rotation-x={-Math.PI / 2}>
            <ringGeometry args={[radius * 1.8, radius * 2.2, 48]} />
            <meshBasicMaterial color={highlightColor} transparent opacity={0.9} />
          </mesh>
        )}
        <mesh
          onClick={(e) => {
            if (!onClick) return;
            e.stopPropagation();
            onClick();
          }}
          onPointerOver={(e) => {
            if (!onClick) return;
            e.stopPropagation();
            document.body.style.cursor = 'pointer';
          }}
          onPointerOut={() => {
            if (!onClick) return;
            document.body.style.cursor = '';
          }}
        >
          <sphereGeometry args={[radius, 48, 48]} />
          {isSun ? (
            <meshBasicMaterial color={color} toneMapped={false} />
          ) : (
            <meshStandardMaterial
              color={color}
              roughness={0.65}
              metalness={0.1}
              emissive={color}
              emissiveIntensity={0.15}
            />
          )}
        </mesh>

        {isSun && (
          <mesh>
            <sphereGeometry args={[radius * 1.6, 32, 32]} />
            <meshBasicMaterial
              color="#fbbf24"
              transparent
              opacity={0.18}
              toneMapped={false}
            />
          </mesh>
        )}

        {showLabel && labelVisible && (
          <Html
            position={[0, radius + 0.04, 0]}
            center
            distanceFactor={isSun ? 10 : moon ? 1.2 : 6}
            zIndexRange={[10, 0]}
            style={{ pointerEvents: 'none', userSelect: 'none' }}
          >
            <div
              className={`px-1.5 py-0.5 rounded text-[10px] font-mono whitespace-nowrap bg-black/40 backdrop-blur-sm border border-white/10 ${
                LABEL_COLORS[body.name] ?? 'text-slate-200'
              }`}
            >
              {body.name}
              {!isSun && (
                <span className="text-slate-500 ml-1.5">
                  {distance.toFixed(2)} AU
                </span>
              )}
            </div>
          </Html>
        )}
      </group>
    </group>
  );
}

function TransitLine({
  source,
  target,
}: {
  source: BodySnapshot;
  target: BodySnapshot;
}) {
  const a = metersToSceneUnits(source.position);
  const b = metersToSceneUnits(target.position);
  return (
    <Line
      points={[a, b]}
      color="#f472b6"
      lineWidth={2}
      dashed
      dashSize={0.2}
      gapSize={0.12}
      transparent
      opacity={0.85}
      depthWrite={false}
    />
  );
}

function OrbitRing({ radius, color }: { radius: number; color: string }) {
  const points = useMemo(() => {
    const arr: number[] = [];
    const n = 256;
    for (let i = 0; i <= n; i++) {
      const t = (i / n) * Math.PI * 2;
      arr.push(Math.cos(t) * radius, 0, Math.sin(t) * radius);
    }
    return new Float32Array(arr);
  }, [radius]);

  return (
    <line>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          args={[points, 3]}
          count={points.length / 3}
        />
      </bufferGeometry>
      <lineBasicMaterial color={color} transparent opacity={0.18} />
    </line>
  );
}

function TrajectoryLine({
  spacecraft,
  horizonSec,
}: {
  spacecraft: SpacecraftSnapshot;
  horizonSec: number;
}) {
  // Forecast in metres → convert each sample into scene units (AU + axis swap).
  const points = useMemo<[number, number, number][]>(() => {
    const raw = forecastTrajectory(
      spacecraft.position,
      spacecraft.velocity,
      horizonSec,
      400,
    );
    const out: [number, number, number][] = [];
    for (let i = 0; i < raw.length; i += 3) {
      out.push([raw[i] / AU, raw[i + 2] / AU, -raw[i + 1] / AU]);
    }
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    spacecraft.position[0],
    spacecraft.position[1],
    spacecraft.position[2],
    spacecraft.velocity[0],
    spacecraft.velocity[1],
    spacecraft.velocity[2],
    horizonSec,
  ]);
  if (points.length < 2) return null;
  return (
    <Line
      points={points}
      color="#22d3ee"
      lineWidth={2}
      dashed
      dashSize={0.08}
      gapSize={0.04}
      transparent
      opacity={0.9}
      depthWrite={false}
    />
  );
}

function Ship({
  spacecraft,
  showLabel,
  showTrajectory,
  trajectoryHorizonSec,
}: {
  spacecraft: SpacecraftSnapshot;
  showLabel: boolean;
  showTrajectory: boolean;
  trajectoryHorizonSec: number;
}) {
  const pos = metersToSceneUnits(spacecraft.position);
  const distance = distanceAU(spacecraft.position);
  const [tx, ty, tz] = spacecraft.thrust_command;
  const thrustMag = Math.hypot(tx, ty, tz);
  const thrusting = thrustMag > 1.0;
  const speed = Math.hypot(...spacecraft.velocity);

  const heading = useMemo(() => {
    const [vx, vy, vz] = spacecraft.velocity;
    const vScene = new THREE.Vector3(vx, vz, -vy);
    if (vScene.lengthSq() < 1e-9) return new THREE.Quaternion();
    vScene.normalize();
    return new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      vScene,
    );
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [spacecraft.velocity[0], spacecraft.velocity[1], spacecraft.velocity[2]]);

  return (
    <>
      {showTrajectory && (
        <TrajectoryLine spacecraft={spacecraft} horizonSec={trajectoryHorizonSec} />
      )}
      <group position={pos}>
        {/* Selection ring (always visible). */}
        <mesh rotation-x={-Math.PI / 2}>
          <ringGeometry args={[0.025, 0.032, 32]} />
          <meshBasicMaterial color="#22d3ee" transparent opacity={0.85} />
        </mesh>

        {/* Hull, oriented along velocity. */}
        <group quaternion={[heading.x, heading.y, heading.z, heading.w]}>
          <mesh>
            <coneGeometry args={[0.012, 0.04, 12]} />
            <meshStandardMaterial
              color="#67e8f9"
              emissive="#06b6d4"
              emissiveIntensity={0.9}
              roughness={0.3}
            />
          </mesh>
          {thrusting && (
            <mesh position={[0, -0.04, 0]} rotation-x={Math.PI}>
              <coneGeometry args={[0.01, 0.08, 12]} />
              <meshBasicMaterial color="#fb923c" transparent opacity={0.7} toneMapped={false} />
            </mesh>
          )}
        </group>

        {/* Velocity arrow — green, in scene-units of 0.001/(km/s) clamped. */}
        <VectorArrow
          inertial={spacecraft.velocity}
          color="#10b981"
          scale={0.0008}
          maxLen={0.4}
        />

        {/* Thrust arrow — orange, only when thrusting. */}
        {thrusting && (
          <VectorArrow
            inertial={spacecraft.thrust_command}
            color="#fb923c"
            scale={1.0e-7}
            maxLen={0.4}
            label={`${thrustPretty(thrustMag)}`}
          />
        )}

        {showLabel && (
          // Offset diagonally up-right of the hull so the label never
          // covers the ship itself. distanceFactor=12 makes the label
          // shrink quickly when the camera moves in.
          <Html
            position={[0.07, 0.07, 0]}
            distanceFactor={12}
            zIndexRange={[10, 0]}
            style={{ pointerEvents: 'none', userSelect: 'none' }}
          >
            <div className="px-2 py-0.5 rounded bg-cyan-500/20 border border-cyan-400/60 text-[10px] font-mono whitespace-nowrap text-cyan-200 backdrop-blur-sm">
              <span className="font-bold tracking-wider">ROCINANTE</span>
              <div className="text-[9px] text-slate-400">
                {distance.toFixed(2)} AU · {(speed / 1000).toFixed(1)} km/s
              </div>
            </div>
          </Html>
        )}
      </group>
    </>
  );
}

/**
 * Render a vector as an arrow originating from the spacecraft (parent group
 * origin). `inertial` is in J2000 m or N; the component remaps to scene
 * axes and scales by `scale`, clamped to `maxLen`.
 */
function VectorArrow({
  inertial,
  color,
  scale,
  maxLen,
  label,
}: {
  inertial: [number, number, number];
  color: string;
  scale: number;
  maxLen: number;
  label?: string;
}) {
  const mag = Math.hypot(...inertial);
  if (mag < 1e-12) return null;
  // J2000 → scene axes (same remap as positions).
  const scene = new THREE.Vector3(inertial[0], inertial[2], -inertial[1])
    .normalize();
  const len = Math.min(mag * scale, maxLen);
  const head = scene.clone().multiplyScalar(len);
  const points = useMemo(
    () => new Float32Array([0, 0, 0, head.x, head.y, head.z]),
    [head.x, head.y, head.z],
  );
  // Cone at the tip
  const q = useMemo(
    () =>
      new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), scene),
    [scene.x, scene.y, scene.z],
  );
  return (
    <>
      <line>
        <bufferGeometry>
          <bufferAttribute
            attach="attributes-position"
            args={[points, 3]}
            count={2}
          />
        </bufferGeometry>
        <lineBasicMaterial color={color} linewidth={2} />
      </line>
      <group
        position={[head.x, head.y, head.z]}
        quaternion={[q.x, q.y, q.z, q.w]}
      >
        <mesh>
          <coneGeometry args={[0.008, 0.02, 10]} />
          <meshBasicMaterial color={color} />
        </mesh>
        {label && (
          <Html
            position={[0, 0.025, 0]}
            distanceFactor={20}
            style={{ pointerEvents: 'none', userSelect: 'none' }}
          >
            <div
              className="px-1 py-0.5 rounded text-[9px] font-mono whitespace-nowrap"
              style={{ color, background: 'rgba(0,0,0,0.4)' }}
            >
              {label}
            </div>
          </Html>
        )}
      </group>
    </>
  );
}

function thrustPretty(n: number): string {
  if (n >= 1e6) return `${(n / 1e6).toFixed(1)} MN`;
  if (n >= 1e3) return `${(n / 1e3).toFixed(1)} kN`;
  return `${n.toFixed(0)} N`;
}
