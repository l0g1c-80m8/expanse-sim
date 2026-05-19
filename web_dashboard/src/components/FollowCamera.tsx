'use client';

import { useEffect, useMemo, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';
import {
  metersToSceneUnits,
  type BodySnapshot,
} from '@/lib/telemetry';

export type FocusTarget = number | 'ship' | 'sun' | null;

/**
 * Pins the OrbitControls target onto a chosen body or the spacecraft every
 * frame. The camera keeps its current offset from the target so the user
 * stays in control of zoom and angle. Lives inside the R3F `<Canvas>` —
 * it uses `useFrame` and `useThree`, which only work in that context.
 */
export function FollowCamera({
  target,
  bodies,
  spacecraftPosition,
}: {
  target: FocusTarget;
  bodies: BodySnapshot[];
  spacecraftPosition: [number, number, number] | null;
}) {
  const { camera, controls } = useThree() as unknown as {
    camera: THREE.PerspectiveCamera;
    controls: { target: THREE.Vector3; update: () => void } | null;
  };
  const lastTarget = useRef<THREE.Vector3>(new THREE.Vector3());
  const firstFrame = useRef<boolean>(true);

  const desired = useMemo<[number, number, number] | null>(() => {
    if (target == null) return null;
    if (target === 'sun') return [0, 0, 0];
    if (target === 'ship') return spacecraftPosition;
    const body = bodies.find((b) => b.id === target);
    return body ? metersToSceneUnits(body.position) : null;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [target, bodies, spacecraftPosition]);

  // Reset the camera offset whenever the focus selection changes so the
  // first frame after a re-target lands the camera at a sensible distance.
  useEffect(() => {
    firstFrame.current = true;
  }, [target]);

  useFrame(() => {
    if (!desired || !controls) return;
    const next = new THREE.Vector3(desired[0], desired[1], desired[2]);
    if (firstFrame.current) {
      const dist = camera.position.distanceTo(controls.target);
      const minDist = target === 'ship' ? 0.3 : target === 'sun' ? 1.5 : 0.6;
      const useDist = Math.max(dist, minDist);
      const dir = camera.position.clone().sub(controls.target).normalize();
      camera.position.copy(next).addScaledVector(dir, useDist);
      firstFrame.current = false;
    } else {
      const delta = next.clone().sub(controls.target);
      camera.position.add(delta);
    }
    controls.target.copy(next);
    lastTarget.current.copy(next);
    controls.update();
  });

  return null;
}
