'use client';

import { useEffect, useMemo, useRef, useState } from 'react';

/**
 * Append-only ring buffer of recent positions. Used by the SolarSystem
 * view to draw the spacecraft's recent ground-truth track as a fading
 * line. State is held client-side (server doesn't keep history) so the
 * trail naturally drops when you reset / reload.
 */
export function useBreadcrumbs(
  position: [number, number, number] | null,
  /** Tick number — used to deduplicate frames at very low warp. */
  tick: number,
  capacity: number = 600,
): Float32Array {
  const buf = useRef<number[]>([]);
  const lastTick = useRef<number>(-1);
  const [version, setVersion] = useState(0);

  useEffect(() => {
    if (!position) return;
    if (tick === lastTick.current) return;
    lastTick.current = tick;
    buf.current.push(position[0], position[1], position[2]);
    const max = capacity * 3;
    if (buf.current.length > max) {
      buf.current.splice(0, buf.current.length - max);
    }
    setVersion((v) => (v + 1) & 0xffff);
  }, [position, tick, capacity]);

  // Snapshot the ring buffer whenever it mutates so the BufferAttribute
  // downstream knows to re-upload. We memo on `version` so we don't churn
  // a new Float32Array on every unrelated render.
  return useMemo(() => new Float32Array(buf.current), [version]);
}
