/**
 * Client-side trajectory forecast for the visual overlay.
 *
 * We forward-integrate the spacecraft under Sun-only gravity (the dominant
 * term well outside planetary spheres of influence) using a leapfrog
 * (kick-drift-kick) integrator. That's good enough to draw a plausible
 * predicted path; it's not what the autonomy stack is consuming.
 *
 * Returns a flat `[x0,y0,z0, x1,y1,z1, …]` array in metres.
 */
export function forecastTrajectory(
  position: [number, number, number],
  velocity: [number, number, number],
  durationSec: number,
  samples: number = 400,
  muSun = 1.327_124_400_18e20,
): Float32Array {
  const dt = durationSec / samples;
  let [x, y, z] = position;
  let [vx, vy, vz] = velocity;
  const out = new Float32Array((samples + 1) * 3);
  out[0] = x;
  out[1] = y;
  out[2] = z;
  for (let i = 1; i <= samples; i++) {
    const r2 = x * x + y * y + z * z;
    const r = Math.sqrt(r2);
    if (r < 1) break; // collided with Sun — bail out
    const k = -muSun / (r2 * r);
    const ax = k * x;
    const ay = k * y;
    const az = k * z;

    // Leapfrog: half-kick, drift, half-kick.
    vx += 0.5 * ax * dt;
    vy += 0.5 * ay * dt;
    vz += 0.5 * az * dt;
    x += vx * dt;
    y += vy * dt;
    z += vz * dt;
    const r2b = x * x + y * y + z * z;
    const rb = Math.sqrt(r2b);
    const kb = -muSun / (r2b * rb);
    vx += 0.5 * kb * x * dt;
    vy += 0.5 * kb * y * dt;
    vz += 0.5 * kb * z * dt;

    out[i * 3] = x;
    out[i * 3 + 1] = y;
    out[i * 3 + 2] = z;
  }
  return out;
}
