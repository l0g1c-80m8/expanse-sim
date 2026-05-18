/**
 * Telemetry frame schema — must stay in lock-step with
 * `sim_server::sim_runner::TelemetryFrame`.
 *
 * Position / velocity are in SI (meters, meters/second) heliocentric J2000.
 */

export interface BodySnapshot {
  id: number;
  name: string;
  position: [number, number, number];
  velocity: [number, number, number];
  radius: number;
  mu: number;
}

export type ThrustModeId =
  | 'off'
  | 'body_x'
  | 'prograde'
  | 'retrograde'
  | 'toward_target'
  | 'away_from_target'
  | 'toward_source'
  | 'away_from_source'
  | 'toward_body'
  | 'away_from_body';

export interface ThrustControllerSnapshot {
  mode: ThrustModeId;
  body: number | null;
  magnitude: number;
}

export interface SpacecraftSnapshot {
  id: number;
  position: [number, number, number];
  velocity: [number, number, number];
  attitude: [number, number, number, number];
  angular_velocity: [number, number, number];
  mass: number;
  propellant_mass: number;
  thrust_command: [number, number, number];
  drive: 'conventional' | 'brachistochrone';
  isp: number;
  max_thrust: number;
}

export interface MissionSnapshot {
  source: number | null;
  target: number | null;
}

export type AutopilotPhase = 'idle' | 'boost' | 'brake' | 'arrived' | 'hold';

export interface AutopilotSnapshot {
  engaged: boolean;
  phase: AutopilotPhase;
  accel_g: number;
  range_m: number;
  closing_m_s: number;
  eta_s: number;
}

export interface RangeReading {
  body_id: number;
  range_m: number;
  range_rate_m_s: number;
  bearing_inertial: [number, number, number];
}

export interface ImuReading {
  specific_force_body: [number, number, number];
  angular_rate_body: [number, number, number];
}

export interface AttitudeReading {
  q_xyzw: [number, number, number, number];
}

export interface SensorPack {
  ranges: RangeReading[];
  imu: ImuReading | null;
  attitude: AttitudeReading | null;
}

export interface TelemetryFrame {
  sim_time: number;
  warp: number;
  paused: boolean;
  bodies: BodySnapshot[];
  spacecraft: SpacecraftSnapshot | null;
  mission: MissionSnapshot;
  thrust_controller: ThrustControllerSnapshot;
  autopilot: AutopilotSnapshot;
  sensors: SensorPack;
  tick: number;
  effective_warp: number;
}

export type ControlCommand =
  | { type: 'set_warp'; warp: number }
  | { type: 'set_paused'; paused: boolean }
  | { type: 'set_thrust'; thrust: [number, number, number] }
  | { type: 'set_thrust_mode'; mode: ThrustModeId; body?: number | null }
  | { type: 'set_thrust_magnitude'; magnitude: number }
  | { type: 'set_drive'; drive: 'conventional' | 'brachistochrone' }
  | { type: 'set_attitude_rate'; angular_velocity: [number, number, number] }
  | { type: 'set_mission'; source: number | null; target: number | null }
  | { type: 'stage_at_source' }
  | { type: 'set_autopilot'; engaged: boolean; accel_g?: number }
  | {
      type: 'set_sensor_config';
      enabled?: boolean;
      range_relative_sigma?: number;
      range_absolute_sigma_m?: number;
      range_rate_sigma_m_s?: number;
      bearing_sigma_rad?: number;
      accel_sigma_m_s2?: number;
      gyro_sigma_rad_s?: number;
      star_tracker_sigma_rad?: number;
    }
  | { type: 'reset' };

// Re-exported for components that need to convert raw metres themselves
// (e.g. the trajectory forecast).
export const AU = 1.495_978_707e11;

/**
 * Convert J2000 ecliptic metres into Three.js scene units.
 *
 * Two changes vs a naive `m / AU`:
 *  - Axis remap: J2000 ecliptic XY → Three.js XZ floor, J2000 +Z (ecliptic
 *    north) → Three.js +Y. The ecliptic plane ends up lying flat on the
 *    floor so a default camera that looks down sees planets spread out.
 *  - AU normalisation so the whole solar system fits in ~30 scene units.
 */
export function metersToSceneUnits(m: [number, number, number]): [number, number, number] {
  return [m[0] / AU, m[2] / AU, -m[1] / AU];
}

/** Heliocentric distance in AU. */
export function distanceAU(m: [number, number, number]): number {
  return Math.hypot(m[0], m[1], m[2]) / AU;
}

/**
 * Per-body fixed visual radius in scene units.
 *
 * Real radii at solar-system scale make every planet sub-pixel, so we use
 * non-physical sizes tuned for legibility. The Sun is largest, gas giants
 * next, rocky planets last — same visual ordering as the real thing.
 */
export const VISUAL_RADIUS_AU: Record<string, number> = {
  Sun: 0.18,
  Mercury: 0.025,
  Venus: 0.04,
  Earth: 0.045,
  Mars: 0.035,
  Jupiter: 0.11,
  Saturn: 0.095,
  Uranus: 0.075,
  Neptune: 0.075,
};

export function visualRadius(name: string): number {
  return VISUAL_RADIUS_AU[name] ?? 0.03;
}
