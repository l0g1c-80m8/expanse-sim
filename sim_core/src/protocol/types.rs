//! JSON wire types shared by every transport.
//!
//! The dashboard's TypeScript types in `web_dashboard/src/lib/telemetry.ts`
//! mirror these structs field-for-field. Both `sim_server` (axum +
//! WebSocket) and `sim_wasm` (in-browser worker) serialise these types
//! verbatim — there is no second protocol.

use serde::{Deserialize, Serialize};

use crate::nav::NavEstimate;
use crate::sensors::SensorPack;

/// Player / dashboard command pushed over the transport.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type")]
pub enum ControlCommand {
    #[serde(rename = "set_warp")]
    SetWarp { warp: f64 },
    #[serde(rename = "set_paused")]
    SetPaused { paused: bool },
    #[serde(rename = "set_thrust")]
    SetThrust { thrust: [f64; 3] },
    #[serde(rename = "set_drive")]
    SetDrive { drive: String },
    #[serde(rename = "set_attitude_rate")]
    SetAttitudeRate { angular_velocity: [f64; 3] },
    #[serde(rename = "set_mission")]
    SetMission {
        source: Option<i32>,
        target: Option<i32>,
    },
    /// Re-spawn the spacecraft on the source body's orbit.
    #[serde(rename = "stage_at_source")]
    StageAtSource,
    #[serde(rename = "set_thrust_mode")]
    SetThrustMode { mode: String, body: Option<i32> },
    #[serde(rename = "set_thrust_magnitude")]
    SetThrustMagnitude { magnitude: f64 },
    /// Engage / disengage the brachistochrone rendezvous autopilot.
    #[serde(rename = "set_autopilot")]
    SetAutopilot {
        engaged: bool,
        accel_g: Option<f64>,
    },
    #[serde(rename = "set_sensor_config")]
    SetSensorConfig {
        enabled: Option<bool>,
        range_relative_sigma: Option<f64>,
        range_absolute_sigma_m: Option<f64>,
        range_rate_sigma_m_s: Option<f64>,
        bearing_sigma_rad: Option<f64>,
        accel_sigma_m_s2: Option<f64>,
        gyro_sigma_rad_s: Option<f64>,
        star_tracker_sigma_rad: Option<f64>,
        light_time_delay: Option<bool>,
    },
    #[serde(rename = "set_mode")]
    SetMode { mode: String },
    /// One-shot mission kickoff: set mode → stage → engage in one transaction.
    /// `warp` optionally bumps the time-warp at engagement so the operator
    /// doesn't have to watch a 3-day transit at 60× warp.
    #[serde(rename = "start_mission")]
    StartMission {
        source: Option<i32>,
        target: Option<i32>,
        accel_g: Option<f64>,
        warp: Option<f64>,
    },
    #[serde(rename = "set_nav_filter")]
    SetNavFilter {
        enabled: bool,
        init_sigma_r_m: Option<f64>,
        init_sigma_v_m_s: Option<f64>,
    },
    #[serde(rename = "reset")]
    Reset,
}

/// One broadcasted frame of telemetry.
#[derive(Debug, Clone, Serialize)]
pub struct TelemetryFrame {
    pub sim_time: f64,
    pub warp: f64,
    pub paused: bool,
    pub bodies: Vec<BodySnapshot>,
    pub spacecraft: Option<SpacecraftSnapshot>,
    pub mission: MissionSnapshot,
    pub thrust_controller: ThrustControllerSnapshot,
    pub autopilot: AutopilotSnapshot,
    pub sensors: SensorPack,
    pub nav: NavEstimate,
    pub mode: String,
    pub tick: u64,
    /// Effective sim-seconds advanced per wall-second over the last second.
    pub effective_warp: f64,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct MissionSnapshot {
    pub source: Option<i32>,
    pub target: Option<i32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ThrustControllerSnapshot {
    pub mode: String,
    pub body: Option<i32>,
    pub magnitude: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct AutopilotSnapshot {
    pub engaged: bool,
    pub phase: String,
    pub accel_g: f64,
    pub range_m: f64,
    pub closing_m_s: f64,
    pub eta_s: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct BodySnapshot {
    pub id: i32,
    pub name: String,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub radius: f64,
    /// Gravitational parameter μ = G·M (m³/s²). Exposed so the dashboard can
    /// run a client-side forecast for the predicted-trajectory overlay.
    pub mu: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct SpacecraftSnapshot {
    pub id: u32,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude: [f64; 4], // x,y,z,w
    pub angular_velocity: [f64; 3],
    pub mass: f64,
    pub propellant_mass: f64,
    pub thrust_command: [f64; 3],
    pub drive: String,
    pub isp: f64,
    pub max_thrust: f64,
}
