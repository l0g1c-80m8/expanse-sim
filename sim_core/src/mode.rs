//! Simulation operating mode — `Sandbox` (live, interactive, gravity-only)
//! vs `Mission` (scripted: source → target executed under autopilot).
//!
//! Both modes share identical physics. The difference is purely about who
//! writes to the propulsion drive:
//!
//! * `Sandbox` — only the operator-thrust controller is allowed to write.
//!   Autopilot is auto-disabled; the ship coasts unless the operator burns.
//!   Useful for orbit study, EKF benchmarking without thrust confounds, and
//!   manual flying.
//!
//! * `Mission` — the autopilot has authority. Operator thrust commands are
//!   ignored while the autopilot is engaged so the planned trajectory plays
//!   out to completion. The host stages the spacecraft at the source body
//!   when entering Mission mode.

use bevy_ecs::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimMode {
    Sandbox,
    Mission,
}

impl Default for SimMode {
    fn default() -> Self {
        SimMode::Sandbox
    }
}

#[derive(Resource, Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct SimModeState {
    pub mode: SimMode,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_sandbox() {
        let state = SimModeState::default();
        assert_eq!(state.mode, SimMode::Sandbox);
    }

    #[test]
    fn modes_round_trip_json() {
        let s = SimModeState { mode: SimMode::Mission };
        let j = serde_json::to_string(&s).unwrap();
        assert!(j.contains("mission"));
        let back: SimModeState = serde_json::from_str(&j).unwrap();
        assert_eq!(back.mode, SimMode::Mission);
    }
}
