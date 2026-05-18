//! Mission descriptor: source → target transit for the autonomy stack.
//!
//! The simulator does not plan the trajectory itself — that's the autonomy
//! stack's job. This resource is the world-state slot the autonomy stack
//! reads to know "where am I trying to go, from where". It's also broadcast
//! to dashboards so operators can see the mission visually.
//!
//! The `source` is most useful as a marker for the autonomy planner; the
//! actual current state of the spacecraft is always authoritative via its
//! `RigidBody` component. The source body is what was originally selected
//! as "departure" — typically the planet whose orbit the spacecraft started
//! near.

use bevy_ecs::prelude::*;
use serde::{Deserialize, Serialize};

/// Departure / arrival selection. NAIF body IDs index into `EphemerisCache`.
#[derive(Resource, Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Mission {
    /// Source body (NAIF id). `None` if the spacecraft launched from free space
    /// or the operator hasn't picked one yet.
    pub source_body: Option<i32>,
    /// Target body (NAIF id). `None` if no destination is set.
    pub target_body: Option<i32>,
}

impl Mission {
    pub fn new(source: i32, target: i32) -> Self {
        Self {
            source_body: Some(source),
            target_body: Some(target),
        }
    }

    pub fn clear(&mut self) {
        self.source_body = None;
        self.target_body = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_mission_is_empty() {
        let m = Mission::default();
        assert!(m.source_body.is_none());
        assert!(m.target_body.is_none());
    }

    #[test]
    fn new_sets_both_endpoints() {
        let m = Mission::new(399, 499); // Earth → Mars
        assert_eq!(m.source_body, Some(399));
        assert_eq!(m.target_body, Some(499));
    }

    #[test]
    fn clear_drops_both() {
        let mut m = Mission::new(399, 499);
        m.clear();
        assert!(m.source_body.is_none());
        assert!(m.target_body.is_none());
    }
}
