//! Autonomy ↔ simulator IPC bridge.
//!
//! Two transports, each gated behind its own feature so the core remains
//! buildable on bare machines:
//!
//! * `zmq_bridge`  → ZeroMQ REQ/REP lockstep barrier carrying small messages
//!                   (telemetry digest + commanded wrench). Used by ROS 2
//!                   control nodes via a thin adapter on the other side.
//! * `iceoryx`     → Iceoryx2 zero-copy shared memory for high-bandwidth
//!                   sensor publishing (point clouds, camera blobs).
//!
//! The bridge is held in the ECS as a `Resource`. If neither feature is
//! enabled, a no-op stub keeps the rest of the schedule deterministic.

use bevy_ecs::prelude::*;
use serde::{Deserialize, Serialize};

use crate::clock::SimTime;
use crate::components::{CommandedWrench, RigidBody, Spacecraft};

/// Telemetry sent to the autonomy stack each tick. Kept small so the
/// lockstep barrier doesn't dominate the tick budget at high warp.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TickTelemetry {
    pub sim_time: f64,
    pub spacecraft_id: u32,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude: [f64; 4], // x, y, z, w
    pub angular_velocity: [f64; 3],
    pub mass: f64,
}

/// Wrench commanded by the autonomy stack in reply to telemetry. Force and
/// torque live in the inertial frame.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CommandPacket {
    pub force: [f64; 3],
    pub torque: [f64; 3],
}

/// Top-level lockstep mode. The driver instantiates exactly one variant.
#[derive(Resource)]
pub enum AutonomyBridge {
    /// No autonomy connected; the simulator runs free. Useful for tests and
    /// for the web dashboard's "demo" mode.
    Disabled,
    #[cfg(feature = "zmq_bridge")]
    Zmq(zmq_impl::ZmqBridge),
}

impl Default for AutonomyBridge {
    fn default() -> Self {
        AutonomyBridge::Disabled
    }
}

impl AutonomyBridge {
    /// Exchange one tick's telemetry for a command packet. Blocks if the
    /// underlying transport is lockstep; returns immediately when disabled.
    pub fn step(&mut self, _tele: &TickTelemetry) -> CommandPacket {
        match self {
            AutonomyBridge::Disabled => CommandPacket::default(),
            #[cfg(feature = "zmq_bridge")]
            AutonomyBridge::Zmq(b) => b.step(_tele),
        }
    }
}

/// Compose telemetry from the first `Spacecraft`-tagged entity. The autonomy
/// harness is single-vehicle for now; extend with a per-vehicle channel when
/// we onboard fleet scenarios.
pub fn lockstep_sync_system(
    sim_time: Res<SimTime>,
    mut bridge: ResMut<AutonomyBridge>,
    mut q_sc: Query<(&Spacecraft, &RigidBody, &mut CommandedWrench)>,
) {
    let Some((sc, rb, mut wrench)) = q_sc.iter_mut().next() else {
        return;
    };
    let tele = TickTelemetry {
        sim_time: sim_time.time,
        spacecraft_id: sc.id,
        position: rb.position.to_array(),
        velocity: rb.velocity.to_array(),
        attitude: [rb.attitude.x, rb.attitude.y, rb.attitude.z, rb.attitude.w],
        angular_velocity: rb.angular_velocity.to_array(),
        mass: rb.mass,
    };
    let reply = bridge.step(&tele);
    wrench.force = glam::DVec3::from_array(reply.force);
    wrench.torque = glam::DVec3::from_array(reply.torque);
}

#[cfg(feature = "zmq_bridge")]
pub mod zmq_impl {
    use super::*;

    pub struct ZmqBridge {
        _ctx: zmq::Context,
        socket: zmq::Socket,
    }

    impl ZmqBridge {
        pub fn bind(endpoint: &str) -> Result<Self, zmq::Error> {
            let ctx = zmq::Context::new();
            let socket = ctx.socket(zmq::REP)?;
            socket.bind(endpoint)?;
            Ok(Self { _ctx: ctx, socket })
        }

        pub fn step(&mut self, tele: &TickTelemetry) -> CommandPacket {
            let mut msg = zmq::Message::new();
            // Wait for autonomy to send its request (commanded wrench, JSON).
            if self.socket.recv(&mut msg, 0).is_err() {
                return CommandPacket::default();
            }
            let cmd: CommandPacket = msg
                .as_str()
                .and_then(|s| serde_json::from_str(s).ok())
                .unwrap_or_default();
            // Reply with this tick's telemetry.
            let body = serde_json::to_string(tele).unwrap_or_else(|_| "{}".into());
            let _ = self.socket.send(&body, 0);
            cmd
        }
    }
}

#[cfg(feature = "iceoryx")]
pub mod iceoryx_impl {
    //! Zero-copy sensor publishing over Iceoryx2 SHM. Wires up when
    //! `--features synthetic_sensors,iceoryx` are both active.
    //!
    //! Real point-cloud / image payloads are projected to land on a separate
    //! service from telemetry so high-bandwidth bursts can't stall the
    //! lockstep barrier.
    use super::*;

    pub struct IceoryxPublisher {
        // Reserved for forward expansion: build the service once during init
        // and republish on every synthetic-sensor tick.
    }

    impl IceoryxPublisher {
        pub fn new() -> Self {
            Self {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};

    #[test]
    fn disabled_bridge_returns_zero_wrench() {
        let mut bridge = AutonomyBridge::Disabled;
        let tele = TickTelemetry {
            sim_time: 0.0,
            spacecraft_id: 0,
            position: [0.0; 3],
            velocity: [0.0; 3],
            attitude: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0; 3],
            mass: 1000.0,
        };
        let cmd = bridge.step(&tele);
        assert_eq!(cmd.force, [0.0; 3]);
        assert_eq!(cmd.torque, [0.0; 3]);
    }

    #[test]
    fn lockstep_system_zeros_wrench_when_no_command() {
        let mut world = World::new();
        world.insert_resource(SimTime::default());
        world.insert_resource(AutonomyBridge::Disabled);
        let id = world
            .spawn((
                Spacecraft { id: 1 },
                RigidBody {
                    velocity: DVec3::new(1.0, 0.0, 0.0),
                    attitude: DQuat::IDENTITY,
                    ..Default::default()
                },
                CommandedWrench {
                    force: DVec3::new(100.0, 0.0, 0.0),
                    torque: DVec3::ZERO,
                },
            ))
            .id();
        let mut schedule = Schedule::default();
        schedule.add_systems(lockstep_sync_system);
        schedule.run(&mut world);
        let w = world.entity(id).get::<CommandedWrench>().unwrap();
        assert_eq!(w.force, DVec3::ZERO);
    }
}
