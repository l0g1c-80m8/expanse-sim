use bevy_ecs::prelude::*;
use zmq::{Context, Socket, Message};
use iceoryx2::prelude::*;
use crate::components::RigidBody;
use crate::systems::SimTime;

#[derive(Resource)]
pub struct AutonomyBridge {
    zmq_ctx: Context,
    sync_socket: Socket,
}

impl AutonomyBridge {
    pub fn new() -> Self {
        let zmq_ctx = Context::new();
        let sync_socket = zmq_ctx.socket(zmq::REP).expect("Failed to create ZMQ REP socket");
        sync_socket.bind("tcp://*:5555").expect("Failed to bind ZMQ socket");

        Self {
            zmq_ctx,
            sync_socket,
        }
    }

    /// Blocks and waits for the external autonomy stack to acknowledge the current simulation step.
    /// In a Request-Reply pattern, the autonomy stack sends "ACK" or an actuator Wrench command.
    pub fn wait_for_autonomy_step(&self, sim_time: f64) {
        let mut msg = Message::new();
        // Wait for incoming command (blocking)
        self.sync_socket.recv(&mut msg, 0).expect("ZMQ recv failed");
        
        // Process command (e.g., apply Wrench to ECS commands queue)
        // For now, we just reply with the current sim time to unblock the client.
        let reply = format!("SIM_TIME: {}", sim_time);
        self.sync_socket.send(&reply, 0).expect("ZMQ send failed");
    }
}

/// ECS System that acts as the IPC lockstep barrier at the end of every physics tick.
pub fn lockstep_sync_system(
    bridge: Res<AutonomyBridge>,
    sim_time: Res<SimTime>,
) {
    // 100% Deterministic execution: we will not proceed until the autonomy stack has processed this tick.
    bridge.wait_for_autonomy_step(sim_time.time);
}

// In a full implementation with feature = "synthetic_sensors", Iceoryx2 would be configured here
// to publish zero-copy PointCloud/Camera image blobs.
#[cfg(feature = "synthetic_sensors")]
pub fn synthetic_sensor_publish_system(
    _query: Query<&RigidBody>
) {
    // TODO: Publish high bandwidth arrays over iceoryx2 SHM
}
