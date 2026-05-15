pub mod components;
pub mod systems;
pub mod ephemeris;

pub mod ipc;

use bevy_ecs::prelude::*;
use systems::{SimTime, rigid_body_kinematics_system, propulsion_system};
use ipc::{AutonomyBridge, lockstep_sync_system};

#[cfg(feature = "thermodynamics")]
use systems::thermodynamics_system;

/// The central physics plant encapsulating the ECS World and Schedule
pub struct ExpanseSim {
    pub world: World,
    pub schedule: Schedule,
}

impl ExpanseSim {
    pub fn new() -> Self {
        let mut world = World::new();
        world.insert_resource(SimTime::default());
        world.insert_resource(AutonomyBridge::new());

        let mut schedule = Schedule::default();
        
        // Propulsion updates velocity/mass, then kinematics updates position/attitude
        schedule.add_systems((
            propulsion_system,
            rigid_body_kinematics_system,
            lockstep_sync_system, // Syncs with external node via ZeroMQ
        ).chain());

        #[cfg(feature = "thermodynamics")]
        schedule.add_systems(thermodynamics_system);

        Self { world, schedule }
    }

    /// Advances the simulation by one discrete `dt` tick.
    pub fn tick(&mut self) {
        self.schedule.run(&mut self.world);
        // Advance sim time
        let mut sim_time = self.world.resource_mut::<SimTime>();
        sim_time.time += sim_time.dt;
    }
}
