//! Thermodynamics / heat-dissipation model.
//!
//! Optional, gated behind `--features thermodynamics`. Models the
//! spacecraft as a lumped thermal mass — engines pour heat in proportional
//! to thrust × Isp efficiency, and a Stefan-Boltzmann term radiates it to
//! the cosmic-microwave-background sink. Adequate for a "did the drive cook
//! itself?" autonomy check.

#![cfg(feature = "thermodynamics")]

use bevy_ecs::prelude::*;

use crate::clock::SimTime;
use crate::components::{PropulsionDrive, Thermodynamics};

/// Cosmic microwave background temperature (K) — the radiative sink.
pub const T_CMB: f64 = 2.725;
/// Stefan-Boltzmann constant (W·m⁻²·K⁻⁴).
pub const SIGMA: f64 = 5.670_374_419e-8;

pub fn thermodynamics_system(
    sim_time: Res<SimTime>,
    mut q: Query<(&mut Thermodynamics, Option<&PropulsionDrive>)>,
) {
    let dt = sim_time.dt;
    for (mut thermo, drive) in q.iter_mut() {
        let mut heat_in = 0.0;
        if let Some(d) = drive {
            // Power dissipated as heat: a fixed fraction of mechanical power.
            // P_mech = F · v_exhaust = F · Isp · g₀; assume 5% becomes heat.
            let f = d.thrust_command.length();
            let p_mech = f * d.isp * crate::propulsion::G0;
            heat_in = 0.05 * p_mech;
        }
        let t = thermo.temperature.max(T_CMB);
        let p_rad = thermo.emissivity
            * SIGMA
            * thermo.radiative_area
            * (t.powi(4) - T_CMB.powi(4));
        let dt_temp = ((heat_in - p_rad) / thermo.heat_capacity) * dt;
        thermo.temperature = (thermo.temperature + dt_temp).max(T_CMB);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::{PropulsionDrive, PropulsionType, Thermodynamics};
    use glam::DVec3;

    #[test]
    fn no_thrust_drives_temperature_toward_cmb() {
        let mut world = World::new();
        world.insert_resource(SimTime::new(60.0));
        let id = world
            .spawn(Thermodynamics {
                temperature: 1000.0,
                ..Default::default()
            })
            .id();
        let mut schedule = Schedule::default();
        schedule.add_systems(thermodynamics_system);
        for _ in 0..200 {
            schedule.run(&mut world);
        }
        let t = world.entity(id).get::<Thermodynamics>().unwrap();
        assert!(t.temperature < 1000.0, "should radiate away heat");
    }

    #[test]
    fn thrust_heats_up_engine() {
        let mut world = World::new();
        world.insert_resource(SimTime::new(0.1));
        let id = world
            .spawn((
                Thermodynamics {
                    temperature: 300.0,
                    heat_capacity: 1000.0,
                    ..Default::default()
                },
                PropulsionDrive {
                    drive_type: PropulsionType::Brachistochrone,
                    thrust_command: DVec3::new(1_000_000.0, 0.0, 0.0),
                    isp: 10_000.0,
                    propellant_mass: 1.0e6,
                    max_thrust: 1.0e7,
                },
            ))
            .id();
        let mut schedule = Schedule::default();
        schedule.add_systems(thermodynamics_system);
        let t0 = world.entity(id).get::<Thermodynamics>().unwrap().temperature;
        schedule.run(&mut world);
        let t1 = world.entity(id).get::<Thermodynamics>().unwrap().temperature;
        assert!(t1 > t0, "thrust must heat the engine: {} → {}", t0, t1);
    }
}
