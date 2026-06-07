use crate::simulation::objects::{Body, BodyId, Rocket};
use crate::simulation::propagator::rk4_step;
use crate::util::math::Vec3d;
use chrono::{DateTime, Utc};
use space_dust::bodies::{Earth, Moon};

pub struct SimulationWorld {
    pub bodies: Vec<Body>,
    pub rockets: Vec<Rocket>,
    pub epoch: DateTime<Utc>,
}

impl SimulationWorld {
    pub fn new() -> Self {
        Self::with_epoch(Utc::now())
    }

    pub fn with_epoch(start_time: DateTime<Utc>) -> Self {
        Self {
            bodies: Self::init_bodies(),
            rockets: Vec::new(),
            epoch: start_time,
        }
    }

    pub fn add_rocket(&mut self, rocket: Rocket) {
        self.rockets.push(rocket);
    }

    pub fn step(&mut self, dt_s: f64) {
        self.epoch += chrono::Duration::milliseconds((dt_s * 1000.0) as i64);

        self.update_bodies();
        self.update_rockets(dt_s);
    }

    fn update_bodies(&mut self) {
        let moon_pos = Moon::eci_position_km(&self.epoch);

        if let Some(moon) = self.bodies.iter_mut().find(|b| b.body_id == BodyId::MOON) {
            moon.position_km = Vec3d::new(moon_pos.x, moon_pos.y, moon_pos.z);
        }
    }

    fn update_rockets(&mut self, dt_s: f64) {
        for rocket in &mut self.rockets {
            rk4_step(rocket, &self.bodies, dt_s);
        }
    }

    fn init_bodies() -> Vec<Body> {
        vec![
            Body {
                body_id: BodyId::EARTH,
                mu_km3_s2: Earth::MU_KM,
                position_km: Vec3d::new(0.0, 0.0, 0.0),
                radius_km: Earth::EQUATORIAL_RADIUS_KM,
            },
            Body {
                body_id: BodyId::MOON,
                mu_km3_s2: Moon::MU / 1e9,
                position_km: Vec3d::new(0.0, 0.0, 0.0),
                radius_km: Moon::RADIUS / 1000.0,
            },
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RocketState {
    pub time: f64,
    pub position_km: Vec3d,
    pub velocity_km: Vec3d,
}

#[derive(Debug, Clone, Copy)]
pub struct MoonState {
    pub time: f64,            // sekundy od start_epoch
    pub position_km: Vec3d,
    pub velocity_km_s: Vec3d,
}

pub fn precompute_moon_states(
    start_epoch: DateTime<Utc>,
    snapshot_dt_s: f64,
    num_snapshots: usize,
) -> Vec<MoonState> {
    let mut states = Vec::with_capacity(num_snapshots);
    for i in 0..num_snapshots {
        let t = i as f64 * snapshot_dt_s;
        let epoch = start_epoch + chrono::Duration::nanoseconds((t * 1_000_000_000.0) as i64);
        let pos = Moon::eci_position_km(&epoch);
        let epoch_next = epoch + chrono::Duration::seconds(1);
        let pos_next = Moon::eci_position_km(&epoch_next);
        let vel = Vec3d::new(
            (pos_next.x - pos.x) / 1.0,
            (pos_next.y - pos.y) / 1.0,
            (pos_next.z - pos.z) / 1.0,
        );
        states.push(MoonState {
            time: t,
            position_km: Vec3d::new(pos.x, pos.y, pos.z),
            velocity_km_s: vel,
        });
    }
    states
}

pub fn generate_rocket_trajectory(
    rocket: &Rocket,
    start_epoch: DateTime<Utc>,
    duration_s: f64,
    dt_s: f64,
    snapshot_dt_s: f64,
) -> Vec<RocketState> {
    let mut world = SimulationWorld::with_epoch(start_epoch);
    let target_id = rocket.id;
    world.add_rocket(rocket.clone());

    let expected_snapshots = (duration_s / dt_s).ceil() as usize + 1;
    let mut trajectory = Vec::with_capacity(expected_snapshots);

    trajectory.push(RocketState {
        time: 0.0, // Relative elapsed time from start_epoch
        position_km: rocket.position_km,
        velocity_km: rocket.velocity_km,
    });

    let mut elapsed = 0.0;
    let mut time_since_last_snapshot = 0.0;

    while elapsed < duration_s {
        world.step(dt_s);
        elapsed += dt_s;
        time_since_last_snapshot += dt_s;

        if time_since_last_snapshot >= snapshot_dt_s {
            if let Some(simulated_rocket) = world.rockets.iter().find(|r| r.id == target_id) {
                trajectory.push(RocketState {
                    time: elapsed,
                    position_km: simulated_rocket.position_km,
                    velocity_km: simulated_rocket.velocity_km,
                });
            }
            time_since_last_snapshot = 0.0;
        }
    }

    trajectory
}
pub fn generate_moon_trajectory(start_epoch: DateTime<Utc>, duration_s: f64) -> Vec<Vec3d> {
    let mut world = SimulationWorld::with_epoch(start_epoch);
    world.step(0.0);

    let hour_step_s = 3600.0;

    let expected_snapshots = (duration_s / hour_step_s).ceil() as usize + 1;
    let mut moon_path = Vec::with_capacity(expected_snapshots);

    if let Some(moon) = world.bodies.iter().find(|b| b.body_id == BodyId::MOON) {
        moon_path.push(moon.position_km);
    }

    let mut elapsed = 0.0;

    while elapsed < duration_s {
        world.step(hour_step_s);
        elapsed += hour_step_s;

        if let Some(moon) = world.bodies.iter().find(|b| b.body_id == BodyId::MOON) {
            moon_path.push(moon.position_km);
        }
    }

    moon_path
}
