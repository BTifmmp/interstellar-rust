use crate::simulation::objects::{Body, MoonState, RocketState};
use crate::simulation::propagator::rk4_step;
use crate::util::math::Vec3d;
use chrono::{DateTime, Utc};
use space_dust::bodies::{Earth, Moon};

pub struct TrajectoryGenerator {
    pub moon_trajectory: Vec<MoonState>,
    duration_s: f64,
    dt_s: f64,
}

impl TrajectoryGenerator {
    pub fn new(duration_s: f64, dt_s: f64) -> Self {
        Self::with_epoch(Utc::now(), duration_s, dt_s)
    }

    pub fn with_epoch(start_time: DateTime<Utc>, duration_s: f64, dt_s: f64) -> Self {
        let moon_trajectory = precompute_moon_states(start_time, duration_s, dt_s);

        Self {
            moon_trajectory,
            duration_s,
            dt_s,
        }
    }

    pub fn generate_rocket_trajectory(&self, rocket: RocketState) -> Vec<RocketState> {
        let earth = Body {
            mu_km3_s2: Earth::MU_KM,
            position_km: Vec3d::new(0.0, 0.0, 0.0),
        };

        let mut moon = Body {
            mu_km3_s2: Moon::MU / 1e9,
            position_km: Vec3d::new(0.0, 0.0, 0.0),
        };

        let mut trajectory: Vec<RocketState> = Vec::with_capacity(self.moon_trajectory.len());
        let mut rocket_cp = rocket.clone();

        for &moon_state in &self.moon_trajectory {
            moon.position_km = moon_state.position_km;
            let bodies = [earth, moon];
            rk4_step(&mut rocket_cp, &bodies, self.dt_s);

            trajectory.push(RocketState {
                time: moon_state.time,
                position_km: rocket_cp.position_km,
                velocity_km: rocket_cp.velocity_km,
            });
        }

        trajectory
    }
}

pub fn simplify_trajectory<T: Copy>(vec: &[T], every_nth: usize) -> Vec<T> {
    vec.iter()
        .enumerate()
        .filter(|(i, _)| i % every_nth == 0)
        .map(|(_, val)| *val)
        .collect()
}

pub fn precompute_moon_states(
    start_epoch: DateTime<Utc>,
    durations_s: f64,
    dt_s: f64,
) -> Vec<MoonState> {
    let num_snapshots = (durations_s / dt_s).floor() as usize + 1;

    let mut states = Vec::with_capacity(num_snapshots);

    for i in 0..num_snapshots {
        let t = i as f64 * dt_s;
        let epoch = start_epoch + chrono::Duration::milliseconds((t * 1000.0) as i64);
        let pos = Moon::eci_position_km(&epoch);
        let epoch_next = epoch + chrono::Duration::seconds(1);
        let pos_next = Moon::eci_position_km(&epoch_next);

        let vel = Vec3d::new(pos_next.x - pos.x, pos_next.y - pos.y, pos_next.z - pos.z);

        states.push(MoonState {
            time: t,
            position_km: Vec3d::new(pos.x, pos.y, pos.z),
            velocity_km_s: vel,
        });
    }

    states
}
