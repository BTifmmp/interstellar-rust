// src/algo/objective.rs
use crate::algo::config::Config;
use crate::simulation::objects::{MoonState, RocketState};
use crate::simulation::world::TrajectoryGenerator;
use crate::util::geometry::{
    earth_rotation_velocity, enu_vector_to_cartesian, geographic_to_cartesian,
};
use crate::util::math::Vec3d;
use space_dust::bodies::{Earth, Moon};

fn compute_start_state(params: &[f64], config: &Config) -> (Vec3d, Vec3d) {
    let (vx, vy, vz, dx, dy, dz) = (
        params[0], params[1], params[2], params[3], params[4], params[5],
    );
    let earth_radius = Earth::EQUATORIAL_RADIUS_KM;
    let start_radius = earth_radius + config.start_point.altitude_km;

    let base_pos = geographic_to_cartesian(
        config.start_point.latitude_deg,
        config.start_point.longitude_deg,
        start_radius,
    );
    let offset = enu_vector_to_cartesian(base_pos, dx, dy, dz);
    let start_pos = base_pos + offset;
    let start_vel = enu_vector_to_cartesian(base_pos, vx, vy, vz);
    let rotation_vel = earth_rotation_velocity(base_pos);
    let start_vel = start_vel + rotation_vel;
    (start_pos, start_vel)
}

fn analyze_trajectory(
    trajectory: &[RocketState],
    moon_states: &[MoonState],
    config: &Config,
) -> (f64, f64, bool, bool) {
    let earth_radius = Earth::EQUATORIAL_RADIUS_KM;
    let moon_radius = Moon::RADIUS / 1000.0;
    let target_offset = geographic_to_cartesian(
        config.target_point.latitude_deg,
        config.target_point.longitude_deg,
        moon_radius + config.target_point.altitude_km,
    );

    let mut best_dist = f64::INFINITY;
    let mut end_speed = 0.0;
    let mut collided_earth = false;
    let mut collided_moon = false;

    for (i, state) in trajectory.iter().enumerate() {
        if i >= moon_states.len() {
            break;
        }
        let moon = &moon_states[i];

        // Kolizja z Ziemią → koniec, ogromna kara
        if state.position_km.norm() < earth_radius {
            collided_earth = true;
            break;
        }

        // Kolizja z Księżycem → koniec, ale bez dodatkowej kary (sukces)
        let target_pos = moon.position_km + target_offset;
        let dist = (state.position_km - target_pos).norm();
        let dist_to_moon_center = (state.position_km - moon.position_km).norm();
        if dist_to_moon_center < moon_radius {
            collided_moon = true;
            // minimalna odległość staje się 0, ale zachowujemy prędkość w momencie zderzenia
            best_dist = dist;
            let rel_vel = state.velocity_km - moon.velocity_km_s;
            end_speed = rel_vel.norm();
            break;
        }
        if dist < best_dist {
            best_dist = dist;
            let rel_vel = state.velocity_km - moon.velocity_km_s;
            end_speed = rel_vel.norm();
        }
    }

    (best_dist, end_speed, collided_earth, collided_moon)
}

/// 3. Główna funkcja kosztu (optymalizowana przez PSO)
pub fn cost_function(params: &[f64], config: &Config, traj_gen: &TrajectoryGenerator) -> f64 {
    let (start_pos, start_vel) = compute_start_state(params, config);
    let rocket = RocketState {
        time: 0.0,
        position_km: start_pos,
        velocity_km: start_vel,
    };

    let trajectory = traj_gen.generate_rocket_trajectory(rocket);

    let (best_dist, end_speed, collided_earth, _collided_moon) =
        analyze_trajectory(&trajectory, &traj_gen.moon_trajectory, config);

    let start_vel = start_vel.norm();
    let collision_penalty = if collided_earth { 1e9 } else { 0.0 };
    let (w_start, w_end) = (config.weights[0], config.weights[1]);

    (best_dist) * end_speed.powf(w_end) * start_vel.powf(w_start) + collision_penalty
}

pub fn generate_trajectory_for_params(
    params: &[f64],
    config: &Config,
    traj_gen: &TrajectoryGenerator,
) -> Vec<RocketState> {
    let (start_pos, start_vel) = compute_start_state(params, config);
    let rocket = RocketState {
        time: 0.0,
        position_km: start_pos,
        velocity_km: start_vel,
    };
    let traj = traj_gen.generate_rocket_trajectory(rocket);
    traj
}

pub fn get_min_distance(params: &[f64], config: &Config, traj_gen: &TrajectoryGenerator) -> f64 {
    let (start_pos, start_vel) = compute_start_state(params, config);
    let rocket = RocketState {
        time: 0.0,
        position_km: start_pos,
        velocity_km: start_vel,
    };
    let trajectory = traj_gen.generate_rocket_trajectory(rocket);
    let (best_dist, _, _, _) = analyze_trajectory(&trajectory, &traj_gen.moon_trajectory, config);
    best_dist
}
