// src/algo/objective.rs
use crate::models::math::Vec3d;
use crate::physics::propagator::propagate_trajectory;

/// Stałe orbity Księżyca (uproszczona, kołowa)
pub const MOON_ORBIT_RADIUS: f64 = 384_400.0;   // km
pub const MOON_PERIOD: f64 = 27.3 * 86400.0;    // sekundy
pub const MOON_ANGULAR_SPEED: f64 = 2.0 * std::f64::consts::PI / MOON_PERIOD; // rad/s

/// Pozycja Księżyca w układzie geocentrycznym w chwili t (sekwundy od startu)
pub fn moon_position(t: f64) -> Vec3d {
    let angle = MOON_ANGULAR_SPEED * t;
    Vec3d::new(angle.cos() * MOON_ORBIT_RADIUS, angle.sin() * MOON_ORBIT_RADIUS, 0.0)
}

/// Prędkość Księżyca w układzie geocentrycznym (pochodna pozycji)
pub fn moon_velocity(t: f64) -> Vec3d {
    let angle = MOON_ANGULAR_SPEED * t;
    let vx = -MOON_ANGULAR_SPEED * MOON_ORBIT_RADIUS * angle.sin();
    let vy = MOON_ANGULAR_SPEED * MOON_ORBIT_RADIUS * angle.cos();
    Vec3d::new(vx, vy, 0.0)
}

pub fn cost_function(
    params: &[f64],
    max_duration: f64,
    dt: f64,
    weights: (f64, f64, f64),
) -> f64 {
    let (vx, vy, vz, dx, dy, dz) = (params[0], params[1], params[2], params[3], params[4], params[5]);

    // Stan początkowy statku (układ geocentryczny)
    let initial_state = (Vec3d::new(dx, dy, dz), Vec3d::new(vx, vy, vz));
    let trajectory = propagate_trajectory(initial_state, dt, max_duration);

    let mut best_dist = f64::INFINITY;
    let mut end_speed = 0.0;
    let mut t = 0.0;
    for (pos, vel) in trajectory {
        let moon_pos = moon_position(t);
        let dist = (pos.x - moon_pos.x).hypot(pos.y - moon_pos.y).hypot(pos.z - moon_pos.z);
        if dist < best_dist {
            best_dist = dist;
            let moon_vel = moon_velocity(t);
            end_speed = (vel.x - moon_vel.x).hypot(vel.y - moon_vel.y).hypot(vel.z - moon_vel.z);
        }
        t += dt;
    }

    let start_speed = (vx*vx + vy*vy + vz*vz).sqrt();
    let (w_dist, w_start, w_end) = weights;
    w_dist * best_dist + w_start * start_speed + w_end * end_speed
}

/// Generuje trajektorię (tylko pozycje) dla wizualizacji
pub fn generate_trajectory(params: &[f64], max_duration: f64, dt: f64) -> Vec<Vec3d> {
    let (vx, vy, vz, dx, dy, dz) = (params[0], params[1], params[2], params[3], params[4], params[5]);
    let initial_state = (Vec3d::new(dx, dy, dz), Vec3d::new(vx, vy, vz));
    let states = propagate_trajectory(initial_state, dt, max_duration);
    states.into_iter().map(|(pos, _)| pos).collect()
}