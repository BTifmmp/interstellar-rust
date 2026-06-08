use crate::simulation::objects::{Body, RocketState};
use crate::util::math::Vec3d;

pub fn acceleration_at(pos: Vec3d, bodies: &[Body]) -> Vec3d {
    let mut total_acc = Vec3d::new(0.0, 0.0, 0.0);

    for body in bodies {
        let r_vec = body.position_km - pos;
        let dist = r_vec.norm();

        if dist > 0.0 {
            let mag = body.mu_km3_s2 / (dist * dist * dist);
            total_acc += r_vec * mag;
        }
    }
    total_acc
}

pub fn rk4_step(rocket: &mut RocketState, bodies: &[Body], dt: f64) {
    let derivatives = |p: Vec3d, _v: Vec3d| -> (Vec3d, Vec3d) {
        let a = acceleration_at(p, bodies);
        (_v, a)
    };

    let p0 = rocket.position_km;
    let v0 = rocket.velocity_km;

    let (k1_p, k1_v) = derivatives(p0, v0);
    let (k2_p, k2_v) = derivatives(p0 + k1_p * (dt / 2.0), v0 + k1_v * (dt / 2.0));
    let (k3_p, k3_v) = derivatives(p0 + k2_p * (dt / 2.0), v0 + k2_v * (dt / 2.0));
    let (k4_p, k4_v) = derivatives(p0 + k3_p * dt, v0 + k3_v * dt);

    rocket.position_km += (k1_p + (k2_p + k3_p) * 2.0 + k4_p) * (dt / 6.0);
    rocket.velocity_km += (k1_v + (k2_v + k3_v) * 2.0 + k4_v) * (dt / 6.0);
}
