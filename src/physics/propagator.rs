// src/physics/propagator.rs
use crate::models::math::Vec3d;
use crate::physics::gravity::acceleration_from_earth;

pub type State = (Vec3d, Vec3d);

pub fn rk4_step(state: &mut State, dt: f64) {
    let (pos, vel) = state;
    let derivatives = |p: Vec3d, v: Vec3d| -> (Vec3d, Vec3d) {
        let a = acceleration_from_earth(p);
        (v, a)
    };
    let (p0, v0) = (*pos, *vel);
    let (k1_p, k1_v) = derivatives(p0, v0);
    let (k2_p, k2_v) = derivatives(p0 + k1_p * (dt/2.0), v0 + k1_v * (dt/2.0));
    let (k3_p, k3_v) = derivatives(p0 + k2_p * (dt/2.0), v0 + k2_v * (dt/2.0));
    let (k4_p, k4_v) = derivatives(p0 + k3_p * dt, v0 + k3_v * dt);
    let new_pos = p0 + (k1_p + k2_p*2.0 + k3_p*2.0 + k4_p) * (dt/6.0);
    let new_vel = v0 + (k1_v + k2_v*2.0 + k3_v*2.0 + k4_v) * (dt/6.0);
    *pos = new_pos;
    *vel = new_vel;
}

pub fn propagate_trajectory(initial_state: State, dt: f64, duration: f64) -> Vec<State> {
    let mut states = Vec::new();
    let mut state = initial_state;
    let mut t = 0.0;
    while t <= duration {
        states.push(state);
        rk4_step(&mut state, dt);
        t += dt;
    }
    states
}