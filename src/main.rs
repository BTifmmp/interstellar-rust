use crate::render::camera::CameraController;
use crate::render::drawing::{
    draw_body, draw_hud, draw_rocket, draw_rocket_trajectory, draw_vec_trajectory,
};
use crate::render::mouse::update_mouse_lock;
use crate::simulation::world::{generate_moon_trajectory, generate_rocket_trajectory};
use macroquad::prelude::*;
mod render;
mod simulation;
mod util;
use simulation::world::SimulationWorld;
use space_dust::bodies::Earth; // Adjusted to match your space_dust path

use crate::simulation::objects::Rocket;
use crate::util::math::Vec3d;
use chrono::Utc;

#[macroquad::main("Orbital Simulation 3D")]
async fn main() {
    let start_time = Utc::now();
    let mut world = SimulationWorld::with_epoch(start_time);

    // 2. Configure our rocket parameters
    let velocity = Vec3d::new(2.0, 9.0, 3.0);

    let test_rocket = Rocket {
        id: 0,
        position_km: Vec3d::new(Earth::EQUATORIAL_RADIUS_KM + 450.0, 0.0, 0.0),
        velocity_km: velocity,
    };

    let duration_s = 86400.0 * 3.0;
    let snapshot_dt_s = 100.0;
    let dt_s = 10.0;

    println!("Calculating rocket trajectory paths...");
    let trajectory =
        generate_rocket_trajectory(&test_rocket, start_time, duration_s, dt_s, snapshot_dt_s);
    println!("Trajectory mapped! Rendered points: {}", trajectory.len());

    let moon_traj = generate_moon_trajectory(start_time, duration_s * 4.0);

    let mut collided = false;
    for state in &trajectory {
        let distance_from_center = state.position_km.norm();

        if distance_from_center < Earth::EQUATORIAL_RADIUS_KM {
            let elapsed_hours = state.time / 3600.0;
            println!(
                "⚠️ COLLISION DETECTED! Rocket crashed into Earth at T + {:.2} hours.",
                elapsed_hours
            );
            println!(
                "   Penetration Depth: {:.2} km below surface.",
                Earth::EQUATORIAL_RADIUS_KM - distance_from_center
            );
            collided = true;
            break;
        }
    }

    if !collided {
        println!("✅ Trajectory clean! No Earth collisions detected within the simulation window.");
    }

    let mut cam_controller = CameraController::new(Vec3d::new(0.0, 0.0, 45000.0));

    let mut mouse_flag: bool = false;

    loop {
        match update_mouse_lock() {
            Some(should_lock) => mouse_flag = should_lock,
            None => {} // FIXED: Enters an empty block to safely do nothing
        }

        if (mouse_flag) {
          cam_controller.update();
        }


        let live_dt = 60.0;
        world.step(live_dt);

        clear_background(BLACK);

        for body in &world.bodies {
            draw_body(&cam_controller.camera, body, BLUE);
        }

        draw_vec_trajectory(&cam_controller.camera, &moon_traj, DARKGRAY);

        draw_rocket_trajectory(&cam_controller.camera, &trajectory, GRAY);

        let current_elapsed = (world.epoch - start_time).num_seconds() as f64;
        let target_index = (current_elapsed / snapshot_dt_s) as usize;

        if let Some(current_state) = trajectory.get(target_index) {
            draw_rocket(&cam_controller.camera, current_state.position_km, YELLOW);
        }

        draw_hud(world.epoch);

        next_frame().await;
    }
}
