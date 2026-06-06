use crate::render::camera::CameraController;
use crate::render::drawing::{draw_body, draw_rocket, draw_trajectory};
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

    let moon_traj = generate_moon_trajectory(start_time, duration_s);

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
    set_cursor_grab(true);
    show_mouse(false);
    let mut update_mouse: bool = false;

    loop {
        if is_key_pressed(KeyCode::Escape) {
            set_cursor_grab(false);
            show_mouse(true);
            update_mouse = false;
        }

        if is_mouse_button_pressed(MouseButton::Left) {
            set_cursor_grab(true);
            show_mouse(false);
            update_mouse = true;
        }

        if update_mouse {
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

        // Draw basic HUD metadata info
        draw_text(
            &format!("Date: {}", world.epoch.format("%Y-%m-%d %H:%M:%S")),
            10.0,
            20.0,
            18.0,
            WHITE,
        );
        draw_text(
            "Controls: WASD to Move | Mouse to Look | ESC to Release Mouse",
            10.0,
            40.0,
            14.0,
            GRAY,
        );

        next_frame().await;
    }
}
