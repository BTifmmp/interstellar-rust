use crate::simulation::objects::Body;
use crate::simulation::world::RocketState;
use crate::util::math::Vec3d;
use crate::{render::camera::DrawCamera, simulation::world::MoonState};
use chrono::{DateTime, Utc};
use macroquad::prelude::*;

pub fn draw_rocket(camera: &DrawCamera, position: Vec3d, color: Color) {
    if let Some(screen_pos) = camera.world_to_screen(position) {
        draw_circle(screen_pos.x, screen_pos.y, 4.0, color);
    }
}

pub fn draw_body(camera: &DrawCamera, body: &Body, color: Color) {
    if let Some(screen_pos) = camera.world_to_screen(body.position_km) {
        // Calculate distance to scale the radius properly in 3D
        let dist = (body.position_km - camera.position_km).norm();
        let screen_h = screen_height() as f64;

        // Apparent size = (Real Size / Distance) * FOV * ScreenHeight
        let radius = ((body.radius_km / dist) * camera.fov * screen_h).max(2.0) as f32;

        draw_circle(screen_pos.x, screen_pos.y, radius, color);
    }
}

pub fn draw_rocket_trajectory(camera: &DrawCamera, trajectory: &[RocketState], color: Color) {
    let mut last_screen_pos: Option<Vec2> = None;

    for state in trajectory {
        if let Some(current_screen_pos) = camera.world_to_screen(state.position_km) {
            if let Some(prev_pos) = last_screen_pos {
                draw_line(
                    prev_pos.x,
                    prev_pos.y,
                    current_screen_pos.x,
                    current_screen_pos.y,
                    1.0,
                    color,
                );
            }
            last_screen_pos = Some(current_screen_pos);
        } else {
            last_screen_pos = None;
        }
    }
}

pub fn draw_moon_trajectory(camera: &DrawCamera, trajectory: &[MoonState], color: Color) {
    let mut last_screen_pos: Option<Vec2> = None;

    for moon in trajectory {
        if let Some(current_screen_pos) = camera.world_to_screen(moon.position_km) {
            if let Some(prev_pos) = last_screen_pos {
                draw_line(
                    prev_pos.x,
                    prev_pos.y,
                    current_screen_pos.x,
                    current_screen_pos.y,
                    1.0,
                    color,
                );
            }
            last_screen_pos = Some(current_screen_pos);
        } else {
            last_screen_pos = None;
        }
    }
}

pub fn draw_vec_trajectory(camera: &DrawCamera, trajectory: &[Vec3d], color: Color) {
    let mut last_screen_pos: Option<Vec2> = None;

    for pos in trajectory {
        if let Some(current_screen_pos) = camera.world_to_screen(*pos) {
            if let Some(prev_pos) = last_screen_pos {
                draw_line(
                    prev_pos.x,
                    prev_pos.y,
                    current_screen_pos.x,
                    current_screen_pos.y,
                    1.0,
                    color,
                );
            }
            last_screen_pos = Some(current_screen_pos);
        } else {
            last_screen_pos = None;
        }
    }
}

pub fn draw_hud(time: DateTime<Utc>) {
    draw_text(
        &format!("Date: {}", time.format("%Y-%m-%d %H:%M:%S")),
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
}
