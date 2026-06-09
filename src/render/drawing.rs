use crate::render::camera::DrawCamera;
use crate::render::has_position::HasPosition;
use crate::util::math::Vec3d;
use chrono::{DateTime, Utc};
use macroquad::prelude::*;

pub fn draw_object<T: HasPosition>(camera: &DrawCamera, obj: &T, radius: f64, color: Color) {
    if let Some(screen_pos) = camera.world_to_screen(obj.get_pos()) {
        let dist = (obj.get_pos() - camera.position_km).norm();
        let screen_h = screen_height() as f64;
        let radius = ((radius / dist) * camera.fov * screen_h) as f32;

        draw_circle(screen_pos.x, screen_pos.y, radius, color);
    }
}

pub fn draw_object_static_size<T: HasPosition>(camera: &DrawCamera, obj: &T, radius: f32, color: Color) {
    if let Some(screen_pos) = camera.world_to_screen(obj.get_pos()) {
        draw_circle(screen_pos.x, screen_pos.y, radius, color);
    }
}

pub fn draw_trajectory_with_thickness<T: HasPosition>(
    camera: &DrawCamera,
    trajectory: &[T],
    color: Color,
    thickness: f32,
) {
    let mut last_screen_pos: Option<Vec2> = None;

    for obj in trajectory {
        if let Some(current_screen_pos) = camera.world_to_screen(obj.get_pos()) {
            if let Some(prev_pos) = last_screen_pos {
                draw_line(
                    prev_pos.x,
                    prev_pos.y,
                    current_screen_pos.x,
                    current_screen_pos.y,
                    thickness,
                    color,
                );
            }
            last_screen_pos = Some(current_screen_pos);
        } else {
            last_screen_pos = None;
        }
    }
}

pub fn draw_trajectory<T: HasPosition>(camera: &DrawCamera, trajectory: &[T], color: Color) {
    draw_trajectory_with_thickness(camera, trajectory, color, 1.0);
}

pub fn draw_text_label<T: HasPosition>(
    camera: &DrawCamera,
    obj: &T,
    text: &str,
    font_size: f32,
    offset_y: f32,
    color: Color,
) {
    if let Some(screen_pos) = camera.world_to_screen(obj.get_pos()) {
        draw_text(
            text,
            screen_pos.x,
            screen_pos.y + offset_y,
            font_size,
            color,
        );
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
        "Controls: WASD to Move | Mouse to Look | ESC to Release Mouse | Arrow UP / DOWN to adjust speed",
        10.0,
        40.0,
        14.0,
        GRAY,
    );
}
