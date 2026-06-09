use macroquad::prelude::*;

use crate::util::math::Vec3d;

#[derive(Debug, Clone, Copy)]
pub struct DrawCamera {
    pub position_km: Vec3d, // Where the camera is in the world (km)
    pub yaw: f64,           // Rotation left/right (radians)s
    pub pitch: f64,         // Rotation up/down (radians)
    pub fov: f64,           // Field of View zoom factor
}

impl DrawCamera {
    pub fn front(&self) -> Vec3d {
        Vec3d::new(
            self.yaw.cos() * self.pitch.cos(),
            self.pitch.sin(),
            self.yaw.sin() * self.pitch.cos(),
        )
        .normalized()
    }

    pub fn right(&self) -> Vec3d {
        let front = self.front();
        let world_up = Vec3d::new(0.0, 1.0, 0.0);

        Vec3d::new(
            front.y * world_up.z - front.z * world_up.y,
            front.z * world_up.x - front.x * world_up.z,
            front.x * world_up.y - front.y * world_up.x,
        )
        .normalized()
    }

    pub fn world_to_screen(&self, world_pos: Vec3d) -> Option<Vec2> {
        let rel = world_pos - self.position_km;

        let front = self.front();
        let right = self.right();

        let up = Vec3d::new(
            right.y * front.z - right.z * front.y,
            right.z * front.x - right.x * front.z,
            right.x * front.y - right.y * front.x,
        );

        let local_x = rel.x * right.x + rel.y * right.y + rel.z * right.z;
        let local_y = rel.x * up.x + rel.y * up.y + rel.z * up.z;
        let local_z = rel.x * front.x + rel.y * front.y + rel.z * front.z;

        if local_z < 0.1 {
            return None;
        }

        let projected_x = (local_x / local_z) * self.fov;
        let projected_y = (local_y / local_z) * self.fov;

        let screen_w = screen_width() as f32;
        let screen_h = screen_height() as f32;

        Some(vec2(
            (projected_x as f32 * screen_w) + (screen_w / 2.0),
            (-projected_y as f32 * screen_h) + (screen_h / 2.0),
        ))
    }
}

pub struct CameraController {
    pub camera: DrawCamera,
    pub move_speed: f64,
    pub sensitivity: f64,
}

impl CameraController {
    pub fn new(start_pos: Vec3d) -> Self {
        Self {
            camera: DrawCamera {
                position_km: start_pos,
                yaw: -std::f64::consts::PI / 2.0,
                pitch: 0.0,
                fov: 1.0,
            },
            move_speed: 50_000.0,
            sensitivity: 1.0,
        }
    }

    pub fn update(&mut self) {
        let front = self.camera.front();
        let right = self.camera.right();
        let dt = get_frame_time() as f64;
        let frame_speed = self.move_speed * dt;

        // 1. WASD Movement
        if is_key_down(KeyCode::W) {
            self.camera.position_km += front * frame_speed;
        }
        if is_key_down(KeyCode::S) {
            self.camera.position_km -= front * frame_speed;
        }
        if is_key_down(KeyCode::D) {
            self.camera.position_km += right * frame_speed;
        }
        if is_key_down(KeyCode::A) {
            self.camera.position_km -= right * frame_speed;
        }

        let delta = mouse_delta_position();

        self.camera.yaw -= delta.x as f64 * self.sensitivity;
        self.camera.pitch += delta.y as f64 * self.sensitivity;

        let limit = 89.0_f64.to_radians();
        self.camera.pitch = self.camera.pitch.clamp(-limit, limit);

        let wheel = mouse_wheel().1;
        if wheel != 0.0 {
            self.camera.fov *= if wheel > 0.0 { 1.1 } else { 0.9 };
        }
    }
}
