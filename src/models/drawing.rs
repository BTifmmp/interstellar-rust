use macroquad::prelude::Vec2;

pub trait Drawable {
    fn draw(&self, camera: &DrawCamera);
}

#[derive(Debug, Clone, Copy)]
pub struct DrawCamera {
    pub center: Vec2,
    pub scale_px_per_km: f64,
}
