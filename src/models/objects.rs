use crate::models::drawing::Drawable;
use crate::models::math::Vec3d;

#[derive(Debug, Clone)]
pub struct Body {
    pub id: i32,

    pub name: String,
    pub color: [f64; 4],
    pub size: f64,

    pub position: Vec3d,
    pub velocity: Vec3d,
}

impl Drawable for Body {
    fn draw(&self, camera: &super::drawing::DrawCamera) {}
}

#[derive(Debug, Clone)]
pub struct Rocket {
    pub id: i32,

    pub color: [f64; 4],
    pub size: f64,
    pub mass_kg: f64,
    
    pub position: Vec3d,
    pub velocity: Vec3d,
}

impl Drawable for Rocket {
    fn draw(&self, camera: &super::drawing::DrawCamera) {}
}