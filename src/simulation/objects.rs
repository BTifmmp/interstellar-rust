use crate::util::math::Vec3d;

#[derive(Debug, Clone, Copy)]
pub struct Body {
    pub mu_km3_s2: f64,
    pub position_km: Vec3d,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RocketState {
    pub time: f64,
    pub position_km: Vec3d,
    pub velocity_km: Vec3d,
}

#[derive(Debug, Clone, Copy)]
pub struct MoonState {
    pub time: f64, // sekundy od start_epoch
    pub position_km: Vec3d,
    pub velocity_km_s: Vec3d,
}
