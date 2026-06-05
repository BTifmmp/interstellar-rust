// src/physics/gravity.rs
use crate::models::math::Vec3d;

/// Stała grawitacyjna Ziemi (km^3/s^2)
pub const MU_EARTH: f64 = 3.986004e5;

/// Przyspieszenie od Ziemi (układ geocentryczny)
pub fn acceleration_from_earth(ship_pos: Vec3d) -> Vec3d {
    let r_sq = ship_pos.norm_squared();
    let r = r_sq.sqrt();
    if r < 1e-5 {
        return Vec3d::new(0.0, 0.0, 0.0);
    }
    let factor = -MU_EARTH / (r_sq * r);
    Vec3d::new(ship_pos.x * factor, ship_pos.y * factor, ship_pos.z * factor)
}