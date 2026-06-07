use crate::util::math::Vec3d;

#[derive(Debug, Clone, PartialEq, Copy)]
pub enum BodyId {
    EARTH,
    MOON,
}

#[derive(Debug, Clone, Copy)]
pub struct Body {
    pub body_id: BodyId,

    pub mu_km3_s2: f64,
    pub position_km: Vec3d,
    pub radius_km: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Rocket {
    pub id: i64,
    pub position_km: Vec3d,
    pub velocity_km: Vec3d,
}
