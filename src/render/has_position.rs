use crate::simulation::objects::{RocketState, MoonState};
use crate::util::math::Vec3d;

pub trait HasPosition {
    fn get_pos(&self) -> Vec3d;
}

impl HasPosition for RocketState {
    fn get_pos(&self) -> Vec3d {
        self.position_km
    }
}
impl HasPosition for MoonState {
    fn get_pos(&self) -> Vec3d {
        self.position_km
    }
}
impl HasPosition for Vec3d {
    fn get_pos(&self) -> Vec3d {
        *self
    }
}
