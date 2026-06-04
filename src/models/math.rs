pub type BodyId = u64;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vec3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3d {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn norm_squared(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn norm(self) -> f64 {
        self.norm_squared().sqrt()
    }

    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n <= 1e-12 {
            Self {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            }
        } else {
            Self {
                x: self.x / n,
                y: self.y / n,
                z: self.z / n,
            }
        }
    }
}
