pub type BodyId = u64;
use std::ops::{Add, Sub, Mul, AddAssign};

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

impl Add for Vec3d {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Vec3d {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vec3d {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Vec3d {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f64> for Vec3d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Vec3d {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

// Mnożenie skalar * wektor (dla wygody)
impl Mul<Vec3d> for f64 {
    type Output = Vec3d;
    fn mul(self, vec: Vec3d) -> Vec3d {
        vec * self
    }
}

impl AddAssign for Vec3d {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}