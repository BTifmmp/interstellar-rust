use std::sync::Arc;
use nyx_space::cosmic::{MetaAlmanac, Almanac, Epoch, Unit, Frame};
use crate::models::objects::{Body, Rocket};
use crate::models::math::Vec3d;
use nyx_space::dynamics::orbital::OrbitalDynamics;
use anise::constants::celestial_objects::{
    MERCURY, VENUS, EARTH, MARS_BARYCENTER, 
    JUPITER_BARYCENTER, SATURN_BARYCENTER, 
    URANUS_BARYCENTER, NEPTUNE_BARYCENTER, 
    SUN, MOON
};

pub struct SimulationWorld {
    pub current_epoch: Epoch,
    pub almanac: Arc<Almanac>,
    pub bodies: Vec<Body>,
    pub rockets: Vec<Rocket>,
}

impl SimulationWorld {
    pub fn new(start_epoch: Epoch) -> Self {
        let almanac = Arc::new(
            MetaAlmanac::latest().expect("Failed to load almanac")
        );

        let mut world = Self {
            current_epoch: start_epoch,
            almanac,
            bodies: Vec::new(),
            rockets: Vec::new(),
        };

        world.init_solar_system();
        world
    }

    fn init_solar_system(&mut self) {
        let planet_configs = [
            ("Sun",     SUN,                [1.0, 0.9, 0.0, 1.0], 20.0),
            ("Mercury", MERCURY,            [0.7, 0.7, 0.7, 1.0],  3.0),
            ("Venus",   VENUS,              [0.9, 0.8, 0.5, 1.0],  5.0),
            ("Earth",   EARTH,              [0.2, 0.5, 1.0, 1.0],  5.0),
            ("Moon",    MOON,               [0.8, 0.8, 0.8, 1.0],  2.0),
            ("Mars",    MARS_BARYCENTER,               [0.8, 0.3, 0.1, 1.0],  4.0),
            ("Jupiter", JUPITER_BARYCENTER, [0.8, 0.6, 0.4, 1.0], 12.0),
            ("Saturn",  SATURN_BARYCENTER,  [0.9, 0.8, 0.5, 1.0], 10.0),
            ("Uranus",  URANUS_BARYCENTER,  [0.5, 0.8, 0.9, 1.0],  7.0),
            ("Neptune", NEPTUNE_BARYCENTER, [0.2, 0.3, 0.9, 1.0],  7.0),
        ];

        for (name, naif_id, color, size) in planet_configs {
            self.bodies.push(Body {
                id: naif_id,
                name: name.to_string(),
                color,
                size,
                position: Vec3d::new(0.0, 0.0, 0.0),
                velocity: Vec3d::new(0.0, 0.0, 0.0),
            });
        }

        self.update_bodies();
    }

    pub fn step(&mut self, dt_s: f64) {
        self.current_epoch += dt_s * Unit::Second;
        self.update_bodies();
    }

    fn update_bodies(&mut self) {
        let ssb_frame = Frame::from_ephem_j2000(SUN);
        let epoch = self.current_epoch;

        for body in &mut self.bodies {
            let body_frame = Frame::from_ephem_j2000(body.id);

            match self.almanac.translate(body_frame, ssb_frame, epoch, None) {
                Ok(state) => {
                    body.position = Vec3d::new(
                        state.radius_km.x,
                        state.radius_km.y,
                        state.radius_km.z,
                    );
                    body.velocity = Vec3d::new(
                        state.radius_km.x,
                        state.radius_km.y,
                        state.radius_km.z,
                    );
                }
                Err(e) => {
                    eprintln!("Could not get state for {}: {e}", body.name);
                }
            }
        }
    }
    fn update_rockets(&mut self) {

    }
}