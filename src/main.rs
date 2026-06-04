use macroquad::prelude::*;
use nyx_space::cosmic::{Epoch, Unit};
use crate::simulation::world::SimulationWorld;
mod models;
mod simulation;

const VIEW_SCALE: f32 = 1e-6; 

#[macroquad::main("Solar System Sim")]
async fn main() {
    let start_epoch = Epoch::from_gregorian_str("2026-06-04T18:00:00").unwrap();
    let mut world = SimulationWorld::new(start_epoch);

    loop {
        clear_background(BLACK);

        world.step(3600.0); 

        let screen_center = vec2(screen_width() / 2.0, screen_height() / 2.0);

        for body in &world.bodies {
            let pos_x = (body.position.x as f32 * VIEW_SCALE) + screen_center.x;
            let pos_y = (body.position.y as f32 * VIEW_SCALE) + screen_center.y;

            // Draw the body
            draw_circle(
                pos_x, 
                pos_y, 
                body.size as f32 * 0.5, 
                Color::new(
                    body.color[0] as f32, 
                    body.color[1] as f32, 
                    body.color[2] as f32, 
                    body.color[3] as f32
                )
            );

            // Draw Name
            draw_text(&body.name, pos_x + 5.0, pos_y + 5.0, 15.0, WHITE);
        }

        next_frame().await
    }
}