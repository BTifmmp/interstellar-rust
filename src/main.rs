// main.rs
mod algo;
mod models;
mod physics;

use macroquad::prelude::*;
use crate::algo::pso::Swarm;
use crate::algo::objective::{cost_function, generate_trajectory, moon_position};

const VIEW_SCALE: f32 = 1e-3; // 1 piksel = 1000 km (orbita Księżyca ~384 px)

#[macroquad::main("PSO - Trajektorie statków do Księżyca")]
async fn main() {
    // Parametry PSO
    let bounds = vec![
        (-10.0, 10.0),   // vx [km/s]
        (-10.0, 10.0),   // vy
        (-10.0, 10.0),   // vz
        (-20000.0, 20000.0), // dx [km]
        (-20000.0, 20000.0), // dy
        (-20000.0, 20000.0), // dz
    ];
    let num_particles = 15;
    let max_iterations = 10;
    let max_duration = 7.0 * 86400.0; // 7 dni
    let dt = 300.0; // krok propagacji w optymalizacji (5 min)

    let objective = |params: &[f64]| {
        cost_function(params, max_duration, dt, (1.0, 0.001, 0.001))
    };

    println!("Rozpoczynam optymalizację PSO...");
    let mut swarm = Swarm::new(num_particles, bounds, 0.7, 1.5, 1.5);
    let (best_params, best_cost) = swarm.optimize(max_iterations, &objective);
    println!("Najlepsze parametry: {:?}, koszt: {}", best_params, best_cost);

    // Generowanie trajektorii do wizualizacji (większa dokładność)
    let dt_viz = 60.0;
    let best_trajectory = generate_trajectory(&best_params, max_duration, dt_viz);
    let mut all_trajectories = Vec::new();
    for particle in &swarm.particles {
        let traj = generate_trajectory(&particle.position, max_duration, dt_viz);
        all_trajectories.push(traj);
    }

    // Czas symulacji dla animacji Księżyca (i ewentualnie statków)
    let mut sim_time = 0.0;

    loop {
        clear_background(BLACK);

        let screen_center = vec2(screen_width() / 2.0, screen_height() / 2.0);

        // 1. Rysowanie wszystkich trajektorii roju (szare, półprzezroczyste)
        for traj in &all_trajectories {
            for i in 0..traj.len().saturating_sub(1) {
                let p1 = traj[i];
                let p2 = traj[i+1];
                let x1 = (p1.x as f32 * VIEW_SCALE) + screen_center.x;
                let y1 = (p1.y as f32 * VIEW_SCALE) + screen_center.y;
                let x2 = (p2.x as f32 * VIEW_SCALE) + screen_center.x;
                let y2 = (p2.y as f32 * VIEW_SCALE) + screen_center.y;
                draw_line(x1, y1, x2, y2, 1.0, Color::new(0.5, 0.5, 0.5, 0.5));
            }
        }

        // 2. Rysowanie najlepszej trajektorii (żółta, grubsza)
        for i in 0..best_trajectory.len().saturating_sub(1) {
            let p1 = best_trajectory[i];
            let p2 = best_trajectory[i+1];
            let x1 = (p1.x as f32 * VIEW_SCALE) + screen_center.x;
            let y1 = (p1.y as f32 * VIEW_SCALE) + screen_center.y;
            let x2 = (p2.x as f32 * VIEW_SCALE) + screen_center.x;
            let y2 = (p2.y as f32 * VIEW_SCALE) + screen_center.y;
            draw_line(x1, y1, x2, y2, 2.5, YELLOW);
        }

        // 3. Rysowanie Ziemi (środek układu)
        let earth_x = screen_center.x;
        let earth_y = screen_center.y;
        draw_circle(earth_x, earth_y, 25.0, BLUE);
        draw_text("Ziemia", earth_x + 15.0, earth_y - 15.0, 20.0, WHITE);

        // 4. Rysowanie Księżyca (pozycja zależna od czasu)
        let moon_pos = moon_position(sim_time);
        let moon_x = (moon_pos.x as f32 * VIEW_SCALE) + screen_center.x;
        let moon_y = (moon_pos.y as f32 * VIEW_SCALE) + screen_center.y;
        draw_circle(moon_x, moon_y, 10.0, GRAY);
        draw_text("Księżyc", moon_x + 8.0, moon_y - 8.0, 16.0, WHITE);

        // 5. Opcjonalnie: narysuj aktualną pozycję najlepszego statku (interpolując trajektorię)
        if sim_time <= max_duration {
            // Znajdź indeks przedziału czasowego (zakładamy stały krok dt_viz)
            let idx = (sim_time / dt_viz) as usize;
            if idx < best_trajectory.len() {
                let ship_pos = best_trajectory[idx];
                let ship_x = (ship_pos.x as f32 * VIEW_SCALE) + screen_center.x;
                let ship_y = (ship_pos.y as f32 * VIEW_SCALE) + screen_center.y;
                draw_circle(ship_x, ship_y, 5.0, RED);
            }
        }

        // Aktualizacja czasu symulacji (np. przyspieszenie 500×)
        let frame_time = get_frame_time();
        sim_time += frame_time as f64 * 3600.0;
        if sim_time > max_duration {
            sim_time = 0.0; // resetujemy, aby zobaczyć ponownie
        }

        next_frame().await;
    }
}