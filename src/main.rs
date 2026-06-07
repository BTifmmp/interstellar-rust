use crate::render::camera::CameraController;
use crate::simulation::world::{precompute_moon_states};
use crate::render::drawing::{
    draw_body, draw_hud, draw_rocket, draw_rocket_trajectory, draw_vec_trajectory,
};
use crate::render::mouse::update_mouse_lock;
use crate::simulation::world::{generate_moon_trajectory, generate_rocket_trajectory};
use macroquad::prelude::*;
mod render;
mod simulation;
mod util;
mod algo;
use simulation::world::SimulationWorld;
use space_dust::bodies::Earth; // Adjusted to match your space_dust path
use algo::config::Config;
use algo::pso::Swarm;
use algo::objective::{cost_function, generate_trajectory_for_params};

use crate::simulation::objects::Rocket;
use crate::util::math::Vec3d;
use chrono::Utc;

#[macroquad::main("Orbital Simulation 3D")]
async fn main() {
    // 1. Wczytaj konfigurację
    let config = Config::from_file("config.json");
    let start_epoch = Utc::now();

    // 2. Przygotuj zakresy dla PSO (6 wymiarów)
    let bounds = vec![
        (config.bounds.vx[0], config.bounds.vx[1]),
        (config.bounds.vy[0], config.bounds.vy[1]),
        (config.bounds.vz[0], config.bounds.vz[1]),
        (config.bounds.dx[0], config.bounds.dx[1]),
        (config.bounds.dy[0], config.bounds.dy[1]),
        (config.bounds.dz[0], config.bounds.dz[1]),
    ];

    // 3. Utwórz rój
    let mut swarm = Swarm::new(
        config.pso_params.num_particles,
        bounds,
        config.pso_params.w,
        config.pso_params.c1,
        config.pso_params.c2,
    );

    
    let snapshot_dt_s = config.simulation_params.snapshot_dt_s;
    let num_snapshots = (config.simulation_params.max_duration_days * 86400.0 / snapshot_dt_s) as usize;

    let moon_states = precompute_moon_states(start_epoch, snapshot_dt_s, num_snapshots);

    // 4. Definicja funkcji kosztu
    let objective = |params: &[f64]| {
        cost_function(params, start_epoch, &config, &moon_states)
    };

    println!("Rozpoczynam optymalizację PSO. To może potrwać kilkadziesiąt sekund...");
    let (best_params, best_cost) = swarm.optimize(config.pso_params.max_iterations, &objective);
    println!("Najlepsze parametry: {:?}", best_params);
    println!("Najlepszy koszt: {}", best_cost);

    // // 5. Generuj trajektorie dla wszystkich cząstek (do rysowania roju)
    let mut all_trajectories = Vec::new();
    for particle in &swarm.particles {
        let traj = generate_trajectory_for_params(&particle.position, start_epoch, &config);
        all_trajectories.push(traj);
    }
    let best_trajectory = generate_trajectory_for_params(&best_params, start_epoch, &config);

    let mut world = SimulationWorld::with_epoch(start_epoch);

    let moon_traj = generate_moon_trajectory(start_epoch, config.simulation_params.max_duration_days * 86400.0 * 4.0);

    let mut cam_controller = CameraController::new(Vec3d::new(0.0, 0.0, 45000.0));

    let mut mouse_flag: bool = false;

    loop {
        match update_mouse_lock() {
            Some(should_lock) => mouse_flag = should_lock,
            None => {} // FIXED: Enters an empty block to safely do nothing
        }

        if (mouse_flag) {
          cam_controller.update();
        }


        let live_dt = 60.0;
        world.step(live_dt);

        clear_background(BLACK);

        for body in &world.bodies {
            draw_body(&cam_controller.camera, body, BLUE);
        }

        // // Rysowanie wszystkich trajektorii roju (szare)
        for traj in &all_trajectories {
            // Funkcja rysująca linię po punktach (możesz użyć draw_vec_trajectory)
            draw_vec_trajectory(&cam_controller.camera, traj, DARKGRAY);
        }

        // Rysowanie trajektorii Księżyca
        draw_vec_trajectory(&cam_controller.camera, &moon_traj, DARKGRAY);

        // POPRAWKA: Rysowanie najlepszej trajektorii znalezionej przez PSO zamiast starej testowej
        draw_vec_trajectory(&cam_controller.camera, &best_trajectory, YELLOW);

        // Obliczanie klatki animacji rakiety na podstawie czasu symulacji i konfiguracji JSON
        let current_elapsed = (world.epoch - start_epoch).num_seconds() as f64;
        let target_index = (current_elapsed / config.simulation_params.snapshot_dt_s) as usize;

        // POPRAWKA: Odczyt pozycji żółtej rakiety z właściwego wektora best_trajectory
        if let Some(current_state) = best_trajectory.get(target_index) {
            draw_rocket(&cam_controller.camera, *current_state, YELLOW);
        }

        draw_hud(world.epoch);

        next_frame().await;
    }
}
