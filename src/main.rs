use std::thread;
use std::time::{Duration, Instant};

use crate::algo::objective::generate_trajectory_for_params;
use crate::render::camera::CameraController;
use crate::render::drawing::{
    draw_body, draw_hud, draw_moon_trajectory, draw_rocket, draw_rocket_trajectory,
    draw_vec_trajectory,
};
use crate::render::mouse::update_mouse_lock;
use crate::simulation::world::{MoonState, precompute_moon_states};
use macroquad::prelude::*;
mod algo;
mod render;
mod simulation;
mod util;
use algo::config::Config;
use algo::objective::cost_function;
use algo::pso::Swarm;
use simulation::world::TrajectoryGenerator;
use space_dust::bodies::Earth; // Adjusted to match your space_dust path

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

    let traj_gen =
        TrajectoryGenerator::new(config.simulation_params.max_duration_days * 86400.0, 10.0);
    

    // 4. Definicja funkcji kosztu
    let objective = |params: &[f64]| cost_function(params, start_epoch, &config, &traj_gen);

    println!("Rozpoczynam optymalizację PSO. To może potrwać kilkadziesiąt sekund...");
    let (best_params, best_cost) = swarm.optimize(config.pso_params.max_iterations, &objective);
    println!("Najlepsze parametry: {:?}", best_params);
    println!("Najlepszy koszt: {}", best_cost);

    // // 5. Generuj trajektorie dla wszystkich cząstek (do rysowania roju)
    let mut all_trajectories = Vec::new();
    for particle in &swarm.particles {
        let traj = generate_trajectory_for_params(&particle.position, start_epoch, &config, &traj_gen);
        let every_nth: Vec<_> = traj
          .iter()
          .enumerate() // Daje nam (indeks, wartość)
          .filter(|(i, _)| (i + 1) % 50 == 0) // Wybiera co n-ty
          .map(|(_, val)| *val) // Wyciąga samą wartość
          .collect();
        all_trajectories.push(every_nth);
    }

    let best_traj = generate_trajectory_for_params(&best_params, start_epoch, &config, &traj_gen);

    let best_trajectory: Vec<Vec3d> = best_traj.iter()
          .enumerate() // Daje nam (indeks, wartość)
          .filter(|(i, _)| (i + 1) % 50 == 0) // Wybiera co n-ty
          .map(|(_, val)| *val) // Wyciąga samą wartość
          .collect();
    
    let simple_moon: Vec<MoonState> = traj_gen.moon_trajectory.iter() // BEZ & na początku
        .enumerate()
        .filter(|(i, _)| (i + 1) % 50 == 0)
        .map(|(_, val)| val.clone()) // Używamy .clone() zamiast *val
        .collect();

    let mut cam_controller = CameraController::new(Vec3d::new(0.0, 0.0, 45000.0));

    let mut mouse_flag: bool = false;

    let frame = 0;
    let target_fps = 400.0;
    let frame_duration = 1.0 / target_fps;

    loop {
        let frame_start = Instant::now();

        match update_mouse_lock() {
            Some(should_lock) => mouse_flag = should_lock,
            None => {} // FIXED: Enters an empty block to safely do nothing
        }

        if (mouse_flag) {
            cam_controller.update();
        }

        clear_background(BLACK);

        // for body in &world.bodies {
        //     draw_body(&cam_controller.camera, body, BLUE);
        // }

        // // Rysowanie wszystkich trajektorii roju (szare)
        for traj in &all_trajectories {
            // Funkcja rysująca linię po punktach (możesz użyć draw_vec_trajectory)
            draw_vec_trajectory(&cam_controller.camera, traj, DARKGRAY);
        }



        // Rysowanie trajektorii Księżyca
        draw_moon_trajectory(&cam_controller.camera, &simple_moon, DARKGRAY);

        // POPRAWKA: Rysowanie najlepszej trajektorii znalezionej przez PSO zamiast starej testowej
        draw_vec_trajectory(&cam_controller.camera, &best_trajectory, YELLOW);

        // POPRAWKA: Odczyt pozycji żółtej rakiety z właściwego wektora best_trajectory
        if let Some(current_state) = best_trajectory.get(frame) {
            draw_rocket(&cam_controller.camera, *current_state, YELLOW);
        }

        // draw_hud(world.epoch);

        next_frame().await;

        let elapsed = frame_start.elapsed().as_secs_f64();
        if elapsed < frame_duration {
            thread::sleep(Duration::from_millis(
                ((frame_duration - elapsed) as u64) * 1000,
            ));
        }
    }
}
