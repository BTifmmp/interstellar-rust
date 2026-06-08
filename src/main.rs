use crate::render::camera::CameraController;
use crate::render::drawing::{draw_hud, draw_trajectory};
use crate::render::iteration_drawer::IterationDrawer;
use crate::render::mouse::update_mouse_lock;
use crate::simulation::world::TrajectoryGenerator;
use macroquad::prelude::*;
mod algo;
mod render;
mod simulation;
mod util;
use crate::util::math::Vec3d;
use algo::config::Config;
use algo::history::{IterationRecord, OptimizationHistory};
use algo::objective::cost_function;
use algo::pso::Swarm;
use chrono::Utc;
use serde_json;
use std::path::Path;

#[macroquad::main("Orbital Simulation 3D")]
async fn main() {
    let config = Config::from_file("config.json");
    let start_epoch = Utc::now();

    let bounds = vec![
        (config.bounds.vx[0], config.bounds.vx[1]),
        (config.bounds.vy[0], config.bounds.vy[1]),
        (config.bounds.vz[0], config.bounds.vz[1]),
        (config.bounds.dx[0], config.bounds.dx[1]),
        (config.bounds.dy[0], config.bounds.dy[1]),
        (config.bounds.dz[0], config.bounds.dz[1]),
    ];

    let mut swarm = Swarm::new(
        config.pso_params.num_particles,
        bounds,
        config.pso_params.w,
        config.pso_params.c1,
        config.pso_params.c2,
    );

    let traj_gen = TrajectoryGenerator::with_epoch(
        start_epoch,
        config.simulation_params.max_duration_days * 86400.0,
        config.simulation_params.dt_s,
    );

    let objective = |params: &[f64]| cost_function(params, &config, &traj_gen);

    let history_path = "pso_history.json";
    let max_iterations = config.pso_params.max_iterations;

    let history = if Path::new(history_path).exists() {
        println!("Wczytywanie historii z pliku...");
        let history = OptimizationHistory::load_from_file(history_path).unwrap();
        println!("Wczytano {} iteracji.", history.records.len());
        history
    } else {
        println!("Rozpoczynam optymalizację PSO...");
        let mut records = Vec::new();

        for iter in 0..max_iterations {
            swarm.update(&objective);

            let mut best_cost_this_iter = f64::INFINITY;
            let mut best_params_this_iter = Vec::new();
            for particle in &swarm.particles {
                let cost = objective(&particle.position);
                if cost < best_cost_this_iter {
                    best_cost_this_iter = cost;
                    best_params_this_iter = particle.position.clone();
                }
            }

            records.push(IterationRecord {
                iteration: iter + 1,
                best_cost: best_cost_this_iter,
                best_params: best_params_this_iter,
            });

            println!(
                "Iteracja {} / {} (najlepszy koszt w iteracji: {:.4})",
                iter + 1,
                max_iterations,
                best_cost_this_iter
            );
        }

        let history = OptimizationHistory {
            config_snapshot: serde_json::to_value(&config).unwrap(),
            start_epoch: start_epoch.to_rfc3339(),
            records,
        };

        history
            .save_to_file(history_path)
            .expect("Failed To save history");
        println!("Historia zapisana do pliku.");
        history
    };

    println!("Wyznaczanie trajektori do rysowania");
    let mut iter_drawer = IterationDrawer::new(
        &history,
        config.simulation_params.dt_s,
        300,
        config.simulation_params.max_duration_days * 86400.0,
        &config,
    );

    println!("Odtwarzanie symulacji");


    let mut cam_controller = CameraController::new(Vec3d::new(0.0, 0.0, 80000.0));
    let mut mouse_flag = false;

    let mut sim_speed: f64 = 3600.0; // Prędkość symulacji (np. 1 sekunda realna = 3600 sekund symulacji)
    let mut simulation_time: f64 = 0.0; // Czas wewnętrzny symulacji

    loop {
        clear_background(BLACK);

        match update_mouse_lock() {
            Some(should_lock) => mouse_flag = should_lock,
            None => {}
        }

        if is_key_pressed(KeyCode::Up) {
            sim_speed = (sim_speed + 200.0).min(7200.0);
        }
        if is_key_pressed(KeyCode::Down) {
            sim_speed = (sim_speed - 200.0).max(-7200.0);
        }
        if is_key_pressed(KeyCode::Space) {
            sim_speed = 0.0;
        }

        if mouse_flag {
            cam_controller.update();
        }

        let dt_real = get_frame_time() as f64;
        simulation_time += dt_real * sim_speed;

        if simulation_time > iter_drawer.duration_s {
            simulation_time = 0.0;
        }

        simulation_time = simulation_time.max(0.0);

        iter_drawer.set_time(simulation_time);
        iter_drawer.draw(&cam_controller.camera);

        draw_hud(start_epoch + chrono::Duration::milliseconds((simulation_time * 1000.0) as i64));

        next_frame().await;
    }
}
