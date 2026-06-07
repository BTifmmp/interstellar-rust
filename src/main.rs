use crate::render::camera::CameraController;
use crate::render::drawing::{
    draw_body, draw_hud, draw_rocket, draw_vec_trajectory,
};
use crate::render::mouse::update_mouse_lock;
use crate::simulation::world::{generate_moon_trajectory, precompute_moon_states};
use macroquad::prelude::*;
mod render;
mod simulation;
mod util;
mod algo;
use simulation::world::SimulationWorld;
use algo::config::Config;
use algo::pso::Swarm;
use algo::objective::{cost_function, generate_trajectory_for_params};
use crate::util::math::Vec3d;
use chrono::Utc;
use algo::history::{OptimizationHistory, IterationRecord};
use std::path::Path;
use serde_json;

/// Zamienia numer iteracji (0..max_iter-1) na kolor (od szarego do czerwonego)
fn iteration_color(iter: usize, max_iter: usize) -> Color {
    let t = iter as f32 / (max_iter - 1) as f32; // 0..1
    let r = 0.5 + t * 0.5;
    let g = 0.5 * (1.0 - t);
    let b = 0.5 * (1.0 - t);
    Color::new(r, g, b, 1.0)
}

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

    let snapshot_dt_s = config.simulation_params.snapshot_dt_s;
    let num_snapshots = (config.simulation_params.max_duration_days * 86400.0 / snapshot_dt_s) as usize;
    let moon_states = precompute_moon_states(start_epoch, snapshot_dt_s, num_snapshots);

    let objective = |params: &[f64]| {
        cost_function(params, start_epoch, &config, &moon_states)
    };

    let history_path = "pso_history.json";
    let max_iterations = config.pso_params.max_iterations;

    let all_iter_trajectories: Vec<Vec<Vec3d>>;

    if Path::new(history_path).exists() {
        println!("Wczytywanie historii z pliku...");
        let history = OptimizationHistory::load_from_file(history_path).unwrap();
        let mut trajs = Vec::new();
        for record in &history.records {
            let traj = generate_trajectory_for_params(&record.best_params, start_epoch, &config);
            trajs.push(traj);
        }
        all_iter_trajectories = trajs;
        println!("Wczytano {} iteracji.", all_iter_trajectories.len());
    } else {
        println!("Rozpoczynam optymalizację PSO...");
        let mut records = Vec::new();

        for iter in 0..max_iterations {
            swarm.update(&objective);

            // --- Nowe: znajdź najlepszą cząstkę w TEJ iteracji (na podstawie aktualnych pozycji) ---
            let mut best_cost_this_iter = f64::INFINITY;
            let mut best_params_this_iter = Vec::new();
            for particle in &swarm.particles {
                let cost = objective(&particle.position);
                if cost < best_cost_this_iter {
                    best_cost_this_iter = cost;
                    best_params_this_iter = particle.position.clone();
                }
            }
            // ------------------------------------------------------------------------------------

            records.push(IterationRecord {
                iteration: iter + 1,
                best_cost: best_cost_this_iter,
                best_params: best_params_this_iter,
            });

            println!("Iteracja {} / {} (najlepszy koszt w iteracji: {:.4})", iter + 1, max_iterations, best_cost_this_iter);
        }

        // Po zakończeniu iteracji generujemy trajektorie dla każdego rekordu
        let mut trajs = Vec::new();
        for record in &records {
            let traj = generate_trajectory_for_params(&record.best_params, start_epoch, &config);
            trajs.push(traj);
        }
        all_iter_trajectories = trajs;

        let history = OptimizationHistory {
            config_snapshot: serde_json::to_value(&config).unwrap(),
            start_epoch: start_epoch.to_rfc3339(),
            records,
        };
        history.save_to_file(history_path).unwrap();
        println!("Historia zapisana do pliku.");
    }

    // Wizualizacja
    let mut world = SimulationWorld::with_epoch(start_epoch);
    let moon_traj = generate_moon_trajectory(start_epoch, config.simulation_params.max_duration_days * 86400.0 * 4.0);
    let mut cam_controller = CameraController::new(Vec3d::new(0.0, 0.0, 45000.0));
    let mut mouse_flag = false;

    loop {
        match update_mouse_lock() {
            Some(should_lock) => mouse_flag = should_lock,
            None => {}
        }
        if mouse_flag {
            cam_controller.update();
        }

        world.step(60.0);
        clear_background(BLACK);

        for body in &world.bodies {
            draw_body(&cam_controller.camera, body, BLUE);
        }

        let num_iter = all_iter_trajectories.len();
        for (i, traj) in all_iter_trajectories.iter().enumerate() {
            let color = iteration_color(i, num_iter);
            draw_vec_trajectory(&cam_controller.camera, traj, color);
        }

        draw_vec_trajectory(&cam_controller.camera, &moon_traj, DARKGRAY);

        if let Some(last_traj) = all_iter_trajectories.last() {
            let current_elapsed = (world.epoch - start_epoch).num_seconds() as f64;
            let target_index = (current_elapsed / snapshot_dt_s) as usize;
            if let Some(pos) = last_traj.get(target_index) {
                draw_rocket(&cam_controller.camera, *pos, YELLOW);
            }
        }

        draw_hud(world.epoch);
        next_frame().await;
    }
}