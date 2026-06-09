use chrono::{DateTime, Utc};
use macroquad::color::{BLUE, Color, GRAY, WHITE, YELLOW};
use space_dust::bodies::{Earth, Moon};

use crate::{
    algo::{
        config::Config, history::OptimizationHistory, objective::generate_trajectory_for_params,
    },
    render::{
        camera::DrawCamera,
        drawing::{draw_object, draw_object_static_size, draw_text_label, draw_trajectory, draw_trajectory_with_thickness},
    },
    simulation::{
        objects::{MoonState, RocketState},
        world::{TrajectoryGenerator, simplify_trajectory},
    },
    util::math::Vec3d,
};

pub struct IterationDrawer<'a, 'b> {
    pub history: &'a OptimizationHistory,

    pub dt_s: f64,
    pub every_nth: usize,
    pub duration_s: f64,

    pub current_time: f64,

    pub moon_trajectory: Vec<MoonState>,
    simple_moon_trajectory: Vec<MoonState>,

    rocket_trajectories: Vec<Vec<RocketState>>,
    simple_rocket_trajectories: Vec<Vec<RocketState>>,

    traj_gen: TrajectoryGenerator,
    best_cost_index: usize,
    pub config: &'b Config,
}

impl<'a, 'b> IterationDrawer<'a, 'b> {
    pub fn new(
        history: &'a OptimizationHistory,
        dt_s: f64,
        every_nth: usize,
        duration_s: f64,
        config: &'b Config,
    ) -> Self {
        let date = DateTime::parse_from_rfc3339(history.start_epoch.as_str())
            .expect("Nieprawidłowy format daty")
            .to_utc();

        let mut drawer = Self {
            history,
            dt_s,
            every_nth,
            duration_s,
            current_time: 0.0,
            moon_trajectory: Vec::new(),
            simple_moon_trajectory: Vec::new(),
            rocket_trajectories: Vec::new(),
            simple_rocket_trajectories: Vec::new(),
            traj_gen: TrajectoryGenerator::with_epoch(date, duration_s, dt_s),
            config: config,
            best_cost_index: 0,
        };

        drawer.simulate_trajectories();
        drawer
    }

    pub fn set_time(&mut self, time: f64) {
        self.current_time = time.clamp(0.0, self.duration_s);
    }

    pub fn draw(&self, draw_camera: &DrawCamera) {
        // Earth
        draw_object(
            draw_camera,
            &Vec3d::new(0.0, 0.0, 0.0),
            Earth::EQUATORIAL_RADIUS_KM,
            BLUE,
        );

        // Moon
        if let Some(pos) = &self.get_moon_pos_at_time() {
            draw_object(draw_camera, pos, Moon::RADIUS / 1000.0, BLUE);
        }

        draw_trajectory(draw_camera, &self.simple_moon_trajectory, BLUE);

        // Rockets
        for (i, traj) in self.simple_rocket_trajectories.iter().enumerate() {
            if i == self.best_cost_index {
                draw_trajectory_with_thickness(draw_camera, traj, YELLOW, 4.0);
                continue;
            }
            draw_trajectory(
                draw_camera,
                traj,
                self.iteration_color(i, self.rocket_trajectories.len()),
            );
        }

        for (i, traj) in self.rocket_trajectories.iter().enumerate() {
            if i == self.best_cost_index {
                if let Some(pos) = &self.get_rocket_pos_at_time(&traj) {
                    draw_object_static_size(draw_camera, pos, 5.0, YELLOW);
                    draw_text_label(
                        draw_camera,
                        pos,
                        &format!("{:.2}", pos.velocity_km.norm()),
                        30.0,
                        20.0,
                        YELLOW,
                    )
                }
                continue;
            }
            if let Some(pos) = &self.get_rocket_pos_at_time(&traj) {
                draw_object_static_size(draw_camera, pos, 3.0, self.iteration_color(i, self.rocket_trajectories.len()));
            }
        }
    }

    fn simulate_trajectories(&mut self) {
        self.moon_trajectory = self.traj_gen.moon_trajectory.clone();
        self.simple_moon_trajectory = simplify_trajectory(&self.moon_trajectory, self.every_nth);

        for record in &self.history.records {
            let params = &record.best_params;
            let rocket_traj = generate_trajectory_for_params(params, self.config, &self.traj_gen);
            self.simple_rocket_trajectories
                .push(simplify_trajectory(&rocket_traj, self.every_nth));
            self.rocket_trajectories.push(rocket_traj);
        }

        for (i, record) in self.history.records.iter().enumerate() {
            if let Some(rec) = self.history.records.get(self.best_cost_index) {
                if record.best_cost < rec.best_cost {
                    self.best_cost_index = i;
                }
            }
        }
    }

    fn get_rocket_pos_at_time(&self, traj: &[RocketState]) -> Option<RocketState> {
        let idx = (self.current_time / self.dt_s) as usize;
        traj.get(idx).cloned()
    }

    fn iteration_color(&self, iter: usize, max_iter: usize) -> Color {
        let t = iter as f32 / (max_iter - 1) as f32; // 0..1
        let r = 0.2 + t * 0.8;
        let g = 0.2 + t * 0.8;
        let b = 0.2 + t * 0.8;
        Color::new(r, g, b, 1.0)
    }

    fn get_moon_pos_at_time(&self) -> Option<Vec3d> {
        let idx = (self.current_time / self.dt_s) as usize;
        self.moon_trajectory.get(idx).map(|s| s.position_km)
    }
}
