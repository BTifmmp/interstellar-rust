use rand::Rng;

#[derive(Debug, Clone)]
pub struct Particle {
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub best_position: Vec<f64>,
    pub best_cost: f64,
}

pub struct Swarm {
    pub particles: Vec<Particle>,
    pub global_best_position: Vec<f64>,
    pub global_best_cost: f64,
    w: f64,
    c1: f64,
    c2: f64,
    bounds: Vec<(f64, f64)>,
}

impl Swarm {
    pub fn new(num_particles: usize, bounds: Vec<(f64, f64)>, w: f64, c1: f64, c2: f64) -> Self {
        let dims = bounds.len();
        let mut rng = rand::thread_rng();

        let points_per_dim = (num_particles as f64).powf(2.0 / dims as f64).ceil() as usize;
        let particles = (0..num_particles)
            .map(|i| {
                let mut grid_coords = i;
                let position: Vec<f64> = (0..dims)
                    .map(|j| {
                        let (min, max) = bounds[j];
                        if (max - min).abs() < 1e-12 {
                            return min;
                        }
                        let idx_in_dim = grid_coords % points_per_dim;
                        grid_coords /= points_per_dim;
                        if points_per_dim > 1 {
                            min + (idx_in_dim as f64) * (max - min) / ((points_per_dim - 1) as f64)
                        } else {
                            min
                        }
                    })
                    .collect();
                let velocity: Vec<f64> = (0..dims).map(|_| rng.gen_range(-1.0..1.0)).collect();
                Particle {
                    position: position.clone(),
                    velocity,
                    best_position: position,
                    best_cost: f64::INFINITY,
                }
            })
            .collect();

        Swarm {
            particles,
            global_best_position: vec![0.0; dims],
            global_best_cost: f64::INFINITY,
            w,
            c1,
            c2,
            bounds,
        }
    }

    pub fn update<F>(&mut self, cost_function: &F)
    where
        F: Fn(&[f64]) -> f64,
    {
        for particle in &mut self.particles {
            let cost = cost_function(&particle.position);
            if cost < particle.best_cost {
                particle.best_cost = cost;
                particle.best_position = particle.position.clone();
            }
            if cost < self.global_best_cost {
                self.global_best_cost = cost;
                self.global_best_position = particle.position.clone();
            }
        }

        let mut rng = rand::thread_rng();
        for particle in &mut self.particles {
            for d in 0..particle.position.len() {
                let r1: f64 = rng.r#gen();
                let r2: f64 = rng.r#gen();
                let cognitive = self.c1 * r1 * (particle.best_position[d] - particle.position[d]);
                let social = self.c2 * r2 * (self.global_best_position[d] - particle.position[d]);
                particle.velocity[d] = self.w * particle.velocity[d] + cognitive + social;
                let mut new_pos = particle.position[d] + particle.velocity[d];
                let (min, max) = self.bounds[d];
                new_pos = new_pos.clamp(min, max);
                particle.position[d] = new_pos;
            }
        }
    }

    pub fn optimize<F>(&mut self, iterations: usize, cost_function: &F) -> (Vec<f64>, f64)
    where
        F: Fn(&[f64]) -> f64,
    {
        for i in 0..iterations {
            self.update(cost_function);
            println!(
                "Iteracja {} / {} (Najlepszy globalny koszt: {:.4})",
                i + 1,
                iterations,
                self.global_best_cost
            );
        }
        (self.global_best_position.clone(), self.global_best_cost)
    }
}
