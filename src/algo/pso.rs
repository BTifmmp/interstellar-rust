// src/algo/pso.rs
use rand::Rng;

/// Pojedyncza cząstka (statek) w przestrzeni poszukiwań
#[derive(Debug, Clone)]
pub struct Particle {
    pub position: Vec<f64>,    // parametry startowe (vx, vy, vz, dx, dy, dz)
    pub velocity: Vec<f64>,    // prędkość zmiany parametrów
    pub best_position: Vec<f64>,
    pub best_cost: f64,
}

/// Rój cząstek
pub struct Swarm {
    pub particles: Vec<Particle>,
    pub global_best_position: Vec<f64>,
    pub global_best_cost: f64,
    w: f64,           // bezwładność (0.5-1.0)
    c1: f64,          // komponent poznawczy (zwykle 1.5-2.0)
    c2: f64,          // komponent społeczny (zwykle 1.5-2.0)
    bounds: Vec<(f64, f64)>,  // zakresy dla każdego wymiaru
}

impl Swarm {
    /// Tworzy nowy rój
    /// - `num_particles`: liczba cząstek
    /// - `bounds`: krotki (min, max) dla każdego z 6 wymiarów
    pub fn new(num_particles: usize, bounds: Vec<(f64, f64)>, w: f64, c1: f64, c2: f64) -> Self {
        let dims = bounds.len();
        let mut rng = rand::thread_rng();
        let particles = (0..num_particles)
            .map(|_| {
                let position: Vec<f64> = (0..dims)
                    .map(|i| {
                        let (min, max) = bounds[i];
                        rng.gen_range(min..max)
                    })
                    .collect();
                let velocity: Vec<f64> = (0..dims)
                    .map(|_| rng.gen_range(-1.0..1.0))
                    .collect();
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

    /// Wykonuje jedną iterację PSO
    pub fn update<F>(&mut self, cost_function: &F)
    where
        F: Fn(&[f64]) -> f64,
    {
        // Oblicz koszty i aktualizuj optima
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
                // Przycinanie do zakresów
                let (min, max) = self.bounds[d];
                new_pos = new_pos.clamp(min, max);
                particle.position[d] = new_pos;
            }
        }
    }

    /// Uruchamia optymalizację na zadaną liczbę iteracji
    pub fn optimize<F>(&mut self, iterations: usize, cost_function: &F) -> (Vec<f64>, f64)
    where
        F: Fn(&[f64]) -> f64,
    {
        for _ in 0..iterations {
            self.update(cost_function);
        }
        (self.global_best_position.clone(), self.global_best_cost)
    }
}