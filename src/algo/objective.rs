// src/algo/objective.rs
use crate::util::geometry::enu_to_cartesian_offset;
use crate::simulation::objects::Rocket;
use crate::simulation::world::generate_rocket_trajectory;
use crate::util::math::Vec3d;
use crate::util::geometry::geographic_to_cartesian;
use chrono::{DateTime, Utc, Duration};
use space_dust::bodies::{Earth, Moon};
use crate::algo::config::Config;

/// Główna funkcja kosztu dla PSO.
/// 
/// Parametry:
/// - `params`: tablica 6 wartości [vx, vy, vz, dx, dy, dz]
/// - `start_epoch`: moment startu misji
/// - `config`: wczytana konfiguracja (punkt startowy, cel, wagi, czas symulacji)
/// 
/// Zwraca koszt (im mniejszy, tym lepszy).
pub fn cost_function(
    params: &[f64],
    start_epoch: DateTime<Utc>,
    config: &Config,
) -> f64 {
    // 1. Rozpakowanie parametrów
    let (vx, vy, vz, dx, dy, dz) = (params[0], params[1], params[2], params[3], params[4], params[5]);

    // 2. Obliczenie pozycji startowej na Ziemi (uwzględniamy wysokość nad poziomem morza)
    let earth_radius = Earth::EQUATORIAL_RADIUS_KM;
    let start_alt = config.start_point.altitude_km;
    let start_radius = earth_radius + start_alt;
    let base_start_pos = geographic_to_cartesian(
        config.start_point.latitude_deg,
        config.start_point.longitude_deg,
        start_radius,
    );
    // Dodajemy przesunięcie (dx,dy,dz) – dzięki temu PSO może startować z okolic punktu bazowego
    let offset = enu_to_cartesian_offset(base_start_pos, dx, dy, dz);
    let start_pos = base_start_pos + offset;

    // 3. Prędkość początkowa w układzie geocentrycznym (inercjalnym)
    let start_vel = Vec3d::new(vx, vy, vz);

    // 4. Tworzymy obiekt rakiety
    let rocket = Rocket {
        id: 0,   // id nie ma znaczenia, bo używamy oddzielnego świata
        position_km: start_pos,
        velocity_km: start_vel,
    };

    // 5. Przygotowanie parametrów symulacji
    let duration_s = config.simulation_params.max_duration_days * 86400.0;
    let dt_s = config.simulation_params.dt_s;
    let snapshot_dt_s = config.simulation_params.snapshot_dt_s;

    // 6. Uruchamiamy symulację (to wywołuje propagację RK4)
    let trajectory = generate_rocket_trajectory(&rocket, start_epoch, duration_s, dt_s, snapshot_dt_s);
    // trajectory to wektor stanów (czas, pozycja, prędkość)

    // 7. Obliczenie docelowego punktu na Księżycu (w układzie związanym z Księżycem)
    let moon_radius = Moon::RADIUS / 1000.0;      // km
    let target_alt = config.target_point.altitude_km;
    let target_radius = moon_radius + target_alt;
    let target_offset = geographic_to_cartesian(
        config.target_point.latitude_deg,
        config.target_point.longitude_deg,
        target_radius,
    );

    // 8. Przeszukanie trajektorii w poszukiwaniu minimalnej odległości od celu
    let mut best_dist = f64::INFINITY;
    let mut best_state: Option<&crate::simulation::world::RocketState> = None;
    let mut best_time = 0.0;

    for state in &trajectory {
        // Obliczamy pozycję środka Księżyca w danym momencie (z efemeryd space_dust)
        let moon_epoch = start_epoch + Duration::milliseconds((state.time * 1000.0) as i64);
        let moon_center = Moon::eci_position_km(&moon_epoch);
        // Pozycja docelowego punktu na powierzchni (lub orbicie) Księżyca
        let target_pos = Vec3d::new(
            moon_center.x + target_offset.x,
            moon_center.y + target_offset.y,
            moon_center.z + target_offset.z,
        );
        let dist = (state.position_km - target_pos).norm();
        if dist < best_dist {
            best_dist = dist;
            best_state = Some(state);
            best_time = state.time;
        }
    }

    // 9. Obliczenie prędkości końcowej (względem powierzchni Księżyca) w momencie największego zbliżenia
    let mut end_speed = 0.0;
    if let Some(state) = best_state {
        let moon_epoch = start_epoch + Duration::milliseconds((best_time * 1000.0) as i64);
        let dt = 1.0; // sekunda
        let moon_epoch_future = moon_epoch + Duration::seconds(1);
        
        let pos_now = Moon::eci_position_km(&moon_epoch);
        let pos_future = Moon::eci_position_km(&moon_epoch_future);
        
        // Prędkość = (Pozycja_2 - Pozycja_1) / czas
        let moon_vel_x = (pos_future.x - pos_now.x) / dt;
        let moon_vel_y = (pos_future.y - pos_now.y) / dt;
        let moon_vel_z = (pos_future.z - pos_now.z) / dt;
        
        let rel_vel = state.velocity_km - Vec3d::new(moon_vel_x, moon_vel_y, moon_vel_z);
        end_speed = rel_vel.norm();
    }

    // 10. Prędkość startowa
    let start_speed = start_vel.norm();

    // 11. Kary: jeśli rakieta kiedykolwiek znajdzie się poniżej powierzchni Ziemi (kolizja)
    let collision_penalty = if trajectory.iter().any(|s| s.position_km.norm() - earth_radius < 1e-9) {
        1e9   // bardzo wysoka kara
    } else {
        0.0
    };

    // 12. Ostateczny koszt = ważona suma
    let (w_dist, w_start, w_end) = (config.weights[0], config.weights[1], config.weights[2]);
    w_dist * best_dist + w_start * start_speed + w_end * end_speed + collision_penalty
}

/// Generuje trajektorię (tylko pozycje) dla podanych parametrów – używane do wizualizacji.
/// Zwraca wektor pozycji (km) w układzie geocentrycznym.
pub fn generate_trajectory_for_params(
    params: &[f64],
    start_epoch: DateTime<Utc>,
    config: &Config,
) -> Vec<Vec3d> {
    let (vx, vy, vz, dx, dy, dz) = (params[0], params[1], params[2], params[3], params[4], params[5]);
    let earth_radius = Earth::EQUATORIAL_RADIUS_KM;
    let start_alt = config.start_point.altitude_km;
    let start_radius = earth_radius + start_alt;
    let base_pos = geographic_to_cartesian(
        config.start_point.latitude_deg,
        config.start_point.longitude_deg,
        start_radius,
    );
    let offset = enu_to_cartesian_offset(base_pos, dx, dy, dz);
    let start_pos = base_pos + offset;
    let start_vel = Vec3d::new(vx, vy, vz);
    let rocket = Rocket { id: 0, position_km: start_pos, velocity_km: start_vel };
    let duration_s = config.simulation_params.max_duration_days * 86400.0;
    let dt_s = config.simulation_params.dt_s;
    let snapshot_dt_s = config.simulation_params.snapshot_dt_s;
    let traj = generate_rocket_trajectory(&rocket, start_epoch, duration_s, dt_s, snapshot_dt_s);
    traj.into_iter().map(|s| s.position_km).collect()
}