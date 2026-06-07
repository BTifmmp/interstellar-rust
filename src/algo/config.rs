use serde::Deserialize;
use std::fs;

#[derive(Debug, Deserialize)]
pub struct Config {
    pub start_point: GeoPoint, // miejsce startu na ziemi
    pub target_point: GeoPoint, // miejsce lądowania na księżycu
    pub pso_params: PsoParams, // parametry PSO (liczba cząsteczek, liczba iteracji, współczynniki(w-bezwładność, c1-siła przyciągania do lokalnego najlepszego wyniku, c2-siła przyciągania cząsteczki do globalnego najlepszego wyniku))
    pub bounds: Bounds, // zakres poszukiwań
    pub simulation_params: SimulationParams, // parametry symulacji fizycznej
    pub weights: [f64; 3],   // wagi dla funkcji kosztu(waga_odległości, waga_prędkości_startowej, waga_prędkości_końcowej)
}

#[derive(Debug, Deserialize)]
pub struct GeoPoint { //Reprezentuje punkt na powierzchni (lub nad nią) ciała niebieskiego
    pub latitude_deg: f64, // w pionie
    pub longitude_deg: f64, // w poziomie
    pub altitude_km: f64,
}

#[derive(Debug, Deserialize)]
pub struct PsoParams {
    pub num_particles: usize,
    pub max_iterations: usize,
    pub w: f64,
    pub c1: f64,
    pub c2: f64,
}

#[derive(Debug, Deserialize)]
pub struct Bounds {
    pub vx: [f64; 2], // zakresy prędkości początkowej rakiety (x,y,z) w km/s
    pub vy: [f64; 2],
    pub vz: [f64; 2],
    pub dx: [f64; 2], // zakresy przesunięcia startowego względem punktu bazowego na Ziemi (w km)
    pub dy: [f64; 2],
    pub dz: [f64; 2],
}

#[derive(Debug, Deserialize)]
pub struct SimulationParams {
    pub max_duration_days: f64, // maksymalny czas trwania symulacji (w dniach)
    pub dt_s: f64, // krok całkowania w propagacji RK4 (w sekundach)
    pub snapshot_dt_s: f64, //  interwał, z jakim zapisujemy stany do trajektorii (co ile sekund)
}

impl Config {
    pub fn from_file(path: &str) -> Self {
        let data = fs::read_to_string(path).expect("Unable to read config file"); // Otwiera plik o podanej ścieżce, czyta całą zawartość do stringa.
        serde_json::from_str(&data).expect("Invalid JSON config") // Próbuje zdeserializować ten string do struktury Config. Jeśli się uda, zwraca instancję Config
    }
}