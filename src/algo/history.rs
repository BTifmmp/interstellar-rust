use serde::{Deserialize, Serialize};
use std::fs;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IterationRecord {
    pub iteration: usize,
    pub best_cost: f64,
    pub best_params: Vec<f64>,
    pub objective_dist: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizationHistory {
    pub config_snapshot: serde_json::Value, // przechowuje cały config w JSON
    pub start_epoch: String,                // jako string ISO 8601
    pub records: Vec<IterationRecord>,
}

impl OptimizationHistory {
    pub fn save_to_file(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        fs::write(path, json)?;
        Ok(())
    }

    pub fn load_from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = fs::read_to_string(path)?;
        let history: OptimizationHistory = serde_json::from_str(&json)?;
        Ok(history)
    }
}
