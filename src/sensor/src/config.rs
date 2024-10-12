use serde::Deserialize;
use std::sync::{Arc, RwLock};

use crate::{ObstacleReleaser, Sensor};

#[derive(Debug, Deserialize)]
pub struct SensorConfig {
    pub name: String,
    pub sensor_type: String,
}

pub fn from_config(sensor_config: SensorConfig) -> Arc<RwLock<Sensor>> {
    match sensor_config.sensor_type.as_str() {
        "obstacle_relaser" => Arc::new(RwLock::new(Sensor::ObstacleReleaser(
            ObstacleReleaser::new(sensor_config.name),
        ))),
        _ => panic!("Unknown sensor type: {}", sensor_config.sensor_type),
    }
}
