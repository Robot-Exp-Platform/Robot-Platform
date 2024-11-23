use serde::Deserialize;
use serde_json::Value;
use std::sync::{Arc, RwLock};

use crate::{ObstacleList, Sensor};

#[derive(Debug, Deserialize)]
pub struct SensorConfig {
    pub name: String,
    pub sensor_type: String,
    pub params: Value,
}

pub fn from_config(sensor_config: &SensorConfig) -> Arc<RwLock<Sensor>> {
    match sensor_config.sensor_type.as_str() {
        "obstacle_list" => Arc::new(RwLock::new(Sensor::ObstacleList(ObstacleList::new(
            sensor_config.name.clone(),
            sensor_config.params.clone(),
        )))),
        _ => panic!("Unknown sensor type: {}", sensor_config.sensor_type),
    }
}
