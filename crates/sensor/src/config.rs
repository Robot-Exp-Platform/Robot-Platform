use std::sync::{Arc, RwLock};

use crate::{sensor_trait::Sensor, sensors::obstacle_releaser::ObstacleReleaser};

pub fn create_sensor(sensor_type: &String, name: &String) -> Arc<RwLock<Sensor>> {
    match sensor_type.as_str() {
        "obstacle_releaser" => Arc::new(RwLock::new(Sensor::ObstacleReleaser(
            ObstacleReleaser::from_name(name.to_owned()),
        ))),
        _ => panic!("Sensor type not found,{}", sensor_type),
    }
}
