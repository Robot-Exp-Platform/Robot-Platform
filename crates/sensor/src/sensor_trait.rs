use crate::sensors::obstacle_releaser::ObstacleReleaser;
use message::collision_object::CollisionObject;
use serde_json::Value;

pub enum Sensor {
    ObstacleReleaser(ObstacleReleaser),
}

impl Sensor {
    pub fn get_name(&self) -> &String {
        match self {
            Sensor::ObstacleReleaser(obstacle_releaser) => &obstacle_releaser.name,
        }
    }

    pub fn get_collision(&self) -> Vec<CollisionObject> {
        match self {
            Sensor::ObstacleReleaser(obstacle_releaser) => obstacle_releaser.get_collision(),
        }
    }

    pub fn set_params(&mut self, params: Value) {
        match self {
            Sensor::ObstacleReleaser(obstacle_releaser) => {
                obstacle_releaser.set_params(params);
            }
        }
    }
}
