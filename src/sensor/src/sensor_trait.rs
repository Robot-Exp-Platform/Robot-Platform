use serde_json::Value;

use crate::ObstacleList;
use message::CollisionObject;

pub enum Sensor {
    ObstacleList(ObstacleList),
}

impl Sensor {
    pub fn name(&self) -> &String {
        match self {
            Sensor::ObstacleList(obstacle_list) => &obstacle_list.name,
        }
    }

    pub fn collision(&self) -> Vec<CollisionObject> {
        match self {
            Sensor::ObstacleList(obstacle_list) => obstacle_list.obstacle.clone(),
        }
    }

    pub fn params(&mut self, params: Value) {
        match self {
            Sensor::ObstacleList(obstacle_list) => {
                obstacle_list.set_params(params);
            }
        }
    }
}
