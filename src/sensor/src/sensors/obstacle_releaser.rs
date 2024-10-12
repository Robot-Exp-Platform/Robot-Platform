use serde_json::Value;

use message::CollisionObject;

#[derive(Default)]
pub struct ObstacleReleaser {
    pub name: String,
    pub obstacle: Vec<(String, CollisionObject)>,
}

impl ObstacleReleaser {
    pub fn new(name: String) -> ObstacleReleaser {
        ObstacleReleaser {
            name,
            obstacle: Vec::new(),
        }
    }

    pub fn get_name(&self) -> String {
        self.name.clone()
    }
    pub fn get_collision(&self) -> Vec<CollisionObject> {
        self.obstacle.iter().map(|x| x.1).collect()
    }

    pub fn set_params(&mut self, params: Value) {
        self.obstacle = serde_json::from_value(params).unwrap();
    }
}
