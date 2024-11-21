use generate_tools::get_fn;
use serde_json::Value;

use message::CollisionObject;

#[derive(Default)]
pub struct ObstacleList {
    pub name: String,
    pub obstacle: Vec<CollisionObject>,
}

impl ObstacleList {
    pub fn new(name: String) -> ObstacleList {
        ObstacleList {
            name,
            obstacle: Vec::new(),
        }
    }

    get_fn!((name: String));

    pub fn set_params(&mut self, params: Value) {
        self.obstacle = serde_json::from_value(params).unwrap();
    }
}
