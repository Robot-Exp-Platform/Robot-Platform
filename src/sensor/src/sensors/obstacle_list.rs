use generate_tools::get_fn;
use serde_json::Value;

use message::{CollisionObject, Pose};

#[derive(Default)]
pub struct ObstacleList {
    pub name: String,
    pub obstacle: Vec<CollisionObject>,
}

impl ObstacleList {
    get_fn!((name: String));
    pub fn new(name: String) -> ObstacleList {
        ObstacleList {
            name,
            obstacle: Vec::new(),
        }
    }

    pub fn update_pose(&mut self, id: usize, pose: Pose) {
        for col_obj in self.obstacle.iter_mut() {
            if col_obj.id() == id {
                col_obj.set_pose(pose);
                return;
            }
        }
    }

    pub fn set_params(&mut self, params: Value) {
        self.obstacle = serde_json::from_value(params).unwrap();
    }
}
