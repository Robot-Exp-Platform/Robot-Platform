use message::collision_object::CollisionObject;

#[derive(Default)]
pub struct ObstacleReleaser {
    pub name: String,
    pub obstacle: Vec<(String, CollisionObject)>,
}

impl ObstacleReleaser {
    pub fn from_name(name: String) -> ObstacleReleaser {
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
}
