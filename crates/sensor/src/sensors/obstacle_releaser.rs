use message::collision_object::CollisionObject;

pub struct ObstacleReleaser {
    obstacle: Vec<(String, CollisionObject)>,
}
