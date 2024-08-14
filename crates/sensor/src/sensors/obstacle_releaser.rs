use message::collision_object::CollisionObject;

#[allow(dead_code)]
pub struct ObstacleReleaser {
    obstacle: Vec<(String, CollisionObject)>,
}
