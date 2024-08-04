use crate::collision_object::CollisionObject;
use crate::state::State;
use crate::target::Target;
use crate::track::Track;

pub enum Massage {
    State(State),
    Target(Target),
    Track(Track),
    CollisionObject(CollisionObject),
}
