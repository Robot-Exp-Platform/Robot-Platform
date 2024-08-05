use crate::collision_object::CollisionObject;
use crate::control_command::ControlCommand;
use crate::state::State;
use crate::target::Target;
use crate::track::Track;

pub enum Message {
    State(State),
    Target(Target),
    Track(Track),
    CollisionObject(CollisionObject),
    ControlCommand(ControlCommand),
}
