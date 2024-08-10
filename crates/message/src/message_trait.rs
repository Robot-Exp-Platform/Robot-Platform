use crate::collision_object::CollisionObject;
use crate::control_command::ControlCommand;
use crate::state::RobotState;
use crate::target::Target;
use crate::track::Track;

#[derive(Debug)]
pub enum Message {
    RobotState(RobotState),
    Target(Target),
    Track(Track),
    CollisionObject(CollisionObject),
    ControlCommand(ControlCommand),
}
