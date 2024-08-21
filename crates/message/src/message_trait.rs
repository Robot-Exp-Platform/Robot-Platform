use crate::collision_object::CollisionObject;
use crate::constraint::Constraint;
use crate::control_command::ControlCommand;
use crate::problem::Problem;
use crate::state::NodeState;
use crate::state::RobotState;
use crate::target::Target;
use crate::track::Track;

#[derive(Debug)]
pub enum Message<'a> {
    CollisionObject(CollisionObject),
    Constraint(Constraint),
    ControlCommand(ControlCommand),
    Problem(Problem<'a>),
    NodeState(NodeState),
    RobotState(RobotState),
    Target(Target),
    Track(Track),
}
