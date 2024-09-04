use nalgebra as na;

use message::CollisionObject;
use message::Constraint;
use robot::Robot;

pub fn distance_with_collision_object(
    _robot: &dyn Robot,
    _joint: &na::DVector<f64>,
    _obj: &CollisionObject,
) -> Constraint {
    unimplemented!()
}

pub fn end_pose(
    _robot: &dyn Robot,
    _joint: &na::DVector<f64>,
    _obj: &CollisionObject,
) -> Constraint {
    unimplemented!()
}

// 这个函数是不是应该放在 branchrobot 中？
pub fn end_space<R1, R2>(
    _robot1: R1,
    _robot2: R2,
    _joint1: na::DVector<f64>,
    _joint2: na::DVector<f64>,
    _trans1: na::Isometry3<f64>,
    _trans2: na::Isometry3<f64>,
) -> Constraint
where
    R1: Robot,
    R2: Robot,
{
    unimplemented!()
}
