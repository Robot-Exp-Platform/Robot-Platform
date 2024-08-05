use robot::robot_trait::Pose;

#[derive(Debug)]
pub enum Target {
    Pose(Pose),
}
