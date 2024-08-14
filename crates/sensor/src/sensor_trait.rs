use crate::sensors::obstacle_releaser::ObstacleReleaser;

pub enum Sensor {
    ObstacleReleaser(ObstacleReleaser),
}

impl Sensor {
    pub fn get_name(&self) -> &String {
        match self {
            Sensor::ObstacleReleaser(obstacle_releaser) => &obstacle_releaser.name,
        }
    }
}
