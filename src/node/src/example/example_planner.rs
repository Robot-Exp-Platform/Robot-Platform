use message::{NodeMessage, Pose};
use nalgebra as na;
use rand::Rng;
use robot::{Gripper, RobotLock};
use serde::Deserialize;

use crate::{Node, NodeBehavior, NodeState};
use robot::{DSeriseRobot, Robot};

pub type ExPlanner = Node<
    ExPlannerState,
    ExPlannerParams,
    (RobotLock<DSeriseRobot>, RobotLock<Gripper>),
    na::DVector<f64>,
>;

#[derive(Default)]
pub struct ExPlannerState {
    // Add state variables begin

    // Add state variables end
}

#[derive(Deserialize)]
pub struct ExPlannerParams {
    period: f64,
    // Add parameters begin

    // Add parameters end
}

// you can alter the `init`, `update`, `finalize` function to implement your own behavior for the robot
impl NodeBehavior for ExPlanner {
    fn init(&mut self) {
        // This function will be used once while the program begin

        // Demo begin
        {
            // set target from joint
            let joint = na::DVector::from_vec(vec![
                0.0124, -0.8838, 0.3749, -2.2172, 0.232, 1.7924, 1.3719,
            ]);
            let target = NodeMessage::Joint(joint);
            self.output_queue.push(target);

            // set target from pose
            let translation = na::Translation::<f64, 3>::new(0.0, 0.0, 0.0);
            let rotation = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
            let pose = Pose::from_parts(translation, rotation);
            let target = NodeMessage::Pose(pose);
            self.output_queue.push(target);
        }
        // Demo end

        // TODO you can delete the demo code above and write your own code here
    }
    fn update(&mut self) {
        // This function will be used in every cycle, you should use `self.state.node_state = NodeState::Finished;` to stop loop

        // Demo begin
        {
            // get robot state
            let robot_read = self.robot.0.as_ref().unwrap().read().unwrap();
            let _q = robot_read.q();
            let q_min_bound = robot_read.q_min_bound();
            let q_max_bound = robot_read.q_max_bound();
            drop(robot_read);

            // Generate a random q_target within the bounds

            let mut rng = rand::thread_rng();
            let len = q_min_bound.len();

            let q_target = na::DVector::from_iterator(
                len,
                (0..len).map(|i| {
                    let center = (q_min_bound[i] + q_max_bound[i]) / 2.0;
                    let half_range = (q_max_bound[i] - q_min_bound[i]) / 4.0;
                    rng.gen_range((center - half_range)..=(center + half_range))
                }),
            );

            let mut gripper_write = self.robot.1.as_mut().unwrap().write().unwrap();
            if gripper_write.width() > 0.01 {
                gripper_write.grasp(0.01);
            } else {
                gripper_write.home();
            }

            // // Interpolate between current q and q_target using lerp
            // let trace = lerp(&q, &vec![q_target], 50);
            let trace = vec![q_target];

            // Set target from trace
            for q in trace {
                let target = NodeMessage::Joint(q);
                self.output_queue.push(target);
            }
        }
        // Demo end

        // TODO you can delete the demo code above and write your own code here
    }
    fn finalize(&mut self) {
        // This function will be used once while the program end

        // TODO you can write your own code here
    }

    // 以下几个函数不要修改，这是供框架调用的函数
    fn state(&mut self) -> NodeState {
        self.node_state
    }
    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }
    fn node_name(&self) -> String {
        self.name.clone()
    }
}
