use generate_tools::{get_fn, set_fn};
use message::{DNodeMessageQueue, NodeMessage, Pose};
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
use std::sync::{Arc, RwLock};

use crate::{Node, NodeBehavior, NodeState};
use robot::{DSeriseRobot, Robot, RobotType};

pub struct ExPlanner {
    name: String,
    state: ExPlannerState,
    params: ExPlannerParams,
    node: ExPlannerNode,
    robot: Option<Arc<RwLock<DSeriseRobot>>>,
}

#[derive(Default)]
struct ExPlannerState {
    target: Option<NodeMessage<na::DVector<f64>>>,
    node_state: NodeState,
    // Add state variables begin

    // Add state variables end
}

#[derive(Deserialize, Default)]
pub struct ExPlannerParams {
    period: f64,
    // Add parameters begin

    // Add parameters end
}

#[derive(Default)]
pub struct ExPlannerNode {
    input_queue: DNodeMessageQueue,
    output_queue: DNodeMessageQueue,
    // Add node variables begin

    // Add node variables end
}

impl ExPlanner {
    pub fn from_json(name: String, json: Value) -> ExPlanner {
        ExPlanner::from_params(name, from_value(json).unwrap())
    }

    pub fn from_params(name: String, params: ExPlannerParams) -> ExPlanner {
        ExPlanner {
            name,
            state: ExPlannerState::default(),
            params,
            node: ExPlannerNode::default(),
            robot: None,
        }
    }
}

impl Node<na::DVector<f64>> for ExPlanner {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot = Some(robot);
        }
    }
    fn set_sensor(&mut self, _: Arc<RwLock<sensor::Sensor>>) {}
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

// you can alter the `init`, `update`, `finalize` function to implement your own behavior for the robot

impl NodeBehavior for ExPlanner {
    fn init(&mut self) {
        // This function will be used once while the program begin

        // Demo begin
        {
            // set target from joint
            let joint = na::DVector::from_vec(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
            let target = NodeMessage::Joint(joint);
            self.node.output_queue.push(target);

            // set target from pose
            let translation = na::Translation::<f64, 3>::new(0.0, 0.0, 0.0);
            let rotation = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
            let pose = Pose::from_parts(translation, rotation);
            let target = NodeMessage::Pose(pose);
            self.node.output_queue.push(target);
        }
        // Demo end

        // TODO you can delete the demo code above and write your own code here
    }
    fn update(&mut self) {
        // This function will be used in every cycle, you should use `self.state.node_state = NodeState::Finished;` to stop loop

        // Demo begin
        {
            // get robot state
            let robot_read = self.robot.as_ref().unwrap().read().unwrap();
            let mut q = robot_read.q();
            drop(robot_read);

            q[0] += 0.01;

            self.state.target = Some(NodeMessage::Joint(q.clone()));
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
        self.state.node_state
    }
    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }
    fn node_name(&self) -> String {
        self.name.clone()
    }
}
