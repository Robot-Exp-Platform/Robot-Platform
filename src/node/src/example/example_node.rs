use message::NodeMessage;
use serde::Deserialize;

use crate::{Node, NodeBehavior};

type ExNode = Node<ExNodeState, ExNodeParams, (), f64>;

#[derive(Default)]
struct ExNodeState {
    target: f64,
    error: f64,
    integral: f64,
    derivative: f64,
}

#[derive(Deserialize)]
struct ExNodeParams {
    period: f64,
    kp: f64,
    ki: f64,
    kd: f64,
}

impl NodeBehavior for ExNode {
    fn update(&mut self) {
        if let Some(NodeMessage::Joint(target)) = self.input_queue.pop() {
            self.state.target = target;
        }

        let new_error = self.state.target;
        self.state.integral += new_error * self.params.period;
        self.state.derivative = (new_error - self.state.error) / self.params.period;
        self.state.error = new_error;

        let output = self.params.kp * self.state.error
            + self.params.ki * self.state.integral
            + self.params.kd * self.state.derivative;

        let control_message = NodeMessage::Joint(output);
        self.output_queue.push(control_message);
    }
}
