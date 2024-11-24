use message::{NodeMessage, Pose};
use nalgebra as na;
use std::sync::{Arc, RwLock};

use super::DSeriseRobot;
use crate::Robot;

macro_rules! merge_fn {
    ($($fn_name:ident),*) => {
        $(
            fn $fn_name(&self) -> na::DVector<f64> {
                self.merge(self.robots.iter().map(|robot| robot.read().unwrap().$fn_name()).collect())
            }
        )*
    };
}

macro_rules! split_fn {
    ($($fn_name:ident),*) => {
        $(
            fn $fn_name(&mut self, combined_vector: na::DVector<f64>) {
                self.split(combined_vector).into_iter().zip(&self.robots).for_each(|(vector, robot)| {
                    robot.write().unwrap().$fn_name(vector)
                });
            }
        )*
    };
}

#[derive(Default)]
pub struct RobotBranch<R> {
    robots: Vec<Arc<RwLock<R>>>,
    indices: Vec<usize>,
}

type DRobotBranch = RobotBranch<DSeriseRobot>;

impl DRobotBranch {
    pub fn indices(&self) -> &Vec<usize> {
        &self.indices
    }

    pub fn new() -> DRobotBranch {
        DRobotBranch {
            robots: Vec::new(),
            indices: Vec::new(),
        }
    }

    pub fn push(&mut self, robot: Arc<RwLock<DSeriseRobot>>) {
        if self.indices.is_empty() {
            self.indices.push(0);
        }
        self.indices
            .push(self.indices.last().unwrap() + robot.read().unwrap().dof());
        self.robots.push(robot);
    }

    /// 合并多机器人的向量
    pub fn merge(&self, vectors: Vec<na::DVector<f64>>) -> na::DVector<f64> {
        let total_dof = self.dof();
        let mut combined_vector = na::DVector::zeros(total_dof);
        let mut offset = 0;
        for vector in vectors {
            combined_vector
                .rows_mut(offset, vector.len())
                .copy_from(&vector);
            offset += vector.len();
        }
        combined_vector
    }

    /// 拆分向量为多机器人的向量
    pub fn split(&self, vector: na::DVector<f64>) -> Vec<na::DVector<f64>> {
        let mut vectors = Vec::new();
        for i in 0..self.indices.len() - 1 {
            let start = self.indices[i];
            let end = self.indices[i + 1] + 1;
            vectors.push(vector.rows(start, end - start).into_owned());
        }
        vectors
    }
}

impl Robot<na::DVector<f64>> for DRobotBranch {
    merge_fn!(
        q,
        q_dot,
        q_ddot,
        q_jerk,
        q_default,
        q_max_bound,
        q_min_bound,
        q_dot_bound,
        q_ddot_bound,
        q_jerk_bound,
        tau_bound,
        tau_dot_bound
    );

    split_fn!(set_q, set_q_dot, set_q_ddot, set_q_jerk);

    fn name(&self) -> String {
        "RobotBranch".to_string()
    }
    fn dof(&self) -> usize {
        self.indices.last().unwrap() + 1
    }
    fn base(&self) -> Pose {
        Pose::default()
    }
    fn control_message(&self) -> NodeMessage<na::DVector<f64>> {
        NodeMessage::default()
    }

    fn set_name(&mut self, _: String) {}
    fn set_control_message(&mut self, _: NodeMessage<na::DVector<f64>>) {}
}
