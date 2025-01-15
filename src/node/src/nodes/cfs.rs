use kernel_macro::node_registration;
use nalgebra as na;
use serde::Deserialize;
use std::time::Duration;
use tracing::info;

use crate::{utilities::*, Node, NodeBehavior, NodeExtBehavior, NodeRegister, NodeState};
use message::{iso_to_vec, Constraint, DNodeMessage, NodeMessage, QuadraticProgramming};
use robot::{DRobot, DSeriseRobot, Robot, RobotLock};
use solver::{OsqpSolver, Solver};

pub type Cfs<R, V> = Node<CfsState<V>, CfsParams, RobotLock<R>, V>;

#[node_registration("cfs")]
pub type DCfs = Cfs<DSeriseRobot, na::DVector<f64>>;
pub type SCfs<R, const N: usize> = Cfs<R, na::SVector<f64, N>>;

#[derive(Default)]
pub struct CfsState<V> {
    target: Option<NodeMessage<V>>,
}

#[derive(Deserialize)]
pub struct CfsParams {
    period: f64,
    ninterp: usize,
    niter: usize,
    cost_weight: Vec<f64>,
    solver: String,
}

impl NodeBehavior for DCfs {
    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let ndof = robot_read.dof();
        let q = robot_read.q();
        let q_min_bound = robot_read.q_min_bound().as_slice().to_vec();
        let q_max_bound = robot_read.q_max_bound().as_slice().to_vec();

        let currect_state = DNodeMessage::Joint(q.clone());
        // println!("{}: currect_pose: {:?}", self.name(), robot_read.end_pose());
        // println!("{}: currect_q: {:?}", self.name(), q);

        // 检查当前状态是否达到储存的目标状态.
        if let Some(target) = self.state.target.clone() {
            if (target / currect_state).abs() < 1e-2 {
                self.state.target = None
            }
        }

        // 获取 target, 如果两侧都没有消息那就说明任务完成了，皆大欢喜
        // 这里的策略是完成当前任务优先
        let target = self.state.target.clone().or_else(|| self.input_queue.pop());
        if target.is_none() {
            self.node_state = NodeState::Finished;
            return;
        }
        let target = target.unwrap();
        info!(node = self.name.as_str(), input = ?target.as_slice());

        // 准备
        let q_ref_list = match &target {
            NodeMessage::Joint(q_target) => lerp(&q, &vec![q_target.clone()], self.params.ninterp),
            NodeMessage::Pose(_) => lerp(&q, &vec![q.clone()], self.params.ninterp),
            _ => panic!("Cfs: Unsupported message type"),
        };
        let collision_objects = self.sensor.as_ref().unwrap().read().unwrap().collision();
        let mut last_result = Vec::new();
        let dim = (self.params.ninterp + 2) * ndof;

        for _ in 0..self.params.niter {
            // =======  建立约束  =======

            // 提前分配空间
            let mut constraints =
                Constraint::CartesianProduct(0, 0, Vec::with_capacity(self.params.ninterp + 1));

            // 起点位置的约束，这是绝对约束
            constraints.push(Constraint::Equared(q.as_slice().to_vec()));

            // 中间过程中的约束
            for q_ref in q_ref_list.iter() {
                // 边界约束
                let mut process_constraint =
                    Constraint::Rectangle(q_min_bound.clone(), q_max_bound.clone());

                // 如果有障碍物的话，增加碰撞约束

                for collision in &collision_objects {
                    let func = |q: &na::DVector<f64>| robot_read.cul_dis_to_collision(q, collision);
                    let (dis, grad) = robot_read.cul_func(q_ref, &func);
                    // 过程中对每个障碍物的碰撞约束与关节角约束
                    process_constraint += Constraint::Halfspace(
                        (-&grad).as_slice().to_vec(),
                        (dis - (&grad * q_ref))[(0, 0)],
                    );
                }

                constraints.push(process_constraint);
            }

            // 终点约束
            match target {
                NodeMessage::Joint(ref q) => {
                    constraints.push(Constraint::Equared(q.clone().as_slice().to_vec()));
                }
                NodeMessage::Pose(ref_pose) => {
                    let mut end_constraint =
                        Constraint::Rectangle(q_min_bound.clone(), q_max_bound.clone());

                    let q_end_ref = if last_result.is_empty() {
                        q.clone()
                    } else {
                        na::DVector::from_vec(last_result[last_result.len() - ndof..].to_vec())
                    };

                    // 在这里写一个测试，用于测试 求解器是否能迭代出一组合理的解
                    // test_pose_constraint(ref_pose, &q_end_ref, &robot_read);

                    let func = |q: &na::DVector<f64>| iso_to_vec(robot_read.cul_end_pose(q));
                    let (value, grad) = robot_read.cul_func(&q_end_ref, &func);

                    let b_bar = iso_to_vec(ref_pose) - value + &grad * q_end_ref;
                    end_constraint += Constraint::Hyperplane(
                        grad.nrows(),
                        grad.ncols(),
                        grad.as_slice().to_vec(),
                        b_bar.as_slice().to_vec(),
                    );

                    constraints.push(end_constraint);
                }
                _ => panic!("Cfs: Unsupported message type"),
            }

            // =======  优化  =======
            let h = get_optimize_function(dim, ndof, self.params.cost_weight.clone());
            let f = na::DVector::<f64>::zeros(dim);
            let problem = QuadraticProgramming {
                h: &h,
                f: f.as_slice(),
                constraints,
            };

            let solver_result = match self.params.solver.as_str() {
                "osqp" => {
                    let mut osqp_solver = OsqpSolver::from_problem(problem);
                    osqp_solver.solve()
                }
                _ => unimplemented!(),
            };

            // 检查是否收敛，更新 q_ref_list

            if last_result.is_empty() {
                last_result = solver_result.clone();
            } else {
                // 一个比较严苛的收敛条件，学长使用的是平均值，而我采用的是轨迹的误差的和，可能会导致部分情况下收敛不通过
                let diff: f64 = solver_result
                    .iter()
                    .zip(last_result.iter())
                    .map(|(a, b)| (a - b).abs())
                    .sum();
                if diff.abs() < 1e-1 {
                    break;
                }
                last_result = solver_result.clone();
            }
        }
        // =======  轨迹发送  =======
        // 生成 track
        let mut track_list = Vec::new();
        for i in 1..self.params.ninterp + 2 {
            track_list.push(DNodeMessage::Joint(na::DVector::from_column_slice(
                &last_result[i * ndof..(i + 1) * ndof],
            )));
        }
        // 发送 track
        while self.output_queue.pop().is_some() {}
        for track in track_list {
            self.output_queue.push(track);
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
    fn node_name(&self) -> String {
        self.name.clone()
    }
    fn state(&mut self) -> NodeState {
        self.node_state
    }
}
