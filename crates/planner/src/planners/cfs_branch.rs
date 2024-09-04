use crossbeam::queue::SegQueue;
use message::problem::QuadraticProgramming;
use message::{Constraint, Target};
use robot::robot_trait::BranchRobot;
use serde::Deserialize;
use serde_json::Value;
use solver::{OsqpSolver, Solver};
// use serde_yaml::Value;
use nalgebra as na;
use std::sync::{Arc, Mutex, RwLock};
use std::time::Duration;

use crate::{utilities, Planner, PlannerState};
use sensor::Sensor;
use task_manager::ROSThread;

pub struct CfsBranch {
    name: String,
    path: String,

    state: CfsBranchState,
    params: CfsBranchParams,

    node: CfsBranchNode,

    planners: Vec<Arc<Mutex<dyn Planner>>>,
    robot: Arc<RwLock<dyn BranchRobot>>,
}

#[derive(Default)]
pub struct CfsBranchState {
    target: Option<Target>,
}

#[derive(Deserialize, Default)]
pub struct CfsBranchParams {
    period: f64,
    interpolation: usize,
    niter: usize,
    cost_weight: Vec<f64>,
    solver: String,
}

#[derive(Default)]
pub struct CfsBranchNode {
    sensor: Option<Arc<RwLock<Sensor>>>,
    target_queue: Arc<SegQueue<Target>>,
    child_target_queue: Vec<Arc<SegQueue<Target>>>,
    state_collector: task_manager::StateCollector,
}

impl CfsBranch {
    pub fn new(name: String, path: String, robot: Arc<RwLock<dyn BranchRobot>>) -> Self {
        Self::from_planners(name, path, robot, vec![])
    }
    pub fn from_planners(
        name: String,
        path: String,
        robot: Arc<RwLock<dyn BranchRobot>>,
        planners: Vec<Arc<Mutex<dyn Planner>>>,
    ) -> Self {
        CfsBranch {
            name,
            path,
            state: CfsBranchState::default(),
            params: CfsBranchParams::default(),
            node: CfsBranchNode::default(),
            planners,
            robot,
        }
    }
}

impl Planner for CfsBranch {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_planner(&self) -> &Vec<Arc<Mutex<dyn Planner>>> {
        &self.planners
    }
    fn get_planner_state(&self) -> PlannerState {
        PlannerState::Unknow
    }

    fn set_params(&mut self, _: Value) {}
    fn set_sensor(&mut self, _: Arc<RwLock<Sensor>>) {}
    fn set_state_collector(&mut self, _: task_manager::StateCollector) {}
    fn set_target_queue(&mut self, _: Arc<SegQueue<Target>>) {}

    fn add_planner(&mut self, planner: Arc<Mutex<dyn Planner>>) {
        self.planners.push(planner);
        // 需要在此时完成对子节点的抑制以及与子节点通讯的搭建。
    }
}

impl ROSThread for CfsBranch {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }
    fn start(&mut self) {
        // 进入启动状态，并通知所有线程
        let (state, cvar) = &*self.node.state_collector;
        state.lock().unwrap().start();
        cvar.notify_all();

        // 进入循环状态，并通知所有线程
        state.lock().unwrap().update();
        cvar.notify_all();
    }
    fn update(&mut self) {
        // 检查上次任务是否完成, 若完成则获取新的 target
        let target = self
            .state
            .target
            .clone()
            .unwrap_or(self.node.target_queue.pop().unwrap());
        self.state.target = Some(target.clone());
        let (target_pose, indices, end_space_trans) = match target {
            // 根据不同的 target 类型，执行不同的任务，也可以将不同的 Target 类型处理为相同的类型
            Target::EndSpace(target_pose, robot_names, end_space_trans) => {
                let indices = self.robot.read().unwrap().get_robot_indices(robot_names);
                (target_pose, indices, end_space_trans)
            }
            _ => unimplemented!("unsupported target type"),
        };

        // 获取 robot 状态
        let q = self.robot.read().unwrap().get_q();

        // TODO 检查任务完成状态,如果已经完成,则将target更新为无

        // ! 第一次求解,意在求出参考路径,第一次求解所得的参考路径并不可靠,需要后续迭代求解

        // 临时变量初始化
        let ndof = indices.len();
        let dim = ndof * (self.params.interpolation + 1);

        // 构建第一次求解,建立参考路径.
        let mut constraint_task = Constraint::CartesianProduct(1, ndof, vec![]);

        // 根据任务生成等式约束,并重复  次
        let mut constraint_once = Constraint::CartesianProduct(1, 0, vec![]);
        // 生成每个机械臂的末端空间约束，
        // for i in 1..indices.len() {
        //     let diff = self.robot.read().unwrap().get_end_trans_difference_with_q(
        //         (indices[0], indices[i]),
        //         na::DVector::<f64>::zeros(7),
        //         na::DVector::<f64>::zeros(7),
        //         trans1,
        //         trans2,
        //     );
        // }
        for _ in 0..self.params.interpolation {
            constraint_task.push(constraint_once.clone());
        }

        // 生成目标函数
        let h = utilities::get_optimize_function(dim, ndof, self.params.cost_weight.clone());

        let f = vec![0.0; dim * (self.params.interpolation + 1)];

        // 第一次求解,获得满足任务约束的参考路径
        let problem = QuadraticProgramming {
            h: &h,
            f: &f,
            constraints: constraint_task,
        };
        let mut solver = match self.params.solver.as_str() {
            "osqp" => OsqpSolver::from_problem(problem),
            _ => unimplemented!(),
        };

        let mut solver_result = solver.solve();

        // TODO 检验参考轨迹是否安全可靠,取不可靠段做重规划
        // TODO 理论上最新轨迹应该为被重新规划的段落
        let mut last_result = solver_result.clone();

        // 已获得初始轨迹,新增避障约束和关节约束后,进行迭代求解

        for _ in 0..self.params.niter {
            // 提前分配空间
            let constraints = Constraint::CartesianProduct(
                0,
                0,
                Vec::with_capacity(self.params.interpolation + 1),
            );

            // TODO 建立约束,包括起点终点约束、障碍物约束、关节边界约束、任务约束

            // 求解优化问题
            solver.update_constraints(constraints);
            solver_result = solver.solve();

            // 检查是否收敛,更新参考轨迹
            if !last_result.is_empty() {
                let diff: f64 = solver_result
                    .iter()
                    .zip(last_result.iter())
                    .map(|(a, b)| (a - b).abs())
                    .sum();
                println!("diff: {}", diff);
                if diff.abs() < 1e-1 {
                    break;
                }
            }
            last_result = solver_result.clone();
        }

        // TODOm 根据indptr 为每个robot 实现 track,单实际上通过 child_target_queue 传递给子节点
    }
    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
