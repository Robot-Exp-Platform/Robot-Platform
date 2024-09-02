use message::{target, Target};
use serde::Deserialize;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};
use std::time::Duration;

use crate::{
    planner_trait::{CfsTrait, OptimizationBasedPlanner},
    Planner, PlannerState,
};
use sensor::Sensor;
use task_manager::ROSThread;

pub struct CfsBranch {
    name: String,
    path: String,

    state: CfsBranchState,
    params: CfsBranchParams,

    node: CfsBranchNode,

    planners: Vec<Arc<Mutex<dyn Planner>>>,
}

#[derive(Default)]
pub struct CfsBranchState {
    solve_suppressor: bool,
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
    target_queue: Arc<crossbeam::queue::SegQueue<message::Target>>,
    state_collector: task_manager::StateCollector,
}

impl CfsBranch {
    pub fn new(name: String, path: String) -> Self {
        Self::from_planners(name, path, vec![])
    }
    pub fn from_planners(
        name: String,
        path: String,
        planners: Vec<Arc<Mutex<dyn Planner>>>,
    ) -> Self {
        CfsBranch {
            name,
            path,
            state: CfsBranchState::default(),
            params: CfsBranchParams::default(),
            node: CfsBranchNode::default(),
            planners,
        }
    }
}

impl CfsTrait for CfsBranch {}

impl OptimizationBasedPlanner for CfsBranch {
    fn supperssion(&mut self) {}
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
    fn set_target_queue(&mut self, _: Arc<crossbeam::queue::SegQueue<message::Target>>) {}

    fn add_planner(&mut self, planner: Arc<Mutex<dyn Planner>>) {
        self.planners.push(planner);
        // 需要在此时完成对子节点的抑制以及与子节点通讯的搭建。
    }

    fn as_cfs_planner(&self) -> Option<&dyn CfsTrait> {
        Some(self)
    }
}

impl ROSThread for CfsBranch {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }
    fn start(&mut self) {}
    fn update(&mut self) {
        // 更新 target
        let target = match self.node.target_queue.pop() {
            Some(Target::EndSpace(target_pose, relative_list)) => {}
            None => return,
            _ => unimplemented!("unsupported target type"),
        };
    }
    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
