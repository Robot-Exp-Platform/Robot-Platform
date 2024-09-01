use serde_json::Value;
// use serde_yaml::Value;
use std::{
    any::Any,
    sync::{Arc, Mutex, RwLock},
};

use crate::{
    planner_trait::{CfsTrait, OptimizationBasedPlanner},
    Cfs, Planner, PlannerState,
};
use sensor::Sensor;
use task_manager::ROSThread;

pub struct CfsBranch {
    name: String,
    path: String,

    planners: Vec<Arc<Mutex<dyn Planner>>>,
    cfsplanners: Vec<Arc<Mutex<dyn CfsTrait>>>,
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
}

impl ROSThread for CfsBranch {
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {
        // 获取target target，发给枝节点的target一般都是任务约束相关的目标，当存在枝节点目标时，需要由枝节点拆分目标给子……需要吗？
        // 获取机器人状态
        // 获取约束
        // 整理约束
        // 提交约束或求解
        // 将求解结果发布到子规划器
    }
}
