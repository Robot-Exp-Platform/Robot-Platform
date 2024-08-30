use std::sync::{Arc, Mutex, RwLock};

use crate::{Cfs, Linear, Planner, PlannerList, PlannerN};
use robot::{Robot, SeriesRobot};

pub fn create_planner<R: SeriesRobot<N> + 'static, const N: usize>(
    planner_type: String,
    robot_name: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn PlannerN<N>>> {
    // !跟着文家新建 Planner 啦啦啦
    let name = format!("{}:{}", planner_type, robot_name);
    match planner_type.as_str() {
        "linear" => Arc::new(Mutex::new(Linear::<R, N>::new(name, path, robot))),
        "cfs" => Arc::new(Mutex::new(Cfs::<R, N>::new(name, path, robot))),
        _ => panic!("Planner type not found"),
    }
}

pub fn create_planner_branch<R: Robot>(
    planner_type: String,
    robot_name: String,
    path: String,
) -> Arc<Mutex<dyn Planner>> {
    let name = format!("{}:{}", planner_type, robot_name);
    match planner_type.as_str() {
        "planner_list" => Arc::new(Mutex::new(PlannerList::new(name, path))),
        _ => unimplemented!(),
    }
}
