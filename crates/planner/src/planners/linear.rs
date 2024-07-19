use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex, RwLock};

use crate::planner_trait::Planner;
use recoder::recoder_trait::Recoder;
use robot::robot_trait::Robot;

#[derive(Serialize, Deserialize)]
pub struct LinearParams {
    // TODO params should be vec of 64,which has deferent length for deferent Robot
    interpolation: i32,
}

pub struct LinearNode {
    #[cfg(target_os = "unix")]
    sub_list: Vec<String>,
    #[cfg(target_os = "unix")]
    pub_list: Vec<String>,
}

pub struct Linear<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    params: LinearParams,

    _rosnode: LinearNode,

    #[allow(dead_code)]
    robot: Arc<RwLock<R>>,
}

impl<R: Robot + 'static, const N: usize> Linear<R, N> {
    pub fn new(
        name: String,
        path: String,
        params: LinearParams,
        robot: Arc<RwLock<R>>,
    ) -> Linear<R, N> {
        Linear {
            name,
            path,

            params,

            _rosnode: LinearNode {
                #[cfg(target_os = "unix")]
                sub_list: Vec::new(),
                #[cfg(target_os = "unix")]
                pub_list: Vec::new(),
            },
            robot,
        }
    }

    pub fn new_without_params(name: String, path: String, robot: Arc<RwLock<R>>) -> Linear<R, N> {
        Linear::new(name, path, LinearParams { interpolation: 0 }, robot)
    }
}

impl<R: Robot + 'static, const N: usize> Planner for Linear<R, N> {
    // fn get_planner_state(&self) -> PlannerState {
    //     self.state
    // }

    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_params(&self) -> Vec<f64> {
        vec![self.params.interpolation as f64]
    }

    fn add_planner(&mut self, _planner: Arc<Mutex<dyn Planner>>) {}
}

impl<R: Robot + 'static, const N: usize> Recoder for Linear<R, N> {
    fn recoder() {
        // TODO Recoder for Linear
    }
}
