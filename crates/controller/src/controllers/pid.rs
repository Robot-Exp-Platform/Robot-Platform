use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::{Deserialize, Deserializer};
use std::sync::{Arc, Mutex, RwLock};

use crate::controller_trait::Controller;
use massage::track::Track;
use robot::robot_trait::Robot;
use task_manager::ros_thread::ROSThread;

#[allow(dead_code)]
pub struct Pid<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    state: PidState<N>,
    params: PidParams<N>,

    msgnode: PidNode,
    robot: Arc<RwLock<R>>,
}
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct PidState<const N: usize> {
    target: na::SVector<f64, N>,
    error: na::SVector<f64, N>,
    integral: na::SVector<f64, N>,
    derivative: na::SVector<f64, N>,
}

#[allow(dead_code)]
pub struct PidParams<const N: usize> {
    period: f64,
    kp: na::SMatrix<f64, N, N>,
    ki: na::SMatrix<f64, N, N>,
    kd: na::SMatrix<f64, N, N>,
}

impl<'de, const N: usize> Deserialize<'de> for PidParams<N> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct PidParamsData {
            period: f64,
            kp: Vec<f64>,
            ki: Vec<f64>,
            kd: Vec<f64>,
        }

        let data = PidParamsData::deserialize(deserializer)?;

        assert_eq!(data.kp.len(), N * N, "Matrix data has incorrect length");
        assert_eq!(data.ki.len(), N * N, "Matrix data has incorrect length");
        assert_eq!(data.kd.len(), N * N, "Matrix data has incorrect length");

        Ok(PidParams {
            period: data.period,
            kp: na::SMatrix::from_vec(data.kp),
            ki: na::SMatrix::from_vec(data.ki),
            kd: na::SMatrix::from_vec(data.kd),
        })
    }
}

pub struct PidNode {
    track_queue: Arc<SegQueue<Track>>,
    // #[cfg(feature = "zmq")]
    // pub sub_list: Vec<zmq::Socket>,
    #[cfg(feature = "ros")]
    pub sub_list: Vec<rosrust::Subscriber>,
}

impl<R: Robot + 'static, const N: usize> Pid<R, N> {
    pub fn new(
        name: String,
        path: String,
        params: PidParams<N>,
        robot: Arc<RwLock<R>>,
    ) -> Pid<R, N> {
        Pid {
            name,
            path,

            state: PidState {
                target: na::SVector::from_element(0.0),
                error: na::SVector::from_element(0.0),
                integral: na::SVector::from_element(0.0),
                derivative: na::SVector::from_element(0.0),
            },
            params,

            msgnode: PidNode {
                track_queue: Arc::new(SegQueue::new()),
                #[cfg(feature = "zmq")]
                sub_list: Vec::new(),
                #[cfg(feature = "ros")]
                sub_list: Vec::new(),
            },
            robot,
        }
    }

    pub fn new_without_params(name: String, path: String, robot: Arc<RwLock<R>>) -> Pid<R, N> {
        Pid::new(
            name,
            path,
            PidParams {
                period: 0.0,
                kp: na::SMatrix::from_element(0.0),
                ki: na::SMatrix::from_element(0.0),
                kd: na::SMatrix::from_element(0.0),
            },
            robot,
        )
    }

    // fn set_kp(&mut self, kp: na::SMatrix<f64, N, N>) {
    //     self.params.kp = kp;
    // }
    // fn set_ki(&mut self, ki: na::SMatrix<f64, N, N>) {
    //     self.params.ki = ki;
    // }
    // fn set_kd(&mut self, kd: na::SMatrix<f64, N, N>) {
    //     self.params.kd = kd;
    // }
}

impl<R: Robot + 'static, const N: usize> Controller for Pid<R, N> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn set_params(&mut self, params: String) {
        let params: PidParams<N> = serde_json::from_str(&params).unwrap();
        self.params = params;
    }
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>) {
        self.msgnode.track_queue = track_queue;
    }

    fn add_controller(&mut self, _: Arc<Mutex<dyn Controller>>) {}
}

impl<R: Robot + 'static, const N: usize> ROSThread for Pid<R, N> {
    fn init(&mut self) {
        #[cfg(feature = "ros")]
        {
            self.msgnode.sub_list.push(
                rosrust::subscribe(&self.name, 1, move |_: rosrust_msg::std_msgs::Float32| {
                    // TODO 从消息中提取数据
                })
                .unwrap(),
            );
        }
    }
    fn start(&mut self) {}

    fn update(&mut self) {
        let robot_read = self.robot.read().unwrap();
        let q = na::SVector::from_row_slice(&robot_read.get_q()[..N]);

        let new_error = self.state.target - q;
        self.state.integral += new_error * self.params.period;
        self.state.derivative = (new_error - self.state.error) / self.params.period;
        self.state.error = new_error;

        let _control_output = self.params.kp * self.state.error
            + self.params.ki * self.state.integral
            + self.params.kd * self.state.derivative;
    }
}
