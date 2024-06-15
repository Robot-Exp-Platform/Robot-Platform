use crate::controller_trait::Controller;
use nalgebra as na;
use recoder::recoder_trait::Recoder;
use robot::robot_trait::Robot;

pub struct Pid<'a, R: Robot, const N: usize> {
    name: String,
    path: String,

    state: PidState<N>,
    params: PidParams<N>,

    _rosnode: PidNode,
    robot: &'a R,
}
#[derive(Clone, Copy)]
pub struct PidState<const N: usize> {
    target: na::SVector<f64, N>,
    error: na::SVector<f64, N>,
    integral: na::SVector<f64, N>,
    derivative: na::SVector<f64, N>,
}

pub struct PidParams<const N: usize> {
    kp: na::SMatrix<f64, N, N>,
    ki: na::SMatrix<f64, N, N>,
    kd: na::SMatrix<f64, N, N>,
}

pub struct PidNode {
    #[cfg(target_os = "unix")]
    sub_list: Vec<ros::Subscriber>,
    #[cfg(target_os = "unix")]
    pub_list: Vec<ros::Publisher>,
}

impl<'a, R: Robot, const N: usize> Pid<'a, R, N> {
    pub fn new(name: String, path: String, params: PidParams<N>, robot: &'a R) -> Pid<'a, R, N> {
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

            _rosnode: PidNode {
                #[cfg(target_os = "unix")]
                sub_list: Vec::new(),
                #[cfg(target_os = "unix")]
                pub_list: Vec::new(),
            },
            robot,
        }
    }

    fn set_kp(&mut self, kp: na::SMatrix<f64, N, N>) {
        self.params.kp = kp;
    }
    fn set_ki(&mut self, ki: na::SMatrix<f64, N, N>) {
        self.params.ki = ki;
    }
    fn set_kd(&mut self, kd: na::SMatrix<f64, N, N>) {
        self.params.kd = kd;
    }
}

impl<'a, R: Robot, const N: usize> Controller for Pid<'a, R, N> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    // fn get_contoller_state(&self) -> ControllerState<N> {
    //     ControllerState::PidState(self.state)
    // }
    // fn set_params(&mut self, params: ControllerParams<N>) {
    //     if let ControllerParams::PidParams(pid_params) = params {
    //         self.params = pid_params;
    //     }
    // }

    fn init(&self) {
        #[cfg(target_os = "unix")]
        {
            // 在这里进行话题的声明，
            // 新建发布者和接收者，并将他们放入list中去
        }
    }
    fn update(&mut self, period: f64) {
        let new_error = self.state.target - self.robot.get_joint_positions();
        self.state.integral += new_error * period;
        self.state.derivative = (new_error - self.state.error) / period;
        self.state.error = new_error;

        let _control_output = self.params.kp * self.state.error
            + self.params.ki * self.state.integral
            + self.params.kd * self.state.derivative;

        #[cfg(target_os = "unix")]
        {
            // publish control_output
        }
    }
}

impl<'a, R: Robot, const N: usize> Recoder for Pid<'a, R, N> {
    fn recoder() {
        // TODO Recoder for Pid
    }
}
