use nalgebra as na;
use nalgebra::Isometry;

use crate::robot_trait::Robot;
use crate::robot_trait::SeriesRobot;
use message::collision_object::{get_distance, Capsule};
use message::{CollisionObject, ControlCommandN, Pose, RobotMessageN};

#[allow(dead_code)]
pub struct RobotNDof<const N: usize, const N_ADD_ONE: usize> {
    name: String,
    path: String,

    state: RobotNDofState<N>,
    params: RobotNDofParams<N, N_ADD_ONE>,
}

#[derive(Clone, Copy)]
pub struct RobotNDofState<const N: usize> {
    // 机器人状态,在运行状态下将会实时改变
    q: na::SVector<f64, N>,
    q_dot: na::SVector<f64, N>,
    q_ddot: na::SVector<f64, N>,
    q_jerk: na::SVector<f64, N>,
    base_pose: Pose,
}

#[derive(Clone, Copy)]
pub struct RobotNDofParams<const N: usize, const N_ADD_ONE: usize> {
    // 机器人参数,在运行状态下一般不会改变
    pub nlink: usize,
    pub q_default: na::SVector<f64, N>,
    pub q_min_bound: na::SVector<f64, N>,
    pub q_max_bound: na::SVector<f64, N>,
    pub q_dot_bound: na::SVector<f64, N>,
    pub q_ddot_bound: na::SVector<f64, N>,
    pub q_jerk_bound: na::SVector<f64, N>,
    pub tau_bound: na::SVector<f64, N>,
    pub tau_dot_bound: na::SVector<f64, N>,
    pub denavit_hartenberg: na::SMatrix<f64, N_ADD_ONE, 4>,
    pub capsules: [Capsule; N_ADD_ONE],
}

impl<const N: usize, const N_ADD_ONE: usize> RobotNDof<N, N_ADD_ONE> {
    pub fn new_from_params(
        name: String,
        path: String,
        params: RobotNDofParams<N, N_ADD_ONE>,
    ) -> RobotNDof<N, N_ADD_ONE> {
        RobotNDof {
            name,
            path,

            state: RobotNDofState::new(),
            params,
        }
    }

    pub fn new_without_params(name: String, path: String) -> RobotNDof<N, N_ADD_ONE> {
        RobotNDof::new_from_params(name, path, RobotNDofParams::new())
    }

    fn check_joint(&self, joint: &na::SVector<f64, N>) -> bool {
        for i in 0..N {
            if joint[i] < self.params.q_min_bound[i] || joint[i] > self.params.q_max_bound[i] {
                return false;
            }
        }
        true
    }
    fn check_vel(&self, vel: &na::SVector<f64, N>) -> bool {
        for i in 0..N {
            if vel[i] > self.params.q_dot_bound[i] {
                return false;
            }
        }
        true
    }
    fn check_acc(&self, acc: &na::SVector<f64, N>) -> bool {
        for i in 0..N {
            if acc[i] > self.params.q_ddot_bound[i] {
                return false;
            }
        }
        true
    }
    fn check_tau(&self, tau: &na::SVector<f64, N>) -> bool {
        for i in 0..N {
            if tau[i] > self.params.tau_bound[i] {
                return false;
            }
        }
        true
    }
}

impl<const N: usize, const N_ADD_ONE: usize> RobotNDofParams<N, N_ADD_ONE> {
    fn new() -> RobotNDofParams<N, N_ADD_ONE> {
        RobotNDofParams {
            nlink: N,
            q_default: na::SVector::from_element(0.0),
            q_min_bound: na::SVector::from_element(0.0),
            q_max_bound: na::SVector::from_element(0.0),
            q_dot_bound: na::SVector::from_element(0.0),
            q_ddot_bound: na::SVector::from_element(0.0),
            q_jerk_bound: na::SVector::from_element(0.0),
            tau_bound: na::SVector::from_element(0.0),
            tau_dot_bound: na::SVector::from_element(0.0),
            denavit_hartenberg: na::SMatrix::from_element(0.0),
            capsules: [Capsule::default(); N_ADD_ONE],
        }
    }
}

impl<const N: usize> RobotNDofState<N> {
    fn new() -> RobotNDofState<N> {
        RobotNDofState {
            q: na::SVector::from_element(0.0),
            q_dot: na::SVector::from_element(0.0),
            q_ddot: na::SVector::from_element(0.0),
            q_jerk: na::SVector::from_element(0.0),
            base_pose: Pose::identity(),
        }
    }
}

impl<const N: usize, const N_ADD_ONE: usize> SeriesRobot<N> for RobotNDof<N, N_ADD_ONE> {
    fn get_q(&self) -> na::SVector<f64, N> {
        self.state.q
    }
    fn get_q_dot(&self) -> na::SVector<f64, N> {
        self.state.q_dot
    }
    fn get_q_ddot(&self) -> na::SVector<f64, N> {
        self.state.q_ddot
    }
    fn get_q_jack(&self) -> na::SVector<f64, N> {
        self.state.q_jerk
    }
    fn get_q_min_bound(&self) -> na::SVector<f64, N> {
        self.params.q_min_bound
    }
    fn get_q_max_bound(&self) -> na::SVector<f64, N> {
        self.params.q_max_bound
    }
    fn get_q_dot_bound(&self) -> nalgebra::SVector<f64, N> {
        self.params.q_dot_bound
    }
    fn get_q_ddot_bound(&self) -> nalgebra::SVector<f64, N> {
        self.params.q_ddot_bound
    }
    fn get_q_jack_bound(&self) -> nalgebra::SVector<f64, N> {
        self.params.q_jerk_bound
    }
    fn get_base(&self) -> Pose {
        self.state.base_pose
    }
    fn get_end_effector_pose_na(&self) -> Pose {
        unimplemented!()
    }
    fn get_joint_capsules_with_joint(&self, joint: &nalgebra::SVector<f64, N>) -> Vec<Capsule> {
        let nlink = N;
        let mut joint_capsules = Vec::new();
        let dh = &self.params.denavit_hartenberg;
        let mut isometry = self.state.base_pose;

        for i in 0..nlink {
            let isometry_increment = Isometry::from_parts(
                na::Translation3::new(
                    dh[(i, 2)],
                    -dh[(i, 1)] * dh[(i, 3)].sin(),
                    dh[(i, 1)] * dh[(i, 3)].cos(),
                ),
                na::UnitQuaternion::from_euler_angles(joint[i], 0.0, dh[(i, 3)]),
            );

            // Update the cumulative transformation matrix
            isometry *= isometry_increment;

            // Calculate the positions of the capsule's end points in the global frame
            let capsule_start = isometry * self.params.capsules[i].ball_center1;
            let capsule_end = isometry * self.params.capsules[i].ball_center2;

            // Create a new Capsule object and add it to the vector
            joint_capsules.push(Capsule {
                ball_center1: capsule_start,
                ball_center2: capsule_end,
                radius: self.params.capsules[i].radius,
            });
        }
        joint_capsules
    }
    fn get_distance_with_joint(
        &self,
        joint: &nalgebra::SVector<f64, N>,
        obj: &CollisionObject,
    ) -> f64 {
        let joint_capsules = self.get_joint_capsules_with_joint(joint);
        let distance = joint_capsules
            .iter()
            .map(|&x| get_distance(&CollisionObject::Capsule(x), obj))
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();
        distance
    }
    fn get_distance_grad_with_joint(
        &self,
        joint: &nalgebra::SVector<f64, N>,
        bj: &CollisionObject,
    ) -> nalgebra::SVector<f64, N> {
        let mut distance_grad = na::SVector::from_element(0.0);
        let epsilon = 1e-3;
        for i in 0..N {
            let mut joint_plus = *joint;
            joint_plus[i] += epsilon;
            let mut joint_minus = *joint;
            joint_minus[i] -= epsilon;
            let distance_plus = self.get_distance_with_joint(&joint_plus, bj);
            let distance_minus = self.get_distance_with_joint(&joint_minus, bj);
            distance_grad[i] = (distance_plus - distance_minus) / (2.0 * epsilon);
        }
        distance_grad
    }

    fn set_q(&mut self, q: nalgebra::SVector<f64, N>) {
        self.state.q = q;
    }
    fn set_q_dot(&mut self, q_dot: nalgebra::SVector<f64, N>) {
        self.state.q_dot = q_dot;
    }

    fn update_dh(&mut self) {
        for i in 0..self.params.nlink {
            self.params.denavit_hartenberg[(i, 0)] = self.state.q[i];
        }
    }

    fn safety_check(&self, msg: &RobotMessageN<N>) -> bool {
        let (time, now_joint, now_vel, now_acc, now_tau) = match msg {
            RobotMessageN::ControlCommandN(command) => match command {
                ControlCommandN::Joint(joint) => (
                    0.0,
                    *joint,
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                ),
                ControlCommandN::JointWithPeriod(period, joint) => (
                    *period,
                    *joint,
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                ),
                ControlCommandN::JointVel(joint, vel) => (
                    0.0,
                    *joint,
                    *vel,
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                ),
                ControlCommandN::JointVelWithPeriod(period, joint, vel) => (
                    *period,
                    *joint,
                    *vel,
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                ),
                ControlCommandN::JointVelAcc(joint, vel, acc) => {
                    (0.0, *joint, *vel, *acc, nalgebra::SVector::zeros())
                }
                ControlCommandN::JointVelAccWithPeriod(period, joint, vel, acc) => {
                    (*period, *joint, *vel, *acc, nalgebra::SVector::zeros())
                }
                ControlCommandN::Tau(tau) => (
                    0.0,
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                    *tau,
                ),
                ControlCommandN::TauWithPeriod(period, tau) => (
                    *period,
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                    nalgebra::SVector::zeros(),
                    *tau,
                ),
            },
            _ => return false, // 处理非ControlCommandN的情况，直接返回false
        };
        let next_vel = now_vel + time * now_acc;
        let next_joint = now_joint + time * now_vel;
        if self.check_joint(&now_joint)
            && self.check_joint(&next_joint)
            && self.check_vel(&now_vel)
            && self.check_vel(&next_vel)
            && self.check_acc(&now_acc)
            && self.check_tau(&now_tau)
        {
            return true;
        }
        false
    }
}

impl<const N: usize, const N_ADD_ONE: usize> Robot for RobotNDof<N, N_ADD_ONE> {
    fn get_ndof(&self) -> usize {
        N
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_q_with_indptr(&self) -> (Vec<usize>, Vec<f64>) {
        (vec![0, N], self.state.q.as_slice().to_vec())
    }
    fn get_end_effector_pose(&self) -> Vec<Pose> {
        vec![self.state.base_pose]
    }
    fn get_end_effector_pose_with_q(&self, _: &na::DVector<f64>) -> Pose {
        unimplemented!()
    }
    fn get_joint_capsules(&self) -> Vec<message::collision_object::Capsule> {
        self.get_joint_capsules_with_joint(&self.state.q)
    }

    fn get_distance_to_collision(&self, obj: &CollisionObject) -> f64 {
        self.get_distance_with_joint(&self.state.q, obj)
    }

    fn get_distance_with_slice(&self, q: &[f64], obj: &CollisionObject) -> f64 {
        let q = na::SVector::from_vec(q.to_vec());
        self.get_distance_with_joint(&q, obj)
    }

    fn get_distance_grad_with_slice(&self, q: &[f64], obj: &CollisionObject) -> Vec<f64> {
        let q = na::SVector::from_vec(q.to_vec());
        self.get_distance_grad_with_joint(&q, obj)
            .as_slice()
            .to_vec()
    }
    fn get_robot_indices(&self, robot_names: Vec<String>) -> Vec<usize> {
        if self.name == robot_names[0] {
            vec![0]
        } else {
            vec![]
        }
    }

    fn set_name(&mut self, name: String) {
        self.name = name
    }
    fn set_path(&mut self, path: String) {
        self.path = path
    }

    fn reset_state(&mut self) {
        self.state.q = self.params.q_default;
        self.state.q_dot = na::SVector::from_element(0.0);
        self.state.q_ddot = na::SVector::from_element(0.0);
        self.state.q_jerk = na::SVector::from_element(0.0);
    }
}
