use nalgebra as na;
use nalgebra::Isometry;

use crate::robot_trait::Robot;
use crate::robot_trait::SeriesRobot;
use message::collision_object::{get_distance, Capsule, CollisionObject};
use message::message_trait::Message;
use message::state::Pose;

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
}

impl<const N: usize, const N_ADD_ONE: usize> RobotNDofParams<N, N_ADD_ONE> {
    fn new() -> RobotNDofParams<N, N_ADD_ONE> {
        RobotNDofParams {
            nlink: N,
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
    fn get_q_na(&self) -> na::SVector<f64, N> {
        self.state.q
    }
    fn get_q_dot_na(&self) -> na::SVector<f64, N> {
        self.state.q_dot
    }
    fn get_q_ddot_na(&self) -> na::SVector<f64, N> {
        self.state.q_ddot
    }
    fn get_q_jack_na(&self) -> na::SVector<f64, N> {
        self.state.q_jerk
    }
    fn get_base(&self) -> Pose {
        self.state.base_pose
    }
    fn get_end_effector_pose_na(&self) -> Pose {
        unimplemented!()
    }
    fn get_joint_capsules_with_joint(&self, joint: nalgebra::SVector<f64, N>) -> Vec<Capsule> {
        let nlink = N;
        let mut joint_capsules = Vec::new();
        let dh = &self.params.denavit_hartenberg;
        let mut isometry = self.state.base_pose;

        for i in 0..nlink + 1 {
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
            let capsule_end = isometry * self.params.capsules[i].ball_center1;

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
        joint: nalgebra::SVector<f64, N>,
        obj: &CollisionObject,
    ) -> f64 {
        let joint_capsules = self.get_joint_capsules_with_joint(joint);
        joint_capsules
            .iter()
            .map(|&x| get_distance(&CollisionObject::Capsule(x), obj))
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap()
    }
    fn get_distance_diff_with_joint(
        &self,
        joint: nalgebra::SVector<f64, N>,
        bj: &CollisionObject,
    ) -> nalgebra::SVector<f64, N> {
        let mut distance_diff = na::SVector::from_element(0.0);
        let epsilon = 1e-6;
        for i in 0..N {
            let mut joint_plus = joint;
            joint_plus[i] += epsilon;
            let mut joint_minus = joint;
            joint_minus[i] -= epsilon;
            let distance_plus = self.get_distance_with_joint(joint_plus, bj);
            let distance_minus = self.get_distance_with_joint(joint_minus, bj);
            distance_diff[i] = (distance_plus - distance_minus) / (2.0 * epsilon);
        }
        distance_diff
    }

    fn update_dh(&mut self) {
        for i in 0..self.params.nlink {
            self.params.denavit_hartenberg[(i, 0)] = self.state.q[i];
        }
    }
}

impl<const N: usize, const N_ADD_ONE: usize> Robot for RobotNDof<N, N_ADD_ONE> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn get_end_effector_pose(&self) -> Vec<Pose> {
        vec![self.state.base_pose]
    }
    fn get_joint_capsules(&self) -> Vec<message::collision_object::Capsule> {
        self.get_joint_capsules_with_joint(self.state.q)
    }

    fn get_distance_to_collision(&self, obj: &CollisionObject) -> f64 {
        self.get_distance_with_joint(self.state.q, obj)
    }

    fn set_name(&mut self, name: String) {
        self.name = name
    }
    fn set_path(&mut self, path: String) {
        self.path = path
    }
    fn set_q(&mut self, q: Vec<f64>) {
        for (i, &q_i) in q.iter().enumerate() {
            self.params.denavit_hartenberg[(i, 0)] = q_i;
        }
        self.state.q = na::SVector::from_vec(q)
    }
    fn set_q_dot(&mut self, q_dot: Vec<f64>) {
        self.state.q_dot = na::SVector::from_vec(q_dot)
    }

    fn reset_state(&mut self) {
        // TODO 位置重置
    }

    fn safety_check(&self, _: &Message) -> bool {
        true
    }
}
