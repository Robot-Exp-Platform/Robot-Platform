use message::{get_distance, Pose};
use nalgebra as na;

use crate::{DRobot, Robot};
use generate_tools::{get_fn, set_fn};
use message::{Capsule, CollisionObject};

pub struct SSeriseRobot<const N: usize, const N_ADD_ONE: usize> {}

pub struct DSeriseRobot {
    pub name: String,

    pub state: DSeriseRobotState,
    pub params: DSeriseRobotParams,
}

pub struct DSeriseRobotState {
    pub q: na::DVector<f64>,
    pub q_dot: na::DVector<f64>,
    pub q_ddot: na::DVector<f64>,
    pub q_jerk: na::DVector<f64>,
    pub base: Pose,
}

pub struct DSeriseRobotParams {
    pub nlink: usize,
    pub q_default: na::DVector<f64>,
    pub q_min_bound: na::DVector<f64>,
    pub q_max_bound: na::DVector<f64>,
    pub q_dot_bound: na::DVector<f64>,
    pub q_ddot_bound: na::DVector<f64>,
    pub q_jerk_bound: na::DVector<f64>,
    pub tau_bound: na::DVector<f64>,
    pub tau_dot_bound: na::DVector<f64>,
    pub dh: na::DMatrix<f64>,
    pub capsules: Vec<Capsule>,
}

impl DSeriseRobot {
    pub fn new(nlink: usize, name: String) -> DSeriseRobot {
        DSeriseRobot::from_params(nlink, name, DSeriseRobotParams::new(nlink))
    }
    pub fn from_params(nlink: usize, name: String, params: DSeriseRobotParams) -> DSeriseRobot {
        DSeriseRobot {
            name,
            state: DSeriseRobotState::new(nlink),
            params,
        }
    }
    pub fn from_file() -> DSeriseRobot {
        unimplemented!()
    }
}

impl DSeriseRobotState {
    pub fn new(nlink: usize) -> DSeriseRobotState {
        DSeriseRobotState {
            q: na::DVector::zeros(nlink),
            q_dot: na::DVector::zeros(nlink),
            q_ddot: na::DVector::zeros(nlink),
            q_jerk: na::DVector::zeros(nlink),
            base: Pose::identity(),
        }
    }
}

impl DSeriseRobotParams {
    pub fn new(nlink: usize) -> DSeriseRobotParams {
        DSeriseRobotParams {
            nlink,
            q_default: na::DVector::zeros(nlink),
            q_min_bound: na::DVector::zeros(nlink),
            q_max_bound: na::DVector::zeros(nlink),
            q_dot_bound: na::DVector::zeros(nlink),
            q_ddot_bound: na::DVector::zeros(nlink),
            q_jerk_bound: na::DVector::zeros(nlink),
            tau_bound: na::DVector::zeros(nlink),
            tau_dot_bound: na::DVector::zeros(nlink),
            dh: na::DMatrix::zeros(nlink + 1, 4),
            capsules: vec![Capsule::default(); nlink],
        }
    }
}

impl DRobot for DSeriseRobot {
    get_fn!(
        (q, state),
        (q_dot, state),
        (q_ddot, state),
        (q_jerk, state),
        (q_default, params),
        (q_min_bound, params),
        (q_max_bound, params),
        (q_dot_bound, params),
        (q_ddot_bound, params),
        (q_jerk_bound, params),
        (tau_bound, params),
        (tau_dot_bound, params)
    );
    get_fn!((base: Pose, state));

    set_fn!(
        (set_q, q, state),
        (set_q_dot, q_dot, state),
        (set_q_ddot, q_ddot, state),
        (set_q_jerk, q_jerk, state)
    );

    fn end_effector(&self) -> Pose {
        self.cul_end_effector(&self.state.q)
    }

    fn cul_end_effector(&self, q: &nalgebra::DVector<f64>) -> Pose {
        let dh = &self.params.dh;
        let mut isometry = self.state.base;
        for i in 0..self.params.nlink {
            let isometry_increment = na::Isometry::from_parts(
                na::Translation3::new(
                    dh[(i, 2)],
                    -dh[(i, 1)] * dh[(i, 3)].sin(),
                    dh[(i, 1)] * dh[(i, 3)].cos(),
                ),
                na::UnitQuaternion::from_euler_angles(q[i], 0.0, dh[(i, 3)]),
            );

            // Update the cumulative transformation matrix
            isometry *= isometry_increment;
        }
        isometry
    }

    fn cul_capsules(&self, q: &nalgebra::DVector<f64>) -> Vec<Capsule> {
        let dh = &self.params.dh;
        let mut capsules = Vec::new();
        let mut isometry = self.state.base;

        for i in 0..self.params.nlink {
            let isometry_increment = na::Isometry::from_parts(
                na::Translation3::new(
                    dh[(i, 2)],
                    -dh[(i, 1)] * dh[(i, 3)].sin(),
                    dh[(i, 1)] * dh[(i, 3)].cos(),
                ),
                na::UnitQuaternion::from_euler_angles(q[i], 0.0, dh[(i, 3)]),
            );

            // Update the cumulative transformation matrix
            isometry *= isometry_increment;

            // Calculate the positions of the capsule's end points in the global frame
            let capsule_start = isometry * self.params.capsules[i].ball_center1;
            let capsule_end = isometry * self.params.capsules[i].ball_center2;

            // Create a new Capsule object and add it to the vector
            capsules.push(Capsule {
                ball_center1: capsule_start,
                ball_center2: capsule_end,
                radius: self.params.capsules[i].radius,
            });
        }
        capsules
    }

    fn cul_dis_to_collision(
        &self,
        q: &nalgebra::DVector<f64>,
        obj: &message::CollisionObject,
    ) -> f64 {
        let capsules = self.cul_capsules(q);
        capsules
            .iter()
            .map(|&c| get_distance(&CollisionObject::Capsule(c), obj))
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap()
    }

    fn cul_dis_grad_to_collision(
        &self,
        q: &nalgebra::DVector<f64>,
        obj: &message::CollisionObject,
    ) -> nalgebra::DVector<f64> {
        let mut dis_grad = na::DVector::zeros(self.params.nlink);
        let epsilon = 1e-3;
        for i in 0..self.params.nlink {
            let mut q_plus = q.clone();
            q_plus[i] += epsilon;
            let mut q_minus = q.clone();
            q_minus[i] -= epsilon;
            let dis_plus = self.cul_dis_to_collision(&q_plus, obj);
            let dis_minus = self.cul_dis_to_collision(&q_minus, obj);
            dis_grad[i] = (dis_plus - dis_minus) / (2.0 * epsilon);
        }
        dis_grad
    }
}

impl Robot for DSeriseRobot {
    get_fn!((name: String));
    set_fn!((set_name, name: String));

    fn dof(&self) -> usize {
        self.params.nlink
    }
    fn capsules(&self) -> Vec<Capsule> {
        self.cul_capsules(&self.state.q)
    }
    fn dis_to_collision(&self, obj: &CollisionObject) -> f64 {
        self.cul_dis_to_collision(&self.state.q, obj)
    }
    fn reset(&mut self) {
        self.state.q = self.params.q_default.clone();
        self.state.q_dot = na::DVector::zeros(self.params.nlink);
        self.state.q_ddot = na::DVector::zeros(self.params.nlink);
        self.state.q_jerk = na::DVector::zeros(self.params.nlink);
    }
}
