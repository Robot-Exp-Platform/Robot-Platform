use nalgebra as na;

use crate::{DRobot, Robot};
use generate_tools::todo_fn;
use message::{iso_to_vec, vec_to_iso, Capsule, NodeMessage, Pose};

#[derive(Default)]
pub struct FakePoseRobot {
    pub pose: Pose,
}

impl Robot<na::DVector<f64>> for FakePoseRobot {
    todo_fn!((name: String));
    todo_fn!(
        (q_dot: na::DVector<f64>, state),
        (q_ddot: na::DVector<f64>, state),
        (q_jerk: na::DVector<f64>, state),
        (q_default: na::DVector<f64>, params),
        (q_min_bound: na::DVector<f64>, params),
        (q_max_bound: na::DVector<f64>, params),
        (q_dot_bound: na::DVector<f64>, params),
        (q_ddot_bound: na::DVector<f64>, params),
        (q_jerk_bound: na::DVector<f64>, params),
        (tau_bound: na::DVector<f64>, params),
        (tau_dot_bound: na::DVector<f64>, params),
        (base: Pose, state),
        (control_message: NodeMessage<na::DVector<f64>>, state)
    );
    todo_fn!((set_name, name: String));
    todo_fn!(
        (set_q_dot, q_dot: na::DVector<f64>),
        (set_q_ddot, q_ddot: na::DVector<f64>),
        (set_q_jerk, q_jerk: na::DVector<f64>),
        (set_control_message, control_message: NodeMessage<na::DVector<f64>>)
    );

    fn dof(&self) -> usize {
        6
    }
    fn q(&self) -> na::DVector<f64> {
        iso_to_vec(self.pose)
    }
    fn set_q(&mut self, q: na::DVector<f64>) {
        self.pose = vec_to_iso(q);
    }
}

impl DRobot for FakePoseRobot {
    todo_fn!((capsules : Vec<Capsule>));

    fn end_pose(&self) -> Pose {
        self.pose
    }

    fn dis_to_collision(&self, _: &message::CollisionObject) -> f64 {
        unimplemented!()
    }

    fn cul_capsules(&self, _: &na::DVector<f64>) -> Vec<Capsule> {
        unimplemented!()
    }

    fn cul_dis_to_collision(
        &self,
        _: &na::DVector<f64>,
        _: &message::CollisionObject,
    ) -> na::DVector<f64> {
        unimplemented!()
    }

    fn cul_end_pose(&self, q: &nalgebra::DVector<f64>) -> Pose {
        vec_to_iso(q.clone())
    }

    fn cul_func(
        &self,
        q: &nalgebra::DVector<f64>,
        func: &dyn Fn(&nalgebra::DVector<f64>) -> nalgebra::DVector<f64>,
    ) -> (nalgebra::DVector<f64>, nalgebra::DMatrix<f64>) {
        let value = func(q);
        let mut grad = na::DMatrix::zeros(value.len(), q.len());
        let epsilon = 1e-3;
        for i in 0..q.len() {
            let mut q_plus = q.clone();
            q_plus[i] += epsilon;
            let mut q_minus = q.clone();
            q_minus[i] -= epsilon;
            let value_plus = func(&q_plus);
            let value_minus = func(&q_minus);
            grad.set_column(i, &((value_plus - value_minus) / (2.0 * epsilon)));
        }
        (value, grad)
    }

    fn reset(&mut self) {
        self.pose = Pose::default();
    }
}
