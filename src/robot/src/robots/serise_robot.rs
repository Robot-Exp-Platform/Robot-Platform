use nalgebra as na;

use crate::{DRobot, Robot, SRobot};
use generate_tools::{get_fn, set_fn};
use message::{Capsule, CollisionObject};
use message::{NodeMessage, Pose};

pub struct SeriseRobot<V>
where
    V: Send + Sync,
{
    pub name: String,

    pub state: SeriseRobotState<V>,
    pub params: SeriseRobotParams<V>,
}

pub type SSeriseRobot<const N: usize> = SeriseRobot<na::SVector<f64, N>>;

pub type DSeriseRobot = SeriseRobot<na::DVector<f64>>;

// #[derive(Default)]
pub struct SeriseRobotState<V> {
    pub q: V,
    pub q_dot: V,
    pub q_ddot: V,
    pub q_jerk: V,
    pub base: Pose,
    pub control_message: NodeMessage<V>,
}

// #[derive(Default)]
pub struct SeriseRobotParams<V> {
    pub nlink: usize,
    pub q_default: V,
    pub q_min_bound: V,
    pub q_max_bound: V,
    pub q_dot_bound: V,
    pub q_ddot_bound: V,
    pub q_jerk_bound: V,
    pub tau_bound: V,
    pub tau_dot_bound: V,
    pub dh: na::DMatrix<f64>,
    pub capsules: Vec<Capsule>,
}

impl<V> SeriseRobot<V>
where
    V: Send + Sync,
{
    // pub fn from_params(name: String, params: SeriseRobotParams<V>) -> SeriseRobot<V> {
    //     SeriseRobot {
    //         name,
    //         state: SeriseRobotState::<V>::default(),
    //         params,
    //     }
    // }
    pub fn from_file() -> SeriseRobot<V> {
        unimplemented!()
    }
}

impl<V> Robot<V> for SeriseRobot<V>
where
    V: Clone + Send + Sync,
{
    get_fn!((name: String));
    get_fn!(
        (q: V, state),
        (q_dot: V, state),
        (q_ddot: V, state),
        (q_jerk: V, state),
        (q_default: V, params),
        (q_min_bound: V, params),
        (q_max_bound: V, params),
        (q_dot_bound: V, params),
        (q_ddot_bound: V, params),
        (q_jerk_bound: V, params),
        (tau_bound: V, params),
        (tau_dot_bound: V, params),
        (base: Pose, state),
        (control_message: NodeMessage<V>, state)
    );

    set_fn!((set_name, name: String));
    set_fn!(
        (set_q, q: V, state),
        (set_q_dot, q_dot: V, state),
        (set_q_ddot, q_ddot: V, state),
        (set_q_jerk, q_jerk: V, state),
        (set_control_message, control_message: NodeMessage<V>, state)
    );

    fn dof(&self) -> usize {
        self.params.nlink
    }
}

impl DRobot for DSeriseRobot {
    fn end_pose(&self) -> Pose {
        self.cul_end_pose(&self.state.q)
    }
    fn capsules(&self) -> Vec<Capsule> {
        self.cul_capsules(&self.state.q)
    }
    fn dis_to_collision(&self, obj: &CollisionObject) -> f64 {
        self.cul_dis_to_collision(&self.state.q, obj)[0]
    }

    /// 重置机器人状态，包括将位置设置为默认值以及将运动归零
    fn reset(&mut self) {
        self.state.q = self.params.q_default.clone();
        self.state.q_dot = na::DVector::zeros(self.params.nlink);
        self.state.q_ddot = na::DVector::zeros(self.params.nlink);
        self.state.q_jerk = na::DVector::zeros(self.params.nlink);
    }

    /// 给定机器人的广义变量，计算末端执行器位姿
    fn cul_end_pose(&self, q: &na::DVector<f64>) -> Pose {
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

    /// 给定机器人的广义变量，计算机器人对应的所有胶囊体，这里需要留意的是，此时的胶囊体不包括末端执行器以及所夹取的物体。
    fn cul_capsules(&self, q: &na::DVector<f64>) -> Vec<Capsule> {
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

            // Create a new Capsule object and add it to the vector
            capsules.push(Capsule {
                pose: isometry,
                ..self.params.capsules[i]
            });
        }
        capsules
    }

    /// 给定机器人的广义变量，计算机器人到碰撞体的最小距禂
    fn cul_dis_to_collision(
        &self,
        q: &nalgebra::DVector<f64>,
        obj: &message::CollisionObject,
    ) -> na::DVector<f64> {
        let capsules = self.cul_capsules(q);
        let dis = capsules
            .iter()
            .map(|&c| CollisionObject::get_distance(&CollisionObject::Capsule(c), obj))
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();
        na::DVector::from_element(1, dis)
    }

    /// 给任意函数的梯度
    fn cul_func(
        &self,
        q: &na::DVector<f64>,
        func: &dyn Fn(&na::DVector<f64>) -> nalgebra::DVector<f64>,
    ) -> (na::DVector<f64>, na::DMatrix<f64>) {
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
}

impl<const N: usize> SRobot<N> for SSeriseRobot<N> {
    fn end_pose(&self) -> Pose {
        self.cul_end_pose(&self.state.q)
    }
    fn capsules(&self) -> Vec<Capsule> {
        self.cul_capsules(&self.state.q)
    }
    fn dis_to_collision(&self, obj: &CollisionObject) -> f64 {
        self.cul_dis_to_collision(&self.state.q, obj)
    }

    /// 重置机器人状态，包括将位置设置为默认值以及将运动归零
    fn reset(&mut self) {
        self.state.q = self.params.q_default;
        self.state.q_dot = na::SVector::zeros();
        self.state.q_ddot = na::SVector::zeros();
        self.state.q_jerk = na::SVector::zeros();
    }

    /// 给定机器人的广义变量，计算末端执行器位姿
    fn cul_end_pose(&self, q: &na::SVector<f64, N>) -> Pose {
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

    /// 给定机器人的广义变量，计算机器人对应的所有胶囊体，这里需要留意的是，此时的胶囊体不包括末端执行器以及所夹取的物体。
    fn cul_capsules(&self, q: &nalgebra::SVector<f64, N>) -> Vec<Capsule> {
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

            // Create a new Capsule object and add it to the vector
            capsules.push(Capsule {
                pose: isometry,
                ..capsules[i]
            });
        }
        capsules
    }

    /// 给定机器人的广义变量，计算机器人到碰撞体的最小距禂
    fn cul_dis_to_collision(
        &self,
        q: &nalgebra::SVector<f64, N>,
        obj: &message::CollisionObject,
    ) -> f64 {
        let capsules = self.cul_capsules(q);
        capsules
            .iter()
            .map(|&c| CollisionObject::get_distance(&CollisionObject::Capsule(c), obj))
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap()
    }

    /// 给定机器人的广义变量，计算机器人到碰撞体的最小距概的梯度
    fn cul_dis_grad_to_collision(
        &self,
        q: &nalgebra::SVector<f64, N>,
        obj: &message::CollisionObject,
    ) -> nalgebra::SVector<f64, N> {
        let mut dis_grad = na::SVector::zeros();
        let epsilon = 1e-3;
        for i in 0..self.params.nlink {
            let mut q_plus = *q;
            q_plus[i] += epsilon;
            let mut q_minus = *q;
            q_minus[i] -= epsilon;
            let dis_plus = self.cul_dis_to_collision(&q_plus, obj);
            let dis_minus = self.cul_dis_to_collision(&q_minus, obj);
            dis_grad[i] = (dis_plus - dis_minus) / (2.0 * epsilon);
        }
        dis_grad
    }
}
