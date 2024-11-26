use message::{NodeMessage, Pose};
use nalgebra as na;
use std::sync::{Arc, RwLock};

use crate::{DRobot, Robot};

macro_rules! merge_fn {
    ($($fn_name:ident),*) => {
        $(
            fn $fn_name(&self) -> na::DVector<f64> {
                self.merge(self.robots.iter().map(|robot| robot.read().unwrap().$fn_name()).collect())
            }
        )*
    };

}

macro_rules! split_fn {
    ($($fn_name:ident),*) => {
        $(
            fn $fn_name(&mut self, combined_vector: na::DVector<f64>) {
                self.split(combined_vector).into_iter().zip(&self.robots).for_each(|(vector, robot)| {
                    robot.write().unwrap().$fn_name(vector)
                });
            }
        )*
    };
}

#[derive(Default)]
pub struct RobotBranch<R> {
    robots: Vec<Arc<RwLock<R>>>,
    indices: Vec<usize>,
}

impl<R: DRobot> RobotBranch<R> {
    pub fn indices(&self) -> &Vec<usize> {
        &self.indices
    }

    pub fn new() -> RobotBranch<R> {
        RobotBranch {
            robots: Vec::new(),
            indices: Vec::new(),
        }
    }

    pub fn push(&mut self, robot: Arc<RwLock<R>>) {
        if self.indices.is_empty() {
            self.indices.push(0);
        }
        self.indices
            .push(self.indices.last().unwrap() + robot.read().unwrap().dof());
        self.robots.push(robot);
    }

    /// 合并多机器人的向量
    pub fn merge(&self, vectors: Vec<na::DVector<f64>>) -> na::DVector<f64> {
        let total_dof = self.dof();
        let mut combined_vector = na::DVector::zeros(total_dof);
        let mut offset = 0;
        for vector in vectors {
            combined_vector
                .rows_mut(offset, vector.len())
                .copy_from(&vector);
            offset += vector.len();
        }
        combined_vector
    }

    /// 拆分向量为多机器人的向量
    pub fn split(&self, vector: na::DVector<f64>) -> Vec<na::DVector<f64>> {
        let mut vectors = Vec::new();
        for i in 0..self.indices.len() - 1 {
            let start = self.indices[i];
            let end = self.indices[i + 1] + 1;
            vectors.push(vector.rows(start, end - start).into_owned());
        }
        vectors
    }

    pub fn read_robot(&self, index: usize) -> Arc<RwLock<R>> {
        self.robots[index].clone()
    }

    pub fn end_poses(&self, index: Vec<usize>) -> Vec<Pose> {
        index
            .iter()
            .map(|i| self.robots[*i].read().unwrap().end_pose())
            .collect()
    }

    pub fn cul_relative_func(
        &self,
        index: (usize, usize),
        q: &na::DVector<f64>,
        func: &dyn Fn(Pose, Pose) -> na::DVector<f64>,
    ) -> (na::DVector<f64>, na::DMatrix<f64>) {
        let (id_1, id_2) = index;
        let q_1 = q
            .rows(
                self.indices[id_1],
                self.indices[id_1 + 1] - self.indices[id_1],
            )
            .into_owned();
        let q_2 = q
            .rows(
                self.indices[id_2],
                self.indices[id_2 + 1] - self.indices[id_2],
            )
            .into_owned();
        let func_1 = |q: &na::DVector<f64>| {
            func(
                self.robots[id_1].read().unwrap().cul_end_pose(q),
                self.robots[id_2].read().unwrap().cul_end_pose(&q_2),
            )
        };
        let func_2 = |q: &na::DVector<f64>| {
            func(
                self.robots[id_1].read().unwrap().cul_end_pose(&q_1),
                self.robots[id_2].read().unwrap().cul_end_pose(q),
            )
        };
        let (value_1, grad_1) = self.robots[id_1].read().unwrap().cul_func(&q_1, &func_1);
        let (value_2, grad_2) = self.robots[id_2].read().unwrap().cul_func(&q_2, &func_2);
        assert_eq!(value_1, value_2);
        let mut grad = na::DMatrix::zeros(grad_1.nrows(), self.dof());
        grad.view_mut((0, self.indices[id_1]), (grad_1.nrows(), grad_1.ncols()))
            .copy_from(&grad_1);
        grad.view_mut((0, self.indices[id_2]), (grad_2.nrows(), grad_2.ncols()))
            .copy_from(&grad_2);
        (value_1, grad)
    }

    pub fn cul_select_func(
        &self,
        indices: &[usize],
        q: &na::DVector<f64>,
        func: &dyn Fn(Vec<Pose>) -> na::DVector<f64>,
    ) -> (na::DVector<f64>, na::DMatrix<f64>) {
        // 提取每个机器人对应的关节变量 q_i
        let qs: Vec<na::DVector<f64>> = indices
            .iter()
            .map(|&id| {
                q.rows(self.indices[id], self.indices[id + 1] - self.indices[id])
                    .into_owned()
            })
            .collect();

        // 计算每个机器人的末端位姿
        let poses: Vec<Pose> = indices
            .iter()
            .zip(qs.iter())
            .map(|(&id, q_i)| self.robots[id].read().unwrap().cul_end_pose(q_i))
            .collect();

        // 计算函数值
        let value = func(poses.clone());

        // 存储每个机器人的梯度
        let mut grads = Vec::new();

        for (i, &id) in indices.iter().enumerate() {
            let q_i = &qs[i];

            // 定义只对 q_i 变量的函数 func_i
            let func_i = |q_var: &na::DVector<f64>| {
                let mut poses_var = poses.clone();
                poses_var[i] = self.robots[id].read().unwrap().cul_end_pose(q_var);
                func(poses_var)
            };

            // 计算梯度
            let (_, grad_i) = self.robots[id].read().unwrap().cul_func(q_i, &func_i);

            grads.push((id, grad_i));
        }

        // 初始化整体梯度矩阵
        let mut grad = na::DMatrix::zeros(value.nrows(), self.dof());

        // 将每个机器人的梯度拷贝到整体梯度矩阵中
        for (id, grad_i) in grads {
            grad.view_mut((0, self.indices[id]), (grad_i.nrows(), grad_i.ncols()))
                .copy_from(&grad_i);
        }

        (value, grad)
    }
}

impl<R: DRobot> Robot<na::DVector<f64>> for RobotBranch<R> {
    merge_fn!(
        q,
        q_dot,
        q_ddot,
        q_jerk,
        q_default,
        q_max_bound,
        q_min_bound,
        q_dot_bound,
        q_ddot_bound,
        q_jerk_bound,
        tau_bound,
        tau_dot_bound
    );

    split_fn!(set_q, set_q_dot, set_q_ddot, set_q_jerk);

    fn name(&self) -> String {
        "RobotBranch".to_string()
    }
    fn dof(&self) -> usize {
        self.indices.last().unwrap() + 1
    }
    fn base(&self) -> Pose {
        Pose::default()
    }
    fn control_message(&self) -> NodeMessage<na::DVector<f64>> {
        NodeMessage::default()
    }

    fn set_name(&mut self, _: String) {}
    fn set_control_message(&mut self, _: NodeMessage<na::DVector<f64>>) {}
}

impl<R: DRobot> DRobot for RobotBranch<R> {
    fn end_pose(&self) -> Pose {
        unimplemented!()
    }

    fn capsules(&self) -> Vec<message::Capsule> {
        self.robots
            .iter()
            .flat_map(|robot| robot.read().unwrap().capsules())
            .collect()
    }

    fn dis_to_collision(&self, _: &message::CollisionObject) -> f64 {
        unimplemented!()
    }

    fn cul_end_pose(&self, _: &na::DVector<f64>) -> Pose {
        unimplemented!()
    }

    fn cul_capsules(&self, _: &na::DVector<f64>) -> Vec<message::Capsule> {
        unimplemented!()
    }

    fn cul_dis_to_collision(
        &self,
        q: &nalgebra::DVector<f64>,
        obj: &message::CollisionObject,
    ) -> nalgebra::DVector<f64> {
        self.merge(
            self.robots
                .iter()
                .map(|robot| robot.read().unwrap().cul_dis_to_collision(q, obj))
                .collect(),
        )
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
        self.robots
            .iter()
            .for_each(|robot| robot.write().unwrap().reset());
    }
}
