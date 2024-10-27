use crate::plant_trait::Plant;
use nalgebra as na;

pub struct FOLParams<const N: usize> {
    k: na::SMatrix<f64, N, N>,
    tau: na::SMatrix<f64, N, N>,
    time_step: f64,
}
pub struct FirstOrderLTI<const N: usize> {
    // First Order Linear Time Invariant System
    // $$\\tau \\dot{y}(t) + y(t) = K u(t)$$
    q: na::SVector<f64, N>,
    q_dot: na::SVector<f64, N>,
    params: FOLParams<N>,
}

impl<const N: usize> FirstOrderLTI<N> {
    pub fn new() -> Self {
        FirstOrderLTI {
            q: na::SVector::from_element(0.0),
            q_dot: na::SVector::from_element(0.0),
            params: FOLParams {
                k: na::SMatrix::identity(),
                tau: na::SMatrix::identity(),
                time_step: 0.001,
            },
        }
    }
}

impl<const N: usize> Default for FirstOrderLTI<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Plant<N, N, N> for FirstOrderLTI<N> {
    fn get_state(&self) -> (na::SVector<f64, N>, na::SVector<f64, N>) {
        (self.q, self.q_dot)
    }

    fn init_state(&mut self, q: na::SVector<f64, N>, q_dot: na::SVector<f64, N>) {
        self.q = q;
        self.q_dot = q_dot;
    }

    fn update(&mut self, u: na::SVector<f64, N>) -> na::SVector<f64, N> {
        // Update the state of the plant for 1 temp_step
        self.q_dot = self.params.tau.try_inverse().unwrap() * (self.params.k * u - self.q);
        self.q += self.q_dot * self.params.time_step;
        self.q
    }
}
