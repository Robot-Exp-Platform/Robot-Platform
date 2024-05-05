use crate::plants::plant_trait::Plant;
use nalgebra as na;

pub struct LSParams<const N: usize, const I: usize, const O: usize> {
    a: na::SMatrix<f64, N, N>,
    b: na::SMatrix<f64, N, I>,
    c: na::SMatrix<f64, O, N>,
    d: na::SMatrix<f64, O, I>,
    time_step: f64,
}

pub struct LinearSystem<const N: usize, const I: usize, const O: usize> {
    // Linear System
    // q_dot = A q + B u
    // y = C q + D u
    q: na::SVector<f64, N>,
    q_dot: na::SVector<f64, N>,
    params: LSParams<N, I, O>,
}

impl<const N: usize, const I: usize, const O: usize> LinearSystem<N, I, O> {
    pub fn new() -> Self {
        LinearSystem {
            q: na::SVector::from_element(0.0),
            q_dot: na::SVector::from_element(0.0),
            params: LSParams {
                a: na::SMatrix::identity(),
                b: na::SMatrix::identity(),
                c: na::SMatrix::identity(),
                d: na::SMatrix::zeros(),
                time_step: 0.001,
            },
        }
    }
}

impl<const N: usize, const I: usize, const O: usize> Plant<N, I, O> for LinearSystem<N, I, O> {
    fn get_state(&self) -> (na::SVector<f64, N>, na::SVector<f64, N>) {
        (self.q.clone(), self.q_dot.clone())
    }

    fn init_state(&mut self, q: na::SVector<f64, N>, q_dot: na::SVector<f64, N>) {
        (self.q, self.q_dot) = (q, q_dot);
    }

    fn update(&mut self, u: na::SVector<f64, I>) -> na::SVector<f64, O> {
        self.q_dot = self.params.a * self.q + self.params.b * u;
        self.q += self.params.time_step * self.q_dot;
        self.params.c * self.q + self.params.d * u
    }
}
