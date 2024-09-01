use nalgebra::SVector;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub enum ControlCommand {
    Joint(Vec<f64>),
    JointWithPeriod(f64, Vec<f64>),
    JointVel(Vec<f64>, Vec<f64>),
    JointVelWithPeriod(f64, Vec<f64>, Vec<f64>),
    JointVelAcc(Vec<f64>, Vec<f64>, Vec<f64>),
    JointVelAccWithPeriod(f64, Vec<f64>, Vec<f64>, Vec<f64>),
    Tau(Vec<f64>),
    TauWithPeriod(f64, Vec<f64>),
}

#[derive(Debug, Deserialize, Serialize)]
pub enum ControlCommandN<const N: usize> {
    Joint(SVector<f64, N>),
    JointWithPeriod(f64, SVector<f64, N>),
    JointVel(SVector<f64, N>, SVector<f64, N>),
    JointVelWithPeriod(f64, SVector<f64, N>, SVector<f64, N>),
    JointVelAcc(SVector<f64, N>, SVector<f64, N>, SVector<f64, N>),
    JointVelAccWithPeriod(f64, SVector<f64, N>, SVector<f64, N>, SVector<f64, N>),
    Tau(SVector<f64, N>),
    TauWithPeriod(f64, SVector<f64, N>),
}

#[derive(Debug, Deserialize)]
pub struct JointWithPeriod {
    pub period: f64,
    pub joint: Vec<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn controlcommandn_to_json() {
        let joint = ControlCommandN::<3>::Joint(SVector::from_vec(vec![1.0, 2.0, 3.0]));
        let joint_with_period =
            ControlCommandN::<3>::JointWithPeriod(0.1, SVector::from_vec(vec![1.0, 2.0, 3.0]));
        let joint_vel = ControlCommandN::<3>::JointVel(
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
        );
        let joint_vel_with_period = ControlCommandN::<3>::JointVelWithPeriod(
            0.1,
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
        );
        let joint_vel_acc = ControlCommandN::<3>::JointVelAcc(
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
        );
        let joint_vel_acc_with_period = ControlCommandN::<3>::JointVelAccWithPeriod(
            0.1,
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
            SVector::from_vec(vec![1.0, 2.0, 3.0]),
        );
        let tau = ControlCommandN::<3>::Tau(SVector::from_vec(vec![1.0, 2.0, 3.0]));
        let tau_with_period =
            ControlCommandN::<3>::TauWithPeriod(0.1, SVector::from_vec(vec![1.0, 2.0, 3.0]));

        let joint_json = serde_json::to_string(&joint).unwrap();
        let joint_with_period_json = serde_json::to_string(&joint_with_period).unwrap();
        let joint_vel_json = serde_json::to_string(&joint_vel).unwrap();
        let joint_vel_with_period_json = serde_json::to_string(&joint_vel_with_period).unwrap();
        let joint_vel_acc_json = serde_json::to_string(&joint_vel_acc).unwrap();
        let joint_vel_acc_with_period_json =
            serde_json::to_string(&joint_vel_acc_with_period).unwrap();
        let tau_json = serde_json::to_string(&tau).unwrap();
        let tau_with_period_json = serde_json::to_string(&tau_with_period).unwrap();

        assert_eq!(joint_json, r#"{"Joint":[1.0,2.0,3.0]}"#);
        assert_eq!(
            joint_with_period_json,
            r#"{"JointWithPeriod":[0.1,[1.0,2.0,3.0]]}"#
        );
        assert_eq!(
            joint_vel_json,
            r#"{"JointVel":[[1.0,2.0,3.0],[1.0,2.0,3.0]]}"#
        );
        assert_eq!(
            joint_vel_with_period_json,
            r#"{"JointVelWithPeriod":[0.1,[1.0,2.0,3.0],[1.0,2.0,3.0]]}"#
        );
        assert_eq!(
            joint_vel_acc_json,
            r#"{"JointVelAcc":[[1.0,2.0,3.0],[1.0,2.0,3.0],[1.0,2.0,3.0]]}"#
        );
        assert_eq!(
            joint_vel_acc_with_period_json,
            r#"{"JointVelAccWithPeriod":[0.1,[1.0,2.0,3.0],[1.0,2.0,3.0],[1.0,2.0,3.0]]}"#
        );
        assert_eq!(tau_json, r#"{"Tau":[1.0,2.0,3.0]}"#);
        assert_eq!(
            tau_with_period_json,
            r#"{"TauWithPeriod":[0.1,[1.0,2.0,3.0]]}"#
        );
    }
}
