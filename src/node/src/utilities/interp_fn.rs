use std::{
    f64,
    ops::{Add, Div, Mul, Sub},
};

/// 线性插值函数，用于在一系列点中实现插值，插值数为 ninterp
/// 所有实现了 Add, Sub, Mul<f64>, Div<f64> 的类型都应该可以插值
pub fn lerp<T>(start: &T, jointlist: &Vec<T>, ninterp: usize) -> Vec<T>
where
    T: Add<Output = T> + Sub<Output = T> + Mul<f64, Output = T> + Div<f64, Output = T> + Clone,
{
    let mut track = Vec::new();
    let mut start = start.clone();
    for end in jointlist {
        for i in 0..ninterp {
            track.push(
                start.clone() + (end.clone() - start.clone()) * (i as f64) / (ninterp as f64),
            );
        }
        start = end.clone();
    }
    track
}
