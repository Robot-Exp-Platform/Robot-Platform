use nalgebra as na;
use message::collision_object::{Capsule, CollisionObject, get_distance};

fn main(){
    let capsule1 = Capsule {
        ball_center1: na::Point3::new(0.0, 0.0, 0.0),
        ball_center2: na::Point3::new(1.0, 0.0, 0.0),
        radius: 0.5,
    };
    
    let capsule2 = Capsule {
        ball_center1: na::Point3::new(0.5, 0.5, 1.0),
        ball_center2: na::Point3::new(0.5, 0.5, 2.0),
        radius: 0.5,
    };
    
    // 期望距离: sqrt(0.5) - 1.0 = sqrt(0.5) - 1.0 (~0.207)
    println!(
        "Distance between capsules (non-parallel): {}",
        get_distance(&CollisionObject::Capsule(capsule1), &CollisionObject::Capsule(capsule2))
    );
}