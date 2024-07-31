#[cfg(feature = "ros")]
use rosrust as ros;
use std::sync::{Arc, RwLock};
use task_manager::ros_thread::ROSThread;

use crate::simlulator_trait::Simlulator;
use robot::robot_trait::Robot;

#[allow(dead_code)]
pub struct Bullet<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    rosnode: BulletNode,
    robot: Arc<RwLock<R>>,
}

#[allow(dead_code)]
struct BulletNode {
    #[cfg(feature = "ros")]
    sub_list: Vec<ros::Subscriber>,
}

impl<R: Robot + 'static, const N: usize> Bullet<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Bullet<R, N> {
        Bullet {
            name,
            path,
            rosnode: BulletNode {
                #[cfg(feature = "ros")]
                sub_list: Vec::new(),
            },
            robot,
        }
    }
}

impl<R: Robot + 'static, const N: usize> Simlulator for Bullet<R, N> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
}

impl<R: Robot + 'static, const N: usize> ROSThread for Bullet<R, N> {
    fn init(&self) {
        #[cfg(feature = "ros")]
        {
            // 在这里进行节点和话题的声明
            ros::init((self.path.clone() + self.name.clone().as_str()).as_str());

            // 新建接收者，并将他们放入list中去
        }
    }

    fn start(&mut self) {}

    fn update(&self) {}
}
