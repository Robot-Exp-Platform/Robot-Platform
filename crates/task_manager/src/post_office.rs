use crossbeam::channel::Receiver;

use crate::ROSThread;

#[derive(Default)]
pub struct PostOffice {
    receivers: Vec<Receiver<(String, String)>>,
}

impl PostOffice {
    pub fn new() -> Self {
        PostOffice {
            receivers: Vec::new(),
        }
    }

    pub fn add_receiver(&mut self, receiver: Receiver<(String, String)>) {
        self.receivers.push(receiver);
    }
}

impl ROSThread for PostOffice {
    fn init(&mut self) {
        // CMD 启动相关仿真器
    }
    fn start(&mut self) {}

    fn update(&mut self) {
        // 从每个接收器中接受消息并分类处理
        for receiver in &self.receivers {
            if let Ok((topic, _msg)) = receiver.try_recv() {
                match topic.as_str() {
                    "bullet" => {
                        // 处理相关仿真器内容
                    }
                    _ => unimplemented!(),
                }
            }
        }
    }
}
