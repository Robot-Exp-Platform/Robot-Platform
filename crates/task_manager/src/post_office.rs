use std::{sync::Arc, thread, time::Duration};

use crossbeam::{
    channel::{Receiver, Sender},
    queue::SegQueue,
};

use crate::ROSThread;

#[derive(Default)]
pub struct PostOffice {
    period: f64,
    receivers: Vec<Receiver<(String, String)>>,
    senders: Vec<Sender<String>>,
    message_queue: Arc<SegQueue<String>>,

    connect_thread: Option<thread::JoinHandle<()>>,
}

impl PostOffice {
    pub fn new() -> Self {
        PostOffice {
            period: 0.0, // 邮局的周期初始化需要调整
            receivers: vec![],
            senders: vec![],
            message_queue: Arc::new(SegQueue::new()),
            connect_thread: None,
        }
    }

    pub fn add_receiver(&mut self, receiver: Receiver<(String, String)>) {
        self.receivers.push(receiver);
    }
    pub fn add_sender(&mut self, sender: Sender<String>) {
        self.senders.push(sender);
    }
}

impl ROSThread for PostOffice {
    fn init(&mut self) {
        // CMD 启动相关仿真器
        // 新建zmq通讯线程
        let period = self.period;
        let message_queue = self.message_queue.clone();
        self.connect_thread = Some(thread::spawn(move || {
            // zmq 通讯
            // 新建zmq上下文 ...

            loop {
                // 等待一个时间周期
                thread::sleep(Duration::from_secs_f64(period));
                // 检查是否有处理好的消息队列,若消息队列为空,则跳过.
                if message_queue.is_empty() {
                    continue;
                }

                // 等待外部消息
                // 发送处理好的消息队列中的消息,如果没有则发送空消息

                // 处理收到的外部消息,并按序列发给对应的接收器
            }
        }))
    }
    fn start(&mut self) {}

    fn update(&mut self) {
        // 从每个接收器中接受消息并分类处理
        // 收集所有第一个字符串为 bullet 的消息,并将其发给 pybullet 仿真器.
        let mut bullet_msgs: Vec<String> = Vec::new();
        for receiver in &self.receivers {
            if let Ok((topic, msg)) = receiver.try_recv() {
                match topic.as_str() {
                    "bullet" => {
                        bullet_msgs.push(msg);
                    }
                    _ => unimplemented!(),
                }
            }
        }
        let bullet_msgs = serde_json::to_string(&bullet_msgs).unwrap();
        if !bullet_msgs.is_empty() {
            self.message_queue.push(bullet_msgs);
        }
    }
}
