use message::NodeMessage;
use nalgebra as na;
use serde::Deserialize;
use std::sync::{Arc, Mutex};

use crate::{Node, NodeBehavior};

pub type ZmqComm = Node<ZmqCommState, ZmqCommParams, (), na::DVector<f64>>;

#[derive(Default)]
pub struct ZmqCommState {
    responder: Option<Arc<Mutex<zmq::Socket>>>,
}

#[derive(Deserialize)]
pub struct ZmqCommParams {
    pub period: f64,
}

impl NodeBehavior for ZmqComm {
    fn init(&mut self) {
        // 建立通讯
        // 使用zmq实现程序通信，通信协议暂定为TCP
        // 以下为responder端
        let context = Arc::new(zmq::Context::new());
        let responder = context.socket(zmq::REP).unwrap();

        match responder.bind("tcp://*:5555") {
            Ok(_) => println!("Socket successfully bound to tcp://*:5555"),
            Err(e) => eprintln!("Failed to bind socket: {}", e),
        }
        self.state.responder = Some(Arc::new(Mutex::new(responder)));
    }

    fn update(&mut self) {
        // 接收消息
        let responder = self.state.responder.as_ref().unwrap().lock().unwrap();
        let message = responder.recv_string(0);

        if let Ok(Ok(message)) = message {
            responder.send("Received Command", 0).unwrap();
            let message: NodeMessage<na::DVector<f64>> = serde_json::from_str(&message).unwrap();
            self.output_queue.push(message);
        } else {
            responder.send("Failed to receive message", 0).unwrap();
        }
    }
}
