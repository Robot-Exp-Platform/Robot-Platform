use std::sync::Arc;

use crossbeam::queue::SegQueue;
use message::DControlCommand;

#[derive(Default)]
pub struct PostOffice {
    /// 为所有仿真器保留的控制指令通道
    pub control_cmd_channer: Vec<(String, Arc<SegQueue<DControlCommand>>)>,
}

impl PostOffice {
    pub fn add_control_cmd_channel(
        &mut self,
        name: String,
        channel: Arc<SegQueue<DControlCommand>>,
    ) {
        self.control_cmd_channer.push((name, channel));
    }
}
