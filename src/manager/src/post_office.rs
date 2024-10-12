use crossbeam::queue::SegQueue;
use message::DControlCommand;

#[derive(Default)]
pub struct PostOffice {
    /// 为所有仿真器保留的控制指令通道
    pub control_cmd_channer: SegQueue<DControlCommand>,
}
