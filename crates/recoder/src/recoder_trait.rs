use message::message_trait::Message;

pub trait Recoder {
    fn record_msg(&self, msg: Message);
}
