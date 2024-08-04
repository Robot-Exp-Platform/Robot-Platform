use massage::massage_trait::Massage;

pub trait Recoder {
    fn record_msg(&self, msg: Massage);
}
