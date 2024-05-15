use crate::robot_trait::Robot;
use crate::robots::panda::Panda;

pub struct Pandas<const N: usize> {
    name: String,
    pub pandas: Vec<Panda>,
}

impl<const M: usize> Pandas<M> {
    pub fn new() -> Pandas<M> {
        Pandas {
            name: "Pandas".to_string(),
            pandas: vec![Panda::new(); M],
        }
    }
}

impl<const M: usize> Robot for Pandas<M> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
}
