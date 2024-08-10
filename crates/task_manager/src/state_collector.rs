use std::sync::{Arc, Condvar, Mutex};

pub type StateCollector = Arc<(Mutex<NodeState>, Condvar)>;

pub struct NodeState {
    node_size: usize,
    waiting: usize,
    starting: usize,
    updating: usize,
    finished: usize,
}

impl NodeState {
    pub fn new() -> Self {
        NodeState {
            node_size: 0,
            waiting: 0,
            starting: 0,
            updating: 0,
            finished: 0,
        }
    }

    pub fn get_node_size(&self) -> usize {
        self.node_size
    }
    pub fn get_starting(&self) -> usize {
        self.starting
    }
    pub fn get_updating(&self) -> usize {
        self.updating
    }
    pub fn get_finished(&self) -> usize {
        self.finished
    }

    pub fn add_node(&mut self) {
        self.node_size += 1;
        self.waiting += 1;
    }

    pub fn wait(&mut self) {
        self.waiting += 1;
        self.finished -= 1;
    }
    pub fn start(&mut self) {
        self.starting += 1;
        self.waiting -= 1;
    }
    pub fn update(&mut self) {
        self.updating += 1;
        self.starting -= 1;
    }
    pub fn finish(&mut self) {
        self.finished += 1;
        self.updating -= 1;
    }

    pub fn refresh(&mut self) {
        self.waiting = self.node_size;
        self.starting = 0;
        self.updating = 0;
        self.finished = 0;
    }
}

impl Default for NodeState {
    fn default() -> Self {
        Self::new()
    }
}
