pub struct Gripper {
    name: String,
    wide: f64,
    pub homing: bool,
}

impl Gripper {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            wide: 0.0,
            homing: false,
        }
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }
    pub fn width(&self) -> f64 {
        self.wide
    }

    pub fn grasp(&mut self, width: f64) {
        self.wide = width;
    }

    pub fn home(&mut self) {
        self.homing = true;
    }
}
