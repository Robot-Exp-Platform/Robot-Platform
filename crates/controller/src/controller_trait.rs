#[derive(Clone, Copy)]
pub enum ControllerState {
    Unknow,
    Uninit,
    Running,
}

pub trait Controller {
    // 控制器特征，控制器包含方法
    // func

    fn get_contoller_state(&self) -> ControllerState {
        ControllerState::Unknow
    }

    fn get_params(&self) -> Vec<f64>;
}
