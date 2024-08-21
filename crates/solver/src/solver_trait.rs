use message::track::Track;

pub trait Solver {
    fn solve(&self) -> Vec<Track>;
}
