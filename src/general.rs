use std::time::Duration;

pub trait UpdateAble {
    fn update(tick: Duration);
}
