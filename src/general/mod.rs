use crate::control::rail_system::components::Speed;
use num_traits::{CheckedSub, One};
use std::hash::Hash;
use std::ops::Add;

pub type DefaultAddressType = u16;
pub type DefaultSpeedType = u8;

pub trait SpeedType: Copy + Clone + Eq + Hash + Add + CheckedSub + Ord + Send + Sync + One + 'static {
    fn sub_to_speed(&self, other: &Self) -> Speed<Self> {
        self.checked_sub(other)
            .map_or(Speed::Stop, |x| Speed::Drive(x))
    }
}

pub trait AddressType: Copy + Clone + Eq + Hash + Send + Sync + Ord + 'static {}

impl SpeedType for u8 {}

impl AddressType for u16 {}
