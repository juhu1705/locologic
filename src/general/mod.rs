use crate::control::rail_system::components::Speed;
use num_traits::{CheckedAdd, CheckedSub};
use std::hash::Hash;

pub type DefaultAddressType = u16;
pub type DefaultSpeedType = u8;

pub trait SpeedType:
    Copy + Clone + Eq + Hash + Ord + Send + Sync + CheckedAdd + CheckedSub + 'static
{
    fn sub_to_speed(&self, other: &Self) -> Speed<Self> {
        self.checked_sub(other)
            .map_or(Speed::Stop, |x| Speed::Drive(x))
    }

    fn default_acceleration() -> Self;
}

pub trait AddressType: Copy + Clone + Eq + Hash + Send + Sync + Ord + 'static {}

impl SpeedType for u8 {
    fn default_acceleration() -> Self {
        5
    }
}

impl AddressType for u16 {}
