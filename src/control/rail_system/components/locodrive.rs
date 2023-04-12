use crate::control::rail_system::components::{Address, Speed};
use locodrive::args::{AddressArg, SpeedArg};

impl Address<u16> {
    pub fn address_arg(&self) -> AddressArg {
        AddressArg::new(self.0)
    }

    pub fn from_arg(adr: &AddressArg) -> Self {
        Address(adr.address())
    }
}

impl Speed<u8> {
    pub fn get_speed(&self) -> SpeedArg {
        match *self {
            Speed::Stop => SpeedArg::Stop,
            Speed::EmergencyStop => SpeedArg::EmergencyStop,
            Speed::Drive(spd) => SpeedArg::Drive(spd),
        }
    }

    pub fn from_arg(spd: &SpeedArg) -> Self {
        match *spd {
            SpeedArg::Stop => Speed::Stop,
            SpeedArg::EmergencyStop => Speed::EmergencyStop,
            SpeedArg::Drive(spd) => Speed::Drive(spd),
        }
    }
}
