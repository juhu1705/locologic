use crate::control::rail_system::components::{Address, Speed};
use locodrive::args::{AddressArg, SpeedArg, SwitchDirection};

use super::SwDir;

impl Address<u16> {
    pub fn address_arg(&self) -> AddressArg {
        AddressArg::new(self.0)
    }

    pub fn from_arg(adr: &AddressArg) -> Self {
        Address(adr.address())
    }
}

impl From<SpeedArg> for Speed<u8> {
    fn from(spd: SpeedArg) -> Self {
        match spd {
            SpeedArg::Stop => Speed::Stop,
            SpeedArg::EmergencyStop => Speed::EmergencyStop,
            SpeedArg::Drive(spd) => Speed::Drive(spd),
        }
    }
}

impl From<Speed<u8>> for SpeedArg {
    fn from(spd: Speed<u8>) -> Self {
        match spd {
            Speed::Stop => SpeedArg::Stop,
            Speed::EmergencyStop => SpeedArg::EmergencyStop,
            Speed::Drive(spd) => SpeedArg::Drive(spd),
        }
    }
}

impl From<SwDir> for SwitchDirection {
    fn from(dir: SwDir) -> Self {
        match dir {
            SwDir::Straight => SwitchDirection::Straight,
            SwDir::Curved => SwitchDirection::Curved,
        }
    }
}
