use crate::control::rail_system::components::{Address, SLevel, Speed, SwDir};

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Message {
    RailOn,
    RailOff,
    TrainSpeed(Address, Speed),
    Switch(Address, SwDir),
    SwitchAck(Address, SwDir),
    UpdateSensor(Address, SLevel),
}
