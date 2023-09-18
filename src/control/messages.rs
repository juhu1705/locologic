use crate::control::rail_system::components::{Address, SLevel, Speed, SwDir};
use crate::general::{AddressType, SpeedType};

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Message<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
> {
    RailOn,
    RailOff,
    TrainSpeed(Address<TrainAddr>, Speed<Spd>),
    Switch(Address<SwitchAddr>, SwDir),
    SwitchAck(Address<SwitchAddr>, SwDir),
    UpdateSensor(Address<SensorAddr>, SLevel),
    UpdateSignal(Address<SignalAddr>, SwDir),
    TrainGranted(Address<SignalAddr>, Address<TrainAddr>),
    TrainOnSensor(Address<SensorAddr>, Address<TrainAddr>),
}
