use crate::control::rail_system::components::{Block, Sensor, Signal};
use crate::control::rail_system::rail_graph::LocoGraph;
use crate::control::train::Train;
use locodrive::args::AddressArg;
use std::collections::HashMap;
use tokio::sync::Mutex;

pub struct Railroad<'t> {
    road: LocoGraph<'t>,
    trains: HashMap<AddressArg, Mutex<Train>>,
    blocks: Vec<Block<'t>>,
    sensors: HashMap<AddressArg, Mutex<Sensor<'t>>>,
    signals: HashMap<AddressArg, Mutex<Signal<'t>>>,
}

impl<'t> Railroad<'t> {
    pub fn get_sensor_mutex<'r>(&'r self, adr: &'r AddressArg) -> Option<&'t Mutex<Sensor<'r>>> {
        self.sensors.get(adr)
    }

    pub fn get_signal_mutex<'r>(&'r self, adr: &AddressArg) -> Option<&'t Mutex<Signal<'r>>> {
        self.signals.get(adr)
    }
}
