use crate::control::rail_system::components::{Block, Sensor, Signal};
use crate::control::rail_system::rail_graph::LocoGraph;
use crate::control::train::Train;
use locodrive::args::AddressArg;
use std::collections::HashMap;
use std::sync::Mutex;

pub struct Railroad {
    road: LocoGraph,
    trains: HashMap<AddressArg, Mutex<Train>>,
    sensors: HashMap<AddressArg, Mutex<Sensor>>,
    signals: HashMap<AddressArg, Mutex<Signal>>,
}

impl Railroad {
    pub fn get_sensor_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Sensor>> {
        self.sensors.get(adr)
    }

    pub fn get_signal_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Signal>> {
        self.signals.get(adr)
    }

    pub fn get_train(&self, adr: &AddressArg) -> Option<&Mutex<Train>> {
        self.trains.get(adr)
    }
}
