use crate::control::rail_system::components::{Block, Sensor, Signal};
use crate::control::rail_system::rail_graph::LocoGraph;
use crate::control::train::Train;
use locodrive::args::AddressArg;
use std::collections::HashMap;
use std::sync::Mutex;

pub struct Railroad<'t> {
    road: LocoGraph<'t>,
    trains: HashMap<AddressArg, Mutex<Train>>,
    blocks: Vec<Block<'t>>,
    sensors: HashMap<AddressArg, Mutex<Sensor<'t>>>,
    signals: HashMap<AddressArg, Mutex<Signal<'t>>>,
}

impl<'t> Railroad<'t> {
    pub fn get_sensor_mutex(&'t self, adr: &'t AddressArg) -> Option<&'t Mutex<Sensor<'t>>> {
        self.sensors.get(adr)
    }

    pub fn get_signal_mutex(&'t self, adr: &AddressArg) -> Option<&'t Mutex<Signal<'t>>> {
        self.signals.get(adr)
    }

    pub fn get_train(&self, adr: &AddressArg) -> Option<&Mutex<Train>> {
        self.trains.get(adr)
    }
}
