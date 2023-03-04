use crate::control::rail_system::components::{Node, Signal};
use crate::control::rail_system::railroad::Railroad;
use locodrive::args::{AddressArg, SlotArg, SpeedArg};
use petgraph::graph::NodeIndex;
use std::time::Duration;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct Clock {
    _time: Duration,
}

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
/// Represents one train
pub struct Train {
    /// The TRAINS address
    address: AddressArg,
    /// The TRAINS slot
    slot: SlotArg,
    /// The TRAINS current speed
    actual_speed: SpeedArg,
    /// The speed the train should read
    speed: SpeedArg,
    /// The trains position
    position: NodeIndex,
    /// Controls the driving process
    route: Option<Vec<NodeIndex>>,
    /// Controls the driving table
    timetable: Vec<Station>,
}

impl Train {
    pub fn new(address: AddressArg, slot: SlotArg, position: NodeIndex) -> Self {
        Train {
            address,
            slot,
            actual_speed: SpeedArg::Stop,
            speed: SpeedArg::Stop,
            position,
            route: None,
            timetable: Vec::new(),
        }
    }

    pub fn stands(&self) -> bool {
        self.speed == SpeedArg::Stop || self.speed == SpeedArg::EmergencyStop
    }

    pub fn address(&self) -> AddressArg {
        self.address
    }

    pub async fn request_route(
        &self,
        signal: AddressArg,
        railroad: &Railroad,
    ) -> Option<Vec<&NodeIndex>> {
        let route = self.route.as_ref()?;

        let road = railroad.road().await;
        let index = route.iter().position(|&r| match road.node_weight(r) {
            Some(Node::Signal(adr, ..)) => *adr == signal,
            _ => false,
        })?;

        let end = if let Some(i) = route[(index + 1)..]
            .iter()
            .position(|&r| matches!(road.node_weight(r), Some(Node::Signal(..))))
        {
            i + index + 2
        } else {
            route.len()
        };

        let vec: Vec<&NodeIndex> = route[index..end].iter().clone().collect();
        Some(vec)
    }

    pub async fn notify(&mut self, _signal: &Signal, road: Vec<AddressArg>, railroad: &Railroad) {
        for adr in road {
            if let Some(mutex) = railroad.get_sensor_mutex(&adr) {
                let mut sensor = mutex.lock().await;
                sensor.block(self.address());
            }
        }
    }

    pub async fn update(&mut self, _tick: Duration, _railroad: &Railroad) {}
}

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
/// One station for a train to drive to
pub struct Station {
    arrive: Box<WaitingNode>,
    depart: Box<WaitingNode>,
}

impl Station {
    pub fn could_arrive(&self) -> bool {
        self.arrive.fulfills()
    }

    pub fn could_depart(&self) -> bool {
        self.depart.fulfills()
    }
}

trait Fulfiller {
    fn fulfills(&self) -> bool;
}

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
struct WaitingNode {
    connector: WaitingReasonOperator,
    waiters: Vec<WaitingReasons>,
    childs: Vec<WaitingNode>,
}

impl Fulfiller for WaitingNode {
    fn fulfills(&self) -> bool {
        let result = self.check(self.waiters.iter());

        match self.connector {
            WaitingReasonOperator::AND => result && self.check(self.childs.iter()),
            WaitingReasonOperator::OR => result || self.check(self.childs.iter()),
            WaitingReasonOperator::XOR => result ^ self.check(self.childs.iter()),
            WaitingReasonOperator::XNOR => result == self.check(self.childs.iter()),
        }
    }
}

impl WaitingNode {
    fn check<'t, I, F>(&self, mut fulfilled: I) -> bool
    where
        I: Iterator<Item = &'t F>,
        F: Fulfiller + 't,
    {
        match self.connector {
            WaitingReasonOperator::AND => fulfilled.all(|next| next.fulfills()),
            WaitingReasonOperator::OR => fulfilled.any(|next| next.fulfills()),
            WaitingReasonOperator::XOR => fulfilled.fold(false, |res, next| res ^ next.fulfills()),
            WaitingReasonOperator::XNOR => fulfilled
                .map(|f| f.fulfills())
                .reduce(|acc, item| acc == item)
                .unwrap_or(true),
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum WaitingReasonOperator {
    AND,
    OR,
    XOR,
    XNOR,
}

/// For what the train should wait
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub enum WaitingReasons {
    /// Waits a specific time
    Time(Duration),
    /// Waits until another train is on a specified sensor
    TrainOnSensor(Train, u16),
    /// Waits until another train holds on a specified sensor
    TrainHoldInStation(Train, u16),
}

impl Fulfiller for WaitingReasons {
    fn fulfills(&self) -> bool {
        match self {
            WaitingReasons::Time(_) => true,
            WaitingReasons::TrainOnSensor(_, _) => true,
            WaitingReasons::TrainHoldInStation(_, _) => true,
        }
    }
}
