use crate::control::rail_system::components::{Address, Node, Speed};
use crate::control::rail_system::railroad::Railroad;
use petgraph::graph::NodeIndex;
use std::sync::Arc;
use std::time::Duration;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct Clock {
    _time: Duration,
}

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
/// Represents one train, please create by calling [Railroad::create_train]
pub struct Train {
    /// The TRAIN'S address
    address: Address,
    /// The TRAIN'S current speed
    actual_speed: Speed,
    /// The speed the train should read
    speed: Speed,
    /// The train's position
    position: NodeIndex,
    /// Controls the driving process
    route: Option<Vec<(NodeIndex, bool)>>,
    /// Controls the driving table
    timetable: Vec<Station>,
}

impl Train {
    pub(crate) fn new(address: Address, position: NodeIndex) -> Self {
        Train {
            address,
            actual_speed: Speed::Stop,
            speed: Speed::Stop,
            position,
            route: None,
            timetable: Vec::new(),
        }
    }

    /// Resets the position of this train and reservates the corresponding signal block
    pub async fn reset_position(&self, position: NodeIndex, rail: &Railroad) -> bool {
        if let Some(signal_node) = rail.get_signal_of_block(position).await {
            let signal = rail.get_signal_mutex_by_index(signal_node).await.unwrap();
            signal.lock().await.request_block(self.address).await;
        }

        false
    }

    /// TODO: Implement
    pub fn check_next_station(&self) {}

    pub async fn trigger_drive_to(
        &mut self,
        destination: NodeIndex,
        railroad: Arc<Railroad>,
    ) -> bool {
        let route = Railroad::shortest_path(railroad, self.position, destination).await;

        let route = if let Some(route) = route {
            route.1.into_iter().map(|index| (index, false)).collect()
        } else {
            return false;
        };

        self.route = Some(route);

        true
    }

    pub fn stands(&self) -> bool {
        self.speed == Speed::Stop || self.speed == Speed::EmergencyStop
    }

    pub fn address(&self) -> Address {
        self.address
    }

    pub(crate) async fn request_route(
        &self,
        signal: Address,
        railroad: &Railroad,
    ) -> Option<Vec<&NodeIndex>> {
        let route = self.route.as_ref()?;

        let road = railroad.road().await;
        let index = route
            .iter()
            .position(|&(index, _)| match road.node_weight(index) {
                Some(Node::Signal(adr, ..)) => *adr == signal,
                _ => false,
            })?;

        let end = if let Some(i) = route[(index + 1)..]
            .iter()
            .position(|&(index, _)| matches!(road.node_weight(index), Some(Node::Signal(..))))
        {
            i + index + 2
        } else {
            route.len()
        };

        let vec: Vec<&NodeIndex> = route[index..end].iter().map(|(x, _)| x).clone().collect();
        Some(vec)
    }

    pub async fn update(&mut self, _tick: Duration, _railroad: &Railroad) {}
}

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
/// One station for a train to drive to
pub struct Station {
    arrive: Box<WaitingNode>,
    depart: Box<WaitingNode>,
    destination: NodeIndex,
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
    TrainOnSensor(Train, Address),
    /// Waits until another train holds on a specified sensor
    TrainHoldInStation(Train, Address),
}

impl Fulfiller for WaitingReasons {
    fn fulfills(&self) -> bool {
        match self {
            WaitingReasons::Time(duration) => duration.is_zero(),
            WaitingReasons::TrainOnSensor(_train, _adr) => true,
            WaitingReasons::TrainHoldInStation(_train, _adr) => true,
        }
    }
}
