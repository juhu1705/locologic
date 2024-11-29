use crate::control::messages::Message;
use crate::control::rail_system::components::{Address, Node, Position, Speed};
use crate::control::rail_system::railroad::Railroad;
use crate::general::{AddressType, SpeedType};
use petgraph::graph::NodeIndex;
use std::collections::VecDeque;
use std::hash::Hash;
use std::sync::Arc;
use std::time::Duration;
use tokio::select;
use tokio::sync::Notify;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct Clock {
    _time: Duration,
}

#[derive(Debug)]
/// Represents one train, please create by calling [Railroad::create_train]
pub struct Train<Spd: SpeedType, TrainAddr: AddressType> {
    /// The train's address
    address: Address<TrainAddr>,
    /// The speed the train should read
    speed: Speed<Spd>,
    end_speed_adjusting: Arc<Notify>,
    speed_updater: Option<tokio::task::JoinHandle<Speed<Spd>>>,
    /// The train's position
    position: NodeIndex,
    /// Controls the driving process
    route: Option<VecDeque<(NodeIndex, bool)>>,
    /// Controls the driving table
    timetable: Vec<Station<Spd, TrainAddr>>,
}

impl<Spd: SpeedType, Ix: AddressType> PartialEq for Train<Spd, Ix> {
    fn eq(&self, other: &Self) -> bool {
        self.address == other.address
    }
}

impl<Spd: SpeedType, Ix: AddressType> Eq for Train<Spd, Ix> {}

impl<Spd: SpeedType, Ix: AddressType> Hash for Train<Spd, Ix> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.address.hash(state);
    }
}

impl<Spd: SpeedType, TrainAddr: AddressType> Train<Spd, TrainAddr> {
    pub(crate) fn new(address: Address<TrainAddr>, position: NodeIndex) -> Train<Spd, TrainAddr> {
        Train {
            address,
            speed: Speed::<Spd>::Stop,
            end_speed_adjusting: Arc::new(Notify::new()),
            speed_updater: None,
            position,
            route: None,
            timetable: Vec::new(),
        }
    }

    /// Sets the speed of this train to the given speed.
    /// The train will accelerate or decelerate to the given speed, by controlled messages.
    /// An emergency stop will be executed immediately without any deceleration delay.
    pub async fn set_speed<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &mut self,
        speed: Speed<Spd>,
        railroad: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) {
        let mut actual_speed = self.speed;
        self.speed = speed;

        self.end_speed_adjusting.notify_waiters();

        if let Some(join) = self.speed_updater.take() {
            actual_speed = join.await.unwrap();
        }

        if speed == Speed::EmergencyStop {
            railroad
                .send(Message::TrainSpeed(self.address, speed))
                .await;
            return;
        }

        let self_address = self.address;
        let interrupter = self.end_speed_adjusting.clone();

        self.speed_updater = Some(tokio::spawn(async move {
            Train::speed_accelerator(self_address, actual_speed, speed, interrupter, railroad).await
        }));
    }

    async fn speed_accelerator<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        address: Address<TrainAddr>,
        mut actual_speed: Speed<Spd>,
        speed: Speed<Spd>,
        interrupter: Arc<Notify>,
        railroad: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) -> Speed<Spd> {
        let duration = Duration::from_millis(10);
        loop {
            select! {
                _ = interrupter.notified() => {
                    return actual_speed;
                },
                _ = tokio::time::sleep(duration) => {

                }
            }

            if actual_speed < speed {
                actual_speed = actual_speed + Speed::<Spd>::Drive(Spd::default_acceleration());
                if actual_speed > speed {
                    break;
                }
            } else {
                actual_speed = actual_speed - Speed::<Spd>::Drive(Spd::default_acceleration());
                if actual_speed < speed {
                    break;
                }
            }

            railroad
                .send(Message::TrainSpeed(address, actual_speed))
                .await;
        }
        railroad.send(Message::TrainSpeed(address, speed)).await;
        speed
    }

    pub fn stands(&self) -> bool {
        self.speed == Speed::Stop || self.speed == Speed::EmergencyStop
    }

    pub fn timetable(&self) -> &Vec<Station<Spd, TrainAddr>> {
        &self.timetable
    }

    pub fn position(&self) -> NodeIndex {
        self.position
    }

    pub fn route(&self) -> Option<&VecDeque<(NodeIndex, bool)>> {
        self.route.as_ref()
    }

    pub fn drive_ok(&mut self, node_index: NodeIndex) {
        if self.route.is_none() {
            return;
        }

        if let Some((_i, b)) = self
            .route
            .as_mut()
            .unwrap()
            .iter_mut()
            .find(|(i, _b)| i == &node_index)
        {
            *b = true;
        }
    }

    pub async fn sensor_entered<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &mut self,
        sensor: NodeIndex,
        railroad: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) {
        if self.route.is_none() {
            return;
        }
        let route = self.route.as_mut().unwrap();
        let road = railroad.road().await;
        if let Some((next_sensor, _b)) = route.clone().iter().find(|(i, _b)| {
            matches!(
                road.node_weight(*i),
                Some(Node::Sensor(..) | Node::Station(..))
            )
        }) {
            if *next_sensor == sensor {
                while let Some((i, _b)) = route.pop_front() {
                    if i == sensor {
                        break;
                    }
                }
                self.position = *next_sensor;
                self.request_next_block(railroad).await;
            }
        }
    }

    /// Checks if the next signal block should be requested and requests it.
    pub async fn request_next_block<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &mut self,
        rail: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) {
        if self.route.is_none() {
            return;
        }

        let route = self.route.as_mut().unwrap();

        if route.is_empty() {
            self.route = None;
            // TODO: Check to call station or something like that
            return;
        }

        let road = rail.road().await;

        let mut blocks_count = 0;
        let mut next_signal = None;

        for (i, b) in route {
            if let Some(Node::Signal(adr, _)) = road.node_weight(*i) {
                if *b {
                    blocks_count += 1;
                } else {
                    next_signal = Some(adr);
                    break;
                }
            }
        }

        if blocks_count < 1
        /* TODO: Replace with config value */
        {
            return;
        }

        if let Some(next_signal) = next_signal {
            if let Some(signal) = rail.get_signal_mutex(next_signal) {
                signal
                    .lock()
                    .await
                    .request_block(self.address(), rail.clone())
                    .await;
            }
        }
    }

    /// Resets the position of this train and reservates the corresponding signal block
    pub async fn reset_position<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &mut self,
        position: NodeIndex,
        rail: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) -> bool {
        let mut subscription = rail.subscribe();

        let mut signal_adr = None;

        if let Some(signal_node) = rail.get_signal_of_block(position).await {
            let signal = rail.get_signal_mutex_by_index(signal_node).await.unwrap();
            let mut signal = signal.lock().await;
            signal.request_block(self.address, rail.clone()).await;
            signal_adr = Some(signal.address());
        }

        if let Some(signal_adr) = signal_adr {
            while let Ok(message) = subscription.recv().await {
                if let Message::TrainGranted(adr1, adr2) = message {
                    if adr1 == signal_adr && adr2 == self.address {
                        self.request_next_block(rail).await;
                        return true;
                    }
                }
            }
        }

        false
    }

    pub async fn trigger_drive_to_sensor<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &mut self,
        destination: &Address<SensorAddr>,
        position: &Position,
        railroad: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) -> bool {
        if let Some(node_index) = railroad.get_sensor_index(destination, position).await {
            self.trigger_drive_to(*node_index, railroad).await
        } else {
            false
        }
    }

    /// Triggers the train to drive to the given destination
    /// Returns true if the train could start driving
    ///
    /// # Parameters
    ///
    /// * `destination` - The destination to drive to
    /// * `railroad` - The railroad to drive on
    ///
    /// # Usage
    ///
    /// ```
    /// # use std::collections::VecDeque;
    /// # use std::ops::Deref;
    /// # use std::sync::Arc;
    /// # use petgraph::adj::NodeIndex;
    /// # use locologic::control::rail_system::components::Address;
    /// # use locologic::control::rail_system::railroad::Railroad;
    /// # use locologic::control::rail_system::railroad_test;
    /// # tokio_test::block_on(async {
    /// # let (mut railroad, switches, bi_dir_switches, sensors, bi_dir_sensors, signals) = railroad_test::create_test_railroad().await;
    /// # let train_address = Address::new(1);
    /// # railroad.create_train(train_address, sensors[2].0).await;
    /// # let mut railroad = Arc::new(railroad);
    /// let mut train = railroad.get_train(&train_address).unwrap().lock().await;
    /// train.trigger_drive_to(sensors[3].0, railroad.clone()).await;
    ///
    /// let expected_road = Railroad::shortest_path(railroad.clone(), sensors[2].0, sensors[3].0)
    ///     .await.unwrap().1.into_iter().map(|index| (index, false)).collect();
    /// assert_eq!(train.route(), Some(&expected_road));
    ///
    /// train.trigger_drive_to(switches[4].0, railroad.clone()).await;
    /// assert!(railroad.road().await.node_weight(train.route().unwrap().iter().last().unwrap().0).unwrap().is_driveable());
    /// # });
    /// ```
    pub async fn trigger_drive_to<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &mut self,
        destination: NodeIndex,
        railroad: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) -> bool {
        let graph = railroad.road().await;
        let route = Railroad::shortest_path(railroad, self.position, destination).await;

        // Short route to the last sensor as destination

        let route = if let Some(route) = route {
            let pos = route.1.len()
                - route
                    .1
                    .iter()
                    .rev()
                    .position(|node| graph.node_weight(*node).is_some_and(|n| n.is_driveable()))
                    .unwrap_or(0);
            route
                .1
                .into_iter()
                .take(pos)
                .map(|index| (index, false))
                .collect()
        } else {
            return false;
        };

        self.route = Some(route);

        true
    }

    pub fn address(&self) -> Address<TrainAddr> {
        self.address
    }

    pub(crate) async fn request_route<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &self,
        signal: Address<SignalAddr>,
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    ) -> Option<Vec<&NodeIndex>> {
        let route = self.route.as_ref()?;

        let road = railroad.road().await;
        let index = route
            .iter()
            .position(|&(index, _)| match road.node_weight(index) {
                Some(Node::Signal(adr, ..)) => *adr == signal,
                _ => false,
            })?;

        let end = if let Some(i) = route
            .range((index + 1)..)
            .position(|&(index, _)| matches!(road.node_weight(index), Some(Node::Signal(..))))
        {
            i + index + 2
        } else {
            route.len()
        };

        let vec: Vec<&NodeIndex> = route.range(index..end).map(|(x, _)| x).clone().collect();
        Some(vec)
    }

    pub async fn update<
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &mut self,
        _tick: Duration,
        _railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    ) {
    }
}

#[derive(Debug, Eq, PartialEq, Hash)]
/// One station for a train to drive to
pub struct Station<Spd: SpeedType, TrainAddr: AddressType> {
    arrive: Box<WaitingNode<Spd, TrainAddr>>,
    depart: Box<WaitingNode<Spd, TrainAddr>>,
    destination: NodeIndex,
}

impl<Spd: SpeedType, TrainAddr: AddressType> Station<Spd, TrainAddr> {
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

#[derive(Debug, Eq, PartialEq, Hash)]
struct WaitingNode<Spd: SpeedType, TrainAddr: AddressType> {
    connector: WaitingReasonOperator,
    waiters: Vec<WaitingReasons<Spd, TrainAddr>>,
    childs: Vec<WaitingNode<Spd, TrainAddr>>,
}

impl<Spd: SpeedType, TrainAddr: AddressType> Fulfiller for WaitingNode<Spd, TrainAddr> {
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

impl<Spd: SpeedType, TrainAddr: AddressType> WaitingNode<Spd, TrainAddr> {
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
#[derive(Debug, Eq, PartialEq, Hash)]
pub enum WaitingReasons<Spd: SpeedType, TrainAddr: AddressType> {
    /// Waits a specific time
    Time(Duration),
    /// Waits until another train is on a specified sensor
    TrainOnSensor(Train<Spd, TrainAddr>, Address),
    /// Waits until another train holds on a specified sensor
    TrainHoldInStation(Train<Spd, TrainAddr>, Address),
}

impl<Spd: SpeedType, TrainAddr: AddressType> Fulfiller for WaitingReasons<Spd, TrainAddr> {
    fn fulfills(&self) -> bool {
        match self {
            WaitingReasons::Time(duration) => duration.is_zero(),
            WaitingReasons::TrainOnSensor(_train, _adr) => true,
            WaitingReasons::TrainHoldInStation(_train, _adr) => true,
        }
    }
}
