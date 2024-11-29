use crate::control::messages::Message;
use crate::control::rail_system::components::{
    Address, Cross, Node, Position, Rail, Sensor, Signal, SignalType, Speed, Switch, SwitchType,
};
use crate::control::train::Train;
use crate::general::{AddressType, DefaultAddressType, DefaultSpeedType, SpeedType};
use petgraph::algo::astar;
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::{Bfs, EdgeRef};
use petgraph::{Direction, Graph};
use std::collections::HashMap;
use std::hash::Hash;
use std::ops::Index;
use std::sync::Arc;
use tokio::sync::broadcast::{channel, Receiver};
use tokio::sync::{broadcast::Sender, Mutex};
use tokio::task::spawn_blocking;

type Road<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr> =
    Mutex<DiGraph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>>;
type Trains<Spd, TrainAddr> = HashMap<Address<TrainAddr>, Mutex<Train<Spd, TrainAddr>>>;
type Sensors<Spd, SensorAddr, TrainAddr> =
    HashMap<Address<SensorAddr>, (Mutex<Sensor<Spd, SensorAddr, TrainAddr>>, Vec<NodeIndex>)>;
type Signals<SignalAddr, TrainAddr, SensorAddr> =
    HashMap<Address<SignalAddr>, Mutex<Signal<SignalAddr, TrainAddr, SensorAddr>>>;
type Crossings<CrossingAddr> = HashMap<Address<CrossingAddr>, Mutex<Cross<CrossingAddr>>>;
type Switches<SwitchAddr> =
    HashMap<Address<SwitchAddr>, (Mutex<Switch<SwitchAddr>>, Vec<NodeIndex>)>;
type Channel<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr> =
    Sender<Message<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>>;

#[derive(Debug)]
pub struct Railroad<
    Spd: SpeedType = DefaultSpeedType,
    TrainAddr: AddressType = DefaultAddressType,
    SensorAddr: AddressType = DefaultAddressType,
    SwitchAddr: AddressType = DefaultAddressType,
    SignalAddr: AddressType = DefaultAddressType,
    CrossingAddr: AddressType = DefaultAddressType,
> {
    road: Road<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    trains: Trains<Spd, TrainAddr>,
    sensors: Sensors<Spd, SensorAddr, TrainAddr>,
    signals: Signals<SignalAddr, TrainAddr, SensorAddr>,
    crossings: Crossings<CrossingAddr>,
    switches: Switches<SwitchAddr>,
    channel: Channel<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>,
}

impl<
        Spd: SpeedType,
        TrainAddr: AddressType,
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    > Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>
{
    /// Returns a **clone** of the railroad
    pub async fn road(
        &self,
    ) -> DiGraph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>> {
        self.road.lock().await.clone()
    }

    /// Trying to create a train and place it on the given position. If failing it will return
    /// None.
    pub async fn create_train(
        &mut self,
        address: Address<TrainAddr>,
        position: NodeIndex,
    ) -> Option<&Mutex<Train<Spd, TrainAddr>>> {
        let t = Train::new(address, position);

        let mut sensor = match self.road.lock().await.node_weight_mut(position)? {
            Node::Sensor(adr, ..) | Node::Station(adr, ..) => self.sensors.get(adr)?.0.lock().await,
            _ => {
                return None;
            }
        };

        if !sensor.block(address) {
            return None;
        }

        let train = Mutex::new(t);

        self.trains.insert(address, train);

        self.trains.get(&address)
    }

    pub fn get_sensor_mutex(
        &self,
        adr: &Address<SensorAddr>,
    ) -> Option<&Mutex<Sensor<Spd, SensorAddr, TrainAddr>>> {
        Some(&self.sensors.get(adr)?.0)
    }

    pub fn get_signal_mutex(
        &self,
        adr: &Address<SignalAddr>,
    ) -> Option<&Mutex<Signal<SignalAddr, TrainAddr, SensorAddr>>> {
        self.signals.get(adr)
    }

    pub async fn get_signal_mutex_by_index(
        &self,
        index: NodeIndex,
    ) -> Option<&Mutex<Signal<SignalAddr, TrainAddr, SensorAddr>>> {
        let road = self.road().await;
        let node = road.node_weight(index)?;
        if let Node::Signal(adr, ..) = node {
            return self.get_signal_mutex(adr);
        }
        None
    }

    pub async fn get_sensor_index(
        &self,
        adr: &Address<SensorAddr>,
        pos: &Position,
    ) -> Option<&NodeIndex> {
        let road = self.road().await;
        self.sensors
            .get(adr)?
            .1
            .iter()
            .find(|ind| match *road.index(**ind) {
                Node::Sensor(adr_check, pos_check) | Node::Station(adr_check, pos_check) => {
                    adr_check == *adr && pos_check == *pos
                }
                _ => false,
            })
    }

    pub async fn get_switch_index(
        &self,
        adr: &Address<SwitchAddr>,
        pos: &Position,
    ) -> Option<&NodeIndex> {
        let road = self.road().await;
        self.switches
            .get(adr)?
            .1
            .iter()
            .find(|ind| match *road.index(**ind) {
                Node::Switch(adr_check, pos_check, ..) => adr_check == *adr && pos_check == *pos,
                _ => false,
            })
    }

    pub fn get_train(&self, adr: &Address<TrainAddr>) -> Option<&Mutex<Train<Spd, TrainAddr>>> {
        self.trains.get(adr)
    }

    pub fn get_switch_mutex(
        &self,
        adr: &Address<SwitchAddr>,
    ) -> Option<&Mutex<Switch<SwitchAddr>>> {
        Some(&self.switches.get(adr)?.0)
    }

    pub fn get_crossing_mutex(
        &self,
        adr: &Address<CrossingAddr>,
    ) -> Option<&Mutex<Cross<CrossingAddr>>> {
        self.crossings.get(adr)
    }

    pub async fn shortest_path(
        rail: Arc<Self>,
        start: NodeIndex,
        destination: NodeIndex,
    ) -> Option<(usize, Vec<NodeIndex>)> {
        let graph = { rail.road.lock().await.clone() };
        if let Ok(result) = spawn_blocking(move || {
            astar(
                &graph,
                start,
                |goal| goal == destination,
                |cost| {
                    let rail_cost: usize = cost.weight().iter().map(Rail::manhattan_distance).sum();
                    rail_cost + node_cost(&graph, cost.target(), rail.clone())
                },
                |node| estimate_costs(&graph, node, rail.clone(), destination),
            )
        })
        .await
        {
            result
        } else {
            None
        }
    }

    /// Returns one possible input signal of a block.
    /// This can be used, if a train is directly placed into one block,
    /// without entering it over a specific signal.
    pub async fn get_signal_of_block(&self, block_node: NodeIndex) -> Option<NodeIndex> {
        let mut road = self.road().await;
        road.reverse();
        let mut traversal = Bfs::new(&road, block_node);
        while let Some(node) = traversal.next(&road) {
            if matches!(road.node_weight(node), Some(Node::Signal(..))) {
                return Some(node);
            }
        }
        None
    }

    /// Subscribes to the railroads general message channel
    pub fn subscribe(
        &self,
    ) -> Receiver<Message<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>> {
        self.channel.subscribe()
    }

    /// Sends a message to the railroads general message channel
    /// ignoring the possibility for now active subscribers receiving that message.
    pub async fn send(&self, msg: Message<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>) {
        let _ = self.channel.send(msg);
    }
}

fn estimate_costs<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
    CrossingAddr: AddressType,
>(
    graph: &Graph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>,
    node: NodeIndex,
    rail: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    dest: NodeIndex,
) -> usize {
    let node_pos = graph.index(node).position(&rail);
    let dest_pos = graph.index(dest).position(&rail);

    node_pos.coord().manhattan_distance(&dest_pos.coord())
}

fn node_cost<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
    CrossingAddr: AddressType,
>(
    graph: &Graph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>,
    node: NodeIndex,
    rail: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
) -> usize {
    fn train_cost<
        Spd: SpeedType,
        TrainAddr: AddressType,
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        sensor_adr: &Address<SensorAddr>,
        rail: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>>,
    ) -> Option<usize> {
        let train = {
            let sensor_mut = rail.get_sensor_mutex(sensor_adr)?;
            (*(sensor_mut.blocking_lock()).train())?
        };
        let train_mut = rail.get_train(&train)?;
        if train_mut.blocking_lock().stands() {
            Some(100)
        } else {
            Some(27)
        }
    }

    match graph.index(node) {
        Node::Sensor(sensor_adr, ..) => {
            if let Some(cost) = train_cost(sensor_adr, rail) {
                cost
            } else {
                2
            }
        }
        Node::Station(..) => 500,
        _ => 2,
    }
}

type BuilderSensors<Spd, SensorAddr, TrainAddr> =
    HashMap<Address<SensorAddr>, (Sensor<Spd, SensorAddr, TrainAddr>, Vec<NodeIndex>)>;

pub struct Builder<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
    CrossingAddr: AddressType,
> {
    road: DiGraph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>,
    trains: HashMap<Address<TrainAddr>, Train<Spd, TrainAddr>>,
    sensors: BuilderSensors<Spd, SensorAddr, TrainAddr>,
    signals: HashMap<Address<SignalAddr>, Signal<SignalAddr, TrainAddr, SensorAddr>>,
    crossings: HashMap<Address<CrossingAddr>, Cross<CrossingAddr>>,
    switches: HashMap<Address<SwitchAddr>, (Switch<SwitchAddr>, Vec<NodeIndex>)>,
    channel: Sender<Message<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>>,
}

impl<
        Spd: SpeedType,
        TrainAddr: AddressType,
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    > Default for Builder<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<
        Spd: SpeedType,
        TrainAddr: AddressType,
        SensorAddr: AddressType,
        SwitchAddr: AddressType,
        SignalAddr: AddressType,
        CrossingAddr: AddressType,
    > Builder<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>
{
    pub fn new() -> Self {
        Builder {
            road: DiGraph::<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>::new(
            ),
            trains: HashMap::new(),
            sensors: HashMap::new(),
            signals: HashMap::new(),
            crossings: HashMap::new(),
            switches: HashMap::new(),
            channel: channel(25).0,
        }
    }

    pub async fn from_railroad(
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    ) -> Self {
        let road = railroad.road.lock().await.clone();

        async fn copy_map<R, T>(input: &HashMap<R, Mutex<T>>) -> HashMap<R, T>
        where
            R: Clone + Hash + Eq,
            T: Clone,
        {
            let mut map = HashMap::new();
            for i in input {
                map.insert(i.0.clone(), i.1.lock().await.clone());
            }
            map
        }

        async fn copy_index_map<R, T, L>(input: &HashMap<R, (Mutex<T>, L)>) -> HashMap<R, (T, L)>
        where
            R: Clone + Hash + Eq,
            T: Clone,
            L: Clone,
        {
            let mut map = HashMap::new();
            for (adr, val) in input {
                map.insert(adr.clone(), (val.0.lock().await.clone(), val.1.clone()));
            }
            map
        }

        async fn copy_trains<R, Spd: SpeedType, TrainAddr: AddressType>(
            input: &HashMap<R, Mutex<Train<Spd, TrainAddr>>>,
        ) -> HashMap<R, Train<Spd, TrainAddr>>
        where
            R: Clone + Hash + Eq,
        {
            let mut map = HashMap::new();
            for i in input {
                let old = i.1.lock().await;
                map.insert(i.0.clone(), Train::new(old.address(), old.position()));
            }
            map
        }

        let trains = copy_trains(&railroad.trains).await;
        let sensors = copy_index_map(&railroad.sensors).await;
        let signals = copy_map(&railroad.signals).await;
        let crossings = copy_map(&railroad.crossings).await;
        let switches = copy_index_map(&railroad.switches).await;
        let channel = railroad.channel.clone();

        Builder {
            road,
            trains,
            sensors,
            signals,
            crossings,
            switches,
            channel,
        }
    }

    pub fn add_sensors(
        &mut self,
        sensors: &[(Address<SensorAddr>, Speed<Spd>, Position)],
    ) -> Vec<(NodeIndex, Address<SensorAddr>)> {
        sensors
            .iter()
            .map(|(s_adr, max_speed, s_pos)| (self.add_sensor(*s_adr, *max_speed, *s_pos), *s_adr))
            .collect()
    }

    pub fn add_bidirectional_sensors(
        &mut self,
        sensors: &[(Address<SensorAddr>, Speed<Spd>, Position)],
    ) -> Vec<((NodeIndex, NodeIndex), Address<SensorAddr>)> {
        sensors
            .iter()
            .map(|(s_adr, max_speed, s_pos)| {
                (
                    self.add_bidirectional_sensor(*s_adr, *max_speed, *s_pos),
                    *s_adr,
                )
            })
            .collect()
    }

    pub fn add_sensor(
        &mut self,
        sensor: Address<SensorAddr>,
        max_speed: Speed<Spd>,
        position: Position,
    ) -> NodeIndex {
        let node = self.road.add_node(Node::Sensor(sensor, position));

        if let Some((_, vector)) = self.sensors.get_mut(&sensor) {
            vector.push(node);
        } else {
            self.sensors
                .insert(sensor, (Sensor::new(sensor, max_speed), vec![node]));
        }

        node
    }

    pub fn add_bidirectional_sensor(
        &mut self,
        sensor: Address<SensorAddr>,
        max_speed: Speed<Spd>,
        position: Position,
    ) -> (NodeIndex, NodeIndex) {
        let node = self.road.add_node(Node::Sensor(sensor, position));
        let node_reverse = self.road.add_node(Node::Sensor(sensor, position));

        if let Some((_, vector)) = self.sensors.get_mut(&sensor) {
            vector.push(node);
            vector.push(node_reverse)
        } else {
            self.sensors.insert(
                sensor,
                (Sensor::new(sensor, max_speed), vec![node, node_reverse]),
            );
        }

        (node, node_reverse)
    }

    pub fn add_stations(
        &mut self,
        stations: &[(Address<SensorAddr>, Speed<Spd>, Position)],
    ) -> Vec<(NodeIndex, Address<SensorAddr>)> {
        stations
            .iter()
            .map(|station| (self.add_station(station.0, station.1, station.2), station.0))
            .collect()
    }

    pub fn add_bidirectional_stations(
        &mut self,
        stations: &[(Address<SensorAddr>, Speed<Spd>, Position)],
    ) -> Vec<((NodeIndex, NodeIndex), Address<SensorAddr>)> {
        stations
            .iter()
            .map(|(s_adr, max_speed, s_pos)| {
                (
                    self.add_bidirectional_station(*s_adr, *max_speed, *s_pos),
                    *s_adr,
                )
            })
            .collect()
    }

    pub fn add_station(
        &mut self,
        station: Address<SensorAddr>,
        max_speed: Speed<Spd>,
        position: Position,
    ) -> NodeIndex {
        let index = self.road.add_node(Node::Station(station, position));

        if let Some((_, vector)) = self.sensors.get_mut(&station) {
            vector.push(index)
        } else {
            self.sensors
                .insert(station, (Sensor::new(station, max_speed), vec![index]));
        }

        index
    }

    pub fn add_bidirectional_station(
        &mut self,
        station: Address<SensorAddr>,
        max_speed: Speed<Spd>,
        position: Position,
    ) -> (NodeIndex, NodeIndex) {
        let index = self.road.add_node(Node::Station(station, position));
        let node_reverse = self.road.add_node(Node::Station(station, position));

        if let Some((_, vector)) = self.sensors.get_mut(&station) {
            vector.push(index);
            vector.push(node_reverse);
        } else {
            self.sensors.insert(
                station,
                (Sensor::new(station, max_speed), vec![index, node_reverse]),
            );
        }

        (index, node_reverse)
    }

    pub fn add_signals(
        &mut self,
        signals: &[(Address<SignalAddr>, SignalType, Position)],
    ) -> HashMap<Address<SignalAddr>, NodeIndex> {
        signals
            .iter()
            .filter_map(|signal| Some((signal.0, self.add_signal(signal.0, signal.1, signal.2)?)))
            .collect()
    }

    pub fn add_signal(
        &mut self,
        signal: Address<SignalAddr>,
        signal_type: SignalType,
        position: Position,
    ) -> Option<NodeIndex> {
        if self.signals.contains_key(&signal) {
            return None;
        }

        let node = self.road.add_node(Node::Signal(signal, position));

        self.signals
            .insert(signal, Signal::new(signal, signal_type, node));
        Some(node)
    }

    pub fn add_crossing(
        &mut self,
        cross: Address<CrossingAddr>,
        pos: Position,
    ) -> Option<(NodeIndex, NodeIndex)> {
        if self.crossings.contains_key(&cross) {
            return None;
        }

        let node1 = self.road.add_node(Node::Cross(cross));
        let node2 = self.road.add_node(Node::Cross(cross));

        self.crossings
            .insert(cross, Cross::new(cross, pos, (node1, node2)));

        Some((node1, node2))
    }

    pub fn add_switches(
        &mut self,
        switches: &[(Address<SwitchAddr>, Position, SwitchType)],
    ) -> Vec<(NodeIndex, Address<SwitchAddr>)> {
        switches
            .iter()
            .map(|switch| (self.add_switch(switch.0, switch.1, switch.2), switch.0))
            .collect()
    }

    pub fn add_bidirectional_switches(
        &mut self,
        switches: &[(Address<SwitchAddr>, Position, SwitchType)],
    ) -> Vec<((NodeIndex, NodeIndex), Address<SwitchAddr>)> {
        switches
            .iter()
            .map(|switch| {
                (
                    self.add_bidirectional_switch(switch.0, switch.1, switch.2),
                    switch.0,
                )
            })
            .collect()
    }

    pub fn add_switch(
        &mut self,
        switch: Address<SwitchAddr>,
        position: Position,
        s_type: SwitchType,
    ) -> NodeIndex {
        let index = self.road.add_node(Node::Switch(
            switch,
            position,
            s_type,
            None,
            Direction::Incoming,
        ));

        if let Some((_, vector)) = self.switches.get_mut(&switch) {
            vector.push(index);
        } else {
            self.switches
                .insert(switch, (Switch::new(switch), vec![index]));
        }

        index
    }

    pub fn add_bidirectional_switch(
        &mut self,
        switch: Address<SwitchAddr>,
        position: Position,
        s_type: SwitchType,
    ) -> (NodeIndex, NodeIndex) {
        let index = self.road.add_node(Node::Switch(
            switch,
            position,
            s_type,
            None,
            Direction::Outgoing,
        ));
        let index_reverse = self.road.add_node(Node::Switch(
            switch,
            position,
            s_type,
            None,
            Direction::Outgoing,
        ));

        if let Some((_, vector)) = self.switches.get_mut(&switch) {
            vector.push(index);
            vector.push(index_reverse)
        } else {
            self.switches
                .insert(switch, (Switch::new(switch), vec![index, index_reverse]));
        }

        (index, index_reverse)
    }

    pub fn remove_train(&mut self, adr: &Address<TrainAddr>) {
        self.trains.remove(adr);
    }

    pub fn remove_sensor(&mut self, adr: &Address<SensorAddr>) {
        self.sensors.remove(adr);
    }

    pub fn remove_signal(&mut self, adr: &Address<SignalAddr>) {
        if let Some(signal) = self.signals.remove(adr) {
            self.road.remove_node(signal.representing_node());
        }
    }

    pub fn remove_crossing(&mut self, adr: &Address<CrossingAddr>) {
        self.crossings.remove(adr);
    }

    pub fn remove_switch(&mut self, adr: &Address<SwitchAddr>) {
        self.switches.remove(adr);
    }

    fn can_add_neighbour(&self, node: NodeIndex, dir: Direction) -> Option<bool> {
        fn switch_max_neighbours(ins: usize, out: usize, dir: Direction) -> bool {
            if ins < 2 && out < 2 {
                true
            } else {
                !((ins >= 2 && dir == Direction::Incoming)
                    || (out >= 2 && dir == Direction::Outgoing))
            }
        }

        let ins = self
            .road
            .neighbors_directed(node, Direction::Incoming)
            .count();
        let out = self
            .road
            .neighbors_directed(node, Direction::Outgoing)
            .count();

        match self.road.node_weight(node)? {
            Node::Switch(..) => Some(switch_max_neighbours(ins, out, dir)),
            _ => Some(
                (dir == Direction::Incoming && ins < 1) || (dir == Direction::Outgoing && out < 1),
            ),
        }
    }

    pub fn connect(
        &mut self,
        from: NodeIndex,
        to: NodeIndex,
        rail: Vec<Rail>,
    ) -> Option<EdgeIndex> {
        if self.can_add_neighbour(from, Direction::Outgoing)?
            && self.can_add_neighbour(to, Direction::Incoming)?
        {
            Some(self.road.update_edge(from, to, rail))
        } else {
            None
        }
    }

    pub fn connect_bidirectional(
        &mut self,
        one_end: (NodeIndex, NodeIndex),
        other_end: (NodeIndex, NodeIndex),
        rail: Vec<Rail>,
    ) -> Option<(EdgeIndex, EdgeIndex)> {
        if self.can_add_neighbour(one_end.0, Direction::Outgoing)?
            && self.can_add_neighbour(one_end.1, Direction::Incoming)?
            && self.can_add_neighbour(other_end.1, Direction::Outgoing)?
            && self.can_add_neighbour(other_end.0, Direction::Incoming)?
        {
            Some((
                self.road.update_edge(one_end.0, other_end.0, rail.clone()),
                self.road.update_edge(other_end.1, one_end.1, rail),
            ))
        } else {
            None
        }
    }

    /// This function is automatically called at railroad building for the first connected neighbour of your switch, if you do not set it manually before.
    pub fn set_switch_default_dir(&mut self, switch: NodeIndex, default_connection: NodeIndex) {
        let ins = self.road.neighbors_directed(switch, Direction::Incoming);

        let mut dir = Direction::Outgoing;

        if ins.count() == 2 {
            dir = Direction::Incoming;
        }

        if let Some(Node::Switch(_, _, _, def_con, old_dir)) = self.road.node_weight_mut(switch) {
            *def_con = Some(default_connection);
            *old_dir = dir
        }
    }

    /// This function is automatically called at railroad building for the first connected neighbour of your switch, if you do not set it manually before.
    pub fn set_switch_default_dir_bidirectional(
        &mut self,
        switch: (NodeIndex, NodeIndex),
        default_connection: (NodeIndex, NodeIndex),
    ) {
        self.set_switch_default_dir(switch.0, default_connection.0);
        self.set_switch_default_dir(switch.1, default_connection.1);
    }

    fn set_first_neighbour_for_switch(&mut self, switch: NodeIndex) {
        if let Some(Node::Switch(_, _, _, Some(_), _)) = self.road.node_weight_mut(switch) {
            return;
        }

        let mut ins = self.road.neighbors_directed(switch, Direction::Incoming);
        let mut out = self.road.neighbors_directed(switch, Direction::Outgoing);

        let mut dir = Direction::Outgoing;
        let connection;

        if ins.clone().count() == 2 {
            connection = ins.next().unwrap();
            dir = Direction::Incoming;
        } else if out.clone().count() == 2 {
            connection = out.next().unwrap();
        } else {
            return;
        }

        if let Some(Node::Switch(_, _, _, def_con, old_dir)) = self.road.node_weight_mut(switch) {
            *def_con = Some(connection);
            *old_dir = dir;
        }
    }

    /// Builds a railroad out of this reader.
    pub async fn build(
        mut self,
    ) -> Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr> {
        {
            let tmp_switches = self.switches.clone();
            let switch_nodes = tmp_switches
                .iter()
                .flat_map(|(_address, (_switch, nodes))| nodes.iter());
            switch_nodes.for_each(|node| self.set_first_neighbour_for_switch(*node));
        }

        let road = Mutex::new(self.road);
        let trains = self
            .trains
            .into_iter()
            .map(|(adr, train)| (adr, Mutex::new(train)))
            .collect();
        let sensors = self
            .sensors
            .into_iter()
            .map(|(adr, sensor)| (adr, (Mutex::new(sensor.0), sensor.1)))
            .collect();
        let signals = self
            .signals
            .into_iter()
            .map(|(adr, signal)| (adr, Mutex::new(signal)))
            .collect();
        let crossings = self
            .crossings
            .into_iter()
            .map(|(adr, cross)| (adr, Mutex::new(cross)))
            .collect();
        let switches = self
            .switches
            .into_iter()
            .map(|(adr, switch)| (adr, (Mutex::new(switch.0), switch.1)))
            .collect();

        let railroad = Railroad {
            road,
            trains,
            sensors,
            signals,
            crossings,
            switches,
            channel: self.channel,
        };

        for signal in railroad.signals.values() {
            let mut signal = signal.lock().await;
            signal.initialize(&railroad).await;
        }

        railroad
    }
}
