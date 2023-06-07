use crate::control::rail_system::components::{
    Address, Cross, Node, Position, Rail, Sensor, Signal, SignalType, Speed, Switch, SwitchType,
};
use crate::control::train::Train;
use petgraph::algo::astar;
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::{Direction, Graph};
use std::collections::HashMap;
use std::hash::Hash;
use std::ops::Index;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::task::spawn_blocking;

pub struct Railroad {
    road: Mutex<DiGraph<Node, Vec<Rail>>>,
    trains: HashMap<Address, Mutex<Train>>,
    sensors: HashMap<Address, (Mutex<Sensor>, Vec<NodeIndex>)>,
    signals: HashMap<Address, Mutex<Signal>>,
    crossings: HashMap<Address, Mutex<Cross>>,
    switches: HashMap<Address, (Mutex<Switch>, Vec<NodeIndex>)>,
}

impl Railroad {
    /// Returns a **clone** of the railroad
    pub async fn road(&self) -> DiGraph<Node, Vec<Rail>> {
        self.road.lock().await.clone()
    }

    pub fn get_sensor_mutex(&self, adr: &Address) -> Option<&Mutex<Sensor>> {
        Some(&self.sensors.get(adr)?.0)
    }

    pub fn get_signal_mutex(&self, adr: &Address) -> Option<&Mutex<Signal>> {
        self.signals.get(adr)
    }

    pub async fn get_sensor_index(&self, adr: &Address, pos: &Position) -> Option<&NodeIndex> {
        let road = self.road().await;
        self.sensors
            .get(adr)?
            .1
            .iter()
            .find(|ind| match *road.index(**ind) {
                Node::Signal(adr_check, pos_check) | Node::Station(adr_check, pos_check) => {
                    adr_check == *adr && pos_check == *pos
                }
                _ => false,
            })
    }

    pub async fn get_switch_index(&self, adr: &Address, pos: &Position) -> Option<&NodeIndex> {
        let road = self.road().await;
        self.switches
            .get(adr)?
            .1
            .iter()
            .find(|ind| match *road.index(**ind) {
                Node::Switch(adr_check, pos_check, ..) => {
                    adr_check == *adr && pos_check == *pos
                }
                _ => false,
            })
    }

    pub fn get_train(&self, adr: &Address) -> Option<&Mutex<Train>> {
        self.trains.get(adr)
    }

    pub fn get_switch_mutex(&self, adr: &Address) -> Option<&Mutex<Switch>> {
        Some(&self.switches.get(adr)?.0)
    }

    pub fn get_crossing_mutex(&self, adr: &Address) -> Option<&Mutex<Cross>> {
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
}

fn estimate_costs(
    graph: &Graph<Node, Vec<Rail>>,
    node: NodeIndex,
    rail: Arc<Railroad>,
    dest: NodeIndex,
) -> usize {
    let node_pos = graph.index(node).position(&rail);
    let dest_pos = graph.index(dest).position(&rail);

    node_pos.coord().manhattan_distance(&dest_pos.coord())
}

fn node_cost(graph: &Graph<Node, Vec<Rail>>, node: NodeIndex, rail: Arc<Railroad>) -> usize {
    fn train_cost(sensor_adr: &Address, rail: Arc<Railroad>) -> Option<usize> {
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

pub struct Builder {
    road: DiGraph<Node, Vec<Rail>>,
    trains: HashMap<Address, Train>,
    sensors: HashMap<Address, (Sensor, Vec<NodeIndex>)>,
    signals: HashMap<Address, Signal>,
    crossings: HashMap<Address, Cross>,
    switches: HashMap<Address, (Switch, Vec<NodeIndex>)>,
}

impl Default for Builder {
    fn default() -> Self {
        Self::new()
    }
}

impl Builder {
    pub fn new() -> Self {
        Builder {
            road: DiGraph::<Node, Vec<Rail>>::new(),
            trains: HashMap::new(),
            sensors: HashMap::new(),
            signals: HashMap::new(),
            crossings: HashMap::new(),
            switches: HashMap::new(),
        }
    }

    pub async fn from_railroad(railroad: &Railroad) -> Self {
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

        let trains = copy_map(&railroad.trains).await;
        let sensors = copy_index_map(&railroad.sensors).await;
        let signals = copy_map(&railroad.signals).await;
        let crossings = copy_map(&railroad.crossings).await;
        let switches = copy_index_map(&railroad.switches).await;

        Builder {
            road,
            trains,
            sensors,
            signals,
            crossings,
            switches,
        }
    }

    pub fn add_train(&mut self, train: Train) {
        self.trains.insert(train.address(), train);
    }

    pub fn add_sensors(
        &mut self,
        sensors: &[(Address, Speed, Position)],
    ) -> Vec<(NodeIndex, Address)> {
        sensors
            .iter()
            .map(|(s_adr, max_speed, s_pos)| (self.add_sensor(*s_adr, *max_speed, *s_pos), *s_adr))
            .collect()
    }

    pub fn add_bidirectional_sensors(
        &mut self,
        sensors: &[(Address, Speed, Position)],
    ) -> Vec<((NodeIndex, NodeIndex), Address)> {
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
        sensor: Address,
        max_speed: Speed,
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
        sensor: Address,
        max_speed: Speed,
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
        stations: &[(Address, Speed, Position)],
    ) -> Vec<(NodeIndex, Address)> {
        stations
            .iter()
            .map(|station| (self.add_station(station.0, station.1, station.2), station.0))
            .collect()
    }

    pub fn add_bidirectional_stations(
        &mut self,
        stations: &[(Address, Speed, Position)],
    ) -> Vec<((NodeIndex, NodeIndex), Address)> {
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
        station: Address,
        max_speed: Speed,
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
        station: Address,
        max_speed: Speed,
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
        signals: &[(Address, SignalType, Position)],
    ) -> HashMap<Address, NodeIndex> {
        signals
            .iter()
            .filter_map(|signal| Some((signal.0, self.add_signal(signal.0, signal.1, signal.2)?)))
            .collect()
    }

    pub fn add_signal(
        &mut self,
        signal: Address,
        signal_type: SignalType,
        position: Position,
    ) -> Option<NodeIndex> {
        if self.signals.get(&signal).is_some() {
            return None;
        }

        let node = self.road.add_node(Node::Signal(signal, position));

        self.signals
            .insert(signal, Signal::new(signal, signal_type, node));
        Some(node)
    }

    pub fn add_crossing(
        &mut self,
        cross: Address,
        pos: Position,
    ) -> Option<(NodeIndex, NodeIndex)> {
        if self.crossings.get(&cross).is_some() {
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
        switches: &[(Address, Position, SwitchType)],
    ) -> Vec<(NodeIndex, Address)> {
        switches
            .iter()
            .map(|switch| (self.add_switch(switch.0, switch.1, switch.2), switch.0))
            .collect()
    }

    pub fn add_bidirectional_switches(
        &mut self,
        switches: &[(Address, Position, SwitchType)],
    ) -> Vec<((NodeIndex, NodeIndex), Address)> {
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
        switch: Address,
        position: Position,
        s_type: SwitchType,
    ) -> NodeIndex {
        let index = self
            .road
            .add_node(Node::Switch(switch, position, s_type, NodeIndex::new(0)));

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
        switch: Address,
        position: Position,
        s_type: SwitchType,
    ) -> (NodeIndex, NodeIndex) {
        let index = self
            .road
            .add_node(Node::Switch(switch, position, s_type, NodeIndex::new(0)));
        let index_reverse =
            self.road
                .add_node(Node::Switch(switch, position, s_type, NodeIndex::new(0)));

        if let Some((_, vector)) = self.switches.get_mut(&switch) {
            vector.push(index);
            vector.push(index_reverse)
        } else {
            self.switches
                .insert(switch, (Switch::new(switch), vec![index, index_reverse]));
        }

        (index, index_reverse)
    }

    pub fn remove_train(&mut self, adr: &Address) {
        self.trains.remove(adr);
    }

    pub fn remove_sensor(&mut self, adr: &Address) {
        self.sensors.remove(adr);
    }

    pub fn remove_signal(&mut self, adr: &Address) {
        if let Some(signal) = self.signals.remove(adr) {
            self.road.remove_node(signal.representing_node());
        }
    }

    pub fn remove_crossing(&mut self, adr: &Address) {
        self.crossings.remove(adr);
    }

    pub fn remove_switch(&mut self, adr: &Address) {
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

    #[warn(unused_assignments)]
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

    pub async fn build(self) -> Railroad {
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
        };

        for signal in railroad.signals.values() {
            let mut signal = signal.lock().await;
            signal.initialize(&railroad).await;
        }

        railroad
    }
}
