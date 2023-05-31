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
    match graph.index(node) {
        Node::Sensor(sensor_adr, ..) => {
            if let Some(sensor_mut) = rail.get_sensor_mutex(sensor_adr) {
                if if let Some(train) = { *(sensor_mut.blocking_lock()).train() } {
                    if let Some(t) = rail.get_train(&train) {
                        t.blocking_lock().stands()
                    } else {
                        false
                    }
                } else {
                    false
                } {
                    100
                } else {
                    2
                }
            } else {
                100
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
        sensors: Vec<(Address, Speed, Position)>,
    ) -> Vec<(NodeIndex, Address)> {
        sensors
            .iter()
            .map(|(s_adr, max_speed, s_pos)| (self.add_sensor(*s_adr, *max_speed, *s_pos), *s_adr))
            .collect()
    }

    pub fn add_bidirectional_sensors(
        &mut self,
        sensors: Vec<(Address, Speed, Position)>,
    ) -> Vec<((NodeIndex, NodeIndex), Address)> {
        sensors
            .iter()
            .map(|(s_adr, max_speed, s_pos)| (self.add_bidirectional_sensor(*s_adr, *max_speed, *s_pos), *s_adr))
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
            self.sensors
                .insert(sensor, (Sensor::new(sensor, max_speed), vec![node, node_reverse]));
        }

        (node, node_reverse)
    }

    pub fn add_stations(
        &mut self,
        stations: Vec<(Address, Speed, Position)>,
    ) -> Vec<(NodeIndex, Address)> {
        stations
            .iter()
            .map(|station| (self.add_station(station.0, station.1, station.2), station.0))
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

    pub fn add_signals(
        &mut self,
        signals: Vec<(Address, SignalType, Position)>,
    ) -> HashMap<Address, NodeIndex> {
        signals
            .into_iter()
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
        switches: Vec<(Address, Position, SwitchType)>,
    ) -> Vec<(NodeIndex, Address)> {
        switches
            .into_iter()
            .map(|switch| (self.add_switch(switch.0, switch.1, switch.2), switch.0))
            .collect()
    }

    pub fn add_bidirectional_switches(
        &mut self,
        switches: Vec<(Address, Position, SwitchType)>,
    ) -> Vec<((NodeIndex, NodeIndex), Address)> {
        switches
            .into_iter()
            .map(|switch| (self.add_bidirectional_switch(switch.0, switch.1, switch.2), switch.0))
            .collect()
    }

    pub fn add_switch(
        &mut self,
        switch: Address,
        position: Position,
        s_type: SwitchType,
    ) -> NodeIndex {
        let index = self.road.add_node(Node::Switch(switch, position, s_type));

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
        let index = self.road.add_node(Node::Switch(switch, position, s_type));
        let index_reverse = self.road.add_node(Node::Switch(switch, position, s_type));

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
        from: (NodeIndex, NodeIndex),
        to: (NodeIndex, NodeIndex),
        rail: Vec<Rail>,
    ) -> Option<(EdgeIndex, EdgeIndex)> {
        if self.can_add_neighbour(from.0, Direction::Outgoing)?
            && self.can_add_neighbour(to.0, Direction::Incoming)?
            && self.can_add_neighbour(from.1, Direction::Outgoing)?
            && self.can_add_neighbour(to.1, Direction::Incoming)?
        {
            Some((
                self.road.update_edge(from.0, to.0, rail.clone()),
                self.road.update_edge(from.1, to.1, rail)
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

#[cfg(test)]
mod railroad_test {
    use crate::control::rail_system::components::{Address, Coord, Direction, Position, Rail, SignalType, Speed, SwitchType};
    use crate::control::rail_system::railroad::{Builder, Railroad};
    use petgraph::graph::NodeIndex;
    use std::sync::Arc;

    #[allow(dead_code)]
    pub async fn create_test_railroad() -> (
        Railroad,
        Vec<(NodeIndex, Address)>,
        Vec<((NodeIndex, NodeIndex), Address)>,
        Vec<(NodeIndex, Address)>,
        Vec<((NodeIndex, NodeIndex), Address)>
    ) {
        let mut builder = Builder::new();

        let sensors = builder.add_sensors(vec![
            (
                Address::new(0),
                Speed::Drive(128),
                Position::new(Coord(6, 9, 0), Direction::South),
            ),
            (
                Address::new(1),
                Speed::Drive(128),
                Position::new(Coord(2, 9, 0), Direction::West),
            ),
            (
                Address::new(8),
                Speed::Drive(128),
                Position::new(Coord(5, 13, 0), Direction::North),
            ),
            (
                Address::new(9),
                Speed::Drive(128),
                Position::new(Coord(5, 14, 0), Direction::North),
            ),
            (
                Address::new(10),
                Speed::Drive(128),
                Position::new(Coord(5, 15, 0), Direction::North),
            ),
            (
                Address::new(11),
                Speed::Drive(128),
                Position::new(Coord(5, 18, 0), Direction::North),
            ),
            (
                Address::new(12),
                Speed::Drive(128),
                Position::new(Coord(5, 16, 0), Direction::North),
            ),
            (
                Address::new(19),
                Speed::Drive(128),
                Position::new(Coord(2, 11, 0), Direction::West),
            )
        ]);

        let bidirectional_sensors = builder.add_bidirectional_sensors(vec![
            (
                Address::new(2),
                Speed::Drive(128),
                Position::new(Coord(8, 8, 1), Direction::West),
            ),
            (
                Address::new(3),
                Speed::Drive(128),
                Position::new(Coord(10, 2, 1), Direction::West),
            ),
            (
                Address::new(4),
                Speed::Drive(128),
                Position::new(Coord(8, 3, 1), Direction::West),
            ),
            (
                Address::new(5),
                Speed::Drive(128),
                Position::new(Coord(1, 4, 1), Direction::West),
            ),
            (
                Address::new(6),
                Speed::Drive(128),
                Position::new(Coord(5, 18, 1), Direction::North),
            ),
            (
                Address::new(7),
                Speed::Drive(128),
                Position::new(Coord(5, 16, 1), Direction::North),
            ),
            (
                Address::new(13),
                Speed::Drive(128),
                Position::new(Coord(5, 6, 1), Direction::Southwest),
            ),
            (
                Address::new(14),
                Speed::Drive(128),
                Position::new(Coord(10, 4, 1), Direction::West),
            ),
            (
                Address::new(14),
                Speed::Drive(128),
                Position::new(Coord(10, 12, 1), Direction::West),
            ),
            (
                Address::new(15),
                Speed::Drive(128),
                Position::new(Coord(7, 3, 1), Direction::West),
            ),
            (
                Address::new(16),
                Speed::Drive(128),
                Position::new(Coord(7, 9, 2), Direction::West),
            ),
            (
                Address::new(17),
                Speed::Drive(128),
                Position::new(Coord(7, 12, 2), Direction::West),
            ),
            (
                Address::new(18),
                Speed::Drive(128),
                Position::new(Coord(7, 15, 2), Direction::West),
            ),
        ]);

        let switches = builder.add_switches(vec![
            (
                Address::new(1),
                Position::new(Coord(2, 12, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(2),
                Position::new(Coord(2, 13, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(3),
                Position::new(Coord(2, 15, 0), Direction::East),
                SwitchType::StraightRight180,
            ),
            (
                Address::new(4),
                Position::new(Coord(2, 16, 0), Direction::East),
                SwitchType::StraightRight180,
            ),
            (
                Address::new(5),
                Position::new(Coord(8, 16, 0), Direction::Northeast),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(6),
                Position::new(Coord(9, 13, 0), Direction::East),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(7),
                Position::new(Coord(9, 12, 0), Direction::East),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(8),
                Position::new(Coord(8, 10, 0), Direction::Southeast),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(14),
                Position::new(Coord(4, 8, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
        ]);

        let bidirectional_switches = builder.add_bidirectional_switches(vec![
            (
                Address::new(9),
                Position::new(Coord(3, 15, 1), Direction::East),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(10),
                Position::new(Coord(7, 0, 1), Direction::South),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(11),
                Position::new(Coord(2, 6, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(12),
                Position::new(Coord(2, 5, 0), Direction::West),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(13),
                Position::new(Coord(4, 6, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(15),
                Position::new(Coord(8, 6, 1), Direction::West),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(16),
                Position::new(Coord(8, 10, 1), Direction::East),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(17),
                Position::new(Coord(8, 7, 1), Direction::West),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(17),
                Position::new(Coord(10, 5, 1), Direction::East),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(18),
                Position::new(Coord(8, 9, 1), Direction::East),
                SwitchType::StraightLeft90,
            ),
            (
                Address::new(18),
                Position::new(Coord(10, 11, 1), Direction::West),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(19),
                Position::new(Coord(1, 12, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                Address::new(20),
                Position::new(Coord(3, 8, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
        ]);

        let signals = builder.add_signals(vec![
            (
                Address::new(80),
                SignalType::Path,
                Position::new(Coord(2, 10, 0), Direction::West),
            ),
            (
                Address::new(81),
                SignalType::Block,
                Position::new(Coord(3, 7, 0), Direction::Southeast)
            ),
            (
                Address::new(82),
                SignalType::Block,
                Position::new(Coord(4, 7, 0), Direction::East)
            ),
            (
                Address::new(83),
                SignalType::Path,
                Position::new(Coord(5, 6, 0), Direction::North)
            ),
            (
                Address::new(84),
                SignalType::Block,
                Position::new(Coord(4, 13, 0), Direction::North)
            ),
            (
                Address::new(85),
                SignalType::Block,
                Position::new(Coord(4, 14, 0), Direction::North)
            ),
            (
                Address::new(86),
                SignalType::Block,
                Position::new(Coord(4, 15, 0), Direction::North)
            ),
            (
                Address::new(87),
                SignalType::Block,
                Position::new(Coord(4, 16, 0), Direction::North)
            ),
            (
                Address::new(88),
                SignalType::Block,
                Position::new(Coord(4, 18, 0), Direction::North)
            ),
            (
                Address::new(89),
                SignalType::Block,
                Position::new(Coord(7, 12, 0), Direction::Northeast)
            ),
            (
                Address::new(90),
                SignalType::Block,
                Position::new(Coord(8, 13, 0), Direction::Northeast)
            ),
            (
                Address::new(91),
                SignalType::Block,
                Position::new(Coord(8, 14, 0), Direction::Northeast)
            ),
            (
                Address::new(92),
                SignalType::Block,
                Position::new(Coord(7, 16, 0), Direction::North)
            ),
            (
                Address::new(93),
                SignalType::Block,
                Position::new(Coord(7, 17, 0), Direction::Northeast)
            ),
            (
                Address::new(94),
                SignalType::Block,
                Position::new(Coord(8, 0, 1), Direction::South)
            ),
            (
                Address::new(95),
                SignalType::Path,
                Position::new(Coord(8, 0, 1), Direction::North)
            ),
            (
                Address::new(96),
                SignalType::Path,
                Position::new(Coord(8, 2, 1), Direction::West)
            ),
            (
                Address::new(97),
                SignalType::Block,
                Position::new(Coord(8, 2, 1), Direction::East)
            ),
            (
                Address::new(98),
                SignalType::Block,
                Position::new(Coord(10, 3, 1), Direction::West)
            ),
            (
                Address::new(99),
                SignalType::Path,
                Position::new(Coord(10, 3, 1), Direction::East)
            ),
            (
                Address::new(100),
                SignalType::Path,
                Position::new(Coord(7, 4, 1), Direction::West)
            ),
            (
                Address::new(101),
                SignalType::Block,
                Position::new(Coord(7, 4, 1), Direction::East)
            ),
            (
                Address::new(102),
                SignalType::Block,
                Position::new(Coord(8, 5, 1), Direction::West)
            ),
            (
                Address::new(103),
                SignalType::Path,
                Position::new(Coord(8, 5, 1), Direction::East)
            ),
            (
                Address::new(104),
                SignalType::Block,
                Position::new(Coord(8, 13, 1), Direction::East)
            ),
            (
                Address::new(105),
                SignalType::Path,
                Position::new(Coord(8, 13, 1), Direction::West)
            ),
            (
                Address::new(106),
                SignalType::Path,
                Position::new(Coord(10, 13, 1), Direction::West)
            ),
            (
                Address::new(107),
                SignalType::Block,
                Position::new(Coord(10, 13, 1), Direction::East)
            ),
            (
                Address::new(108),
                SignalType::Block,
                Position::new(Coord(4, 16, 1), Direction::South)
            ),
            (
                Address::new(109),
                SignalType::Path,
                Position::new(Coord(4, 16, 1), Direction::North)
            ),
            (
                Address::new(110),
                SignalType::Block,
                Position::new(Coord(4, 18, 1), Direction::South)
            ),
            (
                Address::new(111),
                SignalType::Path,
                Position::new(Coord(4, 18, 1), Direction::North)
            ),
            (
                Address::new(112),
                SignalType::Block,
                Position::new(Coord(1, 14, 1), Direction::West)
            ),
            (
                Address::new(113),
                SignalType::Path,
                Position::new(Coord(1, 14, 1), Direction::East)
            ),
            (
                Address::new(114),
                SignalType::Block,
                Position::new(Coord(4, 3, 2), Direction::South)
            ),
            (
                Address::new(115),
                SignalType::Path,
                Position::new(Coord(4, 3, 2), Direction::North)
            )
        ]);

        /*
        Sensors:
        | Index  | Adresse |
        |--------|---------|
        | 0      | 0       |
        | 1      | 1       |
        | 2      | 8       |
        | 3      | 9       |
        | 4      | 10      |
        | 5      | 11      |
        | 6      | 12      |
        | 7      | 19      |

        Bidirectional Sensors:
        | Index | Address |
        |-------|---------|
        | 0     | 2       |
        | 1     | 3       |
        | 2     | 4       |
        | 3     | 5       |
        | 4     | 6       |
        | 5     | 7       |
        | 6     | 13      |
        | 7     | 14 (L)  |
        | 8     | 14 (R)  |
        | 9     | 15      |
        | 10    | 16      |
        | 11    | 17      |
        | 12    | 18      |

        Switches:
        | Index | Switch |
        |-------|--------|
        | 0     | 1      |
        | 1     | 2      |
        | 2     | 3      |
        | 3     | 4      |
        | 4     | 5      |
        | 5     | 6      |
        | 6     | 7      |
        | 7     | 8      |
        | 8     | 14     |

        Bidirectional Switches:
        | Index | Switch |
        |-------|--------|
        | 0     | 9      |
        | 1     | 10     |
        | 2     | 11     |
        | 3     | 12     |
        | 4     | 13     |
        | 5     | 15     |
        | 6     | 16     |
        | 7     | 17 (I) |
        | 8     | 17 (A) |
        | 9     | 18 (I) |
        | 10    | 18 (A) |
        | 11    | 19     |
        | 12    | 20     |

        Signals:
        x -> x + 80 for x in (0..35)
        */
        
        builder.connect(sensors[0].0, switches[7].0, vec![
            Rail::new(Position::new(Coord(6, 9, 0), Direction::Southeast), 1, Direction::South)
        ]);
        builder.connect(switches[7].0, signals[&Address::new(89)], vec![
            Rail::new(Position::new(Coord(8, 11, 0), Direction::Northeast), 1, Direction::East)
        ]);

        builder.connect(sensors[5].0, sensors[3].0, vec![]);
        builder.connect(sensors[3].0, sensors[1].0, vec![]);
        builder.connect(sensors[1].0, sensors[4].0, vec![]);

        (builder.build().await, switches, bidirectional_switches, sensors, bidirectional_sensors)
    }

    #[tokio::test]
    pub async fn test_road() {
        let (r, _switches, _bi_dir_switches, sensors, _bi_dir_sensors) = create_test_railroad().await;

        let railroad = Arc::new(r);

        let sensor5index = sensors
            .iter()
            .find(|(_, address)| address.address() == 11)
            .unwrap()
            .0;
        let sensor4index = sensors
            .iter()
            .find(|(_, address)| address.address() == 10)
            .unwrap()
            .0;
        let sensor3index = sensors
            .iter()
            .find(|(_, address)| address.address() == 9)
            .unwrap()
            .0;
        let _sensor2index = sensors
            .iter()
            .find(|(_, address)| address.address() == 8)
            .unwrap()
            .0;
        let sensor1index = sensors
            .iter()
            .find(|(_, address)| address.address() == 1)
            .unwrap()
            .0;

        let route = Railroad::shortest_path(railroad.clone(), sensor5index, sensor4index).await;

        assert_eq!(
            route,
            Some((
                6,
                vec![sensor5index, sensor3index, sensor1index, sensor4index]
            ))
        )
    }
}
