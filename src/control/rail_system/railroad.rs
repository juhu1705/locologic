use crate::control::rail_system::components::{
    Cross, Node, Position, Rail, Sensor, Signal, SignalType, Switch, SwitchType,
};
use crate::control::train::Train;
use locodrive::args::AddressArg;
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
    trains: HashMap<AddressArg, Mutex<Train>>,
    sensors: HashMap<AddressArg, (Mutex<Sensor>, Vec<NodeIndex>)>,
    signals: HashMap<AddressArg, Mutex<Signal>>,
    crossings: HashMap<AddressArg, Mutex<Cross>>,
    switches: HashMap<AddressArg, (Mutex<Switch>, Vec<NodeIndex>)>,
}

impl Railroad {
    /// Returns a **clone** of the railroad
    pub async fn road(&self) -> DiGraph<Node, Vec<Rail>> {
        self.road.lock().await.clone()
    }

    pub fn get_sensor_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Sensor>> {
        Some(&self.sensors.get(adr)?.0)
    }

    pub fn get_signal_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Signal>> {
        self.signals.get(adr)
    }

    pub fn get_train(&self, adr: &AddressArg) -> Option<&Mutex<Train>> {
        self.trains.get(adr)
    }

    pub fn get_switch_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Switch>> {
        Some(&self.switches.get(adr)?.0)
    }

    pub fn get_crossing_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Cross>> {
        self.crossings.get(adr)
    }

    pub async fn shortest_path(
        rail: Arc<Self>,
        start: NodeIndex,
        destination: NodeIndex,
    ) -> Option<(f64, Vec<NodeIndex>)> {
        let graph = { rail.road.lock().await.clone() };
        if let Ok(result) = spawn_blocking(move || {
            astar(
                &graph,
                start,
                |goal| goal == destination,
                |cost| {
                    let rail_cost: f64 = cost.weight().iter().map(Rail::sqrt_distance).sum();
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
) -> f64 {
    let node_pos = graph.index(node).position(&rail);
    let dest_pos = graph.index(dest).position(&rail);

    node_pos.coord().abs_distance(&dest_pos.coord())
}

fn node_cost(graph: &Graph<Node, Vec<Rail>>, node: NodeIndex, rail: Arc<Railroad>) -> f64 {
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
                    100f64
                } else {
                    1f64
                }
            } else {
                100f64
            }
        }
        Node::Station(..) => 500f64,
        _ => 1f64,
    }
}

pub struct Builder {
    road: DiGraph<Node, Vec<Rail>>,
    trains: HashMap<AddressArg, Train>,
    sensors: HashMap<AddressArg, (Sensor, Vec<NodeIndex>)>,
    signals: HashMap<AddressArg, Signal>,
    crossings: HashMap<AddressArg, Cross>,
    switches: HashMap<AddressArg, (Switch, Vec<NodeIndex>)>,
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
        sensors: Vec<(AddressArg, Position)>,
    ) -> Vec<(NodeIndex, AddressArg)> {
        sensors
            .iter()
            .map(|(s_adr, s_pos)| (self.add_sensor(*s_adr, *s_pos), *s_adr))
            .collect()
    }

    pub fn add_sensor(&mut self, sensor: AddressArg, position: Position) -> NodeIndex {
        let node = self.road.add_node(Node::Sensor(sensor, position));

        if let Some((_, vector)) = self.sensors.get_mut(&sensor) {
            vector.push(node);
        } else {
            self.sensors
                .insert(sensor, (Sensor::new(sensor), vec![node]));
        }

        node
    }

    pub fn add_stations(
        &mut self,
        stations: Vec<(AddressArg, Position)>,
    ) -> Vec<(NodeIndex, AddressArg)> {
        stations
            .iter()
            .map(|station| (self.add_station(station.0, station.1), station.0))
            .collect()
    }

    pub fn add_station(&mut self, station: AddressArg, position: Position) -> NodeIndex {
        let index = self.road.add_node(Node::Station(station, position));

        if let Some((_, vector)) = self.sensors.get_mut(&station) {
            vector.push(index)
        } else {
            self.sensors
                .insert(station, (Sensor::new(station), vec![index]));
        }

        index
    }

    pub fn add_signals(
        &mut self,
        signals: Vec<(AddressArg, SignalType, Position)>,
    ) -> HashMap<AddressArg, NodeIndex> {
        signals
            .into_iter()
            .filter_map(|signal| Some((signal.0, self.add_signal(signal.0, signal.1, signal.2)?)))
            .collect()
    }

    pub fn add_signal(
        &mut self,
        signal: AddressArg,
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
        cross: AddressArg,
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
        switches: Vec<(AddressArg, Position, SwitchType)>,
    ) -> Vec<(NodeIndex, AddressArg)> {
        switches
            .into_iter()
            .map(|switch| (self.add_switch(switch.0, switch.1, switch.2), switch.0))
            .collect()
    }

    pub fn add_switch(
        &mut self,
        switch: AddressArg,
        position: Position,
        s_type: SwitchType,
    ) -> NodeIndex {
        let index = self.road.add_node(Node::Switch(switch, position, s_type));

        if let Some((_, vector)) = self.switches.get_mut(&switch) {
            vector.push(index)
        } else {
            self.switches
                .insert(switch, (Switch::new(switch), vec![index]));
        }

        index
    }

    pub fn remove_train(&mut self, adr: &AddressArg) {
        self.trains.remove(adr);
    }

    pub fn remove_sensor(&mut self, adr: &AddressArg) {
        self.sensors.remove(adr);
    }

    pub fn remove_signal(&mut self, adr: &AddressArg) {
        if let Some(signal) = self.signals.remove(adr) {
            self.road.remove_node(signal.representing_node());
        }
    }

    pub fn remove_crossing(&mut self, adr: &AddressArg) {
        self.crossings.remove(adr);
    }

    pub fn remove_switch(&mut self, adr: &AddressArg) {
        self.switches.remove(adr);
    }

    fn get_max_neighbours(&self, node: NodeIndex, dir: Direction) -> Option<bool> {
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
        if self.get_max_neighbours(from, Direction::Outgoing)?
            && self.get_max_neighbours(to, Direction::Incoming)?
        {
            Some(self.road.update_edge(from, to, rail))
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
    use crate::control::rail_system::components::{Coord, Direction, Position, SwitchType};
    use crate::control::rail_system::railroad::{Builder, Railroad};
    use locodrive::args::AddressArg;
    use petgraph::graph::NodeIndex;
    use std::sync::Arc;

    #[allow(dead_code)]
    pub async fn create_test_railroad() -> (
        Railroad,
        Vec<(NodeIndex, AddressArg)>,
        Vec<(NodeIndex, AddressArg)>,
    ) {
        let mut builder = Builder::new();

        let sensors = builder.add_sensors(vec![
            (
                AddressArg::new(0),
                Position::new(Coord(0, 0, 0), Direction::East),
            ),
            (
                AddressArg::new(1),
                Position::new(Coord(30, 30, 0), Direction::East),
            ),
            (
                AddressArg::new(2),
                Position::new(Coord(50, 50, 1), Direction::East),
            ),
            (
                AddressArg::new(3),
                Position::new(Coord(20, 20, 1), Direction::East),
            ),
            (
                AddressArg::new(4),
                Position::new(Coord(100, 100, 1), Direction::East),
            ),
            (
                AddressArg::new(5),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(6),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(7),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(8),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(9),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(10),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(11),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(12),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(13),
                Position::new(Coord(0, 0, 0), Direction::East),
            ),
            (
                AddressArg::new(14),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(15),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(16),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(17),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(18),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(19),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
            (
                AddressArg::new(20),
                Position::new(Coord(0, 0, 1), Direction::East),
            ),
        ]);

        let switches = builder.add_switches(vec![
            (
                AddressArg::new(0),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(1),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(2),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(3),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(4),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(5),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(6),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(7),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(8),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(9),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(10),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(11),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(12),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(13),
                Position::new(Coord(0, 0, 0), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(14),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(15),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(16),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(17),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(18),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
            (
                AddressArg::new(19),
                Position::new(Coord(0, 0, 1), Direction::East),
                SwitchType::StraightRight90,
            ),
        ]);

        builder.add_signals(vec![]);

        builder.connect(sensors[5].0, sensors[3].0, vec![]);
        builder.connect(sensors[3].0, sensors[1].0, vec![]);
        builder.connect(sensors[1].0, sensors[4].0, vec![]);

        (builder.build().await, switches, sensors)
    }

    #[tokio::test]
    pub async fn test_road() {
        let (r, _switches, sensors) = create_test_railroad().await;

        let railroad = Arc::new(r);

        let sensor5index = sensors
            .iter()
            .find(|(_, address)| address.address() == 5)
            .unwrap()
            .0;
        let sensor4index = sensors
            .iter()
            .find(|(_, address)| address.address() == 4)
            .unwrap()
            .0;
        let sensor3index = sensors
            .iter()
            .find(|(_, address)| address.address() == 3)
            .unwrap()
            .0;
        let _sensor2index = sensors
            .iter()
            .find(|(_, address)| address.address() == 2)
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
                3f64,
                vec![sensor5index, sensor3index, sensor1index, sensor4index]
            ))
        )
    }
}
