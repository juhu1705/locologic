use crate::control::rail_system::components::{
    Cross, Node, Position, Rail, Sensor, Signal, SignalType, Switch, SwitchType,
};
use crate::control::train::Train;
use locodrive::args::AddressArg;
use petgraph::algo::astar;
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use std::collections::HashMap;
use std::hash::Hash;
use std::ops::Index;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::task::spawn_blocking;

pub struct Railroad {
    road: Mutex<DiGraph<Node, Vec<Rail>>>,
    trains: HashMap<AddressArg, Mutex<Train>>,
    sensors: HashMap<AddressArg, Mutex<Sensor>>,
    signals: HashMap<AddressArg, Mutex<Signal>>,
    crossings: HashMap<AddressArg, Mutex<Cross>>,
    switches: HashMap<AddressArg, Mutex<Switch>>,
}

impl Railroad {
    pub async fn road(&self) -> DiGraph<Node, Vec<Rail>> {
        self.road.lock().await.clone()
    }

    pub fn get_sensor_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Sensor>> {
        self.sensors.get(adr)
    }

    pub fn get_signal_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Signal>> {
        self.signals.get(adr)
    }

    pub fn get_train(&self, adr: &AddressArg) -> Option<&Mutex<Train>> {
        self.trains.get(adr)
    }

    pub fn get_switch_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Switch>> {
        self.switches.get(adr)
    }

    pub fn get_crossing_mutex(&self, adr: &AddressArg) -> Option<&Mutex<Cross>> {
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
                |cost| cost.weight().iter().map(Rail::length).sum(),
                |node| match &graph.index(node) {
                    Node::Sensor(sensor_adr, ..) => {
                        if let Some(sensor_mut) = rail.get_sensor_mutex(sensor_adr) {
                            let train = {
                                let sensor = sensor_mut.blocking_lock();
                                *sensor.train()
                            };

                            if if let Some(train) = train {
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
                                1
                            }
                        } else {
                            100
                        }
                    }
                    Node::Station(..) => 50,
                    _ => 1,
                },
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

pub struct Builder {
    road: DiGraph<Node, Vec<Rail>>,
    trains: HashMap<AddressArg, Train>,
    sensors: HashMap<AddressArg, Sensor>,
    signals: HashMap<AddressArg, Signal>,
    crossings: HashMap<AddressArg, Cross>,
    switches: HashMap<AddressArg, Switch>,
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

        let trains = copy_map(&railroad.trains).await;
        let sensors = copy_map(&railroad.sensors).await;
        let signals = copy_map(&railroad.signals).await;
        let crossings = copy_map(&railroad.crossings).await;
        let switches = copy_map(&railroad.switches).await;

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

    pub fn add_sensor(&mut self, sensor: AddressArg, position: Position) {
        if self.sensors.get(&sensor).is_none() {
            self.sensors.insert(sensor, Sensor::new(sensor));
        }

        self.road.add_node(Node::Sensor(sensor, position));
    }

    pub fn add_station(&mut self, station: AddressArg, position: Position) {
        if self.sensors.get(&station).is_none() {
            self.sensors.insert(station, Sensor::new(station));
        }

        self.road.add_node(Node::Station(station, position));
    }

    pub fn add_signal(
        &mut self,
        signal: AddressArg,
        signal_type: SignalType,
        position: Position,
    ) -> Option<NodeIndex> {
        if self.signals.get(&signal).is_none() {
            let node = self.road.add_node(Node::Signal(signal, position));

            self.signals
                .insert(signal, Signal::new(signal, signal_type, node));
            Some(node)
        } else {
            None
        }
    }

    pub fn add_crossing(&mut self, cross: AddressArg, pos: Position) -> (NodeIndex, NodeIndex) {
        let node1 = self.road.add_node(Node::Cross(cross));
        let node2 = self.road.add_node(Node::Cross(cross));

        if self.crossings.get(&cross).is_none() {
            self.crossings
                .insert(cross, Cross::new(cross, pos, (node1, node2)));
        }

        (node1, node2)
    }

    pub fn add_switch(
        &mut self,
        switch: AddressArg,
        position: Position,
        s_type: SwitchType,
    ) -> NodeIndex {
        if self.switches.get(&switch).is_none() {
            self.switches.insert(switch, Switch::new(switch));
        }

        self.road.add_node(Node::Switch(switch, position, s_type))
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

    pub fn connect(
        &mut self,
        from: NodeIndex,
        to: NodeIndex,
        rail: Vec<Rail>,
    ) -> Option<EdgeIndex> {
        let neighbours = self.road.neighbors(from).count();

        if neighbours < {
            if self.road.node_weight(to).is_none() {
                0
            } else if let Some(node) = self.road.node_weight(from) {
                match node {
                    Node::Switch(..) => 2,
                    _ => 1,
                }
            } else {
                0
            }
        } {
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
            .map(|(adr, sensor)| (adr, Mutex::new(sensor)))
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
            .map(|(adr, node)| (adr, Mutex::new(node)))
            .collect();

        let railroad = Railroad {
            road,
            trains,
            sensors,
            signals,
            crossings,
            switches,
        };

        for signal in &railroad.signals {
            let mut signal = signal.1.lock().await;
            signal.initialize(&railroad).await;
        }

        railroad
    }
}
