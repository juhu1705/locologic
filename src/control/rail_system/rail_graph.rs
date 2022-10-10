use std::borrow::Borrow;
use crate::control::rail_system::components::{Direction, Position, Rail, Sensor, Signal, Switch};
use crate::control::train::Train;
use locodrive::args::AddressArg;
use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use petgraph::visit::Walker;
use std::collections::HashMap;
use std::ops::Index;

pub enum Node {
    Signal(Signal),
    Sensor(Sensor),
    Switch(Switch),
    Station(Sensor),
}

pub struct LocoGraph {
    graph: Graph<Node, Vec<Rail>>,
    trains: HashMap<AddressArg, Train>,
}

impl LocoGraph {
    pub fn new() -> Self {
        let mut graph = Graph::<Node, Vec<Rail>>::new();

        let sig = Signal::Path {
            pos: Position::new(10, 10, 10, Direction::East),
            report: vec![]
        };

        // TEST CODE - START
        let _test_node = graph.add_node(Node::Signal(sig));

        // TEST CODE - END

        LocoGraph {
            graph,
            trains: HashMap::new(),
        }
    }

    pub(crate) fn graph(&mut self) -> &mut Graph<Node, Vec<Rail>> {
        &mut self.graph
    }

    pub fn shortest_path(
        &self,
        start: NodeIndex,
        destination: NodeIndex,
    ) -> Option<(usize, Vec<NodeIndex>)> {
        astar(
            &self.graph,
            start,
            |goal| goal == destination,
            |cost| cost.weight().iter().map(|rail| rail.length()).sum(),
            |node: NodeIndex| match self.graph.index(node) {
                Node::Sensor(sensor) => {
                    if sensor.trains().iter().any(|t| {
                        if let Some(t) = self.trains.get(t) {
                            t.stands()
                        } else {
                            false
                        }
                    }) {
                        100
                    } else {
                        1
                    }
                }
                Node::Station(..) => 200,
                _ => 1,
            },
        )
    }
}
