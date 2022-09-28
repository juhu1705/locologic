use crate::control::rail_system::components::{Direction, Position, Rail, Sensor, Signal, Switch};
use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use std::ops::Index;

pub enum Node<'t> {
    Signal(Signal),
    Sensor(Sensor<'t>),
    Switch(Switch),
    Station(Sensor<'t>),
}

pub struct LocoGraph<'t> {
    graph: Graph<Node<'t>, Vec<Rail>>,
}

impl<'t> LocoGraph<'t> {
    pub fn new() -> Self {
        let mut graph = Graph::<Node<'t>, Vec<Rail>>::new();

        let sig = Signal::Path {
            pos: Position::new(
                10,
                10,
                10,
                Direction::East,
            ),
        };

        // TEST CODE - START
        let test_node = graph.add_node(Node::Signal(sig));

        graph.add_edge(test_node, test_node, vec![]);
        // TEST CODE - END

        LocoGraph { graph }
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
                    if let Some(trains) = &sensor.trains {
                        if trains.iter().any(|t| t.stands()) {
                            100
                        } else {
                            1
                        }
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
