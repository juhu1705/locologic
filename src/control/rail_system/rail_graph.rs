use crate::control::rail_system::components::{Rail};
use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use std::ops::Index;
use locodrive::args::AddressArg;
use crate::control::rail_system::railroad::Railroad;

#[derive(Debug, Clone, Eq, PartialEq, Hash, Copy)]
pub enum Node {
    Signal(AddressArg, Position),
    Sensor(AddressArg, Position),
    Switch(AddressArg, Position, bool),
    Station(AddressArg, Position),
}

pub struct LocoGraph {
    graph: Graph<Node, Vec<Rail>>,
}

impl LocoGraph {
    pub fn new() -> Self {
        let graph = Graph::<Node, Vec<Rail>>::new();

        // TEST CODE - START

        // TEST CODE - END

        LocoGraph { graph }
    }

    pub(crate) fn mut_graph(&mut self) -> &mut Graph<Node, Vec<Rail>> {
        &mut self.graph
    }

    pub(crate) fn graph(&self) -> &Graph<Node, Vec<Rail>> {
        &self.graph
    }

    pub fn shortest_path(
        &self,
        start: NodeIndex,
        destination: NodeIndex,
        railroad: &Railroad,
    ) -> Option<(usize, Vec<NodeIndex>)> {
        astar(
            &self.graph,
            start,
            |goal| goal == destination,
            |cost| cost.weight().iter().map(|rail| rail.length()).sum(),
            |node: NodeIndex| {
                match self.graph.index(node) {
                    Node::Sensor(sensor_adr) => {
                        if let Some(sensor_mut) = railroad.get_sensor_mutex(sensor_adr) {
                            let mut trains = vec![];
                            {
                                let sensor = sensor_mut.lock().unwrap();
                                trains = sensor.trains().clone();
                            }

                            if trains.iter().any(|train| {
                                if let Some(t) = railroad.get_train(train) {
                                    t.lock().unwrap().stands()
                                } else {
                                    false
                                }
                            }) {
                                100
                            } else {
                                1
                            }
                        } else {
                            100
                        }
                    }
                    Node::Station(..) => 200,
                    _ => 1,
                }
            },
        )
    }
}
