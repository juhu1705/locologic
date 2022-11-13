use crate::control::rail_system::components::{Position, Rail, Sensor, Signal, Switch};
use petgraph::algo::astar;
use petgraph::graph::{Graph, NodeIndex};
use std::ops::Index;

pub enum Node<'t> {
    Signal(Signal<'t>),
    Sensor(Sensor<'t>),
    Switch(Switch),
    Station(Sensor<'t>),
}

pub struct LocoGraph<'t> {
    graph: Graph<Node<'t>, Vec<Rail>>,
}

impl<'t> LocoGraph<'t> {
    pub fn new() -> Self {
        let mut graph = Graph::<Node, Vec<Rail>>::new();

        // TEST CODE - START

        // TEST CODE - END

        LocoGraph { graph }
    }

    pub(crate) fn mut_graph(&'t mut self) -> &'t mut Graph<Node, Vec<Rail>> {
        &mut self.graph
    }

    pub(crate) fn graph(&'t self) -> &'t Graph<Node, Vec<Rail>> {
        &self.graph
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
                        /*if let Some(t) = self.trains.get(t) {
                            t.stands()
                        } else {
                            false
                        }*/
                        false
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
