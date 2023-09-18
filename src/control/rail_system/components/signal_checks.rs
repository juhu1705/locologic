use super::*;
use fixedbitset::FixedBitSet;
use petgraph::graph::DiGraph;
use tokio::sync::MutexGuard;

fn handle_cross_route<CrossingAddr: AddressType>(
    cross: MutexGuard<Cross<CrossingAddr>>,
    parent_node: &NodeIndex,
    stack: &mut VecDeque<NodeIndex>,
) {
    if cross.nodes.0 == *parent_node {
        stack.push_back(cross.nodes.1);
    } else {
        stack.push_back(cross.nodes.0);
    }
}

async fn handle_found_node<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
    CrossingAddr: AddressType,
>(
    stack: &mut VecDeque<NodeIndex>,
    succ: NodeIndex,
    parent_node: &NodeIndex,
    graph: &DiGraph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>,
    railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
) {
    stack.push_back(succ);
    if let Some(Node::Cross(adr)) = graph.node_weight(*parent_node) {
        if let Some(cross) = railroad.get_crossing_mutex(adr) {
            handle_cross_route(cross.lock().await, parent_node, stack);
        }
    }
}

async fn handle_successor<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
    CrossingAddr: AddressType,
>(
    stack: &mut VecDeque<NodeIndex>,
    (succ, parent_node): (NodeIndex, NodeIndex),
    graph: &DiGraph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>,
    railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    discovered: &mut FixedBitSet,
    signal: &NodeIndex,
    in_signals: &mut Vec<Address<SignalAddr>>,
) {
    if discovered.visit(succ) {
        return;
    }

    match graph.node_weight(parent_node) {
        Some(Node::Signal(adr, ..)) => {
            if !graph
                .neighbors(parent_node)
                .any(|n| discovered.is_visited(&n))
                && parent_node != *signal
            {
                in_signals.push(*adr);
            }
        }
        _ => {
            handle_found_node(stack, succ, &parent_node, graph, railroad).await;
        }
    }
}

async fn search_node_neighbours<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
    CrossingAddr: AddressType,
>(
    stack: &mut VecDeque<NodeIndex>,
    graph: &DiGraph<Node<SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>, Vec<Rail>>,
    railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    discovered: &mut FixedBitSet,
    signal: &NodeIndex,
    in_signals: &mut Vec<Address<SignalAddr>>,
) -> Option<NodeIndex> {
    let node = stack.pop_front()?;
    for succ in graph.neighbors_undirected(node) {
        handle_successor(
            stack,
            (succ, node),
            graph,
            railroad,
            discovered,
            signal,
            in_signals,
        )
        .await;
    }
    Some(node)
}

impl<SignalAddr: AddressType, TrainAddr: AddressType, SensorAddr: AddressType>
    Signal<SignalAddr, TrainAddr, SensorAddr>
{
    pub(super) async fn search_block<
        Spd: SpeedType,
        SwitchAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        signal: &NodeIndex,
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    ) -> (Vec<Address<SignalAddr>>, Vec<Address<SensorAddr>>) {
        let graph = railroad.road().await;
        let mut signal_walker = graph.neighbors(*signal).detach();
        let mut sensors = vec![];
        let mut in_signals = vec![];
        let mut stack = VecDeque::new();
        let mut discovered = graph.visit_map();
        while let Some(start) = signal_walker.next(&graph) {
            discovered.visit(start.1);
            stack.push_front(start.1);
            while let Some(node) = search_node_neighbours(
                &mut stack,
                &graph,
                railroad,
                &mut discovered,
                signal,
                &mut in_signals,
            )
            .await
            {
                if let Some(Node::Sensor(adr, ..)) = graph.node_weight(node) {
                    sensors.push(*adr);
                };
            }
        }
        (in_signals, sensors)
    }

    pub(super) async fn drive<
        Spd: SpeedType,
        SwitchAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &self,
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    ) -> Option<Vec<Address<SensorAddr>>> {
        if self.status != Status::Free {
            return None;
        }

        match &self.sig_type {
            SignalType::Block => self.block_behaviour(&self.block_sensors, railroad).await,
            SignalType::Path => {
                if let Some(path) = self.path_behaviour(railroad).await {
                    Some(path)
                } else {
                    self.block_behaviour(&self.block_sensors, railroad).await
                }
            }
            SignalType::IntelligentPath => {
                if let Some(path) = self.intelligent_path_behaviour(railroad).await {
                    Some(path)
                } else {
                    self.block_behaviour(&self.block_sensors, railroad).await
                }
            }
        }
    }

    async fn get_route<Spd: SpeedType, SwitchAddr: AddressType, CrossingAddr: AddressType>(train: &Address<TrainAddr>, signal: &Address<SignalAddr>, railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>) -> Option<Vec<NodeIndex>> {
        let train = railroad.get_train(train)?.lock().await;
        Some(train.request_route(*signal, railroad).await?.iter().map(|x| **x).collect())
    }

    async fn path_behaviour<Spd: SpeedType, SwitchAddr: AddressType, CrossingAddr: AddressType>(
        &self,
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>
    ) -> Option<Vec<Address<SensorAddr>>> {


        let first = &self.requesters.front()?;
        let route = Signal::get_route(first, &self.address, railroad).await?;
        if !Signal::path_free(&route, railroad, matches!(&self.sig_type, SignalType::Path)).await {
            return None;
        }

        self.block_behaviour(&self.block_sensors, railroad).await
    }

    async fn intelligent_path_behaviour<
        Spd: SpeedType,
        SwitchAddr: AddressType,
        CrossingAddr: AddressType,
    >(
        &self,
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    ) -> Option<Vec<Address<SensorAddr>>> {
        let first = &self.requesters.front()?;
        let route = Signal::get_route(first, &self.address, railroad).await?;
        if !Signal::path_free(&route, railroad, matches!(&self.sig_type, SignalType::Path)).await {
            return None;
        }

        let road = railroad.road().await;
        let adr_route: Vec<Address<SensorAddr>> = route
            .iter()
            .filter_map(|index| {
                let node = road.node_weight(*index)?;
                match node {
                    Node::Sensor(adr, ..) => Some(*adr),
                    _ => None,
                }
            })
            .collect();
        Some(adr_route)
    }

    async fn block_behaviour<Spd: SpeedType, SwitchAddr: AddressType, CrossingAddr: AddressType>(
        &self,
        sensors: &[Address<SensorAddr>],
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
    ) -> Option<Vec<Address<SensorAddr>>> {
        for sensor in sensors {
            if let Some(sensor) = railroad.get_sensor_mutex(sensor) {
                let sensor = sensor.lock().await;

                if !matches!(sensor.status(), Status::Free | Status::PathFree) {
                    return None;
                }
            }
        }
        Some(sensors.to_vec())
    }

    async fn path_free<Spd: SpeedType, SwitchAddr: AddressType, CrossingAddr: AddressType>(
        path: &[NodeIndex],
        railroad: &Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr, CrossingAddr>,
        ignore_signal: bool,
    ) -> bool {
        for x in path.iter() {
            if let Some(status) = {
                if let Node::Signal(sig, ..) = railroad.road().await.index(*x) {
                    return railroad
                        .get_signal_mutex(sig)
                        .unwrap()
                        .lock()
                        .await
                        .status()
                        == Status::Free
                        || ignore_signal;
                } else if let Node::Sensor(sensor, ..) = railroad.road().await.index(*x) {
                    Some(
                        railroad
                            .get_sensor_mutex(sensor)
                            .unwrap()
                            .lock()
                            .await
                            .status(),
                    )
                } else {
                    None
                }
            } {
                if status != Status::Free {
                    return false;
                }
            }
        }
        true
    }
}
