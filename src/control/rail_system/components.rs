use crate::control::rail_system::rail_graph::{LocoGraph, Node};
use crate::control::train::Train;
use locodrive::args::{AddressArg, SwitchDirection};
use petgraph::graph::NodeIndex;
use petgraph::Graph;
use std::ops::Index;
use tokio::sync::watch;
use tokio::sync::watch::Sender;
use tokio::sync::watch::Receiver;

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum Direction {
    North = 0,
    Northeast = 1,
    East = 2,
    Southeast = 3,
    South = 4,
    Southwest = 5,
    West = 6,
    Northwest = 7,
}

impl From<u8> for Direction {
    fn from(dir_num: u8) -> Direction {
        match dir_num % 8 {
            0 => Direction::North,
            1 => Direction::Northeast,
            2 => Direction::East,
            3 => Direction::Southeast,
            4 => Direction::South,
            5 => Direction::Southwest,
            6 => Direction::West,
            7 => Direction::Northwest,
            _ => Direction::North,
        }
    }
}

impl Direction {
    pub fn rotate_right(self) -> Direction {
        self.rotate_by(1)
    }

    pub fn rotate_left(self) -> Direction {
        self.rotate_by(7)
    }

    /// Rotates this positions direction around. A rotation of 1 unit is a 45° right rotation.
    pub fn rotate_by(self, rotation: u8) -> Direction {
        Direction::from((self as u8) + rotation)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Position {
    x: usize,
    y: usize,
    z: usize,
    dir: Direction,
}

impl Position {
    pub fn new(x: usize, y: usize, z: usize, dir: Direction) -> Self {
        Position { x, y, z, dir }
    }

    pub fn rotate_right(&mut self) {
        self.rotate_by(1)
    }

    pub fn rotate_left(&mut self) {
        self.rotate_by(7)
    }

    /// Rotates this positions direction around. A rotation of 1 unit is a 45° right rotation.
    pub fn rotate_by(&mut self, rotation: u8) {
        self.dir = self.dir.rotate_by(rotation)
    }

    pub fn x(&self) -> usize {
        self.x
    }

    pub fn y(&self) -> usize {
        self.y
    }

    pub fn z(&self) -> usize {
        self.z
    }

    pub fn dir(&self) -> Direction {
        self.dir
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Rail {
    length: usize,
    pos: Position,
    start_dir: Direction,
    end_dir: Direction,
}

impl Rail {
    pub fn length(&self) -> usize {
        self.length
    }

    pub fn pos(&self) -> Position {
        self.pos
    }

    pub fn start_dir(&self) -> Direction {
        self.start_dir
    }

    pub fn end_dir(&self) -> Direction {
        self.end_dir
    }
}

pub enum Status {
    Free,
    Reserved,
    PathFree,
    Occupied,
}

#[derive(Clone, Eq, PartialEq)]
pub struct Sensor<'t> {
    address: AddressArg,
    status: Status,
    trains: Vec<AddressArg>,
    actual: Option<AddressArg>,
    pos: Position,
    signals: Vec<&'t Signal<'t>>,
}

impl<'t> Sensor<'t> {
    pub fn new(adr: AddressArg, pos: Position) -> Sensor<'t> {
        Sensor {
            address: adr,
            status: Status::Free,
            trains: vec![],
            actual: None,
            pos,
            signals: vec![]
        }
    }

    pub(crate) fn subscribe(&mut self, train: &Train) {
        self.trains.push(train.address());
    }

    pub(crate) fn trains(&self) -> &Vec<AddressArg> {
        &self.trains
    }

    pub fn can_drive_on_sensor(&mut self, train: &Train) -> bool {
        if let Some(t) = self.actual {
            if t == train.address() {
                true
            } else {
                false
            }
        } else {
            if self.trains.starts_with(&[train.address()]) {
                self.actual = Some(self.trains.pop().unwrap());
                true
            } else {
                false
            }
        }
    }

    pub fn status(&self) -> Status {
        *self.status
    }
}

#[derive(Clone)]
pub enum SignalType<'t> {
    Block,
    Path(Vec<&'t Signal<'t>>)
}

pub struct Signal<'t> {
    sig_type: SignalType<'t>,
    pos: Position,
    status: Status,
    send_to: Sender<bool>,
    watch: Receiver<bool>,
}

impl<'t> Signal<'t> {

    pub fn new(sig_type: SignalType, pos: Position) -> Self {
        let (send_to, mut watch) = watch::channel(false);

        Signal {
            sig_type,
            pos,
            status: Status::Free,
            send_to,
            watch
        }
    }

    pub fn free(
        &self,
    ) -> Status {
        *self.status
    }

    pub fn sensor_update(&self, trigger: &'t Sensor<'t>) {

    }

    pub fn signal_update(&self, trigger: &'t Signal<'t>) {

    }

    pub fn block_free(&self, index: NodeIndex, graph: &LocoGraph) -> bool {
        let rail = graph.graph();

        rail.neighbors(index).all(|x| self.neighbours_free(x, rail))
    }

    fn neighbours_free(&self, index: NodeIndex, rail: &Graph<Node, Vec<Rail>>) -> bool {
        match rail.index(index.clone()) {
            Node::Sensor(sensor) | Node::Station(sensor) => {
                sensor.free() && rail.neighbors(index.clone()).into_iter().all(|x| self.neighbours_free(x, rail))
            }
            Node::Signal(_) => true,
            _ => rail.neighbors(index).all(|index| self.neighbours_free(index, rail)),
        }
    }

    pub fn path_free(&self, path: &Vec<NodeIndex>, graph: &LocoGraph) -> bool {
        let mut is_in_range = false;

        path.iter()
            .filter_map(|x| {
                if let Node::Signal(sig) = graph.graph().index(*x) {
                    if is_in_range {
                        is_in_range = false;
                        return Some(sig.free());
                    }
                } else if let Node::Sensor(sensor) = graph.graph().index(*x) {
                    if is_in_range {
                        return Some(sensor.free());
                    }
                }
                None
            })
            .all(|sensor| sensor == Status::Free)
    }
}

pub struct Switch {
    pos: Position,
    dir: SwitchDirection,
    updated: bool,
}

pub struct DecorationComponents {
    pos: Position,
}
