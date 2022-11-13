use crate::control::rail_system::rail_graph::{LocoGraph, Node};
use crate::control::rail_system::railroad::Railroad;
use crate::control::train::Train;
use locodrive::args::{AddressArg, SwitchDirection};
use petgraph::graph::NodeIndex;
use petgraph::Graph;
use std::ops;
use std::ops::Index;

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

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum Status {
    Free,
    Reserved,
    PathFree,
    Occupied,
}

impl ops::BitOr for Status {
    type Output = Status;

    fn bitor(self, rhs: Self) -> Self::Output {
        match self {
            Status::Free => rhs,
            Status::Reserved => match rhs {
                Status::Free => self,
                Status::Reserved => self,
                Status::PathFree => self,
                Status::Occupied => rhs,
            },
            Status::PathFree => match rhs {
                Status::Free => self,
                Status::Reserved => rhs,
                Status::PathFree => rhs,
                Status::Occupied => rhs,
            },
            Status::Occupied => self,
        }
    }
}

pub struct Sensor<'t> {
    address: AddressArg,
    status: Status,
    trains: Vec<AddressArg>,
    actual: Option<AddressArg>,
    pos: Position,
    block: Box<Block<'t>>,
}

impl<'t> Sensor<'t> {
    pub fn new(adr: AddressArg, pos: Position, block: Box<Block<'t>>) -> Sensor<'t> {
        Sensor {
            address: adr,
            status: Status::Free,
            trains: vec![],
            actual: None,
            pos,
            block,
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
        self.status
    }
}

#[derive(Clone)]
pub enum SignalType {
    Block,
    Path,
    IntelligentPath,
}

pub struct Signal<'t> {
    address: AddressArg,
    sig_type: SignalType,
    pos: Position,
    status: Status,
    trains: Vec<AddressArg>,
    block: Box<Block<'t>>,
}

impl<'t> Signal<'t> {
    pub fn new(
        sig_type: SignalType,
        address: AddressArg,
        pos: Position,
        block: Box<Block<'t>>,
    ) -> Self {
        Signal {
            address,
            sig_type,
            pos,
            status: Status::Free,
            trains: vec![],
            block,
        }
    }

    pub fn status(&self) -> Status {
        self.status
    }

    pub fn sensor_update(&mut self, trigger: &'t Sensor<'t>) {
        match trigger.status {
            Status::Free | Status::PathFree => {}
            Status::Reserved => match self.status {
                Status::Free => self.status = Status::Reserved,
                Status::Reserved => {}
                Status::PathFree => {}
                Status::Occupied => {}
            },
            Status::Occupied => {
                self.status = Status::Occupied;
            }
        }
    }

    pub fn signal_update(&mut self, trigger: &'t Signal<'t>) {
        match trigger.status {
            Status::Free | Status::PathFree => {}
            Status::Reserved => match self.status {
                Status::Free => self.status = Status::Reserved,
                Status::Reserved => {}
                Status::PathFree => {}
                Status::Occupied => {}
            },
            Status::Occupied => {
                self.status = Status::Occupied;
            }
        }
    }

    pub fn block_free<'r>(&self, index: NodeIndex, graph: &'r LocoGraph<'r>) -> bool {
        let rail = graph.graph();

        rail.neighbors(index).all(|x| self.neighbours_free(x, rail))
    }

    fn neighbours_free(&self, index: NodeIndex, rail: &Graph<Node, Vec<Rail>>) -> bool {
        match rail.index(index.clone()) {
            Node::Sensor(sensor) | Node::Station(sensor) => {
                sensor.status() == Status::Free
                    && rail
                        .neighbors(index.clone())
                        .into_iter()
                        .all(|x| self.neighbours_free(x, rail))
            }
            Node::Signal(_) => true,
            _ => rail
                .neighbors(index)
                .all(|index| self.neighbours_free(index, rail)),
        }
    }

    pub fn path_free<'r>(&self, path: &Vec<NodeIndex>, graph: &'r LocoGraph<'r>) -> bool {
        let mut is_in_range = false;

        path.iter()
            .filter_map(|x| {
                if let Node::Signal(sig) = graph.graph().index(*x) {
                    if is_in_range {
                        is_in_range = false;
                        return Some(sig.status());
                    }
                } else if let Node::Sensor(sensor) = graph.graph().index(*x) {
                    if is_in_range {
                        return Some(sensor.status());
                    }
                }
                None
            })
            .all(|sensor| sensor == Status::Free)
    }

    pub fn update(&mut self, status: Status) {
        self.status = status;
    }
}

pub struct Block<'t> {
    sensors: Vec<&'t Sensor<'t>>,
    in_signals: Vec<&'t Signal<'t>>,
    out_signals: Vec<&'t Signal<'t>>,
    status: Status,
    railroad: &'t Railroad<'t>,
}

impl<'t> Block<'t> {
    pub fn new<'r>(railroad: &'r Railroad<'r>) -> Block<'r> {
        Block {
            sensors: vec![],
            in_signals: vec![],
            out_signals: vec![],
            status: Status::Free,
            railroad,
        }
    }

    pub fn get_block_status(&self) -> Status {
        self.status
    }

    pub fn get_block_status_with_signals(&self) -> Status {
        self.out_signals
            .iter()
            .fold(self.status, |status, signal| status | signal.status())
    }

    pub async fn occupy(&mut self) {
        self.status = Status::Occupied;

        self.update_signals().await;
    }

    pub async fn free<'r: 't>(&'r mut self, sensor: &'r Sensor<'r>) {
        self.status = self
            .sensors
            .iter()
            .fold(Status::Free, |status, sensor| status | sensor.status());

        if let Status::Free = self.status {
            self.update_signals_by_sensor(sensor).await;
        }
    }

    async fn update_signals(&mut self) {
        for signal in &self.in_signals {
            if let Some(mut_signal) = self.railroad.get_signal_mutex(&signal.address) {
                let mut m_signal = mut_signal.lock().await;
                m_signal.update(self.status);
            }
        }
    }

    async fn update_signals_by_sensor<'r: 't>(&'r mut self, sensor: &'r Sensor<'r>) {
        for signal in &self.in_signals {
            if let Some(mut_signal) = self.railroad.get_signal_mutex(&signal.address) {
                let mut m_signal = mut_signal.lock().await;
                m_signal.sensor_update(sensor);
            }
        }
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
