use crate::control::rail_system::rail_graph::{LocoGraph, Node};
use crate::control::rail_system::railroad::Railroad;
use crate::control::train::Train;
use locodrive::args::{AddressArg, SwitchDirection};
use petgraph::graph::NodeIndex;
use petgraph::Graph;
use std::ops;
use std::ops::{Index};

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

#[derive(Debug, Clone, Hash)]
pub struct Sensor {
    address: AddressArg,
    status: Status,
    trains: Vec<AddressArg>,
    actual: Option<AddressArg>,
    pos: Position,
}

impl Sensor {
    pub fn new(adr: AddressArg, pos: Position) -> Sensor {
        Sensor {
            address: adr,
            status: Status::Free,
            trains: vec![],
            actual: None,
            pos,
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

#[derive(Debug, Copy, Clone)]
pub enum SignalType {
    Block,
    Path,
    IntelligentPath,
}

#[derive(Debug, Clone)]
pub struct Signal {
    address: AddressArg,
    sig_type: SignalType,
    pos: Position,
    status: Status,
    trains: Vec<AddressArg>,
}

impl Signal {
    pub fn new(
        sig_type: SignalType,
        address: AddressArg,
        pos: Position,
    ) -> Self {
        Signal {
            address,
            sig_type,
            pos,
            status: Status::Free,
            trains: vec![],
        }
    }

    pub fn status(&self) -> Status {
        self.status
    }

    pub fn trigger_update(&mut self, trigger: &Status) {
        self.status = match trigger {
            Status::Free | Status::PathFree => self.status,
            Status::Reserved => match self.status {
                Status::Free | Status::Reserved | Status::PathFree => Status::Reserved,
                Status::Occupied => Status::Occupied,
            },
            Status::Occupied => Status::Occupied,
        }
    }

    pub fn block_free<'r>(&self, index: NodeIndex, graph: &'r LocoGraph, railroad: &'r Railroad) -> bool {
        let rail = graph.graph();

        rail.neighbors(index).all(|x| self.neighbours_free(x, rail, railroad))
    }

    fn neighbours_free<'r>(&self, index: NodeIndex, rail: &'r Graph<Node, Vec<Rail>>, railroad: &'r Railroad) -> bool {
        match rail.index(index.clone()) {
            Node::Sensor(sensor) | Node::Station(sensor) => {
                railroad.get_sensor_mutex(sensor).unwrap().lock().unwrap().status() == Status::Free
                    && rail
                        .neighbors(index.clone())
                        .into_iter()
                        .all(|x| self.neighbours_free(x, rail, railroad))
            }
            Node::Signal(_) => true,
            _ => rail
                .neighbors(index)
                .all(|index| self.neighbours_free(index, rail, railroad)),
        }
    }

    pub fn path_free(&self, path: &Vec<NodeIndex>, graph: &LocoGraph, railroad: &Railroad) -> bool {
        let mut is_in_range = false;

        path.iter()
            .filter_map(|x| {
                if let Node::Signal(sig) = graph.graph().index(*x) {
                    if is_in_range {
                        is_in_range = false;
                        return Some(railroad.get_signal_mutex(sig).unwrap().lock().unwrap().status());
                    }
                } else if let Node::Sensor(sensor) = graph.graph().index(*x) {
                    if is_in_range {
                        return Some(railroad.get_sensor_mutex(sensor).unwrap().lock().unwrap().status());
                    }
                }
                None
            })
            .all(|sensor| sensor == Status::Free)
    }
}

#[derive(Debug, Clone)]
pub struct Block {
    sensors: Vec<AddressArg>,
    in_signals: Vec<AddressArg>,
    out_signals: Vec<AddressArg>,
    status: Status,
}

impl Block {
    pub fn new() -> Block {
        Block {
            sensors: vec![],
            in_signals: vec![],
            out_signals: vec![],
            status: Status::Free,
        }
    }

    pub fn add_sensor(&mut self, sensor: &Sensor) -> &mut Self {
        self.sensors.push(sensor.address);
        self
    }

    pub fn add_in_signal(&mut self, signal: &Signal) -> &mut Self {
        self.in_signals.push(signal.address);
        self
    }

    pub fn add_out_signal(&mut self, signal: &Signal) -> &mut Self {
        self.out_signals.push(signal.address);
        self
    }

    pub fn get_block_status(&self) -> Status {
        self.status
    }

    pub fn get_block_status_with_signals(&self, railroad: &Railroad) -> Status {
        self.out_signals
            .iter()
            .fold(self.status, |status, signal| status | railroad.get_signal_mutex(signal).unwrap().lock().unwrap().status())
    }

    pub fn occupy(&mut self, railroad: &Railroad) {
        self.status = Status::Occupied;

        self.update_signals(railroad);
    }

    pub fn free(&mut self, sensor: &Sensor, railroad: &Railroad) {
        self.status = self
            .sensors
            .iter()
            .fold(Status::Free, |status, sensor| status | railroad.get_sensor_mutex(sensor).unwrap().lock().unwrap().status());

        if let Status::Free = self.status {
            self.update_signals_by_sensor(sensor, railroad);
        }
    }

    fn update_signals(&mut self, railroad: &Railroad) {
        for signal in &self.in_signals {
            if let Some(mut_signal) = railroad.get_signal_mutex(&signal) {
                let mut m_signal = mut_signal.lock().unwrap();
                m_signal.trigger_update(&self.status);
            }
        }
    }

    fn update_signals_by_sensor(&mut self, sensor: &Sensor, railroad: &Railroad) {
        for signal in &self.in_signals {
            if let Some(mut_signal) = railroad.get_signal_mutex(&signal) {
                let mut m_signal = mut_signal.lock().unwrap();
                m_signal.trigger_update(&sensor.status);
            }
        }
    }
}

pub struct Switch {
    pos: Position,
    dir: SwitchDirection,
    updated: bool,
}

impl Switch {
    pub fn new(pos: Position, dir: SwitchDirection, updated: bool) -> Self {
        Switch {
            pos,
            dir,
            updated
        }
    }
}

pub struct DecorationComponents {
    pos: Position,
}
