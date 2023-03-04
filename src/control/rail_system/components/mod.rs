mod signal_checks;

use crate::control::rail_system::railroad::Railroad;
use async_recursion::async_recursion;
use locodrive::args::{AddressArg, SensorLevel, SwitchDirection};
use petgraph::graph::NodeIndex;
use petgraph::visit::{VisitMap, Visitable};
use std::collections::VecDeque;
use std::ops;
use std::ops::Index;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Node {
    Signal(AddressArg, Position),
    Sensor(AddressArg, Position),
    Switch(AddressArg, Position, SwitchType),
    Station(AddressArg, Position),
    Cross(AddressArg),
}

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
            _ => unreachable!(),
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

/// Represents a location in a space
/// 0 = x-Position
/// 1 = y-Position
/// 2 = z-Position
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Coord(usize, usize, usize);

impl Coord {
    pub fn x(&self) -> usize {
        self.0
    }

    pub fn y(&self) -> usize {
        self.1
    }

    pub fn z(&self) -> usize {
        self.2
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Position {
    coord: Coord,
    dir: Direction,
}

impl Position {
    pub fn new(coord: Coord, dir: Direction) -> Self {
        Position { coord, dir }
    }

    pub fn rotate_right(&mut self) {
        self.rotate_by(1);
    }

    pub fn rotate_left(&mut self) {
        self.rotate_by(7);
    }

    /// Rotates this positions direction around. A rotation of 1 unit is a 45° right rotation.
    pub fn rotate_by(&mut self, rotation: u8) {
        self.dir = self.dir.rotate_by(rotation);
    }

    pub fn coord(&self) -> Coord {
        self.coord
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

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum SwitchType {
    StraightRight90 = 0,
    StraightRight180 = 1,
    StraightLeft90 = 2,
    StraightLeft180 = 3,
    RightStraight90 = 4,
    RightStraight180 = 5,
    LeftStraight90 = 6,
    LeftStraight180 = 7,
    LeftRight90 = 8,
    LeftRight180 = 9,
    RightLeft90 = 0xA,
    RightLeft180 = 0xB,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Switch {
    address: AddressArg,
    dir: SwitchDirection,
    updated: bool,
}

impl Switch {
    pub fn new(address: AddressArg) -> Self {
        Switch {
            address,
            dir: SwitchDirection::Straight,
            updated: false,
        }
    }

    pub fn address(&self) -> AddressArg {
        self.address
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Cross {
    address: AddressArg,
    nodes: (NodeIndex, NodeIndex),
    pos: Position,
}

impl Cross {
    pub fn new(address: AddressArg, pos: Position, nodes: (NodeIndex, NodeIndex)) -> Self {
        Cross {
            address,
            nodes,
            pos,
        }
    }

    pub fn address(&self) -> AddressArg {
        self.address
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
                Status::Occupied => rhs,
                _ => self,
            },
            Status::PathFree => match rhs {
                Status::Free => self,
                _ => rhs,
            },
            Status::Occupied => self,
        }
    }
}

#[derive(Debug, Clone, Hash)]
pub struct Sensor {
    address: AddressArg,
    status: Status,
    level: SensorLevel,
    train: Option<AddressArg>,
    max_speed: u8,
}

impl Sensor {
    pub fn new(adr: AddressArg) -> Sensor {
        Sensor {
            address: adr,
            status: Status::Free,
            level: SensorLevel::Low,
            train: None,
            max_speed: 128u8,
        }
    }

    pub fn address(&self) -> AddressArg {
        self.address
    }

    pub fn max_speed(&self) -> u8 {
        self.max_speed
    }

    pub fn train(&self) -> &Option<AddressArg> {
        &self.train
    }

    pub fn block(&mut self, train: AddressArg) -> bool {
        if let Some(t) = self.train {
            t == train
        } else {
            self.train = Some(train);
            self.status = Status::Reserved | self.status;
            true
        }
    }

    pub fn free(&mut self, train: AddressArg) {
        if let Some(t) = self.train {
            if t == train {
                self.train = None;
                self.status = if self.level == SensorLevel::Low {
                    Status::Free
                } else {
                    Status::Occupied
                }
            }
        }
    }

    pub fn status(&self) -> Status {
        self.status
    }
}

#[derive(Debug, Clone)]
pub enum SignalType {
    Block(Vec<AddressArg>),
    Path,
    IntelligentPath,
}

#[derive(Debug, Clone)]
pub struct Signal {
    address: AddressArg,
    representing_node: NodeIndex,
    sig_type: SignalType,
    status: Status,
    trains: Vec<AddressArg>,
    requesters: VecDeque<AddressArg>,
    other_input_signals: Vec<AddressArg>,
    is_calculating: bool,
}

impl Signal {
    pub fn new(address: AddressArg, sig_type: SignalType, representing_node: NodeIndex) -> Self {
        Signal {
            address,
            representing_node,
            sig_type,
            status: Status::Free,
            trains: vec![],
            requesters: VecDeque::new(),
            other_input_signals: vec![],
            is_calculating: false,
        }
    }

    pub fn representing_node(&self) -> NodeIndex {
        self.representing_node
    }

    pub async fn initialize(&mut self, railroad: &Railroad) {
        let (signals, sensors) = Signal::search_block(&self.representing_node, railroad).await;
        if matches!(&self.sig_type, SignalType::Block(_)) {
            self.sig_type = SignalType::Block(sensors);
        }
        self.other_input_signals = signals;
    }

    pub async fn request_block(&mut self, train: AddressArg) {
        self.requesters.push_back(train);
    }

    async fn next(&mut self, railroad: &Railroad) {
        if self.requesters.is_empty() {
            return;
        }

        for adr in &self.other_input_signals {
            if let Some(mutex) = railroad.get_signal_mutex(adr) {
                let signal = mutex.lock().await;
                if signal.is_calculating {
                    return;
                }
            }
        }

        self.is_calculating = true;
        if let Some(free_road) = self.drive(railroad).await {
            let train = self.requesters.pop_front().unwrap();
            self.status = Status::Reserved;
            self.trains.push(train);
            railroad
                .get_train(&train)
                .unwrap()
                .lock()
                .await
                .notify(self, free_road, railroad)
                .await;
        }
        self.is_calculating = false;
    }

    pub fn address(&self) -> AddressArg {
        self.address
    }

    pub fn status(&self) -> Status {
        self.status
    }

    pub fn trigger_update(&mut self, trigger: &Status) {
        self.status = match trigger {
            Status::Reserved => match self.status {
                Status::Occupied => Status::Occupied,
                _ => Status::Reserved,
            },
            Status::Occupied => Status::Occupied,
            _ => self.status,
        }
    }

    pub async fn block_free(&self, index: NodeIndex, railroad: &Railroad) -> bool {
        for neighbor in railroad.road().await.neighbors(index) {
            if Signal::neighbours_free(neighbor, railroad).await {
                return false;
            }
        }
        true
    }

    #[async_recursion]
    async fn neighbours_free(index: NodeIndex, railroad: &Railroad) -> bool {
        match railroad.road().await.index(index) {
            Node::Sensor(sensor, ..) | Node::Station(sensor, ..) => {
                if railroad
                    .get_sensor_mutex(sensor)
                    .unwrap()
                    .lock()
                    .await
                    .status()
                    == Status::Free
                {
                    for neighbor in railroad.road().await.neighbors(index) {
                        if Signal::neighbours_free(neighbor, railroad).await {
                            return true;
                        }
                    }
                    false
                } else {
                    false
                }
            }
            Node::Signal(..) => true,
            _ => {
                for neighbor in railroad.road().await.neighbors(index) {
                    if Signal::neighbours_free(neighbor, railroad).await {
                        return true;
                    }
                }
                false
            }
        }
    }
}

impl Signal {
    pub async fn update(&mut self, railroad: &Railroad) {
        if self.trains.is_empty() {
            self.next(railroad).await;
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct LightComponents {
    pos: Position,
    status: bool,
}
