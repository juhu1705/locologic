/// Interface to locodrive
#[cfg(feature = "locodrive")]
mod locodrive;
/// Implementation of larger methods regarding signals
mod signal_checks;

use crate::control::rail_system::railroad::Railroad;
use async_recursion::async_recursion;
use petgraph::graph::NodeIndex;
use petgraph::visit::{VisitMap, Visitable};
use std::cmp::Ordering;
use std::collections::VecDeque;
use std::ops;
use std::ops::{Index, Not};
use std::sync::Arc;
use tokio::sync::Mutex;

pub type DefaultIx = u16;

#[derive(Debug, Copy, Clone, Default, PartialEq, PartialOrd, Eq, Ord, Hash)]
pub struct Address<Ix = DefaultIx>(Ix);

impl<Ix> Address<Ix>
where
    Ix: Copy,
{
    pub fn new(address: Ix) -> Self {
        Address(address)
    }

    #[inline]
    pub fn address(&self) -> Ix {
        self.0
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Speed<Spd = u8> {
    /// Performs a normal stop. Trains may stop smoothly.
    Stop,
    /// Performs an immediate stop action. Trains do stop immediately.
    EmergencyStop,
    /// Sets the slots speed to a given value.
    Drive(Spd),
}

impl<Spd> PartialOrd<Self> for Speed<Spd>
where
    Spd: Ord,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<Spd> Ord for Speed<Spd>
where
    Spd: Ord,
{
    fn cmp(&self, other: &Self) -> Ordering {
        if *self == *other {
            return Ordering::Equal;
        }

        match self {
            Speed::Stop => {
                if *other == Speed::EmergencyStop {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            }
            Speed::EmergencyStop => Ordering::Less,
            Speed::Drive(spd) => {
                if let Speed::Drive(other) = other {
                    spd.cmp(other)
                } else {
                    Ordering::Greater
                }
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum Node {
    Signal(Address, Position),
    Sensor(Address, Position),
    Switch(
        Address,
        Position,
        SwitchType,
        Option<NodeIndex>,
        petgraph::Direction,
    ),
    Station(Address, Position),
    Cross(Address),
    Buffer(Position),
}

impl Node {
    pub fn position(&self, rail: &Railroad) -> Position {
        match self {
            Node::Signal(_, position) => *position,
            Node::Sensor(_, position) => *position,
            Node::Switch(_, position, ..) => *position,
            Node::Station(_, position) => *position,
            Node::Cross(adr) => {
                if let Some(cross) = rail.get_crossing_mutex(adr) {
                    cross.blocking_lock().pos
                } else {
                    Position::new(Coord(0, 0, 0), Direction::North)
                }
            }
            Node::Buffer(position) => *position,
        }
    }
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
    Up = 8,
    Down = 9,
}

impl Not for Direction {
    type Output = Direction;

    fn not(self) -> Self::Output {
        match self {
            Direction::North => Direction::South,
            Direction::Northeast => Direction::Southwest,
            Direction::East => Direction::West,
            Direction::Southeast => Direction::Northwest,
            Direction::South => Direction::North,
            Direction::Southwest => Direction::Northeast,
            Direction::West => Direction::East,
            Direction::Northwest => Direction::Southeast,
            Direction::Up => Direction::Down,
            Direction::Down => Direction::Up,
        }
    }
}

impl From<u8> for Direction {
    fn from(dir_num: u8) -> Direction {
        match dir_num % 10 {
            0 => Direction::North,
            1 => Direction::Northeast,
            2 => Direction::East,
            3 => Direction::Southeast,
            4 => Direction::South,
            5 => Direction::Southwest,
            6 => Direction::West,
            7 => Direction::Northwest,
            8 => Direction::Up,
            9 => Direction::Down,
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
        match ((self as u8) + rotation) % 8 {
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

/// Represents a location in a space
/// 0 = x-Position
/// 1 = y-Position
/// 2 = z-Position
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Coord(pub usize, pub usize, pub usize);

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

    /// Checks if the given `coord` is positioned in the `dir`ection of this coord
    ///
    /// ```
    /// use locologic::control::rail_system::components::{Coord, Direction};
    ///
    /// assert!(Coord(0,0,0).is_on_line(&Coord(0,3,0), Direction::East),
    ///     "Position lays in the east");
    /// assert!(!Coord(0,0,0).is_on_line(&Coord(1,0,0), Direction::East),
    ///     "Position should not lay in the east");
    /// assert!(Coord(0,0,0).is_on_line(&Coord(5,5,0), Direction::Northeast),
    ///     "Position lays in the northeast");
    /// assert!(!Coord(0,0,0).is_on_line(&Coord(5,4,0), Direction::Northeast),
    ///     "Position should not be found in the northeast");
    /// assert!(!Coord(4,4,0).is_on_line(&Coord(0,0,0), Direction::Northeast),
    ///     "Position is in the southwest, not northeast");
    /// ```
    pub fn is_on_line(&self, coord: &Coord, dir: Direction) -> bool {
        fn diff_equal((x1, x2): (usize, usize), (y1, y2): (usize, usize)) -> bool {
            x1.abs_diff(x2) == y1.abs_diff(y2)
        }

        fn check_diagonal(coord1: &Coord, coord2: &Coord) -> bool {
            coord1.z() == coord2.z()
                && diff_equal((coord1.x(), coord2.x()), (coord1.y(), coord2.y()))
        }

        match dir {
            Direction::North => {
                self.y() == coord.y() && self.z() == coord.z() && self.x() <= coord.x()
            }
            Direction::Northeast => {
                check_diagonal(self, coord) && self.x() <= coord.x() && self.y() <= coord.y()
            }
            Direction::East => {
                self.x() == coord.x() && self.z() == coord.z() && self.y() <= coord.y()
            }
            Direction::Southeast => {
                check_diagonal(self, coord) && self.x() >= coord.x() && self.y() <= coord.y()
            }
            Direction::South => {
                self.y() == coord.y() && self.z() == coord.z() && self.x() >= coord.x()
            }
            Direction::Southwest => {
                check_diagonal(self, coord) && self.x() >= coord.x() && self.y() >= coord.y()
            }
            Direction::West => {
                self.x() == coord.x() && self.z() == coord.z() && self.y() >= coord.y()
            }
            Direction::Northwest => {
                check_diagonal(self, coord) && self.x() <= coord.x() && self.y() >= coord.y()
            }
            Direction::Up => {
                self.x() == coord.x() && self.y() == coord.y() && self.z() <= coord.z()
            }
            Direction::Down => {
                self.x() == coord.x() && self.y() == coord.y() && self.z() >= coord.z()
            }
        }
    }

    /// If the given `coord` `is on line` with this coord in the given `dir`ection,
    /// the distance between this coords position and the given `coord` is returned.
    pub fn distance(&self, coord: &Coord, dir: Direction) -> Option<usize> {
        if !self.is_on_line(coord, dir) {
            return None;
        }

        Some(match dir {
            Direction::North => coord.x() - self.x(),
            Direction::Northeast => coord.x() - self.x(),
            Direction::East => coord.y() - self.y(),
            Direction::Southeast => self.x() - coord.x(),
            Direction::South => self.x() - coord.x(),
            Direction::Southwest => self.x() - coord.x(),
            Direction::West => self.y() - coord.y(),
            Direction::Northwest => self.x() - coord.x(),
            Direction::Up => coord.z() - self.z(),
            Direction::Down => self.z() - coord.z(),
        })
    }

    /// Returns a coordinate amount step ahead in the given `dir`ection.
    /// If the direction is out of the coordinate system it returns None
    pub fn step(&self, dir: Direction, amount: usize) -> Option<Coord> {
        Some(match dir {
            Direction::North => Coord(self.x().checked_sub(amount)?, self.y(), self.z()),
            Direction::Northeast => Coord(
                self.x().checked_sub(amount)?,
                self.y().checked_add(amount)?,
                self.z(),
            ),
            Direction::East => Coord(self.x(), self.y().checked_add(amount)?, self.z()),
            Direction::Southeast => Coord(
                self.x().checked_add(amount)?,
                self.y().checked_add(amount)?,
                self.z(),
            ),
            Direction::South => Coord(self.x().checked_add(amount)?, self.y(), self.z()),
            Direction::Southwest => Coord(
                self.x().checked_add(amount)?,
                self.y().checked_sub(amount)?,
                self.z(),
            ),
            Direction::West => Coord(self.x(), self.y().checked_sub(amount)?, self.z()),
            Direction::Northwest => Coord(
                self.x().checked_sub(amount)?,
                self.y().checked_sub(amount)?,
                self.z(),
            ),
            Direction::Up => Coord(self.x(), self.y(), self.z().checked_add(amount)?),
            Direction::Down => Coord(self.x(), self.y(), self.z().checked_sub(amount)?),
        })
    }

    pub fn abs_distance(&self, coord: &Coord) -> f64 {
        f64::sqrt(
            ((self.x() + coord.x()).pow(2)
                + (self.y() + coord.y()).pow(2)
                + (self.z() + coord.z()).pow(2)) as f64,
        )
    }

    pub fn manhattan_distance(&self, coord: &Coord) -> usize {
        self.0.abs_diff(coord.0) + self.1.abs_diff(coord.1) + self.2.abs_diff(coord.2)
    }

    pub fn fold(&self) -> usize {
        self.0 + self.1 + self.2
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

    pub fn step(&self, amount: usize) -> Option<Position> {
        Some(Position {
            coord: self.coord.step(self.dir, amount)?,
            dir: self.dir,
        })
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct Rail {
    length: usize,
    pos: Position,
    start_dir: Direction,
}

impl Rail {
    /// If possible, creates a Rail from `start` to exclusive `end`.
    ///
    /// # Parameters
    ///
    /// - `start`: The position for the rail to begin.
    /// - `end`: The last position of the rail. (exclusive)
    /// - `in_dir`: The incoming direction the last rail part is placed.
    ///
    /// # Usage
    ///
    /// ```
    /// use locologic::control::rail_system::components::{Coord, Direction, Position, Rail};
    ///
    /// let from = Position::new(Coord(0, 0, 0), Direction::East);
    /// let to = Coord(0, 3, 0);
    /// let wrong_to = Coord(3,0,0);
    /// let in_direction = Direction::South;
    ///
    /// let right = Rail::perform_step(&from, &to, in_direction);
    /// let wrong = Rail::perform_step(&from, &wrong_to, in_direction);
    ///
    /// let check_value_first = Rail::new(from, 2, Direction::South);
    ///
    /// assert_eq!(right, Some(check_value_first));
    /// assert_eq!(wrong, None);
    /// ```
    pub fn perform_step(start: &Position, end: &Coord, in_dir: Direction) -> Option<Rail> {
        if start.coord == *end {
            return None;
        }
        Some(Rail {
            length: start.coord().distance(end, start.dir)? - 1,
            pos: *start,
            start_dir: in_dir,
        })
    }

    /// Returns a row of rails, stepping over the given positions,
    /// if each two following positions are in line to each other,
    /// with the direction of the first position pointing to the next one.
    /// The in_dir would be the initial direction of the first `Rail`
    ///
    /// # Usage
    ///
    /// ```
    /// use locologic::control::rail_system::components::{Coord, Direction, Position, Rail};
    ///
    /// let connection = Rail::new_connection_vec(&vec![
    ///         Position::new(Coord(0, 0, 0), Direction::East),
    ///         Position::new(Coord(0, 4, 0), Direction::North),
    ///         Position::new(Coord(1, 4, 0), Direction::Northeast),
    ///         Position::new(Coord(4, 7, 0), Direction::East)
    ///     ], Direction::North);
    /// let expected_vec = vec![
    ///     Rail::new(Position::new(Coord(0, 0, 0), Direction::East), 3, Direction::North),
    ///     Rail::new(Position::new(Coord(0, 4, 0), Direction::North), 0, Direction::West),
    ///     Rail::new(Position::new(Coord(1, 4, 0), Direction::Northeast), 2, Direction::South),
    /// ];
    ///
    /// assert_eq!(connection, Some(expected_vec));
    /// ```
    pub fn new_connection_vec(steps: &[Position], mut in_dir: Direction) -> Option<Vec<Rail>> {
        let mut connection_rail = vec![];
        let mut previous = *steps.first()?;

        for step in steps.iter().skip(1) {
            connection_rail.push(Rail::perform_step(&previous, &step.coord, in_dir)?);
            in_dir = !previous.dir;
            previous = *step;
        }

        Some(connection_rail)
    }

    /// Creates a row of rails by the instruction of the amount of steps to do and the direction where to go.
    /// Note, that if you say 'go x in direction y' it actually goes 'x + 1' positions in that direction.
    /// This is because the next rail is placed one ahead of the rail placed.
    /// This enforces that two following rails cannot start in one position.
    ///
    /// # Usage
    ///
    /// ```
    /// use locologic::control::rail_system::components::{Coord, Direction, Position, Rail};
    /// let start_pos = Coord(0,0,0);
    /// let steps = [(0, Direction::East), (3, Direction::Southeast), (2, Direction::North)];
    /// let in_dir = Direction::North;
    ///
    /// let calculated = Rail::connection_by_length(&steps, in_dir, start_pos);
    /// let expected = Some(vec![
    ///     Rail::new(Position::new(Coord(0,0,0), Direction::East), 0, Direction::North),
    ///     Rail::new(Position::new(Coord(0,1,0), Direction::Southeast), 3, Direction::West),
    ///     Rail::new(Position::new(Coord(4,5,0), Direction::North), 2, Direction::Northwest)
    /// ]);
    ///
    /// assert_eq!(calculated, expected);
    /// ```
    pub fn connection_by_length(
        steps: &[(usize, Direction)],
        mut in_dir: Direction,
        mut start_pos: Coord,
    ) -> Option<Vec<Rail>> {
        let mut connection_rail = vec![];

        for (step, dir) in steps.iter() {
            connection_rail.push(Rail {
                length: *step,
                pos: Position::new(start_pos, *dir),
                start_dir: in_dir,
            });
            start_pos = start_pos.step(*dir, *step + 1)?;
            in_dir = !*dir;
        }

        Some(connection_rail)
    }

    pub fn new(from: Position, length: usize, start_dir: Direction) -> Self {
        Rail {
            length,
            pos: from,
            start_dir,
        }
    }

    pub fn length(&self) -> usize {
        self.length
    }

    pub fn sqrt_distance(&self) -> f64 {
        match self.pos.dir {
            Direction::North
            | Direction::East
            | Direction::South
            | Direction::West
            | Direction::Up
            | Direction::Down => (self.length + 1) as f64,
            Direction::Northeast
            | Direction::Southeast
            | Direction::Southwest
            | Direction::Northwest => (((self.length + 1).pow(2) * 2) as f64).sqrt(),
        }
    }

    pub fn manhattan_distance(&self) -> usize {
        match self.pos.dir {
            Direction::North
            | Direction::East
            | Direction::South
            | Direction::West
            | Direction::Up
            | Direction::Down => self.length + 1,
            Direction::Northeast
            | Direction::Southeast
            | Direction::Southwest
            | Direction::Northwest => (self.length + 1) * 2,
        }
    }

    pub fn pos(&self) -> Position {
        self.pos
    }

    pub fn start_dir(&self) -> Direction {
        self.start_dir
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
pub enum SwDir {
    Straight,
    Curved,
}

impl Not for SwDir {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            SwDir::Straight => SwDir::Curved,
            SwDir::Curved => SwDir::Straight,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Switch {
    address: Address,
    dir: SwDir,
    updated: bool,
}

impl Switch {
    pub fn new(address: Address) -> Self {
        Switch {
            address,
            dir: SwDir::Straight,
            updated: false,
        }
    }

    pub fn address(&self) -> Address {
        self.address
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Cross {
    address: Address,
    nodes: (NodeIndex, NodeIndex),
    pos: Position,
}

impl Cross {
    pub fn new(address: Address, pos: Position, nodes: (NodeIndex, NodeIndex)) -> Self {
        Cross {
            address,
            nodes,
            pos,
        }
    }

    pub fn address(&self) -> Address {
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

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum SLevel {
    Occupied,
    Free,
}

impl Not for SLevel {
    type Output = Self;

    fn not(self) -> Self::Output {
        match self {
            SLevel::Occupied => SLevel::Free,
            SLevel::Free => SLevel::Occupied,
        }
    }
}

#[derive(Debug, Clone, Hash)]
pub struct Sensor {
    address: Address,
    status: Status,
    level: SLevel,
    train: Option<Address>,
    max_speed: Speed,
}

impl Sensor {
    pub fn new(adr: Address, max_speed: Speed) -> Sensor {
        Sensor {
            address: adr,
            status: Status::Free,
            level: SLevel::Free,
            train: None,
            max_speed,
        }
    }

    pub fn address(&self) -> Address {
        self.address
    }

    pub fn max_speed(&self) -> Speed {
        self.max_speed
    }

    pub fn train(&self) -> &Option<Address> {
        &self.train
    }

    pub fn block(&mut self, train: Address) -> bool {
        if let Some(t) = self.train {
            t == train
        } else {
            self.train = Some(train);
            self.status = Status::Reserved | self.status;
            true
        }
    }

    pub fn free(&mut self, train: Address) {
        if let Some(t) = self.train {
            if t == train {
                self.train = None;
                self.status = if self.level == SLevel::Free {
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

#[derive(Debug, Clone, Copy)]
pub enum SignalType {
    Block,
    Path,
    IntelligentPath,
}

#[derive(Debug, Clone)]
pub struct Signal {
    address: Address,
    representing_node: NodeIndex,
    sig_type: SignalType,
    status: Status,
    trains: Vec<Address>,
    requesters: VecDeque<Address>,
    other_input_signals: Vec<Address>,
    block_sensors: Vec<Address>,
    calculation_group: Arc<Mutex<Address>>,
}

impl Signal {
    pub fn new(address: Address, sig_type: SignalType, representing_node: NodeIndex) -> Self {
        Signal {
            address,
            representing_node,
            sig_type,
            status: Status::Free,
            trains: vec![],
            requesters: VecDeque::new(),
            other_input_signals: vec![],
            block_sensors: vec![],
            calculation_group: Arc::new(Mutex::new(address)),
        }
    }

    pub fn representing_node(&self) -> NodeIndex {
        self.representing_node
    }

    async fn reset_group(&mut self, signal: &Address, railroad: &Railroad) {
        self.calculation_group = railroad
            .get_signal_mutex(signal)
            .unwrap()
            .lock()
            .await
            .calculation_group
            .clone();
    }

    pub async fn initialize(&mut self, railroad: &Railroad) {
        let (signals, sensors) = Signal::search_block(&self.representing_node, railroad).await;
        self.block_sensors = sensors;

        if let Some(min_sig) = signals.iter().min() {
            if *min_sig < self.address {
                self.reset_group(min_sig, railroad).await;
            }
        }

        self.other_input_signals = signals;
    }

    pub async fn request_block(&mut self, train: Address) {
        self.requesters.push_back(train);
    }

    async fn next(&mut self, railroad: &Railroad) {
        if self.requesters.is_empty() {
            return;
        }

        let cloned = self.calculation_group.clone();
        let _calculate = cloned.lock().await;
        if let Some(free_road) = self.drive(railroad).await {
            if !self.requesters.is_empty() {
                return;
            }
            let train = self.requesters.pop_front().unwrap();
            self.status = Status::Reserved;
            self.trains.push(train);

            for adr in free_road {
                if let Some(mutex) = railroad.get_sensor_mutex(&adr) {
                    let mut sensor = mutex.lock().await;
                    sensor.block(self.address());
                }
            }
        }
    }

    pub fn address(&self) -> Address {
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
