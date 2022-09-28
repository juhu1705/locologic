use crate::control::train::Train;

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
}

pub struct Sensor<'t> {
    pub(crate) trains: Option<Vec<&'t Train>>,
    pos: Position,
}

pub enum Signal {
    Block { pos: Position },
    Path { pos: Position },
}

pub struct Switch {}

pub struct DecorationComponents {}
