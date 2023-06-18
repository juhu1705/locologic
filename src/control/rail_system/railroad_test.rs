use crate::control::rail_system::components::{
    Address, Coord, Direction, Position, Rail, SignalType, Speed, SwitchType,
};
use crate::control::rail_system::railroad::{Builder, Railroad};
use petgraph::graph::NodeIndex;
use std::collections::HashMap;
use std::sync::Arc;

pub async fn create_test_railroad() -> (
    Railroad,
    Vec<(NodeIndex, Address)>,
    Vec<((NodeIndex, NodeIndex), Address)>,
    Vec<(NodeIndex, Address)>,
    Vec<((NodeIndex, NodeIndex), Address)>,
    HashMap<Address, NodeIndex>,
) {
    let mut builder = Builder::new();

    let sensors = {
        let mut sensors = builder.add_sensors(&[
            (
                Address::new(0),
                Speed::Drive(128),
                Position::new(Coord(6, 9, 0), Direction::South),
            ),
            (
                Address::new(1),
                Speed::Drive(128),
                Position::new(Coord(2, 9, 0), Direction::West),
            ),
        ]);

        sensors.append(&mut builder.add_stations(&[
            (
                Address::new(8),
                Speed::Drive(128),
                Position::new(Coord(5, 13, 0), Direction::North),
            ),
            (
                Address::new(9),
                Speed::Drive(128),
                Position::new(Coord(5, 14, 0), Direction::North),
            ),
            (
                Address::new(10),
                Speed::Drive(128),
                Position::new(Coord(5, 15, 0), Direction::North),
            ),
            (
                Address::new(11),
                Speed::Drive(128),
                Position::new(Coord(5, 18, 0), Direction::North),
            ),
        ]));

        sensors.append(&mut builder.add_sensors(&[
            (
                Address::new(12),
                Speed::Drive(128),
                Position::new(Coord(5, 16, 0), Direction::North),
            ),
            (
                Address::new(19),
                Speed::Drive(128),
                Position::new(Coord(2, 11, 0), Direction::West),
            ),
        ]));

        sensors
    };

    let bidirectional_sensors = {
        let mut sensors = builder.add_bidirectional_sensors(&[
            (
                Address::new(2),
                Speed::Drive(128),
                Position::new(Coord(8, 8, 1), Direction::West),
            ),
            (
                Address::new(3),
                Speed::Drive(128),
                Position::new(Coord(10, 2, 1), Direction::West),
            ),
            (
                Address::new(5),
                Speed::Drive(128),
                Position::new(Coord(1, 4, 1), Direction::West),
            ),
            (
                Address::new(6),
                Speed::Drive(128),
                Position::new(Coord(5, 18, 1), Direction::North),
            ),
            (
                Address::new(13),
                Speed::Drive(128),
                Position::new(Coord(5, 6, 1), Direction::Southwest),
            ),
            (
                Address::new(14),
                Speed::Drive(128),
                Position::new(Coord(10, 4, 1), Direction::West),
            ),
            (
                Address::new(14),
                Speed::Drive(128),
                Position::new(Coord(10, 12, 1), Direction::West),
            ),
            (
                Address::new(15),
                Speed::Drive(128),
                Position::new(Coord(7, 3, 1), Direction::West),
            ),
            (
                Address::new(16),
                Speed::Drive(128),
                Position::new(Coord(7, 9, 2), Direction::West),
            ),
            (
                Address::new(18),
                Speed::Drive(128),
                Position::new(Coord(7, 15, 2), Direction::West),
            ),
        ]);
        sensors.insert(
            2,
            (
                builder.add_bidirectional_station(
                    Address::new(4),
                    Speed::Drive(128),
                    Position::new(Coord(8, 3, 1), Direction::West),
                ),
                Address::new(4),
            ),
        );
        sensors.insert(
            5,
            (
                builder.add_bidirectional_station(
                    Address::new(7),
                    Speed::Drive(128),
                    Position::new(Coord(5, 16, 1), Direction::North),
                ),
                Address::new(7),
            ),
        );
        sensors.insert(
            11,
            (
                builder.add_bidirectional_station(
                    Address::new(17),
                    Speed::Drive(128),
                    Position::new(Coord(7, 12, 2), Direction::West),
                ),
                Address::new(17),
            ),
        );

        sensors
    };

    let switches = builder.add_switches(&[
        (
            Address::new(1),
            Position::new(Coord(2, 12, 0), Direction::East),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(2),
            Position::new(Coord(2, 13, 0), Direction::East),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(3),
            Position::new(Coord(2, 15, 0), Direction::East),
            SwitchType::StraightRight180,
        ),
        (
            Address::new(4),
            Position::new(Coord(2, 16, 0), Direction::East),
            SwitchType::StraightRight180,
        ),
        (
            Address::new(5),
            Position::new(Coord(8, 16, 0), Direction::Northeast),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(6),
            Position::new(Coord(9, 13, 0), Direction::East),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(7),
            Position::new(Coord(9, 12, 0), Direction::East),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(8),
            Position::new(Coord(8, 10, 0), Direction::Southeast),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(14),
            Position::new(Coord(4, 8, 0), Direction::East),
            SwitchType::StraightRight90,
        ),
    ]);

    let bidirectional_switches = builder.add_bidirectional_switches(&[
        (
            Address::new(9),
            Position::new(Coord(3, 15, 1), Direction::East),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(10),
            Position::new(Coord(7, 0, 1), Direction::South),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(11),
            Position::new(Coord(2, 6, 0), Direction::East),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(12),
            Position::new(Coord(2, 5, 0), Direction::West),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(13),
            Position::new(Coord(4, 6, 0), Direction::East),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(15),
            Position::new(Coord(8, 6, 1), Direction::West),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(16),
            Position::new(Coord(8, 10, 1), Direction::East),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(17),
            Position::new(Coord(8, 7, 1), Direction::West),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(17),
            Position::new(Coord(10, 5, 1), Direction::East),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(18),
            Position::new(Coord(8, 9, 1), Direction::East),
            SwitchType::StraightLeft90,
        ),
        (
            Address::new(18),
            Position::new(Coord(10, 11, 1), Direction::West),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(19),
            Position::new(Coord(1, 12, 1), Direction::East),
            SwitchType::StraightRight90,
        ),
        (
            Address::new(20),
            Position::new(Coord(3, 8, 1), Direction::East),
            SwitchType::StraightRight90,
        ),
    ]);

    let signals = builder.add_signals(&[
        (
            Address::new(80),
            SignalType::Path,
            Position::new(Coord(2, 10, 0), Direction::West),
        ),
        (
            Address::new(81),
            SignalType::Block,
            Position::new(Coord(3, 7, 0), Direction::Southeast),
        ),
        (
            Address::new(82),
            SignalType::Block,
            Position::new(Coord(4, 7, 0), Direction::East),
        ),
        (
            Address::new(83),
            SignalType::Path,
            Position::new(Coord(5, 6, 0), Direction::North),
        ),
        (
            Address::new(84),
            SignalType::Block,
            Position::new(Coord(4, 13, 0), Direction::North),
        ),
        (
            Address::new(85),
            SignalType::Block,
            Position::new(Coord(4, 14, 0), Direction::North),
        ),
        (
            Address::new(86),
            SignalType::Block,
            Position::new(Coord(4, 15, 0), Direction::North),
        ),
        (
            Address::new(87),
            SignalType::Block,
            Position::new(Coord(4, 16, 0), Direction::North),
        ),
        (
            Address::new(88),
            SignalType::Block,
            Position::new(Coord(4, 18, 0), Direction::North),
        ),
        (
            Address::new(89),
            SignalType::Block,
            Position::new(Coord(7, 12, 0), Direction::Northeast),
        ),
        (
            Address::new(90),
            SignalType::Block,
            Position::new(Coord(8, 13, 0), Direction::Northeast),
        ),
        (
            Address::new(91),
            SignalType::Block,
            Position::new(Coord(8, 14, 0), Direction::Northeast),
        ),
        (
            Address::new(92),
            SignalType::Block,
            Position::new(Coord(7, 16, 0), Direction::North),
        ),
        (
            Address::new(93),
            SignalType::Block,
            Position::new(Coord(7, 17, 0), Direction::Northeast),
        ),
        (
            Address::new(94),
            SignalType::Block,
            Position::new(Coord(8, 0, 1), Direction::South),
        ),
        (
            Address::new(95),
            SignalType::Path,
            Position::new(Coord(8, 0, 1), Direction::North),
        ),
        (
            Address::new(96),
            SignalType::Path,
            Position::new(Coord(8, 2, 1), Direction::West),
        ),
        (
            Address::new(97),
            SignalType::Block,
            Position::new(Coord(8, 2, 1), Direction::East),
        ),
        (
            Address::new(98),
            SignalType::Block,
            Position::new(Coord(10, 3, 1), Direction::West),
        ),
        (
            Address::new(99),
            SignalType::Path,
            Position::new(Coord(10, 3, 1), Direction::East),
        ),
        (
            Address::new(100),
            SignalType::Path,
            Position::new(Coord(7, 4, 1), Direction::West),
        ),
        (
            Address::new(101),
            SignalType::Block,
            Position::new(Coord(7, 4, 1), Direction::East),
        ),
        (
            Address::new(102),
            SignalType::Block,
            Position::new(Coord(8, 5, 1), Direction::West),
        ),
        (
            Address::new(103),
            SignalType::Path,
            Position::new(Coord(8, 5, 1), Direction::East),
        ),
        (
            Address::new(104),
            SignalType::Block,
            Position::new(Coord(8, 13, 1), Direction::East),
        ),
        (
            Address::new(105),
            SignalType::Path,
            Position::new(Coord(8, 13, 1), Direction::West),
        ),
        (
            Address::new(106),
            SignalType::Path,
            Position::new(Coord(10, 13, 1), Direction::West),
        ),
        (
            Address::new(107),
            SignalType::Block,
            Position::new(Coord(10, 13, 1), Direction::East),
        ),
        (
            Address::new(108),
            SignalType::Block,
            Position::new(Coord(4, 16, 1), Direction::South),
        ),
        (
            Address::new(109),
            SignalType::Path,
            Position::new(Coord(4, 16, 1), Direction::North),
        ),
        (
            Address::new(110),
            SignalType::Block,
            Position::new(Coord(4, 18, 1), Direction::South),
        ),
        (
            Address::new(111),
            SignalType::Path,
            Position::new(Coord(4, 18, 1), Direction::North),
        ),
        (
            Address::new(112),
            SignalType::Block,
            Position::new(Coord(1, 14, 1), Direction::West),
        ),
        (
            Address::new(113),
            SignalType::Path,
            Position::new(Coord(1, 14, 1), Direction::East),
        ),
        (
            Address::new(114),
            SignalType::Block,
            Position::new(Coord(4, 3, 2), Direction::South),
        ),
        (
            Address::new(115),
            SignalType::Path,
            Position::new(Coord(4, 3, 2), Direction::North),
        ),
        (
            Address::new(116),
            SignalType::Path,
            Position::new(Coord(2, 8, 0), Direction::West),
        ),
    ]);

    /*
    Sensors:
    | Index  | Adresse |
    |--------|---------|
    | 0      | 0       |
    | 1      | 1       |
    | 2      | 8       |
    | 3      | 9       |
    | 4      | 10      |
    | 5      | 11      |
    | 6      | 12      |
    | 7      | 19      |

    Bidirectional Sensors:
    | Index | Address |
    |-------|---------|
    | 0     | 2       |
    | 1     | 3       |
    | 2     | 4       |
    | 3     | 5       |
    | 4     | 6       |
    | 5     | 7       |
    | 6     | 13      |
    | 7     | 14 (L)  |
    | 8     | 14 (R)  |
    | 9     | 15      |
    | 10    | 16      |
    | 11    | 17      |
    | 12    | 18      |

    Switches:
    | Index | Switch |
    |-------|--------|
    | 0     | 1      |
    | 1     | 2      |
    | 2     | 3      |
    | 3     | 4      |
    | 4     | 5      |
    | 5     | 6      |
    | 6     | 7      |
    | 7     | 8      |
    | 8     | 14     |

    Bidirectional Switches:
    | Index | Switch |
    |-------|--------|
    | 0     | 9      |
    | 1     | 10     |
    | 2     | 11     |
    | 3     | 12     |
    | 4     | 13     |
    | 5     | 15     |
    | 6     | 16     |
    | 7     | 17 (I) |
    | 8     | 17 (A) |
    | 9     | 18 (I) |
    | 10    | 18 (A) |
    | 11    | 19     |
    | 12    | 20     |

    Signals:
    x -> x + 80 for x in (0..35)
    */

    // Schattenbahnhof

    // Schattenbahnhof Umfahrgleis
    builder.connect(
        bidirectional_switches[3].0 .0,
        signals[&Address::new(83)],
        Rail::connection_by_length(
            &[
                (1, Direction::West),
                (0, Direction::Southwest),
                (4, Direction::South),
                (0, Direction::Southeast),
                (1, Direction::East),
                (1, Direction::Northeast),
                (0, Direction::North),
            ],
            Direction::Southeast,
            Coord(1, 4, 0),
        )
        .unwrap(),
    );
    builder.connect(
        signals[&Address::new(83)],
        bidirectional_switches[4].0 .1,
        vec![],
    );
    builder.connect(
        bidirectional_switches[4].0 .0,
        signals[&Address::new(82)],
        vec![],
    );
    builder.connect_bidirectional(
        bidirectional_switches[2].0,
        bidirectional_switches[3].0,
        vec![],
    );
    builder.connect(
        bidirectional_switches[2].0 .1,
        signals[&Address::new(81)],
        vec![],
    );
    builder.connect(signals[&Address::new(82)], switches[8].0, vec![]);
    builder.connect(signals[&Address::new(81)], switches[8].0, vec![]);
    builder.connect(
        switches[8].0,
        sensors[0].0,
        vec![Rail::new(
            Position::new(Coord(5, 9, 0), Direction::South),
            0,
            Direction::Northwest,
        )],
    );
    builder.connect(
        sensors[0].0,
        switches[7].0,
        vec![Rail::new(
            Position::new(Coord(7, 9, 0), Direction::Southeast),
            0,
            Direction::North,
        )],
    );
    builder.connect(
        switches[7].0,
        switches[6].0,
        vec![Rail::new(
            Position::new(Coord(9, 11, 0), Direction::East),
            0,
            Direction::Northwest,
        )],
    );
    builder.connect(switches[6].0, switches[5].0, vec![]);
    builder.connect(
        switches[5].0,
        switches[4].0,
        vec![
            Rail::new(
                Position::new(Coord(9, 14, 0), Direction::East),
                0,
                Direction::West,
            ),
            Rail::new(
                Position::new(Coord(9, 15, 0), Direction::Northeast),
                0,
                Direction::West,
            ),
        ],
    );
    builder.connect(switches[4].0, signals[&Address::new(92)], vec![]);
    builder.connect(
        signals[&Address::new(92)],
        sensors[6].0,
        vec![Rail::new(
            Position::new(Coord(9, 16, 0), Direction::North),
            0,
            Direction::South,
        )],
    );
    builder.connect(sensors[6].0, signals[&Address::new(87)], vec![]);
    builder.connect(
        signals[&Address::new(87)],
        switches[3].0,
        vec![Rail::new(
            Position::new(Coord(3, 16, 0), Direction::North),
            0,
            Direction::South,
        )],
    );
    builder.connect(switches[3].0, switches[2].0, vec![]);
    builder.connect(
        switches[2].0,
        switches[1].0,
        vec![Rail::new(
            Position::new(Coord(2, 14, 0), Direction::West),
            0,
            Direction::East,
        )],
    );
    builder.connect(switches[1].0, switches[0].0, vec![]);
    builder.connect(switches[0].0, sensors[7].0, vec![]);
    builder.connect(sensors[7].0, signals[&Address::new(80)], vec![]);
    builder.connect(signals[&Address::new(80)], sensors[1].0, vec![]);
    builder.connect(sensors[1].0, signals[&Address::new(116)], vec![]);
    builder.connect(
        signals[&Address::new(116)],
        bidirectional_switches[2].0 .0,
        vec![Rail::new(
            Position::new(Coord(2, 7, 0), Direction::West),
            0,
            Direction::East,
        )],
    );

    // Schattenbahnhof Gleis 1
    builder.connect(
        switches[7].0,
        signals[&Address::new(89)],
        vec![Rail::new(
            Position::new(Coord(8, 11, 0), Direction::Northeast),
            0,
            Direction::West,
        )],
    );
    builder.connect(
        signals[&Address::new(89)],
        sensors[2].0,
        vec![Rail::new(
            Position::new(Coord(6, 13, 0), Direction::North),
            0,
            Direction::Southwest,
        )],
    );
    builder.connect(sensors[2].0, signals[&Address::new(84)], vec![]);
    builder.connect(
        signals[&Address::new(84)],
        switches[0].0,
        vec![Rail::new(
            Position::new(Coord(3, 13, 0), Direction::Northwest),
            0,
            Direction::South,
        )],
    );

    // Schattenbahnhof Gleis 2
    builder.connect(switches[6].0, signals[&Address::new(90)], vec![]);
    builder.connect(
        signals[&Address::new(90)],
        sensors[3].0,
        vec![Rail::new(
            Position::new(Coord(7, 14, 0), Direction::North),
            1,
            Direction::Southwest,
        )],
    );
    builder.connect(sensors[3].0, signals[&Address::new(85)], vec![]);
    builder.connect(
        signals[&Address::new(85)],
        switches[1].0,
        vec![Rail::new(
            Position::new(Coord(3, 14, 0), Direction::Northwest),
            0,
            Direction::South,
        )],
    );

    // Schattenbahnhof Gleis 3
    builder.connect(switches[5].0, signals[&Address::new(91)], vec![]);
    builder.connect(
        signals[&Address::new(91)],
        sensors[4].0,
        vec![Rail::new(
            Position::new(Coord(7, 15, 0), Direction::North),
            1,
            Direction::Southwest,
        )],
    );
    builder.connect(sensors[4].0, signals[&Address::new(86)], vec![]);
    builder.connect(
        signals[&Address::new(86)],
        switches[2].0,
        vec![Rail::new(
            Position::new(Coord(3, 15, 0), Direction::North),
            0,
            Direction::South,
        )],
    );

    // Schattenbahnhof Gleis 4
    builder.connect(switches[4].0, signals[&Address::new(93)], vec![]);
    builder.connect(
        signals[&Address::new(93)],
        sensors[5].0,
        vec![Rail::new(
            Position::new(Coord(6, 18, 0), Direction::North),
            0,
            Direction::Southwest,
        )],
    );
    builder.connect(sensors[5].0, signals[&Address::new(88)], vec![]);
    builder.connect(
        signals[&Address::new(88)],
        switches[3].0,
        vec![
            Rail::new(
                Position::new(Coord(3, 18, 0), Direction::Northwest),
                0,
                Direction::South,
            ),
            Rail::new(
                Position::new(Coord(2, 17, 0), Direction::West),
                0,
                Direction::Southeast,
            ),
        ],
    );

    // Fahrebene
    builder.connect_bidirectional(
        bidirectional_switches[3].0,
        bidirectional_sensors[9].0,
        Rail::connection_by_length(
            &[
                (0, Direction::West),
                (0, Direction::Southwest),
                (1, Direction::South),
                (0, Direction::Up),
                (1, Direction::South),
                (0, Direction::East),
            ],
            Direction::East,
            Coord(2, 4, 0),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(
        bidirectional_sensors[9].0,
        (signals[&Address::new(100)], signals[&Address::new(101)]),
        vec![],
    );
    builder.connect_bidirectional(
        (signals[&Address::new(100)], signals[&Address::new(101)]),
        bidirectional_switches[5].0,
        vec![Rail::new(
            Position::new(Coord(7, 5, 1), Direction::Southeast),
            0,
            Direction::West,
        )],
    );
    builder.connect_bidirectional(
        bidirectional_sensors[6].0,
        bidirectional_switches[4].0,
        Rail::connection_by_length(
            &[
                (0, Direction::West),
                (0, Direction::South),
                (0, Direction::Up),
                (0, Direction::South),
                (0, Direction::East),
                (0, Direction::Northeast),
            ],
            Direction::East,
            Coord(4, 5, 0),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(
        bidirectional_switches[12].0,
        bidirectional_sensors[6].0,
        vec![Rail::new(
            Position::new(Coord(4, 7, 1), Direction::Northeast),
            0,
            Direction::Southwest,
        )],
    );
    builder.connect_bidirectional(
        bidirectional_switches[6].0,
        bidirectional_switches[12].0,
        Rail::connection_by_length(
            &[
                (2, Direction::East),
                (0, Direction::Southeast),
                (1, Direction::South),
                (0, Direction::Southwest),
                (0, Direction::West),
                (0, Direction::Southwest),
            ],
            Direction::West,
            Coord(3, 9, 1),
        )
        .unwrap(),
    );

    // Weichenfeld
    builder.connect_bidirectional(
        bidirectional_switches[5].0,
        bidirectional_switches[7].0,
        vec![],
    );
    builder.connect_bidirectional(
        bidirectional_switches[7].0,
        bidirectional_sensors[0].0,
        vec![],
    );
    builder.connect_bidirectional(
        bidirectional_sensors[0].0,
        bidirectional_switches[9].0,
        vec![],
    );
    builder.connect_bidirectional(
        bidirectional_switches[9].0,
        bidirectional_switches[6].0,
        vec![],
    );
    builder.connect_bidirectional(
        bidirectional_switches[8].0,
        bidirectional_switches[7].0,
        vec![Rail::new(
            Position::new(Coord(9, 6, 1), Direction::Northeast),
            0,
            Direction::Southwest,
        )],
    );
    builder.connect_bidirectional(
        bidirectional_switches[9].0,
        bidirectional_switches[10].0,
        vec![Rail::new(
            Position::new(Coord(9, 10, 1), Direction::Southeast),
            0,
            Direction::Northwest,
        )],
    );
    builder.connect_bidirectional(
        bidirectional_switches[8].0,
        bidirectional_switches[10].0,
        vec![Rail::new(
            Position::new(Coord(10, 6, 1), Direction::East),
            4,
            Direction::West,
        )],
    );

    let signals_block_w4 = (signals[&Address::new(103)], signals[&Address::new(102)]);
    let signals_block_w3 = (signals[&Address::new(99)], signals[&Address::new(98)]);
    let signals_block_a4 = (signals[&Address::new(97)], signals[&Address::new(96)]);
    let signals_block_a3 = (signals[&Address::new(94)], signals[&Address::new(95)]);
    let signals_block_w7 = (signals[&Address::new(105)], signals[&Address::new(104)]);
    let signals_block_w6 = (signals[&Address::new(107)], signals[&Address::new(106)]);
    let signals_block_a7 = (signals[&Address::new(109)], signals[&Address::new(108)]);
    let signals_block_a6 = (signals[&Address::new(111)], signals[&Address::new(110)]);
    let signals_block_w17 = (signals[&Address::new(114)], signals[&Address::new(115)]);
    let signals_block_a17 = (signals[&Address::new(113)], signals[&Address::new(112)]);

    builder.connect_bidirectional(bidirectional_switches[1].0, signals_block_a3, vec![]);
    builder.connect_bidirectional(
        signals_block_a3,
        bidirectional_sensors[5].0,
        Rail::connection_by_length(
            &[(0, Direction::Southeast), (0, Direction::East)],
            Direction::North,
            Coord(9, 0, 1),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(bidirectional_sensors[5].0, signals_block_w3, vec![]);
    builder.connect_bidirectional(signals_block_w3, bidirectional_sensors[7].0, vec![]);
    builder.connect_bidirectional(
        bidirectional_sensors[7].0,
        bidirectional_switches[8].0,
        vec![],
    );

    builder.connect_bidirectional(
        bidirectional_switches[1].0,
        signals_block_a4,
        vec![Rail::new(
            Position::new(Coord(8, 1, 1), Direction::East),
            0,
            Direction::Northwest,
        )],
    );
    builder.connect_bidirectional(signals_block_a4, bidirectional_sensors[2].0, vec![]);
    builder.connect_bidirectional(
        bidirectional_sensors[2].0,
        signals_block_w4,
        vec![Rail::new(
            Position::new(Coord(8, 4, 1), Direction::East),
            0,
            Direction::West,
        )],
    );
    builder.connect_bidirectional(signals_block_w4, bidirectional_switches[5].0, vec![]);

    builder.connect_bidirectional(
        bidirectional_switches[6].0,
        signals_block_w7,
        vec![Rail::new(
            Position::new(Coord(8, 11, 1), Direction::East),
            1,
            Direction::West,
        )],
    );
    builder.connect_bidirectional(
        signals_block_w7,
        bidirectional_sensors[5].0,
        Rail::connection_by_length(
            &[
                (0, Direction::East),
                (0, Direction::Northeast),
                (1, Direction::North),
            ],
            Direction::West,
            Coord(8, 14, 1),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(bidirectional_sensors[5].0, signals_block_a7, vec![]);
    builder.connect_bidirectional(
        signals_block_a7,
        bidirectional_switches[0].0,
        vec![Rail::new(
            Position::new(Coord(3, 16, 1), Direction::West),
            0,
            Direction::South,
        )],
    );

    builder.connect_bidirectional(
        bidirectional_switches[10].0,
        bidirectional_sensors[8].0,
        vec![],
    );
    builder.connect_bidirectional(bidirectional_sensors[8].0, signals_block_w6, vec![]);
    builder.connect_bidirectional(
        signals_block_w6,
        bidirectional_sensors[4].0,
        Rail::connection_by_length(
            &[
                (1, Direction::East),
                (1, Direction::Northeast),
                (2, Direction::North),
            ],
            Direction::West,
            Coord(10, 15, 1),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(bidirectional_sensors[4].0, signals_block_a6, vec![]);
    builder.connect_bidirectional(
        signals_block_a6,
        bidirectional_switches[0].0,
        Rail::connection_by_length(
            &[
                (0, Direction::Northwest),
                (0, Direction::West),
                (0, Direction::Southwest),
            ],
            Direction::South,
            Coord(3, 18, 1),
        )
        .unwrap(),
    );

    builder.connect_bidirectional(
        bidirectional_switches[12].0,
        signals_block_w17,
        Rail::connection_by_length(
            &[
                (0, Direction::Up),
                (0, Direction::West),
                (0, Direction::Northwest),
                (0, Direction::West),
                (0, Direction::Southwest),
                (0, Direction::South),
            ],
            Direction::East,
            Coord(3, 7, 1),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(
        signals_block_w17,
        bidirectional_sensors[10].0,
        Rail::connection_by_length(
            &[
                (0, Direction::Southeast),
                (1, Direction::East),
                (0, Direction::Southeast),
                (1, Direction::East),
            ],
            Direction::North,
            Coord(5, 3, 2),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(
        bidirectional_sensors[10].0,
        bidirectional_sensors[11].0,
        vec![Rail::new(
            Position::new(Coord(7, 10, 2), Direction::East),
            1,
            Direction::West,
        )],
    );
    builder.connect_bidirectional(
        bidirectional_sensors[11].0,
        bidirectional_sensors[12].0,
        vec![Rail::new(
            Position::new(Coord(7, 13, 2), Direction::East),
            1,
            Direction::West,
        )],
    );
    builder.connect_bidirectional(
        bidirectional_sensors[12].0,
        signals_block_a17,
        Rail::connection_by_length(
            &[
                (0, Direction::East),
                (0, Direction::Northeast),
                (3, Direction::North),
                (0, Direction::Northwest),
                (1, Direction::West),
                (0, Direction::Down),
                (0, Direction::West),
            ],
            Direction::West,
            Coord(7, 16, 2),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(
        signals_block_a17,
        bidirectional_switches[11].0,
        vec![Rail::new(
            Position::new(Coord(1, 13, 1), Direction::West),
            0,
            Direction::East,
        )],
    );

    builder.connect_bidirectional(
        bidirectional_switches[0].0,
        bidirectional_switches[11].0,
        Rail::connection_by_length(
            &[(1, Direction::Northwest)],
            Direction::East,
            Coord(3, 14, 1),
        )
        .unwrap(),
    );
    builder.connect_bidirectional(
        bidirectional_switches[11].0,
        bidirectional_sensors[3].0,
        vec![Rail::new(
            Position::new(Coord(1, 11, 1), Direction::West),
            6,
            Direction::East,
        )],
    );
    builder.connect_bidirectional(
        bidirectional_sensors[3].0,
        bidirectional_switches[1].0,
        Rail::connection_by_length(
            &[
                (1, Direction::West),
                (0, Direction::Southwest),
                (4, Direction::South),
            ],
            Direction::East,
            Coord(1, 3, 1),
        )
        .unwrap(),
    );

    // Set switch default way
    builder.set_switch_default_dir(switches[0].0, switches[1].0);
    builder.set_switch_default_dir(switches[1].0, switches[2].0);
    builder.set_switch_default_dir(switches[2].0, switches[3].0);
    builder.set_switch_default_dir(switches[3].0, signals[&Address::new(87)]);
    builder.set_switch_default_dir(switches[4].0, signals[&Address::new(92)]);
    builder.set_switch_default_dir(switches[5].0, switches[4].0);
    builder.set_switch_default_dir(switches[6].0, switches[5].0);
    builder.set_switch_default_dir(switches[7].0, switches[6].0);
    builder.set_switch_default_dir(switches[8].0, signals[&Address::new(82)]);

    builder.set_switch_default_dir_bidirectional(bidirectional_switches[0].0, signals_block_a7);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[1].0, signals_block_a4);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[2].0, (signals[&Address::new(116)], signals[&Address::new(116)]));
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[3].0, (signals[&Address::new(83)], signals[&Address::new(83)]));
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[4].0, (signals[&Address::new(83)], signals[&Address::new(83)]));
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[5].0, signals_block_w4);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[6].0, signals_block_w7);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[7].0, bidirectional_switches[5].0);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[8].0, bidirectional_switches[10].0);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[9].0, bidirectional_switches[6].0);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[10].0, bidirectional_switches[8].0);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[11].0, signals_block_a17);
    builder.set_switch_default_dir_bidirectional(bidirectional_switches[12].0, signals_block_w17);

    (
        builder.build().await,
        switches,
        bidirectional_switches,
        sensors,
        bidirectional_sensors,
        signals,
    )
}

#[tokio::test]
pub async fn test_road() {
    let (r, switches, bi_dir_switches, sensors, bi_dir_sensors, signals) =
        create_test_railroad().await;

    let railroad = Arc::new(r);

    let calculated_road =
        Railroad::shortest_path(railroad.clone(), sensors[1].0, sensors[7].0).await;

    let expected_road = Some((
        99,
        vec![
            sensors[1].0,
            signals[&Address::new(116)],
            bi_dir_switches[2].0 .0,
            bi_dir_switches[3].0 .0,
            bi_dir_sensors[9].0 .0,
            signals[&Address::new(100)],
            bi_dir_switches[5].0 .0,
            bi_dir_switches[7].0 .0,
            bi_dir_sensors[0].0 .0,
            bi_dir_switches[9].0 .0,
            bi_dir_switches[6].0 .0,
            bi_dir_switches[12].0 .0,
            bi_dir_sensors[6].0 .0,
            bi_dir_switches[4].0 .0,
            signals[&Address::new(82)],
            switches[8].0,
            sensors[0].0,
            switches[7].0,
            switches[6].0,
            switches[5].0,
            switches[4].0,
            signals[&Address::new(92)],
            sensors[6].0,
            signals[&Address::new(87)],
            switches[3].0,
            switches[2].0,
            switches[1].0,
            switches[0].0,
            sensors[7].0,
        ],
    ));

    assert_eq!(calculated_road, expected_road);
}
