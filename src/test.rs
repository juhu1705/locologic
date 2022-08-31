
#[cfg(test)]
mod loco_tests {

}

#[cfg(test)]
mod logic_tests {

}

#[cfg(test)]
mod playing {
    use std::collections::HashMap;
    use std::process::exit;
    use std::thread::sleep;
    use std::time::Duration;
    use locodrive::args::{AddressArg, SensorLevel, SlotArg, SpeedArg, SwitchArg, SwitchDirection};
    use locodrive::loco_controller::{LocoDriveController, LocoDriveMessage};
    use locodrive::protocol::Message;
    use locodrive::protocol::Message::LocoSpd;
    use tokio_serial::FlowControl;
    use crate::train::Train;

    #[tokio::test]
    async fn drive_around() {
        let (sender, mut receiver) = tokio::sync::broadcast::channel(5);


        let mut loco_controller = match LocoDriveController::new(
            "/dev/ttyUSB0",
            115_200,
            5000,
            FlowControl::Software,
            sender,
            false,
        ).await {
            Ok(loco_controller) => loco_controller,
            Err(_err) => {
                eprintln!("Error: Could not connect to the serial port!");
                return;
            }
        };

        let adr = AddressArg::new(5);

        let mut slot_adr_map: HashMap<AddressArg, Train> = HashMap::new();

        match loco_controller.send_message(Message::LocoAdr(adr)).await {
            Ok(()) => {},
            Err(err) => {
                eprintln!("Message was not send! {:?}", err);
                exit(1)
            }
        };

        while let Ok(message) = receiver.recv().await {
            match message {
                LocoDriveMessage::Message(message) => {
                    match message {
                        Message::SlRdData(slot, _, address, ..) => {
                            slot_adr_map.insert(adr, Train {
                                slot,
                                address,
                            });
                            break;
                        },
                        _ => {}
                    }
                }
                LocoDriveMessage::Answer(_, _) => {}
                LocoDriveMessage::Error(err) => {
                    eprintln!("Message could not be read! {:?}", err);
                    exit(1)
                }
                LocoDriveMessage::SerialPortError(err) => {
                    eprintln!("Connection refused! {:?}", err);
                    exit(1)
                }
            }
        }

        for i in 1..10 {
            if i % 2 == 0 {
                loco_controller.send_message(Message::SwReq(SwitchArg::new(1, SwitchDirection::Curved, true))).await.unwrap();
                loco_controller.send_message(Message::SwReq(SwitchArg::new(5, SwitchDirection::Curved, true))).await.unwrap();
            } else {
                loco_controller.send_message(Message::SwReq(SwitchArg::new(1, SwitchDirection::Curved, false))).await.unwrap();
                loco_controller.send_message(Message::SwReq(SwitchArg::new(1, SwitchDirection::Curved, false))).await.unwrap();
            }

            loco_controller.send_message(LocoSpd(slot_adr_map.get(&adr).unwrap().slot, SpeedArg::Drive(5 * i))).await.unwrap();

            let mut waiting = true;

            while let Ok(message) = receiver.recv().await {
                match message {
                    LocoDriveMessage::Message(message) => {
                        match message {
                            Message::InputRep(in_arg) => {
                                if i % 2 == 0 && in_arg.address() == 5 && in_arg.sensor_level() == SensorLevel::High {
                                    waiting = false;
                                } else if i % 2 == 1 && in_arg.address() == 4 && in_arg.sensor_level() == SensorLevel::High {
                                    waiting = false;
                                } else if !waiting && in_arg.address() == 6 && in_arg.sensor_level() == SensorLevel::Low {
                                    break;
                                }
                            },
                            _ => {}
                        }
                    }
                    LocoDriveMessage::Answer(_, _) => {}
                    LocoDriveMessage::Error(err) => {
                        eprintln!("Message could not be read! {:?}", err);
                        exit(1)
                    }
                    LocoDriveMessage::SerialPortError(err) => {
                        eprintln!("Connection refused! {:?}", err);
                        exit(1)
                    }
                }
            }

            loco_controller.send_message(LocoSpd(slot_adr_map.get(&adr).unwrap().slot, SpeedArg::Stop)).await.unwrap();

            sleep(Duration::from_secs(2));
        }

        println!("Drive 10 rounds!")
    }
}
