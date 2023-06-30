use crate::control::messages::Message;
use crate::control::rail_system::components::{Address, Speed, SwDir};
use locodrive::args::{AddressArg, SlotArg, SpeedArg, SwitchArg, SwitchDirection};
use locodrive::loco_controller::LocoDriveController;
use std::collections::HashMap;
use tokio::sync::broadcast::error::RecvError;
use tokio::sync::broadcast::Receiver;

pub struct LocoDriveConnector {
    receiver: Receiver<Message>,
    sender: LocoDriveController,
    loco_receiver: Receiver<locodrive::protocol::Message>,
    slots: HashMap<Address, SlotArg>,
}

impl LocoDriveConnector {
    pub fn new(
        receiver: Receiver<Message>,
        sender: LocoDriveController,
        loco_receiver: Receiver<locodrive::protocol::Message>,
    ) -> Self {
        LocoDriveConnector {
            receiver,
            sender,
            loco_receiver,
            slots: HashMap::new(),
        }
    }

    pub async fn lookup_slot(&mut self, adr: Address) -> Option<SlotArg> {
        if self.slots.contains_key(&adr) {
            return Some(*self.slots.get(&adr).unwrap());
        } else {
            let _ = self
                .sender
                .send_message(locodrive::protocol::Message::LocoAdr(AddressArg::new(
                    adr.address(),
                )))
                .await;
            loop {
                match self.loco_receiver.recv().await {
                    Ok(locodrive::protocol::Message::SlRdData(slot, _, loco_adr, ..)) => {
                        self.slots.insert(Address::new(loco_adr.address()), slot);
                        if self.slots.contains_key(&adr) {
                            return Some(*self.slots.get(&adr).unwrap());
                        }
                    }
                    Err(RecvError::Closed) => break,
                    _ => continue,
                }
            }
        }
        None
    }

    pub async fn handle_message(&mut self, message: Message) {
        if let Some(loco_net_message) =
            match message {
                Message::RailOn => Some(locodrive::protocol::Message::GpOn),
                Message::RailOff => Some(locodrive::protocol::Message::GpOff),
                Message::TrainSpeed(adr, speed) => self
                    .lookup_slot(adr)
                    .await
                    .map(|slot| locodrive::protocol::Message::LocoSpd(slot, SpeedArg::from(speed))),
                Message::Switch(adr, dir) => Some(locodrive::protocol::Message::SwReq(
                    SwitchArg::new(adr.address(), SwitchDirection::from(dir), false),
                )),
                _ => None,
            }
        {
            let _ = self.sender.send_message(loco_net_message).await;
        }
    }
}

async fn run_connector(mut connector: LocoDriveConnector) {
    loop {
        match connector.receiver.recv().await {
            Ok(msg) => {
                connector.handle_message(msg).await;
            }
            Err(RecvError::Closed) => {
                break;
            }
            Err(_) => {
                continue;
            }
        }
    }
}

pub fn run_loconet_connector(
    receiver: Receiver<Message>,
    loco_controller: LocoDriveController,
    loco_receiver: Receiver<locodrive::protocol::Message>,
) {
    let actor = LocoDriveConnector::new(receiver, loco_controller, loco_receiver);
    tokio::spawn(run_connector(actor));
}

impl From<Speed> for SpeedArg {
    fn from(speed: Speed) -> Self {
        match speed {
            Speed::EmergencyStop => SpeedArg::EmergencyStop,
            Speed::Stop => SpeedArg::Stop,
            Speed::Drive(spd) => SpeedArg::Drive(spd),
        }
    }
}

impl From<SwDir> for SwitchDirection {
    fn from(dir: SwDir) -> Self {
        match dir {
            SwDir::Straight => SwitchDirection::Straight,
            SwDir::Curved => SwitchDirection::Curved,
        }
    }
}
