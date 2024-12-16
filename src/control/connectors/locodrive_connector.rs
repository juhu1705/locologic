use crate::control::messages::Message;
use crate::control::rail_system::components::{Address, SwDir};
use crate::control::rail_system::railroad::Railroad;
use async_trait::async_trait;
use locodrive::args::{AddressArg, SlotArg, SpeedArg, SwitchArg, SwitchDirection};
use locodrive::loco_controller::LocoDriveController;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;
use tokio::sync::broadcast::Receiver;
use tokio::sync::Mutex;

use super::RailroadConnector;

type MutRailroads = Mutex<Vec<Arc<Railroad<u8, u16, u16, u16, u16, u16>>>>;

pub struct LocoDriveConnector {
    receiver: Receiver<Message<u8, u16, u16, u16, u16>>,
    sender: LocoDriveController,
    railroad: MutRailroads,
    loco_receiver: Receiver<locodrive::protocol::Message>,
    slots: HashMap<Address, SlotArg>,
}

impl LocoDriveConnector {
    pub async fn new(
        receiver: Receiver<Message<u8, u16, u16, u16, u16>>,
        sender: LocoDriveController,
        loco_receiver: Receiver<locodrive::protocol::Message>,
    ) -> Self {
        LocoDriveConnector {
            receiver,
            sender,
            railroad: Mutex::new(vec![]),
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
                    Err(RecvError::Lagged(_)) => {
                        let _ = self
                            .sender
                            .send_message(locodrive::protocol::Message::LocoAdr(adr.address_arg()))
                            .await;
                    }
                    _ => continue,
                }
            }
        }
        None
    }

    pub async fn handle_message(&mut self, message: Message<u8, u16, u16, u16, u16>) {
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

#[async_trait]
impl RailroadConnector<u8, u16, u16, u16, u16, u16> for LocoDriveConnector {
    async fn handle_message(&mut self, message: Message<u8, u16, u16, u16, u16>) {
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

    async fn reciever(&mut self) -> &mut Receiver<Message<u8, u16, u16, u16, u16>> {
        &mut self.receiver
    }

    async fn register_railroad(&mut self, railroad: Arc<Railroad<u8, u16, u16, u16, u16>>) {
        self.railroad.lock().await.push(railroad.clone());
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

pub async fn run_loconet_connector(
    receiver: Receiver<Message<u8, u16, u16, u16, u16>>,
    loco_controller: LocoDriveController,
    loco_receiver: Receiver<locodrive::protocol::Message>,
) {
    let actor = LocoDriveConnector::new(receiver, loco_controller, loco_receiver).await;

    tokio::spawn(run_connector(actor));
}

impl From<SwDir> for SwitchDirection {
    fn from(dir: SwDir) -> Self {
        match dir {
            SwDir::Straight => SwitchDirection::Straight,
            SwDir::Curved => SwitchDirection::Curved,
        }
    }
}
