use crate::control::messages::Message;
use crate::control::rail_system::components::{Address, SwDir};
use locodrive::args::{AddressArg, SlotArg, SpeedArg, SwitchArg, SwitchDirection};
use locodrive::loco_controller::{LocoDriveController, LocoDriveMessage};
use std::collections::HashMap;
use tokio::sync::broadcast::error::RecvError;
use tokio::sync::broadcast::{self, Receiver, Sender};
use tokio_serial::Error;

pub struct LocoDriveConnector {
    receiver: Receiver<Message<u8, u16, u16, u16, u16>>,
    rail_controller: LocoDriveController,
    rail_messages: Sender<LocoDriveMessage>,
    loco_receiver: Receiver<locodrive::protocol::Message>,
    slots: HashMap<Address, SlotArg>,
}

impl LocoDriveConnector {
    pub async fn new(
        port_name: &str,
        baud_rate: u32,
        sending_timeout: u64,
        flow_control: tokio_serial::FlowControl,
        receiver: Receiver<Message<u8, u16, u16, u16, u16>>,
        loco_receiver: Receiver<locodrive::protocol::Message>,
    ) -> Result<Self, Error> {
        let (rail_messages, _) = broadcast::channel(25);

        let rail_controller = LocoDriveController::new(
            port_name,
            baud_rate,
            sending_timeout,
            flow_control,
            rail_messages.clone(),
            true,
        )
        .await?;

        Ok(LocoDriveConnector {
            receiver,
            rail_controller,
            rail_messages,
            loco_receiver,
            slots: HashMap::new(),
        })
    }

    pub async fn lookup_slot(&mut self, adr: Address) -> Option<SlotArg> {
        if self.slots.contains_key(&adr) {
            return Some(*self.slots.get(&adr).unwrap());
        } else {
            let _ = self
                .rail_controller
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
            let _ = self.rail_controller.send_message(loco_net_message).await;
        }
    }

    pub fn get_reciever(&self) -> Receiver<LocoDriveMessage> {
        self.rail_messages.subscribe()
    }

    pub fn get_sender(&self) -> Sender<LocoDriveMessage> {
        self.rail_messages.clone()
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

impl From<SwDir> for SwitchDirection {
    fn from(dir: SwDir) -> Self {
        match dir {
            SwDir::Straight => SwitchDirection::Straight,
            SwDir::Curved => SwitchDirection::Curved,
        }
    }
}
