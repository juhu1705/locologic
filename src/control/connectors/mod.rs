use std::sync::Arc;

use async_trait::async_trait;
use tokio::sync::broadcast::{error::RecvError, Receiver};

use crate::general::{AddressType, SpeedType};

use super::{messages::Message, rail_system::railroad::Railroad};

/// Note: Only available, when  locodrive  dependency is activated.
///
/// Handles the connection between this controlling programm and a Railroad controlled by the
/// protocoll implemented in the locodrive project.
#[cfg(feature = "locodrive")]
pub mod locodrive_connector;

/// General Railroad connector to connect physical railroads with this programm
#[async_trait]
pub trait RailroadConnector<
    Spd: SpeedType,
    TrainAddr: AddressType,
    SensorAddr: AddressType,
    SwitchAddr: AddressType,
    SignalAddr: AddressType,
    CrossingAddr: AddressType,
>
{
    /// Sends a message to the railroad
    async fn handle_message(
        &mut self,
        message: Message<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>,
    );

    async fn reciever(
        &mut self,
    ) -> &mut Receiver<Message<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>>;

    /// Recieves a message from the railroad
    async fn start_connectors(&mut self) {
        loop {
            match self.reciever().await.recv().await {
                Ok(msg) => {
                    self.handle_message(msg).await;
                }
                Err(RecvError::Closed) => {
                    break;
                }
                Err(_err) => {
                    continue;
                }
            }
        }
    }

    async fn register_railroad(
        &mut self,
        railroad: Arc<Railroad<Spd, TrainAddr, SensorAddr, SwitchAddr, SignalAddr>>,
    );
}
