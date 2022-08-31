use locodrive::args::{AddressArg, SlotArg};

/// Represents one train
pub struct Train {
    /// The trains address
    pub address: AddressArg,
    /// The trains slot
    pub slot: SlotArg,
}
