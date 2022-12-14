//! Board.

use openemc_shared::{BootInfo, ResetStatus};

use crate::{i2c_reg_slave::I2CRegTransaction, BoardInitResult};

/// Board-specific functionality.
pub trait Board {
    /// Processor clock speed.
    const CPU_CLOCK: u32 = 8_000_000;

    /// I2C slave address of the bootloader.
    const I2C_ADDR: u8 = 0x10;

    /// I2C pins remapped?
    const I2C_REMAP: bool = true;

    /// IRQ pin number (PA0, PA1, .., PA15, PB0, PB1, ...)
    const IRQ_PIN: u8 = 6; // PA6

    /// IRQ pin configuration.
    const IRQ_PIN_CFG: u8 = 0b0010; // 2 MHz push-pull output

    /// Timeout ticks.
    const TIMEOUT_TICKS: u32 = 4_200_000_000;

    /// If EMC is reset via reset pin, this prevents automatic
    /// starting of the user program.
    const PIN_RESET_PREVENTS_AUTORUN: bool = true;

    /// Creates a new instance.
    fn new() -> Self;

    /// Board model.
    fn model(&self) -> &'static [u8];

    /// Initialize the board.
    ///
    /// Returns true if control should stay inside the bootloader.
    fn init(&mut self, _boot_reason: u16, _reset_status: ResetStatus) -> BoardInitResult {
        BoardInitResult::Normal
    }

    /// Cleanup before start of user program.
    fn pre_start(&mut self) {}

    /// Board-specific data to pass to user program.
    fn board_data(&self) -> [u8; BootInfo::BOARD_DATA_SIZE] {
        [0; BootInfo::BOARD_DATA_SIZE]
    }

    /// Turn system power on so that system can flash user program.
    fn system_power_on(&mut self) {}

    /// Bootloader I2C request handler.
    fn bootloader_request(&mut self, _t: I2CRegTransaction) {}
}
