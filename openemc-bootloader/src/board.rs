//! Board.

use crate::{
    bootloader::BootloaderResult, i2c_reg_slave::I2CRegTransaction, util::enter_standby, BoardInitResult,
};
use openemc_shared::boot::{BootInfo, ResetStatus};

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

    /// Write board-specific data to pass to user program.
    ///
    /// Return the size of the written data.
    fn write_board_data(&mut self, _board_data: &mut [u8; BootInfo::BOARD_DATA_SIZE]) -> usize {
        0
    }

    /// Turn system power to the desired state.
    ///
    /// If state is on, the board must power up enough so that system can flash user program.
    fn set_system_power(&mut self, _state: bool) {}

    /// Bootloader I2C request handler.
    fn bootloader_request(&mut self, _t: I2CRegTransaction) {}

    /// Bootloader idle function.
    ///
    /// If it returns a value, the bootloader exits with the specified result.
    fn idle(&mut self, _total_s: u16, idle_s: u16, timeout_enabled: bool) -> Option<BootloaderResult> {
        if idle_s >= 600 && timeout_enabled {
            Some(BootloaderResult::Timeout)
        } else {
            None
        }
    }

    /// Shutdown the system and go to sleep.
    fn power_off(&mut self) -> ! {
        enter_standby();
    }
}
