//! Board.

use openemc_shared::BootInfo;

use crate::{boot, i2c_reg_slave, I2C_BUFFER_SIZE};

/// Number of IO ports (PA, PB, PC, etc.).
pub const PORTS: usize = 4;

/// Board-specific functionality.
pub trait Board {
    /// I2C slave address of the bootloader. (only for standalone firmware)
    const STANDALONE_I2C_ADDR: u8 = 0x10;

    /// I2C pins remapped? (only for standalone firmware)
    const STANDALONE_I2C_REMAP: bool = true;

    /// IRQ pin number (PA0, PA1, .., PA15, PB0, PB1, ...) (only for standalone firmware)
    const STANDALONE_IRQ_PIN: u8 = 6; // PA6

    /// IRQ pin configuration. (only for standalone firmware)
    const STANDALONE_IRQ_PIN_CFG: u8 = 0b0010; // 2 MHz push-pull output

    /// Create a new instance.
    fn new(board_data: Option<&[u8; BootInfo::BOARD_DATA_SIZE]>) -> Self;

    /// Returns whether the board id is supported.
    fn is_supported(&self, model: &[u8]) -> bool {
        model.starts_with(Self::model())
    }

    /// Board model.
    fn model() -> &'static [u8];

    /// Initializes board-specific functionality.
    fn init(&mut self) {}

    /// Shutdown the system and go to sleep.
    fn shutdown(&mut self) -> ! {
        defmt::info!("entering standby mode");
        boot::enter_standby();
    }

    /// Limit usable GPIOs.
    fn limit_usable(&mut self, _usable: &mut [u16; PORTS]) {}

    /// Handle board-specific I2C event.
    fn i2c_event<'a>(
        &mut self, event: i2c_reg_slave::Event<'a, I2C_BUFFER_SIZE>,
    ) -> Result<(), UnknownEvent<'a>> {
        Err(UnknownEvent(event))
    }
}

/// Unknown I2C event.
pub struct UnknownEvent<'a>(pub i2c_reg_slave::Event<'a, I2C_BUFFER_SIZE>);
