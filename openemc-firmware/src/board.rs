//! Board.

use openemc_shared::BootInfo;
use stm32f1xx_hal::{afio, i2c};

use crate::{
    boot, i2c_reg_slave,
    stusb4500::{Current, FixedSinkPdo, Voltage},
    Delay, I2C_BUFFER_SIZE,
};

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

    /// Mode for I2C 2 master bus.
    const I2C2_MODE: Option<i2c::Mode> = None;

    /// I2C address on I2C 2 master bus of STUSB4500 USB PD controller, if present.
    const STUSB4500_I2C_ADDR: Option<u8> = None;

    /// Reset STUSB4500 USB PD controller at startup?
    const STUSB4500_RESET: bool = true;

    /// Initial USB PD sink PDO.
    ///
    /// Must have voltage of 5 V.
    const USB_INITIAL_PDO: FixedSinkPdo = FixedSinkPdo {
        operating_current: Current::from_ma(100),
        voltage: Voltage::from_mv(5000),
        fast_role_req_current: Current(0),
        dual_role_data: false,
        dual_role_power: false,
        communication: true,
        unconstrained_power: false,
        higher_capability: false,
    };

    /// Maximum USB input voltage to request via USB PD.
    const USB_MAXIMUM_VOLTAGE: Voltage = Voltage::from_mv(5000);

    /// Create a new instance.
    fn new(
        board_data: Option<&[u8; BootInfo::BOARD_DATA_SIZE]>, afio: &mut afio::Parts, delay: &mut Delay,
    ) -> Self;

    /// Returns whether the board id is supported.
    fn is_supported(&self, model: &[u8]) -> bool {
        model.starts_with(Self::model())
    }

    /// Board model.
    fn model() -> &'static [u8];

    /// Powers on the system.
    fn power_on(&mut self, _delay: &mut Delay) {}

    /// Shutdown the system and go to sleep.
    fn shutdown(&mut self) -> ! {
        defmt::info!("entering standby mode");
        boot::enter_standby();
    }

    /// Limit usable GPIOs.
    fn limit_usable(&mut self, _usable: &mut [u16; PORTS]) {}

    /// Limit usable EXTI lines.
    fn limit_usable_exti(&mut self, _exti: &mut u32) {}

    /// Handle board-specific I2C event.
    fn i2c_event<'a>(
        &mut self, event: i2c_reg_slave::Event<'a, I2C_BUFFER_SIZE>,
    ) -> Result<(), UnknownEvent<'a>> {
        Err(UnknownEvent(event))
    }

    /// Checks whether STUSB4500 is alerting and clears the pending interrupt.
    fn check_stusb4500_alerting(&mut self) -> bool {
        false
    }
}

/// Unknown I2C event.
pub struct UnknownEvent<'a>(pub i2c_reg_slave::Event<'a, I2C_BUFFER_SIZE>);
