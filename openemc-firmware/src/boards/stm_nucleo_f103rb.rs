//! STM Nucleo-F103RB evaluation board.

use cortex_m::peripheral::SCB;
use openemc_shared::BootInfo;
use stm32f1::stm32f103::Peripherals;
use stm32f1xx_hal::{gpio::PinState, prelude::*};

use crate::board::{Board, UnknownEvent, PORTS};
use crate::i2c_reg_slave::Event;
use crate::I2C_BUFFER_SIZE;

/// I2C development mode register.
const REG_DEVELOPMENT_MODE: u8 = 0xe0;

/// STM Nucleo-F103RB evaluation board.
pub struct BoardImpl {
    development_mode: bool,
}

impl Board for BoardImpl {
    const STANDALONE_I2C_REMAP: bool = true;
    const STANDALONE_IRQ_PIN: u8 = 6;

    fn new(board_data: Option<&[u8; BootInfo::BOARD_DATA_SIZE]>) -> BoardImpl {
        Self { development_mode: board_data.map(|bd| bd[0] != 0).unwrap_or_default() }
    }

    fn model() -> &'static [u8] {
        b"stm_nucleo_f103rb"
    }

    fn init(&mut self) {
        let dp = unsafe { Peripherals::steal() };

        // Turn on LED.
        defmt::info!("system power on");
        let mut gpioa = dp.GPIOA.split();
        gpioa.pa5.into_push_pull_output_with_state(&mut gpioa.crl, PinState::High);

        // Check for key.
        let mut gpioc = dp.GPIOC.split();
        gpioc.pc13.as_pull_up_input(&mut gpioc.crh, |pc13| {
            if pc13.is_low() {
                defmt::info!("resetting backup domain");
                dp.RCC.bdcr.modify(|_, w| w.bdrst().enabled());
                while pc13.is_low() {}
                SCB::sys_reset();
            }
        });
    }

    fn limit_usable(&mut self, usable: &mut [u16; PORTS]) {
        usable[0] = 0b1111111111111111;
        usable[1] = 0b1111111111111111;
        usable[2] = 0b1110000000000000;
        usable[3] = 0b0000000000000011;
    }

    fn i2c_event<'a>(&mut self, event: Event<'a, I2C_BUFFER_SIZE>) -> Result<(), UnknownEvent<'a>> {
        match event {
            Event::Read { reg: REG_DEVELOPMENT_MODE, value } => {
                value.set_u8(u8::from(self.development_mode));
                Ok(())
            }
            unknown => Err(UnknownEvent(unknown)),
        }
    }
}
