//! STM Nucleo-F103RB evaluation board.

use openemc_shared::{BootInfo, ResetStatus};
use stm32f1::stm32f103::Peripherals;

use crate::{board::Board, i2c_reg_slave::I2CRegTransaction, util::delay_ms, watchdog, BoardInitResult};

/// I2C development mode register.
const REG_DEVELOPMENT_MODE: u8 = 0xe0;

/// STM Nucleo-F103RB evaluation board.
pub struct BoardImpl {
    development_mode: bool,
}

impl Board for BoardImpl {
    const I2C_REMAP: bool = true;
    const IRQ_PIN: u8 = 6;
    const PIN_RESET_PREVENTS_AUTORUN: bool = false;

    fn new() -> Self {
        Self { development_mode: false }
    }

    fn model(&self) -> &'static [u8] {
        b"stm_nucleo_f103rb"
    }

    fn init(&mut self, _boot_reason: u16, reset_status: ResetStatus) -> BoardInitResult {
        let dp = unsafe { Peripherals::steal() };

        // Configure power pin.
        dp.RCC.apb2enr.modify(|_, w| w.iopaen().enabled());
        dp.GPIOA.crl.modify(|_, w| w.cnf5().push_pull().mode5().output());

        // Configure user button pin.
        dp.RCC.apb2enr.modify(|_, w| w.iopcen().enabled());
        dp.GPIOC.crh.modify(|_, w| w.cnf13().alt_push_pull().mode15().input());
        dp.GPIOC.odr.modify(|_, w| w.odr13().high());
        delay_ms(10);

        // Check for factory reset.
        if reset_status.is_external() {
            defmt::info!("reset by button");

            let mut i = 0;
            while dp.GPIOC.idr.read().idr13().is_low() {
                watchdog::pet();

                i += 1;
                delay_ms(100);

                if i >= 50 {
                    defmt::info!("factory reset requested by button");

                    let mut led = true;
                    while dp.GPIOC.idr.read().idr13().is_low() {
                        watchdog::pet();
                        dp.GPIOA.odr.modify(|_, w| w.odr5().bit(led));
                        delay_ms(100);
                        led = !led;
                    }

                    delay_ms(1000);
                    return BoardInitResult::FactoryReset;
                }
            }
        }

        // Check for development mode.
        let mut i = 0;
        while dp.GPIOC.idr.read().idr13().is_low() {
            watchdog::pet();

            i += 1;
            delay_ms(100);

            if i >= 50 {
                defmt::info!("development mode");
                self.development_mode = true;

                let mut led = true;
                while dp.GPIOC.idr.read().idr13().is_low() {
                    watchdog::pet();
                    dp.GPIOA.odr.modify(|_, w| w.odr5().bit(led));
                    delay_ms(500);
                    led = !led;
                }

                delay_ms(1000);
                return BoardInitResult::StartBootloader;
            }
        }

        BoardInitResult::Normal
    }

    fn pre_start(&mut self) {
        let dp = unsafe { Peripherals::steal() };
        dp.RCC.apb2enr.modify(|_, w| w.iopcen().disabled());
    }

    fn system_power_on(&mut self) {
        let dp = unsafe { Peripherals::steal() };
        dp.GPIOA.odr.modify(|_, w| w.odr5().high());
    }

    fn write_board_data(&mut self, board_data: &mut [u8; BootInfo::BOARD_DATA_SIZE]) -> usize {
        board_data[0] = u8::from(self.development_mode);
        1
    }

    fn bootloader_request(&mut self, t: I2CRegTransaction) {
        match t {
            I2CRegTransaction::Read(mut tx) if tx.reg() == REG_DEVELOPMENT_MODE => {
                tx.send_u8(u8::from(self.development_mode));
            }
            _ => (),
        }
    }
}
