//! STM Nucleo-F103RB evaluation board.
//
// OPENEMC-BOARD-VERSION: 1
// OPENEMC-FLASH-SIZE: 65536
// OPENEMC-RAM-SIZE: 20480
//

use core::mem;
use cortex_m::peripheral::SCB;
use heapless::Vec;
use stm32f1::stm32f103::Peripherals;
use stm32f1xx_hal::{
    afio,
    gpio::{Edge, ExtiPin, Input, Output, Pin, PinState, PullUp, PushPull},
    i2c,
    prelude::*,
};
use systick_monotonic::fugit::Rate;

use crate::{
    board::{
        io_read_available, io_write_available, Board, InitData, InitResources, UnknownI2cRegister, WriteBlock,
        IO_BUFFER_SIZE, PORTS,
    },
    i2c_reg_slave::Response,
    Delay, I2C_BUFFER_SIZE,
};

/// I2C development mode register.
const REG_DEVELOPMENT_MODE: u8 = 0xe0;

/// Reset STUSB4500 at startup?
const RESET_STUSB4500: bool = false;

/// STM Nucleo-F103RB evaluation board.
pub struct BoardImpl {
    development_mode: bool,
    stusb4500_alert: Pin<'A', 15, Input<PullUp>>,
    _stusb4500_reset: Pin<'B', 7, Output<PushPull>>,
    io_data_buf: Vec<u8, { IO_BUFFER_SIZE }>,
}

impl Board for BoardImpl {
    const I2C_REMAP: bool = true;
    const IRQ_PIN: u8 = 6;
    const I2C2_MODE: Option<i2c::Mode> = Some(i2c::Mode::Standard { frequency: Rate::<u32, 1, 1>::Hz(300_000) });
    const STUSB4500_I2C_ADDR: Option<u8> = Some(0x28);
    const USB_MAXIMUM_VOLTAGE: u32 = 10_000;
    type TaskArgs = ();

    fn new(data: InitData, res: InitResources) -> BoardImpl {
        let mut dp = unsafe { Peripherals::steal() };
        let mut gpioa = unsafe { dp.GPIOA.split_without_reset() };
        let mut gpiob = unsafe { dp.GPIOB.split_without_reset() };
        let (pa15, _pb3, _pb4) = res.afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let mut stusb4500_reset = gpiob.pb7.into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);
        if RESET_STUSB4500 {
            defmt::info!("reset STUSB4500");
            stusb4500_reset.set_high();
            res.delay.delay(200.millis());
            stusb4500_reset.set_low();
            res.delay.delay(200.millis());
        }

        let mut stusb4500_alert = pa15.into_pull_up_input(&mut gpioa.crh);
        stusb4500_alert.make_interrupt_source(res.afio);
        stusb4500_alert.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        stusb4500_alert.clear_interrupt_pending_bit();
        stusb4500_alert.enable_interrupt(&mut dp.EXTI);

        #[allow(clippy::get_first)]
        Self {
            development_mode: data
                .boot_info
                .board_data()
                .get(0)
                .map(|dev_mode| *dev_mode != 0)
                .unwrap_or_default(),
            stusb4500_alert,
            _stusb4500_reset: stusb4500_reset,
            io_data_buf: Vec::new(),
        }
    }

    fn model() -> &'static [u8] {
        b"stm_nucleo_f103rb"
    }

    fn power_on(&mut self, _afio: &mut afio::Parts, _delay: &mut Delay) {
        let dp = unsafe { Peripherals::steal() };

        // Turn on LED.
        defmt::info!("system power on");
        let mut gpioa = unsafe { dp.GPIOA.split_without_reset() };
        gpioa.pa5.into_push_pull_output_with_state(&mut gpioa.crl, PinState::High);

        // Check for key.
        let mut gpioc = unsafe { dp.GPIOC.split_without_reset() };
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
        usable[0] = 0b0111111111111111;
        usable[1] = 0b1111000001111111;
        usable[2] = 0b1111111111111111;
        usable[3] = 0b0000000000000011;
    }

    fn limit_usable_exti(&mut self, exti: &mut u32) {
        // PA15 is STUSB4500 alert.
        *exti &= !(1 << 15);
    }

    fn i2c_read(&mut self, reg: u8) -> Result<Response<I2C_BUFFER_SIZE>, UnknownI2cRegister> {
        match reg {
            REG_DEVELOPMENT_MODE => Ok(Response::set_u8(u8::from(self.development_mode))),
            _ => Err(UnknownI2cRegister),
        }
    }

    fn check_stusb4500_alerting(&mut self) -> bool {
        let pending = self.stusb4500_alert.check_interrupt();
        self.stusb4500_alert.clear_interrupt_pending_bit();
        pending
    }

    fn io_write(&mut self, data: &[u8]) -> Result<(), WriteBlock> {
        if self.io_data_buf.is_empty() {
            defmt::info!("Received {} bytes IO board data", data.len());
            self.io_data_buf.clear();
            defmt::unwrap!(self.io_data_buf.extend_from_slice(data));
            if !self.io_data_buf.is_empty() {
                io_read_available();
            }
            Ok(())
        } else {
            defmt::debug!("IO board data not empty");
            Err(WriteBlock)
        }
    }

    fn io_read(&mut self) -> Vec<u8, IO_BUFFER_SIZE> {
        let data = self.io_data_buf.clone();
        defmt::info!("Sent {} bytes IO board data", data.len());

        if self.io_data_buf.is_empty() {
            io_write_available();
        } else {
            self.io_data_buf.truncate(self.io_data_buf.len() - 1);
            io_read_available();
        }

        data
    }
}
