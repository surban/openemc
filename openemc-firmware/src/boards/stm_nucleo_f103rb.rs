//! STM Nucleo-F103RB evaluation board.

use cortex_m::peripheral::SCB;
use openemc_shared::BootInfo;
use stm32f1::stm32f103::Peripherals;
use stm32f1xx_hal::{
    afio,
    gpio::{Edge, ExtiPin, Input, Output, Pin, PinState, PullUp, PushPull, CRH, CRL},
    i2c,
    prelude::*,
};
use systick_monotonic::fugit::Rate;

use crate::{
    board::{Board, UnknownEvent, PORTS},
    i2c_reg_slave::Event,
    stusb4500::Voltage,
    Delay, I2C_BUFFER_SIZE,
};

/// I2C development mode register.
const REG_DEVELOPMENT_MODE: u8 = 0xe0;

/// Reset STUSB4500 at startup?
const RESET_STUSB4500: bool = false;

/// STM Nucleo-F103RB evaluation board.
pub struct BoardImpl {
    development_mode: bool,
    stusb4500_alert: Pin<Input<PullUp>, CRH, 'A', 15>,
    _stusb4500_reset: Pin<Output<PushPull>, CRL, 'B', 7>,
}

impl Board for BoardImpl {
    const STANDALONE_I2C_REMAP: bool = true;
    const STANDALONE_IRQ_PIN: u8 = 6;
    const I2C2_MODE: Option<i2c::Mode> = Some(i2c::Mode::Standard { frequency: Rate::<u32, 1, 1>::Hz(300_000) });
    const STUSB4500_I2C_ADDR: Option<u8> = Some(0x28);
    const USB_MAXIMUM_VOLTAGE: Voltage = Voltage::from_mv(10_000);

    fn new(
        board_data: Option<&[u8; BootInfo::BOARD_DATA_SIZE]>, afio: &mut afio::Parts, delay: &mut Delay,
    ) -> BoardImpl {
        let dp = unsafe { Peripherals::steal() };
        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();
        let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let mut stusb4500_reset = gpiob.pb7.into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);
        if RESET_STUSB4500 {
            defmt::info!("reset STUSB4500");
            stusb4500_reset.set_high();
            delay.delay(200.millis());
            stusb4500_reset.set_low();
            delay.delay(200.millis());
        }

        let mut stusb4500_alert = pa15.into_pull_up_input(&mut gpioa.crh);
        stusb4500_alert.make_interrupt_source(afio);
        stusb4500_alert.trigger_on_edge(&dp.EXTI, Edge::Falling);
        stusb4500_alert.clear_interrupt_pending_bit();
        stusb4500_alert.enable_interrupt(&dp.EXTI);

        Self {
            development_mode: board_data.map(|bd| bd[0] != 0).unwrap_or_default(),
            stusb4500_alert,
            _stusb4500_reset: stusb4500_reset,
        }
    }

    fn model() -> &'static [u8] {
        b"stm_nucleo_f103rb"
    }

    fn power_on(&mut self, _delay: &mut Delay) {
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
        usable[0] = 0b0111111111111111;
        usable[1] = 0b1111000001111111;
        usable[2] = 0b1111111111111111;
        usable[3] = 0b0000000000000011;
    }

    fn limit_usable_exti(&mut self, exti: &mut u32) {
        // PA15 is STUSB4500 alert.
        *exti &= !(1 << 15);
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

    fn check_stusb4500_alerting(&mut self) -> bool {
        let pending = self.stusb4500_alert.check_interrupt();
        self.stusb4500_alert.clear_interrupt_pending_bit();
        pending
    }
}
