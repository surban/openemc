//! I2C master.

use defmt::Format;
use stm32f1::{
    stm32f103,
    stm32f103::{
        gpioa::odr::ODR0_A::{High, Low},
        GPIOB,
    },
};
use stm32f1xx_hal::{
    gpio::{Alternate, OpenDrain},
    i2c,
    rcc::Clocks,
};

use crate::util::DwtDelay;

/// I2C error.
#[derive(Clone, Format, PartialEq, Eq)]
pub enum I2cError {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    /// Timeout
    Timeout,
    /// SDA line stuck high during recovery.
    SdaStuckLow,
    /// Other error.
    Other,
}

impl From<i2c::Error> for I2cError {
    fn from(error: i2c::Error) -> Self {
        match error {
            i2c::Error::Bus => I2cError::Bus,
            i2c::Error::Arbitration => I2cError::Arbitration,
            i2c::Error::Acknowledge => I2cError::Acknowledge,
            i2c::Error::Overrun => I2cError::Overrun,
            i2c::Error::Timeout => I2cError::Timeout,
            _ => I2cError::Other,
        }
    }
}

/// I2C 2 master.
pub type I2c2Master = i2c::BlockingI2c<
    stm32f1xx_hal::pac::I2C2,
    (
        stm32f1xx_hal::gpio::Pin<'B', 10, Alternate<OpenDrain>>,
        stm32f1xx_hal::gpio::Pin<'B', 11, Alternate<OpenDrain>>,
    ),
>;

/// Allows temporary GPIO control over the I2C 2 SCL and SDA lines.
///
/// Drop to release.
pub struct I2c2Gpio {
    gpiob: GPIOB,
    saved_crh: stm32f1xx_hal::pac::gpioa::crh::R,
}

impl I2c2Gpio {
    /// Activates temporary GPIO control over the I2C 2 SCL and SDA lines.
    pub fn new() -> Self {
        let device = unsafe { stm32f103::Peripherals::steal() };
        let gpiob = device.GPIOB;
        let saved_crh = gpiob.crh.read();
        Self { gpiob, saved_crh }
    }

    /// Sets the SCL level.
    pub fn scl(&self, level: bool) {
        self.gpiob.odr.modify(|_, w| w.odr10().variant(if level { High } else { Low }));
        self.gpiob.crh.modify(|_, w| w.cnf10().open_drain().mode10().output());
    }

    /// Sets the SDA level.
    pub fn sda(&self, level: bool) {
        self.gpiob.odr.modify(|_, w| w.odr11().variant(if level { High } else { Low }));
        self.gpiob.crh.modify(|_, w| w.cnf11().open_drain().mode11().output());
    }

    /// Reads the SDA level.
    pub fn read_sda(&self) -> bool {
        self.gpiob.crh.modify(|_, w| w.mode11().input());

        for _ in 0..32 {
            self.gpiob.idr.read();
        }

        self.gpiob.idr.read().idr11().is_high()
    }
}

impl Drop for I2c2Gpio {
    fn drop(&mut self) {
        // Restore I2C pin configuration.
        self.gpiob.crh.modify(|_, w| {
            w.cnf10()
                .variant(self.saved_crh.cnf10().variant())
                .mode10()
                .variant(self.saved_crh.mode10().variant())
                .cnf11()
                .variant(self.saved_crh.cnf11().variant())
                .mode11()
                .variant(self.saved_crh.mode11().variant())
        });
    }
}

const CLOCK_HALF_PERIOD_US: u32 = 500;

/// Recovers the I2C bus.
pub fn recover(i2c: &mut I2c2Master, clocks: &Clocks) -> Result<(), I2cError> {
    defmt::debug!("Recovering I2C bus");
    let dwt = DwtDelay::new(clocks);
    let i2c_gpio = I2c2Gpio::new();

    // Set SCL and SDA to high via GPIO.
    i2c_gpio.scl(true);
    dwt.delay(CLOCK_HALF_PERIOD_US);
    i2c_gpio.sda(true);
    dwt.delay(CLOCK_HALF_PERIOD_US);

    // Output clock so that all in-progress transfers end.
    defmt::debug!("Cycling SCL to end all I2C transfers");
    for _ in 0..15 {
        i2c_gpio.scl(false);
        dwt.delay(CLOCK_HALF_PERIOD_US);
        i2c_gpio.scl(true);
        dwt.delay(CLOCK_HALF_PERIOD_US);
    }

    // Wait until SDA is high.
    defmt::debug!("Wait for SDA to be released");
    let mut sda_value = false;
    for _ in 0..15 {
        i2c_gpio.scl(false);
        dwt.delay(CLOCK_HALF_PERIOD_US);
        i2c_gpio.scl(true);
        dwt.delay(CLOCK_HALF_PERIOD_US);

        sda_value = i2c_gpio.read_sda();
        if sda_value {
            break;
        }
    }

    // Verify SDA is high.
    if !sda_value {
        defmt::error!("I2C SDA is stuck low");
        return Err(I2cError::SdaStuckLow);
    }

    // Generate stop condition.
    defmt::debug!("Generating I2C stop condition");
    i2c_gpio.scl(true);
    dwt.delay(CLOCK_HALF_PERIOD_US);
    i2c_gpio.sda(false);
    dwt.delay(CLOCK_HALF_PERIOD_US);
    i2c_gpio.sda(true);
    dwt.delay(CLOCK_HALF_PERIOD_US);

    // Release I2C GPIO control.
    drop(i2c_gpio);

    // Reset I2C master peripheral.
    defmt::debug!("Performing I2C master reset");
    i2c.reset();

    Ok(())
}

/// Glitches the I2C bus by issuing a start condition (for fault recovery testing).
pub fn glitch(dwt: DwtDelay) {
    if !cfg!(feature = "debug-i2c2-glitch") {
        defmt::panic!("I2C glitching feature is disabled")
    }

    defmt::warn!("Glitching I2C bus");
    let i2c_gpio = I2c2Gpio::new();

    // Start in idle.
    i2c_gpio.scl(true);
    i2c_gpio.sda(true);
    dwt.delay(CLOCK_HALF_PERIOD_US);

    // Issue START condition by dropping SDA while SCL is high.
    i2c_gpio.scl(true);
    i2c_gpio.sda(false);
    dwt.delay(CLOCK_HALF_PERIOD_US);

    // Now also drop SCL.
    i2c_gpio.scl(false);
    i2c_gpio.sda(false);
    dwt.delay(CLOCK_HALF_PERIOD_US);

    // So that SDA can be raised without issuing a STOP condition.
    i2c_gpio.scl(false);
    i2c_gpio.sda(true);
    dwt.delay(CLOCK_HALF_PERIOD_US);
}
