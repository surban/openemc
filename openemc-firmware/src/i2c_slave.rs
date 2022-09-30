//! I2C slave driver with interrupt support.

use core::ops::Deref;

use stm32f1xx_hal::{
    afio::MAPR,
    gpio::{self, Alternate, OpenDrain},
    pac::{I2C1, I2C2, RCC},
    rcc::{BusClock, Clocks, Enable, Reset},
    time::Hertz,
};

/// I2C slave error.
#[derive(Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
}

/// Helper trait to ensure that the correct I2C pins are used for the corresponding interface
pub trait Pins<I2C> {
    const REMAP: bool;
}

impl Pins<I2C1> for (gpio::PB6<Alternate<OpenDrain>>, gpio::PB7<Alternate<OpenDrain>>) {
    const REMAP: bool = false;
}

impl Pins<I2C1> for (gpio::PB8<Alternate<OpenDrain>>, gpio::PB9<Alternate<OpenDrain>>) {
    const REMAP: bool = true;
}

impl Pins<I2C2> for (gpio::PB10<Alternate<OpenDrain>>, gpio::PB11<Alternate<OpenDrain>>) {
    const REMAP: bool = false;
}

/// I2C peripheral operating in slave mode.
///
/// Only a single 7-bit address is supported.
pub struct I2cSlave<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
    addr: u8,
    pclk1: Hertz,
    state: State,
}

#[derive(Clone, Copy)]
enum State {
    Idle,
    Receiving,
    Sending,
}

pub trait Instance: Deref<Target = stm32f1xx_hal::pac::i2c1::RegisterBlock> + Enable + Reset + BusClock {}

impl Instance for I2C1 {}
impl Instance for I2C2 {}

impl<PINS> I2cSlave<I2C1, PINS> {
    /// Creates a generic I2C1 object on pins PB6 and PB7 or PB8 and PB9 (if remapped)
    pub fn i2c1(i2c: I2C1, pins: PINS, mapr: &mut MAPR, addr: u8, clocks: Clocks) -> Self
    where
        PINS: Pins<I2C1>,
    {
        mapr.modify_mapr(|_, w| w.i2c1_remap().bit(PINS::REMAP));
        I2cSlave::<I2C1, _>::configure(i2c, pins, addr, clocks)
    }
}

impl<PINS> I2cSlave<I2C2, PINS> {
    /// Creates a generic I2C2 object on pins PB10 and PB11.
    pub fn i2c2(i2c: I2C2, pins: PINS, addr: u8, clocks: Clocks) -> Self
    where
        PINS: Pins<I2C2>,
    {
        I2cSlave::<I2C2, _>::configure(i2c, pins, addr, clocks)
    }
}

impl<I2C, PINS> I2cSlave<I2C, PINS>
where
    I2C: Instance,
{
    /// Configures the I2C peripheral to work in slave mode.
    fn configure(i2c: I2C, pins: PINS, addr: u8, clocks: Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        I2C::enable(rcc);
        I2C::reset(rcc);

        let mut i2c = Self { i2c, pins, addr, pclk1: I2C::clock(&clocks), state: State::Idle };
        i2c.reset();
        i2c
    }
}

impl<I2C, PINS> I2cSlave<I2C, PINS>
where
    I2C: Instance,
{
    /// Initializes I2C.
    ///
    /// Configures the `I2C_CRX` registers according to the system frequency.
    fn init(&mut self) {
        self.i2c.cr1.write(|w| w.pe().clear_bit());

        let pclk1_mhz = self.pclk1.to_MHz() as u16;
        self.i2c.cr2.write(|w| unsafe { w.freq().bits(pclk1_mhz as u8) });

        self.i2c.oar1.write(|w| w.add().bits((self.addr as u16) << 1));

        self.i2c.cr1.modify(|_, w| w.pe().enabled().ack().ack());
    }

    /// Perform an I2C software reset
    fn reset(&mut self) {
        self.i2c.cr1.write(|w| w.pe().set_bit().swrst().set_bit());
        self.i2c.cr1.reset();
        self.init();
    }

    /// Gets the next event.
    pub fn event<'a>(&'a mut self) -> nb::Result<Event<'a, I2C, PINS>, Error> {
        let sr1 = self.i2c.sr1.read();

        if sr1.berr().is_error() {
            self.i2c.sr1.modify(|_, w| w.berr().clear_bit());
            return Err(Error::Bus.into());
        }

        match self.state {
            State::Idle => {
                if sr1.addr().is_match() {
                    self.i2c.sr1.read();
                    let sr2 = self.i2c.sr2.read();

                    if sr2.tra().bit_is_set() {
                        self.state = State::Sending;
                        Ok(Event::StartWrite)
                    } else {
                        self.state = State::Receiving;
                        self.i2c.cr1.modify(|_, w| w.ack().ack());
                        Ok(Event::StartRead)
                    }
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
            State::Sending => {
                if sr1.af().is_failure() {
                    self.i2c.sr1.modify(|_, w| w.af().no_failure());
                    self.state = State::Idle;
                    Ok(Event::End)
                } else if sr1.tx_e().is_empty() {
                    Ok(Event::Write(Writer(self)))
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
            State::Receiving => {
                if sr1.rx_ne().is_not_empty() {
                    let value = self.i2c.dr.read().dr().bits();
                    Ok(Event::Read(value))
                } else if sr1.stopf().is_stop() || sr1.tx_e().is_empty() {
                    self.i2c.sr1.read();
                    self.i2c.cr1.modify(|_, w| w.pe().enabled().ack().ack());
                    self.state = State::Idle;

                    if sr1.tx_e().is_empty() {
                        // Workaround for I2C bus hanging after restart condition.
                        for _ in 0..100 {
                            self.i2c.sr1.read();
                        }
                        Ok(Event::Restart)
                    } else {
                        Ok(Event::End)
                    }
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }
    }

    /// Enables triggering the event interrupt (I2Cx_EV) when the transmit buffer is
    /// empty or the receive buffer is not empty.
    ///
    /// The event interrupt must be enabled separately using [`listen_event`](Self::listen_event).
    pub fn listen_buffer(&mut self) {
        self.i2c.cr2.modify(|_, w| w.itbufen().enabled());
    }

    /// Disables triggering the event interrupt (I2Cx_EV) when the transmit buffer is
    /// empty or the receive buffer is not empty.
    pub fn unlisten_buffer(&mut self) {
        self.i2c.cr2.modify(|_, w| w.itbufen().disabled());
    }

    /// Enables triggering the event interrupt (I2Cx_EV) when the address is matched,
    /// stop is detected, the byte transfer is finished or an event
    /// described in [`listen_buffer`](Self::listen_buffer) occurrs.
    pub fn listen_event(&mut self) {
        self.i2c.cr2.modify(|_, w| w.itevten().enabled());
    }

    /// Disables triggering the event interrupt (I2Cx_EV) when the address is matched,
    /// stop is detected, the byte transfer is finished or an event
    /// described in [unlisten_buffer](Self::unlisten_buffer) occurrs.
    pub fn unlisten_event(&mut self) {
        self.i2c.cr2.modify(|_, w| w.itevten().disabled());
    }

    /// Enables triggering the error interrupt (I2Cx_ER) when an error is detected.
    pub fn listen_error(&mut self) {
        self.i2c.cr2.modify(|_, w| w.iterren().enabled());
    }

    /// Disables triggering the error interrupt (I2Cx_ER) when an error is detected.
    pub fn unlisten_error(&mut self) {
        self.i2c.cr2.modify(|_, w| w.iterren().disabled());
    }

    /// Releases the I2C peripheral and associated pins.
    pub fn release(self) -> (I2C, PINS) {
        (self.i2c, self.pins)
    }
}

/// An I2C slave event.
pub enum Event<'a, I2C, PINS> {
    /// Start of reception from the I2C master.
    StartRead,
    /// Start of transmission to the I2C master.
    StartWrite,
    /// A byte has been received from the I2C master.
    Read(u8),
    /// A byte must be transmitted to the I2C master.
    Write(Writer<'a, I2C, PINS>),
    /// End of reception and start of transmission.
    Restart,
    /// End of transfer.
    End,
}

/// Transmits a byte to the I2C master.
pub struct Writer<'a, I2C, PINS>(&'a mut I2cSlave<I2C, PINS>);

impl<'a, I2C, PINS> Writer<'a, I2C, PINS>
where
    I2C: Instance,
{
    /// Sends the specified value to the I2C master.
    pub fn write(self, value: u8) {
        self.0.i2c.dr.write(|w| w.dr().bits(value));
    }
}
