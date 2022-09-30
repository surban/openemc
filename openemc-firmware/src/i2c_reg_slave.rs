//! I2C register slave.

use core::ops::Deref;

use crate::i2c_slave::{self, Event as I2cEvent, I2cSlave, Instance};

/// I2C register slave error.
#[derive(Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// The buffer overflowed during reception.
    BufferOverrun,
    /// An I2C error occurred.
    I2c(i2c_slave::Error),
}

impl From<i2c_slave::Error> for Error {
    fn from(err: i2c_slave::Error) -> Self {
        Self::I2c(err)
    }
}

/// I2C slave using the register access model.
pub struct I2CRegSlave<I2C, PINS, const BUFFER: usize> {
    slave: I2cSlave<I2C, PINS>,
    reg: u8,
    state: State,
    buf: [u8; BUFFER],
    pos: usize,
}

/// State of register slave.
#[derive(Clone, Copy)]
enum State {
    /// Idle.
    Idle,
    /// Expecting to receive register.
    ReceiveReg,
    /// Register value received, waiting for write or read restart.
    ReceivedReg,
    /// Receiving register value.
    Receiving,
    /// Received read request, waiting for register value.
    StartSend,
    /// Sending register value.
    Sending,
    /// Waiting for end.
    Clear,
}

impl<I2C, PINS, const BUFFER: usize> I2CRegSlave<I2C, PINS, BUFFER>
where
    I2C: Instance,
{
    /// Initializes the I2C register slave.
    pub fn new(slave: I2cSlave<I2C, PINS>) -> Self {
        Self { slave, reg: 0, state: State::Idle, buf: [0; BUFFER], pos: 0 }
    }

    /// Releases the I2C slave.
    pub fn into_inner(self) -> I2cSlave<I2C, PINS> {
        self.slave
    }

    /// Accesses the I2C slave.
    ///
    /// This should not be used to process events, as doing so would
    /// confuse the state of the I2C register model.
    pub fn as_mut(&mut self) -> &mut I2cSlave<I2C, PINS> {
        &mut self.slave
    }

    fn i2c_event(&mut self) -> nb::Result<i2c_slave::Event<I2C, PINS>, Error> {
        match self.slave.event() {
            Ok(evt) => Ok(evt),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
            Err(nb::Error::Other(err)) => {
                self.state = State::Idle;
                Err(nb::Error::Other(err.into()))
            }
        }
    }

    /// Gets the next event.
    pub fn event<'a>(&'a mut self) -> nb::Result<Event<'a, BUFFER>, Error> {
        match &mut self.state {
            State::Idle => match self.i2c_event()? {
                I2cEvent::StartRead => {
                    self.state = State::ReceiveReg;
                    Err(nb::Error::WouldBlock)
                }
                I2cEvent::StartWrite => {
                    self.state = State::StartSend;
                    self.pos = 0;
                    Ok(Event::Read {
                        reg: self.reg,
                        value: Reader { state: &mut self.state, buf: &mut self.buf },
                    })
                }
                _ => defmt::unreachable!(),
            },
            State::ReceiveReg => match self.i2c_event()? {
                I2cEvent::Read(value) => {
                    self.reg = value;
                    self.state = State::ReceivedReg;
                    Err(nb::Error::WouldBlock)
                }
                I2cEvent::End | I2cEvent::Restart => {
                    self.state = State::Idle;
                    Err(nb::Error::WouldBlock)
                }
                _ => defmt::unreachable!(),
            },
            State::ReceivedReg => match self.i2c_event()? {
                I2cEvent::Read(value) => {
                    if self.buf.len() > 0 {
                        self.buf[0] = value;
                        self.pos = 1;
                        self.state = State::Receiving;
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.state = State::Clear;
                        Err(nb::Error::Other(Error::BufferOverrun))
                    }
                }
                I2cEvent::End => {
                    self.state = State::Idle;
                    Ok(Event::Write { reg: self.reg, value: Value(&[]) })
                }
                I2cEvent::Restart => {
                    self.state = State::Idle;
                    Err(nb::Error::WouldBlock)
                }
                _ => defmt::unreachable!(),
            },
            State::Receiving => match self.i2c_event()? {
                I2cEvent::Read(value) => {
                    if self.pos < self.buf.len() {
                        self.buf[self.pos] = value;
                        self.pos += 1;
                        Err(nb::Error::WouldBlock)
                    } else {
                        Err(nb::Error::Other(Error::BufferOverrun))
                    }
                }
                I2cEvent::End | I2cEvent::Restart => {
                    self.state = State::Idle;
                    Ok(Event::Write { reg: self.reg, value: Value(&self.buf[..self.pos]) })
                }
                _ => defmt::unreachable!(),
            },
            State::StartSend => {
                Ok(Event::Read { reg: self.reg, value: Reader { state: &mut self.state, buf: &mut self.buf } })
            }
            State::Sending => {
                let to_send = self.buf.get(self.pos).cloned().unwrap_or_default();
                match self.i2c_event()? {
                    I2cEvent::Write(w) => {
                        w.write(to_send);
                        self.pos += 1;
                        Err(nb::Error::WouldBlock)
                    }
                    I2cEvent::End | I2cEvent::Restart => {
                        self.state = State::Idle;
                        Err(nb::Error::WouldBlock)
                    }
                    _ => defmt::unreachable!(),
                }
            }
            State::Clear => {
                match self.i2c_event()? {
                    I2cEvent::End => {
                        self.state = State::Idle;
                    }
                    I2cEvent::Write(w) => {
                        w.write(0);
                    }
                    _ => (),
                }
                Err(nb::Error::WouldBlock)
            }
        }
    }
}

/// I2C register slave event.
pub enum Event<'a, const BUFFER: usize> {
    /// Write register.
    Write {
        /// Register.
        reg: u8,
        /// Register value written by I2C master.
        value: Value<'a>,
    },
    /// Read register.
    Read {
        /// Register.
        reg: u8,
        /// Register value I2C master will read.
        value: Reader<'a, BUFFER>,
    },
}

/// The value written into an I2C register.
#[derive(Clone, Copy)]
pub struct Value<'a>(&'a [u8]);

impl<'a> Deref for Value<'a> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.0
    }
}

impl<'a> Value<'a> {
    fn get_or_zero(&self, idx: usize) -> u8 {
        self.get(idx).cloned().unwrap_or_default()
    }

    /// Gets the register value as an u8.
    ///
    /// Missing bytes are treated as zero and superflous bytes are ignored.
    pub fn as_u8(&self) -> u8 {
        self.get_or_zero(0)
    }

    /// Gets the register value as an u16 with LSB first.
    ///
    /// Missing bytes are treated as zero and superflous bytes are ignored.
    pub fn as_u16(&self) -> u16 {
        let b = self.get_or_zero(0) as u16;
        let a = self.get_or_zero(1) as u16;
        (a << 8) | b
    }

    /// Gets the register value as an u32 with LSB first.
    ///
    /// Missing bytes are treated as zero and superflous bytes are ignored.
    pub fn as_u32(&self) -> u32 {
        let d = self.get_or_zero(0) as u32;
        let c = self.get_or_zero(1) as u32;
        let b = self.get_or_zero(2) as u32;
        let a = self.get_or_zero(3) as u32;
        (a << 24) | (b << 16) | (c << 8) | d
    }

    /// Gets the register value as an u64 with LSB first.
    ///
    /// Missing bytes are treated as zero and superflous bytes are ignored.
    pub fn as_u64(&self) -> u64 {
        let h = self.get_or_zero(0) as u64;
        let g = self.get_or_zero(1) as u64;
        let f = self.get_or_zero(2) as u64;
        let e = self.get_or_zero(3) as u64;
        let d = self.get_or_zero(4) as u64;
        let c = self.get_or_zero(5) as u64;
        let b = self.get_or_zero(6) as u64;
        let a = self.get_or_zero(7) as u64;
        (a << 56) | (b << 48) | (c << 40) | (d << 32) | (e << 24) | (f << 16) | (g << 8) | h
    }
}

/// Register reader.
pub struct Reader<'a, const BUFFER: usize> {
    state: &'a mut State,
    buf: &'a mut [u8; BUFFER],
}

impl<'a, const BUFFER: usize> Reader<'a, BUFFER> {
    /// Provides the register value.
    ///
    /// The length of the value must not exceed `BUFFER`.
    pub fn set(self, value: &[u8]) {
        defmt::assert!(value.len() <= BUFFER);
        defmt::debug!("I2C read value: {=[u8]:#x}", value);
        self.buf[..value.len()].copy_from_slice(value);
        self.buf[value.len()..].fill(0);
        *self.state = State::Sending;
    }

    /// Provides the register value, clipping the provided value as necessary.
    pub fn set_clipped(self, value: &[u8]) {
        if value.len() <= BUFFER {
            self.set(value)
        } else {
            self.set(&value[..BUFFER])
        }
    }

    /// Sets the register value to an u8.
    pub fn set_u8(self, value: u8) {
        self.set(&[value])
    }

    /// Sets the register value to an u16 with LSB first.
    pub fn set_u16(self, value: u16) {
        self.set(&[(value & 0xff) as u8, ((value >> 8) & 0xff) as u8]);
    }

    /// Sets the register value to an u32 with LSB first.
    pub fn set_u32(self, value: u32) {
        self.set(&[
            (value & 0xff) as u8,
            ((value >> 8) & 0xff) as u8,
            ((value >> 16) & 0xff) as u8,
            ((value >> 24) & 0xff) as u8,
        ]);
    }

    /// Sets the register value to an u64 with LSB first.
    pub fn set_u64(self, value: u64) {
        self.set(&[
            (value & 0xff) as u8,
            ((value >> 8) & 0xff) as u8,
            ((value >> 16) & 0xff) as u8,
            ((value >> 24) & 0xff) as u8,
            ((value >> 32) & 0xff) as u8,
            ((value >> 40) & 0xff) as u8,
            ((value >> 48) & 0xff) as u8,
            ((value >> 56) & 0xff) as u8,
        ]);
    }
}
