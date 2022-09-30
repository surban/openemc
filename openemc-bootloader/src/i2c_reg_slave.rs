//! I2C slave adapter using the register access model.

use crate::i2c_slave::{I2CReceiver, I2CSender, I2CSlave, I2CTransfer, RecvError, SendError};

/// I2C slave using the register access model.
pub struct I2CRegSlave {
    slave: I2CSlave,
    reg: u8,
}

impl I2CRegSlave {
    /// Initializes an I2C register slave.
    pub fn new(slave: I2CSlave) -> Self {
        defmt::debug!("I2CRegSlave: init");
        Self { slave, reg: 0 }
    }

    /// Accepts the next register transaction from the master, if there is currently a transaction pending.
    pub fn try_accept(&mut self) -> Option<I2CRegTransaction<'_>> {
        match self.slave.try_accept() {
            // Master writes register.
            Some(I2CTransfer::Recv(mut rx)) => {
                match rx.recv() {
                    Ok(reg) => {
                        defmt::debug!("I2CRegSlave: received register 0x{:02x}", reg);
                        self.reg = reg;

                        #[allow(clippy::manual_map)]
                        match rx.recv() {
                            // Master continues writing, thus this is a register write operation.
                            Ok(first_data) => {
                                defmt::debug!("I2CRegSlave: write start for register 0x{:02x}", self.reg);
                                Some(I2CRegTransaction::Write(I2CRegWriter {
                                    reg: self.reg,
                                    first_data: Some(first_data),
                                    rx,
                                }))
                            }
                            // Master stops, thus empty write.
                            Err(RecvError::Stop) => {
                                defmt::debug!("I2CRegSlave: empty write");
                                Some(I2CRegTransaction::Write(I2CRegWriter {
                                    reg: self.reg,
                                    first_data: None,
                                    rx,
                                }))
                            }
                            // Master switches to writing.
                            Err(RecvError::Send) => {
                                defmt::debug!("I2CRegSlave: switch to read");
                                None
                            }
                            Err(RecvError::Inactive) => {
                                defmt::unreachable!("cannot be inactive")
                            }
                        }
                    }
                    Err(_) => None,
                }
            }
            // Master reads from current register.
            Some(I2CTransfer::Send(tx)) => {
                defmt::debug!("I2CRegSlave: read start for register 0x{:02x}", self.reg);
                Some(I2CRegTransaction::Read(I2CRegReader { reg: self.reg, tx }))
            }
            _ => None,
        }
    }
}

/// An I2C register transaction.
pub enum I2CRegTransaction<'a> {
    /// Register read operation.
    Read(I2CRegReader<'a>),
    /// Register write operation.
    Write(I2CRegWriter<'a>),
}

impl<'a> I2CRegTransaction<'a> {
    /// The accessed register.
    #[allow(dead_code)]
    pub fn reg(&self) -> u8 {
        match self {
            Self::Read(r) => r.reg(),
            Self::Write(w) => w.reg(),
        }
    }
}

/// An I2C register read operation.
pub struct I2CRegReader<'a> {
    reg: u8,
    tx: I2CSender<'a>,
}

impl<'a> I2CRegReader<'a> {
    /// The accessed register.
    pub fn reg(&self) -> u8 {
        self.reg
    }

    /// Sends a byte of data.
    pub fn send(&mut self, data: u8) -> Result<(), SendError> {
        self.tx.send(data)
    }

    /// Sends an u8 without error checking.
    pub fn send_u8(&mut self, data: u8) {
        let _ = self.send(data);
    }

    /// Sends an u16 with LSB first without error checking.
    pub fn send_u16(&mut self, data: u16) {
        let _ = self.send((data & 0xff) as u8);
        let _ = self.send(((data >> 8) & 0xff) as u8);
    }

    /// Sends an u32 with LSB first without error checking.
    pub fn send_u32(&mut self, data: u32) {
        let _ = self.send((data & 0xff) as u8);
        let _ = self.send(((data >> 8) & 0xff) as u8);
        let _ = self.send(((data >> 16) & 0xff) as u8);
        let _ = self.send(((data >> 24) & 0xff) as u8);
    }

    /// Sends a slice of data.
    pub fn send_all(&mut self, data: &[u8]) -> Result<(), SendError> {
        for b in data {
            self.send(*b)?;
        }
        while self.send(0).is_ok() {}
        Ok(())
    }
}

/// An I2C register write operation.
pub struct I2CRegWriter<'a> {
    reg: u8,
    first_data: Option<u8>,
    rx: I2CReceiver<'a>,
}

impl<'a> I2CRegWriter<'a> {
    /// The accessed register.
    pub fn reg(&self) -> u8 {
        self.reg
    }

    /// Receives a byte of data.
    pub fn recv(&mut self) -> Option<u8> {
        match self.first_data.take() {
            Some(data) => Some(data),
            None => self.rx.recv().ok(),
        }
    }

    /// Receives an u8, returning zero when reading fails.
    pub fn recv_u8(&mut self) -> u8 {
        self.recv().unwrap_or_default()
    }

    /// Receives an u16 with LSB first, padding with zeros when reading fails.
    #[allow(dead_code)]
    pub fn recv_u16(&mut self) -> u16 {
        let b = self.recv_u8() as u16;
        let a = self.recv_u8() as u16;
        (a << 8) | b
    }

    /// Receives an u32 with LSB first, padding with zeros when reading fails.
    pub fn recv_u32(&mut self) -> u32 {
        let d = self.recv_u8() as u32;
        let c = self.recv_u8() as u32;
        let b = self.recv_u8() as u32;
        let a = self.recv_u8() as u32;
        (a << 24) | (b << 16) | (c << 8) | d
    }

    /// Receives all data and discards it.
    pub fn discard(mut self) {
        while self.recv().is_some() {}
    }
}
