//! I2C slave polling driver.

use stm32f1::stm32f103::Peripherals;

/// I2C receiver state.
#[derive(Clone, Copy)]
enum State {
    /// Idle, ready to receive request.
    Idle,
    /// Send zero for remaining bytes.
    SendClear,
    /// Receive remaining bytes.
    ReceiveClear,
}

/// I2C slave interface.
pub struct I2CSlave {
    dp: Peripherals,
    state: State,
    remap: bool,
}

static mut ACTIVE: bool = false;

impl I2CSlave {
    /// Initializes an I2C slave.
    ///
    /// `remap` controls which pins are used for I2C.
    /// If `remap` is false, then SCL is PB6 and SDA is PB7.
    /// If `remap` is true, then SCL is PB8 and SDA is PB9.
    ///
    /// # Panics
    /// Panics if another instance of this exists.
    pub fn new(addr: u8, clock_mhz: u8, remap: bool) -> Self {
        defmt::assert!(!unsafe { ACTIVE }, "I2C slave already active");
        unsafe { ACTIVE = true };

        defmt::debug!("I2CSlave: init");

        let dp = unsafe { Peripherals::steal() };

        // Enable necessary clocks.
        dp.RCC.apb2enr.modify(|_, w| w.iopben().enabled().afioen().enabled());
        dp.RCC.apb1enr.modify(|_, w| w.i2c1en().enabled());
        dp.RCC.apb1rstr.modify(|_, w| w.i2c1rst().reset());
        dp.RCC.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());

        // Configure I2C pins.
        if remap {
            dp.AFIO.mapr.modify(|_, w| w.i2c1_remap().set_bit());
            dp.GPIOB.crh.modify(|_, w| {
                w.cnf8().alt_open_drain().mode8().output50().cnf9().alt_open_drain().mode9().output50()
            });
        } else {
            dp.AFIO.mapr.modify(|_, w| w.i2c1_remap().clear_bit());
            dp.GPIOB.crl.modify(|_, w| {
                w.cnf6().alt_open_drain().mode6().output50().cnf7().alt_open_drain().mode7().output50()
            });
        }

        // Configure I2C controller.
        dp.I2C1.cr1.write(|w| w.swrst().reset());
        cortex_m::asm::nop();
        dp.I2C1.cr1.write(|w| w.swrst().not_reset());
        cortex_m::asm::nop();
        dp.I2C1.oar1.write(|w| w.add().bits((addr as u16) << 1));
        dp.I2C1.cr2.write(|w| unsafe { w.freq().bits(clock_mhz) });
        dp.I2C1.cr1.write(|w| w.pe().enabled());
        dp.I2C1.cr1.modify(|_, w| w.ack().ack());

        Self { dp, state: State::Idle, remap }
    }

    /// Accepts the next transfer from the master, if there is currently a transfer pending.
    pub fn try_accept(&mut self) -> Option<I2CTransfer<'_>> {
        let sr1 = self.dp.I2C1.sr1.read();

        match self.state {
            State::Idle => {
                // Wait for address match.
                if sr1.addr().is_match() {
                    defmt::debug!("I2CSlave: address matched");
                    self.dp.I2C1.sr1.read();

                    let sr2 = self.dp.I2C1.sr2.read();
                    if sr2.tra().bit_is_set() {
                        defmt::debug!("I2CSlave: send start");
                        self.state = State::SendClear;
                        Some(I2CTransfer::Send(I2CSender { active: true, slave: self }))
                    } else {
                        defmt::debug!("I2CSlave: receive start");
                        self.dp.I2C1.cr1.modify(|_, w| w.ack().ack());
                        self.state = State::ReceiveClear;
                        Some(I2CTransfer::Recv(I2CReceiver { active: true, slave: self }))
                    }
                } else {
                    None
                }
            }

            State::SendClear => {
                // Check for send end by acknowledgement failure.
                if sr1.af().is_failure() {
                    defmt::debug!("I2CSlave: send end");
                    self.dp.I2C1.sr1.modify(|_, w| w.af().no_failure());
                    self.state = State::Idle;
                } else if sr1.tx_e().is_empty() {
                    defmt::debug!("I2CSlave: sending zero");
                    self.dp.I2C1.dr.write(|w| w.dr().bits(0));
                }

                None
            }

            State::ReceiveClear => {
                // Check for receive end by stop.
                if sr1.stopf().is_stop() {
                    defmt::debug!("I2CSlave: receive end");
                    self.dp.I2C1.sr1.read();
                    self.dp.I2C1.cr1.modify(|_, w| w.pe().enabled().ack().ack());
                    self.state = State::Idle;
                } else if sr1.rx_ne().is_not_empty() && sr1.btf().is_finished() {
                    defmt::debug!("I2CSlave: dropping received data");
                    self.dp.I2C1.dr.read();
                }

                None
            }
        }
    }
}

impl Drop for I2CSlave {
    fn drop(&mut self) {
        defmt::debug!("I2CSlave: drop");

        // Disable I2C controller.
        self.dp.I2C1.cr1.write(|w| w.pe().disabled());

        // Deconfigure I2C pins.
        if self.remap {
            self.dp.GPIOB.crh.modify(|_, w| w.cnf8().open_drain().cnf9().open_drain());
        } else {
            self.dp.GPIOB.crl.modify(|_, w| w.cnf6().open_drain().cnf7().open_drain());
        }
        self.dp.AFIO.mapr.modify(|_, w| w.i2c1_remap().clear_bit());

        // Disable I2C clock.
        self.dp.RCC.apb1enr.modify(|_, w| w.i2c1en().disabled());

        unsafe { ACTIVE = false };
    }
}

/// An I2C transfer.
pub enum I2CTransfer<'a> {
    /// An outgoing transfer.
    Send(I2CSender<'a>),
    /// An incoming transfer.
    Recv(I2CReceiver<'a>),
}

/// Sends I2C data.
pub struct I2CSender<'a> {
    active: bool,
    slave: &'a mut I2CSlave,
}

/// Sending failed.
pub struct SendError;

impl<'a> I2CSender<'a> {
    /// Sends a byte.
    pub fn send(&mut self, data: u8) -> Result<(), SendError> {
        if self.active {
            loop {
                let sr1 = self.slave.dp.I2C1.sr1.read();
                //defmt::debug!("send sr1: 0x{:04x}", sr1.bits());
                if sr1.af().is_failure() {
                    defmt::debug!("I2CSender: send end");
                    self.slave.dp.I2C1.sr1.modify(|_, w| w.af().no_failure());
                    self.slave.state = State::Idle;
                    self.active = false;
                    return Err(SendError);
                } else if sr1.tx_e().is_empty() {
                    defmt::debug!("I2CSender: sent 0x{:02x}", data);
                    self.slave.dp.I2C1.dr.write(|w| w.dr().bits(data));
                    return Ok(());
                }
            }
        } else {
            Err(SendError)
        }
    }
}

/// Receives I2C data.
pub struct I2CReceiver<'a> {
    active: bool,
    slave: &'a mut I2CSlave,
}

/// Receive error.
pub enum RecvError {
    /// Stop condition.
    Stop,
    /// Switch to send.
    Send,
    /// Receive failed before.
    Inactive,
}

impl<'a> I2CReceiver<'a> {
    /// Receives a byte.
    /// Returns `None` when no more bytes are available.
    pub fn recv(&mut self) -> Result<u8, RecvError> {
        if self.active {
            loop {
                let sr1 = self.slave.dp.I2C1.sr1.read();
                //defmt::debug!("recv sr1: 0x{:04x}", sr1.bits());
                if sr1.rx_ne().is_not_empty() {
                    let data = self.slave.dp.I2C1.dr.read().dr().bits();
                    defmt::debug!("I2CReceiver: received 0x{:02x}", data);
                    return Ok(data);
                } else if sr1.stopf().is_stop() || sr1.tx_e().is_empty() {
                    self.slave.dp.I2C1.sr1.read();
                    self.slave.dp.I2C1.cr1.modify(|_, w| w.pe().enabled().ack().ack());
                    self.slave.state = State::Idle;
                    self.active = false;
                    if sr1.tx_e().is_empty() {
                        defmt::debug!("I2CReceiver: receive switch to send");
                        for _ in 0..100 {
                            self.slave.dp.I2C1.sr1.read();
                            cortex_m::asm::nop();
                        }
                        return Err(RecvError::Send);
                    } else {
                        defmt::debug!("I2CReceiver: receive end");
                        return Err(RecvError::Stop);
                    }
                }
            }
        } else {
            Err(RecvError::Inactive)
        }
    }
}
