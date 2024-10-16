//
// OpenEMC firmware for embedded controllers
// Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

//! STUSB4500 USB PD controller driver.

mod nvm;

use core::{marker::PhantomData, mem, mem::size_of};
use defmt::Format;
use embedded_hal::blocking::i2c;
use heapless::Vec;
use systick_monotonic::*;

use crate::{app::monotonics, i2c_master::I2cError, Duration, Instant};

use super::PowerSupply;
pub use nvm::StUsb4500Nvm;

/// STUSB4500 error.
#[derive(Clone, Format, PartialEq, Eq)]
pub enum Error {
    /// I2C communication error.
    I2c(I2cError),
    /// Device responded with wrong id.
    WrongId,
    /// Timeout waiting for device.
    Timeout,
    /// Verification of written data failed.
    VerifyFailed,
}

/// STUSB4500 result.
pub type Result<T> = core::result::Result<T, Error>;

/// Voltage.
#[derive(Default, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct Voltage(pub u16);

impl Format for Voltage {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{} mV", self.to_mv())
    }
}

impl Voltage {
    /// Returns the voltage in mV.
    pub const fn to_mv(self) -> u32 {
        self.0 as u32 * 50
    }

    /// Creates the voltage from mV.
    pub const fn from_mv(mv: u32) -> Self {
        Self((mv / 50) as u16)
    }
}

/// Current.
#[derive(Default, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct Current(pub u16);

impl Format for Current {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{} mA", self.to_ma())
    }
}

impl Current {
    /// Returns the current in mA.
    pub const fn to_ma(self) -> u32 {
        self.0 as u32 * 10
    }

    /// Creates the current from mA.
    pub const fn from_ma(ma: u32) -> Self {
        Self((ma / 10) as u16)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ResetState {
    None,
    PinResetHigh(Instant),
    PinResetLow(Instant),
    RegisterReset(Instant),
    Off,
    OnWait { since: Instant, pin_reset: bool },
    Failed,
}

impl ResetState {
    pub fn is_resetting(&self) -> bool {
        matches!(
            self,
            Self::PinResetHigh(_) | Self::PinResetLow(_) | Self::RegisterReset(_) | Self::OnWait { .. }
        )
    }
}

/// STUSB4500 USB PD controller instance.
///
/// [`alert`](Self::alert) must be called when the ALERT pin goes low.
/// [`periodic`](Self::periodic) should be called approximately every second.
///
/// Call [`report`](Self::report) to obtain the negotiated voltage and currently available maximum current.
pub struct StUsb4500<I2C> {
    addr: u8,
    initial_pdo: FixedSinkPdo,
    maximum_voltage: Voltage,
    port_status: PortStatus,
    monitoring_status: MonitoringStatus,
    cc_status: CcStatus,
    type_c_status: TypeCStatus,
    prt_status: PrtStatus,
    hw_fault_status: HwFaultStatus,
    rdo: Option<Rdo>,
    sink_pdos: [SinkPdo; 3],
    active_sink_pdos: u8,
    supply_pdos: Vec<SupplyPdo, 7>,
    last_supply_pdos: Vec<SupplyPdo, 7>,
    snk_ready_since: Option<Instant>,
    fsm_attached_since: Option<Instant>,
    last_fsm_reset: Option<Instant>,
    reset: ResetState,
    last_pin_reset: Option<Instant>,
    last_register_reset: Option<Instant>,
    last_soft_reset: Option<Instant>,
    best_pdo_requested: bool,
    report: PowerSupply,
    started: Instant,
    reset_prohibited_until: Option<Instant>,
    reset_queued: bool,
    _i2c: PhantomData<I2C>,
}

impl<I2C> StUsb4500<I2C>
where
    I2C: i2c::Write<i2c::SevenBitAddress> + i2c::WriteRead<i2c::SevenBitAddress>,
    <I2C as i2c::Write<i2c::SevenBitAddress>>::Error: Into<I2cError>,
    <I2C as i2c::WriteRead<i2c::SevenBitAddress>>::Error: Into<I2cError>,
{
    /// Read I2C register(s).
    fn read(&self, i2c: &mut I2C, reg: u8, len: usize) -> Result<Vec<u8, 32>> {
        let mut buf: Vec<u8, 32> = Vec::new();
        defmt::unwrap!(buf.resize_default(len));
        i2c.write_read(self.addr, &[reg], &mut buf).map_err(|err| Error::I2c(err.into()))?;
        Ok(buf)
    }

    /// Write I2C register(s).
    fn write(&self, i2c: &mut I2C, reg: u8, data: &[u8]) -> Result<()> {
        let mut buf: Vec<u8, 32> = Vec::new();
        defmt::unwrap!(buf.push(reg));
        buf.extend(data.iter().cloned());
        i2c.write(self.addr, &buf).map_err(|err| Error::I2c(err.into()))
    }

    /// Creates a new STUSB4500 controller instance.
    ///
    /// `addr` specifies the I2C address.
    ///
    /// `initial_pdo` specifies the PDO to present when initiating negotiation.
    /// It must have a voltage of 5 V.
    ///
    /// `maximum_voltage` is the maximum voltage to negotiate.
    /// The supply PDO with the highest voltage at or below `maximum_voltage` will be chosen.
    pub fn new(addr: u8, initial_pdo: &super::FixedSinkPdo, maximum_voltage_mv: u32) -> Self {
        let initial_pdo = FixedSinkPdo::from(initial_pdo.clone());
        defmt::assert_eq!(initial_pdo.voltage.to_mv(), 5000);

        defmt::info!("STUSB4500 at 0x{:x} created", addr);

        Self {
            addr,
            initial_pdo,
            maximum_voltage: Voltage::from_mv(maximum_voltage_mv),
            port_status: Default::default(),
            monitoring_status: Default::default(),
            cc_status: Default::default(),
            type_c_status: Default::default(),
            prt_status: Default::default(),
            hw_fault_status: Default::default(),
            rdo: None,
            sink_pdos: [SinkPdo::default(), SinkPdo::default(), SinkPdo::default()],
            active_sink_pdos: 0,
            supply_pdos: Vec::new(),
            last_supply_pdos: Vec::new(),
            snk_ready_since: None,
            fsm_attached_since: None,
            last_fsm_reset: None,
            reset: ResetState::None,
            last_pin_reset: None,
            last_register_reset: None,
            last_soft_reset: None,
            best_pdo_requested: false,
            report: Default::default(),
            started: monotonics::now(),
            reset_prohibited_until: None,
            reset_queued: false,
            _i2c: PhantomData,
        }
    }

    /// Returns the current power supply report and marks it as seen.
    pub fn report(&self) -> &PowerSupply {
        &self.report
    }

    /// Initiates a pin reset.
    fn start_pin_reset(&mut self) {
        if self.reset != ResetState::None {
            return;
        }

        defmt::info!("STUSB4500 pin reset");
        self.reset = ResetState::PinResetHigh(monotonics::now());
        self.reset_prohibited_until = None;
        self.reset_queued = false;
    }

    /// Initiates a register reset.
    fn start_register_reset(&mut self, i2c: &mut I2C, power_on: bool) -> Result<()> {
        if self.reset != ResetState::None {
            return Ok(());
        }

        if power_on {
            defmt::info!("Skipping STUSB4500 register reset due to power on");
        } else if self.reset_prohibited_until.map(|until| until >= monotonics::now()).unwrap_or_default() {
            defmt::info!("Queueing STUSB4500 register reset because it is currently prohibited");
            self.reset_queued = true;
        } else {
            defmt::info!("STUSB4500 register reset");
            self.write(i2c, REG_RESET, &[1])?;
            self.reset_queued = false;
        }

        let regs = self.read(i2c, REG_ALERT_STATUS_1, 12)?;
        defmt::debug!("STUSB4500 alert registers: {:x}", regs.as_slice());

        self.supply_pdos.clear();
        self.snk_ready_since = None;
        self.fsm_attached_since = None;
        self.best_pdo_requested = false;
        self.rdo = None;
        self.reset = ResetState::RegisterReset(monotonics::now());
        Ok(())
    }

    /// Checks the id of the STUSB4500.
    fn check_id(&self, i2c: &mut I2C) -> Result<()> {
        defmt::debug!("read STUSB4500 id");
        let id = self.read(i2c, REG_ID, 1)?;
        defmt::debug!("STUSB4500 id at 0x{:x} is 0x{:x}", self.addr, id[0]);

        if IDS.contains(&id[0]) {
            Ok(())
        } else {
            defmt::warn!("STUSB4500 id 0x{:x} is wrong", id[0]);
            Err(Error::WrongId)
        }
    }

    /// Completes a reset.
    ///
    /// Returns whether a reset is in progress.
    fn resetting(&mut self, i2c: &mut I2C) -> Result<bool> {
        let reset_duration: Duration = 500u64.millis();

        match self.reset {
            ResetState::None => Ok(false),
            ResetState::Failed => {
                self.reset = ResetState::None;
                self.start_pin_reset();
                Ok(true)
            }
            ResetState::Off => {
                self.reset = ResetState::OnWait { since: monotonics::now(), pin_reset: false };
                Ok(true)
            }
            ResetState::OnWait { since, pin_reset } if monotonics::now() - since >= reset_duration => {
                self.reset = ResetState::None;
                self.check_id(i2c)?;
                self.start_register_reset(i2c, !pin_reset)?;
                Ok(true)
            }
            ResetState::PinResetHigh(since) if monotonics::now() - since >= reset_duration => {
                defmt::debug!("STUSB4500 pin reset low");
                self.reset = ResetState::PinResetLow(monotonics::now());
                Ok(true)
            }
            ResetState::PinResetLow(since) if monotonics::now() - since >= reset_duration => {
                defmt::debug!("STUSB4500 pin reset done");
                self.last_pin_reset = Some(monotonics::now());
                self.reset = ResetState::OnWait { since: monotonics::now(), pin_reset: true };
                Ok(true)
            }
            ResetState::RegisterReset(since) if monotonics::now() - since >= reset_duration => {
                defmt::info!("STUSB4500 register reset done");

                // Disable reset.
                self.write(i2c, REG_RESET, &[0])?;

                self.init_alert_mask(i2c)?;
                self.init_sink_pdos(i2c)?;

                self.last_register_reset = Some(monotonics::now());
                self.reset = ResetState::None;
                Ok(false)
            }
            _ => {
                defmt::trace!("reset in progress");
                Ok(true)
            }
        }
    }

    /// Send soft reset message to power source.
    fn soft_reset(&mut self, i2c: &mut I2C) -> Result<()> {
        const CMD_SEND_MESSAGE: u8 = 0x26;
        const CMD_SOFT_RESET_MESSAGE_TYPE: u8 = 0x0d;

        defmt::info!("STUSB4500 USB PD soft reset");

        self.write(i2c, REG_TX_HEADER, &[CMD_SOFT_RESET_MESSAGE_TYPE, 0])?;
        self.write(i2c, REG_CMD_CTRL, &[CMD_SEND_MESSAGE])?;

        self.last_soft_reset = Some(monotonics::now());
        Ok(())
    }

    /// Initializes the alert mask.
    fn init_alert_mask(&mut self, i2c: &mut I2C) -> Result<()> {
        let mask = AlertStatus {
            phy: true,
            prt: false,
            reserved: true,
            pd_type_c: true,
            hw_fault: false,
            monitoring: true,
            cc_detection: false,
            hard_reset: true,
        };
        self.write(i2c, REG_ALERT_STATUS_1_MASK, &[mask.mask()])?;
        Ok(())
    }

    /// Initializes the sink PDOs to 5V with 100mA.
    fn init_sink_pdos(&mut self, i2c: &mut I2C) -> Result<()> {
        // Read sink PDOs.
        (self.sink_pdos, self.active_sink_pdos) = self.read_sink_pdos(i2c)?;
        defmt::debug!("NVM sink PDOs ({} active): {:?}", self.active_sink_pdos, &self.sink_pdos);

        // Set initial PDO.
        self.sink_pdos[0] = SinkPdo::Fixed(self.initial_pdo.clone());
        self.active_sink_pdos = 1;
        self.write_sink_pdos(i2c, &self.sink_pdos, self.active_sink_pdos)?;
        defmt::debug!("Initial sink PDOs ({} active): {:?}", self.active_sink_pdos, &self.sink_pdos);

        Ok(())
    }

    /// Generates and activates a sink PDO with best match to the supply PDOs.
    fn activate_best_supply_pdo(&mut self, i2c: &mut I2C) -> Result<()> {
        let possible_pdos = self.supply_pdos.iter().filter_map(|pdo| match pdo {
            SupplyPdo::Fixed(fixed) if fixed.voltage <= self.maximum_voltage => Some(fixed),
            _ => None,
        });
        let best_pdo = possible_pdos.max_by_key(|pdo| pdo.voltage);

        match best_pdo {
            Some(pdo) => {
                defmt::info!("Matched best USB supply PDO: {:?}", &pdo);

                let new_sink_pdo = SinkPdo::Fixed(FixedSinkPdo {
                    voltage: pdo.voltage,
                    operating_current: pdo.max_operating_current,
                    ..Default::default()
                });
                let reset_req = self.sink_pdos[1] != new_sink_pdo || self.active_sink_pdos != 2;

                defmt::debug!("Setting and activating sink PDO 2: {:?}", &new_sink_pdo);
                self.sink_pdos[1] = new_sink_pdo;
                self.active_sink_pdos = 2;
                self.write_sink_pdos(i2c, &self.sink_pdos, self.active_sink_pdos)?;

                if reset_req {
                    defmt::info!("Issuing USB-PD soft reset to renegotiate");
                    self.soft_reset(i2c)?;
                }
            }
            None => {
                defmt::warn!("cannot find matching supply PDO");
            }
        }

        Ok(())
    }

    /// Reads the last received message header.
    fn read_rx_header(&self, i2c: &mut I2C) -> Result<(MsgHeader, usize)> {
        let buf = self.read(i2c, REG_RX_HEADER, 2)?;
        let header = MsgHeader::parse((buf[0] as u16) | (buf[1] as u16) << 8);

        let data_len = self.read(i2c, REG_RX_BYTE_CNT, 1)?;

        Ok((header, data_len[0] as usize))
    }

    /// Reads the last received messge data.
    fn read_data_objs(&self, i2c: &mut I2C, cnt: usize) -> Result<Vec<u32, 7>> {
        let buf = self.read(i2c, REG_RX_DATA_OBJ, size_of::<u32>() * cnt)?;

        let mut data = Vec::new();
        for n in 0..cnt {
            defmt::unwrap!(
                #[allow(clippy::identity_op)]
                data.push(
                    (buf[size_of::<u32>() * n + 0] as u32) << 0
                        | (buf[size_of::<u32>() * n + 1] as u32) << 8
                        | (buf[size_of::<u32>() * n + 2] as u32) << 16
                        | (buf[size_of::<u32>() * n + 3] as u32) << 24
                )
            );
        }

        Ok(data)
    }

    /// Reads the sink PDOs.
    fn read_sink_pdos(&self, i2c: &mut I2C) -> Result<([SinkPdo; 3], u8)> {
        let buf = self.read(i2c, REG_DPM_SNK_PDO1_0, size_of::<u32>() * 3)?;
        let active = self.read(i2c, REG_DPM_PDO_NUMB, 1)?[0] & 0b111;

        let mut data = [0; 3];
        #[allow(clippy::identity_op)]
        for n in 0..3 {
            data[n] = (buf[size_of::<u32>() * n + 0] as u32) << 0
                | (buf[size_of::<u32>() * n + 1] as u32) << 8
                | (buf[size_of::<u32>() * n + 2] as u32) << 16
                | (buf[size_of::<u32>() * n + 3] as u32) << 24;
        }
        let pdos = [SinkPdo::parse(data[0]), SinkPdo::parse(data[1]), SinkPdo::parse(data[2])];

        Ok((pdos, active))
    }

    /// Writes the sink PDOs.
    fn write_sink_pdos(&self, i2c: &mut I2C, pdos: &[SinkPdo; 3], active: u8) -> Result<()> {
        defmt::assert!(0 < active && active <= 3);

        let mut buf = [0; size_of::<u32>() * 3];
        #[allow(clippy::identity_op)]
        for (n, pdo) in pdos.iter().enumerate() {
            let pdo = pdo.generate();
            buf[size_of::<u32>() * n + 0] = (pdo >> 0) as u8;
            buf[size_of::<u32>() * n + 1] = (pdo >> 8) as u8;
            buf[size_of::<u32>() * n + 2] = (pdo >> 16) as u8;
            buf[size_of::<u32>() * n + 3] = (pdo >> 24) as u8;
        }

        self.write(i2c, REG_DPM_SNK_PDO1_0, &buf)?;
        self.write(i2c, REG_DPM_PDO_NUMB, &[active & 0b111])?;

        Ok(())
    }

    /// Reads the RDO.
    fn read_rdo(&self, i2c: &mut I2C) -> Result<Rdo> {
        let buf = self.read(i2c, REG_RDO_STATUS_0, size_of::<u32>())?;
        Ok(Rdo::parse((buf[0] as u32) | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24))
    }

    /// Reads the hardware fault status.
    fn update_hw_fault_status(&mut self, i2c: &mut I2C) -> Result<()> {
        let buf = self.read(i2c, REG_CC_HW_FAULT_STATUS_0, 2)?;
        let hw_fault_status = HwFaultStatus::parse(buf[1]);
        if self.hw_fault_status != hw_fault_status {
            defmt::info!("Hardware fault status: {:?}", &hw_fault_status);
            self.hw_fault_status = hw_fault_status;
        }
        Ok(())
    }

    /// Reads the monitoring status.
    fn update_monitoring_status(&mut self, i2c: &mut I2C) -> Result<()> {
        let buf = self.read(i2c, REG_MONITORING_STATUS_0, 2)?;
        let buf2 = self.read(i2c, REG_VBUS_CTRL, 1)?;
        let monitoring_status = MonitoringStatus::parse(buf[0], buf[1], buf2[0]);
        if self.monitoring_status != monitoring_status {
            defmt::info!("Monitoring status: {:?}", monitoring_status);
            self.monitoring_status = monitoring_status;
        }
        Ok(())
    }

    /// Generates the power supply report.
    fn generate_report(&mut self) {
        // Check for active USB PD contract.
        if let Some(rdo) = self.rdo.as_ref() {
            if 1 <= rdo.object_pos && rdo.object_pos as usize <= self.supply_pdos.len() {
                let active_pdo = &self.supply_pdos[(rdo.object_pos - 1) as usize];
                defmt::debug!("Active supply PDO: {:?}", active_pdo);

                if let SupplyPdo::Fixed(fixed) = active_pdo {
                    self.report = PowerSupply::PdoContract {
                        voltage_mv: fixed.voltage.to_mv(),
                        max_current_ma: fixed.max_operating_current.to_ma(),
                        communication: fixed.communication,
                    };
                    return;
                } else {
                    defmt::debug!("Active supply PDO is not of fixed type");
                }
            } else {
                defmt::debug!("No supply PDO active");
            }
        }

        // Check for CC pins.
        self.report = match (&self.cc_status.cc1_state, &self.cc_status.cc2_state) {
            (CcState::Default, _) | (_, CcState::Default) => PowerSupply::UsbSdp,
            (CcState::Power15, _) | (_, CcState::Power15) => PowerSupply::CcPins5V1500mA,
            (CcState::Power30, _) | (_, CcState::Power30) => PowerSupply::CcPins5V3000mA,
            _ => PowerSupply::Disconnected,
        };

        // If CC pins do not work, we can still try USB default power.
        if self.type_c_status.typec_fsm_state == TypeCFsmState::AttachWaitSink
            && self.report == PowerSupply::Disconnected
        {
            self.report = PowerSupply::UsbSdp;
        }

        // Inhibit high currents during negotiation phase.
        let init_period: Duration = 1u64.secs();
        let grace_period: Duration = 10u64.secs();
        let now = monotonics::now();
        match [self.last_pin_reset, self.last_register_reset, self.last_soft_reset].into_iter().flatten().max() {
            _ if self.reset.is_resetting() => self.report = PowerSupply::Negotiating,
            Some(latest) if now - latest <= init_period => self.report = PowerSupply::Negotiating,
            Some(latest) if now - latest <= grace_period => match self.report {
                PowerSupply::CcPins5V1500mA | PowerSupply::CcPins5V3000mA => {
                    self.report = PowerSupply::Negotiating;
                }
                _ => (),
            },
            _ => (),
        }
    }

    fn do_alert(&mut self, i2c: &mut I2C) -> Result<()> {
        defmt::trace!("STUSB4500 alert");

        // Read alert status register.
        let buf = self.read(i2c, REG_ALERT_STATUS_1, 2)?;
        let alert = AlertStatus::parse(buf[0] & !buf[1]);
        defmt::trace!("alert status: {:?}", alert);

        if alert.any() {
            if alert.prt {
                let buf = self.read(i2c, REG_PRT_STATUS, 1)?;
                self.prt_status = PrtStatus::parse(buf[0]);
                defmt::debug!("PRT status: {:?}", &self.prt_status);

                if self.prt_status.msg_received {
                    let (header, len) = self.read_rx_header(i2c)?;
                    defmt::trace!("message of length {} bytes received: {:?}", len, &header);

                    match header.msg_type {
                        MsgType::Data(DataMsg::SrcCaps) => {
                            if len == header.num_data_objs as usize * size_of::<u32>() {
                                defmt::trace!("reading {} PDOs", header.num_data_objs);
                                self.supply_pdos = self
                                    .read_data_objs(i2c, header.num_data_objs as usize)?
                                    .into_iter()
                                    .map(SupplyPdo::parse)
                                    .collect();
                                defmt::info!("USB PDOs received: {:?}", self.supply_pdos.as_slice());
                            } else {
                                defmt::warn!("USB PD message length does not match number of PDOs");
                            }
                        }
                        MsgType::Control(ControlMsg::PsReady) => {
                            defmt::debug!("PS ready");
                        }
                        _ => (),
                    }
                }
            }

            let buf = self.read(i2c, REG_CC_STATUS, 1)?;
            self.cc_status = CcStatus::parse(buf[0]);
            defmt::debug!("CC status: {:?}", &self.cc_status);

            if alert.cc_detection {
                let buf = self.read(i2c, REG_PORT_STATUS_0, 2)?;
                self.port_status = PortStatus::parse(buf[1]);
                defmt::debug!("Port status: {:?}", &self.port_status);

                if !self.port_status.cc_attach_state {
                    self.supply_pdos.clear();
                }
            }

            if alert.monitoring {
                self.update_monitoring_status(i2c)?;
            }

            if alert.hw_fault {
                self.update_hw_fault_status(i2c)?;
            }

            self.generate_report();
        }

        defmt::trace!("STUSB4500 alert done");
        Ok(())
    }

    /// Handles an alert generated by the device.
    ///
    /// This must be called immediately from the IRQ handler when the ALERT pin goes low and
    /// is timing critical.
    pub fn alert(&mut self, i2c: &mut I2C, attach_pin_level: bool) -> Result<()> {
        self.do_call(i2c, attach_pin_level, Self::do_alert)
    }

    fn do_periodic(&mut self, i2c: &mut I2C) -> Result<()> {
        const PE_SNK_READY: u8 = 0x18;

        defmt::trace!("STUSB4500 periodic");

        // Perform queued reset, if necessary.
        if !self.reset_prohibited_until.map(|until| until >= monotonics::now()).unwrap_or_default()
            && self.reset_queued
        {
            defmt::info!("Executing queued STUSB4500 register reset");
            self.reset_prohibited_until = None;
            self.start_register_reset(i2c, false)?;
        }

        // Read monitoring status.
        self.update_monitoring_status(i2c)?;
        self.update_hw_fault_status(i2c)?;

        // Read CC status.
        let buf = self.read(i2c, REG_CC_STATUS, 1)?;
        self.cc_status = CcStatus::parse(buf[0]);
        defmt::debug!("CC status: {:?}", &self.cc_status);

        if self.port_status.cc_attach_state {
            defmt::debug!("CC is attached!");
            self.fsm_attached_since = None;
            self.last_fsm_reset = None;

            let pe_state = self.read(i2c, REG_PE_FSM, 1)?;
            defmt::trace!("PE state: {:x}", pe_state[0]);

            // Check if policy engine is ready.
            if pe_state[0] == PE_SNK_READY {
                defmt::trace!("Policy engine is ready");

                // Perform undocumented arcane magic.
                let mut buf = self.read(i2c, REG_MONITORING_CTRL_0, 1)?;
                buf[0] &= 0x24;
                self.write(i2c, REG_MONITORING_CTRL_0, &buf)?;

                // Read RDO.
                let rdo = self.read_rdo(i2c)?;
                if self.rdo.as_ref() != Some(&rdo) {
                    defmt::info!("Received new USB PD RDO: {:?}", &rdo);
                    self.rdo = Some(rdo);
                }

                if self.supply_pdos.is_empty() {
                    let pdo_timeout: Duration = 5u64.secs();
                    match self.snk_ready_since {
                        Some(since)
                            if self.supply_pdos.is_empty() && monotonics::now() - since >= pdo_timeout =>
                        {
                            defmt::info!("No USB PDOs received for too long after SNK_READY, issuing soft reset");
                            self.snk_ready_since = None;
                            self.soft_reset(i2c)?;
                        }
                        Some(_) => (),
                        None => self.snk_ready_since = Some(monotonics::now()),
                    }
                } else if self.last_supply_pdos != self.supply_pdos {
                    defmt::info!("Issuing soft reset to verify PDOs");
                    self.last_supply_pdos = mem::take(&mut self.supply_pdos);
                    self.snk_ready_since = None;
                    self.soft_reset(i2c)?;
                } else {
                    self.snk_ready_since = None;

                    if !self.best_pdo_requested {
                        self.activate_best_supply_pdo(i2c)?;
                        self.best_pdo_requested = true;
                    }
                }
            } else {
                self.snk_ready_since = None;
                self.best_pdo_requested = false;
            }
        } else {
            defmt::debug!("CC is not attached");
            self.snk_ready_since = None;
            self.best_pdo_requested = false;
            self.rdo = None;

            let buf = self.read(i2c, REG_TYPE_C_STATUS, 1)?;
            self.type_c_status = TypeCStatus::parse(buf[0]);
            defmt::debug!("Type C status: {:?}", &self.type_c_status);

            if [TypeCFsmState::AttachWaitSink, TypeCFsmState::AttachedSink]
                .contains(&self.type_c_status.typec_fsm_state)
            {
                let since = self.fsm_attached_since.get_or_insert_with(monotonics::now);
                let now = monotonics::now();
                let fsm_timeout: Duration = 2u64.secs();
                let fsm_prevent: Duration = 5u64.secs();
                if now - *since >= fsm_timeout {
                    match self.last_fsm_reset {
                        Some(last) if now - last <= fsm_prevent => (),
                        _ => {
                            defmt::info!(
                                "USB CC not attached for too long after FSM attach, issuing register reset"
                            );
                            self.fsm_attached_since = None;
                            self.start_register_reset(i2c, false)?;
                        }
                    }
                    self.last_fsm_reset = Some(now);
                }
            } else {
                self.fsm_attached_since = None;
            }
        }

        self.generate_report();

        // STUSB4500 sometimes needs a hard reset to detect a supply.
        let start_grace_period: Duration = 10u64.secs();
        let reset_interval: Duration = 60u64.secs();
        let now = monotonics::now();
        match (&self.report, self.last_pin_reset) {
            (PowerSupply::Unknown | PowerSupply::Disconnected, None)
                if now - self.started > start_grace_period =>
            {
                defmt::warn!("no supply connected, trying STUSB4500 pin reset");
                self.start_pin_reset();
            }
            (PowerSupply::Unknown | PowerSupply::Disconnected, Some(reset)) if now - reset > reset_interval => {
                defmt::debug!("no supply connected, trying STUSB4500 pin reset");
                self.start_pin_reset();
            }
            _ => (),
        }

        defmt::trace!("STUSB4500 periodic done");
        Ok(())
    }

    /// Call this periodically (approx. every second) to handle communication with the device.
    pub fn periodic(&mut self, i2c: &mut I2C, attach_pin_level: bool) -> Result<()> {
        self.do_call(i2c, attach_pin_level, Self::do_periodic)
    }

    fn do_call(
        &mut self, i2c: &mut I2C, attach_pin_level: bool, f: impl FnOnce(&mut Self, &mut I2C) -> Result<()>,
    ) -> Result<()> {
        match (self.reset, attach_pin_level) {
            (ResetState::Failed | ResetState::Off, false) => {
                self.reset = ResetState::Off;
                self.report = PowerSupply::Disconnected;
                Ok(())
            }
            (ResetState::None, false) => {
                defmt::info!("STUSB4500 disconnected");
                self.reset = ResetState::Off;
                self.report = PowerSupply::Disconnected;
                Ok(())
            }
            (ResetState::Failed | ResetState::Off, true) => {
                defmt::info!("STUSB4500 initializing");
                let res = self.resetting(i2c);
                if let Err(err) = &res {
                    defmt::warn!("STUSB4500 reset failed: {}", err);
                    self.reset = ResetState::Failed;
                }
                self.report = PowerSupply::Unknown;
                res.map(|_| ())
            }
            _ => match self.resetting(i2c) {
                Ok(false) => {
                    let res = f(self, i2c);
                    if let Err(err) = &res {
                        defmt::warn!("STUSB4500 failed: {}", err);
                        self.reset = ResetState::Failed;
                        self.report = PowerSupply::Unknown;
                    }
                    res
                }
                Ok(true) => Ok(()),
                Err(err) => {
                    defmt::warn!("STUSB4500 reset failed: {}", err);
                    self.reset = ResetState::Failed;
                    self.report = PowerSupply::Unknown;
                    Err(err)
                }
            },
        }
    }

    /// Whether the STUSB4500 is available.
    pub fn is_available(&self) -> bool {
        matches!(self.reset, ResetState::None)
    }

    /// The level that should be applied on the reset pin.
    pub fn reset_pin_level(&self) -> bool {
        matches!(self.reset, ResetState::PinResetHigh(_))
    }

    /// Initiates a reset.
    pub fn reset(&mut self) {
        defmt::info!("Initiating STUSB4500 reset by request");
        self.reset = ResetState::None;
        self.start_pin_reset();
    }

    /// Prohibits register reset until the specified time.
    pub fn prohibit_reset_until(&mut self, until: Instant) {
        defmt::info!("STUSB4500 reset prohibited");
        self.reset_prohibited_until = Some(until);
    }
}

// Register definitions.
const REG_ID: u8 = 0x2f;
const REG_RESET: u8 = 0x23;
const REG_ALERT_STATUS_1: u8 = 0x0b;
const REG_ALERT_STATUS_1_MASK: u8 = 0x0c;
const REG_TX_HEADER: u8 = 0x51;
const REG_CMD_CTRL: u8 = 0x1a;
const REG_PORT_STATUS_0: u8 = 0x0d;
const REG_MONITORING_STATUS_0: u8 = 0x0f;
const REG_CC_STATUS: u8 = 0x11;
const REG_CC_HW_FAULT_STATUS_0: u8 = 0x12;
const REG_TYPE_C_STATUS: u8 = 0x15;
const REG_PRT_STATUS: u8 = 0x16;
const REG_VBUS_CTRL: u8 = 0x27;
const REG_RX_BYTE_CNT: u8 = 0x30;
const REG_RX_HEADER: u8 = 0x31;
const REG_RX_DATA_OBJ: u8 = 0x33;
const REG_RDO_STATUS_0: u8 = 0x91;
const REG_PE_FSM: u8 = 0x29;
const REG_MONITORING_CTRL_0: u8 = 0x20;
const REG_DPM_PDO_NUMB: u8 = 0x70;
const REG_DPM_SNK_PDO1_0: u8 = 0x85;

/// STUSB4500 chip ids.
const IDS: [u8; 2] = [0x25, 0x21];

#[derive(Clone, Format, Default)]
struct AlertStatus {
    pub phy: bool,
    pub prt: bool,
    pub reserved: bool,
    pub pd_type_c: bool,
    pub hw_fault: bool,
    pub monitoring: bool,
    pub cc_detection: bool,
    pub hard_reset: bool,
}

impl AlertStatus {
    fn parse(status: u8) -> Self {
        Self {
            phy: status & (1 << 0) != 0,
            prt: status & (1 << 1) != 0,
            reserved: status & (1 << 2) != 0,
            pd_type_c: status & (1 << 3) != 0,
            hw_fault: status & (1 << 4) != 0,
            monitoring: status & (1 << 5) != 0,
            cc_detection: status & (1 << 6) != 0,
            hard_reset: status & (1 << 7) != 0,
        }
    }

    fn mask(&self) -> u8 {
        let mut mask = 0;

        if self.phy {
            mask |= 1 << 0
        }
        if self.prt {
            mask |= 1 << 1
        }
        if self.reserved {
            mask |= 1 << 2
        }
        if self.pd_type_c {
            mask |= 1 << 3
        }
        if self.hw_fault {
            mask |= 1 << 4
        }
        if self.monitoring {
            mask |= 1 << 5
        }
        if self.cc_detection {
            mask |= 1 << 6
        }
        if self.hard_reset {
            mask |= 1 << 7
        }

        mask
    }

    fn any(&self) -> bool {
        self.mask() != 0
    }
}

#[derive(Clone, Format, Default)]
struct TypeCStatus {
    reverse: bool,
    typec_fsm_state: TypeCFsmState,
}

#[derive(Clone, Format, Default, PartialEq, Eq)]
pub enum TypeCFsmState {
    #[default]
    UnattachedSink,
    AttachWaitSink,
    AttachedSink,
    DebugAccessorySink,
    TrySrc,
    UnattachedAccessory,
    AttachWaitAccessory,
    TypeCErrorRecovery,
    Other(u8),
}

impl TypeCStatus {
    fn parse(type_c_status: u8) -> Self {
        Self {
            reverse: type_c_status & (1 << 7) != 0,
            typec_fsm_state: match type_c_status & 0b11111 {
                0b00000 => TypeCFsmState::UnattachedSink,
                0b00001 => TypeCFsmState::AttachWaitSink,
                0b00010 => TypeCFsmState::AttachedSink,
                0b00011 => TypeCFsmState::DebugAccessorySink,
                0b01100 => TypeCFsmState::TrySrc,
                0b01101 => TypeCFsmState::UnattachedAccessory,
                0b01110 => TypeCFsmState::AttachWaitAccessory,
                0b10011 => TypeCFsmState::TypeCErrorRecovery,
                other => TypeCFsmState::Other(other),
            },
        }
    }
}

#[derive(Clone, Format, Default)]
struct PortStatus {
    cc_attach_state: bool,
    cc_vconn_supply_state: bool,
    cc_data_role: bool,
    cc_power_role: bool,
    start_up_power_mode: bool,
    cc_attach_mode: CcAttachMode,
}

#[derive(Clone, Format, Default)]
pub enum CcAttachMode {
    #[default]
    None,
    Sink,
    Debug,
    Other(u8),
}

impl PortStatus {
    fn parse(port_status_1: u8) -> Self {
        Self {
            cc_attach_state: port_status_1 & (1 << 0) != 0,
            cc_vconn_supply_state: port_status_1 & (1 << 1) != 0,
            cc_data_role: port_status_1 & (1 << 2) != 0,
            cc_power_role: port_status_1 & (1 << 3) != 0,
            start_up_power_mode: port_status_1 & (1 << 4) != 0,
            cc_attach_mode: match (port_status_1 >> 5) & 0b111 {
                0b000 => CcAttachMode::None,
                0b001 => CcAttachMode::Sink,
                0b011 => CcAttachMode::Debug,
                other => CcAttachMode::Other(other),
            },
        }
    }
}

#[derive(Clone, Format, Default, PartialEq, Eq)]
struct MonitoringStatus {
    vbus_low: bool,
    vbus_high: bool,
    vconn_valid: bool,
    vbus_valid_sink: bool,
    vbus_vsafe_0v: bool,
    vbus_ready: bool,
    vbus_en_sink: bool,
}

impl MonitoringStatus {
    fn parse(monitoring_status_0: u8, monitoring_status_1: u8, vbus_ctrl: u8) -> Self {
        Self {
            vbus_low: monitoring_status_0 & (1 << 4) != 0,
            vbus_high: monitoring_status_0 & (1 << 5) != 0,
            vconn_valid: monitoring_status_1 & (1 << 0) != 0,
            vbus_valid_sink: monitoring_status_1 & (1 << 1) != 0,
            vbus_vsafe_0v: monitoring_status_1 & (1 << 2) != 0,
            vbus_ready: monitoring_status_1 & (1 << 3) != 0,
            vbus_en_sink: vbus_ctrl & (1 << 1) != 0,
        }
    }
}

#[derive(Clone, Format, Default)]
struct CcStatus {
    cc1_state: CcState,
    cc2_state: CcState,
    connect_result: bool,
    looking_for_connection: bool,
}

impl CcStatus {
    fn parse(cc_status: u8) -> Self {
        Self {
            cc1_state: CcState::parse(cc_status),
            cc2_state: CcState::parse(cc_status >> 2),
            connect_result: cc_status & (1 << 4) != 0,
            looking_for_connection: cc_status & (1 << 5) != 0,
        }
    }
}

#[derive(Clone, Format, Default, PartialEq, Eq)]
struct HwFaultStatus {
    vpu_ovp_fault: bool,
    vpu_presence: bool,
    vbus_disch_fault: bool,
    vsrc_disch_fault: bool,
    vconn_sw_rvp_fault: bool,
    vconn_sw_ocp_fault: bool,
    vconn_sw_ovp_fault: bool,
}

impl HwFaultStatus {
    fn parse(cc_hw_fault_status_1: u8) -> Self {
        Self {
            vpu_ovp_fault: cc_hw_fault_status_1 & (1 << 7) != 0,
            vpu_presence: cc_hw_fault_status_1 & (1 << 6) != 0,
            vbus_disch_fault: cc_hw_fault_status_1 & (1 << 4) != 0,
            vsrc_disch_fault: cc_hw_fault_status_1 & (1 << 3) != 0,
            vconn_sw_rvp_fault: cc_hw_fault_status_1 & (1 << 2) != 0,
            vconn_sw_ocp_fault: cc_hw_fault_status_1 & (1 << 1) != 0,
            vconn_sw_ovp_fault: cc_hw_fault_status_1 & (1 << 0) != 0,
        }
    }
}

#[derive(Clone, Format, Default)]
enum CcState {
    #[default]
    None,
    Default,
    Power15,
    Power30,
}

impl CcState {
    fn parse(v: u8) -> Self {
        match v & 0b11 {
            0b00 => Self::None,
            0b01 => Self::Default,
            0b10 => Self::Power15,
            0b11 => Self::Power30,
            _ => defmt::unreachable!(),
        }
    }
}

#[derive(Clone, Format, Default)]
struct PrtStatus {
    hw_reset_received: bool,
    hw_reset_done: bool,
    msg_received: bool,
    msg_sent: bool,
    bist_received: bool,
    bist_sent: bool,
    tx_error: bool,
}

impl PrtStatus {
    fn parse(prt_status: u8) -> Self {
        Self {
            hw_reset_received: prt_status & (1 << 0) != 0,
            hw_reset_done: prt_status & (1 << 1) != 0,
            msg_received: prt_status & (1 << 2) != 0,
            msg_sent: prt_status & (1 << 3) != 0,
            bist_received: prt_status & (1 << 4) != 0,
            bist_sent: prt_status & (1 << 5) != 0,
            tx_error: prt_status & (1 << 7) != 0,
        }
    }
}

#[derive(Clone, Format)]
struct MsgHeader {
    msg_type: MsgType,
    port_data_role: bool,
    specification_rev: u8,
    port_power_role_cable_plug: bool,
    msg_id: u8,
    num_data_objs: u8,
    extended: bool,
}

#[derive(Clone, Copy, Format, PartialEq, Eq)]
pub enum MsgType {
    Data(DataMsg),
    Control(ControlMsg),
}

#[derive(Clone, Copy, Format, PartialEq, Eq)]
pub enum DataMsg {
    SrcCaps,
    Request,
    Bist,
    SinkCaps,
    BatteryStatus,
    Alert,
    GetCountryInfo,
    VendorDefined,
    Other(u8),
}

impl DataMsg {
    fn parse(id: u8) -> Self {
        match id {
            0x01 => Self::SrcCaps,
            0x02 => Self::Request,
            0x03 => Self::Bist,
            0x04 => Self::SinkCaps,
            0x05 => Self::BatteryStatus,
            0x06 => Self::Alert,
            0x07 => Self::GetCountryInfo,
            0x0f => Self::VendorDefined,
            other => Self::Other(other),
        }
    }
}

#[derive(Clone, Copy, Format, PartialEq, Eq)]
pub enum ControlMsg {
    GoodCrc,
    GotoMin,
    Accept,
    Reject,
    Ping,
    PsReady,
    GetSourceCap,
    GetSinkCap,
    DrSwap,
    PrSwap,
    VconnSwap,
    Wait,
    SoftReset,
    NotSupported,
    GetSourceCapExtended,
    GetStatus,
    FrSwap,
    GetPpsStatus,
    GetCountryCodes,
    Other(u8),
}

impl ControlMsg {
    fn parse(id: u8) -> Self {
        match id {
            0x01 => Self::GoodCrc,
            0x02 => Self::GotoMin,
            0x03 => Self::Accept,
            0x04 => Self::Reject,
            0x05 => Self::Ping,
            0x06 => Self::PsReady,
            0x07 => Self::GetSourceCap,
            0x08 => Self::GetSinkCap,
            0x09 => Self::DrSwap,
            0x0a => Self::PrSwap,
            0x0b => Self::VconnSwap,
            0x0c => Self::Wait,
            0x0d => Self::SoftReset,
            0x10 => Self::NotSupported,
            0x11 => Self::GetSourceCapExtended,
            0x12 => Self::GetStatus,
            0x13 => Self::FrSwap,
            0x14 => Self::GetPpsStatus,
            0x15 => Self::GetCountryCodes,
            other => Self::Other(other),
        }
    }
}

impl MsgHeader {
    fn parse(header: u16) -> Self {
        let msg_type = (header & 0b11111) as u8;
        let num_data_objs = ((header >> 12) & 0b111) as u8;

        Self {
            msg_type: if num_data_objs == 0 {
                MsgType::Control(ControlMsg::parse(msg_type))
            } else {
                MsgType::Data(DataMsg::parse(msg_type))
            },
            port_data_role: header & (1 << 5) != 0,
            specification_rev: ((header >> 6) & 0b11) as u8,
            port_power_role_cable_plug: header & (1 << 8) != 0,
            msg_id: ((header >> 9) & 0b111) as u8,
            num_data_objs,
            extended: header & (1 << 15) != 0,
        }
    }
}

/// Power.
#[derive(Default, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct Power(pub u16);

impl Format for Power {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{} mW", self.to_mw())
    }
}

impl Power {
    /// Returns the power in mW.
    pub const fn to_mw(self) -> u32 {
        self.0 as u32 * 250
    }
}

#[derive(Clone, Format, PartialEq, Eq, Default)]
enum SinkPdo {
    Fixed(FixedSinkPdo),
    Variable(VariableSinkPdo),
    Battery(BatterySinkPdo),
    #[default]
    Reserved,
}

impl SinkPdo {
    fn parse(pdo: u32) -> Self {
        match (pdo >> 30) & 0b11 {
            0b00 => Self::Fixed(FixedSinkPdo::parse(pdo)),
            0b01 => Self::Battery(BatterySinkPdo::parse(pdo)),
            0b10 => Self::Variable(VariableSinkPdo::parse(pdo)),
            0b11 => Self::Reserved,
            _ => defmt::unreachable!(),
        }
    }

    fn generate(&self) -> u32 {
        match self {
            Self::Fixed(fixed) => fixed.generate(),
            Self::Variable(var) => var.generate(),
            Self::Battery(bat) => bat.generate(),
            Self::Reserved => 0b11 << 30,
        }
    }
}

#[derive(Clone, Default, PartialEq, Eq, Format)]
struct FixedSinkPdo {
    pub operating_current: Current,
    pub voltage: Voltage,
    pub fast_role_req_current: Current,
    pub dual_role_data: bool,
    pub communication: bool,
    pub unconstrained_power: bool,
    pub higher_capability: bool,
    pub dual_role_power: bool,
}

impl From<super::FixedSinkPdo> for FixedSinkPdo {
    fn from(pdo: super::FixedSinkPdo) -> Self {
        Self {
            operating_current: Current::from_ma(pdo.operating_current_ma),
            voltage: Voltage::from_mv(pdo.voltage_mv),
            fast_role_req_current: Current::from_ma(pdo.fast_role_req_current_ma),
            dual_role_data: pdo.dual_role_data,
            communication: pdo.communication,
            unconstrained_power: pdo.unconstrained_power,
            higher_capability: pdo.higher_capability,
            dual_role_power: pdo.dual_role_power,
        }
    }
}

impl FixedSinkPdo {
    #[allow(clippy::identity_op)]
    fn parse(pdo: u32) -> Self {
        Self {
            operating_current: Current(((pdo >> 0) & 0b1111111111) as u16),
            voltage: Voltage(((pdo >> 10) & 0b1111111111) as u16),
            fast_role_req_current: Current(((pdo >> 23) & 0b11) as u16),
            dual_role_data: pdo & (1 << 25) != 0,
            communication: pdo & (1 << 26) != 0,
            unconstrained_power: pdo & (1 << 27) != 0,
            higher_capability: pdo & (1 << 28) != 0,
            dual_role_power: pdo & (1 << 29) != 0,
        }
    }

    #[allow(clippy::identity_op)]
    fn generate(&self) -> u32 {
        let mut pdo = 0;
        pdo |= (self.operating_current.0 as u32 & 0b1111111111) << 0;
        pdo |= (self.voltage.0 as u32 & 0b1111111111) << 10;
        pdo |= (self.fast_role_req_current.0 as u32 & 0b11) << 23;
        pdo |= (self.dual_role_data as u32) << 25;
        pdo |= (self.communication as u32) << 26;
        pdo |= (self.unconstrained_power as u32) << 27;
        pdo |= (self.higher_capability as u32) << 28;
        pdo |= (self.dual_role_power as u32) << 29;
        pdo |= 0b00 << 30;
        pdo
    }
}

#[derive(Clone, Format, PartialEq, Eq)]
struct VariableSinkPdo {
    operating_current: Current,
    min_voltage: Voltage,
    max_voltage: Voltage,
}

impl VariableSinkPdo {
    #[allow(clippy::identity_op)]
    fn parse(pdo: u32) -> Self {
        Self {
            operating_current: Current(((pdo >> 0) & 0b1111111111) as u16),
            min_voltage: Voltage(((pdo >> 10) & 0b1111111111) as u16),
            max_voltage: Voltage(((pdo >> 20) & 0b1111111111) as u16),
        }
    }

    #[allow(clippy::identity_op)]
    fn generate(&self) -> u32 {
        let mut pdo = 0;
        pdo |= (self.operating_current.0 as u32 & 0b1111111111) << 0;
        pdo |= (self.min_voltage.0 as u32 & 0b1111111111) << 10;
        pdo |= (self.max_voltage.0 as u32 & 0b1111111111) << 20;
        pdo |= 0b10 << 30;
        pdo
    }
}

#[derive(Clone, Format, PartialEq, Eq)]
struct BatterySinkPdo {
    operating_power: Power,
    min_voltage: Voltage,
    max_voltage: Voltage,
}

impl BatterySinkPdo {
    #[allow(clippy::identity_op)]
    fn parse(pdo: u32) -> Self {
        Self {
            operating_power: Power(((pdo >> 0) & 0b1111111111) as u16),
            min_voltage: Voltage(((pdo >> 10) & 0b1111111111) as u16),
            max_voltage: Voltage(((pdo >> 20) & 0b1111111111) as u16),
        }
    }

    #[allow(clippy::identity_op)]
    fn generate(&self) -> u32 {
        let mut pdo = 0;
        pdo |= (self.operating_power.0 as u32 & 0b1111111111) << 0;
        pdo |= (self.min_voltage.0 as u32 & 0b1111111111) << 10;
        pdo |= (self.max_voltage.0 as u32 & 0b1111111111) << 20;
        pdo |= 0b01 << 30;
        pdo
    }
}

#[derive(Clone, Format, PartialEq, Eq)]
enum SupplyPdo {
    Fixed(FixedSupplyPdo),
    Variable(VariableSupplyPdo),
    Battery(BatterySupplyPdo),
    Reserved,
}

impl SupplyPdo {
    fn parse(pdo: u32) -> Self {
        match (pdo >> 30) & 0b11 {
            0b00 => Self::Fixed(FixedSupplyPdo::parse(pdo)),
            0b01 => Self::Battery(BatterySupplyPdo::parse(pdo)),
            0b10 => Self::Variable(VariableSupplyPdo::parse(pdo)),
            0b11 => Self::Reserved,
            _ => defmt::unreachable!(),
        }
    }
}

#[derive(Clone, Format, PartialEq, Eq)]
struct FixedSupplyPdo {
    max_operating_current: Current,
    voltage: Voltage,
    peak_current: Current,
    data_role_swap: bool,
    communication: bool,
    externally_powered: bool,
    suspend_supported: bool,
    dual_role_power: bool,
}

impl FixedSupplyPdo {
    #[allow(clippy::identity_op)]
    fn parse(pdo: u32) -> Self {
        Self {
            max_operating_current: Current(((pdo >> 0) & 0b1111111111) as u16),
            voltage: Voltage(((pdo >> 10) & 0b1111111111) as u16),
            peak_current: Current(((pdo >> 20) & 0b11) as u16),
            data_role_swap: pdo & (1 << 25) != 0,
            communication: pdo & (1 << 26) != 0,
            externally_powered: pdo & (1 << 27) != 0,
            suspend_supported: pdo & (1 << 28) != 0,
            dual_role_power: pdo & (1 << 29) != 0,
        }
    }
}

#[derive(Clone, Format, PartialEq, Eq)]
struct VariableSupplyPdo {
    operating_current: Current,
    min_voltage: Voltage,
    max_voltage: Voltage,
}

impl VariableSupplyPdo {
    #[allow(clippy::identity_op)]
    fn parse(pdo: u32) -> Self {
        Self {
            operating_current: Current(((pdo >> 0) & 0b1111111111) as u16),
            min_voltage: Voltage(((pdo >> 10) & 0b1111111111) as u16),
            max_voltage: Voltage(((pdo >> 20) & 0b1111111111) as u16),
        }
    }
}

#[derive(Clone, Format, PartialEq, Eq)]
struct BatterySupplyPdo {
    operating_power: Power,
    min_voltage: Voltage,
    max_voltage: Voltage,
}

impl BatterySupplyPdo {
    #[allow(clippy::identity_op)]
    fn parse(pdo: u32) -> Self {
        Self {
            operating_power: Power(((pdo >> 0) & 0b1111111111) as u16),
            min_voltage: Voltage(((pdo >> 10) & 0b1111111111) as u16),
            max_voltage: Voltage(((pdo >> 20) & 0b1111111111) as u16),
        }
    }
}

#[derive(Clone, Format, Default, PartialEq, Eq)]
struct Rdo {
    max_current: Current,
    operating_current: Current,
    unchunked_mess_supported: bool,
    usb_suspend: bool,
    usb_com_cap: bool,
    capa_mismatch: bool,
    give_back: bool,
    object_pos: u8,
}

impl Rdo {
    #[allow(clippy::identity_op)]
    fn parse(rdo: u32) -> Self {
        Self {
            max_current: Current(((rdo >> 0) & 0b1111111111) as u16),
            operating_current: Current(((rdo >> 10) & 0b1111111111) as u16),
            unchunked_mess_supported: rdo & (1 << 23) != 0,
            usb_suspend: rdo & (1 << 24) != 0,
            usb_com_cap: rdo & (1 << 25) != 0,
            capa_mismatch: rdo & (1 << 26) != 0,
            give_back: rdo & (1 << 27) != 0,
            object_pos: ((rdo >> 28) & 0b111) as u8,
        }
    }
}
