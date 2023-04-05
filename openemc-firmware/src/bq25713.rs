//
// OpenEMC firmware for embedded controllers
// Copyright (C) 2022 Sebastian Urban <surban@surban.net>
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

//! BQ25713 buck-boost battery charge controller driver.

use core::marker::PhantomData;
use defmt::Format;
use embedded_hal::blocking::i2c;
use heapless::Vec;

/// BQ25713 error.
#[derive(Clone, Format, PartialEq, Eq)]
pub enum Error {
    /// I2C communication error.
    I2c,
    /// Device responded with wrong id.
    WrongId,
    /// Device is not initialized.
    Uninitialized,
}

/// BQ25713 result.
pub type Result<T> = core::result::Result<T, Error>;

/// BQ25713 configuration.
#[derive(Format, Clone, PartialEq, Eq)]
pub struct Bq25713Cfg {
    /// I2C address (0x6b for BQ25713 and 0x6a for BQ25713B).
    pub addr: u8,
    /// Reset at initialization?
    pub reset: bool,
    /// Minimum system voltage in mV.
    pub min_sys_mv: u32,
    /// Minimum input voltage in mV.
    pub min_input_mv: u32,
    /// Maximum battery voltage in mV.
    pub max_battery_mv: u32,
    /// Maximum battery charge current in mA.
    pub max_charge_ma: u32,
    /// Hiccup mode during system short protection.
    pub sys_short: bool,
}

/// BQ25713 measured voltages and currents.
#[derive(Format, Clone, PartialEq, Eq)]
pub struct Bq25713Measurement {
    /// System voltage.
    pub v_sys_mv: u32,
    /// Battery voltage.
    pub v_bat_mv: u32,
    /// Input voltage.
    pub v_bus_mv: u32,
    /// CMPIN voltage.
    pub v_cmpin_mv: u32,
    /// System power.
    pub v_psys_mv: u32,
    /// Input current.
    pub i_in_ma: u32,
    /// Charge current.
    pub i_chg_ma: u32,
    /// Discharge current.
    pub i_dchg_ma: u32,
}

/// BQ25713 status.
#[derive(Format, Default, Clone, PartialEq, Eq)]
pub struct Bq25713Status {
    /// Input source status.
    pub ac_stat: bool,
    /// ICO routine successfully executed?
    pub ico_done: bool,
    /// In VAP mode?
    pub in_vap: bool,
    /// In VINDPM mode?
    pub in_vindpm: bool,
    /// In IINDPM mode?
    pub in_iindpm: bool,
    /// Fast charging?
    pub in_fchrg: bool,
    /// Pre charging?
    pub in_pchrg: bool,
    /// In OTG mode?
    pub in_otg: bool,
    /// ACOV fault.
    pub fault_acov: bool,
    /// BATOC fault.
    pub fault_batoc: bool,
    /// ACOC fault.
    pub fault_acoc: bool,
    /// System overvoltage.
    pub sys_ovp: bool,
    /// System short.
    pub sys_short: bool,
    /// Latch off?
    pub fault_latchoff: bool,
    /// OTG OVP.
    pub otg_ovp: bool,
    /// OTG UVP.
    pub otg_uvp: bool,
}

impl Bq25713Status {
    /// Returns whether a clearable fault is present.
    pub fn has_clearable_fault(&self) -> bool {
        self.sys_ovp || self.sys_short
    }

    /// Returns whether any fault is present.
    pub fn has_any_fault(&self) -> bool {
        self.fault_acov
            || self.fault_batoc
            || self.fault_acoc
            || self.sys_ovp
            || self.sys_short
            || self.fault_latchoff
            || self.otg_ovp
            || self.otg_uvp
    }
}

impl Bq25713Status {
    fn parse(hi: u8, lo: u8) -> Self {
        Self {
            ac_stat: hi & (1 << 7) != 0,
            ico_done: hi & (1 << 6) != 0,
            in_vap: hi & (1 << 5) != 0,
            in_vindpm: hi & (1 << 4) != 0,
            in_iindpm: hi & (1 << 3) != 0,
            in_fchrg: hi & (1 << 2) != 0,
            in_pchrg: hi & (1 << 1) != 0,
            in_otg: hi & (1 << 0) != 0,
            fault_acov: lo & (1 << 7) != 0,
            fault_batoc: lo & (1 << 6) != 0,
            fault_acoc: lo & (1 << 5) != 0,
            sys_ovp: lo & (1 << 4) != 0,
            sys_short: lo & (1 << 3) != 0,
            fault_latchoff: lo & (1 << 2) != 0,
            otg_ovp: lo & (1 << 1) != 0,
            otg_uvp: lo & (1 << 0) != 0,
        }
    }
}

/// Battery status.
#[derive(Format, Default, Clone, PartialEq, Eq)]
pub struct Battery {
    /// Battery present?
    pub present: bool,
    /// Battery voltage in mV.
    pub voltage_mv: u32,
    /// Battery current in mA.
    ///
    /// Positive is charging, negative is discharging.
    pub current_ma: i32,
    /// System voltage in mV.
    pub system_voltage_mv: u32,
    /// Input voltage in mV.
    pub input_voltage_mv: u32,
    /// Input current in mA.
    pub input_current_ma: u32,
    /// Maximum battery voltage in mV.
    pub max_voltage_mv: u32,
    /// Maximum charging current in mA.
    pub max_charge_current_ma: u32,
    /// Charging status.
    pub charging: Charging,
}

impl Battery {
    /// Whether the battery status changed significantly.
    pub fn changed_significantly(&self, other: &Battery) -> bool {
        const DIFF: u32 = 75;

        self.present != other.present
            || self.voltage_mv.abs_diff(other.voltage_mv) >= DIFF
            || self.current_ma.abs_diff(other.current_ma) >= DIFF
            || self.system_voltage_mv.abs_diff(other.system_voltage_mv) >= DIFF
            || self.input_voltage_mv.abs_diff(other.input_voltage_mv) >= DIFF
            || self.input_current_ma.abs_diff(other.input_current_ma) >= DIFF
            || self.max_voltage_mv != other.max_voltage_mv
            || self.max_charge_current_ma != other.max_charge_current_ma
            || self.charging != other.charging
    }
}

/// Battery charging status.
#[derive(Format, Default, Clone, Copy, PartialEq, Eq)]
pub enum Charging {
    /// Not charging.
    #[default]
    Off,
    /// Precharging.
    Pre,
    /// Fast charging.
    Fast,
}

impl Charging {
    /// Returns whether charging.
    pub fn is_charging(&self) -> bool {
        match self {
            Self::Off => false,
            Self::Pre | Self::Fast => true,
        }
    }
}

impl From<Charging> for u8 {
    fn from(ch: Charging) -> Self {
        match ch {
            Charging::Off => 0,
            Charging::Pre => 1,
            Charging::Fast => 2,
        }
    }
}

/// BQ25713 battery charger instance.
pub struct Bq25713<I2C> {
    addr: u8,
    cfg: Bq25713Cfg,
    initialized: bool,
    adc_triggered: bool,
    measurement: Option<Bq25713Measurement>,
    status: Bq25713Status,
    charge_enabled: bool,
    _i2c: PhantomData<I2C>,
}

impl<I2C> Bq25713<I2C>
where
    I2C: i2c::Write<i2c::SevenBitAddress> + i2c::WriteRead<i2c::SevenBitAddress>,
{
    /// Read I2C register(s).
    fn read(&self, i2c: &mut I2C, reg: u8, len: usize) -> Result<Vec<u8, 32>> {
        let mut buf: Vec<u8, 32> = Vec::new();
        defmt::unwrap!(buf.resize_default(len));
        i2c.write_read(self.addr, &[reg], &mut buf).map_err(|_| Error::I2c)?;
        Ok(buf)
    }

    /// Write I2C register(s).
    fn write(&self, i2c: &mut I2C, reg: u8, data: &[u8]) -> Result<()> {
        let mut buf: Vec<u8, 32> = Vec::new();
        defmt::unwrap!(buf.push(reg));
        buf.extend(data.iter().cloned());
        i2c.write(self.addr, &buf).map_err(|_| Error::I2c)
    }

    /// Write 2-byte I2C register.
    fn write_u16(&self, i2c: &mut I2C, reg: u8, value: u16) -> Result<()> {
        self.write(i2c, reg, &[value as u8, (value >> 8) as u8])
    }

    /// Modify an I2C register.
    fn modify(&self, i2c: &mut I2C, reg: u8, mod_fn: impl FnOnce(u8) -> u8) -> Result<()> {
        let v = self.read(i2c, reg, 1)?;
        let v_new = mod_fn(v[0]);
        self.write(i2c, reg, &[v_new])?;
        Ok(())
    }

    /// Creates a new charger instance.
    ///
    /// `cfg` specifies configuration.
    pub fn new(cfg: Bq25713Cfg) -> Self {
        defmt::info!("BQ25713 at 0x{:x} created", cfg.addr);

        Self {
            addr: cfg.addr,
            cfg,
            initialized: false,
            adc_triggered: false,
            measurement: None,
            status: Default::default(),
            charge_enabled: false,
            _i2c: PhantomData,
        }
    }

    /// Initialize the charger.
    fn init(&mut self, i2c: &mut I2C) -> Result<()> {
        defmt::info!("BQ25713 init");

        // Reset.
        if self.cfg.reset {
            defmt::info!("BQ25713 reset");
            self.modify(i2c, REG_CHARGE_OPTION_3_HI, |v| v | 1 << 6)?;
            for _ in 0..10 {
                self.read(i2c, REG_CHARGE_OPTION_3_HI, 1)?;
            }
            while self.read(i2c, REG_CHARGE_OPTION_3_HI, 1)?[0] & (1 << 6) != 0 {
                defmt::trace!("BQ25713 reset in progress");
            }
            for _ in 0..10 {
                self.read(i2c, REG_CHARGE_OPTION_3_HI, 1)?;
            }
        }

        // Verify id.
        defmt::debug!("read BQ25713 id at address 0x{:x}", self.cfg.addr);
        let manufacture_id = self.read(i2c, REG_MANUFACTURE_ID, 1)?;
        let device_id = self.read(i2c, REG_DEVICE_ID, 1)?;
        defmt::debug!(
            "BQ25713 at 0x{:x} has manufacture id 0x{:x} and device id 0x{:x}",
            self.addr,
            manufacture_id[0],
            device_id[0]
        );
        if !(manufacture_id[0] == MANUFACTURE_ID && DEVICE_IDS.contains(&device_id[0])) {
            return Err(Error::WrongId);
        }
        self.initialized = true;

        // Print minimum system voltage.
        let min_sys_mv = self.get_min_system_voltage(i2c)?;
        defmt::debug!("Default minimum system voltage is {} mV", min_sys_mv);

        // Disable Hi-Z mode.
        self.modify(i2c, REG_CHARGE_OPTION_3_HI, |v| v & !(1 << 7))?;

        // Disable low power mode.
        self.modify(i2c, REG_CHARGE_OPTION_0_HI, |v| v & !(1 << 7))?;

        // Disable charging.
        self.set_charge_enable(i2c, false)?;

        // Set watchdog timer to 5s.
        self.modify(i2c, REG_CHARGE_OPTION_0_HI, |v| v & !(0b11 << 5) | (0b01 << 5))?;

        // Set current limit through registers.
        self.modify(i2c, REG_CHARGE_OPTION_2_LO, |v| v & !(1 << 7))?;

        // Enable IBAT and PSYS measurements.
        self.modify(i2c, REG_CHARGE_OPTION_1_HI, |v| v | (1 << 7) | (1 << 4))?;

        // Set configuration.
        self.set_sys_short_hiccup(i2c, self.cfg.sys_short)?;
        self.set_min_system_voltage(i2c, self.cfg.min_sys_mv)?;
        self.set_min_input_voltage(i2c, self.cfg.min_input_mv)?;
        self.set_max_charge_voltage(i2c, self.cfg.max_battery_mv)?;

        // Read initial status.
        self.update_status(i2c)?;

        defmt::info!("BQ25713 initialized");
        Ok(())
    }

    /// Enters low power and Hi-Z mode.
    ///
    /// Afterwards reinitialization is necessary.
    pub fn shutdown(self, i2c: &mut I2C) -> Result<()> {
        defmt::info!("BQ25713 entering low power mode");

        // Enable low power mode.
        self.modify(i2c, REG_CHARGE_OPTION_0_HI, |v| v | (1 << 7))?;

        // Enable Hi-Z mode.
        self.modify(i2c, REG_CHARGE_OPTION_3_HI, |v| v | (1 << 7))?;

        Ok(())
    }

    /// Whether the charger is initiailzied and ready.
    pub fn initialized(&self) -> bool {
        self.initialized
    }

    /// Updates the values from the ADC converter.
    fn update_adc(&mut self, i2c: &mut I2C) -> Result<()> {
        if !self.adc_triggered {
            // Start ADC conversion.
            defmt::trace!("Starting ADC conversion");
            self.write(i2c, REG_ADC_OPTION_LO, &[0xff])?;
            self.write(i2c, REG_ADC_OPTION_HI, &[0b01100000])?;
            self.adc_triggered = true;
        } else {
            // Check if ADC conversion is finished.
            let status = self.read(i2c, REG_ADC_OPTION_HI, 1)?;
            if status[0] & (1 << 6) != 0 {
                defmt::trace!("ADC conversion is in progress");
                return Ok(());
            }
            self.adc_triggered = false;
            defmt::trace!("ADC conversion finished");

            // Read ADC values.
            self.measurement = Some(Bq25713Measurement {
                v_sys_mv: 2880 + (self.read(i2c, REG_ADC_V_SYS, 1)?[0] as u32) * 64,
                v_bat_mv: 2880 + (self.read(i2c, REG_ADC_V_BAT, 1)?[0] as u32) * 64,
                v_bus_mv: 3200 + (self.read(i2c, REG_ADC_V_BUS, 1)?[0] as u32) * 64,
                v_cmpin_mv: (self.read(i2c, REG_ADC_I_CMPIN, 1)?[0] as u32) * 12,
                v_psys_mv: (self.read(i2c, REG_ADC_P_SYS, 1)?[0] as u32) * 12,
                i_in_ma: (self.read(i2c, REG_ADC_I_IN, 1)?[0] as u32) * 50,
                i_chg_ma: (self.read(i2c, REG_ADC_I_CHG, 1)?[0] as u32 & 0b0111_1111) * 64,
                i_dchg_ma: (self.read(i2c, REG_ADC_I_DCHG, 1)?[0] as u32 & 0b0111_1111) * 256,
            });
            defmt::debug!("ADC measurements: {:?}", &self.measurement);
        }

        Ok(())
    }

    /// Updates the status.
    fn update_status(&mut self, i2c: &mut I2C) -> Result<()> {
        let hi = self.read(i2c, REG_CHARGER_STATUS_HI, 1)?;
        let lo = self.read(i2c, REG_CHARGER_STATUS_LO, 1)?;
        self.status = Bq25713Status::parse(hi[0], lo[0]);
        defmt::debug!("BQ25713 status: {:?}", &self.status);
        Ok(())
    }

    /// Checks whether the device is initiailzied.
    fn check_initialized(&self) -> Result<()> {
        match self.initialized {
            true => Ok(()),
            false => Err(Error::Uninitialized),
        }
    }

    /// Clear the fault status.
    pub fn clear_faults(&mut self, i2c: &mut I2C) -> Result<()> {
        self.check_initialized()?;
        defmt::debug!("Clearing BQ25713 fault status");
        self.write(i2c, REG_CHARGER_STATUS_LO, &[0x00])
    }

    /// Sets the input current limit.
    pub fn set_max_input_current(&mut self, i2c: &mut I2C, ma: u32) -> Result<()> {
        self.check_initialized()?;
        defmt::debug!("Setting maximum input current to {} mA", ma);
        let v = (ma / 50) as u8 & 0b01111111;
        self.write(i2c, REG_IIN_HOST, &[v])
    }

    /// Sets the minimum input voltage.
    fn set_min_input_voltage(&mut self, i2c: &mut I2C, mv: u32) -> Result<()> {
        defmt::debug!("Setting minimum input voltage to {} mV", mv);
        let v = ((mv.saturating_sub(3200) / 64) as u16) << 6 & 0b0011_1111_1100_0000;
        self.write_u16(i2c, REG_INPUT_VOLTAGE, v)
    }

    /// Sets the minimum system voltage.
    fn set_min_system_voltage(&mut self, i2c: &mut I2C, mv: u32) -> Result<()> {
        defmt::debug!("Setting minimum system voltage to {} mV", mv);
        let v = (mv / 256) as u8 & 0b00111111;
        self.write(i2c, REG_MIN_SYSTEM_VOLTAGE, &[v])
    }

    /// Gets the minimum system voltage.
    fn get_min_system_voltage(&self, i2c: &mut I2C) -> Result<u32> {
        let v = self.read(i2c, REG_MIN_SYSTEM_VOLTAGE, 1)?;
        Ok((v[0] & 0b00111111) as u32 * 256)
    }

    /// Sets the maximum charge voltage.
    fn set_max_charge_voltage(&mut self, i2c: &mut I2C, mv: u32) -> Result<()> {
        defmt::debug!("Setting maximum charge voltage to {} mV", mv);
        let v = ((mv / 8) as u16) << 3 & 0b0111_1111_1111_1000;
        self.write_u16(i2c, REG_MAX_CHARGE_VOLTAGE, v)
    }

    /// Sets the charge current.
    fn set_charge_current(&mut self, i2c: &mut I2C, ma: u32) -> Result<()> {
        defmt::debug!("Setting charge current to {} mA", ma);
        let v = ((ma / 64) as u16) << 6 & 0b0001_1111_1100_0000;
        self.write_u16(i2c, REG_CHARGE_CURRENT, v)
    }

    /// Sets the system short hiccup mode.
    fn set_sys_short_hiccup(&mut self, i2c: &mut I2C, enable: bool) -> Result<()> {
        defmt::debug!("Setting system short hiccup mode to {}", enable);
        self.modify(i2c, REG_CHARGE_OPTION_0_LO, |v| if enable { v & !(1 << 6) } else { v | (1 << 6) })
    }

    /// Enables or disables charging.
    pub fn set_charge_enable(&mut self, i2c: &mut I2C, enable: bool) -> Result<()> {
        self.check_initialized()?;
        defmt::debug!("Setting charge enabled to {}", enable);
        self.modify(i2c, REG_CHARGE_OPTION_0_LO, |v| if enable { v & !(1 << 0) } else { v | (1 << 0) })?;
        self.charge_enabled = enable;
        Ok(())
    }

    /// Whether charging is enabled.
    pub fn is_charge_enabled(&self) -> bool {
        self.initialized && self.charge_enabled
    }

    /// The current status.
    pub fn status(&self) -> Option<&Bq25713Status> {
        self.initialized.then_some(&self.status)
    }

    /// The current AD converter measurement.
    pub fn measurement(&self) -> Option<&Bq25713Measurement> {
        if self.initialized {
            self.measurement.as_ref()
        } else {
            None
        }
    }

    /// The battery status.
    pub fn battery(&self) -> Option<Battery> {
        self.initialized.then(|| Battery {
            present: true,
            voltage_mv: self.measurement.as_ref().map(|m| m.v_bat_mv).unwrap_or_default(),
            current_ma: self
                .measurement
                .as_ref()
                .map(|m| {
                    if self.status.ac_stat {
                        m.i_chg_ma.try_into().unwrap_or_default()
                    } else {
                        -i32::try_from(m.i_dchg_ma).unwrap_or_default()
                    }
                })
                .unwrap_or_default(),
            system_voltage_mv: self.measurement.as_ref().map(|m| m.v_sys_mv).unwrap_or_default(),
            input_voltage_mv: self
                .measurement
                .as_ref()
                .map(|m| if m.v_bus_mv >= 3300 { m.v_bus_mv } else { 0 })
                .unwrap_or_default(),
            input_current_ma: self.measurement.as_ref().map(|m| m.i_in_ma).unwrap_or_default(),
            max_voltage_mv: self.cfg.max_battery_mv,
            max_charge_current_ma: self.cfg.max_charge_ma,
            charging: if !self.status.ac_stat
                || self.measurement.as_ref().map(|m| m.i_chg_ma <= 50).unwrap_or_default()
            {
                Charging::Off
            } else if self.status.in_pchrg {
                Charging::Pre
            } else if self.status.in_fchrg {
                Charging::Fast
            } else {
                Charging::Off
            },
        })
    }

    fn do_periodic(&mut self, i2c: &mut I2C) -> Result<()> {
        if !self.initialized {
            self.init(i2c)?;
        }

        self.update_status(i2c)?;
        self.update_adc(i2c)?;
        self.set_charge_current(i2c, self.cfg.max_charge_ma)?;

        Ok(())
    }

    /// Call this periodically (approx. every second) to handle communication with the device.
    pub fn periodic(&mut self, i2c: &mut I2C) {
        if let Err(err) = self.do_periodic(i2c) {
            defmt::warn!("BQ25713 failed: {}", err);
            self.initialized = false;
        }
    }
}

// Register definitions.
const REG_CHARGE_OPTION_3_HI: u8 = 0x35;
// const REG_CHARGE_OPTION_3_LO: u8 = 0x34;
// const REG_CHARGE_OPTION_2_HI: u8 = 0x33;
const REG_CHARGE_OPTION_2_LO: u8 = 0x32;
const REG_CHARGE_OPTION_1_HI: u8 = 0x31;
// const REG_CHARGE_OPTION_1_LO: u8 = 0x30;
const REG_CHARGE_OPTION_0_HI: u8 = 0x01;
const REG_CHARGE_OPTION_0_LO: u8 = 0x00;
const REG_ADC_OPTION_HI: u8 = 0x3b;
const REG_ADC_OPTION_LO: u8 = 0x3a;
const REG_MANUFACTURE_ID: u8 = 0x2e;
const REG_DEVICE_ID: u8 = 0x2f;
const REG_ADC_V_BUS: u8 = 0x27;
const REG_ADC_P_SYS: u8 = 0x26;
const REG_ADC_I_CHG: u8 = 0x29;
const REG_ADC_I_DCHG: u8 = 0x28;
const REG_ADC_I_IN: u8 = 0x2b;
const REG_ADC_I_CMPIN: u8 = 0x2a;
const REG_ADC_V_SYS: u8 = 0x2d;
const REG_ADC_V_BAT: u8 = 0x2c;
const REG_IIN_HOST: u8 = 0x0f;
const REG_INPUT_VOLTAGE: u8 = 0x0a;
const REG_MIN_SYSTEM_VOLTAGE: u8 = 0x0d;
const REG_MAX_CHARGE_VOLTAGE: u8 = 0x04;
const REG_CHARGE_CURRENT: u8 = 0x02;
const REG_CHARGER_STATUS_HI: u8 = 0x21;
const REG_CHARGER_STATUS_LO: u8 = 0x20;

/// BQ25713 chip ids.
const MANUFACTURE_ID: u8 = 0x40;
const DEVICE_IDS: [u8; 2] = [0x88, 0x8a];
