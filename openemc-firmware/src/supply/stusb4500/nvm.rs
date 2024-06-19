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

//! STUSB4500 NVM programming interface.

use embedded_hal::blocking::i2c;
use heapless::Vec;

use super::{Error, Result, IDS, REG_ID};

/// STUSB4500 NVM programming interface.
pub struct StUsb4500Nvm<'a, I2C>
where
    I2C: i2c::Write<i2c::SevenBitAddress> + i2c::WriteRead<i2c::SevenBitAddress>,
{
    addr: u8,
    i2c: &'a mut I2C,
    nvm_enabled: bool,
}

impl<'a, I2C> StUsb4500Nvm<'a, I2C>
where
    I2C: i2c::Write<i2c::SevenBitAddress> + i2c::WriteRead<i2c::SevenBitAddress>,
{
    /// Mask of programmable bits within NVM.
    #[rustfmt::skip]
    pub const NVM_PROGRAMABLE: [u8; 40] = [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xC0 - 0xC7
        0x30, 0x20, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xC8 - 0xCF
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xD0 - 0xD7
        0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, // 0xD8 - 0xDF
        0xC0, 0xFF, 0xFF, 0xFF, 0x6F, 0x00, 0x18, 0x00, // 0xE0 - 0xE7
    ];

    /// Read I2C register(s).
    fn read(&mut self, reg: u8, len: usize) -> Result<Vec<u8, 32>> {
        let mut buf: Vec<u8, 32> = Vec::new();
        defmt::unwrap!(buf.resize_default(len));
        self.i2c.write_read(self.addr, &[reg], &mut buf).map_err(|_| Error::I2c)?;
        Ok(buf)
    }

    /// Write I2C register(s).
    fn write(&mut self, reg: u8, data: &[u8]) -> Result<()> {
        let mut buf: Vec<u8, 32> = Vec::new();
        defmt::unwrap!(buf.push(reg));
        buf.extend(data.iter().cloned());
        self.i2c.write(self.addr, &buf).map_err(|_| Error::I2c)
    }

    /// Checks the id of the STUSB4500.
    fn check_id(&mut self) -> Result<()> {
        defmt::debug!("read STUSB4500 id");
        let id = self.read(REG_ID, 1)?;
        defmt::debug!("STUSB4500 id at 0x{:x} is 0x{:x}", self.addr, id[0]);

        if IDS.contains(&id[0]) {
            Ok(())
        } else {
            defmt::warn!("STUSB4500 id 0x{:x} is wrong", id[0]);
            Err(Error::WrongId)
        }
    }

    /// Wait for NVM controller to be ready.
    fn wait_ready(&mut self) -> Result<()> {
        let mut tries = 0;

        loop {
            let buf = self.read(0x96, 1)?;
            if buf[0] & 0x10 == 0 {
                break;
            }

            tries += 1;
            if tries > 1000 {
                return Err(Error::Timeout);
            }
        }

        Ok(())
    }

    /// Enable NVM access.
    fn enable_nvm(&mut self) -> Result<()> {
        defmt::debug!("enabling STUSB4500 NVM access");

        // Unlock NVM.
        self.write(0x95, &[0x47])?;

        // Put NVM internal controller in operational conditions.
        self.write(0x96, &[0x40])?;

        // Reset NVM internal controller.
        self.write(0x96, &[0x00])?;
        self.wait_ready()?;

        // Put NVM internal controller in operational conditions.
        self.write(0x96, &[0x40])?;

        self.nvm_enabled = true;
        defmt::debug!("enabled STUSB4500 NVM access");
        Ok(())
    }

    /// Disable NVM access.
    fn disable_nvm(&mut self) -> Result<()> {
        defmt::debug!("disabling STUSB4500 NVM access");

        // Clear FTP_CTRL registers.
        self.write(0x96, &[0x40, 0x00])?;
        // Clear FTP_KEY register.
        self.write(0x95, &[0x00])?;

        self.nvm_enabled = false;
        defmt::debug!("disabled STUSB4500 NVM access");
        Ok(())
    }

    /// Reads the specified sector from the NVM.
    fn read_nvm_sector(&mut self, sector: usize) -> Result<[u8; 8]> {
        defmt::trace!("reading sector {}", sector);
        defmt::assert!(sector <= 4);

        // Set read sector opcode.
        self.write(0x97, &[0x00])?;

        // Perform read request.
        self.write(0x96, &[0x50 + sector as u8])?;
        self.wait_ready()?;

        // Read.
        let mut buf = [0; 8];
        for (i, v) in buf.iter_mut().enumerate() {
            *v = self.read(0x53 + i as u8, 1)?[0];
        }

        Ok(buf)
    }

    /// Reads the contents of the NVM.
    pub fn read_nvm(&mut self) -> Result<[u8; 40]> {
        defmt::debug!("reading NVM");

        let mut buf = [0; 40];
        for sector in 0..=4 {
            buf[(sector * 8)..((sector + 1) * 8)].copy_from_slice(&self.read_nvm_sector(sector)?);
        }

        Ok(buf)
    }

    /// Erase the contents of the NVM.
    fn erase_nvm(&mut self) -> Result<()> {
        defmt::trace!("erasing NVM");

        // Shift in data on sector erase.
        self.write(0x97, &[0xfa])?;
        self.write(0x96, &[0x50])?;
        self.wait_ready()?;

        // Soft program array.
        self.write(0x97, &[0x07])?;
        self.write(0x96, &[0x50])?;
        self.wait_ready()?;

        // Erase memory array.
        self.write(0x97, &[0x05])?;
        self.write(0x96, &[0x50])?;
        self.wait_ready()?;

        Ok(())
    }

    /// Writes the specified sector to the NVM.
    fn write_nvm_sector(&mut self, sector: usize, data: &[u8; 8]) -> Result<()> {
        defmt::trace!("writing sector {}", sector);
        defmt::assert!(sector <= 4);

        // Load data.
        self.write(0x53, data)?;

        // Shift in data on program load.
        self.write(0x97, &[0x01])?;
        self.write(0x96, &[0x50])?;
        self.wait_ready()?;

        // Program word into EEPROM.
        self.write(0x97, &[0x06])?;
        self.write(0x96, &[0x50 + sector as u8])?;
        self.wait_ready()?;

        Ok(())
    }

    /// Writes the contents of the NVM.
    pub fn write_nvm(&mut self, data: &[u8; 40]) -> Result<()> {
        defmt::debug!("writing NVM");

        self.erase_nvm()?;

        for sector in 0..=4 {
            self.write_nvm_sector(sector, &defmt::unwrap!(data[(sector * 8)..((sector + 1) * 8)].try_into()))?;
        }

        Ok(())
    }

    /// Checks whehther the programmable bits of the NVM are equal.
    fn nvm_equal(&self, left: &[u8; 40], right: &[u8; 40]) -> bool {
        left.iter().zip(right).zip(Self::NVM_PROGRAMABLE).all(|((&a, &b), m)| a & m == b & m)
    }

    /// Ensures that the programable bits of the NVM have the specified contents.
    ///
    /// The NVM is read and, if the contents do not match, reprogrammed.
    ///
    /// Returns whether the NVM has been reprogrammed.
    pub fn ensure_nvm(&mut self, data: &[u8; 40]) -> Result<bool> {
        let nvm = self.read_nvm()?;
        if self.nvm_equal(&nvm, data) {
            return Ok(false);
        }

        defmt::info!("reprogramming NVM");
        self.write_nvm(data)?;

        let nvm = self.read_nvm()?;
        if !self.nvm_equal(&nvm, data) {
            defmt::warn!("NVM mismatch after programming:");
            defmt::warn!("programmed: {:02x}", data);
            defmt::warn!("read back:  {:02x}", &nvm);
            return Err(Error::VerifyFailed);
        }

        Ok(true)
    }

    /// Creates a new STUSB4500 NVM programming interface instance.
    ///
    /// NVM access is enabled during creation and disabled when the interface is dropped.
    pub fn new(addr: u8, i2c: &'a mut I2C) -> Result<Self> {
        let mut this = Self { addr, i2c, nvm_enabled: false };
        this.check_id()?;
        this.enable_nvm()?;

        Ok(this)
    }
}

impl<'a, I2C> Drop for StUsb4500Nvm<'a, I2C>
where
    I2C: i2c::Write<i2c::SevenBitAddress> + i2c::WriteRead<i2c::SevenBitAddress>,
{
    fn drop(&mut self) {
        if self.nvm_enabled {
            if let Err(err) = self.disable_nvm() {
                defmt::warn!("disabling STUSB4500 NVM access failed: {:?}", err);
            }
        }
    }
}
