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

//! OpenEMC shared code.

#![no_std]

use defmt::Format;

/// Bootloader log size.
pub const BOOTLOADER_LOG_SIZE: usize = 1024;

/// Log buffer size.
pub const LOG_SIZE: usize = 8192;

/// Information about the boot for the user program.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct BootInfo {
    /// Signature.
    pub signature: u32,
    /// Bootloader version.
    pub bootloader_version: *const u8,
    /// Length of bootloader version.
    pub bootloader_version_len: u8,
    /// EMC model.
    pub emc_model: u8,
    /// Board model.
    pub board_model: *const u8,
    /// Length of board model.
    pub board_model_len: u8,
    /// Boot reason.
    pub boot_reason: u16,
    /// Reset status.
    pub reset_status: ResetStatus,
    /// Start reason.
    pub start_reason: u8,
    /// Id of booted firmware.
    pub id: u32,
    /// Reserved space.
    pub reserved: [u8; Self::RESERVED_SIZE],
    /// Length of board data.
    pub board_data_len: u8,
    /// Board-specific data.
    pub board_data: [u8; Self::BOARD_DATA_SIZE],
}

impl BootInfo {
    /// Value for bootloader signature.
    pub const SIGNATURE: u32 = 0xb001b001;

    /// Reserved value.
    pub const RESERVED: [u8; Self::RESERVED_SIZE] = [0; Self::RESERVED_SIZE];

    /// Reserved size.
    pub const RESERVED_SIZE: usize = 32;

    /// Empty board data.
    pub const EMPTY_BOARD_DATA: [u8; Self::BOARD_DATA_SIZE] = [0; Self::BOARD_DATA_SIZE];

    /// Board data size.
    pub const BOARD_DATA_SIZE: usize = 64;

    /// Valid board data.
    pub fn board_data(&self) -> &[u8] {
        let len = usize::from(self.board_data_len).min(Self::BOARD_DATA_SIZE);
        &self.board_data[..len]
    }
}

/// Program signature.
pub const PROGRAM_SIGNATURE: [u32; 6] = [0xecececec, 0x84190902, 0x6e65704f, 0x20434d45, 0x764c5047, 0x00000033];

/// Reset status.
#[repr(C)]
#[derive(Default, Clone, Copy, PartialEq, Eq, Format)]
pub struct ResetStatus(pub u8);

impl ResetStatus {
    /// Wakeup occurred.
    pub const WAKEUP: u8 = 1 << 1;

    /// External reset.
    pub const EXTERNAL: u8 = 1 << 2;

    /// Power on.
    pub const POWER_ON: u8 = 1 << 3;

    /// Software reset.
    pub const FIRMWARE: u8 = 1 << 4;

    /// Independent watchdog reset.
    pub const INDEPENDENT_WATCHDOG: u8 = 1 << 5;

    /// Window watchdog reset.
    pub const WINDOW_WATCHDOG: u8 = 1 << 6;

    /// Low power reset.
    pub const LOW_POWER: u8 = 1 << 7;

    /// Create from RCC CSR and PWR CR register values.
    pub fn from_rcc_pwr(rcc_csr: u32, pwr_cr: u32) -> Self {
        let mut value = (rcc_csr >> 24) as u8;

        if pwr_cr & 0b11 != 0 {
            value |= Self::WAKEUP;
        }

        Self(value)
    }

    /// Wakeup occurred.
    pub fn is_wakeup(&self) -> bool {
        self.0 & Self::WAKEUP != 0
    }

    /// External reset.
    pub fn is_external(&self) -> bool {
        (self.0 & 0b11111100) == Self::EXTERNAL
    }

    /// Power on.
    pub fn is_power_on(&self) -> bool {
        self.0 & Self::POWER_ON != 0
    }

    /// Software reset.
    pub fn is_firmware(&self) -> bool {
        self.0 & Self::FIRMWARE != 0
    }

    /// Independent watchdog reset.
    pub fn is_independent_watchdog(&self) -> bool {
        self.0 & Self::INDEPENDENT_WATCHDOG != 0
    }

    /// Window watchdog reset.
    pub fn is_window_watchdog(&self) -> bool {
        self.0 & Self::WINDOW_WATCHDOG != 0
    }

    /// Low power reset.
    pub fn is_low_power(&self) -> bool {
        self.0 & Self::LOW_POWER != 0
    }

    /// Logs the reset status.
    pub fn log(&self) {
        defmt::info!(
            "reset status:   0x{:02x} {}{}{}{}{}{}{}",
            self.0,
            if self.is_wakeup() { "wakeup " } else { "" },
            if self.is_external() { "external " } else { "" },
            if self.is_power_on() { "power-on " } else { "" },
            if self.is_firmware() { "firmware " } else { "" },
            if self.is_independent_watchdog() { "independent-watchdog " } else { "" },
            if self.is_window_watchdog() { "window-watchdog " } else { "" },
            if self.is_low_power() { "low-power " } else { "" },
        );
    }
}

/// Boot reason.
#[allow(dead_code)]
#[repr(u16)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum BootReason {
    /// Unknown boot reason.
    ///
    /// Most likely caused by loss of power to backup domain.
    Unknown = 0x0000,
    /// Surprise reboot while in bootloader.
    ///
    /// This is set by the bootloader and prevents automatic start of user program.
    SurpriseInBootloader = 0xb000,
    /// Surprise reboot while in user program.
    ///
    /// This is set by the bootloader and prevents automatic start of user program.
    SurpriseInUser = 0xb001,
    /// Boot caused by invalid user program.
    ///
    /// This is set by the bootloader and prevents automatic start of user program.
    InvalidUserProgram = 0xb003,
    /// Boot caused by power on event.
    ///
    /// This causes automatic start of user program.
    PowerOn = 0xb010,
    /// Power off system and go to standby mode.
    PowerOff = 0xb011,
    /// Power off system and then restart it.
    Restart = 0xb012,
    /// Reset of EMC.
    Reset = 0xb013,
    /// Restart into bootloader.
    ///
    /// This prevents automatic start of user program.
    StartBootloader = 0xb014,
    /// Boot caused by factory reset.
    ///
    /// This clears the backup domain and prevents automatic start of user program.
    FactoryReset = 0xb020,
    /// Timeout of user controlled watchdog.
    WatchdogTimeout = 0xb030,
}

impl BootReason {
    /// Logs the boot reason.
    pub fn log(boot_reason: u16) {
        let reason = match boot_reason {
            v if v == Self::Unknown as _ => "unknown",
            v if v == Self::SurpriseInBootloader as _ => "surprise in boot loader",
            v if v == Self::SurpriseInUser as _ => "surprise in user program",
            v if v == Self::InvalidUserProgram as _ => "invalid user program",
            v if v == Self::PowerOn as _ => "power on",
            v if v == Self::PowerOff as _ => "power off",
            v if v == Self::Restart as _ => "restart",
            v if v == Self::Reset as _ => "reset",
            v if v == Self::StartBootloader as _ => "start boot loader",
            v if v == Self::FactoryReset as _ => "factory reset",
            v if v == Self::WatchdogTimeout as _ => "watchdog timeout",
            _ => "",
        };
        defmt::info!("boot reason:    0x{:04x} {}", boot_reason, reason,);
    }
}
