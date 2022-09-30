//
// OpenEMC bootloader for embedded controllers
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

//! OpenEMC Bootloader.

#![no_std]
#![no_main]

mod backup;
mod board;
mod boards;
mod bootloader;
mod crc32;
mod flash;
mod i2c_reg_slave;
mod i2c_slave;
mod timer;
mod util;
mod watchdog;

use defmt::Format;
use defmt_rtt as _;
use panic_probe as _;

use core::{ffi::c_void, mem::MaybeUninit, num::NonZeroU32};
use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use defmt::unwrap;
use openemc_shared::{BootInfo, ResetStatus};
use stm32f1::stm32f103::Peripherals;

use crate::{
    board::Board,
    bootloader::{BootloaderInfo, BootloaderResult},
    i2c_reg_slave::I2CRegTransaction,
    i2c_slave::I2CSlave,
    util::delay_ms,
};

/// OpenEMC bootloader version.
pub static VERSION: &[u8] = env!("CARGO_PKG_VERSION").as_bytes();

/// Force starting of user program at boot.
const FORCE_START: bool = false;

/// Boot info block.
#[no_mangle]
#[link_section = ".boot_info"]
pub static mut BOOT_INFO: MaybeUninit<BootInfo> = MaybeUninit::uninit();

extern "C" {
    /// Start of user flash (from linker).
    pub static __user_flash_start: c_void;

    /// End of user flash (from linker).
    pub static __user_flash_end: c_void;

    /// Flash page size (from linker).
    pub static __flash_page_size: c_void;
}

/// Start of user flash.
pub fn user_flash_start() -> usize {
    unsafe { &__user_flash_start as *const _ as usize }
}

/// End of user flash.
pub fn user_flash_end() -> usize {
    unsafe { &__user_flash_end as *const _ as usize }
}

/// EMC model.
fn emc_model() -> u8 {
    match option_env!("OPENEMC_BOOTLOADER") {
        Some("big") => 0xd1,
        _ => 0x01,
    }
}

/// Signature value for backup register.
const BACKUP_REG_SIGNATURE_VALUE: u16 = 0xb001;

/// Backup register for signature.
const BACKUP_REG_SIGNATURE: u8 = 0;

/// Backup register for boot reason.
const BACKUP_REG_BOOT_REASON: u8 = 1;

/// Boot reason.
#[repr(u16)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum BootReason {
    /// Unknown boot reason.
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
    /// Power off system and go to standby mode.
    ///
    /// This prevents start of the watchdog timer and causes automatic start of user program.
    PowerOff = 0xb011,
    /// Restart into bootloader.
    ///
    /// This prevents automatic start of user program.
    StartBootloader = 0xb014,
    /// Boot caused by factory reset.
    ///
    /// This clears the backup domain and prevents automatic start of user program.
    FactoryReset = 0xb020,
}

/// Board init result.
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub enum BoardInitResult {
    /// Normal start.
    Normal,
    /// Bootloader should be launched.
    StartBootloader,
    /// Perform factory reset.
    FactoryReset,
}

use boards::Chosen as ThisBoard;

/// Main entry point.
#[entry]
fn main() -> ! {
    let dp = unsafe { Peripherals::steal() };

    // Disable interrupts.
    // This is useful when we enter the bootloader by jumping here
    // instead of a system reset.
    bootloader::disable_interrupts();
    bootloader::clean_and_invalidate_caches();

    // Enable backup domain and check for valid signature.
    backup::enable();
    if backup::read(BACKUP_REG_SIGNATURE) != BACKUP_REG_SIGNATURE_VALUE {
        for i in 0..10 {
            backup::write(i, 0);
        }
        backup::write(BACKUP_REG_SIGNATURE, BACKUP_REG_SIGNATURE_VALUE);
    }

    // Get and update boot reason.
    let boot_reason = backup::read(BACKUP_REG_BOOT_REASON);
    backup::write(BACKUP_REG_BOOT_REASON, BootReason::SurpriseInBootloader as _);

    // Start watchdog.
    if boot_reason != BootReason::PowerOff as _ {
        watchdog::start();
    }

    // Get and clear reset reason.
    let reset_status = ResetStatus::from_rcc_pwr(dp.RCC.csr.read().bits(), dp.PWR.csr.read().bits());
    dp.RCC.csr.modify(|_, w| w.rmvf().clear());
    dp.RCC.csr.modify(|_, w| w.rmvf().clear_bit());
    dp.PWR.cr.modify(|_, w| w.cwuf().set_bit());
    dp.PWR.cr.modify(|_, w| w.cwuf().clear_bit());

    let mut board = ThisBoard::new();

    // Print header.
    defmt::info!("OpenEMC bootloader version {:a}", VERSION);
    defmt::info!("EMC model:    0x{:02x}", emc_model());
    defmt::info!("board model:  {:a}", board.model());
    defmt::info!("I2C address:  0x{:02x} (remap: {:?})", ThisBoard::I2C_ADDR, ThisBoard::I2C_REMAP);
    defmt::info!(
        "user flash:   0x{:08x} - 0x{:08x} (page 0x{:x})",
        user_flash_start(),
        user_flash_end(),
        crate::flash::Flash::page_size()
    );
    defmt::info!("boot reason:  0x{:04x}", boot_reason);
    defmt::info!(
        "reset status: 0x{:02x} (external: {:?}, watchdog reset: {:?})",
        reset_status.0,
        reset_status.is_external(),
        reset_status.is_independent_watchdog()
    );
    defmt::info!("");

    // Initialize board.
    defmt::info!("board init");
    watchdog::pet();
    let board_init = board.init(boot_reason, reset_status);
    defmt::info!("board init result: {:?}", board_init);

    // Perform factory reset if requested by board.
    if board_init == BoardInitResult::FactoryReset {
        defmt::info!("factory reset");
        backup::reset();
        backup::write(BACKUP_REG_SIGNATURE, BACKUP_REG_SIGNATURE_VALUE);
        backup::write(BACKUP_REG_BOOT_REASON, BootReason::FactoryReset as _);
        backup::disable();

        delay_ms(10);
        SCB::sys_reset();
    }

    // Verify current user program.
    watchdog::pet();
    let program = bootloader::verify_program(user_flash_start(), user_flash_end());
    match &program {
        Ok(p) => {
            defmt::info!("user program is valid with CRC32 0x{:x} and id 0x{:x}", p.crc32, p.id);
        }
        Err(_) => {
            defmt::info!("user program is invalid");
        }
    }

    // Check whether to stay in bootloader.
    watchdog::pet();
    let bl_board = board_init == BoardInitResult::StartBootloader;
    let bl_program = program.is_err();
    let bl_boot_reason = boot_reason == BootReason::InvalidUserProgram as _
        || boot_reason == BootReason::SurpriseInUser as _
        || boot_reason == BootReason::SurpriseInBootloader as _
        || boot_reason == BootReason::StartBootloader as _
        || boot_reason == BootReason::FactoryReset as _;
    let bl_reset_cause = (reset_status.is_external() && ThisBoard::PIN_RESET_PREVENTS_AUTORUN)
        || reset_status.is_independent_watchdog()
        || reset_status.is_window_watchdog();
    watchdog::pet();

    // Print status.
    defmt::info!("");
    defmt::info!("enter bootloader due to");
    defmt::info!("    board:           {:?}", bl_board);
    defmt::info!("    program invalid: {:?}", bl_program);
    defmt::info!("    boot reason:     {:?}", bl_boot_reason);
    defmt::info!("    reset cause:     {:?}", bl_reset_cause);
    defmt::info!("");

    // Enter bootloader if required.
    let mut bootloader_result = None;
    if (bl_board || bl_program || bl_boot_reason || bl_reset_cause) && !FORCE_START {
        defmt::info!("entering bootloader");
        watchdog::pet();

        if boot_reason != BootReason::StartBootloader as _ {
            defmt::info!("system power off delay");
            delay_ms(1000);
            watchdog::pet();
        }

        defmt::info!("system power on");
        board.system_power_on();

        // Run bootloader.
        loop {
            defmt::info!("bootloader start");

            let info = BootloaderInfo {
                emc_model: emc_model(),
                board_model: board.model(),
                user_flash_start: user_flash_start(),
                user_flash_end: user_flash_end(),
                timeout_ticks: Some(unwrap!(NonZeroU32::new(ThisBoard::TIMEOUT_TICKS))),
                boot_reason,
                reset_status,
                id: program.map(|p| p.id).unwrap_or_default(),
                extend_fn: |req: I2CRegTransaction| board.bootloader_request(req),
            };

            let i2c_slave = I2CSlave::new(
                ThisBoard::I2C_ADDR,
                (ThisBoard::CPU_CLOCK / 1_000_000) as u8,
                ThisBoard::I2C_REMAP,
            );

            let blr = bootloader::run(info, i2c_slave);
            bootloader_result = Some(blr);
            match blr {
                BootloaderResult::Start => {
                    defmt::info!("user program start request");
                    break;
                }
                BootloaderResult::Timeout => {
                    defmt::info!("bootloader timeout");
                    break;
                }
                BootloaderResult::Reset => {
                    defmt::info!("reset");

                    backup::write(BACKUP_REG_BOOT_REASON, BootReason::StartBootloader as _);
                    backup::disable();

                    delay_ms(10);
                    SCB::sys_reset();
                }
                BootloaderResult::RestartBootloader => (),
            }
        }
    }

    // Verify program and get vector table address.
    defmt::info!("verifying user program");
    watchdog::pet();
    match bootloader::verify_program(user_flash_start(), user_flash_end()) {
        Ok(program) => {
            // Write new boot reason.
            backup::write(BACKUP_REG_BOOT_REASON, BootReason::SurpriseInUser as _);
            backup::disable();

            // Write boot info.
            let board_model = board.model();
            let boot_info = BootInfo {
                signature: BootInfo::SIGNATURE,
                bootloader_version: VERSION.as_ptr(),
                bootloader_version_len: VERSION.len() as _,
                emc_model: emc_model(),
                board_model: board_model.as_ptr(),
                board_model_len: board_model.len() as _,
                i2c_addr: ThisBoard::I2C_ADDR,
                i2c_remap: ThisBoard::I2C_REMAP,
                irq_pin: ThisBoard::IRQ_PIN,
                irq_pin_cfg: ThisBoard::IRQ_PIN_CFG,
                boot_reason,
                reset_status,
                start_reason: bootloader_result.map(|blr| blr as u8).unwrap_or_default(),
                id: program.id,
                reserved: Default::default(),
                board_data: board.board_data(),
            };
            unsafe {
                BOOT_INFO.write(boot_info);
            }

            // Disable all peripherals that we can disable.
            dp.RCC.apb2enr.modify(|_, w| w.afioen().disabled());
            dp.RCC.apb1enr.modify(|_, w| w.pwren().disabled());
            board.pre_start();

            // Start user program.
            defmt::info!("user program start");
            watchdog::pet();
            delay_ms(10);
            bootloader::start_program(user_flash_start());
        }
        Err(_err) => {
            defmt::info!("user program is invalid: {:?}", _err);

            backup::write(BACKUP_REG_BOOT_REASON, BootReason::InvalidUserProgram as _);
            backup::disable();

            delay_ms(10);
            SCB::sys_reset();
        }
    }
}
