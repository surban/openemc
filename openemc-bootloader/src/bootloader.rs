//! Bootloader with an I2C interface.

use core::{mem::size_of, num::Saturating, ptr};
use cortex_m::peripheral::{scb::Exception, SCB};
use defmt::Format;

use crate::{
    crc32::Crc32,
    flash_writer::{FlashWriter, UnlockError},
    i2c_reg_slave::{I2CRegSlave, I2CRegTransaction},
    i2c_slave::I2CSlave,
    timer::Timer4,
    watchdog,
};
use openemc_shared::{
    boot::{ResetStatus, PROGRAM_SIGNATURE},
    flash,
};

/// Code for unlocking writing to flash.
const MODIFY_UNLOCK_CODE: [u8; 4] = [0xb0, 0xf0, 0x01, 0xaa];

/// Bootloader id.
const ID: u8 = 0xb0;

/// Read id (u8).
const REG_ID: u8 = 0x00;

/// Read EMC model (u8).
const REG_EMC_MODEL: u8 = 0x02;

/// Read board model (string).
const REG_BOARD_MODEL: u8 = 0x03;

/// Read bootloader version (string).
const REG_BOOTLOADER_VERSION: u8 = 0x04;

/// Reads the boot reason (u16).
const REG_BOOT_REASON: u8 = 0x08;

/// Reads the reset status (u8).
const REG_RESET_STATUS: u8 = 0x09;

/// Reads the CRC32 of the user program (u32).
const REG_PROGRAM_ID: u8 = 0x0b;

/// Read flash start address (u32).
const REG_FLASH_START: u8 = 0x10;

/// Read flash end address (not inclusive) (u32).
const REG_FLASH_END: u8 = 0x11;

/// Read flash page size (u32).
const REG_FLASH_PAGE_SIZE: u8 = 0x12;

/// Read or write flash address (u32), must be 16-bit aligned.
const REG_FLASH_ADDR: u8 = 0x13;

/// Read from flash memory at the current address.
/// Increments the address after each read.
const REG_FLASH_READ: u8 = 0x14;

/// Reads the CRC32 of the flash page specified by the
/// current address. (4 bytes)
const REG_FLASH_PAGE_CRC32: u8 = 0x15;

/// Write the code 0xb0 0xf0 0x01 0xaa to unlock writing to flash memory.
/// Write 0x00 0x00 0x00 0x00 to lock writing to flash memory.
/// Reads the status (1=unlocked, 0=locked).
const REG_FLASH_MODIFY_UNLOCK: u8 = 0x16;

/// Erases the flash page specified by the current address.
const REG_FLASH_ERASE_PAGE: u8 = 0x17;

/// Writes to flash memory at the current address (increments of 2 bytes).
/// Increments the address by 2 for each write.
const REG_FLASH_WRITE: u8 = 0x18;

/// Read status of erase or write operation (0x00=okay, 0x01=error).
/// Write empty to clear.
const REG_FLASH_STATUS: u8 = 0x19;

/// Starts the main program.
const REG_START_PROGRAM: u8 = 0x20;

/// Verifies the main program.
/// Wait 100 ms after issuing this command.
const REG_VERIFY_PROGRAM: u8 = 0x21;

/// Reads the verification result of the main program.
/// 0x00=okay, 0x01=signature error, 0xff=unverified.
const REG_VERIFY_PROGRAM_RESULT: u8 = 0x22;

/// Enables or disables the bootloader timeout (0x00=disabled, 0x01=enabled).
const REG_TIMEOUT_ENABLED: u8 = 0x30;

/// Resets the system.
const REG_RESET: u8 = 0xf0;

/// Checksum enable.
///
/// Read: enabled (u8).
/// Checksum is always disabled in bootloader.
const REG_CHECKSUM_ENABLE: u8 = 0xf7;

/// Restarts the bootloader.
const REG_START_BOOTLOADER: u8 = 0xff;

/// Bootloader information.
pub struct BootloaderInfo<'a, Ext, Idle> {
    /// EMC model id.
    pub emc_model: u8,
    /// Board model id.
    pub board_model: &'a [u8],
    /// User flash start address.
    pub user_flash_start: usize,
    /// User flash end address (non-inclusive).
    pub user_flash_end: usize,
    /// Processor clock speed in Hz.
    pub cpu_clock: u32,
    /// Boot reason.
    pub boot_reason: u16,
    /// Reset status.
    pub reset_status: ResetStatus,
    /// Id of user program.
    pub id: u32,
    /// Extended I2C commands.
    pub extend_fn: Ext,
    /// Function to call while idle.
    pub idle_fn: Idle,
}

/// Result of running the bootloader.
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Format)]
pub enum BootloaderResult {
    /// Start the user program.
    Start = 0x01,
    /// Timeout.
    Timeout = 0x02,
    /// Reset the system.
    Reset = 0xf0,
    /// Restart the bootloader.
    RestartBootloader = 0xff,
}

/// Runs the boot loader.
pub fn run<Ext, Idle>(mut info: BootloaderInfo<Ext, Idle>, i2c_slave: I2CSlave) -> BootloaderResult
where
    Ext: FnMut(I2CRegTransaction),
    Idle: FnMut(u16, u16, bool) -> Option<BootloaderResult>,
{
    let mut i2c = I2CRegSlave::new(i2c_slave);
    let mut flash_addr = info.user_flash_start;
    let mut writer = None;
    let mut flash_fail = false;
    let mut verify_result = None;

    let mut timeout_enabled = true;
    let mut total_seconds = Saturating(0);
    let mut idle_seconds = Saturating(0);

    // Program timer tick for one second.
    let mut timer = Timer4::new();
    timer.set_prescaler(u16::MAX);
    timer.set_auto_reload((info.cpu_clock / u16::MAX as u32) as _);
    timer.set_one_pulse_mode(true);
    timer.generate_update();
    timer.set_enabled(true);

    loop {
        watchdog::pet();
        if let Some(res) = (info.idle_fn)(total_seconds.0, idle_seconds.0, timeout_enabled) {
            defmt::info!(
                "bootloader exit with result {:?} after {} seconds with {} seconds idle",
                res,
                total_seconds.0,
                idle_seconds.0
            );
            return res;
        }

        watchdog::pet();
        let mut transacted = true;
        match i2c.try_accept() {
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_ID => {
                tx.send_u8(ID);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_BOOTLOADER_VERSION => {
                let _ = tx.send_all(crate::VERSION);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_EMC_MODEL => {
                tx.send_u8(info.emc_model);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_BOARD_MODEL => {
                let _ = tx.send_all(info.board_model);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_BOOT_REASON => {
                tx.send_u16(info.boot_reason);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_RESET_STATUS => {
                tx.send_u8(info.reset_status.0);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_PROGRAM_ID => {
                tx.send_u32(info.id);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_START => {
                tx.send_u32(info.user_flash_start as u32);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_END => {
                tx.send_u32(info.user_flash_end as u32);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_PAGE_SIZE => {
                tx.send_u32(flash::page_size() as u32);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_ADDR => {
                tx.send_u32(flash_addr as u32);
            }
            Some(I2CRegTransaction::Write(mut rx)) if rx.reg() == REG_FLASH_ADDR => {
                flash_addr = rx.recv_u32() as usize;
                defmt::debug!("bootloader: setting flash_addr to 0x{:08x}", flash_addr);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_READ => {
                while info.user_flash_start <= flash_addr && flash_addr < info.user_flash_end {
                    let data = unsafe { ptr::read_unaligned(flash_addr as *const u8) };
                    if tx.send(data).is_err() {
                        break;
                    }
                    flash_addr += 1;
                }
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_PAGE_CRC32 => {
                flash_addr = flash::page_base(flash_addr);
                if info.user_flash_start <= flash_addr && flash_addr < info.user_flash_end {
                    defmt::debug!("bootloader: calculate CRC32 of page 0x{:08x}", flash_addr);

                    let mut crc = Crc32::new();
                    for addr in (flash_addr..(flash_addr + flash::page_size())).step_by(size_of::<u32>()) {
                        let data = unsafe { ptr::read_volatile(addr as *const u32) };
                        crc.write(data);
                    }

                    let crc32 = crc.crc32();
                    defmt::info!("bootloader: CRC32 of page is 0x{:08x}", crc32);
                    tx.send_u32(crc32);
                }
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_MODIFY_UNLOCK => {
                tx.send_u8(if writer.is_some() { 0xff } else { 0x00 });
            }
            Some(I2CRegTransaction::Write(mut rx)) if rx.reg() == REG_FLASH_MODIFY_UNLOCK => {
                let code = [rx.recv_u8(), rx.recv_u8(), rx.recv_u8(), rx.recv_u8()];
                if code == MODIFY_UNLOCK_CODE {
                    match FlashWriter::new() {
                        Ok(w) => {
                            defmt::debug!("bootloader: flashing unlocked");
                            writer = Some(w);
                            info.id = 0;
                        }
                        Err(UnlockError) => {
                            defmt::warn!("bootloader: flash unlocking failed");
                        }
                    }
                } else {
                    defmt::debug!("bootloader: flashing locked");
                    writer = None;
                }
            }
            Some(I2CRegTransaction::Write(rx)) if rx.reg() == REG_FLASH_ERASE_PAGE => match &mut writer {
                Some(writer) if info.user_flash_start <= flash_addr && flash_addr < info.user_flash_end => {
                    verify_result = None;
                    flash_addr = flash::page_base(flash_addr);
                    defmt::info!("bootloader: erase page 0x{:08x}", flash_addr);
                    flash_fail |= writer.erase_page(flash_addr).is_err();
                }
                _ => {
                    defmt::warn!("bootloader: trying to erase outside of flash range 0x{:08x}", flash_addr);
                    flash_fail = true;
                }
            },
            Some(I2CRegTransaction::Write(mut rx)) if rx.reg() == REG_FLASH_WRITE => match &mut writer {
                Some(writer) => {
                    verify_result = None;
                    while info.user_flash_start <= flash_addr && flash_addr < info.user_flash_end {
                        let data = match (rx.recv(), rx.recv()) {
                            (Some(a), Some(b)) => ((b as u16) << 8) | (a as u16),
                            _ => break,
                        };

                        defmt::debug!("bootloader: write 0x{:04x} to 0x{:08x}", data, flash_addr);
                        flash_fail |= writer.write(flash_addr, data).is_err();
                        flash_addr += size_of::<u16>();
                    }
                }
                _ => {
                    defmt::warn!("bootloader: trying to flash outside of flash range 0x{:08x}", flash_addr);
                    flash_fail = true;
                }
            },
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_FLASH_STATUS => {
                tx.send_u8(u8::from(flash_fail));
            }
            Some(I2CRegTransaction::Write(tx)) if tx.reg() == REG_FLASH_STATUS => {
                defmt::debug!("bootloader: clearing flash_fail");
                flash_fail = false;
            }
            Some(I2CRegTransaction::Write(rx)) if rx.reg() == REG_VERIFY_PROGRAM => {
                let res = verify_program(info.user_flash_start, info.user_flash_end);
                defmt::info!("bootloader: verify program result is {:?}", res);
                verify_result = Some(res);
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_VERIFY_PROGRAM_RESULT => {
                tx.send_u8(match verify_result {
                    Some(Ok(_)) => 0x00,
                    Some(Err(_)) => 0x01,
                    None => 0xff,
                });
            }
            Some(I2CRegTransaction::Write(rx)) if rx.reg() == REG_START_PROGRAM => {
                rx.discard();
                return BootloaderResult::Start;
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_TIMEOUT_ENABLED => {
                tx.send_u8(timeout_enabled as _);
            }
            Some(I2CRegTransaction::Write(mut rx)) if rx.reg() == REG_TIMEOUT_ENABLED => {
                timeout_enabled = rx.recv_u8() != 0;
                defmt::info!("bootloader: set timeout_enabled to {:?}", timeout_enabled);
            }
            Some(I2CRegTransaction::Write(rx)) if rx.reg() == REG_RESET => {
                rx.discard();
                return BootloaderResult::Reset;
            }
            Some(I2CRegTransaction::Read(mut tx)) if tx.reg() == REG_CHECKSUM_ENABLE => {
                tx.send_u8(0);
            }
            Some(I2CRegTransaction::Write(rx)) if rx.reg() == REG_CHECKSUM_ENABLE => {
                rx.discard();
            }
            Some(I2CRegTransaction::Write(rx)) if rx.reg() == REG_START_BOOTLOADER => {
                rx.discard();
                return BootloaderResult::RestartBootloader;
            }
            Some(other) => (info.extend_fn)(other),
            None => {
                transacted = false;
            }
        }

        // Update times.
        if !timer.is_enabled() {
            idle_seconds += 1;
            total_seconds += 1;
            timer.generate_update();
            timer.set_enabled(true);
            defmt::debug!("bootloader total time: {} s    idle time: {} s", total_seconds.0, idle_seconds.0);
        }
        if transacted {
            idle_seconds.0 = 0;
        }
    }
}

/// Program signature matching.
#[derive(Clone, Copy, PartialEq, Eq, Format)]
enum SignatureMatch {
    Sig(usize),
    Start,
    Length,
    Crc32,
    Id,
    Signed { id: u32 },
}

impl Default for SignatureMatch {
    fn default() -> Self {
        Self::Sig(0)
    }
}

impl SignatureMatch {
    fn next(self, data: u32, start: usize, len: usize, crc: u32) -> Self {
        match (self, data) {
            (Self::Sig(i), s) if s == PROGRAM_SIGNATURE[i] => {
                if i < PROGRAM_SIGNATURE.len() - 1 {
                    Self::Sig(i + 1)
                } else {
                    Self::Start
                }
            }
            (Self::Start, s) if s as usize == start => Self::Length,
            (Self::Length, l) if l as usize == len => Self::Crc32,
            (Self::Crc32, c) if c == crc => Self::Id,
            (Self::Id, id) => Self::Signed { id },
            (_, s) if s == PROGRAM_SIGNATURE[0] => Self::Sig(1),
            _ => Self::Sig(0),
        }
    }

    fn is_signed(&self) -> bool {
        matches!(self, Self::Signed { .. })
    }
}

/// Verified program.
#[derive(Copy, Clone, Format)]
pub struct VerifiedProgram {
    /// Length of program.
    pub len: usize,
    /// CRC32 of program.
    pub crc32: u32,
    /// Unique id of program.
    pub id: u32,
}

/// Finds and verifies the program signature.
///
/// Signature is expected to follow after the user program and has the following form of u32s:
///   * 6 signature literals
///   * user flash start address
///   * length in bytes so far
///   * CRC32 of data so far
///   * 32-bit id
pub fn verify_program(
    user_flash_start: usize, user_flash_end: usize,
) -> Result<VerifiedProgram, ProgramNotSigned> {
    defmt::assert!(flash::is_page_aligned(user_flash_start));

    let mut crc = Crc32::new();
    let mut sig = SignatureMatch::default();
    let mut len = 0;

    for addr in (user_flash_start..user_flash_end).step_by(size_of::<u32>()) {
        let last_sig = sig;

        let data = unsafe { ptr::read_volatile(addr as *const u32) };

        if !sig.is_signed() {
            sig = sig.next(data, user_flash_start, len, crc.crc32());
        } else if flash::is_page_aligned(addr) {
            break;
        }

        len += size_of::<u32>();
        crc.write(data);

        if sig != last_sig {
            defmt::trace!("signature match: {:?}", sig);
        }
    }

    match sig {
        SignatureMatch::Signed { id } => Ok(VerifiedProgram { len, crc32: crc.crc32(), id }),
        _ => Err(ProgramNotSigned),
    }
}

/// Program signature not found.
#[derive(Clone, Copy, Format)]
pub struct ProgramNotSigned;

/// Starts the user program.
pub fn start_program(vector_table: usize) -> ! {
    disable_interrupts();
    clean_and_invalidate_caches();

    unsafe {
        let cp = cortex_m::Peripherals::steal();
        cp.SCB.vtor.write(vector_table as u32);
        cortex_m::asm::bootload(vector_table as *const u32);
    }
}

/// Disables and clears all interrupts and fault handlers.
pub fn disable_interrupts() {
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cortex_m::interrupt::disable();

    // Disable and clear SysTick interrupt.
    cp.SYST.disable_interrupt();
    cp.SYST.disable_counter();
    SCB::clear_pendst();

    // Disable and clear all interrupts.
    for icer in cp.NVIC.icer.iter() {
        unsafe { icer.write(0xffffffff) };
    }
    for icpr in cp.NVIC.icpr.iter() {
        unsafe { icpr.write(0xffffffff) };
    }

    // Disable fault handlers.
    cp.SCB.disable(Exception::BusFault);
    cp.SCB.disable(Exception::UsageFault);
    cp.SCB.disable(Exception::MemoryManagement);
}

/// Cleans and invalidates the data and instruction caches.
pub fn clean_and_invalidate_caches() {
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.SCB.clean_invalidate_dcache(&mut cp.CPUID);
    cp.SCB.invalidate_icache();
}
