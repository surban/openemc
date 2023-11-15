//! Flash utility functions.

use core::ptr;
use defmt::panic;
use stm32f1xx_hal::flash::{FlashSize, FlashWriter, SectorSize, FLASH_START};

use openemc_shared::flash;

/// 96-bit unique device identifier.
pub fn unique_device_id() -> u128 {
    const BASE: usize = 0x1FFFF7E8;
    unsafe {
        let a = ptr::read_volatile(BASE as *const u16) as u128;
        let b = ptr::read_volatile((BASE + 2) as *const u16) as u128;
        let c = ptr::read_volatile((BASE + 4) as *const u32) as u128;
        let d = ptr::read_volatile((BASE + 8) as *const u32) as u128;
        a | (b << 16) | (c << 32) | (d << 64)
    }
}

/// Flash utility function.
pub trait FlashUtil {
    /// Create flash writer.
    #[allow(clippy::new_ret_no_self)]
    fn new(parts: &mut stm32f1xx_hal::flash::Parts) -> FlashWriter;

    /// Read and panic if failed.
    fn read_unwrap(&self, addr: usize, length: usize) -> &[u8];

    /// Read from flash into provided buffer.
    fn read_into(&self, addr: usize, buf: &mut [u8]);

    /// Erase and panic if failed.
    fn erase_unwrap(&mut self, addr: usize, length: usize);

    /// Write and panic if failed.
    fn write_unwrap(&mut self, addr: usize, data: &[u8]);
}

impl<'a> FlashUtil for FlashWriter<'a> {
    fn new(parts: &mut stm32f1xx_hal::flash::Parts) -> FlashWriter {
        let sector_size = match flash::page_size() {
            1024 => SectorSize::Sz1K,
            2048 => SectorSize::Sz2K,
            4096 => SectorSize::Sz4K,
            size => panic!("unknown flash page size {} bytes", size),
        };

        let flash_size = match flash::size() {
            16_384 => FlashSize::Sz16K,
            32_768 => FlashSize::Sz32K,
            65_536 => FlashSize::Sz64K,
            131_072 => FlashSize::Sz128K,
            262_144 => FlashSize::Sz256K,
            393_216 => FlashSize::Sz384K,
            524_288 => FlashSize::Sz512K,
            786_432 => FlashSize::Sz768K,
            1_048_576 => FlashSize::Sz1M,
            size => panic!("unknown flash size {} bytes", size),
        };

        parts.writer(sector_size, flash_size)
    }

    fn read_unwrap(&self, addr: usize, length: usize) -> &[u8] {
        let Ok(data) = self.read(addr as u32 - FLASH_START, length) else {
            panic!("flash read of length {} at 0x{:x} failed", length, addr);
        };
        data
    }

    fn read_into(&self, addr: usize, buf: &mut [u8]) {
        let len = buf.len();
        let data = self.read_unwrap(addr, len);
        buf.copy_from_slice(data);
    }

    fn erase_unwrap(&mut self, addr: usize, length: usize) {
        let Ok(()) = self.erase(addr as u32 - FLASH_START, length) else {
            panic!("flash erase of length {} at 0x{:x} failed", length, addr);
        };
    }

    fn write_unwrap(&mut self, addr: usize, data: &[u8]) {
        let Ok(()) = self.write(addr as u32 - FLASH_START, data) else {
            panic!("flash write of length {} at 0x{:x} failed", data.len(), addr);
        };
    }
}
