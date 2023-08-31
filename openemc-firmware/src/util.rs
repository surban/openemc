//! Debug functions.

use core::{mem::size_of, ptr};
use defmt::{panic, unwrap};
use stm32f1xx_hal::flash::{self, FlashSize, FlashWriter, SectorSize, FLASH_START};

use crate::{flash_end, flash_page_size};

// use crate::CPU_CLOCK;
//
// /// Approximately delay execution by specified amount of milliseconds.
// pub fn delay_ms(ms: u32) {
//     let cycles = ms * (crate::CPU_CLOCK / 1000);
//     cortex_m::asm::delay(cycles)
// }

/// Byte representation of an array of u64s.
pub fn array_from_u64<const D: usize, const F: usize>(data: &[u64; D]) -> [u8; F] {
    defmt::assert!(F == D * size_of::<u64>());

    let mut flat = [0; F];
    let mut pos = 0;
    for d in data {
        for f in d.to_le_bytes() {
            flat[pos] = f;
            pos += 1;
        }
    }
    flat
}

/// Convert byte representation to an array of u64s.
pub fn array_to_u64<const D: usize>(flat: &[u8]) -> [u64; D] {
    defmt::assert!(flat.len() == D * size_of::<u64>());

    let mut data = [0; D];
    for (i, d) in data.iter_mut().enumerate() {
        let b: [u8; size_of::<u64>()] =
            unwrap!(flat[i * size_of::<u64>()..(i + 1) * size_of::<u64>()].try_into());
        *d = u64::from_le_bytes(b);
    }
    data
}

/// Byte representation of an array of u16s.
pub fn array_from_u16<const D: usize, const F: usize>(data: &[u16; D]) -> [u8; F] {
    defmt::assert!(F == D * size_of::<u16>());

    let mut flat = [0; F];
    let mut pos = 0;
    for d in data {
        for f in d.to_le_bytes() {
            flat[pos] = f;
            pos += 1;
        }
    }
    flat
}

/// Convert byte representation to an array of u16s.
pub fn array_to_u16<const D: usize>(flat: &[u8]) -> [u16; D] {
    defmt::assert!(flat.len() == D * size_of::<u16>());

    let mut data = [0; D];
    for (i, d) in data.iter_mut().enumerate() {
        let b: [u8; size_of::<u16>()] =
            unwrap!(flat[i * size_of::<u16>()..(i + 1) * size_of::<u16>()].try_into());
        *d = u16::from_le_bytes(b);
    }
    data
}

/// 96-bit unique device identifier.
pub fn unique_device_id() -> u128 {
    const BASE: usize = 0x1FFFF7E8;
    unsafe {
        let a = ptr::read_unaligned(BASE as *const u16) as u128;
        let b = ptr::read_unaligned((BASE + 2) as *const u16) as u128;
        let c = ptr::read_unaligned((BASE + 4) as *const u32) as u128;
        let d = ptr::read_unaligned((BASE + 8) as *const u32) as u128;
        a | (b << 16) | (c << 32) | (d << 64)
    }
}

/// Flash utility function.
pub trait FlashUtil {
    /// Create flash writer.
    #[allow(clippy::new_ret_no_self)]
    fn new(parts: &mut flash::Parts) -> FlashWriter;

    /// Read and panic if failed.
    fn read_unwrap(&self, offset: u32, length: usize) -> &[u8];

    /// Read from flash into provided buffer.
    fn read_into(&self, offset: u32, buf: &mut [u8]);

    /// Erase and panic if failed.
    fn erase_unwrap(&mut self, offset: u32, length: usize);

    /// Write and panic if failed.
    fn write_unwrap(&mut self, offset: u32, data: &[u8]);
}

impl<'a> FlashUtil for FlashWriter<'a> {
    fn new(parts: &mut flash::Parts) -> FlashWriter {
        let sector_size = match flash_page_size() {
            1024 => SectorSize::Sz1K,
            2048 => SectorSize::Sz2K,
            4096 => SectorSize::Sz4K,
            _ => panic!("unknown flash page size"),
        };

        let flash_size = match flash_end() - FLASH_START as usize {
            16_384 => FlashSize::Sz16K,
            32_768 => FlashSize::Sz32K,
            65_536 => FlashSize::Sz64K,
            131_072 => FlashSize::Sz128K,
            262_144 => FlashSize::Sz256K,
            393_216 => FlashSize::Sz384K,
            524_288 => FlashSize::Sz512K,
            786_432 => FlashSize::Sz768K,
            1_048_576 => FlashSize::Sz1M,
            _ => panic!("unknown flash size"),
        };

        parts.writer(sector_size, flash_size)
    }

    fn read_unwrap(&self, offset: u32, length: usize) -> &[u8] {
        let Ok(data) = self.read(offset, length) else {
            panic!("flash read of length {} at offset 0x{:x} failed", length, offset);
        };
        data
    }

    fn read_into(&self, offset: u32, buf: &mut [u8]) {
        let len = buf.len();
        let data = self.read_unwrap(offset, len);
        buf.copy_from_slice(data);
    }

    fn erase_unwrap(&mut self, offset: u32, length: usize) {
        let Ok(()) = self.erase(offset, length) else {
            panic!("flash erase of length {} at offset 0x{:x} failed", length, offset);
        };
    }

    fn write_unwrap(&mut self, offset: u32, data: &[u8]) {
        let Ok(()) = self.write(offset, data) else {
            panic!("flash write of length {} at offset 0x{:x} failed", data.len(), offset);
        };
    }
}
