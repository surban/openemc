//! Data stored in flash memory.

use core::{
    mem::{size_of, zeroed, MaybeUninit},
    ops::Deref,
    slice,
};
use defmt::assert;
use stm32f1xx_hal::{crc::Crc, flash::FlashWriter};

use crate::{crc::crc32, flash_util::FlashUtil};
use openemc_shared::flash;

/// Data storable in flash.
pub trait FlashData: Sized + Copy + Eq + Sized + 'static {
    /// Validate and fix data as needed so that it becomes a valid Rust value.
    fn validate(this: MaybeUninit<Self>) -> Self;
    /// Notify data that it was changed (for logging).
    fn notify_changed(&self) {}
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum FlashCopy {
    Copy1,
    Copy2,
}

/// Data stored in flash.
pub struct FlashBackened<T> {
    flash_addr1: usize,
    flash_addr2: usize,
    reserved: usize,
    data: T,
    version: u8,
    next_save: FlashCopy,
}

impl<T> FlashBackened<T>
where
    T: FlashData,
{
    const HEADER_SIZE: usize = 8;

    /// Create copies at specified addresses in flash.
    pub fn new(
        flash: &mut FlashWriter, crc: &mut Crc, flash_addr1: usize, flash_addr2: usize, reserved: usize,
        erase: bool,
    ) -> Self {
        assert!(size_of::<T>() % 2 == 0, "data must have even size");
        assert!(reserved % flash::page_size() == 0, "reserved size must be a multiple of flash page size");
        assert!(
            reserved >= size_of::<T>() + Self::HEADER_SIZE,
            "reserved size must be at least size of data plus header size"
        );
        assert!(flash::is_page_aligned(flash_addr1), "flash copy 1 is not aligned to page boundary");
        assert!(flash::is_page_aligned(flash_addr2), "flash copy 2 is not aligned to page boundary");

        let mut this = Self {
            flash_addr1,
            flash_addr2,
            reserved,
            data: T::validate(unsafe { zeroed() }),
            version: 0,
            next_save: FlashCopy::Copy1,
        };

        if erase {
            this.erase(flash);
        } else {
            this.load(flash, crc);
        }

        this
    }

    /// Read copy from flash and verify CRC32.
    fn load_from<'a>(
        flash: &'a FlashWriter, crc: &mut Crc, addr: usize, reserved: usize,
    ) -> Option<(u8, &'a MaybeUninit<T>)> {
        let mut crc_buf = [0; 4];
        flash.read_into(addr, &mut crc_buf);
        let crc_read = u32::from_le_bytes(crc_buf);

        let protected = flash.read_unwrap(addr + 4, reserved - 4);
        if crc_read != crc32(crc, protected) {
            return None;
        }

        let data = &protected[4..];
        let (head, body, _) = unsafe { data.align_to::<MaybeUninit<T>>() };
        assert!(head.is_empty());
        Some((protected[0], &body[0]))
    }

    /// Save copy to flash protected with CRC32 checksum.
    fn save_to(flash: &mut FlashWriter, crc: &mut Crc, addr: usize, version: u8, reserved: usize, value: &T) {
        flash.erase_unwrap(addr, reserved);

        flash.write_unwrap(addr + 4, &[version, 0, 0, 0]);

        let data_pos = addr + Self::HEADER_SIZE;
        let data = unsafe { slice::from_raw_parts(value as *const T as *const u8, size_of::<T>()) };
        flash.write_unwrap(data_pos, data);

        for pos in (data_pos + data.len()..addr + reserved).step_by(2) {
            flash.write_unwrap(pos, &[0, 0]);
        }

        let protected = flash.read_unwrap(addr + 4, reserved - 4);
        let crc_buf = crc32(crc, protected).to_le_bytes();
        flash.write_unwrap(addr, &crc_buf);
    }

    /// Load data from flash.
    pub fn load(&mut self, flash: &mut FlashWriter, crc: &mut Crc) {
        let data1 = Self::load_from(flash, crc, self.flash_addr1, self.reserved);
        let data2 = Self::load_from(flash, crc, self.flash_addr2, self.reserved);

        let (version, data, next_save) = match (data1, data2) {
            (Some((version1, data1)), None) => (version1, *data1, FlashCopy::Copy2),
            (None, Some((version2, data2))) => (version2, *data2, FlashCopy::Copy1),
            (Some((version1, data1)), Some((version2, data2))) => {
                if version1 < version2 || (version2 < 50 && version1 > 200) {
                    (version2, *data2, FlashCopy::Copy1)
                } else {
                    (version1, *data1, FlashCopy::Copy2)
                }
            }
            (None, None) => (0, unsafe { zeroed() }, FlashCopy::Copy1),
        };

        self.version = version;
        self.data = T::validate(data);
        self.data.notify_changed();
        self.next_save = next_save;
    }

    /// Save data to flash.
    fn save(&mut self, flash: &mut FlashWriter, crc: &mut Crc) {
        self.version = self.version.wrapping_add(1);
        self.next_save = match self.next_save {
            FlashCopy::Copy1 => {
                Self::save_to(flash, crc, self.flash_addr1, self.version, self.reserved, &self.data);
                FlashCopy::Copy2
            }
            FlashCopy::Copy2 => {
                Self::save_to(flash, crc, self.flash_addr2, self.version, self.reserved, &self.data);
                FlashCopy::Copy1
            }
        };
    }

    /// Erase data in flash and init with defaults.
    pub fn erase(&mut self, flash: &mut FlashWriter) {
        flash.erase_unwrap(self.flash_addr1, self.reserved);
        flash.erase_unwrap(self.flash_addr2, self.reserved);

        self.version = 0;
        self.data = T::validate(unsafe { zeroed() });
        self.data.notify_changed();
        self.next_save = FlashCopy::Copy1;
    }

    /// Modify data and save to flash if changed.
    pub fn modify(&mut self, flash: &mut FlashWriter, crc: &mut Crc, modify: impl FnOnce(&mut T)) {
        let old_data = self.data;
        modify(&mut self.data);

        if old_data == self.data {
            return;
        }

        self.data.notify_changed();
        self.save(flash, crc);
    }
}

impl<T> Deref for FlashBackened<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}
