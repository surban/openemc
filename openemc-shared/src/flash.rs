//! Information about flash memory.

use core::ptr;

/// Address of start of flash memory.
pub const START: usize = 0x08000000;

/// Total flash size.
pub fn size() -> usize {
    const BASE: usize = 0x1ffff7e0;
    let mem_kb = unsafe { ptr::read_volatile(BASE as *const u16) };
    usize::from(mem_kb) * 1024
}

/// Address of end of flash memory (exclusive) for running device.
///
/// Returned address is first byte beyond end of flash memory.
pub fn end() -> usize {
    START + size()
}

/// Flash page size for running device.
pub fn page_size() -> usize {
    page_size_for(size())
}

/// Calculates the flash page size given the flash size.
pub const fn page_size_for(flash_size: usize) -> usize {
    // STM32F1 devices with more than 128 kB of flash have a flash page size of 2 kB;
    // the smaller ones 1 kB.
    if flash_size <= 128 * 1024 {
        1024
    } else {
        2048
    }
}

/// Base address of a flash page.
pub fn page_base(addr: usize) -> usize {
    (addr / page_size()) * page_size()
}

/// Returns true, if the address point to the beginning of a flash page.
pub fn is_page_aligned(addr: usize) -> bool {
    addr % page_size() == 0
}
