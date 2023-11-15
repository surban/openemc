//! Flash driver.

use core::{mem::size_of, ptr};
use stm32f1::stm32f103::Peripherals;

use openemc_shared::flash;

/// Flash writer.
pub struct FlashWriter {
    dp: Peripherals,
    cp: cortex_m::Peripherals,
}

static mut ACTIVE: bool = false;

impl FlashWriter {
    /// Creates a flash writer.
    pub fn new() -> Result<Self, UnlockError> {
        defmt::assert!(!unsafe { ACTIVE }, "Flash already active");
        unsafe { ACTIVE = true };

        let dp = unsafe { Peripherals::steal() };
        let cp = unsafe { cortex_m::Peripherals::steal() };

        // Unlock flash for writing.
        if dp.FLASH.cr.read().lock().bit_is_set() {
            dp.FLASH.keyr.write(|w| unsafe { w.key().bits(0x45670123) });
            dp.FLASH.keyr.write(|w| unsafe { w.key().bits(0xCDEF89AB) });
            cortex_m::asm::nop();

            if dp.FLASH.cr.read().lock().bit_is_set() {
                return Err(UnlockError);
            }
        }

        Ok(Self { dp, cp })
    }

    fn wait_idle(&self) {
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        while self.dp.FLASH.sr.read().bsy().bit_is_set() {}
        cortex_m::asm::nop();
    }

    fn clear_state(&self) {
        self.dp.FLASH.sr.modify(|_, w| w.eop().set_bit().wrprterr().set_bit().pgerr().set_bit());
        cortex_m::asm::nop();
    }

    /// Erases the flash page with the specified address.
    ///
    /// Verifies that the page has been erased.
    pub fn erase_page(&mut self, addr: usize) -> Result<(), EraseError> {
        let page = flash::page_base(addr);

        self.wait_idle();
        self.clear_state();

        self.dp.FLASH.cr.modify(|_, w| w.per().set_bit());
        self.dp.FLASH.ar.write(|w| w.far().variant(page as u32));
        self.dp.FLASH.cr.modify(|_, w| w.strt().set_bit());
        self.wait_idle();
        self.dp.FLASH.cr.modify(|_, w| w.per().clear_bit());

        self.cp.SCB.clean_invalidate_dcache(&mut self.cp.CPUID);

        for addr in (page..page + flash::page_size()).step_by(size_of::<u32>()) {
            let data = unsafe { ptr::read_volatile(addr as *const u32) };
            if data != 0xffffffff {
                return Err(EraseError);
            }
        }

        Ok(())
    }

    /// Writes the u16 at the specified flash address.
    ///
    /// The location must have been erased.
    /// `addr` must be aligned to 2 bytes.
    pub fn write(&mut self, addr: usize, data: u16) -> Result<(), WriteError> {
        defmt::assert!(addr % 2 == 0, "addr unaligned");

        self.wait_idle();
        self.clear_state();

        self.dp.FLASH.cr.modify(|_, w| w.pg().set_bit());
        self.wait_idle();

        unsafe { ptr::write_volatile(addr as *mut u16, data) };
        self.cp.SCB.clean_invalidate_dcache_by_address(addr, size_of::<u16>());
        self.wait_idle();

        while self.dp.FLASH.sr.read().eop().bit_is_clear() {}

        self.dp.FLASH.cr.modify(|_, w| w.pg().clear_bit());
        cortex_m::asm::nop();

        let readback = unsafe { ptr::read_volatile(addr as *const u16) };
        if readback == data {
            Ok(())
        } else {
            Err(WriteError)
        }
    }
}

/// Unlocking flash failed.
///
/// Device must be reset for unlocking to work.
pub struct UnlockError;

/// Flash erase failed.
pub struct EraseError;

/// Writing to flash failed.
pub struct WriteError;

impl Drop for FlashWriter {
    fn drop(&mut self) {
        self.dp.FLASH.cr.write(|w| w.lock().set_bit());
        cortex_m::asm::nop();

        unsafe { ACTIVE = false };
    }
}
