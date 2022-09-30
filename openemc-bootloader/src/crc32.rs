//! CRC32 hardware calculation unit.

use stm32f1::stm32f103::Peripherals;

/// CRC32 calculator.
///
/// Uses CRC-32 (Ethernet) polynomial 0x4C11DB7:
/// `X^32 + X^26 + X^23 + X^22 + X^16 + X^12 + X^11 + X^10 +X^8 + X^7 + X^5 + X^4 + X^2 + X + 1`.
pub struct Crc32 {
    dp: Peripherals,
}

static mut ACTIVE: bool = false;

impl Crc32 {
    /// Accesses the CRC32 calculator.
    pub fn new() -> Self {
        defmt::assert!(!unsafe { ACTIVE }, "CRC32 already active");
        unsafe { ACTIVE = true };

        let dp = unsafe { Peripherals::steal() };
        dp.RCC.ahbenr.modify(|_, w| w.crcen().enabled());
        cortex_m::asm::nop();
        cortex_m::asm::nop();

        let mut this = Self { dp };
        this.reset();

        this
    }

    /// Resets the CRC32 calculator.
    pub fn reset(&mut self) {
        self.dp.CRC.cr.write(|w| w.reset().reset());
        cortex_m::asm::nop();
    }

    /// Adds the data to the CRC32 calculation.
    pub fn write(&mut self, data: u32) {
        self.dp.CRC.dr.write(|w| w.dr().bits(data.reverse_bits()));
    }

    /// The CRC32 value of all previously written data after the
    /// last reset.
    pub fn crc32(&self) -> u32 {
        cortex_m::asm::nop();
        !self.dp.CRC.dr.read().dr().bits().reverse_bits()
    }
}

impl Drop for Crc32 {
    fn drop(&mut self) {
        self.dp.RCC.ahbenr.modify(|_, w| w.crcen().disabled());

        unsafe { ACTIVE = false };
    }
}
