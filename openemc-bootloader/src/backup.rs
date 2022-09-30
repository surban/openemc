//! Backup power domain.

use stm32f1::stm32f103::Peripherals;

static mut ACTIVE: bool = false;

/// Enables access to the backup power domain.
pub fn enable() {
    let dp = unsafe { Peripherals::steal() };
    dp.RCC.apb1enr.modify(|_, w| w.pwren().enabled().bkpen().enabled());
    dp.RCC.bdcr.modify(|_, w| w.bdrst().disabled());
    unsafe { ACTIVE = true };
}

/// Disables access to the backup power domain.
pub fn disable() {
    let dp = unsafe { Peripherals::steal() };
    dp.RCC.apb1enr.modify(|_, w| w.bkpen().disabled());
    unsafe { ACTIVE = false };
}

/// Resets the backup power domain.
pub fn reset() {
    let dp = unsafe { Peripherals::steal() };
    dp.RCC.bdcr.modify(|_, w| w.bdrst().enabled());
    dp.RCC.bdcr.modify(|_, w| w.bdrst().disabled());
}

fn check_active() {
    defmt::assert!(unsafe { ACTIVE }, "backup domain not active");
}

/// Read backup register.
///
/// # Panics
/// Panics if register out of range or backup domain inactive.
pub fn read(reg: u8) -> u16 {
    check_active();
    let dp = unsafe { Peripherals::steal() };
    dp.BKP.dr[reg as usize].read().d().bits()
}

/// Write backup register.
///
/// # Panics
/// Panics if register out of range, backup domain inactive or write was unsuccessful.
pub fn write(reg: u8, data: u16) {
    check_active();
    let dp = unsafe { Peripherals::steal() };

    dp.PWR.cr.modify(|_, w| w.dbp().set_bit());
    dp.BKP.dr[reg as usize].write(|w| w.d().bits(data));
    dp.PWR.cr.modify(|_, w| w.dbp().clear_bit());

    if read(reg) != data {
        defmt::panic!("writing backup register failed");
    }
}

/// Enables tamper protection and clears pending tamper events and interrupts.
#[allow(dead_code)]
pub fn enable_tamper(tamper_level: bool) {
    check_active();
    let dp = unsafe { Peripherals::steal() };

    dp.GPIOC.crh.modify(|_, w| w.cnf13().alt_open_drain());
    dp.BKP.csr.write(|w| w.cti().clear().cte().reset());
    dp.BKP.cr.write(|w| {
        let w = if tamper_level { w.tpal().low() } else { w.tpal().high() };
        w.tpe().alternate()
    });
    dp.BKP.csr.write(|w| w.tpie().enabled().cti().clear().cte().reset());
}

/// Disables tamper protection.
#[allow(dead_code)]
pub fn disable_tamper() {
    check_active();
    let dp = unsafe { Peripherals::steal() };

    dp.BKP.cr.write(|w| w.tpe().general());
}
