//! Utility functions.

use cortex_m::{asm::wfi, peripheral::SCB};
use stm32f1::stm32f103::Peripherals;

use crate::board::Board;

/// Approximately delay execution by specified amount of milliseconds.
pub fn delay_ms(ms: u32) {
    let cycles = ms * (crate::boards::Chosen::CPU_CLOCK / 1000);
    cortex_m::asm::delay(cycles)
}

/// Enters standby mode.
#[allow(dead_code)]
pub fn enter_standby() -> ! {
    defmt::warn!("entering standby mode");

    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { Peripherals::steal() };

    // Enable GPIO A.
    dp.RCC.apb2enr.modify(|_, w| w.afioen().enabled().iopaen().enabled());

    // Check if WKUP pin is low.
    dp.GPIOA.crl.modify(|_, w| w.cnf0().open_drain().mode0().input());
    delay_ms(10);
    let wkup = dp.GPIOA.idr.read().idr0();
    if wkup.is_high() {
        // Device should stay awake, convert shutdown into reset.
        defmt::info!("WKUP pin is high, turning shutdown into reset");
        delay_ms(10);
        SCB::sys_reset();
    }

    // Disable debug workarounds.
    //dp.DBGMCU.cr.modify(|_, w| w.dbg_sleep().clear_bit().dbg_stop().clear_bit().dbg_standby().clear_bit());

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

    // Enable standby mode and wake up using WKUP pin.
    dp.PWR.csr.modify(|_, w| w.ewup().set_bit());
    dp.PWR.cr.modify(|_, w| w.lpds().set_bit().cwuf().set_bit().pdds().standby_mode());
    cp.SCB.set_sleepdeep();

    // Enter standby mode.
    loop {
        wfi()
    }
}
