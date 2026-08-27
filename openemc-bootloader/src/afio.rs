//! Alternate function I/O (AFIO).

use core::sync::atomic::{AtomicU8, Ordering};
use stm32f1::stm32f103::{afio::mapr, Peripherals};

/// Serial wire and JTAG configuration to keep when modifying `AFIO_MAPR`.
static SWJ_CFG: AtomicU8 = AtomicU8::new(0b000);

/// Sets the serial wire and JTAG configuration.
///
/// 0b000 = JTAG and SWD enabled
/// 0b001 = JTAG enabled without NJTRST, SWD enabled
/// 0b010 = JTAG disabled, SWD enabled
/// 0b100 = JTAG and SWD disabled
pub fn set_swj_cfg(swj_cfg: u8) {
    SWJ_CFG.store(swj_cfg, Ordering::Relaxed);
    modify_mapr(|_, w| w);
}

/// Modifies `AFIO_MAPR`, keeping the serial wire and JTAG configuration.
pub fn modify_mapr<F>(f: F)
where
    F: for<'w> FnOnce(&mapr::R, &'w mut mapr::W) -> &'w mut mapr::W,
{
    let swj_cfg = SWJ_CFG.load(Ordering::Relaxed);
    let dp = unsafe { Peripherals::steal() };
    dp.AFIO.mapr().modify(|r, w| unsafe { f(r, w).swj_cfg().bits(swj_cfg) });
}
