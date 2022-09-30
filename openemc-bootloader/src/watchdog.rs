//! Watchdog driver.

use stm32f1::stm32f103::Peripherals;

static mut ENABLED: bool = false;

/// Configures watchdog for timeout of 6.5 seconds.
pub fn start() {
    let dp = unsafe { Peripherals::steal() };
    dp.IWDG.kr.write(|w| w.key().enable());
    dp.IWDG.pr.write(|w| w.pr().divide_by64());
    dp.IWDG.rlr.write(|w| w.rl().bits(0xfff));
    dp.IWDG.kr.write(|w| w.key().start());
    dp.IWDG.kr.write(|w| w.key().reset());

    unsafe { ENABLED = true };
}

/// Pets the watchdog so that it doesn't bite.
pub fn pet() {
    if !unsafe { ENABLED } {
        return;
    }

    let dp = unsafe { Peripherals::steal() };
    dp.IWDG.kr.write(|w| w.key().reset());
}
