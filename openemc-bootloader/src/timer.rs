//! Timers.

use stm32f1::stm32f103::Peripherals;

/// Basic timer 4.
pub struct Timer4 {
    dp: Peripherals,
}

static mut ACTIVE: bool = false;

impl Timer4 {
    /// Accesses general purpose timer 4.
    pub fn new() -> Self {
        defmt::assert!(!unsafe { ACTIVE }, "Timer4 already active");
        unsafe { ACTIVE = true };

        let dp = unsafe { Peripherals::steal() };
        dp.RCC.apb1enr.modify(|_, w| w.tim4en().enabled());
        dp.RCC.apb1rstr.modify(|_, w| w.tim4rst().reset());
        dp.RCC.apb1rstr.modify(|_, w| w.tim4rst().clear_bit());

        Self { dp }
    }

    /// Sets the one-pulse mode.
    ///
    /// If enabled, counter stops counting at the next update event.
    pub fn set_one_pulse_mode(&mut self, one_pulse: bool) {
        self.dp.TIM4.cr1.modify(|_, w| w.opm().bit(one_pulse));
    }

    /// Returns whether the counter is enabled.
    pub fn is_enabled(&self) -> bool {
        self.dp.TIM4.cr1.read().cen().is_enabled()
    }

    /// Enables or disables the counter.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.dp.TIM4.cr1.modify(|_, w| w.cen().bit(enabled));
    }

    /// Sets the prescaler value.
    pub fn set_prescaler(&mut self, prescaler: u16) {
        defmt::debug!("timer: prescaler = 0x{:04x}", prescaler);
        self.dp.TIM4.psc.write(|w| w.psc().bits(prescaler));
    }

    /// Sets the auto-reload value.
    pub fn set_auto_reload(&mut self, auto_reload: u16) {
        defmt::debug!("timer: auto_reload = 0x{:04x}", auto_reload);
        self.dp.TIM4.arr.write(|w| w.arr().bits(auto_reload));
    }

    /// The current counter value.
    #[allow(dead_code)]
    pub fn counter(&self) -> u16 {
        self.dp.TIM4.cnt.read().cnt().bits()
    }

    /// Sets the counter value.
    #[allow(dead_code)]
    pub fn set_counter(&mut self, counter: u16) {
        self.dp.TIM4.cnt.write(|w| w.cnt().bits(counter));
    }

    /// Re-initializes the timer counter and generates an update of the registers.
    pub fn generate_update(&mut self) {
        self.dp.TIM4.egr.write(|w| w.ug().update());
    }
}

impl Drop for Timer4 {
    fn drop(&mut self) {
        self.dp.RCC.apb1enr.modify(|_, w| w.tim4en().disabled());

        unsafe { ACTIVE = false };
    }
}
