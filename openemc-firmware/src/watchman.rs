//! Watchdog management.

use defmt::intern;
use stm32f1xx_hal::{prelude::*, watchdog::IndependentWatchdog};
use systick_monotonic::ExtU64;

use crate::{app::monotonics, Duration, Instant};

/// Watchdog manager.
///
/// Pets the independent hardware watchdog when conditions are satisfied.
pub struct Watchman {
    dog: IndependentWatchdog,
    unlocked: bool,
    interval: Duration,
    active: bool,
    last_pet: Instant,
    pet_code: u32,
}

impl Watchman {
    /// Watchdog unlock code.
    pub const UNLOCK_CODE: u64 = 0x1984_0902_0406_1986;

    /// Creates a new watchdog manager.
    pub fn new(mut dog: IndependentWatchdog, interval: Duration, active: bool) -> Self {
        dog.start(3u32.secs());
        dog.feed();

        Self { dog, interval, active, last_pet: monotonics::now(), unlocked: false, pet_code: 0x11223344 }
    }

    /// Necessary watchdog pet interval.
    pub fn pet_interval() -> Duration {
        1u64.secs()
    }

    /// Unlocks access to `set_*` functions.
    pub fn unlock(&mut self, unlock_code: u64) {
        self.unlocked = unlock_code == Self::UNLOCK_CODE;
    }

    /// Returns whether access to `set_*` functions is unlocked.
    pub fn unlocked(&self) -> bool {
        self.unlocked
    }

    /// Returns whether the watchdog must be petted.
    pub fn active(&self) -> bool {
        self.active
    }

    /// Sets whether the watchdog must be petted.
    pub fn set_active(&mut self, active: bool) {
        if self.unlocked {
            defmt::info!("{} watchdog", if active { intern!("starting") } else { intern!("stopping") });
            self.last_pet = monotonics::now();
            self.active = active;
        } else {
            defmt::warn!("cannot set active state of locked watchdog");
        }
    }

    /// Interval between necessary watchdog pettings.
    pub fn interval(&self) -> Duration {
        self.interval
    }

    /// Sets the interval between necessary watchdog pettings.
    pub fn set_interval(&mut self, interval: Duration) {
        if self.unlocked {
            defmt::info!("setting watchdog interval to {} ms", interval.to_millis());
            self.interval = interval;
        } else {
            defmt::warn!("cannot set interval of locked watchdog");
        }
    }

    /// Sets the petting code.
    pub fn set_pet_code(&mut self, pet_code: u32) {
        defmt::info!("setting watchdog petting code to 0x{:08x}", pet_code);
        self.pet_code = pet_code;
    }

    /// Pets the watchdog.
    pub fn pet(&mut self, pet_code: u32) {
        if pet_code == self.pet_code || option_env!("IGNORE_PET_CODE").is_some() {
            defmt::debug!("watchdog accepts being petted");
            self.last_pet = monotonics::now();
        } else {
            defmt::warn!("watchdog rejects wrong pet code");
        }
    }

    /// Pets the hardware watchdog, if possible.
    ///
    /// Returns whether petting was possible.
    pub fn pet_hardware_watchdog(&mut self) -> bool {
        if !self.active {
            self.dog.feed();
            return true;
        }

        // Reset system as quickly as possible if interval is set to zero.
        if self.interval.ticks() == 0 {
            defmt::info!("resetting system using watchdog");
            self.dog.start(10u32.millis());
            return false;
        }

        match monotonics::now().checked_duration_since(self.last_pet) {
            Some(elapsed) if elapsed <= self.interval => {
                defmt::trace!("petting hardware watchdog");
                self.dog.feed();
                true
            }
            _ if option_env!("DISABLE_WATCHDOG").is_some() => {
                defmt::warn!("petting hardware watchdog because watchdog is disabled");
                self.dog.feed();
                true
            }
            _ => {
                defmt::error!("cannot pet hardware watchdog");
                false
            }
        }
    }
}
