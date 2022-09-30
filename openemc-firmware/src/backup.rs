//! Backup domain.

use stm32f1xx_hal::backup_domain::BackupDomain;

/// Backup domain register.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum BackupReg {
    /// Signature.
    Signature = 0,
    /// Boot reason.
    BootReason = 1,
    /// RTC alarm high part.
    RtcAlarmHigh = 2,
    /// RTC alarm low part.
    RtcAlarmLow = 3,
}

/// Initialzied?
static mut INIT: bool = false;

impl BackupReg {
    /// Value for signature register.
    pub const SIGNATURE: u16 = 0xb001;

    /// Initializes the backup registers.
    ///
    /// Clears them if the signature is invalid.
    pub fn init(bkp: &mut BackupDomain) {
        if bkp.read_data_register_low(Self::Signature as _) != Self::SIGNATURE {
            for n in 0..10 {
                bkp.write_data_register_low(n, 0);
            }
            bkp.write_data_register_low(Self::Signature as _, Self::SIGNATURE);
        }

        unsafe { INIT = true };
    }

    /// Gets the value of the backup register.
    pub fn get(&self, bkp: &BackupDomain) -> u16 {
        unsafe { defmt::assert!(INIT) };
        bkp.read_data_register_low(*self as _)
    }

    /// Sets the value of the backup register.
    pub fn set(&self, bkp: &mut BackupDomain, value: u16) {
        unsafe { defmt::assert!(INIT) };
        bkp.write_data_register_low(*self as _, value)
    }
}
