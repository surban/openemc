//! Configuration.

use core::{mem::MaybeUninit, ptr::addr_of_mut};
use defmt::Format;

use crate::flash_data::FlashData;

/// Behavior when charger is attached while device is powered off.
#[derive(Default, Format, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ChargerAttached {
    /// Go into charge mode.
    #[default]
    ChargeMode = 0,
    /// Power on device.
    PowerOn = 1,
}

impl TryFrom<u8> for ChargerAttached {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::ChargeMode),
            1 => Ok(Self::PowerOn),
            _ => Err(()),
        }
    }
}

/// Configuration stored in flash.
#[derive(Format, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct Cfg {
    /// Behavior when charger is attached while device is powered off.
    pub charger_attached: ChargerAttached,
    /// Whether power off is prohibited.
    ///
    /// If true, power off requests are converted into resets.
    pub prohibit_power_off: bool,
}

impl FlashData for Cfg {
    fn validate(mut this: MaybeUninit<Self>) -> Self {
        unsafe {
            let ptr = this.as_mut_ptr();

            let charger_attached_ptr = addr_of_mut!((*ptr).charger_attached) as *mut u8;
            if charger_attached_ptr.read() > 1 {
                charger_attached_ptr.write(0);
            }

            let prohibit_power_off_ptr = addr_of_mut!((*ptr).prohibit_power_off) as *mut u8;
            if prohibit_power_off_ptr.read() > 1 {
                prohibit_power_off_ptr.write(0);
            }

            this.assume_init()
        }
    }

    fn notify_changed(&self) {
        defmt::info!("Configuration changed:    {:?}", self);
    }
}
