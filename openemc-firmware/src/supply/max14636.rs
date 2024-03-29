//
// OpenEMC firmware for embedded controllers
// Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

//! MAX14636 USB charger detector driver.

use stm32f1xx_hal::gpio::{ErasedPin, Floating, Input, Output, PullUp};

use super::PowerSupply;

/// MAX14636 USB charger detector driver.
pub struct Max14636 {
    _good_bat: ErasedPin<Output>,
    sw_open: ErasedPin<Input<PullUp>>,
    chg_al_n: ErasedPin<Input<PullUp>>,
    chg_det: ErasedPin<Input<Floating>>,
}

impl Max14636 {
    /// Creates a new driver instance.
    pub fn new(
        mut good_bat: ErasedPin<Output>, sw_open: ErasedPin<Input<PullUp>>, chg_al_n: ErasedPin<Input<PullUp>>,
        chg_det: ErasedPin<Input<Floating>>,
    ) -> Self {
        good_bat.set_high();
        Self { _good_bat: good_bat, sw_open, chg_al_n, chg_det }
    }

    /// Power supply report.
    pub fn report(&self) -> PowerSupply {
        match (self.sw_open.is_low(), self.chg_al_n.is_low(), self.chg_det.is_low()) {
            (false, true, false) => PowerSupply::UsbDcp,
            (true, true, false) => PowerSupply::UsbCdp,
            (true, true, true) => PowerSupply::UsbSdp,
            (false, true, true) => PowerSupply::Ps2,
            (false, false, true) => PowerSupply::Disconnected,
            other => {
                defmt::warn!("Unknown MAX14646 status: {:?}", other);
                PowerSupply::Unknown
            }
        }
    }
}
