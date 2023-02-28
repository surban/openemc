//
// OpenEMC firmware for embedded controllers
// Copyright (C) 2022-2023 Sebastian Urban <surban@surban.net>
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

//! Power supply drivers.

use defmt::Format;

pub mod stusb4500;

/// Power supply report.
#[derive(Default, Clone, Format, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PowerSupply {
    /// Power supply is unknown.
    #[default]
    Unknown,
    /// No cable attached.
    Disconnected,
    /// USB standard downstream port.
    ///
    /// Supply voltage is 5 V and current must be limited
    /// according to USB specification.
    /// USB communication is supported.
    UsbSdp,
    /// USB dedicated charging port.
    ///
    /// Supply voltage is 5 V and maximum current is 1.5 A.
    /// USB communication is unsupported.
    UsbDcp,
    /// USB charging downstream port.
    ///
    /// Supply voltage is 5 V and maximum current is 1.5 A.
    /// USB communication is supported.
    UsbCdp,
    /// Resistor encoding on USB CC lines for 5 V and 1.5 A.
    CcPins5V1500mA,
    /// Resistor encoding on USB CC lines for 5 V and 3 A.
    CcPins5V3000mA,
    /// Negotiated USB PD contract with supply.
    PdoContract {
        /// Power supply voltage in mV.
        voltage_mv: u32,
        /// Maximum current in mA provided by power supply.
        max_current_ma: u32,
        /// Whether USB communication is supported.
        communication: bool,
    },
    // USB CC lines are connected and negotiation is in progress.
    Negotiating,
}

impl PowerSupply {
    /// Merge two power supply reports, returning the best available power.
    pub fn merge(&self, other: &Self) -> Self {
        self.max(other).clone()
    }

    /// Returns whether a power supply is surely detected.
    pub fn is_connected(&self) -> bool {
        match self {
            Self::Unknown | Self::Disconnected => false,
            Self::UsbSdp
            | Self::UsbDcp
            | Self::UsbCdp
            | Self::PdoContract { .. }
            | Self::CcPins5V1500mA
            | Self::CcPins5V3000mA
            | Self::Negotiating => true,
        }
    }

    /// Voltage in mV.
    pub fn voltage_mv(&self) -> u32 {
        match self {
            Self::Unknown => 0,
            Self::Disconnected => 0,
            Self::UsbSdp | Self::UsbDcp | Self::UsbCdp => 5000,
            Self::PdoContract { voltage_mv, .. } => *voltage_mv,
            Self::CcPins5V1500mA | Self::CcPins5V3000mA => 5000,
            Self::Negotiating => 5000,
        }
    }

    /// Returns the maximum current available in mA.
    pub fn max_current_ma(&self) -> u32 {
        match self {
            Self::Unknown => 0,
            Self::Disconnected => 0,
            Self::UsbSdp => 100,
            Self::UsbDcp | Self::UsbCdp => 1500,
            Self::PdoContract { max_current_ma, .. } => *max_current_ma,
            Self::CcPins5V1500mA => 1500,
            Self::CcPins5V3000mA => 3000,
            Self::Negotiating => 500,
        }
    }

    /// USB communication support status.
    pub fn communication(&self) -> PowerSupplyUsbCommunication {
        match self {
            Self::Unknown => PowerSupplyUsbCommunication::Unknown,
            Self::Disconnected => PowerSupplyUsbCommunication::Unsupported,
            Self::UsbSdp => PowerSupplyUsbCommunication::Supported,
            Self::UsbDcp => PowerSupplyUsbCommunication::Unsupported,
            Self::UsbCdp => PowerSupplyUsbCommunication::Supported,
            Self::PdoContract { communication, .. } => {
                if *communication {
                    PowerSupplyUsbCommunication::Supported
                } else {
                    PowerSupplyUsbCommunication::Unsupported
                }
            }
            Self::CcPins5V1500mA => PowerSupplyUsbCommunication::Unknown,
            Self::CcPins5V3000mA => PowerSupplyUsbCommunication::Unknown,
            Self::Negotiating => PowerSupplyUsbCommunication::Unknown,
        }
    }
}

impl From<&PowerSupply> for u8 {
    fn from(supply: &PowerSupply) -> Self {
        match supply {
            PowerSupply::Unknown => 0,
            PowerSupply::Disconnected => 1,
            PowerSupply::UsbSdp => 2,
            PowerSupply::UsbDcp => 3,
            PowerSupply::UsbCdp => 4,
            PowerSupply::CcPins5V1500mA => 5,
            PowerSupply::CcPins5V3000mA => 6,
            PowerSupply::PdoContract { .. } => 7,
            PowerSupply::Negotiating => 8,
        }
    }
}

/// USB communication supported.
#[derive(Default, Clone, Copy, Format, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PowerSupplyUsbCommunication {
    /// USB data communication is unsupported.
    Unsupported,
    /// USB data communication is supported.
    Supported,
    /// Unknown whether USB communication is supported.
    #[default]
    Unknown,
}

impl From<PowerSupplyUsbCommunication> for u8 {
    fn from(uc: PowerSupplyUsbCommunication) -> Self {
        match uc {
            PowerSupplyUsbCommunication::Unsupported => 0,
            PowerSupplyUsbCommunication::Supported => 1,
            PowerSupplyUsbCommunication::Unknown => 2,
        }
    }
}

/// Fixed sink USB PDO.
#[derive(Clone, Format, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FixedSinkPdo {
    /// Required current in mA.
    pub operating_current_ma: u32,
    /// Required voltage in mV.
    pub voltage_mv: u32,
    /// Fast role requried current in mA.
    pub fast_role_req_current_ma: u32,
    /// Can be USB host and device.
    pub dual_role_data: bool,
    /// USB data communication supported.
    pub communication: bool,
    /// Unconstrained power.
    pub unconstrained_power: bool,
    /// Has more capabilities if higher PDO is chosen.
    pub higher_capability: bool,
    /// Can be power supply and sink.
    pub dual_role_power: bool,
}
