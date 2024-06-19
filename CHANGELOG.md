# Changelog
All notable changes to OpenEMC will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

# 0.9.2 - 2024-06-19
### Changed
- bootloader: avoid bootloader entry after power-on reset, if possible
- bootloader version 0.3.2
### Fixed
- STUSB4500: NVM programming

# 0.9.1 - 2024-06-18
### Added
- STUSB4500: reset when BQ25713 I2C communication fails
- STUSB4500: log monitoring and hardware fault status
- power: switch to charging mode when battery is below minimum power-on voltage
  or reset was due to power loss with charger attached
### Changed
- BQ25713: improve status logging
- BQ25713: enable ICO for USB-CDP supply
### Fixed
- charging mode: power off when charger is disconnected during grace period

# 0.9.0 - 2024-02-22
### Added
- board IO support
- board ioctl support
- task spawning by board
### Changed
- refactored board init

## 0.8.1 - 2023-11-17
### Added
- allow querying whether power on was by charger attachment
- allow explicit reboot into charging mode
- allow usage of charge LED during init for debugging, enabled by feature `debug-blink`
### Changed
- handle board power on by charger connection in global init

## 0.8.0 - 2023-11-16
### Added
- allow querying of bootloader CRC32 and flash size via sysfs
- power: switch to charging mode when battery is below critical voltage
- bootloader: dynamic flash size detection
- logging: gzip ELF images to save space on target
- logging: find bootloader ELF image based on CRC32 of bootloader

## 0.7.2 - 2023-09-06
### Fixed
- configuration location in flash

## 0.7.1 - 2023-09-05
### Added
- BQ25713: enable input current optimizer (ICO) algorithm on USB-DCP ports

## 0.7.0 - 2023-09-05
### Added
- reliable configuration storage in flash memory
- power: configuration option to prohibit power off
- power: confiugration option to power on when charger is attached
- verify firmware fits into flash during packing
- support board power off request during initialization
### Changed
- optimize firmware size
- build core crate when using Rust nightly toolchain to save flash space
- update dependencies
- bootloader version 0.2.7

## 0.6.6 - 2023-08-19
### Added
- standby entry support from bootloader
### Changed
- STUSB4500: avoid soft reset when PDO is unchanged
- Supply: reduce max current during negotiation

## 0.6.5 - 2023-06-26
### Fixed
- set input current limit to zero if BQ25713 CHRG_OK pin is low
  and reprogram the configured value immediately after it goes high

## 0.6.4 - 2023-06-05
### Fixed
- positions of log sections in RAM was unstable
- possible panic during RTC clock source change

## 0.6.3 - 2023-05-19
### Added
- support for board-specific periodic handler function
- board software versioning
### Fixed
- reconfigure charger when setting input current failed

## 0.6.2 - 2023-04-08
### Added
- read unique device identifier

## 0.6.1 - 2023-03-07
### Fixed
- system not powering on after watchdog reset

## 0.6.0 - 2023-03-07
### Added
- CRC32 checksumming and error correction for I2C requests to main firmware

## 0.5.1 - 2023-03-06
### Fixed
- more robust ring buffer implementation

## 0.5.0 - 2023-03-03
### Added
- logging to host via I2C with temporary storage in persistent memory buffer
- charging mode for charging with system power off
- charging indication via LED
- echo service for I2C testing
- support for MAX14636 USB charger detector
### Changed
- bootloader version 0.2
- tolerate unreliable BQ25713 and STUSB4500 communication
### Fixed
- power loss during firmware start due to GPIO reset
- I2C timeout for unprocessed I2C read requests

## 0.4.0 - 2023-01-04
### Added
- support for BQ25713 battery charger
- Linux drivers for battery and external power supply
- support for firmware-generated interrupts
### Changed
- reset STUSB4500 if PD contract fails

## 0.3.0 - 2022-11-30
### Added
- support for I2C bus 2 master
- support for STUSB4500 USB PD controller

## 0.2.2 - 2022-11-17
### Fixed
- inconsistent logic levels on complementary PWM outputs if a channel on
  timer 1 is disabled

## 0.2.1 - 2022-11-16
### Changed
- use internal oscillator for system clock

## 0.2.0 - 2022-11-16
### Added
- PWM output support

## 0.1.0 - 2022-09-30
- initial release
