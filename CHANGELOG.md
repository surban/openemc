# Changelog
All notable changes to OpenEMC will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
