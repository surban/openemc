# OpenEMC

OpenEMC is an open-source firmware implementing an embedded management
controller (EMC) on an STM32F1 microcontroller.
It consists of a bootloader and firmware (both written in Rust) and
Linux kernel driver modules (written in C).

[![Build](https://github.com/surban/openemc/actions/workflows/build.yml/badge.svg)](https://github.com/surban/openemc/actions/workflows/build.yml)

## Features

The following features are implemented:

  - communication with the host over I2C and one interrupt line
  - field-upgradable firmware
  - power control: system on/off and restart
  - watchdog
  - real-time clock (RTC) with alarm and system wake-up
  - GPIO with interrupts
  - pin control
  - analog digital converter (ADC)
  - full devicetree integration

Due to its open-source nature, modular design and usage of the Rust programming
language, OpenEMC is easy to extend and adapt to a wide variety of boards
and applications.

## Evaluation and development

The easiest method to evaluate and hack on OpenEMC is to get an STM32F103
Nucleo-64 board ([NUCLEO-F103RB]) and connect its I2C bus (SCL, SDA) and D12 pin (IRQ)
to a Raspberry Pi or similar board.

Bootloader flashing can be performed using [probe-run] and requires no additional hardware.

Linux drivers have been tested with Linux v5.19.

[NUCLEO-F103RB]: https://www.st.com/en/evaluation-tools/nucleo-f103rb.html
[probe-run]: https://github.com/knurling-rs/probe-run

## Building

### Requirements

For building the firmware your machine must have the following things installed:

  - a recent enough stable Rust toolchain (install from https://rustup.rs),
  - Rust target thumbv7m-none-eabi (`rustup target add thumbv7m-none-eabi`),
  - Rust LLVM utils (`rustup component add llvm-tools-preview`),
  - Cargo binutils (`cargo install cargo-binutils`),
  - probe-run (`cargo install probe-run`).

For building the Linux kernel drivers the kernel headers and `gcc` are required.
The kernel must have support for devicetree enabled.

### Building the firmware image

Run `cargo xtask <BOARD_NAME>` to build the firmware; it will be placed into the `image` directory.
Here `<BOARD_NAME>` specifies the name of the board; see `openemc-bootloader/src/boards` for a list
of supported boards. If no board is specified a generic board image will be generated.

This crates two files:

  - `openemc_bootloader_*.bin` is the bootloader image and should be flashed onto the device,
  - `openemc_*.emc` is the main firmware and should be placed into the `/lib/firmware` 
    directory on the target device, where it will be picked up by the driver and 
    sent to the device.

For development and testing, both the bootloader and main firmware can be flashed 
and executed directly by invoking 

    OPENEMC_BOARD=<BOARD_NAME> cargo run --release -- --connect-under-reset

from the respective directory. Set the environment variable `DEFMT_LOG=info` to enable logging.


### Building the kernel driver

Run `scripts/driver-build.sh` to build the drivers as kernel modules.
You can install them using `scripts/driver-install.sh`.

### Devicetree

The driver will only be loaded if it is referenced in the system's devicetree.
Examples are provided in the `devicetree` directory.
Use `scripts/devicetree-build.sh` to build them and `scripts/devicetree-load.sh`
to dynamically load a devicetree overlay into a running system.

## License

The OpenEMC firmware is released under the [GNU GPL version 3],
meaning that you must release the full source code of your modifications
if you adapt OpenEMC to your application and/or board.

The OpenEMC drivers are released under the [GNU GPL version 2] or later, 
following the standard license of the Linux kernel.
This also requires you to publish all changes you make to the OpenEMC drivers.

Commercial licensing options without these restrictions are available.
[Please contact us for licensing details.](mailto:surban@surban.net).

[GNU GPL version 2]: https://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
[GNU GPL version 3]: https://www.gnu.org/licenses/gpl-3.0.txt
