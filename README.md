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
  - full logging to host over I2C in production system
  - power control: system on/off, restart, optional power on when external power supplied
  - watchdog
  - real-time clock (RTC) with alarm and system wake-up
  - GPIO with interrupts
  - pin control
  - analog digital converter (ADC)
  - fully customizable board communication (read, write, ioctl) with Linux userspace via device file
  - battery charger (BQ25713)
  - external power supply with USB PD (STUSB4500) and USB charger detector (MAX14636)
  - charging mode with system power off
  - Linux pstore (persistent storage) integration for kernel panic logs
  - configuration storage in flash memory
  - full devicetree integration

Due to its open-source nature, modular design and usage of the Rust programming
language, OpenEMC is easy to extend and adapt to a wide variety of boards
and applications.

## Evaluation and development

The easiest method to evaluate and hack on OpenEMC is to get an STM32F103
Nucleo-64 board ([NUCLEO-F103RB]) and connect its I2C bus (SCL, SDA) and D12 pin (IRQ)
to a Raspberry Pi or similar board.

Bootloader flashing can be performed using the STLink embedded on the board and requires no additional hardware.

Linux drivers have been tested with Linux v5.19.

[NUCLEO-F103RB]: https://www.st.com/en/evaluation-tools/nucleo-f103rb.html

## Building

### Requirements

For building the firmware your machine must have the following things installed:

  - a recent enough stable Rust toolchain (install from https://rustup.rs),
  - Rust target thumbv7m-none-eabi (`rustup target add thumbv7m-none-eabi`),
  - Rust LLVM utils (`rustup component add llvm-tools-preview`),
  - on Rust nightly: Rust source (`rustup component add rust-src`)
  - Cargo binutils (`cargo install cargo-binutils`),
  - STLINK tools (from https://github.com/stlink-org/stlink or your Linux distribution),
  - for development: probe-run (`cargo install probe-run`).

For building the Linux kernel drivers the kernel headers and `gcc` are required.
The kernel must have support for devicetree enabled.

### Building the firmware image

To build both the bootloader and main firmware, run:

    cargo xtask <BOARD_NAME>

They will be placed into the `image` directory.
Here `<BOARD_NAME>` specifies the name of the board; see `openemc-bootloader/src/boards` for a list
of supported boards. If no board is specified a generic board image will be generated.

This crates four files:

  - `openemc_bootloader_*.bin` is the bootloader image and should be flashed onto the device,
  - `openemc_*.emc` is the main firmware and should be placed into the `/lib/firmware`
    directory on the target device, where it will be picked up by the driver and
    sent to the device,
  - `*.elf` are ELF images of the firmware and used for formatting of log messages;
    they should be placed in `/lib/firmware` on the target device if logging is used.

### Flashing the bootloader image

Connect the STM32 to your computer using an (integrated) STLink USB adapter.
Use `st-flash` from STLINK tools to flash the bootloader image:

    st-flash --connect-under-reset write image/openemc_bootloader_*.bin 0x8000000

### Building the kernel driver

Run `scripts/driver-build.sh` to build the drivers as kernel modules.
You can install them using `scripts/driver-install.sh`.

### Logging

Reading logs over I2C and forwarding to syslog is done by `openemc-log`,
which should be built for and placed on the target device.
Example files for systemd integration are located in the `systemd` directory.

### Devicetree

The driver will only be loaded if it is referenced in the system's devicetree.
Examples are provided in the `devicetree` directory.
Use `scripts/devicetree-build.sh` to build them and `scripts/devicetree-load.sh`
to dynamically load a devicetree overlay into a running system.

## Firmware size considerations

When using a nightly Rust toolchain the `core` crate is built along with the firmware and optimized
for small size.
This saves about 2 kBytes of flash memory space.

## Development and testing

For development and testing, both the bootloader and main firmware can be flashed
and executed directly by invoking

    OPENEMC_BOARD=<BOARD_NAME> DEFMT_LOG=info cargo run --release -- --connect-under-reset

from the respective directory. Set the environment variable `DEFMT_LOG` your desired log level.

## Board IO and ioctl

OpenEMC allows direct communication between Linux userspace and board-specific code.
For this purpose the device `/dev/openemc` is provided, which supports reading, writing, polling
and ioctls.
Corresponding to these operations the board-specific functions `Board::io_read`, `Board::io_write` and
`Board::ioctl` are called.
An example is provided in the board file `stm_nucleo_f103rb.rs` together with the host-side code
in `board-io-test`.

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
