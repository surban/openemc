//! Register definitions.

/// Read id (u8).
pub const ID: u8 = 0x00;

/// Read firmware version (string).
pub const VERSION: u8 = 0x01;

/// Read EMC model (u8).
pub const EMC_MODEL: u8 = 0x02;

/// Read board model (u16).
pub const BOARD_MODEL: u8 = 0x03;

/// Read bootloader version (string).
pub const BOOTLOADER_VERSION: u8 = 0x04;

/// Read 96-bit unique device identifier (u128).
pub const UNIQUE_ID: u8 = 0x05;

/// Copyright.
/// Read: string starting at offset.
/// Write u8: set offset.
pub const COPYRIGHT: u8 = 0x06;

/// MFD cell.
/// Read: cell string.
/// Write u8: set cell index.
pub const MFD_CELL: u8 = 0x07;

/// Reads the boot reason (u16).
pub const BOOT_REASON: u8 = 0x08;

/// Reads the reset status (u8).
pub const RESET_STATUS: u8 = 0x09;

/// Reads the start reason (u8).
pub const START_REASON: u8 = 0x0a;

/// Reads the firmware CRC32 (u32).
pub const PROGRAM_ID: u8 = 0x0b;

/// Reads the total size of the flash memory (u32).
pub const FLASH_TOTAL_SIZE: u8 = 0x0c;

/// Interrupt mask (u32).
/// Bits 0 - 15 each represent an EXTI line.
/// (0 = masked, 1 = unmasked)
pub const IRQ_MASK: u8 = 0x10;

/// Pending interrupts (u32).
/// Bits 0 - 15 each represent an EXTI line.
///
/// All pending interrupts are cleared by reading this register.
pub const IRQ_PENDING: u8 = 0x11;

/// Rising edge trigger enable for EXTI line.
/// u32 with each bit representing an EXTI line.
pub const IRQ_EXTI_TRIGGER_RISING_EDGE: u8 = 0x12;

/// Falling edge trigger enable for EXTI line.
/// u32 with each bit representing an EXTI line.
pub const IRQ_EXTI_TRIGGER_FALLING_EDGE: u8 = 0x13;

/// High level trigger enable for EXTI GPIO line.
/// u16 with each bit representing an EXTI GPIO line.
pub const IRQ_EXTI_TRIGGER_HIGH_LEVEL: u8 = 0x14;

/// Low level trigger enable for EXTI GPIO line.
/// u16 with each bit representing an EXTI GPIO line.
pub const IRQ_EXTI_TRIGGER_LOW_LEVEL: u8 = 0x15;

/// Source GPIO banks of EXTI lines.
/// u64 with 4 bits representing an EXTI line.
pub const IRQ_EXTI_GPIO_SRC: u8 = 0x16;

/// Triggers EXTI interrupts for lines with level trigger.
pub const IRQ_EXTI_DO_TRIGGER_LEVEL: u8 = 0x17;

/// Unlock write access to watchdog settings.
/// Write u64 watchdog code to unlock.
/// Read u8 with 1=unlocked, 0=locked.
pub const WDG_UNLOCK: u8 = 0x20;

/// Watchdog petting interval.
/// Writing requires unlocked watchdog.
/// u32 in milliseconds.
pub const WDG_INTERVAL: u8 = 0x21;

/// Watchdog active.
/// Writing requires unlocked watchdog.
/// u8 with 1=active, 0=inactive.
pub const WDG_ACTIVE: u8 = 0x22;

/// Pet watchdog.
/// Send u32 watchdog code previously set.
pub const WDG_PET: u8 = 0x23;

/// Sets watchdog pet code.
/// Send u32 watchdog code.
pub const WDG_PET_CODE: u8 = 0x24;

/// RTC ready.
/// u8 with 1=ready, 0=not ready.
pub const RTC_READY: u8 = 0x30;

/// RTC clock source.
/// 0=none, 1=external, 2=internal.
pub const RTC_SRC: u8 = 0x31;

/// RTC clock prescaler. (read-only)
/// u32 in ticks.
pub const RTC_PRESCALER: u8 = 0x32;

/// RTC clock slowdown for calibration.
/// u8 in 1_000_000/2^20 PPM.
pub const RTC_SLOWDOWN: u8 = 0x33;

/// RTC clock.
/// u32 in seconds.
pub const RTC_CLOCK: u8 = 0x34;

/// RTC alarm time. (write-only)
/// u32 in seconds.
pub const RTC_ALARM: u8 = 0x35;

/// RTC alarm armed.
/// u8 with 1=armed, 0=off.
pub const RTC_ALARM_ARMED: u8 = 0x36;

/// RTC alarm occurred.
/// u8 with 1=occurred, 0=not occurred.
/// Write 0 to clear.
pub const RTC_ALARM_OCCURRED: u8 = 0x37;

/// RTC alarm active at boot.
/// u8 with 0x01=active, 0x00=not active, 0xff=not available yet.
pub const RTC_ALARM_AT_BOOT: u8 = 0x38;

/// Power system off.
/// u16 specifying the time delay in milliseconds.
pub const POWER_OFF: u8 = 0x40;

/// Restart system.
/// u16 specifying the time delay in milliseconds.
/// If bit 15 is set, system restarts into charging mode,
/// even when POWER_ON_BY_CHARGING is enabled.
pub const POWER_RESTART: u8 = 0x41;

/// Prohibit power off.
/// u8 with 1=prohibited, 0=allowed.
/// This is stored in flash.
pub const POWER_OFF_PROHIBITED: u8 = 0x42;

/// Power on system when charger is connected.
/// u8 with 2=quiet power on, 1=power on, 0=charge mode.
/// This is stored in flash.
pub const POWER_ON_BY_CHARGING: u8 = 0x43;

/// Whether the current power on is due to charger attachment.
/// u8 with 1=yes, 0=no.
pub const POWERED_ON_BY_CHARGER: u8 = 0x44;

/// GPIO count.
pub const GPIO_COUNT: u8 = 0x50;

/// GPIO configuration register.
/// One u64 per GPIO port. (CRH followed by CRL)
pub const GPIO_CFG: u8 = 0x51;

/// GPIO output register.
/// One u16 per GPIO port.
pub const GPIO_OUT: u8 = 0x52;

/// GPIO input register.
/// One u16 per GPIO port.
pub const GPIO_IN: u8 = 0x53;

/// Mask of GPIOs that are usable.
/// One u16 per GPIO port.
pub const GPIO_USABLE: u8 = 0x54;

/// Starts ADC conversion.
/// Sets ADC conversion done to in progress.
pub const ADC_CONVERT: u8 = 0x60;

/// ADC conversion done.
/// u8 with 1=done, 0=in progress.
pub const ADC_READY: u8 = 0x61;

/// ADC reference voltage.
/// u16 in mV.
pub const ADC_VREF: u8 = 0x62;

/// ADC values in mV.
/// One u16 for each ADC channel.
pub const ADC_VALUES: u8 = 0x63;

/// ADC temperature in m°C.
/// One u16.
pub const ADC_TEMPERATURE: u8 = 0x64;

/// Number of PWM timers (read-only, u8).
pub const PWM_TIMERS: u8 = 0x70;

/// The PWM timer to configure (u8).
pub const PWM_TIMER: u8 = 0x71;

/// Number of PWM channels of the current PWM timer (read-only, u8).
pub const PWM_TIMER_CHANNELS: u8 = 0x72;

/// Pin remapping configuration of the current PWM timer (u8).
pub const PWM_TIMER_REMAP: u8 = 0x73;

/// Frequency of the current PWM timer in Hz (write-only, u32).
pub const PWM_TIMER_FREQUENCY: u8 = 0x74;

/// The PWM channel to configure (u8).
pub const PWM_CHANNEL: u8 = 0x75;

/// The duty cycle of the current PWM channel (write-only, u16).
pub const PWM_CHANNEL_DUTY_CYCLE: u8 = 0x76;

/// The polarity of the current PWM channel (write-only, u8).
/// 0=not inverted, 1=inverted.
pub const PWM_CHANNEL_POLARITY: u8 = 0x77;

/// Output of current PWM channel. (write-only, u8)
/// 1=enabled, 0=disabled.
pub const PWM_CHANNEL_OUTPUT: u8 = 0x78;

/// Current battery voltage in mV. (read-only, u32)
pub const BATTERY_VOLTAGE: u8 = 0x80;

/// Minimum battery voltage in mV. (read-only, u32)
pub const BATTERY_MIN_VOLTAGE: u8 = 0x81;

/// Maximum battery voltage in mV. (read-only, u32)
pub const BATTERY_MAX_VOLTAGE: u8 = 0x82;

/// Battery charging status. (read-only, u8)
/// 0=off, 1=pre, 2=fast.
pub const BATTERY_CHARGING: u8 = 0x83;

/// Battery charging voltage in mV. (read-only, u32)
pub const BATTERY_CONSTANT_CHARGE_VOLTAGE: u8 = 0x84;

/// Battery charging current in mA. (read-only, u32)
pub const BATTERY_CONSTANT_CHARGE_CURRENT: u8 = 0x85;

/// Battery current in mA. (read-only, i32)
pub const BATTERY_CURRENT: u8 = 0x86;

/// System voltage measured by the battery charger in mV. (read-only, u32)
pub const BATTERY_SYSTEM_VOLTAGE: u8 = 0x87;

/// Battery status. (read-only, u8)
/// 0=unknown, 1=not present, 2=present.
pub const BATTERY_STATUS: u8 = 0x88;

/// External power supply present. (read-only, u8)
/// 0=unknown, 1=disconnected, 2=USB default, 3=USB PD contract, 4=USB 1.5A, 5=USB 3.0A, 6=negotiating.
pub const SUPPLY_TYPE: u8 = 0x90;

/// External power supply requested voltage in mV. (read-only, u32)
pub const SUPPLY_REQUESTED_VOLTAGE: u8 = 0x91;

/// External power supply maximum current in mA. (read-only, u32)
pub const SUPPLY_MAX_CURRENT: u8 = 0x92;

/// External power supply USB communication status. (read-only, u8)
/// 0=unsupported, 1=supported, 2=unknown.
pub const SUPPLY_USB_COMMUNICATION: u8 = 0x93;

/// External power supply actual voltage in mV. (read-only, u32)
pub const SUPPLY_VOLTAGE: u8 = 0x94;

/// External power supply current in mA. (read-only, u32)
pub const SUPPLY_CURRENT: u8 = 0x95;

/// External power supply USB communication connection. (read/write, u8)
/// 0=disconnected, 1=connected.
pub const SUPPLY_CONNECT_DATA: u8 = 0x96;

/// Platform store size. (read-only, u16)
pub const PSTORE_SIZE: u8 = 0xa0;

/// Platform store I/O address. (read-write, u16)
pub const PSTORE_IO_ADDRESS: u8 = 0xa1;

/// Platform store I/O size. (read-write, u8)
pub const PSTORE_IO_SIZE: u8 = 0xa2;

/// Platform store I/O. (read-write, size specified by PSTORE_IO_SIZE)
pub const PSTORE_IO: u8 = 0xa3;

/// Read or write to board handler. (read/write)
/// First byte of read response is length.
pub const BOARD_IO: u8 = 0xe0;

/// Board io status. (read-only, u8)
/// 0=write okay, 1=write block.
pub const BOARD_IO_STATUS: u8 = 0xe1;

/// Board handler ioctl. (write then read when status is done)
pub const BOARD_IOCTL: u8 = 0xe2;

/// Board handler ioctl processing status. (read-only, u8)
/// 0=idle, 1=processing, 2=done.
pub const BOARD_IOCTL_STATUS: u8 = 0xe3;

/// Resets the system.
pub const RESET: u8 = 0xf0;

/// Checksum enable.
///
/// Read: enabled (u8).
/// Write: 8 bytes with the codes below.
pub const CHECKSUM_ENABLE: u8 = 0xf7;

/// Checksum enable code.
pub const CHECKSUM_ENABLE_CODE: u64 = 0x00ccee24771142ff;

/// Checksum disable code.
pub const CHECKSUM_DISABLE_CODE: u64 = 0x00dd1199456296ff;

/// Checksum status. (10 bytes)
///
/// Read:
/// - status (u8, bit 0 = last transaction matched checksum)
/// - CRC32 of last response (u32)
/// - CRC32 of this message (u32)
///
/// Write:
/// - CRC32 of coming request (u32)
/// - read length of response (u8)
pub const CHECKSUM: u8 = 0xf8;

/// Read response of last request without side effects.
pub const REREAD: u8 = 0xf9;

/// Read log.
/// 32 bytes.
/// First byte: length with MSB set if data was lost.
pub const LOG_READ: u8 = 0xfa;

/// Read bootloader log.
/// 32 bytes.
/// First byte: length
pub const BOOTLOADER_LOG_READ: u8 = 0xfb;

/// Reads the CRC32 of the bootloader. (read-only, u32)
pub const BOOTLOADER_CRC32: u8 = 0xfc;

/// Read/write the same data.
pub const ECHO: u8 = 0xfe;

/// Restarts the bootloader.
pub const START_BOOTLOADER: u8 = 0xff;
