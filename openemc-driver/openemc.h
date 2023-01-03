// SPDX-License-Identifier: GPL-2.0+
/*
 * MFD core driver for OpenEMC
 *
 * Copyright (C) 2022 Sebastian Urban <surban@surban.net>
 */

#ifndef __LINUX_MFD_OPENEMC_H
#define __LINUX_MFD_OPENEMC_H

#include <linux/i2c.h>
#include <linux/mfd/core.h>

#define OPENEMC_MAX_PINS 64
#define OPENEMC_MAX_DATA_SIZE 32
#define OPENEMC_MAX_CELLS 32

#define OPENEMC_IRQS 32
#define OPENEMC_EXT_IRQS 16

/* Info register definitions */
#define OPENEMC_ID 0x00
#define OPENEMC_VERSION 0x01
#define OPENEMC_EMC_MODEL 0x02
#define OPENEMC_BOARD_MODEL 0x03
#define OPENEMC_BOOTLOADER_VERSION 0x04
#define OPENEMC_COPYRIGHT 0x06
#define OPENEMC_MFD_CELL 0x07
#define OPENEMC_BOOT_REASON 0x08
#define OPENEMC_RESET_STATUS 0x09
#define OPENEMC_START_REASON 0x0a
#define OPENEMC_PROGRAM_ID 0x0b

/* IRQ register definitions */
#define OPENEMC_IRQ_MASK 0x10
#define OPENEMC_IRQ_PENDING 0x11
#define OPENEMC_IRQ_EXTI_TRIGGER_RISING_EDGE 0x12
#define OPENEMC_IRQ_EXTI_TRIGGER_FALLING_EDGE 0x13
#define OPENEMC_IRQ_EXTI_TRIGGER_HIGH_LEVEL 0x14
#define OPENEMC_IRQ_EXTI_TRIGGER_LOW_LEVEL 0x15
#define OPENEMC_IRQ_EXTI_GPIO_SRC 0x16
#define OPENEMC_IRQ_EXTI_DO_TRIGGER_LEVEL 0x17

/* Watchdog register definitions */
#define OPENEMC_WDG_UNLOCK 0x20
#define OPENEMC_WDG_INTERVAL 0x21
#define OPENEMC_WDG_ACTIVE 0x22
#define OPENEMC_WDG_PET 0x23
#define OPENEMC_WDG_PET_CODE 0x24

/* RTC register definitions */
#define OPENEMC_REG_RTC_READY 0x30
#define OPENEMC_REG_RTC_SRC 0x31
#define OPENEMC_REG_RTC_PRESCALER 0x32
#define OPENEMC_REG_RTC_SLOWDOWN 0x33
#define OPENEMC_REG_RTC_CLOCK 0x34
#define OPENEMC_REG_RTC_ALARM 0x35
#define OPENEMC_REG_RTC_ALARM_ARMED 0x36
#define OPENEMC_REG_RTC_ALARM_OCCURRED 0x37
#define OPENEMC_REG_RTC_ALARM_AT_BOOT 0x38

/* Power register definitions */
#define OPENEMC_POWER_OFF 0x40
#define OPENEMC_POWER_RESTART 0x41

/* GPIO register definitions */
#define OPENEMC_GPIO_COUNT 0x50
#define OPENEMC_GPIO_CFG 0x51
#define OPENEMC_GPIO_OUT 0x52
#define OPENEMC_GPIO_IN 0x53
#define OPENEMC_GPIO_USABLE 0x54

/* AD converter register definitions */
#define OPENEMC_ADC_CONVERT 0x60
#define OPENEMC_ADC_READY 0x61
#define OPENEMC_ADC_VREF 0x62
#define OPENEMC_ADC_VALUES 0x63
#define OPENEMC_ADC_TEMPERATURE 0x64

/* PWM register definitions */
#define OPENEMC_PWM_TIMERS 0x70
#define OPENEMC_PWM_TIMER 0x71
#define OPENEMC_PWM_TIMER_CHANNELS 0x72
#define OPENEMC_PWM_TIMER_REMAP 0x73
#define OPENEMC_PWM_TIMER_FREQUENCY 0x74
#define OPENEMC_PWM_CHANNEL 0x75
#define OPENEMC_PWM_CHANNEL_DUTY_CYCLE 0x76
#define OPENEMC_PWM_CHANNEL_POLARITY 0x77
#define OPENEMC_PWM_CHANNEL_OUTPUT 0x78

/* Battery register definitions */
#define OPENEMC_BATTERY_VOLTAGE 0x80
#define OPENEMC_BATTERY_MIN_VOLTAGE 0x81
#define OPENEMC_BATTERY_MAX_VOLTAGE 0x82
#define OPENEMC_BATTERY_CHARGING 0x83
#define OPENEMC_BATTERY_CONSTANT_CHARGE_VOLTAGE 0x84
#define OPENEMC_BATTERY_CONSTANT_CHARGE_CURRENT 0x85

/* Supply register definitions */
#define OPENEMC_SUPPLY_TYPE 0x90
#define OPENEMC_SUPPLY_VOLTAGE 0x91
#define OPENEMC_SUPPLY_MAX_CURRENT 0x92
#define OPENEMC_SUPPLY_USB_COMMUNICATION 0x93

/* Reset register definitions */
#define OPENEMC_RESET 0xf0
#define OPENEMC_START_BOOTLOADER 0xff

/* Bootloader register definitions */
#define OPENEMC_BOOTLOADER_FLASH_START 0x10
#define OPENEMC_BOOTLOADER_FLASH_END 0x11
#define OPENEMC_BOOTLOADER_FLASH_PAGE_SIZE 0x12
#define OPENEMC_BOOTLOADER_FLASH_ADDR 0x13
#define OPENEMC_BOOTLOADER_FLASH_READ 0x14
#define OPENEMC_BOOTLOADER_FLASH_PAGE_CRC32 0x15
#define OPENEMC_BOOTLOADER_FLASH_MODIFY_UNLOCK 0x16
#define OPENEMC_BOOTLOADER_FLASH_ERASE_PAGE 0x17
#define OPENEMC_BOOTLOADER_FLASH_WRITE 0x18
#define OPENEMC_BOOTLOADER_FLASH_STATUS 0x19
#define OPENEMC_BOOTLOADER_START_PROGRAM 0x20
#define OPENEMC_BOOTLOADER_VERIFY_PROGRAM 0x21
#define OPENEMC_BOOTLOADER_VERIFY_PROGRAM_RESULT 0x22
#define OPENEMC_BOOTLOADER_TIMEOUT_ENABLED 0x30

/* Id responses */
#define OPENEMC_ID_BOOTLOADER 0xb0
#define OPENEMC_ID_FIRMWARE_STANDALONE 0xf0
#define OPENEMC_ID_FIRMWARE_WITH_BOOTLOADER 0xfb

/* Watchdog codes */
#define OPENEMC_WDT_UNLOCK_CODE 0x1984090204061986

/* Reset status bits */
#define OPENEMC_RESET_WAKEUP BIT(1)
#define OPENEMC_RESET_EXTERNAL BIT(2)
#define OPENEMC_RESET_POWER_ON BIT(3)
#define OPENEMC_RESET_FIRMWARE BIT(4)
#define OPENEMC_RESET_WATCHDOG BIT(5)
#define OPENEMC_RESET_WINDOW_WATCHDOG BIT(6)
#define OPENEMC_RESET_LOW_POWER BIT(7)

/* Interrupts */
#define OPENEMC_IRQ_RTC_ALARM 17
#define OPENEMC_IRQ_BATTERY 20
#define OPENEMC_IRQ_SUPPLY 21

enum openemc_pin_mode {
	OPENEMC_PIN_MODE_GPIO,
	OPENEMC_PIN_MODE_ALTERNATE_OUT,
	OPENEMC_PIN_MODE_ANALOG,
};

enum openemc_pin_direction {
	OPENEMC_PIN_DIRECTION_IN,
	OPENEMC_PIN_DIRECTION_OUT,
};

enum openemc_pin_bias {
	OPENEMC_PIN_BIAS_DISABLE,
	OPENEMC_PIN_BIAS_PULL_DOWN,
	OPENEMC_PIN_BIAS_PULL_UP,
};

enum openemc_pin_drive {
	OPENEMC_PIN_DRIVE_OPEN_DRAIN,
	OPENEMC_PIN_DRIVE_PUSH_PULL,
};

enum openemc_pin_speed {
	OPENEMC_PIN_SPEED_2_MHZ,
	OPENEMC_PIN_SPEED_10_MHZ,
	OPENEMC_PIN_SPEED_50_MHZ,
};

struct openemc_pin_state {
	unsigned int pin;
	bool usable;

	enum openemc_pin_mode mode;
	enum openemc_pin_direction direction;

	/* input configuration */
	enum openemc_pin_bias in_bias;

	/* output configuration */
	enum openemc_pin_drive out_drive;
	enum openemc_pin_speed out_speed;
	bool out_level;
};

struct openemc {
	struct device *dev;
	struct i2c_client *i2c;
	struct mutex lock;

	u8 id;
	char version[OPENEMC_MAX_DATA_SIZE + 1];
	char bootloader_version[OPENEMC_MAX_DATA_SIZE + 1];
	char copyright[8 * OPENEMC_MAX_DATA_SIZE];
	u8 emc_model;
	char board_model[OPENEMC_MAX_DATA_SIZE + 1];
	u16 boot_reason;
	u8 reset_status;
	u8 start_reason;
	u32 program_id;
	struct mfd_cell cells[OPENEMC_MAX_CELLS];
	int n_cells;

	unsigned int npin;
	struct openemc_pin_state pin[OPENEMC_MAX_PINS];
	char *pin_names[OPENEMC_MAX_PINS];

	bool pinctrl_loaded;
	struct module *pinctrl_mod;

	u32 wdt_pet_code;

	unsigned int irq;
	struct irq_domain *irq_domain;
	struct mutex irq_lock;
	u32 irq_mask;
	u32 irq_alloced;
	unsigned int irq_types[OPENEMC_IRQS];
	int exti_bank[OPENEMC_EXT_IRQS];
};

int openemc_cmd(struct openemc *emc, u8 command);
int openemc_read_u8(struct openemc *emc, u8 command, u8 *value);
int openemc_write_u8(struct openemc *emc, u8 command, u8 value);
int openemc_read_u16(struct openemc *emc, u8 command, u16 *value);
int openemc_write_u16(struct openemc *emc, u8 command, u16 value);
int openemc_read_u32(struct openemc *emc, u8 command, u32 *value);
int openemc_write_u32(struct openemc *emc, u8 command, u32 value);
int openemc_read_u64(struct openemc *emc, u8 command, u64 *value);
int openemc_write_u64(struct openemc *emc, u8 command, u64 value);
int openemc_read_data(struct openemc *emc, u8 command, u8 len, u8 *data);
int openemc_write_data(struct openemc *emc, u8 command, u8 len, const u8 *data);

void openemc_irq_do_sync(struct openemc *emc);
void openemc_irq_do_mask(struct openemc *emc, unsigned long hwirq);
void openemc_irq_do_unmask(struct openemc *emc, unsigned long hwirq);
int openemc_irq_check_type(unsigned int type);
int openemc_irq_do_set_type(struct openemc *emc, unsigned long hwirq,
			    unsigned int type);

int openemc_pins_write_cfg(struct openemc *emc);
int openemc_pins_write_out(struct openemc *emc);

#endif /* __LINUX_MFD_OPENEMC_H */
