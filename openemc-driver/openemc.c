// SPDX-License-Identifier: GPL-2.0+
/*
 * MFD core driver for OpenEMC
 *
 * Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/random.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "openemc.h"

static const struct of_device_id openemc_of_match[] = {
	{ .compatible = "openemc,openemc" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_of_match);

#define MODE_MASK 0b0011
#define MODE_OUTPUT_10_MHZ 0b0001
#define MODE_OUTPUT_2_MHZ 0b0010
#define MODE_OUTPUT_50_MHZ 0b0011

#define CNF_INPUT_FLOATING 0b0100
#define CNF_INPUT_PULL 0b1000

#define CNF_OUTPUT_ALTERNATE 0b1000
#define CNF_OUTPUT_OPEN_DRAIN 0b0100

enum openemc_req { REQ_READ, REQ_WRITE };

static u32 openemc_crc32(const void *data, size_t length)
{
	if (length % 4 == 0) {
		return crc32(0xffffffff, data, length) ^ 0xffffffff;
	} else if (length <= OPENEMC_MAX_DATA_SIZE + 1) {
		u8 buf[OPENEMC_MAX_DATA_SIZE + 1 + 4];

		memset(buf, 0, ARRAY_SIZE(buf));
		memcpy(buf, data, length);

		while (length % 4 != 0)
			length++;

		return crc32(0xffffffff, buf, length) ^ 0xffffffff;
	} else {
		BUG();
	}
}

static int openemc_read_checksum(struct openemc *emc, bool *command_okay,
				 u32 *crc32)
{
	struct i2c_client *i2c = emc->i2c;
	int retry, ret;

	for (retry = 0; retry < OPENEMC_MAX_RETRIES; retry++) {
		u8 buf[9];
		u32 checksum_crc32, expected_checksum_crc32;

		ret = i2c_smbus_read_i2c_block_data(i2c, OPENEMC_CHECKSUM,
						    ARRAY_SIZE(buf), buf);
		if (ret < 0)
			continue;

		*command_okay = buf[0] & BIT(0);
		*crc32 = (u32)buf[1] | (u32)buf[2] << 8 | (u32)buf[3] << 16 |
			 (u32)buf[4] << 24;
		checksum_crc32 = (u32)buf[5] | (u32)buf[6] << 8 |
				 (u32)buf[7] << 16 | (u32)buf[8] << 24;

		expected_checksum_crc32 = openemc_crc32(buf, 5);
		if (expected_checksum_crc32 == checksum_crc32) {
			ret = 0;
			break;
		} else {
			dev_warn(emc->dev, "checksum CRC32 mismatch\n");
			ret = -EIO;
		}
	}

	return ret;
}

static int openemc_write_checksum(struct openemc *emc, u32 crc32, u8 len)
{
	struct i2c_client *i2c = emc->i2c;
	int retry, ret;
	u8 buf[5];

	buf[0] = crc32 & 0xff;
	buf[1] = (crc32 >> 8) & 0xff;
	buf[2] = (crc32 >> 16) & 0xff;
	buf[3] = (crc32 >> 24) & 0xff;
	buf[4] = len;

	for (retry = 0; retry < OPENEMC_MAX_RETRIES; retry++) {
		ret = i2c_smbus_write_i2c_block_data(i2c, OPENEMC_CHECKSUM,
						     ARRAY_SIZE(buf), buf);
		if (ret >= 0)
			break;
	}

	return ret;
}

static int openemc_request(struct openemc *emc, enum openemc_req req,
			   u8 command, u8 len, u8 *data)
{
	struct i2c_client *i2c = emc->i2c;
	int ret = 0;
	int retry, restarts = 0;
	u8 verify_data[OPENEMC_MAX_DATA_SIZE];

	if (len > OPENEMC_MAX_DATA_SIZE) {
		dev_err(emc->dev, "maximum data size exceeded\n");
		return -EINVAL;
	}

	mutex_lock(&emc->req_lock);

restart:
	/* send crc32 of coming request */
	if (emc->checksum_enabled) {
		u8 req_buf[OPENEMC_MAX_DATA_SIZE + 1];
		size_t req_len;
		u8 rsp_len;

		req_buf[0] = command;

		switch (req) {
		case REQ_READ:
			req_len = 1;
			rsp_len = len;
			break;
		case REQ_WRITE:
			memcpy(&req_buf[1], data, len);
			req_len = 1 + len;
			rsp_len = 0;
			break;
		default:
			BUG();
		}

		ret = openemc_write_checksum(
			emc, openemc_crc32(req_buf, req_len), rsp_len);
		if (ret < 0)
			goto out;
	}

	/* send request */
	for (retry = 0; retry < OPENEMC_MAX_RETRIES; retry++) {
		switch (req) {
		case REQ_READ:
			ret = i2c_smbus_read_i2c_block_data(i2c, command, len,
							    data);
			break;
		case REQ_WRITE:
			ret = i2c_smbus_write_i2c_block_data(i2c, command, len,
							     data);
			break;
		default:
			BUG();
		}
		if (ret >= 0)
			break;
	}
	if (ret < 0)
		goto out;

	if (emc->checksum_enabled) {
		/* get and verify crc32 of response */
		for (retry = 0; retry < OPENEMC_MAX_RETRIES; retry++) {
			bool command_okay;
			u32 crc32;
			u32 expected_crc32;

			/* re-read response */
			if (retry != 0 && req == REQ_READ) {
				ret = i2c_smbus_read_i2c_block_data(
					i2c, OPENEMC_REREAD, len, data);
				if (ret < 0)
					continue;
			}

			ret = openemc_read_checksum(emc, &command_okay, &crc32);
			if (ret < 0)
				goto out;

			if (!command_okay) {
				dev_warn(emc->dev, "command CRC32 mismatch\n");
				ret = -EIO;
				restarts++;
				if (restarts < OPENEMC_MAX_RETRIES)
					goto restart;
				else
					goto out;
			}

			switch (req) {
			case REQ_READ:
				expected_crc32 = openemc_crc32(data, len);
				if (expected_crc32 == crc32) {
					ret = 0;
					goto out;
				} else {
					dev_warn(emc->dev,
						 "data CRC32 mismatch\n");
					ret = -EIO;
				}
				break;
			case REQ_WRITE:
				ret = 0;
				goto out;
			default:
				BUG();
			}
		}
	} else if (req == REQ_READ) {
		/* read twice to verify if checksum is disabled */
		for (retry = 0; retry < OPENEMC_MAX_RETRIES; retry++) {
			ret = i2c_smbus_read_i2c_block_data(i2c, command, len,
							    verify_data);
			if (ret >= 0)
				break;
		}
		if (ret < 0)
			goto out;

		if (memcmp(data, verify_data, len) != 0) {
			dev_warn(emc->dev, "verify read mismatch\n");
			ret = -EIO;
			restarts++;
			if (restarts < OPENEMC_MAX_RETRIES)
				goto restart;
			else
				goto out;
		} else {
			ret = 0;
		}
	}

out:
	mutex_unlock(&emc->req_lock);
	return ret;
}

int openemc_cmd(struct openemc *emc, u8 command)
{
	u8 buf[0];

	return openemc_request(emc, REQ_WRITE, command, ARRAY_SIZE(buf), buf);
}
EXPORT_SYMBOL_GPL(openemc_cmd);

int openemc_read_u8(struct openemc *emc, u8 command, u8 *value)
{
	u8 buf[1];
	int ret;

	ret = openemc_request(emc, REQ_READ, command, ARRAY_SIZE(buf), buf);
	if (ret < 0)
		return ret;

	*value = buf[0];

	return 0;
}
EXPORT_SYMBOL_GPL(openemc_read_u8);

int openemc_write_u8(struct openemc *emc, u8 command, u8 value)
{
	u8 buf[1];

	buf[0] = value;

	return openemc_request(emc, REQ_WRITE, command, ARRAY_SIZE(buf), buf);
}
EXPORT_SYMBOL_GPL(openemc_write_u8);

int openemc_read_u16(struct openemc *emc, u8 command, u16 *value)
{
	u8 buf[2];
	int ret;

	ret = openemc_request(emc, REQ_READ, command, ARRAY_SIZE(buf), buf);
	if (ret < 0)
		return ret;

	*value = (u16)buf[0] | (u16)buf[1] << 8;

	return 0;
}
EXPORT_SYMBOL_GPL(openemc_read_u16);

int openemc_write_u16(struct openemc *emc, u8 command, u16 value)
{
	u8 buf[2];

	buf[0] = value & 0xff;
	buf[1] = (value >> 8) & 0xff;

	return openemc_request(emc, REQ_WRITE, command, ARRAY_SIZE(buf), buf);
}
EXPORT_SYMBOL_GPL(openemc_write_u16);

int openemc_read_u32(struct openemc *emc, u8 command, u32 *value)
{
	u8 buf[4];
	int ret;

	ret = openemc_request(emc, REQ_READ, command, ARRAY_SIZE(buf), buf);
	if (ret < 0)
		return ret;

	*value = (u32)buf[0] | (u32)buf[1] << 8 | (u32)buf[2] << 16 |
		 (u32)buf[3] << 24;

	return 0;
}
EXPORT_SYMBOL_GPL(openemc_read_u32);

int openemc_write_u32(struct openemc *emc, u8 command, u32 value)
{
	u8 buf[4];

	buf[0] = value & 0xff;
	buf[1] = (value >> 8) & 0xff;
	buf[2] = (value >> 16) & 0xff;
	buf[3] = (value >> 24) & 0xff;

	return openemc_request(emc, REQ_WRITE, command, ARRAY_SIZE(buf), buf);
}
EXPORT_SYMBOL_GPL(openemc_write_u32);

int openemc_read_u64(struct openemc *emc, u8 command, u64 *value)
{
	u8 buf[8];
	int ret;

	ret = openemc_request(emc, REQ_READ, command, ARRAY_SIZE(buf), buf);
	if (ret < 0)
		return ret;

	*value = (u64)buf[0] | (u64)buf[1] << 8 | (u64)buf[2] << 16 |
		 (u64)buf[3] << 24 | (u64)buf[4] << 32 | (u64)buf[5] << 40 |
		 (u64)buf[6] << 48 | (u64)buf[7] << 56;

	return 0;
}
EXPORT_SYMBOL_GPL(openemc_read_u64);

int openemc_read_u128(struct openemc *emc, u8 command, u64 *value_lo,
		      u64 *value_hi)
{
	u8 buf[16];
	int ret;

	ret = openemc_request(emc, REQ_READ, command, ARRAY_SIZE(buf), buf);
	if (ret < 0)
		return ret;

	*value_lo = (u64)buf[0] | (u64)buf[1] << 8 | (u64)buf[2] << 16 |
		    (u64)buf[3] << 24 | (u64)buf[4] << 32 | (u64)buf[5] << 40 |
		    (u64)buf[6] << 48 | (u64)buf[7] << 56;
	*value_hi = (u64)buf[8] | (u64)buf[9] << 8 | (u64)buf[10] << 16 |
		    (u64)buf[11] << 24 | (u64)buf[12] << 32 |
		    (u64)buf[13] << 40 | (u64)buf[14] << 48 |
		    (u64)buf[15] << 56;

	return 0;
}
EXPORT_SYMBOL_GPL(openemc_read_u128);

int openemc_write_u64(struct openemc *emc, u8 command, u64 value)
{
	u8 buf[8];

	buf[0] = value & 0xff;
	buf[1] = (value >> 8) & 0xff;
	buf[2] = (value >> 16) & 0xff;
	buf[3] = (value >> 24) & 0xff;
	buf[4] = (value >> 32) & 0xff;
	buf[5] = (value >> 40) & 0xff;
	buf[6] = (value >> 48) & 0xff;
	buf[7] = (value >> 56) & 0xff;

	return openemc_request(emc, REQ_WRITE, command, ARRAY_SIZE(buf), buf);
}
EXPORT_SYMBOL_GPL(openemc_write_u64);

int openemc_write_data(struct openemc *emc, u8 command, u8 len, const u8 *data)
{
	return openemc_request(emc, REQ_WRITE, command, len, (u8 *)data);
}
EXPORT_SYMBOL_GPL(openemc_write_data);

int openemc_read_data(struct openemc *emc, u8 command, u8 len, u8 *data)
{
	return openemc_request(emc, REQ_READ, command, len, data);
}
EXPORT_SYMBOL_GPL(openemc_read_data);

static int openemc_set_checksum_enabled(struct openemc *emc, bool enabled)
{
	int retry, ret;
	u64 req;

	if (enabled)
		req = OPENEMC_CHECKSUM_ENABLE_CODE;
	else
		req = OPENEMC_CHECKSUM_DISABLE_CODE;

	emc->checksum_enabled = false;

	for (retry = 0; retry < OPENEMC_MAX_RETRIES; retry++) {
		u8 ret_enabled;

		ret = openemc_write_u64(emc, OPENEMC_CHECKSUM_ENABLE, req);
		if (ret < 0)
			continue;

		ret = openemc_read_u8(emc, OPENEMC_CHECKSUM_ENABLE,
				      &ret_enabled);
		if (ret < 0)
			continue;

		if ((ret_enabled != 0) == enabled) {
			emc->checksum_enabled = enabled;
			break;
		}
	}

	return ret;
}

static u8 openemc_flash_lock_code[] = { 0x00, 0x00, 0x00, 0x00 };
static u8 openemc_flash_unlock_code[] = { 0xb0, 0xf0, 0x01, 0xaa };

static int openemc_set_flash_lock(struct openemc *emc, bool lock)
{
	int ret;
	u8 result;

	if (lock) {
		ret = openemc_write_data(emc,
					 OPENEMC_BOOTLOADER_FLASH_MODIFY_UNLOCK,
					 ARRAY_SIZE(openemc_flash_lock_code),
					 openemc_flash_lock_code);
	} else {
		ret = openemc_write_data(emc,
					 OPENEMC_BOOTLOADER_FLASH_MODIFY_UNLOCK,
					 ARRAY_SIZE(openemc_flash_unlock_code),
					 openemc_flash_unlock_code);
	}

	if (ret < 0)
		return ret;

	ret = openemc_read_u8(emc, OPENEMC_BOOTLOADER_FLASH_MODIFY_UNLOCK,
			      &result);
	if (ret < 0)
		return ret;

	if (lock && result) {
		dev_err(emc->dev, "locking flash failed\n");
		return -EIO;
	} else if (!lock && !result) {
		dev_err(emc->dev, "unlocking flash failed\n");
		return -EIO;
	}

	return 0;
}

static int openemc_flash_firmware_page(struct openemc *emc,
				       const struct firmware *firmware,
				       u32 start, u32 page, u32 page_size)
{
	u32 addr = start + page;
	u32 i, flash_crc32, firmware_crc32;
	u8 *page_data;
	u8 len, status;
	int ret;

	dev_info(emc->dev, "flashing 0x%08x - 0x%08x\n", addr,
		 addr + page_size - 1);

	ret = openemc_write_u32(emc, OPENEMC_BOOTLOADER_FLASH_ADDR, addr);
	if (ret < 0)
		goto out;

	ret = openemc_read_u8(emc, OPENEMC_BOOTLOADER_FLASH_STATUS, &status);
	if (ret < 0)
		goto out;

	ret = openemc_cmd(emc, OPENEMC_BOOTLOADER_FLASH_ERASE_PAGE);
	if (ret < 0)
		goto out;

	ret = openemc_read_u8(emc, OPENEMC_BOOTLOADER_FLASH_STATUS, &status);
	if (ret < 0)
		goto out;
	if (status != 0x00) {
		dev_err(emc->dev, "erasing page 0x%08x failed: %d\n", addr,
			status);
		ret = -ENODEV;
		goto out;
	}

	if (page_size > OPENEMC_MAX_PAGE_SIZE) {
		ret = -EINVAL;
		goto out;
	}

	page_data = devm_kzalloc(emc->dev, page_size, GFP_KERNEL);
	if (!page_data) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < page_size; i++) {
		if (page + i < firmware->size)
			page_data[i] = firmware->data[page + i];
		else
			page_data[i] = 0;
	}

	firmware_crc32 = openemc_crc32(page_data, page_size);

	for (i = 0; i < page_size; i += OPENEMC_MAX_DATA_SIZE) {
		len = min((u32)OPENEMC_MAX_DATA_SIZE, page_size - i);
		ret = openemc_write_data(emc, OPENEMC_BOOTLOADER_FLASH_WRITE,
					 len, page_data + i);
		if (ret < 0)
			goto out;
	}

	ret = openemc_read_u8(emc, OPENEMC_BOOTLOADER_FLASH_STATUS, &status);
	if (ret < 0)
		goto out;
	if (status != 0x00) {
		dev_err(emc->dev, "flashing page 0x%08x failed: %d\n", addr,
			status);
		ret = -ENODEV;
		goto out;
	}

	addr = start + page;
	ret = openemc_write_u32(emc, OPENEMC_BOOTLOADER_FLASH_ADDR, addr);
	if (ret < 0)
		goto out;

	ret = openemc_read_u32(emc, OPENEMC_BOOTLOADER_FLASH_PAGE_CRC32,
			       &flash_crc32);
	if (ret < 0)
		goto out;

	if (flash_crc32 != firmware_crc32) {
		dev_err(emc->dev,
			"CRC32 mismatch after flashing page "
			"0x%08x (flash 0x%08x, firmware 0x%08x)\n",
			addr, flash_crc32, firmware_crc32);
		ret = -EIO;
		goto out;
	}

out:
	if (page_data)
		devm_kfree(emc->dev, page_data);
	return ret;
}

static int openemc_flash_firmware(struct openemc *emc,
				  const struct firmware *firmware)
{
	int ret;
	u32 start, end, size, page, page_size;

	ret = openemc_read_u32(emc, OPENEMC_BOOTLOADER_FLASH_START, &start);
	if (ret < 0)
		return ret;
	ret = openemc_read_u32(emc, OPENEMC_BOOTLOADER_FLASH_END, &end);
	if (ret < 0)
		return ret;
	ret = openemc_read_u32(emc, OPENEMC_BOOTLOADER_FLASH_PAGE_SIZE,
			       &page_size);
	if (ret < 0)
		return ret;

	dev_dbg(emc->dev,
		"flash range is 0x%08x - 0x%08x with page size 0x%08x\n", start,
		end, page_size);

	if (end < start) {
		dev_err(emc->dev, "invalid flash range: 0x%08x - 0x%08x\n",
			start, end);
		return -ENODEV;
	}
	size = end - start;
	if (size % page_size != 0) {
		dev_err(emc->dev, "invalid flash page size %d\n", page_size);
		return -ENODEV;
	}
	if (size < firmware->size) {
		dev_err(emc->dev,
			"flash has only %u bytes, but firmware has %zu bytes",
			size, firmware->size);
		return -ENODEV;
	}

	ret = openemc_set_flash_lock(emc, false);
	if (ret < 0)
		return ret;

	for (page = 0; page < firmware->size; page += page_size) {
		ret = openemc_flash_firmware_page(emc, firmware, start, page,
						  page_size);
		if (ret < 0)
			break;
	}

	openemc_set_flash_lock(emc, true);

	return ret;
}

static int openemc_start_firmware(struct openemc *emc, const char *filename)
{
	int ret;
	u8 verified;
	u32 firmware_id;
	char filename_buf[128 + OPENEMC_MAX_DATA_SIZE];
	char *delim;
	const struct firmware *firmware = NULL;

	ret = openemc_set_checksum_enabled(emc, false);
	if (ret < 0) {
		dev_err(emc->dev, "failed to disable checksum: %d\n", ret);
		goto out;
	}

	ret = openemc_read_u8(emc, OPENEMC_ID, &emc->id);
	if (ret < 0) {
		dev_err(emc->dev, "failed to read id: %d\n", ret);
		goto out;
	}

	if (emc->id != OPENEMC_ID_FIRMWARE_STANDALONE &&
	    emc->id != OPENEMC_ID_FIRMWARE_WITH_BOOTLOADER &&
	    emc->id != OPENEMC_ID_BOOTLOADER) {
		dev_err(emc->dev, "unknown id: 0x%02x\n", emc->id);
		return -ENODEV;
	}

	if (emc->id != OPENEMC_ID_BOOTLOADER) {
		ret = openemc_set_checksum_enabled(emc, true);
		if (ret < 0)
			goto out;
	}

	ret = openemc_read_data(emc, OPENEMC_BOOTLOADER_VERSION,
				OPENEMC_MAX_DATA_SIZE, emc->bootloader_version);
	if (ret < 0)
		goto out;

	ret = openemc_read_u8(emc, OPENEMC_EMC_MODEL, &emc->emc_model);
	if (ret < 0)
		goto out;

	ret = openemc_read_data(emc, OPENEMC_BOARD_MODEL, OPENEMC_MAX_DATA_SIZE,
				emc->board_model);
	if (ret < 0)
		goto out;

	ret = openemc_read_u32(emc, OPENEMC_PROGRAM_ID, &emc->program_id);
	if (ret < 0)
		goto out;

	if (emc->id == OPENEMC_ID_FIRMWARE_STANDALONE) {
		dev_info(emc->dev, "OpenEMC is running standalone firmware\n");
		ret = 0;
		goto out;
	}

	dev_info(emc->dev, "OpenEMC bootloader version %s on board %s\n",
		 emc->bootloader_version, emc->board_model);

	if (filename) {
		ret = request_firmware(&firmware, filename, emc->dev);
		if (ret < 0)
			goto out;
	} else {
		sprintf(filename_buf, "openemc_%02x_%s.emc", emc->emc_model,
			emc->board_model);
		filename = filename_buf;

		while (true) {
			dev_dbg(emc->dev, "trying firmware %s\n", filename);

			ret = firmware_request_nowarn(&firmware, filename,
						      emc->dev);
			if (ret == 0)
				break;

			delim = strrchr(filename_buf, '_');
			if (!delim)
				break;
			sprintf(delim, ".emc");
		}

		if (ret < 0) {
			dev_warn(emc->dev, "cannot load firmware: %d\n", ret);

			if (emc->id == OPENEMC_ID_BOOTLOADER &&
			    !emc->program_id) {
				dev_err(emc->dev, "no firmware available\n");
				ret = -ENODEV;
				goto out;
			}
		}
	}

	if (firmware && firmware->size >= sizeof(u32)) {
		firmware_id = *((u32 *)(firmware->data + firmware->size) - 1);

		dev_info(emc->dev,
			 "using firmware %s with id %08x (installed: %08x)\n",
			 filename, firmware_id, emc->program_id);

		strncpy(emc->firmware, filename, ARRAY_SIZE(emc->firmware));

		if (emc->program_id == 0 || emc->program_id != firmware_id) {
			ret = openemc_set_checksum_enabled(emc, false);
			if (ret < 0)
				goto out;

			if (emc->id != OPENEMC_ID_BOOTLOADER) {
				dev_info(emc->dev, "starting bootloader\n");

				ret = openemc_cmd(emc,
						  OPENEMC_START_BOOTLOADER);
				if (ret < 0)
					goto out;

				msleep(1000);
			}

			ret = openemc_read_u8(emc, OPENEMC_ID, &emc->id);
			if (ret < 0)
				goto out;
			if (emc->id != OPENEMC_ID_BOOTLOADER) {
				dev_err(emc->dev,
					"failed to start bootloader\n");
				ret = -ENODEV;
				goto out;
			}

			ret = openemc_flash_firmware(emc, firmware);
			if (ret < 0)
				goto out;

			dev_dbg(emc->dev, "verifying firmware\n");
			ret = openemc_cmd(emc,
					  OPENEMC_BOOTLOADER_VERIFY_PROGRAM);
			if (ret < 0)
				goto out;

			msleep(200);

			ret = openemc_read_u8(
				emc, OPENEMC_BOOTLOADER_VERIFY_PROGRAM_RESULT,
				&verified);
			if (ret < 0)
				goto out;
			if (verified != 0x00) {
				dev_err(emc->dev,
					"verifying firmware failed: 0x%02x",
					verified);
				ret = -ENODEV;
				goto out;
			}
		}
	}

	ret = openemc_read_u8(emc, OPENEMC_ID, &emc->id);
	if (ret < 0)
		goto out;

	if (emc->id == OPENEMC_ID_BOOTLOADER) {
		dev_info(emc->dev, "starting firmware\n");
		ret = openemc_cmd(emc, OPENEMC_BOOTLOADER_START_PROGRAM);
		if (ret < 0)
			goto out;

		msleep(1000);
	}

	ret = openemc_set_checksum_enabled(emc, true);
	if (ret < 0)
		goto out;

	ret = openemc_read_u8(emc, OPENEMC_ID, &emc->id);
	if (ret < 0)
		goto out;
	if (emc->id != OPENEMC_ID_FIRMWARE_WITH_BOOTLOADER) {
		dev_err(emc->dev, "failed to start firmware\n");
		ret = -ENODEV;
		goto out;
	}

	ret = 0;

out:
	if (firmware)
		release_firmware(firmware);
	return ret;
}

static int openemc_pins_read(struct openemc *emc)
{
	u8 usable[OPENEMC_MAX_PINS / 8];
	u8 cfgs[OPENEMC_MAX_PINS * 4 / 8];
	u8 out[OPENEMC_MAX_PINS / 8];
	u8 cnf_mode, npin;
	bool on;
	int n, ret = 0;

	ret = openemc_read_u8(emc, OPENEMC_GPIO_COUNT, &npin);
	if (ret < 0)
		return ret;
	emc->npin = min((int)npin, OPENEMC_MAX_PINS);

	ret = openemc_read_data(emc, OPENEMC_GPIO_USABLE, emc->npin / 8,
				usable);
	if (ret < 0)
		return ret;

	ret = openemc_read_data(emc, OPENEMC_GPIO_CFG, emc->npin * 4 / 8, cfgs);
	if (ret < 0)
		return ret;

	ret = openemc_read_data(emc, OPENEMC_GPIO_OUT, emc->npin / 8, out);
	if (ret < 0)
		return ret;

	for (n = 0; n < emc->npin; n++) {
		emc->pin[n].pin = n;
		emc->pin[n].usable = usable[n / 8] & BIT(n % 8);

		emc->pin_names[n] = devm_kzalloc(emc->dev, 5, GFP_KERNEL);
		if (!emc->pin_names[n])
			return -ENOMEM;
		snprintf(emc->pin_names[n], 5, "P%c%d", 'A' + n / 16, n % 16);

		cnf_mode = (cfgs[n * 4 / 8] >> (n * 4 % 8)) & 0b1111;
		on = out[n / 8] & BIT(n % 8);

		if (cnf_mode & MODE_MASK) {
			emc->pin[n].direction = OPENEMC_PIN_DIRECTION_OUT;

			if (cnf_mode & CNF_OUTPUT_OPEN_DRAIN)
				emc->pin[n].out_drive =
					OPENEMC_PIN_DRIVE_OPEN_DRAIN;
			else
				emc->pin[n].out_drive =
					OPENEMC_PIN_DRIVE_PUSH_PULL;

			if (cnf_mode & CNF_OUTPUT_ALTERNATE)
				emc->pin[n].mode =
					OPENEMC_PIN_MODE_ALTERNATE_OUT;
			else
				emc->pin[n].mode = OPENEMC_PIN_MODE_GPIO;

			if ((cnf_mode & MODE_MASK) == MODE_OUTPUT_50_MHZ)
				emc->pin[n].out_speed =
					OPENEMC_PIN_SPEED_50_MHZ;
			else if ((cnf_mode & MODE_MASK) == MODE_OUTPUT_10_MHZ)
				emc->pin[n].out_speed =
					OPENEMC_PIN_SPEED_10_MHZ;
			else if ((cnf_mode & MODE_MASK) == MODE_OUTPUT_2_MHZ)
				emc->pin[n].out_speed = OPENEMC_PIN_SPEED_2_MHZ;

			emc->pin[n].out_level = on;
			emc->pin[n].in_bias = OPENEMC_PIN_BIAS_DISABLE;

		} else {
			emc->pin[n].direction = OPENEMC_PIN_DIRECTION_IN;

			if (cnf_mode & CNF_INPUT_PULL) {
				emc->pin[n].mode = OPENEMC_PIN_MODE_GPIO;
				if (on)
					emc->pin[n].in_bias =
						OPENEMC_PIN_BIAS_PULL_UP;
				else
					emc->pin[n].in_bias =
						OPENEMC_PIN_BIAS_PULL_DOWN;
			} else if (cnf_mode & CNF_INPUT_FLOATING) {
				emc->pin[n].mode = OPENEMC_PIN_MODE_GPIO;
				emc->pin[n].in_bias = OPENEMC_PIN_BIAS_DISABLE;
			} else {
				emc->pin[n].mode = OPENEMC_PIN_MODE_ANALOG;
				emc->pin[n].in_bias = OPENEMC_PIN_BIAS_DISABLE;
			}

			emc->pin[n].out_drive = OPENEMC_PIN_DRIVE_PUSH_PULL;
			emc->pin[n].out_speed = OPENEMC_PIN_SPEED_2_MHZ;
			emc->pin[n].out_level = on;
		}
	}

	return 0;
}

int openemc_pins_write_cfg(struct openemc *emc)
{
	u8 cfgs[OPENEMC_MAX_PINS * 4 / 8] = { 0 };
	u8 cnf_mode;
	int n;

	for (n = 0; n < emc->npin; n++) {
		switch (emc->pin[n].mode) {
		case OPENEMC_PIN_MODE_GPIO:
			switch (emc->pin[n].direction) {
			case OPENEMC_PIN_DIRECTION_IN:
				if (emc->pin[n].in_bias ==
				    OPENEMC_PIN_BIAS_DISABLE)
					cnf_mode = CNF_INPUT_FLOATING;
				else
					cnf_mode = CNF_INPUT_PULL;
				break;

			case OPENEMC_PIN_DIRECTION_OUT:
				if (emc->pin[n].out_drive ==
				    OPENEMC_PIN_DRIVE_OPEN_DRAIN)
					cnf_mode = CNF_OUTPUT_OPEN_DRAIN;
				else
					cnf_mode = 0;

				switch (emc->pin[n].out_speed) {
				case OPENEMC_PIN_SPEED_2_MHZ:
					cnf_mode |= MODE_OUTPUT_2_MHZ;
					break;
				case OPENEMC_PIN_SPEED_10_MHZ:
					cnf_mode |= MODE_OUTPUT_10_MHZ;
					break;
				case OPENEMC_PIN_SPEED_50_MHZ:
					cnf_mode |= MODE_OUTPUT_50_MHZ;
					break;
				}

				break;
			}
			break;

		case OPENEMC_PIN_MODE_ALTERNATE_OUT:
			cnf_mode = CNF_OUTPUT_ALTERNATE;

			if (emc->pin[n].out_drive ==
			    OPENEMC_PIN_DRIVE_OPEN_DRAIN)
				cnf_mode |= CNF_OUTPUT_OPEN_DRAIN;

			switch (emc->pin[n].out_speed) {
			case OPENEMC_PIN_SPEED_2_MHZ:
				cnf_mode |= MODE_OUTPUT_2_MHZ;
				break;
			case OPENEMC_PIN_SPEED_10_MHZ:
				cnf_mode |= MODE_OUTPUT_10_MHZ;
				break;
			case OPENEMC_PIN_SPEED_50_MHZ:
				cnf_mode |= MODE_OUTPUT_50_MHZ;
				break;
			}

			break;

		case OPENEMC_PIN_MODE_ANALOG:
			cnf_mode = 0;
			break;
		}

		cfgs[n * 4 / 8] |= cnf_mode << (n * 4 % 8);
	}

	return openemc_write_data(emc, OPENEMC_GPIO_CFG, emc->npin * 4 / 8,
				  cfgs);
}
EXPORT_SYMBOL_GPL(openemc_pins_write_cfg);

int openemc_pins_write_out(struct openemc *emc)
{
	u8 out[OPENEMC_MAX_PINS / 8] = { 0 };
	bool on;
	int n;

	for (n = 0; n < emc->npin; n++) {
		on = false;

		if (emc->pin[n].mode == OPENEMC_PIN_MODE_GPIO) {
			switch (emc->pin[n].direction) {
			case OPENEMC_PIN_DIRECTION_IN:
				on = (emc->pin[n].in_bias ==
				      OPENEMC_PIN_BIAS_PULL_UP);
				break;
			case OPENEMC_PIN_DIRECTION_OUT:
				on = emc->pin[n].out_level;
				break;
			}
		}

		if (on)
			out[n / 8] |= BIT(n % 8);
	}

	return openemc_write_data(emc, OPENEMC_GPIO_OUT, emc->npin / 8, out);
}
EXPORT_SYMBOL_GPL(openemc_pins_write_out);

void openemc_irq_do_sync(struct openemc *emc)
{
	u32 rising_edge = 0, falling_edge = 0;
	u16 high_level = 0, low_level = 0;
	u64 exti_bank = 0;
	int hwirq, exti;

	dev_dbg(emc->dev, "Syncing IRQ state\n");

	emc->irq_mask |= BIT(OPENEMC_IRQ_BOARD_IO_READ) |
			 BIT(OPENEMC_IRQ_BOARD_IO_WRITE) |
			 BIT(OPENEMC_IRQ_BOARD_IOCTL);

	for (exti = 0; exti < OPENEMC_EXT_IRQS; exti++) {
		if (emc->exti_bank[exti] < 0)
			continue;

		exti_bank |= ((u64)emc->exti_bank[exti] & 0b1111) << (exti * 4);
	}

	for (hwirq = 0; hwirq < OPENEMC_IRQS; hwirq++) {
		switch (emc->irq_types[hwirq]) {
		case IRQ_TYPE_EDGE_RISING:
			rising_edge |= BIT(hwirq);
			break;
		case IRQ_TYPE_EDGE_FALLING:
			falling_edge |= BIT(hwirq);
			break;
		case IRQ_TYPE_EDGE_BOTH:
			rising_edge |= BIT(hwirq);
			falling_edge |= BIT(hwirq);
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			if (hwirq < OPENEMC_EXT_IRQS)
				high_level |= BIT(hwirq);
			break;
		case IRQ_TYPE_LEVEL_LOW:
			if (hwirq < OPENEMC_EXT_IRQS)
				low_level |= BIT(hwirq);
			break;
		}
	}

	openemc_write_u64(emc, OPENEMC_IRQ_EXTI_GPIO_SRC, exti_bank);
	openemc_write_u32(emc, OPENEMC_IRQ_EXTI_TRIGGER_RISING_EDGE,
			  rising_edge);
	openemc_write_u32(emc, OPENEMC_IRQ_EXTI_TRIGGER_FALLING_EDGE,
			  falling_edge);
	openemc_write_u16(emc, OPENEMC_IRQ_EXTI_TRIGGER_HIGH_LEVEL, high_level);
	openemc_write_u16(emc, OPENEMC_IRQ_EXTI_TRIGGER_LOW_LEVEL, low_level);
	openemc_write_u32(emc, OPENEMC_IRQ_MASK, emc->irq_mask);

	openemc_cmd(emc, OPENEMC_IRQ_EXTI_DO_TRIGGER_LEVEL);
}
EXPORT_SYMBOL_GPL(openemc_irq_do_sync);

void openemc_irq_do_mask(struct openemc *emc, unsigned long hwirq)
{
	dev_dbg(emc->dev, "Masking IRQ %ld\n", hwirq);
	emc->irq_mask &= ~(BIT(hwirq));
}
EXPORT_SYMBOL_GPL(openemc_irq_do_mask);

void openemc_irq_do_unmask(struct openemc *emc, unsigned long hwirq)
{
	dev_dbg(emc->dev, "Unmasking IRQ %ld\n", hwirq);
	emc->irq_mask |= BIT(hwirq);
}
EXPORT_SYMBOL_GPL(openemc_irq_do_unmask);

int openemc_irq_check_type(unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_BOTH:
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		return 0;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(openemc_irq_check_type);

int openemc_irq_do_set_type(struct openemc *emc, unsigned long hwirq,
			    unsigned int type)
{
	int ret;

	dev_dbg(emc->dev, "Setting IRQ %ld type to %d\n", hwirq, type);

	ret = openemc_irq_check_type(type);
	if (ret < 0)
		return ret;

	emc->irq_types[hwirq] = type;

	return 0;
}
EXPORT_SYMBOL_GPL(openemc_irq_do_set_type);

static void openemc_irq_bus_lock(struct irq_data *data)
{
	struct openemc *emc = irq_data_get_irq_chip_data(data);

	mutex_lock(&emc->irq_lock);
}

static void openemc_irq_bus_sync_unlock(struct irq_data *data)
{
	struct openemc *emc = irq_data_get_irq_chip_data(data);

	openemc_irq_do_sync(emc);
	mutex_unlock(&emc->irq_lock);
}

static void openemc_irq_mask(struct irq_data *data)
{
	struct openemc *emc = irq_data_get_irq_chip_data(data);

	openemc_irq_do_mask(emc, data->hwirq);
}

static void openemc_irq_unmask(struct irq_data *data)
{
	struct openemc *emc = irq_data_get_irq_chip_data(data);

	openemc_irq_do_unmask(emc, data->hwirq);
}

static int openemc_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct openemc *emc = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = data->hwirq;

	return openemc_irq_do_set_type(emc, hwirq, type);
}

static struct irq_chip openemc_irq_chip = {
	.name = "openemc",
	.irq_mask = openemc_irq_mask,
	.irq_unmask = openemc_irq_unmask,
	.irq_set_type = openemc_irq_set_type,
	.irq_bus_lock = openemc_irq_bus_lock,
	.irq_bus_sync_unlock = openemc_irq_bus_sync_unlock,
};

static irqreturn_t openemc_irq_handler(int irq, void *ptr)
{
	struct openemc *emc = ptr;
	u32 pending;
	bool handled = false;
	int ret, n;

	ret = openemc_read_u32(emc, OPENEMC_IRQ_PENDING, &pending);
	if (ret < 0)
		return IRQ_NONE;

	if (pending & BIT(OPENEMC_IRQ_LOG)) {
		sysfs_notify(&emc->dev->kobj, NULL, "openemc_log");

		emc->log_outstanding_irq++;
		if (emc->log_outstanding_irq > 100) {
			dev_dbg(emc->dev, "disabling log interrupt\n");
			emc->irq_mask &= ~BIT(OPENEMC_IRQ_LOG);
			openemc_write_u32(emc, OPENEMC_IRQ_MASK, emc->irq_mask);
		}

		pending &= ~BIT(OPENEMC_IRQ_LOG);
		handled = true;
	}

	if (pending & BIT(OPENEMC_IRQ_BOARD_IO_READ)) {
		mutex_lock(&emc->io_lock);
		emc->io_read_avail = true;
		mutex_unlock(&emc->io_lock);
		wake_up_all(&emc->io_wait_queue);

		pending &= ~BIT(OPENEMC_IRQ_BOARD_IO_READ);
		handled = true;
	}

	if (pending & BIT(OPENEMC_IRQ_BOARD_IO_WRITE)) {
		mutex_lock(&emc->io_lock);
		emc->io_write_avail = true;
		mutex_unlock(&emc->io_lock);
		wake_up_all(&emc->io_wait_queue);

		pending &= ~BIT(OPENEMC_IRQ_BOARD_IO_WRITE);
		handled = true;
	}

	if (pending & BIT(OPENEMC_IRQ_BOARD_IOCTL)) {
		mutex_lock(&emc->io_lock);
		emc->ioctl_avail = true;
		mutex_unlock(&emc->io_lock);
		wake_up_all(&emc->io_wait_queue);

		pending &= ~BIT(OPENEMC_IRQ_BOARD_IOCTL);
		handled = true;
	}

	for (n = 0; n < OPENEMC_IRQS; n++) {
		if (pending & BIT(n)) {
			irq = irq_find_mapping(emc->irq_domain, n);
			if (irq) {
				dev_dbg(emc->dev, "handling HWIRQ %d\n", n);
				handle_nested_irq(irq);
				handled = true;
			} else {
				dev_warn(emc->dev, "HWIRQ %d not mapped\n", n);
			}
		}
	}

	openemc_cmd(emc, OPENEMC_IRQ_EXTI_DO_TRIGGER_LEVEL);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int openemc_irq_translate(struct irq_domain *d,
				 struct irq_fwspec *fwspec,
				 unsigned long *hwirq, unsigned int *type)
{
	if (fwspec->param_count != 2 || fwspec->param[0] >= OPENEMC_IRQS)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = fwspec->param[1];

	return 0;
}

static int openemc_irq_alloc(struct irq_domain *d, unsigned int virq,
			     unsigned int nr_irqs, void *data)
{
	struct openemc *emc = d->host_data;
	struct irq_fwspec *fwspec = data;
	irq_hw_number_t hwirq = fwspec->param[0];

	dev_dbg(emc->dev, "Allocating IRQ %ld\n", hwirq);

	if (emc->irq_alloced & BIT(hwirq)) {
		dev_err(emc->dev, "IRQ %ld already allocated\n", hwirq);
		return -EBUSY;
	}

	emc->irq_alloced |= BIT(hwirq);

	irq_domain_set_hwirq_and_chip(d, virq, hwirq, &openemc_irq_chip, emc);

	return 0;
}

static void openemc_irq_free(struct irq_domain *d, unsigned int virq,
			     unsigned int nr_irqs)
{
	struct openemc *emc = d->host_data;
	struct irq_data *irq_data = irq_domain_get_irq_data(d, virq);
	irq_hw_number_t hwirq = irq_data->hwirq;

	dev_dbg(emc->dev, "Freeing IRQ %ld\n", hwirq);

	irq_domain_free_irqs_common(d, virq, nr_irqs);

	emc->irq_alloced &= ~BIT(hwirq);
}

static struct irq_domain_ops openemc_irq_domain_ops = {
	.translate = openemc_irq_translate,
	.alloc = openemc_irq_alloc,
	.free = openemc_irq_free,
};

static ssize_t openemc_firmware_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%s", emc->firmware);
}
static DEVICE_ATTR_RO(openemc_firmware);

static ssize_t openemc_version_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%s", emc->version);
}
static DEVICE_ATTR_RO(openemc_version);

static ssize_t openemc_bootloader_version_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%s", emc->bootloader_version);
}
static DEVICE_ATTR_RO(openemc_bootloader_version);

static ssize_t openemc_bootloader_crc32_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%08x", emc->bootloader_crc32);
}
static DEVICE_ATTR_RO(openemc_bootloader_crc32);

static ssize_t openemc_copyright_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%s", emc->copyright);
}
static DEVICE_ATTR_RO(openemc_copyright);

static ssize_t openemc_flashable_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);

	if (emc->id == OPENEMC_ID_FIRMWARE_WITH_BOOTLOADER)
		return sprintf(buf, "yes");
	else
		return sprintf(buf, "no");
}
static DEVICE_ATTR_RO(openemc_flashable);

static ssize_t openemc_flash_size_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%d", emc->flash_total_size);
}
static DEVICE_ATTR_RO(openemc_flash_size);

static ssize_t openemc_emc_model_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%02x", emc->emc_model);
}
static DEVICE_ATTR_RO(openemc_emc_model);

static ssize_t openemc_board_model_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%s", emc->board_model);
}
static DEVICE_ATTR_RO(openemc_board_model);

static ssize_t openemc_unique_id_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%08llx%016llx", emc->unique_id[1],
		       emc->unique_id[0]);
}
static DEVICE_ATTR_RO(openemc_unique_id);

static ssize_t openemc_boot_reason_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%04x", emc->boot_reason);
}
static DEVICE_ATTR_RO(openemc_boot_reason);

static ssize_t openemc_reset_status_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);

	if (emc->reset_status == OPENEMC_RESET_EXTERNAL)
		return sprintf(buf, "external");
	else if (emc->reset_status & OPENEMC_RESET_WAKEUP)
		return sprintf(buf, "wakeup");
	else if (emc->reset_status & OPENEMC_RESET_POWER_ON)
		return sprintf(buf, "power-on");
	else if (emc->reset_status & OPENEMC_RESET_FIRMWARE)
		return sprintf(buf, "firmware");
	else if (emc->reset_status & OPENEMC_RESET_WATCHDOG)
		return sprintf(buf, "watchdog");
	else if (emc->reset_status & OPENEMC_RESET_WINDOW_WATCHDOG)
		return sprintf(buf, "window-watchdog");
	else if (emc->reset_status & OPENEMC_RESET_LOW_POWER)
		return sprintf(buf, "low-power");
	else
		return sprintf(buf, "%x", emc->reset_status);
}
static DEVICE_ATTR_RO(openemc_reset_status);

static ssize_t openemc_start_reason_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%02x", emc->start_reason);
}
static DEVICE_ATTR_RO(openemc_start_reason);

static ssize_t openemc_program_id_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	return sprintf(buf, "%08x", emc->program_id);
}
static DEVICE_ATTR_RO(openemc_program_id);

static ssize_t openemc_cells_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i;

	for (i = 0; i < emc->n_cells; i++) {
		len += sprintf(buf + len, "%s\n", emc->cells[i].of_compatible);
	}

	return len;
}
static DEVICE_ATTR_RO(openemc_cells);

static ssize_t openemc_echo_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct openemc *emc = dev_get_drvdata(dev);
	int ret;

	ret = openemc_read_data(emc, OPENEMC_ECHO, OPENEMC_MAX_DATA_SIZE, buf);
	if (ret < 0)
		return 0;

	return OPENEMC_MAX_DATA_SIZE;
}
static ssize_t openemc_echo_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct openemc *emc = dev_get_drvdata(dev);
	int ret;

	if (count > OPENEMC_MAX_DATA_SIZE)
		count = OPENEMC_MAX_DATA_SIZE;

	ret = openemc_write_data(emc, OPENEMC_ECHO, count, buf);
	if (ret < 0)
		return 0;

	return count;
}
static DEVICE_ATTR_RW(openemc_echo);

static ssize_t openemc_log_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr, char *buf,
				loff_t off, size_t count)
{
	struct openemc *emc = dev_get_drvdata(kobj_to_dev(kobj));
	int ret;

	if (count < OPENEMC_LOG_LEN)
		return -EINVAL;

	emc->log_outstanding_irq = 0;
	if (!(emc->irq_mask & BIT(OPENEMC_IRQ_LOG))) {
		dev_dbg(emc->dev, "enabling log interrupt\n");
		emc->irq_mask |= BIT(OPENEMC_IRQ_LOG);
		openemc_write_u32(emc, OPENEMC_IRQ_MASK, emc->irq_mask);
	}

	if (emc->log_data[0] == 0) {
		ret = openemc_read_data(emc, OPENEMC_LOG_READ,
					OPENEMC_MAX_DATA_SIZE, emc->log_data);
		if (ret < 0)
			return ret;
	}

	if (emc->log_data[0] & OPENEMC_LOG_LOST) {
		emc->log_data[0] &= ~OPENEMC_LOG_LOST;
		return -EPIPE;
	}

	count = emc->log_data[0] & OPENEMC_LOG_LEN;
	memcpy(buf, emc->log_data + 1, count);

	emc->log_data[0] = 0;

	return count;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static BIN_ATTR_ADMIN_RO(openemc_log, S32_MAX - 1);
#else
static BIN_ATTR_RO(openemc_log, S32_MAX - 1);
#endif

static ssize_t openemc_bootloader_log_read(struct file *filp,
					   struct kobject *kobj,
					   struct bin_attribute *bin_attr,
					   char *buf, loff_t off, size_t count)
{
	struct openemc *emc = dev_get_drvdata(kobj_to_dev(kobj));
	u8 tmp[OPENEMC_MAX_DATA_SIZE];
	int ret;

	if (count < OPENEMC_LOG_LEN)
		return -EINVAL;

	ret = openemc_read_data(emc, OPENEMC_BOOTLOADER_LOG_READ,
				OPENEMC_MAX_DATA_SIZE, tmp);
	if (ret < 0)
		return ret;

	count = tmp[0] & OPENEMC_LOG_LEN;
	memcpy(buf, tmp + 1, count);

	return count;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static BIN_ATTR_ADMIN_RO(openemc_bootloader_log, 65536);
#else
static BIN_ATTR_RO(openemc_bootloader_log, 65536);
#endif

static struct attribute *openemc_attrs[] = {
	&dev_attr_openemc_firmware.attr,
	&dev_attr_openemc_version.attr,
	&dev_attr_openemc_bootloader_version.attr,
	&dev_attr_openemc_bootloader_crc32.attr,
	&dev_attr_openemc_copyright.attr,
	&dev_attr_openemc_flashable.attr,
	&dev_attr_openemc_flash_size.attr,
	&dev_attr_openemc_emc_model.attr,
	&dev_attr_openemc_board_model.attr,
	&dev_attr_openemc_unique_id.attr,
	&dev_attr_openemc_boot_reason.attr,
	&dev_attr_openemc_reset_status.attr,
	&dev_attr_openemc_start_reason.attr,
	&dev_attr_openemc_program_id.attr,
	&dev_attr_openemc_cells.attr,
	&dev_attr_openemc_echo.attr,
	NULL
};

static struct bin_attribute *openemc_bin_attrs[] = {
	&bin_attr_openemc_log, &bin_attr_openemc_bootloader_log, NULL
};

static const struct attribute_group openemc_group = {
	.attrs = openemc_attrs,
	.bin_attrs = openemc_bin_attrs
};

static inline struct openemc *file_to_openemc(struct file *file)
{
	return container_of(file->private_data, struct openemc, io_dev);
}

static ssize_t openemc_fops_read(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct openemc *emc = file_to_openemc(file);
	u8 data[OPENEMC_MAX_DATA_SIZE];
	size_t avail;
	ssize_t ret;

	if (count < OPENEMC_MAX_DATA_SIZE - 1)
		return -EINVAL;

	while (true) {
		mutex_lock(&emc->io_lock);
		emc->io_read_avail = false;
		mutex_unlock(&emc->io_lock);

		ret = openemc_read_data(emc, OPENEMC_BOARD_IO,
					OPENEMC_MAX_DATA_SIZE, data);
		if (ret < 0)
			return ret;

		avail = min_t(size_t, data[0], OPENEMC_MAX_DATA_SIZE - 1);
		if (avail > 0)
			break;

		mutex_lock(&emc->io_lock);
		while (!emc->io_read_avail) {
			mutex_unlock(&emc->io_lock);
			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;
			ret = wait_event_interruptible(emc->io_wait_queue,
						       emc->io_read_avail);
			if (ret)
				return ret;
			mutex_lock(&emc->io_lock);
		}
		mutex_unlock(&emc->io_lock);
	}

	if (copy_to_user(buf, data + 1, avail))
		return -EFAULT;

	return avail;
}

static ssize_t openemc_fops_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct openemc *emc = file_to_openemc(file);
	u8 data[OPENEMC_MAX_DATA_SIZE];
	u8 len = min_t(size_t, OPENEMC_MAX_DATA_SIZE, count);
	u8 status;
	ssize_t ret;

	if (copy_from_user(data, buf, len))
		return -EFAULT;

	while (true) {
		mutex_lock(&emc->io_lock);
		emc->io_write_avail = false;
		ret = openemc_write_data(emc, OPENEMC_BOARD_IO, len, data);
		if (ret < 0) {
			mutex_unlock(&emc->io_lock);
			return ret;
		}
		ret = openemc_read_u8(emc, OPENEMC_BOARD_IO_STATUS, &status);
		if (ret < 0) {
			mutex_unlock(&emc->io_lock);
			return ret;
		}
		mutex_unlock(&emc->io_lock);

		if (status == 0)
			break;

		mutex_lock(&emc->io_lock);
		while (!emc->io_write_avail) {
			mutex_unlock(&emc->io_lock);
			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;
			ret = wait_event_interruptible(emc->io_wait_queue,
						       emc->io_write_avail);
			if (ret)
				return ret;
			mutex_lock(&emc->io_lock);
		}
		mutex_unlock(&emc->io_lock);
	}

	return len;
}

static int openemc_wait_ioctl_avail(struct openemc *emc, u8 *info)
{
	u8 buf[2];
	int ret;

	while (true) {
		mutex_lock(&emc->io_lock);
		emc->ioctl_avail = false;
		mutex_unlock(&emc->io_lock);

		ret = openemc_read_data(emc, OPENEMC_BOARD_IOCTL_STATUS,
					sizeof(buf), buf);
		if (ret < 0)
			return ret;

		if (buf[0] != OPENEMC_BOARD_IOCTL_PROCESSING)
			break;

		mutex_lock(&emc->io_lock);
		while (!emc->ioctl_avail) {
			mutex_unlock(&emc->io_lock);
			ret = wait_event_interruptible(emc->io_wait_queue,
						       emc->ioctl_avail);
			if (ret)
				return ret;
			mutex_lock(&emc->io_lock);
		}
		mutex_unlock(&emc->io_lock);
	}

	*info = buf[1];
	return buf[0];
}

static long openemc_fops_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	struct openemc *emc = file_to_openemc(file);
	__user u8 *buf = (__user u8 *)arg;
	u8 len = cmd;
	u8 data[OPENEMC_MAX_DATA_SIZE];
	u8 info;
	long ret;

	if (cmd > OPENEMC_MAX_DATA_SIZE)
		return -EINVAL;

	if (copy_from_user(data, buf, len))
		return -EFAULT;

	mutex_lock(&emc->ioctl_lock);

	ret = openemc_wait_ioctl_avail(emc, &info);
	if (ret < 0)
		goto out;

	ret = openemc_write_data(emc, OPENEMC_BOARD_IOCTL, len, data);
	if (ret < 0)
		goto out;

	ret = openemc_wait_ioctl_avail(emc, &info);
	if (ret < 0)
		goto out;
	if (ret == OPENEMC_BOARD_IOCTL_FAILED) {
		if (info == 0)
			ret = -EINVAL;
		else
			ret = -info;
		goto out;
	} else if (ret != OPENEMC_BOARD_IOCTL_DONE) {
		ret = -EIO;
		goto out;
	}

	len = min_t(u8, info, OPENEMC_MAX_DATA_SIZE);
	ret = openemc_read_data(emc, OPENEMC_BOARD_IOCTL, len, data);
	if (ret < 0)
		goto out;

	if (copy_to_user(buf, data, len)) {
		ret = -EFAULT;
		goto out;
	}

	ret = len;

out:
	mutex_unlock(&emc->ioctl_lock);
	return ret;
}

static __poll_t openemc_fops_poll(struct file *file, poll_table *wait)
{
	struct openemc *emc = file_to_openemc(file);
	__poll_t mask = 0;

	poll_wait(file, &emc->io_wait_queue, wait);

	mutex_lock(&emc->io_lock);
	if (emc->io_read_avail)
		mask |= EPOLLIN;
	if (emc->io_write_avail)
		mask |= EPOLLOUT;
	mutex_unlock(&emc->io_lock);

	return mask;
}

static const struct file_operations openemc_fops = {
	.owner = THIS_MODULE,
	.read = openemc_fops_read,
	.write = openemc_fops_write,
	.unlocked_ioctl = openemc_fops_ioctl,
	.poll = openemc_fops_poll,
};

static int openemc_get_mfd_cells(struct openemc *emc)
{
	char *comma;
	int ret, i;

	dev_dbg(emc->dev, "Detected cells:");

	i = 0;
	while (true) {
		emc->cells[i].of_compatible = devm_kzalloc(
			emc->dev, OPENEMC_MAX_DATA_SIZE + 1, GFP_KERNEL);

		ret = openemc_write_u8(emc, OPENEMC_MFD_CELL, i);
		if (ret < 0)
			return ret;

		ret = openemc_read_data(emc, OPENEMC_MFD_CELL,
					OPENEMC_MAX_DATA_SIZE,
					(char *)emc->cells[i].of_compatible);
		if (ret < 0)
			return ret;
		if (!strlen(emc->cells[i].of_compatible))
			break;

		comma = strchr(emc->cells[i].of_compatible, ',');
		if (comma)
			emc->cells[i].name = comma + 1;

		dev_dbg(emc->dev, " %s", emc->cells[i].of_compatible);

		i++;
	}

	emc->n_cells = i;

	dev_dbg(emc->dev, "\n");

	return 0;
}

static int openemc_get_info(struct openemc *emc)
{
	int i, ret;

	ret = openemc_read_u8(emc, OPENEMC_ID, &emc->id);
	if (ret < 0)
		return ret;

	ret = openemc_read_data(emc, OPENEMC_VERSION, OPENEMC_MAX_DATA_SIZE,
				emc->version);
	if (ret < 0)
		return ret;

	ret = openemc_read_data(emc, OPENEMC_BOOTLOADER_VERSION,
				OPENEMC_MAX_DATA_SIZE, emc->bootloader_version);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(emc, OPENEMC_BOOTLOADER_CRC32,
			       &emc->bootloader_crc32);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(emc->copyright);
	     i += OPENEMC_MAX_DATA_SIZE) {
		ret = openemc_write_u8(emc, OPENEMC_COPYRIGHT, i);
		if (ret < 0)
			return ret;

		ret = openemc_read_data(emc, OPENEMC_COPYRIGHT,
					OPENEMC_MAX_DATA_SIZE,
					emc->copyright + i);
		if (ret < 0)
			return ret;
	}
	emc->copyright[ARRAY_SIZE(emc->copyright) - 1] = 0;

	ret = openemc_read_u32(emc, OPENEMC_FLASH_TOTAL_SIZE,
			       &emc->flash_total_size);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(emc, OPENEMC_EMC_MODEL, &emc->emc_model);
	if (ret < 0)
		return ret;

	ret = openemc_read_data(emc, OPENEMC_BOARD_MODEL, OPENEMC_MAX_DATA_SIZE,
				emc->board_model);
	if (ret < 0)
		return ret;

	ret = openemc_read_u128(emc, OPENEMC_UNIQUE_ID, &emc->unique_id[0],
				&emc->unique_id[1]);
	if (ret < 0)
		return ret;

	ret = openemc_read_u16(emc, OPENEMC_BOOT_REASON, &emc->boot_reason);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(emc, OPENEMC_RESET_STATUS, &emc->reset_status);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(emc, OPENEMC_START_REASON, &emc->start_reason);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(emc, OPENEMC_PROGRAM_ID, &emc->program_id);
	if (ret < 0)
		return ret;

	dev_info(emc->dev, "OpenEMC firmware version %s on board %s\n%s\n",
		 emc->version, emc->board_model, emc->copyright);

	return openemc_get_mfd_cells(emc);
}

static int openemc_irq_setup(struct openemc *emc)
{
	u32 pending;
	unsigned int exti;
	int ret;

	if (!emc->irq) {
		dev_warn(emc->dev, "No interrupt specified, no interrupts\n");
		return -ENODEV;
	}

	mutex_init(&emc->irq_lock);

	for (exti = 0; exti < OPENEMC_EXT_IRQS; exti++)
		emc->exti_bank[exti] = -1;

	ret = openemc_write_u32(emc, OPENEMC_IRQ_MASK, 0);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(emc, OPENEMC_IRQ_PENDING, &pending);
	if (ret < 0)
		return ret;

	emc->irq_domain = irq_domain_add_linear(emc->dev->of_node, OPENEMC_IRQS,
						&openemc_irq_domain_ops, emc);
	if (!emc->irq_domain) {
		dev_err(emc->dev, "Cannot create IRQ domain\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(emc->dev, emc->irq, NULL,
					openemc_irq_handler, IRQF_ONESHOT,
					dev_name(emc->dev), emc);
	if (ret < 0) {
		dev_err(emc->dev, "Failed to request IRQ %d\n", emc->irq);
		return ret;
	}

	return 0;
}

static int openemc_i2c_probe(struct i2c_client *i2c)
{
	const struct of_device_id *of_id;
	const char *firmware = NULL;
	struct openemc *emc;
	int ret, retry;

	of_id = of_match_device(openemc_of_match, &i2c->dev);
	if (!of_id) {
		dev_err(&i2c->dev, "Failed to find matching DT ID\n");
		return -EINVAL;
	}

	emc = devm_kzalloc(&i2c->dev, sizeof(*emc), GFP_KERNEL);
	if (!emc)
		return -ENOMEM;

	i2c_set_clientdata(i2c, emc);

	mutex_init(&emc->lock);
	mutex_init(&emc->req_lock);
	emc->i2c = i2c;
	emc->irq = i2c->irq;
	emc->dev = &i2c->dev;

	of_property_read_string(i2c->dev.of_node, "firmware", &firmware);

	for (retry = 0; retry < OPENEMC_MAX_RETRIES; retry++) {
		ret = openemc_start_firmware(emc, firmware);
		if (ret == 0)
			break;

		dev_warn(&i2c->dev, "retrying start\n");
		msleep(1000);
	}
	if (ret != 0)
		return ret;

	ret = openemc_get_info(emc);
	if (ret < 0)
		return ret;

	emc->wdt_pet_code = get_random_u32();
	ret = openemc_write_u32(emc, OPENEMC_WDG_PET_CODE, emc->wdt_pet_code);
	if (ret < 0)
		return ret;

	ret = openemc_pins_read(emc);
	if (ret < 0)
		return ret;

	openemc_irq_setup(emc);

	ret = devm_mfd_add_devices(&i2c->dev, PLATFORM_DEVID_NONE, emc->cells,
				   emc->n_cells, NULL, 0, NULL);
	if (ret < 0) {
		dev_err(emc->dev, "failed to add sub-devices: %d\n", ret);
		return ret;
	}

	ret = devm_device_add_group(&i2c->dev, &openemc_group);
	if (ret < 0)
		return ret;

	/* Init board io interface */
	mutex_init(&emc->io_lock);
	mutex_init(&emc->ioctl_lock);
	init_waitqueue_head(&emc->io_wait_queue);
	emc->io_dev.minor = MISC_DYNAMIC_MINOR;
	emc->io_dev.name = "openemc";
	emc->io_dev.fops = &openemc_fops;
	emc->io_dev.parent = &i2c->dev;
	ret = misc_register(&emc->io_dev);
	if (ret < 0)
		return ret;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void openemc_i2c_remove(struct i2c_client *i2c)
#else
static int openemc_i2c_remove(struct i2c_client *i2c)
#endif
{
	struct openemc *emc = i2c_get_clientdata(i2c);

	if (emc->irq_domain) {
		devm_free_irq(emc->dev, emc->irq, emc);
		irq_domain_remove(emc->irq_domain);
	}

	misc_deregister(&emc->io_dev);

#if !(LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}

static struct i2c_driver openemc_i2c_driver = {
	.driver = {
		.name = "openemc",
		.of_match_table = of_match_ptr(openemc_of_match),
	},
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
	.probe = openemc_i2c_probe,
#else
	.probe_new = openemc_i2c_probe,
#endif
	.remove = openemc_i2c_remove,
};
module_i2c_driver(openemc_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC");
MODULE_VERSION("0.1");
