// SPDX-License-Identifier: GPL-2.0+
/*
 * Platform store driver for OpenEMC
 *
 * Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/pstore_blk.h>
#include <linux/delay.h>

#include "openemc.h"

static const struct of_device_id openemc_pstore_of_match[] = {
	{ .compatible = "openemc,openemc_pstore" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_pstore_of_match);

static struct openemc_pstore_context {
	struct openemc *emc;
	struct pstore_blk_config info;
	struct pstore_device_info dev;
} oops_cxt;

static ssize_t openemc_pstore_read(char *buf, size_t size, loff_t off)
{
	struct openemc *emc = oops_cxt.emc;
	u16 io_size = 0;
	size_t pos = 0;
	int ret;

	ret = openemc_write_u16(emc, OPENEMC_PSTORE_IO_ADDRESS, off);
	if (ret < 0)
		return ret;

	while (pos < size) {
		u16 new_io_size =
			min_t(size_t, OPENEMC_MAX_DATA_SIZE, size - pos);
		if (io_size != new_io_size) {
			ret = openemc_write_u16(emc, OPENEMC_PSTORE_IO_SIZE,
						new_io_size);
			if (ret < 0)
				return ret;

			io_size = new_io_size;
		}

		ret = openemc_read_data(emc, OPENEMC_PSTORE_IO, io_size,
					buf + pos);
		if (ret < 0)
			return ret;

		pos += io_size;
	}

	return size;
}

static ssize_t openemc_pstore_write(const char *buf, size_t size, loff_t off)
{
	struct openemc *emc = oops_cxt.emc;
	u16 io_size = 0;
	size_t pos = 0;
	int ret;

	ret = openemc_write_u16(emc, OPENEMC_PSTORE_IO_ADDRESS, off);
	if (ret < 0)
		return ret;

	while (pos < size) {
		u16 new_io_size =
			min_t(size_t, OPENEMC_MAX_DATA_SIZE, size - pos);
		if (io_size != new_io_size) {
			ret = openemc_write_u16(emc, OPENEMC_PSTORE_IO_SIZE,
						new_io_size);
			if (ret < 0)
				return ret;

			io_size = new_io_size;
		}

		ret = openemc_write_data(emc, OPENEMC_PSTORE_IO, io_size,
					 buf + pos);
		if (ret < 0)
			return ret;

		pos += io_size;
	}

	return size;
}

static ssize_t openemc_pstore_panic_write(const char *buf, size_t size,
					  loff_t off)
{
	struct openemc *emc = oops_cxt.emc;

	if (mutex_is_locked(&emc->req_lock)) {
		pr_emerg("Breaking OpenEMC lock for panic write\n");
		mutex_unlock(&emc->req_lock);
	}

	return openemc_pstore_write(buf, size, off);
}

static int openemc_pstore_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc_pstore_context *cxt = &oops_cxt;
	u16 total_size;
	int ret;

	match = of_match_node(openemc_pstore_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	cxt->emc = dev_get_drvdata(pdev->dev.parent);

	ret = pstore_blk_get_config(&cxt->info);
	if (ret)
		return ret;

	if (!cxt->info.kmsg_size) {
		pr_err("no backend enabled (kmsg_size is 0)\n");
		return -EINVAL;
	}

	ret = openemc_read_u16(cxt->emc, OPENEMC_PSTORE_SIZE, &total_size);
	if (ret < 0)
		return ret;

	if (total_size < 4096) {
		dev_err(&pdev->dev, "OpenEMC pstore buffer is too small");
		return -EINVAL;
	}

	cxt->dev.flags = PSTORE_FLAGS_DMESG;
	cxt->dev.zone.read = openemc_pstore_read;
	cxt->dev.zone.write = openemc_pstore_write;
	cxt->dev.zone.erase = NULL;
	cxt->dev.zone.panic_write = openemc_pstore_panic_write;
	cxt->dev.zone.total_size = total_size;

	ret = register_pstore_device(&cxt->dev);
	if (ret) {
		dev_err(&pdev->dev, "cannot register pstore device: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "OpenEMC pstore registered with %d bytes\n",
		 total_size);

	return 0;
}

static int openemc_pstore_remove(struct platform_device *pdev)
{
	struct openemc_pstore_context *cxt = &oops_cxt;

	unregister_pstore_device(&cxt->dev);
	cxt->emc = NULL;

	return 0;
}

static struct platform_driver openemc_pstore_driver = {
	.driver = {
		.name = "openemc_pstore",
		.of_match_table = of_match_ptr(openemc_pstore_of_match),
	},
	.probe = openemc_pstore_probe,
	.remove = openemc_pstore_remove,
};
module_platform_driver(openemc_pstore_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC Platform Store");
MODULE_VERSION("0.1");
