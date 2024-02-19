// SPDX-License-Identifier: GPL-2.0+
/*
 * Watchdog driver for OpenEMC
 *
 * Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/watchdog.h>
#include <linux/delay.h>

#include "openemc.h"

static const struct of_device_id openemc_wdt_of_match[] = {
	{ .compatible = "openemc,openemc_wdt" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_wdt_of_match);

struct openemc_wdt {
	struct device *dev;
	struct openemc *emc;

	struct watchdog_device wdd;
};

static int openemc_wdt_unlock(struct openemc_wdt *wdt)
{
	u8 unlocked;
	int ret;

	ret = openemc_write_u64(wdt->emc, OPENEMC_WDG_UNLOCK,
				OPENEMC_WDT_UNLOCK_CODE);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(wdt->emc, OPENEMC_WDG_UNLOCK, &unlocked);
	if (ret < 0)
		return ret;

	if (!unlocked) {
		dev_err(wdt->dev, "unlocking watchdog failed\n");
		return -EFAULT;
	}

	return 0;
}

static int openemc_wdt_lock(struct openemc_wdt *wdt)
{
	u8 unlocked;
	int ret;

	ret = openemc_write_u8(wdt->emc, OPENEMC_WDG_UNLOCK, 0);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(wdt->emc, OPENEMC_WDG_UNLOCK, &unlocked);
	if (ret < 0)
		return ret;

	if (unlocked) {
		dev_err(wdt->dev, "locking watchdog failed\n");
		return -EFAULT;
	}

	return 0;
}

static int openemc_wdt_start(struct watchdog_device *wdd)
{
	struct openemc_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	ret = openemc_wdt_unlock(wdt);
	if (ret < 0)
		return ret;

	ret = openemc_write_u8(wdt->emc, OPENEMC_WDG_ACTIVE, 1);
	if (ret < 0)
		return ret;

	ret = openemc_wdt_lock(wdt);
	if (ret < 0)
		return ret;

	return 0;
}

static int openemc_wdt_stop(struct watchdog_device *wdd)
{
	struct openemc_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	ret = openemc_wdt_unlock(wdt);
	if (ret < 0)
		return ret;

	ret = openemc_write_u8(wdt->emc, OPENEMC_WDG_ACTIVE, 0);
	if (ret < 0)
		return ret;

	ret = openemc_wdt_lock(wdt);
	if (ret < 0)
		return ret;

	return 0;
}

static int openemc_wdt_ping(struct watchdog_device *wdd)
{
	struct openemc_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	ret = openemc_write_u32(wdt->emc, OPENEMC_WDG_PET,
				wdt->emc->wdt_pet_code);
	if (ret < 0) {
		dev_crit(wdt->dev, "pinging watchdog failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int openemc_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	struct openemc_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	ret = openemc_wdt_unlock(wdt);
	if (ret < 0)
		return ret;

	ret = openemc_write_u32(wdt->emc, OPENEMC_WDG_INTERVAL, timeout * 1000);
	if (ret < 0)
		return ret;

	wdd->timeout = timeout;

	ret = openemc_wdt_lock(wdt);
	if (ret < 0)
		return ret;

	return openemc_wdt_start(wdd);
}

static int openemc_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	struct openemc_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	dev_crit(wdt->dev, "restarting system using watchdog\n");

	ret = openemc_wdt_set_timeout(wdd, 0);
	if (ret < 0)
		return ret;

	mdelay(5000);

	dev_crit(wdt->dev, "watchdog restart failed\n");

	return -EFAULT;
}

static const struct watchdog_ops openemc_wdt_ops = {
	.owner = THIS_MODULE,
	.start = openemc_wdt_start,
	.stop = openemc_wdt_stop,
	.ping = openemc_wdt_ping,
	.set_timeout = openemc_wdt_set_timeout,
	.restart = openemc_wdt_restart,
};

static const struct watchdog_info openemc_wdt_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT,
	.identity = "openemc_wdt",
};

static int openemc_wdt_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_wdt *wdt;
	u32 timeout;
	u8 active, reset_status;
	int ret;

	match = of_match_node(openemc_wdt_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->dev = &pdev->dev;
	wdt->emc = emc;
	wdt->wdd.info = &openemc_wdt_info;
	wdt->wdd.ops = &openemc_wdt_ops;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = 6000;
	wdt->wdd.parent = wdt->dev;

	ret = openemc_read_u8(wdt->emc, OPENEMC_WDG_ACTIVE, &active);
	if (ret < 0)
		return ret;
	if (active) {
		dev_info(wdt->dev, "watchdog enabled pre-boot\n");
		wdt->wdd.status |= BIT(WDOG_HW_RUNNING);
	}

	ret = openemc_read_u32(wdt->emc, OPENEMC_WDG_INTERVAL, &timeout);
	if (ret < 0)
		return ret;

	ret = watchdog_init_timeout(&wdt->wdd, timeout / 1000, wdt->dev);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(wdt->emc, OPENEMC_RESET_STATUS, &reset_status);
	if (ret < 0)
		return ret;
	if (reset_status & OPENEMC_RESET_POWER_ON ||
	    reset_status & OPENEMC_RESET_LOW_POWER) {
		dev_info(wdt->dev, "boot was caused by power loss\n");
		wdt->wdd.bootstatus |= WDIOF_POWERUNDER;
	} else if (reset_status & OPENEMC_RESET_WATCHDOG) {
		dev_info(wdt->dev, "boot was caused by watchdog reset\n");
		wdt->wdd.bootstatus |= WDIOF_CARDRESET;
	} else if (reset_status == OPENEMC_RESET_EXTERNAL) {
		dev_info(wdt->dev, "boot was caused by external reset\n");
		wdt->wdd.bootstatus |= WDIOF_EXTERN1;
	}

	platform_set_drvdata(pdev, wdt);
	watchdog_set_drvdata(&wdt->wdd, wdt);

	ret = devm_watchdog_register_device(wdt->dev, &wdt->wdd);
	if (ret < 0) {
		dev_err(wdt->dev, "cannot register watchdog: %d\n", ret);
		return ret;
	}

	dev_info(wdt->dev, "OpenEMC watchdog registered");

	return 0;
}

static struct platform_driver openemc_wdt_driver = {
	.driver = {
		.name = "openemc_wdt",
		.of_match_table = of_match_ptr(openemc_wdt_of_match),
	},
	.probe = openemc_wdt_probe,
};
module_platform_driver(openemc_wdt_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC Watchdog");
MODULE_VERSION("0.1");
