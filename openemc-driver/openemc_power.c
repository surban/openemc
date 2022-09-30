// SPDX-License-Identifier: GPL-2.0+
/*
 * Power control driver for OpenEMC
 *
 * Copyright (C) 2022 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/reboot.h>
#include <linux/delay.h>

#include "openemc.h"

static const struct of_device_id openemc_power_of_match[] = {
	{ .compatible = "openemc,openemc_power" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_power_of_match);

struct openemc_power {
	struct device *dev;
	struct openemc *emc;

	struct notifier_block *restart_handler;
	struct notifier_block *reboot_notifier;

	bool i2c_has_atomic;
};

static struct openemc_power *openemc_power_restart_data = NULL;

static int openemc_power_off_prepare(struct sys_off_data *data)
{
	struct openemc_power *power = data->cb_data;
	struct openemc *emc = power->emc;
	int ret;

	dev_crit(power->dev, "Requesting system power off in 1 second\n");

	ret = openemc_write_u16(emc, OPENEMC_POWER_OFF, 1000);
	if (ret < 0)
		dev_err(power->dev, "Requesting power off failed: %d\n", ret);

	return NOTIFY_DONE;
}

static int openemc_power_off(struct sys_off_data *data)
{
	struct openemc_power *power = data->cb_data;
	struct openemc *emc = power->emc;
	int ret;

	if (power->i2c_has_atomic) {
		dev_crit(power->dev, "Powering off system\n");

		ret = openemc_write_u16(emc, OPENEMC_POWER_OFF, 0);
		if (ret < 0)
			dev_err(power->dev, "Requesting power off failed: %d\n",
				ret);
	} else {
		dev_crit(power->dev, "Waiting for power off\n");
	}

	mdelay(1500);

	dev_crit(power->dev, "Power off failed\n");

	return NOTIFY_DONE;
}

static int openemc_power_restart_prepare(struct notifier_block *nb,
					 unsigned long action, void *data)
{
	struct openemc_power *power = openemc_power_restart_data;
	struct openemc *emc;
	int ret;

	if (!power || action != SYS_RESTART)
		return NOTIFY_DONE;

	emc = power->emc;

	dev_crit(power->dev, "Restarting system in 1 second\n");

	ret = openemc_write_u16(emc, OPENEMC_POWER_RESTART, 1000);
	if (ret < 0)
		dev_err(power->dev, "Requesting restart failed: %d\n", ret);

	return NOTIFY_DONE;
}

static int openemc_power_restart(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct openemc_power *power = openemc_power_restart_data;
	struct openemc *emc;
	int ret;

	if (!power)
		return NOTIFY_DONE;

	emc = power->emc;

	if (power->i2c_has_atomic) {
		dev_crit(power->dev, "Restarting system\n");

		ret = openemc_write_u16(emc, OPENEMC_POWER_RESTART, 0);
		if (ret < 0)
			dev_err(power->dev, "Requesting restart failed: %d\n",
				ret);
	} else {
		dev_crit(power->dev, "Waiting for restart\n");
	}

	mdelay(1500);

	dev_crit(power->dev, "Restart failed\n");

	return NOTIFY_DONE;
}

static int openemc_power_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_power *power;
	int ret;

	match = of_match_node(openemc_power_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	power = devm_kzalloc(&pdev->dev, sizeof(*power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

	power->dev = &pdev->dev;
	power->emc = emc;
	power->i2c_has_atomic = emc->i2c->adapter->algo->master_xfer_atomic ||
				emc->i2c->adapter->algo->smbus_xfer_atomic;

	platform_set_drvdata(pdev, power);

	if (of_device_is_system_power_controller(pdev->dev.of_node)) {
		openemc_power_restart_data = power;

		if (!power->i2c_has_atomic) {
			ret = devm_register_sys_off_handler(
				power->dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
				SYS_OFF_PRIO_DEFAULT, openemc_power_off_prepare,
				power);
			if (ret < 0) {
				dev_err(power->dev,
					"cannot register system off prepare "
					"handler: %d\n",
					ret);
				return ret;
			}

			power->reboot_notifier =
				devm_kzalloc(power->dev,
					     sizeof(*power->reboot_notifier),
					     GFP_KERNEL);
			if (!power->reboot_notifier)
				return -ENOMEM;
			power->reboot_notifier->notifier_call =
				&openemc_power_restart_prepare;
			power->reboot_notifier->priority = 255;

			ret = devm_register_reboot_notifier(
				power->dev, power->reboot_notifier);
			if (ret < 0) {
				dev_err(power->dev,
					"cannot register reboot notifier: %d\n",
					ret);
				return ret;
			}
		}

		ret = devm_register_sys_off_handler(power->dev,
						    SYS_OFF_MODE_POWER_OFF,
						    SYS_OFF_PRIO_DEFAULT,
						    openemc_power_off, power);
		if (ret < 0) {
			dev_err(power->dev,
				"cannot register system off handler: %d\n",
				ret);
			return ret;
		}

		power->restart_handler =
			devm_kzalloc(power->dev,
				     sizeof(*power->restart_handler),
				     GFP_KERNEL);
		if (!power->restart_handler)
			return -ENOMEM;
		power->restart_handler->notifier_call = &openemc_power_restart;
		power->restart_handler->priority = 255;

		ret = register_restart_handler(power->restart_handler);
		if (ret < 0) {
			dev_err(power->dev,
				"cannot register system restart handler: %d\n",
				ret);
			return ret;
		}

		dev_info(power->dev,
			 "OpenEMC handles system power off and restart");
	}

	dev_info(power->dev, "OpenEMC power control registered");

	return 0;
}

static int openemc_power_remove(struct platform_device *pdev)
{
	struct openemc_power *power = platform_get_drvdata(pdev);

	if (power->restart_handler)
		unregister_restart_handler(power->restart_handler);

	openemc_power_restart_data = NULL;

	return 0;
}

static struct platform_driver openemc_power_driver = {
	.driver = {
		.name = "openemc_power",
		.of_match_table = of_match_ptr(openemc_power_of_match),
	},
	.probe = openemc_power_probe,
	.remove = openemc_power_remove,
};
module_platform_driver(openemc_power_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC Power Control");
MODULE_VERSION("0.1");
