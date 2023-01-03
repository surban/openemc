// SPDX-License-Identifier: GPL-2.0+
/*
 * Battery driver for OpenEMC
 *
 * Copyright (C) 2022-2023 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/irq.h>

#include "openemc.h"

static const struct of_device_id openemc_battery_of_match[] = {
	{ .compatible = "openemc,openemc_battery" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_battery_of_match);

struct openemc_battery {
	struct device *dev;
	struct openemc *emc;

	struct power_supply *psy;
	unsigned int irq;

	u32 voltage;
	u32 min_voltage;
	u32 max_voltage;
	u8 charging;
	u32 constant_charge_voltage;
	u32 constant_charge_current;
	u32 supply_max_current;
	s32 battery_current;
	u32 system_voltage;
};

static int openemc_battery_update(struct openemc_battery *bat)
{
	int ret;

	ret = openemc_read_u32(bat->emc, OPENEMC_BATTERY_VOLTAGE,
			       &bat->voltage);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(bat->emc, OPENEMC_BATTERY_MIN_VOLTAGE,
			       &bat->min_voltage);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(bat->emc, OPENEMC_BATTERY_MAX_VOLTAGE,
			       &bat->max_voltage);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(bat->emc, OPENEMC_BATTERY_CHARGING,
			      &bat->charging);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(bat->emc,
			       OPENEMC_BATTERY_CONSTANT_CHARGE_VOLTAGE,
			       &bat->constant_charge_voltage);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(bat->emc,
			       OPENEMC_BATTERY_CONSTANT_CHARGE_CURRENT,
			       &bat->constant_charge_current);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(bat->emc, OPENEMC_SUPPLY_MAX_CURRENT,
			       &bat->supply_max_current);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(bat->emc, OPENEMC_BATTERY_CURRENT,
			       &bat->battery_current);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(bat->emc, OPENEMC_BATTERY_SYSTEM_VOLTAGE,
			       &bat->system_voltage);
	if (ret < 0)
		return ret;

	return 0;
}

int openemc_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct openemc_battery *bat = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (bat->charging == 0)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (bat->voltage < bat->max_voltage - 100)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (bat->charging) {
		case 0:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat->voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = bat->max_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat->min_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bat->battery_current * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = bat->supply_max_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = bat->constant_charge_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = bat->constant_charge_voltage * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t openemc_battery_irq_handler(int irq, void *ptr)
{
	struct openemc_battery *bat = ptr;
	int ret;

	ret = openemc_battery_update(bat);
	if (ret < 0) {
		dev_err(bat->dev, "update failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	power_supply_changed(bat->psy);

	return IRQ_HANDLED;
}

static const enum power_supply_property openemc_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static const struct power_supply_desc openemc_battery_desc = {
	.name = "openemc",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = openemc_battery_props,
	.num_properties = ARRAY_SIZE(openemc_battery_props),
	.get_property = openemc_battery_get_property,
};

static int openemc_battery_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_battery *bat;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg;
	struct irq_fwspec fwspec;
	int ret;

	match = of_match_node(openemc_battery_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	bat = devm_kzalloc(&pdev->dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = &pdev->dev;
	bat->emc = emc;

	dev_set_drvdata(&pdev->dev, bat);

	memset(&psy_cfg, 1, sizeof(psy_cfg));
	psy_cfg.of_node = pdev->dev.of_node;
	psy_cfg.drv_data = bat;

	psy_desc = devm_kzalloc(&pdev->dev, sizeof(*psy_desc), GFP_KERNEL);
	memcpy(psy_desc, &openemc_battery_desc, sizeof(*psy_desc));
	of_property_read_string(pdev->dev.of_node, "battery-name",
				&psy_desc->name);

	ret = openemc_battery_update(bat);
	if (ret < 0) {
		dev_err(bat->dev, "init failed: %d\n", ret);
		return ret;
	}

	bat->psy =
		devm_power_supply_register_no_ws(bat->dev, psy_desc, &psy_cfg);
	if (IS_ERR(bat->psy))
		return PTR_ERR(bat->psy);

	if (bat->emc->irq) {
		fwspec.fwnode = emc->dev->fwnode;
		fwspec.param_count = 2;
		fwspec.param[0] = OPENEMC_IRQ_BATTERY;
		fwspec.param[1] = IRQ_TYPE_NONE;

		bat->irq = irq_create_fwspec_mapping(&fwspec);
		if (!bat->irq) {
			dev_err(bat->dev, "failed to map battery IRQ\n");
			return -ENODEV;
		}

		ret = devm_request_threaded_irq(
			bat->dev, bat->irq, NULL, openemc_battery_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(bat->dev),
			bat);
		if (ret < 0) {
			dev_err(bat->dev, "failed to request irq %d\n",
				bat->irq);
			return ret;
		}
	}

	dev_info(bat->dev, "OpenEMC battery %s registered\n", psy_desc->name);

	return 0;
}

static int openemc_battery_remove(struct platform_device *pdev)
{
	struct openemc_battery *bat = platform_get_drvdata(pdev);

	if (bat->irq) {
		devm_free_irq(bat->dev, bat->irq, bat);
		irq_dispose_mapping(bat->irq);
	}

	return 0;
}

static struct platform_driver openemc_battery_driver = {
	.driver = {
		.name = "openemc_battery",
		.of_match_table = of_match_ptr(openemc_battery_of_match),
	},
	.probe = openemc_battery_probe,
	.remove = openemc_battery_remove,
};
module_platform_driver(openemc_battery_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC Battery");
MODULE_VERSION("0.1");
