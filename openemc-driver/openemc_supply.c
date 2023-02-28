// SPDX-License-Identifier: GPL-2.0+
/*
 * External power supply driver for OpenEMC
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

#define OPENEMC_POWER_SUPPLY_UNKNOWN 0
#define OPENEMC_POWER_SUPPLY_DISCONNECTED 1
#define OPENEMC_POWER_SUPPLY_USB_SDP 2
#define OPENEMC_POWER_SUPPLY_USB_DCP 3
#define OPENEMC_POWER_SUPPLY_USB_CDP 4
#define OPENEMC_POWER_SUPPLY_CC_PINS_5V_1500MA 5
#define OPENEMC_POWER_SUPPLY_CC_PINS_5V_3000MA 6
#define OPENEMC_POWER_SUPPLY_PDO_CONTRACT 7
#define OPENEMC_POWER_SUPPLY_NEGOTIATING 8

static const struct of_device_id openemc_supply_of_match[] = {
	{ .compatible = "openemc,openemc_supply" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_supply_of_match);

struct openemc_supply {
	struct device *dev;
	struct openemc *emc;

	struct power_supply *psy;
	unsigned int irq;

	u8 type;
	u32 requested_voltage;
	u32 max_current;
	u8 usb_communication;
	u32 supply_voltage;
	u32 supply_current;
};

static int openemc_supply_update(struct openemc_supply *sup)
{
	int ret;

	ret = openemc_read_u8(sup->emc, OPENEMC_SUPPLY_TYPE, &sup->type);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(sup->emc, OPENEMC_SUPPLY_REQUESTED_VOLTAGE,
			       &sup->requested_voltage);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(sup->emc, OPENEMC_SUPPLY_MAX_CURRENT,
			       &sup->max_current);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(sup->emc, OPENEMC_SUPPLY_USB_COMMUNICATION,
			      &sup->usb_communication);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(sup->emc, OPENEMC_SUPPLY_VOLTAGE,
			       &sup->supply_voltage);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(sup->emc, OPENEMC_SUPPLY_CURRENT,
			       &sup->supply_current);
	if (ret < 0)
		return ret;

	return 0;
}

int openemc_supply_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct openemc_supply *sup = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sup->type != OPENEMC_POWER_SUPPLY_UNKNOWN &&
			      sup->type != OPENEMC_POWER_SUPPLY_DISCONNECTED;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sup->supply_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = sup->requested_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = sup->supply_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = sup->max_current * 1000;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		switch (sup->type) {
		case OPENEMC_POWER_SUPPLY_UNKNOWN:
		case OPENEMC_POWER_SUPPLY_DISCONNECTED:
		case OPENEMC_POWER_SUPPLY_NEGOTIATING:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		case OPENEMC_POWER_SUPPLY_USB_SDP:
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case OPENEMC_POWER_SUPPLY_USB_DCP:
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case OPENEMC_POWER_SUPPLY_USB_CDP:
			val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		case OPENEMC_POWER_SUPPLY_PDO_CONTRACT:
			val->intval = POWER_SUPPLY_USB_TYPE_PD;
			break;
		case OPENEMC_POWER_SUPPLY_CC_PINS_5V_1500MA:
		case OPENEMC_POWER_SUPPLY_CC_PINS_5V_3000MA:
			val->intval = POWER_SUPPLY_USB_TYPE_C;
			break;
		default:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t openemc_supply_irq_handler(int irq, void *ptr)
{
	struct openemc_supply *sup = ptr;
	int ret;

	ret = openemc_supply_update(sup);
	if (ret < 0) {
		dev_err(sup->dev, "update failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	power_supply_changed(sup->psy);

	return IRQ_HANDLED;
}

static const enum power_supply_property openemc_supply_props[] = {
	POWER_SUPPLY_PROP_ONLINE,      POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX, POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX, POWER_SUPPLY_PROP_USB_TYPE,
};

static const enum power_supply_usb_type openemc_supply_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN, POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,     POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,       POWER_SUPPLY_USB_TYPE_PD,
};

static const struct power_supply_desc openemc_supply_desc = {
	.name = "openemc-supply",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = openemc_supply_usb_types,
	.num_usb_types = ARRAY_SIZE(openemc_supply_usb_types),
	.properties = openemc_supply_props,
	.num_properties = ARRAY_SIZE(openemc_supply_props),
	.get_property = openemc_supply_get_property,
};

static int openemc_supply_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_supply *sup;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg;
	struct irq_fwspec fwspec;
	int ret;

	match = of_match_node(openemc_supply_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	sup = devm_kzalloc(&pdev->dev, sizeof(*sup), GFP_KERNEL);
	if (!sup)
		return -ENOMEM;

	sup->dev = &pdev->dev;
	sup->emc = emc;

	dev_set_drvdata(&pdev->dev, sup);

	memset(&psy_cfg, 0, sizeof(psy_cfg));
	psy_cfg.of_node = pdev->dev.of_node;
	psy_cfg.drv_data = sup;

	psy_desc = devm_kzalloc(&pdev->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;
	memcpy(psy_desc, &openemc_supply_desc, sizeof(*psy_desc));
	of_property_read_string(pdev->dev.of_node, "supply-name",
				&psy_desc->name);

	ret = openemc_supply_update(sup);
	if (ret < 0) {
		dev_err(sup->dev, "init failed: %d\n", ret);
		return ret;
	}

	sup->psy =
		devm_power_supply_register_no_ws(sup->dev, psy_desc, &psy_cfg);
	if (IS_ERR(sup->psy))
		return PTR_ERR(sup->psy);

	if (sup->emc->irq) {
		fwspec.fwnode = emc->dev->fwnode;
		fwspec.param_count = 2;
		fwspec.param[0] = OPENEMC_IRQ_SUPPLY;
		fwspec.param[1] = IRQ_TYPE_NONE;

		sup->irq = irq_create_fwspec_mapping(&fwspec);
		if (!sup->irq) {
			dev_err(sup->dev, "failed to map supply IRQ\n");
			return -ENODEV;
		}

		ret = devm_request_threaded_irq(
			sup->dev, sup->irq, NULL, openemc_supply_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(sup->dev),
			sup);
		if (ret < 0) {
			dev_err(sup->dev, "failed to request irq %d\n",
				sup->irq);
			return ret;
		}
	}

	dev_info(sup->dev, "OpenEMC power supply %s registered\n",
		 psy_desc->name);

	return 0;
}

static int openemc_supply_remove(struct platform_device *pdev)
{
	struct openemc_supply *sup = platform_get_drvdata(pdev);

	if (sup->irq) {
		devm_free_irq(sup->dev, sup->irq, sup);
		irq_dispose_mapping(sup->irq);
	}

	return 0;
}

static struct platform_driver openemc_supply_driver = {
	.driver = {
		.name = "openemc_supply",
		.of_match_table = of_match_ptr(openemc_supply_of_match),
	},
	.probe = openemc_supply_probe,
	.remove = openemc_supply_remove,
};
module_platform_driver(openemc_supply_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC External Power Supply");
MODULE_VERSION("0.1");
