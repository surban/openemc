// SPDX-License-Identifier: GPL-2.0+
/*
 * Pin control driver for OpenEMC
 *
 * Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>

#include "openemc.h"

static const struct of_device_id openemc_pinctrl_of_match[] = {
	{ .compatible = "openemc,openemc_pinctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_pinctrl_of_match);

enum openemc_pinctrl_pinmux_function {
	PINMUX_FUNCTION_GPIO,
	PINMUX_FUNCTION_ALT_PUSH_PULL,
	PINMUX_FUNCTION_ALT_OPEN_DRAIN,
	PINMUX_FUNCTION_ANALOG,
};

static const char *openemc_pinctrl_pinmux_function_names[] = {
	"gpio",
	"alt_push_pull",
	"alt_open_drain",
	"analog",
};

struct openemc_pinctrl {
	struct device *dev;
	struct openemc *emc;
	struct pinctrl_dev *pctl_dev;
	struct pinctrl_desc pctl_desc;
	struct pinctrl_pin_desc pctl_pin_descs[OPENEMC_MAX_PINS];
};

static int openemc_pinctrl_pin_config_get(struct pinctrl_dev *pctldev,
					  unsigned int pin,
					  unsigned long *config)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg = 0;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_IN)
			return -EINVAL;
		if (emc->pin[pin].in_bias != OPENEMC_PIN_BIAS_DISABLE)
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_IN)
			return -EINVAL;
		if (emc->pin[pin].in_bias != OPENEMC_PIN_BIAS_PULL_DOWN)
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_IN)
			return -EINVAL;
		if (emc->pin[pin].in_bias != OPENEMC_PIN_BIAS_PULL_UP)
			return -EINVAL;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_OUT)
			return -EINVAL;
		break;
	case PIN_CONFIG_OUTPUT:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_OUT)
			return -EINVAL;
		arg = emc->pin[pin].out_level;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_OUT)
			return -EINVAL;
		if (emc->pin[pin].out_drive != OPENEMC_PIN_DRIVE_OPEN_DRAIN)
			return -EINVAL;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_OUT)
			return -EINVAL;
		if (emc->pin[pin].out_drive != OPENEMC_PIN_DRIVE_PUSH_PULL)
			return -EINVAL;
		break;
	case PIN_CONFIG_SLEW_RATE:
		if (emc->pin[pin].direction != OPENEMC_PIN_DIRECTION_OUT)
			return -EINVAL;
		switch (emc->pin[pin].out_speed) {
		case OPENEMC_PIN_SPEED_2_MHZ:
			arg = 2;
			break;
		case OPENEMC_PIN_SPEED_10_MHZ:
			arg = 10;
			break;
		case OPENEMC_PIN_SPEED_50_MHZ:
			arg = 50;
			break;
		}
		break;
	default:
		dev_dbg(emc->dev, "Invalid config param %04x\n", param);
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int openemc_pinctrl_pin_config_set(struct pinctrl_dev *pctldev,
					  unsigned int pin,
					  unsigned long *configs,
					  unsigned int num_configs)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;
	enum pin_config_param param;
	u32 arg;
	int ret = 0;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			emc->pin[pin].in_bias = OPENEMC_PIN_BIAS_DISABLE;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			emc->pin[pin].in_bias = OPENEMC_PIN_BIAS_PULL_DOWN;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			emc->pin[pin].in_bias = OPENEMC_PIN_BIAS_PULL_UP;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			emc->pin[pin].direction =
				arg ? OPENEMC_PIN_DIRECTION_OUT :
				      OPENEMC_PIN_DIRECTION_IN;
			break;
		case PIN_CONFIG_OUTPUT:
			emc->pin[pin].direction = OPENEMC_PIN_DIRECTION_OUT;
			emc->pin[pin].out_level = arg;
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			emc->pin[pin].out_drive = OPENEMC_PIN_DRIVE_OPEN_DRAIN;
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			emc->pin[pin].out_drive = OPENEMC_PIN_DRIVE_PUSH_PULL;
			break;
		case PIN_CONFIG_SLEW_RATE:
			if (arg <= 2)
				emc->pin[pin].out_speed =
					OPENEMC_PIN_SPEED_2_MHZ;
			else if (arg <= 10)
				emc->pin[pin].out_speed =
					OPENEMC_PIN_SPEED_10_MHZ;
			else if (arg <= 50)
				emc->pin[pin].out_speed =
					OPENEMC_PIN_SPEED_50_MHZ;
			else
				return -EINVAL;
			break;
		default:
			dev_dbg(emc->dev, "Invalid config param %04x\n", param);
			return -ENOTSUPP;
		}
	}

	ret = openemc_pins_write_out(emc);
	if (ret < 0)
		return ret;

	ret = openemc_pins_write_cfg(emc);
	if (ret < 0)
		return ret;

	return 0;
}

static int openemc_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;

	return emc->npin;
}

static const char *openemc_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						  unsigned int group)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;

	return emc->pin_names[group];
}

static int openemc_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					  unsigned int group,
					  const unsigned int **pins,
					  unsigned int *num_pins)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;

	*pins = &emc->pin[group].pin;
	*num_pins = 1;

	return 0;
}

int openemc_pinctrl_pinmux_request(struct pinctrl_dev *pctldev,
				   unsigned int offset)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;

	return emc->pin[offset].usable ? 0 : -EINVAL;
}

int openemc_pinctrl_pinmux_free(struct pinctrl_dev *pctldev,
				unsigned int offset)
{
	return 0;
}

int openemc_pinctrl_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(openemc_pinctrl_pinmux_function_names);
}

const char *
openemc_pinctrl_pinmux_get_function_name(struct pinctrl_dev *pctldev,
					 unsigned int selector)
{
	return openemc_pinctrl_pinmux_function_names[selector];
}

int openemc_pinctrl_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
					       unsigned int selector,
					       const char *const **groups,
					       unsigned int *num_groups)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;

	*groups = (const char *const *)emc->pin_names;
	*num_groups = emc->npin;

	return 0;
}

int openemc_pinctrl_pinmux_set_mux(struct pinctrl_dev *pctldev,
				   unsigned int func_selector,
				   unsigned int group_selector)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;
	int ret = 0;

	mutex_lock(&emc->lock);

	dev_dbg(emc->dev, "setting mux of pin %d to %s\n", group_selector,
		openemc_pinctrl_pinmux_function_names[func_selector]);

	switch (func_selector) {
	case PINMUX_FUNCTION_GPIO:
		emc->pin[group_selector].mode = OPENEMC_PIN_MODE_GPIO;
		break;
	case PINMUX_FUNCTION_ALT_PUSH_PULL:
		emc->pin[group_selector].mode = OPENEMC_PIN_MODE_ALTERNATE_OUT;
		emc->pin[group_selector].out_drive =
			OPENEMC_PIN_DRIVE_PUSH_PULL;
		break;
	case PINMUX_FUNCTION_ALT_OPEN_DRAIN:
		emc->pin[group_selector].mode = OPENEMC_PIN_MODE_ALTERNATE_OUT;
		emc->pin[group_selector].out_drive =
			OPENEMC_PIN_DRIVE_OPEN_DRAIN;
		break;
	case PINMUX_FUNCTION_ANALOG:
		emc->pin[group_selector].mode = OPENEMC_PIN_MODE_ANALOG;
		break;
	}

	ret = openemc_pins_write_out(emc);
	if (ret < 0)
		goto out;

	ret = openemc_pins_write_cfg(emc);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&emc->lock);
	return ret;
}

int openemc_pinctrl_pinmux_gpio_request_enable(struct pinctrl_dev *pctldev,
					       struct pinctrl_gpio_range *range,
					       unsigned int offset)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;
	int ret = 0;

	mutex_lock(&emc->lock);

	dev_dbg(emc->dev, "setting mux of pin %d to gpio\n", offset);

	emc->pin[offset].mode = OPENEMC_PIN_MODE_GPIO;

	ret = openemc_pins_write_out(emc);
	if (ret < 0)
		goto out;

	ret = openemc_pins_write_cfg(emc);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&emc->lock);
	return ret;
}

void openemc_pinctrl_pinmux_gpio_disable_free(struct pinctrl_dev *pctldev,
					      struct pinctrl_gpio_range *range,
					      unsigned int offset)
{
	/* empty */
}

int openemc_pinctrl_pinmux_gpio_set_direction(struct pinctrl_dev *pctldev,
					      struct pinctrl_gpio_range *range,
					      unsigned int offset, bool input)
{
	struct openemc_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct openemc *emc = pinctrl->emc;
	int ret = 0;

	mutex_lock(&emc->lock);

	dev_dbg(emc->dev, "setting mux of pin %d to gpio %s\n", offset,
		input ? "in" : "out");

	emc->pin[offset].mode = OPENEMC_PIN_MODE_GPIO;
	emc->pin[offset].direction =
		input ? OPENEMC_PIN_DIRECTION_IN : OPENEMC_PIN_DIRECTION_OUT;

	ret = openemc_pins_write_out(emc);
	if (ret < 0)
		goto out;

	ret = openemc_pins_write_cfg(emc);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&emc->lock);
	return ret;
}

static const struct pinctrl_ops openemc_pinctrl_ops = {
	.get_groups_count = openemc_pinctrl_get_groups_count,
	.get_group_name = openemc_pinctrl_get_group_name,
	.get_group_pins = openemc_pinctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinconf_ops openemc_pinctrl_conf_ops = {
	.pin_config_get = openemc_pinctrl_pin_config_get,
	.pin_config_set = openemc_pinctrl_pin_config_set,
	.is_generic = true,
};

static const struct pinmux_ops openemc_pinctrl_mux_ops = {
	.request = openemc_pinctrl_pinmux_request,
	.free = openemc_pinctrl_pinmux_free,
	.get_functions_count = openemc_pinctrl_pinmux_get_functions_count,
	.get_function_name = openemc_pinctrl_pinmux_get_function_name,
	.get_function_groups = openemc_pinctrl_pinmux_get_function_groups,
	.set_mux = openemc_pinctrl_pinmux_set_mux,
	.gpio_request_enable = openemc_pinctrl_pinmux_gpio_request_enable,
	.gpio_disable_free = openemc_pinctrl_pinmux_gpio_disable_free,
	.gpio_set_direction = openemc_pinctrl_pinmux_gpio_set_direction,
	.strict = true,
};

static const struct pinctrl_desc openemc_pinctrl_template = {
	.name = "openemc_pinctrl",
	.owner = THIS_MODULE,
	.pctlops = &openemc_pinctrl_ops,
	.confops = &openemc_pinctrl_conf_ops,
	.pmxops = &openemc_pinctrl_mux_ops,
};

static int openemc_pinctrl_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_pinctrl *pinctrl;
	int ret, n;

	match = of_match_node(openemc_pinctrl_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	pinctrl->dev = &pdev->dev;
	pinctrl->emc = emc;
	pinctrl->emc->pinctrl_mod = THIS_MODULE;
	barrier();
	pinctrl->emc->pinctrl_loaded = true;

	platform_set_drvdata(pdev, pinctrl);

	for (n = 0; n < emc->npin; n++) {
		pinctrl->pctl_pin_descs[n].number = n;
		pinctrl->pctl_pin_descs[n].name = emc->pin_names[n];
	}

	pinctrl->pctl_desc = openemc_pinctrl_template;
	pinctrl->pctl_desc.npins = emc->npin;
	pinctrl->pctl_desc.pins =
		(const struct pinctrl_pin_desc *)&pinctrl->pctl_pin_descs;

	ret = devm_pinctrl_register_and_init(&pdev->dev, &pinctrl->pctl_desc,
					     pinctrl, &pinctrl->pctl_dev);
	if (ret < 0) {
		dev_err(pinctrl->dev, "Could not register pinctrl: %d\n", ret);
		return ret;
	}

	ret = pinctrl_enable(pinctrl->pctl_dev);
	if (ret < 0) {
		dev_err(pinctrl->dev, "Could not enable pinctrl: %d\n", ret);
		return ret;
	}

	dev_info(pinctrl->dev, "OpenEMC pinctrl with %d pins registered", emc->npin);

	return 0;
}

static int openemc_pinctrl_remove(struct platform_device *pdev)
{
	struct openemc_pinctrl *pinctrl = platform_get_drvdata(pdev);

	pinctrl->emc->pinctrl_loaded = true;
	pinctrl->emc->pinctrl_mod = NULL;

	return 0;
}

static struct platform_driver openemc_pinctrl_driver = {
	.driver = {
		.name = "openemc_pinctrl",
		.of_match_table = of_match_ptr(openemc_pinctrl_of_match),
	},
	.probe = openemc_pinctrl_probe,
	.remove = openemc_pinctrl_remove,
};
module_platform_driver(openemc_pinctrl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC Pin Control");
MODULE_VERSION("0.1");
