// SPDX-License-Identifier: GPL-2.0+
/*
 * GPIO driver for OpenEMC
 *
 * Copyright (C) 2022 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio/driver.h>
#include <linux/version.h>

#include "openemc.h"

#define LONG_BITS (sizeof(long) * 8)

static const struct of_device_id openemc_gpio_of_match[] = {
	{ .compatible = "openemc,openemc_gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_gpio_of_match);

struct openemc_gpio {
	struct device *dev;
	struct openemc *emc;
	struct gpio_chip chip;
	bool initialized;

	struct irq_domain *irq_domain;
	unsigned int emc_irq[OPENEMC_EXT_IRQS];
	int irq_type[OPENEMC_MAX_PINS];
};

static int openemc_gpio_read_in(struct openemc_gpio *gpio, u8 *in)
{
	return openemc_read_data(gpio->emc, OPENEMC_GPIO_IN,
				 gpio->chip.ngpio / 8, in);
}

static bool openemc_gpio_get_in_value(u8 *in, unsigned int offset)
{
	return (in[offset / 8] & BIT(offset % 8)) != 0;
}

static int openemc_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);
	u8 in[OPENEMC_MAX_PINS / 8];
	int ret;

	ret = openemc_gpio_read_in(gpio, in);
	if (ret < 0)
		return ret;

	return openemc_gpio_get_in_value(in, offset);
}

static int openemc_gpio_get_multiple(struct gpio_chip *chip,
				     unsigned long *mask, unsigned long *bits)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);
	u8 in[OPENEMC_MAX_PINS / 8];
	int ret;
	int i;

	ret = openemc_gpio_read_in(gpio, in);
	if (ret < 0)
		return ret;

	for (i = 0; i < chip->ngpio; i++) {
		if (mask[i / LONG_BITS] & BIT(i % LONG_BITS)) {
			if (openemc_gpio_get_in_value(in, i)) {
				bits[i / LONG_BITS] |= BIT(i % LONG_BITS);
			} else {
				bits[i / LONG_BITS] &= ~BIT(i % LONG_BITS);
			}
		}
	}

	return 0;
}

static void openemc_gpio_set(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);

	mutex_lock(&gpio->emc->lock);

	gpio->emc->pin[offset].out_level = value;
	openemc_pins_write_out(gpio->emc);

	mutex_unlock(&gpio->emc->lock);
}

static void openemc_gpio_set_multiple(struct gpio_chip *chip,
				      unsigned long *mask, unsigned long *bits)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);
	int i;

	mutex_lock(&gpio->emc->lock);

	for (i = 0; i < chip->ngpio; i++) {
		if (mask[i / LONG_BITS] & BIT(i % LONG_BITS)) {
			gpio->emc->pin[i].out_level =
				bits[i / LONG_BITS] & BIT(i % LONG_BITS);
		}
	}

	openemc_pins_write_out(gpio->emc);

	mutex_unlock(&gpio->emc->lock);
}

static int openemc_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);

	switch (gpio->emc->pin[offset].direction) {
	case OPENEMC_PIN_DIRECTION_IN:
		return GPIO_LINE_DIRECTION_IN;
	case OPENEMC_PIN_DIRECTION_OUT:
	default:
		return GPIO_LINE_DIRECTION_OUT;
	}
}

static int openemc_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);
	int ret = 0;

	mutex_lock(&gpio->emc->lock);

	if (gpio->emc->pin[offset].direction != OPENEMC_PIN_DIRECTION_IN) {
		gpio->emc->pin[offset].direction = OPENEMC_PIN_DIRECTION_IN;

		ret = openemc_pins_write_cfg(gpio->emc);
		if (ret < 0)
			goto out;

		ret = openemc_pins_write_out(gpio->emc);
		if (ret < 0)
			goto out;
	}

out:
	mutex_unlock(&gpio->emc->lock);
	return 0;
}

static int openemc_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);
	int ret = 0;

	mutex_lock(&gpio->emc->lock);

	if (gpio->emc->pin[offset].direction != OPENEMC_PIN_DIRECTION_OUT) {
		gpio->emc->pin[offset].out_level = value;
		gpio->emc->pin[offset].direction = OPENEMC_PIN_DIRECTION_OUT;

		ret = openemc_pins_write_out(gpio->emc);
		if (ret < 0)
			goto out;

		ret = openemc_pins_write_cfg(gpio->emc);
		if (ret < 0)
			goto out;
	} else {
		mutex_unlock(&gpio->emc->lock);
		openemc_gpio_set(chip, offset, value);
		return 0;
	}

out:
	mutex_unlock(&gpio->emc->lock);
	return 0;
}

static int openemc_gpio_init_valid_mask(struct gpio_chip *chip,
					unsigned long *valid_mask,
					unsigned int ngpios)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);
	int n;

	for (n = 0; n < ngpios; n++) {
		if (gpio->emc->pin[n].usable) {
			valid_mask[n / LONG_BITS] |= BIT(n % LONG_BITS);
		} else {
			valid_mask[n / LONG_BITS] &= ~BIT(n % LONG_BITS);
		}
	}

	return 0;
}

static int openemc_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct openemc_gpio *gpio = gpiochip_get_data(chip);
	struct irq_fwspec fwspec;

	fwspec.fwnode = gpio->dev->fwnode;
	fwspec.param_count = 2;
	fwspec.param[0] = offset;
	fwspec.param[1] = IRQ_TYPE_NONE;

	return irq_create_fwspec_mapping(&fwspec);
}

static int openemc_gpio_of_xlate(struct gpio_chip *chip,
				 const struct of_phandle_args *gpio_desc,
				 u32 *flags)
{
	if (chip->of_gpio_n_cells < 2)
		return -EINVAL;

	if (flags)
		*flags = gpio_desc->args[1];

	return gpio_desc->args[0];
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
static int openemc_gpio_of_gpio_ranges_fallback(struct gpio_chip *gc,
						struct device_node *np)
{
	struct pinctrl_dev *pctldev = of_pinctrl_get(np);
	struct openemc_gpio *gpio = gpiochip_get_data(gc);

	dev_warn(gpio->dev, "Using GPIO ranges fallback for %s\n", np->name);

	// of_node_put(np);

	if (!pctldev)
		return 0;

	gpiochip_add_pin_range(gc, pinctrl_dev_get_devname(pctldev), 0, 0,
			       gc->ngpio);

	return 0;
}
#endif

static const struct gpio_chip openemc_gpio_template = {
	.label = "openemc_gpio",
	.owner = THIS_MODULE,
	.get_direction = openemc_gpio_get_direction,
	.direction_input = openemc_gpio_direction_input,
	.direction_output = openemc_gpio_direction_output,
	.get = openemc_gpio_get,
	.get_multiple = openemc_gpio_get_multiple,
	.set = openemc_gpio_set,
	.set_multiple = openemc_gpio_set_multiple,
	.set_config = gpiochip_generic_config,
	.init_valid_mask = openemc_gpio_init_valid_mask,
	.to_irq = openemc_gpio_to_irq,
	.of_xlate = openemc_gpio_of_xlate,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
	.of_gpio_ranges_fallback = openemc_gpio_of_gpio_ranges_fallback,
#endif
	.of_gpio_n_cells = 2,
	.base = -1,
	.can_sleep = true,
};

static void openemc_gpio_irq_mask(struct irq_data *data)
{
	struct openemc_gpio *gpio = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = data->hwirq;
	unsigned int bank = hwirq / OPENEMC_EXT_IRQS;
	unsigned int exti = hwirq % OPENEMC_EXT_IRQS;

	dev_dbg(gpio->dev, "Mask GPIO IRQ %ld\n", hwirq);

	if (bank != gpio->emc->exti_bank[exti]) {
		dev_warn(gpio->dev, "Ignoring mask for inactive GPIO IRQ %ld\n",
			 hwirq);
		return;
	}

	openemc_irq_do_mask(gpio->emc, exti);
}

static void openemc_gpio_irq_unmask(struct irq_data *data)
{
	struct openemc_gpio *gpio = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = data->hwirq;
	unsigned int bank = hwirq / OPENEMC_EXT_IRQS;
	unsigned int exti = hwirq % OPENEMC_EXT_IRQS;

	dev_dbg(gpio->dev, "Unmask GPIO IRQ %ld\n", hwirq);

	if (bank != gpio->emc->exti_bank[exti]) {
		dev_warn(gpio->dev,
			 "Ignoring unmask for inactive GPIO IRQ %ld\n", hwirq);
		return;
	}

	openemc_irq_do_unmask(gpio->emc, exti);
}

static int openemc_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct openemc_gpio *gpio = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = data->hwirq;
	unsigned int bank = hwirq / OPENEMC_EXT_IRQS;
	unsigned int exti = hwirq % OPENEMC_EXT_IRQS;
	int ret;

	dev_dbg(gpio->dev, "Set GPIO IRQ %ld type to %d\n", data->hwirq, type);

	ret = openemc_irq_check_type(type);
	if (ret < 0)
		return ret;

	gpio->irq_type[data->hwirq] = type;

	if (bank == gpio->emc->exti_bank[exti])
		return openemc_irq_do_set_type(gpio->emc, exti, type);
	else
		return 0;
}

static int openemc_gpio_irq_request_resources(struct irq_data *data)
{
	struct openemc_gpio *gpio = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = data->hwirq;
	int ret;

	if (!gpio->initialized) {
		dev_info(gpio->dev,
			 "Defering IRQ request after GPIO is initialized\n");
		return -EPROBE_DEFER;
	}

	dev_dbg(gpio->dev, "Locking GPIO %ld as IRQ\n", hwirq);

	ret = openemc_gpio_direction_input(&gpio->chip, hwirq);
	if (ret < 0)
		return ret;

	ret = gpiochip_lock_as_irq(&gpio->chip, hwirq);
	if (ret < 0) {
		dev_err(gpio->dev, "unable to lock GPIO %lu for IRQ\n", hwirq);
		return ret;
	}

	/* We must prevent the module from getting unloaded when it has
	   IRQs registered in its domain. */
	try_module_get(THIS_MODULE);

	return 0;
}

static void openemc_gpio_irq_release_resources(struct irq_data *data)
{
	struct openemc_gpio *gpio = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = data->hwirq;

	dev_dbg(gpio->dev, "Unlocking GPIO %ld as IRQ\n", hwirq);

	gpiochip_unlock_as_irq(&gpio->chip, hwirq);

	module_put(THIS_MODULE);
}

static void openemc_gpio_irq_bus_lock(struct irq_data *data)
{
	struct openemc_gpio *gpio = irq_data_get_irq_chip_data(data);

	mutex_lock(&gpio->emc->irq_lock);
}

static void openemc_gpio_irq_bus_sync_unlock(struct irq_data *data)
{
	struct openemc_gpio *gpio = irq_data_get_irq_chip_data(data);
	int exti, line;

	for (exti = 0; exti < OPENEMC_EXT_IRQS; exti++) {
		if (gpio->emc->exti_bank[exti] < 0)
			continue;

		line = exti + gpio->emc->exti_bank[exti] * OPENEMC_EXT_IRQS;
		gpio->emc->irq_types[exti] = gpio->irq_type[line];
	}

	openemc_irq_do_sync(gpio->emc);

	mutex_unlock(&gpio->emc->irq_lock);
}

static struct irq_chip openemc_gpio_irq_chip = {
	.name = "openemc_gpio",
	.irq_mask = openemc_gpio_irq_mask,
	.irq_unmask = openemc_gpio_irq_unmask,
	.irq_set_type = openemc_gpio_irq_set_type,
	.irq_request_resources = openemc_gpio_irq_request_resources,
	.irq_release_resources = openemc_gpio_irq_release_resources,
	.irq_bus_lock = openemc_gpio_irq_bus_lock,
	.irq_bus_sync_unlock = openemc_gpio_irq_bus_sync_unlock,
};

static int openemc_gpio_irq_translate(struct irq_domain *d,
				      struct irq_fwspec *fwspec,
				      unsigned long *hwirq, unsigned int *type)
{
	if (fwspec->param_count != 2 || fwspec->param[0] >= OPENEMC_MAX_PINS)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = fwspec->param[1];

	return 0;
}

static int openemc_gpio_irq_alloc(struct irq_domain *d, unsigned int virq,
				  unsigned int nr_irqs, void *data)
{
	struct openemc_gpio *gpio = d->host_data;
	struct irq_fwspec *fwspec = data;
	irq_hw_number_t hwirq = fwspec->param[0];

	dev_dbg(gpio->dev, "Allocating GPIO IRQ %ld\n", hwirq);

	if (gpio->irq_type[hwirq] >= 0) {
		dev_err(gpio->dev, "GPIO IRQ %ld already allocated\n", hwirq);
		return -EBUSY;
	}

	gpio->irq_type[hwirq] = IRQ_TYPE_NONE;

	irq_domain_set_hwirq_and_chip(d, virq, hwirq, &openemc_gpio_irq_chip,
				      gpio);

	return 0;
}

static void openemc_gpio_irq_free(struct irq_domain *d, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct openemc_gpio *gpio = d->host_data;
	struct irq_data *irq_data = irq_domain_get_irq_data(d, virq);
	irq_hw_number_t hwirq = irq_data->hwirq;

	dev_dbg(gpio->dev, "Freeing GPIO IRQ %ld\n", hwirq);

	irq_domain_free_irqs_common(d, virq, nr_irqs);

	gpio->irq_type[hwirq] = -1;
}

static int openemc_gpio_irq_activate(struct irq_domain *d,
				     struct irq_data *irq_data, bool reserve)
{
	struct openemc_gpio *gpio = d->host_data;
	irq_hw_number_t hwirq = irq_data->hwirq;
	unsigned int bank = hwirq / OPENEMC_EXT_IRQS;
	unsigned int exti = hwirq % OPENEMC_EXT_IRQS;

	dev_info(gpio->dev,
		 "Activating GPIO IRQ %ld by connecting EXTI %d to bank %d\n",
		 hwirq, exti, bank);

	if (gpio->emc->exti_bank[exti] >= 0) {
		dev_err(gpio->dev, "EXTI %d needed by GPIO IRQ %ld is busy\n",
			exti, hwirq);
		return -EBUSY;
	}

	gpio->emc->exti_bank[exti] = bank;
	openemc_irq_do_set_type(gpio->emc, exti, gpio->irq_type[hwirq]);

	return 0;
}

static void openemc_gpio_irq_deactivate(struct irq_domain *d,
					struct irq_data *irq_data)
{
	struct openemc_gpio *gpio = d->host_data;
	irq_hw_number_t hwirq = irq_data->hwirq;
	unsigned int exti = hwirq % OPENEMC_EXT_IRQS;

	dev_dbg(gpio->dev, "Deactivating GPIO IRQ %ld\n", hwirq);

	gpio->emc->exti_bank[exti] = -1;
}

static struct irq_domain_ops openemc_gpio_irq_domain_ops = {
	.translate = openemc_gpio_irq_translate,
	.alloc = openemc_gpio_irq_alloc,
	.free = openemc_gpio_irq_free,
	.activate = openemc_gpio_irq_activate,
	.deactivate = openemc_gpio_irq_deactivate,
};

static irqreturn_t openemc_gpio_irq_handler(int irq, void *ptr)
{
	struct openemc_gpio *gpio = ptr;
	struct irq_data *irq_data = irq_get_irq_data(irq);
	unsigned int exti = irq_data->hwirq;
	struct irq_desc *irq_desc;
	irq_hw_number_t hwirq;
	unsigned long flags;

	dev_dbg(gpio->dev, "handling EXTI %d IRQ\n", exti);

	if (gpio->emc->exti_bank[exti] < 0) {
		dev_warn(gpio->dev,
			 "EXTI %d IRQ without connected GPIO, masking\n", exti);
		openemc_irq_do_mask(gpio->emc, exti);
		openemc_irq_do_sync(gpio->emc);
		return IRQ_NONE;
	}

	hwirq = exti + gpio->emc->exti_bank[exti] * OPENEMC_EXT_IRQS;

	irq_desc = irq_resolve_mapping(gpio->irq_domain, hwirq);
	if (!irq_desc)
		return IRQ_NONE;

	dev_dbg(gpio->dev, "triggering GPIO %ld IRQ\n", hwirq);

	local_irq_save(flags);
	handle_simple_irq(irq_desc);
	local_irq_restore(flags);

	return IRQ_HANDLED;
}

static int openemc_gpio_irq_setup(struct openemc_gpio *gpio)
{
	struct openemc *emc = gpio->emc;
	struct irq_fwspec fwspec;
	unsigned int exti;
	int n;
	int ret;

	if (!emc->irq_domain) {
		dev_warn(gpio->dev, "Interrupts not active\n");
		return 0;
	}

	for (n = 0; n < OPENEMC_MAX_PINS; n++)
		gpio->irq_type[n] = -1;

	gpio->irq_domain =
		irq_domain_add_linear(gpio->dev->of_node, OPENEMC_MAX_PINS,
				      &openemc_gpio_irq_domain_ops, gpio);
	if (!gpio->irq_domain) {
		dev_warn(gpio->dev, "cannot create IRQ domain\n");
		return -ENODEV;
	}

	for (exti = 0; exti < OPENEMC_EXT_IRQS; exti++) {
		fwspec.fwnode = emc->dev->fwnode;
		fwspec.param_count = 2;
		fwspec.param[0] = exti;
		fwspec.param[1] = IRQ_TYPE_NONE;

		gpio->emc_irq[exti] = irq_create_fwspec_mapping(&fwspec);
		if (!gpio->emc_irq[exti]) {
			dev_err(gpio->dev, "failed to map EXTI %d IRQ\n", exti);
			return -ENODEV;
		}

		ret = devm_request_threaded_irq(gpio->dev, gpio->emc_irq[exti],
						NULL, openemc_gpio_irq_handler,
						IRQF_ONESHOT,
						dev_name(gpio->dev), gpio);
		if (ret < 0) {
			dev_err(gpio->dev, "failed to request irq %d\n",
				gpio->emc_irq[exti]);
			return ret;
		}
	}

	return 0;
}

static int openemc_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc_gpio *gpio;
	struct openemc *emc;
	int ret;

	match = of_match_node(openemc_gpio_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	/* We must keep an explicit reference to the pinctrl module, otherwise
	   it may be unloaded first and we crash when our GPIO pinctrl ranges
	   are unregistered. */
	if (!emc->pinctrl_loaded)
		return -EPROBE_DEFER;
	try_module_get(emc->pinctrl_mod);

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, gpio);

	gpio->dev = &pdev->dev;
	gpio->emc = emc;
	gpio->chip = openemc_gpio_template;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
	gpio->chip.fwnode = gpio->dev->fwnode;
#else
	gpio->chip.of_node = gpio->dev->of_node;
#endif
	gpio->chip.parent = gpio->dev;
	gpio->chip.ngpio = emc->npin;

	if (!of_find_property(pdev->dev.of_node, "gpio-line-names", NULL))
		gpio->chip.names = (const char *const *)emc->pin_names;

	ret = openemc_gpio_irq_setup(gpio);
	if (ret < 0) {
		dev_err(gpio->dev, "Could not register IRQs, %d\n", ret);
		return ret;
	}

	ret = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (ret < 0) {
		dev_err(gpio->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}

	barrier();
	gpio->initialized = true;

	dev_info(gpio->dev, "OpenEMC provides %d GPIOs at base %d\n", emc->npin,
		 gpio->chip.base);

	return 0;
}

static int openemc_gpio_remove(struct platform_device *pdev)
{
	struct openemc_gpio *gpio = platform_get_drvdata(pdev);
	unsigned int exti, n, irq;

	for (exti = 0; exti < OPENEMC_EXT_IRQS; exti++) {
		if (gpio->emc_irq[exti]) {
			devm_free_irq(gpio->dev, gpio->emc_irq[exti], gpio);
			irq_dispose_mapping(gpio->emc_irq[exti]);
		}
	}

	if (gpio->irq_domain) {
		for (n = 0; n < OPENEMC_MAX_PINS; n++) {
			irq = irq_find_mapping(gpio->irq_domain, n);
			if (irq)
				irq_dispose_mapping(irq);
		}
		irq_domain_remove(gpio->irq_domain);
	}

	module_put(gpio->emc->pinctrl_mod);

	return 0;
}

static struct platform_driver openemc_gpio_driver = {
	.driver = {
		.name = "openemc_gpio",
		.of_match_table = of_match_ptr(openemc_gpio_of_match),
	},
	.probe = openemc_gpio_probe,
	.remove = openemc_gpio_remove,
};
module_platform_driver(openemc_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC GPIO");
MODULE_VERSION("0.1");
