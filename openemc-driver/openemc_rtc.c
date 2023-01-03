// SPDX-License-Identifier: GPL-2.0+
/*
 * RTC driver for OpenEMC
 *
 * Copyright (C) 2022 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include "openemc.h"

#define READY_TIMEOUT_MS 10000

static const struct of_device_id openemc_rtc_of_match[] = {
	{ .compatible = "openemc,openemc_rtc" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_rtc_of_match);

struct openemc_rtc {
	struct device *dev;
	struct openemc *emc;

	struct rtc_device *rtc;
	struct rtc_time alarm;
	unsigned int irq;
};

static int openemc_rtc_wait_ready(struct openemc_rtc *rtc)
{
	ktime_t start = ktime_get();
	u8 ready;
	int ret;

	while (true) {
		ret = openemc_read_u8(rtc->emc, OPENEMC_REG_RTC_READY, &ready);
		if (ret < 0)
			return ret;

		if (ready)
			break;

		if (ktime_ms_delta(ktime_get(), start) >= READY_TIMEOUT_MS) {
			dev_err(rtc->dev, "timeout waiting for RTC\n");
			return -ETIMEDOUT;
		}

		msleep(50);
	}

	return 0;
}

static int openemc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct openemc_rtc *rtc = dev_get_drvdata(dev);
	u64 clock;
	int ret;

	ret = openemc_rtc_wait_ready(rtc);
	if (ret < 0)
		return ret;

	ret = openemc_read_u64(rtc->emc, OPENEMC_REG_RTC_CLOCK, &clock);
	if (ret < 0)
		return ret;

	rtc_time64_to_tm(clock >> 32, tm);

	return 0;
}

static int openemc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct openemc_rtc *rtc = dev_get_drvdata(dev);
	time64_t time = rtc_tm_to_time64(tm);
	int ret;

	ret = openemc_rtc_wait_ready(rtc);
	if (ret < 0)
		return ret;

	ret = openemc_write_u32(rtc->emc, OPENEMC_REG_RTC_CLOCK, (u32)time);
	if (ret < 0)
		return ret;

	return 0;
}

static int openemc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct openemc_rtc *rtc = dev_get_drvdata(dev);
	u32 clock;
	int ret;

	ret = openemc_rtc_wait_ready(rtc);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(rtc->emc, OPENEMC_REG_RTC_ALARM, &clock);
	if (ret < 0)
		return ret;

	ret = openemc_read_u8(rtc->emc, OPENEMC_REG_RTC_ALARM_OCCURRED,
			      &alarm->pending);
	if (ret < 0)
		return ret;

	if (clock != 0) {
		rtc_time64_to_tm(clock, &alarm->time);
		rtc->alarm = alarm->time;
		alarm->enabled = true;
	} else {
		alarm->time = rtc->alarm;
		alarm->enabled = false;
	}

	return 0;
}

static int openemc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct openemc_rtc *rtc = dev_get_drvdata(dev);
	time64_t time = rtc_tm_to_time64(&alarm->time);
	int ret;

	if (!alarm->enabled)
		time = 0;

	dev_dbg(rtc->dev, "set alarm: time=%lld  enabled=%d\n", time,
		alarm->enabled);

	ret = openemc_rtc_wait_ready(rtc);
	if (ret < 0)
		return ret;

	ret = openemc_write_u8(rtc->emc, OPENEMC_REG_RTC_ALARM_ARMED, 0);
	if (ret < 0)
		return ret;

	ret = openemc_write_u32(rtc->emc, OPENEMC_REG_RTC_ALARM, (u32)time);
	if (ret < 0)
		return ret;

	ret = openemc_write_u8(rtc->emc, OPENEMC_REG_RTC_ALARM_OCCURRED, 0);
	if (ret < 0)
		return ret;

	if (alarm->enabled) {
		ret = openemc_write_u8(rtc->emc, OPENEMC_REG_RTC_ALARM_ARMED,
				       1);
		if (ret < 0)
			return ret;
	}

	rtc->alarm = alarm->time;

	return 0;
}

static int openemc_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct rtc_wkalrm alarm;
	int ret;

	ret = openemc_rtc_read_alarm(dev, &alarm);
	if (ret < 0)
		return ret;

	alarm.enabled = !!enabled;

	ret = openemc_rtc_set_alarm(dev, &alarm);
	if (ret < 0)
		return ret;

	return 0;
}

static int openemc_rtc_read_offset(struct device *dev, long *offset)
{
	struct openemc_rtc *rtc = dev_get_drvdata(dev);
	u8 slowdown;
	int ret;

	ret = openemc_read_u8(rtc->emc, OPENEMC_REG_RTC_SLOWDOWN, &slowdown);
	if (ret < 0)
		return ret;

	*offset = (long)slowdown * 1000 * 1000000 / 1048576;

	return 0;
}

static int openemc_rtc_set_offset(struct device *dev, long offset)
{
	struct openemc_rtc *rtc = dev_get_drvdata(dev);
	u8 slowdown = offset * 1048576 / 1000000 / 1000;
	int ret;

	ret = openemc_write_u8(rtc->emc, OPENEMC_REG_RTC_SLOWDOWN, slowdown);
	if (ret < 0)
		return ret;

	return 0;
}

static struct rtc_class_ops openemc_rtc_ops = {
	.read_time = openemc_rtc_read_time,
	.set_time = openemc_rtc_set_time,
	.read_alarm = openemc_rtc_read_alarm,
	.set_alarm = openemc_rtc_set_alarm,
	.alarm_irq_enable = openemc_rtc_alarm_irq_enable,
	.read_offset = openemc_rtc_read_offset,
	.set_offset = openemc_rtc_set_offset,
};

static ssize_t openemc_rtc_wakeup_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct openemc_rtc *rtc = dev_get_drvdata(dev->parent);
	int ret;
	u8 wakeup;

	ret = openemc_read_u8(rtc->emc, OPENEMC_REG_RTC_ALARM_AT_BOOT, &wakeup);
	if (ret < 0)
		return ret;

	if (wakeup == 0)
		return sprintf(buf, "no");
	else if (wakeup == 1)
		return sprintf(buf, "yes");
	else
		return sprintf(buf, "wait");
}
static DEVICE_ATTR_RO(openemc_rtc_wakeup);

static struct attribute *openemc_rtc_attrs[] = {
	&dev_attr_openemc_rtc_wakeup.attr, NULL
};
ATTRIBUTE_GROUPS(openemc_rtc);

static irqreturn_t openemc_rtc_irq_handler(int irq, void *ptr)
{
	struct openemc_rtc *rtc = ptr;

	openemc_rtc_wait_ready(rtc);
	openemc_write_u8(rtc->emc, OPENEMC_REG_RTC_ALARM_OCCURRED, 0);

	rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static int openemc_rtc_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_rtc *rtc;
	struct irq_fwspec fwspec;
	int ret;

	match = of_match_node(openemc_rtc_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->dev = &pdev->dev;
	rtc->emc = emc;

	rtc->rtc = devm_rtc_allocate_device(rtc->dev);
	if (!rtc->rtc)
		return -ENOMEM;

	rtc->rtc->owner = THIS_MODULE;
	rtc->rtc->ops = &openemc_rtc_ops;
	rtc->rtc->range_min = 0;
	rtc->rtc->range_max = UINT_MAX;
	rtc_time64_to_tm(0, &rtc->alarm);

	dev_set_drvdata(&pdev->dev, rtc);

	ret = openemc_rtc_wait_ready(rtc);
	if (ret < 0)
		return ret;

	if (device_property_read_bool(rtc->dev, "wakeup-source")) {
		ret = device_init_wakeup(rtc->dev, true);
		if (ret < 0)
			return ret;

		dev_info(rtc->dev, "RTC alarm can wake system\n");
	}

	ret = devm_rtc_register_device(rtc->rtc);
	if (ret < 0) {
		dev_err(rtc->dev, "failed to register RTC device: %d\n", ret);
		return ret;
	}

	if (rtc->emc->irq) {
		fwspec.fwnode = emc->dev->fwnode;
		fwspec.param_count = 2;
		fwspec.param[0] = OPENEMC_IRQ_RTC_ALARM;
		fwspec.param[1] = IRQ_TYPE_NONE;

		rtc->irq = irq_create_fwspec_mapping(&fwspec);
		if (!rtc->irq) {
			dev_err(rtc->dev, "failed to map RTC IRQ\n");
			return -ENODEV;
		}

		ret = devm_request_threaded_irq(
			rtc->dev, rtc->irq, NULL, openemc_rtc_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(rtc->dev),
			rtc);
		if (ret < 0) {
			dev_err(rtc->dev, "failed to request irq %d\n",
				rtc->irq);
			return ret;
		}
	}

	ret = devm_device_add_groups(&rtc->rtc->dev, openemc_rtc_groups);
	if (ret < 0)
		return ret;

	dev_info(rtc->dev, "OpenEMC RTC registered\n");

	return 0;
}

static int openemc_rtc_remove(struct platform_device *pdev)
{
	struct openemc_rtc *rtc = platform_get_drvdata(pdev);

	if (rtc->irq) {
		devm_free_irq(rtc->dev, rtc->irq, rtc);
		irq_dispose_mapping(rtc->irq);
	}

	return 0;
}

static struct platform_driver openemc_rtc_driver = {
	.driver = {
		.name = "openemc_rtc",
		.of_match_table = of_match_ptr(openemc_rtc_of_match),
	},
	.probe = openemc_rtc_probe,
	.remove = openemc_rtc_remove,
};
module_platform_driver(openemc_rtc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC RTC");
MODULE_VERSION("0.1");
