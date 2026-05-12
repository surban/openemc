// SPDX-License-Identifier: GPL-2.0+
/*
 * PWM driver for OpenEMC
 *
 * Copyright (C) 2022-2026 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/pwm.h>

#include "openemc.h"

#define MAX_TIMERS 16
#define MAX_CHANNELS_PER_TIMER 4
#define MAX_CHANNELS (MAX_TIMERS * MAX_CHANNELS_PER_TIMER)

static const struct of_device_id openemc_pwm_of_match[] = {
	{ .compatible = "openemc,openemc_pwm" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_pwm_of_match);

struct openemc_pwm {
	struct device *dev;
	struct openemc *emc;

	struct mutex lock;
	struct pwm_chip chip;
	u8 n_channels[MAX_TIMERS + 1];
	u8 complementary_invert[MAX_CHANNELS];
};

static inline struct openemc_pwm *to_openemc_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct openemc_pwm, chip);
}

static int openemc_timer_channel(struct openemc_pwm *pwm, unsigned int hwpwm,
				 u8 *timer, u8 *channel)
{
	u8 t;

	for (t = 0; pwm->n_channels[t] != 0; t++) {
		if (hwpwm < pwm->n_channels[t]) {
			*timer = t;
			*channel = hwpwm;
			return 0;
		}

		hwpwm -= pwm->n_channels[t];
	}

	return -EINVAL;
}

static int openemc_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm_dev,
			     const struct pwm_state *state)
{
	struct openemc_pwm *pwm = to_openemc_pwm(chip);
	u16 freq = 1000000000 / max(state->period, 1ULL);
	u16 duty = U16_MAX * state->duty_cycle / state->period;
	bool enabled = state->enabled;
	u8 timer, channel;
	u8 main_invert, comp_invert, polarity_bits;
	int ret = 0;

	mutex_lock(&pwm->lock);

	ret = openemc_timer_channel(pwm, pwm_dev->hwpwm, &timer, &channel);
	if (ret < 0)
		goto out;

	/* Dsabling the on-chip PWM generator causes inconsistent logic levels
	   on the complementary outputs of timer 1. Thus we always enable it,
	   but set its duty cycle to 0% if it should be disabled. */
	if (timer == 0 && !enabled) {
		duty = 0;
		enabled = true;
	}

	dev_dbg(pwm->dev,
		"%s PWM %d (timer %d, channel %d) with "
		"frequency %d Hz, duty cycle %d%% and %s polarity\n",
		enabled ? "enabling" : "disabling", pwm_dev->hwpwm, timer,
		channel, freq, duty * 100 / U16_MAX,
		state->polarity == PWM_POLARITY_INVERSED ? "inverted" :
							   "normal");

	ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_TIMER, timer);
	if (ret < 0)
		goto out;

	ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_CHANNEL, channel);
	if (ret < 0)
		goto out;

	if (!enabled) {
		ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_CHANNEL_OUTPUT, 0);
		if (ret < 0)
			goto out;
	}

	main_invert = (state->polarity == PWM_POLARITY_INVERSED) ? 1 : 0;
	comp_invert = main_invert ^ pwm->complementary_invert[pwm_dev->hwpwm];
	polarity_bits = (main_invert ? OPENEMC_PWM_CHANNEL_MAIN : 0) |
			(comp_invert ? OPENEMC_PWM_CHANNEL_COMPLEMENTARY : 0);
	ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_CHANNEL_POLARITY,
			       polarity_bits);
	if (ret < 0)
		goto out;

	ret = openemc_write_u32(pwm->emc, OPENEMC_PWM_TIMER_FREQUENCY, freq);
	if (ret < 0)
		goto out;

	ret = openemc_write_u16(pwm->emc, OPENEMC_PWM_CHANNEL_DUTY_CYCLE, duty);
	if (ret < 0)
		goto out;

	if (enabled) {
		ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_CHANNEL_OUTPUT,
				       OPENEMC_PWM_CHANNEL_MAIN |
					       OPENEMC_PWM_CHANNEL_COMPLEMENTARY);
		if (ret < 0)
			goto out;
	}

out:
	mutex_unlock(&pwm->lock);
	return ret;
}

static const struct pwm_ops openemc_pwm_ops = {
	.apply = openemc_pwm_apply,
	.owner = THIS_MODULE,
};

static int openemc_pwm_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_pwm *pwm;
	u8 timer, timers;
	u8 remap[MAX_TIMERS];
	u32 ramp_time_ms[MAX_CHANNELS];
	int ramp_time_count;
	int comp_invert_count;
	unsigned int hwpwm = 0;
	u8 ch;
	int ret;

	match = of_match_node(openemc_pwm_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->dev = &pdev->dev;
	pwm->emc = emc;
	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &openemc_pwm_ops;
	pwm->chip.npwm = 0;

	mutex_init(&pwm->lock);

	memset(remap, 0, MAX_TIMERS);
	of_property_read_u8_array(pdev->dev.of_node, "timer-remap", remap,
				  MAX_TIMERS);

	memset(ramp_time_ms, 0, sizeof(ramp_time_ms));
	ramp_time_count = of_property_count_u32_elems(
		pdev->dev.of_node, "openemc,duty-cycle-ramp-time-ms");
	if (ramp_time_count > 0) {
		ramp_time_count = min(ramp_time_count, (int)MAX_CHANNELS);
		ret = of_property_read_u32_array(
			pdev->dev.of_node, "openemc,duty-cycle-ramp-time-ms",
			ramp_time_ms, ramp_time_count);
		if (ret < 0)
			return ret;
	}

	memset(pwm->complementary_invert, 0, sizeof(pwm->complementary_invert));
	comp_invert_count = of_property_count_u8_elems(
		pdev->dev.of_node, "openemc,complementary-invert");
	if (comp_invert_count > 0) {
		comp_invert_count = min(comp_invert_count, (int)MAX_CHANNELS);
		ret = of_property_read_u8_array(
			pdev->dev.of_node, "openemc,complementary-invert",
			pwm->complementary_invert, comp_invert_count);
		if (ret < 0)
			return ret;
	}

	ret = openemc_read_u8(pwm->emc, OPENEMC_PWM_TIMERS, &timers);
	if (ret < 0)
		return ret;
	timers = min(timers, (u8)MAX_TIMERS);

	for (timer = 0; timer < timers; timer++) {
		ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_TIMER, timer);
		if (ret < 0)
			return ret;

		ret = openemc_read_u8(pwm->emc, OPENEMC_PWM_TIMER_CHANNELS,
				      &pwm->n_channels[timer]);
		if (ret < 0)
			return ret;

		pwm->chip.npwm += pwm->n_channels[timer];

		dev_dbg(pwm->dev, "Timer %d remap: %d\n", timer, remap[timer]);
		ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_TIMER_REMAP,
				       remap[timer]);
		if (ret < 0)
			return ret;

		for (ch = 0; ch < pwm->n_channels[timer]; ch++) {
			ret = openemc_write_u8(pwm->emc, OPENEMC_PWM_CHANNEL,
					       ch);
			if (ret < 0)
				return ret;

			dev_dbg(pwm->dev,
				"PWM %u (timer %d, channel %d) duty cycle ramp time: %u ms\n",
				hwpwm, timer, ch, ramp_time_ms[hwpwm]);
			ret = openemc_write_u32(
				pwm->emc,
				OPENEMC_PWM_CHANNEL_DUTY_CYCLE_RAMP_TIME,
				ramp_time_ms[hwpwm]);
			if (ret < 0)
				return ret;

			/* Configure the complementary output polarity early
			   (before the PWM consumer applies its first state) so
			   that the idle pin level matches the board wiring. */
			ret = openemc_write_u8(
				pwm->emc, OPENEMC_PWM_CHANNEL_POLARITY,
				(pwm->complementary_invert[hwpwm] & 1) ?
					OPENEMC_PWM_CHANNEL_COMPLEMENTARY :
					0);
			if (ret < 0)
				return ret;

			ret = openemc_write_u8(
				pwm->emc, OPENEMC_PWM_CHANNEL_OUTPUT, 0);
			if (ret < 0)
				return ret;

			hwpwm++;
		}
	}

	ret = devm_pwmchip_add(pwm->dev, &pwm->chip);
	if (ret < 0) {
		dev_err(pwm->dev, "failed to register PWM chip: %d\n", ret);
		return ret;
	}

	dev_info(pwm->dev, "OpenEMC PWM registered with %d devices\n",
		 pwm->chip.npwm);

	return 0;
}

static struct platform_driver openemc_pwm_driver = {
	.driver = {
		.name = "openemc_pwm",
		.of_match_table = of_match_ptr(openemc_pwm_of_match),
	},
	.probe = openemc_pwm_probe,
};
module_platform_driver(openemc_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC PWM");
MODULE_VERSION("1.3");
