// SPDX-License-Identifier: GPL-2.0+
/*
 * ADC driver for OpenEMC
 *
 * Copyright (C) 2022-2024 Sebastian Urban <surban@surban.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/hwmon.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/buffer.h>

#include "openemc.h"

#define READY_TIMEOUT_MS 200
#define ACQUIRE_INTERVAL_MS 100
#define N_CHANNELS 16

static const struct of_device_id openemc_adc_of_match[] = {
	{ .compatible = "openemc,openemc_adc" },
	{}
};
MODULE_DEVICE_TABLE(of, openemc_adc_of_match);

struct openemc_adc_sample {
	ktime_t acquired;
	u16 v[N_CHANNELS];
	u16 vref;
	int temp;
};

struct openemc_adc {
	struct device *dev;
	struct openemc *emc;

	struct mutex lock;
	struct openemc_adc_sample sample;
	const char *labels[N_CHANNELS];
};

struct openemc_iio {
	struct openemc_adc *adc;
};

static int openemc_adc_acquire(struct openemc_adc *adc)
{
	ktime_t start;
	u8 ready;
	u8 buf[N_CHANNELS * 2];
	u32 temp;
	int ret, ch;

	dev_dbg(adc->dev, "ADC acquiring\n");

	ret = openemc_cmd(adc->emc, OPENEMC_ADC_CONVERT);
	if (ret < 0)
		return ret;

	start = ktime_get();
	while (true) {
		ret = openemc_read_u8(adc->emc, OPENEMC_ADC_READY, &ready);
		if (ret < 0)
			return ret;

		if (ready)
			break;

		if (ktime_ms_delta(ktime_get(), start) >= READY_TIMEOUT_MS) {
			dev_err(adc->dev, "timeout waiting for ADC\n");
			return -ETIMEDOUT;
		}

		msleep(50);
	}

	adc->sample.acquired = 0;

	ret = openemc_read_u16(adc->emc, OPENEMC_ADC_VREF, &adc->sample.vref);
	if (ret < 0)
		return ret;

	ret = openemc_read_u32(adc->emc, OPENEMC_ADC_TEMPERATURE, &temp);
	adc->sample.temp = temp * 1000;
	if (ret < 0)
		return ret;

	ret = openemc_read_data(adc->emc, OPENEMC_ADC_VALUES, N_CHANNELS * 2,
				buf);
	if (ret < 0)
		return ret;

	for (ch = 0; ch < N_CHANNELS; ch++)
		adc->sample.v[ch] = (u16)buf[2 * ch] | buf[2 * ch + 1] << 8;

	adc->sample.acquired = start;

	return 0;
}

static int openemc_adc_acquire_if_too_old(struct openemc_adc *adc)
{
	if (!adc->sample.acquired ||
	    ktime_ms_delta(ktime_get(), adc->sample.acquired) >=
		    ACQUIRE_INTERVAL_MS)
		return openemc_adc_acquire(adc);
	else
		return 0;
}

umode_t openemc_adc_hwmon_is_visible(const void *drvdata,
				     enum hwmon_sensor_types type, u32 attr,
				     int ch)
{
	const struct openemc_adc *adc = drvdata;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_label:
		case hwmon_in_input:
			if (ch < 16) {
				if (adc->labels[ch] && strlen(adc->labels[ch]))
					return 0444;
				else
					return 0;
			} else {
				return 0444;
			}
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_label:
		case hwmon_temp_input:
			return 0444;
		}
		break;
	default:
		break;
	};

	return 0;
}

static int openemc_adc_hwmon_read(struct device *dev,
				  enum hwmon_sensor_types type, u32 attr,
				  int ch, long *val)
{
	struct openemc_adc *adc = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&adc->lock);

	ret = openemc_adc_acquire_if_too_old(adc);
	if (ret < 0)
		goto out;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			if (ch < N_CHANNELS)
				*val = adc->sample.v[ch];
			else
				*val = adc->sample.vref;
			break;
		default:
			ret = -EOPNOTSUPP;
			goto out;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			*val = adc->sample.temp;
			break;
		default:
			ret = -EOPNOTSUPP;
			goto out;
		}
		break;
	default:
		break;
	};

out:
	mutex_unlock(&adc->lock);
	return ret;
}

static int openemc_adc_hwmon_read_string(struct device *dev,
					 enum hwmon_sensor_types type, u32 attr,
					 int ch, const char **str)
{
	struct openemc_adc *adc = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_label:
			if (ch < N_CHANNELS) {
				if (adc->labels[ch])
					*str = adc->labels[ch];
				else
					*str = "";
				break;
			} else {
				*str = "V_ref";
				break;
			}
		default:
			return -EOPNOTSUPP;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_label:
			*str = "T_emc";
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	default:
		break;
	};

	return 0;
}

static const struct hwmon_channel_info *openemc_adc_hwmon_info[] = {
	HWMON_CHANNEL_INFO(
		in, /* Ch0-15 */ HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL,
		/* Ch16: Vref */ HWMON_I_INPUT | HWMON_I_LABEL),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_LABEL), NULL
};

static const struct hwmon_ops openemc_adc_hwmon_ops = {
	.is_visible = openemc_adc_hwmon_is_visible,
	.read = openemc_adc_hwmon_read,
	.read_string = openemc_adc_hwmon_read_string,
};

static const struct hwmon_chip_info openemc_adc_hwmon_chip_info = {
	.ops = &openemc_adc_hwmon_ops,
	.info = openemc_adc_hwmon_info,
};

static int openemc_adc_iio_read_raw(struct iio_dev *iio_dev,
				    struct iio_chan_spec const *chan, int *val,
				    int *val2, long info)
{
	struct openemc_iio *iio = iio_priv(iio_dev);
	struct openemc_adc *adc = iio->adc;
	int ret = 0;

	mutex_lock(&adc->lock);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = openemc_adc_acquire_if_too_old(adc);
		if (ret < 0)
			goto out;

		if (chan->channel < N_CHANNELS)
			*val = adc->sample.v[chan->channel];
		else if (chan->channel == N_CHANNELS)
			*val = adc->sample.vref;
		else
			*val = adc->sample.temp;

		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&adc->lock);
	return ret;
}

static irqreturn_t openemc_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct openemc_iio *iio = iio_priv(indio_dev);
	struct openemc_adc *adc = iio->adc;
	int i, ch, ret;
	struct {
		u32 values[N_CHANNELS + 2];
		int64_t timestamp;
	} data __aligned(8);

	mutex_lock(&adc->lock);

	ret = openemc_adc_acquire(adc);
	if (ret < 0) {
		dev_err(adc->dev, "failed to acquire ADC data: %d\n", ret);
		goto out;
	}

	i = 0;
	memset(&data, 0, sizeof(data));
	for_each_set_bit (ch, indio_dev->active_scan_mask,
			  indio_dev->masklength) {
		if (ch < N_CHANNELS)
			data.values[i] = adc->sample.v[ch];
		else if (ch == N_CHANNELS)
			data.values[i] = adc->sample.vref;
		else
			data.values[i] = adc->sample.temp;

		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &data,
					   iio_get_time_ns(indio_dev));
out:
	iio_trigger_notify_done(indio_dev->trig);

	mutex_unlock(&adc->lock);

	return IRQ_HANDLED;
}

static const struct iio_info openemc_adc_iio_info = {
	.read_raw = &openemc_adc_iio_read_raw,
};

static const char *openemc_adc_default_labels[] = {
	"PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7",
	"PB0", "PB1", "PC0", "PC1", "PC2", "PC3", "PC4", "PC5"
};

static int openemc_adc_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct openemc *emc;
	struct openemc_adc *adc;
	struct openemc_iio *iio;
	struct device *hwmon;
	struct iio_dev *indio_dev;
	struct iio_chan_spec *iio_chs;
	int ch, ret;

	match = of_match_node(openemc_adc_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	emc = dev_get_drvdata(pdev->dev.parent);

	adc = devm_kzalloc(&pdev->dev, sizeof(*adc), GFP_KERNEL);
	if (!adc)
		return -ENOMEM;

	mutex_init(&adc->lock);
	adc->dev = &pdev->dev;
	adc->emc = emc;

	ret = of_property_read_string_array(pdev->dev.of_node, "labels",
					    adc->labels, N_CHANNELS);
	if (ret < 0) {
		for (ch = 0; ch < N_CHANNELS; ch++)
			adc->labels[ch] = openemc_adc_default_labels[ch];
	}

	hwmon = devm_hwmon_device_register_with_info(
		adc->dev, pdev->name, adc, &openemc_adc_hwmon_chip_info, NULL);
	if (IS_ERR(hwmon)) {
		ret = PTR_ERR(hwmon);
		dev_err(adc->dev, "failed to register hwmon device: %d\n", ret);
		return ret;
	}

	indio_dev = devm_iio_device_alloc(adc->dev, sizeof(*iio));
	if (!indio_dev) {
		dev_err(adc->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	iio = iio_priv(indio_dev);
	iio->adc = adc;

	iio_chs = devm_kzalloc(&pdev->dev,
			       sizeof(struct iio_chan_spec) * (N_CHANNELS + 3),
			       GFP_KERNEL);
	if (!iio_chs)
		return -ENOMEM;

	for (ch = 0; ch < N_CHANNELS; ch++) {
		iio_chs[ch].type = IIO_VOLTAGE;
		iio_chs[ch].indexed = 1;
		iio_chs[ch].channel = ch;
		iio_chs[ch].scan_index = ch;
		iio_chs[ch].scan_type.sign = 'u';
		iio_chs[ch].scan_type.realbits = 16;
		iio_chs[ch].scan_type.storagebits = 32;
		iio_chs[ch].scan_type.endianness = IIO_CPU;
		iio_chs[ch].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		iio_chs[ch].datasheet_name = openemc_adc_default_labels[ch];
		iio_chs[ch].extend_name = adc->labels[ch];
	}

	iio_chs[N_CHANNELS].type = IIO_VOLTAGE;
	iio_chs[N_CHANNELS].indexed = 1;
	iio_chs[N_CHANNELS].channel = N_CHANNELS;
	iio_chs[N_CHANNELS].scan_index = N_CHANNELS;
	iio_chs[N_CHANNELS].scan_type.sign = 'u';
	iio_chs[N_CHANNELS].scan_type.realbits = 16;
	iio_chs[N_CHANNELS].scan_type.storagebits = 32;
	iio_chs[N_CHANNELS].scan_type.endianness = IIO_CPU;
	iio_chs[N_CHANNELS].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	iio_chs[N_CHANNELS].datasheet_name = "V_ref";
	iio_chs[N_CHANNELS].extend_name = "V_ref";

	iio_chs[N_CHANNELS + 1].type = IIO_TEMP;
	iio_chs[N_CHANNELS + 1].indexed = 1;
	iio_chs[N_CHANNELS + 1].channel = N_CHANNELS + 1;
	iio_chs[N_CHANNELS + 1].scan_index = N_CHANNELS + 1;
	iio_chs[N_CHANNELS + 1].scan_type.sign = 's';
	iio_chs[N_CHANNELS + 1].scan_type.realbits = 32;
	iio_chs[N_CHANNELS + 1].scan_type.storagebits = 32;
	iio_chs[N_CHANNELS + 1].scan_type.endianness = IIO_CPU;
	iio_chs[N_CHANNELS + 1].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	iio_chs[N_CHANNELS + 1].datasheet_name = "T_emc";
	iio_chs[N_CHANNELS + 1].extend_name = "T_emc";

	iio_chs[N_CHANNELS + 2].type = IIO_TIMESTAMP;
	iio_chs[N_CHANNELS + 2].channel = -1;
	iio_chs[N_CHANNELS + 2].scan_index = N_CHANNELS + 2;
	iio_chs[N_CHANNELS + 2].scan_type.sign = 's';
	iio_chs[N_CHANNELS + 2].scan_type.realbits = 64;
	iio_chs[N_CHANNELS + 2].scan_type.storagebits = 64;

	indio_dev->name = "openemc_adc";
	indio_dev->info = &openemc_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = iio_chs;
	indio_dev->num_channels = N_CHANNELS + 3;

	ret = devm_iio_triggered_buffer_setup(adc->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &openemc_adc_trigger_handler,
					      NULL);
	if (ret < 0)
		return ret;

	ret = devm_iio_device_register(adc->dev, indio_dev);
	if (ret < 0) {
		dev_err(adc->dev, "failed to register iio device: %d\n", ret);
		return ret;
	}

	dev_info(adc->dev, "OpenEMC ADC registered\n");

	return 0;
}

static struct platform_driver openemc_adc_driver = {
	.driver = {
		.name = "openemc_adc",
		.of_match_table = of_match_ptr(openemc_adc_of_match),
	},
	.probe = openemc_adc_probe,
};
module_platform_driver(openemc_adc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("OpenEMC ADC");
MODULE_VERSION("0.1");
