/* The Sensor ps
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/stat.h>
#include "sensor-core.h"
#include "sensor-sysfs.h"
#include "sensor-iio.h"


#define SENSOR_IIO_PS_CHANNELS(device_type, index, mod, endian, bits, addr) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = bits, \
		.shift = 0, \
		.storagebits = bits, \
		.endianness = endian, \
	}, \
}

static const struct iio_chan_spec sensor_iio_ps_ch[] = {
	/* adc raw data */
	SENSOR_IIO_PS_CHANNELS(IIO_PROXIMITY, 0, IIO_MOD_X, IIO_LE,
					16, 0),
	/* near/far status */
	SENSOR_IIO_PS_CHANNELS(IIO_PROXIMITY, 1, IIO_MOD_Y, IIO_LE,
					16, 1),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static DEVICE_ATTR(ps_enable, 0660,
		sensor_dev_sysfs_get_enable,
		sensor_dev_sysfs_set_enable);

static DEVICE_ATTR(ps_wakeup_enable, 0660,
		sensor_dev_sysfs_get_wakeup,
		sensor_dev_sysfs_set_wakeup);

static DEVICE_ATTR(ps_data, 0440,
		sensor_dev_sysfs_get_x_raw_data,
		NULL);

static DEVICE_ATTR(ps_calibration, 0660,
		sensor_dev_sysfs_get_calibrate,
		sensor_dev_sysfs_set_calibrate);

static DEVICE_ATTR(ps_calibbias, 0440,
		sensor_dev_sysfs_get_x_calibbias,
		NULL);

static DEVICE_ATTR(ps_offset, 0664,
		sensor_dev_sysfs_get_x_offset,
		sensor_dev_sysfs_set_x_offset);

static DEVICE_ATTR(ps_threshold, 0664,
		sensor_dev_sysfs_get_x_threshold,
		sensor_dev_sysfs_set_x_threshold);

static DEVICE_ATTR(name, 0444,
		sensor_dev_sysfs_get_name,
		NULL);

static DEVICE_ATTR(ps_irq_gpio, 0440,
		sensor_dev_sysfs_get_irq_gpio,
		NULL);

static DEVICE_ATTR(version, 0444,
		sensor_dev_sysfs_get_version,
		NULL);

static DEVICE_ATTR(debug, 0660,
		sensor_dev_sysfs_get_debug,
		sensor_dev_sysfs_set_debug);

static struct attribute *sensor_ps_attrs[] = {
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_wakeup_enable.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_ps_calibration.attr,
	&dev_attr_ps_calibbias.attr,
	&dev_attr_ps_offset.attr,
	&dev_attr_ps_threshold.attr,
	&dev_attr_name.attr,
	&dev_attr_ps_irq_gpio.attr,
	&dev_attr_version.attr,
	&dev_attr_debug.attr,
	NULL
};

static struct attribute_group sensor_ps_attr_group = {
	.attrs = sensor_ps_attrs,
};


struct sensor_dev * sensor_ps_create(const char *name,
	const struct sensor_ops *ops, void *drvdata)
{

	struct sensor_info info = {
		.name = name,
		.type = "ps",
		.mode = SENSOR_IIO_MODE_INTERRUPT,
		.attr_group = &sensor_ps_attr_group,
		.iio_ch_cnt = ARRAY_SIZE(sensor_iio_ps_ch),
		.iio_ch = sensor_iio_ps_ch,
		.ops = ops,
		.flag = SENSOR_DEV_FLAG_OFFSET_AFTER_CALIB
	};
	return sensor_create(&info, drvdata);
}
EXPORT_SYMBOL(sensor_ps_create);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor ps");
MODULE_LICENSE("GPL");
