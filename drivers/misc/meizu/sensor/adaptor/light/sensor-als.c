/* The Sensor als
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

#define SENSOR_IIO_ALS_CHANNELS(device_type, index, mod, endian, bits, addr) \
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

#define SENSOR_IIO_ALS_X_L_ADDR		0x28
static const struct iio_chan_spec sensor_iio_als_ch[] = {
	SENSOR_IIO_ALS_CHANNELS(IIO_LIGHT, 0, IIO_MOD_X, IIO_LE,
					32, SENSOR_IIO_ALS_X_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static DEVICE_ATTR(als_enable, 0660,
		sensor_dev_sysfs_get_enable,
		sensor_dev_sysfs_set_enable);

static DEVICE_ATTR(als_data, 0440,
		sensor_dev_sysfs_get_x_raw_data,
		NULL);

static DEVICE_ATTR(name, 0444,
		sensor_dev_sysfs_get_name,
		NULL);

static DEVICE_ATTR(version, 0444,
		sensor_dev_sysfs_get_version,
		NULL);

static DEVICE_ATTR(debug, 0660,
		sensor_dev_sysfs_get_debug,
		sensor_dev_sysfs_set_debug);

static struct attribute *sensor_als_attrs[] = {
	&dev_attr_als_enable.attr,
	&dev_attr_als_data.attr,
	&dev_attr_name.attr,
	&dev_attr_version.attr,
	&dev_attr_debug.attr,
	NULL
};

static struct attribute_group sensor_als_attr_group = {
	.attrs = sensor_als_attrs,
};


struct sensor_dev *sensor_als_create(const char *name,
	const struct sensor_ops *ops, void *drvdata)
{

	struct sensor_info info = {
		.name = name,
		.type = "als",
		.mode = SENSOR_IIO_MODE_POLLING,
		.attr_group = &sensor_als_attr_group,
		.iio_ch_cnt = ARRAY_SIZE(sensor_iio_als_ch),
		.iio_ch = sensor_iio_als_ch,
		.ops = ops
	};
	return sensor_create(&info, drvdata);
}
EXPORT_SYMBOL(sensor_als_create);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor als");
MODULE_LICENSE("GPL");
