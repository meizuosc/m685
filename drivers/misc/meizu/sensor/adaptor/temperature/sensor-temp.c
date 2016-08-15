/* The Sensor Temperature
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

#define SENSOR_IIO_TEMP_CHANNELS(device_type, index, mod, endian, bits, addr) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = 's', \
		.realbits = bits, \
		.shift = 0, \
		.storagebits = bits, \
		.endianness = endian, \
	}, \
}

#define SENSOR_IIO_TEMP_X_L_ADDR		0x01
static const struct iio_chan_spec sensor_iio_temp_ch[] = {
	SENSOR_IIO_TEMP_CHANNELS(IIO_TEMP, 0, IIO_MOD_X, IIO_LE,
					16, SENSOR_IIO_TEMP_X_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static DEVICE_ATTR(temp_enable, 0660,
		sensor_dev_sysfs_get_enable,
		sensor_dev_sysfs_set_enable);

static DEVICE_ATTR(temp_data, S_IRUGO,
		sensor_dev_sysfs_get_x_raw_data,
		NULL);

static DEVICE_ATTR(name, S_IRUGO,
		sensor_dev_sysfs_get_name,
		NULL);

static struct attribute *sensor_temp_attrs[] = {
	&dev_attr_temp_enable.attr,
	&dev_attr_temp_data.attr,
	&dev_attr_name.attr,
	NULL
};

static struct attribute_group sensor_temp_attr_group = {
	.attrs = sensor_temp_attrs,
};


struct sensor_dev *sensor_temp_create(const char *name,
	const struct sensor_ops *ops, void *drvdata)
{
	struct sensor_info info = {
		.name = name,
		.type = "temp",
		.mode = SENSOR_IIO_MODE_POLLING,
		.attr_group = &sensor_temp_attr_group,
		.iio_ch_cnt = ARRAY_SIZE(sensor_iio_temp_ch),
		.iio_ch = sensor_iio_temp_ch,
		.ops = ops
	};
	return sensor_create(&info, drvdata);
}
EXPORT_SYMBOL(sensor_temp_create);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor Pressure");
MODULE_LICENSE("GPL");
