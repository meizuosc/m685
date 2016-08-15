/* The Sensor Pressure
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

#define SENSOR_IIO_PRES_CHANNELS(device_type, index, mod, endian, bits, addr) \
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

#define SENSOR_IIO_PRES_X_L_ADDR		0x01
#define SENSOR_IIO_TEMP_X_L_ADDR		0x02
static const struct iio_chan_spec sensor_iio_pres_ch[] = {
	SENSOR_IIO_PRES_CHANNELS(IIO_PRESSURE, 0, IIO_MOD_X, IIO_LE,
					32, SENSOR_IIO_PRES_X_L_ADDR),
	SENSOR_IIO_PRES_CHANNELS(IIO_TEMP, 1, IIO_MOD_X, IIO_LE,
					16, SENSOR_IIO_TEMP_X_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static DEVICE_ATTR(pres_enable, 0660,
		sensor_dev_sysfs_get_enable,
		sensor_dev_sysfs_set_enable);

static DEVICE_ATTR(pres_data, 0440,
		sensor_dev_sysfs_get_x_raw_data,
		NULL);

static DEVICE_ATTR(name, 0444,
		sensor_dev_sysfs_get_name,
		NULL);

static DEVICE_ATTR(pres_calibration, 0660,
		sensor_dev_sysfs_get_calibrate,
		sensor_dev_sysfs_set_calibrate);

static DEVICE_ATTR(pres_calibbias, 0440,
		sensor_dev_sysfs_get_x_calibbias,
		NULL);

static DEVICE_ATTR(pres_offset, 0660,
		sensor_dev_sysfs_get_x_offset,
		sensor_dev_sysfs_set_x_offset);

static DEVICE_ATTR(version, 0444,
		sensor_dev_sysfs_get_version,
		NULL);

static struct attribute *sensor_pres_attrs[] = {
	&dev_attr_pres_enable.attr,
	&dev_attr_pres_data.attr,
	&dev_attr_name.attr,
	&dev_attr_pres_calibration.attr,
	&dev_attr_pres_calibbias.attr,
	&dev_attr_pres_offset.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group sensor_pres_attr_group = {
	.attrs = sensor_pres_attrs,
};


struct sensor_dev *sensor_pres_create(const char *name,
	const struct sensor_ops *ops, void *drvdata)
{
	struct sensor_info info = {
		.name = name,
		.type = "pres",
		.mode = SENSOR_IIO_MODE_POLLING,
		.attr_group = &sensor_pres_attr_group,
		.iio_ch_cnt = ARRAY_SIZE(sensor_iio_pres_ch),
		.iio_ch = sensor_iio_pres_ch,
		.ops = ops
	};
	return sensor_create(&info, drvdata);
}
EXPORT_SYMBOL(sensor_pres_create);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor Pressure");
MODULE_LICENSE("GPL");
