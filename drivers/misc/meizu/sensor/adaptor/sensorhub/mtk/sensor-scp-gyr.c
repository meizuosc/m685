/* The Sensor gry
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

#define DEV_SCP_GYR_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_scp_gyr_##_name = __ATTR(_name, _mode, _show, _store)

static DEV_SCP_GYR_ATTR(gyr_enable, 0660,
		sensor_dev_sysfs_get_enable,
		sensor_dev_sysfs_set_enable);

static DEV_SCP_GYR_ATTR(gyr_self_test, 0440,
		sensor_dev_sysfs_get_self_test,
		NULL);

static DEV_SCP_GYR_ATTR(name, 0444,
		sensor_dev_sysfs_get_name,
		NULL);

static DEV_SCP_GYR_ATTR(version, 0444,
		sensor_dev_sysfs_get_version,
		NULL);

static DEV_SCP_GYR_ATTR(debug, 0660,
		sensor_dev_sysfs_get_debug,
		sensor_dev_sysfs_set_debug);

static struct attribute *sensor_scp_gyr_attrs[] = {
	&dev_attr_scp_gyr_gyr_enable.attr,
	&dev_attr_scp_gyr_gyr_self_test.attr,
	&dev_attr_scp_gyr_name.attr,
	&dev_attr_scp_gyr_version.attr,
	&dev_attr_scp_gyr_debug.attr,
	NULL
};

static struct attribute_group sensor_scp_gyr_attr_group = {
	.attrs = sensor_scp_gyr_attrs,
};


struct sensor_dev * sensor_scp_gyr_create(const char *name,
	const struct sensor_ops *ops, void *drvdata)
{

	struct sensor_info info = {
		.name = name,
		.type = "gyr",
		.mode = SENSOR_IIO_MODE_POLLING,
		.attr_group = &sensor_scp_gyr_attr_group,
		.iio_ch_cnt = 0,
		.iio_ch = NULL,
		.ops = ops,
		.flag = SENSOR_DEV_FLAG_DISABLE_IIO
	};
	return sensor_create(&info, drvdata);
}
EXPORT_SYMBOL(sensor_scp_gyr_create);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor gyr");
MODULE_LICENSE("GPL");
