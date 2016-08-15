/* The Sensor acc
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

#define DEV_SCP_ACC_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_attr_scp_acc_##_name = __ATTR(_name, _mode, _show, _store)

static DEV_SCP_ACC_ATTR(acc_enable, 0660,
		sensor_dev_sysfs_get_enable,
		sensor_dev_sysfs_set_enable);

static DEV_SCP_ACC_ATTR(acc_self_test, 0440,
		sensor_dev_sysfs_get_self_test,
		NULL);

static DEV_SCP_ACC_ATTR(acc_calibration, 0660,
		sensor_dev_sysfs_get_calibrate,
		sensor_dev_sysfs_set_calibrate);

static DEV_SCP_ACC_ATTR(acc_x_calibbias, 0440,
		sensor_dev_sysfs_get_x_calibbias,
		NULL);

static DEV_SCP_ACC_ATTR(acc_x_offset, 0664,
		sensor_dev_sysfs_get_x_offset,
		sensor_dev_sysfs_set_x_offset);

static DEV_SCP_ACC_ATTR(acc_y_calibbias, 0440,
		sensor_dev_sysfs_get_y_calibbias,
		NULL);

static DEV_SCP_ACC_ATTR(acc_y_offset, 0664,
		sensor_dev_sysfs_get_y_offset,
		sensor_dev_sysfs_set_y_offset);

static DEV_SCP_ACC_ATTR(acc_z_calibbias, 0440,
		sensor_dev_sysfs_get_z_calibbias,
		NULL);

static DEV_SCP_ACC_ATTR(acc_z_offset, 0664,
		sensor_dev_sysfs_get_z_offset,
		sensor_dev_sysfs_set_z_offset);

static DEV_SCP_ACC_ATTR(name, 0444,
		sensor_dev_sysfs_get_name,
		NULL);

static DEV_SCP_ACC_ATTR(version, 0444,
		sensor_dev_sysfs_get_version,
		NULL);

static DEV_SCP_ACC_ATTR(debug, 0660,
		sensor_dev_sysfs_get_debug,
		sensor_dev_sysfs_set_debug);

static struct attribute *sensor_scp_acc_attrs[] = {
	&dev_attr_scp_acc_acc_enable.attr,
	&dev_attr_scp_acc_acc_self_test.attr,
	&dev_attr_scp_acc_acc_calibration.attr,
	&dev_attr_scp_acc_acc_x_calibbias.attr,
	&dev_attr_scp_acc_acc_x_offset.attr,
	&dev_attr_scp_acc_acc_y_calibbias.attr,
	&dev_attr_scp_acc_acc_y_offset.attr,
	&dev_attr_scp_acc_acc_z_calibbias.attr,
	&dev_attr_scp_acc_acc_z_offset.attr,
	&dev_attr_scp_acc_name.attr,
	&dev_attr_scp_acc_version.attr,
	&dev_attr_scp_acc_debug.attr,
	NULL
};

static struct attribute_group sensor_scp_acc_attr_group = {
	.attrs = sensor_scp_acc_attrs,
};


struct sensor_dev * sensor_scp_acc_create(const char *name,
	const struct sensor_ops *ops, void *drvdata)
{

	struct sensor_info info = {
		.name = name,
		.type = "acc",
		.mode = SENSOR_IIO_MODE_POLLING,
		.attr_group = &sensor_scp_acc_attr_group,
		.iio_ch_cnt = 0,
		.iio_ch = NULL,
		.ops = ops,
		.flag = SENSOR_DEV_FLAG_OFFSET_AFTER_CALIB
		      | SENSOR_DEV_FLAG_DISABLE_IIO
	};
	return sensor_create(&info, drvdata);
}
EXPORT_SYMBOL(sensor_scp_acc_create);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor acc");
MODULE_LICENSE("GPL");
