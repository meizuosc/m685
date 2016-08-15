/* The Sensor sysfs
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SENSOR_SYSFS_H_
#define _SENSOR_SYSFS_H_

#include <linux/device.h>

ssize_t sensor_dev_sysfs_get_self_test(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_get_wakeup(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_set_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_get_x_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_y_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_z_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_set_x_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_set_y_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_set_z_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_get_x_offset(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_y_offset(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_z_offset(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_calibrate(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_set_calibrate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_get_x_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_y_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_z_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_x_threshold(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_y_threshold(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_z_threshold(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_set_x_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_set_y_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_set_z_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t sensor_dev_sysfs_get_name(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_irq_gpio(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_version(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_get_debug(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sensor_dev_sysfs_set_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

int meizu_sensor_sysfs_create(struct device *dev, const char *name);
void meizu_sensor_sysfs_destory(struct device *dev, const char *name);

#endif /* _SENSOR_SYSFS_H_ */