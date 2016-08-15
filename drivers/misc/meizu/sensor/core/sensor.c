/* The Sensor
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
#include "sensor.h"

int sensor_push_data(struct sensor_dev *sdev, uint8_t *data, int64_t timestamp)
{
	return sensor_iio_push_data(sdev, data, timestamp);
}

struct sensor_dev *sensor_create(struct sensor_info *info, void *drvdata)
{
	int ret = -1;
	struct sensor_dev *sdev = NULL;

	sdev = sensor_device_alloc(info->name);
	if (!sdev) {
		pr_err("sensor_device_alloc error, name: %s\n", info->name);
		goto err_nothing;
	}

	if (drvdata)
		sensor_device_set_drvdata(sdev, drvdata);

	sdev->ops  = info->ops;
	sdev->flag = info->flag;
	sdev->groups[sdev->groupcounter++] = info->attr_group;

	ret = sensor_device_register(sdev);
	if (ret < 0) {
		pr_err("sensor_device_register error, name: %s, ret: %d\n",
			info->name, ret);
		goto err_free_sensor_device;
	}

	if (!(info->flag & SENSOR_DEV_FLAG_DISABLE_IIO)) {
		ret = sensor_iio_register(sdev, info->iio_ch,
			info->iio_ch_cnt, info->mode);
		if (ret < 0) {
			dev_err(&sdev->dev,
				"sensor_iio_register error, ret: %d\n",
				ret);
			goto err_unregister_sensor_device;
		}
	}

	ret = meizu_sensor_sysfs_create(&sdev->dev, info->type);
	if (ret < 0) {
		dev_err(&sdev->dev,
			"meizu_sensor_sysfs_create error, ret: %d\n",
			ret);
		goto err_unregister_sensor_iio;
	}

	dev_info(&sdev->dev, "sensor_create success, name: %s\n", info->name);
	return sdev;

err_unregister_sensor_iio:
	if (!(info->flag & SENSOR_DEV_FLAG_DISABLE_IIO))
		sensor_iio_unregister(sdev);
err_unregister_sensor_device:
	sensor_device_unregister(sdev);
err_free_sensor_device:
	sensor_device_free(sdev);
err_nothing:
	pr_err("sensor_create failed, name: %s\n", info->name);
	return NULL;
}
EXPORT_SYMBOL(sensor_create);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor");
MODULE_LICENSE("GPL");
