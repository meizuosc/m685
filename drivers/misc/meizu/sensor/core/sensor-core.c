/* The Sensor core
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include "sensor-core.h"

struct bus_type sensor_bus_type = {
	.name = "sensor",
};
EXPORT_SYMBOL(sensor_bus_type);

static void sensor_dev_release(struct device *dev)
{
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	kfree(sdev);
}

struct device_type sensor_device_type = {
	.name = "sensor_device",
	.release = sensor_dev_release,
};

#define SENSOR_DEVICE_NAME_MAX_LEN 50
struct sensor_dev *sensor_device_alloc(const char *name)
{
	struct sensor_dev *sdev;

	if (!name)
		return NULL;

	if (strlen(name) > SENSOR_DEVICE_NAME_MAX_LEN) {
		printk(KERN_ERR "the length of name is too long, name: %s",
			name);
		return NULL;
	}

	sdev = kzalloc(sizeof(struct sensor_dev), GFP_KERNEL);
	if (sdev) {
		sdev->dev.type   = &sensor_device_type;
		sdev->dev.bus    = &sensor_bus_type;
		sdev->dev.groups = sdev->groups;
		device_initialize(&sdev->dev);
		mutex_init(&sdev->mlock);

		dev_set_name(&sdev->dev, "%s", name);
		sdev->name = name;
	}

	return sdev;
}
EXPORT_SYMBOL(sensor_device_alloc);

void sensor_device_free(struct sensor_dev *sdev)
{
	if (sdev)
		put_device(&sdev->dev);
}
EXPORT_SYMBOL(sensor_device_free);

static const struct sensor_ops noop_sensor_ops;

int sensor_device_register(struct sensor_dev *sdev)
{
	/* If the calling driver did not initialize of_node, do it here */
	if (!sdev->dev.of_node && sdev->dev.parent)
		sdev->dev.of_node = sdev->dev.parent->of_node;

	/* todo: regist debug fs */

	if (!sdev->ops)
		sdev->ops = &noop_sensor_ops;

	return device_add(&sdev->dev);
}
EXPORT_SYMBOL(sensor_device_register);


void sensor_device_unregister(struct sensor_dev *sdev)
{
	if (sdev)
		device_del(&sdev->dev);
}
EXPORT_SYMBOL(sensor_device_unregister);

static int __init sensor_init(void)
{
	int ret;

	/* Register sysfs bus */
	ret  = bus_register(&sensor_bus_type);
	if (ret < 0) {
		pr_err("%s could not register bus type\n",
			__FILE__);
		goto err;
	}

	pr_info("sensor_init success\n");
	return 0;

err:
	return ret;
}

static void __exit sensor_exit(void)
{
	bus_unregister(&sensor_bus_type);
}

subsys_initcall(sensor_init);
module_exit(sensor_exit);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor core");
MODULE_LICENSE("GPL");
