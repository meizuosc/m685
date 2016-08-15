/* The Sensor sysfs
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/module.h>
#include "sensor-core.h"
#include "sensor-iio.h"

/* -- sensors sysfs interfaces -- */

/* self test interface */
ssize_t sensor_dev_sysfs_get_self_test(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->self_test) {
		mutex_lock(&sdev->mlock);
		ret = ops->self_test(sdev);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "self test result: %d\n", ret);
	} else {
		dev_err(dev, "self_test not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return sprintf(buf, "%d\n", ret);
}



/* enable interface */
ssize_t sensor_dev_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int state;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_enable) {
		mutex_lock(&sdev->mlock);
		ret = ops->get_enable(sdev, &state);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "get enable state: %d, ret: %d\n", state, ret);
	} else {
		dev_err(dev, "get_enable not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return sprintf(buf, "%d\n", ret < 0 ? ret : !!state);
}

ssize_t sensor_dev_sysfs_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int state;
	const char * version = NULL;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	ret = kstrtoint(buf, 10, &state);
	if (ret < 0) {
		dev_err(dev, "kstrtoint error in %s\n", __func__);
		return ret;
	}

	mutex_lock(&sdev->mlock);
	if (ops->set_enable) {
		if (!state && !(sdev->flag & SENSOR_DEV_FLAG_DISABLE_IIO))
			sensor_iio_disable(sdev);

		ret = ops->set_enable(sdev, state);

		if (!ret && state
			&& !(sdev->flag & SENSOR_DEV_FLAG_DISABLE_IIO))
			sensor_iio_enable(sdev);

		dev_info(dev, "set enable state: %d, ret: %d\n", state, ret);
	} else {
		dev_err(dev, "set_enable not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	if (ops->get_version) {
		ops->get_version(sdev, &version);
		dev_info(dev, "name: %s, version: %s\n", sdev->name, version);
	}
	mutex_unlock(&sdev->mlock);

	return ret < 0 ? ret : count;
}


/* wakeup interface */
ssize_t sensor_dev_sysfs_get_wakeup(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int state;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_wakeup) {
		mutex_lock(&sdev->mlock);
		ret = ops->get_wakeup(sdev, &state);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "get wakeup state: %d, ret: %d\n", state, ret);
	} else {
		dev_err(dev, "get_wakeup not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return sprintf(buf, "%d\n", ret < 0 ? ret : !!state);
}

ssize_t sensor_dev_sysfs_set_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int state;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	ret = kstrtoint(buf, 10, &state);
	if (ret < 0) {
		dev_err(dev, "kstrtoint error in %s\n", __func__);
		return ret;
	}

	if (ops->set_wakeup) {
		mutex_lock(&sdev->mlock);
		ret = ops->set_wakeup(sdev, state);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "set wakeup state: %d, ret: %d\n", state, ret);
	} else {
		dev_err(dev, "set_wakeup not implemented in sensor ops\n");
		ret = -ENOSYS;
	}


	return ret < 0 ? ret : count;
}


/* calibbias interfaces */
static int sensor_get_calibbias(struct device *dev, int32_t *bias)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_calibbias) {
		ret = ops->get_calibbias(sdev, bias);
		dev_info(dev, "get calibbias: %d, %d, %d, ret: %d\n",
			bias[0], bias[1], bias[2], ret);
	} else {
		dev_err(dev, "get_calibbias not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret;
}

ssize_t sensor_dev_sysfs_get_x_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t bias[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_calibbias(dev, bias);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get x calibbias: %d\n", bias[0]);
	return sprintf(buf, "%d\n", bias[0]);
}

ssize_t sensor_dev_sysfs_get_y_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t bias[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_calibbias(dev, bias);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get y calibbias: %d\n", bias[1]);
	return sprintf(buf, "%d\n", bias[1]);
}

ssize_t sensor_dev_sysfs_get_z_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t bias[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_calibbias(dev, bias);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get z calibbias: %d\n", bias[2]);
	return sprintf(buf, "%d\n", bias[2]);
}



/* offset interfaces */
static int sensor_set_offset(struct device *dev,
	const char *buf, int offset, int axis)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	/* when buf is not NULL, we use the offset in the buf */
	if (buf != NULL) {
		ret = kstrtoint(buf, 10, &offset);
		if (ret < 0) {
			dev_err(dev, "kstrtoint error in %s\n", __func__);
			return ret;
		}
	}

	if (ops->set_offset) {
		ret = ops->set_offset(sdev, offset, axis);
		dev_info(dev, "set offset[%d]: %d, ret: %d\n",
			axis, offset, ret);
	} else {
		dev_err(dev, "set_offset not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret;
}

ssize_t sensor_dev_sysfs_set_x_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_set_offset(dev, buf, 0, 0);
	mutex_unlock(&sdev->mlock);

	return ret < 0 ? ret : count;
}

ssize_t sensor_dev_sysfs_set_y_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_set_offset(dev, buf, 0, 1);
	mutex_unlock(&sdev->mlock);

	return ret < 0 ? ret : count;
}

ssize_t sensor_dev_sysfs_set_z_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_set_offset(dev, buf, 0, 2);
	mutex_unlock(&sdev->mlock);

	return ret < 0 ? ret : count;
}

static int sensor_get_offset(struct device *dev,
	int32_t *offset)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_offset) {
		ret = ops->get_offset(sdev, offset);
		dev_info(dev, "get offset: %d, %d, %d, ret: %d\n",
			offset[0], offset[1], offset[2], ret);
	} else {
		dev_err(dev, "get_offset not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret;
}

ssize_t sensor_dev_sysfs_get_x_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t offset[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_offset(dev, offset);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get x offset: %d\n", offset[0]);
	return sprintf(buf, "%d\n", offset[0]);
}

ssize_t sensor_dev_sysfs_get_y_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t offset[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_offset(dev, offset);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get y offset: %d\n", offset[1]);
	return sprintf(buf, "%d\n", offset[1]);
}

ssize_t sensor_dev_sysfs_get_z_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t offset[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_offset(dev, offset);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get z offset: %d\n", offset[2]);
	return sprintf(buf, "%d\n", offset[2]);
}

/* calibrate interfaces */
static int sensor_do_calibrate(struct device *dev)
{
	int ret;
	int32_t bias[3] = {0,0,0};
	int32_t offset[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->calibrate) {

		sensor_get_offset(dev, offset);

		sensor_set_offset(dev, NULL, 0, 0);
		sensor_set_offset(dev, NULL, 0, 1);
		sensor_set_offset(dev, NULL, 0, 2);

		ret = ops->calibrate(sdev);
		dev_info(dev, "calibrate result: %d\n", ret);
		if (ret < 0) {
			dev_info(dev, "calibrate failed, "
						"reset offset\n");
			sensor_set_offset(dev, NULL, offset[0], 0);
			sensor_set_offset(dev, NULL, offset[1], 1);
			sensor_set_offset(dev, NULL, offset[2], 2);
		}

	} else {
		dev_err(dev, "calibrate not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	if ((ret >= 0) && (sdev->flag & SENSOR_DEV_FLAG_OFFSET_AFTER_CALIB)) {
		/* apply the bias immediately if required */
		dev_info(dev, "apply the bias after calibrate\n");
		sensor_get_calibbias(dev, bias);
		sensor_set_offset(dev, NULL, bias[0], 0);
		sensor_set_offset(dev, NULL, bias[1], 1);
		sensor_set_offset(dev, NULL, bias[2], 2);
	}

	return ret;
}

ssize_t sensor_dev_sysfs_get_calibrate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_do_calibrate(dev);
	mutex_unlock(&sdev->mlock);

	return sprintf(buf, "%d\n", ret);
}



ssize_t sensor_dev_sysfs_set_calibrate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int state;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	ret = kstrtoint(buf, 10, &state);
	if (ret < 0) {
		dev_err(dev, "kstrtoint error in %s\n", __func__);
		return ret;
	}

	if (state != 1) {
		dev_err(dev, "only write 1 is valid when set calibrate\n");
		return -EINVAL;
	}

	mutex_lock(&sdev->mlock);
	ret = sensor_do_calibrate(dev);
	mutex_unlock(&sdev->mlock);

	return ret < 0 ? ret : count;
}



/* raw data interfaces */
static int sensor_get_raw_data(struct device *dev,
	int32_t *raw)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_raw_data) {
		ret = ops->get_raw_data(sdev, raw);
		dev_info(dev, "get raw data: %d, %d, %d, ret: %d\n",
			raw[0], raw[1], raw[2], ret);
	} else {
		dev_err(dev, "get_raw_data not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret;
}

ssize_t sensor_dev_sysfs_get_x_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t raw[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_raw_data(dev, raw);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get x raw data: %d\n", raw[0]);
	return sprintf(buf, "%d\n", raw[0]);
}

ssize_t sensor_dev_sysfs_get_y_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t raw[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_raw_data(dev, raw);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get y raw data: %d\n", raw[1]);
	return sprintf(buf, "%d\n", raw[1]);
}

ssize_t sensor_dev_sysfs_get_z_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t raw[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_raw_data(dev, raw);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get z raw data: %d\n", raw[2]);
	return sprintf(buf, "%d\n", raw[2]);
}

/* threshold interfaces */
static int sensor_get_threshold(struct device *dev,
	int32_t *threshold)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_threshold) {
		ret = ops->get_threshold(sdev, threshold);
		dev_info(dev, "get threshold: %d, %d, %d, ret: %d\n",
			threshold[0], threshold[1], threshold[2], ret);
	} else {
		dev_err(dev, "get_threshold not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret;
}

ssize_t sensor_dev_sysfs_get_x_threshold(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t threshold[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_threshold(dev, threshold);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get x threshold: %d\n", threshold[0]);
	return sprintf(buf, "%d\n", threshold[0]);
}

ssize_t sensor_dev_sysfs_get_y_threshold(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t threshold[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_threshold(dev, threshold);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get y threshold: %d\n", threshold[1]);
	return sprintf(buf, "%d\n", threshold[1]);
}

ssize_t sensor_dev_sysfs_get_z_threshold(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t threshold[3] = {0,0,0};
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_get_threshold(dev, threshold);
	mutex_unlock(&sdev->mlock);
	if (ret < 0)
		return ret;

	dev_info(dev, "get z threshold: %d\n", threshold[2]);
	return sprintf(buf, "%d\n", threshold[2]);
}

static int sensor_set_threshold(struct device *dev,
	const char *buf, int threshold, int axis)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (buf != NULL) {
		ret = kstrtoint(buf, 10, &threshold);
		if (ret < 0) {
			dev_err(dev, "kstrtoint error in %s\n", __func__);
			return ret;
		}
	}

	if (ops->set_threshold) {
		ret = ops->set_threshold(sdev, threshold, axis);
		dev_info(dev, "set threshold[%d]: %d, ret: %d\n",
			axis, threshold, ret);
	} else {
		dev_err(dev, "set_threshold not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret;
}

ssize_t sensor_dev_sysfs_set_x_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_set_threshold(dev, buf, 0, 0);
	mutex_unlock(&sdev->mlock);
	return ret < 0 ? ret : count;
}

ssize_t sensor_dev_sysfs_set_y_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_set_threshold(dev, buf, 0, 1);
	mutex_unlock(&sdev->mlock);

	return ret < 0 ? ret : count;
}

ssize_t sensor_dev_sysfs_set_z_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	mutex_lock(&sdev->mlock);
	ret = sensor_set_threshold(dev, buf, 0, 2);
	mutex_unlock(&sdev->mlock);

	return ret < 0 ? ret : count;
}

/* name interfaces */
ssize_t sensor_dev_sysfs_get_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);

	dev_info(dev, "get name: %s\n", sdev->name);
	return sprintf(buf, "%s\n", sdev->name);
}

ssize_t sensor_dev_sysfs_get_irq_gpio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int state = 0;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_irq_gpio) {
		mutex_lock(&sdev->mlock);
		ret = ops->get_irq_gpio(sdev, &state);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "get irq gpio, state: %d, ret: %d\n",
			state, ret);
	} else {
		dev_err(dev, "get_irq_gpio not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", state);
}

ssize_t sensor_dev_sysfs_get_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	const char * version = NULL;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_version) {
		mutex_lock(&sdev->mlock);
		ret = ops->get_version(sdev, &version);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "get version: %s, ret: %d\n",
			version, ret);
	} else {
		dev_err(dev, "get_version not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n", version);
}

ssize_t sensor_dev_sysfs_get_debug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->get_debug) {
		mutex_lock(&sdev->mlock);
		ret = ops->get_debug(sdev, buf);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "get debug: %s, ret: %d\n", buf, ret);
	} else {
		dev_err(dev, "get_debug not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret;
}

ssize_t sensor_dev_sysfs_set_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct sensor_dev *sdev = dev_to_sensor_dev(dev);
	const struct sensor_ops *ops = sdev->ops;

	if (ops->set_debug) {
		mutex_lock(&sdev->mlock);
		ret = ops->set_debug(sdev, buf, count);
		mutex_unlock(&sdev->mlock);
		dev_info(dev, "set debug: %s, ret: %d\n", buf, ret);
	} else {
		dev_err(dev, "set_debug not implemented in sensor ops\n");
		ret = -ENOSYS;
	}

	return ret < 0 ? ret : count;
}

#define USING_MEIZU_SYSFS_LINK_INTERFACE 1

#if USING_MEIZU_SYSFS_LINK_INTERFACE
extern int meizu_sysfslink_register_name(struct device *dev, char *name);
#else
static struct class *meizu_sensor_class;
static struct kset  *meizu_sensor_class_kset;
#endif
/*
 * Create link ysfs interface
 */
int meizu_sensor_sysfs_create(struct device *dev, const char *name)
{
#if USING_MEIZU_SYSFS_LINK_INTERFACE
	return meizu_sysfslink_register_name(dev, (char *)name);
#else

	int ret = 0;

	if(!dev || !meizu_sensor_class_kset)
		return -EINVAL;

	if(!name)
		name = dev->driver->name;

	ret = sysfs_create_link(&meizu_sensor_class_kset->kobj,
				&dev->kobj, name);
	if (ret < 0) {
		pr_err("%s()->%d:can not create sysfs link!\n",
			__func__, __LINE__);
		return ret;
	}

	pr_info("meizu_sensor_sysfs_create [%s] success\n", name);
	return 0;

#endif
}
EXPORT_SYMBOL(meizu_sensor_sysfs_create);

void meizu_sensor_sysfs_destory(struct device *dev, const char *name)
{

#if USING_MEIZU_SYSFS_LINK_INTERFACE
	/* We do nothing here */
#else
	if (!dev || !meizu_sensor_class_kset)
		return;

	sysfs_delete_link(&meizu_sensor_class_kset->kobj,
				&dev->kobj, name);
#endif
}
EXPORT_SYMBOL(meizu_sensor_sysfs_destory);


static int __init meizu_sensor_class_init(void)
{
#if USING_MEIZU_SYSFS_LINK_INTERFACE
	/* We do nothing here */
	return 0;
#else
	meizu_sensor_class = class_create(THIS_MODULE, "mz_sensor");
	if (IS_ERR(meizu_sensor_class)) {
		pr_err("%s, create meizu_sensor_class failed.\n",
			__func__);
		return PTR_ERR(meizu_sensor_class);
	}

	/*
	 * The struct subsys_private is defined in drivers/base/base.h
	 * and we can't reference it here. We use the tricky way to get
	 * the kset (assuming the first member of subsys_private is kset).
        */
	meizu_sensor_class_kset = (struct kset *)meizu_sensor_class->p;
	pr_info("meizu_sensor_class_init success\n");
	return 0;
#endif
}

static void __exit meizu_sensor_class_exit(void)
{
#if USING_MEIZU_SYSFS_LINK_INTERFACE
	/* We do nothing here */
#else
	class_destroy(meizu_sensor_class);
	meizu_sensor_class = NULL;
#endif
}

subsys_initcall(meizu_sensor_class_init);
module_exit(meizu_sensor_class_exit);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor sysfs");
MODULE_LICENSE("GPL");
