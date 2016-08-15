/* The Sensor Head File
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <linux/device.h>

#define SENSOR_DEBUG 0

#if SENSOR_DEBUG
#define LOG_TAG_SENSOR "[sensor]"
#define pr_info(format, arg...)         printk(KERN_EMERG LOG_TAG_SENSOR format , ## arg)
#define pr_err(format, arg...)          printk(KERN_EMERG LOG_TAG_SENSOR format , ## arg)
#define dev_err(dev, format, arg...)    dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_info(dev, format, arg...)   dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_dbg(dev, format, arg...)    dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_warn(dev, format, arg...)   dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_notice(dev, format, arg...) dev_printk(KERN_EMERG, dev, format , ## arg)
#endif

struct sensor_dev;

/**
 * struct sensors_ops - sensors operation interfaces
 * @self_test:		Sensor self test interface, return >= 0 if success,
 *			return < 0 if failed.
 * @set_enable:		Enable or disable, state: 1 for enable, 0 for disable
 * @get_enable:		Get the sensor's enable state.
 * @set_wakeup:		Enable or disable wakeup function,
 * 				state: 1 for enable wakeup, 0 for disable wakeup
 * @get_wakeup:		Get the sensor's wakeup state.
 * @calibrate:		Perform sensor calibration.
 * @set_offset:		Set the specific axis offset of the sensor,
 *			axis:
 *				for acc and gry, 0: x axis, 1: y axis, 2:z axis
 *				for ps, only 0 is valid
 * @get_offset:		Get offset of the sensor
 * @get_calibbias:	Get the calibbias of the sensor.
 * @get_raw_data:	Get the raw data of the sensor.
 * @set_threshold:	Get the threshold of the sensor.
 * @get_threshold:	Get the threshold of the sensor.
 * @get_irq_gpio:	Get the irq gpio state
 * @get_version:	Get the driver version
 *
 * return 0 if ops success
 * return a negative error number if failed
 */
struct sensor_ops {
	int (*self_test)(struct sensor_dev *sdev);

	int (*set_enable)(struct sensor_dev *sdev, int state);
	int (*get_enable)(struct sensor_dev *sdev, int *state);

	int (*set_wakeup)(struct sensor_dev *sdev, int state);
	int (*get_wakeup)(struct sensor_dev *sdev, int *state);

	int (*calibrate)(struct sensor_dev *sdev);
	int (*set_offset)(struct sensor_dev *sdev, int offset, int axis);
	int (*get_offset)(struct sensor_dev *sdev, int32_t offset[3]);
	int (*get_calibbias)(struct sensor_dev *sdev, int32_t calibbias[3]);

	int (*get_raw_data)(struct sensor_dev *sdev, int32_t raw[3]);

	int (*set_threshold)(struct sensor_dev *sdev, int threshold, int axis);
	int (*get_threshold)(struct sensor_dev *sdev, int32_t threshold[3]);

	int (*get_irq_gpio)(struct sensor_dev *sdev, int *state);
	int (*get_version)(struct sensor_dev *sdev, const char **version);

	int (*get_max_sample_freq)(struct sensor_dev *sdev,
						int32_t *sample_freq);
	int (*set_debug)(struct sensor_dev *sdev, const char *buf, size_t count);
	int (*get_debug)(struct sensor_dev *sdev, char *buf);
};

/* apply the bias to hw offset immediately after calibrated */
#define SENSOR_DEV_FLAG_OFFSET_AFTER_CALIB	(0x01)
/* do not register iio devices */
#define SENSOR_DEV_FLAG_DISABLE_IIO		(0x02)

/**
 * struct sensor_dev - sensor device
 * @dev:		[INTERN] device structure, should be assigned a parent
 *			and owner
 * @mlock:		[INTERN] lock used to prevent simultaneous device state
 *			changes
 * @name:		[DRIVER] name of the device.
 * @flag:		[DRIVER] flag of the device, can be SENSOR_DEV_FLAG_*.
 * @ops:		[INTERN] sensor callbacks
 * @groups:		[INTERN] attribute groups
 * @groupcounter:	[INTERN] index of next attribute group
 * @private:		[DRIVER] data private to the driver.
 */
struct sensor_dev {
	struct device			dev;
	struct mutex			mlock;
	const char			*name;
	uint32_t			flag;
	const struct sensor_ops		*ops;

#define SENSOR_MAX_GROUPS 6
	const struct attribute_group	*groups[SENSOR_MAX_GROUPS + 1];
	int				groupcounter;

	void				*private;
};


struct sensor_info {
	const char *name;
	const char *type;
	int mode;
	uint32_t flag;
	const struct attribute_group *attr_group;
	int iio_ch_cnt;
	const struct iio_chan_spec *iio_ch;
	const struct sensor_ops *ops;
};

struct sensor_dev *sensor_create(struct sensor_info *info, void *drvdata);

/**
 * dev_to_sensor_dev() - Get sensor device struct from a device struct
 * @dev: 		The device embedded in the sensor device
 *
 * Note: The device must be a sensor device, otherwise the result is undefined.
 */
static inline struct sensor_dev *dev_to_sensor_dev(struct device *dev)
{
	return dev ? container_of(dev, struct sensor_dev, dev) : NULL;
}

/**
 * sensor_device_set_drvdata() - Set device driver data
 * @dev: sensor device structure
 * @data: Driver specific data
 *
 * Allows to attach an arbitrary pointer to an sensor device, which can later be
 * retrieved by sensor_device_get_drvdata().
 */
static inline void sensor_device_set_drvdata(struct sensor_dev *sdev, void *drvdata)
{
	if (sdev)
		dev_set_drvdata(&sdev->dev, drvdata);
}

/**
 * sensor_device_get_drvdata() - Get device driver data
 * @dev: sensor device structure
 *
 * Returns the data previously set with sensor_device_set_drvdata()
 */
static inline void *sensor_device_get_drvdata(struct sensor_dev *sdev)
{
	return sdev ? dev_get_drvdata(&sdev->dev) : NULL;
}


/* TODO: move these interfaces to sensor adaptor */
struct sensor_dev *sensor_pres_create(const char *name,
	const struct sensor_ops *ops, void *drvdata);

struct sensor_dev *sensor_temp_create(const char *name,
	const struct sensor_ops *ops, void *drvdata);

int sensor_push_data(struct sensor_dev *sdev, uint8_t *data, int64_t timestamp);

#endif
