
#ifndef  __SENSOR_ALS_DEV_H__
#define  __SENSOR_ALS_DEV_H__

#include "sensor.h"
#include <linux/notifier.h>

#define SENSOR_ALS_DEVICE_NAME "sensor-als"

struct sensor_als_dev;

struct sensor_als_ops {
	int (*check_id)(struct sensor_als_dev *als_dev);
	int (*get_name)(struct sensor_als_dev *als_dev, const char **name);

	int (*get_lux)(struct sensor_als_dev *als_dev, uint32_t *lux);

	int (*enable)(struct sensor_als_dev *als_dev);
	int (*disable)(struct sensor_als_dev *als_dev);

	void (*hw_init)(struct sensor_als_dev *als_dev);

	void (*suspend)(struct sensor_als_dev * als_dev);
	void (*resume)(struct sensor_als_dev * als_dev);

	int (*get_debug)(struct sensor_als_dev * als_dev, char *buf);
	int (*set_debug)(struct sensor_als_dev * als_dev,
					const char *buf, size_t count);
	int (*get_max_sample_freq)(struct sensor_als_dev *als_dev,
							int32_t *sample_freq);
};

struct sensor_als_dev {
	struct sensor_dev *sdev;
	struct i2c_client *client;
	struct sensor_als_ops *ops;
	struct mutex mlock;
	struct notifier_block fb_notifier;
	int als_enable;
	uint32_t pre_lux;
	int als_range;
	int coefficient;
	int cover_type;
};

#endif