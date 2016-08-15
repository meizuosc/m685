
#ifndef  __sensor_scp_mag_dev_H__
#define  __sensor_scp_mag_dev_H__

#include "sensor.h"

#define SENSOR_SCP_MAG_DEVICE_NAME "sensor-scp-mag"

struct sensor_scp_mag_dev;

struct sensor_scp_mag_ops {
	int (*check_id)(struct sensor_scp_mag_dev *scp_mag_dev);
	int (*get_name)(struct sensor_scp_mag_dev *scp_mag_dev, const char **name);

	int (*enable)(struct sensor_scp_mag_dev *scp_mag_dev);
	int (*disable)(struct sensor_scp_mag_dev *scp_mag_dev);

	int (*self_test)(struct sensor_scp_mag_dev *scp_mag_dev);

	int (*get_debug)(struct sensor_scp_mag_dev * scp_mag_dev, char *buf);
	int (*set_debug)(struct sensor_scp_mag_dev * scp_mag_dev,
					const char *buf, size_t count);
};

struct sensor_scp_mag_dev {
	struct sensor_dev *sdev;
	struct i2c_client *client;
	struct sensor_scp_mag_ops *ops;
	struct mutex mlock;

	int enable;
};

#endif