
#ifndef  __SENSOR_SCP_Gyr_DEV_H__
#define  __SENSOR_SCP_Gyr_DEV_H__

#include "sensor.h"

struct sensor_scp_gyr_dev;

struct sensor_scp_gyr_ops {
	int (*get_name)(struct sensor_scp_gyr_dev *scp_gyr_dev, const char **name);

	int (*enable)(struct sensor_scp_gyr_dev *scp_gyr_dev);
	int (*disable)(struct sensor_scp_gyr_dev *scp_gyr_dev);

	int (*self_test)(struct sensor_scp_gyr_dev *scp_gyr_dev);

	int (*get_debug)(struct sensor_scp_gyr_dev * scp_gyr_dev, char *buf);
	int (*set_debug)(struct sensor_scp_gyr_dev * scp_gyr_dev,
					const char *buf, size_t count);
};

struct sensor_scp_gyr_dev {
	struct sensor_dev *sdev;
	struct i2c_client *client;
	struct sensor_scp_gyr_ops *ops;
	struct mutex mlock;

	int enable;
};


#endif