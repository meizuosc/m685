
#ifndef  __sensor_scp_pres_DEV_H__
#define  __sensor_scp_pres_DEV_H__

#include "sensor.h"

#define SENSOR_SCP_PRES_DEVICE_NAME "sensor-scp-pres"


struct sensor_scp_pres_dev;

struct sensor_scp_pres_ops {
	int (*check_id)(struct sensor_scp_pres_dev *scp_pres_dev);
	int (*get_name)(struct sensor_scp_pres_dev *scp_pres_dev, const char **name);

	int (*set_offset)(struct sensor_scp_pres_dev *scp_pres_dev, int offset, int axis);

	int (*enable)(struct sensor_scp_pres_dev *scp_pres_dev);
	int (*disable)(struct sensor_scp_pres_dev *scp_pres_dev);

	int (*calibrate)(struct sensor_scp_pres_dev *scp_pres_dev, int32_t calibbias[3]);
	int (*self_test)(struct sensor_scp_pres_dev *scp_pres_dev);

	int (*get_debug)(struct sensor_scp_pres_dev * scp_pres_dev, char *buf);
	int (*set_debug)(struct sensor_scp_pres_dev * scp_pres_dev,
					const char *buf, size_t count);
};

struct sensor_scp_pres_dev {
	struct sensor_dev *sdev;
	struct i2c_client *client;
	struct sensor_scp_pres_ops *ops;
	struct mutex mlock;

	int enable;
	int32_t offset[3];
	int32_t calibbias[3];
};

#endif