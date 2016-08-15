
#ifndef  __SENSOR_SCP_ACC_DEV_H__
#define  __SENSOR_SCP_ACC_DEV_H__

#include "sensor.h"

#define SENSOR_SCP_ACC_I2C_DEV_NAME "sensor-scp-acc"

struct sensor_scp_acc_dev;

struct sensor_scp_acc_ops {
	int (*check_id)(struct sensor_scp_acc_dev *scp_acc_dev);
	int (*get_name)(struct sensor_scp_acc_dev *scp_acc_dev, const char **name);

	int (*set_offset)(struct sensor_scp_acc_dev *scp_acc_dev, int offset, int axis);

	int (*enable)(struct sensor_scp_acc_dev *scp_acc_dev);
	int (*disable)(struct sensor_scp_acc_dev *scp_acc_dev);

	int (*calibrate)(struct sensor_scp_acc_dev *scp_acc_dev, int32_t calibbias[3]);
	int (*self_test)(struct sensor_scp_acc_dev *scp_acc_dev);

	int (*get_debug)(struct sensor_scp_acc_dev * scp_acc_dev, char *buf);
	int (*set_debug)(struct sensor_scp_acc_dev * scp_acc_dev,
					const char *buf, size_t count);
};

struct sensor_scp_acc_dev {
	struct sensor_dev *sdev;
	struct i2c_client *client;
	struct sensor_scp_acc_ops *ops;
	struct mutex mlock;

	int enable;
	int32_t offset[3];
	int32_t calibbias[3];
};

#endif