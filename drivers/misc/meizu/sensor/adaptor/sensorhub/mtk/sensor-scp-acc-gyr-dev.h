
#ifndef  __SENSOR_SCP_ACC_GYR_DEV_H__
#define  __SENSOR_SCP_ACC_GYR_DEV_H__

#include "sensor.h"
#include "sensor-scp-acc-dev.h"
#include "sensor-scp-gyr-dev.h"

#define SENSOR_SCP_ACC_GYR_DEVICE_NAME "sensor-scp-acc-gyr"

struct sensor_scp_acc_gyr_ops {
	struct sensor_scp_acc_ops *acc_ops;
	struct sensor_scp_gyr_ops *gyr_ops;
};

struct sensor_scp_acc_gyr_dev {
	struct sensor_scp_acc_dev acc_dev;
	struct sensor_scp_gyr_dev gyr_dev;
};

#endif