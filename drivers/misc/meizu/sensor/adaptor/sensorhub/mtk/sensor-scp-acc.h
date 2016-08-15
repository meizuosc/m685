
#ifndef  __SENSOR_SCP_ACC_H__
#define  __SENSOR_SCP_ACC_H__

#include "sensor.h"


struct sensor_dev *sensor_scp_acc_create(const char *name,
	const struct sensor_ops *ops, void *drvdata);

#endif