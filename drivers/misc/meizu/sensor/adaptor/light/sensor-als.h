
#ifndef  __SENSOR_ALS_H__
#define  __SENSOR_ALS_H__

#include "sensor.h"

struct sensor_dev *sensor_als_create(const char *name,
	const struct sensor_ops *ops, void *drvdata);

#endif