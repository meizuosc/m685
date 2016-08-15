
#ifndef  __SENSOR_PS_H__
#define  __SENSOR_PS_H__

#include "sensor.h"

struct sensor_dev *sensor_ps_create(const char *name,
	const struct sensor_ops *ops, void *drvdata);

#endif