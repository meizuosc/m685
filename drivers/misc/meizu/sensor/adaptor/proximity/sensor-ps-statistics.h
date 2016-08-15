
#ifndef  __SENSOR_PS_STATISTICS_H__
#define  __SENSOR_PS_STATISTICS_H__

#include "sensor-ps-dev.h"

#define SENSOR_PS_STATISTICS_ENABLE 1

extern int sensor_ps_statistics_init(struct sensor_ps_dev *ps_dev);
extern void sensor_ps_statistics_dump(void);
extern void sensor_ps_statistics_start(void);
extern void sensor_ps_statistics_stop(void);
#endif