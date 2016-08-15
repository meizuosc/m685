/* The Sensor core
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SENSOR_CORE_H_
#define _SENSOR_CORE_H_

#include <linux/device.h>
#include "sensor.h"

/**
 * sensor_device_alloc() - allocate an sensor_dev from a driver
 * @name:		the name of sensor, can not be NULL.
 **/
struct sensor_dev *sensor_device_alloc(const char *name);

/**
 * sensor_device_free() - free an sensor_dev from a driver
 * @dev: 		the sensor_dev associated with the device
 **/
void sensor_device_free(struct sensor_dev *sdev);

/**
 * sensor_device_register() - register a device with the Sensor subsystem
 * @dev:		Device structure filled by the device driver
 **/
int sensor_device_register(struct sensor_dev *sdev);

/**
 * sensor_device_unregister() - unregister a device from the Sensor subsystem
 * @dev:		Device structure representing the device.
 **/
void sensor_device_unregister(struct sensor_dev *sdev);

#endif /* _SENSOR_CORE_H_ */
