/* The Sensor iio
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SENSOR_IIO_H_
#define _SENSOR_IIO_H_

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/iio/types.h>

#include "sensor.h"

#define SENSOR_IIO_UINT16_MAX (0xFFFF)

/* *
 * When using polling mode, the sensor iio will poll the
 * raw data after enabled.
 * */
#define SENSOR_IIO_MODE_POLLING   (1)

/* *
 * When using interrupt mode, the device driver should invoke
 * sensor_iio_data_ready to triger data pushing,
 * or push data directly by using sensor_iio_push_data or
 * sensor_iio_push_event
 * */
#define SENSOR_IIO_MODE_INTERRUPT (2)



#define SENSOR_PS_EVENT_NEAR IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,	\
						0,			\
						IIO_EV_TYPE_THRESH,	\
						IIO_EV_DIR_FALLING)

#define SENSOR_PS_EVENT_FAR IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,	\
						0,			\
						IIO_EV_TYPE_THRESH,	\
						IIO_EV_DIR_RISING)

int sensor_iio_push_event(struct sensor_dev *sdev,
	uint64_t event, int64_t timestamp);

int sensor_iio_push_data(struct sensor_dev *sdev,
	uint8_t *data, int64_t timestamp);

int sensor_iio_data_ready(struct sensor_dev *sdev);

int sensor_iio_disable(struct sensor_dev *sdev);

int sensor_iio_enable(struct sensor_dev *sdev);

int sensor_iio_register(struct sensor_dev *sdev,
	const struct iio_chan_spec *channels, int num_channels, int mode);

void sensor_iio_unregister(struct sensor_dev *sdev);

#endif /* _SENSOR_IIO_H_ */