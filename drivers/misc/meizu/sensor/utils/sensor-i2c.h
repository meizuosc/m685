/* The Sensor i2c
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SENSOR_I2C_H_
#define _SENSOR_I2C_H_

#include <linux/i2c.h>

int sensor_i2c_read_8bit(struct i2c_client *client, uint8_t addr, uint8_t *val);
int sensor_i2c_write_8bit(struct i2c_client *client, uint8_t addr, uint8_t val);
int sensor_i2c_read_16bits(struct i2c_client *client,
	uint8_t addr, uint16_t *val);

int sensor_i2c_set_debug_8bit(struct i2c_client *client,
	const char *buf, size_t count);

#endif