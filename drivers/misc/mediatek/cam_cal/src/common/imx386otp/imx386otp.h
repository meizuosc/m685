/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   imx386otp.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 * John Wei (MTK07407)
 *
 *============================================================================*/
#ifndef __IMX386OTP_H
#define __IMX386OTP_H
#include <linux/i2c.h>


unsigned int imx386otp_selective_read_region(struct i2c_client *client, unsigned int addr,
							unsigned char *data, unsigned int size);


#endif /* __IMX386OTP_H */

