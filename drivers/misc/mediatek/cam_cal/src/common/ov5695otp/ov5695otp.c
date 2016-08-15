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
/*
 * Driver for CAM_CAL
 *
 *
 */

#ifndef CONFIG_MTK_I2C_EXTENSION
#define CONFIG_MTK_I2C_EXTENSION
#endif
#include <linux/i2c.h>
#undef CONFIG_MTK_I2C_EXTENSION
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "ov5695otp.h"
#include "meizu_otp.h"
/* #include <asm/system.h>  // for SMP */
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


/* #define CAM_CALGETDLT_DEBUG */
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define PFX "ov5695otp"
#define CAM_CALINF(fmt, arg...)    pr_info(PFX "[%s] " fmt, __func__, ##arg)
#define CAM_CALDB(fmt, arg...)    pr_info(PFX "[%s] " fmt, __func__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_info(PFX "[%s] " fmt, __func__, ##arg)
#else
#define CAM_CALINF(fmt, arg...)
#define CAM_CALDB(fmt, arg...)
#define CAM_CALERR(fmt, arg...)
#endif
#define PAGE_SIZE_ 256
#define BUFF_SIZE 8

static DEFINE_SPINLOCK(g_CAM_CALLock);/*for SMP*/
#define CAM_CAL_I2C_BUSNUM 3

#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define OV5695OTP_DEVICE_ID							0xA0
/*#define I2C_UNIT_SIZE                                  1 //in byte*/
/*#define OTP_START_ADDR                            0x0A04*/
/*#define OTP_SIZE                                      24*/

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 /* seanlin111208 */
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DEV2"

#define QTECH_MODULE_ID   0x06
#define SUNNY_MODULE_ID   0x01
extern ov5695_otp_struct *ov5695_sunny_otp;
extern ov5695_otp_struct *ov5695_qtech_otp;

static struct i2c_client *g_pstI2Cclient;

unsigned int ov5695otp_selective_read_region(struct i2c_client *client, unsigned int addr,
						unsigned char *data, unsigned int size)
{
	int *otp_data = NULL;
	int size_of_otp = 0;
	int i;

	size_of_otp = sizeof(ov5695_otp_struct) / sizeof(int);

	if ((ov5695_sunny_otp) && (ov5695_sunny_otp->module_id == SUNNY_MODULE_ID)) {
		otp_data = (int *)ov5695_sunny_otp;
	} else if ((ov5695_qtech_otp) && (ov5695_qtech_otp->module_id == QTECH_MODULE_ID)) {
		otp_data = (int *)ov5695_qtech_otp;
	}

	if ((addr + size) > size_of_otp) {
		CAM_CALERR("[OV5695OTP] Read ERR. size_of_otp:%d, Request Read addr:0x%x, Read size:%d.\n", size_of_otp, addr, size);
		return -EFAULT;
	} else {
		if (otp_data == NULL)
			return 0;

		for (i = 0; i < size; i++) {
			data[i] = (u8)otp_data[addr + i];
			CAM_CALDB("[OV5695OTP] Get otp data addr:0x%x, data:0x%x.\n", (addr + i), data[i]);
		}
	}

	return 0;
}

static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	CAM_CALDB("[ov5695otp] Attach I2C\n");

	/* get sensor i2c client */
	spin_lock(&g_CAM_CALLock); /* for SMP */
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr = OV5695OTP_DEVICE_ID >> 1;
	spin_unlock(&g_CAM_CALLock); /* for SMP */

	CAM_CALDB("[ov5695otp] g_pstI2Cclient->addr = 0x%x\n", g_pstI2Cclient->addr);
	CAM_CALDB("[ov5695otp] Attached!!\n");

	return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME, 0}, {} };

static struct i2c_driver SUB_CAM_CAL_i2c_driver = {
	.probe = CAM_CAL_i2c_probe,
	.remove = CAM_CAL_i2c_remove,
	.driver.name = CAM_CAL_DRVNAME,
	.id_table = CAM_CAL_i2c_id,
};

module_i2c_driver(SUB_CAM_CAL_i2c_driver);

MODULE_DESCRIPTION("CAM_CAL_SUB driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
