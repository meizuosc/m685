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
#include "imx386otp.h"
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
#define PFX "imx386otp"
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
#define IMX386OTP_DEVICE_ID							0xA0
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
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"

#define SUNNY_MODULE	0x07
#define PRIMAX_MODULE	0x03

extern u8 *primax_otp_buf;
extern u8 *sunny_otp_buf;

static struct i2c_client *g_pstI2Cclient;

static int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
    int  i4RetValue = 0;

    spin_lock(&g_CAM_CALLock);
    g_pstI2Cclient->addr = (i2cId >> 1);
    g_pstI2Cclient->ext_flag = (g_pstI2Cclient->ext_flag)&(~I2C_DMA_FLAG);

    spin_unlock(&g_CAM_CALLock);
    i4RetValue = i2c_master_send(g_pstI2Cclient, a_pSendData, a_sizeSendData);
    if (i4RetValue != a_sizeSendData) {
        CAM_CALERR("[imx386otp]I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
        return -1;
    }
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_pRecvData, a_sizeRecvData);
    if (i4RetValue != a_sizeRecvData) {
        CAM_CALERR("[imx386otp]I2C read failed!! \n");
        return -1;
    }

    return 0;
}

static int get_module_type(void)
{
	int ret = 0;
	u8 module_id = 0;
	u8 puSendCmd[2] = {0x00, 0x01};	/* module id reg addr = 0x0001 for main-camera module */

	ret = iReadRegI2C(puSendCmd , 2, &module_id, 1, IMX386OTP_DEVICE_ID);
	if (ret < 0) {
		CAM_CALERR("[imx386otp]I2C read module id failed!! \n");
		return -ENODEV;
	} if (module_id == 0x03) {
		CAM_CALDB("[imx386otp] primax main-camera module.\n");
		return PRIMAX_MODULE;
	} else {
		CAM_CALDB("[imx386otp] sunny main-camera module.\n");
		return SUNNY_MODULE;
	}
}

static int selective_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	int i;
	CAM_CALDB("[imx386otp]ENTER  address:0x%x buffersize:%d\n ", addr, size);

	if (get_module_type() == PRIMAX_MODULE) {
		memcpy(data, (primax_otp_buf + addr), size);
	} else if (get_module_type() == SUNNY_MODULE) {
		memcpy(data, (sunny_otp_buf + addr), size);
	}

	for (i = 0; i < size; i++) {
		/*
		 * Don't need to read it from EEPROM because otp data had been
		 * read out and stored into otp_buf when kernel bootup.
		 */
			
		CAM_CALDB("[imx386otp]address+i = 0x%x,readbuff = 0x%x\n",(addr+i),*(data+i));
	}

	return 0;
}

unsigned int imx386otp_selective_read_region(struct i2c_client *client, unsigned int addr,
						unsigned char *data, unsigned int size)
{
	g_pstI2Cclient = client;
	return selective_read_region(addr, data, g_pstI2Cclient->addr, size);
}

static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	CAM_CALDB("[imx386otp] Attach I2C\n");

	/* get sensor i2c client */
	spin_lock(&g_CAM_CALLock); /* for SMP */
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr = IMX386OTP_DEVICE_ID >> 1;
	spin_unlock(&g_CAM_CALLock); /* for SMP */

	CAM_CALDB("[imx386otp] g_pstI2Cclient->addr = 0x%x\n", g_pstI2Cclient->addr);
	CAM_CALDB("[imx386otp] Attached!!\n");

	return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME, 0}, {} };

static struct i2c_driver CAM_CAL_i2c_driver = {
	.probe = CAM_CAL_i2c_probe,
	.remove = CAM_CAL_i2c_remove,
	.driver.name = CAM_CAL_DRVNAME,
	.id_table = CAM_CAL_i2c_id,
};

module_i2c_driver(CAM_CAL_i2c_driver);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
