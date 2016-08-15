/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __FOCALTECH_COMM_H__
#define __FOCALTECH_COMM_H__
 /*******************************************************************************
*
* File Name: focaltech_comm.h
*
*    Author: Software Department, FocalTech
*
*   Created: 2016-03-16
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/cdev.h>

#include <linux/dma-mapping.h>
//#include "focaltech_flash.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_COMMON_LIB_INFO  "Common_Lib_Version  V1.0.0 2016-02-24"

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
// Function Switchs: define to open,  comment to close
#define FTS_APK_DEBUG

//define and initiate in focaltech_core.c
extern struct i2c_client *fts_i2c_client;
extern struct input_dev *fts_input_dev;
extern int fts_i2c_communication_init(void);
extern int fts_i2c_communication_exit(void);
extern int fts_i2c_read_universal(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int fts_i2c_write_universal(struct i2c_client *client, char *writebuf, int writelen);
extern int fts_i2c_read_dma(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int fts_i2c_write_dma(struct i2c_client *client, char *writebuf, int writelen);

//define and initiate in focaltech_flash.c
extern struct fts_Upgrade_Info fts_upgrade_info_curr;
//define and implement in focaltech_flash.c
extern int fts_flash_init(struct i2c_client *client);
extern int fts_flash_exit(void);
extern int fts_flash_upgrade_with_bin_file( char *firmware_name);
extern int fts_flash_upgrade_with_i_file(void);
extern int fts_flash_get_i_file_version(void);
extern int fts_ctpm_auto_clb(struct i2c_client *client);

//define and implement in focaltech_monitor.c
extern int fts_monitor_init(struct input_dev *dev,  int bUseProtocolB);
extern int fts_monitor_exit(void);
extern int fts_monitor_record(char *readbuf, unsigned char readlen);

//define and implement in focaltech_platform_data.c
extern int fts_platform_data_init(struct i2c_client *client);
extern int fts_platform_data_exit(void);

//define and implement in focaltech_report.c
extern int fts_report_init(struct i2c_client *client, void *platform_data);
extern int fts_report_exit(struct i2c_client *client, void *platform_data);
extern int fts_report_resume(void);
extern int fts_report_suspend(void);
	
//define and implement in focaltech_apk_node.c
extern int fts_apk_node_init(void);
extern int fts_apk_node_exit(void);

//define and implement in focaltech_sysfs.c
extern int fts_sysfs_init(void);
extern int fts_sysfs_exit(void);

//define and implement in focaltech_gesture.c		
extern int fts_gesture_init(struct input_dev *input_dev);	
extern int fts_gesture_exit(void);
extern int fts_gesture_handle(void);

//define and implement in focaltech_proximity.c
extern int fts_touch_proximity_init(void);
extern int fts_touch_proximity_exit(void);

//define and implement in focaltech_esd_protection.c
extern int fts_esd_protection_init(void);
extern int fts_esd_protection_exit(void);
extern int fts_esd_protection_notice(void);
extern int  fts_esd_protection_suspend(void);
extern int  fts_esd_protection_resume(void);

//define and implement in focaltech_test.c
extern int fts_test_init(void);
extern int fts_test_exit(void);

/******************************************************************************** Static function prototypes*******************************************************************************/
//#define FTS_DBG_EN
#ifdef FTS_DBG_EN	
#define FTS_COMMON_DBG(fmt, args...)	do {printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##args);} while (0)
//#define FTS_TEST_DBG(fmt, args...)      do {printk("[FTS] %s. line: %d.  "fmt"\n",  __FUNCTION__, __LINE__, ##args);} while (0)
//#define FTS_COMMON_DBG(fmt, arg...)	printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##arg)
#else
#define FTS_COMMON_DBG(fmt, args...) do{}while(0)
//#define FTS_COMMON_DBG(fmt, arg...) (void)(0)
#endif
#define ERROR_COMMON_FTS(fmt, arg...)	printk(KERN_ERR"[FTS] ERROR. %s. line: %d.  "fmt"  !!!\n", __FUNCTION__, __LINE__, ##arg)
#endif
