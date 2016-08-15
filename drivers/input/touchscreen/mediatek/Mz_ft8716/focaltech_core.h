/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

#ifndef __LINUX_FTXXXX_H__
#define __LINUX_FTXXXX_H__

#include "../tpd.h"
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <asm/unistd.h>
//#include <mach/irqs.h>
#include <linux/jiffies.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>
#include <linux/input/mz_gesture_ts.h>
#include "focaltech_global.h"
#include "focaltech_test_ft8716.h"
#include "focaltech_test_config_ft8716.h"

/**********************Custom define begin************/

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0))
	#if defined(MODULE) || defined(CONFIG_HOTPLUG)
		#define __devexit_p(x) 				x
	#else
		#define __devexit_p(x) 				NULL
	#endif
	// Used for HOTPLUG
	#define __devinit        					__section(.devinit.text) __cold notrace
	#define __devinitdata    					__section(.devinit.data)
	#define __devinitconst   					__section(.devinit.rodata)
	#define __devexit        					__section(.devexit.text) __exitused __cold notrace
	#define __devexitdata    					__section(.devexit.data)
	#define __devexitconst   					__section(.devexit.rodata)
#endif

#define TPD_AUTO_UPGRADE					1
//#define CONFIG_MTK_I2C_EXTENSION				1
//#define TPD_HAVE_BUTTON									// if have virtual key,need define the MACRO
//#define TPD_BUTTON_HEIGH        				(40)  				// 100
//#define TPD_KEY_COUNT           				3    				// 4
//#define TPD_KEYS                				{ KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
//#define TPD_KEYS_DIM            				{{80,900,20,TPD_BUTTON_HEIGH}, {240,900,20,TPD_BUTTON_HEIGH}, {400,900,20,TPD_BUTTON_HEIGH}}
#define CTP_ESD_PROTECT						1
#if CTP_ESD_PROTECT
#define FT_ESD_PROTECT  					1
#endif

#define TPD_WAKEUP_GPIO						67
#define TPD_RESET_GPIO						211
#define TPD_EINT_GPIO						68
/*********************Custom Define end******************/
#define MT_PROTOCOL_B
#define TPD_NAME    						"FTS"
#define TPD_I2C_NUMBER           				4
#define TPD_WAKEUP_TRIAL         				60
#define TPD_WAKEUP_DELAY         				100
#define TPD_VELOCITY_CUSTOM_X 					15
#define TPD_VELOCITY_CUSTOM_Y 					20
#define TPD_SUPPORT_POINTS					10
#define CFG_MAX_TOUCH_POINTS					10
#define MT_MAX_TOUCH_POINTS					10
#define FTS_MAX_ID						0x0F
#define FTS_TOUCH_STEP						6
#define FTS_FACE_DETECT_POS					1
#define FTS_TOUCH_X_H_POS					3
#define FTS_TOUCH_X_L_POS					4
#define FTS_TOUCH_Y_H_POS					5
#define FTS_TOUCH_Y_L_POS					6
#define FTS_TOUCH_EVENT_POS					3
#define FTS_TOUCH_ID_POS					5
#define FT_TOUCH_POINT_NUM					2
#define FTS_TOUCH_XY_POS					7
#define FTS_TOUCH_MISC						8
#define POINT_READ_BUF						(3 + FTS_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)
#define FT_FW_NAME_MAX_LEN					50
#define TPD_DELAY              					(2*HZ/100)
#define TPD_CALIBRATION_MATRIX  				{962,0,0,0,1600,0,0,0};

#if defined(MEIZU_FACTORY_BUILD)
#define FTS_MCAP_TEST						1
#endif
/******************************************************************************/
/* Chip Device Type */
#define IC_FT5X06						0				/* x=2,3,4 */
#define IC_FT5606						1				/* ft5506/FT5606/FT5816 */
#define IC_FT5316						2				/* ft5x16 */
#define IC_FT6208						3	  			/* ft6208 */
#define IC_FT6x06     						4				/* ft6206/FT6306 */
#define IC_FT5x06i     						5				/* ft5306i */
#define IC_FT5x36     						6				/* ft5336/ft5436/FT5436i */

/*register address*/
#define FTS_REG_CHIP_ID						0xA3    			// chip ID
#define FTS_REG_FW_VER						0xA6   				// FW  version
#define FTS_REG_VENDOR_ID					0xA8   				// TP vendor ID
#define FTS_REG_POINT_RATE					0x88   				// report rate
#define FT_GESTRUE_MODE_SWITCH_REG				0xD0
#define TPD_MAX_POINTS_2                        		2
#define TPD_MAX_POINTS_5                        		5
#define TPD_MAX_POINTS_10                        		10
#define AUTO_CLB_NEED                           		1
#define AUTO_CLB_NONEED                         		0
#define LEN_FLASH_ECC_MAX 					0xFFFE
#define FTS_GESTRUE_POINTS 					255
#define FTS_GESTRUE_POINTS_ONETIME  				62
#define FTS_GESTRUE_POINTS_HEADER 				8
#define FTS_GESTURE_OUTPUT_ADRESS 				0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 				4
#define TPD_CALIBRATION_MATRIX_ROTATION_NORMAL  		{-4096, 0, 800*4096, 0, -4096, 1280*4096, 0, 0}
#define TPD_CALIBRATION_MATRIX_ROTATION_FACTORY 		{-4096, 0, 800*4096, 0, -4096, 1280*4096, 0, 0}
//#define MEIZU_MARGIN_CONTROL					1

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
/* IC info */
struct fts_Upgrade_Info
{
	u8 CHIP_ID;
	u8 TPD_MAX_POINTS;
	u8 AUTO_CLB;
	u16 delay_aa;						/* delay of write FT_UPGRADE_AA */
	u16 delay_55;						/* delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;					/* upgrade id 1 */
	u8 upgrade_id_2;					/* upgrade id 2 */
	u16 delay_readid;					/* delay of read id */
	u16 delay_erase_flash; 					/* delay of earse flash */
};

/*touch event info*/
struct ts_event
{
	u16 au16_x[CFG_MAX_TOUCH_POINTS];			/* x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];			/* y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];		/* touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];			/* touch ID */
	u16 pressure[CFG_MAX_TOUCH_POINTS];
	u16 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	int touchs;
	u8 touch_point_num;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct ftxxxx_ts_platform_data *pdata;
	struct work_struct 	touch_event_work;
	struct workqueue_struct *ts_workqueue;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
};

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
#define FTS_APK_DEBUG
extern int ft_rw_iic_drv_init(struct i2c_client *client);
extern void  ft_rw_iic_drv_exit(void);
#endif

extern struct i2c_client *fts_i2c_client;
extern struct input_dev *fts_input_dev;
extern unsigned int touch_irq;
extern struct tpd_device *tpd;
extern bool upmu_is_chr_det(void);
extern struct fts_Upgrade_Info fts_updateinfo_curr;
extern int fts_rw_iic_drv_init(struct i2c_client *client);
extern void  fts_rw_iic_drv_exit(void);
extern void fts_get_upgrade_array(void);
extern int fts_Gesture_init(struct input_dev *input_dev);
extern int fts_read_Gestruedata(void);
extern int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
extern int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
extern int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int hidi2c_to_stdi2c(struct i2c_client * client);
extern int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name);
extern int fts_ctpm_auto_clb(struct i2c_client *client);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
extern int fts_ctpm_get_i_file_ver(void);
extern int fts_remove_sysfs(struct i2c_client *client);
extern void fts_release_apk_debug_channel(void);
extern int fts_ctpm_auto_upgrade(struct i2c_client *client);
extern int fts_create_sysfs(struct i2c_client *client);
extern int fts_create_apk_debug_channel(struct i2c_client * client);

#if FT_ESD_PROTECT
void force_reset_guitar(void);
extern void esd_switch(s32 on);
extern int  fts_esd_protection_init(void);
extern int  fts_esd_protection_exit(void);
extern int  fts_esd_protection_notice(void);
extern int  fts_esd_protection_suspend(void);
extern int  fts_esd_protection_resume(void);
#endif

