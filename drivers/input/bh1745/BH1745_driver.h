/* include/linux/i2c/BH1745_driver.h - ROHM BH1745 Linux kernel driver
 *
 * Copyright (C) 2015
 * Written by Grace Huang <grace-huang@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
SE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

 #ifndef _ROHM_BH1745_I2C_H_
#define _ROHM_BH1745_I2C_H_

#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>


 /*************** Definitions ******************/
/* GENERAL */
#define BH1745_DRV_NAME	"bh1745"
#define DRIVER_VERSION		"1.0"


/* BH1745 REGSTER */
#define REG_SYSTEMCONTROL      (0x40)
#define REG_MODECONTROL1       (0x41)
#define REG_MODECONTROL2       (0x42)
#define REG_MODECONTROL3       (0x44)
#define REG_READ_DATA          (0x50)
#define REG_RED_DATA          (0x50)
#define REG_GREEN_DATA          (0x52)
#define REG_BLUE_DATA          (0x54)
#define REG_CLEAR_DATA          (0x56)
#define REG_INTERRUPT          (0x60)
#define REG_THRED_HIGH          (0x62)
#define REG_THRED_LOW          (0x64)
#define REG_MANUFACT_ID           (0x92)

/************ define parameter for register ************/
/* REG_SYSTEMCONTROL(0x40) */
#define SW_RESET               (1 << 7)
#define INT_RESET              (1 << 6)

/* REG_MODECONTROL1(0x41) */
#define MEASURE_160MS          (0x00)
#define MEASURE_320MS          (0x01)
#define MEASURE_640MS          (0x02)
#define MEASURE_1280MS          (0x03)
#define MEASURE_2560MS          (0x04)
#define MEASUREMENT_MAX        (0x05)

/* REG_MODECONTROL2(0x42) */
#define ADC_GAIN_X1            (0x00)
#define ADC_GAIN_X2            (0x01)
#define ADC_GAIN_X16            (0x02)
#define RGBC_EN_ON             (1 << 4)
#define RGBC_EN_OFF            (0 << 4)
#define RGBC_VALID_HIGH        (1 << 7)

/************ definition to dependent on sensor IC ************/
#define BH1745_I2C_ADDRESS     (0x39) //7 bits slave address 011 1001

/************ set initial parameter to IC ************/
// #define RGB_SET_MODE_CONTROL1  (MEASURE_320MS)
#define RGB_SET_MODE_CONTROL1  (MEASURE_160MS)
#define RGB_SET_MODE_CONTROL2  (ADC_GAIN_X16 | RGBC_EN_OFF)

/* Interface parameter of file system */
#define RGB_DISABLE            (0)
#define RGB_ENABLE             (1)

#ifdef CONFIG_AAL_CONTROL
#define	RGB_ENABLE_ALS_NONE				0
#define	RGB_ENABLE_ALS_AAL				1
#define	RGB_ENABLE_ALS_USER				2
#endif

#define PS_ALS_SET_MIN_DELAY_TIME (500)	

#define FT_VTG_MIN_UV           2600000
#define FT_VTG_MAX_UV           3300000
#define FT_I2C_VTG_MIN_UV       1800000
#define FT_I2C_VTG_MAX_UV       1800000

/* OTHER */
#ifdef _ALS_BIG_ENDIAN_
#define CONVERT_TO_BE(value) ((((value) >> 8) & 0xFF) | (((value) << 8) & 0xFF00))
#else
#define CONVERT_TO_BE(value) (value)
#endif


/*************** Structs ******************/
struct RGB_DATA {
	struct i2c_client *client;
    	struct regulator *vdd;
    	struct regulator *vcc_i2c;
	struct mutex update_lock;
	struct delayed_work    als_dwork; /* for ALS polling */
	struct input_dev *input_dev_als;

	unsigned int enable;	//used to indicate working mode	
	unsigned int als_th_l;	//als threshold low, not used in the program
	unsigned int als_th_h;	//als threshold high, not used in the program

	unsigned int suspend_enable;	//used to indicate working mode before suspend
	
#ifdef CONFIG_AAL_CONTROL
	unsigned int enable_als_type;
#endif

	/* ALS parameters */
	unsigned int als_data;			/* to store ALS data */
	unsigned int als_level;
	unsigned int als_poll_delay;	// the unit is ms I think. needed for als polling
	
	unsigned int type;
	unsigned char resume;
	unsigned char tptype;
	unsigned char als_cali;
	unsigned char als_scale;
};

/************ typedef struct ************/
/* structure to read data value from sensor */
typedef struct {
    unsigned short red;         /* data value of red data from sensor */
    unsigned short green;       /* data value of green data from sensor */
    unsigned short blue;        /* data value of blue data from sensor */
    unsigned short clear;       /* data value of clear data from sensor */
} READ_DATA_ARG;

/* structure to read data value from sensor */
typedef struct {
    //unsigned long lux;          /* data value of lux data from sensor */
    //unsigned long color_temp;   /* data value of temperature data from sensor */
    int lux;
    int color_temp;
} CALC_DATA_ARG;

#endif /* _ROHM_BH1745_I2C_H_ */



/*
 * custom code
 */
#include <linux/meizu-sys.h>
#define INPUT_ALS_NAME     "bh1745"
#define CALI_OK   (1)
#define CALI_FAIL (0)

#define ABS_RED (ABS_MISC+1)
#define ABS_GREEN (ABS_MISC+2)
#define ABS_BLUE (ABS_MISC+3)
#define ABS_CLEAR (ABS_MISC+4)

#define INFOR(fmt,args...)  printk(KERN_EMERG "BH1745: %s " fmt,__func__,##args)
extern double my_exp(double x);

struct als_reg {
	unsigned char reg;
	unsigned char data;
};
