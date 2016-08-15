
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/gpio.h>

#include "../../../kernel/irq/internals.h"
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

#define LINUX_KERNEL_2_6_X	0

#define APDS9921_DRV_NAME	"apds9921"
#define DRIVER_VERSION		"1.0.0"

#define ABS_LIGHT	0x29	// added to support LIGHT - light sensor

#define APDS9921_INT		(20)

#define APDS9921_PS_DETECTION_THRESHOLD		50
#define APDS9921_PS_HSYTERESIS_THRESHOLD	40
#define APDS9921_PS_PULSE_NUMBER			8

#define APDS9921_ALS_THRESHOLD_HSYTERESIS	1	/* 1 = 1% */

#define APDS9921_ALS_CAL_LOOP			1
#define APDS9921_ALS_CAL_LUX			300
#define APDS9921_ALS_CAL_LUX_LOW		((70*APDS9921_ALS_CAL_LUX)/100)		// 70% of 300 lux
#define	APDS9921_ALS_CAL_LUX_HIGH		((130*APDS9921_ALS_CAL_LUX)/100)	// 130% of 300 lux

#define APDS9921_PS_CAL_LOOP			1
#define APDS9921_PS_CAL_CROSSTALK_LOW	0
#define	APDS9921_PS_CAL_CROSSTALK_HIGH	1
#define APDS9921_PS_CROSSTALK_DELTA		50


/* Change History
 *
 * 1.0.0	Fundamental Functions of APDS-9921
 *
 */

#define APDS_IOCTL_PS_ENABLE				1
#define APDS_IOCTL_PS_GET_ENABLE			2
#define APDS_IOCTL_PS_POLL_DELAY			3
#define APDS_IOCTL_ALS_ENABLE				4
#define APDS_IOCTL_ALS_GET_ENABLE			5
#define APDS_IOCTL_ALS_POLL_DELAY			6
#define APDS_IOCTL_PS_GET_PDATA				7	// ps_data
#define APDS_IOCTL_ALS_GET_CLEAR_DATA		8	// clr_data
#define APDS_IOCTL_ALS_GET_ALS_DATA			9	// als_data
#define APDS_IOCTL_ALS_GET_CAL_FACTOR		10 	// als calibration factor

#define APDS_DISABLE_PS						0
#define APDS_ENABLE_PS_WITH_INT				1
#define APDS_ENABLE_PS_NO_INT				2
#define	APDS_ENABLE_PS_CALIBRATION			3

#define APDS_DISABLE_ALS					0
#define APDS_ENABLE_ALS_WITH_INT			1
#define APDS_ENABLE_ALS_NO_INT				2
#define	APDS_ENABLE_ALS_CALIBRATION			3

#ifdef CONFIG_AAL_CONTROL
#define	APDS_ENABLE_ALS_NONE				0
#define	APDS_ENABLE_ALS_AAL				1
#define	APDS_ENABLE_ALS_USER				2
#endif

#define APDS_ALS_POLL_500MS					0	// 2 Hz (500ms)
#define APDS_ALS_POLL_200MS				    1	// 10 Hz (200ms)
#define APDS_ALS_POLL_100MS				    2	// 10 Hz (100ms)
#define APDS_ALS_POLL_50MS				    3	// 10 Hz (50ms)
#define APDS_ALS_POLL_25MS					4	// 40 Hz (25ms)
/*
 * Defines
 */

/* Register Addresses define */
#define APDS9921_DD_MAIN_CTRL_ADDR			0x00
#define APDS9921_DD_PRX_LED_ADDR			0x01
#define APDS9921_DD_PRX_PULSES_ADDR			0x02
#define APDS9921_DD_PRX_MEAS_RATE_ADDR		0x03
#define APDS9921_DD_ALS_MEAS_RATE_ADDR		0x04
#define APDS9921_DD_ALS_GAIN_ADDR			0x05
#define APDS9921_DD_PART_ID_ADDR			0x06
#define APDS9921_DD_MAIN_STATUS_ADDR		0x07
#define APDS9921_DD_PRX_DATA_ADDR			0x08
#define APDS9921_DD_PRX_DATA_0_ADDR			0x08
#define APDS9921_DD_PRX_DATA_1_ADDR			0x09
#define APDS9921_DD_CLEAR_DATA_ADDR			0x0A
#define APDS9921_DD_CLEAR_DATA_0_ADDR		0x0A
#define APDS9921_DD_CLEAR_DATA_1_ADDR		0x0B
#define APDS9921_DD_CLEAR_DATA_2_ADDR		0x0C
#define APDS9921_DD_ALS_DATA_ADDR			0x0D
#define APDS9921_DD_ALS_DATA_0_ADDR			0x0D
#define APDS9921_DD_ALS_DATA_1_ADDR			0x0E
#define APDS9921_DD_ALS_DATA_2_ADDR			0x0F
#define APDS9921_DD_COMP_DATA_ADDR			0x16
#define APDS9921_DD_COMP_DATA_0_ADDR		0x16
#define APDS9921_DD_COMP_DATA_1_ADDR		0x17
#define APDS9921_DD_COMP_DATA_2_ADDR		0x18
#define APDS9921_DD_INT_CFG_ADDR			0x19
#define APDS9921_DD_INT_PERSISTENCE_ADDR	0x1A
#define APDS9921_DD_PRX_THRES_UP_ADDR		0x1B
#define APDS9921_DD_PRX_THRES_UP_0_ADDR		0x1B
#define APDS9921_DD_PRX_THRES_UP_1_ADDR		0x1C
#define APDS9921_DD_PRX_THRES_LOW_ADDR		0x1D
#define APDS9921_DD_PRX_THRES_LOW_0_ADDR	0x1D
#define APDS9921_DD_PRX_THRES_LOW_1_ADDR	0x1E
#define APDS9921_DD_PRX_CAN_ADDR			0x1F
#define APDS9921_DD_PRX_CAN_0_ADDR			0x1F
#define APDS9921_DD_PRX_CAN_1_ADDR			0x20
#define	APDS9921_DD_ALS_THRES_UP_ADDR		0x21
#define	APDS9921_DD_ALS_THRES_UP_0_ADDR		0x21
#define	APDS9921_DD_ALS_THRES_UP_1_ADDR		0x22
#define	APDS9921_DD_ALS_THRES_UP_2_ADDR		0x23
#define	APDS9921_DD_ALS_THRES_LOW_ADDR		0x24
#define	APDS9921_DD_ALS_THRES_LOW_0_ADDR	0x24
#define	APDS9921_DD_ALS_THRES_LOW_1_ADDR	0x25
#define	APDS9921_DD_ALS_THRES_LOW_2_ADDR	0x26
#define	APDS9921_DD_ALS_THRES_VAR_ADDR		0x27
#define	APDS9921_DD_DEVICE_CONFIG_ADDR		0x2F

/* Register Value define : MAIN_CTRL */
#define APDS9921_DD_PRX_EN					0x01
#define APDS9921_DD_ALS_EN					0x02
#define APDS9921_DD_SW_RESET				0x10

/* Register Value define : PS_LED */
#define APDS9921_DD_LED_CURRENT_2_5_MA		0x00  /* 2.5 mA */
#define APDS9921_DD_LED_CURRENT_5_MA		0x01  /* 5 mA */
#define APDS9921_DD_LED_CURRENT_10_MA		0x02  /* 10 mA */
#define APDS9921_DD_LED_CURRENT_25_MA		0x03  /* 25 mA */
#define APDS9921_DD_LED_CURRENT_50_MA		0x04  /* 50 mA */
#define APDS9921_DD_LED_CURRENT_75_MA		0x05  /* 75 mA */
#define APDS9921_DD_LED_CURRENT_100_MA		0x06  /* 100 mA */
#define APDS9921_DD_LED_CURRENT_125_MA		0x07  /* 125 mA */

#define APDS9921_DD_LED_CURRENT_PEAK_ON		0x08

#define APDS9921_DD_LED_FREQ_60_KHZ			0x30  /* LED Pulse frequency = 60KHz */
#define APDS9921_DD_LED_FREQ_70_KHZ			0x40  /* LED Pulse frequency = 70KHz */
#define APDS9921_DD_LED_FREQ_80_KHZ			0x50  /* LED Pulse frequency = 80KHz */
#define APDS9921_DD_LED_FREQ_90_KHZ			0x60  /* LED Pulse frequency = 90KHz */
#define APDS9921_DD_LED_FREQ_100_KHZ		0x70  /* LED Pulse frequency = 100KHz */

/* Register Value define : PS_MEAS_RATE */
#define APDS9921_DD_PRX_MEAS_RATE_6_25_MS	0x01  /* PS Measurement rate = 6.25 ms */
#define APDS9921_DD_PRX_MEAS_RATE_12_5_MS	0x02  /* PS Measurement rate = 12.5 ms */
#define APDS9921_DD_PRX_MEAS_RATE_25_MS		0x03  /* PS Measurement rate = 25 ms */
#define APDS9921_DD_PRX_MEAS_RATE_50_MS		0x04  /* PS Measurement rate = 50 ms */
#define APDS9921_DD_PRX_MEAS_RATE_100_MS	0x05  /* PS Measurement rate = 100 ms */
#define APDS9921_DD_PRX_MEAS_RATE_200_MS	0x06  /* PS Measurement rate = 200 ms */
#define APDS9921_DD_PRX_MEAS_RATE_400_MS	0x07  /* PS Measurement rate = 400 ms */

#define APDS9921_DD_PRX_MEAS_RES_8_BIT		0x00  /* PS resolution 8 bit (full range : 0 ~ 1023) */
#define APDS9921_DD_PRX_MEAS_RES_9_BIT		0x08  /* PS resolution 9 bit (full range : 0 ~ 2047) */
#define APDS9921_DD_PRX_MEAS_RES_10_BIT		0x10  /* PS resolution 10 bit (full range : 0 ~ 3071) */
#define APDS9921_DD_PRX_MEAS_RES_11_BIT		0x18  /* PS resolution 11 bit (full range : 0 ~ 4095) */

/* Register Value define : ALS_MEAS_RATE */
#define APDS9921_DD_ALS_MEAS_RATE_25_MS		0x00  /* ALS Measurement rate = 25 ms */
#define APDS9921_DD_ALS_MEAS_RATE_50_MS		0x01  /* ALS Measurement rate = 50 ms */
#define APDS9921_DD_ALS_MEAS_RATE_100_MS	0x02  /* ALS Measurement rate = 100 ms */
#define APDS9921_DD_ALS_MEAS_RATE_200_MS	0x03  /* ALS Measurement rate = 200 ms */
#define APDS9921_DD_ALS_MEAS_RATE_500_MS	0x04  /* ALS Measurement rate = 500 ms */
#define APDS9921_DD_ALS_MEAS_RATE_1000_MS	0x05  /* ALS Measurement rate = 1000 ms */
#define APDS9921_DD_ALS_MEAS_RATE_2000_MS	0x06  /* ALS Measurement rate = 2000 ms */

#define APDS9921_DD_ALS_MEAS_RES_20_BIT		0x00  /* ALS resolution 20 bit (full range : 0 ~ 1048575) [ADC conversion time = 400ms] */
#define APDS9921_DD_ALS_MEAS_RES_19_BIT		0x10  /* ALS resolution 19 bit (full range : 0 ~ 524287) [ADC conversion time = 200ms]  */
#define APDS9921_DD_ALS_MEAS_RES_18_BIT		0x20  /* ALS resolution 18 bit (full range : 0 ~ 262143) [ADC conversion time = 100ms]  */
#define APDS9921_DD_ALS_MEAS_RES_17_BIT		0x30  /* ALS resolution 17 bit (full range : 0 ~ 131071) [ADC conversion time = 50ms]  */
#define APDS9921_DD_ALS_MEAS_RES_16_BIT		0x40  /* ALS resolution 16 bit (full range : 0 ~ 65535) [ADC conversion time = 25ms]  */

/* Register Value define : ALS_GAIN */
#define APDS9921_DD_ALS_GAIN_1				0x00  /* ALS Gain 1 */
#define APDS9921_DD_ALS_GAIN_3				0x01  /* ALS Gain 3 */
#define APDS9921_DD_ALS_GAIN_6				0x02  /* ALS Gain 6 */
#define APDS9921_DD_ALS_GAIN_9				0x03  /* ALS Gain 9 */
#define APDS9921_DD_ALS_GAIN_18				0x04  /* ALS Gain 18 */

/* Register Value define : MAIN_STATUS */
#define APDS9921_DD_PRX_DATA_STATUS			0x01  /* 1: New data, not read yet (cleared after read) */
#define APDS9921_DD_PRX_INT_STATUS			0x02  /* 1: Interrupt condition fulfilled (cleared after read) */
#define APDS9921_DD_PRX_LOGICAL_STATUS		0x04  /* 1: object is close */
#define APDS9921_DD_ALS_DATA_STATUS			0x08  /* 1: New data, not read yet (cleared after read) */
#define APDS9921_DD_ALS_INT_STATUS			0x10  /* 1: Interrupt condition fulfilled (cleared after read) */
#define APDS9921_DD_POWER_ON_STATUS			0x20  /* 1: Power on cycle */

/* Register Value define : INT_CFG */
#define APDS9921_DD_PRX_INT_EN				0x01  /* 1: PS Interrupt enabled */
#define APDS9921_DD_PRX_LOGIC_MODE			0x02  /* 1: PS Logic Output Mode: INT pad is updated after every measurement and maintains output state between measurements */
#define APDS9921_DD_ALS_INT_EN				0x04  /* 1: ALS Interrupt enabled */
#define APDS9921_DD_ALS_VAR_MODE			0x08  /* 1: ALS variation interrupt mode */
#define APDS9921_DD_ALS_INT_SEL_ALS			0x10  /* ALS channel selected for interrupt */
#define APDS9921_DD_ALS_INT_SEL_CLEAR		0x00  /* Clear channel selected for interrupt */

/* Register Value define : INT_PERSISTENCE */
#define APDS9921_DD_PRX_PERS_1				0x00  /* Every PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_2				0x01  /* 2 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_3				0x02  /* 3 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_4				0x03  /* 4 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_5				0x04  /* 5 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_6				0x05  /* 6 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_7				0x06  /* 7 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_8				0x07  /* 8 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_9				0x08  /* 9 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_10				0x09  /* 10 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_11				0x0A  /* 11 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_12				0x0B  /* 12 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_13				0x0C  /* 13 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_14				0x0D  /* 14 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_15				0x0E  /* 15 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_16				0x0F  /* 16 consecutive PS value out of threshold range */

#define APDS9921_DD_ALS_PERS_1				0x00  /* Every ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_2				0x10  /* 2 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_3				0x20  /* 3 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_4				0x30  /* 4 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_5				0x40  /* 5 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_6				0x50  /* 6 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_7				0x60  /* 7 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_8				0x70  /* 8 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_9				0x80  /* 9 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_10				0x90  /* 10 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_11				0xA0  /* 11 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_12				0xB0  /* 12 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_13				0xC0  /* 13 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_14				0xD0  /* 14 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_15				0xE0  /* 15 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_16				0xF0  /* 16 consecutive ALS value out of threshold range */

/* Register Value define : ALS_THRES_VAR */
#define APDS9921_DD_ALS_VAR_8_COUNT			0x00  /* ALS result varies by 8 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_16_COUNT		0x01  /* ALS result varies by 16 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_32_COUNT		0x02  /* ALS result varies by 32 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_64_COUNT		0x03  /* ALS result varies by 64 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_128_COUNT		0x04  /* ALS result varies by 128 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_256_COUNT		0x05  /* ALS result varies by 256 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_512_COUNT		0x06  /* ALS result varies by 512 counts compared to previous result */
#define APDS9921_DD_ALS_VAR_1024_COUNT		0x07  /* ALS result varies by 1024 counts compared to previous result */

typedef enum
{
    APDS9921_DD_ALS_RES_16BIT = 0,  /* 25ms integration time */
            APDS9921_DD_ALS_RES_17BIT = 1,  /* 50ms integration time */
            APDS9921_DD_ALS_RES_18BIT = 2,  /* 100ms integration time */
            APDS9921_DD_ALS_RES_19BIT = 3,  /* 200ms integration time */
            APDS9921_DD_ALS_RES_20BIT = 4   /* 400ms integration time */
} apds9921_dd_als_res_e;

typedef enum
{
    APDS9921_DD_ALS_GAIN_1X = 0,    /* 1x ALS GAIN */
            APDS9921_DD_ALS_GAIN_3X = 1,    /* 3x ALS GAIN */
            APDS9921_DD_ALS_GAIN_6X = 2,    /* 6x ALS GAIN */
            APDS9921_DD_ALS_GAIN_9X = 3,    /* 9x ALS GAIN */
            APDS9921_DD_ALS_GAIN_18X = 4    /* 18x ALS GAIN */
} apds9921_dd_als_gain_e;

typedef enum
{
    APDS9921_DD_PRX_RES_8BIT = 0,
    APDS9921_DD_PRX_RES_9BIT = 1,
    APDS9921_DD_PRX_RES_10BIT = 2,
    APDS9921_DD_PRX_RES_11BIT = 3
} apds9921_dd_prx_res_e;

#define APDS9921_DD_LUX_FACTOR					30

#define APDS9921_DD_ALS_DEFAULT_RES				APDS9921_DD_ALS_MEAS_RES_18_BIT
#define APDS9921_DD_ALS_DEFAULT_MEAS_RATE		APDS9921_DD_ALS_MEAS_RATE_100_MS
#define APDS9921_DD_ALS_DEFAULT_GAIN			APDS9921_DD_ALS_GAIN_9 // 19bit adc-->gain_9

#define	APDS9921_DD_PRX_DEFAULT_PULSE			7	// drop to 16 if crosstalk is too high, >=5 pulses
#define APDS9921_DD_PRX_DEFAULT_LED_CURRENT		APDS9921_DD_LED_CURRENT_100_MA
#define APDS9921_DD_PRX_DEFAULT_LED_FREQ		APDS9921_DD_LED_FREQ_60_KHZ
#define APDS9921_DD_PRX_DEFAULT_RES				APDS9921_DD_PRX_MEAS_RES_8_BIT
#define APDS9921_DD_PRX_DEFAULT_MEAS_RATE		APDS9921_DD_PRX_MEAS_RATE_50_MS

/*
 * Structs
 */
struct apds9921_data {
    struct i2c_client *client;

    //struct mutex update_lock;
    struct delayed_work	dwork;		/* for interrupt */
    struct delayed_work	als_dwork;	/* for ALS polling */
    struct delayed_work	ps_dwork;	/* for PS polling */
    struct input_dev *input_dev_als;
    struct input_dev *input_dev_ps;

    int irq;
    int als_suspended;
    int ps_suspended;
    unsigned int main_ctrl_suspended_value;	/* suspend_resume usage */

    unsigned int enable;
    unsigned int atime;
    unsigned int ptime;
    unsigned int wtime;
    unsigned int ailt;
    unsigned int aiht;
    unsigned int pilt;
    unsigned int piht;
    unsigned int pers;
    unsigned int config;
    unsigned int ppcount;
    unsigned int control;
    unsigned int poffset;

    /* control flag from HAL */
    unsigned int enable_ps_sensor;

    unsigned int enable_als_sensor;
#ifdef CONFIG_AAL_CONTROL
    unsigned int enable_als_type;
#endif

    /* PS parameters */
    unsigned int ps_threshold;
    unsigned int ps_hysteresis_threshold; 	/* always lower than ps_threshold */
    unsigned int ps_detection;				/* 0 = near-to-far; 1 = far-to-near */
    unsigned int ps_data;					/* to store PS data */
    unsigned int ps_overflow;				/* to store PS overflow flag */
    unsigned int first_int_enable;			/* to force first interrupt */
    unsigned int ps_poll_delay;				/* needed for proximity sensor polling : ms */
    unsigned int ps_offset;					/* needed if crosstalk under cover glass is big */

    /* ALS parameters */
    unsigned int als_threshold_l;	/* low threshold */
    unsigned int als_threshold_h;	/* high threshold */

    unsigned int clr_data;			/* to store CLEAR data */
    unsigned int als_data;			/* to store ALS data */
    int als_prev_lux;				/* to store previous lux value */
    int als_cal_factor;				/* to store the ALS calibration factor */
    int als_cal_loop;				/* loop counter for ALS calibration */

    unsigned int als_gain;			/* needed for Lux calculation */
    unsigned int als_poll_delay;	/* needed for light sensor polling : ms */
    unsigned int als_res_index;		/* storage for als integratiion time */
    unsigned int als_gain_index;	/* storage for als GAIN */
    unsigned int als_reduce;		/* flag indicate ALS 6x reduction */

    struct regulator *dvddio18;
    struct mutex mutex_apds;
    int wakeup_enable;
    int non_wakeup_enable;
    int irq_wakeup_stat;
    unsigned char ps_state_pre;
    int ps_cali_result;
    unsigned char als_cali_result;
    int gpio;
    unsigned char tptype;

};


#include <linux/meizu-sys.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define I2C_PRX_ALS_NAME   "i2c_prx_als_name"
#define INPUT_ALS_NAME     "input_als_apds9922"
#define INPUT_PRX_NAME     "input_prx_apds9922"
#define WAKE      (1)
#define NON_WAKE  (0)
#define CALI_OK   (1)
#define CALI_FAIL (-1)
#define FAR        (1)
#define NEAR       (0)
#define PS_ADC_11_BIT (2048) //2^11
#define ID_PX  (0x01)
#define ID_PXW (0x02)


#define THREDHOLD_UP  (0x003f)
#define THREDHOLD_LOW (0x0038)
#define THREDHOLD_UP_W  (0x005d)
#define THREDHOLD_LOW_W (0x0050)
#define OFFSET  (0x0060)


struct ps_reg {
    unsigned char reg;
    unsigned char data;
};

extern unsigned char debug;
#define INFOR(fmt,args...)  do { if( debug ) printk(KERN_EMERG "AWAGO: %s " fmt,__func__,##args); } while(0)
