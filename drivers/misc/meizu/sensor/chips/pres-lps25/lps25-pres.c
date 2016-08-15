/*
 * Copyright (C) 2015 Meizu.
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

 #include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
//#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>

#include "sensor.h"

// #define LPS25_DEBUG 1
#ifdef LPS25_DEBUG
#define LOG_TAG_LPS25 "[lps25]"
#define pr_info(format, arg...)         printk(KERN_EMERG LOG_TAG_LPS25 format , ## arg)
#define dev_err(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_LPS25 format , ## arg)
#define dev_info(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_LPS25 format , ## arg)
#define dev_dbg(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_LPS25 format , ## arg)
#define dev_warn(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_LPS25 format , ## arg)
#define dev_notice(dev, format, arg...) printk(KERN_EMERG LOG_TAG_LPS25 format , ## arg)
#endif

#define	LPS25_PRS_DEV_NAME		"lps25h"

#define	WHOAMI_LPS25_PRS	0xBD	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define	REF_P_XL	0x08		/*	pressure reference	*/
#define	REF_P_L		0x09		/*	pressure reference	*/
#define	REF_P_H		0x0A		/*	pressure reference	*/
#define	REF_T_L		0x0B		/*	temperature reference	*/
#define	REF_T_H		0x0C		/*	temperature reference	*/


#define	WHO_AM_I	0x0F		/*	WhoAmI register		*/

#define	RESOL_CONF	0x10		/*	Pres Temp resolution set*/
#define	TP_RESOL	0x10		/*	Pres Temp resolution set*/

#define	DGAIN_L		0x18		/*	Dig Gain (3 regs)	*/

#define	CTRL_REG1	0x20		/*	power / ODR control reg	*/
#define	CTRL_REG2	0x21		/*	boot reg		*/
#define	CTRL_REG3	0x22		/*	interrupt control reg	*/
#define	CTRL_REG4	0x23		/*	interrupt control reg	*/
#define	INT_CFG_REG	0x24		/*	interrupt config reg	*/
#define	INT_SRC_REG	0x25		/*	interrupt source reg	*/

#define	STATUS_REG	0X27		/*	status reg		*/

#define	PRESS_OUT_XL	0x28		/*	press output (3 regs)	*/
#define	TEMP_OUT_L	0x2B		/*	temper output (2 regs)	*/

#define	FIFO_CTRL	0x2E
#define	FIFO_STATUS	0x2F

#define	THS_P_L		0x30		/*	pressure threshold	*/
#define	THS_P_H		0x31		/*	pressure threshold	*/


#define	RPDS_TRM_L	0x39		/*	NEW	*/
#define	RPDS_TRM_H	0x3A		/*	NEW	*/


/*	REGISTERS ALIASES	*/
#define	P_REF_INDATA_REG	REF_P_XL
#define	T_REF_INDATA_REG	REF_T_L
#define	P_THS_INDATA_REG	THS_P_L
#define	P_OUTDATA_REG		PRESS_OUT_XL
#define	T_OUTDATA_REG		TEMP_OUT_L
#define	OUTDATA_REG		PRESS_OUT_XL

/* */
#define	ENABLE_MASK		0x80	/*  ctrl_reg1 */
#define	ODR_MASK		0x70	/*  ctrl_reg1 */
#define	DIFF_MASK		0x08	/*  ctrl_reg1 */
#define	BDU_MASK		0x04	/*  ctrl_reg1 */
#define	RESET_AZ		0x02	/*  ctrl_reg1 */

#define	AUTOZ_MASK		0x02	/*  ctrl_reg2 */
#define	AUTOZ_ON		0x02	/* Enab AutoZero Function */
#define	AUTOZ_OFF		0x00	/* Disab Difference Function */
/* Pressure Sensor Operating Mode */
#define	AUTOZ_ENABLE		1
#define	AUTOZ_DISABLE		0


#define	LPS25_PRS_DELTA_EN_MASK	0x02	/*  ctrl_reg1 */
#define	LPS25_PRS_AUTOZ_MASK		0x02	/*  ctrl_reg2 */

#define	LPS25_PRS_RESET_AZ_MASK		0x02	/*  ctrl_reg1 */


#define	PM_NORMAL			0x80	/* Power Normal Mode*/
#define	PM_OFF				0x00	/* Power Down */

#define	LPS25_PRS_DIFF_ON		0x08	/* En Difference circuitry */
#define	LPS25_PRS_DIFF_OFF		0x00	/* Dis Difference circuitry */

#define	LPS25_PRS_AUTOZ_ON		0x02	/* En AutoZero Function */
#define	LPS25_PRS_AUTOZ_OFF		0x00	/* Dis Difference Function */

#define	LPS25_PRS_BDU_ON		0x04	/* En BDU Block Data Upd */

#define	RES_AVGTEMP_064			0X0C
#define	RES_AVGTEMP_032			0X08

#define	RES_AVGPRES_512			0X03
#define	RES_AVGPRES_128			0X02

#define	RES_MAX	(RES_AVGTEMP_064 | RES_AVGPRES_512)	/* Max Resol. */


#define	LPS25_PRS_DELTA_EN_ON	0x02	/* En Delta Press registers */

#define FIFO_MODE_MASK		0xE0	// 3 MSB FIFO_CTRL: Mode
#define FIFO_SAMPLE_MASK	0x1F	// 5 LSB FIFO_CTRL: Num Samples

#define	I2C_AUTO_INCREMENT	0x80

#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5

/* RESUME STATE INDICES */
#define	RES_REF_P_XL		0
#define	RES_REF_P_L		1
#define	RES_REF_P_H		2
#define	RES_REFT_L		3
#define	RES_REFT_H		4
//#define	LPS25_RES_TP_RESOL		5
#define	RES_RESOL_CONF			5
#define	RES_CTRL_REG1		6
#define	RES_CTRL_REG2		7
#define	RES_CTRL_REG3		8
#define	RES_CTRL_REG4		9
#define	RES_INT_CFG_REG		10
#define	RES_FIFO_CTRL		11
#define	RES_THS_P_L		12
#define	RES_THS_P_H		13
#define	RES_RPSD_TRIM_L		14
#define	RES_RPSD_TRIM_H		15

#define	RESUME_ENTRIES		16


/* end RESUME STATE INDICES */

/* Pressure Sensor Operating Mode */
#define	LPS25_PRS_DIFF_ENABLE	1
#define LPS25_PRS_DIFF_DISABLE	0
#define	LPS25_PRS_AUTOZ_ENABLE	1
#define	LPS25_PRS_AUTOZ_DISABLE	0

#define	BDU_ON			0x04	/* En BDU Block Data Upd */

/* Barometer and Termometer output data rate ODR */
#define	ODR_ONESH	0x00	/* one shot both		*/
#define	ODR_1_1		0x10	/*  1  Hz baro,  1  Hz term ODR	*/
#define	ODR_7_7		0x20	/*  7  Hz baro,  7  Hz term ODR	*/
#define	ODR_12_12	0x30	/* 12.5Hz baro, 12.5Hz term ODR	*/
#define	ODR_25_25	0x40	/* 25  Hz baro, 25  Hz term ODR	*/


struct lps25_prs_data {
	struct i2c_client *client;
	struct mutex lock;
	int hw_initialized;
	atomic_t enabled;
	int on_before_suspend;
	u8 resume_state[RESUME_ENTRIES];
	int use_smbus;
};

struct outputdata {
	s32 press;
	s16 temperature;
};

static int lps25_prs_i2c_read(struct lps25_prs_data *prs, u8 *buf, int len)
{
        int ret;
	int tries = 0;
        u8 reg = buf[0];
        u8 cmd = reg;

#ifdef LPS25_DEBUG
        unsigned int ii;
#endif

	if (len > 7) {
		dev_err(&prs->client->dev,
			"read i2c len error, len: %d\n", len);
		return -1;
	}


        if (len > 1)
                cmd = (I2C_AUTO_INCREMENT | reg);
        if (prs->use_smbus) {
                if (len == 1) {
                        ret = i2c_smbus_read_byte_data(prs->client, cmd);
                        buf[0] = ret & 0xff;
#ifdef LPS25_DEBUG
                        dev_warn(&prs->client->dev,
                                "i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
                                "command=0x%02x, buf[0]=0x%02x\n",
                                ret, len, cmd , buf[0]);
#endif
                } else if (len > 1) {
                        ret = i2c_smbus_read_i2c_block_data(prs->client,
                                                                cmd, len, buf);
#ifdef LPS25_DEBUG
                        dev_warn(&prs->client->dev,
                                "i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
                                "command=0x%02x, ",
                                ret, len, cmd);
                        for (ii = 0; ii < len; ii++)
                                printk(KERN_DEBUG "buf[%d]=0x%02x,",
                                                                ii, buf[ii]);

                        printk("\n");
#endif
                } else
                        ret = -1;

                if (ret < 0) {
                        dev_err(&prs->client->dev,
                                "read transfer error: len:%d, command=0x%02x\n",
                                len, cmd);
                        return 0;
                }
                return len;
        }

	do {
		ret = i2c_master_send(prs->client, &cmd, sizeof(cmd));
	        if (ret != sizeof(cmd))
			msleep_interruptible (I2C_RETRY_DELAY);
	}	while ( (ret != sizeof(cmd)) && (++tries < I2C_RETRIES));
        if (ret != sizeof(cmd))
                return ret;
        return i2c_master_recv(prs->client, (char *)buf, len);
}


static int lps25_prs_i2c_write(struct lps25_prs_data *prs,
				   u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = prs->client->addr,
		 .flags = prs->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	if (msgs[0].len > 7) {
		dev_err(&prs->client->dev,
			"write i2c len error, len: %d\n", len);
		return -1;
	}

	do {
		err = i2c_transfer(prs->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible (I2C_RETRY_DELAY);
	}	while ( (err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&prs->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}
	return 0;
}

static int isfirst = 0;
static int lps25_prs_hw_init(struct lps25_prs_data *prs)
{
	int err;
	u8 buf[6];
	isfirst = 1;

	buf[0] = (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
	buf[1] = prs->resume_state[RES_REF_P_XL];
	buf[2] = prs->resume_state[RES_REF_P_L];
	buf[3] = prs->resume_state[RES_REF_P_H];

	err = lps25_prs_i2c_write(prs, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = RESOL_CONF;
	buf[1] = prs->resume_state[RES_RESOL_CONF];
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = prs->resume_state[RES_CTRL_REG2];
	buf[2] = prs->resume_state[RES_CTRL_REG3];
	buf[3] = prs->resume_state[RES_CTRL_REG4];
	err = lps25_prs_i2c_write(prs, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL_REG1;
	buf[1] = prs->resume_state[RES_CTRL_REG1];
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL;
	buf[1] = prs->resume_state[RES_FIFO_CTRL];
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;

	prs->hw_initialized = 1;
	dev_dbg(&prs->client->dev, "%s: hw init done\n", LPS25_PRS_DEV_NAME);
	return 0;

err_resume_state:
	prs->hw_initialized = 0;
	dev_err(&prs->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;	
}

static void lps25_prs_device_power_off(struct lps25_prs_data *prs)
{
	int err;
	u8 buf[2] = { CTRL_REG1, PM_OFF };

	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		dev_err(&prs->client->dev, "soft power off failed: %d\n", err);

	prs->hw_initialized = 0;
}

static int lps25_prs_device_power_on(struct lps25_prs_data *prs)
{
	int err = -1;

	if (!prs->hw_initialized) {
		err = lps25_prs_hw_init(prs);
		if (err < 0) {
			lps25_prs_device_power_off(prs);
			return err;
		}
	}
	return 0;
}

static int lps25_hw_check(struct lps25_prs_data *prs)
{
	unsigned char buf[4];

	memset(buf, 0, 4);
	
	/* get device id of pressure */
	buf[0] = 0x0f;
	lps25_prs_i2c_read(prs, buf, 1);
	msleep(60);
	if (buf[0] != 0xBD) {
		dev_err(&prs->client->dev, "device is error. Replies: 0x%02x\n", buf[0]);
		return -1;
	}

	dev_dbg(&prs->client->dev, "pressure device is lps25\n");

	return 0;
}

static int lps25_prs_get_presstemp_data(struct lps25_prs_data *prs,
						struct outputdata *out)
{
	int err;
	/* Data bytes from hardware	PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H, */
	/*				TEMP_OUT_L, TEMP_OUT_H */

	u8 prs_data[5];

	s32 pressure;
	s16 temperature;

	int regToRead = 5;

	prs_data[0] = (I2C_AUTO_INCREMENT | OUTDATA_REG);
	err = lps25_prs_i2c_read(prs, prs_data, regToRead);
	if (err < 0)
		return err;

#ifdef LPS25_DEBUG
	pr_info("temp out tH = 0x%02x, tL = 0x%02x,"
			"press_out: pH = 0x%02x, pL = 0x%02x, pXL= 0x%02x\n",
					prs_data[4],
					prs_data[3],
					prs_data[2],
					prs_data[1],
					prs_data[0]);
#endif

	pressure = (s32)((((s8) prs_data[2]) << 16) |
				(prs_data[1] <<  8) |
						( prs_data[0]));
	temperature = (s16) ((((s8) prs_data[4]) << 8) | (prs_data[3]));

#ifdef LPS25_DEBUG
	pr_info("\n Pressure: %d ", (int32_t)pressure);
#endif

	out->press = pressure;

	out->temperature = temperature;

	return err;
}

#define LPS25_CALIBBIAS_COUNT 64  /* 2^6 */
static int sensor_pres_lps25_status = 0;
static int sensor_pres_lps25_offset = 0;
static int sensor_pres_lps25_mean = 0;
static struct lps25_prs_data *lps25_prs_temp;
static int sensor_pres_lps25_set_enable(struct sensor_dev *sdev, int state)
{
	sensor_pres_lps25_status = state;
	if (state)
		lps25_prs_device_power_on(lps25_prs_temp);
	else
		lps25_prs_device_power_off(lps25_prs_temp);
	return 0;
}

static int sensor_pres_lps25_get_enable(struct sensor_dev *sdev, int *state)
{
	*state = sensor_pres_lps25_status;
	return 0;
}

static int sensor_pres_lps25_get_data(struct sensor_dev *sdev, int32_t raw[3])
{
	static struct outputdata output;

	if(1 == isfirst){
		msleep(100);
		isfirst = 0;
	}

	mutex_lock(&lps25_prs_temp->lock);
	lps25_prs_get_presstemp_data(lps25_prs_temp, &output);
	mutex_unlock(&lps25_prs_temp->lock);
	//raw[0] = output.press;
	raw[0] = output.press + sensor_pres_lps25_offset;
	raw[1] = output.temperature;
	pr_info("get lps25 press: %d, %d", raw[0], raw[1]);
	return 0;
}

static int sensor_pres_lps25_get_offset(struct sensor_dev *sdev, int32_t offset[3])
{
	offset[0] = sensor_pres_lps25_offset;
	offset[1] = 0;
	offset[2] = 0;
	return 0;
}

static int sensor_pres_lps25_set_offset(struct sensor_dev *sdev, int offset, int axis)
{
	if(0 == axis)
		if((offset > 122880) || (offset < -122880))
			return 0;
		sensor_pres_lps25_offset = offset;
	return 0;
}

static int sensor_pres_lps25_calibrate(struct sensor_dev *sdev)
{
	int i = 0;
	int raw[3] = {0,0,0};
	int sum = 0;
	int adc_min = S32_MAX;
	int adc_max = 0;

	if(1 != sensor_pres_lps25_status) {
		sensor_pres_lps25_set_enable(sdev, 1);
		//sensor_iio_enable(sdev);
	}

	/* dummy data */
	for (i = 0; i<10; i++) {
		sensor_pres_lps25_get_data(sdev,raw);
		msleep(100);
	}

	for (i = 0; i<LPS25_CALIBBIAS_COUNT+2; i++) {
		sensor_pres_lps25_get_data(sdev,raw);

		if (raw[0] < adc_min)
			adc_min = raw[0];

		if (raw[0] > adc_max)
			adc_max = raw[0];

		sum += raw[0];
		msleep(100);//12hz=83.333
	}

	sum -= adc_min;
	sum -= adc_max;

	if(1 != sensor_pres_lps25_status) {
		//sensor_iio_disable(sdev);
		sensor_pres_lps25_set_enable(sdev, 0);
	}

	sensor_pres_lps25_mean = sum >> 6;
	return 0;
}

static int sensor_pres_lps25_get_calibbias(struct sensor_dev *sdev, int32_t calibbias[3])
{
	calibbias[0] = sensor_pres_lps25_mean;
	calibbias[1] = 0;
	calibbias[2] = 0;
	return 0;
}

static int sensor_pres_lps25_get_version(struct sensor_dev *sdev, const char **version)
{
	*version = "2015-11-19 16:30";
	return 0;
}

struct sensor_ops sensor_ops_pres_lps25 = {
	.set_enable    = sensor_pres_lps25_set_enable,
	.get_enable    = sensor_pres_lps25_get_enable,
	.get_raw_data  = sensor_pres_lps25_get_data,
	.get_offset    = sensor_pres_lps25_get_offset,
	.set_offset    = sensor_pres_lps25_set_offset,
	.get_calibbias = sensor_pres_lps25_get_calibbias,
	.calibrate     = sensor_pres_lps25_calibrate,
	.get_version   = sensor_pres_lps25_get_version,
};

static int lps25_prs_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int err = -1;
	struct lps25_prs_data *prs;
	u32 smbus_func = 0;

	pr_info("%s: probe start.\n", LPS25_PRS_DEV_NAME);

	smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
						I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	prs = kzalloc(sizeof(struct lps25_prs_data), GFP_KERNEL);
	if (prs == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto err1;
	}

	prs->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)){
			prs->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto err2;
		}
	}

	msleep(100);

	mutex_init(&prs->lock);
	mutex_lock(&prs->lock);

	prs->client = client;
	i2c_set_clientdata(client, prs);

	err = lps25_hw_check(prs);
	if(err < 0)
		goto err3;

	msleep(60);

	memset(prs->resume_state, 0, ARRAY_SIZE(prs->resume_state));
	/* init registers which need values different from zero */
	prs->resume_state[RES_RESOL_CONF] = 0x0F;
	prs->resume_state[RES_CTRL_REG1] = 0xA4;
	prs->resume_state[RES_CTRL_REG2] = 0x40;
	prs->resume_state[RES_FIFO_CTRL] = 0xDF;

	err = lps25_prs_device_power_on(prs);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err3;
	}
	atomic_set(&prs->enabled, 1);

	lps25_prs_device_power_off(prs);
	atomic_set(&prs->enabled, 0);

	mutex_unlock(&prs->lock);

	lps25_prs_temp  = prs;
	sensor_pres_create("lps25hb-pres", &sensor_ops_pres_lps25, NULL);
	dev_info(&client->dev, "%s: probed end\n", LPS25_PRS_DEV_NAME);

	return 0;

err3:
	mutex_unlock(&prs->lock);
err2:
	kfree(prs);
err1:
	pr_err("%s: Driver Init failed\n", LPS25_PRS_DEV_NAME);
	return err;
}

static const struct i2c_device_id lps25_prs_id[]
		= { { LPS25_PRS_DEV_NAME, 0}, { },};

MODULE_DEVICE_TABLE(i2c, lps25_prs_id);

static struct i2c_driver lps25_prs_driver = {
	.driver = {
			.name = LPS25_PRS_DEV_NAME,
			.owner = THIS_MODULE,
	},
	.probe = lps25_prs_probe,
	//.remove = lps25_prs_remove,
	.id_table = lps25_prs_id,
	//.resume = lps25_prs_resume,
	//.suspend = lps25_prs_suspend,
};

static struct i2c_board_info i2c_lps25[] = {
	{
		I2C_BOARD_INFO(LPS25_PRS_DEV_NAME, 0x5c)
	}
};

static int __init lps25_prs_init(void)
{
	pr_info("lps25_prs_init\n");
	i2c_register_board_info(1, i2c_lps25, 1);
	return 0;
}

rootfs_initcall(lps25_prs_init);
module_i2c_driver(lps25_prs_driver);

MODULE_AUTHOR("Hu Jian <hujian@meizu.com>");
MODULE_DESCRIPTION("Meizu Pressure linux driver");
MODULE_LICENSE("GPL");
