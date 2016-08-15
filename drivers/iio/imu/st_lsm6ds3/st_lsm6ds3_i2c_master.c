/*
 * STMicroelectronics lsm6ds3 i2c master driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <asm/unaligned.h>

#include "st_lsm6ds3.h"

#define ST_LSM6DS3_EXT0_INDEX			0
#define ST_LSM6DS3_EXT1_INDEX			1

#define ST_LSM6DS3_I2C_MASTER_ODR_LIST_NUM	4
#define ST_LSM6DS3_EN_BIT			0x01
#define ST_LSM6DS3_DIS_BIT			0x00
#define ST_LSM6DS3_HUB_REG1_ADDR		0x2e
#define ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR		0x01
#define ST_LSM6DS3_FUNC_CFG_REG2_MASK		0x80
#define ST_LSM6DS3_SLV0_ADDR_ADDR		0x02
#define ST_LSM6DS3_SLV0_SUBADDR_ADDR		0x03
#define ST_LSM6DS3_SLV0_CONFIG_ADDR		0x04
#define ST_LSM6DS3_SLV0_CONFIG_MASK		0x07
#define ST_LSM6DS3_SLV1_ADDR_ADDR		0x05
#define ST_LSM6DS3_SLV1_SUBADDR_ADDR		0x06
#define ST_LSM6DS3_SLV1_CONFIG_ADDR		0x07
#define ST_LSM6DS3_SLV1_CONFIG_MASK		0x07
#define ST_LSM6DS3_SLV2_ADDR_ADDR		0x08
#define ST_LSM6DS3_SLV2_SUBADDR_ADDR		0x09
#define ST_LSM6DS3_SLV2_CONFIG_ADDR		0x0a
#define ST_LSM6DS3_SLV2_CONFIG_MASK		0x07
#define ST_LSM6DS3_SLV_AUX_ADDR			0x04
#define ST_LSM6DS3_SLV_AUX_MASK			0x30
#define ST_LSM6DS3_SLV_AUX_1			0x00
#define ST_LSM6DS3_SLV_AUX_2			0x01
#define ST_LSM6DS3_SLV_AUX_3			0x02

/* External sensors configuration */
#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_LIS3MDL
static int lis3mdl_initialization(struct lsm6ds3_sensor_data *sdata,
								int ext_num);

#define ST_LSM6DS3_EXT0_ADDR			0x1e
#define ST_LSM6DS3_EXT0_WAI_ADDR		0x0f
#define ST_LSM6DS3_EXT0_WAI_VALUE		0x3d
#define ST_LSM6DS3_EXT0_RESET_ADDR		0x21
#define ST_LSM6DS3_EXT0_RESET_MASK		0x04
#define ST_LSM6DS3_EXT0_FULLSCALE_ADDR		0x21
#define ST_LSM6DS3_EXT0_FULLSCALE_MASK		0x60
#define ST_LSM6DS3_EXT0_FULLSCALE_VALUE		0x02
#define ST_LSM6DS3_EXT0_ODR_ADDR		0x20
#define ST_LSM6DS3_EXT0_ODR_MASK		0x1c
#define ST_LSM6DS3_EXT0_ODR0_HZ			10
#define ST_LSM6DS3_EXT0_ODR0_VALUE		0x04
#define ST_LSM6DS3_EXT0_ODR1_HZ			20
#define ST_LSM6DS3_EXT0_ODR1_VALUE		0x05
#define ST_LSM6DS3_EXT0_ODR2_HZ			40
#define ST_LSM6DS3_EXT0_ODR2_VALUE		0x06
#define ST_LSM6DS3_EXT0_ODR3_HZ			80
#define ST_LSM6DS3_EXT0_ODR3_VALUE		0x07
#define ST_LSM6DS3_EXT0_PW_ADDR			0x22
#define ST_LSM6DS3_EXT0_PW_MASK			0x03
#define ST_LSM6DS3_EXT0_PW_OFF			0x02
#define ST_LSM6DS3_EXT0_PW_ON			0x00
#define ST_LSM6DS3_EXT0_GAIN_VALUE		438
#define ST_LSM6DS3_EXT0_OUT_X_L_ADDR		0x28
#define ST_LSM6DS3_EXT0_OUT_Y_L_ADDR		0x2a
#define ST_LSM6DS3_EXT0_OUT_Z_L_ADDR		0x2c
#define ST_LSM6DS3_EXT0_READ_DATA_LEN		6
#define ST_LSM6DS3_EXT0_BDU_ADDR		0x24
#define ST_LSM6DS3_EXT0_BDU_MASK		0x40
#define ST_LSM6DS3_EXT0_STD			3
#define ST_LSM6DS3_EXT0_BOOT_FUNCTION		(&lis3mdl_initialization)
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_LIS3MDL */

#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912
static int akm09912_initialization(struct lsm6ds3_sensor_data *sdata,
								int ext_num);

#define ST_LSM6DS3_EXT0_ADDR			0x0c
#define ST_LSM6DS3_EXT0_WAI_ADDR		0x01
#define ST_LSM6DS3_EXT0_WAI_VALUE		0x04
#define ST_LSM6DS3_EXT0_RESET_ADDR		0x32
#define ST_LSM6DS3_EXT0_RESET_MASK		0x01
#define ST_LSM6DS3_EXT0_FULLSCALE_ADDR		0x00
#define ST_LSM6DS3_EXT0_FULLSCALE_MASK		0x00
#define ST_LSM6DS3_EXT0_FULLSCALE_VALUE		0x00
#define ST_LSM6DS3_EXT0_ODR_ADDR		0x31
#define ST_LSM6DS3_EXT0_ODR_MASK		0x1f
#define ST_LSM6DS3_EXT0_ODR0_HZ			10
#define ST_LSM6DS3_EXT0_ODR0_VALUE		0x02
#define ST_LSM6DS3_EXT0_ODR1_HZ			20
#define ST_LSM6DS3_EXT0_ODR1_VALUE		0x04
#define ST_LSM6DS3_EXT0_ODR2_HZ			50
#define ST_LSM6DS3_EXT0_ODR2_VALUE		0x06
#define ST_LSM6DS3_EXT0_ODR3_HZ			100
#define ST_LSM6DS3_EXT0_ODR3_VALUE		0x08
#define ST_LSM6DS3_EXT0_PW_ADDR			ST_LSM6DS3_EXT0_ODR_ADDR
#define ST_LSM6DS3_EXT0_PW_MASK			ST_LSM6DS3_EXT0_ODR_MASK
#define ST_LSM6DS3_EXT0_PW_OFF			0x00
#define ST_LSM6DS3_EXT0_PW_ON			ST_LSM6DS3_EXT0_ODR0_VALUE
#define ST_LSM6DS3_EXT0_GAIN_VALUE		1500
#define ST_LSM6DS3_EXT0_OUT_X_L_ADDR		0x11
#define ST_LSM6DS3_EXT0_OUT_Y_L_ADDR		0x13
#define ST_LSM6DS3_EXT0_OUT_Z_L_ADDR		0x15
#define ST_LSM6DS3_EXT0_READ_DATA_LEN		6
#define ST_LSM6DS3_EXT0_SENSITIVITY_ADDR	0x60
#define ST_LSM6DS3_EXT0_SENSITIVITY_LEN		3
#define ST_LSM6DS3_EXT0_STD			3
#define ST_LSM6DS3_EXT0_BOOT_FUNCTION		(&akm09912_initialization)
#define ST_LSM6DS3_EXT0_DATA_STATUS		0x18
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */

#ifdef CONFIG_ST_LSM6DS3_IIO_EXT1_LPS22HB
static int lps22hb_initialization(struct lsm6ds3_sensor_data *sdata,
								int ext_num);

#define ST_LSM6DS3_EXT1_ADDR			0x5d
#define ST_LSM6DS3_EXT1_WAI_ADDR		0x0f
#define ST_LSM6DS3_EXT1_WAI_VALUE		0xb1
#define ST_LSM6DS3_EXT1_RESET_ADDR		0x11
#define ST_LSM6DS3_EXT1_RESET_MASK		0x80
#define ST_LSM6DS3_EXT1_FULLSCALE_ADDR		0x00
#define ST_LSM6DS3_EXT1_FULLSCALE_MASK		0x00
#define ST_LSM6DS3_EXT1_FULLSCALE_VALUE		0x00
#define ST_LSM6DS3_EXT1_ODR_ADDR		0x10
#define ST_LSM6DS3_EXT1_ODR_MASK		0x70
#define ST_LSM6DS3_EXT1_ODR0_HZ			1
#define ST_LSM6DS3_EXT1_ODR0_VALUE		0x01
#define ST_LSM6DS3_EXT1_ODR1_HZ			10
#define ST_LSM6DS3_EXT1_ODR1_VALUE		0x02
#define ST_LSM6DS3_EXT1_ODR2_HZ			25
#define ST_LSM6DS3_EXT1_ODR2_VALUE		0x03
#define ST_LSM6DS3_EXT1_ODR3_HZ			50
#define ST_LSM6DS3_EXT1_ODR3_VALUE		0x04
#define ST_LSM6DS3_EXT1_PW_ADDR			ST_LSM6DS3_EXT1_ODR_ADDR
#define ST_LSM6DS3_EXT1_PW_MASK			ST_LSM6DS3_EXT1_ODR_MASK
#define ST_LSM6DS3_EXT1_PW_OFF			0x00
#define ST_LSM6DS3_EXT1_PW_ON			ST_LSM6DS3_EXT1_ODR0_VALUE
#define ST_LSM6DS3_EXT1_GAIN_VALUE		244
#define ST_LSM6DS3_EXT1_OUT_P_L_ADDR		0x28
#define ST_LSM6DS3_EXT1_OUT_T_L_ADDR		0x2b
#define ST_LSM6DS3_EXT1_READ_DATA_LEN		5
#define ST_LSM6DS3_EXT1_BDU_ADDR		0x10
#define ST_LSM6DS3_EXT1_BDU_MASK		0x02
#define ST_LSM6DS3_EXT1_STD			1
#define ST_LSM6DS3_EXT1_BOOT_FUNCTION		(&lps22hb_initialization)
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_LPS22HB */

/* SENSORS SUFFIX NAMES */
#define ST_LSM6DS3_EXT0_SUFFIX_NAME		"magn"
#define ST_LSM6DS3_EXT1_SUFFIX_NAME		"press"

struct st_lsm6ds3_i2c_master_odr_reg {
	unsigned int hz;
	u8 value;
};

struct st_lsm6ds3_i2c_master_odr_table {
	u8 addr;
	u8 mask;
	struct st_lsm6ds3_i2c_master_odr_reg odr_avl[ST_LSM6DS3_I2C_MASTER_ODR_LIST_NUM];
};

static int st_lsm6ds3_i2c_master_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *ch, int *val, int *val2, long mask);

static const struct iio_chan_spec st_lsm6ds3_ext0_ch[] = {
	ST_LSM6DS3_LSM_CHANNELS(IIO_MAGN, 1, 0, IIO_MOD_X, IIO_LE,
				16, 16, ST_LSM6DS3_EXT0_OUT_X_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_MAGN, 1, 1, IIO_MOD_Y, IIO_LE,
				16, 16, ST_LSM6DS3_EXT0_OUT_Y_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_MAGN, 1, 2, IIO_MOD_Z, IIO_LE,
				16, 16, ST_LSM6DS3_EXT0_OUT_Z_L_ADDR, 's'),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
static const struct iio_chan_spec st_lsm6ds3_ext1_ch[] = {
	ST_LSM6DS3_LSM_CHANNELS(IIO_PRESSURE, 0, 0, IIO_NO_MOD, IIO_LE,
				24, 24, ST_LSM6DS3_EXT1_OUT_P_L_ADDR, 'u'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_TEMP, 0, 1, IIO_NO_MOD, IIO_LE,
				16, 16, ST_LSM6DS3_EXT1_OUT_T_L_ADDR, 's'),
	IIO_CHAN_SOFT_TIMESTAMP(2)
};
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */

static int st_lsm6ds3_i2c_master_set_odr(struct lsm6ds3_sensor_data *sdata,
							unsigned int odr);

static ssize_t st_lsm6ds3_i2c_master_sysfs_sampling_frequency_avail(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
				"%d %d %d %d %d\n", 26, 52, 104, 208, 416);
}

static ssize_t st_lsm6ds3_i2c_master_sysfs_get_sampling_frequency(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t st_lsm6ds3_i2c_master_sysfs_set_sampling_frequency(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err, err2 = -EINVAL;
	unsigned int odr;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&indio_dev->mlock);

	switch (odr) {
	case 26:
	case 52:
	case 104:
	case 208:
	case 416:
		if (sdata->c_odr != odr) {

			mutex_lock(&sdata->cdata->passthrough_lock);

			disable_irq(sdata->cdata->irq);
			st_lsm6ds3_flush_works();

			err = st_lsm6ds3_enable_passthrough(sdata->cdata, true);
			if (err < 0) {
				enable_irq(sdata->cdata->irq);
				mutex_unlock(&sdata->cdata->passthrough_lock);
				mutex_unlock(&indio_dev->mlock);
				return err;
			}

			err2 = st_lsm6ds3_i2c_master_set_odr(sdata, odr);

			err = st_lsm6ds3_enable_passthrough(sdata->cdata,
									false);
			if (err < 0) {
				enable_irq(sdata->cdata->irq);
				mutex_unlock(&sdata->cdata->passthrough_lock);
				mutex_unlock(&indio_dev->mlock);
				return err;
			}

			enable_irq(sdata->cdata->irq);
			mutex_unlock(&sdata->cdata->passthrough_lock);
		}
		break;
	default:
		err2 = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);

	return err2 < 0 ? err2 : size;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
			st_lsm6ds3_i2c_master_sysfs_get_sampling_frequency,
			st_lsm6ds3_i2c_master_sysfs_set_sampling_frequency);

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(
			st_lsm6ds3_i2c_master_sysfs_sampling_frequency_avail);
static ST_LSM6DS3_FIFO_LENGHT();
static ST_LSM6DS3_FIFO_FLUSH();

static struct attribute *st_lsm6ds3_ext0_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_ext0_attribute_group = {
	.attrs = st_lsm6ds3_ext0_attributes,
};

static const struct iio_info st_lsm6ds3_ext0_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_ext0_attribute_group,
	.read_raw = &st_lsm6ds3_i2c_master_read_raw,
};

static struct attribute *st_lsm6ds3_ext1_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_ext1_attribute_group = {
	.attrs = st_lsm6ds3_ext1_attributes,
};

static const struct iio_info st_lsm6ds3_ext1_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_ext1_attribute_group,
	.read_raw = &st_lsm6ds3_i2c_master_read_raw,
};

struct st_lsm6ds3_iio_info_data {
	char suffix_name[20];
	struct iio_info *info;
	struct iio_chan_spec *channels;
	int num_channels;
};

struct st_lsm6ds3_reg {
	u8 addr;
	u8 mask;
	u8 def_value;
};

struct st_lsm6ds3_power_reg {
	u8 addr;
	u8 mask;
	u8 off_value;
	u8 on_value;
	bool isodr;
};

struct st_lsm6ds3_custom_function {
	int (*boot_initialization) (struct lsm6ds3_sensor_data *sdata,
								int ext_num);
};

static struct st_lsm6ds3_exs_list {
	struct st_lsm6ds3_reg wai;
	struct st_lsm6ds3_reg reset;
	struct st_lsm6ds3_reg fullscale;
	struct st_lsm6ds3_i2c_master_odr_table odr;
	struct st_lsm6ds3_power_reg power;
	u8 fullscale_value;
	u8 samples_to_discard;
	u8 read_data_len;
	u8 num_data_channels;
	bool available;
	unsigned int gain;
	struct i2c_board_info board_info;
	struct st_lsm6ds3_iio_info_data data;
	struct st_lsm6ds3_custom_function cf;
} st_lsm6ds3_exs_list[] = {
	{
		.wai = {
			.addr = ST_LSM6DS3_EXT0_WAI_ADDR,
			.def_value = ST_LSM6DS3_EXT0_WAI_VALUE,
		},
		.reset = {
			.addr = ST_LSM6DS3_EXT0_RESET_ADDR,
			.mask = ST_LSM6DS3_EXT0_RESET_MASK,
		},
		.fullscale = {
			.addr = ST_LSM6DS3_EXT0_FULLSCALE_ADDR,
			.mask = ST_LSM6DS3_EXT0_FULLSCALE_MASK,
			.def_value = ST_LSM6DS3_EXT0_FULLSCALE_VALUE,
		},
		.odr = {
			.addr = ST_LSM6DS3_EXT0_ODR_ADDR,
			.mask = ST_LSM6DS3_EXT0_ODR_MASK,
			.odr_avl = {
				{
				.hz = ST_LSM6DS3_EXT0_ODR0_HZ,
				.value = ST_LSM6DS3_EXT0_ODR0_VALUE,
				},
				{
				.hz = ST_LSM6DS3_EXT0_ODR1_HZ,
				.value = ST_LSM6DS3_EXT0_ODR1_VALUE,
				},
				{
				.hz = ST_LSM6DS3_EXT0_ODR2_HZ,
				.value = ST_LSM6DS3_EXT0_ODR2_VALUE,
				},
				{
				.hz = ST_LSM6DS3_EXT0_ODR3_HZ,
				.value = ST_LSM6DS3_EXT0_ODR3_VALUE,
				},
			},
		},
		.power = {
			.addr = ST_LSM6DS3_EXT0_PW_ADDR,
			.mask = ST_LSM6DS3_EXT0_PW_MASK,
			.off_value = ST_LSM6DS3_EXT0_PW_OFF,
			.on_value = ST_LSM6DS3_EXT0_PW_ON,
		},
		.samples_to_discard = ST_LSM6DS3_EXT0_STD,
		.read_data_len = ST_LSM6DS3_EXT0_READ_DATA_LEN,
		.num_data_channels = ARRAY_SIZE(st_lsm6ds3_ext0_ch) - 1,
		.available = false,
		.gain = ST_LSM6DS3_EXT0_GAIN_VALUE,
		.board_info = { .addr = ST_LSM6DS3_EXT0_ADDR, },
		.data = {
			.suffix_name = ST_LSM6DS3_EXT0_SUFFIX_NAME,
			.info = (struct iio_info *)&st_lsm6ds3_ext0_info,
			.channels = (struct iio_chan_spec *)&st_lsm6ds3_ext0_ch,
			.num_channels = ARRAY_SIZE(st_lsm6ds3_ext0_ch),
		},
		.cf.boot_initialization = ST_LSM6DS3_EXT0_BOOT_FUNCTION,
	},
#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
	{
		.wai = {
			.addr = ST_LSM6DS3_EXT1_WAI_ADDR,
			.def_value = ST_LSM6DS3_EXT1_WAI_VALUE,
		},
		.reset = {
			.addr = ST_LSM6DS3_EXT1_RESET_ADDR,
			.mask = ST_LSM6DS3_EXT1_RESET_MASK,
		},
		.fullscale = {
			.addr = ST_LSM6DS3_EXT1_FULLSCALE_ADDR,
			.mask = ST_LSM6DS3_EXT1_FULLSCALE_MASK,
			.def_value = ST_LSM6DS3_EXT1_FULLSCALE_VALUE,
		},
		.odr = {
			.addr = ST_LSM6DS3_EXT1_ODR_ADDR,
			.mask = ST_LSM6DS3_EXT1_ODR_MASK,
			.odr_avl = {
				{
				.hz = ST_LSM6DS3_EXT1_ODR0_HZ,
				.value = ST_LSM6DS3_EXT1_ODR0_VALUE,
				},
				{
				.hz = ST_LSM6DS3_EXT1_ODR1_HZ,
				.value = ST_LSM6DS3_EXT1_ODR1_VALUE,
				},
				{
				.hz = ST_LSM6DS3_EXT1_ODR2_HZ,
				.value = ST_LSM6DS3_EXT1_ODR2_VALUE,
				},
				{
				.hz = ST_LSM6DS3_EXT1_ODR3_HZ,
				.value = ST_LSM6DS3_EXT1_ODR3_VALUE,
				},
			},
		},
		.power = {
			.addr = ST_LSM6DS3_EXT1_PW_ADDR,
			.mask = ST_LSM6DS3_EXT1_PW_MASK,
			.off_value = ST_LSM6DS3_EXT1_PW_OFF,
			.on_value = ST_LSM6DS3_EXT1_PW_ON,
		},
		.samples_to_discard = ST_LSM6DS3_EXT1_STD,
		.read_data_len = ST_LSM6DS3_EXT1_READ_DATA_LEN,
		.num_data_channels = ARRAY_SIZE(st_lsm6ds3_ext1_ch) - 1,
		.available = false,
		.gain = ST_LSM6DS3_EXT1_GAIN_VALUE,
		.board_info = { .addr = ST_LSM6DS3_EXT1_ADDR, },
		.data = {
			.suffix_name = ST_LSM6DS3_EXT1_SUFFIX_NAME,
			.info = (struct iio_info *)&st_lsm6ds3_ext1_info,
			.channels = (struct iio_chan_spec *)&st_lsm6ds3_ext1_ch,
			.num_channels = ARRAY_SIZE(st_lsm6ds3_ext1_ch),
		},
		.cf.boot_initialization = ST_LSM6DS3_EXT1_BOOT_FUNCTION,
	},
#else /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */
	{
	},
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */
};

static int st_lsm6ds3_i2c_master_read(struct i2c_client *client,
						u8 reg_addr, int len, u8 *data)
{
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	return i2c_transfer(client->adapter, msg, 2);
}

static int st_lsm6ds3_i2c_master_write(struct i2c_client *client,
					u8 reg_addr, int len, u8 *data)
{
	u8 send[len + 1];
	struct i2c_msg msg;

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	return i2c_transfer(client->adapter, &msg, 1);
}

static int st_lsm6ds3_i2c_master_write_data_with_mask(struct i2c_client *client,
						u8 reg_addr, u8 mask, u8 data)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = st_lsm6ds3_i2c_master_read(client, reg_addr, 1, &old_data);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return st_lsm6ds3_i2c_master_write(client, reg_addr, 1, &new_data);
}

#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_LIS3MDL
static int lis3mdl_initialization(struct lsm6ds3_sensor_data *sdata,
								int ext_num)
{

	return st_lsm6ds3_i2c_master_write_data_with_mask(
				sdata->cdata->master_client[ext_num],
				ST_LSM6DS3_EXT0_BDU_ADDR,
				ST_LSM6DS3_EXT0_BDU_MASK, ST_LSM6DS3_EN_BIT);
}
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_LIS3MDL */

#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912
static int akm09912_initialization(struct lsm6ds3_sensor_data *sdata,
								int ext_num)
{
	int err;
	u8 data[ST_LSM6DS3_EXT0_SENSITIVITY_LEN];

	err = st_lsm6ds3_i2c_master_read(sdata->cdata->master_client[ext_num],
				ST_LSM6DS3_EXT0_SENSITIVITY_ADDR,
				ST_LSM6DS3_EXT0_SENSITIVITY_LEN,
				data);
	if (err < 0)
		return err;

	sdata->c_gain[0] *= ((((data[0] - 128) * 1000) >> 8) + 1000);
	sdata->c_gain[1] *= ((((data[1] - 128) * 1000) >> 8) + 1000);
	sdata->c_gain[2] *= ((((data[2] - 128) * 1000) >> 8) + 1000);

	return 0;
}
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */

#ifdef CONFIG_ST_LSM6DS3_IIO_EXT1_LPS22HB
static int lps22hb_initialization(struct lsm6ds3_sensor_data *sdata,
								int ext_num)
{

	return st_lsm6ds3_i2c_master_write_data_with_mask(
				sdata->cdata->master_client[ext_num],
				ST_LSM6DS3_EXT1_BDU_ADDR,
				ST_LSM6DS3_EXT1_BDU_MASK, ST_LSM6DS3_EN_BIT);
}
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_LPS22HB */

static int st_lsm6ds3_i2c_master_set_odr(struct lsm6ds3_sensor_data *sdata,
							unsigned int odr)
{
	int err, i, ext_num = sdata->sindex - ST_INDIO_DEV_EXT0;

	for (i = 0; i < ST_LSM6DS3_I2C_MASTER_ODR_LIST_NUM; i++) {
		if (st_lsm6ds3_exs_list[ext_num].odr.odr_avl[i].hz >= odr)
			break;
	}
	if (i == ST_LSM6DS3_I2C_MASTER_ODR_LIST_NUM)
		i--;

	if (sdata->cdata->sensors_enabled & (1 << sdata->sindex)) {
		err = st_lsm6ds3_i2c_master_write_data_with_mask(
			sdata->cdata->master_client[ext_num],
			st_lsm6ds3_exs_list[ext_num].odr.addr,
			st_lsm6ds3_exs_list[ext_num].odr.mask,
			st_lsm6ds3_exs_list[ext_num].odr.odr_avl[i].value);
		if (err < 0)
			return err;

		sdata->cdata->ext_samples_to_discard[ext_num] =
				st_lsm6ds3_exs_list[ext_num].samples_to_discard;

		sdata->c_odr = odr;

		if (st_lsm6ds3_exs_list[ext_num].power.isodr)
			st_lsm6ds3_exs_list[ext_num].power.on_value =
				st_lsm6ds3_exs_list[ext_num].odr.odr_avl[i].value;

		st_lsm6ds3_reconfigure_fifo(sdata->cdata, false);
	} else {
		sdata->c_odr = odr;

		if (st_lsm6ds3_exs_list[ext_num].power.isodr)
			st_lsm6ds3_exs_list[ext_num].power.on_value =
				st_lsm6ds3_exs_list[ext_num].odr.odr_avl[i].value;
	}

	return 0;
}

static int st_lsm6ds3_i2c_master_set_enable(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	u8 reg_value;
	int err, ext_num = sdata->sindex - ST_INDIO_DEV_EXT0;

	if (enable)
		reg_value = st_lsm6ds3_exs_list[ext_num].power.on_value;
	else
		reg_value = st_lsm6ds3_exs_list[ext_num].power.off_value;

	err = st_lsm6ds3_i2c_master_write_data_with_mask(
				sdata->cdata->master_client[ext_num],
				st_lsm6ds3_exs_list[ext_num].power.addr,
				st_lsm6ds3_exs_list[ext_num].power.mask,
				reg_value);
	if (err < 0)
		return err;

	sdata->cdata->ext_samples_to_discard[ext_num] =
				st_lsm6ds3_exs_list[ext_num].samples_to_discard;

	if (enable)
		sdata->cdata->sensors_enabled |= (1 << sdata->sindex);
	else
		sdata->cdata->sensors_enabled &= ~(1 << sdata->sindex);

	return 0;
}

static int st_lsm6ds3_i2c_master_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	int err;
	u8 outdata[(ch->scan_type.storagebits >> 3) + 1];
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		mutex_lock(&sdata->cdata->passthrough_lock);

		err = st_lsm6ds3_enable_accel_dependency(sdata, true);
		if (err < 0)
			return err;

		disable_irq(sdata->cdata->irq);
		st_lsm6ds3_flush_works();

		err = st_lsm6ds3_enable_passthrough(sdata->cdata, true);
		if (err < 0)
			goto read_raw_reset_passthrough;

		err = st_lsm6ds3_i2c_master_set_enable(sdata, true);
		if (err < 0)
			goto read_raw_reset_passthrough;

		memset(outdata, 0, (ch->scan_type.storagebits >> 3) + 1);

		msleep(200);

		err = st_lsm6ds3_i2c_master_read(
			sdata->cdata->master_client[sdata->sindex -
				ST_INDIO_DEV_EXT0], ch->address,
				ch->scan_type.storagebits >> 3, outdata);
		if (err < 0)
			goto read_raw_reset_passthrough;

		err = st_lsm6ds3_i2c_master_set_enable(sdata, false);
		if (err < 0)
			goto read_raw_reset_passthrough;

		err = st_lsm6ds3_enable_passthrough(sdata->cdata, false);
		if (err < 0)
			goto read_raw_reset_passthrough;

		enable_irq(sdata->cdata->irq);
		mutex_unlock(&sdata->cdata->passthrough_lock);

		err = st_lsm6ds3_enable_accel_dependency(sdata, false);
		if (err < 0)
			return err;

		if ((ch->scan_type.storagebits >> 3) > 2)
			*val = (s32)get_unaligned_le32(outdata);
		else
			*val = (s16)get_unaligned_le16(outdata);

		*val = *val >> ch->scan_type.shift;

		mutex_unlock(&indio_dev->mlock);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sdata->c_gain[ch->scan_index];

		if (ch->type == IIO_TEMP) {
			*val = 1;
			*val2 = 0;
			return IIO_VAL_INT;
		}

#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912
		if (sdata->sindex == ST_INDIO_DEV_EXT0)
			return IIO_VAL_INT_PLUS_NANO;
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;

read_raw_reset_passthrough:
	st_lsm6ds3_enable_passthrough(sdata->cdata, false);
	enable_irq(sdata->cdata->irq);
	mutex_unlock(&sdata->cdata->passthrough_lock);
	st_lsm6ds3_enable_accel_dependency(sdata, false);
	return err;
}

static int st_lsm6ds3_i2c_master_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	mutex_lock(&sdata->cdata->passthrough_lock);

	err = st_lsm6ds3_enable_accel_dependency(sdata, true);
	if (err < 0)
		return err;

	disable_irq(sdata->cdata->irq);
	st_lsm6ds3_flush_works();

	err = st_lsm6ds3_enable_passthrough(sdata->cdata, true);
	if (err < 0)
		goto preenable_reset_passthrough;

	err = st_lsm6ds3_i2c_master_set_enable(sdata, true);
	if (err < 0)
		goto preenable_reset_passthrough;

	err = st_lsm6ds3_enable_passthrough(sdata->cdata, false);
	if (err < 0)
		goto preenable_reset_passthrough;

	enable_irq(sdata->cdata->irq);
	mutex_unlock(&sdata->cdata->passthrough_lock);

	err = st_lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	return iio_sw_buffer_preenable(indio_dev);

preenable_reset_passthrough:
	st_lsm6ds3_enable_passthrough(sdata->cdata, false);
	enable_irq(sdata->cdata->irq);
	mutex_unlock(&sdata->cdata->passthrough_lock);

	return err;
}

static int st_lsm6ds3_i2c_master_buffer_postenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	sdata->buffer_data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (sdata->buffer_data == NULL)
		return -ENOMEM;

	err = iio_triggered_buffer_postenable(indio_dev);
	if (err < 0)
		goto free_buffer_data;

	return 0;

free_buffer_data:
	kfree(sdata->buffer_data);

	return err;
}

static int st_lsm6ds3_i2c_master_buffer_predisable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = iio_triggered_buffer_predisable(indio_dev);
	if (err < 0)
		return err;

	mutex_lock(&sdata->cdata->passthrough_lock);

	disable_irq(sdata->cdata->irq);
	st_lsm6ds3_flush_works();

	err = st_lsm6ds3_enable_passthrough(sdata->cdata, true);
	if (err < 0)
		goto predisable_reset_passthrough;

	err = st_lsm6ds3_i2c_master_set_enable(sdata, false);
	if (err < 0)
		goto predisable_reset_passthrough;

	err = st_lsm6ds3_enable_passthrough(sdata->cdata, false);
	if (err < 0)
		goto predisable_reset_passthrough;

	enable_irq(sdata->cdata->irq);
	mutex_unlock(&sdata->cdata->passthrough_lock);

	err = st_lsm6ds3_enable_accel_dependency(sdata, false);
	if (err < 0)
		return err;

	err = st_lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	kfree(sdata->buffer_data);

	return 0;

predisable_reset_passthrough:
	st_lsm6ds3_enable_passthrough(sdata->cdata, false);
	enable_irq(sdata->cdata->irq);
	mutex_unlock(&sdata->cdata->passthrough_lock);

	return err;
}

static const struct iio_trigger_ops st_lsm6ds3_i2c_master_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &st_lsm6ds3_trig_set_state,
};

int st_lsm6ds3_i2c_master_allocate_trigger(struct lsm6ds3_data *cdata,
								int dev_index)
{
	int err;

	cdata->trig[dev_index] = iio_trigger_alloc("%s-trigger",
					cdata->indio_dev[dev_index]->name);
	if (!cdata->trig[dev_index]) {
		dev_err(cdata->dev, "failed to allocate iio trigger.\n");
		return -ENOMEM;
	}

	iio_trigger_set_drvdata(cdata->trig[dev_index],
						cdata->indio_dev[dev_index]);
	cdata->trig[dev_index]->ops = &st_lsm6ds3_i2c_master_trigger_ops;
	cdata->trig[dev_index]->dev.parent = cdata->dev;

	err = iio_trigger_register(cdata->trig[dev_index]);
	if (err < 0) {
		dev_err(cdata->dev, "failed to register iio trigger.\n");
		goto deallocate_trigger;
	}

	cdata->indio_dev[dev_index]->trig = cdata->trig[dev_index];

	return 0;

deallocate_trigger:
	iio_trigger_free(cdata->trig[dev_index]);
	return err;
}

static void st_lsm6ds3_i2c_master_deallocate_trigger(struct lsm6ds3_data *cdata,
								int dev_index)
{
	iio_trigger_unregister(cdata->trig[dev_index]);
}

static const struct iio_buffer_setup_ops st_lsm6ds3_i2c_master_buffer_setup_ops = {
	.preenable = &st_lsm6ds3_i2c_master_buffer_preenable,
	.postenable = &st_lsm6ds3_i2c_master_buffer_postenable,
	.predisable = &st_lsm6ds3_i2c_master_buffer_predisable,
};

static inline irqreturn_t st_lsm6ds3_i2c_master_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}

static int st_lsm6ds3_i2c_master_allocate_buffer(struct lsm6ds3_data *cdata,
								int dev_index)
{
	return iio_triggered_buffer_setup(cdata->indio_dev[dev_index],
				&st_lsm6ds3_i2c_master_handler_empty, NULL,
				&st_lsm6ds3_i2c_master_buffer_setup_ops);
}

static void st_lsm6ds3_i2c_master_deallocate_buffer(struct lsm6ds3_data *cdata,
								int dev_index)
{
	iio_triggered_buffer_cleanup(cdata->indio_dev[dev_index]);
}

static int st_lsm6ds3_i2c_master_send_sensor_hub_parameters(
				struct lsm6ds3_sensor_data *sdata, int ext_num)
{
	int err;
	u8 i2c_address_reg, data_start_address_reg, slave_num;
	u8 i2c_address, data_start_address, config_reg_addr, config_reg_mask;
#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912
	u8 i2c_address_reg_akm, data_start_address_reg_akm, temp_reg;
	u8 config_reg_addr_akm, config_reg_mask_akm;
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */

	switch (ext_num) {
	case ST_LSM6DS3_EXT0_INDEX:
		i2c_address_reg = ST_LSM6DS3_SLV0_ADDR_ADDR;
		data_start_address_reg = ST_LSM6DS3_SLV0_SUBADDR_ADDR;
		config_reg_addr = ST_LSM6DS3_SLV0_CONFIG_ADDR;
		config_reg_mask = ST_LSM6DS3_SLV0_CONFIG_MASK;
		break;
	case ST_LSM6DS3_EXT1_INDEX:
		i2c_address_reg = ST_LSM6DS3_SLV1_ADDR_ADDR;
		data_start_address_reg = ST_LSM6DS3_SLV1_SUBADDR_ADDR;
		config_reg_addr = ST_LSM6DS3_SLV1_CONFIG_ADDR;
		config_reg_mask = ST_LSM6DS3_SLV1_CONFIG_MASK;
		break;
	default:
		return -EINVAL;
	}

	i2c_address = (st_lsm6ds3_exs_list[ext_num].board_info.addr << 1) |
							ST_LSM6DS3_EN_BIT;

	data_start_address =
			st_lsm6ds3_exs_list[ext_num].data.channels[0].address;

	mutex_lock(&sdata->cdata->bank_registers_lock);

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata, i2c_address_reg,
					1, &i2c_address, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					data_start_address_reg,
					1, &data_start_address, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				config_reg_addr, config_reg_mask,
				st_lsm6ds3_exs_list[ext_num].read_data_len,
				false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	if (ext_num == ST_LSM6DS3_EXT0_INDEX) {
		if (sdata->cdata->ext1_available) {
#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912
			slave_num = ST_LSM6DS3_SLV_AUX_3;
			i2c_address_reg_akm = ST_LSM6DS3_SLV2_ADDR_ADDR;
			data_start_address_reg_akm = ST_LSM6DS3_SLV2_SUBADDR_ADDR;
			config_reg_addr_akm = ST_LSM6DS3_SLV2_CONFIG_ADDR;
			config_reg_mask_akm = ST_LSM6DS3_SLV2_CONFIG_MASK;
#else /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */
			slave_num = ST_LSM6DS3_SLV_AUX_2;
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */
		} else {
#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912
			slave_num = ST_LSM6DS3_SLV_AUX_2;
			i2c_address_reg_akm = ST_LSM6DS3_SLV1_ADDR_ADDR;
			data_start_address_reg_akm = ST_LSM6DS3_SLV1_SUBADDR_ADDR;
			config_reg_addr_akm = ST_LSM6DS3_SLV1_CONFIG_ADDR;
			config_reg_mask_akm = ST_LSM6DS3_SLV1_CONFIG_MASK;
#else /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */
			slave_num = ST_LSM6DS3_SLV_AUX_1;
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */
		}

		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_SLV_AUX_ADDR,
						ST_LSM6DS3_SLV_AUX_MASK,
						slave_num, false);
		if (err < 0)
			goto st_lsm6ds3_init_sensor_mutex_unlock;

#ifdef CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912
		temp_reg = (ST_LSM6DS3_EXT0_ADDR << 1) | ST_LSM6DS3_EN_BIT;

		err = sdata->cdata->tf->write(sdata->cdata, i2c_address_reg_akm,
						1, &temp_reg, false);
		if (err < 0)
			goto st_lsm6ds3_init_sensor_mutex_unlock;

		temp_reg = ST_LSM6DS3_EXT0_DATA_STATUS;

		err = sdata->cdata->tf->write(sdata->cdata,
						data_start_address_reg_akm,
						1, &temp_reg, false);
		if (err < 0)
			goto st_lsm6ds3_init_sensor_mutex_unlock;

		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				config_reg_addr_akm, config_reg_mask_akm,
				1, false);
		if (err < 0)
			goto st_lsm6ds3_init_sensor_mutex_unlock;
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT0_AKM09912 */
	}

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	mutex_unlock(&sdata->cdata->bank_registers_lock);

	return 0;

st_lsm6ds3_init_sensor_mutex_unlock:
	mutex_unlock(&sdata->cdata->bank_registers_lock);
	return err;
}

static int st_lsm6ds3_i2c_master_init_sensor(struct lsm6ds3_sensor_data *sdata,
						int ext_num, int dev_index)
{
	int err;

	mutex_lock(&sdata->cdata->passthrough_lock);

	err = st_lsm6ds3_enable_passthrough(sdata->cdata, true);
	if (err < 0)
		goto unlock_passthrough;

	sdata->c_gain[0] = st_lsm6ds3_exs_list[ext_num].gain;
	sdata->c_gain[1] = st_lsm6ds3_exs_list[ext_num].gain;
	sdata->c_gain[2] = st_lsm6ds3_exs_list[ext_num].gain;

	if ((st_lsm6ds3_exs_list[ext_num].power.addr ==
				st_lsm6ds3_exs_list[ext_num].odr.addr) &&
			(st_lsm6ds3_exs_list[ext_num].power.mask ==
					st_lsm6ds3_exs_list[ext_num].odr.mask))
		st_lsm6ds3_exs_list[ext_num].power.isodr = true;
	else
		st_lsm6ds3_exs_list[ext_num].power.isodr = false;

	err = st_lsm6ds3_i2c_master_write_data_with_mask(
					sdata->cdata->master_client[ext_num],
					st_lsm6ds3_exs_list[ext_num].reset.addr,
					st_lsm6ds3_exs_list[ext_num].reset.mask,
					ST_LSM6DS3_EN_BIT);
	if (err < 0)
		goto unlock_passthrough;

	usleep_range(3000, 8000);

	if (st_lsm6ds3_exs_list[ext_num].fullscale.addr > 0) {
		err = st_lsm6ds3_i2c_master_write_data_with_mask(
			sdata->cdata->master_client[ext_num],
			st_lsm6ds3_exs_list[ext_num].fullscale.addr,
			st_lsm6ds3_exs_list[ext_num].fullscale.mask,
			st_lsm6ds3_exs_list[ext_num].fullscale.def_value);
		if (err < 0)
			goto unlock_passthrough;
	}

	if (st_lsm6ds3_exs_list[ext_num].cf.boot_initialization != NULL) {
		err = st_lsm6ds3_exs_list[ext_num].cf.boot_initialization(sdata, ext_num);
		if (err < 0)
			goto unlock_passthrough;
	}

	err = st_lsm6ds3_i2c_master_set_enable(sdata, false);
	if (err < 0)
		goto unlock_passthrough;

	err = st_lsm6ds3_i2c_master_set_odr(sdata, 26);
	if (err < 0)
		goto unlock_passthrough;

	err = st_lsm6ds3_i2c_master_send_sensor_hub_parameters(sdata, ext_num);
	if (err < 0)
		goto unlock_passthrough;

	err = st_lsm6ds3_enable_passthrough(sdata->cdata, false);
	if (err < 0)
		goto unlock_passthrough;

	mutex_unlock(&sdata->cdata->passthrough_lock);

	return 0;

unlock_passthrough:
	mutex_unlock(&sdata->cdata->passthrough_lock);
	return err;
}

static int st_lsm6ds3_i2c_master_allocate_device(struct lsm6ds3_data *cdata,
								int ext_num)
{
	int err, dev_index;
	struct lsm6ds3_sensor_data *sdata_ext;

	switch (ext_num) {
	case ST_LSM6DS3_EXT0_INDEX:
		dev_index = ST_INDIO_DEV_EXT0;
		break;
	case ST_LSM6DS3_EXT1_INDEX:
		dev_index = ST_INDIO_DEV_EXT1;
		break;
	default:
		return -EINVAL;
	}

	cdata->indio_dev[dev_index] = iio_device_alloc(sizeof(*sdata_ext));
	if (!cdata->indio_dev[dev_index])
		return -ENOMEM;

	sdata_ext = iio_priv(cdata->indio_dev[dev_index]);
	sdata_ext->cdata = cdata;
	sdata_ext->sindex = dev_index;

	sdata_ext->num_data_channels =
				st_lsm6ds3_exs_list[ext_num].num_data_channels;

	cdata->indio_dev[dev_index]->name = kasprintf(GFP_KERNEL,
				"%s_%s", cdata->name,
				st_lsm6ds3_exs_list[ext_num].data.suffix_name);

	cdata->indio_dev[dev_index]->info =
				st_lsm6ds3_exs_list[ext_num].data.info;
	cdata->indio_dev[dev_index]->channels =
				st_lsm6ds3_exs_list[ext_num].data.channels;
	cdata->indio_dev[dev_index]->num_channels =
				st_lsm6ds3_exs_list[ext_num].data.num_channels;

	cdata->indio_dev[dev_index]->modes = INDIO_DIRECT_MODE;

	err = st_lsm6ds3_i2c_master_init_sensor(sdata_ext, ext_num, dev_index);
	if (err < 0)
		goto iio_device_free;

	err = st_lsm6ds3_i2c_master_allocate_buffer(cdata, dev_index);
	if (err < 0)
		goto iio_device_free;

	err = st_lsm6ds3_i2c_master_allocate_trigger(cdata, dev_index);
	if (err < 0)
		goto iio_deallocate_buffer;

	err = iio_device_register(cdata->indio_dev[dev_index]);
	if (err < 0)
		goto iio_deallocate_trigger;

	return 0;

iio_deallocate_trigger:
	st_lsm6ds3_i2c_master_deallocate_trigger(cdata, dev_index);
iio_deallocate_buffer:
	st_lsm6ds3_i2c_master_deallocate_buffer(cdata, dev_index);
iio_device_free:
	iio_device_free(cdata->indio_dev[dev_index]);

	return err;
}

static void st_lsm6ds3_i2c_master_deallocate_device(struct lsm6ds3_data *cdata,
								int ext_num)
{
	int dev_index;

	switch (ext_num) {
	case ST_LSM6DS3_EXT0_INDEX:
		dev_index = ST_INDIO_DEV_EXT0;
		break;
	case ST_LSM6DS3_EXT1_INDEX:
		dev_index = ST_INDIO_DEV_EXT1;
		break;
	default:
		return;
	}

	iio_device_unregister(cdata->indio_dev[dev_index]);
	st_lsm6ds3_i2c_master_deallocate_trigger(cdata, dev_index);
	st_lsm6ds3_i2c_master_deallocate_buffer(cdata, dev_index);
	iio_device_free(cdata->indio_dev[dev_index]);
}

int st_lsm6ds3_i2c_master_probe(struct lsm6ds3_data *cdata)
{
	u8 wai;
	int err;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	cdata->ext0_available = false;
	cdata->ext1_available = false;

	cdata->ext0_samples_in_pattern = 0;
	cdata->ext1_samples_in_pattern = 0;

	sprintf(st_lsm6ds3_exs_list[ST_LSM6DS3_EXT0_INDEX].board_info.type,
			"%s_ext%d", client->name, ST_LSM6DS3_EXT0_INDEX);

	cdata->master_client[ST_LSM6DS3_EXT0_INDEX] =
			i2c_new_device(client->adapter,
			&st_lsm6ds3_exs_list[ST_LSM6DS3_EXT0_INDEX].board_info);
	if (!cdata->master_client[ST_LSM6DS3_EXT0_INDEX])
		return -ENOMEM;

#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
	sprintf(st_lsm6ds3_exs_list[ST_LSM6DS3_EXT1_INDEX].board_info.type,
			"%s_ext%d", client->name, ST_LSM6DS3_EXT1_INDEX);

	cdata->master_client[ST_LSM6DS3_EXT1_INDEX] =
			i2c_new_device(client->adapter,
			&st_lsm6ds3_exs_list[ST_LSM6DS3_EXT1_INDEX].board_info);
	if (!cdata->master_client[ST_LSM6DS3_EXT1_INDEX]) {
		err = -ENOMEM;
		goto unregister_ext0_i2c_client;
	}
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */

	mutex_lock(&cdata->passthrough_lock);

	err = st_lsm6ds3_enable_passthrough(cdata, true);
	if (err < 0)
		goto master_probe_passthrough_lock;

	err = st_lsm6ds3_i2c_master_read(
		cdata->master_client[ST_LSM6DS3_EXT0_INDEX],
		st_lsm6ds3_exs_list[ST_LSM6DS3_EXT0_INDEX].wai.addr, 1, &wai);
	if (err < 0) {
		dev_err(cdata->dev, "external sensor 0 not available\n");
		i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT0_INDEX]);
	} else {
		if (wai != st_lsm6ds3_exs_list[ST_LSM6DS3_EXT0_INDEX].wai.def_value) {
			dev_err(cdata->dev, "wai value of external sensor 0 mismatch\n");
			i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT0_INDEX]);
		} else
			cdata->ext0_available = true;
	}

#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
	err = st_lsm6ds3_i2c_master_read(
		cdata->master_client[ST_LSM6DS3_EXT1_INDEX],
		st_lsm6ds3_exs_list[ST_LSM6DS3_EXT1_INDEX].wai.addr, 1, &wai);
	if (err < 0) {
		dev_err(cdata->dev, "external sensor 1 not available\n");
		i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT1_INDEX]);
	} else {
		if (wai != st_lsm6ds3_exs_list[ST_LSM6DS3_EXT1_INDEX].wai.def_value) {
			dev_err(cdata->dev, "wai value of external sensor 1 mismatch\n");
			i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT1_INDEX]);
		} else
			cdata->ext1_available = true;
	}
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */

	err = st_lsm6ds3_enable_passthrough(cdata, false);
	if (err < 0) {
		if (cdata->ext0_available)
			i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT0_INDEX]);

#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
		if (cdata->ext1_available)
			i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT1_INDEX]);
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */

		mutex_unlock(&cdata->passthrough_lock);

		return err;
	}

	mutex_unlock(&cdata->passthrough_lock);

	if (cdata->ext0_available) {
		err = st_lsm6ds3_i2c_master_allocate_device(cdata, ST_LSM6DS3_EXT0_INDEX);
		if (err < 0)
			goto unregister_with_check_i2c_clients;

	}

#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
	if (cdata->ext1_available) {
		err = st_lsm6ds3_i2c_master_allocate_device(cdata, ST_LSM6DS3_EXT1_INDEX);
		if (err < 0)
			goto deallocate_ext0_device;

	}
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */

	return 0;

#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
deallocate_ext0_device:
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */
	st_lsm6ds3_i2c_master_deallocate_device(cdata, ST_LSM6DS3_EXT0_INDEX);
unregister_with_check_i2c_clients:
#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
	if (cdata->ext1_available)
		i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT1_INDEX]);
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */
	if (cdata->ext0_available)
		i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT0_INDEX]);

	return err;

master_probe_passthrough_lock:
	mutex_unlock(&cdata->passthrough_lock);
#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
	i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT1_INDEX]);
unregister_ext0_i2c_client:
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */
	i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT0_INDEX]);

	return err;
}
EXPORT_SYMBOL(st_lsm6ds3_i2c_master_probe);

int st_lsm6ds3_i2c_master_exit(struct lsm6ds3_data *cdata)
{
	if (cdata->ext0_available) {
		st_lsm6ds3_i2c_master_deallocate_device(cdata, ST_LSM6DS3_EXT0_INDEX);
		i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT0_INDEX]);
	}
#ifndef CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED
	if (cdata->ext1_available) {
		st_lsm6ds3_i2c_master_deallocate_device(cdata, ST_LSM6DS3_EXT1_INDEX);
		i2c_unregister_device(cdata->master_client[ST_LSM6DS3_EXT1_INDEX]);
	}
#endif /* CONFIG_ST_LSM6DS3_IIO_EXT1_DISABLED */

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_i2c_master_exit);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 i2c master driver");
MODULE_LICENSE("GPL v2");
