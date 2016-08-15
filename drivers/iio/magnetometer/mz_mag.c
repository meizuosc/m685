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
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <asm/unaligned.h>

#include <linux/meizu-sys.h>
#include <linux/meizu-sensors.h>

#define MZ_MAG_DEV_NAME "mz_mag"
#define MZ_MAG_I2C_NAME "mz_mag_i2c"
#define MZ_MAG_DEFAULT_MIN_POLLRATE 5
#define MZ_MAG_DEFAULT_POLLRATE 100
#define MZ_MAG_DEFAULT_RANGE 2
#define ABSMIN_MAG	-32767
#define ABSMAX_MAG	32767

#define	I2C_RETRY_DELAY     5      /* Waiting for signals [ms] */
#define	I2C_RETRIES         5      /* Number of retries */
#define	I2C_AUTO_INCREMENT  0x80   /* Autoincrement i2c address */

#define MZ_MAG_DEBUG 0
#ifdef MZ_MAG_DEBUG
#define LOG_TAG_MZ_MAG "[mz_mag]"
#define pr_info(format, arg...)         printk(KERN_EMERG LOG_TAG_MZ_MAG format , ## arg)
#define dev_err(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_MZ_MAG format , ## arg)
#define dev_info(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_MZ_MAG format , ## arg)
#define dev_dbg(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_MZ_MAG format , ## arg)
#define dev_warn(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_MZ_MAG format , ## arg)
#define dev_notice(dev, format, arg...) printk(KERN_EMERG LOG_TAG_MZ_MAG format , ## arg)
#endif

/*
	axis_map_x, axis_map_y, axis_map_z can
	be used to swap x, y and z axes.
	If axis_map_x is 0 then it will be mapped to x-axis in android,
	if it is 1 then it will be mapped to y axis in Android and so on.

	To change the axes direction we can set negate_x,
	negate_y and negate_z value to 1.
	If negate_x, negate_y and negate_z values are set to 1
	then driver will negate accelerometer axes readings
	before passing to user space (Android).
*/
struct mz_mag_rotation {
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

struct mz_mag_data;

struct mz_mag_ops
{
	int (*enable_device)(struct mz_mag_data *mz_mag);
	int (*disable_device)(struct mz_mag_data *mz_mag);
	int (*set_range)(struct mz_mag_data *mz_mag, int range);
	int (*get_range)(struct mz_mag_data *mz_mag);
	int (*set_pollrate)(struct mz_mag_data *mz_mag, int pollrate);
	int (*get_pollrate)(struct mz_mag_data *mz_mag);
	int (*get_mag_data)(struct mz_mag_data *mz_mag, int16_t *mag_buf);
	int (*self_test)(struct mz_mag_data *mz_mag);
};

struct mz_mag_data {

	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work input_work;

	struct iio_dev *indio_dev;
	struct iio_trigger *trig;
	u8    *iio_buffer_data;
	struct workqueue_struct *iio_workq;
	struct delayed_work iio_work;

	struct mutex lock;

	struct mz_mag_rotation *rota;
	struct mz_mag_ops *ops;

	uint16_t device_id;
	char    *device_name;
	uint8_t  sens[3];
	atomic_t enabled;
	int      range;
	int      pollrate;
	int      odr;
	int      min_pollrate;

	struct hrtimer mag_timer;
	u8    *buffer_data;
	int   num;
};



/* ====================================================
	st480 Driver
   ==================================================== */

#define ST480_DEVICE_ID 0x7C
#define ST480_DEVICE_NAME "st480"
/*
 * register shift
 */
#define ST480_REG_DRR_SHIFT 2

/*
 * BURST MODE(INT)
 */
#define ST480_BURST_MODE 0
#define BURST_MODE_CMD 0x1F
#define BURST_MODE_DATA_LOW 0x01

/*
 * SINGLE MODE
 */
#define ST480_SINGLE_MODE 1
#define SINGLE_MEASUREMENT_MODE_CMD 0x3F

/*
 * register
 */
#define READ_MEASUREMENT_CMD 0x4F
#define WRITE_REGISTER_CMD 0x60
#define READ_REGISTER_CMD 0x50
#define EXIT_REGISTER_CMD 0x80
#define MEMORY_RECALL_CMD 0xD0
#define MEMORY_STORE_CMD 0xE0
#define RESET_CMD 0xF0

#define CALIBRATION_REG (0x02 << ST480_REG_DRR_SHIFT)
#define CALIBRATION_DATA_LOW 0x1C
#define CALIBRATION_DATA_HIGH 0x00

#define ONE_INIT_DATA_LOW 0x7C
#define ONE_INIT_DATA_HIGH 0x00
#define ONE_INIT_REG (0x00 << ST480_REG_DRR_SHIFT)

#define TWO_INIT_DATA_LOW 0x00
#define TWO_INIT_DATA_HIGH 0x00
#define TWO_INIT_REG (0x02 << ST480_REG_DRR_SHIFT)

#define TEMP_DATA_LOW 0x00
#define TEMP_DATA_HIGH 0x00
#define TEMP_REG (0x01 << ST480_REG_DRR_SHIFT)

static int st480_i2c_transfer_data(struct i2c_client *client,
							int len, char *buf, int length)
{

	int ret;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr  =  client->addr,
			.flags  =  0,
			.len  =  len,
			.buf  =  buf,
		},
		{
			.addr  =  client->addr,
			.flags  = I2C_M_RD,
			.len  =  length,
			.buf  =  buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 2) && (++tries < I2C_RETRIES));

	if (ret != 2) {
		dev_err(&client->dev, "read transfer error\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;

}

/* the MTK platform can only use dma to transfer more tha 8 bytes data */
static u8 *gpDMABuf_va;
static dma_addr_t gpDMABuf_pa;

static char * st480_read_measurement(struct i2c_client *client)
{
	int ret = 0;
	char cmd[1] = {READ_MEASUREMENT_CMD};

	struct i2c_msg msgs[] = {
		{
			.addr  =  client->addr,
			.flags  =  0,
			.len  =  1,
			.buf  =  cmd,
			//.scl_rate = 400 * 1000,
		},
		{
			.addr  =  client->addr,
			/* the ext_flag is mtk specific */
			//.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags  = I2C_M_RD,
			.len  =  9,
			.buf  =  (u8 *)gpDMABuf_pa,
			//.scl_rate = 400 * 1000,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if(ret < 0){
		printk(KERN_EMERG "st480_i2c_transfer_data i2c_transfer error, ret: %d\n", ret);
		return NULL;
	}

	return gpDMABuf_va;

}


static int st480_self_test(struct mz_mag_data *mz_mag)
{
	int    ret;
	char * buffer;
	char * xyz_data;
	s16    hw_data_z0;
	s16    hw_data_z1;
	s16    z_bist;
	unsigned char buf[5];

	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, buf, 1);

	msleep(500);

	/* read the normal z data */
	buffer = st480_read_measurement(mz_mag->client);
	if (NULL == buffer) {
		dev_err(&mz_mag->client->dev, "Failed to read measurement\n");
		ret = -1;
		goto err1;
	}

	xyz_data   = buffer + 3;
	hw_data_z0 = (xyz_data[4]<<8)|xyz_data[5];

	/* enter self test mode */
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = 0x01;
	buf[2] = 0x7C;
	buf[3] = ONE_INIT_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	msleep(250);

	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, buf, 1);

	msleep(500);

	/* read the self test z data */
	buffer = st480_read_measurement(mz_mag->client);
	if (NULL == buffer) {
		dev_err(&mz_mag->client->dev, "Failed to read measurement\n");
		ret = -1;
		goto err2;
	}

	xyz_data   = buffer + 3;
	hw_data_z1 = (xyz_data[4]<<8)|xyz_data[5];

	/* calculate z bist */
	z_bist = hw_data_z1 - hw_data_z0;
	dev_dbg(&mz_mag->client->dev, "z1: %d, z0: %d, z_bist: %d\n",
		hw_data_z1, hw_data_z0, z_bist);
	if (z_bist >= -180 && z_bist <= -90) {
		dev_dbg(&mz_mag->client->dev, "st480 self test success!\n");
		ret = 0;
	} else {
		dev_err(&mz_mag->client->dev, "self test failed\n");
		ret = -1;
	}

err2:
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = 0x00;
	buf[2] = 0x7C;
	buf[3] = ONE_INIT_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);
err1:
	return ret;

}

#define READ_MEASUREMENT_XYZ_CMD        (0x4E)
#define READ_MEASUREMENT_T_CMD          (0x41)
static int st480_get_mag_data(struct mz_mag_data *mz_mag, int16_t * mag_data)
{
	int    ret;
	char   cmd[1];
	char * buffer;
	char * temp_data;
	char * xyz_data;
	s16    hw_data[3];
	int16_t mag_x, mag_y, mag_z;

	buffer = st480_read_measurement(mz_mag->client);
	if (NULL == buffer) {
		dev_err(&mz_mag->client->dev, "Failed to read measurement\n");
		ret = -1;
		goto out;
	}

	temp_data = buffer + 1;
	xyz_data  = buffer + 3;

	if(!((buffer[0]>>4) & 0X01)) {

		hw_data[0] = (xyz_data[0]<<8)|xyz_data[1];
		hw_data[1] = (xyz_data[2]<<8)|xyz_data[3];
		hw_data[2] = (xyz_data[4]<<8)|xyz_data[5];

		mag_x = ((mz_mag->rota->negate_x)?(-hw_data[mz_mag->rota->axis_map_x])
				: (hw_data[mz_mag->rota->axis_map_x]));
		mag_y = ((mz_mag->rota->negate_y)?(-hw_data[mz_mag->rota->axis_map_y])
				: (hw_data[mz_mag->rota->axis_map_y]));
		mag_z = ((mz_mag->rota->negate_z)?(-hw_data[mz_mag->rota->axis_map_z])
				: (hw_data[mz_mag->rota->axis_map_z]));

		if( ((temp_data[0]<<8)|(temp_data[1])) > 46244) {

			mag_x = mag_x * (1 + (70/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_y = mag_y * (1 + (70/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_z = mag_z * (1 + (70/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));

		} else if( ((temp_data[0]<<8)|(temp_data[1])) < 46244) {

			mag_x = mag_x * (1 + (60/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_y = mag_y * (1 + (60/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
			mag_z = mag_z * (1 + (60/128/4096)
						  * (((temp_data[0]<<8)|(temp_data[1])) - 46244));
		}
		dev_dbg(&mz_mag->client->dev,
				"st480 raw data: x = %d, y = %d, z = %d \n",
				mag_x,mag_y,mag_z);

		mag_data[0] = mag_x;
		mag_data[1] = mag_y;
		mag_data[2] = mag_z;
		ret = 0;

	} else {

		dev_dbg(&mz_mag->client->dev,
			"get data state: 0x%X \n", buffer[0]);
		dev_dbg(&mz_mag->client->dev,
			"temp data: 0x%02X 0x%02X\n", temp_data[0], temp_data[1]);
		dev_dbg(&mz_mag->client->dev,
			"xyz  data: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n",
			xyz_data[0], xyz_data[1], xyz_data[2],
			xyz_data[3], xyz_data[4], xyz_data[5]);

		ret = -1;
	}

out:
	cmd[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, cmd, 1);
	return ret;
}

struct mz_mag_ops st480_ops = {
	.get_mag_data = st480_get_mag_data,
	.self_test    = st480_self_test,
};

struct mz_mag_rotation st480_rota = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x   = 1,
	.negate_y   = 1,
	.negate_z   = 0,
};
static int st480_hw_init(struct mz_mag_data *mz_mag)
{
	unsigned char buf[5];

#define ST480_DMA_MAX_TRANSACTION_LENGTH  255
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&mz_mag->client->dev,
		ST480_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va){
		printk(KERN_EMERG "Allocate DMA I2C Buffer failed!\n");
		return -1;
	}

	memset(buf, 0, 5);

	buf[0] = READ_REGISTER_CMD;
	buf[1] = 0x00;
	st480_i2c_transfer_data(mz_mag->client, 2, buf, 3);

	if(buf[2] != ST480_DEVICE_ID) {
		dev_dbg(&mz_mag->client->dev, "mag device is not st480\n");
		return -1;
	}

	dev_dbg(&mz_mag->client->dev, "mag device is st480\n");
	dev_dbg(&mz_mag->client->dev, "device id: 0x00%02X\n", buf[2]);

	mz_mag->device_id    = ST480_DEVICE_ID;
	mz_mag->device_name  = ST480_DEVICE_NAME;
	mz_mag->rota         = &st480_rota;
	mz_mag->ops          = &st480_ops;
	mz_mag->min_pollrate = 5;

	//init register step 1
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = ONE_INIT_DATA_HIGH;
	buf[2] = ONE_INIT_DATA_LOW;
	buf[3] = ONE_INIT_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//init register step 2
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TWO_INIT_DATA_HIGH;
	buf[2] = TWO_INIT_DATA_LOW;
	buf[3] = TWO_INIT_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//disable temperature compensation register
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TEMP_DATA_HIGH;
	buf[2] = TEMP_DATA_LOW;
	buf[3] = TEMP_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//set calibration register
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = CALIBRATION_DATA_HIGH;
	buf[2] = CALIBRATION_DATA_LOW;
	buf[3] = CALIBRATION_REG;
	st480_i2c_transfer_data(mz_mag->client, 4, buf, 1);

	//set mode config
	buf[0] = SINGLE_MEASUREMENT_MODE_CMD;
	st480_i2c_transfer_data(mz_mag->client, 1, buf, 1);

	return 0;
}


/* ====================================================
	akm09911 Driver
   ==================================================== */



#define AKM09911_DEVICE_ID    0x0548
#define AKM09911_DEVICE_NAME  "ak09911"
#define AKM09911_REG_WIA1     0x00
#define AKM09911_REG_WIA2     0x01
#define AKM09911_REG_INFO1    0x02
#define AKM09911_REG_INFO2    0x03
#define AKM09911_REG_ST1      0x10
#define AKM09911_REG_DATA     AKM09911_REG_HXL
#define AKM09911_REG_HXL      0x11
#define AKM09911_REG_HXH      0x12
#define AKM09911_REG_HYL      0x13
#define AKM09911_REG_HYH      0x14
#define AKM09911_REG_HZL      0x15
#define AKM09911_REG_HZH      0x16
#define AKM09911_REG_TMPS     0x17
#define AKM09911_REG_ST2      0x18
#define AKM09911_REG_CNTL1    0x30
#define AKM09911_REG_CNTL2    0x31
#define AKM09911_REG_CNTL3    0x32

#define AKM09911_FUSE_ASAX     0x60
#define AKM09911_FUSE_ASAY     0x61
#define AKM09911_FUSE_ASAZ     0x62

#define AKM09911_MODE_SNG_MEASURE   0x01
#define AKM09911_MODE_SELF_TEST     0x10
#define AKM09911_MODE_FUSE_ACCESS   0x1F
#define AKM09911_MODE_POWERDOWN     0x00
#define AKM09911_RESET_DATA         0x01


static int akm09911_i2c_read(struct i2c_client *client,
				u8 * buf, int len)
{
	int ret;
	int tries = 0;
	u8 reg = buf[0];
	u8 cmd = reg;

	do {
		ret = i2c_master_send(client, &cmd, sizeof(cmd));
		if (ret != sizeof(cmd))
			msleep_interruptible (I2C_RETRY_DELAY);
	} while ( (ret != sizeof(cmd)) && (++tries < I2C_RETRIES));
        
	if (ret != sizeof(cmd))
	{
		dev_err(&client->dev, "i2c_master_send error,ret = %d\n",ret);
		return ret;
	}
	
	ret = i2c_master_recv(client, (char *)buf, len);
	if (ret < 0)
	{
		dev_err(&client->dev, "i2c_master_recv error,ret = %d\n",ret);
		return ret;
	}
	return ret;
}


static int akm09911_i2c_write(struct i2c_client *client, u8 * buf,
								int len)
{
	int ret;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 1) && (++tries < I2C_RETRIES));

	if (ret != 1) {
		dev_err(&client->dev, "write transfer error\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}


static int akm09911_get_mag_data(struct mz_mag_data *mz_mag, int16_t * mag_data)
{
	int ret;
	u8 buf[9];
	int hw_data[3];
	u8 *sens;
	int16_t mag_x, mag_y, mag_z;

	sens = mz_mag->sens;

	buf[0] = AKM09911_REG_ST1;
	ret = akm09911_i2c_read(mz_mag->client, buf, 1);
	if (ret < 0) {
		goto out;
	}

	if (!(buf[0] & 0x01)) {
		dev_dbg(&mz_mag->client->dev, "data not ready\n");
		ret = -1;
		goto out;
	}

	buf[0] = AKM09911_REG_DATA;
	ret = akm09911_i2c_read(mz_mag->client, buf, 8);
	if (ret < 0) {
		ret = -1;
		goto out;
	}

	if (buf[7] & 0x08) {
		dev_dbg(&mz_mag->client->dev, "magnetic sensor overflow\n");
		ret = -1;
		goto out;
	}

	hw_data[0] = (int)((int16_t)(buf[1]<<8)+((int16_t)buf[0]));
	hw_data[1] = (int)((int16_t)(buf[3]<<8)+((int16_t)buf[2]));
	hw_data[2] = (int)((int16_t)(buf[5]<<8)+((int16_t)buf[4]));

	hw_data[0] = ((hw_data[0] * (sens[0] + 128)) >> 7);
	hw_data[1] = ((hw_data[1] * (sens[1] + 128)) >> 7);
	hw_data[2] = ((hw_data[2] * (sens[2] + 128)) >> 7);

#if 0
	mag_x = -hw_data[0];
	mag_y =  hw_data[1];
	mag_z = -hw_data[2];
#else
	mag_x = ((mz_mag->rota->negate_x)
			? (-hw_data[mz_mag->rota->axis_map_x])
			: (hw_data[mz_mag->rota->axis_map_x]));
	mag_y = ((mz_mag->rota->negate_y)
			? (-hw_data[mz_mag->rota->axis_map_y])
			: (hw_data[mz_mag->rota->axis_map_y]));
	mag_z = ((mz_mag->rota->negate_z)
			? (-hw_data[mz_mag->rota->axis_map_z])
			: (hw_data[mz_mag->rota->axis_map_z]));
#endif

#if 0
	dev_dbg(&mz_mag->client->dev, "mag data: %d %d %d\n",
		mag_x, mag_y, mag_z);
#endif

	mag_data[0] = mag_x;
	mag_data[1] = mag_y;
	mag_data[2] = mag_z;
	ret = 0;

out:
	buf[0] = 0x31;
	buf[1] = 0x01;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	return ret;
}

static int akm09911_self_test(struct mz_mag_data *mz_mag)
{
	int i;
	int ret;
	int retry;
	int hw_data[3];
	unsigned char buf[8];
	int ak09911_st_lower[3] = {-50, -50, -400};
	int ak09911_st_upper[3] = {50, 50, -100};

	/* power down mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_POWERDOWN;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	/* self test mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_SELF_TEST;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	/* wait for data ready */
	retry = 10;
	while (retry--) {
		msleep(100);
		buf[0] = AKM09911_REG_ST1;
		ret = akm09911_i2c_read(mz_mag->client, buf, 1);
		if (ret < 0) {
			goto err1;
		}

		if (!(buf[0] & 0x01)) {
			dev_dbg(&mz_mag->client->dev, "data not ready\n");
			continue;
		} else {
			dev_dbg(&mz_mag->client->dev, "data is ready\n");
			break;
		}
	}

	if (!(buf[0] & 0x01)) {
		dev_err(&mz_mag->client->dev, "failed to wait data ready\n");
		ret = -1;
		goto err2;
	}

	/* read sensor data */
	buf[0] = AKM09911_REG_DATA;
	ret = akm09911_i2c_read(mz_mag->client, buf, 8);
	if (ret < 0) {
		ret = -1;
		goto err2;
	}

	if (buf[7] & 0x08) {
		dev_dbg(&mz_mag->client->dev, "magnetic sensor overflow\n");
		ret = -1;
		goto err2;
	}

	hw_data[0] = (int)((int16_t)(buf[1]<<8)+((int16_t)buf[0]));
	hw_data[1] = (int)((int16_t)(buf[3]<<8)+((int16_t)buf[2]));
	hw_data[2] = (int)((int16_t)(buf[5]<<8)+((int16_t)buf[4]));

	hw_data[0] = ((hw_data[0] * (mz_mag->sens[0] + 128)) >> 7);
	hw_data[1] = ((hw_data[1] * (mz_mag->sens[1] + 128)) >> 7);
	hw_data[2] = ((hw_data[2] * (mz_mag->sens[2] + 128)) >> 7);

	dev_dbg(&mz_mag->client->dev, "data:  %d %d %d\n",
		hw_data[0], hw_data[1], hw_data[2]);
	dev_dbg(&mz_mag->client->dev, "upper: %d %d %d\n",
		ak09911_st_upper[0], ak09911_st_upper[1], ak09911_st_upper[2]);
	dev_dbg(&mz_mag->client->dev, "lower: %d %d %d\n",
		ak09911_st_lower[0], ak09911_st_lower[1], ak09911_st_lower[2]);

	ret = 0;
	for (i=0; i<3; i++) {
		if (hw_data[i]>ak09911_st_upper[i]
		 || hw_data[i]<ak09911_st_lower[i]) {
			dev_err(&mz_mag->client->dev, "self test failed\n");
			ret = -1;
			break;
		}
	}

	if (ret == 0) {
		dev_dbg(&mz_mag->client->dev, "self test success\n");
	}

err2:
err1:
	/* revert to power down mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_POWERDOWN;
	akm09911_i2c_write(mz_mag->client, buf, 1);
	return ret;
}

struct mz_mag_ops akm09911_ops = {
	.get_mag_data = akm09911_get_mag_data,
	.self_test    = akm09911_self_test,
};

struct mz_mag_rotation akm09911_rota = {
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x   = 0,
	.negate_y   = 0,
	.negate_z   = 1,
};

static int akm09911_hw_init(struct mz_mag_data *mz_mag)
{
	unsigned char buf[4];
	memset(buf, 0, 4);

	printk("i2c client name = %s, address = 0x%x, adapter name = %s\n", mz_mag->client->name, 
		mz_mag->client->addr, mz_mag->client->adapter->name);

	/* get device id of magnetometer */
	buf[0] = AKM09911_REG_WIA1;
	akm09911_i2c_read(mz_mag->client, buf, 2);

	if (AKM09911_DEVICE_ID != (buf[1]<<8|buf[0])) {
		dev_dbg(&mz_mag->client->dev, "mag device is not akm09911\n");
		return -1;
	}

	dev_dbg(&mz_mag->client->dev, "mag device is akm09911\n");
	dev_dbg(&mz_mag->client->dev, "device id: 0x%02X%02X\n", buf[1], buf[0]);

	mz_mag->device_id = AKM09911_DEVICE_ID;
	mz_mag->device_name = AKM09911_DEVICE_NAME;
	mz_mag->rota      = &akm09911_rota;
	mz_mag->ops       = &akm09911_ops;

	/* set AKM to Fuse ROM access mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_FUSE_ACCESS;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	/* read sens */
	buf[0] = AKM09911_FUSE_ASAX;
	akm09911_i2c_read(mz_mag->client, buf, 3);

	dev_dbg(&mz_mag->client->dev, "mag sens: 0x%X 0x%X 0x%X\n",
		buf[0], buf[1], buf[2]);

	mz_mag->sens[0] = buf[0];
	mz_mag->sens[1] = buf[1];
	mz_mag->sens[2] = buf[2];

	/* revert to power down mode */
	buf[0] = AKM09911_REG_CNTL2;
	buf[1] = AKM09911_MODE_POWERDOWN;
	akm09911_i2c_write(mz_mag->client, buf, 1);

	return 0;
}


/* ====================================================
	af8133 Driver
   ==================================================== */

/*AF8133 register address*/
#define AF8133_REG_PCODE	0x00
#define AF8133_REG_STATUS1	0x02
#define AF8133_REG_DATA  	0x03
#define AF8133_REG_STATUS	0x0A
#define AF8133_REG_RANGE1	0x0B
#define AF8133_REG_SWR		0x11
#define AF8133_DEVICE_ID	0x50

#define FIND_SW_OFFSET_LOOP    5
#define FIND_SW_OFFSET_INDEX   2

static int mag_pos[3][FIND_SW_OFFSET_LOOP];
static int mag_neg[3][FIND_SW_OFFSET_LOOP];
static int mag_offset[3];
static int mag_cnt=0;

static int af8133_i2c_read(struct i2c_client *client,
				u8 * buf, int len)
{
	int ret;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = (client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 2) && (++tries < I2C_RETRIES));

	if (ret != 2) {
		dev_err(&client->dev, "read transfer error\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}


static int af8133_i2c_write(struct i2c_client *client, u8 * buf,
								int len)
{
	int ret;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((ret != 1) && (++tries < I2C_RETRIES));

	if (ret != 1) {
		dev_err(&client->dev, "write transfer error\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}


static int af8133_get_mag_data(struct mz_mag_data *mz_mag, int16_t * mag_data)
{
	int ret;
	u8 buf[9];
	int hw_data[3];
	u8 *sens;
	int i, j, k;
	int16_t mag_x, mag_y, mag_z;

	sens = mz_mag->sens;

	buf[0] = AF8133_REG_STATUS1;
	ret = af8133_i2c_read(mz_mag->client, buf, 1);
	if (ret < 0) {
		dev_dbg(&mz_mag->client->dev, "read status1 failed\n");
		goto out;
	}

	if (!(buf[0] & 0x01)) {
		dev_dbg(&mz_mag->client->dev, "data not ready\n");
		ret = -1;
		goto out;
	}

	buf[0] = AF8133_REG_DATA;
	ret = af8133_i2c_read(mz_mag->client, buf, 6);
	if (ret < 0) {
		dev_dbg(&mz_mag->client->dev, "read data failed\n");
		ret = -1;
		goto out;
	}

	hw_data[0] = (int)((int16_t)(buf[1]<<8)+((int16_t)buf[0]));
	hw_data[1] = (int)((int16_t)(buf[3]<<8)+((int16_t)buf[2]));
	hw_data[2] = (int)((int16_t)(buf[5]<<8)+((int16_t)buf[4]));


	for(i = 0; i < 3; i++)
		hw_data[i] = (hw_data[i] > 32767) ? (hw_data[i] - 65536) : hw_data[i];

	if(mag_cnt >= 0 && mag_cnt <= (FIND_SW_OFFSET_LOOP*2)) {

		/*
		   Using the set and reset function to calculate the offset.
		   offset = (value_set + value_reset) / 2
		 */
		if(mag_cnt >= 1 && mag_cnt <= FIND_SW_OFFSET_LOOP) {
			for(i = 0; i < 3; i++)
				mag_neg[i][mag_cnt-1] = hw_data[i];
		} else if (mag_cnt > FIND_SW_OFFSET_LOOP && mag_cnt <= (FIND_SW_OFFSET_LOOP*2)) {
			for(i = 0; i < 3; i++)
				mag_pos[i][mag_cnt-FIND_SW_OFFSET_LOOP-1] = hw_data[i];
		}

		mag_cnt++;

		if(mag_cnt >= 1 && mag_cnt <= FIND_SW_OFFSET_LOOP) {
			buf[0] = 0x14;
			buf[1] = 0x34;
			ret = af8133_i2c_write(mz_mag->client, buf, 1);
		}
		else if(mag_cnt > FIND_SW_OFFSET_LOOP && mag_cnt <= (FIND_SW_OFFSET_LOOP*2))
		{
			buf[0] = 0x14;
			buf[1] = 0x38;
			ret = af8133_i2c_write(mz_mag->client, buf, 1);
		}

		if(mag_cnt > (FIND_SW_OFFSET_LOOP*2)) {
			for(i=0;i<3;i++) {
				for(j=0;j<(FIND_SW_OFFSET_LOOP-1);j++) {
					for(k=0;k<(FIND_SW_OFFSET_LOOP-1);k++){
						if(mag_neg[i][k] < mag_neg[i][k+1]) {
							int tmp = mag_neg[i][k];
							mag_neg[i][k] = mag_neg[i][k+1];
							mag_neg[i][k+1] = tmp;
						}

						if(mag_pos[i][k] < mag_pos[i][k+1]) {
							int tmp = mag_pos[i][k];
							mag_pos[i][k] = mag_pos[i][k+1];
							mag_pos[i][k+1] = tmp;
						}
					}
				}
				mag_offset[i] = (mag_pos[i][(FIND_SW_OFFSET_INDEX)] + mag_neg[i][FIND_SW_OFFSET_INDEX]) / 2;
			}
		}

	} else {

		buf[0] = 0x14;
		buf[1] = 0x38;
		ret = af8133_i2c_write(mz_mag->client, buf, 1);
		if(ret < 0) return ret;
	}

	for(i = 0; i < 3; i++)
		hw_data[i] -= mag_offset[i];

	//dev_dbg(&mz_mag->client->dev, "hw_data: %d %d %d\n", hw_data[0], hw_data[1], hw_data[2]);
#if 1
	/* the rotation is in the hal layer */
	mag_x = hw_data[0];
	mag_y = hw_data[1];
	mag_z = hw_data[2];
#else
	mag_x = ((mz_mag->rota->negate_x)
			? (-hw_data[mz_mag->rota->axis_map_x])
			: (hw_data[mz_mag->rota->axis_map_x]));
	mag_y = ((mz_mag->rota->negate_y)
			? (-hw_data[mz_mag->rota->axis_map_y])
			: (hw_data[mz_mag->rota->axis_map_y]));
	mag_z = ((mz_mag->rota->negate_z)
			? (-hw_data[mz_mag->rota->axis_map_z])
			: (hw_data[mz_mag->rota->axis_map_z]));
#endif

#if 0
	dev_dbg(&mz_mag->client->dev, "mag data: %d %d %d\n",
		mag_x, mag_y, mag_z);
#endif

	mag_data[0] = mag_x;
	mag_data[1] = mag_y;
	mag_data[2] = mag_z;
	ret = 0;

out:
	buf[0] = AF8133_REG_STATUS;
	buf[1] = 0x01;
	af8133_i2c_write(mz_mag->client, buf, 1);

	return ret;
}

struct mz_mag_ops af8133_ops = {
	.get_mag_data = af8133_get_mag_data,
};

struct mz_mag_rotation af8133_rota = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x   = 0,
	.negate_y   = 0,
	.negate_z   = 0,
};

static int af8133_hw_init(struct mz_mag_data *mz_mag)
{
	unsigned char buf[4];

	memset(buf, 0, 4);

	/* get device id of magnetometer */
	buf[0] = AF8133_REG_PCODE;
	af8133_i2c_read(mz_mag->client, buf, 1);

	if (AF8133_DEVICE_ID != buf[0]) {
		dev_dbg(&mz_mag->client->dev, "mag device is not af8133, id:0x%x\n", buf[0]);
		return -1;
	}

	dev_dbg(&mz_mag->client->dev, "mag device is af8133\n");
	dev_dbg(&mz_mag->client->dev, "device id: 0x%02X\n", buf[0]);

	mz_mag->device_id = AF8133_DEVICE_ID;
	mz_mag->rota      = &af8133_rota;
	mz_mag->ops       = &af8133_ops;

	mz_mag->sens[0] = 0;
	mz_mag->sens[1] = 0;
	mz_mag->sens[2] = 0;

	buf[0] = 0x10;
	buf[1] = 0x55;
	af8133_i2c_write(mz_mag->client, buf, 1);

	buf[0] = 0x14;
	buf[1] = 0x34;
	af8133_i2c_write(mz_mag->client, buf, 1);

	buf[0] = 0x33;
	buf[1] = 0x16;
	af8133_i2c_write(mz_mag->client, buf, 1);

	buf[0] = 0x0B;
	buf[1] = 0x3C;
	af8133_i2c_write(mz_mag->client, buf, 1);

	buf[0] = 0x13;
	buf[1] = 0x00;
	af8133_i2c_write(mz_mag->client, buf, 1);

	buf[0] = 0x0A;
	buf[1] = 0x01;
	af8133_i2c_write(mz_mag->client, buf, 1);

	mag_cnt       = 0;
	mag_offset[0] = 0;
	mag_offset[1] = 0;
	mag_offset[2] = 0;

	return 0;
}



/* ====================================================
	Meizu Magnetometer Driver
   ==================================================== */

/* sysfs interfaces */

static inline int mz_mag_enable(struct mz_mag_data *mz_mag)
{
	int ret = 0;

	if (!atomic_cmpxchg(&mz_mag->enabled, 0, 1)) {

		if (mz_mag->ops->enable_device)
			ret = mz_mag->ops->enable_device(mz_mag);

		if (ret < 0) {
			atomic_set(&mz_mag->enabled, 0);
			return ret;
		}

		dev_dbg(&mz_mag->client->dev, "mag input work start\n");
		schedule_delayed_work(&mz_mag->input_work,
			msecs_to_jiffies(mz_mag->pollrate));
	}

	return 0;
}

static inline int mz_mag_disable(struct mz_mag_data *mz_mag)
{
	if (atomic_cmpxchg(&mz_mag->enabled, 1, 0)) {

		dev_dbg(&mz_mag->client->dev, "mag input work stop\n");
		cancel_delayed_work_sync(&mz_mag->input_work);

		if (mz_mag->ops->disable_device)
			mz_mag->ops->disable_device(mz_mag);
	}

	return 0;
}


static inline int mz_mag_iio_enable(struct mz_mag_data *mz_mag)
{
	int ret = 0;

	if (!atomic_cmpxchg(&mz_mag->enabled, 0, 1)) {

		if (mz_mag->ops->enable_device)
			ret = mz_mag->ops->enable_device(mz_mag);

		if (ret < 0) {
			atomic_set(&mz_mag->enabled, 0);
			return ret;
		}

		dev_dbg(&mz_mag->client->dev, "mag iio work start\n");
		queue_delayed_work(mz_mag->iio_workq, &mz_mag->iio_work,
			msecs_to_jiffies(mz_mag->pollrate));
		hrtimer_start(&mz_mag->mag_timer, ns_to_ktime(3 * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}

	return 0;
}

static inline int mz_mag_iio_disable(struct mz_mag_data *mz_mag)
{
	if (atomic_cmpxchg(&mz_mag->enabled, 1, 0)) {

		dev_dbg(&mz_mag->client->dev, "mag iio work stop\n");
		cancel_delayed_work_sync(&mz_mag->iio_work);

		if (mz_mag->ops->disable_device)
			mz_mag->ops->disable_device(mz_mag);
		hrtimer_cancel(&mz_mag->mag_timer);
	}

	return 0;
}


static ssize_t attr_pollrate_ms_show(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);

	mutex_lock(&mz_mag->lock);
	val = mz_mag->pollrate;
	mutex_unlock(&mz_mag->lock);

	dev_dbg(&mz_mag->client->dev, "get mag pollrate: %d\n", val);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_pollrate_ms_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	unsigned long rate;

	dev_dbg(&mz_mag->client->dev, "set mag pollrate: %s\n", buf);

	if (kstrtoul(buf, 10, &rate))
		return -EINVAL;
	if (!rate)
		return -EINVAL;

	rate = rate > mz_mag->min_pollrate ? rate : mz_mag->min_pollrate;

	mutex_lock(&mz_mag->lock);

	if (mz_mag->ops->set_pollrate)
		ret = mz_mag->ops->set_pollrate(mz_mag, rate);

	if(ret >= 0) {
		mz_mag->pollrate = rate;
	}

	mutex_unlock(&mz_mag->lock);

	return size;
}


static ssize_t attr_full_scale_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	int range = 2;

	mutex_lock(&mz_mag->lock);

	range = mz_mag->range;

	mutex_unlock(&mz_mag->lock);

	dev_dbg(&mz_mag->client->dev, "get mag range: %d\n", range);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_full_scale_store(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	unsigned long range;

	dev_dbg(&mz_mag->client->dev, "set mag range: %s\n", buf);

	if (kstrtoul(buf, 10, &range))
		return -EINVAL;

	mutex_lock(&mz_mag->lock);

	if (mz_mag->ops->set_range)
		mz_mag->ops->set_range(mz_mag, range);

	mz_mag->range = range;
	mutex_unlock(&mz_mag->lock);

	return size;
}

static ssize_t attr_enable_device_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	int val = atomic_read(&mz_mag->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_device_store(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		mz_mag_enable(mz_mag);
	else
		mz_mag_disable(mz_mag);

	return size;
}

static ssize_t attr_self_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 1;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);

	mutex_lock(&mz_mag->lock);

	if (mz_mag->ops->self_test)
		ret = mz_mag->ops->self_test(mz_mag);

	mutex_unlock(&mz_mag->lock);

	return sprintf(buf, "%d\n", ret);
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0664, attr_pollrate_ms_show, attr_pollrate_ms_store),
	__ATTR(full_scale, 0664, attr_full_scale_show, attr_full_scale_store),
	__ATTR(enable_device, 0664, attr_enable_device_show,
		attr_enable_device_store),
	__ATTR(compass_self_test, S_IRUGO, attr_self_test_show, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}
#if 0
static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
#endif

/* input device interfaces */

static void mz_mag_input_func(struct work_struct *work)
{
	int ret;
	struct mz_mag_data *mz_mag;
	int16_t mag_data[3];

	mz_mag = container_of((struct delayed_work *)work,
					struct mz_mag_data, input_work);

	if (mz_mag->ops->get_mag_data)
		ret = mz_mag->ops->get_mag_data(mz_mag, mag_data);
	else
		ret = -1;

	if (!ret) {
		input_report_abs(mz_mag->input_dev, ABS_X, mag_data[0]);
		input_report_abs(mz_mag->input_dev, ABS_Y, mag_data[1]);
		input_report_abs(mz_mag->input_dev, ABS_Z, mag_data[2]);
		input_event(mz_mag->input_dev, EV_SYN, SYN_REPORT, mz_mag->device_id);
	}

	schedule_delayed_work(&mz_mag->input_work,
					msecs_to_jiffies(mz_mag->pollrate));
}

static int mz_mag_input_init(struct mz_mag_data *mz_mag)
{
	int ret;

	INIT_DELAYED_WORK(&mz_mag->input_work, mz_mag_input_func);

	mz_mag->input_dev = input_allocate_device();
	if (!mz_mag->input_dev) {
		ret = -ENOMEM;
		dev_err(&mz_mag->client->dev, "Failed to allocate input device\n");
		goto err0;
	}

	/* Setup input device */
	set_bit(EV_ABS, mz_mag->input_dev->evbit);

	input_set_capability(mz_mag->input_dev, EV_ABS, ABS_X);
	input_set_capability(mz_mag->input_dev, EV_ABS, ABS_Y);
	input_set_capability(mz_mag->input_dev, EV_ABS, ABS_Z);

	/* x-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(mz_mag->input_dev, ABS_X, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* y-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(mz_mag->input_dev, ABS_Y, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
	/* z-axis of raw magnetic vector (-32768, 32767) */
	input_set_abs_params(mz_mag->input_dev, ABS_Z, ABSMIN_MAG, ABSMAX_MAG, 0, 0);

	/* Set name */
	mz_mag->input_dev->name       = MZ_MAG_DEV_NAME;
	mz_mag->input_dev->dev.parent = &mz_mag->client->dev;

	/* Register */
	ret = input_register_device(mz_mag->input_dev);
	if (ret) {
		dev_err(&mz_mag->client->dev, "Failed to register input device\n");
		goto err1;
	}
	input_set_drvdata(mz_mag->input_dev, mz_mag);

	return 0;

err1:
	input_free_device(mz_mag->input_dev);
	mz_mag->input_dev = NULL;

err0:
	return ret;
}

/* iio device interfaces */

#define MZ_MAG_IIO_NUMBER_DATA_CHANNELS		3
#define MZ_MAG_IIO_BYTE_FOR_CHANNEL		2
static void mz_mag_iio_func(struct work_struct *work)
{
	int ret;
	int i, n = 0;
	struct mz_mag_data *mz_mag;
	int16_t mag_data[3];
	int64_t timestamp;
	struct timespec ts;

	mz_mag = container_of((struct delayed_work *)work,
					struct mz_mag_data, iio_work);

	//dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);

	if (mz_mag->ops->get_mag_data)
		ret = mz_mag->ops->get_mag_data(mz_mag, mag_data);
	else
		ret = -1;

	if (!ret) {
		//dev_dbg(&mz_mag->client->dev, "iio_buffer_data: ================\n");
		for (i = 0; i < MZ_MAG_IIO_NUMBER_DATA_CHANNELS; i++) {

			if (mz_mag->indio_dev->active_scan_mask == NULL) {
				printk(KERN_EMERG "mag bug: active_scan_mask: %p\n",
					mz_mag->indio_dev->active_scan_mask);
				return;
			}

			if (test_bit(i, mz_mag->indio_dev->active_scan_mask)) {
				memcpy(&mz_mag->iio_buffer_data[n * MZ_MAG_IIO_BYTE_FOR_CHANNEL],
						&mag_data[i],
						MZ_MAG_IIO_BYTE_FOR_CHANNEL);

				memcpy(&mz_mag->buffer_data[n * MZ_MAG_IIO_BYTE_FOR_CHANNEL],
					   &mag_data[i],
					   MZ_MAG_IIO_BYTE_FOR_CHANNEL);

				#if 0
				dev_dbg(&mz_mag->client->dev, "%d ",
					*(int16_t *)&mz_mag->iio_buffer_data[n * MZ_MAG_IIO_BYTE_FOR_CHANNEL]);
				#endif
				n++;
			}
		}

		mz_mag->num = n;

		//timestamp = iio_get_time_ns();
		get_monotonic_boottime(&ts);
		timestamp = timespec_to_ns(&ts);
		if (mz_mag->indio_dev->scan_timestamp) {
			*(s64 *)((u8 *)mz_mag->iio_buffer_data +
				ALIGN(n * MZ_MAG_IIO_BYTE_FOR_CHANNEL,
						sizeof(s64))) = timestamp;
				#if 0
				dev_dbg(&mz_mag->client->dev, "%lld ",
					*(s64 *)((u8 *)mz_mag->iio_buffer_data +
					ALIGN(n * MZ_MAG_IIO_BYTE_FOR_CHANNEL,
						sizeof(s64))));
				#endif
		}
		//dev_dbg(&mz_mag->client->dev, "\niio_buffer_data ============= \n");
		//dev_dbg(&mz_mag->client->dev, "%s: push buffer\n", __func__);
#if 0
		iio_push_to_buffers(mz_mag->indio_dev, mz_mag->iio_buffer_data);
#endif
	}

	queue_delayed_work(mz_mag->iio_workq, &mz_mag->iio_work,
					msecs_to_jiffies(mz_mag->pollrate));
	//dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
}


int mz_mag_trig_set_state(struct iio_trigger *trig, bool state)
{
	int ret;
	struct mz_mag_data *mz_mag = iio_device_get_drvdata(
		iio_trigger_get_drvdata(trig));

	dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);
	if (state) {
		ret = mz_mag_iio_enable(mz_mag);
	} else {
		ret = mz_mag_iio_disable(mz_mag);
	}

	return ret;
}


static const struct iio_trigger_ops mz_mag_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &mz_mag_trig_set_state,
};

static inline irqreturn_t mz_mag_iio_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}


static int mz_mag_iio_buffer_preenable(struct iio_dev *indio_dev)
{
	//return iio_sw_buffer_preenable(indio_dev);
	return 0;
}

static int mz_mag_iio_buffer_postenable(struct iio_dev *indio_dev)
{
	int err;
	struct mz_mag_data *mz_mag = iio_device_get_drvdata(indio_dev);
	dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);

	err = iio_triggered_buffer_postenable(indio_dev);
	if (err < 0) {
		dev_err(&mz_mag->client->dev,
			"failed to call iio_triggered_buffer_postenable\n");
		return err;
	}

	dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
	return 0;
}

static int mz_mag_iio_buffer_predisable(struct iio_dev *indio_dev)
{
	int err;
	struct mz_mag_data *mz_mag = iio_device_get_drvdata(indio_dev);
	dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);
	err = iio_triggered_buffer_predisable(indio_dev);
	if (err < 0) {
		dev_err(&mz_mag->client->dev,
			"failed to call iio_triggered_buffer_predisable\n");
		return err;
	}

	dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
	return 0;
}

static int mz_mag_iio_buffer_postdisable(struct iio_dev *indio_dev)
{
	int err;
	struct mz_mag_data *mz_mag = iio_device_get_drvdata(indio_dev);
	dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);
	mz_mag_iio_disable(mz_mag);
	dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
	return 0;
}

static const struct iio_buffer_setup_ops mz_mag_iio_buffer_setup_ops = {
	.preenable  = &mz_mag_iio_buffer_preenable,
	.postenable = &mz_mag_iio_buffer_postenable,
	.predisable = &mz_mag_iio_buffer_predisable,
	.postdisable = &mz_mag_iio_buffer_postdisable,
};


static int mz_mag_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	u8 outdata[2];
	struct mz_mag_data *mz_mag = iio_device_get_drvdata(indio_dev);
	dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
			return -EBUSY;
		}

		/* get data from device */
		/* outdata = xxx */

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		mutex_unlock(&indio_dev->mlock);
		dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* the scale is always 1 */
		*val = 1;
		*val2 = 0;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
	dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
	return 0;
}


#define MZ_MAG_IIO_CHANNELS(device_type, index, mod, endian, bits, addr) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = 's', \
		.realbits = bits, \
		.shift = 16 - bits, \
		.storagebits = 16, \
		.endianness = endian, \
	}, \
}

#define MZ_MAG_X_L_ADDR		0x28
#define MZ_MAG_Y_L_ADDR		0x2a
#define MZ_MAG_Z_L_ADDR		0x2c

static const struct iio_chan_spec mz_mag_iio_ch[] = {
	MZ_MAG_IIO_CHANNELS(IIO_MAGN, 0, IIO_MOD_X, IIO_LE,
					16, MZ_MAG_X_L_ADDR),
	MZ_MAG_IIO_CHANNELS(IIO_MAGN, 1, IIO_MOD_Y, IIO_LE,
					16, MZ_MAG_Y_L_ADDR),
	MZ_MAG_IIO_CHANNELS(IIO_MAGN, 2, IIO_MOD_Z, IIO_LE,
					16, MZ_MAG_Z_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

/* we use the hw_fifo_len to identify which ic we are using */
#define ST480_HW_FIFO_LEN   (1)
#define AK09911_HW_FIFO_LEN (2)
#define AF8133_HW_FIFO_LEN  (3)
ssize_t mz_mag_iio_sysfs_get_hw_fifo_lenght(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int hw_fifo_lenght;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	if (mz_mag->device_id == ST480_DEVICE_ID) {
		hw_fifo_lenght = ST480_HW_FIFO_LEN;
	} else if (mz_mag->device_id == AKM09911_DEVICE_ID) {
		hw_fifo_lenght = AK09911_HW_FIFO_LEN;
	} else if (mz_mag->device_id == AF8133_DEVICE_ID) {
		hw_fifo_lenght = AF8133_HW_FIFO_LEN;
	} else {
		hw_fifo_lenght = 0;
	}
	return sprintf(buf, "%d\n", hw_fifo_lenght);
}

ssize_t mz_mag_iio_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t mz_mag_iio_sysfs_sampling_frequency_avail(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
				"%d %d %d %d %d\n", 26, 52, 104, 208, 416);
}

static ssize_t mz_mag_iio_sysfs_get_sampling_frequency(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);
	dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);
	return sprintf(buf, "%d\n", mz_mag->odr);
}

static ssize_t mz_mag_iio_sysfs_set_sampling_frequency(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err, err2 = -EINVAL;
	int ret = 0;
	unsigned int odr;
	int      rate;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);

	dev_dbg(&mz_mag->client->dev, "%s: enter\n", __func__);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0) {
		dev_err(&mz_mag->client->dev, "kstrtoint error\n");
		return err;
	}

	dev_dbg(&mz_mag->client->dev, "set up odr: %d\n", odr);
#if 1
	mutex_lock(&mz_mag->indio_dev->mlock);

	switch (odr) {
	case 26:
	case 52:
		rate = 1000 / 52;
		rate = rate > mz_mag->min_pollrate ? rate : mz_mag->min_pollrate;

		if(ret >= 0) {
			mz_mag->pollrate = rate;
			mz_mag->odr      = 52;
			err2 = 0;
		}

		break;
	case 104:
		mz_mag->pollrate = 1000 / 120;
		mz_mag->odr      = 120;
		err2 = 0;
			break;
	case 208:
	case 416:
		mz_mag->pollrate = 1000 / 208;
		mz_mag->odr      = 208;
		err2 = 0;
		break;
	default:
		mz_mag->pollrate = 1000 / 104;
		mz_mag->odr      = 104;
		err2 = 0;
		break;
	}

	mutex_unlock(&mz_mag->indio_dev->mlock);
#endif
	dev_dbg(&mz_mag->client->dev, "%s: leave\n", __func__);
	return err2 < 0 ? err2 : size;
}


static ssize_t mz_mag_iio_sysfs_get_selftest_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 1;
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);

	mutex_lock(&mz_mag->indio_dev->mlock);

	if (mz_mag->ops->self_test)
		ret = mz_mag->ops->self_test(mz_mag);

	mutex_unlock(&mz_mag->indio_dev->mlock);

	return sprintf(buf, "%d\n", ret);
}

static ssize_t mz_mag_iio_sysfs_set_selftest_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	/* we do nothing when writing to the self_test */
	return size;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
			mz_mag_iio_sysfs_get_sampling_frequency,
			mz_mag_iio_sysfs_set_sampling_frequency);

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(mz_mag_iio_sysfs_sampling_frequency_avail);
static IIO_DEVICE_ATTR(hw_fifo_lenght, S_IRUGO, mz_mag_iio_sysfs_get_hw_fifo_lenght, NULL, 0);
static IIO_DEVICE_ATTR(flush, S_IWUSR, NULL, mz_mag_iio_sysfs_flush_fifo, 0);
static IIO_DEVICE_ATTR(compass_self_test, S_IWUSR | S_IRUGO,
				mz_mag_iio_sysfs_get_selftest_status,
				mz_mag_iio_sysfs_set_selftest_status, 0);

static struct attribute *mz_mag_iio_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_compass_self_test.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group mz_mag_iio_attribute_group = {
	.attrs = mz_mag_iio_attributes,
};


static const struct iio_info mz_mag_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &mz_mag_iio_attribute_group,
	.read_raw = &mz_mag_iio_read_raw,
};


static int mz_sensors_mag_get_name(struct device *dev, char **name)
{
	struct mz_mag_data *mz_mag = dev_get_drvdata(dev);

	*name = mz_mag->device_name;

	dev_info(dev, "ltr559_als_get_name succeeded\n");
	return 0;
}

static int mz_sensors_mag_get_version(struct device *dev, const char **version)
{
	*version = "2015-11-19 16:30";
	return 0;
}

struct meizu_sensors_ops meizu_mag_ops = {
	.get_name = &mz_sensors_mag_get_name,
	.get_version = &mz_sensors_mag_get_version,
};

static int mz_mag_iio_init(struct mz_mag_data *mz_mag)
{
	int ret = -1;

	mz_mag->iio_buffer_data = kmalloc(32, GFP_KERNEL);
	if (mz_mag->iio_buffer_data == NULL) {
		dev_err(&mz_mag->client->dev, "failed to call kmalloc\n");
		return -ENOMEM;
	}

	mz_mag->buffer_data = kzalloc(32, GFP_KERNEL);
	if (mz_mag->buffer_data == NULL) {
		dev_err(&mz_mag->client->dev, "failed to call kzalloc\n");
		return -ENOMEM;
	}


	INIT_DELAYED_WORK(&mz_mag->iio_work, mz_mag_iio_func);

	mz_mag->iio_workq = create_singlethread_workqueue("mz_mag_iio");
	if (!mz_mag->iio_workq) {
		dev_err(&mz_mag->client->dev,
			"failed to call create_singlethread_workqueue\n");
		goto free_iio_buffer_data;
	}

	mz_mag->indio_dev = iio_device_alloc(0);
	if (!mz_mag->indio_dev) {
		dev_err(&mz_mag->client->dev, "failed to call iio_device_alloc\n");
		goto wq_destory;
	}

	mz_mag->indio_dev->name = kasprintf(GFP_KERNEL, "%s_%s", "lsm6ds3", "magn");
	mz_mag->indio_dev->info = &mz_mag_iio_info;
	mz_mag->indio_dev->channels = (struct iio_chan_spec const	*)&mz_mag_iio_ch;
	mz_mag->indio_dev->num_channels = ARRAY_SIZE(mz_mag_iio_ch);
	mz_mag->indio_dev->modes = INDIO_DIRECT_MODE;
	iio_device_set_drvdata(mz_mag->indio_dev, mz_mag);

	ret = iio_triggered_buffer_setup(mz_mag->indio_dev,
				&mz_mag_iio_handler_empty, NULL,
				&mz_mag_iio_buffer_setup_ops);
	if (ret < 0) {
		dev_err(&mz_mag->client->dev, "failed to setup triggered buffer\n");
		goto iio_device_free;
	}

	mz_mag->trig = iio_trigger_alloc("%s-trigger", "magn");
	if (!mz_mag->trig) {
		dev_err(&mz_mag->client->dev, "failed to call iio_trigger_alloc\n");
		goto iio_deallocate_buffer;
	}

	mz_mag->trig->ops = &mz_mag_trigger_ops;
	mz_mag->trig->dev.parent = &mz_mag->client->dev;
	iio_trigger_set_drvdata(mz_mag->trig, mz_mag->indio_dev);

	ret = iio_trigger_register(mz_mag->trig);
	if (ret < 0) {
		dev_err(&mz_mag->client->dev, "failed to register iio trigger.\n");
		goto iio_deallocate_trigger;
	}
	mz_mag->indio_dev->trig = mz_mag->trig;

	ret = iio_device_register(mz_mag->indio_dev);
	if (ret < 0) {
		dev_err(&mz_mag->client->dev, "failed to register iio device.\n");
		goto iio_deallocate_trigger;
	}
#if 0
	ret = meizu_sysfslink_register_name(&mz_mag->indio_dev->dev, "magnetometer");
	if (ret < 0) {
		dev_err(&mz_mag->client->dev, "failed to meizu_sysfslink_register_name.\n");
		goto iio_deallocate_trigger;
	}
#endif
	ret = meizu_sensor_register(MEIZU_SENSOR_ID_COMPASS,
		&mz_mag->indio_dev->dev, &meizu_mag_ops);
	if (ret < 0) {
		dev_err(&mz_mag->client->dev, "failed to meizu_sensor_register.\n");
		goto iio_deallocate_trigger;
	}
	return 0;


iio_deallocate_trigger:
	iio_trigger_free(mz_mag->trig);
iio_deallocate_buffer:
	iio_triggered_buffer_cleanup(mz_mag->indio_dev);
iio_device_free:
	iio_device_free(mz_mag->indio_dev);
wq_destory:
	destroy_workqueue(mz_mag->iio_workq);
free_iio_buffer_data:
	kfree(mz_mag->iio_buffer_data);
	return ret;
}

static int mz_mag_setup(struct mz_mag_data *mz_mag)
{
	int ret;

	//dev_dbg(&mz_mag->client->dev, "setup magnetometer sensor, mz_mag: %llu!!\n", (uint64_t)mz_mag);

	ret = akm09911_hw_init(mz_mag);
	if (ret == 0)
		return 0;

	//ret = st480_hw_init(mz_mag);
	//if (ret == 0)
	//	return 0;

	//ret = af8133_hw_init(mz_mag);
	//if (ret == 0)
	//	return 0;

	dev_err(&mz_mag->client->dev, "Failed to setup magnetometer sensor\n");

	return ret;
}


static enum hrtimer_restart mag_timer_func(struct hrtimer *timer)
{
	int64_t timestamp;
	struct timespec ts;
	struct mz_mag_data *mz_mag = container_of(timer, struct mz_mag_data, mag_timer);

#if 0
	queue_delayed_work(mz_mag->iio_workq, &mz_mag->iio_work, 0);
#else
	get_monotonic_boottime(&ts);
	timestamp = timespec_to_ns(&ts);
	*(s64 *)((u8 *)mz_mag->buffer_data + ALIGN(mz_mag->num * MZ_MAG_IIO_BYTE_FOR_CHANNEL, sizeof(s64))) = timestamp;
	iio_push_to_buffers(mz_mag->indio_dev, mz_mag->buffer_data);
#endif
//	printk("mz_mag: mag_timer_func  rate ms:%d\n",mz_mag->pollrate);
	hrtimer_forward_now(&mz_mag->mag_timer, ns_to_ktime(mz_mag->pollrate * NSEC_PER_MSEC));
	return HRTIMER_RESTART;
}

int mz_mag_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct mz_mag_data *mz_mag;

	pr_info("%s mz_mag_probe\n", MZ_MAG_DEV_NAME);

	mz_mag = kzalloc(sizeof(struct mz_mag_data), GFP_KERNEL);
	if (!mz_mag) {
		pr_err("%s mz_mag_probe: memory allocation failed.\n", MZ_MAG_DEV_NAME);
		ret = -ENOMEM;
		goto err0;
	}

	mz_mag->client       = client;
	mz_mag->range        = MZ_MAG_DEFAULT_RANGE;
	mz_mag->pollrate     = MZ_MAG_DEFAULT_POLLRATE;
	mz_mag->min_pollrate = MZ_MAG_DEFAULT_MIN_POLLRATE;
	atomic_set(&mz_mag->enabled, 0);
	mutex_init(&mz_mag->lock);

	i2c_set_clientdata(client, mz_mag);

	ret = mz_mag_setup(mz_mag);
	if(ret < 0)
		goto err1;
#if 0
	ret = mz_mag_input_init(mz_mag);
	if (ret < 0)
		goto err2;

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0)
		goto err3;
#endif
	ret = mz_mag_iio_init(mz_mag);
	if (ret < 0)
		goto err3;

	hrtimer_init(&mz_mag->mag_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mz_mag->pollrate = 10; //ns_to_ktime(110 * NSEC_PER_MSEC);
	mz_mag->mag_timer.function = mag_timer_func;

	dev_dbg(&mz_mag->client->dev, "mz mag probe done\n");

	return 0;

err3:
err2:
err1:
	kfree(mz_mag);
err0:
	pr_err("%s mz_mag_probe failed.\n", MZ_MAG_DEV_NAME);
	return ret;

}

static const struct i2c_device_id mz_mag_id[] = {
	{MZ_MAG_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver mz_mag_driver = {
	.probe     = mz_mag_probe,
	.id_table  = mz_mag_id,
	.driver = {
		.name  = MZ_MAG_DEV_NAME,
		.owner = THIS_MODULE,
	},
};

static struct i2c_board_info mz_mag_i2c[] = {
	{
		//I2C_BOARD_INFO(MZ_MAG_DEV_NAME, 0x1e), /* af8133 */
		I2C_BOARD_INFO(MZ_MAG_DEV_NAME, 0x0c), /* st480 & akm09911 */
	},
};

static int __init mz_mag_init(void)
{
	int ret;

	pr_info("%s mz_mag_init\n", MZ_MAG_DEV_NAME);
#if 1
	ret = i2c_register_board_info(1, mz_mag_i2c, 1);
	if (ret < 0) {
		pr_err("%s i2c_register_board_info failed\n", MZ_MAG_DEV_NAME);
	}

	pr_info("%s i2c_register_board_info success\n", MZ_MAG_DEV_NAME);
#endif
	return 0;
}

static void __exit mz_mag_exit(void)
{
	i2c_del_driver(&mz_mag_driver);
}

//module_init(mz_mag_init);
//module_exit(mz_mag_exit);

postcore_initcall(mz_mag_init);
module_i2c_driver(mz_mag_driver);

MODULE_AUTHOR("Hu Jian <hujian@meizu.com>");
MODULE_DESCRIPTION("Meizu magnetometer linux driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.1");
