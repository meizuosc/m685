

#include <linux/delay.h>
#include <linux/iio/iio.h>
#include "st_lsm6ds3.h"

/* selftest start */
#define SELFTEST_MEASURE_TIMEOUT 100
#define ACC_SELFTEST_XLDA (0x1)
#define GYR_SELFTEST_XLDA (0x2)
#define SELFTEST_SAMPLES 5

static int lsm6ds3_acc_i2c_write(struct lsm6ds3_sensor_data *acc,  u8 * buf,
								int len)
{
	return acc->cdata->tf->write(acc->cdata, buf[0], len, &buf[1], true);
}

static int lsm6ds3_acc_i2c_read(struct lsm6ds3_sensor_data *acc,
				u8 * buf, int len)
{
	int err;

	err = acc->cdata->tf->read(acc->cdata, buf[0],
			len, (uint8_t *)&buf, true);
	if (err < 0) {
		pr_err("failed to read acc raw data.\n");
		return err;
	}

	return 0;
}

static int acc_selftest_init(struct lsm6ds3_sensor_data *acc)
{
	unsigned char buf[5];

	/* BDU=1, ODR=52Hz, FS=2G */
	buf[0] = 0x10;
	buf[1] = 0x30;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x11;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x12;
	buf[1] = 0x44;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x13;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x14;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x15;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x16;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x16;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x17;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x18;
	buf[1] = 0x38;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x19;
	buf[1] = 0x00;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

 	return 0;
}

static int lsm6ds3_acc_get_raw_data(struct lsm6ds3_sensor_data *acc, int *xyz)
{
	int err;
	int16_t raw[3] = {0,0,0};

	err = acc->cdata->tf->read(acc->cdata, 0x28,
			6, (uint8_t *)&raw, true);
	if (err < 0) {
		pr_err("failed to read channel raw data.\n");
		return err;
	}

	xyz[0] = raw[0];
	xyz[1] = raw[1];
	xyz[2] = raw[2];

	return 0;
}

#define LSM6DS3_STATUS_REG 0x1e
static int acc_selftest_wait_data_ready(struct lsm6ds3_sensor_data *acc)
{
	int i, ret;
	unsigned char data_ready;

	pr_info("%s\n", __func__);

	for (i = SELFTEST_MEASURE_TIMEOUT; i != 0; i--) {
		data_ready = LSM6DS3_STATUS_REG;
		ret = lsm6ds3_acc_i2c_read(acc, &data_ready, 1);
		if (ret < 0) {
			pr_err("%s: lsm6ds3_acc_i2c_read fail, retry %d\n",
					__func__, i);
			msleep(5);
			continue;
		} else if (data_ready & ACC_SELFTEST_XLDA) {
			pr_info("%s: data ready\n", __func__);
			break;
		}
	}
	if (i == 0) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	return 0;
}

static int acc_selftest_read(struct lsm6ds3_sensor_data *acc, int data[3])
{
	int total[3];
	int i, ret;

	pr_info("%s\n", __func__);

	total[0] = 0;
	total[1] = 0;
	total[2] = 0;

	pr_info("%s: Read OUTX/OUTY/OUTZ to clear XLDA bit\n", __func__);
	lsm6ds3_acc_get_raw_data(acc, data);
	for (i = 0; i < SELFTEST_SAMPLES; i++) {
		ret = acc_selftest_wait_data_ready(acc);
		if (ret) {
			pr_err("%s: selftest_check_XLDA fail\n", __func__);
			return ret;
		}
		ret = lsm6ds3_acc_get_raw_data(acc, data);
		if (ret < 0) {
			pr_err("%s: lsm6ds3_acc_get_raw_data fail\n", __func__);
			return ret;
		}
		/* convert to mg */
		data[0] = data[0] * 61 / 1000;
		data[1] = data[1] * 61 / 1000;
		data[2] = data[2] * 61 / 1000;
		pr_info("%s: data: x = %d, y = %d, z = %d\n", __func__,
				data[0], data[1], data[2]);
		total[0] += data[0];
		total[1] += data[1];
		total[2] += data[2];
		pr_info("%s: total: x = %d, y = %d, z = %d\n", __func__,
				total[0], total[1], total[2]);
	}
	data[0] = total[0] / SELFTEST_SAMPLES;
	data[1] = total[1] / SELFTEST_SAMPLES;
	data[2] = total[2] / SELFTEST_SAMPLES;
	pr_info("%s: average: x = %d, y = %d, z = %d\n", __func__,
			data[0], data[1], data[2]);

	return 0;
}


static int acc_selftest_enable(struct lsm6ds3_sensor_data *acc)
{
	unsigned char buf[2];

	buf[0] = 0x14;
	buf[1] = 0x01;	/* BDU=1, ST1 = 1, ST0 = 0 */

	pr_info("%s\n", __func__);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);
	return lsm6ds3_acc_i2c_write(acc, buf, 1);
}



/*
 * Part Number Min_X Max_X Min_Y Max_Y Min_Z Max_Z Unit
 * LSM6DS3      90    1700  90    1700  90    1700  LSB (@ FS = +/-2g,2.8v)
 */

static int acc_check_selftest_result(int data_nost[3],
		int data_st[3])
{
	pr_info("data_st:   %d %d %d\n", data_st[0],data_st[1],data_st[2]);
	pr_info("data_nost: %d %d %d\n", data_nost[0],data_nost[1],data_nost[2]);
	data_st[0] = abs(data_st[0] - data_nost[0]);
	data_st[1] = abs(data_st[1] - data_nost[1]);
	data_st[2] = abs(data_st[2] - data_nost[2]);
	pr_info("abs(st - nost):  %d %d %d\n", data_st[0],data_st[1],data_st[2]);

	if(data_st[0] >= 90 && data_st[0] <= 1700){
		pr_info("expect 90 =< x <= 1700, x = %d selftest pass\n", data_st[0]);
	} else {
		pr_err("expect 90 =< x <= 1700, x = %d selftest failed\n", data_st[0]);
	}

	if(data_st[1] >= 90 && data_st[1] <= 1700){
		pr_info("expect 90 =< y <= 1700, y = %d selftest pass\n", data_st[1]);
	} else {
		pr_err("expect 90 =< y <= 1700, x = %d selftest failed\n", data_st[1]);
	}

	if(data_st[2] >= 90 && data_st[2]<= 1700){
		pr_info("expect 90 =< z <= 1700, z = %d selftest pass\n",data_st[2]);
	} else {
		pr_err("expect 90 =< z <= 1700, x = %d selftest failed\n", data_st[2]);
	}

	if (data_st[0] >= 90 && data_st[0] <= 1700 && 
	    data_st[1] >= 90 && data_st[1] <= 1700 && 
	    data_st[2] >= 90 && data_st[2] <= 1700) {
		return 1;
	}

	return -1;
}


static void acc_selftest_disable(struct lsm6ds3_sensor_data *acc)
{
	unsigned char buf[2];

	buf[1] = 0x00;
	pr_debug("%s\n", __func__);

	/* Disable sensor */
	buf[0] = 0x10;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	/* Disable selftest */
	buf[0] = 0x14;
	lsm6ds3_acc_i2c_write(acc, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);
}

int lsm6ds3_acc_selftest(struct lsm6ds3_sensor_data *acc, int *test_result)
{
	int ret;
	int data_nost[3];
	int self_data[3];

	*test_result = 0;

	pr_info("%s: =========================\n", __func__);
	pr_info("%s: lsm6ds3_acc_selftest begin\n", __func__);

	/* Initialize Sensor, turn on sensor, enable P/R/Y */
	ret = acc_selftest_init(acc);
	if (ret < 0) {
		pr_err("%s: selftest_init fail\n", __func__);
		return ret;
	}

	/* Wait for stable output */
	pr_info("%s: wait 200 ms for stable output\n", __func__);
	msleep(200);

	/* Read out normal output */
	ret = acc_selftest_read(acc, data_nost);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	pr_info("%s: normal output: x = %d, y = %d, z = %d\n",
			__func__, data_nost[0], data_nost[1], data_nost[2]);

	/* Enable self test */
	ret = acc_selftest_enable(acc);
	if (ret < 0) {
		pr_err("%s: selftest_enable failed\n", __func__);
		return ret;
	}

	pr_info("%s: wait 200 ms for stable output\n", __func__);
	mdelay(200);

	/* Read out selftest output */
	ret = acc_selftest_read(acc, self_data);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}

	/* Check output */
	ret = acc_check_selftest_result(data_nost, self_data);
	if (ret < 0) {
		pr_err("%s: ***fail***\n", __func__);
		*test_result = -1;
	} else {
		pr_info("%s: ***success***\n", __func__);
		*test_result = 1;
	}

	/* selftest disable */
	acc_selftest_disable(acc);

	pr_info("%s: lsm6ds3_acc_selftest end\n\n\n\n", __func__);
	return ret;
}




static int lsm6ds3_gyr_i2c_write(struct lsm6ds3_sensor_data *gyr,  u8 * buf,
								int len)
{
	return gyr->cdata->tf->write(gyr->cdata, buf[0], len, &buf[1], true);
}

static int lsm6ds3_gyr_i2c_read(struct lsm6ds3_sensor_data *gyr,
				u8 * buf, int len)
{
	int err;

	err = gyr->cdata->tf->read(gyr->cdata, buf[0],
			len, (uint8_t *)&buf, true);
	if (err < 0) {
		pr_err("failed to read gyr raw data.\n");
		return err;
	}

	return 0;
}

static int gyr_selftest_init(struct lsm6ds3_sensor_data *gyr)
{
	unsigned char buf[5];

	/* BDU=1, ODR=208Hz, FS=2000dps */
	buf[0] = 0x10;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x11;
	buf[1] = 0x5C;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x12;
	buf[1] = 0x44;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x13;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x14;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x15;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x16;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x16;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x17;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x18;
	buf[1] = 0x00;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	buf[0] = 0x19;
	buf[1] = 0x38;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

 	return 0;
}

static int lsm6ds3_gyr_get_raw_data(struct lsm6ds3_sensor_data *gyr, int *xyz)
{
	int err;
	int16_t raw[3] = {0,0,0};

	err = gyr->cdata->tf->read(gyr->cdata, 0x22,
			6, (uint8_t *)&raw, true);
	if (err < 0) {
		pr_err("failed to read channel raw data.\n");
		return err;
	}

	xyz[0] = raw[0];
	xyz[1] = raw[1];
	xyz[2] = raw[2];

	return 0;
}

#define LSM6DS3_STATUS_REG 0x1e
static int gyr_selftest_wait_data_ready(struct lsm6ds3_sensor_data *gyr)
{
	int i, ret;
	unsigned char data_ready;

	pr_info("%s\n", __func__);

	for (i = SELFTEST_MEASURE_TIMEOUT; i != 0; i--) {
		data_ready = LSM6DS3_STATUS_REG;
		ret = lsm6ds3_gyr_i2c_read(gyr, &data_ready, 1);
		if (ret < 0) {
			pr_err("%s: lsm6ds3_gyr_i2c_read fail, retry %d\n",
					__func__, i);
			msleep(5);
			continue;
		} else if (data_ready & GYR_SELFTEST_XLDA) {
			pr_info("%s: data ready\n", __func__);
			break;
		}
	}
	if (i == 0) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	return 0;
}

static int gyr_selftest_read(struct lsm6ds3_sensor_data *gyr, int data[3])
{
	int total[3];
	int i, ret;

	pr_info("%s\n", __func__);

	total[0] = 0;
	total[1] = 0;
	total[2] = 0;

	pr_info("%s: Read OUTX/OUTY/OUTZ to clear GDA bit\n", __func__);
	lsm6ds3_gyr_get_raw_data(gyr, data);
	for (i = 0; i < SELFTEST_SAMPLES; i++) {
		ret = gyr_selftest_wait_data_ready(gyr);
		if (ret) {
			pr_err("%s: selftest_check_GDA fail\n", __func__);
			return ret;
		}
		ret = lsm6ds3_gyr_get_raw_data(gyr, data);
		if (ret < 0) {
			pr_err("%s: lsm6ds3_gyr_get_raw_data fail\n", __func__);
			return ret;
		}
		/* convert to dps */
		data[0] = data[0] * 70 / 1000;
		data[1] = data[1] * 70 / 1000;
		data[2] = data[2] * 70 / 1000;
		pr_info("%s: data: x = %d, y = %d, z = %d\n", __func__,
				data[0], data[1], data[2]);
		total[0] += data[0];
		total[1] += data[1];
		total[2] += data[2];
		pr_info("%s: total: x = %d, y = %d, z = %d\n", __func__,
				total[0], total[1], total[2]);
	}
	data[0] = total[0] / SELFTEST_SAMPLES;
	data[1] = total[1] / SELFTEST_SAMPLES;
	data[2] = total[2] / SELFTEST_SAMPLES;
	pr_info("%s: average: x = %d, y = %d, z = %d\n", __func__,
			data[0], data[1], data[2]);

	return 0;
}


static int gyr_selftest_enable(struct lsm6ds3_sensor_data *gyr)
{
	unsigned char buf[2];

	buf[0] = 0x14;
	buf[1] = 0x04;	/* BDU=1, ST1 = 1, ST0 = 0 */

	pr_info("%s\n", __func__);
	pr_info("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);
	return lsm6ds3_gyr_i2c_write(gyr, buf, 1);
}



/*
 * Part Number Min_X Max_X Min_Y Max_Y Min_Z Max_Z Unit
 * LSM6DS3      250    700  250    700  250    700  LSB (@ FS = 2000dps)
 */

static int gyr_check_selftest_result(int data_nost[3],
		int data_st[3])
{
	pr_info("data_st:   %d %d %d\n", data_st[0],data_st[1],data_st[2]);
	pr_info("data_nost: %d %d %d\n", data_nost[0],data_nost[1],data_nost[2]);
	data_st[0] = abs(data_st[0] - data_nost[0]);
	data_st[1] = abs(data_st[1] - data_nost[1]);
	data_st[2] = abs(data_st[2] - data_nost[2]);
	pr_info("abs(st - nost):  %d %d %d\n", data_st[0],data_st[1],data_st[2]);

	if(data_st[0] >= 250 && data_st[0] <= 700){
		pr_info("expect 250 =< x <= 700, x = %d selftest pass\n", data_st[0]);
	} else {
		pr_err("expect 250 =< x <= 700, x = %d selftest failed\n", data_st[0]);
	}

	if(data_st[1] >= 250 && data_st[1] <= 700){
		pr_info("expect 250 =< y <= 700, y = %d selftest pass\n", data_st[1]);
	} else {
		pr_err("expect 250 =< y <= 700, x = %d selftest failed\n", data_st[1]);
	}

	if(data_st[2] >= 250 && data_st[2]<= 700){
		pr_info("expect 250 =< z <= 700, z = %d selftest pass\n",data_st[2]);
	} else {
		pr_err("expect 250 =< z <= 700, x = %d selftest failed\n", data_st[2]);
	}

	if (data_st[0] >= 250 && data_st[0] <= 700 && 
	    data_st[1] >= 250 && data_st[1] <= 700 && 
	    data_st[2] >= 250 && data_st[2] <= 700) {
		return 1;
	}

	return -1;
}


static void gyr_selftest_disable(struct lsm6ds3_sensor_data *gyr)
{
	unsigned char buf[2];

	buf[1] = 0x00;
	pr_debug("%s\n", __func__);

	/* Disable sensor */
	buf[0] = 0x11;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);

	/* Disable selftest */
	buf[0] = 0x14;
	lsm6ds3_gyr_i2c_write(gyr, buf, 1);
	pr_debug("%s: write reg: %x, val: %x\n", __func__, buf[0], buf[1]);
}

int lsm6ds3_gyr_selftest(struct lsm6ds3_sensor_data *gyr, int *test_result)
{
	int ret;
	int data_nost[3];
	int self_data[3];

	*test_result = 0;

	pr_info("%s: =========================\n", __func__);
	pr_info("%s: lsm6ds3_gyr_selftest begin\n", __func__);

	/* Initialize Sensor, turn on sensor, enable P/R/Y */
	ret = gyr_selftest_init(gyr);
	if (ret < 0) {
		pr_err("%s: selftest_init fail\n", __func__);
		return ret;
	}

	/* Wait for stable output */
	pr_info("%s: wait 800 ms for stable output\n", __func__);
	msleep(800);

	/* Read out normal output */
	ret = gyr_selftest_read(gyr, data_nost);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	pr_info("%s: normal output: x = %d, y = %d, z = %d\n",
			__func__, data_nost[0], data_nost[1], data_nost[2]);

	/* Enable self test */
	ret = gyr_selftest_enable(gyr);
	if (ret < 0) {
		pr_err("%s: selftest_enable failed\n", __func__);
		return ret;
	}

	pr_info("%s: wait 60 ms for stable output\n", __func__);
	mdelay(60);

	/* Read out selftest output */
	ret = gyr_selftest_read(gyr, self_data);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}

	/* Check output */
	ret = gyr_check_selftest_result(data_nost, self_data);
	if (ret < 0) {
		pr_err("%s: ***fail***\n", __func__);
		*test_result = -1;
	} else {
		pr_info("%s: ***success***\n", __func__);
		*test_result = 1;
	}

	/* selftest disable */
	gyr_selftest_disable(gyr);

	pr_info("%s: lsm6ds3_gyr_selftest end\n\n\n\n", __func__);
	return ret;
}
