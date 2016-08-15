
#include <linux/delay.h>
#include <linux/module.h>

#include "sensor-i2c.h"
#include "sensor-scp-acc-gyr-dev.h"
#include "SCP_sensorHub.h"
#include <hwmsensor.h>

int sensor_scp_lsm6ds3_acc_dev_check_id(struct sensor_scp_acc_dev *scp_acc_dev)
{
	int ret;
	uint8_t val;
#if 0
	ret = sensor_i2c_read_8bit(scp_acc_dev->client, ISL29125_REG_CHIP_ID, &val);
	if(0x7D != val) {
		dev_err(&scp_acc_dev->client->dev,
			"the chip id [%d] is not isl29125, ret: %d\n",
			val, ret);
		return -1;
	}
#endif
	dev_info(&scp_acc_dev->client->dev, "sensor acc dev is lsm6ds3\n");
	(void)ret;
	(void)val;

	return 0;
}

int sensor_scp_lsm6ds3_acc_dev_get_name(
	struct sensor_scp_acc_dev *scp_acc_dev,
	const char **name)
{
	*name = "acc-mtk-lsm6ds3";
	return 0;
}

int sensor_scp_lsm6ds3_acc_dev_enable(struct sensor_scp_acc_dev *scp_acc_dev)
{
	int err = 0;
	dev_info(&scp_acc_dev->client->dev, "%s\n", __func__);
	/* we enabled the als when driver probe and keep als always on */
	err = sensor_set_delay_to_hub(ID_ACCELEROMETER, 10);
	if (err < 0) {
		dev_err(&scp_acc_dev->client->dev, "gsensor_set_delay fail!\n");
		return err;
	}
	err = sensor_enable_to_hub(ID_ACCELEROMETER, 1);
	if (err < 0) {
		dev_err(&scp_acc_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_lsm6ds3_acc_dev_disable(struct sensor_scp_acc_dev *scp_acc_dev)
{
	int err = 0;
	dev_info(&scp_acc_dev->client->dev, "%s\n", __func__);
	err = sensor_enable_to_hub(ID_ACCELEROMETER, 0);
	if (err < 0) {
		dev_err(&scp_acc_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_lsm6ds3_acc_dev_self_test(struct sensor_scp_acc_dev *scp_acc_dev)
{
	int err = 0;
	int dat = 4;
	struct data_unit_t data;
	dev_info(&scp_acc_dev->client->dev, "%s\n", __func__);
	sensor_scp_lsm6ds3_acc_dev_disable(scp_acc_dev);
	err = sensor_set_cmd_to_hub(ID_ACCELEROMETER, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_acc_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_ACCELEROMETER,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	msleep(3500);
	sensor_scp_lsm6ds3_acc_dev_enable(scp_acc_dev);
	msleep(300);
	for(err = 0; err < 10; err++) {
		msleep(50);
		sensor_get_data_from_hub(ID_ACCELEROMETER, &data);
	}
	err = sensor_get_data_from_hub(ID_ACCELEROMETER, &data);
	if (err < 0) {
		dev_err(&scp_acc_dev->client->dev, "sensor_get_data_from_hub fail!\n");
		return err;
	}

	dat = 0;
	err = sensor_set_cmd_to_hub(ID_ACCELEROMETER, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_acc_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_ACCELEROMETER,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	sensor_scp_lsm6ds3_acc_dev_disable(scp_acc_dev);
	dev_err(&scp_acc_dev->client->dev, "x = %d\n", data.accelerometer_t.x);

	if(9 == data.accelerometer_t.x)
		return 0;
	else
		return -1;
}

int sensor_scp_lsm6ds3_acc_dev_calibrate(
	struct sensor_scp_acc_dev *scp_acc_dev, int32_t calibbias[3])
{
	int err = 0;
	int dat = 2;
	struct data_unit_t data;
	dev_info(&scp_acc_dev->client->dev, "%s\n", __func__);
	sensor_scp_lsm6ds3_acc_dev_disable(scp_acc_dev);
	err = sensor_set_cmd_to_hub(ID_ACCELEROMETER, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_acc_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_ACCELEROMETER,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	msleep(9000);
	sensor_scp_lsm6ds3_acc_dev_enable(scp_acc_dev);
	msleep(300);
	for(err = 0; err < 10; err++) {
		msleep(50);
		sensor_get_data_from_hub(ID_ACCELEROMETER, &data);
	}
	err = sensor_get_data_from_hub(ID_ACCELEROMETER, &data);
	if (err < 0) {
		dev_err(&scp_acc_dev->client->dev, "sensor_get_data_from_hub fail!\n");
		return err;
	}

	calibbias[0] = data.accelerometer_t.x;
	calibbias[1] = data.accelerometer_t.y;
	calibbias[2] = data.accelerometer_t.z;

	dat = 0;
	err = sensor_set_cmd_to_hub(ID_ACCELEROMETER, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_acc_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_ACCELEROMETER,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	sensor_scp_lsm6ds3_acc_dev_disable(scp_acc_dev);
	dev_err(&scp_acc_dev->client->dev, "x = %d, y = %d, z = %d\n", calibbias[0], calibbias[1], calibbias[2]);
	return 0;
}

int sensor_scp_lsm6ds3_acc_dev_set_offset(
	struct sensor_scp_acc_dev *scp_acc_dev, int offset, int axis)
{
	int err = 0;
	int cali[3] = {0,0,0};
	dev_info(&scp_acc_dev->client->dev, "%s set offset: %d, axis: %d\n",
		__func__, offset, axis);

	cali[axis] = offset;

	err = sensor_set_cmd_to_hub(ID_ACCELEROMETER, CUST_ACTION_SET_CALI, cali);
	if (err < 0) {
		dev_err(&scp_acc_dev->client->dev, "sensor_get_data_from_hub fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_lsm6ds3_acc_dev_set_debug(struct sensor_scp_acc_dev * scp_acc_dev,
					const char *buf, size_t count)
{
	int err = 0;
	int dat = 0;
	switch(buf[0]) {
		case '4':
			dat = 5;
			break;
		default:
			dat = 0;
			break;
	}

	err = sensor_set_cmd_to_hub(ID_ACCELEROMETER, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_acc_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_ACCELEROMETER,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	return 0;
}

int sensor_scp_lsm6ds3_acc_dev_get_debug(struct sensor_scp_acc_dev * scp_acc_dev, char *buf)
{
	return 0;
}

/* gyr */

int sensor_scp_lsm6ds3_gyr_dev_get_name(
	struct sensor_scp_gyr_dev *scp_gyr_dev,
	const char **name )
{
	*name = "gyr-mtk-lsm6ds3";
	return 0;
}

int sensor_scp_lsm6ds3_gyr_dev_enable(struct sensor_scp_gyr_dev *scp_gyr_dev)
{
	int err = 0;
	dev_info(&scp_gyr_dev->client->dev, "%s\n", __func__);
	/* we enabled the als when driver probe and keep als always on */
	err = sensor_set_delay_to_hub(ID_GYROSCOPE, 64);
	if (err < 0) {
		dev_err(&scp_gyr_dev->client->dev, "gyrsensor_set_delay fail!\n");
		return err;
	}
	err = sensor_enable_to_hub(ID_GYROSCOPE, 1);
	if (err < 0) {
		dev_err(&scp_gyr_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_lsm6ds3_gyr_dev_disable(struct sensor_scp_gyr_dev *scp_gyr_dev)
{
	int err = 0;
	dev_info(&scp_gyr_dev->client->dev, "%s\n", __func__);
	err = sensor_enable_to_hub(ID_GYROSCOPE, 0);
	if (err < 0) {
		dev_err(&scp_gyr_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_lsm6ds3_gyr_dev_self_test(struct sensor_scp_gyr_dev *scp_gyr_dev)
{
	int err = 0;
	int dat = 4;
	struct data_unit_t data;
	dev_info(&scp_gyr_dev->client->dev, "%s\n", __func__);
	sensor_scp_lsm6ds3_gyr_dev_disable(scp_gyr_dev);
	err = sensor_set_cmd_to_hub(ID_GYROSCOPE, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_gyr_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_GYROSCOPE,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	msleep(3500);
	sensor_scp_lsm6ds3_gyr_dev_enable(scp_gyr_dev);
	msleep(300);
	for(err = 0; err < 10; err++) {
		msleep(50);
		sensor_get_data_from_hub(ID_GYROSCOPE, &data);
	}
	err = sensor_get_data_from_hub(ID_GYROSCOPE, &data);
	if (err < 0) {
		dev_err(&scp_gyr_dev->client->dev, "sensor_get_data_from_hub fail!\n");
		return err;
	}

	dat = 0;
	err = sensor_set_cmd_to_hub(ID_GYROSCOPE, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_gyr_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_GYROSCOPE,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	sensor_scp_lsm6ds3_gyr_dev_disable(scp_gyr_dev);
	dev_err(&scp_gyr_dev->client->dev, "x = %d\n", data.gyroscope_t.x);

	if(9 == data.gyroscope_t.x)
		return 0;
	else
		return -1;
}

int sensor_scp_lsm6ds3_gyr_dev_set_debug(struct sensor_scp_gyr_dev * scp_gyr_dev,
					const char *buf, size_t count)
{
	int err = 0;
	int dat = 0;
	switch(buf[0]) {
		case '4':
			dat = 5;
			break;
		default:
			dat = 0;
			break;
	}

	err = sensor_set_cmd_to_hub(ID_GYROSCOPE, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_gyr_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_GYROSCOPE,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	return 0;
}

int sensor_scp_lsm6ds3_gyr_dev_get_debug(struct sensor_scp_gyr_dev * scp_gyr_dev, char *buf)
{
	return 0;
}

struct sensor_scp_acc_ops sensor_scp_lsm6ds3_acc_ops = {
	.check_id = &sensor_scp_lsm6ds3_acc_dev_check_id,
	.get_name = &sensor_scp_lsm6ds3_acc_dev_get_name,

	.set_offset = &sensor_scp_lsm6ds3_acc_dev_set_offset,

	.enable = &sensor_scp_lsm6ds3_acc_dev_enable,
	.disable = &sensor_scp_lsm6ds3_acc_dev_disable,

	.set_debug = &sensor_scp_lsm6ds3_acc_dev_set_debug,
	.get_debug = &sensor_scp_lsm6ds3_acc_dev_get_debug,

	.calibrate = &sensor_scp_lsm6ds3_acc_dev_calibrate,
	.self_test = &sensor_scp_lsm6ds3_acc_dev_self_test,
};

struct sensor_scp_gyr_ops sensor_scp_lsm6ds3_gyr_ops = {
	.get_name = &sensor_scp_lsm6ds3_gyr_dev_get_name,

	.enable = &sensor_scp_lsm6ds3_gyr_dev_enable,
	.disable = &sensor_scp_lsm6ds3_gyr_dev_disable,

	.set_debug = &sensor_scp_lsm6ds3_gyr_dev_set_debug,
	.get_debug = &sensor_scp_lsm6ds3_gyr_dev_get_debug,

	.self_test = &sensor_scp_lsm6ds3_gyr_dev_self_test,
};

struct sensor_scp_acc_gyr_ops sensor_scp_lsm6ds3_ops = {
	.acc_ops = &sensor_scp_lsm6ds3_acc_ops,
	.gyr_ops = &sensor_scp_lsm6ds3_gyr_ops,
};

static struct i2c_board_info sensor_scp_lsm6ds3_i2c_dev[] = {
	{
		.type = SENSOR_SCP_ACC_GYR_DEVICE_NAME,
		.addr = 0x6a,
		.platform_data = &sensor_scp_lsm6ds3_ops,
	}
};

static int __init sensor_scp_lsm6ds3_dev_module_init(void)
{
	i2c_register_board_info(1, sensor_scp_lsm6ds3_i2c_dev, 1);
	return 0;
}

postcore_initcall(sensor_scp_lsm6ds3_dev_module_init);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor lsm6ds3 device");
MODULE_LICENSE("GPL");
