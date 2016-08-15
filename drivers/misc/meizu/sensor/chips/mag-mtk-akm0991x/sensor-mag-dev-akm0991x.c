
#include <linux/delay.h>
#include <linux/module.h>

#include "sensor-i2c.h"
#include "sensor-scp-mag-dev.h"
#include "SCP_sensorHub.h"
#include <hwmsensor.h>

int sensor_scp_akm0991x_mag_dev_check_id(struct sensor_scp_mag_dev *scp_mag_dev)
{
	int ret;
	uint8_t val;
#if 0
	ret = sensor_i2c_read_8bit(scp_mag_dev->client, ISL29125_REG_CHIP_ID, &val);
	if(0x7D != val) {
		dev_err(&scp_mag_dev->client->dev,
			"the chip id [%d] is not isl29125, ret: %d\n",
			val, ret);
		return -1;
	}
#endif
	dev_info(&scp_mag_dev->client->dev, "sensor mag dev is akm0991x\n");
	(void)ret;
	(void)val;

	return 0;
}

int sensor_scp_akm0991x_mag_dev_get_name(
	struct sensor_scp_mag_dev *scp_mag_dev,
	const char **name)
{
	*name = "mag-mtk-akm0991x";
	return 0;
}

int sensor_scp_akm0991x_mag_dev_enable(struct sensor_scp_mag_dev *scp_mag_dev)
{
	int err = 0;
	dev_info(&scp_mag_dev->client->dev, "%s\n", __func__);
	/* we enabled the als when driver probe and keep als always on */
	err = sensor_set_delay_to_hub(ID_MAGNETIC, 10);
	if (err < 0) {
		dev_err(&scp_mag_dev->client->dev, "msensor_set_delay fail!\n");
		return err;
	}
	err = sensor_enable_to_hub(ID_MAGNETIC, 1);
	if (err < 0) {
		dev_err(&scp_mag_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_akm0991x_mag_dev_disable(struct sensor_scp_mag_dev *scp_mag_dev)
{
	int err = 0;
	dev_info(&scp_mag_dev->client->dev, "%s\n", __func__);
	err = sensor_enable_to_hub(ID_MAGNETIC, 0);
	if (err < 0) {
		dev_err(&scp_mag_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_akm0991x_mag_dev_self_test(struct sensor_scp_mag_dev *scp_mag_dev)
{
	int err = 0;
	int dat = 4;
	struct data_unit_t data;
	dev_info(&scp_mag_dev->client->dev, "%s\n", __func__);
	sensor_scp_akm0991x_mag_dev_disable(scp_mag_dev);
	err = sensor_set_cmd_to_hub(ID_MAGNETIC, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_mag_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_MAGNETIC,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	msleep(1500);
	sensor_scp_akm0991x_mag_dev_enable(scp_mag_dev);
	msleep(300);
	for(err = 0; err < 10; err++) {
		msleep(50);
		sensor_get_data_from_hub(ID_MAGNETIC, &data);
	}
	err = sensor_get_data_from_hub(ID_MAGNETIC, &data);
	if (err < 0) {
		dev_err(&scp_mag_dev->client->dev, "sensor_get_data_from_hub fail!\n");
		return err;
	}

	dat = 0;
	err = sensor_set_cmd_to_hub(ID_MAGNETIC, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_mag_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_MAGNETIC,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	sensor_scp_akm0991x_mag_dev_disable(scp_mag_dev);
	dev_err(&scp_mag_dev->client->dev, "x = %d\n", data.magnetic_t.x);

	if(9 == data.magnetic_t.x)
		return 0;
	else
		return -1;
}

int sensor_scp_akm0991x_mag_dev_set_debug(struct sensor_scp_mag_dev * scp_mag_dev,
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

	err = sensor_set_cmd_to_hub(ID_MAGNETIC, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_mag_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_MAGNETIC,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	return 0;
}

int sensor_scp_akm0991x_mag_dev_get_debug(struct sensor_scp_mag_dev * scp_mag_dev, char *buf)
{
	return 0;
}

struct sensor_scp_mag_ops sensor_scp_akm0991x_mag_ops = {
	.check_id = &sensor_scp_akm0991x_mag_dev_check_id,
	.get_name = &sensor_scp_akm0991x_mag_dev_get_name,

	.enable = &sensor_scp_akm0991x_mag_dev_enable,
	.disable = &sensor_scp_akm0991x_mag_dev_disable,

	.set_debug = &sensor_scp_akm0991x_mag_dev_set_debug,
	.get_debug = &sensor_scp_akm0991x_mag_dev_get_debug,

	.self_test = &sensor_scp_akm0991x_mag_dev_self_test,
};

static struct i2c_board_info sensor_scp_akm0991x_i2c_dev[] = {
	{
		.type = SENSOR_SCP_MAG_DEVICE_NAME,
		.addr = 0x68,
		.platform_data = &sensor_scp_akm0991x_mag_ops,
	}
};

static int __init sensor_scp_akm0991x_mag_dev_module_init(void)
{
	i2c_register_board_info(1, sensor_scp_akm0991x_i2c_dev, 1);
	return 0;
}

postcore_initcall(sensor_scp_akm0991x_mag_dev_module_init);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor lsm6ds3 device");
MODULE_LICENSE("GPL");
