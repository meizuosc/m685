
#include <linux/delay.h>
#include <linux/module.h>

#include "sensor-i2c.h"
#include "sensor-scp-pres-dev.h"
#include "SCP_sensorHub.h"
#include <hwmsensor.h>

int sensor_scp_lps25_pres_dev_check_id(struct sensor_scp_pres_dev *scp_pres_dev)
{
	int ret;
	uint8_t val;
#if 0
	ret = sensor_i2c_read_8bit(scp_pres_dev->client, ISL29125_REG_CHIP_ID, &val);
	if(0x7D != val) {
		dev_err(&scp_pres_dev->client->dev,
			"the chip id [%d] is not lps25, ret: %d\n",
			val, ret);
		return -1;
	}
#endif
	dev_info(&scp_pres_dev->client->dev, "sensor pres dev is lps25\n");
	(void)ret;
	(void)val;

	return 0;
}

int sensor_scp_lps25_pres_dev_get_name(
	struct sensor_scp_pres_dev *scp_pres_dev,
	const char ** name)
{
	*name = "pres-mtk-lps25";
	return 0;
}

int sensor_scp_lps25_pres_dev_enable(struct sensor_scp_pres_dev *scp_pres_dev)
{
	int err = 0;
	dev_info(&scp_pres_dev->client->dev, "%s\n", __func__);
	/* we enabled the als when driver probe and keep als always on */
	err = sensor_enable_to_hub(ID_PRESSURE, 1);
	if (err < 0) {
		dev_err(&scp_pres_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}

	return 0;
}

int sensor_scp_lps25_pres_dev_disable(struct sensor_scp_pres_dev *scp_pres_dev)
{
	int err = 0;
	dev_info(&scp_pres_dev->client->dev, "%s\n", __func__);
	err = sensor_enable_to_hub(ID_PRESSURE, 0);
	if (err < 0) {
		dev_err(&scp_pres_dev->client->dev,"SCP_sensorHub_req_send fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_lps25_pres_dev_self_test(struct sensor_scp_pres_dev *scp_pres_dev)
{
	dev_info(&scp_pres_dev->client->dev, "%s\n", __func__);
	return 0;
}

int sensor_scp_lps25_pres_dev_calibrate(
	struct sensor_scp_pres_dev *scp_pres_dev, int32_t calibbias[3])
{
	int err = 0;
	int dat = 2;
	struct data_unit_t data;
	dev_info(&scp_pres_dev->client->dev, "%s\n", __func__);
	sensor_scp_lps25_pres_dev_disable(scp_pres_dev);
	err = sensor_set_cmd_to_hub(ID_PRESSURE, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_pres_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_PRESSURE,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	msleep(9000);
	sensor_scp_lps25_pres_dev_enable(scp_pres_dev);
	msleep(300);
	for(err = 0; err < 10; err++) {
		msleep(50);
		sensor_get_data_from_hub(ID_PRESSURE, &data);
	}
	err = sensor_get_data_from_hub(ID_PRESSURE, &data);
	if (err < 0) {
		dev_err(&scp_pres_dev->client->dev, "sensor_get_data_from_hub fail!\n");
		return err;
	}

	calibbias[0] = data.pressure_t.pressure;

	dat = 0;
	err = sensor_set_cmd_to_hub(ID_PRESSURE, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_pres_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_PRESSURE,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	sensor_scp_lps25_pres_dev_disable(scp_pres_dev);
	dev_err(&scp_pres_dev->client->dev, "x = %d\n", calibbias[0]);

	return 0;
}

int sensor_scp_lps25_pres_dev_set_offset(
	struct sensor_scp_pres_dev *scp_pres_dev, int offset, int axis)
{
	int err = 0;
	int cali[3] = {0,0,0};
	dev_info(&scp_pres_dev->client->dev, "%s set offset: %d, axis: %d\n",
		__func__, offset, axis);

	cali[axis] = offset;

	err = sensor_set_cmd_to_hub(ID_PRESSURE, CUST_ACTION_SET_CALI, cali);
	if (err < 0) {
		dev_err(&scp_pres_dev->client->dev, "sensor_set_cmd_to_hub fail!\n");
		return err;
	}
	return 0;
}

int sensor_scp_lps25_pres_dev_set_debug(struct sensor_scp_pres_dev * scp_pres_dev,
					const char *buf, size_t count)
{
	int err = 0;
	int dat = 0;
	switch(buf[0]) {
		case '5':
			dat = 5;
			break;
		default:
			dat = 0;
			break;
	}

	err = sensor_set_cmd_to_hub(ID_PRESSURE, CUST_ACTION_SET_FACTORY, &dat);
	if (err < 0) {
		dev_info(&scp_pres_dev->client->dev, "sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_PRESSURE,
			CUST_ACTION_SET_FACTORY);
		return err;
	}
	return 0;
}

int sensor_scp_lps25_pres_dev_get_debug(struct sensor_scp_pres_dev * scp_pres_dev, char *buf)
{
	return 0;
}

struct sensor_scp_pres_ops sensor_scp_lps25_pres_ops = {
	.check_id = &sensor_scp_lps25_pres_dev_check_id,
	.get_name = &sensor_scp_lps25_pres_dev_get_name,

	.set_offset = &sensor_scp_lps25_pres_dev_set_offset,

	.enable = &sensor_scp_lps25_pres_dev_enable,
	.disable = &sensor_scp_lps25_pres_dev_disable,

	.set_debug = &sensor_scp_lps25_pres_dev_set_debug,
	.get_debug = &sensor_scp_lps25_pres_dev_get_debug,

	.calibrate = &sensor_scp_lps25_pres_dev_calibrate,
	.self_test = &sensor_scp_lps25_pres_dev_self_test,
};

static struct i2c_board_info sensor_scp_lps25_i2c_dev[] = {
	{
		.type = SENSOR_SCP_PRES_DEVICE_NAME,
		.addr = 0x5c,
		.platform_data = &sensor_scp_lps25_pres_ops,
	}
};

static int __init sensor_scp_lps25_pres_dev_module_init(void)
{
	i2c_register_board_info(1, sensor_scp_lps25_i2c_dev, 1);
	return 0;
}

postcore_initcall(sensor_scp_lps25_pres_dev_module_init);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor lps25 device");
MODULE_LICENSE("GPL");
