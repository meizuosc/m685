
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/module.h>

#include "sensor-i2c.h"
#include "sensor-scp-pres.h"
#include "sensor-scp-pres-dev.h"
#include "sensor-scp-pres-adaptor.h"

static int sensor_scp_pres_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	const char *name;
	struct sensor_scp_pres_dev *scp_pres_dev;
	struct sensor_ops *ops = sensor_scp_pres_adaptor_get_ops();

	dev_info(&client->dev, "sensor-scp-pres probe begin\n");

	scp_pres_dev = kzalloc(sizeof(struct sensor_scp_pres_dev), GFP_KERNEL);
	if (scp_pres_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "kzalloc failed");
		goto err_kzalloc;
	}

	scp_pres_dev->client = client;
	scp_pres_dev->ops = (struct sensor_scp_pres_ops *)client->dev.platform_data;
	mutex_init(&scp_pres_dev->mlock);

	ret = scp_pres_dev->ops->check_id(scp_pres_dev);
	if(ret < 0) {
		dev_err(&client->dev, "scp_pres_dev->ops->check_id failed\n");
		goto err_id;
	}

	scp_pres_dev->ops->get_name(scp_pres_dev, &name);
	scp_pres_dev->sdev = sensor_scp_pres_create(name, ops, scp_pres_dev);
	if (NULL == scp_pres_dev->sdev) {
		ret = -EINVAL;
		dev_err(&client->dev, "sensor_scp_pres_create failed");
		goto err_sensor_create;
	}

	dev_info(&client->dev, "sensor-scp-pres probe success\n");
	return 0;

err_sensor_create:
err_id:
	kfree(scp_pres_dev);
err_kzalloc:
	dev_err(&client->dev, "sensor-scp-pres probe faild, ret: %d\n", ret);
	return ret;
}

#define SENSOR_SCP_PRES_I2C_DRIVER_NAME SENSOR_SCP_PRES_DEVICE_NAME
static const struct i2c_device_id sensor_scp_pres_i2c_dev_id[] = {
	{SENSOR_SCP_PRES_I2C_DRIVER_NAME, 0},
	{},
};

static struct of_device_id sensor_scp_pres_i2c_of_match_table[] = {
	{
		.compatible = "sensors,sensor-scp-pres",
	},
	{},
};

static struct i2c_driver sensor_scp_pres_i2c_driver = {
	.driver = {
		.name	= SENSOR_SCP_PRES_I2C_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_scp_pres_i2c_of_match_table,
	},
	.probe	  = sensor_scp_pres_i2c_probe,
	.id_table = sensor_scp_pres_i2c_dev_id,
};

module_i2c_driver(sensor_scp_pres_i2c_driver);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor scp i2c driver");
MODULE_LICENSE("GPL");
