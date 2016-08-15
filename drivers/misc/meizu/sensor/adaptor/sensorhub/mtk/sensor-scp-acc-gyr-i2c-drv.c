
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/module.h>

#include "sensor-i2c.h"
#include "sensor-scp-acc.h"
#include "sensor-scp-acc-dev.h"
#include "sensor-scp-acc-adaptor.h"

#include "sensor-scp-gyr.h"
#include "sensor-scp-gyr-dev.h"
#include "sensor-scp-gyr-adaptor.h"

#include "sensor-scp-acc-gyr-dev.h"

static int sensor_scp_acc_probe(struct i2c_client *client, struct sensor_scp_acc_ops *acc_ops)
{
	int ret = 0;
	const char *name;
	struct sensor_scp_acc_dev *scp_acc_dev;
	struct sensor_ops *ops = sensor_scp_acc_adaptor_get_ops();

	dev_info(&client->dev, "sensor-scp-acc probe begin\n");

	scp_acc_dev = kzalloc(sizeof(struct sensor_scp_acc_dev), GFP_KERNEL);
	if (scp_acc_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "kzalloc failed");
		goto err_kzalloc;
	}

	scp_acc_dev->ops = acc_ops;
	scp_acc_dev->client = client;
	mutex_init(&scp_acc_dev->mlock);

	ret = scp_acc_dev->ops->check_id(scp_acc_dev);
	if(ret < 0) {
		dev_err(&client->dev, "scp_acc_dev->ops->check_id failed\n");
		goto err_id;
	}

	scp_acc_dev->ops->get_name(scp_acc_dev, &name);
	scp_acc_dev->sdev = sensor_scp_acc_create(name, ops, scp_acc_dev);
	if (NULL == scp_acc_dev->sdev) {
		ret = -EINVAL;
		dev_err(&client->dev, "sensor_scp_acc_create failed");
		goto err_sensor_create;
	}

	dev_set_drvdata(&client->dev, scp_acc_dev);

	dev_info(&client->dev, "sensor-scp-acc probe success\n");
	return 0;

err_sensor_create:
err_id:
	kfree(scp_acc_dev);
err_kzalloc:
	dev_err(&client->dev, "sensor-scp-acc probe faild, ret: %d\n", ret);
	return ret;
}


static int sensor_scp_gyr_probe(struct i2c_client *client, struct sensor_scp_gyr_ops *gyr_ops)
{
	int ret = 0;
	const char *name;
	struct sensor_scp_gyr_dev *scp_gyr_dev;
	struct sensor_ops *ops = sensor_scp_gyr_adaptor_get_ops();

	dev_info(&client->dev, "sensor-scp-gyr probe begin\n");

	scp_gyr_dev = kzalloc(sizeof(struct sensor_scp_gyr_dev), GFP_KERNEL);
	if (scp_gyr_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "kzalloc failed");
		goto err_kzalloc;
	}

	scp_gyr_dev->ops = gyr_ops;
	scp_gyr_dev->client = client;
	mutex_init(&scp_gyr_dev->mlock);

	scp_gyr_dev->ops->get_name(scp_gyr_dev, &name);
	scp_gyr_dev->sdev = sensor_scp_gyr_create(name, ops, scp_gyr_dev);
	if (NULL == scp_gyr_dev->sdev) {
		ret = -EINVAL;
		dev_err(&client->dev, "sensor_scp_gyr_create failed");
		goto err_sensor_create;
	}

	dev_info(&client->dev, "sensor-scp-gyr probe success\n");
	return 0;

err_sensor_create:
	kfree(scp_gyr_dev);
err_kzalloc:
	dev_err(&client->dev, "sensor-scp-gyr probe faild, ret: %d\n", ret);
	return ret;
}

static int sensor_scp_acc_gyr_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	struct sensor_scp_acc_gyr_ops *ops;

	ops = (struct sensor_scp_acc_gyr_ops *)client->dev.platform_data;

	ret= sensor_scp_acc_probe(client, ops->acc_ops);
	if (ret < 0) {
		dev_err(&client->dev, "%s acc probe failed\n", __func__);
		goto err_acc;
	}

	ret= sensor_scp_gyr_probe(client, ops->gyr_ops);
	if (ret < 0) {
		dev_err(&client->dev, "%s gyr probe failed\n", __func__);
		goto err_gyr;
	}

	return 0;

err_acc:
err_gyr:
	return ret;
}


#define SENSOR_SCP_ACC_GYR_I2C_DRIVER_NAME SENSOR_SCP_ACC_GYR_DEVICE_NAME
static const struct i2c_device_id sensor_scp_acc_gyr_i2c_dev_id[] = {
	{SENSOR_SCP_ACC_GYR_I2C_DRIVER_NAME, 0},
	{},
};

static struct of_device_id sensor_scp_acc_gyr_i2c_of_match_table[] = {
	{
		.compatible = "sensors,sensor-scp-acc-gyr",
	},
	{},
};

static struct i2c_driver sensor_scp_acc_gyr_i2c_driver = {
	.driver = {
		.name	= SENSOR_SCP_ACC_GYR_I2C_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sensor_scp_acc_gyr_i2c_of_match_table,
	},
	.probe	  = sensor_scp_acc_gyr_i2c_probe,
	.id_table = sensor_scp_acc_gyr_i2c_dev_id,
};

module_i2c_driver(sensor_scp_acc_gyr_i2c_driver);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor scp acc i2c driver");
MODULE_LICENSE("GPL");
