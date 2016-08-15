
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/notifier.h>

#include "sensor-als.h"
#include "sensor-i2c.h"
#include "sensor-als-dev.h"
#include "sensor-als-adaptor.h"

static int sensor_als_fb_notifier_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{

	struct fb_event *evdata = data;
	unsigned int blank;
	struct sensor_als_dev *als_dev = container_of(nb,
					struct sensor_als_dev, fb_notifier);

	if(val != FB_EVENT_BLANK)
		return 0;

	if(evdata && evdata->data && val == FB_EVENT_BLANK) {
		blank = *(int *)(evdata->data);

		switch(blank) {
		case FB_BLANK_POWERDOWN:
			if (als_dev->als_enable) {
				dev_info(&als_dev->client->dev,
					"non-wakeup als not released"
					" when screen off\n");
			} else {
				dev_info(&als_dev->client->dev,
					"als not opened when screen off\n");
			}
			break;

		case FB_BLANK_UNBLANK:
			if (als_dev->als_enable) {
				dev_info(&als_dev->client->dev,
					"non wakeup als re-enalbe"
					" in screen on\n");
			} else {
				dev_info(&als_dev->client->dev,
					"als not opened in screen on\n");
			}
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}

static int sensor_als_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	const char *name;
	struct sensor_als_dev *als_dev;
	struct sensor_ops *ops = sensor_als_adaptor_get_ops();

	dev_info(&client->dev, "sensor-als probe begin\n");

	als_dev = kzalloc(sizeof(struct sensor_als_dev), GFP_KERNEL);
	if (als_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "kzalloc failed");
		goto err_kzalloc;
	}

	als_dev->client = client;
	als_dev->ops = (struct sensor_als_ops *)client->dev.platform_data;
	if (!als_dev->ops) {
		ret = -EINVAL;
		dev_err(&client->dev, "sensor als ops error");
		goto err_ops;
	}

	ret = als_dev->ops->check_id(als_dev);
	if(ret < 0) {
		dev_err(&client->dev, "sensor als check_id failed\n");
		goto err_id;
	}

	als_dev->fb_notifier.notifier_call = sensor_als_fb_notifier_callback;
	ret = fb_register_client(&als_dev->fb_notifier);
	if (ret < 0) {
		dev_err(&client->dev, "fb_register_client failed");
		goto err_fp_notifier;
	}

	als_dev->client = client;
	mutex_init(&als_dev->mlock);

	als_dev->ops->get_name(als_dev, &name);
	als_dev->sdev = sensor_als_create(name, ops, als_dev);
	if (NULL == als_dev->sdev) {
		ret = -EINVAL;
		dev_err(&client->dev, "sensor_als_create failed");
		goto err_sensor_create;
	}

	dev_set_drvdata(&client->dev, als_dev);
	als_dev->ops->hw_init(als_dev);

	dev_info(&client->dev, "sensor-als probe success\n");
	return 0;

err_sensor_create:
	fb_unregister_client(&als_dev->fb_notifier);
err_fp_notifier:
err_id:
err_ops:
	kfree(als_dev);
err_kzalloc:

	dev_err(&client->dev, "sensor-als probe faild, ret: %d\n", ret);
	return ret;
}

static int sensor_als_i2c_suspend(struct device *dev)
{
	struct sensor_als_dev *als_dev = dev_get_drvdata(dev);

	dev_info(&als_dev->client->dev, "%s\n", __func__);

	als_dev->ops->suspend(als_dev);

	return 0;
}

static int sensor_als_i2c_resume(struct device *dev)
{
	struct sensor_als_dev *als_dev = dev_get_drvdata(dev);

	dev_info(&als_dev->client->dev, "%s\n", __func__);

	als_dev->ops->resume(als_dev);

	return 0;
}

static const struct dev_pm_ops sensor_als_i2c_pm_ops = {
	.suspend = sensor_als_i2c_suspend,
	.resume  = sensor_als_i2c_resume,
};

#define SENSOR_ALS_I2C_DRIVER_NAME SENSOR_ALS_DEVICE_NAME

static const struct i2c_device_id sensor_als_i2c_dev_id[] = {
	{SENSOR_ALS_I2C_DRIVER_NAME, 0},
	{},
};

static struct of_device_id sensor_als_i2c_of_match_table[] = {
	{
		.compatible = "sensors,sensor-als",
	},
	{},
};

static struct i2c_driver sensor_als_i2c_driver = {
	.driver = {
		.name	= SENSOR_ALS_I2C_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= &sensor_als_i2c_pm_ops,
		.of_match_table = sensor_als_i2c_of_match_table,
	},
	.probe	= sensor_als_i2c_probe,
	.id_table = sensor_als_i2c_dev_id,
};

module_i2c_driver(sensor_als_i2c_driver);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor als i2c driver");
MODULE_LICENSE("GPL");
