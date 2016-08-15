
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include "sensor-i2c.h"
#include "sensor-scp-gyr-dev.h"
#include "sensor-scp-gyr-adaptor.h"

static int sensor_scp_gyr_adaptor_set_enable(struct sensor_dev *sdev, int state)
{
	struct sensor_scp_gyr_dev *scp_gyr_dev = sensor_device_get_drvdata(sdev);

	if (scp_gyr_dev->enable == state) {
		dev_info(&sdev->dev, "%s, scp-gyr is already %d\n",
				__func__, state);
		return 0;
	}

	dev_info(&sdev->dev, "%s, set enable: %d\n", __func__, state);

	scp_gyr_dev->enable = state;

	if (state) {
		scp_gyr_dev->ops->enable(scp_gyr_dev);
	} else {
		scp_gyr_dev->ops->disable(scp_gyr_dev);
	}

	return 0;
}

static int sensor_scp_gyr_adaptor_get_enable(struct sensor_dev *sdev, int *state)
{
	struct sensor_scp_gyr_dev *scp_gyr_dev = sensor_device_get_drvdata(sdev);
	*state = scp_gyr_dev->enable;

	dev_info(&sdev->dev, "%s, state: %d\n", __func__, *state);

	return 0;
}

static int sensor_scp_gyr_adaptor_get_version(struct sensor_dev *sdev,
						const char **version)
{
	*version = "2016-01-15 20:30";
	return 0;
}

static int sensor_scp_gyr_adaptor_self_test(struct sensor_dev *sdev)
{
	struct sensor_scp_gyr_dev *scp_gyr_dev = sensor_device_get_drvdata(sdev);
	dev_info(&sdev->dev, "%s\n", __func__);

	return scp_gyr_dev->ops->self_test(scp_gyr_dev);
}

static int sensor_scp_gyr_adaptor_set_debug(struct sensor_dev *sdev,
					const char *buf, size_t count)
{
	int ret = -1;
	struct sensor_scp_gyr_dev *scp_gyr_dev = sensor_device_get_drvdata(sdev);

	if (scp_gyr_dev->ops->set_debug)
		ret = scp_gyr_dev->ops->set_debug(scp_gyr_dev, buf, count);
	if (ret < 0) {
		return sensor_i2c_set_debug_8bit(scp_gyr_dev->client, buf, count);
	}

	return count;
}

static int sensor_scp_gyr_adaptor_get_debug(struct sensor_dev *sdev, char *buf)
{
	struct sensor_scp_gyr_dev *scp_gyr_dev = sensor_device_get_drvdata(sdev);
	if (scp_gyr_dev->ops->get_debug)
		return scp_gyr_dev->ops->get_debug(scp_gyr_dev, buf);
	else
		return -ENOSYS;
}

struct sensor_ops sensor_ops_scp_gyr_adaptor = {
	.set_enable    = sensor_scp_gyr_adaptor_set_enable,
	.get_enable    = sensor_scp_gyr_adaptor_get_enable,

	.get_version   = sensor_scp_gyr_adaptor_get_version,

	.self_test     = sensor_scp_gyr_adaptor_self_test,

	.set_debug     = sensor_scp_gyr_adaptor_set_debug,
	.get_debug     = sensor_scp_gyr_adaptor_get_debug,
};

struct sensor_ops * sensor_scp_gyr_adaptor_get_ops(void)
{
	return &sensor_ops_scp_gyr_adaptor;
}

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor Hub SCP Gyr Adaptor");
MODULE_LICENSE("GPL");
