
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include "sensor-i2c.h"
#include "sensor-scp-pres-dev.h"
#include "sensor-scp-pres-adaptor.h"

static int sensor_scp_pres_adaptor_set_enable(struct sensor_dev *sdev, int state)
{
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);

	if (scp_pres_dev->enable == state) {
		dev_info(&sdev->dev, "%s, scp-pres is already %d\n",
				__func__, state);
		return 0;
	}

	dev_info(&sdev->dev, "%s, set enable: %d\n", __func__, state);

	scp_pres_dev->enable = state;

	if (state) {
		scp_pres_dev->ops->enable(scp_pres_dev);
	} else {
		scp_pres_dev->ops->disable(scp_pres_dev);
	}

	return 0;
}

static int sensor_scp_pres_adaptor_get_enable(struct sensor_dev *sdev, int *state)
{
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);
	*state = scp_pres_dev->enable;

	dev_info(&sdev->dev, "%s, state: %d\n", __func__, *state);

	return 0;
}

static int sensor_scp_pres_adaptor_get_version(struct sensor_dev *sdev,
						const char **version)
{
	*version = "2016-01-15 20:30";
	return 0;
}

static int sensor_scp_pres_adaptor_calibrate(struct sensor_dev *sdev)
{
	int ret;
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);
	ret = scp_pres_dev->ops->calibrate(scp_pres_dev, scp_pres_dev->calibbias);
	return ret;
}

static int sensor_scp_pres_adaptor_set_offset(struct sensor_dev *sdev,
	int offset, int axis)
{
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);

	dev_info(&sdev->dev, "%s, offset: %d, axis: %d\n",
			__func__, offset, axis);

	scp_pres_dev->offset[axis] = offset;

	/* write the offset to sensor's offset register */
	scp_pres_dev->ops->set_offset(scp_pres_dev, offset, axis);

	return 0;
}

static int sensor_scp_pres_adaptor_get_offset(struct sensor_dev *sdev,
	int32_t offset[3])
{
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);

	dev_info(&sdev->dev, "%s\n", __func__);

	offset[0] = scp_pres_dev->offset[0];
	offset[1] = scp_pres_dev->offset[1];
	offset[2] = scp_pres_dev->offset[2];

	dev_info(&sdev->dev, "%s offset: %d, %d, %d\n",
				__func__, offset[0], offset[1], offset[2]);

	return 0;
}

static int sensor_scp_pres_adaptor_get_calibbias(struct sensor_dev *sdev,
	int32_t calibbias[3])
{
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);
	dev_info(&sdev->dev, "%s\n", __func__);

	calibbias[0] = scp_pres_dev->calibbias[0];
	calibbias[1] = scp_pres_dev->calibbias[1];
	calibbias[2] = scp_pres_dev->calibbias[2];

	return 0;
}

static int sensor_scp_pres_adaptor_self_test(struct sensor_dev *sdev)
{
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);
	dev_info(&sdev->dev, "%s\n", __func__);

	return scp_pres_dev->ops->self_test(scp_pres_dev);
}

static int sensor_scp_pres_adaptor_set_debug(struct sensor_dev *sdev,
					const char *buf, size_t count)
{
	int ret = -1;
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);

	if (scp_pres_dev->ops->set_debug)
		ret = scp_pres_dev->ops->set_debug(scp_pres_dev, buf, count);
	if (ret < 0) {
		return sensor_i2c_set_debug_8bit(scp_pres_dev->client, buf, count);
	}

	return count;
}

static int sensor_scp_pres_adaptor_get_debug(struct sensor_dev *sdev, char *buf)
{
	struct sensor_scp_pres_dev *scp_pres_dev = sensor_device_get_drvdata(sdev);
	if (scp_pres_dev->ops->get_debug)
		return scp_pres_dev->ops->get_debug(scp_pres_dev, buf);
	else
		return -ENOSYS;
}

struct sensor_ops sensor_ops_scp_pres_adaptor = {
	.set_enable    = sensor_scp_pres_adaptor_set_enable,
	.get_enable    = sensor_scp_pres_adaptor_get_enable,

	.get_version   = sensor_scp_pres_adaptor_get_version,

	.calibrate     = sensor_scp_pres_adaptor_calibrate,
	.set_offset    = sensor_scp_pres_adaptor_set_offset,
	.get_offset    = sensor_scp_pres_adaptor_get_offset,
	.get_calibbias = sensor_scp_pres_adaptor_get_calibbias,

	.self_test     = sensor_scp_pres_adaptor_self_test,

	.set_debug     = sensor_scp_pres_adaptor_set_debug,
	.get_debug     = sensor_scp_pres_adaptor_get_debug,
};

struct sensor_ops * sensor_scp_pres_adaptor_get_ops(void)
{
	return &sensor_ops_scp_pres_adaptor;
}

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor Hub SCP pres Adaptor");
MODULE_LICENSE("GPL");
