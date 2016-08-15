
#include <linux/module.h>

#include "sensor-i2c.h"
#include "sensor-als-dev.h"

static int sensor_als_adaptor_set_enable(struct sensor_dev *sdev, int state)
{
	struct sensor_als_dev *als_dev = sensor_device_get_drvdata(sdev);

	if (als_dev->als_enable == state) {
		dev_info(&sdev->dev, "%s, als_enable is already %d\n",
				__func__, state);
		return 0;
	}

	dev_info(&sdev->dev, "%s, set enable: %d\n", __func__, state);

	als_dev->als_enable = state;

	if (state) {
		als_dev->ops->enable(als_dev);
	} else {
		als_dev->ops->disable(als_dev);
	}

	return 0;
}

static int sensor_als_adaptor_get_enable(struct sensor_dev *sdev, int *state)
{
	struct sensor_als_dev *als_dev = sensor_device_get_drvdata(sdev);
	*state = als_dev->als_enable;

	dev_info(&sdev->dev, "%s, state: %d\n", __func__, *state);

	return 0;
}

static int sensor_als_adaptor_get_data(struct sensor_dev *sdev, int32_t raw[3])
{
	int ret;
	uint32_t lux;
	static uint32_t debug_counter = 0;
	struct sensor_als_dev *als_dev = sensor_device_get_drvdata(sdev);

	ret = als_dev->ops->get_lux(als_dev, &lux);
	if (ret < 0) {
		dev_err(&sdev->dev, "error get lux, ret: %d\n", ret);
		goto error;
	}

	raw[0] = lux;

	debug_counter++;
	if (!(debug_counter%512)) {
		dev_info(&sdev->dev, "%s lux: %d\n", __func__, lux);
		dev_info(&als_dev->client->dev, "%s cover type is %d\n",
			__func__,
			als_dev->cover_type);
	}

	return 1;

error:
	return ret;
}

static int sensor_als_adaptor_get_version(struct sensor_dev *sdev,
						const char **version)
{
	*version = "2016-01-18 17:38";
	return 0;
}

static int sensor_als_adaptor_set_debug(struct sensor_dev *sdev,
					const char *buf, size_t count)
{
	int ret = -1;
	struct sensor_als_dev *als_dev = sensor_device_get_drvdata(sdev);

	if (als_dev->ops->set_debug)
		ret = als_dev->ops->set_debug(als_dev, buf, count);
	if (ret < 0) {
		return sensor_i2c_set_debug_8bit(als_dev->client, buf, count);
	}

	return count;
}

static int sensor_als_adaptor_get_debug(struct sensor_dev *sdev, char *buf)
{
	struct sensor_als_dev *als_dev = sensor_device_get_drvdata(sdev);
	if (als_dev->ops->get_debug)
		return als_dev->ops->get_debug(als_dev, buf);
	else
		return -ENOSYS;
}

static int sensor_als_adaptor_get_max_sample_freq(struct sensor_dev *sdev,
							int32_t *sample_freq)
{
	struct sensor_als_dev *als_dev = sensor_device_get_drvdata(sdev);
	return als_dev->ops->get_max_sample_freq(als_dev, sample_freq);
}

struct sensor_ops sensor_ops_als_adaptor = {
	.set_enable   = sensor_als_adaptor_set_enable,
	.get_enable   = sensor_als_adaptor_get_enable,
	.get_raw_data = sensor_als_adaptor_get_data,
	.get_version  = sensor_als_adaptor_get_version,

	.set_debug     = sensor_als_adaptor_set_debug,
	.get_debug     = sensor_als_adaptor_get_debug,

	.get_max_sample_freq = sensor_als_adaptor_get_max_sample_freq,
};

struct sensor_ops * sensor_als_adaptor_get_ops(void)
{
	return &sensor_ops_als_adaptor;
}

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor als adaptor");
MODULE_LICENSE("GPL");