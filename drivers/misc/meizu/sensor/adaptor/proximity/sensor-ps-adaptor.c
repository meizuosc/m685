
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include "sensor-i2c.h"
#include "sensor-ps-dev.h"
#include "sensor-ps-adaptor.h"
#include "sensor-ps-statistics.h"

static int sensor_ps_adaptor_set_enable(struct sensor_dev *sdev, int state)
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);

	if (ps_dev->ps_enable == state) {
		dev_info(&sdev->dev, "%s, ps_enable is already %d\n",
				__func__, state);
		return 0;
	}

	dev_info(&sdev->dev, "%s, set enable: %d\n", __func__, state);

	mutex_lock(&ps_dev->mlock);
	ps_dev->ps_enable = state;

	if (state) {
		ps_dev->ops->enable(ps_dev);
		enable_irq(ps_dev->irq);
		ps_dev->irq_enable = 1;

		#if SENSOR_PS_STATISTICS_ENABLE
		sensor_ps_statistics_start();
		#endif

	} else {
		#if SENSOR_PS_STATISTICS_ENABLE
		sensor_ps_statistics_stop();
		#endif

		/* NOTE: we must use disable_irq_nosync here,
		   disable_irq() can cause dead lock on sdev->mlock
		   when interrupt triggered during disable */
		disable_irq_nosync(ps_dev->irq);
		ps_dev->irq_enable = 0;
		ps_dev->ops->disable(ps_dev);
	}
	mutex_unlock(&ps_dev->mlock);
	return 0;
}

static int sensor_ps_adaptor_get_enable(struct sensor_dev *sdev, int *state)
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);
	*state = ps_dev->ps_enable;

	dev_info(&sdev->dev, "%s, state: %d\n", __func__, *state);

	return 0;
}

static int sensor_ps_adaptor_set_wakeup(struct sensor_dev *sdev, int state)
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);
	ps_dev->wakeup = state;

	if (state) {
		#if SENSOR_PS_STATISTICS_ENABLE
		sensor_ps_statistics_dump();
		#endif
	}

	dev_info(&sdev->dev, "%s, state: %d, pre adc: %d, pre flag: %d\n",
			__func__, state, ps_dev->pre_adc, ps_dev->pre_flag);

	return 0;
}

static int sensor_ps_adaptor_get_wakeup(struct sensor_dev *sdev, int *state)
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);
	*state = ps_dev->wakeup;

	dev_info(&sdev->dev, "%s, state: %d\n", __func__, *state);

	return 0;
}

static int sensor_ps_adaptor_get_data(struct sensor_dev *sdev, int32_t raw[3])
{
	int ret;
	uint16_t adc_data;
	uint8_t near_far_flag;
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);

	ret = ps_dev->ops->read_adc_data(ps_dev, &adc_data);
	if (ret < 0) {
		dev_err(&sdev->dev, "error read adc_data, ret: %d\n", ret);
		goto error;
	}

	ret = ps_dev->ops->read_near_far_flag(ps_dev, &near_far_flag);
	if (ret < 0) {
		dev_err(&sdev->dev, "error read nf_flag, ret:%d\n",
			ret);
		goto error;
	}

	ps_dev->pre_adc  = adc_data;
	ps_dev->pre_flag = near_far_flag;

	raw[0] = adc_data;
	raw[1] = near_far_flag;

	dev_info(&sdev->dev,
		"%s adc_data: %d, flag(0 for near): %d\n",
		__func__, adc_data, raw[1]);

	return 2;

error:
	return ret;
}

static int sensor_ps_adaptor_get_version(struct sensor_dev *sdev,
						const char **version)
{
	*version = "2016-02-01 16:45";
	return 0;
}

static int sensor_ps_adaptor_set_debug(struct sensor_dev *sdev,
					const char *buf, size_t count)
{
	int ret = -1;
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);

	if (ps_dev->ops->set_debug)
		ret = ps_dev->ops->set_debug(ps_dev, buf, count);
	if (ret < 0) {
		return sensor_i2c_set_debug_8bit(ps_dev->client, buf, count);
	}

	return count;
}

static int sensor_ps_adaptor_get_debug(struct sensor_dev *sdev, char *buf)
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);
	if (ps_dev->ops->get_debug)
		return ps_dev->ops->get_debug(ps_dev, buf);
	else
		return -ENOSYS;
}

#define SENSOR_PS_CALIBBIAS_COUNT 10
static int sensor_ps_adaptor_calibrate(struct sensor_dev *sdev)
{
	int i;
	int sum = 0;
	int ps_enable;
	int offset_min = 0;
	int offset_max = U16_MAX;
	uint16_t adc_data;
	uint16_t adc_max = 0;
	uint16_t adc_min = U16_MAX;
	int calibbias;

	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);

	dev_info(&sdev->dev, "%s\n", __func__);

	ps_enable = ps_dev->ps_enable;

	ps_dev->ops->disable(ps_dev);
	ps_dev->ops->enable(ps_dev);
	msleep(200);

	for (i = 0; i < SENSOR_PS_CALIBBIAS_COUNT+2; i++) {
		ps_dev->ops->read_adc_data(ps_dev, &adc_data);

		if (adc_data < adc_min)
			adc_min = adc_data;

		if (adc_data > adc_max)
			adc_max = adc_data;

		sum += adc_data;
		msleep(200);
	}

	sum -= adc_min;
	sum -= adc_max;
	calibbias = sum / SENSOR_PS_CALIBBIAS_COUNT;
	dev_info(&sdev->dev, "%s, calibbias: %d\n",
		__func__, calibbias);

	if (!ps_enable)
		ps_dev->ops->disable(ps_dev);

	ps_dev->ops->get_max_offset(ps_dev, &offset_max);
	ps_dev->ops->get_min_offset(ps_dev, &offset_min);
	if ((calibbias > offset_max) || (calibbias < offset_min)) {
		dev_err(&sdev->dev, "calibbias is too large, val: %d\n",
			calibbias);
		return -EINVAL;
	}

	ps_dev->calibbias = calibbias;
	return 0;
}

static int sensor_ps_adaptor_set_offset(struct sensor_dev *sdev,
	int offset, int axis)
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);
	int offset_min = 0;
	int offset_max = U16_MAX;

	dev_info(&sdev->dev, "%s, offset: %d\n", __func__, offset);

	if (axis != 0)
		return 0;

	ps_dev->ops->get_max_offset(ps_dev, &offset_max);
	ps_dev->ops->get_min_offset(ps_dev, &offset_min);
	if ((offset > offset_max) || (offset < offset_min)) {
		dev_err(&sdev->dev, "offset is too large, val: %d\n", offset);
		return -EINVAL;
	}

	ps_dev->adc_offset = offset;

	/* write the offset to sensor's offset register */
	ps_dev->ops->set_offset(ps_dev, offset);

	return 0;
}

static int sensor_ps_adaptor_get_offset(struct sensor_dev *sdev,
	int32_t offset[3])
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);

	dev_info(&sdev->dev, "%s\n", __func__);

	offset[0] = ps_dev->adc_offset;
	offset[1] = 0;
	offset[2] = 0;

	return 0;
}

static int sensor_ps_adaptor_get_calibbias(struct sensor_dev *sdev,
	int32_t calibbias[3])
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);
	dev_info(&sdev->dev, "%s\n", __func__);

	calibbias[0] = ps_dev->calibbias;

	return 0;
}

static int sensor_ps_adaptor_get_irq_gpio(struct sensor_dev *sdev, int *state)
{
	struct sensor_ps_dev *ps_dev = sensor_device_get_drvdata(sdev);
	dev_info(&sdev->dev, "%s\n", __func__);

	*state = ps_dev->irq_triggered;

	return 0;
}

struct sensor_ops sensor_ops_ps_adaptor = {
	.set_enable    = sensor_ps_adaptor_set_enable,
	.get_enable    = sensor_ps_adaptor_get_enable,
	.set_wakeup    = sensor_ps_adaptor_set_wakeup,
	.get_wakeup    = sensor_ps_adaptor_get_wakeup,
	.get_raw_data  = sensor_ps_adaptor_get_data,
	.get_version   = sensor_ps_adaptor_get_version,

	.calibrate     = sensor_ps_adaptor_calibrate,
	.set_offset    = sensor_ps_adaptor_set_offset,
	.get_offset    = sensor_ps_adaptor_get_offset,
	.get_calibbias = sensor_ps_adaptor_get_calibbias,
	.get_irq_gpio  = sensor_ps_adaptor_get_irq_gpio,

	.set_debug     = sensor_ps_adaptor_set_debug,
	.get_debug     = sensor_ps_adaptor_get_debug,
};

struct sensor_ops * sensor_ps_adaptor_get_ops(void)
{
	return &sensor_ops_ps_adaptor;
}

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor ps adaptor");
MODULE_LICENSE("GPL");
