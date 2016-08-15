
#include <linux/fb.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>

#include "sensor-ps.h"
#include "sensor-i2c.h"
#include "sensor-ps-dev.h"
#include "sensor-ps-adaptor.h"
#include "sensor-ps-statistics.h"

static int sensor_ps_dev_interrupt_debouncing(
	struct sensor_ps_dev * ps_dev,
	uint16_t adc_data,
	uint8_t near_far_flag)
{
	int ret = 0;
	int need_check = 0;
	uint8_t flag_check;
	uint16_t adc_data_check;

	if (near_far_flag && (adc_data > ps_dev->adc_near)) {
		need_check = 1;
		dev_warn(&ps_dev->sdev->dev,
			"%s: report far but adc too large\n", __func__);
	} else if (!near_far_flag && (adc_data < ps_dev->adc_far)) {
		need_check = 1;
		dev_warn(&ps_dev->sdev->dev,
			"%s: report near but adc too small\n", __func__);
	}

	if (need_check) {
		dev_warn(&ps_dev->sdev->dev,
			"%s: [pre] raw adc: %d, flag: %d\n",
			__func__, adc_data, near_far_flag);

		msleep(80);

		ret = ps_dev->ops->read_raw_adc_data(ps_dev, &adc_data_check);
		if (ret < 0) {
			dev_err(&ps_dev->client->dev,
				"%s: error read adc_data, ret: %d\n",
				__func__, ret);
			goto out;
		}

		ret = ps_dev->ops->read_near_far_flag(ps_dev, &flag_check);
		if (ret < 0) {
			dev_err(&ps_dev->client->dev,
				"%s: error read nf_flag, ret:%d\n",
				__func__, ret);
			goto out;
		}

		dev_warn(&ps_dev->sdev->dev,
			"%s: [chk] raw adc: %d, flag: %d\n",
			__func__, adc_data_check, flag_check);

		if (flag_check == near_far_flag) {
			if (flag_check && (adc_data_check > ps_dev->adc_near)) {
				ret = -1;
				dev_warn(&ps_dev->sdev->dev,
					"%s: drop event\n", __func__);
			} else if (!flag_check && (adc_data_check < ps_dev->adc_far)) {
				ret = -1;
				dev_warn(&ps_dev->sdev->dev,
					"%s: drop event\n", __func__);
			} else {
				/* if adc raw data is sync with interrput flag
				   when re-check, we keep that event */
				ret = 0;
				dev_warn(&ps_dev->sdev->dev,
					"%s: keep event\n", __func__);
			}
		} else {
			/* when adc raw data is not sync with interrput flag
			   we drop that event */
			ret = -1;
			dev_warn(&ps_dev->sdev->dev,
				"%s: drop event\n", __func__);
		}
	}

out:
	return ret;
}

static int sensor_ps_dev_push_data(struct sensor_ps_dev * ps_dev)
{
	int ret;
	uint16_t sdev_data[3];
	uint16_t adc_data;
	uint8_t near_far_flag;

	dev_info(&ps_dev->sdev->dev,
		"%s: offset: %d, far: %d, near: %d\n",
		__func__,
		ps_dev->adc_offset, ps_dev->adc_far, ps_dev->adc_near);

	ret = ps_dev->ops->read_raw_adc_data(ps_dev, &adc_data);
	if (ret < 0) {
		dev_err(&ps_dev->client->dev, "error read adc_data, ret: %d\n",
			ret);
		goto error;
	}

	ret = ps_dev->ops->read_near_far_flag(ps_dev, &near_far_flag);
	if (ret < 0) {
		dev_err(&ps_dev->client->dev, "error read nf_flag, ret:%d\n",
			ret);
		goto error;
	}

	ret = sensor_ps_dev_interrupt_debouncing(
		ps_dev, adc_data, near_far_flag);
	if (ret < 0)
		goto error;

	sdev_data[0] = adc_data;
	sdev_data[1] = near_far_flag;
	sdev_data[2] = 0;

	ps_dev->pre_adc  = adc_data;
	ps_dev->pre_flag = near_far_flag;

	dev_info(&ps_dev->sdev->dev,
		"%s: push event, raw adc: %d, flag: %d\n",
		__func__, sdev_data[0], sdev_data[1]);

	return sensor_push_data(ps_dev->sdev, (uint8_t *)sdev_data, 0);

error:
	dev_info(&ps_dev->sdev->dev,
		"%s: event dropped\n", __func__);
	return ret;
}

static irqreturn_t sensor_ps_irq_thread_fn(int irq, void *data)
{
	struct sensor_ps_dev *ps_dev = data;

	dev_info(&ps_dev->client->dev, "%s\n", __func__);

	sensor_ps_dev_push_data(ps_dev);

	return IRQ_HANDLED;
}

static irqreturn_t sensor_ps_irq_handler(int irq, void *data)
{
	struct sensor_ps_dev *ps_dev = data;

#if SENSOR_PS_INTERRUPT_DEBUG
	BUG();
#endif

	dev_info(&ps_dev->client->dev, "%s\n", __func__);

	if (ps_dev->adc_offset != SENSOR_PS_NO_CALIBBIAS)
		ps_dev->irq_triggered = 1;

	/* prevent suspend in 500ms,
	   so that we have enought time to report event to upper layer */
	pm_wakeup_event(&ps_dev->client->dev, 500);

	return IRQ_WAKE_THREAD;
}

static inline void sensor_ps_on_screen_off(struct sensor_ps_dev *ps_dev)
{
	mutex_lock(&ps_dev->mlock);
	if (ps_dev->ps_enable) {
		if (ps_dev->wakeup) {
			dev_info(&ps_dev->client->dev, "wakeup ps not released"
				" when screen off, pre adc:%d, pre flag:%d\n",
				ps_dev->pre_adc, ps_dev->pre_flag);
		} else {
			if (ps_dev->irq_enable) {
				disable_irq(ps_dev->irq);
				ps_dev->irq_enable = 0;
			}
			dev_info(&ps_dev->client->dev, "nwakeup ps not released"
				" when screen off, pre adc:%d, pre flag:%d\n",
				ps_dev->pre_adc, ps_dev->pre_flag);
		}
	} else {
		dev_info(&ps_dev->client->dev,
			"ps not opened when screen off\n");
	}
	mutex_unlock(&ps_dev->mlock);
}

static inline void sensor_ps_on_screen_on(struct sensor_ps_dev *ps_dev)
{
	mutex_lock(&ps_dev->mlock);
	if (ps_dev->ps_enable) {
		if (ps_dev->wakeup) {
			dev_info(&ps_dev->client->dev, "wakeup ps re-enalbe"
				" in screen on, pre adc:%d, pre flag:%d\n",
				ps_dev->pre_adc, ps_dev->pre_flag);
		} else {
			if (!ps_dev->irq_enable) {
				enable_irq(ps_dev->irq);
				ps_dev->irq_enable = 1;
			}
			dev_info(&ps_dev->client->dev, "non wakeup ps re-enalbe"
				" in screen on, pre adc:%d, pre flag:%d\n",
				ps_dev->pre_adc, ps_dev->pre_flag);
		}
	} else {
		dev_info(&ps_dev->client->dev,
			"ps not opened in screen on\n");
	}
	mutex_unlock(&ps_dev->mlock);
}

static int sensor_ps_fb_notifier_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{

	struct fb_event *evdata = data;
	unsigned int blank;
	struct sensor_ps_dev *ps_dev = container_of(nb,
					struct sensor_ps_dev, fb_notifier);

	if(val != FB_EVENT_BLANK)
		return 0;

	if(evdata && evdata->data && val == FB_EVENT_BLANK) {
		blank = *(int *)(evdata->data);

		switch(blank) {
		case FB_BLANK_POWERDOWN:
			sensor_ps_on_screen_off(ps_dev);
			break;

		case FB_BLANK_UNBLANK:
			sensor_ps_on_screen_on(ps_dev);
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}

static int sensor_ps_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	const char *name;
	unsigned int ps_irq;
	struct sensor_ps_dev *ps_dev;
	struct sensor_ops *ops = sensor_ps_adaptor_get_ops();

	dev_info(&client->dev, "sensor-ps probe begin\n");

	ps_dev = kzalloc(sizeof(struct sensor_ps_dev), GFP_KERNEL);
	if (ps_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "kzalloc failed");
		goto err_kzalloc;
	}

	ps_dev->client = client;
	ps_dev->ops = (struct sensor_ps_ops *)client->dev.platform_data;
	if (!ps_dev->ops) {
		ret = -EINVAL;
		dev_err(&client->dev, "sensor ps ops error");
		goto err_ops;
	}

	ret = ps_dev->ops->check_id(ps_dev);
	if (ret < 0) {
		dev_err(&client->dev, "sensor ps check_id failed\n");
		goto err_id;
	}

	ps_dev->fb_notifier.notifier_call = sensor_ps_fb_notifier_callback;
	ret = fb_register_client(&ps_dev->fb_notifier);
	if (ret < 0) {
		dev_err(&client->dev, "fb_register_client failed");
		goto err_fp_notifier;
	}

	ret = ps_dev->ops->get_irq(ps_dev, &ps_irq);
	if (ret < 0) {
		dev_err(&client->dev, "%s get ps irq failed, ret： %d \n",
					__func__, ret);
		goto err_get_irq;
	}

	ret = request_threaded_irq(ps_irq, sensor_ps_irq_handler,
		sensor_ps_irq_thread_fn,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
		"sensor-ps", ps_dev);
	if(ret < 0) {
		dev_err(&client->dev, "%s request irq %d failed\n",
					__func__, ps_irq);
		goto err_request_irq;
	}

#if !SENSOR_PS_INTERRUPT_DEBUG
	disable_irq(ps_irq);
#endif

	ps_dev->irq_enable = 0;

	ps_dev->irq    = ps_irq;
	ps_dev->client = client;
	ps_dev->calibbias  = SENSOR_PS_NO_CALIBBIAS;
	ps_dev->adc_offset = SENSOR_PS_NO_CALIBBIAS;
	mutex_init(&ps_dev->mlock);

	/* workaround for mtk triger iio gic getting 0 irq num */
	sensor_temp_create("dummy-temp", NULL, NULL);

#if !SENSOR_PS_INTERRUPT_DEBUG
	ps_dev->ops->get_name(ps_dev, &name);
	ps_dev->sdev = sensor_ps_create(name, ops, ps_dev);
	if (NULL == ps_dev->sdev) {
		ret = -EINVAL;
		dev_err(&client->dev, "sensor_ps_create failed");
		goto err_sensor_create;
	}
#endif

	dev_set_drvdata(&client->dev, ps_dev);
	device_init_wakeup(&client->dev, true);
	ps_dev->ops->hw_init(ps_dev);

	#if SENSOR_PS_STATISTICS_ENABLE
	sensor_ps_statistics_init(ps_dev);
	#endif

	dev_info(&client->dev, "sensor-ps probe success\n");
	return 0;

err_sensor_create:
	free_irq(ps_irq, ps_dev);
err_request_irq:
	fb_unregister_client(&ps_dev->fb_notifier);
err_get_irq:
err_fp_notifier:
err_id:
err_ops:
	kfree(ps_dev);
err_kzalloc:

	dev_err(&client->dev, "sensor-ps probe faild, ret: %d\n", ret);
	return ret;
}

static int sensor_ps_i2c_suspend(struct device *dev)
{
	struct sensor_ps_dev *ps_dev = dev_get_drvdata(dev);

	dev_info(&ps_dev->client->dev, "%s\n", __func__);

	ps_dev->ops->suspend(ps_dev);

	if (ps_dev->wakeup)
		enable_irq_wake(ps_dev->irq);

	return 0;
}

static int sensor_ps_i2c_resume(struct device *dev)
{
	struct sensor_ps_dev *ps_dev = dev_get_drvdata(dev);

	dev_info(&ps_dev->client->dev, "%s\n", __func__);

	/* NOTE: in mt6797 platform, enable irq wake is not implemented */
	if (ps_dev->wakeup)
		disable_irq_wake(ps_dev->irq);

	ps_dev->ops->resume(ps_dev);

	return 0;
}

static const struct dev_pm_ops sensor_ps_i2c_pm_ops = {
	.suspend = sensor_ps_i2c_suspend,
	.resume  = sensor_ps_i2c_resume,
};

#define SENSOR_PS_I2C_DRIVER_NAME SENSOR_PS_DEVICE_NAME

static const struct i2c_device_id sensor_ps_i2c_dev_id[] = {
	{SENSOR_PS_I2C_DRIVER_NAME, 0},
	{},
};

static struct of_device_id sensor_ps_i2c_of_match_table[] = {
	{
		.compatible = "sensors,sensor-ps",
	},
	{},
};

static struct i2c_driver sensor_ps_i2c_driver = {
	.driver = {
		.name	= SENSOR_PS_I2C_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= &sensor_ps_i2c_pm_ops,
		.of_match_table = sensor_ps_i2c_of_match_table,
	},
	.probe	= sensor_ps_i2c_probe,
	.id_table = sensor_ps_i2c_dev_id,
};

module_i2c_driver(sensor_ps_i2c_driver);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor ps i2c driver");
MODULE_LICENSE("GPL");
