/* The Sensor iio
 *
 * Copyright (c) 2015 Zhang Jiajing
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <asm/unaligned.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>

#include "sensor-core.h"
#include "sensor-iio.h"

#define SENSOR_IIO_MAX_BUF_LEN 32
#define NS_TO_FREQUENCY(x)			(1000000000L / (x))
#define FREQUENCY_TO_NS(x)			(1000000000L / (x))
#define SENSOR_IIO_FREQ_DEFULT    (10)
#define SENSOR_IIO_MAX_NUM_DATA_CHANNELS (3)
struct sensor_iio {
	struct mutex mlock;
	struct sensor_dev *sdev;
	struct iio_dev *indio_dev;
	struct iio_trigger *trig;
	struct hrtimer timer;
	int     enable;
	uint8_t buffer_data[SENSOR_IIO_MAX_BUF_LEN];
	int32_t num_data_channels;
	int32_t sample_freq;
	uint64_t period_ns;
	int8_t mode;

};

int sensor_iio_data_ready(struct sensor_dev *sdev)
{
	struct sensor_iio *siio;

	if (!sdev || !sdev->private)
		BUG();

	siio = sdev->private;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
	iio_trigger_poll(siio->trig, iio_get_time_ns());
#else
	iio_trigger_poll(siio->trig);
#endif
	return 0;
}

int sensor_iio_push_data(struct sensor_dev *sdev,
	uint8_t *data, int64_t timestamp)
{
	int i, ret;
	uint16_t bytes_per_ch, ch_out = 0, ch_in = 0;
	struct sensor_iio *siio;
	struct iio_dev    *indio_dev;
	struct iio_chan_spec const *chs;

	if (!sdev || !sdev->private || !data) {
		printk(KERN_EMERG "sdev: %p, data: %p", sdev, data);
		BUG();
	}

	/* lock sdev to avoid pushing data before enabled or after disabled */
	mutex_lock(&sdev->mlock);

	siio = sdev->private;
	if (siio->enable == 0) {
		dev_err(&siio->indio_dev->dev,
			"%s is not enabled when pushing data\n", sdev->name);
		ret = -EINVAL;
		goto out;
	}

	if (!timestamp)
		timestamp = iio_get_time_ns();

	indio_dev = siio->indio_dev;
	chs = indio_dev->channels;

	mutex_lock(&siio->mlock);
	for (i = 0; i < siio->num_data_channels; i++) {
		bytes_per_ch = chs[i].scan_type.storagebits >> 3;
		if (test_bit(i, indio_dev->active_scan_mask)) {
			memcpy(&siio->buffer_data[ch_out],
				&data[ch_in], bytes_per_ch);
			ch_out += bytes_per_ch;
		}

		ch_in += bytes_per_ch;
	}

	if (indio_dev->scan_timestamp)
		*(int64_t *)((uint8_t *)siio->buffer_data +
			ALIGN(ch_out, sizeof(int64_t))) = timestamp;
	mutex_unlock(&siio->mlock);

	mutex_lock(&indio_dev->mlock);
	/* avoid pushing data after sensor iio disabled */
	if (!list_empty(&indio_dev->buffer_list)) {
		ret = iio_push_to_buffers(indio_dev, siio->buffer_data);
		if (ret < 0)
			dev_err(&indio_dev->dev,
				"iio_push_to_buffers failed, name: %s\n",
				sdev->name);
	} else {
		dev_err(&indio_dev->dev,
			"sensor iio disabled, name: %s\n",
			sdev->name);
		ret = -1;
	}
	mutex_unlock(&indio_dev->mlock);
#if 0
	dev_dbg(&indio_dev->dev,
		"sensor_iio_push_data, name: %s, ret: %d.\n",
		sdev->name, ret);
#endif

out:
	mutex_unlock(&sdev->mlock);
	return ret;
}

int sensor_iio_push_event(struct sensor_dev *sdev,
	uint64_t event, int64_t timestamp)
{
	struct sensor_iio *siio;
	struct iio_dev    *indio_dev;

	if (!sdev || !sdev->private)
		BUG();

	siio = sdev->private;
	indio_dev = siio->indio_dev;

	iio_push_event(indio_dev, event, timestamp);

	return 0;
}

/* ref to iio_buffer_is_active */
static bool sensor_iio_buffer_is_active(struct iio_dev *indio_dev,
				 struct iio_buffer *buf)
{
	struct list_head *p;

	list_for_each(p, &indio_dev->buffer_list)
		if (p == &buf->buffer_list)
			return true;

	return false;
}

int sensor_iio_disable(struct sensor_dev *sdev)
{
	int ret = 0;
	bool inlist;
	struct sensor_iio *siio;
	struct iio_dev    *indio_dev;

	if (!sdev || !sdev->private)
		BUG();

	siio = sdev->private;
	indio_dev = siio->indio_dev;
#if 0
	if (siio->mode == SENSOR_IIO_MODE_POLLING) {
		dev_info(&indio_dev->dev, "disable sensor polling\n");
		hrtimer_cancel(&siio->timer);
	}
#else
	hrtimer_cancel(&siio->timer);
#endif

	inlist = false;
	#if 0
	/* note: we will never disable iio buffer to speedup enable operation
	   It takes about 20ms to enable iio buffer */
	/* ref to iio_buffer_store_enable */
	mutex_lock(&indio_dev->mlock);
	inlist = sensor_iio_buffer_is_active(indio_dev, indio_dev->buffer);
	if (inlist)
		ret = iio_update_buffers(indio_dev, NULL, indio_dev->buffer);
	mutex_unlock(&indio_dev->mlock);
	#endif
	siio->enable = 0;
	dev_info(&indio_dev->dev, "sensor_iio_disable, ret: %d\n", ret);
	return ret;
}

int sensor_iio_enable(struct sensor_dev *sdev)
{
	int i;
	int ret = 0;
	bool inlist;
	struct sensor_iio *siio;
	struct iio_dev    *indio_dev;

	if (!sdev || !sdev->private)
		BUG();

	siio = sdev->private;
	indio_dev = siio->indio_dev;

	/* ref to iio_buffer_store_enable */
	mutex_lock(&indio_dev->mlock);
	inlist = sensor_iio_buffer_is_active(indio_dev, indio_dev->buffer);
	if (!inlist) {
		/* enable data channels, so that we can skip writing 1 to
		   scan_elements/in_***_x_en sysfs files */
		for (i=0; i<indio_dev->num_channels-1; i++)
			iio_scan_mask_set(indio_dev, indio_dev->buffer, i);

		/* enable timestamp channel */
		indio_dev->buffer->scan_timestamp = 1;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
		ret = iio_update_buffers(indio_dev, indio_dev->buffer, NULL);
	}
	mutex_unlock(&indio_dev->mlock);
#else
		mutex_unlock(&indio_dev->mlock);
		ret = iio_update_buffers(indio_dev, indio_dev->buffer, NULL);
	} else {
		mutex_unlock(&indio_dev->mlock);
	}
#endif
	if (ret < 0) {
		dev_err(&indio_dev->dev, "iio_update_buffers failed, ret: %d\n",
				ret);
		siio->enable = 0;
		goto out;
	} else  {
		siio->enable = 1;
	}

#if 0
	if ((!ret) && (siio->mode == SENSOR_IIO_MODE_POLLING)) {
		dev_info(&indio_dev->dev, "enable sensor polling\n");
		hrtimer_start(&siio->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	}
#else
	/* we start the hrtimer to push the first data when sensor enabled.
	   This is required by on-change sensor */
	hrtimer_start(&siio->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
#endif

out:
	dev_info(&indio_dev->dev, "sensor_iio_enable, ret: %d\n", ret);
	return ret;
}

irqreturn_t sensor_iio_trigger_handler(int irq, void *p)
{
	int i;
	int ret = -ENOSYS;
	uint8_t data[4 * 3];
	uint8_t *data_ptr = data;
	uint16_t *u16_ptr = (uint16_t *)data_ptr;
	int16_t  *s16_ptr = (int16_t  *)data_ptr;
	uint32_t *u32_ptr = (uint32_t *)data_ptr;
	int32_t  *s32_ptr = (int32_t  *)data_ptr;
	int32_t raw[3];

	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct iio_chan_spec const *chs = indio_dev->channels;

	struct sensor_iio *siio = iio_priv(indio_dev);
	struct sensor_dev *sdev = siio->sdev;
	const struct sensor_ops *ops = sdev->ops;
#if 0
	dev_dbg(&indio_dev->dev,
		"sensor_iio_trigger_handler, name: %s.\n", sdev->name);
#endif
	mutex_lock(&sdev->mlock);
	if (ops->get_raw_data)
		ret = ops->get_raw_data(sdev, raw);
	mutex_unlock(&sdev->mlock);


	if (ret < 0) {
		dev_info(&indio_dev->dev,
			"sensor data not ready, ret: %d\n", ret);
	} else {
		/* we assume the data is 16 or 32 bits */
		for (i = 0; i < siio->num_data_channels; i++) {
			u16_ptr = (uint16_t *)data_ptr;
			s16_ptr = (int16_t  *)data_ptr;
			u32_ptr = (uint32_t *)data_ptr;
			s32_ptr = (int32_t  *)data_ptr;

			if (chs[i].scan_type.storagebits == 16)
				if (chs[i].scan_type.sign == 's') {
					*s16_ptr++ = raw[i];
					data_ptr = (uint8_t *)s16_ptr;
				} else {
					*u16_ptr++ = raw[i];
					data_ptr = (uint8_t *)u16_ptr;
				}
			else if (chs[i].scan_type.storagebits == 32)
				if (chs[i].scan_type.sign == 's') {
					*s32_ptr++ = raw[i];
					data_ptr = (uint8_t *)s32_ptr;
				} else {
					*u32_ptr++ = raw[i];
					data_ptr = (uint8_t *)u32_ptr;
				}
			else {
				dev_info(&indio_dev->dev,
					"ch%d storagebits %d is not support\n",
					i, chs[i].scan_type.storagebits);
				goto out;
			}
		}

		sensor_iio_push_data(sdev, data, pf->timestamp);
	}

out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static ssize_t sensor_iio_sysfs_get_sampling_frequency_avali(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	//struct sensor_iio  *siio  = iio_priv(indio_dev);

	dev_dbg(&indio_dev->dev, "%s: enter\n", __func__);
	return sprintf(buf, "%s\n", "200 150 100 70 50 20");
}

static ssize_t sensor_iio_sysfs_get_sampling_frequency(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sensor_iio  *siio  = iio_priv(indio_dev);

	dev_dbg(&indio_dev->dev, "%s: enter\n", __func__);
	return sprintf(buf, "%d\n", siio->sample_freq);
}

static ssize_t sensor_iio_sysfs_set_sampling_frequency(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret;
	int32_t sample_freq;
	int32_t max_sample_freq = S32_MAX;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct sensor_iio  *siio  = iio_priv(indio_dev);
	struct sensor_dev *sdev = siio->sdev;
	const struct sensor_ops *ops = sdev->ops;

	dev_dbg(&indio_dev->dev, "%s: enter\n", __func__);

	ret = kstrtoint(buf, 10, &sample_freq);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "kstrtoint error\n");
		return ret;
	}

	dev_dbg(&indio_dev->dev, "set up sample_freq: %d\n", sample_freq);

	if (ops->get_max_sample_freq)
		ret = ops->get_max_sample_freq(sdev, &max_sample_freq);
		if (ret < 0)
			max_sample_freq = S32_MAX;

	if (sample_freq > max_sample_freq) {
		dev_info(&indio_dev->dev, "%s: %d is larger than max (%d)\n",
				__func__, sample_freq, max_sample_freq);
		sample_freq = max_sample_freq;
	}

	mutex_lock(&indio_dev->mlock);
#if 0
	switch (sample_freq) {
	case 26:
		siio->sample_freq = 26;
		siio->period_ns = FREQUENCY_TO_NS(siio->sample_freq);
		break;
	default:
		ret = -EINVAL;
		break;
	}
#endif
	siio->sample_freq = sample_freq;
	siio->period_ns = FREQUENCY_TO_NS(siio->sample_freq);
	dev_info(&indio_dev->dev, "update sample freq: %d, period_ns: %lld\n",
		siio->sample_freq, siio->period_ns);
	ret = 0;
	mutex_unlock(&indio_dev->mlock);


	dev_dbg(&indio_dev->dev, "%s: leave\n", __func__);
	return ret < 0 ? ret : size;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
		sensor_iio_sysfs_get_sampling_frequency,
		sensor_iio_sysfs_set_sampling_frequency);

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(
	sensor_iio_sysfs_get_sampling_frequency_avali);

static int sensor_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	u8 outdata[2];
	//struct sensor_iio *siio = iio_priv(indio_dev);
	//struct sensor_dev *sdev = siio->sdev;

	dev_dbg(&indio_dev->dev, "%s: enter\n", __func__);

	switch (mask) {

	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			dev_dbg(&indio_dev->dev, "%s: leave\n", __func__);
			return -EBUSY;
		}

		/* get data from device */
		/* outdata = xxx */

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		mutex_unlock(&indio_dev->mlock);
		dev_dbg(&indio_dev->dev, "%s: leave\n", __func__);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* the scale is always 1 */
		*val = 1;
		*val2 = 0;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;

	}

	dev_dbg(&indio_dev->dev, "%s: leave\n", __func__);
	return 0;
}

static struct attribute *sensor_iio_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	NULL,
};

static const struct attribute_group sensor_iio_attribute_group = {
	.attrs = sensor_iio_attributes,
};


static const struct iio_info sensor_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &sensor_iio_attribute_group,
	.read_raw = &sensor_iio_read_raw,
};

enum hrtimer_restart sensor_iio_hrtimer_trigger_func(struct hrtimer *timer)
{
	enum hrtimer_restart ret;
	struct sensor_iio *siio;

	siio = container_of(timer, struct sensor_iio, timer);
#if 0
	dev_dbg(&siio->indio_dev->dev,
		"sensor_iio_hrtimer_trigger, name: %s.\n", siio->sdev->name);
#endif

	/* for polling mode, we start the hrtimer to push all sensor data */
	/* for interrput mode, we only use the hrtimer to push the first data */
	if (siio->mode == SENSOR_IIO_MODE_POLLING) {
		hrtimer_forward_now(timer, ns_to_ktime(siio->period_ns));
		ret = HRTIMER_RESTART;
	} else {
		ret = HRTIMER_NORESTART;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
	iio_trigger_poll(siio->trig, iio_get_time_ns());
#else
	iio_trigger_poll(siio->trig);
#endif

	return ret;
}


int sensor_iio_register(struct sensor_dev *sdev,
	const struct iio_chan_spec *channels, int num_channels, int mode)
{
	int i;
	int ret;
	int num_data_channels = 0;
	int num_timestamp_channels = 0;
	struct sensor_iio  *siio;
	struct iio_trigger *trig;
	struct iio_dev     *indio_dev;

	if (!sdev || !channels)
		BUG();

	for (i=0; i<num_channels; i++) {
		if (channels[i].type == IIO_TIMESTAMP)
			num_timestamp_channels++;
		else
			num_data_channels++;
	}

	/*
	 * num_data_channels must be 1~3
	*/
	if (!num_data_channels ||
	     num_data_channels > SENSOR_IIO_MAX_NUM_DATA_CHANNELS) {
		dev_err(&sdev->dev, "number of data channels %d error\n",
			num_data_channels);
		ret = -EINVAL;
		goto err_nothing;
	}

	/*
	 * num_timestamp_channels must be 1, and the timestamp channel must be
	 * the last channel
	*/
	if (num_timestamp_channels != 1 ||
	    channels[num_channels-1].type != IIO_TIMESTAMP) {
		dev_err(&sdev->dev, "timestamp channel error\n");
		ret = -EINVAL;
		goto err_nothing;
	}

	/* setup the industrialio driver allocated elements */
	indio_dev = iio_device_alloc(sizeof(struct sensor_iio));
	if (!indio_dev) {
		dev_err(&sdev->dev, "failed to iio_device_alloc\n");
		ret = -ENOMEM;
		goto err_nothing;
	}
	siio = iio_priv(indio_dev);

	indio_dev->name  = sdev->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info  = &sensor_iio_info;

	indio_dev->channels     = channels;
	indio_dev->num_channels = num_channels;
	indio_dev->dev.parent   = &sdev->dev;


	ret = iio_triggered_buffer_setup(indio_dev,
				&iio_pollfunc_store_time,
				&sensor_iio_trigger_handler,
				NULL /* ops set to NULL */);
	if (ret < 0) {
		dev_err(&sdev->dev, "failed to setup triggered buffer\n");
		goto err_iio_device_free;
	}

	/* enable data channels by default */
	for (i=0; i<num_channels-1; i++)
		iio_scan_mask_set(indio_dev, indio_dev->buffer, i);

	/* enable timestamp channel by default, so that */
	indio_dev->buffer->scan_timestamp = 1;
	/* set default number of datums in buffer */
	indio_dev->buffer->length = 500;


	trig = iio_trigger_alloc("%s-trigger", sdev->name);
	if (!trig) {
		dev_err(&sdev->dev, "failed to call iio_trigger_alloc\n");
		goto err_iio_deallocate_buffer;
	}

	trig->ops = NULL;
	trig->dev.parent = &sdev->dev;
	iio_trigger_set_drvdata(trig, indio_dev);

	ret = iio_trigger_register(trig);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "failed to register iio trigger.\n");
		goto err_iio_deallocate_trigger;
	}
	indio_dev->trig = trig;


	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&sdev->dev, "failed to register iio device.\n");
		goto err_iio_unregister_trigger;
	}

	hrtimer_init(&siio->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	siio->timer.function = &sensor_iio_hrtimer_trigger_func;

	mutex_init(&siio->mlock);
	sdev->private   = siio;
	siio->trig	= trig;
	siio->sdev      = sdev;
	siio->mode      = mode;
	siio->indio_dev = indio_dev;
	siio->sample_freq = SENSOR_IIO_FREQ_DEFULT;
	siio->period_ns   = FREQUENCY_TO_NS(SENSOR_IIO_FREQ_DEFULT);
	siio->num_data_channels = num_data_channels;

	dev_info(&sdev->dev,
		"sensor_iio_register success, name: %s\n", sdev->name);
	return 0;

err_iio_unregister_trigger:
	iio_trigger_unregister(trig);
err_iio_deallocate_trigger:
	iio_trigger_free(trig);
err_iio_deallocate_buffer:
	iio_triggered_buffer_cleanup(indio_dev);
err_iio_device_free:
	iio_device_free(indio_dev);
err_nothing:
	dev_err(&sdev->dev,
		"sensor_iio_register failed\n");
	return ret;
}

void sensor_iio_unregister(struct sensor_dev *sdev)
{
	struct sensor_iio  *siio;
	struct iio_trigger *trig;
	struct iio_dev     *indio_dev;

	if (!sdev || !sdev->private)
		BUG();

	siio = sdev->private;
	trig = siio->trig;
	indio_dev = siio->indio_dev;

	iio_trigger_unregister(trig);
	iio_trigger_free(trig);
	iio_triggered_buffer_cleanup(indio_dev);
	iio_device_free(indio_dev);
	sdev->private = NULL;
}
