/*
 * STMicroelectronics lsm6ds3 buffer driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "st_lsm6ds3.h"

#define ST_LSM6DS3_ENABLE_AXIS			0x07
#define ST_LSM6DS3_FIFO_DIFF_L			0x3a
#define ST_LSM6DS3_FIFO_DIFF_MASK		0x0fff
#define ST_LSM6DS3_FIFO_DATA_OUT_L		0x3e
#define ST_LSM6DS3_FIFO_DATA_OVR_2REGS		0x4000
#define ST_LSM6DS3_ACCEL_OUT_X_L_ADDR		0x28
#define ST_LSM6DS3_GYRO_OUT_X_L_ADDR		0x22
void st_lsm6ds3_push_data_with_timestamp(struct lsm6ds3_data *cdata,
					u8 index, u8 *data, int64_t timestamp)
{
	int i, n = 0;
	struct iio_chan_spec const *chs = cdata->indio_dev[index]->channels;
	uint16_t bfch, bfchs_out = 0, bfchs_in = 0;
	struct lsm6ds3_sensor_data *sdata = iio_priv(cdata->indio_dev[index]);
	int err;
	int16_t raw_data[3]={0,};

	struct timespec ts;

	get_monotonic_boottime(&ts);
	timestamp = timespec_to_ns(&ts);

	/* read raw data from registers, workaround for bug #176708 */
	if (ST_INDIO_DEV_ACCEL == index) {
		err = cdata->tf->read(cdata, ST_LSM6DS3_ACCEL_OUT_X_L_ADDR,
				6, (uint8_t *)raw_data, true);
		if (err < 0) {
			pr_err("failed to read acc raw data.\n");
			return;
		}
		raw_data[0] += sdata->c_offset[0];
		raw_data[1] += sdata->c_offset[1];
		raw_data[2] += sdata->c_offset[2];

	} else if (ST_INDIO_DEV_GYRO == index) {
		err = cdata->tf->read(cdata, ST_LSM6DS3_GYRO_OUT_X_L_ADDR,
				6, (uint8_t *)raw_data, true);
		if (err < 0) {
			pr_err("failed to read gyr raw data.\n");
			return;
		}
	}

	memcpy(sdata->buffer_data, (uint8_t *)raw_data, 6);

	if (cdata->indio_dev[index]->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
			ALIGN(6, sizeof(s64))) = timestamp;

	iio_push_to_buffers(cdata->indio_dev[index], sdata->buffer_data);
	return;
#if 0
	for (i = 0; i < sdata->num_data_channels; i++) {
		bfch = chs[i].scan_type.storagebits >> 3;

		if (cdata->indio_dev[index] == NULL || cdata->indio_dev[index]->active_scan_mask == NULL) {
			printk(KERN_EMERG "lsm6ds3 bug: cdata->indio_dev[index]: %p, index: %d\n",
				cdata->indio_dev[index], index);
			return;
		}

		if (test_bit(i, cdata->indio_dev[index]->active_scan_mask)) {
			if (ST_INDIO_DEV_ACCEL == index) {
				// cali the acc data
				*(int16_t *)(&data[bfchs_in]) = *(int16_t *)(&data[bfchs_in]) + sdata->c_offset[i];
			}
			memcpy(&sdata->buffer_data[bfchs_out],
							&data[bfchs_in], bfch);
			n++;
			bfchs_out += bfch;
		}

		bfchs_in += bfch;
	}

	if (cdata->indio_dev[index]->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
			ALIGN(bfchs_out, sizeof(s64))) = timestamp;

	iio_push_to_buffers(cdata->indio_dev[index], sdata->buffer_data);
#endif
}

static void st_lsm6ds3_parse_fifo_data(struct lsm6ds3_data *cdata, u16 read_len)
{
	u16 fifo_offset = 0;
	u8 gyro_sip, accel_sip;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	u8 ext0_sip, ext1_sip;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	while (fifo_offset < read_len) {
		gyro_sip = cdata->gyro_samples_in_pattern;
		accel_sip = cdata->accel_samples_in_pattern;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		ext0_sip = cdata->ext0_samples_in_pattern;
		ext1_sip = cdata->ext1_samples_in_pattern;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		do {
			if (gyro_sip > 0) {
				if (cdata->gyro_samples_to_discard > 0)
					cdata->gyro_samples_to_discard--;
				else {
					st_lsm6ds3_push_data_with_timestamp(
						cdata, ST_INDIO_DEV_GYRO,
						&cdata->fifo_data[fifo_offset],
						cdata->gyro_timestamp);
				}

				cdata->gyro_timestamp += cdata->gyro_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				gyro_sip--;
			}

			if (accel_sip > 0) {
				if (cdata->accel_samples_to_discard > 0)
					cdata->accel_samples_to_discard--;
				else {
					st_lsm6ds3_push_data_with_timestamp(
						cdata, ST_INDIO_DEV_ACCEL,
						&cdata->fifo_data[fifo_offset],
						cdata->accel_timestamp);
				}

				cdata->accel_timestamp +=
							cdata->accel_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				accel_sip--;
			}

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
			if (ext0_sip > 0) {
				if (cdata->ext_samples_to_discard[0] > 0)
					cdata->ext_samples_to_discard[0]--;
				else {
					st_lsm6ds3_push_data_with_timestamp(
						cdata, ST_INDIO_DEV_EXT0,
						&cdata->fifo_data[fifo_offset],
						cdata->ext0_timestamp);
				}

				cdata->ext0_timestamp += cdata->ext0_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				ext0_sip--;
			}

			if (ext1_sip > 0) {
				if (cdata->ext_samples_to_discard[1] > 0)
					cdata->ext_samples_to_discard[1]--;
				else {
					st_lsm6ds3_push_data_with_timestamp(
						cdata, ST_INDIO_DEV_EXT1,
						&cdata->fifo_data[fifo_offset],
						cdata->ext1_timestamp);
				}

				cdata->ext1_timestamp += cdata->ext1_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				ext1_sip--;
			}

		} while ((accel_sip > 0) || (gyro_sip > 0) ||
					(ext0_sip > 0) || (ext1_sip > 0));
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		} while ((accel_sip > 0) || (gyro_sip > 0));
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	}

	return;
}

void st_lsm6ds3_cleanup_fifo(struct lsm6ds3_data *cdata)
{
	int err;

	u16 read_len = cdata->fifo_threshold, byte_in_pattern;

	if (!cdata->fifo_data)
		return;

	dev_info(cdata->dev, "st_lsm6ds3_cleanup_fifo enter\n");

	err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DIFF_L,
					2, (u8 *)&read_len, true);
	if (err < 0)
		return;

	if (read_len & ST_LSM6DS3_FIFO_DATA_OVR_2REGS) {
		dev_err(cdata->dev,
			"data fifo overrun, failed to read it.\n");
		return;
	}

	read_len &= ST_LSM6DS3_FIFO_DIFF_MASK;
	read_len *= ST_LSM6DS3_BYTE_FOR_CHANNEL;

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	byte_in_pattern = (cdata->accel_samples_in_pattern +
				cdata->gyro_samples_in_pattern +
				cdata->ext0_samples_in_pattern +
				cdata->ext1_samples_in_pattern) *
				ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	byte_in_pattern = (cdata->accel_samples_in_pattern +
				cdata->gyro_samples_in_pattern) *
				ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	read_len = (read_len / byte_in_pattern) * byte_in_pattern;

	if (read_len > cdata->fifo_threshold)
		read_len = cdata->fifo_threshold;

	if (read_len == 0)
		return;

	err = st_lsm6ds3_i2c_read_fifo(cdata, read_len, cdata->fifo_data);
	if (err < 0)
		return;

	dev_info(cdata->dev, "st_lsm6ds3_cleanup_fifo succeed\n");
}

void st_lsm6ds3_read_fifo(struct lsm6ds3_data *cdata, bool check_fifo_len)
{
	int err;
#if (CONFIG_ST_LSM6DS3_IIO_LIMIT_FIFO > 0)
	u16 data_remaining, data_to_read;
#endif /* CONFIG_ST_LSM6DS3_IIO_LIMIT_FIFO */
	u16 read_len = cdata->fifo_threshold, byte_in_pattern;

	if (!cdata->fifo_data)
		return;

	if (check_fifo_len) {
		err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DIFF_L,
						2, (u8 *)&read_len, true);
		if (err < 0)
			return;

		if (read_len & ST_LSM6DS3_FIFO_DATA_OVR_2REGS) {
			dev_err(cdata->dev,
				"data fifo overrun, failed to read it.\n");
			return;
		}

		read_len &= ST_LSM6DS3_FIFO_DIFF_MASK;
		read_len *= ST_LSM6DS3_BYTE_FOR_CHANNEL;

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		byte_in_pattern = (cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern +
					cdata->ext0_samples_in_pattern +
					cdata->ext1_samples_in_pattern) *
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		byte_in_pattern = (cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern) *
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		read_len = (read_len / byte_in_pattern) * byte_in_pattern;

		if (read_len > cdata->fifo_threshold)
			read_len = cdata->fifo_threshold;
	}

	if (read_len == 0)
		return;
#if 1
	err = st_lsm6ds3_i2c_read_fifo(cdata, read_len, cdata->fifo_data);
	if (err < 0)
		return;
#else
#if (CONFIG_ST_LSM6DS3_IIO_LIMIT_FIFO == 0)
	err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DATA_OUT_L,
					read_len, cdata->fifo_data, true);
	if (err < 0)
		return;
#else /* CONFIG_ST_LSM6DS3_IIO_LIMIT_FIFO */
	data_remaining = read_len;

	do {
		if (data_remaining > CONFIG_ST_LSM6DS3_IIO_LIMIT_FIFO)
			data_to_read = CONFIG_ST_LSM6DS3_IIO_LIMIT_FIFO;
		else
			data_to_read = data_remaining;

		err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DATA_OUT_L,
			data_to_read, &cdata->fifo_data[read_len - data_remaining], true);
		if (err < 0)
			return;

		data_remaining -= data_to_read;
	} while (data_remaining > 0);
#endif /* CONFIG_ST_LSM6DS3_IIO_LIMIT_FIFO */
#endif

	st_lsm6ds3_parse_fifo_data(cdata, read_len);
}

static irqreturn_t st_lsm6ds3_step_counter_trigger_handler(int irq, void *p)
{
	int err;
	struct timespec ts;
	int64_t timestamp = 0;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	if (!sdata->cdata->reset_steps) {
		err = sdata->cdata->tf->read(sdata->cdata,
					(u8)indio_dev->channels[0].address,
					ST_LSM6DS3_BYTE_FOR_CHANNEL,
					sdata->buffer_data, true);
		if (err < 0)
			goto st_lsm6ds3_step_counter_done;

		timestamp = sdata->cdata->timestamp;
	} else {
		memset(sdata->buffer_data, 0, ST_LSM6DS3_BYTE_FOR_CHANNEL);
		get_monotonic_boottime(&ts);
		timestamp = timespec_to_ns(&ts);
		sdata->cdata->reset_steps = false;
	}

	if (indio_dev->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
				ALIGN(ST_LSM6DS3_BYTE_FOR_CHANNEL,
						sizeof(s64))) = timestamp;

	iio_push_to_buffers(indio_dev, sdata->buffer_data);

st_lsm6ds3_step_counter_done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static inline irqreturn_t st_lsm6ds3_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}

int st_lsm6ds3_trig_set_state(struct iio_trigger *trig, bool state)
{
	int err;
	struct lsm6ds3_sensor_data *sdata;

	sdata = iio_priv(iio_trigger_get_drvdata(trig));

	err = st_lsm6ds3_set_drdy_irq(sdata, state);

	return err < 0 ? err : 0;
}

static int st_lsm6ds3_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = st_lsm6ds3_set_enable(sdata, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	//return iio_sw_buffer_preenable(indio_dev);
	return 0;
}

static int st_lsm6ds3_buffer_postenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);
#if 0
	if ((1 << sdata->sindex) & ST_LSM6DS3_USE_BUFFER) {
		sdata->buffer_data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
		if (sdata->buffer_data == NULL)
			return -ENOMEM;
	}
#endif
	if ((sdata->sindex == ST_INDIO_DEV_ACCEL) ||
					(sdata->sindex == ST_INDIO_DEV_GYRO)) {
		err = st_lsm6ds3_set_axis_enable(sdata,
					(u8)indio_dev->active_scan_mask[0]);
		if (err < 0)
			goto free_buffer_data;
	}

	err = iio_triggered_buffer_postenable(indio_dev);
	if (err < 0)
		goto free_buffer_data;

	if (sdata->sindex == ST_INDIO_DEV_STEP_COUNTER) {
		iio_trigger_poll_chained(
			sdata->cdata->trig[ST_INDIO_DEV_STEP_COUNTER]);
	}

	if (sdata->sindex == ST_INDIO_DEV_SIGN_MOTION)
		sdata->cdata->sign_motion_event_ready = true;

	return 0;

free_buffer_data:
#if 0
	if ((1 << sdata->sindex) & ST_LSM6DS3_USE_BUFFER) {
		if (sdata->buffer_data != NULL) {
			kfree(sdata->buffer_data);
			sdata->buffer_data = NULL;
		}
	}
#endif
	return err;
}

static int st_lsm6ds3_buffer_postdisable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);
#if 0
	err = iio_triggered_buffer_predisable(indio_dev);
	if (err < 0)
		return err;
#endif
	if ((sdata->sindex == ST_INDIO_DEV_ACCEL) ||
					(sdata->sindex == ST_INDIO_DEV_GYRO)) {
		err = st_lsm6ds3_set_axis_enable(sdata, ST_LSM6DS3_ENABLE_AXIS);
		if (err < 0)
			return err;
	}

	if (sdata->sindex == ST_INDIO_DEV_SIGN_MOTION)
		sdata->cdata->sign_motion_event_ready = false;

	err = st_lsm6ds3_set_enable(sdata, false);
	if (err < 0)
		return err;

	err = st_lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;
#if 0
	if ((1 << sdata->sindex) & ST_LSM6DS3_USE_BUFFER) {
		if (sdata->buffer_data != NULL) {
			kfree(sdata->buffer_data);
			sdata->buffer_data = NULL;
		}
	}
#endif
	return 0;
}

static const struct iio_buffer_setup_ops st_lsm6ds3_buffer_setup_ops = {
	.preenable = &st_lsm6ds3_buffer_preenable,
	.postenable = &st_lsm6ds3_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &st_lsm6ds3_buffer_postdisable,
};

int st_lsm6ds3_allocate_rings(struct lsm6ds3_data *cdata)
{
	int err;

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_INDIO_DEV_ACCEL],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		return err;

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_INDIO_DEV_GYRO],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_accel;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_gyro;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER],
				NULL,
				&st_lsm6ds3_step_counter_trigger_handler,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_sign_motion;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_step_counter;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_TILT],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_step_detector;

	return 0;

buffer_cleanup_step_detector:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]);
buffer_cleanup_step_counter:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]);
buffer_cleanup_sign_motion:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]);
buffer_cleanup_gyro:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_GYRO]);
buffer_cleanup_accel:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	return err;
}

void st_lsm6ds3_deallocate_rings(struct lsm6ds3_data *cdata)
{
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_TILT]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_GYRO]);
}

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 buffer driver");
MODULE_LICENSE("GPL v2");
