
#ifndef  __SENSOR_PS_DEV_H__
#define  __SENSOR_PS_DEV_H__

#include "sensor.h"
#include <linux/notifier.h>

#define SENSOR_PS_DEVICE_NAME "sensor-ps"

#define SENSOR_PS_INTERRUPT_DEBUG (0)
#define SENSOR_PS_NO_CALIBBIAS    (-1)

struct sensor_ps_dev;

struct sensor_ps_ops {
	int (*check_id)(struct sensor_ps_dev *ps_dev);
	int (*get_name)(struct sensor_ps_dev *ps_dev, const char **name);
	int (*get_irq)(struct sensor_ps_dev *ps_dev, unsigned int *irq);

	int (*get_max_offset)(struct sensor_ps_dev *ps_dev, int *offset);
	int (*get_min_offset)(struct sensor_ps_dev *ps_dev, int *offset);
	int (*set_offset)(struct sensor_ps_dev *ps_dev, int offset);

	int (*read_adc_data)(struct sensor_ps_dev *ps_dev, uint16_t *adc);
	int (*read_raw_adc_data)(struct sensor_ps_dev *ps_dev, uint16_t *adc);
	int (*read_near_far_flag)(struct sensor_ps_dev *ps_dev, uint8_t *flag);

	int (*enable)(struct sensor_ps_dev *ps_dev);
	int (*disable)(struct sensor_ps_dev *ps_dev);

	void (*hw_init)(struct sensor_ps_dev *ps_dev);

	void (*suspend)(struct sensor_ps_dev * ps_dev);
	void (*resume)(struct sensor_ps_dev * ps_dev);

	int (*get_debug)(struct sensor_ps_dev * ps_dev, char *buf);
	int (*set_debug)(struct sensor_ps_dev * ps_dev,
					const char *buf, size_t count);
};

struct sensor_ps_dev {
	struct sensor_dev *sdev;
	struct i2c_client *client;
	struct sensor_ps_ops *ops;
	struct mutex mlock;
	struct notifier_block fb_notifier;
	int irq;
	int irq_triggered;
	int irq_enable;
	int ps_enable;
	int ps_data;
	uint16_t pre_flag;
	uint16_t pre_adc;
	int wakeup;
	int32_t calibbias;
	int32_t adc_offset;
	uint8_t adc_near;
	uint8_t adc_far;
};

#endif