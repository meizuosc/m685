
#ifndef __H_MEIZU_SENSORS_H__
#define __H_MEIZU_SENSORS_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/sysfs.h>

/**
 * struct meizu_sensors_ops - sensors operation interfaces
 * @self_test:	Sensor self test interface
 * @set_enable:	Enable or disable, state: 1 for enable, 0 for disable
 * @get_enable:	Get the sensor's enable state.
 * @calibrate:	Perform sensor calibration.
 * @set_offset:	Set the specific axis offset of the sensor,
 *		axis:
 *		for acc and gry, 0: x axis, 1: y axis, 2:z axis
 *		for ps, only 0 is valid
 * @get_offset:	Get offset of the sensor
 * @get_calibbias:	Get the calibbias of the sensor.
 * @get_raw_data:	Get the raw data of the sensor.
 *
 * return 0 if ops success
 * return a negative error number if failed
 */
struct meizu_sensors_ops {
	int (*self_test)(struct device *dev);

	int (*set_enable)(struct device *dev, int state);
	int (*get_enable)(struct device *dev, int *state);

	int (*calibrate)(struct device *dev);

	int (*set_offset)(struct device *dev, int offset, int axis);
	int (*get_offset)(struct device *dev, int32_t offset[3]);
	int (*get_calibbias)(struct device *dev, int32_t calibbias[3]);
	int (*get_raw_data)(struct device *dev, int32_t raw[3]);

	int (*get_name)(struct device *dev, char **name);
	int (*get_id)(struct device *dev, char **id);
	int (*get_irq_gpio)(struct device *dev, int *state);
	int (*get_version)(struct device *dev, const char **version);
};


enum meizu_sensor_id
{
	MEIZU_SENSOR_ID_ACC	= 0,
	MEIZU_SENSOR_ID_GYR	= 1,
	MEIZU_SENSOR_ID_COMPASS	= 2,
	MEIZU_SENSOR_ID_ALS	= 3,
	MEIZU_SENSOR_ID_PS	= 4,
	MEIZU_SENSOR_ID_MAX	= 5
};
typedef enum meizu_sensor_id meizu_sensor_id_t;


/**
 * meizu_sensor_register - register sensor to create sysfs attribute files.
 * @id: sensor id.
 * @dev: sensor device.
 * @ops: sensor callback functions
 *
 * Note: This interface will create a list of sysfs files according to sensor id.
 * And it will skip the files already created by sensor driver.
 *
 * If the sensor doesnt need to create sysfs files, set the ops to NULL. And the
 * sensor driver should create the standard sysfs files interfaces.
 *
 */
int meizu_sensor_register(meizu_sensor_id_t id,
			  struct device *dev,
			  struct meizu_sensors_ops *ops);


void meizu_sensor_unregister(meizu_sensor_id_t id,
			  struct device *dev,
			  struct meizu_sensors_ops *ops);

#endif /*__H_MEIZU_SENSORS_H__*/