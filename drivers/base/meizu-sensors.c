
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include "base.h"

#include <linux/meizu-sys.h>
#include <linux/meizu-sensors.h>

//#define MZ_SENSORS_DEBUG 1
#ifdef MZ_SENSORS_DEBUG
#define LOG_TAG_MZ_SENSORS "[mz_sensors]"
#define pr_info(format, arg...)         printk(KERN_EMERG LOG_TAG_MZ_SENSORS format , ## arg)
#define dev_err(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_MZ_SENSORS format , ## arg)
#define dev_info(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_MZ_SENSORS format , ## arg)
#define dev_dbg(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_MZ_SENSORS format , ## arg)
#define dev_warn(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_MZ_SENSORS format , ## arg)
#define dev_notice(dev, format, arg...) printk(KERN_EMERG LOG_TAG_MZ_SENSORS format , ## arg)
#endif

/* -- sensors ops -- */
struct meizu_sensors_table {
	struct device *dev;
	struct meizu_sensors_ops *ops;
	const char *name;
	struct device_attribute *attr;
	int attr_cnt;
} meizu_sensors_table[MEIZU_SENSOR_ID_MAX];

static int meizu_sensors_get_ops(struct device *dev,
	struct meizu_sensors_ops **ops)
{
	int i;

	for (i=0; i<MEIZU_SENSOR_ID_MAX; i++) {
		if (meizu_sensors_table[i].dev == dev) {
			*ops = meizu_sensors_table[i].ops;
			dev_dbg(dev, "get sensor(%d)'s ops: %p\n", i, *ops);
			return 0;
		}
	}

	dev_err(dev, "failed to get sensor ops\n");
	return -1;
}

static void meizu_sensors_set_ops(meizu_sensor_id_t id,
				  struct device *dev,
				  struct meizu_sensors_ops *ops)
{
	if (meizu_sensors_table[id].dev == NULL) {
		meizu_sensors_table[id].dev = dev;
		meizu_sensors_table[id].ops = ops;
		dev_dbg(dev, "set sensor(%d) ops: %pf\n", id, ops);
		return;
	}

	dev_err(dev, "failed to set sensor(%d) ops\n", id);
	BUG();
}



/* -- sensors sysfs interfaces -- */
/* self test interface */
static ssize_t meizu_sensors_sysfs_get_self_test(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct meizu_sensors_ops *ops;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->self_test) {
		ret = ops->self_test(dev);
	} else {
		dev_err(dev, "sensor has no self test interface\n");
		ret = -ENOSYS;
	}

	dev_info(dev, "self test result: %d\n", ret);

	return sprintf(buf, "%d\n", ret);
}



/* enable interface */
static ssize_t meizu_sensors_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int state;
	struct meizu_sensors_ops *ops;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_enable) {
		ret = ops->get_enable(dev, &state);
	} else {
		dev_err(dev, "sensor has no enable interface\n");
		ret = -ENOSYS;
	}

	return sprintf(buf, "%d\n", ret < 0 ? ret : !!state);
}

static ssize_t meizu_sensors_sysfs_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint16_t state;
	struct meizu_sensors_ops *ops;

	ret = kstrtou16(buf, 10, &state);
	if (ret < 0) {
		dev_err(dev, "kstrtoint error\n");
		return ret;
	}

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->set_enable) {
		ret = ops->set_enable(dev, state);
	} else {
		dev_err(dev, "sensor has no enable interface\n");
		ret = -ENOSYS;
	}

	return ret < 0 ? ret : count;
}



/* calibbias interfaces */
static int meizu_sensors_get_calibbias(struct device *dev, int32_t *bias)
{
	int ret;
	struct meizu_sensors_ops *ops;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_calibbias) {
		ret = ops->get_calibbias(dev, bias);
	} else {
		dev_err(dev, "sensor has no get calibbias interface\n");
		ret = -ENOSYS;
	}

	return ret;
}

static ssize_t meizu_sensors_sysfs_get_x_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t bias[3];

	ret = meizu_sensors_get_calibbias(dev, bias);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", bias[0]);
}

static ssize_t meizu_sensors_sysfs_get_y_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t bias[3];

	ret = meizu_sensors_get_calibbias(dev, bias);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", bias[1]);
}

static ssize_t meizu_sensors_sysfs_get_z_calibbias(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t bias[3];

	ret = meizu_sensors_get_calibbias(dev, bias);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", bias[2]);
}



/* offset interfaces */
static int meizu_sensors_set_offset(struct device *dev,
	const char *buf, int offset, int axis)
{
	int ret;
	struct meizu_sensors_ops *ops;

	if (buf != NULL) {
		ret = kstrtoint(buf, 10, &offset);
		if (ret < 0) {
			dev_err(dev, "kstrtoint error\n");
			return ret;
		}
	}

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->set_offset) {
		ret = ops->set_offset(dev, offset, axis);
	} else {
		dev_err(dev, "sensor has no set offset interface\n");
		ret = -ENOSYS;
	}

	return ret;
}

static ssize_t meizu_sensors_sysfs_set_x_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = meizu_sensors_set_offset(dev, buf, 0, 0);

	return ret < 0 ? ret : count;
}

static ssize_t meizu_sensors_sysfs_set_y_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = meizu_sensors_set_offset(dev, buf, 0, 1);

	return ret < 0 ? ret : count;
}

static ssize_t meizu_sensors_sysfs_set_z_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = meizu_sensors_set_offset(dev, buf, 0, 2);

	return ret < 0 ? ret : count;
}

static int meizu_sensors_get_offset(struct device *dev,
	int *offset)
{
	int ret;
	struct meizu_sensors_ops *ops;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_offset) {
		ret = ops->get_offset(dev, offset);
	} else {
		dev_err(dev, "sensor has no get offset interface\n");
		ret = -ENOSYS;
	}

	return ret;
}

static ssize_t meizu_sensors_sysfs_get_x_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int offset[3];

	ret = meizu_sensors_get_offset(dev, offset);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", offset[0]);
}

static ssize_t meizu_sensors_sysfs_get_y_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int offset[3];

	ret = meizu_sensors_get_offset(dev, offset);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", offset[1]);
}

static ssize_t meizu_sensors_sysfs_get_z_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int offset[3];

	ret = meizu_sensors_get_offset(dev, offset);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", offset[2]);
}


static int meizu_sensors_create_sysfs_interfaces(struct device *dev,
	struct device_attribute *attributes, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++) {
		if (device_create_file(dev, attributes + i)) {
			/* ignore the fail, the file may created by driver */
			dev_info(dev, "%s has created by sensor driver\n",
				attributes[i].attr.name);
		}
	}
	return 0;

}

static void meizu_sensors_destory_sysfs_interfaces(struct device *dev,
	struct device_attribute *attributes, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++)
		device_remove_file(dev, attributes + i);
}



/* calibrate interfaces */
static ssize_t meizu_sensors_sysfs_get_calibrate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct meizu_sensors_ops *ops;
	int32_t bias[3] = {0,0,0};

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->calibrate) {
		ret = ops->calibrate(dev);
	} else {
		dev_err(dev, "sensor has no calibrate interface\n");
		ret = -ENOSYS;
	}

	if (ret >= 0) {
		/* apply the bias immediately */
		meizu_sensors_get_calibbias(dev, bias);
		meizu_sensors_set_offset(dev, NULL, bias[0], 0);
		meizu_sensors_set_offset(dev, NULL, bias[1], 1);
		meizu_sensors_set_offset(dev, NULL, bias[2], 2);
	}

	return sprintf(buf, "%d\n", ret);
}



static ssize_t meizu_sensors_sysfs_set_calibrate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint16_t state;
	struct meizu_sensors_ops *ops;
	int32_t bias[3] = {0,0,0};

	ret = kstrtou16(buf, 10, &state);
	if (ret < 0) {
		dev_err(dev, "kstrtoint error\n");
		return ret;
	}

	if (state != 1) {
		dev_err(dev, "only write 1 is valid\n");
		return -EINVAL;
	}

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->calibrate) {
		ret = ops->calibrate(dev);
	} else {
		dev_err(dev, "sensor has no calibrate interface\n");
		ret = -ENOSYS;
	}

	if (ret >= 0) {
		/* apply the bias immediately */
		meizu_sensors_get_calibbias(dev, bias);
		meizu_sensors_set_offset(dev, NULL, bias[0], 0);
		meizu_sensors_set_offset(dev, NULL, bias[1], 1);
		meizu_sensors_set_offset(dev, NULL, bias[2], 2);
	}
	return ret < 0 ? ret : count;
}



/* raw data interfaces */
static int meizu_sensors_get_raw_data(struct device *dev,
	int *raw)
{
	int ret;
	struct meizu_sensors_ops *ops;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_raw_data) {
		ret = ops->get_raw_data(dev, raw);
	} else {
		dev_err(dev, "sensor has no get offset interface\n");
		ret = -ENOSYS;
	}

	return ret;
}

static ssize_t meizu_sensors_sysfs_get_x_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int raw[3];

	ret = meizu_sensors_get_raw_data(dev, raw);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", raw[0]);
}

#if 0   //delete by yubin for build error, because they are not used.
static ssize_t meizu_sensors_sysfs_get_y_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int raw[3];

	ret = meizu_sensors_get_raw_data(dev, raw);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", raw[1]);
}

static ssize_t meizu_sensors_sysfs_get_z_raw_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	int raw[3];

	ret = meizu_sensors_get_raw_data(dev, raw);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", raw[2]);
}
#endif


/* name and id interfaces */
static ssize_t meizu_sensors_sysfs_get_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct meizu_sensors_ops *ops;
	char *name;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_name) {
		ret = ops->get_name(dev, &name);
	} else {
		dev_err(dev, "sensor has no name interface\n");
		ret = -ENOSYS;
	}

	return sprintf(buf, "%s\n", ret < 0 ? "NA" : name);
}

static ssize_t meizu_sensors_sysfs_get_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct meizu_sensors_ops *ops;
	char *id;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_id) {
		ret = ops->get_id(dev, &id);
	} else {
		dev_err(dev, "sensor has no id interface\n");
		ret = -ENOSYS;
	}

	return sprintf(buf, "%s\n", ret < 0 ? "NA" : id);
}


static ssize_t meizu_sensors_sysfs_get_irq_gpio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct meizu_sensors_ops *ops;
	int state = 0;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_irq_gpio) {
		ret = ops->get_irq_gpio(dev, &state);
	} else {
		dev_err(dev, "sensor has no irq_gpio interface\n");
		ret = -ENOSYS;
	}

	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", state);
}

static ssize_t meizu_sensors_sysfs_get_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct meizu_sensors_ops *ops;
	const char *version = NULL;

	ret = meizu_sensors_get_ops(dev, &ops);
	if (ret < 0)
		return ret;

	if (ops->get_version) {
		ret = ops->get_version(dev, &version);
	} else {
		dev_err(dev, "sensor has no version interface\n");
		ret = -ENOSYS;
	}
	if (ret < 0)
		return ret;

	return sprintf(buf, "%s\n", version);
}

/* -- sensor attributes -- */
static struct device_attribute compass_attributes[] = {
	__ATTR(compass_name, 0440,
		meizu_sensors_sysfs_get_name,
		NULL),
	__ATTR(compass_id, 0440,
		meizu_sensors_sysfs_get_id,
		NULL),

	__ATTR(compass_self_test, 0440,
		meizu_sensors_sysfs_get_self_test,
		NULL),
	__ATTR(compass_enable, 0660,
		meizu_sensors_sysfs_get_enable,
		meizu_sensors_sysfs_set_enable),
	__ATTR(version, 0440,
		meizu_sensors_sysfs_get_version,
		NULL),
};

static struct device_attribute acc_attributes[] = {
	__ATTR(acc_name, 0440,
		meizu_sensors_sysfs_get_name,
		NULL),
	__ATTR(acc_id, 0440,
		meizu_sensors_sysfs_get_id,
		NULL),

	__ATTR(acc_self_test, 0440,
		meizu_sensors_sysfs_get_self_test,
		NULL),

	__ATTR(acc_enable, 0660,
		meizu_sensors_sysfs_get_enable,
		meizu_sensors_sysfs_set_enable),

	__ATTR(acc_calibration, 0660,
		meizu_sensors_sysfs_get_calibrate,
		meizu_sensors_sysfs_set_calibrate),

	__ATTR(acc_x_calibbias, 0440,
		meizu_sensors_sysfs_get_x_calibbias,
		NULL),
	__ATTR(acc_y_calibbias, 0440,
		meizu_sensors_sysfs_get_y_calibbias,
		NULL),
	__ATTR(acc_z_calibbias, 0440,
		meizu_sensors_sysfs_get_z_calibbias,
		NULL),

	__ATTR(acc_x_offset, 0660,
		meizu_sensors_sysfs_get_x_offset,
		meizu_sensors_sysfs_set_x_offset),
	__ATTR(acc_y_offset, 0660,
		meizu_sensors_sysfs_get_y_offset,
		meizu_sensors_sysfs_set_y_offset),
	__ATTR(acc_z_offset, 0660,
		meizu_sensors_sysfs_get_z_offset,
		meizu_sensors_sysfs_set_z_offset),
	__ATTR(version, 0440,
		meizu_sensors_sysfs_get_version,
		NULL),
};

static struct device_attribute gyr_attributes[] = {
	__ATTR(gyr_name, 0440,
		meizu_sensors_sysfs_get_name,
		NULL),
	__ATTR(gyr_id, 0440,
		meizu_sensors_sysfs_get_id,
		NULL),

	__ATTR(gyr_self_test, 0440,
		meizu_sensors_sysfs_get_self_test,
		NULL),

	__ATTR(gyr_enable, 0660,
		meizu_sensors_sysfs_get_enable,
		meizu_sensors_sysfs_set_enable),
	__ATTR(version, 0440,
		meizu_sensors_sysfs_get_version,
		NULL),
};

static struct device_attribute als_attributes[] = {
	__ATTR(als_name, 0440,
		meizu_sensors_sysfs_get_name,
		NULL),
	__ATTR(als_id, 0440,
		meizu_sensors_sysfs_get_id,
		NULL),

	__ATTR(als_enable, 0660,
		meizu_sensors_sysfs_get_enable,
		meizu_sensors_sysfs_set_enable),

	__ATTR(als_data, 0440,
		meizu_sensors_sysfs_get_x_raw_data,
		NULL),
};

static struct device_attribute ps_attributes[] = {
	__ATTR(ps_name, 0440,
		meizu_sensors_sysfs_get_name,
		NULL),
	__ATTR(ps_id, 0440,
		meizu_sensors_sysfs_get_id,
		NULL),

	__ATTR(ps_enable, 0660,
		meizu_sensors_sysfs_get_enable,
		meizu_sensors_sysfs_set_enable),

	__ATTR(ps_calibration, 0660,
		meizu_sensors_sysfs_get_calibrate,
		meizu_sensors_sysfs_set_calibrate),

	__ATTR(ps_data, 0440,
		meizu_sensors_sysfs_get_x_raw_data,
		NULL),

	__ATTR(ps_calibbias, 0440,
		meizu_sensors_sysfs_get_x_calibbias,
		NULL),

	__ATTR(ps_offset, 0660,
		meizu_sensors_sysfs_get_x_offset,
		meizu_sensors_sysfs_set_x_offset),

	__ATTR(ps_irq_gpio, 0440,
		meizu_sensors_sysfs_get_irq_gpio,
		NULL),
};

int meizu_sensor_register(meizu_sensor_id_t id,
			  struct device *dev,
			  struct meizu_sensors_ops *ops)
{
	int ret;

	if (!dev || id >= MEIZU_SENSOR_ID_MAX || id<0)
		return -EINVAL;

	dev_info(dev, "meizu_sensor_register id: %d, name: %s\n",
		id, meizu_sensors_table[id].name);

	ret = meizu_sysfslink_register_name(dev, (char*)meizu_sensors_table[id].name);
	if (ret < 0) {
		dev_err(dev, "failed to register sysfs link for sensor.\n");
		return ret;
	}

	if (ops == NULL) {
		/* if ops is NULL, it means we dont need to create sysfs */
		dev_info(dev, "no sensor callback.\n");
		return 0;
	}

	ret = meizu_sensors_create_sysfs_interfaces(dev,
		meizu_sensors_table[id].attr, meizu_sensors_table[id].attr_cnt);
	if (ret < 0) {
		dev_err(dev, "failed to create_sysfs for sensor (%d).\n", id);
		goto unregister_sysfslink;
	}

	meizu_sensors_set_ops(id, dev, ops);

	return 0;

unregister_sysfslink:
#if 0 /* bug in meizu_sysfslink_unregister */
	meizu_sysfslink_unregister(dev);
#endif
	return ret;

}


void meizu_sensor_unregister(meizu_sensor_id_t id,
			  struct device *dev,
			  struct meizu_sensors_ops *ops)
{
	int ret;
	struct meizu_sensors_ops *tmp_ops;

	if (!dev || !ops || id >= MEIZU_SENSOR_ID_MAX || id<0)
		return;

	dev_info(dev, "meizu_sensor_unregister id: %d, name: %s\n",
		id, meizu_sensors_table[id].name);

	ret = meizu_sensors_get_ops(dev, &tmp_ops);
	if (ret < 0)
		return;

	if (tmp_ops != ops) {
		return;
	}

#if 0 /* bug in meizu_sysfslink_unregister */
	meizu_sysfslink_unregister(dev);
#endif

	meizu_sensors_destory_sysfs_interfaces(dev,
		meizu_sensors_table[id].attr, meizu_sensors_table[id].attr_cnt);
}

static int __init meizu_sensors_init(void)
{
	meizu_sensors_table[MEIZU_SENSOR_ID_ACC].name = "acc";
	meizu_sensors_table[MEIZU_SENSOR_ID_ACC].attr = acc_attributes;
	meizu_sensors_table[MEIZU_SENSOR_ID_ACC].attr_cnt = ARRAY_SIZE(acc_attributes);

	meizu_sensors_table[MEIZU_SENSOR_ID_GYR].name = "gyr";
	meizu_sensors_table[MEIZU_SENSOR_ID_GYR].attr = gyr_attributes;
	meizu_sensors_table[MEIZU_SENSOR_ID_GYR].attr_cnt = ARRAY_SIZE(gyr_attributes);

	meizu_sensors_table[MEIZU_SENSOR_ID_COMPASS].name = "compass";
	meizu_sensors_table[MEIZU_SENSOR_ID_COMPASS].attr = compass_attributes;
	meizu_sensors_table[MEIZU_SENSOR_ID_COMPASS].attr_cnt = ARRAY_SIZE(compass_attributes);

	meizu_sensors_table[MEIZU_SENSOR_ID_ALS].name = "als";
	meizu_sensors_table[MEIZU_SENSOR_ID_ALS].attr = als_attributes;
	meizu_sensors_table[MEIZU_SENSOR_ID_ALS].attr_cnt = ARRAY_SIZE(als_attributes);

	meizu_sensors_table[MEIZU_SENSOR_ID_PS].name = "ps";
	meizu_sensors_table[MEIZU_SENSOR_ID_PS].attr = ps_attributes;
	meizu_sensors_table[MEIZU_SENSOR_ID_PS].attr_cnt = ARRAY_SIZE(ps_attributes);

	pr_info("meizu_sensors_init done\n");

	return 0;
}

static void __exit meizu_sensors_exit(void)
{
	return;
}

subsys_initcall(meizu_sensors_init);
module_exit(meizu_sensors_exit);

MODULE_DESCRIPTION("Meizu technology sensors sysfs interfaces");
MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_LICENSE("GPL");
