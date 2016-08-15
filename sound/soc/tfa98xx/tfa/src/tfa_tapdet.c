#include <linux/module.h>
#include <sound/core.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>

#include "tfa_tapdet.h"
#include "tfa.h"

extern int mt_soc_smartpa_i2s_clk_enable(bool enable);
extern int tfa98xx_set_tapdet_profile_enable(struct tfa98xx *tfa98xx, bool enable);

static bool tapdet_enable = true;
static unsigned long input_jiffies;
static DEFINE_MUTEX(input_mutex);

#define TAPDET_ENABLE_DELAY_TIME	300

void tapdet_update_input_event_time(void)
{
	mutex_lock(&input_mutex);
	input_jiffies = jiffies;
	mutex_unlock(&input_mutex);
}
EXPORT_SYMBOL(tapdet_update_input_event_time);


int tfa98xx_tapdet_report_check(struct tfa98xx *tfa98xx)
{
	pr_info("%s: time_delta = %ld , HZ = %d\n", __func__, jiffies - input_jiffies, HZ);

	if (time_after(jiffies, input_jiffies + HZ / 2)) {	//500ms
		return 0;
	} else {
		return -1;
	}
}

static int tfa98xx_tapdet_clk_enable(bool enable)
{
	return mt_soc_smartpa_i2s_clk_enable(enable);
}

static void tfa98xx_tapdet_enable_work(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, tapdet_enable_work.work);

	tfa98xx_tapdet_clk_enable(true);
	if (tfa98xx_set_tapdet_profile_enable(tfa98xx, true)) {
		pr_err("%s: set tapdet profile failed.\n", __func__);
		tfa98xx_tapdet_clk_enable(false);
	}
}

int tfa98xx_tapdet_enable(struct tfa98xx *tfa98xx, bool enable)
{
	static bool tapdet_enabled = 0;

	pr_info("%s: tapdet_enabled = %d, enable = %d.\n", __func__, tapdet_enabled, enable);

	if (tapdet_enabled == enable) {
		return 0;
	}

	cancel_delayed_work_sync(&tfa98xx->tapdet_enable_work);

	if (enable) {
		queue_delayed_work(tfa98xx->tfa98xx_wq,
					&tfa98xx->tapdet_enable_work,
					msecs_to_jiffies(TAPDET_ENABLE_DELAY_TIME));
	} else {
		tfa98xx_set_tapdet_profile_enable(tfa98xx, false);
		tfa98xx_tapdet_clk_enable(false);
	}

	tapdet_enabled = enable;

	return 0;
}

static ssize_t tapdet_store_active(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret = -1;
	struct tfa98xx * tfa98xx = dev_get_drvdata(dev);

	pr_info("%s: buf(%s), tapdet_enable(%d)\n", __func__, buf, tapdet_enable);

	mutex_lock(&tfa98xx->tapdet_mutex);

	if (!tapdet_enable) {
		mutex_unlock(&tfa98xx->tapdet_mutex);
		return -1;
	}

	if (!strncmp(buf, "1", 1)) {
		if (tfa98xx->status == TFA98XX_STANDBY) {
			ret = tfa98xx_tapdet_enable(tfa98xx, true);
			if (!ret) {
				tfa98xx->status = TFA98XX_TAPDET;
			}
		} else {
			pr_err("%s: tfa98xx->status is %d, can't active tapdet.\n", __func__, tfa98xx->status);
			//force enable tapdet
			//mutex_unlock(&tfa98xx->tapdet_mutex);
			//return -1;
		}
		mutex_lock(&input_mutex);
		input_jiffies = jiffies;
		mutex_unlock(&input_mutex);
		tfa98xx->tapdet_active = true;
	} else {
		if (tfa98xx->status == TFA98XX_TAPDET) {
			tfa98xx_tapdet_enable(tfa98xx, false);
			tfa98xx->status = TFA98XX_STANDBY;
		}
		tfa98xx->tapdet_active = false;
	}
	mutex_unlock(&tfa98xx->tapdet_mutex);
    return count;
}

static ssize_t tapdet_show_active(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct tfa98xx * tfa98xx = dev_get_drvdata(dev);
	int active;

	mutex_lock(&tfa98xx->tapdet_mutex);
	active = tfa98xx->tapdet_active;
	mutex_unlock(&tfa98xx->tapdet_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", active);
}

static ssize_t tapdet_store_enable(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct tfa98xx * tfa98xx = dev_get_drvdata(dev);

	pr_info("%s: buf(%s)\n", __func__, buf);

	mutex_lock(&tfa98xx->tapdet_mutex);
	if (!strncmp(buf, "1", 1)) {
		tapdet_enable = true;
	} else {
		tapdet_enable = false;
		if (tfa98xx->status == TFA98XX_TAPDET) {
			tfa98xx_tapdet_enable(tfa98xx, false);
			tfa98xx->status = TFA98XX_STANDBY;
		}
		tfa98xx->tapdet_active = false;
	}
	mutex_unlock(&tfa98xx->tapdet_mutex);
    return count;
}

static ssize_t tapdet_show_enable(struct device *dev,
				 struct device_attribute *attr, char *buf)
{

	pr_info("%s: tapdet_enable(%d)\n", __func__, tapdet_enable);

	return snprintf(buf, PAGE_SIZE, "%d\n", tapdet_enable);
}


static DEVICE_ATTR(active,	S_IWUSR | S_IRUGO, tapdet_show_active, tapdet_store_active);
static DEVICE_ATTR(enable,	S_IWUSR | S_IRUGO, tapdet_show_enable, tapdet_store_enable);

static struct attribute *tapdet_attributes[] = {
	&dev_attr_active.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group tapdet_attribute_group = {
	.attrs = tapdet_attributes
};


int tfa98xx_tapdet_register_device(struct tfa98xx *tfa98xx)
{
	int ret = -1;
	struct device *this_device;

	tfa98xx->tapdet_device.minor = MISC_DYNAMIC_MINOR;
    tfa98xx->tapdet_device.name = "tapdet";

	ret = misc_register(&tfa98xx->tapdet_device);		//tfa98xx_tapdet_device
	if (ret < 0) {
		return -1;
	}

	INIT_DELAYED_WORK(&tfa98xx->tapdet_enable_work, tfa98xx_tapdet_enable_work);

	this_device = tfa98xx->tapdet_device.this_device;
	
	dev_set_drvdata(this_device, tfa98xx);
	ret = sysfs_create_group(&this_device->kobj, &tapdet_attribute_group);
	if (ret < 0) {
		pr_err("unable to create tapdet attribute file\n");
		misc_deregister(&tfa98xx->tapdet_device);
		return -1;
	}
	kobject_uevent(&this_device->kobj, KOBJ_ADD);

	//move to tfa98xx i2c probe
	//mutex_init(&tfa98xx->tapdet_mutex);

	return 0;
}

int tfa98xx_tapdet_deregister_device(struct tfa98xx *tfa98xx)
{
	struct device *this_device = tfa98xx->tapdet_device.this_device;

	cancel_delayed_work_sync(&tfa98xx->tapdet_enable_work);

	sysfs_remove_group(&this_device->kobj, &tapdet_attribute_group);
	misc_deregister(&tfa98xx->tapdet_device);

	//mutex_destroy(&tfa98xx->tapdet_mutex);

	return 0;
}
