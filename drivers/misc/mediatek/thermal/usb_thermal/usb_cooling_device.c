#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/wait.h>

#include <linux/meizu-sys.h>

#include "usb_cooling_device.h"

//struct pinctrl *charge_pinctrl = NULL;
//struct pinctrl_state *charge_enable_state = NULL, *charge_diable_state = NULL;

static atomic_t is_usb_closed_by_thermal = ATOMIC_INIT(0);
static int need_close_thermal;
static int is_running = 0;  /*the thermal thread is running or not.*/
static DEFINE_SPINLOCK(usb_spinlock);

static struct delayed_work usb_thermal_work;
static int work_delay_time = HZ / 2; /* 500ms */

static void (*usb_cooling_listener)(int is_hot, int temp) = (void *)NULL;
////////////////////////////////////////////////////////

#define CONFIG_USB_COOLING_DEVICE_DEBUG

#ifdef CONFIG_USB_COOLING_DEVICE_DEBUG
static int temp_from_user = 0;
static ssize_t temp_from_user_store(struct device *dev,   
                    struct device_attribute *attr,   
                    const char *buf, size_t count)   
{  
    sscanf(buf, "%d", &temp_from_user);
    return count;  
}

static ssize_t temp_from_user_show(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    int len = sprintf(buf, "%d\n", temp_from_user);
    return len;
}

static struct device_attribute dev_attr_temp_from_user = {
	.attr = {
		.name = "temp_from_user",
		.mode = 0666,
	},
	.show	= temp_from_user_show,
	.store	= temp_from_user_store,
};
//static DEVICE_ATTR(temp_from_user, 0666, temp_from_user_show, temp_from_user_store);
#endif /*CONFIG_USB_COOLING_DEVICE_DEBUG*/

////////////////////////////////////////////////////////

static ssize_t temp_show(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    int len = sprintf(buf, "%d\n", usb_thermal_get_temp());
    return len;
}

static struct device_attribute dev_attr_temp = {
	.attr = {
		.name = "temp",
		.mode = 0444,
	},
	.show	= temp_show,
	.store	= NULL,
};

static struct device_attribute hi_temp_warn_attr_temp = {
	.attr = {
		.name = "hi_temp_warn",
		.mode = 0444,
	},
	.show	= temp_show,
	.store	= NULL,
};

/////////////////////////////////////////////////////////
static DECLARE_WAIT_QUEUE_HEAD(sync_temp_waitqueue);
static int is_hot = 0;
static ssize_t sync_temp_show(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
	int len = 0;
	static int last_is_hot = 1;

	wait_event_interruptible(sync_temp_waitqueue, is_hot != last_is_hot);
	last_is_hot = is_hot;

	len = sprintf(buf, "%d\n", usb_thermal_get_temp());
	return len;
}

static struct device_attribute sync_temp_attr = {
	.attr = {
		.name = "sync_temp",
		.mode = 0444,
	},
	.show	= sync_temp_show,
	.store	= NULL,
};
//static DEVICE_ATTR(temp, 0, temp_show, NULL);

//////////////////////////////////////////////////////////

int set_usb_cooling_listener(void (*listener)(int is_hot, int temp))
{
	spin_lock(&usb_spinlock);
	usb_cooling_listener = listener;
	spin_unlock(&usb_spinlock);
	return 0;
}

///////////////////////////////////////////////////////

static inline int workqueue_must_running(void)
{
	return atomic_read(&is_usb_closed_by_thermal);
}

static void usb_cooling_handler(struct work_struct *work)
{
	int usb_temp = 0;
	usb_temp = usb_thermal_get_temp();

#ifdef CONFIG_USB_COOLING_DEVICE_DEBUG
	if(temp_from_user) {
		usb_temp = temp_from_user;
	}
#endif /*CONFIG_USB_COOLING_DEVICE_DEBUG*/

	//printk("[usb_thermal] cooling is running. temp:%d, is:%d\n", 
	//	usb_temp, atomic_read(&is_usb_closed_by_thermal));

	spin_lock(&usb_spinlock);
	if(usb_temp >= CONFIG_DISABLE_CHARGE_TEMP) {
		if(!atomic_read(&is_usb_closed_by_thermal)) {
			is_hot = 1;
			if(usb_cooling_listener) {
				usb_cooling_listener(is_hot, usb_temp);
			}
			wake_up(&sync_temp_waitqueue);
		}
		atomic_set(&is_usb_closed_by_thermal, 1);
		//pinctrl_select_state(charge_pinctrl, charge_diable_state);
		printk("[usb_thermal] WORNING: Diable usb charge. temp:%d\n", usb_temp);
	} else if(usb_temp <= CONFIG_REOPEN_CHARGE_TEMP) {
		if(atomic_read(&is_usb_closed_by_thermal)) {
			is_hot = 0;
			if(usb_cooling_listener) {
				usb_cooling_listener(is_hot, usb_temp);
			}
			wake_up(&sync_temp_waitqueue);
		}
		atomic_set(&is_usb_closed_by_thermal, 0);
		//pinctrl_select_state(charge_pinctrl, charge_enable_state);
	}

	if(workqueue_must_running()) {  /* must running*/
		schedule_delayed_work(&usb_thermal_work, work_delay_time);
		is_running = 1;
	} else if(!need_close_thermal) {
		schedule_delayed_work(&usb_thermal_work, work_delay_time);
		is_running = 1;
	} else {
		is_running = 0;
	}
	spin_unlock(&usb_spinlock);
}

int diable_usb_cooling_by_async(void)
{
	spin_lock(&usb_spinlock);
	need_close_thermal = 1;
	spin_unlock(&usb_spinlock);
	return 0;
}

/**
*   enable usb cooling device.
*/
int enable_usb_cooling(void)
{
	spin_lock(&usb_spinlock);
	need_close_thermal = 0;

	/* cooling handler is not running*/
	if(!is_running) {
		INIT_DELAYED_WORK(&usb_thermal_work, usb_cooling_handler);
		is_running = 1;
		schedule_delayed_work(&usb_thermal_work, 0);
	}
	spin_unlock(&usb_spinlock);
	return 0;
}

static int usb_thermal_probe(struct platform_device *pdev)
{
	int ret = 0;

#if 0
	/*Get pinctrl*/
	charge_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(charge_pinctrl)) {
		ret = PTR_ERR(charge_pinctrl);
		dev_err(&pdev->dev, "Cannot find charge pinctrl!\n");
		return ret;
	}

	/*enable state*/
	charge_enable_state = pinctrl_lookup_state(charge_pinctrl, "charge_gpio_enable");
	if (IS_ERR(charge_enable_state)) {
		ret = PTR_ERR(charge_enable_state);
		dev_err(&pdev->dev, "Cannot find charge pinctrl charge_enable_state!\n");
		return ret;
	}

	/*diable state*/
	charge_diable_state = pinctrl_lookup_state(charge_pinctrl, "charge_gpio_diable");
	if (IS_ERR(charge_diable_state)) {
		ret = PTR_ERR(charge_diable_state);
		dev_err(&pdev->dev, "Cannot find charge pinctrl charge_diable_state!\n");
		return ret;
	}
#endif

#ifdef CONFIG_USB_COOLING_DEVICE_DEBUG
	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_temp_from_user.attr);
	if(ret < 0) {
		printk("[usb_thermal]ERROR: create temp_from_user file failed.\n");
		goto exit;
	}
#endif /*CONFIG_USB_COOLING_DEVICE_DEBUG*/

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_temp.attr);
	if(ret < 0) {
		printk("[usb_thermal]ERROR: create temp file failed.\n");
		goto exit;
	}

	ret = sysfs_create_file(&pdev->dev.kobj, &hi_temp_warn_attr_temp.attr);
	if(ret < 0) {
		printk("[usb_thermal]ERROR: create hi_temp_warn file failed.\n");
	}

	ret = sysfs_create_file(&pdev->dev.kobj, &sync_temp_attr.attr);
	if(ret < 0) {
		printk("[usb_thermal]ERROR: create sync_temp file failed.\n");
	}

	meizu_sysfslink_register_name(&pdev->dev, "usb_thermal");

	/* enable */
	//pinctrl_select_state(charge_pinctrl, charge_enable_state);

	enable_usb_cooling();

exit:
	return ret;
}

static struct of_device_id usb_cooling_deivce_match[] = {
	{ .compatible = "usb-cooling-device", },
	{ /* sentinel */ }
};

static struct platform_driver usb_thermal_driver = {
	.driver = { 
		.owner = THIS_MODULE,
		.name = "usb-cooling",
		.of_match_table = usb_cooling_deivce_match,
	},
	.probe = usb_thermal_probe,
};

static __init int usb_cooling_device_init(void)
{
	int ret = 0;
	printk("[usb_thermal] usb cooling deivce register.\n");
	ret = platform_driver_register(&usb_thermal_driver);
	if(ret < 0) {
		printk("[usb_thermal] cooling device failed.\n");
	}
	return ret;
}
late_initcall(usb_cooling_device_init);

MODULE_AUTHOR("bsp@meizu.com");
MODULE_DESCRIPTION("usb cooling device driver");
MODULE_LICENSE("GPL");
