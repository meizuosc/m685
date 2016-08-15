#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/mutex.h>

#include "timed_output.h"

#define VIBR_DEVICE	"mtk_vibrator"
#define TAG "[vibrator]"

/*****************************************************
 * DEBUG MACROS
 ****************************************************/
static int debug_enable_vib = 0;
#define VIBR_DEBUG(format, args...) do { \
	if (debug_enable_vib) {\
		pr_info("[vibrator]"format, ##args);\
	} \
} while (0)

static struct workqueue_struct *vibrator_queue;
static struct work_struct work_vibrator;
static struct hrtimer vibe_timer;
static spinlock_t vibe_lock;
static int vibe_state;
static int shutdown_flag;

extern void motor_enable(void);
extern void motor_disable(void);

static void update_vibrator(struct work_struct *work)
{
	if (vibe_state)
		motor_enable();
	else
		motor_disable();
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
        unsigned long   flags;
        spin_lock_irqsave(&vibe_lock, flags);
        hrtimer_cancel(&vibe_timer);

        if (value == 0 || shutdown_flag == 1)
                vibe_state = 0;
        else {
                value = (value > 15000 ? 15000 : value);
                vibe_state = 1;
                hrtimer_start(&vibe_timer,
                              ktime_set(value / 1000, (value % 1000) * 1000000),
                              HRTIMER_MODE_REL);
        }
        spin_unlock_irqrestore(&vibe_lock, flags);
        queue_work(vibrator_queue,&work_vibrator);
}
static int vibrator_get_time(struct timed_output_dev *dev)
{
        if (hrtimer_active(&vibe_timer)) {
                ktime_t r = hrtimer_get_remaining(&vibe_timer);
                return ktime_to_ms(r);
        } else
                return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
        vibe_state = 0;
        queue_work(vibrator_queue,&work_vibrator);
        return HRTIMER_NORESTART;
}

static struct timed_output_dev timed_vibrator = {
        .name = "vibrator",
        .get_time = vibrator_get_time,
        .enable = vibrator_enable,
};

static int vib_probe(struct platform_device *pdev)
{
	return 0;
}

static int vib_remove(struct platform_device *pdev)
{
	return 0;
}

static void vib_shutdown(struct platform_device *pdev)
{
	unsigned long  flags;
	VIBR_DEBUG("%s: enter!\n", __func__);
	spin_lock_irqsave(&vibe_lock, flags);
	shutdown_flag = 1;
	if(vibe_state) {
		VIBR_DEBUG("%s: vibrator will disable\n", __func__);
		vibe_state = 0;
		spin_unlock_irqrestore(&vibe_lock, flags);
		motor_disable();
	} else {
		spin_unlock_irqrestore(&vibe_lock, flags);
	}
}

/******************************************************************************
Device driver structure
*****************************************************************************/
static struct platform_driver vibrator_driver =
{
	.probe = vib_probe,
	.remove = vib_remove,
	.shutdown = vib_shutdown,
	.driver = {
		.name = VIBR_DEVICE,
		.owner = THIS_MODULE,
	},
};

static struct platform_device vibrator_device =
{
    .name = VIBR_DEVICE,
    .id   = -1,
};

static ssize_t store_vibr_on(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(buf != NULL && size != 0) {
		VIBR_DEBUG("%s: buf is %s and size is %zu \n",
				__func__, buf, size);
		if(buf[0]== '0')
			motor_disable();
		else
			motor_enable();
	}
	return size;
}

static DEVICE_ATTR(vibr_on, 0220, NULL, store_vibr_on);

static int __init init_vibrator(void)
{
	int ret = 0;

	ret = platform_device_register(&vibrator_device);
	if (ret != 0) {
		pr_err(TAG"%s: Unable to register vibrator device (%d)\n",
				__func__, ret);
		return ret;
	}
	vibrator_queue = create_singlethread_workqueue(VIBR_DEVICE);
	if(!vibrator_queue) {
		pr_err(TAG"%s: Unable to create workqueue\n", __func__);
		return -ENODATA;
	}

        INIT_WORK(&work_vibrator, update_vibrator);

        spin_lock_init(&vibe_lock);
	shutdown_flag = 0;
        vibe_state = 0;

        hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        vibe_timer.function = vibrator_timer_func;

        timed_output_dev_register(&timed_vibrator);

	ret = platform_driver_register(&vibrator_driver);
	if(ret) {
		pr_err(TAG"%s: Unable to register vibrator driver (%d)\n",
				__func__, ret);
		return ret;
	}

	ret = device_create_file(timed_vibrator.dev,&dev_attr_vibr_on);
	if(ret) {
		pr_err(TAG"%s: device_create_file vibr_on fail!\n",
				__func__);
	}

	return 0;
}

static void __exit exit_vibrator(void)
{
	if(vibrator_queue) {
		destroy_workqueue(vibrator_queue);
	}
	VIBR_DEBUG("vibrator exit\n");
}

module_init(init_vibrator);
module_exit(exit_vibrator);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("Timed output vibrator device");
MODULE_LICENSE("GPL v2");
