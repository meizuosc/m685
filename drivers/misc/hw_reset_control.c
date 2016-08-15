/*
 *  drivers/misc/hw_reset_control.c
 *
 * Copyright (c) 2016 MEIZU Technology Co., Ltd.
 * Author: xuhanlong <xuhanlong@meizu.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/meizu-sys.h>
#include <linux/hw_reset.h>

typedef struct hw_reset_control_device {
	const char    *name;
	int           disable;
	struct device *dev;
} hrc_dev_t;

hrc_dev_t hrc_dev = {
	.name		= "hw_reset",
	.disable	= 0,
	.dev		= NULL
};

static BLOCKING_NOTIFIER_HEAD(hw_reset_notifiers);

int register_hw_reset_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hw_reset_notifiers, nb);
}

int unregister_hw_reset_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hw_reset_notifiers, nb);
}

static int hw_reset_notifier_call_chain(unsigned long val)
{
	int ret = blocking_notifier_call_chain(&hw_reset_notifiers, val, NULL);

	return notifier_to_errno(ret);
}

static ssize_t disable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	hrc_dev_t *hdev = (hrc_dev_t *)dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", hdev->disable);
}

static ssize_t disable_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int disable;
	ssize_t ret = 0;
	hrc_dev_t *hdev = (hrc_dev_t *)dev_get_drvdata(dev);

	ret = sscanf(buf, "%d", &disable);
	if (ret == 0)
		return -EINVAL;

	dev_info(dev, "state: %d\n", disable);

	hdev->disable = !!disable;
	hw_reset_notifier_call_chain(hdev->disable ? HW_RESET_DISABLE : HW_RESET_ENABLE);

	return count;
}
static DEVICE_ATTR_RW(disable);

static struct work_struct power_key_work;

static void power_key_work_func(struct work_struct *work)
{
	hrc_dev.disable = 0;
	hw_reset_notifier_call_chain(HW_RESET_ENABLE);
}

static void power_key_event(struct input_handle *handle, unsigned int type,
			   unsigned int code, int value)
{
	if (type != EV_KEY || code != KEY_POWER)
		return;

	/* power key release */
	if (!value)
		schedule_work(&power_key_work);
}

static int power_key_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "power-key";

	error = input_register_handle(handle);
	if (error) {
		pr_err("Failed to register input power handler, error %d\n",
		       error);
		kfree(handle);
		return error;
	}

	error = input_open_device(handle);
	if (error) {
		pr_err("Failed to open input power device, error %d\n", error);
		input_unregister_handle(handle);
		kfree(handle);
		return error;
	}

	return 0;
}

static void power_key_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id power_key_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = {BIT_MASK(EV_KEY)},
		.keybit = {[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER)},
	},
	{}
};

static struct input_handler power_key_handler = {
	.event		= power_key_event,
	.connect	= power_key_connect,
	.disconnect	= power_key_disconnect,
	.name		= "power_key",
	.id_table	= power_key_ids,
};

static int __init hw_reset_control_init(void)
{
	int ret;

	hrc_dev.dev = device_create(meizu_class, NULL, MKDEV(0, 0), NULL, hrc_dev.name);
	if (IS_ERR(hrc_dev.dev))
		return PTR_ERR(hrc_dev.dev);

	ret = device_create_file(hrc_dev.dev, &dev_attr_disable);
	if (ret < 0)
		return ret;

	dev_set_drvdata(hrc_dev.dev, &hrc_dev);

	INIT_WORK(&power_key_work, power_key_work_func);

	ret = input_register_handler(&power_key_handler);
	if (ret < 0)
		goto err_input_handler;

	return 0;

err_input_handler:
	device_remove_file(hrc_dev.dev, &dev_attr_disable);
	return ret;
}

fs_initcall(hw_reset_control_init);

MODULE_DESCRIPTION("Meizu technology hardware reset control driver");
MODULE_AUTHOR("xuhanlong <xuhanlong@meizu.com>");
MODULE_LICENSE("GPL");

