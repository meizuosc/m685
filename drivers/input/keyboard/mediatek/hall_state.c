#include <linux/init.h>
#include <linux/module.h>
#include <linux/meizu-sys.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of.h>

#define TAG "[HALL]"

extern int meizu_hall_state;

static ssize_t hall_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", meizu_hall_state);
}

static DEVICE_ATTR(hall_state, 0444, hall_state_show, NULL);

static struct attribute *gpio_keys_attrs[] = {
        &dev_attr_hall_state.attr,
        NULL,
};

static struct attribute_group gpio_keys_attr_group = {
        .attrs = gpio_keys_attrs,
};

static int hall_probe(struct platform_device *pdev)
{
	int rc = 0;

	rc = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (rc) {
		pr_err(TAG"%s : sysfs create failed!\n", __func__);
		return -1;
	}
	rc = meizu_sysfslink_register(&pdev->dev);
	if (rc < 0) {
		pr_err(TAG"%s : create sysfs link failed!\n", __func__);
		sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
		return rc;
	}
	pr_debug(TAG"%s : success\n", __func__);

	return 0;
}

static int hall_remove(struct platform_device *pdev)
{
	pr_debug(TAG"%s : &pdev=%p\n", __func__, pdev);
	meizu_sysfslink_unregister(&pdev->dev);
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	return 0;
}

static const struct of_device_id hall_state_of_match[] = {
	{ .compatible = "mediatek,hall", },
	{},
};
MODULE_DEVICE_TABLE(of, hall_state_of_match);

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		   .name = "hall",
		   .owner = THIS_MODULE,
		   .of_match_table = hall_state_of_match,
	},
};

static int __init hall_init(void)
{
	/* register hall state platform driver */
	return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
	printk(TAG"Goodbye sim to gpio module exit.\n");
	platform_driver_unregister(&hall_driver);
}

late_initcall(hall_init);
module_exit(hall_exit);

MODULE_AUTHOR("Yin ShunQing <ysq@meizu.com>");
MODULE_DESCRIPTION("HALL state show");
MODULE_LICENSE("GPL v2");
