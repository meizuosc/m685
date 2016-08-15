#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define TAG "[product_test]"
struct pinctrl *gpctrl = NULL;
struct pinctrl_state *set_sim1_gpio_mode = NULL;
struct pinctrl_state *set_sim1_eint_mode = NULL;
static unsigned int gpio_sim1_num; /* SIM1 detect hot plug gpio number */

static int sim_card_status = 3;
static bool sim_card_detect;

module_param(sim_card_status,int, S_IRUGO);

static int enable_sim_gpio(const char *val, const struct kernel_param *kp)
{
	int rt = param_set_bool(val, kp);
	int temp = 3;

	if (rt)
		return rt;

	if (sim_card_detect) {
		pinctrl_select_state(gpctrl, set_sim1_gpio_mode);
		usleep_range(900, 1000);

		if (gpio_sim1_num != 0)
			temp = __gpio_get_value(gpio_sim1_num);
		printk(TAG"gpio_sim1_num=%u\n", gpio_sim1_num);

		printk(TAG"meizu sim_card_detect enable\n");
	} else {
		pinctrl_select_state(gpctrl, set_sim1_eint_mode);
		usleep_range(900, 1000);
		printk(TAG"meizu sim_card_detect disable\n");
	}

	/* M80 project */
	if (temp == 0)
		sim_card_status = 0; /* removed sim slot */
	if (temp == 1)
		sim_card_status = 1; /* inserted sim slot */

	printk(TAG"INT_SIM gpio status=%d\n", sim_card_status);

	return 0;
}

static struct kernel_param_ops md_enabled_gpio_ops = {
	.set = enable_sim_gpio,
	.get = param_get_bool,
};

static int check_sim_slot_probe(struct platform_device *pdev)
{
	struct device_node *node = NULL;

	/* pinctrl init */
	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		pr_err(TAG"%s: Cannot find pinctrl!\n", __func__);
		return -1;
	}

	set_sim1_gpio_mode = pinctrl_lookup_state(gpctrl, "sim1_gpio_mode");
	if (NULL == set_sim1_gpio_mode) {
		pr_err(TAG"%s: Cannot find sim1 slot pin default state!\n", __func__);
		return -1;
	}

	set_sim1_eint_mode = pinctrl_lookup_state(gpctrl, "sim1_eint_mode");
	if (NULL == set_sim1_eint_mode) {
		pr_err(TAG"%s: Cannot find sim1 eint mode state!\n", __func__);
		return -1;
	}

	/* gpio init */
	node = of_find_compatible_node(NULL, NULL, "mediatek,check_sim_slot");
	if (node) {
		of_property_read_u32_array(node, "gpio-detect-sim1-hot-plug",
					   &(gpio_sim1_num), 1);
	} else {
		pr_err(TAG"%s: get sim1 slot pin gpio num err.\n", __func__);
		return -1;
	}

	pr_info(TAG"%s : success\n", __func__);

	return 0;
}

static int check_sim_slot_remove(struct platform_device *pdev)
{
	pr_debug(TAG"%s : &pdev=%p\n", __func__, pdev);
	return 0;
}

static const struct of_device_id check_sim_slot_match[] = {
	{ .compatible = "mediatek,check_sim_slot", },
	{},
};
MODULE_DEVICE_TABLE(of, check_sim_slot_match);

static struct platform_driver check_sim_slot_driver = {
	.probe = check_sim_slot_probe,
	.remove = check_sim_slot_remove,
	.driver = {
		   .name = "check_sim_slot",
		   .owner = THIS_MODULE,
		   .of_match_table = check_sim_slot_match,
	},
};

static int sim_int_to_gpio_init(void)
{
	return platform_driver_register(&check_sim_slot_driver);
}

static void sim_int_to_gpio_exit(void)
{
	printk(TAG"Goodbye sim to gpio module exit.\n");
	platform_driver_unregister(&check_sim_slot_driver);
}

late_initcall(sim_int_to_gpio_init);
module_exit(sim_int_to_gpio_exit);
module_param_cb(sim_card_detect, &md_enabled_gpio_ops, &sim_card_detect, S_IRUGO | S_IWUSR);
MODULE_AUTHOR("Yin ShunQing <ysq@meizu.com>");
MODULE_DESCRIPTION("Read sim card slot status");
MODULE_ALIAS("a product test program");
MODULE_LICENSE("GPL v2");
