#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <typec.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>

static struct usb_typc_driver g_typc_driver={NULL};

#define K_EMERG	(1<<7)
#define K_QMU	(1<<7)
#define K_ALET		(1<<6)
#define K_CRIT		(1<<5)
#define K_ERR		(1<<4)
#define K_WARNIN	(1<<3)
#define K_NOTICE	(1<<2)
#define K_INFO		(1<<1)
#define K_DEBUG	(1<<0)

static u32 debug_level = 255;

#define fusb_printk(level, fmt, args...) do { \
		if (debug_level & level) { \
			pr_err("[USB_TYPEC]" fmt, ## args); \
		} \
	} while (0)


int trigger_driver(int type, int stat, int dir)
{
#ifdef CONFIG_MTK_SIB_USB_SWITCH
	if (g_typc_driver.sib_enable) {
		fusb_printk(K_INFO, "SIB enable!\n");
		goto end;
	}
#endif
	fusb_printk(K_DEBUG, "trigger_driver: type:%d, stat:%d, dir%d\n", type, stat, dir);

	if (type == HOST_TYPE) {
		g_typc_driver.conn_state = stat;
	}

	if (type == DEVICE_TYPE && g_typc_driver.device_driver) {
		if ((stat == DISABLE) && (g_typc_driver.device_driver->disable)
		    && (g_typc_driver.device_driver->on == ENABLE)) {
			g_typc_driver.device_driver->disable(g_typc_driver.device_driver->priv_data);
			g_typc_driver.device_driver->on = DISABLE;

			fusb_printk(K_INFO, "trigger_driver: disable dev drv\n");
		} else if ((stat == ENABLE) && (g_typc_driver.device_driver->enable)
			   && (g_typc_driver.device_driver->on == DISABLE)) {
			g_typc_driver.device_driver->enable(g_typc_driver.device_driver->priv_data);
			g_typc_driver.device_driver->on = ENABLE;

			fusb_printk(K_INFO, "trigger_driver: enable dev drv\n");
		} else {
			fusb_printk(K_INFO, "%s No device driver to enable\n", __func__);
		}
	} else if (type == HOST_TYPE && g_typc_driver.host_driver) {
		if ((stat == DISABLE) && (g_typc_driver.host_driver->disable)
		    && (g_typc_driver.host_driver->on == ENABLE)) {
			g_typc_driver.host_driver->disable(g_typc_driver.host_driver->priv_data);
			g_typc_driver.host_driver->on = DISABLE;

			fusb_printk(K_INFO, "trigger_driver: disable host drv\n");
		} else if ((stat == ENABLE) &&
			   (g_typc_driver.host_driver->enable) && (g_typc_driver.host_driver->on == DISABLE)) {
			g_typc_driver.host_driver->enable(g_typc_driver.host_driver->priv_data);
			g_typc_driver.host_driver->on = ENABLE;

			fusb_printk(K_INFO, "trigger_driver: enable host drv\n");
		} else {
			fusb_printk(K_INFO, "%s No device driver to enable\n", __func__);
		}
	} else {
		fusb_printk(K_INFO, "trigger_driver: no callback func\n");
	}
#ifdef CONFIG_MTK_SIB_USB_SWITCH
end:
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(trigger_driver);

/* /////////////////////////////////////////////////////////////////////////////// */
int register_typec_switch_callback(struct typec_switch_data *new_driver)
{
	fusb_printk(K_INFO, "Register driver %s %d\n", new_driver->name, new_driver->type);

	if (new_driver->type == DEVICE_TYPE) {
		g_typc_driver.device_driver = new_driver;
		g_typc_driver.device_driver->on = 0;
		return 0;
	}

	if (new_driver->type == HOST_TYPE) {
		g_typc_driver.host_driver = new_driver;
		g_typc_driver.host_driver->on = 0;
		if (g_typc_driver.conn_state  == ENABLE)
			trigger_driver(HOST_TYPE, ENABLE, DONT_CARE);
		return 0;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(register_typec_switch_callback);

int unregister_typec_switch_callback(struct typec_switch_data *new_driver)
{
	fusb_printk(K_INFO, "Unregister driver %s %d\n", new_driver->name, new_driver->type);

	if ((new_driver->type == DEVICE_TYPE) && (g_typc_driver.device_driver == new_driver))
		g_typc_driver.device_driver = NULL;

	if ((new_driver->type == HOST_TYPE) && (g_typc_driver.host_driver == new_driver))
		g_typc_driver.host_driver = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(unregister_typec_switch_callback);


#ifdef CONFIG_MTK_SIB_USB_SWITCH
static int usb_sib_get(void *data, u64 *val)
{
	struct usb_typc_driver *typec = data;

	*val = typec->sib_enable;

	fusb_printk(K_INFO, "usb_sib_get %d %llu\n", typec->sib_enable, *val);

	return 0;
}

static int usb_sib_set(void *data, u64 val)
{
	struct usb_typc_driver *typec = data;

	typec->sib_enable = !!val;

	fusb_printk(K_INFO, "usb_sib_set %d %llu\n", typec->sib_enable, val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(usb_sib_debugfs_fops, usb_sib_get, usb_sib_set, "%llu\n");
#endif

#ifdef CONFIG_U3_PHY_SMT_LOOP_BACK_SUPPORT
static int usb_smt_set(void *data, u64 val)
{
	int sel = val;

	fusb_printk(K_INFO, "usb_smt_set %d\n", sel);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(usb_smt_debugfs_fops, NULL, usb_smt_set, "%llu\n");
#endif

void usbc_enable_hw(int enable)
{
	if(enable){
		if (!IS_ERR(g_typc_driver.en_low))
			g_typc_driver.en_low = pinctrl_lookup_state(g_typc_driver.pinctrl, "gpio_en_low");//enable chip

	}else{
		if (!IS_ERR(g_typc_driver.en_high))
			g_typc_driver.en_high = pinctrl_lookup_state(g_typc_driver.pinctrl, "gpio_en_high");//disable chip
	}
}

EXPORT_SYMBOL_GPL(usbc_enable_hw);

static int usbc_pinctrl_probe(struct platform_device *pdev)
{
	int retval = 0;

	g_typc_driver.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(g_typc_driver.pinctrl)) {
		fusb_printk(K_ERR, "Cannot find usb pinctrl!\n");
	} else {
		fusb_printk(K_INFO, "pinctrl=%p\n", g_typc_driver.pinctrl);

		g_typc_driver.int_cfg = pinctrl_lookup_state(g_typc_driver.pinctrl, "eint_init");
		if (IS_ERR(g_typc_driver.int_cfg))
			fusb_printk(K_ERR, "Can *NOT* find eint_init\n");
		else
			fusb_printk(K_INFO, "Find eint_init\n");

		g_typc_driver.en_low = pinctrl_lookup_state(g_typc_driver.pinctrl, "gpio_en_low");
		if (IS_ERR(g_typc_driver.en_low))
			fusb_printk(K_ERR, "Can *NOT* find gpio_en_low\n");
		else
			fusb_printk(K_INFO, "Find gpio_en_low\n");

		g_typc_driver.en_high = pinctrl_lookup_state(g_typc_driver.pinctrl, "gpio_en_high");
		if (IS_ERR(g_typc_driver.en_high))
			fusb_printk(K_ERR, "Can *NOT* find gpio_en_high\n");
		else
			fusb_printk(K_INFO, "Find gpio_en_high\n");
		/********************************************************/

		/*init irq gpio*/
		pinctrl_select_state(g_typc_driver.pinctrl, g_typc_driver.int_cfg);
		/*default enable chip (en_low)*/
		pinctrl_select_state(g_typc_driver.pinctrl, g_typc_driver.en_low);
		fusb_printk(K_INFO, "Finish parsing pinctrl\n");
	}

	return retval;
}

static const struct of_device_id usbc_pinctrl_ids[] = {
	{.compatible = "mediatek,usb_c_pinctrl",},
	{},
};

static struct platform_driver usbc_pinctrl_driver = {
	.probe = usbc_pinctrl_probe,
	.driver = {
		.name = "usbc_pinctrl",
#ifdef CONFIG_OF
		.of_match_table = usbc_pinctrl_ids,
#endif
	},
};

static int __init usb_typec_init(void)
{
	struct dentry *root;

	if (!platform_driver_register(&usbc_pinctrl_driver))
		fusb_printk(K_INFO, "register usbc pinctrl succeed!!\n");
	else {
		fusb_printk(K_INFO, "register usbc pinctrl fail!!\n");
	}

	root = debugfs_create_dir("usb_c", NULL);
	if (!root) {
		return  -ENOMEM;
	}

#ifdef CONFIG_MTK_SIB_USB_SWITCH
	debugfs_create_file("sib", S_IRUGO|S_IWUSR, root, &g_typc_driver, &usb_sib_debugfs_fops);
#endif
#ifdef CONFIG_U3_PHY_SMT_LOOP_BACK_SUPPORT
	debugfs_create_file("smt", S_IWUSR, root, &g_typc_driver, &usb_smt_debugfs_fops);
#endif

	return 0;
}


fs_initcall(usb_typec_init);
