/*
 * pinctrl-regulator.c
 *
 * Copyright (c) 2016 Meizu Technology Co. Ltd.
 * WenBin Wu <wenbinwu@meizu.com>
 *
 * based on fixed.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This is useful for systems with mixed controllable and
 * non-controllable regulators, as well as for allowing testing on
 * systems with no controllable regulators.
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/pinctrl-regulator.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

struct pinctrl_regulator_data {
	struct pinctrl *pctrl;
	struct regulator_desc desc;
	struct regulator_dev *dev;

	int state;
};

typedef enum tagPINCTRL_STATE {
	PINCTRL_OUTPUT_ON = 0,
	PINCTRL_OUTPUT_OFF,
	PINCTRL_STATE_MAX,
} PINCTRL_STATE;

static const char *pinctrl_state_name[PINCTRL_STATE_MAX] = {
	"on_cfg",
	"off_cfg",
};

static long pinctrl_regulator_select_state(struct pinctrl_regulator_data *data, PINCTRL_STATE s)
{
	struct pinctrl_state *pState = 0;

	BUG_ON(!((unsigned int)(s) < (unsigned int)(PINCTRL_STATE_MAX)));

	pState = pinctrl_lookup_state(data->pctrl, pinctrl_state_name[s]);
	if (IS_ERR(pState)) {
		pr_err("lookup state '%s' failed\n", pinctrl_state_name[s]);
		return PTR_ERR(pState);
	}

	/* select state! */
	return (long)pinctrl_select_state(data->pctrl, pState);
}

static int pinctrl_regulator_enable(struct regulator_dev *dev)
{
	struct pinctrl_regulator_data *data = rdev_get_drvdata(dev);

	dev_err(&dev->dev, "enable\n");
	pinctrl_regulator_select_state(data, PINCTRL_OUTPUT_ON);
	data->state = PINCTRL_OUTPUT_ON;
	return 0;
}

static int pinctrl_regulator_disable(struct regulator_dev *dev)
{
	struct pinctrl_regulator_data *data = rdev_get_drvdata(dev);

	dev_err(&dev->dev, "disable\n");
	pinctrl_regulator_select_state(data, PINCTRL_OUTPUT_OFF);
	data->state = PINCTRL_OUTPUT_OFF;
	return 0;
}

static int pinctrl_regulator_is_enable(struct regulator_dev *dev)
{
	struct pinctrl_regulator_data *data = rdev_get_drvdata(dev);

	if (data->state == PINCTRL_OUTPUT_ON)
		return 1;

	return 0;
}

static struct regulator_ops pinctrl_regulator_voltage_ops = {
	.enable = pinctrl_regulator_enable,
	.disable = pinctrl_regulator_disable,
	.is_enabled = pinctrl_regulator_is_enable,
};

static struct pinctrl_regulator_config *
of_get_pinctrl_regulator_config(struct device *dev, struct device_node *np)
{
	struct pinctrl_regulator_config *config;

	config = devm_kzalloc(dev, sizeof(struct pinctrl_regulator_config), GFP_KERNEL);
	if (!config)
		return ERR_PTR(-ENOMEM);

	config->init_data = of_get_regulator_init_data(dev, np);
	if (!config->init_data)
		return ERR_PTR(-EINVAL);

	config->supply_name = config->init_data->constraints.name;

	if (config->init_data->constraints.boot_on)
		config->enabled_at_boot = 1;
	else
		config->enabled_at_boot = 0;

	config->microvolts = config->init_data->constraints.min_uV;
	config->startup_delay = config->init_data->constraints.enable_time;

	config->type = REGULATOR_VOLTAGE;

	return config;
}

static int pinctrl_regulator_probe(struct platform_device *pdev)
{
	struct pinctrl_regulator_config *config = dev_get_platdata(&pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl_regulator_data *drvdata;
	struct regulator_config cfg = { };
	int ret;
	struct pinctrl *pctrl;

	dev_err(&pdev->dev, "%s!", __func__);
	pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pctrl)) {
		dev_err(&pdev->dev, "Cannot find gpio regulator pinctrl!");
		return  PTR_ERR(pctrl);
	}
	
	if (np) {
		config = of_get_pinctrl_regulator_config(&pdev->dev, np);
		if (IS_ERR(config))
			return PTR_ERR(config);
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct pinctrl_regulator_data),
			       GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata-> pctrl = pctrl;
	drvdata->desc.name = kstrdup(config->supply_name, GFP_KERNEL);
	if (drvdata->desc.name == NULL) {
		dev_err(&pdev->dev, "Failed to allocate supply name\n");
		ret = -ENOMEM;
		goto err;
	}
	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.enable_time = config->startup_delay;
	drvdata->desc.type = REGULATOR_VOLTAGE;
	drvdata->desc.ops = &pinctrl_regulator_voltage_ops;
	drvdata->desc.n_voltages = 1;
	drvdata->desc.fixed_uV = config->microvolts;

	/* build initial state from gpio init data. */
	if(config->enabled_at_boot){
		pinctrl_regulator_select_state(drvdata, PINCTRL_OUTPUT_ON);
		drvdata->state = PINCTRL_OUTPUT_ON;
		dev_err(&pdev->dev, "enable\n");
	}else{
		pinctrl_regulator_select_state(drvdata, PINCTRL_OUTPUT_OFF);
		drvdata->state = PINCTRL_OUTPUT_OFF;
		dev_err(&pdev->dev, "disable\n");
	}


	cfg.dev = &pdev->dev;
	cfg.init_data = config->init_data;
	cfg.driver_data = drvdata;
	cfg.of_node = np;

	drvdata->dev = regulator_register(&drvdata->desc, &cfg);
	if (IS_ERR(drvdata->dev)) {
		ret = PTR_ERR(drvdata->dev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		goto err_reg;
	}

	platform_set_drvdata(pdev, drvdata);

	return 0;

err_reg:
	kfree(drvdata->desc.name);
err:
	return ret;
}

static int pinctrl_regulator_remove(struct platform_device *pdev)
{
	struct pinctrl_regulator_data *drvdata = platform_get_drvdata(pdev);

	regulator_unregister(drvdata->dev);

	kfree(drvdata->desc.name);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id pinctrl_regulator_of_match[] = {
	{ .compatible = "pinctrl-regulator", },
	{},
};
#endif

static struct platform_driver pinctrl_regulator_driver = {
	.probe		= pinctrl_regulator_probe,
	.remove		= pinctrl_regulator_remove,
	.driver		= {
		.name		= "regulator-pinctrl",
		.owner		= THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(pinctrl_regulator_of_match),
#endif
	},
};

static int __init pinctrl_regulator_init(void)
{
	return platform_driver_register(&pinctrl_regulator_driver);
}
subsys_initcall(pinctrl_regulator_init);

static void __exit pinctrl_regulator_exit(void)
{
	platform_driver_unregister(&pinctrl_regulator_driver);
}
module_exit(pinctrl_regulator_exit);

MODULE_AUTHOR("WenBin Wu@meizu.com>");
MODULE_DESCRIPTION("pinctrl voltage regulator");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pinctrl-regulator");
