/*
 * RT5735 Fairchild Digitally Programmable TinyBuck Regulator Driver.
 *
 * Supported Part Numbers:
 * RT5735
 *
 * Copyright (c) 2016 Meizu Technology Co. Ltd.
 * WenBin Wu <wenbinwu@meizu.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/regulator/rt5735.h>

/* Voltage setting */
#define RT5735_ID	0x8810//VendoID&ProductID

/* IC Type */
#define RT5735_PRODUCTID	0x03
#define RT5735_REVISIONID	0x04
#define RT5735_FEATUREID	0x05
#define RT5735_VENDORID		0x06
/* Voltage setting */
#define RT5735_VSEL1		0x10
#define RT5735_VSEL0		0x11
/*status register*/
#define RT5735_PGOOD		0x12
#define RT5735_TIME			0x13
#define RT5735_COMMAND		0x14
#define RT5735_LIMCONF		0x16

#define RT5735_UP_SLEW_REG		RT5735_TIME
#define RT5735_UP_SLEW_MASK		0x7<<2
#define RT5735_UP_SLEW_SHIFT		0x2

#define RT5735_DOWN_SLEW_REG			RT5735_LIMCONF
#define RT5735_DOWN_SLEW_MASK		0x3<<1
#define RT5735_DOWN_SLEW_SHIFT		0x1

/* VSEL bit definitions */
#define VSEL_BUCK_EN	(1 << 7)
#define VSEL0_MODE		(1 << 7)
#define VSEL0_MODE_SHIFT		7
#define VSEL1_MODE		(1 << 6)
#define VSEL1_MODE_SHIFT		6
#define VSEL_NSEL_MASK	0x7F
/* Chip ID and Verison */
#define DIE_VID		0xFF	/* ID1 */
#define DIE_PID		0xFF	/* ID2 */
/* Control bit definitions */

/* Slew Control bit definitions */
#define CTL_SLEW_MASK		(0x7 << 2)
#define CTL_SLEW_SHIFT		2

#define RT5735_NVOLTAGES	128	/* 2^7 Numbers of voltages */

enum rt5735_vendor {
	RT5735_VENDOR_FAIRCHILD = 0,
};

/* IC Type */
enum {
	RT5735_CHIP_ID_00 = RT5735_ID,
};

struct rt5735_slew_map{
	int slew;
	int value;
};

struct rt5735_device_info {
	enum rt5735_vendor vendor;
	struct regmap *regmap;
	struct device *dev;
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	struct regulator_init_data *regulator;
	/* IC Type and Rev */
	int chip_id;
	int chip_rev;
	/* Voltage setting register */
	unsigned int vol_reg;
	unsigned int sleep_reg;
	/* Voltage range and step(linear) */
	unsigned int vsel_min;
	unsigned int vsel_step;
	/* Voltage slew rate limiting */
	unsigned int slew_rate;
	/* Sleep voltage cache */
	unsigned int sleep_vol_cache;

	unsigned int mode_reg;
	unsigned int mode_mask;
	unsigned int mode_shift;
};

static int rt5735_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	struct rt5735_device_info *di = rdev_get_drvdata(rdev);
	int ret;

	if (di->sleep_vol_cache == uV)
		return 0;
	ret = regulator_map_voltage_linear(rdev, uV, uV);
	if (ret < 0)
		return ret;
	ret = regmap_update_bits(di->regmap, di->sleep_reg,
					VSEL_NSEL_MASK, ret);
	if (ret < 0)
		return ret;
	/* Cache the sleep voltage setting.
	 * Might not be the real voltage which is rounded */
	di->sleep_vol_cache = uV;

	return 0;
}

static int rt5735_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct rt5735_device_info *di = rdev_get_drvdata(rdev);

	switch (mode) {
	case REGULATOR_MODE_FAST:
		regmap_update_bits(di->regmap, di->mode_reg, di->mode_mask, 1 << di->mode_shift);
		dev_info(di->dev, "set mode REGULATOR_MODE_FAST\n");
		break;
	case REGULATOR_MODE_NORMAL:
		regmap_update_bits(di->regmap, di->mode_reg, di->mode_mask, 0 << di->mode_shift);
		dev_info(di->dev, "set mode REGULATOR_MODE_NORMAL\n");
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned int rt5735_get_mode(struct regulator_dev *rdev)
{
	struct rt5735_device_info *di = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret = 0;

	ret = regmap_read(di->regmap, di->mode_reg, &val);
	if (ret < 0)
		return ret;

	if (val & di->mode_mask)
		return REGULATOR_MODE_FAST;
	else
		return REGULATOR_MODE_NORMAL;
}

static struct rt5735_slew_map rt5735_upslew_rates[] = {
	{4000, 4},
	{8000, 3},
	{16000, 1},
	{32000, 2},
	{64000, 0},
};

static struct rt5735_slew_map rt5735_downslew_rates[] = {
	{4000, 1},
	{8000, 2},
	{16000, 3},
	{32000, 0},
};

static int rt5735_set_ramp(struct regulator_dev *rdev, int ramp)
{
	struct rt5735_device_info *di = rdev_get_drvdata(rdev);
	int up_regval = -1, i;
	int down_regval = -1, ret = -1;

	for (i = 0; i < ARRAY_SIZE(rt5735_upslew_rates); i++) {
		if (ramp >= rt5735_upslew_rates[i].slew)
			up_regval = rt5735_upslew_rates[i].value;
		else
			break;
	}
	if (up_regval < 0) {
		dev_err(di->dev, "unsupported up ramp value %d\n", ramp);
		return -EINVAL;
	}
	ret = regmap_update_bits(di->regmap, RT5735_UP_SLEW_REG,
				  RT5735_UP_SLEW_MASK, up_regval << RT5735_UP_SLEW_SHIFT);

	dev_info(di->dev, "set ramp value %d,  up = %d\n", ramp, up_regval);

	for (i = 0; i < ARRAY_SIZE(rt5735_downslew_rates); i++) {
		if (ramp >= rt5735_downslew_rates[i].slew)
			down_regval = rt5735_downslew_rates[i].value;
		else
			break;
	}
	if (down_regval < 0) {
		dev_err(di->dev, "unsupported down ramp value %d\n", ramp);
		return -EINVAL;
	}

	ret |= regmap_update_bits(di->regmap, RT5735_DOWN_SLEW_REG,
				  RT5735_DOWN_SLEW_MASK, down_regval << RT5735_DOWN_SLEW_SHIFT);

	dev_info(di->dev, "set ramp value %d,  down =%d\n", ramp, down_regval);
	return ret;
}

static struct regulator_ops rt5735_regulator_ops = {
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.map_voltage = regulator_map_voltage_linear,
	.list_voltage = regulator_list_voltage_linear,
	.set_suspend_voltage = rt5735_set_suspend_voltage,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_mode = rt5735_set_mode,
	.get_mode = rt5735_get_mode,
	.set_ramp_delay = rt5735_set_ramp,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static int rt5735_voltages_setup(struct rt5735_device_info *di)
{
	/* Init voltage range and step */
	switch (di->chip_id) {
	case RT5735_CHIP_ID_00:
		di->vsel_min = 600000;
		di->vsel_step = 6250;
		break;
	default:
		dev_err(di->dev,
			"Chip ID %d not supported!\n", di->chip_id);
		return -EINVAL;
	}

	return 0;
}

/* For 00,01,03,05 options:
 * For FAN53200:
 * VOUT = 0.600V + NSELx * 6.25mV, from 0.600 to 1.3875V.
 * */
static int rt5735_device_setup(struct rt5735_device_info *di,
				struct rt5735_platform_data *pdata)
{
	int ret = 0;

	/* Setup voltage control register */
	switch (pdata->sleep_vsel_id) {
	case RT5735_VSEL_ID_0:
		di->sleep_reg = RT5735_VSEL0;
		di->vol_reg = RT5735_VSEL1;
		di->mode_reg = RT5735_COMMAND;
		di->mode_mask = VSEL1_MODE;
		di->mode_shift = VSEL1_MODE_SHIFT;
		break;
	case RT5735_VSEL_ID_1:
		di->sleep_reg = RT5735_VSEL1;
		di->vol_reg = RT5735_VSEL0;
		di->mode_reg = RT5735_COMMAND;
		di->mode_mask = VSEL0_MODE;
		di->mode_shift = VSEL0_MODE_SHIFT;
		break;
	default:
		dev_err(di->dev, "Invalid VSEL ID!\n");
		return -EINVAL;
	}

	ret = rt5735_voltages_setup(di);

	return ret;
}

static int rt5735_regulator_register(struct rt5735_device_info *di,
			struct regulator_config *config)
{
	struct regulator_desc *rdesc = &di->desc;

	rdesc->name = "rt5735-reg";
	rdesc->supply_name = "vin";
	rdesc->ops = &rt5735_regulator_ops;
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->n_voltages = RT5735_NVOLTAGES;
	rdesc->enable_reg = di->vol_reg;
	rdesc->enable_mask = VSEL_BUCK_EN;
	rdesc->min_uV = di->vsel_min;
	rdesc->uV_step = di->vsel_step;
	rdesc->vsel_reg = di->vol_reg;
	rdesc->vsel_mask = VSEL_NSEL_MASK;
	rdesc->owner = THIS_MODULE;

	di->rdev = devm_regulator_register(di->dev, &di->desc, config);
	return PTR_ERR_OR_ZERO(di->rdev);
}

static struct regmap_config rt5735_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static struct rt5735_platform_data *rt5735_alloc_platform_data(struct device *dev,
							struct device_node *np)
{
	struct rt5735_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->regulator = of_get_regulator_init_data(dev, np);

	pdata->regulator->constraints.valid_modes_mask |= REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST;
	pdata->regulator->constraints.valid_ops_mask |= REGULATOR_CHANGE_MODE;

	pdata->sleep_vsel_id = RT5735_VSEL_ID_1; // VSEL1 no used, set as sleep VSEL

	return pdata;
}

static const struct of_device_id rt5735_dt_ids[] = {
	{
		.compatible = "mediatek, rt5735",
		.data = (void *)RT5735_VENDOR_FAIRCHILD
	},
	{ }
};
MODULE_DEVICE_TABLE(of, rt5735_dt_ids);

static int rt5735_regulator_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct rt5735_device_info *di;
	struct rt5735_platform_data *pdata;
	struct regulator_config config = { };
	unsigned int val, vid, pid;
	int ret;

	dev_info(&client->dev, "rt5735_regulator_probe!\n");

	pdata = rt5735_alloc_platform_data(&client->dev, np);

	if (!pdata || !pdata->regulator) {
		dev_err(&client->dev, "Platform data not found!\n");
		return -ENOMEM;
	}

	di = devm_kzalloc(&client->dev, sizeof(struct rt5735_device_info),
					GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->regulator = pdata->regulator;

	di->regmap = devm_regmap_init_i2c(client, &rt5735_regmap_config);
	if (IS_ERR(di->regmap)) {
		dev_err(&client->dev, "Failed to allocate regmap!\n");
		return PTR_ERR(di->regmap);
	}
	di->dev = &client->dev;
	i2c_set_clientdata(client, di);
	/* Get chip ID */
	ret = regmap_read(di->regmap, RT5735_VENDORID, &val);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to get chip ID!\n");
		return ret;
	}
	vid = val & DIE_VID;
	/* Get chip revision */
	ret = regmap_read(di->regmap, RT5735_PRODUCTID, &val);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to get chip Rev!\n");
		return ret;
	}
	pid = val & DIE_PID;

	dev_info(&client->dev, "RT5735 Vendor [%d] Product [%d] Detected!\n",
				vid, pid);

	di->chip_id = vid << 8 | pid;

	/* Device init */
	ret = rt5735_device_setup(di, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to setup device!\n");
		return ret;
	}
	/* Register regulator */
	config.dev = di->dev;
	config.init_data = di->regulator;
	config.regmap = di->regmap;
	config.driver_data = di;
	config.of_node = np;

	ret = rt5735_regulator_register(di, &config);
	if (ret < 0)
		dev_err(&client->dev, "Failed to register regulator!\n");
	return ret;

}

static const struct i2c_device_id rt5735_id[] = {
	{
		.name = "rt5735",
		.driver_data = RT5735_VENDOR_FAIRCHILD
	},
	{ },
};

static struct i2c_driver rt5735_regulator_driver = {
	.driver = {
		.name = "rt5735-regulator",
		.of_match_table = of_match_ptr(rt5735_dt_ids),
	},
	.probe = rt5735_regulator_probe,
	.id_table = rt5735_id,
};

module_i2c_driver(rt5735_regulator_driver);

MODULE_AUTHOR("WenBin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("RT5735 regulator driver");
MODULE_LICENSE("GPL v2");
