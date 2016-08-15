/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_PINCTRL_MT6797
#include "disp_dts_gpio.h"
#include "disp_helper.h"
#include <linux/kernel.h> /* printk */
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

static struct pinctrl *this_pctrl; /* static pinctrl instance */

/* DTS state mapping name */
static const char *this_state_name[DTS_GPIO_STATE_MAX] = {
	"mode_te_gpio",
	"mode_te_te",
	"lcm_rst_out0_gpio",
	"lcm_rst_out1_gpio",
};

enum {
	TP_VDDI = 0,
	LCD_ENP,
	LCD_ENN,
	MAX_REGURATORS,
};

static const char *lcm_regulator_name[MAX_REGURATORS] = {
	"tpvddi",
	"lcd_enp",
	"lcd_enn",
};

static struct regulator *lcm_regulator[MAX_REGURATORS];

static long lcm_set_regulator_state(int index, int enable)
{
	int ret = -1;

	if(IS_ERR(lcm_regulator[index]))
		return -1;

	if(enable){
		ret = regulator_enable(lcm_regulator[index]);
	}else{
		ret = regulator_disable(lcm_regulator[index]);
	}
	return ret;
}

/* pinctrl implementation */
static long _set_state(const char *name)
{
	long ret = 0;
	struct pinctrl_state *pState = 0;

	BUG_ON(!this_pctrl);

	pState = pinctrl_lookup_state(this_pctrl, name);
	if (IS_ERR(pState)) {
		pr_err("lookup state '%s' failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	pinctrl_select_state(this_pctrl, pState);

exit:
	return ret; /* Good! */
}

long disp_dts_gpio_init(struct platform_device *pdev)
{
	int i=0;
	long ret = 0;
	struct pinctrl *pctrl;

	dev_err(&pdev->dev, "disp_dts_gpio_init!");
	/* retrieve */
	pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pctrl)) {
		dev_err(&pdev->dev, "Cannot find disp pinctrl!");
		ret = PTR_ERR(pctrl);
		goto exit;
	}

	for(i=0; i<MAX_REGURATORS; i++){
	 	lcm_regulator[i] = regulator_get(&pdev->dev, lcm_regulator_name[i]);
		if (IS_ERR(lcm_regulator[i])){
			dev_err(&pdev->dev, " Failed to get lcm regulators %s\n", lcm_regulator_name[i]);
		}else{
			regulator_enable(lcm_regulator[i]);//count +, becuse lcm initail in lk
			dev_err(&pdev->dev, "enable regulator %s!", lcm_regulator_name[i]);
		}
	}

	this_pctrl = pctrl;

exit:
	return ret;
}

long disp_dts_gpio_select_state(DTS_GPIO_STATE s)
{
	long ret = 0;

	BUG_ON(!((unsigned int)(s) < (unsigned int)(DTS_GPIO_STATE_MAX)));
	switch (s){
	case DTS_GPIO_STATE_LCD_BIAS_ENP0:
		ret = lcm_set_regulator_state(LCD_ENP, 0);
		break;
	case DTS_GPIO_STATE_LCD_BIAS_ENP1:
		ret = lcm_set_regulator_state(LCD_ENP, 1);
		break;
	case DTS_GPIO_STATE_LCD_DVDD1V8_ON:
		ret = lcm_set_regulator_state(TP_VDDI, 1);
		break;
	case DTS_GPIO_STATE_LCD_DVDD1V8_OFF:
		ret = lcm_set_regulator_state(TP_VDDI, 0);
		break;
	case DTS_GPIO_STATE_LCD_BIAS_ENN0:
		ret = lcm_set_regulator_state(LCD_ENN, 0);
		break;
	case DTS_GPIO_STATE_LCD_BIAS_ENN1:
		ret = lcm_set_regulator_state(LCD_ENN, 1);
		break;
	default:
		return _set_state(this_state_name[s]);
	}
	return ret;
}
#endif
