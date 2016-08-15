/*
 * cs43l36.h -- CS43L36 ALSA SoC audio driver
 *
 * Copyright 2015 CirrusLogic, Inc.
 *
 * Author: Brian Austin <brian.austin@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <dt-bindings/sound/cs43l36.h>

#include "cs43l36.h"

#define CS43L36_DEBUG(format, args...) pr_debug(format, ##args)
#define CS43L36_INFO(format, args...) pr_info(format, ##args)
#define CS43L36_ERROR(format, args...) pr_err(format, ##args)

#define CHECK_OK(func)			\
		{ 						\
			int ret_val; 		\
			ret_val = func; 	\
			if (ret_val != 0) { \
				pr_err("%s: Line(%d) return %d.\n", __func__, __LINE__, ret_val); \
			} 					\
		}

extern int meizu_sysfslink_register_name(struct device *dev, char *name);

#undef CS43L36_STANDBY_DISABLE_REGULATOR

static u32 impedance = 30;

static struct i2c_client *new_client = NULL;
ssize_t static cs43l36_read_register(u8 addr, char *returnData)
{
	int 	ret=0;
	char	cmd_buf[1]={0x00};
	//char	readData[2] = {0};
	char	readData = 0;
	//u16  value;

	if(!new_client)	{
		CS43L36_ERROR("I2C client not initialized!!\n");
		return -1;
	}

	cmd_buf[0] = addr;
	ret = i2c_master_send(new_client, &cmd_buf[0], 1);
	if (ret < 0) {
		CS43L36_ERROR("read sends command error!! addr=0x%x, ret=%d\n", cmd_buf[0], ret);
		return -1;
	}
	ret = i2c_master_recv(new_client, &readData, 1);
	if (ret < 0) {
		CS43L36_ERROR("%s: reads recv data error!!\n", __func__);
		return -1;
	} else{
		CS43L36_DEBUG("cs43l36 read reg:0x%x, readData:0x%x\n", addr, readData);
	}
	*returnData = readData;

	return ret;
}

static ssize_t cs43l36_write_page(u8 page)
{
	char   write_data[2] = {0};
	int    ret=-1;

	if(!new_client){
		CS43L36_ERROR("I2C client not initialized!!\n");
		return -1;
	}

	write_data[0] = 0x00;// page addr
	write_data[1] = (char)(page);
	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		CS43L36_ERROR("write sends command error!! ret=%d\n", ret);
		return ret;
	}

	CS43L36_DEBUG("cs43l36 write page:0x%x, ret=%d\n", page, ret);
	return 0;
}

static ssize_t cs43l36_write_data(u8 reg, u8 data)
{
	char   write_data[2] = {0};
	int    ret=-1;

	if(!new_client){
		CS43L36_ERROR("I2C client not initialized!!\n");
		return -1;
	}

	write_data[0] = (char)reg;
	write_data[1] = (char)(data);
	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		CS43L36_ERROR("write sends command error!!\n");
		return -1;
	}

	CS43L36_DEBUG("cs43l36 write reg:0x%x data:0x%x, ret=%d\n", reg, data, ret);
	return 0;
}

static int cs43l36_read_i2c_register(u16 addr, u8 *returnData)
{
	int ret = -1;

	ret = cs43l36_write_page(addr>>8);
	if (ret < 0) {
		return ret;
	}

	ret = cs43l36_read_register(addr&0xff, (char *)returnData);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int cs43l36_write_i2c_register(u16 addr, u8 writedata)
{
	int ret = -1;

	ret = cs43l36_write_page(addr>>8);
	if (ret < 0) {
		return ret;
	}

	ret = cs43l36_write_data(addr&0xff, (u8)writedata);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

#if 0
static int cs43l36_update_bits(u16 addr, unsigned int mask, unsigned int val)
{
	int ret;
	u8 tmp, orig;

	ret = cs43l36_read_i2c_register(addr, &orig);
	if (ret != 0)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig) {
		ret = cs43l36_write_i2c_register(addr, tmp);
	} else {
		CS43L36_DEBUG("%s: update bits is equal.\n", __func__);
	}

	return ret;
}
#endif

static int cs43l36_regulator_enable(struct cs43l36_private *cs43l36)
{
	if (IS_ERR(cs43l36->pctrl)) {
		return -1;
	}

	if (!IS_ERR(cs43l36->state_vio_en)) {
		pinctrl_select_state(cs43l36->pctrl, cs43l36->state_vio_en);
	}
	return 0;
}

#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
static int cs43l36_regulator_disable(struct cs43l36_private *cs43l36)
{
	if (IS_ERR(cs43l36->pctrl)) {
		return -1;
	}

	if (!IS_ERR(cs43l36->state_vio_dis)) {
		pinctrl_select_state(cs43l36->pctrl, cs43l36->state_vio_dis);
	}
	return 0;
}
#endif

static u16 th2_new = 0;
static int cs43l36_load_threshold_cal(void)
{
	u8 th1_b1, th1_b0, th2_b1, th2_b0, cap_th1_b1, cap_th1_b0;
	u16 th1, th2, cap_th1;

	cs43l36_write_i2c_register(0x107E, 0x99);
	cs43l36_write_i2c_register(0x1911, 0xC0);

	cs43l36_read_i2c_register(0x1917, &th2_b1);
	cs43l36_read_i2c_register(0x1918, &th2_b0);

	cs43l36_read_i2c_register(0x1919, &th1_b1);
	cs43l36_read_i2c_register(0x191A, &th1_b0);

	cs43l36_read_i2c_register(0x191B, &cap_th1_b1);
	cs43l36_read_i2c_register(0x191C, &cap_th1_b0);

	th1 = th1_b0 + th1_b1*256;
	th2 = th2_b0 + th2_b1*256;
	cap_th1 = cap_th1_b0 + cap_th1_b1*256;

	//116--0x4D
	th2_new = 0x4D + (th2 - 116);

	pr_info("%s: th1=%d, th2=%d, cap_th1=%d, th2_new=%d\n", __func__, th1, th2, cap_th1, th2_new);

	cs43l36_write_i2c_register(0x1917, (th2_new & 0xFF00) >> 8);
	cs43l36_write_i2c_register(0x1918, th2_new & 0x00FF);

	cs43l36_write_i2c_register(0x107E, 0x00);

	return 0;
}

static int cs43l36_reset(struct cs43l36_private *cs43l36)
{
	if (IS_ERR(cs43l36->pctrl)
		|| IS_ERR(cs43l36->state_rst_l) || IS_ERR(cs43l36->state_rst_h)) {
		return -1;
	}

	pinctrl_select_state(cs43l36->pctrl, cs43l36->state_rst_l);
	msleep(10);
	pinctrl_select_state(cs43l36->pctrl, cs43l36->state_rst_h);
	msleep(2);

	cs43l36_load_threshold_cal();
	return 0;
}

int cs43l36_dts_gpio_init(struct i2c_client *client)
{
	struct cs43l36_private *cs43l36 = i2c_get_clientdata(client);
	int ret = 0;

	cs43l36->pctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(cs43l36->pctrl)) {
		dev_err(&client->dev, "Cannot find hifi pinctrl!");
		return PTR_ERR(cs43l36->pctrl);
	}

	cs43l36->state_rst_h = pinctrl_lookup_state(cs43l36->pctrl, "hifi_rst1");
	if (IS_ERR(cs43l36->state_rst_h)) {
		ret = PTR_ERR(cs43l36->state_rst_h);
		CS43L36_ERROR("%s : pinctrl err, state_rst_h\n", __func__);
	}

	cs43l36->state_rst_l = pinctrl_lookup_state(cs43l36->pctrl, "hifi_rst0");
	if (IS_ERR(cs43l36->state_rst_l)) {
		ret = PTR_ERR(cs43l36->state_rst_l);
		CS43L36_ERROR("%s : pinctrl err, state_rst_l\n", __func__);
	}

	cs43l36->state_vio_en = pinctrl_lookup_state(cs43l36->pctrl, "hifi_vio_en");
	if (IS_ERR(cs43l36->state_vio_en)) {
		ret = PTR_ERR(cs43l36->state_vio_en);
		CS43L36_ERROR("%s : pinctrl err, state_vio_en\n", __func__);
	}

	cs43l36->state_vio_dis = pinctrl_lookup_state(cs43l36->pctrl, "hifi_vio_dis");
	if (IS_ERR(cs43l36->state_vio_dis)) {
		ret = PTR_ERR(cs43l36->state_vio_dis);
		CS43L36_ERROR("%s : pinctrl err, state_vio_dis\n", __func__);
	}

	cs43l36->state_irq = pinctrl_lookup_state(cs43l36->pctrl, "hifi_eint");
	if (IS_ERR(cs43l36->state_irq)) {
		ret = PTR_ERR(cs43l36->state_irq);
		CS43L36_ERROR("%s : pinctrl err, state_irq\n", __func__);
	}

	CS43L36_DEBUG("%s exit.\n", __func__);

	return 0;
}

static const struct reg_default cs43l36_reg_defaults[] = {
	{ CS43L36_FRZ_CTL,			0x00 },
	{ CS43L36_SRC_CTL,			0x08 },
	{ CS43L36_MCLK_CTL,			0x02 },
	{ CS43L36_SFTRAMP_RATE,			0xA4 },
	{ CS43L36_I2C_DEBOUNCE,			0x88 },
	{ CS43L36_I2C_STRETCH,			0x03 },
	{ CS43L36_I2C_TIMEOUT,			0xB7 },
	{ CS43L36_PWR_CTL1,			0xFF },
	{ CS43L36_PWR_CTL2,			0x84 },
	{ CS43L36_PWR_CTL3,			0x10 },
	{ CS43L36_OSC_SWITCH,			0x00 },
	{ CS43L36_OSC_SWITCH_STATUS,		0x05 },
	{ CS43L36_TSENSE_CTL,			0x1B },
	{ CS43L36_MCLK_SRC_SEL,			0x00 },
	{ CS43L36_FSYNC_PW_LOWER,		0x00 },
	{ CS43L36_FSYNC_PW_UPPER,		0x00 },
	{ CS43L36_FSYNC_P_LOWER,		0xF9 },
	{ CS43L36_FSYNC_P_UPPER,		0x00 },
	{ CS43L36_ASP_CLK_CFG,			0x00 },
	{ CS43L36_ASP_FRM_CFG,			0x08 },
	{ CS43L36_FS_RATE_EN,			0x00 },
	{ CS43L36_IN_ASRC_CLK,			0x00 },
	{ CS43L36_PLL_DIV_CFG1,			0x00 },
	{ CS43L36_SRC_MASK,			0x0F },
	{ CS43L36_ASP_RX_MASK,			0x1F },
	{ CS43L36_DAC_INT_MASK,			0x03 },
	{ CS43L36_DAC_LOCK_MASK,		0XFF },
	{ CS43L36_VPMON_MASK,			0x01 },
	{ CS43L36_PLL_LOCK_MASK,		0x01 },
	{ CS43L36_TS_PLUG_MASK,			0x0F },
	{ CS43L36_PLL_CTL1,			0x00 },
	{ CS43L36_PLL_DIV_FRAC0,		0x00 },
	{ CS43L36_PLL_DIV_FRAC1,		0x00 },
	{ CS43L36_PLL_DIV_FRAC2,		0x00 },
	{ CS43L36_PLL_DIV_INT,			0x40 },
	{ CS43L36_PLL_CTL3,			0x10 },
	{ CS43L36_PLL_CAL_RATIO,		0x80 },
	{ CS43L36_PLL_CTL4,			0x03 },
	{ CS43L36_LOAD_DET_RCSTAT,		0x00 },
	{ CS43L36_LOAD_DET_DONE,		0x00 },
	{ CS43L36_LOAD_DET_EN,			0x00 },
	{ CS43L36_WAKE_CTL,			0xC0 },
	{ CS43L36_TIPSENSE_CTL,			0x02 },
	{ CS43L36_MIC_DET_CTL,			0x1F },
	{ CS43L36_DET_STATUS,			0xE0 },
	{ CS43L36_DET_INT_MASK,			0xFF },
	{ CS43L36_DAC_CTL1,			0x00 },
	{ CS43L36_DAC_CTL2,			0x02 },
	{ CS43L36_HP_CTL,			0x0D },
	{ CS43L36_CLASSH_CTL,			0x07 },
	{ CS43L36_MIXER_CHA_VOL,		0x3F },
	{ CS43L36_MIXER_CHB_VOL,		0x3F },
	{ CS43L36_SP_RX_CH_SEL,			0x04 },
	{ CS43L36_SP_RX_ISOC_CTL,		0x04 },
	{ CS43L36_SP_RX_FS,			0x0C },
	{ CS43L36_SRC_SDIN_FS,			0x40 },
	{ CS43L36_ASP_RX_DAI0_EN,		0x00 },
	{ CS43L36_ASP_RX_DAI0_CH1_PH_RES,	0x03 },
	{ CS43L36_ASP_RX_DAI0_CH1_BIT_MSB,	0x00 },
	{ CS43L36_ASP_RX_DAI0_CH1_BIT_LSB,	0x00 },
	{ CS43L36_ASP_RX_DAI0_CH2_PH_RES,	0x03 },
	{ CS43L36_ASP_RX_DAI0_CH2_BIT_MSB,	0x00 },
	{ CS43L36_ASP_RX_DAI0_CH2_BIT_LSB,	0x00 },
};

static bool cs43l36_readable_register(struct device *dev, unsigned int reg)
{
#if 0
	switch (reg) {
	case CS43L36_PAGE_REGISTER:
	case CS43L36_DEVID_AB:
	case CS43L36_DEVID_CD:
	case CS43L36_DEVID_E:
	case CS43L36_REVID:
	case CS43L36_FRZ_CTL:
	case CS43L36_SRC_CTL:
	case CS43L36_MCLK_STAT:
	case CS43L36_MCLK_CTL:
	case CS43L36_SFTRAMP_RATE:
	case CS43L36_I2C_DEBOUNCE:
	case CS43L36_I2C_STRETCH:
	case CS43L36_I2C_TIMEOUT:
	case CS43L36_PWR_CTL1:
	case CS43L36_PWR_CTL2:
	case CS43L36_PWR_CTL3:
	case CS43L36_OSC_SWITCH:
	case CS43L36_OSC_SWITCH_STATUS:
	case CS43L36_TSENSE_CTL:
	case CS43L36_MCLK_SRC_SEL:
	case CS43L36_FSYNC_PW_LOWER:
	case CS43L36_FSYNC_PW_UPPER:
	case CS43L36_FSYNC_P_LOWER:
	case CS43L36_FSYNC_P_UPPER:
	case CS43L36_ASP_CLK_CFG:
	case CS43L36_ASP_FRM_CFG:
	case CS43L36_FS_RATE_EN:
	case CS43L36_IN_ASRC_CLK:
	case CS43L36_PLL_DIV_CFG1:
	case CS43L36_SRC_STATUS:
	case CS43L36_ASP_RX_STATUS:
	case CS43L36_DAC_INT_STATUS:
	case CS43L36_DETECT_STATUS1:
	case CS43L36_DAC_LOCK_STATUS:
	case CS43L36_VPMON_STATUS:
	case CS43L36_PLL_LOCK:
	case CS43L36_TS_PLUG_STATUS:
	case CS43L36_SRC_MASK:
	case CS43L36_ASP_RX_MASK:
	case CS43L36_DAC_INT_MASK:
	case CS43L36_DAC_LOCK_MASK:
	case CS43L36_VPMON_MASK:
	case CS43L36_PLL_LOCK_MASK:
	case CS43L36_TS_PLUG_MASK:
	case CS43L36_PLL_CTL1:
	case CS43L36_PLL_DIV_FRAC0:
	case CS43L36_PLL_DIV_FRAC1:
	case CS43L36_PLL_DIV_FRAC2:
	case CS43L36_PLL_DIV_INT:
	case CS43L36_PLL_CTL3:
	case CS43L36_PLL_CAL_RATIO:
	case CS43L36_PLL_CTL4:
	case CS43L36_LOAD_DET_RCSTAT:
	case CS43L36_LOAD_DET_DONE:
	case CS43L36_LOAD_DET_EN:
	case CS43L36_WAKE_CTL:
	case CS43L36_TIPSENSE_CTL:
	case CS43L36_MIC_DET_CTL:
	case CS43L36_DET_STATUS:
	case CS43L36_DET_INT_MASK:
	case CS43L36_DAC_CTL1:
	case CS43L36_DAC_CTL2:
	case CS43L36_HP_CTL:
	case CS43L36_CLASSH_CTL:
	case CS43L36_MIXER_CHA_VOL:
	case CS43L36_MIXER_CHB_VOL:
	case CS43L36_SP_RX_CH_SEL:
	case CS43L36_SP_RX_ISOC_CTL:
	case CS43L36_SP_RX_FS:
	case CS43L36_SRC_SDIN_FS:
	case CS43L36_ASP_RX_DAI0_EN:
	case CS43L36_ASP_RX_DAI0_CH1_PH_RES:
	case CS43L36_ASP_RX_DAI0_CH1_BIT_MSB:
	case CS43L36_ASP_RX_DAI0_CH1_BIT_LSB:
	case CS43L36_ASP_RX_DAI0_CH2_PH_RES:
	case CS43L36_ASP_RX_DAI0_CH2_BIT_MSB:
	case CS43L36_ASP_RX_DAI0_CH2_BIT_LSB:
	case CS43L36_SUB_REVID:
		return true;
	default:
		return false;
	}
#endif
	/* enable read access for all registers */
	return 1;

}

static bool cs43l36_volatile_register(struct device *dev, unsigned int reg)
{
#if 0
	switch (reg) {
	case CS43L36_CH_OVFL_STATUS:
	case CS43L36_SRC_STATUS:
	case CS43L36_ASP_RX_STATUS:
	case CS43L36_DAC_INT_STATUS:
	case CS43L36_DETECT_STATUS1:
	case CS43L36_DAC_LOCK_STATUS:
	case CS43L36_VPMON_STATUS:
	case CS43L36_PLL_LOCK:
	case CS43L36_TS_PLUG_STATUS:
	case CS43L36_DET_STATUS:
	case CS43L36_MCLK_CTL:
	case CS43L36_LOAD_DET_DONE:
	case CS43L36_LOAD_DET_RCSTAT:
		return true;
	default:
		return false;
	}
#endif
	/* enable volatile access for all registers */
	return 1;
}

static const struct regmap_range_cfg cs43l36_page_range = {
	.name = "Pages", .range_min = 0,
	.range_max = CS43L36_MAX_REGISTER,
	.selector_reg = CS43L36_PAGE_REGISTER,
	.selector_mask = 0xff,
	.selector_shift = 0,
	.window_start = 0, .window_len = 256,
};

const struct regmap_config cs43l36_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.readable_reg = cs43l36_readable_register,
	.volatile_reg = cs43l36_volatile_register,

	.ranges = &cs43l36_page_range,
	.num_ranges = 1,

	.max_register = CS43L36_MAX_REGISTER,
	.reg_defaults = cs43l36_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(cs43l36_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

struct cs43l36_pll_params {
	u32 sclk;
	u8 mclk_src_sel;
	u8 sclk_prediv;
	u8 pll_div_int;
	u32 pll_div_frac;
	u8 pll_mode;
	u8 pll_divout;
	u32 mclk_int;
	u8 pll_cal_ratio;
};

/*
 * Common PLL Settings for given SCLK
 * Table 4-5 from the Datasheet
 *
 * If mclk_src_sel = 0 PLL is bypassed
 */
static const struct cs43l36_pll_params pll_ratio_table[] = {
	//16K
	{ 1024000, 1, 0x00, 0xAC, 0x440000, 0x01, 0x10, 11289600, 118 },
	{ 1024000, 1, 0x00, 0xBB, 0x800000, 0x03, 0x10, 12000000, 125 },
	{ 1024000, 1, 0x00, 0xC0, 0x000000, 0x03, 0x10, 12288000, 127 },//128

	//24K
	{ 1536000, 1, 0x00, 0x72, 0xD80000, 0x01, 0x10, 11289600, 118 },
	{ 1536000, 1, 0x00, 0x7D, 0x000000, 0x03, 0x10, 12000000, 125 },
	{ 1536000, 1, 0x00, 0x80, 0x000000, 0x03, 0x10, 12288000, 127 },//128
	{ 1536000, 1, 0x00, 0x7D, 0x000000, 0x03, 0x08, 24000000, 125 },
	{ 1536000, 1, 0x00, 0x80, 0x000000, 0x03, 0x08, 24576000, 127 },

	//32K
	{ 2048000, 1, 0x00, 0x56, 0x220000, 0x01, 0x10, 11289600, 88 },
	{ 2048000, 1, 0x00, 0x5D, 0xC00000, 0x03, 0x10, 12000000, 94 },
	{ 2048000, 1, 0x00, 0x60, 0x000000, 0x03, 0x10, 12288000, 95 },//96

	//44.1K
	{ 2822400, 1, 0x00, 0x40, 0x000000, 0x03, 0x10, 11289600, 127 },//128
	{ 2822400, 1, 0x00, 0x40, 0x000000, 0x03, 0x08, 22579200, 127 },//128

	{ 3000000, 1, 0x00, 0x3C, 0x361134, 0x03, 0x10, 11289600, 120 },
	{ 3000000, 1, 0x00, 0x40, 0x000000, 0x03, 0x10, 12000000, 128 },
	{ 3000000, 1, 0x00, 0x40, 0x000000, 0x01, 0x10, 12288000, 131 },
	{ 3000000, 1, 0x00, 0x40, 0x000000, 0x03, 0x08, 24000000, 255 },//128
	{ 3000000, 1, 0x00, 0x40, 0x000000, 0x01, 0x08, 24576000, 131 },

	//48K
	{ 3072000, 1, 0x00, 0x39, 0x6C0000, 0x01, 0x10, 11289600, 118 },
	{ 3072000, 1, 0x00, 0x3E, 0x800000, 0x03, 0x10, 12000000, 125 },
	{ 3072000, 1, 0x00, 0x40, 0x000000, 0x03, 0x10, 12288000, 127 },//128
	{ 3072000, 1, 0x00, 0x3E, 0x800000, 0x03, 0x08, 24000000, 125 },//125
	{ 3072000, 1, 0x00, 0x40, 0x000000, 0x03, 0x08, 24576000, 127 },//128

	{ 4000000, 1, 0x00, 0x30, 0x800000, 0x03, 0x10, 12000000, 96 },
	{ 4000000, 1, 0x00, 0x30, 0x000000, 0x01, 0x10, 12288000, 98 },

	//64K
	{ 4096000, 1, 0x00, 0x2B, 0x110000, 0x01, 0x10, 11289600, 88 },
	{ 4096000, 1, 0x00, 0x2E, 0xE00000, 0x03, 0x10, 12000000, 94 },
	{ 4096000, 1, 0x00, 0x30, 0x000000, 0x03, 0x10, 12288000, 95 },//96

	//88.2K
	{ 5644800, 1, 0x01, 0x40, 0x000000, 0x03, 0x10, 11289600, 127 },//128
	{ 5644800, 1, 0x01, 0x40, 0x000000, 0x03, 0x08, 22579200, 127 },

	{ 6000000, 1, 0x01, 0x40, 0x000000, 0x03, 0x10, 12000000, 128 },
	{ 6000000, 1, 0x01, 0x40, 0x000000, 0x01, 0x10, 12288000, 131 },
	{ 6000000, 1, 0x01, 0x40, 0x000000, 0x03, 0x08, 24000000, 255 },
	{ 6000000, 1, 0x01, 0x40, 0x000000, 0x01, 0x08, 24576000, 131 },

	//96K
	{ 6144000, 1, 0x01, 0x3E, 0x800000, 0x03, 0x10, 12000000, 125 },
	{ 6144000, 1, 0x01, 0x40, 0x000000, 0x03, 0x10, 12288000, 127 },//128
	{ 6144000, 1, 0x01, 0x3E, 0x800000, 0x03, 0x08, 24000000, 125 },
	{ 6144000, 1, 0x01, 0x40, 0x000000, 0x03, 0x08, 24576000, 127 },

	//176.4K
	{ 11289600, 0, 0, 0, 0, 0, 0, 11289600, 0 },
	{ 11289600, 1, 0x02, 0x40, 0x000000, 0x03, 0x08, 22579200, 127 },//255

	{ 12000000, 0, 0, 0, 0, 0, 0, 12000000, 0 },
	{ 12000000, 1, 0x02, 0x40, 0x000000, 0x01, 0x10, 12288000, 131 },
	{ 12000000, 1, 0x02, 0x40, 0x000000, 0x03, 0x08, 24000000, 255 },
	{ 12000000, 1, 0x02, 0x40, 0x000000, 0x01, 0x08, 24576000, 131 },

	//192K
	{ 12288000, 1, 0x02, 0x3E, 0x800000, 0x03, 0x10, 12000000, 125 },
	{ 12288000, 0, 0, 0, 0, 0, 0, 12288000, 0 },
	{ 12288000, 1, 0x01, 0x3E, 0x800000, 0x03, 0x08, 24000000, 125 },
	{ 12288000, 1, 0x01, 0x40, 0x000000, 0x03, 0x08, 24576000, 127 },

	{ 22579200, 1, 0x03, 0x40, 0x000000, 0x03, 0x10, 11289600, 128 },
	{ 22579200, 0, 0, 0, 0, 0, 0, 22579200, 0 },

	{ 24000000, 1, 0x03, 0x40, 0x000000, 0x03, 0x10, 12000000, 128 },
	{ 24000000, 1, 0x03, 0x40, 0x000000, 0x01, 0x10, 12288000, 131 },
	{ 24000000, 0, 0, 0, 0, 0, 0, 24000000, 0 },
	{ 24000000, 1, 0x03, 0x40, 0x000000, 0x01, 0x08, 24576000, 131 },

	{ 24576000, 1, 0x03, 0x40, 0x000000, 0x03, 0x10, 12288000, 128 },
	{ 24576000, 0, 0, 0, 0, 0, 0, 24576000, 0 },
};

static int cs43l36_pll_config(struct snd_soc_codec *codec)
{
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);
	int i;

	CS43L36_INFO("%s(): cs43l36->fs(%d), cs43l36->mclk(%d)\n",
		__func__, cs43l36->fs, cs43l36->mclk);

	for (i = 0; i < ARRAY_SIZE(pll_ratio_table); i++) {
		if (pll_ratio_table[i].sclk == cs43l36->sclk &&
			pll_ratio_table[i].mclk_int == cs43l36->mclk) {

//			snd_soc_update_bits(codec, CS43L36_MCLK_SRC_SEL,
//						CS43L36_MCLK_SRC_SEL_MASK,
//						pll_ratio_table[i].mclk_src_sel);

			if (pll_ratio_table[i].mclk_src_sel == 0) {
				snd_soc_update_bits(codec, CS43L36_PLL_CTL1,
							CS43L36_PLL_START_MASK, 0);
				cs43l36->pll_bypass = true;
				return 0;
			}

			snd_soc_update_bits(codec, CS43L36_PLL_DIV_CFG1,
						CS43L36_SCLK_PREDIV_MASK,
						pll_ratio_table[i].sclk_prediv);

			snd_soc_write(codec, CS43L36_PLL_DIV_INT,
						pll_ratio_table[i].pll_div_int);

			snd_soc_write(codec, CS43L36_PLL_DIV_FRAC0,
						pll_ratio_table[i].pll_div_frac	& 0xFF);

			snd_soc_write(codec, CS43L36_PLL_DIV_FRAC1,
						(pll_ratio_table[i].pll_div_frac & 0xFF00) >> 8);

			snd_soc_write(codec, CS43L36_PLL_DIV_FRAC2,
						(pll_ratio_table[i].pll_div_frac & 0xFF0000) >> 16);

			snd_soc_update_bits(codec, CS43L36_PLL_CTL4,
						CS43L36_PLL_MODE_MASK,
						pll_ratio_table[i].pll_mode);

			snd_soc_write(codec, CS43L36_PLL_CTL3,
						pll_ratio_table[i].pll_divout);

			snd_soc_write(codec, CS43L36_PLL_CAL_RATIO,
						pll_ratio_table[i].pll_cal_ratio);

			snd_soc_update_bits(codec, CS43L36_PLL_CTL1,
						CS43L36_PLL_START_MASK, 1);

			cs43l36->pll_bypass = false;

			return 0;
		}
	}
	return -EINVAL;
}

static int cs43l36_wait_pdn_done(struct snd_soc_codec *codec)
{
	unsigned reg_value = 0;
	unsigned loop_time = 0;

	while (loop_time < 30) {
		reg_value = snd_soc_read(codec, CS43L36_DAC_INT_STATUS);
		if (reg_value & 0x01) {
			pr_debug("%s: loop_time = %d\n", __func__, loop_time);
			return 0;
		}
		msleep(1);
		loop_time++;
	}

	pr_warn("%s(): power down not done\n", __func__);
	return -1;
}

#if 0
static int cs43l36_wait_mclk_done(struct snd_soc_codec *codec)
{
	int ret, loop = 0;

	snd_soc_write(codec, 0x1107, 0x01); //SCLK present
	usleep_range(150, 180);

	while (loop < 200) {
		ret = snd_soc_write(codec, 0x1107, 0x01); //SCLK present
		if (ret < 0) {
			usleep_range(100, 150);
		} else {
			pr_debug("%s: switch to internal MCLK success, loop = %d\n", __func__, loop);
			return 0;
		}
		loop++;
	}

	snd_soc_write(codec, 0x1107, 0x00); //Select MCLK to RCO
	pr_err("%s: switch to internal MCLK failed\n", __func__);
	return -1;
}
#endif

static int cs43l36_stop(struct snd_soc_codec *codec)
{
	//struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	pr_info("%s\n", __func__);

	//page10
	snd_soc_write(codec, 0x100A, 0xA0); //ASR_RATE
	snd_soc_write(codec, 0x100B, 0x50);

	//page23
	snd_soc_write(codec, 0x2301, 0x3F); //CHA_Vol = Mute
	snd_soc_write(codec, 0x2303, 0x3F); //CHB_Vol = Mute

	//page20
	snd_soc_write(codec, 0x2001, 0x0F); //mute analog A and B

	//page11
	snd_soc_write(codec, 0x1101, 0xFE); //power down HP

	msleep(50);

	//page12
	snd_soc_write(codec, 0x1201, 0x00); //MCLKDIV = /1, MCLK Source = SCLK pin
	snd_soc_write(codec, 0x1207, 0x00); //Disable SCLK

	//page15
	snd_soc_write(codec, 0x1501, 0x00); //PLL Start = Powered OFF

	msleep(2);

	//page11
	snd_soc_write(codec, 0x1107, 0x00); //SCLK_PRESENT = 0

	//page12
	snd_soc_write(codec, 0x120a, 0x00); //ASRC clock select = 6MHz
	snd_soc_write(codec, 0x1209, 0x00); //Disable IASRC

	//page11
	snd_soc_write(codec, 0x1102, 0x84); //Power Down SRC, Page 30
	snd_soc_write(codec, 0x1101, 0xFF); //Power Down All

	cs43l36_wait_pdn_done(codec);

	//page11
	snd_soc_write(codec, 0x1102, 0x94); //FILT+ clamped to GND

	return 0;
}

static int cs43l36_start(struct snd_soc_codec *codec)
{
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);
	int ret;

	pr_info("%s: full_scale = %d\n", __func__, cs43l36->full_scale);

	//page10
	snd_soc_write(codec, 0x1007, 0x10); //I2C normal drive, SRC NOT bypassed
	snd_soc_write(codec, 0x1009, 0x02); //Internal Fs = MCLKint/256, read always zero
	snd_soc_write(codec, 0x100a, 0xa0); //ASR_RATE
	snd_soc_write(codec, 0x100b, 0x50);

	//page11
	snd_soc_write(codec, 0x1102, 0x86); //Power up DAC SRC

	//page12
	snd_soc_write(codec, 0x1201, 0x00); //MCLKDIV = /1, MCLK Source = SCLK pin
	snd_soc_write(codec, 0x1203, 0x1f); //FSYNC Pulse Width, Lower Byte = 31
	snd_soc_write(codec, 0x1204, 0x00); //FSYNC Pulse Width, Upper Byte = 0
	snd_soc_write(codec, 0x1205, 0x3f); //FSYNC Period, Lower Byte = 63
	snd_soc_write(codec, 0x1206, 0x00); //FSYNC Period, Upper Byte = 0
	snd_soc_write(codec, 0x1207, 0x00); //Disable SCLK
	snd_soc_write(codec, 0x1208, 0x0a); //Start Phase = high-to-low, 50/50 Mode = 1, Frame Start Delay = 1 SCLKs

	cs43l36_pll_config(codec);
//	snd_soc_write(codec, 0x120C, 0x00);
//	snd_soc_write(codec, 0x1505, 0x40);
//	snd_soc_write(codec, 0x1502, 0x00);
//	snd_soc_write(codec, 0x1503, 0x00);
//	snd_soc_write(codec, 0x1504, 0x00);
//	snd_soc_write(codec, 0x151B, 0x03);
//	snd_soc_write(codec, 0x1508, 0x10);
//	snd_soc_write(codec, 0x150A, 0x80);
//	snd_soc_write(codec, 0x1501, 0x01);

	if (cs43l36->mclk >= 22579200) {		//~24M
		snd_soc_update_bits(codec, 0x1201, 0x02, 0x02); //MCLKint = internal PLL/2
	} else {
		snd_soc_update_bits(codec, 0x1201, 0x02, 0x00); //MCLKint = internal PLL/2
	}

	if (cs43l36->fs >= 88200 && cs43l36->fs <= 96000) {
		snd_soc_write(codec, 0x1209, 0x01); //Enable IASRC 96K and lower rates
		snd_soc_write(codec, 0x120a, 0x01); //ASRC clock select = 12MHz
	} else if (cs43l36->fs > 96000) {
		snd_soc_write(codec, 0x1209, 0x04); //Enable IASRC 192,176.4K
		snd_soc_write(codec, 0x120a, 0x02); //ASRC clock select = ~24MHz
	} else {
		snd_soc_write(codec, 0x1209, 0x00); //Disable IASRC
		snd_soc_write(codec, 0x120a, 0x00); //ASRC clock select = 6MHz
	}

	//page25
	snd_soc_write(codec, 0x2501, 0x04); //CHB high, CHA low
	snd_soc_write(codec, 0x2503, 0x8c); //ASP = 48 kHz

	//page26
	snd_soc_write(codec, 0x2601, 0x40); //SRC sample rate auto-detected

	//page2a
	snd_soc_write(codec, 0x2a01, 0x0c); //Enable RX0 CH2, Enable RX0 CH1
	snd_soc_write(codec, 0x2a02, 0x02); //RX0 CH1 phase = low, RX0 CH1 RES = 24 Bits
	snd_soc_write(codec, 0x2a03, 0x00); //RX0 CH1 MSB loc = 0 (default)
	snd_soc_write(codec, 0x2a04, 0x00); //RX0 CH1 LSB loc = 0 (default)
	snd_soc_write(codec, 0x2a05, 0x42); //RX0 CH2 phase = High, RX0 CH2 RES = 24 Bits
	snd_soc_write(codec, 0x2a06, 0x00); //RX0 CH2 MSB loc = 0 (default)
	snd_soc_write(codec, 0x2a07, 0x00); //RX0 CH2 LSB loc = 0 (default)

	//page12
	if (cs43l36->pll_bypass == false) {
		//snd_soc_write(codec, 0x1201, 0x01);
		snd_soc_update_bits(codec, 0x1201, 0x01, 0x01); //MCLK Source = PLL clock
	}

	snd_soc_write(codec, 0x1107, 0x01); //SCLK_PRESENT = 1
	msleep(10);

	ret = snd_soc_write(codec, 0x1207, 0x24); //Enable SCLK, Invert SCLK to DAC
	if (ret < 0) {
		pr_err("%s: Chip don't work while MCLK is sourced from sclk or pll clock.\n", __func__);
		snd_soc_write(codec, 0x1107, 0x00); //SCLK_PRESENT = 0
		snd_soc_write(codec, 0x1201, 0x00); //MCLKDIV = /1, MCLK Source = SCLK pin
		snd_soc_write(codec, 0x1501, 0x00); //PLL STOP
		snd_soc_write(codec, 0x120a, 0x00); //ASRC clock select = 6MHz
		snd_soc_write(codec, 0x1209, 0x00); //Disable IASRC
		return -1;
	}

	//page1f
	snd_soc_write(codec, 0x1f01, 0x00); //DACA/B not inverted
	snd_soc_write(codec, 0x1f04, 0x6a);
	snd_soc_write(codec, 0x1f05, 0x6a);
	snd_soc_write(codec, 0x1f06, 0x02); //1nF Mode, HPF on, Auto-Clamp

	//page23
	snd_soc_write(codec, 0x2301, 0x00); //CHA = 0 dB
	snd_soc_write(codec, 0x2303, 0x00); //CHB = 0 dB

	//page20
	snd_soc_write(codec, 0x2001, 0x02);

	//page11
	snd_soc_write(codec, 0x1101, 0x96); //Power Up Everything
	snd_soc_write(codec, 0x1102, 0x06); //Power Up Page 30

	//page18
	snd_soc_write(codec, 0x1801, 0x41); //Enable Page 30 Reads

	msleep(50);

	//page10
	snd_soc_write(codec, 0x107E, 0x99);

	//page20
	snd_soc_write(codec, 0x2015, 0xD5);
	snd_soc_write(codec, 0x2004, 0x00);
	snd_soc_write(codec, 0x2009, 0x00);

	if (cs43l36->full_scale) {
		snd_soc_write(codec, 0x2001, 0x01); //Unmuted, Unmuted, 0 dB
	} else {
		snd_soc_write(codec, 0x2001, 0x03); //Unmuted, Unmuted, -6 dB
	}

	snd_soc_write(codec, 0x2015, 0x00);
	snd_soc_write(codec, 0x2004, 0x0C);
	snd_soc_write(codec, 0x2009, 0x16);

	//page10
	snd_soc_write(codec, 0x107e, 0x00);
	snd_soc_write(codec, 0x100a, 0xA4);
	snd_soc_write(codec, 0x100b, 0x70);

	return 0;
}

#if 0
static const char const *hp_output_text[] = { "Off", "On" };
static int hp_output_ctl = 0;

static const struct soc_enum hp_output_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hp_output_text), hp_output_text);

static int cs43l36_hp_output_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hp_output_ctl;
	return 0;
}

static int cs43l36_hp_output_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	CS43L36_DEBUG("+%s(): value(%ld)\n", __func__, ucontrol->value.integer.value[0]);

	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(hp_output_text)) {
		pr_err("return -EINVAL\n");
		return -EINVAL;
	}

	hp_output_ctl = ucontrol->value.integer.value[0];

	cs43l36->fs = 48000;

	if (hp_output_ctl) {
		cs43l36_start(codec);
	} else {
		cs43l36_stop(codec);
	}

	return 0;
}
#endif

static const DECLARE_TLV_DB_SCALE(hpa_tlv, -6200, 100, 0);

//static const char *const full_scale_vol_text[] = { "0Db", "-6Db" };
//static const struct soc_enum full_scale_vol_enum =
//	SOC_ENUM_SINGLE(CS43L36_HP_CTL, 1, ARRAY_SIZE(full_scale_vol_text), full_scale_vol_text);

static const char *const full_scale_vol_text[] = { "Off", "On" };
static const struct soc_enum full_scale_vol_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(full_scale_vol_text), full_scale_vol_text);

static int cs43l36_full_scale_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = cs43l36->full_scale;
	return 0;
}

static int cs43l36_full_scale_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	CS43L36_DEBUG("+%s(): value(%ld)\n", __func__, ucontrol->value.integer.value[0]);

	if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(full_scale_vol_text)) {
		pr_err("return -EINVAL\n");
		return -EINVAL;
	}

	cs43l36->full_scale = ucontrol->value.integer.value[0];

	return 0;
}

static const char *const ana_mute_text[] = { "Unmute", "Mute" };
static const struct soc_enum ana_mute_enum[] = {
	SOC_ENUM_SINGLE(CS43L36_HP_CTL, 2, ARRAY_SIZE(ana_mute_text), ana_mute_text),	//Channel A
	SOC_ENUM_SINGLE(CS43L36_HP_CTL, 3, ARRAY_SIZE(ana_mute_text), ana_mute_text),	//Channel B
};

static const struct snd_kcontrol_new cs43l36_snd_controls[] = {
	/* DAC Volume and Filter Controls */
	SOC_SINGLE("DAC B=A Switch", CS43L36_DAC_CTL1, 2, 1, 0),
	SOC_SINGLE("DACA Invert Switch", CS43L36_DAC_CTL1, 0, 1, 0),
	SOC_SINGLE("DACB Invert Switch", CS43L36_DAC_CTL1, 1, 1, 0),
	SOC_SINGLE("DAC HPF Switch", CS43L36_DAC_CTL2, 1, 1, 0),
	SOC_DOUBLE_R_TLV("Input Volume", CS43L36_MIXER_CHA_VOL,
				  CS43L36_MIXER_CHB_VOL, 0, 0x3F, 1, hpa_tlv),
	//SOC_ENUM("Full-Scale Volume", full_scale_vol_enum),
	SOC_ENUM_EXT("Full-Scale Volume", full_scale_vol_enum,
			cs43l36_full_scale_get, cs43l36_full_scale_put),
	SOC_ENUM("Channel A Switch", ana_mute_enum[0]),
	SOC_ENUM("Channel B Switch", ana_mute_enum[1]),
//	SOC_ENUM_EXT("hp_output", hp_output_enum,
//		cs43l36_hp_output_get, cs43l36_hp_output_put),
};

#if 0
static int cs43l36_hp_dre_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	CS43L36_DEBUG("+%s\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
	case SND_SOC_DAPM_PRE_PMU:
		if (!cs43l36->hp_ana_mute) {
			snd_soc_update_bits(codec, CS43L36_HP_CTL,
						CS43L36_HP_ANA_AMUTE_MASK,
					CS43L36_HP_ANA_AMUTE_MASK);
		}
		break;
	default:
		pr_err("Invalid event = 0x%x\n", event);
	}
	return 0;
}

static const struct snd_kcontrol_new hp_dre_ctl =
	SOC_DAPM_SINGLE("Switch", CS43L36_HP_CTL, 0, 1, 0);

static const struct snd_soc_dapm_widget cs43l36_dapm_widgets[] = {

	SND_SOC_DAPM_SWITCH_E("DRE", CS43L36_HP_CTL, 0, 0, &hp_dre_ctl, \
			cs43l36_hp_dre_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_PRE_PMD),

};
#endif

static const struct snd_soc_dapm_route cs43l36_audio_map[] = {

};

static int cs43l36_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level)
{
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);
#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
	int ret;
#endif

	CS43L36_DEBUG("%s: level(%d)\n", __func__, level);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		//if (snd_soc_codec_get_bias_level(codec) == SND_SOC_BIAS_OFF) {
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			regcache_cache_only(cs43l36->regmap, false);
			regcache_sync(cs43l36->regmap);
#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
			ret = cs43l36_regulator_enable(cs43l36);
			if (ret != 0) {
				dev_err(codec->dev,
					"Failed to enable regulators: %d\n",
					ret);
				return ret;
			}
#endif
		}

		break;
	case SND_SOC_BIAS_OFF:
		regcache_cache_only(cs43l36->regmap, true);
#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
		cs43l36_regulator_disable(cs43l36);
#endif
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static struct regmap *cs43l36_get_regmap(struct device *dev)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);

	return cs43l36->regmap;
}


static unsigned int cs43l36_read(struct snd_soc_codec *codec, unsigned int addr)
{
	u8 value = 0;
	cs43l36_read_i2c_register((unsigned short)addr, &value);
	return value;
}

static int cs43l36_write(struct snd_soc_codec *codec, unsigned int address, unsigned int value)
{
	int ret = 0;
	ret = cs43l36_write_i2c_register(address, value);
	return ret;
}

static const struct snd_soc_codec_driver soc_codec_dev_cs43l36 = {
	.set_bias_level = cs43l36_set_bias_level,
	//.dapm_widgets = cs43l36_dapm_widgets,
	//.num_dapm_widgets = ARRAY_SIZE(cs43l36_dapm_widgets),
	.dapm_routes = cs43l36_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cs43l36_audio_map),

	.controls = cs43l36_snd_controls,
	.num_controls = ARRAY_SIZE(cs43l36_snd_controls),

	.get_regmap = cs43l36_get_regmap,
	.read = cs43l36_read,
	.write = cs43l36_write,
};

static int cs43l36_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		cs43l36->asp_hybrid_mode = CS43L36_ASP_MASTER_MODE;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		cs43l36->asp_hybrid_mode = CS43L36_ASP_SLAVE_MODE;
		break;
	default:
		return -EINVAL;
	}

	 /* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, CS43L36_ASP_CLK_CFG, CS43L36_ASP_MODE_MASK,
				cs43l36->asp_hybrid_mode << CS43L36_ASP_MODE_SHIFT);

	return 0;
}

static int cs43l36_pcm_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);
//	int ret;

	cs43l36->fs = params_rate(params);
	//if (cs43l36->fs == 16000) {
	//	cs43l36->sclk = 48000 * 32 * 2;
	//} else {
		cs43l36->sclk = cs43l36->fs * 32 * 2;	//force 2ch, 32bit
	//}

	if (!(cs43l36->fs % 8000)) { //8K serial
		cs43l36->mclk = 12288000;
	} else { //11.025k serial
		cs43l36->mclk = 11289600;
	}

	if (cs43l36->fs > 96000) { //~24M
		cs43l36->mclk *= 2;
	}

	//fix the 16k mode
	if (cs43l36->fs == 16000) {
		regcache_mark_dirty(cs43l36->regmap);
		cs43l36_reset(cs43l36);
		regcache_sync(cs43l36->regmap);
	}

	dev_info(codec->dev, "(sclk)%d : (rate)%d \n", cs43l36->sclk, cs43l36->fs);

	return 0;
}

static int cs43l36_set_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	CS43L36_DEBUG("%s: freq(%d)\n", __func__, freq);

	cs43l36->sclk = freq;

	return 0;
}

static int cs43l36_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);
	int try_cnt = 2, ret;
	CS43L36_DEBUG("%s\n", __func__);

	if (!cs43l36->prepared) {
restart:
		ret = cs43l36_start(codec);
		if(ret < 0) {
			regcache_mark_dirty(cs43l36->regmap);
			cs43l36_reset(cs43l36);
			regcache_sync(cs43l36->regmap);
			if (try_cnt > 0) {
				pr_info("%s: try_cnt(%d), restart chip\n", __func__, try_cnt);
				try_cnt--;
				goto restart;
			} else {
				return -1;
			}
		}
		cs43l36->prepared = 1;
	}

	return 0;
}

static int cs43l36_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	CS43L36_DEBUG("%s: cmd(%d)\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	}

	return 0;
}

static void cs43l36_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs43l36_private *cs43l36 = snd_soc_codec_get_drvdata(codec);

	CS43L36_DEBUG("%s\n", __func__);

	if (cs43l36->prepared) {
		cs43l36_stop(codec);
		cs43l36->prepared = 0;
	}
}

static struct snd_soc_dai_ops cs43l36_ops = {
	.hw_params  = cs43l36_pcm_hw_params,
	.set_fmt    = cs43l36_set_dai_fmt,
	.set_sysclk = cs43l36_set_sysclk,
	.prepare    = cs43l36_prepare,
	.trigger    = cs43l36_trigger,
	.shutdown   = cs43l36_shutdown,
};

#define CS43L36_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver cs43l36_dai = {
		.name = "cs43l36-aif",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 192000,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = CS43L36_FORMATS,
		},
		.ops = &cs43l36_ops,
};

static int cs43l36_handle_device_data(struct i2c_client *i2c_client, struct cs43l36_private *cs43l36)
{
	struct device_node *np = i2c_client->dev.of_node;
	unsigned int val;

	//val = of_get_named_gpio(np, "reset-gpio", 0);

	//CS43L36_DEBUG("GPIO NUMBER = %d\n", val);

	of_property_read_u32(np, "cirrus,hpout-dacmon", &val);
	switch (val) {
	case CS43L36_DAC_MON_PWRUP:
	case CS43L36_DAC_MON_NOPWR:
		cs43l36->hpout_dacmon = val;
		break;
	default:
		dev_err(&i2c_client->dev,
			"Wrong cirrus,hpout-dacmon DT value %d\n",
			val);
		cs43l36->hpout_dacmon = CS43L36_DAC_MON_PWRUP;
	}
	of_property_read_u32(np, "cirrus,hpout-clamp", &val);
	switch (val) {
	case CS43L36_HPOUT_CLAMP_EN:
	case CS43L36_HPOUT_CLAMP_DIS:
		cs43l36->hpout_clamp = val;
		break;
	default:
		dev_err(&i2c_client->dev,
			"Wrong cirrus,hpout-clamp DT value %d\n",
			val);
		cs43l36->hpout_clamp = CS43L36_HPOUT_CLAMP_EN;
	}
	of_property_read_u32(np, "cirrus,hpout-load", &val);
	switch (val) {
	case CS43L36_HPOUT_LOAD_1NF:
	case CS43L36_HPOUT_LOAD_10NF:
		cs43l36->hpout_load = val;
		break;
	default:
		dev_err(&i2c_client->dev,
			"Wrong cirrus,hpout-load DT value %d\n",
			val);
		cs43l36->hpout_load = CS43L36_HPOUT_LOAD_1NF;
	}
	of_property_read_u32(np, "cirrus,hpout-pulldn", &val);
	switch (val) {
	case CS43L36_HPOUT_PULLDN_1K:
	case CS43L36_HPOUT_PULLDN_3_7K:
	case CS43L36_HPOUT_PULLDN_6K:
	case CS43L36_HPOUT_PULLDN_9_6K:
	case CS43L36_HPOUT_PULLDN_787_OHM:
	case CS43L36_HPOUT_PULLDN_857_OHM:
		cs43l36->hpout_pulldn = val;
		break;
	default:
		dev_err(&i2c_client->dev,
			"Wrong cirrus,hpout-pulldn DT value %d\n",
			val);
		cs43l36->hpout_pulldn = CS43L36_HPOUT_PULLDN_AUTO;
	}

	return 0;
}


static ssize_t cs43l36_rw_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);
	u8 val = 0;
	int ret;

	val = simple_strtol(buf, NULL, 16);

	pr_info("%s: write 0x%.4x reg 0x%.2x\n", __func__, cs43l36->reg, val);

	ret = cs43l36_write_i2c_register(cs43l36->reg, val);
	if (ret < 0) {
		pr_err("%s: write 0x%.4x reg return %d\n", __func__, cs43l36->reg, ret);
		return -EIO;
	}

	return count;
}

static ssize_t cs43l36_rw_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);
	u8 val = 0;
	int ret;

	ret = cs43l36_read_i2c_register(cs43l36->reg, &val);
	if (ret < 0) {
		pr_err("%s: read 0x%.4x reg return %d\n", __func__, cs43l36->reg, ret);
		return -EIO;
	}

	pr_info("%s: read 0x%.4x reg return 0x%.2x\n", __func__, cs43l36->reg, val);

	return snprintf(buf, PAGE_SIZE, "reg:0x%.4x value is 0x%.2x\n", cs43l36->reg, val);
}

static ssize_t cs43l36_reg_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);

	cs43l36->reg = simple_strtol(buf, NULL, 16);
	pr_info("%s: cs43l36->reg = 0x%.4x\n", __func__, cs43l36->reg);

	return count;
}

static int cs43l36_wait_load_det_done(struct cs43l36_private *cs43l36)
{
	unsigned int reg_value = 0;
	unsigned loop_time = 0;

	while (loop_time < 30) {
		regmap_read(cs43l36->regmap, CS43L36_LOAD_DET_DONE, &reg_value);
		if (reg_value & 0x01) {
			pr_debug("%s: loop_time = %d\n", __func__, loop_time);
			return 0;
		}
		msleep(1);
		loop_time++;
	}

	pr_warn("%s(): load det not done\n", __func__);
	return -1;
}

int cs43l36_load_detection(struct device *dev)
{
	int ret = 0;
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);
	unsigned int pwr_ctrl1_val, hp_ctl_val, dac_ctl2_val, mic_det_ctl_val,
	classh_ctl_val, sftramp_rate_val, rcstat_val;
	u8 res_val;

	pr_debug("%s\n", __func__);

	//power down headphone.
	regmap_read(cs43l36->regmap, CS43L36_PWR_CTL1, &pwr_ctrl1_val);
	//regmap_update_bits(cs43l36->regmap, CS43L36_PWR_CTL1,
	//		1 << 3, 1 << 3);
	//regmap_update_bits(cs43l36->regmap, CS43L36_PWR_CTL1,
	//		1 << 0, 0 << 0);
	regmap_write(cs43l36->regmap, CS43L36_PWR_CTL1, 0xFE);

	//mute the analog outputs.
	regmap_read(cs43l36->regmap, CS43L36_HP_CTL, &hp_ctl_val);
	//regmap_update_bits(cs43l36->regmap, CS43L36_HP_CTL,
	//		1 << 3, 1 << 3);
	//regmap_update_bits(cs43l36->regmap, CS43L36_HP_CTL,
	//		1 << 2, 1 << 2);
	regmap_write(cs43l36->regmap, CS43L36_HP_CTL, 0x0D);

	//disable DAC high pass filter
	regmap_read(cs43l36->regmap, CS43L36_DAC_CTL2, &dac_ctl2_val);
	//regmap_update_bits(cs43l36->regmap, CS43L36_DAC_CTL2,
	//		1 << 1, 0 << 1);
	regmap_write(cs43l36->regmap, CS43L36_DAC_CTL2, 0x00);

	//latch to VP.
	regmap_read(cs43l36->regmap, CS43L36_MIC_DET_CTL, &mic_det_ctl_val);
	//regmap_update_bits(cs43l36->regmap, CS43L36_MIC_DET_CTL,
	//		1 << 7, 1 << 7);
	regmap_write(cs43l36->regmap, CS43L36_MIC_DET_CTL, 0x9f);

	//set ADPTPWR to "100".
	regmap_read(cs43l36->regmap, CS43L36_CLASSH_CTL, &classh_ctl_val);
	regmap_write(cs43l36->regmap, CS43L36_CLASSH_CTL, 0x04);

	//set ASR_RATE to "0111" and DSR_RATE to "0001".
	regmap_read(cs43l36->regmap, CS43L36_SFTRAMP_RATE, &sftramp_rate_val);
	regmap_write(cs43l36->regmap, CS43L36_SFTRAMP_RATE, 0x71);

	//HP load detect enable
	regmap_write(cs43l36->regmap, CS43L36_LOAD_DET_EN, 0x01);

	//Delay 10mS
	msleep(10);

	ret = cs43l36_wait_load_det_done(cs43l36);
	if (ret < 0) {
		int ret_val;
		regmap_read(cs43l36->regmap, CS43L36_PWR_CTL1, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_PWR_CTL1, ret_val);

		regmap_read(cs43l36->regmap, CS43L36_HP_CTL, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_HP_CTL, ret_val);

		regmap_read(cs43l36->regmap, CS43L36_DAC_CTL2, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_DAC_CTL2, ret_val);

		regmap_read(cs43l36->regmap, CS43L36_MIC_DET_CTL, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_MIC_DET_CTL, ret_val);

		regmap_read(cs43l36->regmap, CS43L36_CLASSH_CTL, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_CLASSH_CTL, ret_val);

		regmap_read(cs43l36->regmap, CS43L36_SFTRAMP_RATE, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_SFTRAMP_RATE, ret_val);

		regmap_read(cs43l36->regmap, CS43L36_LOAD_DET_EN, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_LOAD_DET_EN, ret_val);

		regmap_read(cs43l36->regmap, CS43L36_LOAD_DET_DONE, &ret_val);
		pr_warn("%s: 0x%.4x = 0x%.2x\n", __func__, CS43L36_LOAD_DET_DONE, ret_val);

		impedance = 30;
		goto det_done;
	}

	regmap_read(cs43l36->regmap, CS43L36_LOAD_DET_RCSTAT, &rcstat_val);
	pr_info("%s: CS43L36_LOAD_DET_RCSTAT = 0x%.2x\n", __func__, rcstat_val);
	switch (rcstat_val & 0x03) {
	case 0:
		impedance = 15;
		break;
	case 1:
		impedance = 30;
		break;
	case 2:
		impedance = 3000;
		break;
	default:
		ret = -EIO;
		break;
	}

	if (impedance < 3000) {
		cs43l36->hpout_load = 1;
	} else {
		cs43l36->hpout_load = (rcstat_val & 0x10) ? 0 : 1;
		if (cs43l36->hpout_load == 1) {
			impedance = 300;
		}
	}

	cs43l36_read_i2c_register(0x191E, &res_val);
	pr_info("%s: th2_new = 0x%.2x, 0x191E = 0x%.2x\n", __func__, th2_new, res_val);

//	dac_ctl2_val = ((cs43l36->hpout_load << 3) & 0x08) | (dac_ctl2_val & 0xf7);
//	pr_info("%s: dac_ctl2_val = 0x%.2x, hpout_load = %d\n", __func__,
//			dac_ctl2_val, cs43l36->hpout_load);

det_done:
	//disable HP load detect.
	regmap_write(cs43l36->regmap, CS43L36_LOAD_DET_EN, 0x00);
	regmap_write(cs43l36->regmap, CS43L36_SFTRAMP_RATE, sftramp_rate_val);
	regmap_write(cs43l36->regmap, CS43L36_CLASSH_CTL, classh_ctl_val);
	regmap_write(cs43l36->regmap, CS43L36_MIC_DET_CTL, mic_det_ctl_val);
	regmap_write(cs43l36->regmap, CS43L36_DAC_CTL2, dac_ctl2_val);
	regmap_write(cs43l36->regmap, CS43L36_HP_CTL, hp_ctl_val);
	regmap_write(cs43l36->regmap, CS43L36_PWR_CTL1, pwr_ctrl1_val);

	pr_info("%s: exit, impedance(%d)\n", __func__, impedance);

	return 0;
}
EXPORT_SYMBOL(cs43l36_load_detection);

extern void accdet_get_impedance(struct device *dev);
static ssize_t cs43l36_impedance_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	accdet_get_impedance(dev);

	pr_info("%s: impedance(%d)\n", __func__, impedance);
	return snprintf(buf, PAGE_SIZE, "%d", impedance);
}

static DEVICE_ATTR(reg,	S_IWUSR | S_IRUGO, NULL, cs43l36_reg_store);
static DEVICE_ATTR(rw,	S_IWUSR | S_IRUGO, cs43l36_rw_show, cs43l36_rw_store);
static DEVICE_ATTR(impedance,	S_IWUSR | S_IRUGO, cs43l36_impedance_show, NULL);

static struct attribute *cs43l36_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_rw.attr,
	&dev_attr_impedance.attr,
	NULL
};

static struct attribute_group cs43l36_attribute_group = {
	.attrs = cs43l36_attributes
};

static int cs43l36_i2c_probe(struct i2c_client *i2c_client,
					   const struct i2c_device_id *id)
{
	struct cs43l36_private *cs43l36;
	int ret;
	unsigned int devid = 0;
	unsigned int reg;

	CS43L36_INFO("+%s()\n", __func__);

	new_client = i2c_client;

	cs43l36 = devm_kzalloc(&i2c_client->dev, sizeof(struct cs43l36_private),
				   GFP_KERNEL);
	if (!cs43l36) {
		dev_err(&i2c_client->dev, "could not allocate codec\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c_client, cs43l36);

	ret = cs43l36_dts_gpio_init(i2c_client);
	if (ret != 0)
		return ret;

	cs43l36->regmap = devm_regmap_init_i2c(i2c_client, &cs43l36_regmap);
	if (IS_ERR(cs43l36->regmap)) {
		ret = PTR_ERR(cs43l36->regmap);
		dev_err(&i2c_client->dev, "regmap_init() failed: %d\n", ret);
		return ret;
	}

	if (i2c_client->dev.of_node) {
		ret = cs43l36_handle_device_data(i2c_client, cs43l36);
		if (ret != 0)
			return ret;
	}

#if 0
	for (i = 0; i < ARRAY_SIZE(cs43l36->supplies); i++)
		cs43l36->supplies[i].supply = cs43l36_supply_names[i];

	ret = devm_regulator_bulk_get(&i2c_client->dev,
					  ARRAY_SIZE(cs43l36->supplies),
					  cs43l36->supplies);
	if (ret != 0) {
		dev_err(&i2c_client->dev,
			"Failed to request supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(cs43l36->supplies),
					cs43l36->supplies);
#endif
	ret = cs43l36_regulator_enable(cs43l36);
	if (ret != 0) {
		dev_err(&i2c_client->dev,
			"Failed to enable supplies: %d\n", ret);
		return ret;
	}

	/* Reset the Device */
	cs43l36_reset(cs43l36);

	/*readID patch to set Page 30 for rev's */
	ret = regmap_update_bits(cs43l36->regmap, CS43L36_PWR_CTL2,
				CS43L36_PG30_PDN_MASK, 0 << CS43L36_PG30_PDN_SHIFT);

	ret = regmap_write(cs43l36->regmap, CS43L36_PG30_READ_EN, 0x01);

	/* initialize codec */
	ret = regmap_read(cs43l36->regmap, CS43L36_DEVID_AB, &reg);
	devid = (reg & 0xFF) << 12;

	ret = regmap_read(cs43l36->regmap, CS43L36_DEVID_CD, &reg);
	devid |= (reg & 0xFF) << 4;

	ret = regmap_read(cs43l36->regmap, CS43L36_DEVID_E, &reg);
	devid |= (reg & 0xF0) >> 4;

	if (devid != CS43L36_CHIP_ID) {
		ret = -ENODEV;
		dev_err(&i2c_client->dev,
			"CS43L36 Device ID (%X). Expected %X\n",
			reg, CS43L36_CHIP_ID);
		//return ret;
	}

	ret = regmap_read(cs43l36->regmap, CS43L36_REVID, &reg);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "Get Revision ID failed, reg(%02x)\n", reg);
		//return ret;
	}

	dev_info(&i2c_client->dev,
		 "Cirrus Logic CS43L36, Revision: %02X\n", reg & 0xFF);

#if 0
	/* Handle HPOUT Device Data */
	if (cs43l36->hpout_load)
		regmap_update_bits(cs43l36->regmap, CS43L36_DAC_CTL2,
				   CS43L36_HPOUT_LOAD_MASK,
				cs43l36->hpout_load <<
				  CS43L36_HPOUT_LOAD_SHIFT);
	if (cs43l36->hpout_clamp)
		regmap_update_bits(cs43l36->regmap, CS43L36_DAC_CTL2,
				   CS43L36_HPOUT_CLAMP_MASK,
				cs43l36->hpout_clamp <<
				  CS43L36_HPOUT_CLAMP_SHIFT);
	if (cs43l36->hpout_pulldn)
		regmap_update_bits(cs43l36->regmap, CS43L36_DAC_CTL2,
				   CS43L36_HPOUT_PULLDN_MASK,
				cs43l36->hpout_pulldn <<
				  CS43L36_HPOUT_PULLDN_SHIFT);
	if (cs43l36->hpout_dacmon)
		regmap_update_bits(cs43l36->regmap, CS43L36_DAC_CTL2,
				   CS43L36_DAC_MON_EN_MASK,
				cs43l36->hpout_dacmon <<
				  CS43L36_DAC_MON_EN_SHIFT);
#endif

	if (i2c_client->dev.of_node)
		dev_set_name(&i2c_client->dev, "%s", "cs43l36");

	ret =  snd_soc_register_codec(&i2c_client->dev,
			&soc_codec_dev_cs43l36, &cs43l36_dai, 1);
	if (ret < 0)
		goto err_disable;

	CS43L36_DEBUG("-%s()\n", __func__);

	dev_set_drvdata(&i2c_client->dev, cs43l36);
	ret = sysfs_create_group(&i2c_client->dev.kobj, &cs43l36_attribute_group);
	if (ret < 0) {
		dev_info(&i2c_client->dev, "error creating sysfs attr files\n");
	} else {
		meizu_sysfslink_register_name(&i2c_client->dev, "hifi");	//cs43l36
	}

	cs43l36->prepared = 0;
	return 0;

err_disable:
#if 0
	regulator_bulk_disable(ARRAY_SIZE(cs43l36->supplies),
				   cs43l36->supplies);
#endif
#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
	cs43l36_regulator_disable(cs43l36);
#endif

	return ret;
}

static int cs43l36_i2c_remove(struct i2c_client *i2c_client)
{
	struct cs43l36_private *cs43l36 = i2c_get_clientdata(i2c_client);

	snd_soc_unregister_codec(&i2c_client->dev);

	/* Hold down reset */
	if (!IS_ERR(cs43l36->state_rst_l)) {
		pinctrl_select_state(cs43l36->pctrl, cs43l36->state_rst_l);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cs43l36_suspend(struct device *dev)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	if (cs43l36->prepared) {
		pr_info("%s: cs43l36 already prepared, return\n", __func__);
		return 0;
	}

	regcache_cache_only(cs43l36->regmap, true);
	regcache_mark_dirty(cs43l36->regmap);

	/* Hold down reset */
	if (!IS_ERR(cs43l36->state_rst_l)) {
		pinctrl_select_state(cs43l36->pctrl, cs43l36->state_rst_l);
	}

	/* remove power */
#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
	cs43l36_regulator_disable(cs43l36);
#endif

	return 0;
}

static int cs43l36_resume(struct device *dev)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);
#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
	int ret;
#endif

	pr_info("%s\n", __func__);

	if (cs43l36->prepared) {
		pr_info("%s: cs43l36 already prepared, return\n", __func__);
		return 0;
	}

#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
	/* Enable power */
	ret = cs43l36_regulator_enable(cs43l36);
	if (ret != 0) {
		dev_err(dev, "Failed to enable supplies: %d\n",
			ret);
		return ret;
	}
#endif

	if (!IS_ERR(cs43l36->state_rst_l)) {
		pinctrl_select_state(cs43l36->pctrl, cs43l36->state_rst_h);
	}

	msleep(2);

	cs43l36_load_threshold_cal();

	regcache_cache_only(cs43l36->regmap, false);
	regcache_sync(cs43l36->regmap);

	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int cs43l36_runtime_suspend(struct device *dev)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);

	regcache_cache_only(cs43l36->regmap, true);
	regcache_mark_dirty(cs43l36->regmap);

	/* Hold down reset */
	if (!IS_ERR(cs43l36->state_rst_l)) {
		pinctrl_select_state(cs43l36->pctrl, cs43l36->state_rst_l);
	}

	/* remove power */
#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
	cs43l36_regulator_disable(cs43l36);
#endif

	return 0;
}

static int cs43l36_runtime_resume(struct device *dev)
{
	struct cs43l36_private *cs43l36 = dev_get_drvdata(dev);
	int ret;

#ifdef CS43L36_STANDBY_DISABLE_REGULATOR
	/* Enable power */
	ret = cs43l36_regulator_enable(cs43l36);
	if (ret != 0) {
		dev_err(dev, "Failed to enable supplies: %d\n",
			ret);
		return ret;
	}
#endif

	if (!IS_ERR(cs43l36->state_rst_l)) {
		pinctrl_select_state(cs43l36->pctrl, cs43l36->state_rst_h);
	}

	msleep(2);

	cs43l36_load_threshold_cal();

	regcache_cache_only(cs43l36->regmap, false);
	regcache_sync(cs43l36->regmap);

	return 0;
}
#endif

static const struct dev_pm_ops cs43l36_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(cs43l36_suspend, cs43l36_resume)
	SET_RUNTIME_PM_OPS(cs43l36_runtime_suspend, cs43l36_runtime_resume,
			   NULL)
};

static const struct of_device_id cs43l36_of_match[] = {
	{ .compatible = "cirrus,cs43l36", },
	{ .compatible = "mediatek,hifi", },
	{},
};
MODULE_DEVICE_TABLE(of, cs43l36_of_match);


static const struct i2c_device_id cs43l36_id[] = {
	{"cs43l36", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs43l36_id);

static struct i2c_driver cs43l36_i2c_driver = {
	.driver = {
		   .name = "cs43l36",
		   .owner = THIS_MODULE,
		   .pm = &cs43l36_pm,
		   .of_match_table = cs43l36_of_match,
		   },
	.id_table = cs43l36_id,
	.probe = cs43l36_i2c_probe,
	.remove = cs43l36_i2c_remove,
};

static int __init cs43l36_init(void)
{
	if (i2c_add_driver(&cs43l36_i2c_driver))
	{
		pr_err("%s: fail to add device into i2c \n", __func__);
		return -1;
	}

	return 0;
}

static void __exit cs43l36_exit(void)
{
	i2c_del_driver(&cs43l36_i2c_driver);
}

module_init(cs43l36_init);
module_exit(cs43l36_exit);
//module_i2c_driver(cs43l36_i2c_driver);

MODULE_DESCRIPTION("ASoC CS43L36 driver");
MODULE_AUTHOR("Brian Austin, Cirrus Logic Inc, <brian.austin@cirrus.com>");
MODULE_LICENSE("GPL");
