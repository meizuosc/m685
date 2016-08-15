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

#ifndef __CS43L36_H__
#define __CS43L36_H__

#include <linux/pinctrl/consumer.h>

#define CS43L36_PAGE_REGISTER	0x00	/* Page Select Register */
#define CS43L36_WIN_START	0x00
#define CS43L36_WIN_LEN		0x100
#define CS43L36_RANGE_MIN	0x00
#define CS43L36_RANGE_MAX	0x7F

#define CS43L36_PAGE_10		0x1000
#define CS43L36_PAGE_11		0x1100
#define CS43L36_PAGE_12		0x1200
#define CS43L36_PAGE_13		0x1300
#define CS43L36_PAGE_14		0x1400
#define CS43L36_PAGE_15		0x1500
#define CS43L36_PAGE_18		0x1800
#define CS43L36_PAGE_19		0x1900
#define CS43L36_PAGE_1B		0x1B00
#define CS43L36_PAGE_1F		0x1F00
#define CS43L36_PAGE_20		0x2000
#define CS43L36_PAGE_21		0x2100
#define CS43L36_PAGE_23		0x2300
#define CS43L36_PAGE_25		0x2500
#define CS43L36_PAGE_26		0x2600
#define CS43L36_PAGE_2A		0x2A00
#define CS43L36_PAGE_30		0x3000

#define CS43L36_CHIP_ID			0x43A36
/* Page 0x10 Global Registers */

#define CS43L36_REVID			(CS43L36_PAGE_10 + 0x05)
#define CS43L36_FRZ_CTL			(CS43L36_PAGE_10 + 0x06)
#define CS43L36_SRC_CTL			(CS43L36_PAGE_10 + 0x07)
#define CS43L36_MCLK_STAT		(CS43L36_PAGE_10 + 0x08)
#define CS43L36_MCLK_CTL		(CS43L36_PAGE_10 + 0x09)
#define CS43L36_SFTRAMP_RATE		(CS43L36_PAGE_10 + 0x0A)
#define CS43L36_SLOWSTART_EN		(CS43L36_PAGE_10 + 0x0B)
#define CS43L36_I2C_DEBOUNCE		(CS43L36_PAGE_10 + 0x0E)
#define CS43L36_I2C_STRETCH		(CS43L36_PAGE_10 + 0x0F)
#define CS43L36_I2C_TIMEOUT		(CS43L36_PAGE_10 + 0x10)

/* Page 0x11 Power and Headset Detect Registers */
#define CS43L36_PWR_CTL1		(CS43L36_PAGE_11 + 0x01)
#define CS43L36_PWR_CTL2		(CS43L36_PAGE_11 + 0x02)
#define CS43L36_PWR_CTL3		(CS43L36_PAGE_11 + 0x03)
#define CS43L36_OSC_SWITCH		(CS43L36_PAGE_11 + 0x07)
#define CS43L36_OSC_SWITCH_STATUS	(CS43L36_PAGE_11 + 0x09)
#define CS43L36_TSENSE_CTL		(CS43L36_PAGE_11 + 0x13)
#define CS43L36_TSENSE_STATUS		(CS43L36_PAGE_11 + 0x15)

/* Page 0x12 Clocking Registers */
#define CS43L36_MCLK_SRC_SEL		(CS43L36_PAGE_12 + 0x01)
#define CS43L36_FSYNC_PW_LOWER		(CS43L36_PAGE_12 + 0x03)
#define CS43L36_FSYNC_PW_UPPER		(CS43L36_PAGE_12 + 0x04)
#define CS43L36_FSYNC_P_LOWER		(CS43L36_PAGE_12 + 0x05)
#define CS43L36_FSYNC_P_UPPER		(CS43L36_PAGE_12 + 0x06)
#define CS43L36_ASP_CLK_CFG		(CS43L36_PAGE_12 + 0x07)
#define CS43L36_ASP_FRM_CFG		(CS43L36_PAGE_12 + 0x08)
#define CS43L36_FS_RATE_EN		(CS43L36_PAGE_12 + 0x09)
#define CS43L36_IN_ASRC_CLK		(CS43L36_PAGE_12 + 0x0A)
#define CS43L36_PLL_DIV_CFG1		(CS43L36_PAGE_12 + 0x0C)

/* Page 0x13 Interrupt Registers */
/* Interrupts */
#define CS43L36_CH_OVFL_STATUS		(CS43L36_PAGE_13 + 0x02)
#define CS43L36_SRC_STATUS		(CS43L36_PAGE_13 + 0x03)
#define CS43L36_ASP_RX_STATUS		(CS43L36_PAGE_13 + 0x04)
#define CS43L36_DAC_INT_STATUS		(CS43L36_PAGE_13 + 0x08)
#define CS43L36_DETECT_STATUS1		(CS43L36_PAGE_13 + 0x09)
#define CS43L36_DAC_LOCK_STATUS		(CS43L36_PAGE_13 + 0x0B)
#define CS43L36_VPMON_STATUS		(CS43L36_PAGE_13 + 0x0D)
#define CS43L36_PLL_LOCK		(CS43L36_PAGE_13 + 0x0E)
#define CS43L36_TS_PLUG_STATUS		(CS43L36_PAGE_13 + 0x0F)
/* Masks */
#define CS43L36_CH_OVFL_MASK		(CS43L36_PAGE_13 + 0x17)
#define CS43L36_SRC_MASK		(CS43L36_PAGE_13 + 0x18)
#define CS43L36_ASP_RX_MASK		(CS43L36_PAGE_13 + 0x19)
#define CS43L36_DAC_INT_MASK		(CS43L36_PAGE_13 + 0x1B)
#define CS43L36_DAC_LOCK_MASK		(CS43L36_PAGE_13 + 0x1C)
#define CS43L36_VPMON_MASK		(CS43L36_PAGE_13 + 0x1E)
#define CS43L36_PLL_LOCK_MASK		(CS43L36_PAGE_13 + 0x1F)
#define CS43L36_TS_PLUG_MASK		(CS43L36_PAGE_13 + 0x20)

/* Page 0x15 Fractional-N PLL Registers */
#define CS43L36_PLL_CTL1		(CS43L36_PAGE_15 + 0x01)
#define CS43L36_PLL_DIV_FRAC0		(CS43L36_PAGE_15 + 0x02)
#define CS43L36_PLL_DIV_FRAC1		(CS43L36_PAGE_15 + 0x03)
#define CS43L36_PLL_DIV_FRAC2		(CS43L36_PAGE_15 + 0x04)
#define CS43L36_PLL_DIV_INT		(CS43L36_PAGE_15 + 0x05)
#define CS43L36_PLL_CTL3		(CS43L36_PAGE_15 + 0x08)
#define CS43L36_PLL_CAL_RATIO		(CS43L36_PAGE_15 + 0x0A)
#define CS43L36_PLL_CTL4		(CS43L36_PAGE_15 + 0x1B)

/* Page 0x18 Page 30 Enable Registers */
#define CS43L36_PG30_READ_EN		(CS43L36_PAGE_18 + 0x01)

/* Page 0x19 HP Load Detect Registers */
#define CS43L36_LOAD_DET_RCSTAT		(CS43L36_PAGE_19 + 0x25)
#define CS43L36_LOAD_DET_DONE		(CS43L36_PAGE_19 + 0x26)
#define CS43L36_LOAD_DET_EN		(CS43L36_PAGE_19 + 0x27)

/* Page 0x1B Headset Interface Registers */
#define CS43L36_WAKE_CTL		(CS43L36_PAGE_1B + 0x71)
#define CS43L36_TIPSENSE_CTL		(CS43L36_PAGE_1B + 0x73)
#define CS43L36_MIC_DET_CTL		(CS43L36_PAGE_1B + 0x75)
#define CS43L36_DET_STATUS		(CS43L36_PAGE_1B + 0x77)
#define CS43L36_DET_INT_MASK		(CS43L36_PAGE_1B + 0x79)

/* Page 0x1F DAC Registers */
#define CS43L36_DAC_CTL1		(CS43L36_PAGE_1F + 0x01)
#define CS43L36_DAC_CTL2		(CS43L36_PAGE_1F + 0x06)

/* Page 0x20 HP CTL Registers */
#define CS43L36_HP_CTL			(CS43L36_PAGE_20 + 0x01)

/* Page 0x21 Class H Registers */
#define CS43L36_CLASSH_CTL		(CS43L36_PAGE_21 + 0x01)

/* Page 0x23 Mixer Volume Registers */
#define CS43L36_MIXER_CHA_VOL		(CS43L36_PAGE_23 + 0x01)
#define CS43L36_MIXER_CHB_VOL		(CS43L36_PAGE_23 + 0x03)

/* Page 0x25 Audio Port Registers */
#define CS43L36_SP_RX_CH_SEL		(CS43L36_PAGE_25 + 0x01)
#define CS43L36_SP_RX_ISOC_CTL		(CS43L36_PAGE_25 + 0x02)
#define CS43L36_SP_RX_FS		(CS43L36_PAGE_25 + 0x03)

/* Page 0x26 SRC Registers */
#define CS43L36_SRC_SDIN_FS		(CS43L36_PAGE_26 + 0x01)

/* Page 0x2A Serial Port RX Registers */
#define CS43L36_ASP_RX_DAI0_EN		(CS43L36_PAGE_2A + 0x01)
#define CS43L36_ASP_RX_DAI0_CH1_PH_RES	(CS43L36_PAGE_2A + 0x02)
#define CS43L36_ASP_RX_DAI0_CH1_BIT_MSB	(CS43L36_PAGE_2A + 0x03)
#define CS43L36_ASP_RX_DAI0_CH1_BIT_LSB	(CS43L36_PAGE_2A + 0x04)
#define CS43L36_ASP_RX_DAI0_CH2_PH_RES	(CS43L36_PAGE_2A + 0x05)
#define CS43L36_ASP_RX_DAI0_CH2_BIT_MSB	(CS43L36_PAGE_2A + 0x06)
#define CS43L36_ASP_RX_DAI0_CH2_BIT_LSB	(CS43L36_PAGE_2A + 0x07)

/* Page 0x30 ID Registers */
#define CS43L36_SUB_REVID		(CS43L36_PAGE_30 + 0x14)
#define CS43L36_DEVID_AB		(CS43L36_PAGE_30 + 0x15)
#define CS43L36_DEVID_CD		(CS43L36_PAGE_30 + 0x16)
#define CS43L36_DEVID_E			(CS43L36_PAGE_30 + 0x17)
#define CS43L36_MAX_REGISTER		(CS43L36_PAGE_30 + 0x18)

/* ASP Config Values */
#define CS43L36_ASP_MASTER_MODE		0x01
#define CS43L36_ASP_SLAVE_MODE		0x00
#define CS43L36_ASP_MODE_MASK		(1 << 4)
#define CS43L36_ASP_MODE_SHIFT		4

/* PLL Config Shift Values */
#define CS43L36_MCLK_SRC_SEL_MASK	(1 << 0)
#define CS43L36_SCLK_PREDIV_MASK	(1 << 0)
#define CS43L36_PLL_DIV_INT_MASK	0xFF
#define CS43L36_PLL_DIV_FRAC_MASK	0xFF
#define CS43L36_PLL_MODE_MASK		(3 << 0)
#define CS43L36_PLL_DIVOUT_MASK		0xFF
#define CS43L36_PLL_CAL_RATIO_MASK	0xFF
#define CS43L36_PLL_START_MASK		(1 << 0)

/* DAC/HP Config Mask/Shift Values */
#define CS43L36_HPOUT_PULLDN_MASK	0x10
#define CS43L36_HPOUT_PULLDN_SHIFT	4
#define CS43L36_HPOUT_LOAD_MASK		0x08
#define CS43L36_HPOUT_LOAD_SHIFT	3
#define CS43L36_HPOUT_CLAMP_MASK	0x04
#define CS43L36_HPOUT_CLAMP_SHIFT	2
#define CS43L36_DAC_HPF_EN_MASK		0x02
#define CS43L36_DAC_HPF_EN_SHIFT	1
#define CS43L36_DAC_MON_EN_MASK		0x01
#define CS43L36_DAC_MON_EN_SHIFT	0	
#define CS43L36_HP_ANA_AMUTE_MASK	0x04
#define CS43L36_HP_ANA_AMUTE_SHIFT	2
#define CS43L36_HP_ANA_BMUTE_MASK	0x08
#define CS43L36_HP_ANA_BMUTE_SHIFT	3


#define CS43L36_PG30_PDN_MASK		0x80
#define CS43L36_PG30_PDN_SHIFT		7


#define CS43L36_NUM_SUPPLIES 5
static const char *const cs43l36_supply_names[CS43L36_NUM_SUPPLIES] = {
	"VA",
	"VP",
	"VCP",
	"VD_FILT",
	"VL",
};

struct  cs43l36_private {
	struct regmap *regmap;
	struct snd_soc_codec *codec;
	struct regulator_bulk_data supplies[CS43L36_NUM_SUPPLIES];
//	struct gpio_desc *reset_gpio;
//	struct gpio_desc *vio_en_gpio;
	struct pinctrl *pctrl;
	struct pinctrl_state *state_rst_h;
	struct pinctrl_state *state_rst_l;
	struct pinctrl_state *state_vio_en;
	struct pinctrl_state *state_vio_dis;
	struct pinctrl_state *state_irq;
	u32 sclk;
	u32 fs;
	u32 mclk;
	u32 asp_hybrid_mode;
	bool pll_bypass;
	u8 hpout_load;
	u8 hpout_clamp;
	u8 hpout_pulldn;
	u8 hpout_dacmon;
	bool hp_ana_mute;
	int16_t reg;
	bool full_scale;
	bool prepared;
};

#endif
