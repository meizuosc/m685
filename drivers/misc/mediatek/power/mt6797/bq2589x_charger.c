;/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Copyright (C) 2014  Meizu Technology Co.Ltd 
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <linux/meizu-sys.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/mtk_thermal_typedefs.h>
#include <linux/interrupt.h>
#include <mt-plat/mt_boot_common.h>
#include <linux/power/bq2589x_reg.h>
#include <linux/hw_reset.h>

#define BQ2589x_REG_NUM	21 

#define BQ25892_I2C0 0
#define BQ25892_I2C5 1

static DEFINE_MUTEX(i2c_mutex);
static DEFINE_MUTEX(i2c_update_mutex);

static struct bq2589x *g_bq = NULL;
static struct bq2589x *g_bq_sed = NULL;
wait_queue_head_t vbus_wq;
int vbus_change_event = KAL_FALSE;

int bq2589x_get_boot_mode(void)
{
	return get_boot_mode();
}

static int bq2589x_read_byte(u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_read_byte_data(g_bq->client, reg);
	if (ret < 0) {
		dev_err(g_bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&i2c_mutex);

	return 0;
}

static int bq2589x_write_byte(u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_write_byte_data(g_bq->client, reg, data);
	if (ret < 0 ) {
		dev_err(g_bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	mutex_unlock(&i2c_mutex);
	return ret;
}

int bq2589x_update_bits(u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_update_mutex);
	ret = bq2589x_read_byte(&tmp, reg);
	if (ret) {
		mutex_unlock(&i2c_update_mutex);
		return ret;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = bq2589x_write_byte(reg, tmp);

	mutex_unlock(&i2c_update_mutex);
	return ret;
}

int bq2589x_sed_read_byte(u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_read_byte_data(g_bq_sed->client, reg);
	if (ret < 0) {
		dev_err(g_bq_sed->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	*data = (u8)ret;

	mutex_unlock(&i2c_mutex);
	return 0;
}
 int bq2589x_sed_write_byte(u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_write_byte_data(g_bq_sed->client, reg, data);
	if (ret < 0 ) {
		dev_err(g_bq_sed->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	mutex_unlock(&i2c_mutex);
	return ret;
}

int bq2589x_sed_update_bits(u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_update_mutex);
	ret = bq2589x_sed_read_byte(&tmp, reg);
	if (ret) {
		mutex_unlock(&i2c_update_mutex);
		return ret;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = bq2589x_sed_write_byte(reg, tmp);

	mutex_unlock(&i2c_update_mutex);
	return ret;
}

//CON0--------------------------------------
void bq2589x_charge_hiz_set_main(int enable)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK,
			enable << BQ2589X_ENHIZ_SHIFT);
}

void bq2589x_charge_hiz_set_sedcond(int enable)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK,
			enable << BQ2589X_ENHIZ_SHIFT);
}

void bq2589x_en_ilim_pin(int enable)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENILIM_MASK,
			enable << BQ2589X_ENILIM_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_00, BQ2589X_ENILIM_MASK,
			enable << BQ2589X_ENILIM_SHIFT);
}

void bq2589x_input_current_limit_second(int curr)
{
	u8 val;

	val = (curr/100 - BQ2589X_IINLIM_BASE)/BQ2589X_IINLIM_LSB;
	bq2589x_sed_update_bits(BQ2589X_REG_00,BQ2589X_IINLIM_MASK,
			val << BQ2589X_IINLIM_SHIFT);
}

void bq2589x_input_current_limit_main(int curr)
{
	u8 val = 0;
	int val_curr;

	val = (curr/100 - BQ2589X_IINLIM_BASE)/BQ2589X_IINLIM_LSB;
	bq2589x_update_bits(BQ2589X_REG_00,BQ2589X_IINLIM_MASK,
			val << BQ2589X_IINLIM_SHIFT);
}

void bq2589x_input_current_limit(int curr)
{
	bq2589x_input_current_limit_main(curr);
	bq2589x_input_current_limit_second(curr);
}

int bq2589x_adc_read_charger_volt(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_11);
	if(ret < 0) {
		dev_err(g_bq->dev,"read vbus voltage failed :%d\n",ret);
		return ret;
	} else{
		volt = BQ2589X_VBUSV_BASE +
			((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
		return volt;
	}
}

//I2C2为主IC, VBUS直接与其相连
int bq2589x_adc_start(bool oneshot)
{
    u8 val, val1;
    int ret;

    ret = bq2589x_read_byte(&val1,BQ2589X_REG_02);
    if(ret < 0){
        dev_err(g_bq_sed->dev,"%s failed to read register 0x02:%d\n",__func__,ret);
        return ret;
    }

    if(oneshot) {
        ret = bq2589x_update_bits(BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
        ret = bq2589x_update_bits(BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
        ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
        ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
    } else {
        ret = bq2589x_update_bits(BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
        ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
    }

    return ret;
}

int bq2589x_adc_stop(void)//stop continue scan 
{
	int ret;

    	ret = bq2589x_update_bits(BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK,
		    BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);

    	ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK,
		    BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
	return ret;
}

void bq2589x_charge_ico_enable(int enable)
{
	int ret;
	u8 val;

	if (enable) {
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	} else {
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;
	}
	ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);
}

//CON3--------------------------------------
void bq2589x_i2c_watchdog_reset(void)
{	
	int ret;

	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
	ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}

void bq2589x_set_sys_min(int sysv_min)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_03,BQ2589X_SYS_MINV_MASK,
			sysv_min << BQ2589X_SYS_MINV_SHIFT);
}

void bq2589x_fast_charge_limit_second(int curr)	
{
	u8 ichg;
	int ret;

	ichg = (curr/100 - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
    	ret = bq2589x_sed_update_bits(BQ2589X_REG_04,BQ2589X_ICHG_MASK,
			ichg << BQ2589X_ICHG_SHIFT);
}

void bq2589x_fast_charge_limit_main(int curr)	
{
	u8 ichg;
	int ret;

	ichg = (curr/100 - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
    	ret = bq2589x_update_bits(BQ2589X_REG_04,BQ2589X_ICHG_MASK,
			ichg << BQ2589X_ICHG_SHIFT);
}

void bq2589x_fast_charge_limit(int curr)
{
	bq2589x_fast_charge_limit_second(curr);
	bq2589x_fast_charge_limit_main(curr);
}

//CON5-------------------------------------------
void bq2589x_iprechg_current_limit(int curr)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_05, BQ2589X_IPRECHG_MASK,
			curr << BQ2589X_IPRECHG_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_05, BQ2589X_IPRECHG_MASK,
			curr << BQ2589X_IPRECHG_SHIFT);
}

void bq2589x_charge_iterm_set_main(int curr)
{
	int ret;

	// termination current
	ret = bq2589x_update_bits(BQ2589X_REG_05,BQ2589X_ITERM_MASK,
			curr << BQ2589X_ITERM_SHIFT);
}

void bq2589x_charge_iterm_set_sedcond(int curr)
{
	int ret;

	// termination current
	ret = bq2589x_sed_update_bits(BQ2589X_REG_05,BQ2589X_ITERM_MASK,
			curr << BQ2589X_ITERM_SHIFT);
}

//CON6------------------------------------------------
void bq2589x_charge_voltage_limit(int volt)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_06,BQ2589X_VREG_MASK,
			volt << BQ2589X_VREG_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_06,BQ2589X_VREG_MASK,
			volt << BQ2589X_VREG_SHIFT);
}

void bq2589x_charge_batlowv_set(int low_volt)
{
	int ret;
	// precharge voltage

        ret = bq2589x_update_bits(BQ2589X_REG_06,BQ2589X_BATLOWV_MASK,
			low_volt << BQ2589X_BATLOWV_SHIFT);

        ret = bq2589x_sed_update_bits(BQ2589X_REG_06,BQ2589X_BATLOWV_MASK,
			low_volt << BQ2589X_BATLOWV_SHIFT);
}

void bq2589x_recharge_threshold_set(int offset)
{
	int ret;
        ret = bq2589x_update_bits(BQ2589X_REG_06,BQ2589X_VRECHG_MASK,
			offset << BQ2589X_VRECHG_SHIFT);
	
        ret = bq2589x_sed_update_bits(BQ2589X_REG_06,BQ2589X_VRECHG_MASK,
			offset << BQ2589X_VRECHG_SHIFT);
}

void bq2589x_charge_timer_set(int timer)
{
	int ret;
    // charger timer
        ret = bq2589x_update_bits(BQ2589X_REG_07,BQ2589X_CHG_TIMER_MASK,
			timer << BQ2589X_CHG_TIMER_SHIFT);

        ret = bq2589x_sed_update_bits(BQ2589X_REG_07,BQ2589X_CHG_TIMER_MASK,
			timer << BQ2589X_CHG_TIMER_SHIFT);
}

void bq2589x_i2c_watchdog_timer_set(int timer)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK,
			timer << BQ2589X_WDT_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK,
			timer << BQ2589X_WDT_SHIFT);
}

void bq2589x_set_ir_comp_resistance(int val)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_08, BQ2589X_BAT_COMP_MASK,
			val << BQ2589X_BAT_COMP_SHIFT);

	ret = bq2589x_update_bits(BQ2589X_REG_08, BQ2589X_BAT_COMP_MASK,
			val << BQ2589X_BAT_COMP_SHIFT);
}

void bq2589x_set_ir_comp_voltage_clamp(int val)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_08, BQ2589X_VCLAMP_MASK,
			val << BQ2589X_VCLAMP_SHIFT);

	ret = bq2589x_update_bits(BQ2589X_REG_08, BQ2589X_VCLAMP_MASK,
			val << BQ2589X_VCLAMP_SHIFT);


}

//CON9-----------------------------------------------
void bq2589x_charge_enter_ship_mode(void)
{
	int ret;

	u8 val = (BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT);
	u8 val1 = BQ2589X_BATFET_DLY << BQ2589X_BATFET_DLY_SHIFT;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DLY_MASK, val1);
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DLY_MASK, val1);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
}

void bq2589x_charge_force_ico_set(void)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK,
			1 << BQ2589X_FORCE_ICO_SHIFT);
}

void bq2589x_pumpx_enable(int enable)
{
	int ret;
	u8 val;

	if (enable)
		val = 1 << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = 0 << BQ2589X_EN_PUMPX_SHIFT;

    	ret = bq2589x_update_bits(BQ2589X_REG_04,BQ2589X_EN_PUMPX_MASK, val);
}

void bq2589x_pumpx_up(void)
{
	int ret;
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK,
			1 << BQ2589X_PUMPX_UP_SHIFT);
}

int bq2589x_pumpx_up_done(void)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_09);
	if(val & BQ2589X_PUMPX_UP_MASK)
		return 1;   // not finished
	else
		return 0;   // pumpx up finished
}

void bq2589x_pumpx_down(void)
{
	int ret;
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_DN_MASK,
			1 << BQ2589X_PUMPX_DN_SHIFT);
}

int bq2589x_pumpx_down_done(void)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_09);
	if(val & BQ2589X_PUMPX_DN_MASK)
		return 1;   // not finished
	else
		return 0;   // pumpx up finished
}

void bq2589x_boost_mode_voltage_set(int volt)
{
    u8 val = 0;
    int ret;

    if (volt < BQ2589X_BOOSTV_BASE)
        volt = BQ2589X_BOOSTV_BASE;
    if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
        volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE)/BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;
	ret = bq2589x_update_bits(BQ2589X_REG_0A,BQ2589X_BOOSTV_MASK,val);
}

void bq2589x_boost_mode_current_limit(int curr)
{
	u8 temp;

	if(curr  == 500)
	temp = BQ2589X_BOOST_LIM_500MA;
	else if(curr == 700)
	temp = BQ2589X_BOOST_LIM_700MA;
	else if(curr == 1100)
	temp = BQ2589X_BOOST_LIM_1100MA;
	else if(curr == 1600)
	temp = BQ2589X_BOOST_LIM_1600MA;
	else if(curr == 1800)
	temp = BQ2589X_BOOST_LIM_1800MA;
	else if(curr == 2100)
	temp = BQ2589X_BOOST_LIM_2100MA;
	else if(curr == 2400)
	temp = BQ2589X_BOOST_LIM_2400MA;
	else
	temp = BQ2589X_BOOST_LIM_1300MA;
	
	bq2589x_update_bits(BQ2589X_REG_0A,BQ2589X_BOOST_LIM_MASK,
		    temp << BQ2589X_BOOST_LIM_SHIFT);
}

void bq2589x_force_vindpm(int enable)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, 
			enable << BQ2589X_FORCE_VINDPM_SHIFT);
}

void bq2589x_vindpm_threshold_set(int volt)
{
	int ret;

        ret = bq2589x_update_bits(BQ2589X_REG_0D,BQ2589X_VINDPM_MASK,
			volt << BQ2589X_VINDPM_SHIFT);
        ret = bq2589x_sed_update_bits(BQ2589X_REG_0D,BQ2589X_VINDPM_MASK,
			volt << BQ2589X_VINDPM_SHIFT);
}

void bq2589x_get_vindpm_status(int *data)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_13);

	*data = (val & BQ2589X_VDPM_STAT_MASK) >> BQ2589X_VDPM_STAT_SHIFT;
}

void bq2589x_get_iindpm_status(int *data)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_13);
	*data = (val & BQ2589X_IDPM_STAT_MASK) >> BQ2589X_IDPM_STAT_SHIFT;
}

int bq2589x_get_idpm_limit(void)
{
	int ret;
	u8 val;
	int idpm_limit = 0;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_13);
	idpm_limit = ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB + BQ2589X_IDPM_LIM_BASE;
	return idpm_limit;
}

int bq2589x_get_ico_status(void)
{
	int ret;
	u8 val;
	int ico_status;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_14);
	ico_status = (val & BQ2589X_ICO_OPTIMIZED_MASK) >> BQ2589X_ICO_OPTIMIZED_SHIFT; 

	return ico_status;
}

void bq2589x_charge_reg_reset(void)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	if(ret < 0) return;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	if(ret < 0) return;

	mdelay(100);
}

void bq2589x_charge_dump_register(void)
{
	int i=0;
	u8 regval;

	for (i=0;i<BQ2589x_REG_NUM;i++)
	{
		bq2589x_read_byte(&regval, i);
		printk("[bq2589x_dump_register_main] Reg[0x%X]=0x%X\n", i, regval);        
	}

	for (i=0;i<BQ2589x_REG_NUM;i++)
	{
		bq2589x_sed_read_byte(&regval, i);
		printk("[bq2589x_dump_register_sed] Reg[0x%X]=0x%X\n", i, regval);        
	}
}

void bq2589x_charge_enable_main(int enable)
{
	int ret;
	u8 val = enable << BQ2589X_CHG_CONFIG_SHIFT;

	if (enable) {
		/*打开charger之前，需要禁止调ILIM的硬件限流功能*/
		bq2589x_en_ilim_pin(0);
		ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
		bq2589x_charge_hiz_set_main(0);
	} else {
		ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	}
}

void bq2589x_charge_enable_sedcond(int enable)
{
	int ret;
	u8 val = enable << BQ2589X_CHG_CONFIG_SHIFT;

	if (enable) {
		pinctrl_select_state(g_bq->chgctrl, g_bq->chg1_dis_h);
		bq2589x_charge_hiz_set_sedcond(0);
		ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	} else {
		pinctrl_select_state(g_bq->chgctrl, g_bq->chg1_dis_l);
		ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
		bq2589x_charge_hiz_set_sedcond(1);
	}
}

void bq2589x_charge_enable(int enable)
{
	bq2589x_charge_enable_sedcond(enable);
	bq2589x_charge_enable_main(enable);
}

int bq2589x_enable_otg(void)
{
	int ret;
	
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;
	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);

	return ret;
}

int bq2589x_disable_otg(void)
{
	int ret;

	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;
	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);

	return ret;
}
	
void bq2589x_boost_mode_enable(int enable)
{
    int ret;

	if(enable){
	bq2589x_charge_hiz_set_main(!enable);
	msleep(10);
	 /*Set max current output */
	 bq2589x_boost_mode_current_limit(2400);
      ret = bq2589x_enable_otg();
        if(ret < 0){
            dev_err(g_bq->dev,"%s:Failed to enable otg-%d\n",__func__,ret);
            return;
        }
		bq2589x_charge_dump_register();
    }
    else{
		bq2589x_charge_dump_register();
        ret = bq2589x_disable_otg();
        if(ret < 0){
            dev_err(g_bq->dev,"%s:Failed to disable otg-%d\n",__func__,ret);
        }
    }
}

int bq2589x_get_charging_status(void)
{
	u8 valb = 0;
	int chr_stat;

	bq2589x_read_byte(&valb, BQ2589X_REG_0B);

	chr_stat = (valb & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	
	return chr_stat;
}

int bq2589x_get_vbus_status(void)
{
	u8 valb = 0;
	int vbus_stat = 0;

	bq2589x_read_byte(&valb, BQ2589X_REG_0B);

	vbus_stat = (valb & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
	return vbus_stat;
}

int bq2589x_get_pg_status(void)
{
	u8 valb = 0;
	int pg_stat = 0;

	bq2589x_read_byte(&valb, BQ2589X_REG_0B);

	pg_stat = (valb & BQ2589X_PG_STAT_MASK) >> BQ2589X_PG_STAT_SHIFT;
	return pg_stat;	
}

void bq2589x_charge_init(void)
{
	/*VINDPM Enable*/
	bq2589x_force_vindpm(1);
	/*VINDPM threshold offset*/
	bq2589x_vindpm_threshold_set(0x11);
	/*termination main charger current:192mA*/
	bq2589x_charge_iterm_set_main(0x02);
	/*termination main charger current:384mA*/
	bq2589x_charge_iterm_set_sedcond(0x05);
	/*charger voltage limit*/
	bq2589x_charge_voltage_limit(0x20);
	/*charger safety timer 8hours*/
	bq2589x_charge_timer_set(0x01);
	/*ir compensation resistance:40*/
	bq2589x_set_ir_comp_resistance(0x02);
	/* ir compensation voltage
	 the max ir compensation voltage(above Charge Voltage Limit):120MV)*/
	bq2589x_set_ir_comp_voltage_clamp(0x04);
	/*watchdog enable*/
	bq2589x_i2c_watchdog_timer_set(0x2);
	/*Rechg threshold:200MV*/
	bq2589x_recharge_threshold_set(1);
}

static void set_chr_current(kal_uint32 chr_cur_val)
{
	bq2589x_charge_enable_sedcond(0); //STOP THE I2C0 bq25892 charger IC
	bq2589x_charge_enable_main(1); //ENABLE I2C2

	bq2589x_input_current_limit_main(chr_cur_val);
}

void charge_set_current_pattern(int increase)
{
	kal_uint32 array_size; 
	kal_uint32 set_ta_on_current_reg_value = CHARGE_CURRENT_900_00_MA;
	kal_uint32 set_ta_off_current_reg_value= CHARGE_CURRENT_100_00_MA;

	//CURRENT is higher than 550mA is "1"
	//CURRENT is lower than 130mA is "0"

	if(increase == KAL_TRUE)
	{
	 printk("increase start (%d), on %d\n", set_ta_off_current_reg_value, set_ta_on_current_reg_value);

	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 // patent start
	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(100);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(100);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(300);
	 set_chr_current(set_ta_off_current_reg_value);
	 msleep(100);
	 
	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(300);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);
	 
	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(300);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(500);
	 // patent end

	 printk("mtk_ta_increase() end \n");
	}
	else    //decrease
	{
	 printk( "mtk_ta_decrease() start\n");

	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 // patent start    
	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(300);
	 set_chr_current(set_ta_off_current_reg_value);
	 msleep(100);
	 
	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(300);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(300);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);
	  
	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(100);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(100);
	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 set_chr_current(set_ta_on_current_reg_value); 
	 msleep(500);        

	 set_chr_current(set_ta_off_current_reg_value); 
	 msleep(50);
	 // patent end

	 printk("mtk_ta_decrease() end \n"); 
	}
}

void mtk_ta_reset_vchr(void)
{
	CHR_CURRENT_ENUM chr_current = CHARGE_CURRENT_70_00_MA;

	bq2589x_charge_enable_sedcond(0);
	bq2589x_input_current_limit_main(chr_current);
	msleep(250);    // reset Vchr to 5V

	printk("%s:reset Vchr to 5V , is_screen_on %d\n", __func__, BMT_status.is_screen_on);
}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	int count = 0;

	for (addr = 0x0; addr <= 0x14; addr++) {
		bq2589x_read_byte(&val, addr);
		count+=sprintf(buf+count,"[Main:0x%x] = (0x%0x)\n", 0x00+addr,val);
	}
	for (addr = 0x0; addr <= 0x14; addr++) {
		bq2589x_sed_read_byte(&val, addr);
		count+=sprintf(buf+count,"[Sedcond:0x%x] = (0x%0x)\n",0x00+addr,val);
	}

	return count;
}
static DEVICE_ATTR(regs_dump, 0644, bq2589x_show_registers, NULL);

static ssize_t bq2589x_store_cmddischarging(struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t count)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	int val = 0;

	kstrtoint(buf, 10, &val);

	cmd_discharging = val;	
	wake_up_bat();

	return count;
}
static DEVICE_ATTR(cmd_discharging, 0644, NULL, bq2589x_store_cmddischarging);


static struct attribute *bq2589x_attributes[] = {
	&dev_attr_regs_dump.attr,
	&dev_attr_cmd_discharging.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};


static void bq2589x_chg0_irq_workfunc(struct work_struct *work)
{
	u8 valb, valc;
	int vbus_stat = 0;

	wake_lock_timeout(&g_bq->charger_wake_lock, 1*HZ);

	bq2589x_read_byte(&valb, BQ2589X_REG_0B);
	bq2589x_read_byte(&valc, BQ2589X_REG_0C);
	
	vbus_stat = bq2589x_get_vbus_status();
	printk("%s:valb 0x%02x, valc 0x%02x\n", __func__, valb, valc);
	if (valb == 0x02) {
		vbus_change_event = KAL_TRUE;
		wake_up(&vbus_wq);
	} else if (vbus_stat != 0 && vbus_stat != CHARGER_OTG_MODE)
		vbus_change_event = KAL_FALSE;

	return;	
}

static irqreturn_t chg_eint_handler(int irq, void *data)
{
	schedule_delayed_work(&g_bq->irq0_dwork,HZ/5);
	return IRQ_HANDLED;
}

static void bq2589x_irq_init(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	int chg_irq;

	node = of_find_compatible_node(NULL, NULL, "mediatek,charger0-eint");

	if (node) {
		chg_irq = irq_of_parse_and_map(node, 0);

		 ret = request_threaded_irq(chg_irq, NULL, chg_eint_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"charger0-eint", g_bq);
	} else {
		printk("charger request_irq can not find touch eint device node!.");
	}

	return ret;
}

static void bq2589x_gpio_init(struct bq2589x *bq, int chip_id)
{
	int ret;

	bq->chgctrl = devm_pinctrl_get(&bq->client->dev);
	if (IS_ERR(bq->chgctrl)) {
		dev_err(&bq->dev, "Cannot find charger bq->chgctrl!");
		ret = PTR_ERR(bq);
	}

	bq->chg0_dis_l = pinctrl_lookup_state(bq->chgctrl, "chg0_dis_low_cfg");
	if (IS_ERR(bq->chg0_dis_l)) {
		ret = PTR_ERR(bq->chg0_dis_h);
		printk("%s : pinctrl err, chg0_dis_l\n", __func__);
	}
	bq->chg0_dis_h = pinctrl_lookup_state(bq->chgctrl, "chg0_dis_high_cfg");
	if (IS_ERR(bq->chg0_dis_h)) {
		ret = PTR_ERR(bq->chg0_dis_h);
		printk("%s : pinctrl err, chg0_dis_h\n", __func__);
	}
	bq->chg1_dis_l = pinctrl_lookup_state(bq->chgctrl, "chg1_dis_low_cfg");
	if (IS_ERR(bq->chg1_dis_l)) {
		ret = PTR_ERR(bq->chg1_dis_l);
		printk("%s : pinctrl err, chg1_dis_l\n", __func__);
	}
	bq->chg1_dis_h = pinctrl_lookup_state(bq->chgctrl, "chg1_dis_high_cfg");
	if (IS_ERR(bq->chg1_dis_h)) {
		ret = PTR_ERR(bq->chg1_dis_h);
		printk("%s : pinctrl err, chg1_dis_h\n", __func__);
	}
}

int fb_event_notify(struct notifier_block *this, unsigned long code,
        void *data)
{
	struct fb_event *evdata = data;
	unsigned int blank;
	kal_bool hiz_enable;

	if(code != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)(evdata->data);

	switch(blank) {
	case FB_BLANK_POWERDOWN:	//lcd power off
		wake_lock_timeout(&g_bq->fb_wake_lock, 2*HZ);
		BMT_status.is_screen_on = KAL_FALSE;
		wake_up_bat();
		break;

	case FB_BLANK_UNBLANK:
		BMT_status.is_screen_on = KAL_TRUE;
		wake_up_bat();
		break;
    }

    return NOTIFY_OK;
}

static int charge_batfet_rst_notifier(struct notifier_block *this,
		unsigned long code, void *data)
{
	if (code) {//code = 1, disable the charger batfet reset function
		bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_RST_EN_MASK,
				0 << BQ2589X_BATFET_RST_EN_SHIFT);
	} else {
		/*enable the charger batfet reset function*/
		bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_RST_EN_MASK,
				1 << BQ2589X_BATFET_RST_EN_SHIFT);
	}
	return NOTIFY_OK;
}

static struct notifier_block charge_rst_notifier = {
	.notifier_call = charge_batfet_rst_notifier,
};

static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	static struct bq2589x *bq;
	int ret;
	char *name;
	int num;
	int chip_id;
	u8 device_conf = 0;
	u8 batfet_on = 0 << BQ2589X_BATFET_DIS_SHIFT;

	bq = kzalloc(sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq){
		dev_err(&client->dev,"%s:out of memory\n",__func__);
		return -ENOMEM;
	}
	chip_id = id->driver_data;
	
	if (chip_id == BQ25892_I2C0) {

		bq->dev = &client->dev;
		bq->client = client;
		i2c_set_clientdata(client, bq);
		g_bq = bq;

		ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
		if (ret) {
			dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		}
		meizu_sysfslink_register_name(bq->dev, "charger");

		wake_lock_init(&g_bq->charger_wake_lock, WAKE_LOCK_SUSPEND, "charger wakelock");
		wake_lock_init(&g_bq->fb_wake_lock, WAKE_LOCK_SUSPEND, "fb wakelock");
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
		wake_lock_init(&TA_charger_suspend_lock, WAKE_LOCK_SUSPEND, "TA charger suspend wakelock");  
#endif
		init_waitqueue_head(&vbus_wq);
		INIT_DELAYED_WORK(&bq->irq0_dwork, bq2589x_chg0_irq_workfunc);
		bq2589x_gpio_init(bq, chip_id);
		bq2589x_irq_init();
		wake_up(&vbus_wq);

		bq->chg_fb_notifier.notifier_call = fb_event_notify;
		ret = fb_register_client(&bq->chg_fb_notifier);
		if(ret)
			pr_err("wmt register chg_fb_notifier failed! ret(%d)\n", ret);
		else
			pr_info("wmt register chg_fb_notifier OK!\n");

		ret = register_hw_reset_notifier(&charge_rst_notifier);

	} else if (chip_id == BQ25892_I2C5) {
		bq->dev = &client->dev;
		bq->client = client;
		i2c_set_clientdata(client, bq);
		g_bq_sed = bq;
	}
	chg_hw_init_done = KAL_TRUE;

	printk("[%s]chip_id %d,client addr 0x%02x\n", __func__, chip_id, client->addr);
	return 0;
err_0:
	kfree(bq);
	return ret;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
	meizu_sysfslink_unregister(bq->dev);
	unregister_hw_reset_notifier(&charge_rst_notifier);

	kfree(bq);
	g_bq_sed = NULL;
	g_bq = NULL;

	return 0;
}

static void bq2589x_shutdown(struct i2c_client *client)
{	
	mtk_ta_reset_vchr();
	bq2589x_charge_reg_reset();
}

static const struct of_device_id bq2589x_of_match[] = {
	{.compatible = "mediatek,bq2589x_main", .data = (void *)MAIN_CHARGER },
	{.compatible = "mediatek,bq2589x_sed",  .data = (void *)SED_CHARGER },
	{},
};

static const struct i2c_device_id bq2589x_id[] = {
    { "bq2589x_main", BQ25892_I2C0},
    { "bq2589x_sed", BQ25892_I2C5},
    {},
};
MODULE_DEVICE_TABLE(i2c, bq2589x_id);

static struct i2c_driver bq2589x_charger = {
	.probe		= bq2589x_charger_probe,
	.remove		= bq2589x_charger_remove,
	.shutdown	= bq2589x_shutdown,
	.id_table	= bq2589x_id,
	.driver		= {
		.name	= "bq2589x",
		.of_match_table = bq2589x_of_match,
	},
};

static int __init bq2589x_init(void)
{    
    int ret=0;

    if(i2c_add_driver(&bq2589x_charger) != 0)
    {
        printk("[bq2589x_init] failed to register bq2589x i2c driver.\n");
    } else {
        printk("[bq2589x_init] Success to register bq2589x i2c driver.\n");
    }
    
    return 0;        
}

static void __exit bq2589x_exit(void)
{
	fb_unregister_client(&g_bq->chg_fb_notifier);
	i2c_del_driver(&bq2589x_charger);
}

module_init(bq2589x_init);
module_exit(bq2589x_exit);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments, Meizu");
