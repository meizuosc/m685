/*
 * FAN54511 battery charging driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#if 0
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>

#include <mach/irqs.h>
#include <mach/eint.h>
//#include "cust_eint.h"
#include <mach/mt_clkmgr.h>
#include <linux/kthread.h>
#include <linux/earlysuspend.h>
#include <mach/battery_common.h>
//#include <linux/meizu-sys.h>
#include <linux/wakelock.h>
#include <mach/fan54511_reg.h>

#else
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include "fan54511_reg.h"
#endif


/* Please remove it in the official version */
#if 0
/* charger enable pin */
#define GPIO_CHARGER0_EN_PIN  (GPIO243 | 0x80000000)   //CHG0_DIS
#define GPIO_CHARGER1_EN_PIN  (GPIO242 | 0x80000000)   //CHG1_DIS

/*vbus over voltage protect pin*/
#define GPIO_VBUS_OV1_EN_PIN  (GPIO244 | 0x80000000)   //VBUS_OV1
#define GPIO_VBUS_OV2_EN_PIN  (GPIO251 | 0x80000000)   //VBUS_OV2

/* main charger eint pin */
#define CUST_EINT_CHARGER0_EINT_NUM 5
#define GPIO_CHARGER0_INT_PIN   (GPIO66 | 0x80000000)

/* 2sub charger eint pin */
#define CUST_EINT_CHARGER1_EINT_NUM 4
#define GPIO_CHARGER1_INT_PIN   (GPIO65 | 0x80000000)

/* gpio mode */
#define GPIO_CHARGER0_PIN_M_GPIO GPIO_MODE_00
#define GPIO_CHARGER1_PIN_M_GPIO GPIO_MODE_00
#define GPIO_VBUS_OV1_PIN_M_GPIO GPIO_MODE_00
#define GPIO_VBUS_OV2_EN_PIN_M_GPIO GPIO_MODE_00

#endif

/******************************************************************************
* Register bits	
******************************************************************************/
/*    REG_STAT0 (0x00)    */
#define STAT0_NOBAT					0x01
#define STAT0_SLP_SHIFT				1
#define STAT0_SLP					(0x01 << STAT0_SLP_SHIFT)
#define STAT0_PRE_SHIFT				2
#define STAT0_PRE 					(0x01 << STAT0_PRE_SHIFT)
#define STAT0_STAT_SHIFT			3
#define STAT0_STAT 					(0x01 << STAT0_STAT_SHIFT)
#define STAT0_PWROK_SHIFT			4
#define STAT0_PWROK 				(0x01 << STAT0_PWROK_SHIFT)
#define STAT0_VBUSPWR_SHIFT			5
#define STAT0_VBUSPWR 				(0x01 << STAT0_VBUSPWR_SHIFT)
#define STAT0_VINPWR_SHIFT			6
#define STAT0_VINPWR 				(0x01 << STAT0_VINPWR_SHIFT)
#define STAT0_BATID_SHIFT 			7
#define STAT0_BATID 				(0x01 << STAT0_BATID_SHIFT)

/*    REG_STAT1 (0x01)    */
#define STAT1_BATLO					0x01
#define STAT1_BOOST_SHIFT			1
#define STAT1_BOOST 				(0x01 << STAT1_BOOST_SHIFT)  
#define STAT1_DIVC_SHIFT 			2
#define STAT1_DIVC 					(0x01 << STAT1_DIVC_SHIFT)
#define STAT1_TOCHG_SHIFT			3
#define STAT1_TOCHG 				(0x01 << STAT1_TOCHG_SHIFT)
#define STAT1_CHGCMP_SHIFT			4
#define STAT1_CHGCMP 				(0x01 << STAT1_CHGCMP_SHIFT)
#define STAT1_CHGDET_SHIFT 			5
#define STAT1_CHGDET 				(0x03 << STAT1_CHGDET_SHIFT)
#define STAT1_LOIBAT_SHIFT 			7
#define STAT1_LOIBAT  				(0x01 << STAT1_LOIBAT_SHIFT)

/*    REG_STAT2 (0x02)    */
#define STAT2_TMRTO					0x01
#define STAT2_WDTTO_SHIFT			1
#define STAT2_WDTTO 				(0x01 << STAT2_WDTTO_SHIFT)
#define STAT2_TBAT_SHIFT			2
#define STAT2_TBAT 					(0x01 << STAT2_TBAT_SHIFT)
#define STAT2_JEITA_SHIFT 			3
#define STAT2_JEITA 				(0x01 << STAT2_JEITA_SHIFT)
#define STAT2_TEMPFB_SHIFT 			4
#define STAT2_TEMPFB 				(0X01 << STAT2_TEMPFB_SHIFT)
#define STAT2_TEMPSD_SHIFT 			5
#define STAT2_TEMPSD 				(0x01 << STAT2_TEMPSD_SHIFT)
#define STAT2_INPUTOVP_SHIFT 		6
#define STAT2_INPUTOVP 				(0x01 << STAT2_INPUTOVP_SHIFT)
#define STAT2_INPUTSEL_SHIFT 		7
#define STAT2_INPUTSEL 				(0x01 << STAT2_INPUTSEL_SHIFT)

/*    REG_INT0 (0x04)    */
#define INT0_BATINT 				0x01
#define INT0_WKBAT					0x02
#define INT0_CHGMOD					0x04
#define INT0_CHGEND					0x08
#define INT0_VLOWTH					0x10
#define INT0_VBUSINT				0x20
#define INT0_VININT					0x40
#define INT0_VALFAIL				0x80

/*    REG_INT1 (0x05)    */
#define INT1_VBATLV					0x01
#define INT1_BSTWDTTO				0x02
#define INT1_BATUVL					0x04
#define INT1_BSTFAIL				0x08
#define INT1_BSTTSD					0x10
#define INT1_BSTOVP					0x20
#define INT1_RCHGN					0x40
#define INT1_IBATLO					0x80

/*    REG_INT2 (0x06)    */
#define INT2_TIMER					0x01
#define INT2_BATOCP					0x02
#define INT2_OTGOCP					0x04
#define INT2_BATTEMP				0x08
#define INT2_ICTEMP					0x10
#define INT2_SHORTBAT				0x20
#define INT2_OVPINPUT				0x40
#define INT2_TOCMP					0x80

/*    REG_MINT0 (0x08)    */
#define MINT0_BATINT 				0x01
#define MINT0_WKBAT					0x02
#define MINT0_CHGMOD				0x04
#define MINT0_CHGEND				0x08
#define MINT0_VLOWTH				0x10
#define MINT0_VBUSINT				0x20
#define MINT0_VININT				0x40
#define MINT0_VALFAIL				0x80

/*    REG_MINT1 (0x09)    */
#define MINT1_VBATLV				0x01
#define MINT1_BSTWDTTO				0x02
#define MINT1_BATUVL				0x04
#define MINT1_BSTFAIL				0x08
#define MINT1_BSTTSD				0x10
#define MINT1_BSTOVP				0x20
#define MINT1_RCHGN					0x40
#define MINT1_IBATLO				0x80

/*    REG_MINT2 (0x0A)    */
#define MINT2_TIMER					0x01
#define MINT2_BATOCP				0x02
#define MINT2_OTGOCP				0x04
#define MINT2_BATTEMP				0x08
#define MINT2_ICTEMP				0x10
#define MINT2_SHORTBAT				0x20
#define MINT2_OVPINPUT				0x40
#define MINT2_TOCMP					0x80

/*    REG_CON0 (0x0C)    */
#define CON0_VBATMIN				0x07
#define CON0_VLOWV_SHIFT			3
#define CON0_VLOWV 					(0x07 << CON0_VLOWV_SHIFT)

/*    REG_CON1 (0x0D)    */
#define CON1_VSYS					0x07
#define CON1_VLDO_SHIFT				3
#define CON1_VLDO 					(0x03 << CON1_VLDO_SHIFT)
#define CON1_LDOOFF_SHIFT			5
#define CON1_LDOOFF 				(0x01 << CON1_LDOOFF_SHIFT)
#define CON1_GPO1_SHIFT 			6
#define CON1_GPO1 					(0x01 << CON1_GPO1_SHIFT)
#define CON1_GPO2_SHIFT 			7
#define CON1_GPO2 					(0x01 << CON1_GPO2_SHIFT)

/*    REG_CON2 (0x0E)    */
#define CON2_HZMOD_SHIFT 			1
#define CON2_HZMOD 					(0x01 << CON2_HZMOD_SHIFT)
#define CON2_TOEN_SHIFT 			2
#define CON2_TOEN 					(0x01 << CON2_TOEN_SHIFT)
#define CON2_TE_SHIFT 				3
#define CON2_TE 					(0x01 << CON2_TE_SHIFT)
#define CON2_NOBATOP_SHIFT 			4
#define CON2_NOBATOP 				(0x01 << CON2_NOBATOP_SHIFT)
#define CON2_RCHGDIS_SHIFT 			5
#define CON2_RCHGDIS 				(0x01 << CON2_RCHGDIS_SHIFT)
#define CON2_CONT_SHIFT 			7
#define CON2_CONT 					(0x01 << CON2_CONT_SHIFT)

/*    REG_CON3 (0x0F)    */
#define CON3_CE					0x01
#define CON3_PPOFF_SHIFT 			1
#define CON3_PPOFF 					(0x01 << CON3_PPOFF_SHIFT)
#define CON3_PPOFFSLP_SHIFT 		2
#define CON3_PPOFFSLP 				(0x01 << CON3_PPOFFSLP_SHIFT)
#define CON3_TREGTH_SHIFT 			5
#define CON3_TREGTH 				(0x03 << CON3_TREGTH_SHIFT)
#define CON3_RESET_SHIFT 				7
#define CON3_RESET 					(0x01 << CON3_RESET_SHIFT)

/*    REG_VFLOAT (0x)    */

/*    REG_IOCHRG (0x)    */
#define IOCHRG_VAL 					0x3F

/*    REG_IBAT (0x13)    */
#define IBAT_PRECHG 				0x0F
#define IBAT_ITERM_SHIFT 			4
#define IBAT_ITERM 					(0x0F << IBAT_ITERM_SHIFT)

/*    REG_IBUS (0x14)    */
#define IBUS_LIM 					0x7F

/*    REG_VBUS (0x15)    */
#define VBUS_LIM 					0x0F
#define VBUS_OVP_SHIFT 				4
#define VBUS_OVP 					(0x03 << VBUS_OVP_SHIFT)

/*    REG_IIN (0x16)    */
#define IIN_VAL 					0x7F

/*    REG_VIN (0x17)    */
#define VIN_LIM						0x0F
#define VIN_OVP_SHIFT 				4
#define VIN_OVP 					(0x03 << VIN_OVP_SHIFT)

/*    REG_NTC (0x18)    */
#define NTC_1 						0x01
#define NTC_2_SHIFT 				1
#define NTC_2 						(0x01 << NTC_2_SHIFT)
#define NTC_3_SHIFT 				2
#define NTC_3 						(0x01 << NTC_3_SHIFT)
#define NTC_4_SHIFT 				3
#define NTC_4 						(0x01 << NTC_4_SHIFT)
#define NTC_OK_SHIFT 				4
#define NTC_OK 						(0x01 << NTC_OK_SHIFT
#define NTC_TEMPDIS_SHIFT			5
#define NTC_TEMPDIS 				(0x01 << NTC_TEMPDIS_SHIFT)

/*    REG_TIMER (0x19)    */
#define TIMER_FCTMR 				0x03
#define TIMER_PRETMR_SHIFT			3
#define TIMER_PRETMR 				(0x03 << TIMER_PRETMR_SHIFT)
#define TIMER_WDEN_SHIFT 			6 
#define TIMER_WDEN 					(0x01 << TIMER_WDEN_SHIFT)
#define TIMER_TMRRST_SHIFT 			7
#define TIMER_TMRRST 				(0x01 << TIMER_TMRRST_SHIFT)

/*    REG_SAFETY (0x1A)    */
#define SAFETY_I 					0x0F
#define SAFETY_V_SHIFT 				4
#define SAFETY_V 					(0x0F << 4)

/*    REG_TOPOFF (0x1B)    */
#define TOPOFF_TMR 					0x07

/*    REG_BOOST (0x1C)    */
#define BOOST_V 					0x1F
#define BOOST_EN_SHIFT 				5
#define BOOST_EN 					(0x01 << BOOST_EN_SHIFT)
#define BOOST_OTG_SHIFT 			6
#define BOOST_OTG 					(0x01 << BOOST_OTG_SHIFT)

/*    REG_DPLUS (0x1F)    */
#define DPLUS_SETTMR0 				0x01
#define DPLUS_BC12DET_SHIFT 		7
#define DPLUS_BC12DET 				(0x01 << DPLUS_BC12DET_SHIFT)

/*    REG_MNT0 (0x20)    */
#define MNT0_CV 					0x01
#define MNT0_ICHG 					0x02
#define MNT0_IBUS 					0x04
#define MNT0_HIVBAT 				0x08
#define MNT0_BATSHORT 				0x10
#define MNT0_VLOWVCMP 				0x20
#define MNT0_VBATCMP 				0x40
#define MNT0_ITERMCMP				0x80

/*    REG_MNT1 (0x21)    */
#define MNT1_ILINPIN				0x01
#define MNT1_DISPIN 				0x02
#define MNT1_NTCGND 				0x04
#define MNT1_VBUSCMP 				0x08
#define MNT1_BUCKON 				0x10
#define MNT1_PPON 					0x20
#define MNT1_PMIDVBAT 				0x40

/*    REG_IC_INFO (0x2D)    */
#define IC_INFO_REV 				0x07
#define IC_INFO_PN_SHIFT 			3
#define IC_INFO_PN 					(0x07 << IC_INFO_PN_SHIFT)
#define IC_INFO_VC_SHIFT 			6
#define IC_INFO_VC 					(0x03 << IC_INFO_VC_SHIFT)

/*    REG_FEAT_CON (0x30)    */
#define FEAT_CON_ENREF_SHIFT		4
#define FEAT_CON_ENREF 				(0x01 << FEAT_CON_ENREF_SHIFT)
#define FEAT_CON_DIVCON_SHIFT		5
#define FEAT_CON_DIVCON				(0x01 << FEAT_CON_DIVCON_SHIFT)

/******************************************************************************/

#ifdef CONFIG_OF
#else
#define FAN54511_SLAVE_ADDR 0x6B
#endif

#define FAN54511_BUSNUM0 0
#define FAN54511_BUSNUM5 5

kal_bool fast_charge_current_1c = KAL_TRUE;
static kal_bool power_good = KAL_TRUE;

static DEFINE_MUTEX(i2c_mutex);
static DEFINE_MUTEX(i2c_update_mutex);
static DEFINE_MUTEX(handler_mutex);

extern  void mtk_ta_reset_vchr(void);


struct fan54511 {
	struct device *dev;
	struct i2c_client *client;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend fast_handler;
#endif
	struct delayed_work irq_dwork;
	struct wake_lock charger_wake_lock;
};
	
static struct fan54511 *g_main = NULL;
static struct fan54511 *g_sub = NULL;

/* GPIO Pin control*/
struct pinctrl *chgctrl = NULL;
struct pinctrl_state *chg0_dis_h = NULL; /* main charger gpio */
struct pinctrl_state *chg0_dis_l = NULL;
struct pinctrl_state *chg1_dis_h = NULL; /* 2nd charger gpio */
struct pinctrl_state *chg1_dis_l = NULL;
struct pinctrl_state *vbus_ov1_h = NULL; /* vbus ov1 gpio */
struct pinctrl_state *vbus_ov1_l = NULL;
struct pinctrl_state *vbus_ov2_h = NULL; /* vbus ov2 gpio */
struct pinctrl_state *vbus_ov2_l = NULL;


kal_bool chargin_hw_init_done = KAL_TRUE;
static DEFINE_MUTEX(chgreset_mutex);
static DEFINE_MUTEX(chgreset1_mutex);



void charger0_en_poweron(int en);
void charger1_en_poweron(int en);
void vbus_ov1_en_poweron(int en);
void vbus_ov2_en_poweron(int en);
 

unsigned int fan54511_main_read_byte(unsigned char *returnData, unsigned char cmd)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&i2c_mutex);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = g_main->client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = g_main->client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(g_main->client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", g_main->client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&i2c_mutex);

	return ret == xfers ? 1 : -1;
}

unsigned int fan54511_main_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&i2c_mutex);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = g_main->client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(g_main->client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", g_main->client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&i2c_mutex);

	return ret == xfers ? 1 : -1;
}


int fan54511_main_update_bits(u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_update_mutex);
	ret = fan54511_main_read_byte(&tmp, reg);
    
	tmp &= ~mask;
	tmp |= data & mask;

	ret = fan54511_main_write_byte(reg, tmp);
    
	mutex_unlock(&i2c_update_mutex);
	return ret;
}


unsigned int fan54511_sub_read_byte(unsigned char *returnData, unsigned char cmd)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&i2c_mutex);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = g_sub->client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = g_sub->client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(g_sub->client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", g_sub->client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&i2c_mutex);

	return ret == xfers ? 1 : -1;
}

unsigned int fan54511_sub_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&i2c_mutex);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = g_sub->client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(g_sub->client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			battery_log(BAT_LOG_CRTI, "skipping non-existent adapter %s\n", g_sub->client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&i2c_mutex);

	return ret == xfers ? 1 : -1;
}



int fan54511_sub_update_bits(u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_update_mutex);
	ret = fan54511_sub_read_byte(&tmp, reg);


	tmp &= ~mask;
	tmp |= data & mask;

	ret = fan54511_sub_write_byte(reg, tmp);

	mutex_unlock(&i2c_update_mutex);
	return ret;
}

//Control0 0CH
int fan54511_set_vlowv(u8 data)
{
	int ret = -1;
	ret = fan54511_main_update_bits(FAN54511_REG_CONTROL0, CON0_VLOWV,
		data << CON0_VLOWV_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_CONTROL0, CON0_VLOWV,
		data << CON0_VLOWV_SHIFT);
	return ret;
	
}

int fan54511_set_vbatmin(u8 data)
{
	int ret = -1;
	ret = fan54511_main_update_bits(FAN54511_REG_CONTROL0, CON0_VBATMIN, data);
	ret = fan54511_sub_update_bits(FAN54511_REG_CONTROL0, CON0_VBATMIN, data);
	return ret;
}

//Control 1
int fan54511_main_set_GPO2(u8 level)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL1, CON1_GPO2,
		level << CON1_GPO2_SHIFT);
}
int fan54511_sub_set_GPO2(u8 level)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL1, CON1_GPO2,
		level << CON1_GPO2_SHIFT);
}

int fan54511_main_set_GPO1(u8 level)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL1, CON1_GPO1,
		level << CON1_GPO1_SHIFT);
}

int fan54511_sub_set_GPO1(u8 level)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL1, CON1_GPO1,
		level << CON1_GPO1_SHIFT);
}

int fan54511_main_set_LDOOFF(u8 data)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL1, CON1_LDOOFF, 
		data << CON1_LDOOFF_SHIFT);
}

int fan54511_sub_set_LDOOFF(u8 data)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL1, CON1_LDOOFF,
		data << CON1_LDOOFF_SHIFT);
}

int fan54511_main_set_VLDO(int mv)
{
	u8 data;
	switch (mv)
	{
	case 3300:
		data = 0b00;
		break;
	case 3600:
		data = 0b01;
		break;
	case 4950:
		data = 0b10;
		break;		
	case 5050:
	default:
		data = 0b11;
		break;		
	}
	return fan54511_main_update_bits(FAN54511_REG_CONTROL1, CON1_VLDO, 
		data << CON1_VLDO_SHIFT);
}

int fan54511_sub_set_VLDO(int mv)
{
	u8 data;
	switch (mv)
	{
	case 3300:
		data = 0b00;
		break;
	case 3600:
		data = 0b01;
		break;
	case 4950:
		data = 0b10;
		break;		
	case 5050:
	default:
		data = 0b11;
		break;		
	}
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL1, CON1_VLDO,
		data << CON1_VLDO_SHIFT);
}

int fan54511_set_VSYS(u8 vsys)
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_CONTROL1, CON1_VSYS, vsys);
	ret = fan54511_sub_update_bits(FAN54511_REG_CONTROL1, CON1_VSYS, vsys);
	return ret;
}

//Control 2
int fan54511_set_CONT(u8 val)
{
	int ret = -1;
	ret = fan54511_main_update_bits(FAN54511_REG_CONTROL2, CON2_CONT, 
		val << CON2_CONT_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_CONTROL2, CON2_CONT,
		val << CON2_CONT_SHIFT);
	return ret;
}

int fan54511_main_set_RCHGDIS(u8 val)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL2, CON2_RCHGDIS,
		val << CON2_RCHGDIS_SHIFT);
}

int fan54511_sub_set_RCHGDIS(u8 val)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL2, CON2_RCHGDIS,
		val << CON2_RCHGDIS_SHIFT);
}

int fan54511_main_set_NOBATOP(u8 val)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL2, CON2_NOBATOP,
		val << CON2_NOBATOP_SHIFT);
}

int fan54511_sub_set_NOBATOP(u8 val)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL2, CON2_NOBATOP,
		val << CON2_NOBATOP_SHIFT);
}

int fan54511_set_TE(u8 en)
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_CONTROL2, CON2_TE,
		en << CON2_TE_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_CONTROL2, CON2_TE,
		en << CON2_TE_SHIFT);
	return ret;
}

int fan54511_main_set_TOEN(u8 en)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL2, CON2_TOEN,
		en << CON2_TOEN_SHIFT);
}

int fan54511_sub_set_TOEN(u8 en)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL2, CON2_TOEN,
		en << CON2_TOEN_SHIFT);
}

int fan54511_main_set_HZMODE(u8 en)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL2, CON2_HZMOD,
		en << CON2_HZMOD_SHIFT);
}

int fan54511_sub_set_HZMODE(u8 en)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL2, CON2_HZMOD, 
		en << CON2_HZMOD_SHIFT);
}

//Control 3
int fan54511_set_reset()
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_CONTROL3, CON3_RESET, 1);
	ret = fan54511_main_update_bits(FAN54511_REG_CONTROL3, CON3_RESET, 1);
	return ret;
}

/* 
 *   0 : 70,  1 : 85, 2: 100, 3 : 120
 */
int fan54511_main_tregth(u8 data)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL3, CON3_TREGTH,
		data << CON3_TREGTH_SHIFT);
}
int fan54511_sub_tregth(u8 data)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL3, CON3_TREGTH,
		data << CON3_TREGTH_SHIFT);
}

int fan54511_main_PPOFFSLP(u8 data)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL3, CON3_PPOFFSLP,
		data << CON3_PPOFFSLP_SHIFT);
}

int fan54511_sub_PPOFFSLP(u8 data)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL3, CON3_PPOFFSLP,
		data << CON3_PPOFFSLP_SHIFT);
}

int fan54511_main_PPOFF(u8 data)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL3, CON3_PPOFF,
		data << CON3_PPOFF_SHIFT);
}

int fan54511_sub_PPOFF(u8 data)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL3, CON3_PPOFF,
		data << CON3_PPOFF_SHIFT);
}

int fan54511_main_set_CE(u8 dis)
{
	return fan54511_main_update_bits(FAN54511_REG_CONTROL3, CON3_CE, dis);
}

int fan54511_sub_set_CE(u8 dis)
{
	return fan54511_sub_update_bits(FAN54511_REG_CONTROL3, CON3_CE, dis);
}

//VFloat 11H
int fan54511_set_VFLOAT(u8 data)
{
	int ret;
	ret = fan54511_main_write_byte(FAN54511_REG_VFLOAT, data);
	ret = fan54511_sub_write_byte(FAN54511_REG_VFLOAT, data);
	return ret;
}

//IOCHRG 12H
int fan54511_main_set_IOCHRG(int mA)
{
    printk(KERN_EMERG "++++++++fan54511_main_set_IOCHRG :  mA = %d\n",mA);

	int data;
	if (mA < 200)
		mA = 200;
	else if (mA > 3200)
		mA = 3200;

	data = (mA -200) /50;
        printk(KERN_EMERG "++++++++fan54511_main_set_IOCHRG :  data = 0x%x\n",data);
	return fan54511_main_write_byte(FAN54511_REG_IOCHRG, data);
}
int fan54511_sub_set_IOCHRG(int mA)
{    printk(KERN_EMERG "++++++++fan54511_sub_set_IOCHRG :  mA = %d\n",mA);
	int data;
	if (mA < 200)
		mA = 200;
	else if (mA > 3200)
		mA = 3200;
	data = (mA - 200) / 50;
            printk(KERN_EMERG "++++++++fan54511_sub_set_IOCHRG :  data = 0x%x\n",data);
	return fan54511_sub_write_byte(FAN54511_REG_IOCHRG, data);
}

//IBAT 13H
int fan54511_set_ITERM(u8 data)
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_IBAT, IBAT_ITERM, 
		data << IBAT_ITERM_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_IBAT, IBAT_ITERM, 
		data << IBAT_ITERM_SHIFT);
	return ret;
}

int fan54511_set_PRECHG(u8 data)
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_IBAT, IBAT_PRECHG, data);
	ret = fan54511_sub_update_bits(FAN54511_REG_IBAT, IBAT_PRECHG, data);
	return ret;
}

//IBUS 14H
int fan54511_main_set_IBUSLIM(int mA)
{
	int data;
	if (mA < 100)
		mA = 100;
	else if (mA > 3000)
		mA = 3000;
	
	data = (mA - 100) / 25;
           printk("%s:**********ibuslim_current_value: 0x14 = 0x%x mA***************\n", __func__, data);
           
	return fan54511_main_update_bits(FAN54511_REG_IBUS, IBUS_LIM, data);
}

int fan54511_sub_set_IBUSLIM(int mA)
{
	int data;
	if (mA < 100)
		mA = 100;
	else if (mA > 3000)
		mA = 3000;
	
	data = (mA - 100) / 25;
           printk("%s:**********ibuslim_current_value: 0x14 = 0x%x mA***************\n", __func__, data);
	return fan54511_sub_update_bits(FAN54511_REG_IBUS, IBUS_LIM, data);
}

//VBUS 15H
//0 : 6.5V, 1 : 10.5V, 2: 13.7V
int fan54511_main_VBUSOVP(u8 data)
{
	return fan54511_main_update_bits(FAN54511_REG_VBUS, VBUS_OVP,
		data << VBUS_OVP_SHIFT);
}

int fan54511_sub_VBUSOVP(u8 data)
{
	return fan54511_sub_update_bits(FAN54511_REG_VBUS, VBUS_OVP,
		data << VBUS_OVP_SHIFT);
}

int fan54511_set_VBUSLIM(u8 data)
{
	int ret;
	ret =  fan54511_main_update_bits(FAN54511_REG_VBUS, VBUS_LIM, data);
	ret =  fan54511_sub_update_bits(FAN54511_REG_VBUS, VBUS_LIM, data);
	return ret;
}

//IIN 16H
int fan54511_main_IINLIM(int mA)
{
	int data = (mA - 325)/25;
	return fan54511_main_update_bits(FAN54511_REG_IIN, IIN_VAL, data);
}

int fan54511_sub_IINLIM(int mA)
{
	int data = (mA - 325)/25;
	return fan54511_sub_update_bits(FAN54511_REG_IIN, IIN_VAL, data);
}

//VIN 17H
//0 : 6.5V, 1 : 10.5V, 2: 13.7V
int fan54511_main_VINOVP(u8 data)
{
	return fan54511_main_update_bits(FAN54511_REG_VIN, VIN_OVP, 
		data << VIN_OVP_SHIFT);
}

int fan54511_sub_VINOVP(u8 data)
{
	return fan54511_sub_update_bits(FAN54511_REG_VIN, VIN_OVP, 
		data << VIN_OVP_SHIFT);
}

int fan54511_main_VINLIM(int mV)
{
	int data;
	if (mV <= 4800)
		data = (mV - 4240) / 80;
	else
		data = (mV - 7632)/ 144 + 7;
	
	return fan54511_main_update_bits(FAN54511_REG_VIN, VIN_LIM, data);
}

int fan54511_sub_VINLIM(int mV)
{
	int data;
	if (mV <= 4800)
		data = (mV - 4240) / 80;
	else
		data = (mV - 7632)/ 144 + 7;

	return fan54511_sub_update_bits(FAN54511_REG_VIN, VIN_LIM, data);
}

//NTC 18H
int fan54511_main_TEMPDIS(u8 data)
{
	return fan54511_main_update_bits(FAN54511_REG_NTC, NTC_TEMPDIS, 
		data << NTC_TEMPDIS_SHIFT);
}

int fan54511_sub_TEMPDIS(u8 data)
{
	return fan54511_sub_update_bits(FAN54511_REG_NTC, NTC_TEMPDIS,
		data << NTC_TEMPDIS_SHIFT);
}

/* Timer 19H */
int fan54511_set_TMRRST()
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_TIMER, TIMER_TMRRST, 
		1 << TIMER_TMRRST_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_TIMER, TIMER_TMRRST,
		1 << TIMER_TMRRST_SHIFT);
	return ret;
}
int fan54511_set_WDEN(u8 en)
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_TIMER, TIMER_WDEN, 
		en << TIMER_WDEN_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_TIMER, TIMER_WDEN,
		en << TIMER_WDEN_SHIFT);
	return ret;
}

/* 0 : useFCTMR, 1: 100 seconds,  2: 15 minutes, 3 : 36 minutes */
int fan54511_set_PRETMR(u8 data)
{
	int ret = -1;
	ret = fan54511_main_update_bits(FAN54511_REG_TIMER, TIMER_PRETMR,
		data << TIMER_PRETMR_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_TIMER, TIMER_PRETMR, 
		data << TIMER_PRETMR_SHIFT);
	return ret;
}

int fan54511_set_FCTMR(u8 data)
{
	int ret = -1;
	ret = fan54511_main_update_bits(FAN54511_REG_TIMER, TIMER_FCTMR, data);
	ret = fan54511_sub_update_bits(FAN54511_REG_TIMER, TIMER_FCTMR, data);
	return ret;	
}

/* SAFETY 1AH*/
int fan54511_set_SAFE(u8 data)
{
	int ret = -1;
	ret = fan54511_main_write_byte(FAN54511_REG_SAFETY, data);
	ret = fan54511_sub_write_byte(FAN54511_REG_SAFETY, data);
	return ret;
}

/* TOPOFF */
int fan54511_main_TOTMR(u8 mins)
{
	u8 data = mins / 10;
	return fan54511_main_update_bits(FAN54511_REG_TOPOFF, TOPOFF_TMR, data);
}

int fan54511_sub_TOTMR(u8 mins)
{
	u8 data = mins / 10;
	return fan54511_sub_update_bits(FAN54511_REG_TOPOFF, TOPOFF_TMR, data);
}

/* BOOST */
int fan54511_set_OTG(u8 en)
{
	int ret;
	ret = fan54511_main_update_bits(FAN54511_REG_BOOST, BOOST_OTG, 
		en << BOOST_OTG_SHIFT);
	return ret;
}

int fan54511_main_BOOSTEN(u8 en)
{
	return fan54511_main_update_bits(FAN54511_REG_BOOST, BOOST_EN,
		en << BOOST_EN_SHIFT);
}
int fan54511_sub_BOOSTEN(u8 en)
{
	return fan54511_sub_update_bits(FAN54511_REG_BOOST, BOOST_EN,
		en << BOOST_EN_SHIFT);
}

int fan54511_main_set_VBOOST(int mV)
{
	int data;
	if ( mV < 4520)
		mV = 4520;
	else if (mV > 5347)
		mV = 5347;
			
	data = (mV - 4520)*100/2667;
	return fan54511_main_update_bits(FAN54511_REG_BOOST, BOOST_V, data);
}

int fan54511_sub_set_VBOOST(int mV)
{
	int data;
	if ( mV < 4520)
		mV = 4520;
	else if (mV > 5347)
		mV = 5347;
			
	data = (mV - 4520)*100/2667;
	return fan54511_main_update_bits(FAN54511_REG_BOOST, BOOST_V, data);
}

/* Feature Control 30H */
int fan54511_set_DIVCON(u8 en)
{
	int ret = -1;
	ret = fan54511_main_update_bits(FAN54511_REG_FEATURE_CONTROL, FEAT_CON_DIVCON, 
		en << FEAT_CON_DIVCON_SHIFT);
	ret = fan54511_sub_update_bits(FAN54511_REG_FEATURE_CONTROL, FEAT_CON_DIVCON, 
		en << FEAT_CON_DIVCON_SHIFT);
	return ret;
}

int fan54511_main_ENREF(u8 dis)
{
	int ret = -1;
	ret = fan54511_main_update_bits(FAN54511_REG_FEATURE_CONTROL, FEAT_CON_ENREF,
		dis << FEAT_CON_ENREF_SHIFT);
	ret = fan54511_main_update_bits(FAN54511_REG_FEATURE_CONTROL, FEAT_CON_ENREF,
		dis << FEAT_CON_ENREF_SHIFT);
	return ret;
}

//main
void fan54511_main_start_charging(void)
{
	//bq2589x_set_en_ilim(0);
	//mt_set_gpio_out(GPIO_CHARGER0_EN_PIN, 0);
	charger0_en_poweron(0);
	fan54511_main_set_CE(0);
}

//sub
void fan54511_sub_start_charging(void)
{
	//mt_set_gpio_out(GPIO_CHARGER1_EN_PIN, 1);
	charger1_en_poweron(1);
	fan54511_sub_set_CE(0);
}

//main
void fan54511_main_stop_charging(void)
{
	//mt_set_gpio_out(GPIO_CHARGER0_EN_PIN, 1);
            charger0_en_poweron(1);
	fan54511_main_set_CE(1);
}

//sub
void fan54511_sub_stop_charging(void)
{
	//mt_set_gpio_out(GPIO_CHARGER1_EN_PIN, 0);	
	charger1_en_poweron(0);
	fan54511_sub_set_CE(1);
}

int fan54511_get_charging_status(void)
{
    u8 val = 0;
    int ret;

    ret = fan54511_main_read_byte(&val, FAN54511_REG_STATUS0);
    if(ret < 0){
        dev_err(g_main->dev,"%s Failed to read register 0x0b:%d\n",__func__,ret);
        return ret;
    }
    val &= STAT0_STAT;
    val >>= STAT0_STAT_SHIFT;
    return val;
}

int fan54511_get_pg_stat(void)
{
	int ret;	
	u8 val;

	ret = fan54511_sub_read_byte(&val, FAN54511_REG_STATUS0);
	if(ret < 0){
		dev_err(g_sub->dev,"%s Failed to read register 0x0b:%d\n",__func__,ret);
		return ret;
	}

	val &= STAT0_PWROK;
	val >>= STAT0_PWROK_SHIFT;
	return val;
}

void fan54511_dump_register(void)
{
	u8 reg_addrs[] = {
		FAN54511_REG_STATUS0,
		FAN54511_REG_STATUS1,
		FAN54511_REG_STATUS2,
		FAN54511_REG_INT0,
		FAN54511_REG_INT1,
		FAN54511_REG_INT2,
		FAN54511_REG_MINT0,
		FAN54511_REG_MINT1,
		FAN54511_REG_MINT2,
		FAN54511_REG_CONTROL0,
		FAN54511_REG_CONTROL1,
		FAN54511_REG_CONTROL2,
		FAN54511_REG_CONTROL3,
		FAN54511_REG_VFLOAT,
		FAN54511_REG_IOCHRG,
		FAN54511_REG_IBAT,
		FAN54511_REG_IBUS,
		FAN54511_REG_VBUS,
		FAN54511_REG_IIN,
		FAN54511_REG_VIN,
		FAN54511_REG_NTC,
		FAN54511_REG_TIMER,
		FAN54511_REG_SAFETY,
		FAN54511_REG_TOPOFF,
		FAN54511_REG_BOOST,
		FAN54511_REG_DPLUS,
		FAN54511_REG_MONITOR0,
		FAN54511_REG_MONITOR1,
		FAN54511_REG_IC_INFO,
		FAN54511_REG_FEATURE_CONTROL
		};
	int arrSize = sizeof(reg_addrs)/sizeof(reg_addrs[0]);
	u8 addr;
	u8 val;
    
            printk("[fan54511_dump_register] main charger reg:\n");
            
	for (addr = 0; addr < arrSize; addr++) {
		fan54511_main_read_byte(&val, reg_addrs[addr]);
		printk("[fan54511_dump_register] Reg[0x%X]=0x%X\n", reg_addrs[addr], val);
	}
            printk("\n**************************************************\n");
            printk("[fan54511_dump_register] 2nd charger reg:\n");

	for (addr = 0; addr < arrSize; addr++) {
		fan54511_sub_read_byte(&val, reg_addrs[addr]);
		printk("[fan54511_dump_register] Reg[0x%X]=0x%X\n", reg_addrs[addr], val);
	}
}

static ssize_t fan54511_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 reg_addrs[] = {
		FAN54511_REG_STATUS0,
		FAN54511_REG_STATUS1,
		FAN54511_REG_STATUS2,
		FAN54511_REG_INT0,
		FAN54511_REG_INT1,
		FAN54511_REG_INT2,
		FAN54511_REG_MINT0,
		FAN54511_REG_MINT1,
		FAN54511_REG_MINT2,
		FAN54511_REG_CONTROL0,
		FAN54511_REG_CONTROL1,
		FAN54511_REG_CONTROL2,
		FAN54511_REG_CONTROL3,
		FAN54511_REG_VFLOAT,
		FAN54511_REG_IOCHRG,
		FAN54511_REG_IBAT,
		FAN54511_REG_IBUS,
		FAN54511_REG_VBUS,
		FAN54511_REG_IIN,
		FAN54511_REG_VIN,
		FAN54511_REG_NTC,
		FAN54511_REG_TIMER,
		FAN54511_REG_SAFETY,
		FAN54511_REG_TOPOFF,
		FAN54511_REG_BOOST,
		FAN54511_REG_DPLUS,
		FAN54511_REG_MONITOR0,
		FAN54511_REG_MONITOR1,
		FAN54511_REG_IC_INFO,
		FAN54511_REG_FEATURE_CONTROL
		};
	int arrSize = sizeof(reg_addrs)/sizeof(reg_addrs[0]);
	struct fan54511 *fan = dev_get_drvdata(dev);
	u8 addr;
	u8 val;

            fan54511_dump_register();
	for (addr = 0; addr < arrSize; addr++) {
		fan54511_main_read_byte(&val, reg_addrs[addr]);
		dev_info(fan->dev, "main_[0x%.2x] = 0x%.2x\n", addr, val);
	}

	for (addr = 0; addr < arrSize; addr++) {
		fan54511_sub_read_byte(&val, reg_addrs[addr]);
		dev_info(fan->dev, "sub_[0x%.2x] = 0x%.2x\n", addr, val);
	}
	return 0;
}

static DEVICE_ATTR(registers, S_IRUGO, fan54511_show_registers, NULL);
static ssize_t charger_gpio_down_store_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
        int down_enable = 0;
        down_enable = simple_strtol(buf, NULL,10);
        printk(KERN_EMERG "charger_gpio_down_show_registers: down_enable = %d\n",down_enable);
    if(down_enable == 0){
        charger0_en_poweron(1);
        charger1_en_poweron(1);
        vbus_ov1_en_poweron(1);
        vbus_ov2_en_poweron(1);
        }else{
        charger0_en_poweron(0);
        charger1_en_poweron(0);
        vbus_ov1_en_poweron(0);
        vbus_ov2_en_poweron(0);
        }
	return 0;
}

///sys/devices/soc/1101c000.i2c/i2c-5/5-006b/charger_gpio_down
static DEVICE_ATTR(charger_gpio_down, 0644, NULL, charger_gpio_down_store_registers);

static struct attribute *fan54511_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_charger_gpio_down.attr,
	NULL,
};

static const struct attribute_group fan54511_attr_group = {
	.attrs = fan54511_attributes,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fast_early_suspend(struct early_suspend *h)
{
#if 0
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	fast_charge_current_1c = KAL_FALSE;
	wake_up_bat();
#endif
#endif
}

static void fast_late_resume(struct early_suspend *h)
{
#if 0
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if (fast_charge_current_1c == KAL_FALSE) {
		fast_charge_current_1c = KAL_TRUE;		
		wake_up_bat();
	}
#endif
#endif
}
#endif

static void fan54511_irq_handle(struct work_struct *work)
{
	u8 int0,int1,int2;
	u8 status0, status2;
	u8 mint0,mint1,mint2;	

	int pg_stat = 0;
	int chrg_stat;

	int watchdog_fault;
//	int chrg_fault;
	int bat_fault;
	int ntc_fault;
	
	unsigned int charging_enable = KAL_FALSE;

	mutex_lock(&handler_mutex);
	wake_lock(&g_sub->charger_wake_lock);

	fan54511_sub_read_byte(&int0, FAN54511_REG_INT0);
	fan54511_sub_read_byte(&int1, FAN54511_REG_INT1);
	fan54511_sub_read_byte(&int2, FAN54511_REG_INT2);
	printk("%s:int0 0x%02x, int1 0x%02x, int2 0x%02x\n", __func__, int0, int1, int2);

	fan54511_sub_read_byte(&status0, FAN54511_REG_STATUS0);
	fan54511_sub_read_byte(&status2, FAN54511_REG_STATUS2);
	fan54511_sub_read_byte(&mint0, FAN54511_REG_MINT0);
	fan54511_sub_read_byte(&mint1, FAN54511_REG_MINT1);
	fan54511_sub_read_byte(&mint2, FAN54511_REG_MINT2);

	fan54511_main_read_byte(&int0, FAN54511_REG_INT0);
	fan54511_main_read_byte(&int1, FAN54511_REG_INT1);
	fan54511_main_read_byte(&int2, FAN54511_REG_INT2);
	fan54511_main_read_byte(&mint0, FAN54511_REG_MINT0);
	fan54511_main_read_byte(&mint1, FAN54511_REG_MINT1);
	fan54511_main_read_byte(&mint2, FAN54511_REG_MINT2);


	printk("%s:status0 0x%02x, status1 0x%02x\n", __func__, status0, status2);

	chrg_stat = (status0 & STAT0_STAT) >> STAT0_STAT_SHIFT;
	pg_stat = (status0 & STAT0_PWROK) >> STAT0_PWROK_SHIFT;

	watchdog_fault = (status2 & STAT2_WDTTO) >> STAT2_WDTTO_SHIFT;
//	chrg_fault = (status2 & BQ2589X_FAULT_CHRG_MASK) >> BQ2589X_FAULT_CHRG_SHIFT;
	bat_fault = (status0 & STAT0_NOBAT);
	ntc_fault = (status2 & STAT2_TEMPSD) >> STAT2_TEMPSD_SHIFT;

#if 0
	if (pg_stat && power_good == KAL_FALSE) {
		printk("Charger insert now, and the chrg_stat is %d\n", chrg_stat);
		do_chrdet_int_task();
		power_good = KAL_TRUE;
	} else if (!pg_stat && power_good == KAL_TRUE) {
		printk("charger remove now\n");
		do_chrdet_int_task();
		power_good = KAL_FALSE;
	}
#endif
	if (watchdog_fault == 1
		       /* || chrg_fault != BQ2589X_FAULT_CHRG_NORMAL 
			|| bat_fault == 1 || ntc_fault != 0*/) {
		printk("%s:charger fault occurs\n", __func__);
		fan54511_set_VFLOAT(0x69); //VREG 4.35V
		charging_enable = KAL_FALSE;
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
		msleep(500);
		charging_enable = KAL_TRUE;
		battery_charging_control(CHARGING_CMD_ENABLE_MAJOR, &charging_enable);
	}

#if 0
	mt_eint_unmask(CUST_EINT_CHARGER0_EINT_NUM);
#endif

	wake_unlock(&g_sub->charger_wake_lock);
	mutex_unlock(&handler_mutex);

	return;	
}

static void fan54511_isr(void)
{
        schedule_delayed_work_on(0, &g_sub->irq_dwork, 0);
}

static void fan54511_irq_init(void)
{
        /* main charger INT pin */
       #if 0
	mt_set_gpio_dir(GPIO_CHARGER0_INT_PIN, GPIO_DIR_IN);
	mt_eint_set_sens(CUST_EINT_CHARGER0_EINT_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_CHARGER0_EINT_NUM, 0);
	mt_eint_registration(CUST_EINT_CHARGER0_EINT_NUM, EINTF_TRIGGER_LOW, fan54511_isr, 0);
	mt_eint_unmask(CUST_EINT_CHARGER0_EINT_NUM);
    #endif
        /* sub charger INT pin */
        /*
	mt_set_gpio_dir(GPIO_CHARGER1_INT_PIN, GPIO_DIR_IN);
	mt_eint_set_sens(CUST_EINT_CHARGER1_EINT_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_CHARGER1_EINT_NUM, 0);
	mt_eint_registration(CUST_EINT_CHARGER1_EINT_NUM, EINTF_TRIGGER_LOW, fan54511_isr, 0);
	mt_eint_unmask(CUST_EINT_CHARGER1_EINT_NUM);
	*/
}

void charger0_en_poweron(int en)
{
/*
    	switch (en) {
	case 1:
		pinctrl_select_state(chgctrl, chg0_dis_h);
		printk("set motor enable pin to oh\n");
		break;
	case 0:
		pinctrl_select_state(chgctrl, chg0_dis_l);
		printk("set motor enable pin to ol\n");
		break;

	default:
		printk("%d mode not defined for motor enable pin !!!\n", state);
		break;
	}
	return 0;
*/
}

void charger1_en_poweron(int en)
{
    	switch (en) {
	case 1:
		pinctrl_select_state(chgctrl, chg1_dis_h);
		printk("set 2nd charger dis enable pin to oh\n");
		break;
	case 0:
		pinctrl_select_state(chgctrl, chg1_dis_l);
		printk("set 2nd charger dis enable pin to ol\n");
		break;

	default:
		printk(" 2nd charger dis not defined for 2nd charger dis enable pin !!!\n");
		break;
	}
	return 0;

}
void vbus_ov1_en_poweron(int en)
{
    	switch (en) {
	case 1:
		pinctrl_select_state(chgctrl, vbus_ov1_h);
		printk("set vbus ov1 enable pin to oh\n");
		break;
	case 0:
		pinctrl_select_state(chgctrl, vbus_ov1_l);
		printk("set vbus ov1 enable pin to ol\n");
		break;

	default:
		printk("%d mode not defined for vbus ov1 enable pin !!!\n");
		break;
	}
	return 0;
	
}

void vbus_ov2_en_poweron(int en)
{/*
    	switch (en) {
	case 1:
		pinctrl_select_state(chgctrl, vbus_ov2_h);
		printk("set motor enable pin to oh\n");
		break;
	case 0:
		pinctrl_select_state(chgctrl, vbus_ov2_l);
		printk("set motor enable pin to ol\n");
		break;

	default:
		printk("%d mode not defined for motor enable pin !!!\n", state);
		break;
	}
	return 0;
	*/
}

 void fan54511_gpio_set(void)
{
/* charger enable pin */
    charger0_en_poweron(1);//default:   Enable main charger charging
    charger1_en_poweron(0);//default:  Disable 2nd charger charging

/*vbus over voltage protect pin*/
/*****************************************************
when VBUS_OV1 = L , VBUS_OV2 = L ,then vbus <= 5.9v;
when VBUS_OV1 = H , VBUS_OV2 = L ,then vbus <= 10v;
when VBUS_OV1 = L , VBUS_OV2 = H ,then vbus <= 14v;
when VBUS_OV1 = H , VBUS_OV2 = H ,then vbus <= 23v;
******************************************************/
    vbus_ov1_en_poweron(0); /*VBUS_OV1 = H*/
    vbus_ov2_en_poweron(1); /*VBUS_OV2 = H*/
}
 
int fan54511_gpio_set_value(int en)
{
    	switch (en) {
	case 1:
		pinctrl_select_state(chgctrl, chg0_dis_h);
                      pinctrl_select_state(chgctrl, chg1_dis_h);
                      pinctrl_select_state(chgctrl, vbus_ov1_h);
                      pinctrl_select_state(chgctrl, vbus_ov2_h);

		printk("set fan54511_gpio_set_value enable pin to oh\n");
		break;
	case 0:
		pinctrl_select_state(chgctrl, chg0_dis_l);
                      pinctrl_select_state(chgctrl, chg1_dis_l);
                      pinctrl_select_state(chgctrl, vbus_ov1_l);
                      pinctrl_select_state(chgctrl, vbus_ov2_l);
		printk("set fan54511_gpio_set_value enable pin to ol\n");
		break;

	default:
		printk("%d mode not defined for fan54511_gpio_set_value enable pin !!!\n", en);
		break;
	}
	return 0;}

int fan54511_gpio_init(void)
{
	int ret = 0;
    
            printk("     fan54511_gpio_init      \n");
            
	chgctrl = devm_pinctrl_get(&g_main->client->dev);
	if (IS_ERR(chgctrl)) {
		dev_err(&g_main->dev, "Cannot find charger chgctrl!");
		ret = PTR_ERR(g_main);
	}
    
	/*Chg0 dis gpio  initialization */
    
	chg0_dis_h = pinctrl_lookup_state(chgctrl, "chg0_dis_high_cfg");
	if (IS_ERR(chg0_dis_h)) {
		ret = PTR_ERR(chg0_dis_h);
		printk("%s : pinctrl err, chg0_dis_h\n", __func__);
	}

	chg0_dis_l = pinctrl_lookup_state(chgctrl, "chg0_dis_low_cfg");
	if (IS_ERR(chg0_dis_l)) {
		ret = PTR_ERR(chg0_dis_l);
		printk("%s : pinctrl err, chg0_dis_l\n", __func__);
	}


	chg1_dis_h = pinctrl_lookup_state(chgctrl, "chg1_dis_high_cfg");
	if (IS_ERR(chg1_dis_h)) {
		ret = PTR_ERR(chg1_dis_h);
		printk("%s : pinctrl err, chg1_dis_h\n", __func__);
	}

	chg1_dis_l = pinctrl_lookup_state(chgctrl, "chg1_dis_low_cfg");
	if (IS_ERR(chg1_dis_l)) {
		ret = PTR_ERR(chg1_dis_l);
		printk("%s : pinctrl err, chg1_dis_l\n", __func__);
	}

	vbus_ov1_h = pinctrl_lookup_state(chgctrl, "vubs_ov1_high_cfg");
	if (IS_ERR(vbus_ov1_h)) {
		ret = PTR_ERR(vbus_ov1_h);
		printk("%s : pinctrl err, vbus_ov1_h\n", __func__);
	}

	vbus_ov1_l = pinctrl_lookup_state(chgctrl, "vubs_ov1_low_cfg");
	if (IS_ERR(vbus_ov1_l)) {
		ret = PTR_ERR(vbus_ov1_l);
		printk("%s : pinctrl err, vbus_ov1_l\n", __func__);
	}
    pinctrl_select_state(chgctrl, vbus_ov1_h);


	vbus_ov2_h = pinctrl_lookup_state(chgctrl, "vubs_ov2_high_cfg");
	if (IS_ERR(vbus_ov2_h)) {
		ret = PTR_ERR(vbus_ov2_h);
		printk("%s : pinctrl err, vbus_ov2_h\n", __func__);
	}


	vbus_ov2_l = pinctrl_lookup_state(chgctrl, "vubs_ov2_low_cfg");
	if (IS_ERR(vbus_ov2_l)) {
		ret = PTR_ERR(vbus_ov2_l);
		printk("%s : pinctrl err, vbus_ov2_l\n", __func__);
	}

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id fan54511_of_match[] = {
	{.compatible = "mediatek,fan54511_main", .data = (void *)MAIN_CHARGER },
	{.compatible = "mediatek,fan54511_sub",    .data = (void *)SUB_CHARGER },
	{},
};
#else
static struct i2c_board_info __initdata i2c_fan54511_main = 
{ 
	I2C_BOARD_INFO("fan54511_main", FAN54511_SLAVE_ADDR)
};

static struct i2c_board_info __initdata i2c_fan54511_sub = 
{ 
	I2C_BOARD_INFO("fan54511_sub", FAN54511_SLAVE_ADDR)
};
#endif



static int fan54511_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct fan54511 *fan;
	int ret;
	unsigned long chip_id;
	u8 device_conf = 0;

            printk("%s \n", __func__);

	fan = kzalloc(sizeof(struct fan54511), GFP_KERNEL);
	if (!fan){
		dev_err(&client->dev,"%s:out of memory\n",__func__);
		return -ENOMEM;
	}

		fan->dev = &client->dev;
		fan->client = client;
		i2c_set_clientdata(client, fan);

	if (fan->dev->of_node ) {
		const struct of_device_id *id;

		id = of_match_device(of_match_ptr(fan54511_of_match), fan->dev);
		if (!id)
			return -ENODEV;

		chip_id = (unsigned long) id->data;
	} else {
	    	chip_id = id->driver_data;
		printk("fan54511_charger_probe  failed get chip_id !!! \n");
	}
    
    
            printk("chip_id = %d \n", chip_id);
	if (chip_id == MAIN_CHARGER) {
                     printk("%s:[fan54511 main charger probe]\n", __func__);
                    

		g_main = fan;

		ret = fan54511_main_read_byte(&device_conf, FAN54511_REG_IC_INFO);
		if (ret < 0) {
			printk("%s:[fan54511_main]the slaver addr is not 0x6B\n", __func__);
		}
                    fan54511_gpio_init();
                    pinctrl_select_state(chgctrl, chg1_dis_l);
       //            fan54511_gpio_set_value(1);
        
	} else if (chip_id == SUB_CHARGER) {
                     printk("%s:[fan54511 2nd charger probe]\n", __func__);
		g_sub = fan;

		ret = sysfs_create_group(&fan->dev->kobj, &fan54511_attr_group);
		if (ret) {
			dev_err(fan->dev, "failed to register sysfs. err: %d\n", ret);
			goto err_0;
		}
//s		meizu_sysfslink_register_n(fan->dev, "charger");

		ret = fan54511_sub_read_byte(&device_conf, FAN54511_REG_IC_INFO);
		if (ret < 0) {
			printk("%s:[FAN54511_sub]the slaver addr is not 0x6B\n", __func__);
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		fan->fast_handler.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
		fan->fast_handler.suspend = fast_early_suspend;
		fan->fast_handler.resume = fast_late_resume;
		register_early_suspend(&fan->fast_handler);
#endif
		wake_lock_init(&g_sub->charger_wake_lock, WAKE_LOCK_SUSPEND, "charger wakelock");
		INIT_DELAYED_WORK(&g_sub->irq_dwork, fan54511_irq_handle);
		//fan54511_irq_init();
	}

    /*default disbale charger */
    #if 0
	mt_set_gpio_out(GPIO_CHARGER0_EN_PIN, 1);		
	mt_set_gpio_out(GPIO_CHARGER1_EN_PIN, 1);
    #endif
    
	printk("%s:chip_id %d,client addr 0x%02x\n", __func__, chip_id, client->addr);
	return 0;
err_0:
	kfree(fan);
	return ret;
}

static int fan54511_charger_remove(struct i2c_client *client)
{
	struct fan54511 *fan = i2c_get_clientdata(client);

	sysfs_remove_group(&fan->dev->kobj, &fan54511_attr_group);
//	meizu_sysfslink_unregister(fan->dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&fan->fast_handler);
#endif
	kfree(fan);
	g_main = NULL;
	g_sub = NULL;

	return 0;
}

static void fan54511_shutdown(struct i2c_client *client)
{
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if (is_ta_connect == KAL_TRUE)
		mtk_ta_reset_vchr();
#endif
}

static const struct i2c_device_id fan54511_id[] = {
    	{ "fan54511_main", 0},
    	{ "fan54511_sub", 1},
	{},
};
MODULE_DEVICE_TABLE(i2c, fan54511_id);



static struct i2c_driver fan54511_charger = {
	.probe		= fan54511_charger_probe,
	.remove		= fan54511_charger_remove,
	.shutdown	= fan54511_shutdown,
	.id_table	= fan54511_id,
	.driver		= {
		.name	= "fan54511",
#ifdef CONFIG_OF
                      .of_match_table = fan54511_of_match,
#endif
	},
};

static int __init fan54511_init(void)
{    
    int ret=0;

	/* i2c registeration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	battery_log(BAT_LOG_CRTI, "[fan54511_init] init start with i2c DTS");
#else
    i2c_register_board_info(FAN54511_BUSNUM0, &i2c_fan54511_main, 1);
    i2c_register_board_info(FAN54511_BUSNUM5, &i2c_fan54511_sub, 1);
#endif

    if(i2c_add_driver(&fan54511_charger) != 0)
    {
        printk("fan54511_init] failed to register fan54511 i2c driver.\n");
    } else {
        printk("[fan54511_init] Success to register fan54511 i2c driver.\n");
    }
    
    return ret;        
}

static void __exit fan54511_exit(void)
{
    i2c_del_driver(&fan54511_charger);
}

module_init(fan54511_init);
module_exit(fan54511_exit);

MODULE_DESCRIPTION("Fairchild FAN54511 Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fairchild Semiconductor");


