#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <mach/gpio_const.h>
#include <cust_gpio_usage.h>
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_i2c.h>
#endif

#define LCM_DEBUG_LOG
#ifdef LCM_DEBUG_LOG
#ifdef BUILD_LK
#define lcm_print(string, args...) printf("[LCM:SAMSUNG D53G]"string, ##args)
#else
#define lcm_print(string, args...) printk("[LCM:SAMSUNG D53G]"string, ##args)
#endif
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01; /*  haobing modified 2013.07.11 */
/**
 * Local Constants
 */
#define LCM_DSI_CMD_MODE	1
#define FRAME_WIDTH		(1080)
#define FRAME_HEIGHT		(1920)

#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN

#define REGFLAG_PORT_SWAP	0xFFFA
#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_END_OF_TABLE	0xFFFD   /* END OF REGISTERS MARKER */

/* static unsigned int lcm_esd_test = FALSE; */ /* only for ESD test */
/**
 * Local Variables
 */
static unsigned int last_brightness_level = 0;
static const unsigned int BL_MIN_LEVEL = 20;
unsigned int display_hbm = 0;
unsigned int display_lut = 3;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))

/*
 * Local Functions
 */
#define dsi_set_cmd_by_cmdq_v22(handle, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(handle, cmd, (unsigned char)(count), \
					  (unsigned char *)(ppara), (unsigned char)(force_update))
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)					lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)					lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_swap_port(swap)				lcm_util.dsi_swap_port(swap)

#define set_gpio_lcd_enp(cmd) lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_power(cmd) lcm_util.set_gpio_lcd_power_sequence(cmd)
static void lcm_setbacklight(unsigned int level);
static unsigned int lcm_set_lut(unsigned int val);

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

#define D53G_LUT_ON

static struct LCM_setting_table lcm_initialization_setting[] = {
	//{REGFLAG_DELAY, 15, {}},
	/*sleep out*/
	{0x11, 1, {0}},
	{REGFLAG_DELAY, 22, {}},
	/*TE on (must)*/
	{0x35, 1, {0x00}},
	//{0x36, 1, {0xC0}},
	{0xF0, 2, {0x5A, 0x5A}},
	{0xEE, 1, {0x01}},
	{0x53, 1, {0x20}},
	{0x55, 1, {0x00}},
	{0x51, 1, {0x00}},

    {0xFC, 2, {0x5A, 0x5A}},
    {0xB0, 1, {0x01}},
    {0xD2, 1, {0x20}},
    {0xFC, 2, {0xA5, 0xA5}},

#ifdef D53G_LUT_ON
	{0xF0, 2, {0x5A, 0x5A}},
	{0xED, 1, {0xBC}},//0xBE
	{0xB0, 1, {0x06}},
	{0xED, 1, {0x00}},//0X70
//SRGB
	{0xB0, 1, {0x08}},
	{0xED, 21, {0x9F, 0x04, 0x04, 0x38, 0xCF, 0x12, 0x06, 0x05, 0xB1, 0x46, 0xF0, 0xD2, 0xBD, 0x0A, 0xC3, 0xE0, 0xE5, 0x18, 0xFF, 0xFF, 0xFF}},
//ADOBERGB
	{0xB0, 1, {0x1D}},
	{0xED, 21, {0xDF, 0x05, 0x06, 0x00, 0xD2, 0x00, 0x07, 0x05, 0xC1, 0x00, 0xE9, 0xCC, 0xFA, 0x0C, 0xCC, 0xDF, 0xE4, 0x0E, 0xFF, 0xFF, 0xFF}},
//ADAPTIVE
	{0xB0, 1, {0x32}},
	{0xED, 21, {0xB5, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x03, 0x00, 0xE0, 0x01, 0xFF, 0xCF, 0xA9, 0x00, 0xAD, 0xE3, 0xFF, 0x01, 0xFF, 0xFF, 0xFF}},
//DCI_P3
	{0xB0, 1, {0x32}},
	{0xED, 21, {0xBA, 0x00, 0x00, 0x0F, 0xCE, 0x00, 0x07, 0x06, 0xC1, 0x18, 0xF5, 0xD3, 0xE0, 0x00, 0xD1, 0xDD, 0xE2, 0x00, 0xFF, 0xFF, 0xFF}},
	{0xF0, 2, {0xA5, 0xA5}},
//ADAPTIVE_DEFAULT
	{0x57, 1, {0xA4}},//0XA7
#endif
#ifdef D53G_LUT_SEED_OFF
	{0x57, 1, {0x40}},//0X43
#endif
	{REGFLAG_DELAY, 90, {}},
	/*display on*/
	{0x29, 1, {0}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

static struct LCM_setting_table sec_lut_sRGB[] = {
	{0xF0, 2, {0x5A, 0x5A}},
	{0xED, 1, {0xBC}},//0xBE
	{0xB0, 1, {0x06}},
	{0xED, 1, {0x00}},//0X70
	{0xB0, 1, {0x08}},
	{0xED, 21, {0x9F, 0x04, 0x04, 0x38, 0xCF, 0x12, 0x06, 0x05, 0xB1, 0x46, 0xF0, 0xD2, 0xBD, 0x0A, 0xC3, 0xE0, 0xE5, 0x18, 0xFF, 0xFF, 0xFF}},
	{0x57, 1, {0x4C}},
	{0xF0, 2, {0xA5, 0xA5}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_lut_AdobeRGB[] = {
	{0xF0, 2, {0x5A, 0x5A}},
	{0xED, 1, {0xBC}},//0xBE
	{0xB0, 1, {0x06}},
	{0xED, 1, {0x00}},//0X70
	{0xB0, 1, {0x1D}},
	{0xED, 21, {0xDF, 0x05, 0x06, 0x00, 0xD2, 0x00, 0x07, 0x05, 0xC1, 0x00, 0xE9, 0xCC, 0xFA, 0x0C, 0xCC, 0xDF, 0xE4, 0x0E, 0xFF, 0xFF, 0xFF}},
	{0x57, 1, {0x44}},
	{0xF0, 2, {0xA5, 0xA5}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_lut_Adaptive[] = {
	{0xF0, 2, {0x5A, 0x5A}},
	{0xED, 1, {0xBC}},//0xBE
	{0xB0, 1, {0x06}},
	{0xED, 1, {0x00}},//0X70
	{0xB0, 1, {0x32}},
	{0xED, 21, {0xB5, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x03, 0x00, 0xE0, 0x01, 0xFF, 0xCF, 0xA9, 0x00, 0xAD, 0xE3, 0xFF, 0x01, 0xFF, 0xFF, 0xFF}},
	{0x57, 1, {0xA4}},
	{0xF0, 2, {0xA5, 0xA5}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_lut_off[] = {
	{0x57, 1, {0x40}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table sec_hbmoff_1frm[] = {
	{0x53, 1, {0x20}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_hbmon_1frm[] = {
	{0x53, 1, {0xE0}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


#if 1
static struct LCM_setting_table lcm_suspend_setting[] = {
	/*display off*/
	{0x28, 1, {0}},
	{REGFLAG_DELAY, 40, {}},
	/*sleep in*/
	{0x10, 1, {0}},
	{REGFLAG_DELAY, 160, {}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};
#endif
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0x44} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;
		case REGFLAG_PORT_SWAP:
			dsi_swap_port(1);
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/*
 * LCM Driver Implementations
 */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	lcm_print("[LCM] %s\n", __func__);
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		//params->physical_width = 69;//69.21/68.31
		//params->physical_height = 121;//121.44 122.3

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;
		params->dsi.mode   = BURST_VDO_MODE;
#endif
		params->dsi.esd_check_enable = 1;  //for esd test.
		params->dsi.customization_esd_check_enable = 0;

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size = 256;
		//params->dsi.clk_lp_per_line_enable = 1;
		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;
		params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count = FRAME_WIDTH * 3;
		
		params->dsi.vertical_sync_active				= 13;
		params->dsi.vertical_backporch					= 11;
		params->dsi.vertical_frontporch					= 5;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		//params->dsi.vertical_frontporch_for_low_power = 400; // for saving power in screen idle

		params->dsi.horizontal_sync_active				= 14; //+ backprot = 36
		params->dsi.horizontal_backporch				= 18;
		params->dsi.horizontal_frontporch				= 18;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		params->dsi.PLL_CLOCK = 440;
		//1 Every lane speed
	//	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4
	//	params->dsi.fbk_div =0x13;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}

static unsigned int lcm_power_switch(unsigned int enable)
{
	unsigned int ret = 0;

	lcm_print("%s enable = %d \n", __func__, enable);
	if (enable)
	{
		ret = set_gpio_lcd_power(1);
		MDELAY(13); 
		SET_RESET_PIN(1);
		MDELAY(1);
		SET_RESET_PIN(0);
		MDELAY(1);
		SET_RESET_PIN(1);
		MDELAY(15);
	}else{
		SET_RESET_PIN(0);
		MDELAY(15);
		ret = set_gpio_lcd_power(0);
		//MDELAY(50);
	}

	return ret;
}

static void lcm_init_power(void)
{
	lcm_print("%s\n", __func__);
}

static void lcm_resume_power(void)
{
	lcm_print("%s\n", __func__);
}

static void lcm_suspend_power(void)
{
	lcm_print("%s\n", __func__);
}

static void lcm_init(void)
{
	int ret = 0;

	lcm_print("%s\n", __func__);
	ret = lcm_power_switch(1);
	/* when phone initial , config output high, enable backlight drv chip */
	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	if (last_brightness_level != 0)//for esd recovery turn on backlight
	{
		lcm_setbacklight(last_brightness_level);
	}
}

static void lcm_suspend(void)
{
	lcm_print("%s\n", __func__);
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(200);
	lcm_power_switch(0);
}

static void lcm_resume(void)
{
	lcm_print("%s\n", __func__);
	lcm_init();
	lcm_set_lut(display_lut);
}

static void lcm_update(unsigned int x, unsigned int y,
		       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	/* BEGIN PN:DTS2013013101431 modified by s00179437 , 2013-01-31 */
	/* delete high speed packet */
	/* data_array[0] =0x00290508; */
	/* dsi_set_cmdq(data_array, 1, 1); */
	/* END PN:DTS2013013101431 modified by s00179437 , 2013-01-31 */

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

#define LCM_ID_D53G (0x00)

static unsigned int lcm_compare_id(void)
{
	unsigned char buffer[5];
	unsigned int array[16];
	unsigned int lcd_id = 0;
return  1;

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	array[0] = 0x00053700;/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xBF, buffer, 5);
	MDELAY(20);
	lcd_id = (buffer[2] << 8) | buffer[3];

	lcm_print("%s, id = 0x%08x\n", __func__, lcd_id);


	if (lcd_id == LCM_ID_D53G)
		return 1;
	else
		return 0;

}
static unsigned int lcm_ata_check(unsigned char *buffer)
{
	return 1;
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	/* Refresh value of backlight level. */
	unsigned int cmd = 0x51;
	unsigned int count = 1;
	unsigned int value = level;

	lcm_print("%s, kernel backlight: level = %d\n", __func__, level);
	last_brightness_level = level;
	dsi_set_cmd_by_cmdq_v22(handle, cmd, count, &value, 1);
	 //lcm_backlight_level_setting[0].para_list[0] = level;
	 //push_table(lcm_backlight_level_setting,
	//	      sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_setbacklight(unsigned int level)
{
	lcm_print("%s, kernel backlight: level = %d\n", __func__, level);

	 lcm_backlight_level_setting[0].para_list[0] = level;
	 push_table(lcm_backlight_level_setting,
		      sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_set_hbm(unsigned int onoff)
{
	lcm_print("%s, onoff = %d\n", __func__, onoff);
	if (onoff)
	{
		push_table(sec_hbmon_1frm,
		   sizeof(sec_hbmon_1frm) / sizeof(struct LCM_setting_table), 1);
	}
	else
	{
		push_table(sec_hbmoff_1frm,
		   sizeof(sec_hbmoff_1frm) / sizeof(struct LCM_setting_table), 1);
	}
}

static unsigned int lcm_set_lut(unsigned int val)
{
	lcm_print("%s, val = %d\n", __func__, val);
	switch(val)
	{
		case 0://Seed off
			//sec_lut_setting[0].para_list[0] = 0x40;
		push_table(sec_lut_off,
		   sizeof(sec_lut_off) / sizeof(struct LCM_setting_table), 1);
		break;
		case 1://sRGB
			//sec_lut_setting[0].para_list[0] = 0x4C;
		push_table(sec_lut_sRGB,
		   sizeof(sec_lut_sRGB) / sizeof(struct LCM_setting_table), 1);
		break;
		case 2://AdobeRGB
			//sec_lut_setting[0].para_list[0] = 0x44;
		push_table(sec_lut_AdobeRGB,
		   sizeof(sec_lut_AdobeRGB) / sizeof(struct LCM_setting_table), 1);
		break;
		case 3:// Adaptive
			//sec_lut_setting[0].para_list[0] = 0xA4;
		push_table(sec_lut_Adaptive,
		   sizeof(sec_lut_Adaptive) / sizeof(struct LCM_setting_table), 1);
		break;
		default:// Adaptive
			//sec_lut_setting[0].para_list[0] = 0xA4;
		push_table(sec_lut_Adaptive,
		   sizeof(sec_lut_Adaptive) / sizeof(struct LCM_setting_table), 1);
		break;
	}
	//push_table(sec_lut_setting,
	//	   sizeof(sec_lut_setting) / sizeof(struct LCM_setting_table), 1);
}

#ifdef MEIZU_M80
static unsigned int amoled_get_panel_id(void)
{
	unsigned char products_id[4] = {0};
	unsigned int panel_id = 0;

	read_reg_v2(0xDA, &products_id[0], 1);
	read_reg_v2(0xDB, &products_id[1], 1);
	read_reg_v2(0xDC, &products_id[2], 1);
	panel_id = (products_id[2] | (products_id[1]<<8) | (products_id[0] << 16));
	lcm_print("get panel id :%x\n",panel_id);
	return panel_id;
}
#endif

LCM_DRIVER d53g_fhd_dsi_cmd_sec_lcm_drv = {
	.name		= "d53g_fhd_dsi_cmd_sec",
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params	= lcm_get_params,
	.init		= lcm_init,
	.suspend	= lcm_suspend,
	.resume		= lcm_resume,
	.compare_id	= lcm_compare_id,
	.init_power	= lcm_init_power,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.ata_check	= lcm_ata_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
#if (LCM_DSI_CMD_MODE)
	.update		= lcm_update,
#endif
#ifdef MEIZU_M80
	.set_hbm		= lcm_set_hbm,
	.set_lut		= lcm_set_lut,
	.get_panel_id 	= amoled_get_panel_id,
#endif
};
/* END PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
