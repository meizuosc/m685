#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
#ifdef CONFIG_MTK_LEGACY
	#include <mach/mt_pm_ldo.h>
	#include <mach/mt_gpio.h>
#endif
#endif
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  pr_debug(fmt)
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_i2c.h>
#include <mach/gpio_const.h>
#include <cust_gpio_usage.h>
#endif

/**
 * Local Constants
 */
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH	(1080)
#define FRAME_HEIGHT	(1920)

/**
 * Local Variables
 */

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;
static unsigned int id_code = 0;

#ifndef BUILD_LK
extern void tpd_config_gpio(void);
#endif

#define REGFLAG_DELAY 0xFFFC
#define REGFLAG_UDELAY 0xFFFB

#define REGFLAG_END_OF_TABLE 0xFFFD /* END OF REGISTERS MARKER*/
#define REGFLAG_RESET_LOW 0xFFFE
#define REGFLAG_RESET_HIGH 0xFFFF


#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

/**
 * Local Functions
 */
#define dsi_set_cmd_by_cmdq_dual(handle, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V23(handle, cmd, (unsigned char)(count), \
					  (unsigned char *)(ppara), (unsigned char)(force_update))
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)		lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)					lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_swap_port(swap)				lcm_util.dsi_swap_port(swap)

#ifdef BUILD_LK
#define lcm_print(string, args...) printf("[LCM]"string, ##args)
#else
#define lcm_print(string, args...) printk("[LCM]"string, ##args)
#endif

#define set_gpio_lcd_bias_enp(cmd) lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_bias_enn(cmd) lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_vddi18(cmd) lcm_util.set_gpio_lcd_vddi18(cmd)
#define ESD_CHECK

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/*#include <linux/jiffies.h> */
#include <linux/uaccess.h>
/*#include <linux/delay.h> */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif
/*****************************************************************************
* Define
*****************************************************************************/

/*****************************************************************************
* GLobal Variable
*****************************************************************************/
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_slpout_ack[] = {
	{0x35, 1, {0}},
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_slpout[] = {
#if 0
	{0xff,3,{0x87,0x16,0x01}},
	{0x00,1,{0x80}},
	{0xff,2,{0x87,0x16}},
	{0x00,1,{0x80}}, //C080                                                                                                                                         
	{0xC0,15,{0x00,0x7E,0x00,0x10,0x07,0x00,0x7E,0x10,0x10,0x00,0x7E,0x00,0x10,0x10,0x00}}, //
	{0x00,1,{0xA0}}, //C0A0                                                                                                                                         
	{0xC0,7,{0x0B,0x01,0x05,0x01,0x01,0x1B,0x07}},
	{0x00,1,{0xD0}}, //C0D0
	{0xC0,7,{0x0B,0x01,0x05,0x01,0x01,0x1B,0x07}},
	{0x00,1,{0x87}},                                                                                                                                        
	{0xA5,4,{0x00,0x07,0x77,0x77}},
	{0x00,1,{0x80}},                                                                                                                                        
	{0xC2,8,{0x82,0x00,0x14,0x0A,0x81,0x00,0x14,0x0A}},
	{0x00,1,{0xB0}}, //C2B0                                                                                                                                         
	{0xC2,15,{0x00,0x01,0x00,0x14,0x00,0x01,0x02,0x00,0x14,0x00,0x82,0x00,0x00,0x14,0x00}},
	{0x00,1,{0xC0}}, //C2C0                                                                                                                                         
	{0xC2,5,{0x81,0x00,0x00,0x14,0x00}},
	{0x00,1,{0xDA}}, //C2DA                                                                                                                                         
	{0xC2,2,{0x33,0x33}},
	{0x00,1,{0xE0}}, //C2E0                                                                                                                                         
	{0xC2,4,{0x05,0x00,0x00,0x03}},
	{0x00,1,{0xAA}}, //C3AA                                                                                                                                         
	{0xC3,2,{0x9C,0x9C}},
	{0x00,1,{0xD0}}, //C3D0                                                                                                                                         
	{0xC3,15,{0x00,0x02,0x02,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00,0x00,0x6B,0x00,0x09}},
	{0x00,1,{0x80}}, //CE80
	{0xCE,6,{0x25,0x00,0xA0,0x00,0x78,0xFF}},//ce80=25
	{0x00,1,{0x90}}, //CE90                                                                                                                                         
	{0xCE,8,{0x00,0x49,0x0E,0x53,0x00,0x49,0x00,0x6E}},
	{0x00,1,{0xB0}}, //CEB0                                                                                                                                         
	{0xCE,6,{0x00,0x00,0x60,0x79,0x00,0x60}},
	{0x00,1,{0x80}}, //CC80                                                                                                                                         
	{0xCC,12,{0x01,0x02,0x03,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E}},
	{0x00,1,{0x90}}, //CC90
	{0xCC,12,{0x01,0x03,0x02,0x09,0x08,0x07,0x06,0x0E,0x0D,0x0C,0x0B,0x0A}},
	{0x00,1,{0xA0}}, //CCA0
	{0xCC,15,{0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x18,0x19,0x20,0x21,0x10,0x22,0x22,0x22,0x22}},
	{0x00,1,{0x80}}, //CB80
	{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0x90}}, //CB90
	{0xCB,15,{0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xA0}},
	{0xCB,15,{0xF0,0xF0,0xF0,0x50,0x50,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xB0}},
	{0xCB,2,{0x00,0x00}},
	{0x00,1,{0xC0}}, //CBC0
	{0xCB,15,{0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0xF5,0xF5,0xF5}},
	{0x00,1,{0xD0}},
	{0xCB,15,{0x05,0x05,0x05,0x55,0x55,0x05,0x05,0xF0,0x0F,0x30,0x05,0x05,0x05,0x05,0x05}},
	{0x00,1,{0xE0}},
	{0xCB,2,{0x05,0x05}},
	{0x00,1,{0xF0}}, //CBF0
	{0xCB,8,{0x00,0x00,0xFF,0x3F,0x50,0x30,0x03,0x00}},
	{0x00,1,{0x80}}, //CD80
	{0xCD,15,{0x05,0x07,0x24,0x24,0x24,0x24,0x01,0x03,0x17,0x24,0x12,0x11,0x10,0x0F,0x0E}},
	{0x00,1,{0x90}}, //CD90
	{0xCD,3,{0x0D,0x24,0x24}},
	{0x00,1,{0xA0}}, //CDA0
	{0xCD,15,{0x04,0x06,0x24,0x24,0x24,0x13,0x14,0x02,0x24,0x24,0x12,0x11,0x10,0x0F,0x0E}},
	{0x00,1,{0xB0}}, //CDB0
	{0xCD,3,{0x0D,0x24,0x24}},
	{0x00,1,{0xA0}}, //B3A0
	{0xB3,7,{0x03,0x04,0x38,0x07,0x80,0x11,0x48}}, 
	{0x00,1,{0x86}}, //B086
	{0xB0,1,{0x0B}},
	{0x00,1,{0x80}},  //C580
	{0xC5,10,{0x00,0xC1,0xDD,0xC4,0x14,0x1E,0x00,0x55,0x50,0x03}}, // sx=3v
	{0x00,1,{0x90}},  //C590
	{0xC5,10,{0x44,0x24,0x14,0xC0,0x88,0x00,0x29,0x37,0x55,0x50}}, // VGH=10.1v  VGL=-10.5v
	{0x00,1,{0x90}}, 
	{0xCF,4,{0xFF,0x00,0xFE,0x00}}, 
	{0x00,1,{0x00}},  //GVDD/NGVDD=+/-5.25
	{0xD8,2,{0x30,0x30}}, 
	{0x00,1,{0x00}}, 
	{0xD9,5,{0x80,0x79,0x79,0x79,0x79}}, 
	{0x00,1,{0xC1}},  //VDD18%LVDSVDD
	{0xC5,1,{0x44}},             
	{0x00,1,{0x80}},  //SD_SAP 
	{0xC4,1,{0x41}}, 
	{0x00,1,{0x94}},  //IBIAS(GP,AP)
	{0xC5,1,{0x48}}, 
	{0x00,1,{0x95}},  //VGH/VGL Pump x2
	{0xC5,1,{0x00}}, 
	{0x00,1,{0xC0}}, 
	{0xC0,2,{0X01,0X10}}, 
	{0x00,1,{0x88}}, //B3A0
	{0xC3,2,{0x33,0x33}}, 
	{0x00,1,{0x98}}, //B3A0
	{0xC3,2,{0x33,0x33}},
	{0x00,1,{0x84}}, 
	{0xA5,1,{0x80}}, 
	{0x00,1,{0x84}}, 
	{0xCF,2,{0x10,0xA0}}, 
	{0x00,1,{0xD1}}, //CFD1 // add noise
	{0xCF,4,{0x02,0x04,0x02,0xC9}},
	{0x00,1,{0xD7}}, //CFD7 // add noise
	{0xCF,4,{0x02,0x04,0x02,0xC9}},
    {0x00,1,{0xB0}},
    {0xF6,3,{0x69,0x16,0x1F}},
	{0x00,1,{0x00}},
	{0xff,3,{0x00,0x00,0x00}},
	{0x00,1,{0x80}},
	{0xff,2,{0x00,0x00}},
#endif
	{0x00, 1, {0x00}},
	{0x99, 2, {0x95, 0x27}},
#ifdef ESD_CHECK
	{0x35, 1, {0}},
#endif
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_dispon[] = {
	{0x29, 0, {} },
	{0x00, 1, {0x00}},
	{0x99, 2, {0,0}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_dispoff[] = {
	{0x00, 1, {0x00}},
	{0x99, 2, {0x95, 0x27}},
	{ 0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{ 0x10, 0, {} },
	{0x00, 1, {0x00}},
	{0x99, 2, {0,0}},
	{ REGFLAG_DELAY, 120, {} },
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
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/**
 * LCM Driver Implementations
 */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->lcm_if = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if = LCM_INTERFACE_DSI0;
	params->dsi.mode   = BURST_VDO_MODE;/*have no SYNC_PULSE_VDO_MODE; */
	//params->dsi.mode   = CMD_MODE;/*have no SYNC_PULSE_VDO_MODE; */
	//params->dsi.mode =SYNC_PULSE_VDO_MODE;

//	params->dsi.dual_dsi_type = DUAL_DSI_VDO;
	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	/*The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	params->dsi.ssc_disable = 1;
	params->dsi.ssc_range = 3;
	/*video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 2; //2
	params->dsi.vertical_backporch					= 16; //6
	params->dsi.vertical_frontporch					= 16; //6
	params->dsi.vertical_frontporch_for_low_power			= 600;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 4;
	params->dsi.horizontal_backporch				= 32;//10;/* hsa+hbp 60~80; */
	params->dsi.horizontal_frontporch				= 32;//90;/* >150 */
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 450; /*this value must be in MTK suggested table */
#else
	/* params->dsi.PLL_CLOCK = 480; */
	params->dsi.PLL_CLOCK = 430;
#endif
#ifdef ESD_CHECK
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable      = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x53;/*0x0A; */
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x2C;/*0x1C; */
#endif
	lcm_print("get lcm parameters\n");
}

static unsigned int lcm_power_switch(unsigned int enable)
{
	unsigned int ret = 0;

	lcm_print("%s enable = %d \n", __func__, enable);

	if (enable)
	{
#ifdef BUILD_LK
		mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCM_RST, 0);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
		MDELAY(5);
		mt_set_gpio_out(GPIO_LCM_RST, 1);
		MDELAY(5);
		mt_set_gpio_out(GPIO_LCD_ENP_PIN, GPIO_OUT_ONE);
		MDELAY(5);
		mt_set_gpio_out(GPIO_LCD_ENN_PIN, GPIO_OUT_ONE);
		MDELAY(5);
		mt_set_gpio_out(GPIO_LCM_RST, 0);
		MDELAY(1);
		mt_set_gpio_out(GPIO_LCM_RST, 1);
		MDELAY(10);
#else
		set_gpio_lcd_vddi18(1);
		MDELAY(5);
		SET_RESET_PIN(0);
		MDELAY(5);
		SET_RESET_PIN(1);
		MDELAY(5);
		set_gpio_lcd_bias_enp(1);
		MDELAY(5);
		set_gpio_lcd_bias_enn(1);
		MDELAY(5);
		SET_RESET_PIN(0);
		MDELAY(1);
		SET_RESET_PIN(1);
		MDELAY(10);
#endif
	}else{
#ifdef BUILD_LK
		mt_set_gpio_out(GPIO_LCD_ENN_PIN, GPIO_OUT_ZERO);
		MDELAY(5);
		mt_set_gpio_out(GPIO_LCD_ENP_PIN, GPIO_OUT_ZERO);
		MDELAY(5);
#else
		set_gpio_lcd_bias_enn(0);
		MDELAY(5);
		set_gpio_lcd_bias_enp(0);
		MDELAY(5);
		set_gpio_lcd_vddi18(0);
#endif
		SET_RESET_PIN(0);
	}

	return ret;
}

static void lcm_init_power(void)
{
}


static void lcm_resume_power(void)
{
}


static void lcm_suspend_power(void)
{
}


static void lcm_init(void)
{
    //unsigned int data_array[16];
    //unsigned char buffer[16] = {0};
	unsigned int array[16];
	unsigned char c3[4] = {0};

	lcm_power_switch(1);
#ifdef BUILD_LK
	read_reg_v2(0xDA, &c3[0], 1);
	read_reg_v2(0xDB, &c3[1], 1);
	read_reg_v2(0xDC, &c3[2], 1);
	id_code = (c3[2] | (c3[1] << 8) | (c3[0] << 16));
	lcm_print("LCM ID code 0x%x\n", id_code);
#endif
	/*read routine*/
	//data_array[0] = 0x00013700;
	//dsi_set_cmdq(data_array, 1, 1);
	//read_reg_v2(0x0C, buffer, 1);
	//lcm_print("0x0c = %x\n", buffer[0]);
	push_table(lcm_slpout, sizeof(lcm_slpout)/sizeof(struct LCM_setting_table),1);
	push_table(lcm_dispon, sizeof(lcm_dispon)/sizeof(struct LCM_setting_table),1);
}
#define POWER_OFF_SUSPEND
static void lcm_suspend(void)
{
	push_table(lcm_dispoff, sizeof(lcm_dispoff)/sizeof(struct LCM_setting_table), 1);
#ifdef  POWER_OFF_SUSPEND
	lcm_power_switch(0);
#endif
}

static void lcm_resume(void)
{
#ifndef BUILD_LK
	tpd_config_gpio();
#endif
#ifdef  POWER_OFF_SUSPEND
	lcm_power_switch(1);
#endif
	push_table(lcm_slpout, sizeof(lcm_slpout)/sizeof(struct LCM_setting_table),1);
	push_table(lcm_dispon, sizeof(lcm_dispon)/sizeof(struct LCM_setting_table), 1);
}

#if 1 /* defined but not used */
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
	/*BEGIN PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/
	/*delete high speed packet */
	/*data_array[0] =0x00290508; */
	/*dsi_set_cmdq(data_array, 1, 1); */
	/*END PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

static unsigned int lcm_compare_id(void)
{
	return id_code;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	return 1;
#endif
}

LCM_DRIVER ft8716_fhd_vdo_sharp_lcm_drv = {
	.name           = "ft8716_fhd_vdo_sharp",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power	= lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
	.ata_check	= lcm_ata_check,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};
