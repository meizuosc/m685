#ifdef BUILD_LK
    #include <platform/mt_gpio.h>	
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
    #include <platform/mt_pmic.h>
#else
    #include <mt-plat/mt_gpio.h>
    #include <mach/gpio_const.h>
#endif
#include <mt-plat/mt_gpio_core.h>//add by cassy 2
#include "lcm_drv.h"

#ifdef BUILD_LK
	#define LCD_DEBUG(fmt, args...) printf(fmt, ##args)		
#else
	#define LCD_DEBUG(fmt, args...) printk(fmt, ##args)
#endif

#ifndef BUILD_LK
	#include <linux/string.h>
	#include <linux/kernel.h>
	#include <linux/module.h>  
	#include <linux/fs.h>
	#include <linux/slab.h>
	#include <linux/init.h>
	#include <linux/list.h>
	#include <linux/i2c.h>
	#include <linux/irq.h>
	//#include <linux/jiffies.h>
	#include <linux/uaccess.h>
	//#include <linux/delay.h>
	#include <linux/interrupt.h>
	#include <linux/io.h>
	#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING
#define TPS_I2C_BUSNUM  		0//for I2C channel 0
#define I2C_ID_NAME 			"nt50358"
#define TPS_ADDR 			0x3E
//add by cassy 2 begin
#define ENP_PIN 138
#define ENN_PIN 65
//add by cassy 2 end
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata nt50358_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};

#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{ .compatible = "mediatek,i2c_lcd_bias" },
		{},
};
#endif
static struct i2c_client *nt50358_i2c_client = NULL;

/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int nt50358_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int nt50358_remove(struct i2c_client *client);

/***************************************************************************** 
 * Data Structure
 *****************************************************************************/
 struct nt50358_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id nt50358_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver nt50358_iic_driver = {
	.id_table	= nt50358_id,
	.probe		= nt50358_probe,
	.remove		= nt50358_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "nt50358",
#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
#endif
	},
 
};

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int nt50358_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	LCD_DEBUG("*********r63350 nt50358_I2C_probe\n");
	LCD_DEBUG("*********r63350 TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	nt50358_i2c_client  = client;		
	return 0;      
}

static int nt50358_remove(struct i2c_client *client)
{  	
	LCD_DEBUG( "*********r63350 nt50358_I2C_remove\n");
	nt50358_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

 static int nt50358_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = nt50358_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    	ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	LCD_DEBUG("*********r63350 nt50358 write data fail !!\n");	
	return ret ;
}

static int __init nt50358_iic_init(void)
{

   LCD_DEBUG( "*********r63350 nt50358_iic_init\n");
   i2c_register_board_info(TPS_I2C_BUSNUM, &nt50358_board_info, 1);
   LCD_DEBUG( "*********r63350 nt50358_iic_init2\n");
   i2c_add_driver(&nt50358_iic_driver);
   LCD_DEBUG( "*********r63350 nt50358_iic_init success\n");	
   return 0;
}

static void __exit nt50358_iic_exit(void)
{
  LCD_DEBUG( "*********r63350 nt50358_iic_exit\n");
  i2c_del_driver(&nt50358_iic_driver);  
}


module_init(nt50358_iic_init);
module_exit(nt50358_iic_exit);

MODULE_AUTHOR("Longfang.liu");
MODULE_DESCRIPTION("MTK nt50358 I2C Driver");
MODULE_LICENSE("GPL"); 
#endif
#endif

#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
#define nt50358_SLAVE_ADDR_WRITE  0x7C  //(0X3E<<1==0X7C)
#include <platform/mt_i2c.h>

static struct mt_i2c_t nt50358_i2c;
static int nt50358_write_byte_lk(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    nt50358_i2c.id = 0;//I2C0;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    nt50358_i2c.addr = (nt50358_SLAVE_ADDR_WRITE >> 1);
    nt50358_i2c.mode = ST_MODE;
    nt50358_i2c.speed = 100;
    len = 2;
    ret_code = i2c_write(&nt50358_i2c, write_data, len);
    return ret_code;
}

#endif
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE					0
#define FRAME_WIDTH  						(1080)
#define FRAME_HEIGHT 						(1920)
#define REGFLAG_DELAY						0xFC
#define REGFLAG_END_OF_TABLE					0xFD

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

/*#define GPIO_LCD_ENN        			(GPIO65   | 0x80000000)
#define GPIO_LCD_ENP        			(GPIO138  | 0x80000000)
#define GPIO_LCM_ID1				(GPIO135  | 0x80000000)
#define GPIO_LCM_ID2				(GPIO140  | 0x80000000)*/

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS 			lcm_util = {0};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define SET_RESET_PIN(v)					(lcm_util.set_reset_pin((v)))
#define MDELAY(n)						(lcm_util.mdelay(n))
#define UDELAY(n)						(lcm_util.udelay(n))
#define dsi_set_cmdq_V3(para_tbl,size,force_update)		lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   
#define set_gpio_lcd_enp(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) \
	lcm_util.set_gpio_lcd_enn_bias(cmd)

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xB0, 1,  {0x04} },
	{0xB3, 6,  {0x14, 0x00, 0x00,0x00, 0x00, 0x00} },
	{0xB4, 2,  {0x0C, 0x00} },
	{0xB6, 3,  {0x4B, 0xDB, 0x00} }, //2th 0xCB change to 0xDB on phone platfrom; MIPI Stop closed
	{0xBB, 2,  {0x14, 0x14} },
	{0xBC, 2,  {0x37, 0x32} },
	{0xBD, 2,  {0x64, 0x32} },
	{0xBE, 2,  {0x00, 0x04} },
	{0xC0, 1,  {0x00} },
	{0xC1, 34, {0xC4, 0x60, 0x10, 0xF1, 0xFF, 0xCF, 0xDA, 0xFF, 
		    0xFF, 0x9F, 0x12, 0xE4, 0xA4, 0x74, 0x4A, 0x40, 
		    0xCC, 0xFF, 0xFF, 0x37, 0xF6, 0xFF, 0x8F, 0x10, 
		    0x10, 0x10, 0x10, 0x00, 0x62, 0x01, 0x03, 0x22, 
		    0x00, 0x01} },

	{0xC2, 8,  {0x31, 0xF7, 0x80, 0x08, 0x08, 0x00, 0x00, 0x08} },
	{0xC3, 3,  {0x00, 0x00, 0x00} },
	{0xC4, 11, {0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		    0x00, 0x06, 0x05, } },
	{0xC6, 21, {0xC8, 0x01, 0x69, 0x01, 0x69, 0x00, 0x00, 0x00, 
		    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x0B, 0x17, 0x09, 0xC8} },
	{0xC7, 30, {0x00, 0x07, 0x0C, 0x14, 0x22, 0x30, 0x3A, 0x4B,
		    0x30, 0x38, 0x46, 0x55, 0x63, 0x6C, 0x76, 0x00,
		    0x07, 0x0C, 0x14, 0x22, 0x30, 0x3A, 0x4B, 0x30,
		    0x38, 0x46, 0x55, 0x63, 0x6C, 0x76} },
	
	{0xCB, 15, {0x31, 0xFC, 0x3F, 0x8C, 0x00, 0x00, 0x00, 0x00,
		    0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00} },	
	{0xCC, 1,  {0x0B} },
	{0xD0, 10, {0x11, 0x00, 0x00, 0x59, 0xD9, 0x40, 0x19, 0x19,
		    0x09, 0x00} },
	{0xD1, 4,  {0x00, 0x48, 0x16, 0x0F} },	
	{0xD3, 26, {0x1B, 0x33, 0xBB, 0xBB, 0xB3, 0x33, 0x33, 0x33,
		    0x33, 0x01, 0x01, 0x00, 0x00, 0xD8, 0xA0, 0x0D, 
		    0x1F, 0x1F, 0x33, 0x33, 0x72, 0x12, 0x8A, 0x07,
		    0x3D, 0xBC} },

	{0xD4, 3,  {0x41,0x04,0x00} },	
	{0xD5, 7,  {0x06, 0x00, 0x00, 0x01, 0x4B, 0x01, 0x4B} },	
	{0xD7, 24, {0xBF, 0xF8, 0x7F, 0xA8, 0xCE, 0x3E, 0xFC, 0xC1, 
		    0xE1, 0xEF, 0x83, 0x07, 0x3F, 0x10, 0x7F, 0xC0,
		    0x01, 0xE7, 0x40, 0x1C, 0x00, 0xC0, 0x00, 0x00} },
	{0xD8, 3, {0x00, 0x00, 0x00} },
	{0xD9, 3, {0x00, 0x00, 0x00} },
	{0xDD, 4, {0x30, 0x06, 0x23, 0x65} },
	
	{0xD6, 1, {0x01} },
	{0x35, 1, {0x00} },

	{0x29, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
#endif

	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	params->dsi.data_format.color_order 		= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq 		= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding 		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format			= LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active 		= 4;
	params->dsi.vertical_backporch 			= 4;
	params->dsi.vertical_frontporch 		= 8;
	params->dsi.vertical_active_line 		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active 		= 8;
	params->dsi.horizontal_backporch 		= 60;
	params->dsi.horizontal_frontporch 		= 140;
	params->dsi.horizontal_active_pixel 		= FRAME_WIDTH;
	                                                    
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 500;
#else
	params->dsi.PLL_CLOCK = 450;
#endif
	params->dsi.ssc_disable = 1;  
}

static void push_table(struct LCM_setting_table *table, unsigned int count,
unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		unsigned cmd;

        cmd = table[i].cmd;

        switch (cmd) {

	case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

	case REGFLAG_END_OF_TABLE :
                break;

	default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
	}
    }
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	//Set AVDD to +5.5V
	cmd=0x00;
	data=0x0F;   //data=0x0A;VSP=5V;data=0x0E;VSP=5.4V

#ifndef CONFIG_FPGA_EARLY_PORTING
	//enable power
	//add by cassy 2 begin
	mt_set_gpio_mode_base(ENP_PIN,0);
mt_set_gpio_dir_base(ENP_PIN,1);
mt_set_gpio_pull_enable_base(ENP_PIN,0);
mt_set_gpio_out_base(ENP_PIN,1);
MDELAY(5);
	mt_set_gpio_mode_base(ENN_PIN,0);
mt_set_gpio_dir_base(ENN_PIN,1);
mt_set_gpio_pull_enable_base(ENN_PIN,0);
mt_set_gpio_out_base(ENN_PIN,1);
	//add by cassy 2 end
	
	
	//delete by cassy 2 begin
	//set_gpio_lcd_enp(1);
	//MDELAY(5);
	//set_gpio_lcd_enn(1);
//delete by cassy 2 end
#ifdef BUILD_LK
	ret=nt50358_write_byte_lk(cmd,data);
	if(ret)    	
	LCD_DEBUG("[LK]-----nt50358----cmd=%0x--i2c write error----\n",cmd);    	
	else
	LCD_DEBUG("[LK]----nt50358----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=nt50358_write_bytes(cmd,data);
	if(ret<0)
	LCD_DEBUG("[KERNEL]-----nt50358---cmd=%0x-- i2c write error-----\n",cmd);
	else
	LCD_DEBUG("[KERNEL]-----nt50358---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	//Set AVEE to -5.5V
	cmd=0x01;
	data=0x0F;
#ifdef BUILD_LK
	ret=nt50358_write_byte_lk(cmd,data);
    if(ret)    	
	LCD_DEBUG("[LK]-----nt50358----cmd=%0x--i2c write error----\n",cmd);    	
	else
	LCD_DEBUG("[LK]----nt50358----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=nt50358_write_bytes(cmd,data);
	if(ret<0)
	LCD_DEBUG("[KERNEL]-----nt50358---cmd=%0x-- i2c write error-----\n",cmd);
	else
	LCD_DEBUG("[KERNEL]-----nt50358---cmd=%0x-- i2c write success-----\n",cmd);
#endif
#endif

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);	

	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef BUILD_LK
	LCD_DEBUG("lcm_init func:LK Flash3_r63350_fhd_vdo_tdt lcm init ok!\n");
#else
	LCD_DEBUG("lcm_init func:Kernel Flash3_r63350_fhd_vdo_tdt lcm init ok!\n");
#endif
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting,
		   sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

	SET_RESET_PIN(0);
	MDELAY(10);
//add by cassy 2 begin
mt_set_gpio_mode_base(ENN_PIN,0);
mt_set_gpio_dir_base(ENN_PIN,1);
mt_set_gpio_pull_enable_base(ENN_PIN,0);
mt_set_gpio_out_base(ENN_PIN,0);

MDELAY(5);
mt_set_gpio_mode_base(ENP_PIN,0);
mt_set_gpio_dir_base(ENP_PIN,1);
mt_set_gpio_pull_enable_base(ENP_PIN,0);
mt_set_gpio_out_base(ENP_PIN,0);	
	//add by cassy 2 end
	
	//delete by cassy 2 begin
	//set_gpio_lcd_enn(0);
	//MDELAY(5);
	//set_gpio_lcd_enp(0);
    //delete by cassy 2 end	
	LCD_DEBUG("lcm_suspend func:Flash3_r63350_fhd_vdo_tdt lcm suspend ok!\n");
}

static void lcm_resume(void)
{

    lcm_init();
    LCD_DEBUG("lcm_resume func:Flash3_r63350_fhd_vdo_tdt lcm resume ok!\n");
}

#define LCM_ID_r63350  (0x3350)

static unsigned int lcm_compare_id(void)
{
	unsigned char buffer[5];
	unsigned int array[16];
	unsigned int lcd_id = 0;

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	array[0] = 0x00053700;/* read id return two byte, version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xBF, buffer, 5); //BFH:0x2 0x3c 0x33 0x50 0x0
	MDELAY(20);
	lcd_id = (buffer[2] << 8) | buffer[3];

#ifdef BUILD_LK
	LCD_DEBUG("lcm_compare_id func %s, LK r63350 debug: r63350 id = 0x%08x\n", __func__, lcd_id);
#else
	LCD_DEBUG("lcm_compare_id func %s, kernel r63350 debug: r63350 id = 0x%08x\n", __func__, lcd_id);
#endif

	if (lcd_id == LCM_ID_r63350)
		return 1;
	else
		return 0;
}

LCM_DRIVER r63350_fhd_vdo_tdt_flash3_lcm_drv =
{
    .name           = "r63350_fhd_vdo_tdt_flash3",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
   
};
