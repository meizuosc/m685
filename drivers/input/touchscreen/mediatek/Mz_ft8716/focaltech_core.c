/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/input/mz_gesture_ts.h>
#include <linux/meizu-sys.h>
#include "focaltech_core.h"
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <mt_i2c.h>

/* #define TIMER_DEBUG */
#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#undef CONFIG_MTK_SENSOR_HUB_SUPPORT
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif

#if CTP_ESD_PROTECT
int apk_debug_flag = 0;
int power_switch_gesture = 0;
static int tp_ready = 0;
void ctp_esd_check_func(void);
#endif

static int tp_suspend  = 0;
enum {
	TP_VDDI = 0,
	LCD_ENP,
	LCD_ENN,
	MAX_REGURATORS,
};

static const char *fts_regulator_name[MAX_REGURATORS] = {
	"tpvddi",
	"lcd_enp",
	"lcd_enn",
};

static struct regulator *fts_regulator[MAX_REGURATORS];

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
enum DOZE_T {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static DOZE_T doze_status = DOZE_DISABLED;
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client);

enum TOUCH_IPI_CMD_T {
	/* SCP->AP */
	IPI_COMMAND_SA_GESTURE_TYPE,
	/* AP->SCP */
	IPI_COMMAND_AS_CUST_PARAMETER,
	IPI_COMMAND_AS_ENTER_DOZEMODE,
	IPI_COMMAND_AS_ENABLE_GESTURE,
	IPI_COMMAND_AS_GESTURE_SWITCH,
};

struct Touch_Cust_Setting {
	u32 i2c_num;
	u32 int_num;
	u32 io_int;
	u32 io_rst;
};

struct Touch_IPI_Packet {
	u32 cmd;
	union {
		u32 data;
		Touch_Cust_Setting tcs;
	} param;
};
/* static bool tpd_scp_doze_en = FALSE; */
static bool tpd_scp_doze_en = TRUE;
DEFINE_MUTEX(i2c_access);
#endif

struct task_struct *thread_tpd = NULL;
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client 				= NULL;
struct input_dev *fts_input_dev					=NULL;

#ifdef TPD_AUTO_UPGRADE
static bool is_update = false;
#endif

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;
#endif

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static int tpd_flag;
static u8 fts_gesture_state = MZ_GESTURE_DISABLE;
static u8 finger_num = 0;
unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
unsigned int touch_irq = 0;

#define TPD_OK 		0
/* Register define */
#define DEVICE_MODE	0x00
#define GEST_ID		0x01
#define TD_STATUS	0x02

#define TOUCH1_XH	0x03
#define TOUCH1_XL	0x04
#define TOUCH1_YH	0x05
#define TOUCH1_YL	0x06

#define TOUCH2_XH	0x09
#define TOUCH2_XL	0x0A
#define TOUCH2_YH	0x0B
#define TOUCH2_YL	0x0C

#define TOUCH3_XH	0x0F
#define TOUCH3_XL	0x10
#define TOUCH3_YH	0x11
#define TOUCH3_YL	0x12

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3

#ifdef TIMER_DEBUG
static struct timer_list test_timer;

static void timer_func(unsigned long data)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	mod_timer(&test_timer, jiffies + 100*(1000/HZ));
}

static int init_test_timer(void)
{
	memset((void *)&test_timer, 0, sizeof(test_timer));
	test_timer.expires  = jiffies + 100*(1000/HZ);
	test_timer.function = timer_func;
	test_timer.data     = 0;
	init_timer(&test_timer);
	add_timer(&test_timer);
	return 0;
}
#endif

#if defined(CONFIG_TPD_ROTATE_90) || defined(CONFIG_TPD_ROTATE_270) || defined(CONFIG_TPD_ROTATE_180)
/*
static void tpd_swap_xy(int *x, int *y)
{
	int temp = 0;

	temp = *x;
	*x = *y;
	*y = temp;
}
*/
/*
static void tpd_rotate_90(int *x, int *y)
{
//	int temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
static void tpd_rotate_180(int *x, int *y)
{
	*y = TPD_RES_Y + 1 - *y;
	*x = TPD_RES_X + 1 - *x;
}
/*
static void tpd_rotate_270(int *x, int *y)
{
//	int temp;

	*y = TPD_RES_Y + 1 - *y;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
#endif

struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

#ifdef CONFIG_MTK_I2C_EXTENSION
	u8 *g_dma_buff_va = NULL;
	dma_addr_t g_dma_buff_pa = 0;
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION

	static void msg_dma_alloct(void)
	{
	    if (NULL == g_dma_buff_va)
    		{
       		 tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
       		 g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
    		}

	    	if(!g_dma_buff_va)
		{
	        	TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	    	}
	}
	static void msg_dma_release(void){
		if(g_dma_buff_va)
		{
	     		dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
	        	g_dma_buff_va = NULL;
	        	g_dma_buff_pa = 0;
			TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	    	}
	}
#endif

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
/* static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX; */
/* static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX; */
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id ft8716_tpd_id[] = {{"ft8716", 0}, {} };
static const struct of_device_id ft8716_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, ft8716_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft8716_dt_match),
		.name = "cap_touch",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft8716_tpd_id,
	.detect = tpd_i2c_detect,
};

static int of_get_ft8716_platform_data(struct device *dev)
{
	int ret, num;
	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ft8716_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}

	ret = of_property_read_u32(dev->of_node, "rst-gpio", &num);
	if (!ret)
		tpd_rst_gpio_number = num;
	ret = of_property_read_u32(dev->of_node, "int-gpio", &num);
	if (!ret)
		tpd_int_gpio_number = num;

	TPD_DMESG("reset_gpio_number %d\n", tpd_rst_gpio_number);
	TPD_DMESG("eint_gpio_number  %d\n", tpd_int_gpio_number);
	return 0;
}

static struct kobject * sysfs_get_devices_parent(void)
{
	struct device *tmp = NULL;
	struct kset *pdevices_kset;

	 tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	 if (!tmp){
		 return NULL;
	 }

	 device_initialize(tmp);
	 pdevices_kset = tmp->kobj.kset;
	 kfree(tmp);
	 return &pdevices_kset->kobj;
}

void fts_gesture_handler(u8 state)
{
	fts_gesture_state = state;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static ssize_t show_scp_ctrl(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t store_scp_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u32 cmd;
	Touch_IPI_Packet ipi_pkt;

	if (kstrtoul(buf, 10, &cmd)) {
		TPD_DEBUG("[SCP_CTRL]: Invalid values\n");
		return -EINVAL;
	}

	TPD_DEBUG("SCP_CTRL: Command=%d", cmd);
	switch (cmd) {
	case 1:
	    /* make touch in doze mode */
	    tpd_scp_wakeup_enable(TRUE);
	    tpd_suspend(NULL);
	    break;
	case 2:
	    tpd_resume(NULL);
	    break;
	/*
	case 3:
	    // emulate in-pocket on
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 1;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;
	case 4:
	    // emulate in-pocket off
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 0;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;
	*/
	case 5:
	    {
		Touch_IPI_Packet ipi_pkt;
		ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
		ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
		ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
		ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
		ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;
		if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0) < 0)
			TPD_DEBUG("[TOUCH] IPI cmd failed (%d)\n", ipi_pkt.cmd);
			break;
	    }
	default:
	    TPD_DEBUG("[SCP_CTRL] Unknown command");
	    break;
	}

	return size;
}
static DEVICE_ATTR(tpd_scp_ctrl, 0664, show_scp_ctrl, store_scp_ctrl);
#endif

static struct device_attribute *ft8716_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif
};

#ifndef MT_PROTOCOL_B
static void tpd_down(int x, int y, int p, int id)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
		TPD_DEBUG("%s x:%d y:%d p:%d\n", __func__, x, y, p);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(tpd->dev);
	}
}

static void tpd_up(int x, int y)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);
	}
}

#ifdef TPD_HAVE_CALIBRATION
static void tpd_calibrate_driver(int *x, int *y)
{
	int tx;

	tx = ((tpd_def_calmat[0] * (*x)) + (tpd_def_calmat[1] * (*y)) + (tpd_def_calmat[2])) >> 12;
	*y = ((tpd_def_calmat[3] * (*x)) + (tpd_def_calmat[4] * (*y)) + (tpd_def_calmat[5])) >> 12;
	*x = tx;
}
#endif

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[80] = {0};
	u16 high_byte, low_byte;
	char writebuf[10]={0};

	writebuf[0]=0x00;
	fts_i2c_read(i2c_client, writebuf,  1, data, 32);
	if ((data[0] & 0x70) != 0)
		return false;

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	for (i = 0; i < TPD_SUPPORT_POINTS; i++)
		cinfo->p[i] = 1;	/* Put up */

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;
	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);

	TPD_DEBUG("Procss raw data...\n");

	for (i = 0; i < cinfo->count; i++) {
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
		cinfo->id[i] = data[3+6*i+2]>>4; 						// touch id

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;

		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
		cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}

	return true;
};
#endif

 /************************************************************************
* Name: fts_read_Touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
#ifdef MT_PROTOCOL_B
static int fts_read_Touchdata(struct ts_event *data)
{
       u8 buf[POINT_READ_BUF] = { 0 };//0xFF
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_ID;
	//mutex_lock(&i2c_access);
	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0)
	{
		dev_err(&fts_i2c_client->dev, "%s read touchdata failed.\n",__func__);
		//mutex_unlock(&i2c_access);
		return ret;
	}

	//mutex_unlock(&i2c_access);
	memset(data, 0, sizeof(struct ts_event));
	data->touch_point = 0;
	data->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;
	finger_num = data->touch_point_num;

	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
	{
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			data->touch_point++;
		data->au16_x[i] =
		    (s16) (buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
		data->au16_y[i] =
		    (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
		data->au8_touch_event[i] =
		    buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
		data->au8_finger_id[i] =
		    (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;

		data->pressure[i] =
			(buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);	//cannot constant value
		data->area[i] =
			(buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;

		if((data->au8_touch_event[i]==0 || data->au8_touch_event[i]==2)&&((data->touch_point_num==0)/*||(data->pressure[i]==0 && data->area[i]==0  )*/))
			return 1;

		if(data->pressure[i]<=0)
		{
			data->pressure[i]=0x3f;
		}
		if(data->area[i]<=0)
		{
			data->area[i]=0x05;
		}

		//if ( pinfo->au16_x[i]==0 && pinfo->au16_y[i] ==0)
		//	pt00f++;
	}

	return 0;
}
#endif

 /************************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
#ifdef MT_PROTOCOL_B
static int fts_report_value(struct ts_event *data)
 {
	int i = 0,j=0;
	int up_point = 0;
 	int touchs = 0;

	for (i = 0; i < data->touch_point; i++)
	{
		 input_mt_slot(tpd->dev, data->au8_finger_id[i]);

		if (data->au8_touch_event[i]== 0 || data->au8_touch_event[i] == 2)
		{
			 input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
			 input_report_abs(tpd->dev, ABS_MT_PRESSURE,data->pressure[i]);
			 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,data->area[i]);
			 input_report_abs(tpd->dev, ABS_MT_POSITION_X,data->au16_x[i]);
			 input_report_abs(tpd->dev, ABS_MT_POSITION_Y,data->au16_y[i]);
			 touchs |= BIT(data->au8_finger_id[i]);
   			 data->touchs |= BIT(data->au8_finger_id[i]);
		}
		else
		{
			 up_point++;
			 input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
			 data->touchs &= ~BIT(data->au8_finger_id[i]);
		}

	}
 	//if(unlikely(data->touchs ^ touchs)){
		for(i = 0; i < 10; i++){
			if(BIT(i) & (data->touchs ^ touchs)){
				printk("\n Up by manual  id=%d \n", i);

				data->touchs &= ~BIT(i);
				input_mt_slot(tpd->dev, i);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
			}
		}
	//}

	data->touchs = touchs;
	if((data->touch_point_num==0))    // release all touches in final
	{
		for(j = 0; j <10; j++)
		{
			input_mt_slot( tpd->dev, j);
			input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
		}
		//last_touchpoint=0;
		data->touchs=0;
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_sync(tpd->dev);

		return 0;
    	}

	if(data->touch_point == up_point)
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
	else
		 input_report_key(tpd->dev, BTN_TOUCH, 1);

	input_sync(tpd->dev);
	return 0;

}
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION
int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
#if CTP_ESD_PROTECT
	int i = 0;
	//	fts_esd_protection_notice();
	for(i = 0; i < 3; i++)
	{
		ret = fts_esd_protection_notice();
		if(0 == ret)
			break;	// can use I2C
		else
		{
			//FTS_COMMON_DBG("[focal] fts_esd_protection_notice return :%d \n", ret);
			continue;
		}
	}
	if(3 == i)
	{
		//FTS_COMMON_DBG("[focal] ESD are still use I2C. \n");
	}
#endif

	mutex_lock(&i2c_rw_access);
	if(writelen!=0)
	{
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
			dev_err(&client->dev, "###%s i2c write len=%x faild ###\n", __func__, ret);
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}
	if(readlen!=0)
	{
		client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);
		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);
		memcpy(readbuf, g_dma_buff_va, readlen);
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}
	mutex_unlock(&i2c_rw_access);
	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	mutex_lock(&i2c_rw_access);
	memcpy(g_dma_buff_va, writebuf, writelen);
	client->addr = ((client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG);
	if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
		TPD_DMESG("i2c write failed\n");
	client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

	mutex_unlock(&i2c_rw_access);

	return ret;
}

#else

int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
#if CTP_ESD_PROTECT
	int i = 0 ;
	//	fts_esd_protection_notice();
		for(i = 0; i < 3; i++)
		{
			ret = fts_esd_protection_notice();
			if(0 == ret)
				break;	// can use I2C
			else
			{
				//FTS_COMMON_DBG("[focal] fts_esd_protection_notice return :%d \n", ret);
				continue;
			}
		}
		if(3 == i)
		{
			//FTS_COMMON_DBG("[focal] ESD are still use I2C. \n");
		}
#endif
	mutex_lock(&i2c_rw_access);
	if(readlen > 0)
	{
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
					 .addr = client->addr,
					 .flags = 0,
					 .len = writelen,
					 .buf = writebuf,
				 },
				{
					 .addr = client->addr,
					 .flags = I2C_M_RD,
					 .len = readlen,
					 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret < 0)
				dev_err(&client->dev, "%s: i2c read error.\n", __func__);
		} else {
			struct i2c_msg msgs[] = {
				{
					 .addr = client->addr,
					 .flags = I2C_M_RD,
					 .len = readlen,
					 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0)
				dev_err(&client->dev, "%s:i2c read error.\n", __func__);
		}
	}

	mutex_unlock(&i2c_rw_access);
	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;
	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};

#if CTP_ESD_PROTECT
	fts_esd_protection_notice();
#endif

	mutex_lock(&i2c_rw_access);

	if(writelen > 0)
	{
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c write error.\n", __func__);
	}

	mutex_unlock(&i2c_rw_access);
	return ret;
}

#endif
/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;
	return fts_i2c_write(client, buf, sizeof(buf));
}
/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{

	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);

}

static void tpd_charger_check(int resume)
{
	static int ft5x46_charger_state = 0;

	int ret = 0;
	int charger_state = upmu_is_chr_det();
	TPD_DEBUG("[FTS] charger_state: %d, cur_state: %d\n", charger_state, ft5x46_charger_state);

	if(resume || (ft5x46_charger_state != charger_state)) {
		ft5x46_charger_state = charger_state;

		if(ft5x46_charger_state != 0) { // charger plugged in
			ret = fts_write_reg(fts_i2c_client, 0x8b, 0x03);
		} else {
			ret = fts_write_reg(fts_i2c_client, 0x8b, 0x01);
		}

		if(ret < 0)
			TPD_DEBUG("fts i2c write 0x8b error\n");
	}
}

static int touch_event_handler(void *unused)
{
	int i = 0;
	int ret = 0;
	u8 state = 0;

	struct touch_info finfo;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
#ifdef MT_PROTOCOL_B
       struct ts_event pevent={};
#else
       struct touch_info cinfo, pinfo;
#endif

	if (tpd_dts_data.use_tpd_button) {
		memset(&finfo, 0, sizeof(struct touch_info));
		for (i = 0; i < TPD_SUPPORT_POINTS; i++)
			finfo.p[i] = 1;
	}

	sched_setscheduler(current, SCHED_RR, &param);
	do {
		/*enable_irq(touch_irq);*/
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		tpd_charger_check(0);
		if (fts_gesture_state != MZ_GESTURE_DISABLE) {
			ret = fts_read_reg(fts_i2c_client, 0xd0, &state);
			if (ret < 0)
			{
				msleep(50);
				ret = fts_read_reg(fts_i2c_client, 0xd0, &state);
				if (ret < 0) {
					TPD_DMESG("[Focal][Touch] read 0xd0 value fail");
					continue;
				}
			}

		     	if(state == 1)
		     	{
			        fts_read_Gestruedata();
			        continue;
		    	}
		}

		#ifdef MT_PROTOCOL_B
		{

#if CTP_ESD_PROTECT
			apk_debug_flag = 1;
#endif
            		ret = fts_read_Touchdata(&pevent);
			if (ret == 0) {
				TPD_DEBUG("%s: read touch data success, start report event \n", __func__);
				fts_report_value(&pevent);
			} else {
				TPD_DMESG("%s: read touch data faild \n", __func__);
			}
#if CTP_ESD_PROTECT
				apk_debug_flag = 0;
#endif
		}
		#else
		{
#if CTP_ESD_PROTECT
			apk_debug_flag = 1;
#endif
		if (tpd_touchinfo(&cinfo, &pinfo)) {
			if (tpd_dts_data.use_tpd_button) {
				if (cinfo.p[0] == 0)
					memcpy(&finfo, &cinfo, sizeof(struct touch_info));
			}

			if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
			     && ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
				TPD_DEBUG("Dummy release --->\n");
				tpd_up(pinfo.x[0], pinfo.y[0]);
				input_sync(tpd->dev);
				continue;
			}

			if (tpd_dts_data.use_tpd_button) {
				if ((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y)
				&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
					TPD_DEBUG("Dummy key release --->\n");
					tpd_button(pinfo.x[0], pinfo.y[0], 0);
					input_sync(tpd->dev);
					continue;
				}

			if ((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y)) {
				if (finfo.y[0] > TPD_RES_Y) {
					if ((cinfo.p[0] == 0) || (cinfo.p[0] == 2)) {
							TPD_DEBUG("Key press --->\n");
							tpd_button(pinfo.x[0], pinfo.y[0], 1);
					} else if ((cinfo.p[0] == 1) &&
						((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
							TPD_DEBUG("Key release --->\n");
							tpd_button(pinfo.x[0], pinfo.y[0], 0);
					}
					input_sync(tpd->dev);
				}
				continue;
			}
			}

			if (cinfo.count > 0) {
				for (i = 0; i < cinfo.count; i++)
					tpd_down(cinfo.x[i], cinfo.y[i], i + 1, cinfo.id[i]);
			} else {
#ifdef TPD_SOLVE_CHARGING_ISSUE
				tpd_up(1, 48);
#else
				tpd_up(cinfo.x[0], cinfo.y[0]);
#endif

			}
			input_sync(tpd->dev);

#if CTP_ESD_PROTECT
			apk_debug_flag = 0;
#endif
		}
	}
#endif
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "tpd-eint", NULL);
			if (ret > 0)
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}

static int  tpd_get_regulators(void)
{
	int i = 0;

	for(i=0; i<MAX_REGURATORS; i++){
	 	fts_regulator[i] = regulator_get(tpd->tpd_dev, fts_regulator_name[i]);
		if (IS_ERR(fts_regulator[i])){
			dev_err(tpd->tpd_dev, " Failed to get lcm regulators %s\n", fts_regulator_name[i]);
		}
	}
	return 0;
}

static int  tpd_enable_regulators(void)
{
	int i = 0;
	int ret =0 ;

	for(i=0; i<MAX_REGURATORS; i++){
		if(IS_ERR(fts_regulator[i]))
			continue;
		mdelay(5);//
		ret |= regulator_enable(fts_regulator[i]);
	}
	return ret;
}

static int  tpd_disable_regulators(void)
{
	int i = 0;
	int ret =0 ;

	for(i=MAX_REGURATORS -1; i>=0;  i--){
		if(IS_ERR(fts_regulator[i]))
			continue;
		mdelay(5);//
		ret |= regulator_disable(fts_regulator[i]);
	}
	return ret;
}

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = TPD_OK;
	int reset_count = 0;
	char data;
	int err = 0;

	fts_i2c_client = client;
	fts_input_dev = tpd->dev;
	if(fts_i2c_client->addr != 0x38)
	{
		fts_i2c_client->addr = 0x38;
		TPD_DMESG("fts_i2c_client->addr=%d\n",fts_i2c_client->addr);
	}

	of_get_ft8716_platform_data(&client->dev);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	retval = tpd_enable_regulators();
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	/* set INT mode */
	tpd_gpio_as_int(tpd_int_gpio_number);
	tpd_irq_registration();
	msleep(20);

#ifdef CONFIG_MTK_I2C_EXTENSION
	msg_dma_alloct();
#endif
	fts_Gesture_init(tpd->dev);

#ifdef MT_PROTOCOL_B
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
		input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS);
	#else
		input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS, 2);
	#endif
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#else
        input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, MT_MAX_TOUCH_POINTS, 0, 0);
#endif

reset_proc:
	/* Reset CTP */
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(300);

	err = fts_read_reg(fts_i2c_client, 0x00, &data);
	TPD_DMESG("tpd_probe: fts read 0x0 return %d, data:%d\n", err,data);
	if(err< 0 || data!=0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
		if ( ++reset_count < TPD_MAX_RESET_COUNT )
		{
			goto reset_proc;
		}
#endif
		/*
		retval = tpd_disable_regulators();
		if(retval)
		{
			printk("focaltech tpd_probe regulator_disable() failed!\n");
		}
		*/
	#ifdef CONFIG_MTK_I2C_EXTENSION
		msg_dma_release();
	#endif
		gpio_free(tpd_rst_gpio_number);
		gpio_free(tpd_int_gpio_number);
		return -1;
	}

	tpd_load_status = 1;
	fts_get_upgrade_array();

#ifdef FTS_CTL_IIC
	 if (fts_rw_iic_drv_init(fts_i2c_client) < 0)
		 dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
#endif

#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(fts_i2c_client);
#endif

#ifdef TPD_AUTO_UPGRADE
	TPD_DEBUG("********************Enter CTP Auto Upgrade********************\n");
	if(is_update == false) {
		is_update = true;
		fts_ctpm_auto_upgrade(fts_i2c_client);
		tpd_gpio_as_int(tpd_int_gpio_number);
		msleep(10);
		tpd_gpio_output(tpd_rst_gpio_number, 0);
		msleep(20);
		tpd_gpio_output(tpd_rst_gpio_number, 1);
		msleep(300);
	}
	is_update = false;
#endif

#if CTP_ESD_PROTECT
	tp_ready = 1;
	fts_esd_protection_init();
#endif
	thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd)) {
		retval = PTR_ERR(thread_tpd);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd: %d\n", retval);
	}
	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef TIMER_DEBUG
	init_test_timer();
#endif
	{
		u8 ver;
		fts_read_reg(client, FTS_REG_FW_VER, &ver);
		TPD_DMESG(TPD_DEVICE " fts_read_reg fw version : 0x%x\n", ver);
	}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int ret;
	ret = get_md32_semaphore(SEMAPHORE_TOUCH);
	if (ret < 0)
		TPD_DMESG("[TOUCH] HW semaphore reqiure timeout\n");
#endif

	fts_create_sysfs(fts_i2c_client);
	err = sysfs_create_link(sysfs_get_devices_parent(), &client->dev.kobj, "mx_tsp");
	if (err)
		TPD_DMESG("sysfs_create_link mx_tsp error\n");
	mz_gesture_handle_register(fts_gesture_handler);
	meizu_sysfslink_register_name(&client->dev, "tp");

	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
#ifdef FTS_CTL_IIC
     	fts_rw_iic_drv_exit();
#endif

#ifdef SYSFS_DEBUG
     	fts_remove_sysfs(client);
#endif

#ifdef FTS_APK_DEBUG
     	fts_release_apk_debug_channel();
#endif

#ifdef CONFIG_MTK_I2C_EXTENSION
	msg_dma_release();
#endif
	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

	return 0;
}

#if FT_ESD_PROTECT
void esd_switch(s32 on)
{
	return;
}
#endif


#if CTP_ESD_PROTECT
 /************************************************************************
 * Name: force_reset_guitar
 * Brief: reset
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
void force_reset_guitar(void)
{
 	/* Reset CTP */
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	tpd_gpio_as_int(tpd_int_gpio_number);
	msleep(10);
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
}

#define A3_REG_VALUE					0x87
#define RESET_91_REGVALUE_SAMECOUNT 			5
static u8 g_old_91_Reg_Value = 0x00;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count = 0;
 /************************************************************************
 * Name: ctp_esd_check_func
 * Brief: esd check function
 * Input: struct work_struct
 * Output: no
 * Return: 0
 ***********************************************************************/
void ctp_esd_check_func(void)
{
	int ret = -1;
	u8 data;
	int reset_flag = 0;
	if(apk_debug_flag)
		return;

	if(tp_ready == 0)
		return;

	ret = fts_read_reg(fts_i2c_client, 0x91,&data);
	if (ret < 0)
	{
		TPD_DMESG("[Focal][Touch] read value of 0x91 register fail  !!!");
		return;
	}

	TPD_DMESG("focal--91 register value = %d , old value = %d \n", data, g_old_91_Reg_Value);
	if(0x01 == g_first_read_91)
	{
		g_old_91_Reg_Value = data;
		g_first_read_91 = 0x00;
	}
	else
	{
		if(g_old_91_Reg_Value == data)
		{
			g_91value_same_count++;
			if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count)
			{
				force_reset_guitar();
				TPD_DMESG("focal--tpd reset. g_91value_same_count = 5\n");
				g_91value_same_count = 0;
				reset_flag = 1;
			}
			g_old_91_Reg_Value = data;
		}
		else
		{
			g_old_91_Reg_Value = data;
			g_91value_same_count = 0;
		}
	}
	return;
 }
#endif

/*set TP volt*/
static int tpd_local_init(void)
{
	int retval;

	pr_err("Focaltech ft8716 I2C Touchscreen Driver local init...\n");
	retval = tpd_get_regulators();
	if (retval < 0){
		TPD_DMESG("Failed to get tp&lcd fts_regulators\n");
		return -1;
	}

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}

	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);
#endif

	TPD_DMESG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	char gestrue_on = 0x01;
	char gestrue_data;
	int i;

	/* TPD_DEBUG("Entering doze mode..."); */
	TPD_DMESG("Entering doze mode...");

	/* Enter gestrue recognition mode */
	ret = fts_write_reg(fts_i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);
	if (ret < 0) {
		/* TPD_DEBUG("Failed to enter Doze %d", retry); */
		pr_alert("Failed to enter Doze %d", retry);
		return ret;
	}
	msleep(30);
	for (i = 0; i < 10; i++) {
		fts_read_reg(fts_i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
		if (gestrue_data == 0x01) {
			doze_status = DOZE_ENABLED;
			/* TPD_DEBUG("FTP has been working in doze mode!"); */
			pr_alert("FTP has been working in doze mode!");
			break;
		}
		msleep(20);
		fts_write_reg(fts_i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);

	}

	return ret;
}
#endif

static void tpd_resume(struct device *h)
{
	int retval = TPD_OK;
	int j;
	int retry = 0;

	if(tp_suspend){
		tpd_gpio_output(tpd_rst_gpio_number, 1);
		tpd_gpio_as_int(tpd_int_gpio_number);
		retval = tpd_enable_regulators();
		if (retval != 0)
			TPD_DMESG("Failed to enable fts_regulators: %d\n", retval);
		tp_suspend = 0;
		msleep(200);
	}

	if (finger_num > 0)    // release all touches in final
	{
		for(j = 0; j < finger_num; j++)
		{
			input_mt_slot( tpd->dev, j);
			input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
		}
		TPD_DEBUG("resume: clear touch point.  num %d \n", finger_num);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_sync(tpd->dev);
		finger_num = 0;
    	}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int ret;
	if (tpd_scp_doze_en) {
		ret = get_md32_semaphore(SEMAPHORE_TOUCH);
		if (ret < 0) {
			TPD_DEBUG("[TOUCH] HW semaphore reqiure timeout\n");
		} else {
			Touch_IPI_Packet ipi_pkt = {.cmd = IPI_COMMAND_AS_ENABLE_GESTURE, .param.data = 0};

			md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		}
	}
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	doze_status = DOZE_DISABLED;
#else
	while (retry++ < 5){
		retval = fts_write_reg(fts_i2c_client, FT_GESTRUE_MODE_SWITCH_REG, 0x00);  	//GESTRUE SWITCH == 0xD0
		if (retval < 0) {
			TPD_DMESG("TPD resume write 0x01 to 0 faild, retry %d \n", retval);
			msleep(50);
			continue;

		} else {
			TPD_DEBUG("tpd_resume: write 0xd0 0 success \n");
			break;
		}
	}
#endif
	tpd_charger_check(1);

#if CTP_ESD_PROTECT
	fts_esd_protection_resume();
	tp_ready = 1;
#endif
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
void tpd_scp_wakeup_enable(bool en)
{
	tpd_scp_doze_en = en;
}

void tpd_enter_doze(void)
{

}
#endif

static void tpd_suspend(struct device *h)
{
	int retval = 0;
	static char data = 0x3;
	int i = 0;
	u8 state = 0;

#if FT_ESD_PROTECT
	fts_esd_protection_suspend();
	tp_ready = 0;
#endif
       	if (fts_gesture_state != MZ_GESTURE_DISABLE){
		TPD_DMESG("TPD suspend try to gesture mode\n");
		fts_write_reg(fts_i2c_client, 0xd0, 0x01);
	  	fts_write_reg(fts_i2c_client, 0xd1, 0xff);
		fts_write_reg(fts_i2c_client, 0xd2, 0xff);
		fts_write_reg(fts_i2c_client, 0xd5, 0xff);
		fts_write_reg(fts_i2c_client, 0xd6, 0xff);
		fts_write_reg(fts_i2c_client, 0xd7, 0xff);
		fts_write_reg(fts_i2c_client, 0xd8, 0xff);
		msleep(10);
		for(i = 0; i < 5; i++)
		{
		  	fts_read_reg(fts_i2c_client, 0xd0, &state);
			if(state == 1)
			{
				TPD_DMESG("TPD gesture write 0x01 into gesture susessful\n");
        			return;
			}
			else
			{
				fts_write_reg(fts_i2c_client, 0xd0, 0x01);
				fts_write_reg(fts_i2c_client, 0xd1, 0xff);
	 			fts_write_reg(fts_i2c_client, 0xd2, 0xff);
			     	fts_write_reg(fts_i2c_client, 0xd5, 0xff);
				fts_write_reg(fts_i2c_client, 0xd6, 0xff);
				fts_write_reg(fts_i2c_client, 0xd7, 0xff);
			  	fts_write_reg(fts_i2c_client, 0xd8, 0xff);
				msleep(20);
			}
		}
		if(i >= 5)
		{
			TPD_DMESG("TPD suspend: gesture write 0x01 to d0 fail \n");
			return;
		}
	}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int sem_ret;
	tpd_enter_doze();
	int ret;
	char gestrue_data;
	char gestrue_cmd = 0x03;
	static int scp_init_flag;
	/* TPD_DEBUG("[tpd_scp_doze]:init=%d en=%d", scp_init_flag, tpd_scp_doze_en); */
	mutex_lock(&i2c_access);
	sem_ret = release_md32_semaphore(SEMAPHORE_TOUCH);
	if (scp_init_flag == 0) {
		Touch_IPI_Packet ipi_pkt;
		ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
		ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
		ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
		ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
		ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;
		TPD_DEBUG("[TOUCH]SEND CUST command :%d ", IPI_COMMAND_AS_CUST_PARAMETER);
		ret = md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		if (ret < 0)
			TPD_DMESG(" IPI cmd failed (%d)\n", ipi_pkt.cmd);

		msleep(20);
		/* delay added between continuous command */
		/* Workaround if suffer MD32 reset */
		/* scp_init_flag = 1; */
	}

	if (tpd_scp_doze_en) {
		TPD_DEBUG("[TOUCH]SEND ENABLE GES command :%d ", IPI_COMMAND_AS_ENABLE_GESTURE);
		ret = ftp_enter_doze(fts_i2c_client);
		if (ret < 0) {
			TPD_DMESG("FTP Enter Doze mode failed\n");
	} else {
		int retry = 5;
	    	fts_read_reg(fts_i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
		TPD_DEBUG("=======>0x%x", gestrue_data);
	   	msleep(20);
		Touch_IPI_Packet ipi_pkt = {.cmd = IPI_COMMAND_AS_ENABLE_GESTURE, .param.data = 1};

		do {
			if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 1) == DONE)
				break;
			msleep(20);
			TPD_DEBUG("==>retry=%d", retry);
		} while (retry--);

	    	if (retry <= 0)
			TPD_DMESG("############# md32_ipi_send failed retry=%d", retry);

		}
		/* disable_irq(touch_irq); */
	}

	mutex_unlock(&i2c_access);
#else
	TPD_DMESG("TP suspend disable irq and enter sleep mode \n");
	fts_write_reg(fts_i2c_client, 0xA5, data);  		/* TP enter sleep mode */
	tpd_gpio_output(tpd_rst_gpio_number, 0);		//reset pin output low
	if(tp_suspend == 0){
		retval = tpd_disable_regulators();
		if (retval != 0)
			TPD_DMESG("Failed to disable fts_regulators: %d\n", retval);
		tp_suspend = 1;
	}
#endif

}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "cap_touch",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft8716_attrs,
		.num  = ARRAY_SIZE(ft8716_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	TPD_DMESG("focaltech ft8716 touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add ft8716 driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("focaltech ft8716 touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

