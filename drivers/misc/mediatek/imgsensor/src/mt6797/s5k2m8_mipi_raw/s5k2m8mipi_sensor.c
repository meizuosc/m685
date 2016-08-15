/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5K2M8mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6797
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *	

 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k2m8mipi_sensor.h"

/*===FEATURE SWITH===*/
 // #define FPTPDAFSUPPORT   //for pdaf switch
 // #define FANPENGTAO   //for debug log
 #define LOG_INF LOG_INF_LOD
 //#define NONCONTINUEMODE
/*===FEATURE SWITH===*/

/****************************Modify Following Strings for Debug****************************/
#define PFX "s5k2m8"
#define LOG_INF_LOD(format, args...)    pr_info(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_1 LOG_INF("S5K2M8,MIPI 4LANE\n")
#define SENSORDB LOG_INF
/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK(imgsensor_drv_lock);

u8 *primax_otp_buf;
#define	MAX_READ_WRITE_SIZE	8
#define	OTP_DATA_SIZE	3400
#define	OTP_START_ADDR	0x0000
#define	E2PROM_WRITE_ID	0xA0

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K2M8_SENSOR_ID,		//Sensor ID Value: 0x30C8//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value =  0xe1b26f6c,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4864,				//record different mode's linelength
		.framelength = 3280,			//record different mode's framelength
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},

	.cap = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4864,//5808,				//record different mode's linelength
		.framelength = 3280,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4208,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},

#if 1 //fps =24
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 400000000,				//record different mode's pclk
		.linelength  = 4704,				//record different mode's linelength
		.framelength = 3536,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4208,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 240,	
	},
#endif
#if 0 //fps 15
	.cap1 = {							//capture for PIP 15ps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 240000000,				//record different mode's pclk
		.linelength  = 4704,				//record different mode's linelength
		.framelength = 3392,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4208,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 150,	
	},
#endif
	.normal_video = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4864,				//record different mode's linelength
		.framelength = 3280,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4208,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	}, 
#if 0
	.hs_video = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4560,				//record different mode's linelength
		.framelength = 876,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 640,		//record different mode's width of grabwindow
		.grabwindow_height = 480,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,	
	},
#endif
	.hs_video = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4560,				//record different mode's linelength
		.framelength = 876,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,	
	},
	.slim_video = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength  = 4560,				//record different mode's linelength
		.framelength = 3504,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom1 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom2 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom3 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom4 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom5 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},

	.margin = 5,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0xFFFF,//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2, 
    .custom3_delay_frame = 2, 
    .custom4_delay_frame = 2, 
    .custom5_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x5a, 0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
    .i2c_speed = 300, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0,//record current sensor's i2c write id
};


/* Sensor output window information*/
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =	 
{
 { 4208, 3120,	  0,  	0, 4208, 3120, 2104, 1560,   0,	0, 2104, 1560, 	 0, 0, 2104, 1560}, // Preview 
 { 4208, 3120,	  0,  	0, 4208, 3120, 4208, 3120,   0,	0, 4208, 3120, 	 0, 0, 4208, 3120}, // capture 
 { 4208, 3120,	  0,  	0, 4208, 3120, 4208, 3120,   0,	0, 4208, 3120, 	 0, 0, 4208, 3120}, // video 
 //{ 4208, 3120,	184,  120, 3840, 2880,  640,  480,   0,	0,  640,  480, 	 0, 0,  640,  480}, //hight speed video
 { 4208, 3120,	184,  480, 3840, 2160, 1280,  720,   0,	0, 1280,  720, 	 0, 0, 1280,  720}, //hight speed video
 { 4208, 3120,	184,  480, 3840, 2160, 1280,  720,   0,	0, 1280,  720, 	 0, 0, 1280,  720},// slim video 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom1 (defaultuse preview) 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom2 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom3 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom4 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom5 
};

 static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
 //for 2M8 non mirror flip
{
 
    .i4OffsetX = 28,
 
    .i4OffsetY = 31,
 
    .i4PitchX = 64,
 
    .i4PitchY = 64,
 
    .i4PairNum =16,
 
    .i4SubBlkW =16,
 
    .i4SubBlkH =16, 
 
    .i4PosL = {{28,31},{80,31},{44,35},{64,35},{32,51},{76,51},{48,55},{60,55},{48,63},{60,63},{32,67},{76,67},{44,83},{64,83},{28,87},{80,87}},
 
    .i4PosR = {{28,35},{80,35},{44,39},{64,39},{32,47},{76,47},{48,51},{60,51},{48,67},{60,67},{32,71},{76,71},{44,79},{64,79},{28,83},{80,83}},
 
};


// //for 2M8 mirror flip
//{
// 
//    .i4OffsetX = 31,
// 
//    .i4OffsetY = 24,
// 
//    .i4PitchX = 64,
// 
//    .i4PitchY = 64,
// 
//    .i4PairNum =16,
// 
//    .i4SubBlkW =16,
// 
//    .i4SubBlkH =16, 
// 
//    .i4PosL = {{31,28},{83,28},{47,32},{67,32},{35,40},{79,40},{51,44},{63,44},{51,60},{63,60},{35,64},{79,64},{47,72},{67,72},{31,76},{83,76}},
// 
//    .i4PosR = {{31,24},{83,24},{47,28},{67,28},{35,44},{79,44},{51,48},{63,48},{51,56},{63,56},{35,60},{79,60},{47,76},{67,76},{31,80},{83,80}},
// 
//};

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static int s5k2m8_read_otp(u16 addr, u8 *buf)
{
	int ret = 0;
	u8 pu_send_cmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};

	ret = iReadRegI2C(pu_send_cmd, 2, (u8*)buf, 1, E2PROM_WRITE_ID);
	if (ret < 0)
		LOG_INF("read data from s5k2m8 primax otp e2prom failed!\n");

	return ret;
}

static void s5k2m8_read_otp_burst(u16 addr, u8 *otp_buf)
{
	int i;
	int ret;
	u8 pu_send_cmd[2];

	for (i = 0; i < OTP_DATA_SIZE; i += MAX_READ_WRITE_SIZE) {
		pu_send_cmd[0] = (u8)(addr >> 8);
		pu_send_cmd[1] = (u8)(addr & 0xFF);

		if (i + MAX_READ_WRITE_SIZE > OTP_DATA_SIZE)
			ret = iReadRegI2C(pu_send_cmd, 2, (u8 *)(otp_buf + i), (OTP_DATA_SIZE - i), E2PROM_WRITE_ID);
		else
			ret = iReadRegI2C(pu_send_cmd, 2, (u8 *)(otp_buf + i), MAX_READ_WRITE_SIZE, E2PROM_WRITE_ID);

		if (ret < 0)
			LOG_INF("read lsc table from s5k2m8 primax otp e2prom failed!\n");

		addr += MAX_READ_WRITE_SIZE;
	}
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable(%d) \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, (shutter) & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	//write_shutter(shutter);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0X0202, shutter & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	//gain = 64 = 1x real gain.
    reg_gain = gain/2;
	//reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	LOG_INF("set_gain %d \n", gain);
  //gain = 64 = 1x real gain.
	kal_uint16 reg_gain;
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;		 
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));    
	return gain;
}	/*	set_gain  */

//ihdr_write_shutter_gain not support for s5k2M8
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
		// Extend frame length first
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3512, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3511, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3510, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101,0X00); //GR
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101,0X01); //R
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101,0X02); //B	
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101,0X03); //GB
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
static void sensor_init(void)
{
  LOG_INF("%s. start\n", __func__);

  write_cmos_sensor(0x6028,0x4000);
  write_cmos_sensor(0x6010,0x0001);
	mdelay(3);	
  write_cmos_sensor(0x6214,0x7970);
  write_cmos_sensor(0x6218,0x7150);
  write_cmos_sensor(0x6028,0x2000);
  write_cmos_sensor(0x602A,0x4A34);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x0449);
  write_cmos_sensor(0x6F12,0x0348);
  write_cmos_sensor(0x6F12,0x044A);
  write_cmos_sensor(0x6F12,0x4860);
  write_cmos_sensor(0x6F12,0x101A);
  write_cmos_sensor(0x6F12,0x0881);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x89B8);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x4C48);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x2AB0);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x9400);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x2DE9);
  write_cmos_sensor(0x6F12,0xF041);
  write_cmos_sensor(0x6F12,0x0746);
  write_cmos_sensor(0x6F12,0x4F48);
  write_cmos_sensor(0x6F12,0x0022);
  write_cmos_sensor(0x6F12,0x0068);
  write_cmos_sensor(0x6F12,0x85B2);
  write_cmos_sensor(0x6F12,0x040C);
  write_cmos_sensor(0x6F12,0x2946);
  write_cmos_sensor(0x6F12,0x2046);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0xAAF8);
  write_cmos_sensor(0x6F12,0x4B48);
  write_cmos_sensor(0x6F12,0x0023);
  write_cmos_sensor(0x6F12,0x1A46);
  write_cmos_sensor(0x6F12,0xB0F8);
  write_cmos_sensor(0x6F12,0xB016);
  write_cmos_sensor(0x6F12,0x00F2);
  write_cmos_sensor(0x6F12,0xFC60);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0xA6F8);
  write_cmos_sensor(0x6F12,0x4748);
  write_cmos_sensor(0x6F12,0x90F8);
  write_cmos_sensor(0x6F12,0xF006);
  write_cmos_sensor(0x6F12,0x38B9);
  write_cmos_sensor(0x6F12,0x4548);
  write_cmos_sensor(0x6F12,0x90F8);
  write_cmos_sensor(0x6F12,0xFD06);
  write_cmos_sensor(0x6F12,0x18B1);
  write_cmos_sensor(0x6F12,0x0020);
  write_cmos_sensor(0x6F12,0x18B1);
  write_cmos_sensor(0x6F12,0x0B21);
  write_cmos_sensor(0x6F12,0x02E0);
  write_cmos_sensor(0x6F12,0x0120);
  write_cmos_sensor(0x6F12,0xFAE7);
  write_cmos_sensor(0x6F12,0x0921);
  write_cmos_sensor(0x6F12,0x3F4E);
  write_cmos_sensor(0x6F12,0x96F8);
  write_cmos_sensor(0x6F12,0x1602);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x97F8);
  write_cmos_sensor(0x6F12,0x3846);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x99F8);
  write_cmos_sensor(0x6F12,0x3C48);
  write_cmos_sensor(0x6F12,0xB0F8);
  write_cmos_sensor(0x6F12,0xB428);
  write_cmos_sensor(0x6F12,0x96F8);
  write_cmos_sensor(0x6F12,0x1F02);
  write_cmos_sensor(0x6F12,0x0228);
  write_cmos_sensor(0x6F12,0x01D1);
  write_cmos_sensor(0x6F12,0x42F4);
  write_cmos_sensor(0x6F12,0x8072);
  write_cmos_sensor(0x6F12,0x3848);
  write_cmos_sensor(0x6F12,0x3849);
  write_cmos_sensor(0x6F12,0xB0F8);
  write_cmos_sensor(0x6F12,0xB208);
  write_cmos_sensor(0x6F12,0x0880);
  write_cmos_sensor(0x6F12,0x881C);
  write_cmos_sensor(0x6F12,0x0280);
  write_cmos_sensor(0x6F12,0x3348);
  write_cmos_sensor(0x6F12,0x90F8);
  write_cmos_sensor(0x6F12,0x0006);
  write_cmos_sensor(0x6F12,0x08B1);
  write_cmos_sensor(0x6F12,0x2622);
  write_cmos_sensor(0x6F12,0x00E0);
  write_cmos_sensor(0x6F12,0x1922);
  write_cmos_sensor(0x6F12,0x3248);
  write_cmos_sensor(0x6F12,0x8C30);
  write_cmos_sensor(0x6F12,0x0280);
  write_cmos_sensor(0x6F12,0x2946);
  write_cmos_sensor(0x6F12,0x2046);
  write_cmos_sensor(0x6F12,0xBDE8);
  write_cmos_sensor(0x6F12,0xF041);
  write_cmos_sensor(0x6F12,0x0122);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x69B8);
  write_cmos_sensor(0x6F12,0x2E48);
  write_cmos_sensor(0x6F12,0x90F8);
  write_cmos_sensor(0x6F12,0x3B10);
  write_cmos_sensor(0x6F12,0x0229);
  write_cmos_sensor(0x6F12,0x02D1);
  write_cmos_sensor(0x6F12,0x017B);
  write_cmos_sensor(0x6F12,0x0229);
  write_cmos_sensor(0x6F12,0xFCD3);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x73B8);
  write_cmos_sensor(0x6F12,0x10B5);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x75F8);
  write_cmos_sensor(0x6F12,0x2849);
  write_cmos_sensor(0x6F12,0x0020);
  write_cmos_sensor(0x6F12,0x0880);
  write_cmos_sensor(0x6F12,0x10BD);
  write_cmos_sensor(0x6F12,0x0028);
  write_cmos_sensor(0x6F12,0x07D0);
  write_cmos_sensor(0x6F12,0x264A);
  write_cmos_sensor(0x6F12,0x92F8);
  write_cmos_sensor(0x6F12,0x832B);
  write_cmos_sensor(0x6F12,0x5300);
  write_cmos_sensor(0x6F12,0x0222);
  write_cmos_sensor(0x6F12,0x9A40);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x6CB8);
  write_cmos_sensor(0x6F12,0x7047);
  write_cmos_sensor(0x6F12,0x10B5);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x6DF8);
  write_cmos_sensor(0x6F12,0x1A48);
  write_cmos_sensor(0x6F12,0x1B49);
  write_cmos_sensor(0x6F12,0x1230);
  write_cmos_sensor(0x6F12,0xC1F8);
  write_cmos_sensor(0x6F12,0xD404);
  write_cmos_sensor(0x6F12,0x801F);
  write_cmos_sensor(0x6F12,0xC1F8);
  write_cmos_sensor(0x6F12,0xEC04);
  write_cmos_sensor(0x6F12,0xA0F1);
  write_cmos_sensor(0x6F12,0x0804);
  write_cmos_sensor(0x6F12,0x1B49);
  write_cmos_sensor(0x6F12,0x2088);
  write_cmos_sensor(0x6F12,0x891C);
  write_cmos_sensor(0x6F12,0xFFF7);
  write_cmos_sensor(0x6F12,0xE3FF);
  write_cmos_sensor(0x6F12,0x6088);
  write_cmos_sensor(0x6F12,0x1849);
  write_cmos_sensor(0x6F12,0xBDE8);
  write_cmos_sensor(0x6F12,0x1040);
  write_cmos_sensor(0x6F12,0x8231);
  write_cmos_sensor(0x6F12,0xDCE7);
  write_cmos_sensor(0x6F12,0x10B5);
  write_cmos_sensor(0x6F12,0x0022);
  write_cmos_sensor(0x6F12,0xAFF2);
  write_cmos_sensor(0x6F12,0x0711);
  write_cmos_sensor(0x6F12,0x1548);
  write_cmos_sensor(0x6F12,0x00F0);
  write_cmos_sensor(0x6F12,0x56F8);
  write_cmos_sensor(0x6F12,0x0C49);
  write_cmos_sensor(0x6F12,0x0860);
  write_cmos_sensor(0x6F12,0xAFF2);
  write_cmos_sensor(0x6F12,0x7D01);
  write_cmos_sensor(0x6F12,0x1248);
  write_cmos_sensor(0x6F12,0x8161);
  write_cmos_sensor(0x6F12,0xAFF2);
  write_cmos_sensor(0x6F12,0x7101);
  write_cmos_sensor(0x6F12,0x4161);
  write_cmos_sensor(0x6F12,0xAFF2);
  write_cmos_sensor(0x6F12,0x5101);
  write_cmos_sensor(0x6F12,0x0160);
  write_cmos_sensor(0x6F12,0x0648);
  write_cmos_sensor(0x6F12,0x4021);
  write_cmos_sensor(0x6F12,0x001D);
  write_cmos_sensor(0x6F12,0x0172);
  write_cmos_sensor(0x6F12,0x8172);
  write_cmos_sensor(0x6F12,0x1021);
  write_cmos_sensor(0x6F12,0x8173);
  write_cmos_sensor(0x6F12,0x0174);
  write_cmos_sensor(0x6F12,0x0B49);
  write_cmos_sensor(0x6F12,0x4DF6);
  write_cmos_sensor(0x6F12,0x6E70);
  write_cmos_sensor(0x6F12,0x0968);
  write_cmos_sensor(0x6F12,0x4883);
  write_cmos_sensor(0x6F12,0x10BD);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x4C30);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x2B40);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x06A0);
  write_cmos_sensor(0x6F12,0x4000);
  write_cmos_sensor(0x6F12,0xF438);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x2AB0);
  write_cmos_sensor(0x6F12,0x4000);
  write_cmos_sensor(0x6F12,0x8800);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x0F60);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6F12,0x5689);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x0580);
  write_cmos_sensor(0x6F12,0x2000);
  write_cmos_sensor(0x6F12,0x0310);
  write_cmos_sensor(0x6F12,0x40F6);
  write_cmos_sensor(0x6F12,0x3D2C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x44F6);
  write_cmos_sensor(0x6F12,0xF57C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x46F6);
  write_cmos_sensor(0x6F12,0x270C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x45F2);
  write_cmos_sensor(0x6F12,0x896C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x48F2);
  write_cmos_sensor(0x6F12,0xF51C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x48F2);
  write_cmos_sensor(0x6F12,0xDD1C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x47F6);
  write_cmos_sensor(0x6F12,0xCD0C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x43F6);
  write_cmos_sensor(0x6F12,0x4F2C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x40F2);
  write_cmos_sensor(0x6F12,0xE11C);
  write_cmos_sensor(0x6F12,0xC0F2);
  write_cmos_sensor(0x6F12,0x000C);
  write_cmos_sensor(0x6F12,0x6047);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x602A,0x1AE2);
  write_cmos_sensor(0x6F12,0x0A02);
  write_cmos_sensor(0x602A,0x4C34);
  write_cmos_sensor(0x6F12,0x03B8);
  write_cmos_sensor(0x6F12,0x03D8);
  write_cmos_sensor(0x6028,0x4000);
  write_cmos_sensor(0xF4A0,0x004F);
  write_cmos_sensor(0x3A92,0x04C6);
  write_cmos_sensor(0x3A58,0x010B);
  write_cmos_sensor(0x3A5C,0x0B0B);
  write_cmos_sensor(0x3A60,0x0707);
  write_cmos_sensor(0x3A62,0x0709);
  write_cmos_sensor(0xF482,0x001A);
  write_cmos_sensor(0xF47A,0x0010);
  write_cmos_sensor(0x3A82,0x0010);
  write_cmos_sensor(0x3192,0x0000);
  write_cmos_sensor(0x316A,0x00B4);
  write_cmos_sensor(0x316C,0x10F3);
  write_cmos_sensor(0x3206,0x000B);  
  write_cmos_sensor(0x320C,0x000B);  
  write_cmos_sensor(0x3212,0x000C);  
  write_cmos_sensor(0x3218,0x000C);
  write_cmos_sensor(0x321E,0x00C5);
  write_cmos_sensor(0x3224,0x00F9);
  write_cmos_sensor(0x322A,0x00C1);
  write_cmos_sensor(0x3230,0x00FD);
  write_cmos_sensor(0x3236,0x0086);
  write_cmos_sensor(0x323C,0x0058);
  write_cmos_sensor(0x3242,0x004F);
  write_cmos_sensor(0x3248,0x000F);
  write_cmos_sensor(0x324E,0x007E);
  write_cmos_sensor(0x3254,0x0060);
  write_cmos_sensor(0x325A,0x0047);
  write_cmos_sensor(0x3260,0x0017);
  write_cmos_sensor(0x3266,0x004B);
  write_cmos_sensor(0x326C,0x0013);
  write_cmos_sensor(0x3272,0x0014);
  write_cmos_sensor(0x3278,0x00C0);
  write_cmos_sensor(0x327E,0x022D);
  write_cmos_sensor(0x3284,0x00C5);
  write_cmos_sensor(0x328A,0x022B);
  write_cmos_sensor(0x3290,0x00C0);
  write_cmos_sensor(0x3296,0x0105);
  write_cmos_sensor(0x329C,0x000C);
  write_cmos_sensor(0x32A2,0x004A);
  write_cmos_sensor(0x32A8,0x000C);
  write_cmos_sensor(0x32AE,0x0038);
  write_cmos_sensor(0x32B4,0x000C);
  write_cmos_sensor(0x32BA,0x0044);
  write_cmos_sensor(0x32C0,0x000C);
  write_cmos_sensor(0x32C6,0x00C0);
  write_cmos_sensor(0x32CC,0x0018);
  write_cmos_sensor(0x32D2,0x001B);
  write_cmos_sensor(0x32D8,0x00F9);
  write_cmos_sensor(0x32DE,0x0108);
  write_cmos_sensor(0x32E4,0x0018);
  write_cmos_sensor(0x32EA,0x001B);
  write_cmos_sensor(0x32F0,0x00F9);
  write_cmos_sensor(0x32F6,0x00FC);
  write_cmos_sensor(0x32FC,0x000C);
  write_cmos_sensor(0x3302,0x000C);
  write_cmos_sensor(0x3308,0x000B);
  write_cmos_sensor(0x330E,0x0014);
  write_cmos_sensor(0x3314,0x004E);
  write_cmos_sensor(0x331A,0x0229);
  write_cmos_sensor(0x3320,0x004D);
  write_cmos_sensor(0x3326,0x0005);
  write_cmos_sensor(0x332C,0x003B);
  write_cmos_sensor(0x3332,0x0005);
  write_cmos_sensor(0x3338,0x004D);
  write_cmos_sensor(0x333E,0x0005);
  write_cmos_sensor(0x3344,0x003B);
  write_cmos_sensor(0x334A,0x0005);
  write_cmos_sensor(0x3350,0x000A);
  write_cmos_sensor(0x3356,0x0005);
  write_cmos_sensor(0x335C,0x001F);
  write_cmos_sensor(0x3362,0x0005);
  write_cmos_sensor(0x3368,0x008C);
  write_cmos_sensor(0x336E,0x00C0);
  write_cmos_sensor(0x3374,0x0165);
  write_cmos_sensor(0x337A,0x0229);
  write_cmos_sensor(0x3380,0x00C8);
  write_cmos_sensor(0x3386,0x00DF);
  write_cmos_sensor(0x338C,0x00CF);
  write_cmos_sensor(0x3392,0x00E7);
  write_cmos_sensor(0x3398,0x00D7);
  write_cmos_sensor(0x339E,0x00E7);
  write_cmos_sensor(0x33A4,0x00C8);
  write_cmos_sensor(0x33AA,0x00CA);
  write_cmos_sensor(0x33B0,0x0001);
  write_cmos_sensor(0x33B6,0x0005);
  write_cmos_sensor(0x33BC,0x00CF);
  write_cmos_sensor(0x33C2,0x00E7);
  write_cmos_sensor(0x33C8,0x0001);
  write_cmos_sensor(0x33CE,0x0005);
  write_cmos_sensor(0x33D4,0x00C8);
  write_cmos_sensor(0x33DA,0x00CA);
  write_cmos_sensor(0x33E0,0x008A);
  write_cmos_sensor(0x33E6,0x00C2);
  write_cmos_sensor(0x33EC,0x0163);
  write_cmos_sensor(0x33F2,0x022B);
  write_cmos_sensor(0x33F8,0x0001);
  write_cmos_sensor(0x33FE,0x0005);
  write_cmos_sensor(0x3404,0x000A);
  write_cmos_sensor(0x340A,0x003B);
  write_cmos_sensor(0x3410,0x003D);
  write_cmos_sensor(0x3416,0x006E);
  write_cmos_sensor(0x341C,0x0070);
  write_cmos_sensor(0x3422,0x00A1);
  write_cmos_sensor(0x3428,0x00A3);
  write_cmos_sensor(0x342E,0x00D4);
  write_cmos_sensor(0x3434,0x00D6);
  write_cmos_sensor(0x343A,0x0107);
  write_cmos_sensor(0x3440,0x0109);
  write_cmos_sensor(0x3446,0x013A);
  write_cmos_sensor(0x344C,0x013C);
  write_cmos_sensor(0x3452,0x016D);
  write_cmos_sensor(0x3458,0x016F);
  write_cmos_sensor(0x345E,0x01A0);
  write_cmos_sensor(0x3464,0x01A2);
  write_cmos_sensor(0x346A,0x01D3);
  write_cmos_sensor(0x3470,0x01D5);
  write_cmos_sensor(0x3476,0x0206);
  write_cmos_sensor(0x347C,0x0208);
  write_cmos_sensor(0x3482,0x0239);
  write_cmos_sensor(0x3488,0x000B);
  write_cmos_sensor(0x348E,0x003C);
  write_cmos_sensor(0x3494,0x003E);
  write_cmos_sensor(0x349A,0x006F);
  write_cmos_sensor(0x34A0,0x0071);
  write_cmos_sensor(0x34A6,0x00A2);
  write_cmos_sensor(0x34AC,0x00A4);
  write_cmos_sensor(0x34B2,0x00D5);
  write_cmos_sensor(0x34B8,0x00D7);
  write_cmos_sensor(0x34BE,0x0108);
  write_cmos_sensor(0x34C4,0x010A);
  write_cmos_sensor(0x34CA,0x013B);
  write_cmos_sensor(0x34D0,0x013D);
  write_cmos_sensor(0x34D6,0x016E);
  write_cmos_sensor(0x34DC,0x0170);
  write_cmos_sensor(0x34E2,0x01A1);
  write_cmos_sensor(0x34E8,0x01A3);
  write_cmos_sensor(0x34EE,0x01D4);
  write_cmos_sensor(0x34F4,0x01D6);
  write_cmos_sensor(0x34FA,0x0000);
  write_cmos_sensor(0x3500,0x0000);
  write_cmos_sensor(0x3506,0x0207);
  write_cmos_sensor(0x350C,0x0209);
  write_cmos_sensor(0x3512,0x023A);
  write_cmos_sensor(0x3518,0x000C);
  write_cmos_sensor(0x351E,0x003D);
  write_cmos_sensor(0x3524,0x003F);
  write_cmos_sensor(0x352A,0x0070);
  write_cmos_sensor(0x3530,0x0072);
  write_cmos_sensor(0x3536,0x00A3);
  write_cmos_sensor(0x353C,0x00A5);
  write_cmos_sensor(0x3542,0x00D6);
  write_cmos_sensor(0x3548,0x00D8);
  write_cmos_sensor(0x354E,0x0109);
  write_cmos_sensor(0x3554,0x010B);
  write_cmos_sensor(0x355A,0x013C);
  write_cmos_sensor(0x3560,0x013E);
  write_cmos_sensor(0x3566,0x016F);
  write_cmos_sensor(0x356C,0x0171);
  write_cmos_sensor(0x3572,0x01A2);
  write_cmos_sensor(0x3578,0x01A4);
  write_cmos_sensor(0x357E,0x01D5);
  write_cmos_sensor(0x3584,0x01D7);
  write_cmos_sensor(0x358A,0x0208);
  write_cmos_sensor(0x3590,0x020A);
  write_cmos_sensor(0x3596,0x023B);
  write_cmos_sensor(0x359C,0x000D);
  write_cmos_sensor(0x35A2,0x003E);
  write_cmos_sensor(0x35A8,0x0040);
  write_cmos_sensor(0x35AE,0x0071);
  write_cmos_sensor(0x35B4,0x0073);
  write_cmos_sensor(0x35BA,0x00A4);
  write_cmos_sensor(0x35C0,0x00A6);
  write_cmos_sensor(0x35C6,0x00D7);
  write_cmos_sensor(0x35CC,0x00D9);
  write_cmos_sensor(0x35D2,0x010A);
  write_cmos_sensor(0x35D8,0x010C);
  write_cmos_sensor(0x35DE,0x013D);
  write_cmos_sensor(0x35E4,0x013F);
  write_cmos_sensor(0x35EA,0x0170);
  write_cmos_sensor(0x35F0,0x0172);
  write_cmos_sensor(0x35F6,0x01A3);
  write_cmos_sensor(0x35FC,0x01A5);
  write_cmos_sensor(0x3602,0x01D6);
  write_cmos_sensor(0x3608,0x01D8);
  write_cmos_sensor(0x360E,0x0209);
  write_cmos_sensor(0x3614,0x020B);
  write_cmos_sensor(0x361A,0x023C);
  write_cmos_sensor(0x3620,0x0009);
  write_cmos_sensor(0x3626,0x003C);
  write_cmos_sensor(0x362C,0x006F);
  write_cmos_sensor(0x3632,0x00A2);
  write_cmos_sensor(0x3638,0x00D5);
  write_cmos_sensor(0x363E,0x0108);
  write_cmos_sensor(0x3644,0x013B);
  write_cmos_sensor(0x364A,0x016E);
  write_cmos_sensor(0x3650,0x01A1);
  write_cmos_sensor(0x3656,0x01D4);
  write_cmos_sensor(0x365C,0x0207);
  write_cmos_sensor(0x3662,0x0009);
  write_cmos_sensor(0x3668,0x000C);
  write_cmos_sensor(0x366E,0x00C8);
  write_cmos_sensor(0x3674,0x00E2);
  write_cmos_sensor(0x367A,0x0009);
  write_cmos_sensor(0x3680,0x000C);
  write_cmos_sensor(0x3686,0x00C8);
  write_cmos_sensor(0x368C,0x00CA);
  write_cmos_sensor(0x3692,0x0000);
  write_cmos_sensor(0x3698,0x0000);
  write_cmos_sensor(0x369E,0x0056);
  write_cmos_sensor(0x36A4,0x022F);
  write_cmos_sensor(0x36AA,0x0057);
  write_cmos_sensor(0x36B0,0x0059);
  write_cmos_sensor(0x36B6,0x00C2);
  write_cmos_sensor(0x36BC,0x00C5);
  write_cmos_sensor(0x36C2,0x022B);
  write_cmos_sensor(0x36C8,0x022E);
  write_cmos_sensor(0x36CE,0x00C2);
  write_cmos_sensor(0x386C,0x00F9); 
  write_cmos_sensor(0x3872,0x0148); 
  write_cmos_sensor(0x3878,0x0262); 
  write_cmos_sensor(0x387E,0x0000); 
  write_cmos_sensor(0x3884,0x0073); 
  write_cmos_sensor(0x388A,0x024C); 
  write_cmos_sensor(0x3890,0x001F); 
  write_cmos_sensor(0x3896,0x0130); 
  write_cmos_sensor(0x389C,0x0148); 
  write_cmos_sensor(0x38A2,0x0299); 
  write_cmos_sensor(0x38A8,0x0000); 
  write_cmos_sensor(0x38AE,0x0073); 
  write_cmos_sensor(0x38B4,0x024C); 
  write_cmos_sensor(0x38BA,0x001F); 
  write_cmos_sensor(0x38C0,0x00F9); 
  write_cmos_sensor(0x38C6,0x0150); 
  write_cmos_sensor(0x38CC,0x0262); 
  write_cmos_sensor(0x38D2,0x0000); 
  write_cmos_sensor(0x38D8,0x0073); 
  write_cmos_sensor(0x38DE,0x024C); 
  write_cmos_sensor(0x38E4,0x001F); 
  write_cmos_sensor(0x38EA,0x0130); 
  write_cmos_sensor(0x38F0,0x012E);
  write_cmos_sensor(0x38F6,0x024A); 
  write_cmos_sensor(0x38FC,0x0000); 
  write_cmos_sensor(0x3902,0x0073); 
  write_cmos_sensor(0x3908,0x024C); 
  write_cmos_sensor(0x390E,0x0001); 
  write_cmos_sensor(0x3A5A,0x0B0B);
  write_cmos_sensor(0x3A64,0x0904);
  write_cmos_sensor(0x3208,0x000B);
  write_cmos_sensor(0x320E,0x000B);
  write_cmos_sensor(0x3214,0x000C);
  write_cmos_sensor(0x321A,0x000C);
  write_cmos_sensor(0x3220,0x00C5);
  write_cmos_sensor(0x3226,0x00F5);
  write_cmos_sensor(0x322C,0x00C1);
  write_cmos_sensor(0x3232,0x00F9);
  write_cmos_sensor(0x3238,0x0086);
  write_cmos_sensor(0x323E,0x0058);
  write_cmos_sensor(0x3244,0x004F);
  write_cmos_sensor(0x324A,0x000F);
  write_cmos_sensor(0x3250,0x007E);
  write_cmos_sensor(0x3256,0x0060);
  write_cmos_sensor(0x325C,0x0047);
  write_cmos_sensor(0x3262,0x0017);
  write_cmos_sensor(0x3268,0x004B);
  write_cmos_sensor(0x326E,0x0013);
  write_cmos_sensor(0x3274,0x0014);
  write_cmos_sensor(0x327A,0x00BC);
  write_cmos_sensor(0x3280,0x0219);
  write_cmos_sensor(0x3286,0x00C5);
  write_cmos_sensor(0x328C,0x0217);
  write_cmos_sensor(0x3292,0x00BC);
  write_cmos_sensor(0x3298,0x0101);
  write_cmos_sensor(0x329E,0x000C);
  write_cmos_sensor(0x32A4,0x004A);
  write_cmos_sensor(0x32AA,0x000C);
  write_cmos_sensor(0x32B0,0x0034);
  write_cmos_sensor(0x32B6,0x000C);
  write_cmos_sensor(0x32BC,0x0040);
  write_cmos_sensor(0x32C2,0x000C);
  write_cmos_sensor(0x32C8,0x00BC);
  write_cmos_sensor(0x32CE,0x0018);
  write_cmos_sensor(0x32D4,0x001B);
  write_cmos_sensor(0x32DA,0x00F5);
  write_cmos_sensor(0x32E0,0x0104);
  write_cmos_sensor(0x32E6,0x0018);
  write_cmos_sensor(0x32EC,0x001B);
  write_cmos_sensor(0x32F2,0x00F5);
  write_cmos_sensor(0x32F8,0x00F8);
  write_cmos_sensor(0x32FE,0x000C);
  write_cmos_sensor(0x3304,0x000C);
  write_cmos_sensor(0x330A,0x000B);
  write_cmos_sensor(0x3310,0x0014);
  write_cmos_sensor(0x3316,0x004E);
  write_cmos_sensor(0x331C,0x0215);
  write_cmos_sensor(0x3322,0x004D);
  write_cmos_sensor(0x3328,0x0005);
  write_cmos_sensor(0x332E,0x0037);
  write_cmos_sensor(0x3334,0x0005);
  write_cmos_sensor(0x333A,0x004D);
  write_cmos_sensor(0x3340,0x0005);
  write_cmos_sensor(0x3346,0x0037);
  write_cmos_sensor(0x334C,0x0005);
  write_cmos_sensor(0x3352,0x000A);
  write_cmos_sensor(0x3358,0x0005);
  write_cmos_sensor(0x335E,0x001F);
  write_cmos_sensor(0x3364,0x0005);
  write_cmos_sensor(0x336A,0x008C);
  write_cmos_sensor(0x3370,0x00BC);
  write_cmos_sensor(0x3376,0x0155);
  write_cmos_sensor(0x337C,0x0215);
  write_cmos_sensor(0x3382,0x00C4);
  write_cmos_sensor(0x3388,0x00DB);
  write_cmos_sensor(0x338E,0x00CB);
  write_cmos_sensor(0x3394,0x00E3);
  write_cmos_sensor(0x339A,0x00D3);
  write_cmos_sensor(0x33A0,0x00E3);
  write_cmos_sensor(0x33A6,0x00C4);
  write_cmos_sensor(0x33AC,0x00C6);
  write_cmos_sensor(0x33B2,0x0001);
  write_cmos_sensor(0x33B8,0x0005);
  write_cmos_sensor(0x33BE,0x00CB);
  write_cmos_sensor(0x33C4,0x00E3);
  write_cmos_sensor(0x33CA,0x0001);
  write_cmos_sensor(0x33D0,0x0005);
  write_cmos_sensor(0x33D6,0x00C4);
  write_cmos_sensor(0x33DC,0x00C6);
  write_cmos_sensor(0x33E2,0x008A);
  write_cmos_sensor(0x33E8,0x00BE);
  write_cmos_sensor(0x33EE,0x0153);
  write_cmos_sensor(0x33F4,0x0217);
  write_cmos_sensor(0x33FA,0x0001);
  write_cmos_sensor(0x3400,0x0005);
  write_cmos_sensor(0x3406,0x000A);
  write_cmos_sensor(0x340C,0x0037);
  write_cmos_sensor(0x3412,0x0039);
  write_cmos_sensor(0x3418,0x0066);
  write_cmos_sensor(0x341E,0x0068);
  write_cmos_sensor(0x3424,0x0095);
  write_cmos_sensor(0x342A,0x0097);
  write_cmos_sensor(0x3430,0x00C4);
  write_cmos_sensor(0x3436,0x00C6);
  write_cmos_sensor(0x343C,0x00F3);
  write_cmos_sensor(0x3442,0x00F5);
  write_cmos_sensor(0x3448,0x0122);
  write_cmos_sensor(0x344E,0x0124);
  write_cmos_sensor(0x3454,0x0151);
  write_cmos_sensor(0x345A,0x0153);
  write_cmos_sensor(0x3460,0x0180);
  write_cmos_sensor(0x3466,0x0182);
  write_cmos_sensor(0x346C,0x01AF);
  write_cmos_sensor(0x3472,0x01B1);
  write_cmos_sensor(0x3478,0x01DE);
  write_cmos_sensor(0x347E,0x01E0);
  write_cmos_sensor(0x3484,0x020D);
  write_cmos_sensor(0x348A,0x000B);
  write_cmos_sensor(0x3490,0x0038);
  write_cmos_sensor(0x3496,0x003A);
  write_cmos_sensor(0x349C,0x0067);
  write_cmos_sensor(0x34A2,0x0069);
  write_cmos_sensor(0x34A8,0x0096);
  write_cmos_sensor(0x34AE,0x0098);
  write_cmos_sensor(0x34B4,0x00C5);
  write_cmos_sensor(0x34BA,0x00C7);
  write_cmos_sensor(0x34C0,0x00F4);
  write_cmos_sensor(0x34C6,0x00F6);
  write_cmos_sensor(0x34CC,0x0123);
  write_cmos_sensor(0x34D2,0x0125);
  write_cmos_sensor(0x34D8,0x0152);
  write_cmos_sensor(0x34DE,0x0154);
  write_cmos_sensor(0x34E4,0x0181);
  write_cmos_sensor(0x34EA,0x0183);
  write_cmos_sensor(0x34F0,0x01B0);
  write_cmos_sensor(0x34F6,0x01B2);
  write_cmos_sensor(0x34FC,0x0000);
  write_cmos_sensor(0x3502,0x0000);
  write_cmos_sensor(0x3508,0x01DF);
  write_cmos_sensor(0x350E,0x01E1);
  write_cmos_sensor(0x3514,0x020E);
  write_cmos_sensor(0x351A,0x000C);
  write_cmos_sensor(0x3520,0x0039);
  write_cmos_sensor(0x3526,0x003B);
  write_cmos_sensor(0x352C,0x0068);
  write_cmos_sensor(0x3532,0x006A);
  write_cmos_sensor(0x3538,0x0097);
  write_cmos_sensor(0x353E,0x0099);
  write_cmos_sensor(0x3544,0x00C6);
  write_cmos_sensor(0x354A,0x00C8);
  write_cmos_sensor(0x3550,0x00F5);
  write_cmos_sensor(0x3556,0x00F7);
  write_cmos_sensor(0x355C,0x0124);
  write_cmos_sensor(0x3562,0x0126);
  write_cmos_sensor(0x3568,0x0153);
  write_cmos_sensor(0x356E,0x0155);
  write_cmos_sensor(0x3574,0x0182);
  write_cmos_sensor(0x357A,0x0184);
  write_cmos_sensor(0x3580,0x01B1);
  write_cmos_sensor(0x3586,0x01B3);
  write_cmos_sensor(0x358C,0x01E0);
  write_cmos_sensor(0x3592,0x01E2);
  write_cmos_sensor(0x3598,0x020F);
  write_cmos_sensor(0x359E,0x000D);
  write_cmos_sensor(0x35A4,0x003A);
  write_cmos_sensor(0x35AA,0x003C);
  write_cmos_sensor(0x35B0,0x0069);
  write_cmos_sensor(0x35B6,0x006B);
  write_cmos_sensor(0x35BC,0x0098);
  write_cmos_sensor(0x35C2,0x009A);
  write_cmos_sensor(0x35C8,0x00C7);
  write_cmos_sensor(0x35CE,0x00C9);
  write_cmos_sensor(0x35D4,0x00F6);
  write_cmos_sensor(0x35DA,0x00F8);
  write_cmos_sensor(0x35E0,0x0125);
  write_cmos_sensor(0x35E6,0x0127);
  write_cmos_sensor(0x35EC,0x0154);
  write_cmos_sensor(0x35F2,0x0156);
  write_cmos_sensor(0x35F8,0x0183);
  write_cmos_sensor(0x35FE,0x0185);
  write_cmos_sensor(0x3604,0x01B2);
  write_cmos_sensor(0x360A,0x01B4);
  write_cmos_sensor(0x3610,0x01E1);
  write_cmos_sensor(0x3616,0x01E3);
  write_cmos_sensor(0x361C,0x0210);
  write_cmos_sensor(0x3622,0x0009);
  write_cmos_sensor(0x3628,0x0038);
  write_cmos_sensor(0x362E,0x006F);
  write_cmos_sensor(0x3634,0x009E);
  write_cmos_sensor(0x363A,0x00D5);
  write_cmos_sensor(0x3640,0x0104);
  write_cmos_sensor(0x3646,0x013B);
  write_cmos_sensor(0x364C,0x016A);
  write_cmos_sensor(0x3652,0x01A1);
  write_cmos_sensor(0x3658,0x01D0);
  write_cmos_sensor(0x365E,0x0207);
  write_cmos_sensor(0x3664,0x0009);
  write_cmos_sensor(0x366A,0x000C);
  write_cmos_sensor(0x3670,0x00C4);
  write_cmos_sensor(0x3676,0x00DE);
  write_cmos_sensor(0x367C,0x0009);
  write_cmos_sensor(0x3682,0x000C);
  write_cmos_sensor(0x3688,0x00C4);
  write_cmos_sensor(0x368E,0x00C6);
  write_cmos_sensor(0x3694,0x0000);
  write_cmos_sensor(0x369A,0x0000);
  write_cmos_sensor(0x36A0,0x0056);
  write_cmos_sensor(0x36A6,0x021B);
  write_cmos_sensor(0x36AC,0x0057);
  write_cmos_sensor(0x36B2,0x0059);
  write_cmos_sensor(0x36B8,0x00BE);
  write_cmos_sensor(0x36BE,0x00C1);
  write_cmos_sensor(0x36C4,0x0217);
  write_cmos_sensor(0x36CA,0x021A);
  write_cmos_sensor(0x36D0,0x00BE);
  write_cmos_sensor(0x386E,0x00F5);
  write_cmos_sensor(0x3874,0x0138);
  write_cmos_sensor(0x387A,0x024E);
  write_cmos_sensor(0x3880,0x0000);
  write_cmos_sensor(0x3886,0x0073);
  write_cmos_sensor(0x388C,0x0226);
  write_cmos_sensor(0x3892,0x001F);
  write_cmos_sensor(0x3898,0x012C);
  write_cmos_sensor(0x389E,0x0138);
  write_cmos_sensor(0x38A4,0x0285);
  write_cmos_sensor(0x38AA,0x0000);
  write_cmos_sensor(0x38B0,0x0073);
  write_cmos_sensor(0x38B6,0x0226);
  write_cmos_sensor(0x38BC,0x001F);
  write_cmos_sensor(0x38C2,0x00F5);
  write_cmos_sensor(0x38C8,0x0140);
  write_cmos_sensor(0x38CE,0x024E);
  write_cmos_sensor(0x38D4,0x0000);
  write_cmos_sensor(0x38DA,0x0073);
  write_cmos_sensor(0x38E0,0x0226);
  write_cmos_sensor(0x38E6,0x001F);
  write_cmos_sensor(0x38EC,0x012C);
  write_cmos_sensor(0x38F2,0x012A);
  write_cmos_sensor(0x38F8,0x0224);
  write_cmos_sensor(0x38FE,0x0000);
  write_cmos_sensor(0x3904,0x0073);
  write_cmos_sensor(0x390A,0x0226);
  write_cmos_sensor(0x3910,0x0001);
  write_cmos_sensor(0x3968,0x0007); 
  write_cmos_sensor(0x396A,0x0707); 
  write_cmos_sensor(0x396C,0x0707); 
  write_cmos_sensor(0x396E,0x0707);
  write_cmos_sensor(0x3970,0x0707);
  write_cmos_sensor(0x3972,0x0707);
  write_cmos_sensor(0x3974,0x0707);
  write_cmos_sensor(0x3976,0x0707);
  write_cmos_sensor(0x3978,0x0707);
  write_cmos_sensor(0x397A,0x0707);
  write_cmos_sensor(0x397C,0x0707);
  write_cmos_sensor(0x397E,0x0707);
  write_cmos_sensor(0x3980,0x0707);
  write_cmos_sensor(0x3982,0x0707);
  write_cmos_sensor(0x3984,0x0707);
  write_cmos_sensor(0x3986,0x0707);
  write_cmos_sensor(0x3988,0x0707);
  write_cmos_sensor(0x398A,0x0707);
  write_cmos_sensor(0x398C,0x0707);
  write_cmos_sensor(0x398E,0x0707);
  write_cmos_sensor(0x3990,0x0707);
  write_cmos_sensor(0x3992,0x0707);
  write_cmos_sensor(0x3994,0x0707);
  write_cmos_sensor(0x3996,0x0707);
  write_cmos_sensor(0x3998,0x0707);
  write_cmos_sensor(0x399A,0x0707);
  write_cmos_sensor(0x399C,0x0707);
  write_cmos_sensor(0x399E,0x0707);
  write_cmos_sensor(0x39A0,0x0707);
  write_cmos_sensor(0x39A2,0x0707);
  write_cmos_sensor(0x39A4,0x0707);
  write_cmos_sensor(0x39A6,0x0707);
  write_cmos_sensor(0x39A8,0x0707);
  write_cmos_sensor(0x39AA,0x0707);
  write_cmos_sensor(0x39AC,0x0707);
  write_cmos_sensor(0x39AE,0x0707);
  write_cmos_sensor(0x39B0,0x0707);
  write_cmos_sensor(0x39B2,0x0707);
  write_cmos_sensor(0x39B4,0x0707);
  write_cmos_sensor(0x39B6,0x0707);
  write_cmos_sensor(0x39B8,0x0707);
  write_cmos_sensor(0x39BA,0x0707);
  write_cmos_sensor(0x39BC,0x0707);
  write_cmos_sensor(0x39BE,0x0707);
  write_cmos_sensor(0x39C0,0x0707);
  write_cmos_sensor(0x39C2,0x0700);
  write_cmos_sensor(0x623E,0x0004);
  write_cmos_sensor(0x317E,0x0001);
  write_cmos_sensor(0x0B04,0x0101);
  write_cmos_sensor(0x308C,0x0CD0);
  write_cmos_sensor(0x6028,0x2000);   
  write_cmos_sensor(0x602A,0x1DD0);
  write_cmos_sensor(0x6F12,0x0010);
  write_cmos_sensor(0x602A,0x28E4);
  write_cmos_sensor(0x6F12,0xC814);
  write_cmos_sensor(0x602A,0x1F26);
  write_cmos_sensor(0x6F12,0x0C90);
  write_cmos_sensor(0x602A,0x2636);
  write_cmos_sensor(0x6F12,0x0100);
  write_cmos_sensor(0x602A,0x280C);
  write_cmos_sensor(0x6F12,0x0100);
  write_cmos_sensor(0x602A,0x22CE);
  write_cmos_sensor(0x6F12,0x8101);
  write_cmos_sensor(0x602A,0x22D4);
  write_cmos_sensor(0x6F12,0x001A);
  write_cmos_sensor(0x306A,0x0001);

  LOG_INF("%s end.\n", __func__);
 }


 
static void preview_setting(void)
{
  LOG_INF("%s start.\n", __func__);
//$MV1[MCLK:24,Width:2104,Height:1560,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
  write_cmos_sensor(0x0100,0x0000);
//$MV1[MCLK:24,Width:2104,Height:1560,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1344,pvi_pclk_inverse:0]
  write_cmos_sensor(0x6028,0x2000);
  write_cmos_sensor(0x602A,0x282E);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6028,0x4000);
  write_cmos_sensor(0x0344,0x0008);
  write_cmos_sensor(0x0346,0x0008);
  write_cmos_sensor(0x0348,0x10B7);
  write_cmos_sensor(0x034A,0x0C47);
  write_cmos_sensor(0x034C,0x0838);
  write_cmos_sensor(0x034E,0x0618);
  write_cmos_sensor(0x0900,0x0112);
  write_cmos_sensor(0x0380,0x0001);
  write_cmos_sensor(0x0382,0x0001);
  write_cmos_sensor(0x0384,0x0001);
  write_cmos_sensor(0x0386,0x0003);
  write_cmos_sensor(0x0400,0x0000);
  write_cmos_sensor(0x0404,0x0010);
  write_cmos_sensor(0x3060,0x0002);
  write_cmos_sensor(0x0114,0x0300);
  write_cmos_sensor(0x0110,0x1002);
  write_cmos_sensor(0x0136,0x1800);
  write_cmos_sensor(0x0300,0x0004);
  write_cmos_sensor(0x0302,0x0001);
  write_cmos_sensor(0x0304,0x0006);
  write_cmos_sensor(0x0306,0x0078);
  write_cmos_sensor(0x0308,0x0008);
  write_cmos_sensor(0x030A,0x0001);
  write_cmos_sensor(0x030C,0x0000);
  write_cmos_sensor(0x030E,0x0004);
  write_cmos_sensor(0x0310,0x0070);
  write_cmos_sensor(0x0312,0x0000);
  write_cmos_sensor(0x0340,0x0CD0);
  write_cmos_sensor(0x0342,0x1300);
  write_cmos_sensor(0x0202,0x0100);
  write_cmos_sensor(0x0200,0x0000);
  write_cmos_sensor(0x3164,0x10C0);
  write_cmos_sensor(0x3166,0x0C50);
  write_cmos_sensor(0x0408,0x0010);
  write_cmos_sensor(0x040A,0x0004);
  write_cmos_sensor(0xF49E,0x1000);
  write_cmos_sensor(0x3A76,0x3DBC);
  write_cmos_sensor(0x3A88,0x7C80);
  write_cmos_sensor(0x3A94,0x0022);
  write_cmos_sensor(0x3068,0x0100);
  write_cmos_sensor(0x0B0E,0x0000);
  write_cmos_sensor(0x3100,0x1F40);
  write_cmos_sensor(0x0100,0x0100);
  LOG_INF("%s end.\n", __func__);
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
  LOG_INF("%s start. currefps:%d.\n", __func__, currefps);
	if(currefps == 300)
		{
  write_cmos_sensor(0x0100,0x0000);
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1344,pvi_pclk_inverse:0]
  write_cmos_sensor(0x6028,0x2000);
  write_cmos_sensor(0x602A,0x282E);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6028,0x4000);
  write_cmos_sensor(0x0344,0x0008);
  write_cmos_sensor(0x0346,0x0008);
  write_cmos_sensor(0x0348,0x10D7);
  write_cmos_sensor(0x034A,0x0C47);
  write_cmos_sensor(0x034C,0x1070);
  write_cmos_sensor(0x034E,0x0C30);
  write_cmos_sensor(0x0900,0x0011);
  write_cmos_sensor(0x0380,0x0001);
  write_cmos_sensor(0x0382,0x0001);
  write_cmos_sensor(0x0384,0x0001);
  write_cmos_sensor(0x0386,0x0001);
  write_cmos_sensor(0x0400,0x0000);
  write_cmos_sensor(0x0404,0x0010);
  write_cmos_sensor(0x3060,0x0001);
  write_cmos_sensor(0x0114,0x0300);
  write_cmos_sensor(0x0110,0x1002);
  write_cmos_sensor(0x0136,0x1800);
  write_cmos_sensor(0x0300,0x0004);
  write_cmos_sensor(0x0302,0x0001);
  write_cmos_sensor(0x0304,0x0006);
  write_cmos_sensor(0x0306,0x0078);
  write_cmos_sensor(0x0308,0x0008);
  write_cmos_sensor(0x030A,0x0001);
  write_cmos_sensor(0x030C,0x0000);
  write_cmos_sensor(0x030E,0x0004);
  write_cmos_sensor(0x0310,0x0070);
  write_cmos_sensor(0x0312,0x0000);
  write_cmos_sensor(0x0340,0x0CD0);
  write_cmos_sensor(0x0342,0x1300);
  write_cmos_sensor(0x0202,0x0100);
  write_cmos_sensor(0x0200,0x0000);
  write_cmos_sensor(0x3164,0x10E0);
  write_cmos_sensor(0x3166,0x0C50);
  write_cmos_sensor(0x0408,0x0030);
  write_cmos_sensor(0x040A,0x0008);
  write_cmos_sensor(0xF49E,0x1010);
  write_cmos_sensor(0x3A76,0x3DFC);
  write_cmos_sensor(0x3A88,0x7C80);
  write_cmos_sensor(0x3A94,0x0022);
  write_cmos_sensor(0x3068,0x0100);
  write_cmos_sensor(0x0B0E,0x0000);
  write_cmos_sensor(0x3100,0x1F40);
  write_cmos_sensor(0x0100,0x0100);

	}
	
	else if (currefps == 240) {
		LOG_INF("else if (currefps == 240)\n");
write_cmos_sensor(0x0100,0x0000);		
  //$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
  write_cmos_sensor(0x6028,0x2000);
  write_cmos_sensor(0x602A,0x282E);
  write_cmos_sensor(0x6F12,0x0000);
  write_cmos_sensor(0x6028,0x4000);
  write_cmos_sensor(0x0344,0x0008);
  write_cmos_sensor(0x0346,0x0008);
  write_cmos_sensor(0x0348,0x10D7);
  write_cmos_sensor(0x034A,0x0C47);
  write_cmos_sensor(0x034C,0x1070);
  write_cmos_sensor(0x034E,0x0C30);
  write_cmos_sensor(0x0900,0x0011);
  write_cmos_sensor(0x0380,0x0001);
  write_cmos_sensor(0x0382,0x0001);
  write_cmos_sensor(0x0384,0x0001);
  write_cmos_sensor(0x0386,0x0001);
  write_cmos_sensor(0x0400,0x0000);
  write_cmos_sensor(0x0404,0x0010);
  write_cmos_sensor(0x3060,0x0001);
  write_cmos_sensor(0x0114,0x0300);
  write_cmos_sensor(0x0110,0x1002);
  write_cmos_sensor(0x0136,0x1800);
  write_cmos_sensor(0x0300,0x0004);
  write_cmos_sensor(0x0302,0x0001);
  write_cmos_sensor(0x0304,0x0006);
  write_cmos_sensor(0x0306,0x0064);
  write_cmos_sensor(0x0308,0x0008);
  write_cmos_sensor(0x030A,0x0001);
  write_cmos_sensor(0x030C,0x0000);
  write_cmos_sensor(0x030E,0x0004);
  write_cmos_sensor(0x0310,0x0054);
  write_cmos_sensor(0x0312,0x0000);
  write_cmos_sensor(0x0340,0x0DD0);
  write_cmos_sensor(0x0342,0x1260);
  write_cmos_sensor(0x0202,0x0100);
  write_cmos_sensor(0x0200,0x0000);
  write_cmos_sensor(0x3164,0x10E0);
  write_cmos_sensor(0x3166,0x0C50);
  write_cmos_sensor(0x0408,0x0030);
  write_cmos_sensor(0x040A,0x0008);
  write_cmos_sensor(0xF49E,0x1010);
  write_cmos_sensor(0x3A76,0x3DFC);
  write_cmos_sensor(0x3A88,0x7C80);
  write_cmos_sensor(0x3A94,0x0022);
  write_cmos_sensor(0x3068,0x0100);
  write_cmos_sensor(0x0B0E,0x0100);
  write_cmos_sensor(0x3100,0x1F40);
  write_cmos_sensor(0x0100,0x0100);


	}
	else if (currefps == 150) {
//PIP 15fps settings,?∏Ê?Full 30fps
//    -VT : 560-> 400M
//    -Frame length: 3206-> 4589
//   -Linelength: 5808‰∏çË?
//
write_cmos_sensor(0x0100,0x0000);
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]

write_cmos_sensor(0x6028,0x2000);
write_cmos_sensor(0x602A,0x282E);
write_cmos_sensor(0x6F12,0x0000);
write_cmos_sensor(0x6028,0x4000);
write_cmos_sensor(0x0344,0x0008);
write_cmos_sensor(0x0346,0x0008);
write_cmos_sensor(0x0348,0x10D7);
write_cmos_sensor(0x034A,0x0C47);
write_cmos_sensor(0x034C,0x1070);
write_cmos_sensor(0x034E,0x0C30);
write_cmos_sensor(0x0900,0x0011);
write_cmos_sensor(0x0380,0x0001);
write_cmos_sensor(0x0382,0x0001);
write_cmos_sensor(0x0384,0x0001);
write_cmos_sensor(0x0386,0x0001);
write_cmos_sensor(0x0400,0x0000);
write_cmos_sensor(0x0404,0x0010);
write_cmos_sensor(0x3060,0x0001);
write_cmos_sensor(0x0114,0x0300);
write_cmos_sensor(0x0110,0x1002);
write_cmos_sensor(0x0136,0x1800);
write_cmos_sensor(0x0300,0x0004);
write_cmos_sensor(0x0302,0x0001);
write_cmos_sensor(0x0304,0x0006);
write_cmos_sensor(0x0306,0x0078);
write_cmos_sensor(0x0308,0x0008);
write_cmos_sensor(0x030A,0x0001);
write_cmos_sensor(0x030C,0x0001);
write_cmos_sensor(0x030E,0x0004);
write_cmos_sensor(0x0310,0x0032);
write_cmos_sensor(0x0312,0x0000);
write_cmos_sensor(0x0340,0x0D40);
write_cmos_sensor(0x0342,0x1260);
write_cmos_sensor(0x0202,0x0100);
write_cmos_sensor(0x0200,0x0000);
write_cmos_sensor(0x3164,0x10E0);
write_cmos_sensor(0x3166,0x0C50);
write_cmos_sensor(0x0408,0x0030);
write_cmos_sensor(0x040A,0x0008);
write_cmos_sensor(0xF49E,0x1010);
write_cmos_sensor(0x3A76,0x3DFC);
write_cmos_sensor(0x3A88,0x7C80);
write_cmos_sensor(0x3A94,0x0022);
write_cmos_sensor(0x3068,0x0100);
write_cmos_sensor(0x0B0E,0x0100);
write_cmos_sensor(0x3100,0x1F40);
write_cmos_sensor(0x0100,0x0100);
	}
	else { //default fps =15
//PIP 15fps settings,?∏Ê?Full 30fps
//    -VT : 560-> 400M
//    -Frame length: 3206-> 4589
//   -Linelength: 5808‰∏çË?
//
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]

		LOG_INF("else  150fps\n");

write_cmos_sensor(0x0100,0x0000);
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]

write_cmos_sensor(0x6028,0x2000);
write_cmos_sensor(0x602A,0x282E);
write_cmos_sensor(0x6F12,0x0000);
write_cmos_sensor(0x6028,0x4000);
write_cmos_sensor(0x0344,0x0008);
write_cmos_sensor(0x0346,0x0008);
write_cmos_sensor(0x0348,0x10D7);
write_cmos_sensor(0x034A,0x0C47);
write_cmos_sensor(0x034C,0x1070);
write_cmos_sensor(0x034E,0x0C30);
write_cmos_sensor(0x0900,0x0011);
write_cmos_sensor(0x0380,0x0001);
write_cmos_sensor(0x0382,0x0001);
write_cmos_sensor(0x0384,0x0001);
write_cmos_sensor(0x0386,0x0001);
write_cmos_sensor(0x0400,0x0000);
write_cmos_sensor(0x0404,0x0010);
write_cmos_sensor(0x3060,0x0001);
write_cmos_sensor(0x0114,0x0300);
write_cmos_sensor(0x0110,0x1002);
write_cmos_sensor(0x0136,0x1800);
write_cmos_sensor(0x0300,0x0004);
write_cmos_sensor(0x0302,0x0001);
write_cmos_sensor(0x0304,0x0006);
write_cmos_sensor(0x0306,0x0078);
write_cmos_sensor(0x0308,0x0008);
write_cmos_sensor(0x030A,0x0001);
write_cmos_sensor(0x030C,0x0001);
write_cmos_sensor(0x030E,0x0004);
write_cmos_sensor(0x0310,0x0032);
write_cmos_sensor(0x0312,0x0000);
write_cmos_sensor(0x0340,0x0D40);
write_cmos_sensor(0x0342,0x1260);
write_cmos_sensor(0x0202,0x0100);
write_cmos_sensor(0x0200,0x0000);
write_cmos_sensor(0x3164,0x10E0);
write_cmos_sensor(0x3166,0x0C50);
write_cmos_sensor(0x0408,0x0030);
write_cmos_sensor(0x040A,0x0008);
write_cmos_sensor(0xF49E,0x1010);
write_cmos_sensor(0x3A76,0x3DFC);
write_cmos_sensor(0x3A88,0x7C80);
write_cmos_sensor(0x3A94,0x0022);
write_cmos_sensor(0x3068,0x0100);
write_cmos_sensor(0x0B0E,0x0000);
write_cmos_sensor(0x3100,0x1F40);
write_cmos_sensor(0x0100,0x0100);
	}
}

static void normal_video_setting(kal_uint16 currefps)
{
  LOG_INF("%s start.\n", __func__);
  capture_setting(currefps);
}

static void hs_video_setting(void)
{

  LOG_INF("%s start.\n", __func__);
write_cmos_sensor(0x0100,0x0000);	
//$MV1[MCLK:24,Width:1280,Height:720,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1344,pvi_pclk_inverse:0]
write_cmos_sensor(0x6028,0x2000);
write_cmos_sensor(0x602A,0x282E);
write_cmos_sensor(0x6F12,0x0000);
write_cmos_sensor(0x6028,0x4000);
write_cmos_sensor(0x0344,0x00C0);
write_cmos_sensor(0x0346,0x01E8);
write_cmos_sensor(0x0348,0x0FEF);
write_cmos_sensor(0x034A,0x0A6F);
write_cmos_sensor(0x034C,0x0500);
write_cmos_sensor(0x034E,0x02D0);
write_cmos_sensor(0x0900,0x0113);
write_cmos_sensor(0x0380,0x0001);
write_cmos_sensor(0x0382,0x0001);
write_cmos_sensor(0x0384,0x0001);
write_cmos_sensor(0x0386,0x0005);
write_cmos_sensor(0x0400,0x0001);
write_cmos_sensor(0x0404,0x0030);
write_cmos_sensor(0x3060,0x0001);
write_cmos_sensor(0x0114,0x0300);
write_cmos_sensor(0x0110,0x1002);
write_cmos_sensor(0x0136,0x1800);
write_cmos_sensor(0x0300,0x0004);
write_cmos_sensor(0x0302,0x0001);
write_cmos_sensor(0x0304,0x0006);
write_cmos_sensor(0x0306,0x0078);
write_cmos_sensor(0x0308,0x0008);
write_cmos_sensor(0x030A,0x0001);
write_cmos_sensor(0x030C,0x0000);
write_cmos_sensor(0x030E,0x0004);
write_cmos_sensor(0x0310,0x0070);
write_cmos_sensor(0x0312,0x0000);
write_cmos_sensor(0x0340,0x036C);
write_cmos_sensor(0x0342,0x11D0);
write_cmos_sensor(0x0202,0x0100);
write_cmos_sensor(0x0200,0x0000);
write_cmos_sensor(0x3164,0x10B0);
write_cmos_sensor(0x3166,0x0C58);
write_cmos_sensor(0x0408,0x0008);
write_cmos_sensor(0x040A,0x0004);
write_cmos_sensor(0xF49E,0x1000);
write_cmos_sensor(0x3A76,0x3DBC);
write_cmos_sensor(0x3A88,0x2C80);
write_cmos_sensor(0x3A94,0x00A2);
write_cmos_sensor(0x3068,0x0000);
write_cmos_sensor(0x0B0E,0x0000);
write_cmos_sensor(0x3100,0x1F40);
write_cmos_sensor(0x0100,0x0100);


}

static void slim_video_setting(void)
{
  LOG_INF("%s start.\n", __func__);
	
write_cmos_sensor(0x0100,0x0000);
  //$MV1[MCLK:24,Width:1280,Height:720,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
write_cmos_sensor(0x6028,0x2000);
write_cmos_sensor(0x602A,0x282E);
write_cmos_sensor(0x6F12,0x0000);
write_cmos_sensor(0x6028,0x4000);
write_cmos_sensor(0x0344,0x00C0);
write_cmos_sensor(0x0346,0x01E8);
write_cmos_sensor(0x0348,0x0FEF);
write_cmos_sensor(0x034A,0x0A6F);
write_cmos_sensor(0x034C,0x0500);
write_cmos_sensor(0x034E,0x02D0);
write_cmos_sensor(0x0900,0x0113);
write_cmos_sensor(0x0380,0x0001);
write_cmos_sensor(0x0382,0x0001);
write_cmos_sensor(0x0384,0x0001);
write_cmos_sensor(0x0386,0x0005);
write_cmos_sensor(0x0400,0x0001);
write_cmos_sensor(0x0404,0x0030);
write_cmos_sensor(0x3060,0x0001);
write_cmos_sensor(0x0114,0x0300);
write_cmos_sensor(0x0110,0x1002);
write_cmos_sensor(0x0136,0x1800);
write_cmos_sensor(0x0300,0x0004);
write_cmos_sensor(0x0302,0x0001);
write_cmos_sensor(0x0304,0x0006);
write_cmos_sensor(0x0306,0x0078);
write_cmos_sensor(0x0308,0x0008);
write_cmos_sensor(0x030A,0x0001);
write_cmos_sensor(0x030C,0x0000);
write_cmos_sensor(0x030E,0x0004);
write_cmos_sensor(0x0310,0x0032);
write_cmos_sensor(0x0312,0x0000);
write_cmos_sensor(0x0340,0x0DB0);
write_cmos_sensor(0x0342,0x11D0);
write_cmos_sensor(0x0202,0x0100);
write_cmos_sensor(0x0200,0x0000);
write_cmos_sensor(0x3164,0x10B0);
write_cmos_sensor(0x3166,0x0C58);
write_cmos_sensor(0x0408,0x0008);
write_cmos_sensor(0x040A,0x0004);
write_cmos_sensor(0xF49E,0x1000);
write_cmos_sensor(0x3A76,0x3DBC);
write_cmos_sensor(0x3A88,0x2C80);
write_cmos_sensor(0x3A94,0x00A2);
write_cmos_sensor(0x3068,0x0000);
write_cmos_sensor(0x0B0E,0x0000);
write_cmos_sensor(0x3100,0x1F40);
write_cmos_sensor(0x0100,0x0100);
}


static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    int j;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
		LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
		goto otp_read;
            }
		LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
            retry--;
        } while(retry > 0);
        i++;
        retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

otp_read:
	/*
	 * read lsc calibration from OTP E2PROM.
	 */
	primax_otp_buf = (u8 *)kzalloc(OTP_DATA_SIZE, GFP_KERNEL);

	/* read lsc calibration from E2PROM */
	s5k2m8_read_otp_burst(OTP_START_ADDR, primax_otp_buf);
	for (j = 0; j < OTP_DATA_SIZE; j++) {
		LOG_INF("========s5k2m8_primax_otp RegIndex-%d=====val:0x%x======\n", j, *(primax_otp_buf + j));
	}

	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
	LOG_1;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("2015_12_24 i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("%s.\n", __func__);

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);	
	mdelay(5);
	
	/*  
	#ifdef FANPENGTAO
	int i=0;
	for(i=0; i<10; i++){
		LOG_INF("delay time = %d, the frame no = %d\n", i*10, read_cmos_sensor(0x0005));
		mdelay(10);
	}
	#endif 
	*/
	
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 
	else  
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("cap115fps: use cap1's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else  { //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("Warning:=== current_fps %d fps is not support, so use cap1's setting\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps); 
	set_mirror_flip(IMAGE_NORMAL);	
	mdelay(10);

#if 0
	if(imgsensor.test_pattern == KAL_TRUE)
	{
		//write_cmos_sensor(0x5002,0x00);
  }
#endif

	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);	
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}
	
/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom4   */
static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom5   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("%s.\n", __func__);
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame; 
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame; 
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame; 
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame; 
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame; 

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;
	#ifdef FPTPDAFSUPPORT
	sensor_info->PDAF_Support = 1;
	#else 
	sensor_info->PDAF_Support = 0;
	#endif

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
        case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            Custom5(image_window, sensor_config_data); // Custom1
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();            
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
           // set_dummy();            
            break; 
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();            
            break; 
        case MSDK_SCENARIO_ID_CUSTOM4:
            frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();            
            break; 
        case MSDK_SCENARIO_ID_CUSTOM5:
            frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *framerate = imgsensor_info.custom4.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *framerate = imgsensor_info.custom5.max_framerate;
            break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
	
		 write_cmos_sensor(0x0600, 0x0002);
 
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		
		 write_cmos_sensor(0x0600, 0x0000);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
			break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        /******************** PDAF START >>> *********/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // video & capture use same setting
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", *feature_data);
			PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:	
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//S5K2M8_read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;	
        /******************** PDAF END   <<< *********/
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 S5K2M8_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	S5K2M8_MIPI_RAW_SensorInit	*/



