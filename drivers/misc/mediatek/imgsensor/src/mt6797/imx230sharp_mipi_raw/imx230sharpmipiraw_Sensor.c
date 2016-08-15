/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX230sharpmipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx230sharpmipiraw_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "[imx230_sharp]"
#define LOG_1 LOG_INF("IMX230,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2672*2008@30fps; video 5344*4016@30fps; capture 21M@24fps\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#define BYTE               unsigned char

static BOOL read_spc_flag = FALSE;


static DEFINE_SPINLOCK(imgsensor_drv_lock);

u8 *sharp_otp_buf;
extern int main_module_id;
extern int AF_Inf_pos;
extern int AF_Macro_pos;

#define MAX_READ_WRITE_SIZE 8
#define	IMX230_OTP_SIZE	0x096C
#define	OTP_START_ADDR	0x00
#define	E2PROM_WRITE_ID	0xA0

static BYTE imx230_SPC_data[352]={0};

extern void read_imx230_SPC( BYTE* data );
extern void read_imx230_DCC( kal_uint16 addr,BYTE* data, kal_uint32 size);

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX230SHARP_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0xafd83a68,        //checksum value for Camera Auto Test

    .pre = {
        .pclk = 531000000,                //record different mode's pclk
        .linelength = 6024,                //record different mode's linelength
        .framelength = 2880,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2672,        //record different mode's width of grabwindow
        .grabwindow_height = 2008,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .zsd = {
		.pclk = 600000000,
		.linelength = 6024,
		.framelength = 3304,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5312,
		.grabwindow_height = 2988,
		.mipi_data_lp2hs_settle_dc = 60,//unit , ns
		.max_framerate = 300,
    },
    .cap = {
		.pclk = 600000000,
		.linelength = 6024,
		.framelength = 4130,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5344,
		.grabwindow_height = 4016,
		.mipi_data_lp2hs_settle_dc = 60,//unit , ns
		.max_framerate = 240,
    },
    .cap1 = { //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 375000000,
        .linelength = 6024,
        .framelength = 4126,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 5344,
        .grabwindow_height = 4016,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 150,    //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
    },
    .normal_video = {
        .pclk = 531000000,                //record different mode's pclk
        .linelength = 6024,                //record different mode's linelength
        .framelength = 2880,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2672,        //record different mode's width of grabwindow
        .grabwindow_height = 2008,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 597000000,
        .linelength = 6024,
        .framelength = 824,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 1200,
    },
    .hd_4k_video = {
	.pclk = 411000000,
	.linelength = 6024,
	.framelength = 2274,
	.startx = 0,
	.starty = 0,
	.grabwindow_width = 3840,
	.grabwindow_height = 2160,
	.mipi_data_lp2hs_settle_dc = 85,//unit , ns
	.max_framerate = 300,
    },
    .slim_video = {
        .pclk = 201000000,
        .linelength = 6024,
        .framelength = 1112,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .margin = 4,            //sensor framelength & shutter margin
    .min_shutter = 1,        //min shutter
    .max_frame_length = 0xFFE0,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 7,      //support sensor mode num

    .cap_delay_frame = 1,        //enter capture delay frame num
    .pre_delay_frame = 1,         //enter preview delay frame num
    .video_delay_frame = 1,        //enter video delay frame num
    .hs_video_delay_frame = 3,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 3,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = {0x34, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_mode = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x6c,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] =
{{ 5344, 4016,    0,    0, 5344, 4016, 2672, 2008, 0000, 0000, 2672, 2008,      0,    0, 2672, 2008}, // Preview
 { 5344, 4016,    0,    0, 5344, 4016, 5344, 4016, 0000, 0000, 5344, 4016,      0,    0, 5344, 4016}, // capture
 { 5344, 4016,    0,    0, 5344, 4016, 2672, 2008, 0000, 0000, 2672, 2008,      0,    0, 2672, 2008}, // video
 { 5344, 4016,    0,  568, 5344, 4016, 1280,  720, 0000, 0000, 1280,  720,      0,    0, 1280,  720}, //hight speed video
 { 5344, 4016,    0,  504, 5344, 4016, 1280,  720, 0000, 0000, 1280,  720,      0,    0, 1280,  720}, //slim video
 { 5344, 4016,    0,    0, 5312, 2988, 5312, 2988, 0000, 0000, 5312, 2988,      0,    0, 5312, 2988}, // zsd
 { 5344, 4016,    0,    0, 3840, 2160, 3840, 2160, 0000, 0000, 3840, 2160,      0,    0, 3840, 2160}};// 4k video

 /*VC1 for HDR(DT=0X35) , VC2 for PDAF(DT=0X36), unit : 10bit*/
 static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
 {/* Preview mode setting */
 {0x03, 0x0a,   0x00,   0x08, 0x40, 0x00,
  0x00, 0x2b, 0x0A70, 0x07D8, 0x00, 0x35, 0x0280, 0x0001,
  0x00, 0x36, 0x0C48, 0x0001, 0x03, 0x00, 0x0000, 0x0000},
  /* Capture mode setting */
  {0x03, 0x0a,	 0x00,	 0x08, 0x40, 0x00,
   0x00, 0x2b, 0x14E0, 0x0FB0, 0x00, 0x35, 0x0280, 0x0001,
  0x00, 0x36, 0x1a18, 0x0001, 0x03, 0x00, 0x0000, 0x0000},
   /* Video mode setting */
  {0x02, 0x0a,	 0x00,	 0x08, 0x40, 0x00,
   0x00, 0x2b, 0x14E0, 0x0FB0, 0x01, 0x00, 0x0000, 0x0000,
   0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000}
};

typedef struct
{
    MUINT16 DarkLimit_H;
    MUINT16 DarkLimit_L;
    MUINT16 OverExp_Min_H;
    MUINT16 OverExp_Min_L;
    MUINT16 OverExp_Max_H;
    MUINT16 OverExp_Max_L;
}SENSOR_ATR_INFO, *pSENSOR_ATR_INFO;
#if 0
static SENSOR_ATR_INFO sensorATR_Info[4]=
{/* Strength Range Min */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /* Strength Range Std */
    {0x00, 0x32, 0x00, 0x3c, 0x03, 0xff},
    /* Strength Range Max */
    {0x3f, 0xff, 0x3f, 0xff, 0x3f, 0xff},
    /* Strength Range Custom */
    {0x3F, 0xFF, 0x00, 0x0, 0x3F, 0xFF}};
#endif

#define IMX230MIPI_MaxGainIndex (115+32)
static kal_uint16 IMX230MIPI_sensorGainMapping[IMX230MIPI_MaxGainIndex][2] ={
	{64 ,0	},
	{65 ,8	},
	{66 ,16 },
	{67 ,25 },
	{68 ,30 },
	{69 ,37 },
	{70 ,45 },
	{71 ,51 },
	{72 ,57 },
	{73 ,63 },
	{74 ,67 },
	{75 ,75 },
	{76 ,81 },
	{77 ,85 },
	{78 ,92 },
	{79 ,96 },
	{80 ,103},
	{81 ,107},
	{82 ,112},
	{83 ,118},
	{84 ,122},
	{86 ,133},
	{88 ,140},
	{89 ,144},
	{90 ,148},
	{93 ,159},
	{96 ,171},
	{97 ,175},
	{99 ,182},
	{101,188},
	{102,192},
	{104,197},
	{106,202},
	{107,206},
	{109,211},
	{112,220},
	{113,222},
	{115,228},
	{118,235},
	{120,239},
	{125,250},
	{126,252},
	{128,256},
	{129,258},
	{130,260},
	{132,264},
	{133,266},
	{135,269},
	{136,271},
	{138,274},
	{139,276},
	{141,279},
	{142,282},
	{144,285},
	{145,286},
	{147,290},
	{149,292},
	{150,294},
	{155,300},
	{157,303},
	{158,305},
	{161,309},
	{163,311},
	{170,319},
	{172,322},
	{174,324},
	{176,326},
	{179,329},
	{181,331},
	{185,335},
	{189,339},
	{193,342},
	{195,344},
	{196,345},
	{200,348},
	{202,350},
	{205,352},
	{207,354},
	{210,356},
	{211,357},
	{214,359},
	{217,361},
	{218,362},
	{221,364},
	{224,366},
	{231,370},
	{237,374},
	{246,379},
	{250,381},
	{252,382},
	{256,384},
	{260,386},
	{262,387},
	{273,392},
	{275,393},
	{280,395},
	{290,399},
	{306,405},
	{312,407},
	{321,410},
	{331,413},
	{345,417},
	{352,419},
	{360,421},
	{364,422},
	{372,424},
	{386,427},
	{400,430},
	{410,432},
	{420,434},
	{431,436},
	{437,437},
	{449,439},
	{468,442},
	{512,448},
	{521,449},
	{530,450},
	{538,451},
	{548,452},
	{558,453},
	{568,454},
	{578,455},
	{589,456},
	{601,457},
	{612,458},
	{625,459},
	{637,460},
	{649,461},
	{665,462},
	{678,463},
	{693,464},
	{707,465},
	{725,466},
	{742,467},
	{759,468},
	{776,469},
	{797,470},
	{819,471},
	{840,472},
	{864,473},
	{887,474},
	{913,475},
	{939,476},
	{968,477},
	{999,478},
	{1031,479},
	{1062,480},
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
static kal_uint16 zvhdr_setting(kal_uint8 flag){
	if(flag == 1)
	{
		write_cmos_sensor(0x30b4,0x01);
	    write_cmos_sensor(0x30b5,0x01);
	    write_cmos_sensor(0x30b6,0x01);
	    write_cmos_sensor(0x30b7,0x01);
	    write_cmos_sensor(0x30b8,0x01);
	    write_cmos_sensor(0x30b9,0x01);
	    write_cmos_sensor(0x30ba,0x01);
	    write_cmos_sensor(0x30bb,0x01);
		write_cmos_sensor(0x30bc,0x01);
	}
	else
	{
		write_cmos_sensor(0x30b4,0x00);
	    write_cmos_sensor(0x30b5,0x00);
	    write_cmos_sensor(0x30b6,0x00);
	    write_cmos_sensor(0x30b7,0x00);
	    write_cmos_sensor(0x30b8,0x00);
	    write_cmos_sensor(0x30b9,0x00);
	    write_cmos_sensor(0x30ba,0x00);
	    write_cmos_sensor(0x30bb,0x00);
		write_cmos_sensor(0x30bc,0x00);
	}
	return 0;
}
static int imx230_read_otp_byte(u16 addr, u8 *buf)
{
	int ret = 0;
	u8 pu_send_cmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};

	ret = iReadRegI2C(pu_send_cmd, 2, (u8*)buf, 1, E2PROM_WRITE_ID);
	if (ret < 0)
		LOG_INF("read data from sharp otp e2prom failed!\n");

	return ret;
}


static void imx230_read_otp_all(u16 addr, u8 *otp_buf)
{
	int i;
	int ret = 0;

	for (i = 0; i < IMX230_OTP_SIZE; i ++) {
		
		ret = imx230_read_otp_byte(addr+i, otp_buf+i);
		if(ret < 0){
		   LOG_INF("read sharp otp failed!   error address: 0x%X\n",addr+i);
		   break;
		}
	}
}

static MUINT32 cur_startpos = 0;
static MUINT32 cur_size = 0;

static void imx230_set_pd_focus_area(MUINT32 startpos, MUINT32 size)
{
	UINT16 start_x_pos, start_y_pos, end_x_pos, end_y_pos;
	UINT16 focus_width, focus_height;

	if((cur_startpos == startpos) && (cur_size == size))
	{
		LOG_INF("Not to need update focus area!\n");
		return;
	}
	else
	{
		cur_startpos = startpos;
		cur_size = size;
	}
	
	start_x_pos = (startpos >> 16) & 0xFFFF;
	start_y_pos = startpos & 0xFFFF;
	focus_width = (size >> 16) & 0xFFFF;
	focus_height = size & 0xFFFF;

	end_x_pos = start_x_pos + focus_width;
	end_y_pos = start_y_pos + focus_height;

	if(imgsensor.pdaf_mode == 1)
	{
		LOG_INF("GC pre PDAF\n");
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);

		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,(start_x_pos >> 8) & 0xFF);
		write_cmos_sensor(0x3159,start_x_pos & 0xFF);// X start
		write_cmos_sensor(0x315a,(start_y_pos >> 8) & 0xFF);
		write_cmos_sensor(0x315b,start_y_pos & 0xFF);// Y start
		write_cmos_sensor(0x315c,(end_x_pos >> 8) & 0xFF);
		write_cmos_sensor(0x315d,end_x_pos & 0xFF);//X end 
		write_cmos_sensor(0x315e,(end_y_pos >> 8) & 0xFF);
		write_cmos_sensor(0x315f,end_y_pos & 0xFF);// Y end

		
	}


	LOG_INF("start_x_pos:%d, start_y_pos:%d, focus_width:%d, focus_height:%d, end_x_pos:%d, end_y_pos:%d\n", \
			start_x_pos, start_y_pos, focus_width, focus_height, end_x_pos, end_y_pos);
	
	return;
}

static void imx230_apply_SPC(void)
{
	unsigned int start_reg = 0x7c00;
	int i;

	if(read_spc_flag == FALSE)
	{
		read_imx230_SPC(imx230_SPC_data);
		read_spc_flag = TRUE;
		return;
	}

	for(i=0;i<352;i++)
	{
		write_cmos_sensor(start_reg, imx230_SPC_data[i]);
	//	LOG_INF("SPC[%d]= %x \n", i , imx230_SPC_data[i]);

		start_reg++;
	}

}

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    /* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0104, 0x01);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	write_cmos_sensor(0x0A02, 0x1F);
	write_cmos_sensor(0x0A00, 0x01);
	write_cmos_sensor(0x0A01, 0x01);
    return ((read_cmos_sensor(0x0A38) << 4) | (read_cmos_sensor(0x0A39)>>4));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

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
}    /*    set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	#define MAX_SHUTTER  40039602  /*max exposure time:402s---> shutter=40039602*/
	#define FRAME_LENGTH_MAX 65072 

	kal_uint16 realtime_fps = 0;
	//kal_uint32 frame_length = 0;
	kal_uint32 line_length=0;
	kal_uint32 origi_shutter;
	kal_uint32 exposure_time;
	kal_uint16 long_exposure_value;

	LOG_INF("enter shutter =%d \n", shutter);
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

	/* Just should be called in capture case with long exposure */
        if(shutter == 5)
        {
	LOG_INF("Quit long exposure mode\n");
        line_length = 6024;
	/* Stream off */
	write_cmos_sensor(0x0100,0x00);
	write_cmos_sensor(0x3028, 0x0);
	/* Stream on */
	write_cmos_sensor(0x0100,0x01);
	
	spin_lock(&imgsensor_drv_lock);
	if(IMGSENSOR_MODE_CAPTURE == imgsensor.sensor_mode){
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
	}
	spin_unlock(&imgsensor_drv_lock);
        }

        if(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
        {
	/* limit the max long exposure to 20s */
        if(shutter > MAX_SHUTTER)
		shutter = MAX_SHUTTER;

	exposure_time = shutter/600 * 6024/10000; /*Make sure exposure_time is int,so x100*/
	exposure_time = (exposure_time+1)/2*2;

	if(exposure_time >= 2000){    /*exposure time >= 20s,0x3028=0x07*/
        	write_cmos_sensor(0x3028, 0x07);
		long_exposure_value = 128;
	}
	else if(exposure_time >= 1100){    /*exposure time >= 11s,0x3028=0x06*/
        	write_cmos_sensor(0x3028, 0x06);
		long_exposure_value = 64;
	}
	else if(exposure_time >= 600){    /*exposure time >= 6s,0x3028=0x05*/
        	write_cmos_sensor(0x3028, 0x05);
		long_exposure_value = 32;
	}
	else if(exposure_time >= 300){    /*exposure time >= 3s,0x3028=0x04*/
        	write_cmos_sensor(0x3028, 0x04);
		long_exposure_value = 16;
	}
	else if(exposure_time >= 200){    /*exposure time >= 2s,0x3028=0x03*/
        	write_cmos_sensor(0x3028, 0x03);
		long_exposure_value = 8;
	}
	else if(exposure_time >= 70){    /*exposure time >= 0.7s,0x3028=0x02*/
        	write_cmos_sensor(0x3028, 0x02);
		long_exposure_value = 4;
	}
	else if(exposure_time >= 40){    /*exposure time >= 0.4s,0x3028=0x01*/
        	write_cmos_sensor(0x3028, 0x01);
		long_exposure_value = 2;
	}
	else{    /*exposure time < 0.4s,0x3028=0x00*/
        	write_cmos_sensor(0x3028, 0x00);
		long_exposure_value = 1;
	}
		
	shutter = shutter/long_exposure_value;
	if(shutter >= FRAME_LENGTH_MAX){
		line_length = (exposure_time/100)*600000000/shutter/128;
	}
	else{
		line_length = 6024;
	}
	
	LOG_INF("long exposure mode,shutter =%d exposure time :%d,long exposure value(0x3028):%d\n", shutter,exposure_time,long_exposure_value);
 
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
        
	write_cmos_sensor(0x0342, (line_length >> 8) & 0xFF);
        write_cmos_sensor(0x0343, line_length & 0xFF);
 
	imgsensor.line_length = line_length;
        }
	else{
		/*Reset 0x3028 to default value*/
        	write_cmos_sensor(0x3028, 0x00);
	}

	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        	if(realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 237 && realtime_fps <= 245) {
            		set_max_framerate(236,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 183 && realtime_fps <= 220) {
            		set_max_framerate(146,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 157 && realtime_fps <= 183) {
            		set_max_framerate(157,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 138 && realtime_fps <= 157) {
            		set_max_framerate(138,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 129 && realtime_fps <= 138) {
            		set_max_framerate(129,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 116 && realtime_fps <= 129) {
            		set_max_framerate(116,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 105 && realtime_fps <= 116) {
            		set_max_framerate(105,0);
            		write_cmos_sensor(0x0104, 0x01);
        	} else if(realtime_fps >= 96 && realtime_fps <= 105) {
            		set_max_framerate(96,0);
            		write_cmos_sensor(0x0104, 0x01);
		} else {
			// Extend frame length
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		}
	} else {
        	// Extend frame length
        	write_cmos_sensor(0x0104, 0x01);
        	write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
        	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter */
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	
	LOG_INF("Exit! shutter =%d, framelength =%d, linelength = %d\n", shutter,imgsensor.frame_length, imgsensor.line_length);

}    /*    set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;
    LOG_INF("[IMX230MIPI]enter IMX230MIPIGain2Reg function\n");
    for (iI = 0; iI < IMX230MIPI_MaxGainIndex; iI++)
	{
		if(gain < IMX230MIPI_sensorGainMapping[iI][0])
		{
			return IMX230MIPI_sensorGainMapping[iI][1];
		}


    }
	if(iI != IMX230MIPI_MaxGainIndex)
	{
    	if(gain != IMX230MIPI_sensorGainMapping[iI][0])
    	{
        	 LOG_INF("Gain mapping don't correctly:%d %d \n", gain, IMX230MIPI_sensorGainMapping[iI][0]);
    	}
    }
	LOG_INF("exit IMX230MIPIGain2Reg function\n");
    return IMX230MIPI_sensorGainMapping[iI-1][1];
}

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    //
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

	write_cmos_sensor(0x0104, 0x01);
    write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
    write_cmos_sensor(0x0104, 0x00);

    return gain;
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{

    kal_uint16 realtime_fps = 0;
    kal_uint16 reg_gain;
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    spin_lock(&imgsensor_drv_lock);
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        write_cmos_sensor(0x0104, 0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0x00);
        }
    } else {
        write_cmos_sensor(0x0104, 0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0x00);
    }
    write_cmos_sensor(0x0104, 0x01);
    /* Long exposure */
    write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor(0x0203, le  & 0xFF);
    /* Short exposure */
    write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor(0x0225, se  & 0xFF);
    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    /* Global analog Gain for Long expo*/
    write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
    write_cmos_sensor(0x0205, reg_gain & 0xFF);
    /* Global analog Gain for Short expo*/
    write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
    write_cmos_sensor(0x0217, reg_gain & 0xFF);
    write_cmos_sensor(0x0104, 0x00);

}

static void hdr_write_shutter(kal_uint16 le, kal_uint16 se)
{
    kal_uint16 realtime_fps = 0;
    LOG_INF("le:0x%x, se:0x%x\n",le,se);
    spin_lock(&imgsensor_drv_lock);
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        write_cmos_sensor(0x0104, 0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0x00);
        }
    } else {
        write_cmos_sensor(0x0104, 0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0x00);
    }
    write_cmos_sensor(0x0104, 0x01);
    /* Long exposure */
    write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor(0x0203, le  & 0xFF);
    /* Short exposure */
    write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor(0x0225, se  & 0xFF);
    write_cmos_sensor(0x0104, 0x00);

}


#if 0
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

    switch (image_mirror) {
        case IMAGE_NORMAL:
			write_cmos_sensor(0x0101, 0x00);
			write_cmos_sensor(0x3A27, 0x00);
			write_cmos_sensor(0x3A28, 0x00);
			write_cmos_sensor(0x3A29, 0x01);
			write_cmos_sensor(0x3A2A, 0x00);
			write_cmos_sensor(0x3A2B, 0x00);
			write_cmos_sensor(0x3A2C, 0x00);
			write_cmos_sensor(0x3A2D, 0x01);
			write_cmos_sensor(0x3A2E, 0x01);
			break;
        case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101, 0x01);
			write_cmos_sensor(0x3A27, 0x01);
			write_cmos_sensor(0x3A28, 0x01);
			write_cmos_sensor(0x3A29, 0x00);
			write_cmos_sensor(0x3A2A, 0x00);
			write_cmos_sensor(0x3A2B, 0x01);
			write_cmos_sensor(0x3A2C, 0x00);
			write_cmos_sensor(0x3A2D, 0x00);
			write_cmos_sensor(0x3A2E, 0x01);
            break;
        case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101, 0x02);
			write_cmos_sensor(0x3A27, 0x10);
			write_cmos_sensor(0x3A28, 0x10);
			write_cmos_sensor(0x3A29, 0x01);
			write_cmos_sensor(0x3A2A, 0x01);
			write_cmos_sensor(0x3A2B, 0x00);
			write_cmos_sensor(0x3A2C, 0x01);
			write_cmos_sensor(0x3A2D, 0x01);
			write_cmos_sensor(0x3A2E, 0x00);
            break;
        case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, 0x03);
			write_cmos_sensor(0x3A27, 0x11);
			write_cmos_sensor(0x3A28, 0x11);
			write_cmos_sensor(0x3A29, 0x00);
			write_cmos_sensor(0x3A2A, 0x01);
			write_cmos_sensor(0x3A2B, 0x01);
			write_cmos_sensor(0x3A2C, 0x01);
			write_cmos_sensor(0x3A2D, 0x00);
			write_cmos_sensor(0x3A2E, 0x00);
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }

}
#endif
/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void sensor_init(void)
{
	LOG_INF("%s\n", __func__);
	/* External Clock Setting */
	write_cmos_sensor(0x0136, 0x1B);
	write_cmos_sensor(0x0137, 0x00);

	/* Global Setting */
	//add for test
	write_cmos_sensor(0x452c, 0x01);
	write_cmos_sensor(0x4630, 0x01);
	write_cmos_sensor(0x4F61, 0x64);
	write_cmos_sensor(0x4F65, 0x64);
	write_cmos_sensor(0x4F79, 0x64);
	write_cmos_sensor(0x4F7D, 0x64);
	//add end
	write_cmos_sensor(0x4800, 0x0E);
	write_cmos_sensor(0x4890, 0x01);
	write_cmos_sensor(0x4D1E, 0x01);
	write_cmos_sensor(0x4D1F, 0xFF);
	write_cmos_sensor(0x4FA0, 0x00);
	write_cmos_sensor(0x4FA1, 0x00);
	write_cmos_sensor(0x4FA2, 0x00);
	write_cmos_sensor(0x4FA3, 0x83);
	write_cmos_sensor(0x6153, 0x01);
	write_cmos_sensor(0x6156, 0x01);
	write_cmos_sensor(0x69BB, 0x01);
	write_cmos_sensor(0x69BC, 0x05);
	write_cmos_sensor(0x69BD, 0x05);
	write_cmos_sensor(0x69C1, 0x00);
	write_cmos_sensor(0x69C4, 0x01);
	write_cmos_sensor(0x69C6, 0x01);
	write_cmos_sensor(0x7300, 0x00);
	write_cmos_sensor(0x9009, 0x1A);
	write_cmos_sensor(0xB040, 0x90);
	write_cmos_sensor(0xB041, 0x14);
	write_cmos_sensor(0xB042, 0x6B);
	write_cmos_sensor(0xB043, 0x43);
	write_cmos_sensor(0xB044, 0x63);
	write_cmos_sensor(0xB045, 0x2A);
	write_cmos_sensor(0xB046, 0x68);
	write_cmos_sensor(0xB047, 0x06);
	write_cmos_sensor(0xB048, 0x68);
	write_cmos_sensor(0xB049, 0x07);
	write_cmos_sensor(0xB04A, 0x68);
	write_cmos_sensor(0xB04B, 0x04);
	write_cmos_sensor(0xB04C, 0x68);
	write_cmos_sensor(0xB04D, 0x05);
	write_cmos_sensor(0xB04E, 0x68);
	write_cmos_sensor(0xB04F, 0x16);
	write_cmos_sensor(0xB050, 0x68);
	write_cmos_sensor(0xB051, 0x17);
	write_cmos_sensor(0xB052, 0x68);
	write_cmos_sensor(0xB053, 0x74);
	write_cmos_sensor(0xB054, 0x68);
	write_cmos_sensor(0xB055, 0x75);
	write_cmos_sensor(0xB056, 0x68);
	write_cmos_sensor(0xB057, 0x76);
	write_cmos_sensor(0xB058, 0x68);
	write_cmos_sensor(0xB059, 0x77);
	write_cmos_sensor(0xB05A, 0x68);
	write_cmos_sensor(0xB05B, 0x7A);
	write_cmos_sensor(0xB05C, 0x68);
	write_cmos_sensor(0xB05D, 0x7B);
	write_cmos_sensor(0xB05E, 0x68);
	write_cmos_sensor(0xB05F, 0x0A);
	write_cmos_sensor(0xB060, 0x68);
	write_cmos_sensor(0xB061, 0x0B);
	write_cmos_sensor(0xB062, 0x68);
	write_cmos_sensor(0xB063, 0x08);
	write_cmos_sensor(0xB064, 0x68);
	write_cmos_sensor(0xB065, 0x09);
	write_cmos_sensor(0xB066, 0x68);
	write_cmos_sensor(0xB067, 0x0E);
	write_cmos_sensor(0xB068, 0x68);
	write_cmos_sensor(0xB069, 0x0F);
	write_cmos_sensor(0xB06A, 0x68);
	write_cmos_sensor(0xB06B, 0x0C);
	write_cmos_sensor(0xB06C, 0x68);
	write_cmos_sensor(0xB06D, 0x0D);
	write_cmos_sensor(0xB06E, 0x68);
	write_cmos_sensor(0xB06F, 0x13);
	write_cmos_sensor(0xB070, 0x68);
	write_cmos_sensor(0xB071, 0x12);
	write_cmos_sensor(0xB072, 0x90);
	write_cmos_sensor(0xB073, 0x0E);
	write_cmos_sensor(0xD000, 0xDA);
	write_cmos_sensor(0xD001, 0xDA);
	write_cmos_sensor(0xD002, 0xAF);
	write_cmos_sensor(0xD003, 0xE1);
	write_cmos_sensor(0xD004, 0x55);
	write_cmos_sensor(0xD005, 0x34);
	write_cmos_sensor(0xD006, 0x21);
	write_cmos_sensor(0xD007, 0x00);
	write_cmos_sensor(0xD008, 0x1C);
	write_cmos_sensor(0xD009, 0x80);
	write_cmos_sensor(0xD00A, 0xFE);
	write_cmos_sensor(0xD00B, 0xC5);
	write_cmos_sensor(0xD00C, 0x55);
	write_cmos_sensor(0xD00D, 0xDC);
	write_cmos_sensor(0xD00E, 0xB6);
	write_cmos_sensor(0xD00F, 0x00);
	write_cmos_sensor(0xD010, 0x31);
	write_cmos_sensor(0xD011, 0x02);
	write_cmos_sensor(0xD012, 0x4A);
	write_cmos_sensor(0xD013, 0x0E);
	write_cmos_sensor(0xD014, 0x55);
	write_cmos_sensor(0xD015, 0xF0);
	write_cmos_sensor(0xD016, 0x1B);
	write_cmos_sensor(0xD017, 0x00);
	write_cmos_sensor(0xD018, 0xFA);
	write_cmos_sensor(0xD019, 0x2C);
	write_cmos_sensor(0xD01A, 0xF1);
	write_cmos_sensor(0xD01B, 0x7E);
	write_cmos_sensor(0xD01C, 0x55);
	write_cmos_sensor(0xD01D, 0x1C);
	write_cmos_sensor(0xD01E, 0xD8);
	write_cmos_sensor(0xD01F, 0x00);
	write_cmos_sensor(0xD020, 0x76);
	write_cmos_sensor(0xD021, 0xC1);
	write_cmos_sensor(0xD022, 0xBF);
	write_cmos_sensor(0xD044, 0x40);
	write_cmos_sensor(0xD045, 0xBA);
	write_cmos_sensor(0xD046, 0x70);
	write_cmos_sensor(0xD047, 0x47);
	write_cmos_sensor(0xD048, 0xC0);
	write_cmos_sensor(0xD049, 0xBA);
	write_cmos_sensor(0xD04A, 0x70);
	write_cmos_sensor(0xD04B, 0x47);
	write_cmos_sensor(0xD04C, 0x82);
	write_cmos_sensor(0xD04D, 0xF6);
	write_cmos_sensor(0xD04E, 0xDA);
	write_cmos_sensor(0xD04F, 0xFA);
	write_cmos_sensor(0xD050, 0x00);
	write_cmos_sensor(0xD051, 0xF0);
	write_cmos_sensor(0xD052, 0x02);
	write_cmos_sensor(0xD053, 0xF8);
	write_cmos_sensor(0xD054, 0x81);
	write_cmos_sensor(0xD055, 0xF6);
	write_cmos_sensor(0xD056, 0xCE);
	write_cmos_sensor(0xD057, 0xFD);
	write_cmos_sensor(0xD058, 0x10);
	write_cmos_sensor(0xD059, 0xB5);
	write_cmos_sensor(0xD05A, 0x0D);
	write_cmos_sensor(0xD05B, 0x48);
	write_cmos_sensor(0xD05C, 0x40);
	write_cmos_sensor(0xD05D, 0x7A);
	write_cmos_sensor(0xD05E, 0x01);
	write_cmos_sensor(0xD05F, 0x28);
	write_cmos_sensor(0xD060, 0x15);
	write_cmos_sensor(0xD061, 0xD1);
	write_cmos_sensor(0xD062, 0x0C);
	write_cmos_sensor(0xD063, 0x49);
	write_cmos_sensor(0xD064, 0x0C);
	write_cmos_sensor(0xD065, 0x46);
	write_cmos_sensor(0xD066, 0x40);
	write_cmos_sensor(0xD067, 0x3C);
	write_cmos_sensor(0xD068, 0x48);
	write_cmos_sensor(0xD069, 0x8A);
	write_cmos_sensor(0xD06A, 0x62);
	write_cmos_sensor(0xD06B, 0x8A);
	write_cmos_sensor(0xD06C, 0x80);
	write_cmos_sensor(0xD06D, 0x1A);
	write_cmos_sensor(0xD06E, 0x8A);
	write_cmos_sensor(0xD06F, 0x89);
	write_cmos_sensor(0xD070, 0x00);
	write_cmos_sensor(0xD071, 0xB2);
	write_cmos_sensor(0xD072, 0x10);
	write_cmos_sensor(0xD073, 0x18);
	write_cmos_sensor(0xD074, 0x0A);
	write_cmos_sensor(0xD075, 0x46);
	write_cmos_sensor(0xD076, 0x20);
	write_cmos_sensor(0xD077, 0x32);
	write_cmos_sensor(0xD078, 0x12);
	write_cmos_sensor(0xD079, 0x88);
	write_cmos_sensor(0xD07A, 0x90);
	write_cmos_sensor(0xD07B, 0x42);
	write_cmos_sensor(0xD07C, 0x00);
	write_cmos_sensor(0xD07D, 0xDA);
	write_cmos_sensor(0xD07E, 0x10);
	write_cmos_sensor(0xD07F, 0x46);
	write_cmos_sensor(0xD080, 0x80);
	write_cmos_sensor(0xD081, 0xB2);
	write_cmos_sensor(0xD082, 0x88);
	write_cmos_sensor(0xD083, 0x81);
	write_cmos_sensor(0xD084, 0x84);
	write_cmos_sensor(0xD085, 0xF6);
	write_cmos_sensor(0xD086, 0x06);
	write_cmos_sensor(0xD087, 0xF8);
	write_cmos_sensor(0xD088, 0xE0);
	write_cmos_sensor(0xD089, 0x67);
	write_cmos_sensor(0xD08A, 0x85);
	write_cmos_sensor(0xD08B, 0xF6);
	write_cmos_sensor(0xD08C, 0x4B);
	write_cmos_sensor(0xD08D, 0xFC);
	write_cmos_sensor(0xD08E, 0x10);
	write_cmos_sensor(0xD08F, 0xBD);
	write_cmos_sensor(0xD090, 0x00);
	write_cmos_sensor(0xD091, 0x18);
	write_cmos_sensor(0xD092, 0x1E);
	write_cmos_sensor(0xD093, 0x78);
	write_cmos_sensor(0xD094, 0x00);
	write_cmos_sensor(0xD095, 0x18);
	write_cmos_sensor(0xD096, 0x17);
	write_cmos_sensor(0xD097, 0x98);

	/* Load Setting */
	write_cmos_sensor(0x5869, 0x01);

	/* DPC2D Setting */
	write_cmos_sensor(0x6953, 0x01);
	write_cmos_sensor(0x6962, 0x3A);
	write_cmos_sensor(0x69CD, 0x3A);
	write_cmos_sensor(0x9258, 0x00);
	write_cmos_sensor(0x9906, 0x00);
	write_cmos_sensor(0x9907, 0x28);
	write_cmos_sensor(0x9976, 0x0A);
	write_cmos_sensor(0x99B0, 0x20);
	write_cmos_sensor(0x99B1, 0x20);
	write_cmos_sensor(0x99B2, 0x20);
	write_cmos_sensor(0x99C6, 0x6E);
	write_cmos_sensor(0x99C7, 0x6E);
	write_cmos_sensor(0x99C8, 0x6E);
	write_cmos_sensor(0x9A1F, 0x0A);
	write_cmos_sensor(0x9AB0, 0x20);
	write_cmos_sensor(0x9AB1, 0x20);
	write_cmos_sensor(0x9AB2, 0x20);
	write_cmos_sensor(0x9AC6, 0x6E);
	write_cmos_sensor(0x9AC7, 0x6E);
	write_cmos_sensor(0x9AC8, 0x6E);
	write_cmos_sensor(0x9B01, 0x08);
	write_cmos_sensor(0x9B03, 0x1B);
	write_cmos_sensor(0x9B05, 0x20);
	write_cmos_sensor(0x9B07, 0x28);
	write_cmos_sensor(0x9B08, 0x01);
	write_cmos_sensor(0x9B09, 0x33);
	write_cmos_sensor(0x9B0A, 0x01);
	write_cmos_sensor(0x9B0B, 0x40);
	write_cmos_sensor(0x9B13, 0x10);
	write_cmos_sensor(0x9B15, 0x1D);
	write_cmos_sensor(0x9B17, 0x20);
	write_cmos_sensor(0x9B25, 0x60);
	write_cmos_sensor(0x9B27, 0x60);
	write_cmos_sensor(0x9B29, 0x60);
	write_cmos_sensor(0x9B2B, 0x40);
	write_cmos_sensor(0x9B2D, 0x40);
	write_cmos_sensor(0x9B2F, 0x40);
	write_cmos_sensor(0x9B37, 0x80);
	write_cmos_sensor(0x9B39, 0x80);
	write_cmos_sensor(0x9B3B, 0x80);
	write_cmos_sensor(0x9B5D, 0x08);
	write_cmos_sensor(0x9B5E, 0x0E);
	write_cmos_sensor(0x9B60, 0x08);
	write_cmos_sensor(0x9B61, 0x0E);
	write_cmos_sensor(0x9B76, 0x0A);
	write_cmos_sensor(0x9BB0, 0x20);
	write_cmos_sensor(0x9BB1, 0x20);
	write_cmos_sensor(0x9BB2, 0x20);
	write_cmos_sensor(0x9BC6, 0x6E);
	write_cmos_sensor(0x9BC7, 0x6E);
	write_cmos_sensor(0x9BC8, 0x6E);
	write_cmos_sensor(0x9BCC, 0x20);
	write_cmos_sensor(0x9BCD, 0x20);
	write_cmos_sensor(0x9BCE, 0x20);
	write_cmos_sensor(0x9C01, 0x10);
	write_cmos_sensor(0x9C03, 0x1D);
	write_cmos_sensor(0x9C05, 0x20);
	write_cmos_sensor(0x9C13, 0x10);
	write_cmos_sensor(0x9C15, 0x10);
	write_cmos_sensor(0x9C17, 0x10);
	write_cmos_sensor(0x9C19, 0x04);
	write_cmos_sensor(0x9C1B, 0x67);
	write_cmos_sensor(0x9C1D, 0x80);
	write_cmos_sensor(0x9C1F, 0x0A);
	write_cmos_sensor(0x9C21, 0x29);
	write_cmos_sensor(0x9C23, 0x32);
	write_cmos_sensor(0x9C27, 0x56);
	write_cmos_sensor(0x9C29, 0x60);
	write_cmos_sensor(0x9C39, 0x67);
	write_cmos_sensor(0x9C3B, 0x80);
	write_cmos_sensor(0x9C3D, 0x80);
	write_cmos_sensor(0x9C3F, 0x80);
	write_cmos_sensor(0x9C41, 0x80);
	write_cmos_sensor(0x9C55, 0xC8);
	write_cmos_sensor(0x9C57, 0xC8);
	write_cmos_sensor(0x9C59, 0xC8);
	write_cmos_sensor(0x9C87, 0x48);
	write_cmos_sensor(0x9C89, 0x48);
	write_cmos_sensor(0x9C8B, 0x48);
	write_cmos_sensor(0x9CB0, 0x20);
	write_cmos_sensor(0x9CB1, 0x20);
	write_cmos_sensor(0x9CB2, 0x20);
	write_cmos_sensor(0x9CC6, 0x6E);
	write_cmos_sensor(0x9CC7, 0x6E);
	write_cmos_sensor(0x9CC8, 0x6E);
	write_cmos_sensor(0x9D13, 0x10);
	write_cmos_sensor(0x9D15, 0x10);
	write_cmos_sensor(0x9D17, 0x10);
	write_cmos_sensor(0x9D19, 0x04);
	write_cmos_sensor(0x9D1B, 0x67);
	write_cmos_sensor(0x9D1F, 0x0A);
	write_cmos_sensor(0x9D21, 0x29);
	write_cmos_sensor(0x9D23, 0x32);
	write_cmos_sensor(0x9D55, 0xC8);
	write_cmos_sensor(0x9D57, 0xC8);
	write_cmos_sensor(0x9D59, 0xC8);
	write_cmos_sensor(0x9D91, 0x20);
	write_cmos_sensor(0x9D93, 0x20);
	write_cmos_sensor(0x9D95, 0x20);
	write_cmos_sensor(0x9E01, 0x10);
	write_cmos_sensor(0x9E03, 0x1D);
	write_cmos_sensor(0x9E13, 0x10);
	write_cmos_sensor(0x9E15, 0x10);
	write_cmos_sensor(0x9E17, 0x10);
	write_cmos_sensor(0x9E19, 0x04);
	write_cmos_sensor(0x9E1B, 0x67);
	write_cmos_sensor(0x9E1D, 0x80);
	write_cmos_sensor(0x9E1F, 0x0A);
	write_cmos_sensor(0x9E21, 0x29);
	write_cmos_sensor(0x9E23, 0x32);
	write_cmos_sensor(0x9E25, 0x30);
	write_cmos_sensor(0x9E27, 0x56);
	write_cmos_sensor(0x9E29, 0x60);
	write_cmos_sensor(0x9E39, 0x67);
	write_cmos_sensor(0x9E3B, 0x80);
	write_cmos_sensor(0x9E3D, 0x80);
	write_cmos_sensor(0x9E3F, 0x80);
	write_cmos_sensor(0x9E41, 0x80);
	write_cmos_sensor(0x9E55, 0xC8);
	write_cmos_sensor(0x9E57, 0xC8);
	write_cmos_sensor(0x9E59, 0xC8);
	write_cmos_sensor(0x9E91, 0x20);
	write_cmos_sensor(0x9E93, 0x20);
	write_cmos_sensor(0x9E95, 0x20);
	write_cmos_sensor(0x9F8F, 0xA0);
	write_cmos_sensor(0xA027, 0x67);
	write_cmos_sensor(0xA029, 0x80);
	write_cmos_sensor(0xA02D, 0x67);
	write_cmos_sensor(0xA02F, 0x80);
	write_cmos_sensor(0xA031, 0x80);
	write_cmos_sensor(0xA033, 0x80);
	write_cmos_sensor(0xA035, 0x80);
	write_cmos_sensor(0xA037, 0x80);
	write_cmos_sensor(0xA039, 0x80);
	write_cmos_sensor(0xA03B, 0x80);
	write_cmos_sensor(0xA067, 0x20);
	write_cmos_sensor(0xA068, 0x20);
	write_cmos_sensor(0xA069, 0x20);
	write_cmos_sensor(0xA071, 0x48);
	write_cmos_sensor(0xA073, 0x48);
	write_cmos_sensor(0xA075, 0x48);
	write_cmos_sensor(0xA08F, 0xA0);
	write_cmos_sensor(0xA091, 0x3A);
	write_cmos_sensor(0xA093, 0x3A);
	write_cmos_sensor(0xA095, 0x0A);
	write_cmos_sensor(0xA097, 0x0A);
	write_cmos_sensor(0xA099, 0x0A);
	/* AE Setting */
	write_cmos_sensor(0x9012, 0x00);
	write_cmos_sensor(0x9098, 0x1A);
	write_cmos_sensor(0x9099, 0x04);
	write_cmos_sensor(0x909A, 0x20);
	write_cmos_sensor(0x909B, 0x20);
	write_cmos_sensor(0x909C, 0x13);
	write_cmos_sensor(0x909D, 0x13);
	write_cmos_sensor(0xA716, 0x13);
	write_cmos_sensor(0xA801, 0x08);
	write_cmos_sensor(0xA803, 0x0C);
	write_cmos_sensor(0xA805, 0x10);
	write_cmos_sensor(0xA806, 0x00);
	write_cmos_sensor(0xA807, 0x18);
	write_cmos_sensor(0xA808, 0x00);
	write_cmos_sensor(0xA809, 0x20);
	write_cmos_sensor(0xA80A, 0x00);
	write_cmos_sensor(0xA80B, 0x30);
	write_cmos_sensor(0xA80C, 0x00);
	write_cmos_sensor(0xA80D, 0x40);
	write_cmos_sensor(0xA80E, 0x00);
	write_cmos_sensor(0xA80F, 0x60);
	write_cmos_sensor(0xA810, 0x00);
	write_cmos_sensor(0xA811, 0x80);
	write_cmos_sensor(0xA812, 0x00);
	write_cmos_sensor(0xA813, 0xC0);
	write_cmos_sensor(0xA814, 0x01);
	write_cmos_sensor(0xA815, 0x00);
	write_cmos_sensor(0xA816, 0x01);
	write_cmos_sensor(0xA817, 0x80);
	write_cmos_sensor(0xA818, 0x02);
	write_cmos_sensor(0xA819, 0x00);
	write_cmos_sensor(0xA81A, 0x03);
	write_cmos_sensor(0xA81B, 0x00);
	write_cmos_sensor(0xA81C, 0x03);
	write_cmos_sensor(0xA81D, 0xAC);
	write_cmos_sensor(0xA838, 0x03);
	write_cmos_sensor(0xA83C, 0x28);
	write_cmos_sensor(0xA83D, 0x5F);
	write_cmos_sensor(0xA881, 0x08);
	write_cmos_sensor(0xA883, 0x0C);
	write_cmos_sensor(0xA885, 0x10);
	write_cmos_sensor(0xA886, 0x00);
	write_cmos_sensor(0xA887, 0x18);
	write_cmos_sensor(0xA888, 0x00);
	write_cmos_sensor(0xA889, 0x20);
	write_cmos_sensor(0xA88A, 0x00);
	write_cmos_sensor(0xA88B, 0x30);
	write_cmos_sensor(0xA88C, 0x00);
	write_cmos_sensor(0xA88D, 0x40);
	write_cmos_sensor(0xA88E, 0x00);
	write_cmos_sensor(0xA88F, 0x60);
	write_cmos_sensor(0xA890, 0x00);
	write_cmos_sensor(0xA891, 0x80);
	write_cmos_sensor(0xA892, 0x00);
	write_cmos_sensor(0xA893, 0xC0);
	write_cmos_sensor(0xA894, 0x01);
	write_cmos_sensor(0xA895, 0x00);
	write_cmos_sensor(0xA896, 0x01);
	write_cmos_sensor(0xA897, 0x80);
	write_cmos_sensor(0xA898, 0x02);
	write_cmos_sensor(0xA899, 0x00);
	write_cmos_sensor(0xA89A, 0x03);
	write_cmos_sensor(0xA89B, 0x00);
	write_cmos_sensor(0xA89C, 0x03);
	write_cmos_sensor(0xA89D, 0xAC);
	write_cmos_sensor(0xA8B8, 0x03);
	write_cmos_sensor(0xA8BB, 0x13);
	write_cmos_sensor(0xA8BC, 0x28);
	write_cmos_sensor(0xA8BD, 0x25);
	write_cmos_sensor(0xA8BE, 0x1D);
	write_cmos_sensor(0xA8C0, 0x3A);
	write_cmos_sensor(0xA8C1, 0xE0);
	write_cmos_sensor(0xB24F, 0x80);
	/* PDAF Setting */
	write_cmos_sensor(0x3198, 0x0F);
   //fix pdaf hunting issue
	write_cmos_sensor(0x31A0, 0x02);
	write_cmos_sensor(0x31A1, 0x02);
	write_cmos_sensor(0x31A2, 0x02);
	write_cmos_sensor(0x31A3, 0x03);
   //end
	write_cmos_sensor(0x31A8, 0x18);
	write_cmos_sensor(0x822C, 0x01);
	write_cmos_sensor(0x9503, 0x07);
	write_cmos_sensor(0x9504, 0x07);
	write_cmos_sensor(0x9505, 0x07);
	write_cmos_sensor(0x9506, 0x00);
	write_cmos_sensor(0x9507, 0x00);
	write_cmos_sensor(0x9508, 0x00);
	/* RMSC Setting */
	write_cmos_sensor(0x8858, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x6B42, 0x40);
	write_cmos_sensor(0x6B46, 0x00);
	write_cmos_sensor(0x6B47, 0x4B);
	write_cmos_sensor(0x6B4A, 0x00);
	write_cmos_sensor(0x6B4B, 0x4B);
	write_cmos_sensor(0x6B4E, 0x00);
	write_cmos_sensor(0x6B4F, 0x4B);
	write_cmos_sensor(0x6B44, 0x00);
	write_cmos_sensor(0x6B45, 0x8C);
	write_cmos_sensor(0x6B48, 0x00);
	write_cmos_sensor(0x6B49, 0x8C);
	write_cmos_sensor(0x6B4C, 0x00);
	write_cmos_sensor(0x6B4D, 0x8C);
    
	LOG_INF("Apply SPC data\n");
	imx230_apply_SPC();
}   /*  sensor_init  */

static void imx230_5344_4016_setting(void)
{
	/*************************
	 *4:3 ratio size setting
	 *pclk: 600 MHz
	 *line_length:6024
	 *frame_length:4130
	 *************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x10);
	write_cmos_sensor(0x0341, 0x22);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0F);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x14);
	write_cmos_sensor(0x3A01, 0xE0);
	/* Output Size*/
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x14);
	write_cmos_sensor(0x034D, 0xE0);
	write_cmos_sensor(0x034E, 0x0F);
	write_cmos_sensor(0x034F, 0xB0);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x14);
	write_cmos_sensor(0x040D, 0xE0);
	write_cmos_sensor(0x040E, 0x0F);
	write_cmos_sensor(0x040F, 0xB0);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC8);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x16);
	write_cmos_sensor(0x0821, 0xA6);
	write_cmos_sensor(0x0822, 0x66);
	write_cmos_sensor(0x0823, 0x66);
	/* Integration */
	write_cmos_sensor(0x0202, 0x10);
	write_cmos_sensor(0x0203, 0x18);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/*PDAF Setting*/ 
	write_cmos_sensor(0x697D, 0x20);
	write_cmos_sensor(0x6985, 0x14);
	write_cmos_sensor(0x698D, 0xE0);
	write_cmos_sensor(0x6995, 0x0F);
	write_cmos_sensor(0x699D, 0xB0);
	write_cmos_sensor(0x69A5, 0x00);
	write_cmos_sensor(0x69AD, 0x00);
	write_cmos_sensor(0x69B5, 0x00);
	/*DPC2D*/
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0F);
	write_cmos_sensor(0x3A26, 0xB0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x00);
	write_cmos_sensor(0x3A32, 0x00);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0F);
	write_cmos_sensor(0x3A36, 0xAF);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x00);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
	zvhdr_setting(0);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);
		
		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x06);
		write_cmos_sensor(0x3159,0x45);// X start = 1605
		write_cmos_sensor(0x315a,0x04);
		write_cmos_sensor(0x315b,0xB5);// Y start= 1205
		write_cmos_sensor(0x315c,0x0E);
		write_cmos_sensor(0x315d,0x9B);//X end = 3739
		write_cmos_sensor(0x315e,0x0A);
		write_cmos_sensor(0x315f,0xFB);// Y end = 2811	

		cur_startpos = (0x0E9B << 16) + 0x0645;
		cur_size = (0x0E9B - 0x0645 + 1) << 16 + (0x0AFB - 0x04B5 + 1);
	}

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 5344x4016 setting*/

static void imx230_5344_3008_setting(void)
{
	/************************
	 *16:9 ratio size setting
	 *pclk:600 MHz
	 *line_length:6024
	 *frame_length:3304
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x0C);
	write_cmos_sensor(0x0341, 0xE8);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xF8);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xB7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x14);
	write_cmos_sensor(0x3A01, 0xE0);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x14);
	write_cmos_sensor(0x034D, 0xE0);
	write_cmos_sensor(0x034E, 0x0B);
	write_cmos_sensor(0x034F, 0xC0);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x14);
	write_cmos_sensor(0x040D, 0xE0);
	write_cmos_sensor(0x040E, 0x0B);
	write_cmos_sensor(0x040F, 0xC0);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC8);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x16);
	write_cmos_sensor(0x0821, 0xA6);
	write_cmos_sensor(0x0822, 0x66);
	write_cmos_sensor(0x0823, 0x66);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0C);
	write_cmos_sensor(0x0203, 0xDE);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x02);
	write_cmos_sensor(0x6985, 0x02);
	write_cmos_sensor(0x698D, 0x0B);
	write_cmos_sensor(0x6995, 0x0B);
	write_cmos_sensor(0x699D, 0x16);
	write_cmos_sensor(0x69A5, 0x16);
	write_cmos_sensor(0x69AD, 0x1F);
	write_cmos_sensor(0x69B5, 0x1F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0B);
	write_cmos_sensor(0x3A26, 0xC0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x01);
	write_cmos_sensor(0x3A32, 0xF8);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0xB7);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x00);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
	zvhdr_setting(0);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);

		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x09);
		write_cmos_sensor(0x3159,0x28);// X start
		write_cmos_sensor(0x315a,0x04);
		write_cmos_sensor(0x315b,0xE2);// Y start
		write_cmos_sensor(0x315c,0x0C);
		write_cmos_sensor(0x315d,0xE7);//X end 
		write_cmos_sensor(0x315e,0x07);
		write_cmos_sensor(0x315f,0xCF);// Y end 

		cur_startpos = (0x0928 << 16) + 0x04E2;
		cur_size = (0x0CE7 - 0x0928 + 1) << 16 + (0x07CF - 0x04E2 + 1);
	}

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 5344x3008 setting*/

static void imx230_4K2K_setting(void)
{
#if 0
	/************************
	 *4272x2404 size setting
	 *pclk:600Mhz
	 *line_length:6024
	 *frame_length:3304
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x0C);
	write_cmos_sensor(0x0341, 0xE8);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xF8);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xB7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x10);
	write_cmos_sensor(0x3A01, 0xB0);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x10);
	write_cmos_sensor(0x034D, 0xB0);
	write_cmos_sensor(0x034E, 0x09);
	write_cmos_sensor(0x034F, 0x64);
	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x14);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x02);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x02);
	write_cmos_sensor(0x040C, 0x14);
	write_cmos_sensor(0x040D, 0xDE);
	write_cmos_sensor(0x040E, 0x0B);
	write_cmos_sensor(0x040F, 0xBE);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC8);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x16);
	write_cmos_sensor(0x0821, 0xA6);
	write_cmos_sensor(0x0822, 0x66);
	write_cmos_sensor(0x0823, 0x66);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0C);
	write_cmos_sensor(0x0203, 0xDE);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x02);
	write_cmos_sensor(0x6985, 0x02);
	write_cmos_sensor(0x698D, 0x0B);
	write_cmos_sensor(0x6995, 0x0B);
	write_cmos_sensor(0x699D, 0x16);
	write_cmos_sensor(0x69A5, 0x16);
	write_cmos_sensor(0x69AD, 0x1F);
	write_cmos_sensor(0x69B5, 0x1F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0B);
	write_cmos_sensor(0x3A26, 0xC0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x01);
	write_cmos_sensor(0x3A32, 0xF8);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0xB7);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x00);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
#else
	/************************
	 *3856x2170 size setting
	 *pclk:411Mhz
	 *line_length:6024
	 *frame_length:2274
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0222,0x01);
	write_cmos_sensor(0x0340,0x08);
	write_cmos_sensor(0x0341,0xE2);
	write_cmos_sensor(0x0342,0x17);
	write_cmos_sensor(0x0343,0x88);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x03);
	write_cmos_sensor(0x0347,0x9C);
	write_cmos_sensor(0x0348,0x14);
	write_cmos_sensor(0x0349,0xDF);
	write_cmos_sensor(0x034A,0x0C);
	write_cmos_sensor(0x034B,0x15);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x00);
	write_cmos_sensor(0x0901,0x11);
	write_cmos_sensor(0x0902,0x00);
	write_cmos_sensor(0x3000,0x74);
	write_cmos_sensor(0x3001,0x00);
	write_cmos_sensor(0x305C,0x11);
	
	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x034C,0x0F);
	write_cmos_sensor(0x034D,0x10);
	write_cmos_sensor(0x034E,0x08);
	write_cmos_sensor(0x034F,0x7A);
	write_cmos_sensor(0x0401,0x00);
	write_cmos_sensor(0x0404,0x00);
	write_cmos_sensor(0x0405,0x10);
	write_cmos_sensor(0x0408,0x02);
	write_cmos_sensor(0x0409,0xE8);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x0F);
	write_cmos_sensor(0x040D,0x10);
	write_cmos_sensor(0x040E,0x08);
	write_cmos_sensor(0x040F,0x7A);
	
	write_cmos_sensor(0x0301,0x04);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x89);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x0E);
	write_cmos_sensor(0x030E,0x01);
	write_cmos_sensor(0x030F,0xA4);
	write_cmos_sensor(0x0310,0x01);
	
	write_cmos_sensor(0x0820,0x0B);
	write_cmos_sensor(0x0821,0x40);
	write_cmos_sensor(0x0822,0x00);
	write_cmos_sensor(0x0823,0x00);
	
	write_cmos_sensor(0x0202,0x08);
	write_cmos_sensor(0x0203,0xD8);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);
	
	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0210,0x01);
	write_cmos_sensor(0x0211,0x00);
	write_cmos_sensor(0x0212,0x01);
	write_cmos_sensor(0x0213,0x00);
	write_cmos_sensor(0x0214,0x01);
	write_cmos_sensor(0x0215,0x00);
	
	write_cmos_sensor(0x3006,0x01);
	write_cmos_sensor(0x3007,0x02);
	write_cmos_sensor(0x31E0,0x03);
	write_cmos_sensor(0x31E1,0xFF);
	write_cmos_sensor(0x31E4,0x02);
	
	write_cmos_sensor(0x3A22,0x20);
	write_cmos_sensor(0x3A23,0x14);
	write_cmos_sensor(0x3A24,0xE0);
	write_cmos_sensor(0x3A25,0x08);
	write_cmos_sensor(0x3A26,0x7A);
	write_cmos_sensor(0x3A2F,0x00);
	write_cmos_sensor(0x3A30,0x00);
	write_cmos_sensor(0x3A31,0x03);
	write_cmos_sensor(0x3A32,0x9C);
	write_cmos_sensor(0x3A33,0x14);
	write_cmos_sensor(0x3A34,0xDF);
	write_cmos_sensor(0x3A35,0x0C);
	write_cmos_sensor(0x3A36,0x15);
	write_cmos_sensor(0x3A37,0x00);
	write_cmos_sensor(0x3A38,0x00);
	write_cmos_sensor(0x3A39,0x00);
	
	write_cmos_sensor(0x3A21,0x00);
	
	write_cmos_sensor(0x3011,0x00);
	write_cmos_sensor(0x3013,0x01);
	
	zvhdr_setting(0);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);

		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x06);
		write_cmos_sensor(0x3159,0x97);// X start
		write_cmos_sensor(0x315a,0x03);
		write_cmos_sensor(0x315b,0x89);// Y start
		write_cmos_sensor(0x315c,0x09);
		write_cmos_sensor(0x315d,0x69);//X end 
		write_cmos_sensor(0x315e,0x05);
		write_cmos_sensor(0x315f,0xA5);// Y end 

		cur_startpos = (0x0697 << 16) + 0x0389;
		cur_size = (0x0969 - 0x0697 + 1) << 16 + (0x05A5 - 0x0389 + 1);
	}

#endif
	write_cmos_sensor(0x0100,0x01);
}	/* imx230 4272x2404 setting*/

static void imx230_2672_2008_setting(void)
{
	/************************
	 *4:3(full\2)ratio setting
	 *pclk:531 Mhz
	 *line_length:6024
	 *frame_length:2880
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);
	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x40);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0F);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x0A);
	write_cmos_sensor(0x3A01, 0x70);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x0A);
	write_cmos_sensor(0x034D, 0x70);
	write_cmos_sensor(0x034E, 0x07);
	write_cmos_sensor(0x034F, 0xD8);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x0A);
	write_cmos_sensor(0x040D, 0x70);
	write_cmos_sensor(0x040E, 0x07);
	write_cmos_sensor(0x040F, 0xD8);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB1);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x01);
	write_cmos_sensor(0x030F, 0xEF);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x0C);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x36);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x3F);
	write_cmos_sensor(0x6985, 0x3F);
	write_cmos_sensor(0x698D, 0x3F);
	write_cmos_sensor(0x6995, 0x3F);
	write_cmos_sensor(0x699D, 0x3F);
	write_cmos_sensor(0x69A5, 0x3F);
	write_cmos_sensor(0x69AD, 0x3F);
	write_cmos_sensor(0x69B5, 0x3F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x07);
	write_cmos_sensor(0x3A26, 0xD8);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x00);
	write_cmos_sensor(0x3A32, 0x00);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0F);
	write_cmos_sensor(0x3A36, 0xAF);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x01);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x00);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
	zvhdr_setting(0);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);

		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x03);
		write_cmos_sensor(0x3159,0x22);// X start = 802
		write_cmos_sensor(0x315a,0x02);
		write_cmos_sensor(0x315b,0x5B);// Y start= 603
		write_cmos_sensor(0x315c,0x07);
		write_cmos_sensor(0x315d,0x4E);//X end = 1870
		write_cmos_sensor(0x315e,0x05);
		write_cmos_sensor(0x315f,0x7D);// Y end = 1405

		cur_startpos = (0x0322 << 16) + 0x025B;
		cur_size = (0x074E - 0x0322 + 1) << 16 + (0x057D - 0x025B + 1);
		
	}

	write_cmos_sensor(0x0100,0x01);
}	/* 2672x2008 setting */

static void imx230_1080P_setting(void)
{
	/************************
	 *2136x1202 raw size setting
	 *pclk:531 MHz
	 *line_length:6024
	 *frame_length:2880
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x40);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xF8);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xB7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x22);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x08);
	write_cmos_sensor(0x3A01, 0x58);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x08);
	write_cmos_sensor(0x034D, 0x58);
	write_cmos_sensor(0x034E, 0x04);
	write_cmos_sensor(0x034F, 0xB2);
	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x14);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x0A);
	write_cmos_sensor(0x040D, 0x70);
	write_cmos_sensor(0x040E, 0x05);
	write_cmos_sensor(0x040F, 0xE0);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB1);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x0B);
	write_cmos_sensor(0x0821, 0x53);
	write_cmos_sensor(0x0822, 0x33);
	write_cmos_sensor(0x0823, 0x33);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x36);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x3F);
	write_cmos_sensor(0x6985, 0x3F);
	write_cmos_sensor(0x698D, 0x3F);
	write_cmos_sensor(0x6995, 0x3F);
	write_cmos_sensor(0x699D, 0x3F);
	write_cmos_sensor(0x69A5, 0x3F);
	write_cmos_sensor(0x69AD, 0x3F);
	write_cmos_sensor(0x69B5, 0x3F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x05);
	write_cmos_sensor(0x3A26, 0xE0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x01);
	write_cmos_sensor(0x3A32, 0xF8);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0xB7);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x01);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x00);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 1080P setting*/

static void imx230_1336_1004_setting(void)
{
	/************************
	 *1\4 binning size setting
	 *pclk:531 MHz
	 *line_length:6024
	 *frame_length:2880
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x40);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0F);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x44);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x05);
	write_cmos_sensor(0x3A01, 0x38);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0x38);
	write_cmos_sensor(0x034E, 0x03);
	write_cmos_sensor(0x034F, 0xEC);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x05);
	write_cmos_sensor(0x040D, 0x38);
	write_cmos_sensor(0x040E, 0x03);
	write_cmos_sensor(0x040F, 0xEC);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB1);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x0B);
	write_cmos_sensor(0x0821, 0x53);
	write_cmos_sensor(0x0822, 0x33);
	write_cmos_sensor(0x0823, 0x33);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x36);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x3F);
	write_cmos_sensor(0x6985, 0x3F);
	write_cmos_sensor(0x698D, 0x3F);
	write_cmos_sensor(0x6995, 0x3F);
	write_cmos_sensor(0x699D, 0x3F);
	write_cmos_sensor(0x69A5, 0x3F);
	write_cmos_sensor(0x69AD, 0x3F);
	write_cmos_sensor(0x69B5, 0x3F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x03);
	write_cmos_sensor(0x3A26, 0xEC);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x00);
	write_cmos_sensor(0x3A32, 0x00);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0F);
	write_cmos_sensor(0x3A36, 0xAF);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x02);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x00);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 1336x1004 setting*/

static void imx230_5344_4016_HDR_setting(void)
{
	/*************************
	 *4:3 ratio size setting
	 *pclk: 600 MHz
	 *line_length:6024
	 *frame_length:4130
	 *************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x23);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x10);
	write_cmos_sensor(0x0341, 0x22);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0F);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x75);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x14);
	write_cmos_sensor(0x3A01, 0xE0);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x14);
	write_cmos_sensor(0x034D, 0xE0);
	write_cmos_sensor(0x034E, 0x0F);
	write_cmos_sensor(0x034F, 0xB0);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x14);
	write_cmos_sensor(0x040D, 0xE0);
	write_cmos_sensor(0x040E, 0x0F);
	write_cmos_sensor(0x040F, 0xB0);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC8);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x16);
	write_cmos_sensor(0x0821, 0xA6);
	write_cmos_sensor(0x0822, 0x66);
	write_cmos_sensor(0x0823, 0x66);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x10);
	write_cmos_sensor(0x0203, 0x18);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0x01);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x01);
	write_cmos_sensor(0x31E0, 0x3F);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x00);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x02);
	write_cmos_sensor(0x6985, 0x02);
	write_cmos_sensor(0x698D, 0x0B);
	write_cmos_sensor(0x6995, 0x0B);
	write_cmos_sensor(0x699D, 0x16);
	write_cmos_sensor(0x69A5, 0x16);
	write_cmos_sensor(0x69AD, 0x1F);
	write_cmos_sensor(0x69B5, 0x1F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x00);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0F);
	write_cmos_sensor(0x3A26, 0xB0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x00);
	write_cmos_sensor(0x3A32, 0x00);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0F);
	write_cmos_sensor(0x3A36, 0xAF);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */         
	write_cmos_sensor(0x3A21, 0x02);
	/*Stats Setting*/         
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
	zvhdr_setting(1);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);
		
		//write_cmos_sensor(0x0100,0x01);
		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x06);
		write_cmos_sensor(0x3159,0x45);// X start = 1605
		write_cmos_sensor(0x315a,0x04);
		write_cmos_sensor(0x315b,0xB5);// Y start= 1205
		write_cmos_sensor(0x315c,0x0E);
		write_cmos_sensor(0x315d,0x9B);//X end = 3739
		write_cmos_sensor(0x315e,0x0A);
		write_cmos_sensor(0x315f,0xFB);// Y end = 2811	

		cur_startpos = (0x0E9B << 16) + 0x0645;
		cur_size = (0x0E9B - 0x0645 + 1) << 16 + (0x0AFB - 0x04B5 + 1);
	}

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 5344x4016 HDR setting*/

static void imx230_5344_3008_HDR_setting(void)
{
	/************************
	 *16:9 ratio size setting
	 *pclk:600 MHz
	 *line_length:6024
	 *frame_length:3304
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x23);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x0C);
	write_cmos_sensor(0x0341, 0xE8);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xF8);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xB7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x75);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x14);
	write_cmos_sensor(0x3A01, 0xE0);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x14);
	write_cmos_sensor(0x034D, 0xE0);
	write_cmos_sensor(0x034E, 0x0B);
	write_cmos_sensor(0x034F, 0xC0);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x14);
	write_cmos_sensor(0x040D, 0xE0);
	write_cmos_sensor(0x040E, 0x0B);
	write_cmos_sensor(0x040F, 0xC0);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC8);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x16);
	write_cmos_sensor(0x0821, 0xA6);
	write_cmos_sensor(0x0822, 0x66);
	write_cmos_sensor(0x0823, 0x66);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0C);
	write_cmos_sensor(0x0203, 0xDE);
	write_cmos_sensor(0x0224, 0x00);
	write_cmos_sensor(0x0225, 0xCD);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x01);
	write_cmos_sensor(0x31E0, 0x3F);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x00);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x02);
	write_cmos_sensor(0x6985, 0x02);
	write_cmos_sensor(0x698D, 0x0B);
	write_cmos_sensor(0x6995, 0x0B);
	write_cmos_sensor(0x699D, 0x16);
	write_cmos_sensor(0x69A5, 0x16);
	write_cmos_sensor(0x69AD, 0x1F);
	write_cmos_sensor(0x69B5, 0x1F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x00);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0B);
	write_cmos_sensor(0x3A26, 0xC0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x01);
	write_cmos_sensor(0x3A32, 0xF8);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0xB7);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */         
	write_cmos_sensor(0x3A21, 0x02);
	/*Stats Setting*/         
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
	zvhdr_setting(1);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);

		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x09);
		write_cmos_sensor(0x3159,0x28);// X start
		write_cmos_sensor(0x315a,0x04);
		write_cmos_sensor(0x315b,0xE2);// Y start
		write_cmos_sensor(0x315c,0x0C);
		write_cmos_sensor(0x315d,0xE7);//X end 
		write_cmos_sensor(0x315e,0x07);
		write_cmos_sensor(0x315f,0xCF);// Y end 

		cur_startpos = (0x0928 << 16) + 0x04E2;
		cur_size = (0x0CE7 - 0x0928 + 1) << 16 + (0x07CF - 0x04E2 + 1);
	}

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 5344x3008 HDR setting*/

static void imx230_4K2K_HDR_setting(void)
{
	/************************
	 *4272x2404 HDR mode setting
	 *pclk:600 M
	 *line_length:6024
	 *frame_length:3304
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x23);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x0C);
	write_cmos_sensor(0x0341, 0xE8);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xF8);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xB7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x75);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x10);
	write_cmos_sensor(0x3A01, 0xB0);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x10);
	write_cmos_sensor(0x034D, 0xB0);
	write_cmos_sensor(0x034E, 0x09);
	write_cmos_sensor(0x034F, 0x64);
	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x14);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x02);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x02);
	write_cmos_sensor(0x040C, 0x14);
	write_cmos_sensor(0x040D, 0xDE);
	write_cmos_sensor(0x040E, 0x0B);
	write_cmos_sensor(0x040F, 0xBE);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC8);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x16);
	write_cmos_sensor(0x0821, 0xA6);
	write_cmos_sensor(0x0822, 0x66);
	write_cmos_sensor(0x0823, 0x66);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0C);
	write_cmos_sensor(0x0203, 0xDE);
	write_cmos_sensor(0x0224, 0x00);
	write_cmos_sensor(0x0225, 0xCD);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x01);
	write_cmos_sensor(0x31E0, 0x3F);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x00);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x02);
	write_cmos_sensor(0x6985, 0x02);
	write_cmos_sensor(0x698D, 0x0B);
	write_cmos_sensor(0x6995, 0x0B);
	write_cmos_sensor(0x699D, 0x16);
	write_cmos_sensor(0x69A5, 0x16);
	write_cmos_sensor(0x69AD, 0x1F);
	write_cmos_sensor(0x69B5, 0x1F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x00);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0B);
	write_cmos_sensor(0x3A26, 0xC0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x01);
	write_cmos_sensor(0x3A32, 0xF8);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0xB7);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */         
	write_cmos_sensor(0x3A21, 0x02);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
	zvhdr_setting(1);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);

		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x09);
		write_cmos_sensor(0x3159,0x28);// X start
		write_cmos_sensor(0x315a,0x04);
		write_cmos_sensor(0x315b,0xE2);// Y start
		write_cmos_sensor(0x315c,0x0C);
		write_cmos_sensor(0x315d,0xE7);//X end 
		write_cmos_sensor(0x315e,0x07);
		write_cmos_sensor(0x315f,0xCF);// Y end 

		cur_startpos = (0x0928 << 16) + 0x04E2;
		cur_size = (0x0CE7 - 0x0928 + 1) << 16 + (0x07CF - 0x04E2 + 1);
	}
	zvhdr_setting(1);
	write_cmos_sensor(0x0100,0x01);
}	/* imx230 4272x2404 HDR setting*/

static void imx230_2672_2008_HDR_setting(void)
{
	/************************
	 *2672x2008 HDR mode setting
	 *line_length:6024
	 *frame_length:2880
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x23);
	write_cmos_sensor(0x0221, 0x22);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x40);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0F);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x75);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x0A);
	write_cmos_sensor(0x3A01, 0x70);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x0A);
	write_cmos_sensor(0x034D, 0x70);
	write_cmos_sensor(0x034E, 0x07);
	write_cmos_sensor(0x034F, 0xD8);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x0A);
	write_cmos_sensor(0x040D, 0x70);
	write_cmos_sensor(0x040E, 0x07);
	write_cmos_sensor(0x040F, 0xD8);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB1);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x01);
	write_cmos_sensor(0x030F, 0xEF);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x0C);
	write_cmos_sensor(0x0821, 0x60);
	write_cmos_sensor(0x0822, 0x00);
	write_cmos_sensor(0x0823, 0x00);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x36);
	write_cmos_sensor(0x0224, 0x00);
	write_cmos_sensor(0x0225, 0xB3);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x01);
	write_cmos_sensor(0x31E0, 0x3F);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x00);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x3F);
	write_cmos_sensor(0x6985, 0x3F);
	write_cmos_sensor(0x698D, 0x3F);
	write_cmos_sensor(0x6995, 0x3F);
	write_cmos_sensor(0x699D, 0x3F);
	write_cmos_sensor(0x69A5, 0x3F);
	write_cmos_sensor(0x69AD, 0x3F);
	write_cmos_sensor(0x69B5, 0x3F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x00);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0F);
	write_cmos_sensor(0x3A26, 0xB0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x00);
	write_cmos_sensor(0x3A32, 0x00);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0F);
	write_cmos_sensor(0x3A36, 0xAF);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x10);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x02);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);

	zvhdr_setting(1);
	if(imgsensor.pdaf_mode == 1)
	{
		/*PDAF*/
		/*PD_CAL_ENALBE*/
		write_cmos_sensor(0x3121,0x01);
		/*AREA MODE*/
		write_cmos_sensor(0x31B0,0x02);// 8x6 output
		write_cmos_sensor(0x31B4,0x01);// 8x6 output
		/*PD_OUT_EN=1*/
		write_cmos_sensor(0x3123,0x01);

		/*Fixed area mode*/
		
		write_cmos_sensor(0x3158,0x03);
		write_cmos_sensor(0x3159,0x22);// X start = 802
		write_cmos_sensor(0x315a,0x02);
		write_cmos_sensor(0x315b,0x5B);// Y start= 603
		write_cmos_sensor(0x315c,0x07);
		write_cmos_sensor(0x315d,0x4E);//X end = 1870
		write_cmos_sensor(0x315e,0x05);
		write_cmos_sensor(0x315f,0x7D);// Y end = 1405

		cur_startpos = (0x0322 << 16) + 0x025B;
		cur_size = (0x074E - 0x0322 + 1) << 16 + (0x057D - 0x025B + 1);
		
	}
	write_cmos_sensor(0x0100,0x01);
}	/* imx230 2672x2008 HDR setting*/

static void imx230_1080P_HDR_setting(void)
{
	/************************
	 *2136x1202 HDR mode setting
	 *line_length:6024
	 *frame_length:2880
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x23);
	write_cmos_sensor(0x0221, 0x22);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x40);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xF8);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xB7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x75);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x08);
	write_cmos_sensor(0x3A01, 0x58);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x08);
	write_cmos_sensor(0x034D, 0x58);
	write_cmos_sensor(0x034E, 0x04);
	write_cmos_sensor(0x034F, 0xB2);
	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x14);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x0A);
	write_cmos_sensor(0x040D, 0x70);
	write_cmos_sensor(0x040E, 0x05);
	write_cmos_sensor(0x040F, 0xE0);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB1);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x0B);
	write_cmos_sensor(0x0821, 0x53);
	write_cmos_sensor(0x0822, 0x33);
	write_cmos_sensor(0x0823, 0x33);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x36);
	write_cmos_sensor(0x0224, 0x00);
	write_cmos_sensor(0x0225, 0xB3);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x01);
	write_cmos_sensor(0x31E0, 0x3F);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x00);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x3F);
	write_cmos_sensor(0x6985, 0x3F);
	write_cmos_sensor(0x698D, 0x3F);
	write_cmos_sensor(0x6995, 0x3F);
	write_cmos_sensor(0x699D, 0x3F);
	write_cmos_sensor(0x69A5, 0x3F);
	write_cmos_sensor(0x69AD, 0x3F);
	write_cmos_sensor(0x69B5, 0x3F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x00);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0B);
	write_cmos_sensor(0x3A26, 0xC0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x01);
	write_cmos_sensor(0x3A32, 0xF8);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0xB7);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x10);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x02);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 1080P HDR setting*/

static void imx230_720P_HDR_setting(void)
{
	/************************
	 *1332x750 HDR mode setting
	 *line_length:6024
	 *frame_length:2880
	*************************/
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);

	/* Mode Setting*/
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x23);
	write_cmos_sensor(0x0221, 0x22);
	write_cmos_sensor(0x0222, 0x10);
	write_cmos_sensor(0x0340, 0x0B);
	write_cmos_sensor(0x0341, 0x40);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0xF8);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0xB7);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x75);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	write_cmos_sensor(0x3A00, 0x05);
	write_cmos_sensor(0x3A01, 0x34);
	/* Output Size*/          
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0x34);
	write_cmos_sensor(0x034E, 0x02);
	write_cmos_sensor(0x034F, 0xEE);
	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x20);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x04);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x02);
	write_cmos_sensor(0x040C, 0x0A);
	write_cmos_sensor(0x040D, 0x6A);
	write_cmos_sensor(0x040E, 0x05);
	write_cmos_sensor(0x040F, 0xDE);
	/* Clock Setting*/	      
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xB1);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0F);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x8A);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate */           
	write_cmos_sensor(0x0820, 0x0B);
	write_cmos_sensor(0x0821, 0x53);
	write_cmos_sensor(0x0822, 0x33);
	write_cmos_sensor(0x0823, 0x33);
	/* Integration */         
	write_cmos_sensor(0x0202, 0x0B);
	write_cmos_sensor(0x0203, 0x36);
	write_cmos_sensor(0x0224, 0x00);
	write_cmos_sensor(0x0225, 0xB3);
	/* Gain Setting*/         
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x01);
	write_cmos_sensor(0x31E0, 0x3F);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x00);
	/*PDAF Setting*/          
	write_cmos_sensor(0x697D, 0x3F);
	write_cmos_sensor(0x6985, 0x3F);
	write_cmos_sensor(0x698D, 0x3F);
	write_cmos_sensor(0x6995, 0x3F);
	write_cmos_sensor(0x699D, 0x3F);
	write_cmos_sensor(0x69A5, 0x3F);
	write_cmos_sensor(0x69AD, 0x3F);
	write_cmos_sensor(0x69B5, 0x3F);
	/*DPC2D*/                 
	write_cmos_sensor(0x3A22, 0x00);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0B);
	write_cmos_sensor(0x3A26, 0xC0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x01);
	write_cmos_sensor(0x3A32, 0xF8);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0xB7);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x10);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x02);
	/*Stats Setting*/
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);

	write_cmos_sensor(0x0100,0x01);
}	/* imx230 720P HDR setting*/

static void imx230_720p_hs_setting(void)
{
	LOG_INF("%s\n", __func__);
	write_cmos_sensor(0x0100,0x00);	
	/* Mode Setting */
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x03);
	write_cmos_sensor(0x0341, 0x34);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x02);
	write_cmos_sensor(0x0347, 0x28);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0D);
	write_cmos_sensor(0x034B, 0x8F);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x44);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	/* Output Size Setting */ 
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0x10);
	write_cmos_sensor(0x034E, 0x02);
	write_cmos_sensor(0x034F, 0xDA);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x14);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x05);
	write_cmos_sensor(0x040D, 0x10);
	write_cmos_sensor(0x040E, 0x02);
	write_cmos_sensor(0x040F, 0xDA);
	/* Clock Setting */       
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC6);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x02);
	write_cmos_sensor(0x030D, 0x0E);
	write_cmos_sensor(0x030E, 0x01);
	write_cmos_sensor(0x030F, 0xD2);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate Setting */   
	write_cmos_sensor(0x0820, 0x06);
	write_cmos_sensor(0x0821, 0x3D);
	write_cmos_sensor(0x0822, 0xB6);
	write_cmos_sensor(0x0823, 0xDB);
	/* Integration Time Settin*/
	write_cmos_sensor(0x0202, 0x03);
	write_cmos_sensor(0x0203, 0x2A);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting */        
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */         
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/* DPC2D Setting */       
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x02);
	write_cmos_sensor(0x3A26, 0xDA);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x02);
	write_cmos_sensor(0x3A32, 0x28);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0D);
	write_cmos_sensor(0x3A36, 0x8F);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x02);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */         
	write_cmos_sensor(0x3A21, 0x00);
	/* Stats Setting */       
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);
	zvhdr_setting(0);
	write_cmos_sensor(0x0100,0x01);	
}	/* 720p high speed video setting */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s\n", __func__);
	/*capture
	 * H : 
	 * V : 
	 */
	write_cmos_sensor(0x0100,0x00);	
	/* Mode Setting */
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0220, 0x00);
	write_cmos_sensor(0x0221, 0x11);
	write_cmos_sensor(0x0222, 0x01);
	write_cmos_sensor(0x0340, 0x10);
	write_cmos_sensor(0x0341, 0x0C);
	write_cmos_sensor(0x0342, 0x17);
	write_cmos_sensor(0x0343, 0x88);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x14);
	write_cmos_sensor(0x0349, 0xDF);
	write_cmos_sensor(0x034A, 0x0F);
	write_cmos_sensor(0x034B, 0xAF);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0902, 0x00);
	write_cmos_sensor(0x3000, 0x74);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x305C, 0x11);
	/* Output Size Setting */
	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x034C, 0x14);
	write_cmos_sensor(0x034D, 0xE0);
	write_cmos_sensor(0x034E, 0x0F);
	write_cmos_sensor(0x034F, 0xB0);
	write_cmos_sensor(0x0401, 0x00);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x10);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x00);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x14);
	write_cmos_sensor(0x040D, 0xE0);
	write_cmos_sensor(0x040E, 0x0F);
	write_cmos_sensor(0x040F, 0xB0);
	/* Clock Setting */
	write_cmos_sensor(0x0301, 0x04);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x04);
	write_cmos_sensor(0x0306, 0x00);
	write_cmos_sensor(0x0307, 0xC6);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0E);
	write_cmos_sensor(0x030E, 0x03);
	write_cmos_sensor(0x030F, 0x30);
	write_cmos_sensor(0x0310, 0x01);
	/* Data Rate Setting */
	write_cmos_sensor(0x0820, 0x15);
	write_cmos_sensor(0x0821, 0xDB);
	write_cmos_sensor(0x0822, 0x6D);
	write_cmos_sensor(0x0823, 0xB6);
	/* Integration Time Setting */
	write_cmos_sensor(0x0202, 0x10);
	write_cmos_sensor(0x0203, 0x02);
	write_cmos_sensor(0x0224, 0x01);
	write_cmos_sensor(0x0225, 0xF4);
	/* Gain Setting */
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x0216, 0x00);
	write_cmos_sensor(0x0217, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0210, 0x01);
	write_cmos_sensor(0x0211, 0x00);
	write_cmos_sensor(0x0212, 0x01);
	write_cmos_sensor(0x0213, 0x00);
	write_cmos_sensor(0x0214, 0x01);
	write_cmos_sensor(0x0215, 0x00);
	/* HDR Setting */
	write_cmos_sensor(0x3006, 0x01);
	write_cmos_sensor(0x3007, 0x02);
	write_cmos_sensor(0x31E0, 0x03);
	write_cmos_sensor(0x31E1, 0xFF);
	write_cmos_sensor(0x31E4, 0x02);
	/* DPC2D Setting */
	write_cmos_sensor(0x3A22, 0x20);
	write_cmos_sensor(0x3A23, 0x14);
	write_cmos_sensor(0x3A24, 0xE0);
	write_cmos_sensor(0x3A25, 0x0F);
	write_cmos_sensor(0x3A26, 0xB0);
	write_cmos_sensor(0x3A2F, 0x00);
	write_cmos_sensor(0x3A30, 0x00);
	write_cmos_sensor(0x3A31, 0x00);
	write_cmos_sensor(0x3A32, 0x00);
	write_cmos_sensor(0x3A33, 0x14);
	write_cmos_sensor(0x3A34, 0xDF);
	write_cmos_sensor(0x3A35, 0x0F);
	write_cmos_sensor(0x3A36, 0xAF);
	write_cmos_sensor(0x3A37, 0x00);
	write_cmos_sensor(0x3A38, 0x00);
	write_cmos_sensor(0x3A39, 0x00);
	/* LSC Setting */
	write_cmos_sensor(0x3A21, 0x00);
	/* Stats Setting */
	write_cmos_sensor(0x3011, 0x00);
	write_cmos_sensor(0x3013, 0x01);

	write_cmos_sensor(0x0100,0x01);	
}	/* capture_setting */


static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        write_cmos_sensor(0x0601, 0x02);
    } else {
        write_cmos_sensor(0x0601, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	u8 module_id = 0;
	kal_uint8 i2c_write_id;
	int read_sensor_id;

    //sensor have two i2c address 0x6c 0x6d & 0x35 0x34, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	i2c_write_id = imgsensor.i2c_write_id;
	imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	spin_unlock(&imgsensor_drv_lock);
        do {
	    read_sensor_id = return_sensor_id();
	    if (read_sensor_id == 0x230) {
		imx230_read_otp_byte(0x0001,&module_id);
		if(0x02 == module_id){
			main_module_id = module_id;
			printk("[IMX230_SHARP]Get sharp module success!! write id: 0x%X,module id: 0x%X\n", imgsensor.i2c_write_id, module_id);
			*sensor_id = imgsensor_info.sensor_id;
                	goto otp_read;
            	}
            }
            retry--;
        } while(retry > 0);
	printk("Get sharp module fail!! write id: 0x%x, sensor_id:0x%x, module_id:0x%x\n", imgsensor.i2c_write_id,read_sensor_id, module_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = i2c_write_id;
	spin_unlock(&imgsensor_drv_lock);
        i++;
        retry = 2;
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
	sharp_otp_buf = (u8 *)kzalloc(IMX230_OTP_SIZE, GFP_KERNEL);
	/* read lsc calibration from E2PROM */
	imx230_read_otp_all(OTP_START_ADDR, sharp_otp_buf);

	/*Get AF infinity and macro position value*/
	AF_Inf_pos   = (sharp_otp_buf[64]<<8)|sharp_otp_buf[65];
	AF_Macro_pos = (sharp_otp_buf[68]<<8)|sharp_otp_buf[69];

	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == 0x230) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == 0x230)
            break;
        retry = 2;
    }
    if (0x230 != sensor_id)
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
    imgsensor.ihdr_mode = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    if(imgsensor.ihdr_mode == 9)
		imx230_2672_2008_HDR_setting();
	else
		imx230_2672_2008_setting();

    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
	if(imgsensor.ihdr_mode == 9)
		imx230_5344_4016_HDR_setting();
	else
		imx230_5344_4016_setting();

    return ERROR_NONE;
}    /* capture() */

static kal_uint32 zsd_capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_ZSD_CAPTURE;
	imgsensor.pclk = imgsensor_info.zsd.pclk;
	imgsensor.line_length = imgsensor_info.zsd.linelength;
	imgsensor.frame_length = imgsensor_info.zsd.framelength;  
	imgsensor.min_frame_length = imgsensor_info.zsd.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	if(imgsensor.ihdr_mode == 9)
		imx230_5344_3008_HDR_setting();
	else
		imx230_5344_3008_setting();	

	return ERROR_NONE;
}	/* zsd capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	if(imgsensor.ihdr_mode == 9)
		imx230_2672_2008_HDR_setting();
	else
		imx230_2672_2008_setting();
	
    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    imx230_720p_hs_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 hd_4k_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_4K_VIDEO;
	imgsensor.pclk = imgsensor_info.hd_4k_video.pclk;
	imgsensor.line_length = imgsensor_info.hd_4k_video.linelength;
	imgsensor.frame_length = imgsensor_info.hd_4k_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.hd_4k_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	if(imgsensor.ihdr_mode == 9)
		imx230_4K2K_HDR_setting();
	else
		imx230_4K2K_setting();

	return ERROR_NONE;
}	/* 4k video */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    imx230_1336_1004_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);

    return ERROR_NONE;
}    /*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

    sensor_resolution->SensorCustom1Width = imgsensor_info.zsd.grabwindow_width;
    sensor_resolution->SensorCustom1Height = imgsensor_info.zsd.grabwindow_height;

    sensor_resolution->SensorCustom2Width = imgsensor_info.hd_4k_video.grabwindow_width;
    sensor_resolution->SensorCustom2Height = imgsensor_info.hd_4k_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */

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
    sensor_info->Custom1DelayFrame = imgsensor_info.zsd_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.hd_4k_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->PDAF_Support = 2; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/
    sensor_info->HDR_Support = 3; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
    sensor_info->ZHDR_Mode = 8;

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
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

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
            sensor_info->SensorGrabStartX = imgsensor_info.zsd.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.zsd.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.zsd.mipi_data_lp2hs_settle_dc;

            break;
	case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.hd_4k_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hd_4k_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hd_4k_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


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
            zsd_capture(image_window, sensor_config_data);
            break;
	case MSDK_SCENARIO_ID_CUSTOM2:
            hd_4k_video(image_window, sensor_config_data);
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{//This Function not used after ROME
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
            //set_dummy();
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
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
	case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.zsd.pclk / framerate * 10 / imgsensor_info.zsd.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.zsd.framelength) ? (frame_length - imgsensor_info.zsd.framelength): 0;
            imgsensor.frame_length = imgsensor_info.zsd.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
	case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.hd_4k_video.pclk / framerate * 10 / imgsensor_info.hd_4k_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hd_4k_video.framelength) ? (frame_length - imgsensor_info.hd_4k_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.hd_4k_video.framelength + imgsensor.dummy_line;
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
            //set_dummy();
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
            *framerate = imgsensor_info.zsd.max_framerate;
            break;
	case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.hd_4k_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}


static kal_uint32 imx230_awb_gain(SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
    UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;
	  LOG_INF("imx230_awb_gain\n");
    
    grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
    rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
    bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
    gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

    LOG_INF("[imx230_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GR, grgain_32);
    LOG_INF("[imx230_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n", pSetSensorAWB->ABS_GAIN_R, rgain_32);
    LOG_INF("[imx230_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n", pSetSensorAWB->ABS_GAIN_B, bgain_32);
    LOG_INF("[imx230_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

    write_cmos_sensor(0x0b8e, (grgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b8f, grgain_32 & 0xFF);
    write_cmos_sensor(0x0b90, (rgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b91, rgain_32 & 0xFF);
    write_cmos_sensor(0x0b92, (bgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b93, bgain_32 & 0xFF);
    write_cmos_sensor(0x0b94, (gbgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b95, gbgain_32 & 0xFF);
    return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long*)feature_para;
	//unsigned long long *feature_return_data = (unsigned long long*)feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    SENSOR_VC_INFO_STRUCT *pvcinfo;
    SET_SENSOR_AWB_GAIN *pSetSensorAWB=(SET_SENSOR_AWB_GAIN *)feature_para;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

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
			set_shutter((UINT32)*feature_data_32);
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
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			read_imx230_DCC((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
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
		case MSDK_SCENARIO_ID_CUSTOM1:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
		case MSDK_SCENARIO_ID_CUSTOM2:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
		/*HDR CMD*/
		case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain(*feature_data,*(feature_data+1),*(feature_data+2));
            break;
		case SENSOR_FEATURE_GET_VC_INFO:
            LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
            pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            }
            break;
		case SENSOR_FEATURE_SET_AWB_GAIN:
            imx230_awb_gain(pSetSensorAWB);
            break;
#if 1  //Enable PDAF
		/*END OF HDR CMD*/
		/*PDAF CMD*/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			//PDAF capacity enable or not, 2p8 only full size support PDAF
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					break;
				default:
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			imgsensor.pdaf_mode= *feature_data_16;
			break;
		/*End of PDAF*/
#endif
		case SENSOR_FEATURE_SET_PDFOCUS_AREA:
            		LOG_INF("SENSOR_FEATURE_SET_IMX230_PDFOCUS_AREA Start Pos=%d, Size=%d\n",(UINT32)*feature_data,(UINT32)*(feature_data+1));
            		imx230_set_pd_focus_area(*feature_data,*(feature_data+1));
			break;
		case SENSOR_FEATURE_SET_HDR_SHUTTER:
			LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
			hdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1));
			break;
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

UINT32 IMX230SHARP_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    IMX230_MIPI_RAW_SensorInit    */
