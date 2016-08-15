/*
 *  stmvl53l0.c - Linux kernel modules for STM VL53l0 FlightSense Time-of-Flight sensor
 *
 *  Copyright (C) 2014 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/delay.h>
//#include <linux/meizu-sys.h>
#include <asm/atomic.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "vl53l0_api.h"
#include "vl53l010_api.h"
#include "vl53l0_platform.h"

#ifndef ABS
#define ABS(x)              (((x) > 0) ? (x) : (-(x)))
#endif

#define VL53l0_NORMAL_MODE	0x00
#define VL53l0_OFFSET_CALIB	0x02
#define VL53l0_XTALK_CALIB		0x03

#define VL53l0_MAGIC 'A'

#define VL53l0_IOCTL_INIT 			_IO(VL53l0_MAGIC, 	0x01)
#define VL53l0_IOCTL_GETOFFCALB	_IOR(VL53l0_MAGIC, 	VL53l0_OFFSET_CALIB, int)
#define VL53l0_IOCTL_GETXTALKCALB	_IOR(VL53l0_MAGIC, 	VL53l0_XTALK_CALIB, int)
#define VL53l0_IOCTL_SETOFFCALB	_IOW(VL53l0_MAGIC, 	0x04, int)
#define VL53l0_IOCTL_SETXTALKCALB	_IOW(VL53l0_MAGIC, 	0x05, int)
#define VL53l0_IOCTL_GETDATA 		_IOR(VL53l0_MAGIC, 	0x0a, LaserInfo)


#define LASER_I2C_BUSNUM 3

#define LASER_DRVNAME "stmvl53l0"
#define I2C_SLAVE_ADDRESS          0x52
#define I2C_REGISTER_ID            0x52
#define PLATFORM_DRIVER_NAME "laser_actuator_stmvl53l0"
#define LASER_DRIVER_CLASS_NAME "laser_stmvl53l0"

static struct i2c_board_info __initdata kd_laser_dev={ I2C_BOARD_INFO(LASER_DRVNAME, I2C_REGISTER_ID)};

#define PK_INF(fmt, args...)     pr_info(LASER_DRVNAME "[%s] " fmt, __FUNCTION__, ##args)

static spinlock_t g_Laser_SpinLock;

static struct i2c_client * g_pstLaser_I2Cclient = NULL;

static dev_t g_Laser_devno;
static struct cdev * g_pLaser_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int    g_s4Laser_Opened = 0;

VL53L0_Error Status = VL53L0_ERROR_NONE;  //global status
VL53L0_Dev_t MyDevice;
VL53L0_Dev_t *pMyDevice = &MyDevice;

int offset_init = 0;

static int g_Laser_OffsetCalib = 0xFFFFFFFF;
static int g_Laser_XTalkCalib = 0xFFFFFFFF;

typedef enum
{
    STATUS_RANGING_VALID             = 0x0,  // reference laser ranging distance
    STATUS_MOVE_DMAX                 = 0x1,  // Search range [DMAX  : infinity]
    STATUS_MOVE_MAX_RANGING_DIST     = 0x2,  // Search range [xx cm : infinity], according to the laser max ranging distance
    STATUS_NOT_REFERENCE             = 0x3
} LASER_STATUS_T;

typedef struct
{
    //current position
    int u4LaserCurPos;
    //laser status
    int u4LaserStatus;
    //DMAX
    int u4LaserDMAX;
} LaserInfo;
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int Laser_Enable(bool on);
//extern int laser_data_write_emmc(int *data, int size);
//extern int laser_data_read_emmc(int *data, int size);

/////////////////////////////////////////

int s4VL53l0_ReadRegByte(u8 addr, u8 *data)
{
    u8 pu_send_cmd[1] = {(u8)(addr & 0xFF) };

    g_pstLaser_I2Cclient->addr = (I2C_SLAVE_ADDRESS) >> 1;	
    
    if (i2c_master_send(g_pstLaser_I2Cclient, pu_send_cmd, 1) < 0){
        PK_INF("I2C write failed!! \n");
        return -1;
    }
	
    if (i2c_master_recv(g_pstLaser_I2Cclient, data , 1) < 0){
        PK_INF("I2C read failed!! \n");
        return -1;
    }

    return 0;
}

int s4VL53l0_WriteRegByte(u8 addr, u8 data)
{

    u8 pu_send_cmd[2] = {	(u8)(addr&0xFF),(u8)( data&0xFF)};
    g_pstLaser_I2Cclient->addr = (I2C_SLAVE_ADDRESS) >> 1;
	
    if (i2c_master_send(g_pstLaser_I2Cclient, pu_send_cmd , 2) < 0)
    {
        PK_INF("I2C write failed!! \n");
        return -1;
    }

    return 0;
}

int VL53l0_SetOffsetValue(int32_t OffsetValue);
int VL53l0_SetCrosstalkValue(FixPoint1616_t OffsetValue);

void VL53l0_SystemInit(int CalibMode)
{
	VL53L0_DeviceInfo_t VL53L0_DeviceInfo;
	int32_t defaultOffset;
	int32_t defaultXtalk;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint8_t val;

	VL53L0_GetDeviceInfo(&MyDevice, &VL53L0_DeviceInfo);
	MyDevice.chip_version = VL53L0_DeviceInfo.ProductRevisionMinor;

	PK_INF("[VL53L0] module id = %02X\n", VL53L0_DeviceInfo.ProductType);
	PK_INF("[VL53L0] product version = %d.%d\n", VL53L0_DeviceInfo.ProductRevisionMajor, VL53L0_DeviceInfo.ProductRevisionMinor);
	PK_INF("[VL53L0] minor version = %d\n", MyDevice.chip_version);

	if( MyDevice.chip_version == 0 )
	{
       	PK_INF ("Call of VL53L010_DataInit\n");
       	Status = VL53L010_DataInit(&MyDevice); // Data initialization
       	PK_INF ("Call of VL53L010_StaticInit\n");
       	Status = VL53L010_StaticInit(&MyDevice); // Device Initialization
		#if 0
		VL53L010_GetOffsetCalibrationDataMicroMeter(&MyDevice, &defaultOffset);
		PK_INF("[VL53L0] Default offset = %d before set offset calibration\n", defaultOffset);
		//VL53l0_SetOffsetValue(3000);
		VL53L010_SetOffsetCalibrationDataMicroMeter(&MyDevice,3000);
		PK_INF("[VL53L0] VL53l0_SetOffsetValue = 3000\n");
		VL53L010_GetOffsetCalibrationDataMicroMeter(&MyDevice, &defaultOffset);
		PK_INF("[VL53L0] Default offset = %d after set offset calibration\n", defaultOffset);
		#endif
        	PK_INF ("Call of VL53L010_SetDeviceMode\n");
        	Status = VL53L010_SetDeviceMode(&MyDevice, VL53L0_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		#if 0
		VL53L010_GetXTalkCompensationRateMegaCps(&MyDevice,&defaultXtalk);
		PK_INF("[VL53L0] Default xtalk = 0x%X before set xtalk calibration\n", defaultXtalk);

		mdelay(10);
		s4VL53l0_ReadRegByte(0xC0, &val);
		PK_INF("[VL53L0] Reg 0xC0 = 0x%X\n", val);
		s4VL53l0_ReadRegByte(0x20, &val);
		PK_INF("[VL53L0] Reg 0x20 = 0x%X\n", val);
		s4VL53l0_ReadRegByte(0x21, &val);
		PK_INF("[VL53L0] Reg 0x21 = 0x%X\n", val);
		s4VL53l0_ReadRegByte(0xC3, &val);
		PK_INF("[VL53L0] Reg 0xC3 = 0x%X\n", val);
		mdelay(10);

		VL53L010_SetXTalkCompensationRateMegaCps(&MyDevice, 80);
		mdelay(10);
		VL53L010_GetXTalkCompensationRateMegaCps(&MyDevice,&defaultXtalk);
		PK_INF("[VL53L0] Default xtalk = 0x%X after set xtalk calibration\n", defaultXtalk);
		#endif
   	 	PK_INF("VL53L0 init Done ~\n");
	} else if( MyDevice.chip_version == 1 )
	{
       	PK_INF ("Call of VL53L0_DataInit\n");
       	Status = VL53L0_DataInit(&MyDevice); // Data initialization
       	PK_INF ("Call of VL53L0_StaticInit\n");
       	Status = VL53L0_StaticInit(&MyDevice); // Static Initialization
	
       	PK_INF ("Call of VL53L0_PerformRefCalibration\n");
	Status = VL53L0_PerformRefCalibration(pMyDevice,&VhvSettings, &PhaseCal); // PerformRefCalibration Initialization
       	PK_INF ("Call of VL53L0_PerformRefSpadManagement\n");
	Status = VL53L0_PerformRefSpadManagement(&MyDevice,&refSpadCount, &isApertureSpads); // PerformRefSpadManagement Initialization
		#if 0
		VL53L0_GetOffsetCalibrationDataMicroMeter(&MyDevice, &defaultOffset);
		PK_INF("[VL53L0] Default offset = %d before set offset calibration\n", defaultOffset);
		//VL53l0_SetOffsetValue(3000);
		VL53L0_SetOffsetCalibrationDataMicroMeter(&MyDevice,3000);
		PK_INF("[VL53L0] VL53l0_SetOffsetValue = 3000\n");
		VL53L0_GetOffsetCalibrationDataMicroMeter(&MyDevice, &defaultOffset);
		PK_INF("[VL53L0] Default offset = %d after set offset calibration\n", defaultOffset);
		#endif
	
        	PK_INF ("Call of VL53L0_SetDeviceMode\n");
        	Status = VL53L0_SetDeviceMode(&MyDevice, VL53L0_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		#if 0
 		VL53L0_GetXTalkCompensationRateMegaCps(&MyDevice,&defaultXtalk);
		PK_INF("[VL53L0] Default xtalk = %d before set xtalk calibration\n", defaultXtalk);
		VL53L0_SetXTalkCompensationRateMegaCps(&MyDevice, 80);
		VL53L0_GetXTalkCompensationRateMegaCps(&MyDevice,&defaultXtalk);
		PK_INF("[VL53L0] Default xtalk = %d after set xtalk calibration\n", defaultXtalk);
		#endif
   	 	PK_INF("VL53L0 init Done ~\n");
	} else {
		// error. non-support chip mode
		// TODO...
	}	

	#if 0
	if(CalibMode == VL53l0_OFFSET_CALIB)
		g_Laser_OffsetCalib = 0xFFFFFFFF;
	else if(CalibMode == VL53l0_XTALK_CALIB)
		g_Laser_XTalkCalib = 0xFFFFFFFF;
	if(g_Laser_OffsetCalib != 0xFFFFFFFF)
		VL53l0_SetOffsetValue(g_Laser_OffsetCalib);
	if(g_Laser_XTalkCalib != 0xFFFFFFFF)
		VL53l0_SetCrosstalkValue(g_Laser_XTalkCalib);
	#endif
		
}

LaserInfo VL53l0_GetRangeInfo(VL53L0_RangingMeasurementData_t *RangingMeasurementData)
{
	LaserInfo Result;
	int i=0;
	
	if( MyDevice.chip_version == 0 )
	{
		//PK_INF ("Call of VL53L010_PerformSingleRangingMeasurement\n");
		for(i=0;i<5;i++){
		Status = VL53L010_PerformSingleRangingMeasurement(&MyDevice, RangingMeasurementData);
		if(Status == VL53L0_ERROR_NONE)
			break;
		}
	} else if( MyDevice.chip_version == 1 )
	{
		//PK_INF ("Call of VL53L0_PerformSingleRangingMeasurement\n");
	
		for(i=0;i<5;i++){
		Status = VL53L0_PerformSingleRangingMeasurement(&MyDevice, RangingMeasurementData);
		if(Status == VL53L0_ERROR_NONE)
			break;
		}
	} else
	{
		// error. non-support chip mode
		// TODO...
	}

	#if 0
	if (Status == VL53L0_ERROR_NONE) {
		Result = RangingMeasurementData->RangeMilliMeter;
		PK_INF("Measured distance: %i,try:%d\n", RangingMeasurementData->RangeMilliMeter,i+1);
		return Result;
	}
	#else
	Result.u4LaserCurPos = RangingMeasurementData->RangeMilliMeter;
	switch(RangingMeasurementData->RangeStatus)
	{
		case 0:
			Result.u4LaserStatus = 0;//Ranging valid
			break;
		case 1:
		case 2:
			Result.u4LaserStatus = 1;//Sigma or Signal
			break;
		case 4:
			Result.u4LaserStatus = 2;//Phase fail
			break;
		default:
			Result.u4LaserStatus = 3;//others
			break;
	}
	//Result.u4LaserStatus = RangingMeasurementData->RangeStatus;
	Result.u4LaserDMAX = RangingMeasurementData->RangeDMaxMilliMeter;
	printk("Measured distance/status/MTK status/Dmax: %i/%i/%i/%i\n\n", RangingMeasurementData->RangeMilliMeter, RangingMeasurementData->RangeStatus, Result.u4LaserStatus, RangingMeasurementData->RangeDMaxMilliMeter);
	return Result;
	#endif
}

int VL53l0_GetOffsetValue(int32_t *pOffsetValue)
{
	// TODO...
	// should be not necessary
	//VL53l0_SystemInit();
	// end of TODO ...

	if( MyDevice.chip_version == 0 ){
		Status = VL53L010_GetOffsetCalibrationDataMicroMeter(&MyDevice,pOffsetValue);
	}
	else if( MyDevice.chip_version == 1 ){
		Status = VL53L0_GetOffsetCalibrationDataMicroMeter(&MyDevice,pOffsetValue);
	}
	else
	{
		// TODO...
		// error handling
	}
	
	return -1;
}

int VL53l0_SetOffsetValue(int32_t OffsetValue)
{
	// TODO...
	// should be not necessary
	//VL53l0_SystemInit();
	// end of TODO

	if( MyDevice.chip_version == 0 ){
		Status = VL53L010_SetOffsetCalibrationDataMicroMeter(&MyDevice,OffsetValue);
	}
	else if( MyDevice.chip_version == 1 ){
		Status = VL53L0_SetOffsetCalibrationDataMicroMeter(&MyDevice,OffsetValue);
	}
	else{
		// TODO...
		// error handling
	}
	
	return -1;
}

int VL53l0_OffsetCalibration(int32_t *pCalibratedValue)
{
	FixPoint1616_t calibration_position;
	
	//VL53l0_SystemInit();

	if( MyDevice.chip_version == 0 )
	{
		// disable crosstalk calibration
		VL53L010_SetXTalkCompensationEnable(&MyDevice, 0);
	
		// TODO...
		// need to know what the exactly calibration position is.
		// ref to calibration guide.
		// assume calibration_position is 10cm
		calibration_position = 100*65536;
			
		Status = VL53L010_PerformOffsetCalibration(&MyDevice,
					calibration_position,
					pCalibratedValue);
	} else if( MyDevice.chip_version == 1 )
	{
		// disable crosstalk calibration
		VL53L0_SetXTalkCompensationEnable(&MyDevice, 0);
	
		// TODO...
		// need to know what the exactly calibration position is.
		// ref to calibration guide.
		// assume calibration_position is 10cm
		calibration_position = 100*65536;
			
		Status = VL53L0_PerformOffsetCalibration(&MyDevice,
					calibration_position,
					pCalibratedValue);
	}
	else
	{
		// TODO...
		// error handling
	}
	
	//VL53l0_SystemInit();
	
	
	return -1;
}

int VL53l0_GetCrosstalkValue(FixPoint1616_t *pOffsetValue)
{
	//VL53l0_SystemInit();

	if( MyDevice.chip_version == 0 )
	{
		Status = VL53L010_GetXTalkCompensationRateMegaCps(&MyDevice,
			pOffsetValue);
	}
	else if( MyDevice.chip_version == 1 )
	{
		Status = VL53L0_GetXTalkCompensationRateMegaCps(&MyDevice,
			pOffsetValue);
	}
	else
	{
		// TODO...
		// error handling
	}
	
	return -1;
}

int VL53l0_SetCrosstalkValue(FixPoint1616_t OffsetValue)
{
	VL53L0_RangingMeasurementData_t _RangingMeasurementData;
	int i;
	msleep(5);
	//VL53l0_SystemInit();

	if( MyDevice.chip_version == 0 )
	{
		Status = VL53L010_SetXTalkCompensationRateMegaCps(&MyDevice,
			OffsetValue);
	}
	else if( MyDevice.chip_version == 1 )
	{
		Status = VL53L0_SetXTalkCompensationRateMegaCps(&MyDevice,
			OffsetValue);
	}
	else
	{
		// TODO...
		// error handling
	}

	
	if(Status == VL53L0_ERROR_NONE)
	{
		for(i=0;i<2;i++){
			printk ("Call of VL53L010_PerformSingleRangingMeasurement\n");
			Status = VL53L010_PerformSingleRangingMeasurement(pMyDevice, &_RangingMeasurementData);
	
			//print_pal_error(Status);
			//print_range_status(&RangingMeasurementData);
	
			if (Status != VL53L0_ERROR_NONE) 
				break;
			printk("Measured distance: RangeMilliMeter:%i, SignalRateRtnMegaCps:%i \n\n", _RangingMeasurementData.RangeMilliMeter, _RangingMeasurementData.SignalRateRtnMegaCps);
	
	
		}
	}
	Status = VL53L0_SetDmaxCalParameters(pMyDevice, 600, _RangingMeasurementData.SignalRateRtnMegaCps * 65536 , 9 * 65536);

	return -1;
}

int VL53l0_XtalkCalibration(FixPoint1616_t *pCalibratedValue)
{
	FixPoint1616_t calibration_position;
	
	//VL53l0_SystemInit();

	if( MyDevice.chip_version == 0 )
	{
		// TODO...
		// need to know what the exactly calibration position is.
		// ref to calibration guide.
		// assume calibration_position is 60cm
		calibration_position = 600*65536;
			
		Status = VL53L010_PerformXTalkCalibration(&MyDevice,
					calibration_position,
					pCalibratedValue);
	} else if( MyDevice.chip_version == 1 )
	{
		// TODO...
		// need to know what the exactly calibration position is.
		// ref to calibration guide.
		// assume calibration_position is 60cm
		calibration_position = 600*65536;
			
		Status = VL53L0_PerformXTalkCalibration(&MyDevice,
					calibration_position,
					pCalibratedValue);
	}
	else
	{
		// TODO...
		// error handling
	}
	
	//VL53l0_SystemInit();
	
	return -1;
}

////////////////////////////////////////////////////////////////
static long Laser_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    	long i4RetValue = 0;
	
		
	switch(a_u4Command)
	{
		case VL53l0_IOCTL_INIT:	   /* init.  */
			// to speed up power on time, do nothing in initial stage
			if(g_s4Laser_Opened == 1)
			{
				uint8_t Device_Model_ID = 0;
				s4VL53l0_ReadRegByte(VL53L010_REG_IDENTIFICATION_MODEL_ID, &Device_Model_ID);

				if(Device_Model_ID != 0xEE)
				{
					PK_INF("Not found VL53l0!Device_Model_ID = 0x%x", Device_Model_ID);
					return -1;
				}
			}
			break;

		case VL53l0_IOCTL_GETDATA:
			if( g_s4Laser_Opened == 1 )
			{
				VL53l0_SystemInit(VL53l0_NORMAL_MODE);

				spin_lock(&g_Laser_SpinLock);
        			g_s4Laser_Opened = 2;
        			spin_unlock(&g_Laser_SpinLock);
			}
			else	if( g_s4Laser_Opened == 2 )	
			{
				void __user *p_u4Param = (void __user *)a_u4Param;
				LaserInfo ParamVal;
				VL53L0_RangingMeasurementData_t RangingMeasurementData;
				
				ParamVal = VL53l0_GetRangeInfo(&RangingMeasurementData);
				PK_INF("[stmvl53l0]Get result distance:%d\n", ParamVal);

				if(copy_to_user(p_u4Param , &ParamVal , sizeof(LaserInfo)))
 			      {
 			 	     PK_INF("copy to user failed when getting motor information \n");
 			      }

				//spin_unlock(&g_Laser_SpinLock);  //system breakdown 
				
			}
			break;

		case VL53l0_IOCTL_GETOFFCALB:  //Offset Calibrate place white target at 100mm from glass
			{
				void __user *p_u4Param = (void __user *)a_u4Param;
				FixPoint1616_t CalibratedValue;
				
				spin_lock(&g_Laser_SpinLock);
        			g_s4Laser_Opened = 3;
        			spin_unlock(&g_Laser_SpinLock);
			
				VL53l0_OffsetCalibration(&CalibratedValue);
				g_Laser_OffsetCalib = CalibratedValue;
				spin_lock(&g_Laser_SpinLock);
        			g_s4Laser_Opened = 2;
        			spin_unlock(&g_Laser_SpinLock);
					
				if(copy_to_user(p_u4Param , &CalibratedValue , sizeof(FixPoint1616_t)))
				{
					PK_INF("copy to user failed when getting VL53l0_IOCTL_GETOFFCALB \n");
				}
			}
			break;
		case VL53l0_IOCTL_SETOFFCALB:
			/* WORKAROUND: FIXME */
			g_Laser_OffsetCalib = (int)a_u4Param;
			VL53l0_SetOffsetValue(g_Laser_OffsetCalib);
			PK_INF("g_Laser_OffsetCalib : %d\n", g_Laser_OffsetCalib);
			break;

		case VL53l0_IOCTL_GETXTALKCALB: // Place a dark target at 400mm ~ Lower reflectance target recommended, e.g. 17% gray card.
			{
				void __user *p_u4Param = (void __user *)a_u4Param;
				FixPoint1616_t CalibratedValue;
				
				spin_lock(&g_Laser_SpinLock);
				g_s4Laser_Opened = 3;
				spin_unlock(&g_Laser_SpinLock);
				
				VL53l0_XtalkCalibration(&CalibratedValue);
				g_Laser_XTalkCalib = CalibratedValue;
				
				spin_lock(&g_Laser_SpinLock);
				g_s4Laser_Opened = 2;
				spin_unlock(&g_Laser_SpinLock);

				if(copy_to_user(p_u4Param , &CalibratedValue , sizeof(FixPoint1616_t)))
				{		
					PK_INF("copy to user failed when getting VL53l0_IOCTL_GETOFFCALB \n");
				}
			}
			break;

		case VL53l0_IOCTL_SETXTALKCALB:
			g_Laser_XTalkCalib = (int)a_u4Param;
			VL53l0_SetCrosstalkValue(g_Laser_XTalkCalib);
			PK_INF("g_Laser_XTalkCalib : %d\n", g_Laser_XTalkCalib);
			break;
		default :
			PK_INF("No CMD \n");
			i4RetValue = -EPERM;
			break;
	}

    return i4RetValue;
}

//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
// 3.Update f_op pointer.
// 4.Fill data structures into private_data
//CAM_RESET
static int Laser_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    PK_INF("Start \n");

    if( g_s4Laser_Opened )
    {
        PK_INF("The device is opened \n");
        return -EBUSY;
    }

    spin_lock(&g_Laser_SpinLock);
    g_s4Laser_Opened = 1;
    offset_init = 0;
    spin_unlock(&g_Laser_SpinLock);

    PK_INF("End \n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int Laser_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    PK_INF("Start \n");

    if (g_s4Laser_Opened)
    {
        PK_INF("Free \n");

       spin_lock(&g_Laser_SpinLock);
       g_s4Laser_Opened = 0;
       offset_init = 0;
       spin_unlock(&g_Laser_SpinLock);
    }

    PK_INF("End \n");

    return 0;
}

static const struct file_operations g_stLaser_fops =
{
    .owner = THIS_MODULE,
    .open = Laser_Open,
    .release = Laser_Release,
    .unlocked_ioctl = Laser_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = Laser_Ioctl,
#endif
};


static ssize_t flight_calibration_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	char *p = buf;
		
	return (p - buf);
}

static ssize_t flight_calibration_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int val;
	
	
	sscanf(buf, "%d", &val);
	PK_INF("[VL53L0] %s : %d\n", __func__, val);
	
	switch (val) {
	case 1:
	{
		// offset calibration
		int32_t CalibratedOffset;
		VL53l0_SystemInit(VL53l0_OFFSET_CALIB);
		VL53l0_OffsetCalibration(&CalibratedOffset);
		PK_INF("[VL53L0] Offset calibration result = %d\n", CalibratedOffset);
		MyDevice.OffsetCalibratedValue = CalibratedOffset;
	}
		break;
	case 2:
	{
		// crosstalk calibration
		FixPoint1616_t CalibratedValue;
		VL53l0_SystemInit(VL53l0_XTALK_CALIB);
		VL53l0_XtalkCalibration(&CalibratedValue);
		PK_INF("[VL53L0] Crosstalk calibration result = %d\n", CalibratedValue);
		MyDevice.CrosstalkCalibratedValue = CalibratedValue;
	}
		break;
	default:
		break;
	}

	return count;
}

static ssize_t flight_offset_info_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int32_t OffsetValue;
	char *p = buf;

	VL53l0_GetOffsetValue(&OffsetValue);
	
	p += sprintf(p, "Offset=%d\n", OffsetValue);
		
	return (p - buf);
}

static ssize_t flight_offset_info_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int32_t val;
	
	
	sscanf(buf, "%d", &val);
	PK_INF("[VL53L0] %s : %d\n", __func__, val);

	VL53l0_SetOffsetValue(val);

	return count;
}

static ssize_t flight_crosstalk_info_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	FixPoint1616_t CrosstalkValue;
	char *p = buf;

	VL53l0_GetCrosstalkValue(&CrosstalkValue);
	
	p += sprintf(p, "Crosstalk=%d\n", CrosstalkValue);
		
	return (p - buf);
}

static ssize_t flight_crosstalk_info_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t val;
	FixPoint1616_t CrosstalkValue;
	
	sscanf(buf, "%d", &val);
	PK_INF("[VL53L0] %s : %d\n", __func__, val);
	#if 0
	VL53l0_SystemInit(VL53l0_NORMAL_MODE);
	VL53l0_GetCrosstalkValue(&CrosstalkValue);
	PK_INF("[VL53L0] %s :VL53l0_GetCrosstalkValue= 0x%X\n", __func__, CrosstalkValue);
	#endif
	VL53l0_SetCrosstalkValue(val);
	PK_INF("[VL53L0] %s :VL53l0_SetCrosstalkValue 0x%X\n", __func__, val);
	//VL53l0_SystemInit(VL53l0_NORMAL_MODE);
	#if 0
	s4VL53l0_ReadRegByte(0x20, &val);
	PK_INF("[VL53L0] Reg 0x20 = 0x%X\n", val);
	s4VL53l0_ReadRegByte(0x21, &val);
	PK_INF("[VL53L0] Reg 0x21 = 0x%X\n", val);
	VL53l0_GetCrosstalkValue(&CrosstalkValue);
	PK_INF("[VL53L0] %s :VL53l0_GetCrosstalkValue= 0x%X\n", __func__, CrosstalkValue);
	#endif
	return count;
}

static ssize_t flight_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	char *p = buf;
	int distance = 0;
	VL53L0_RangingMeasurementData_t RangingMeasurementData;

	Laser_Enable(true);
	//VL53l0_SystemInit(VL53l0_NORMAL_MODE);
	distance = VL53l0_GetRangeInfo(&RangingMeasurementData).u4LaserCurPos;
	p += sprintf(p, "%d\n", distance);
	
	msleep(50);
	Laser_Enable(false);
		
	return (p - buf);
}

static ssize_t flight_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int val;
	int distance = 0;
	VL53L0_RangingMeasurementData_t RangingMeasurementData_store;
	
	sscanf(buf, "%d", &val);

	PK_INF("%d\n",val);
	
	switch (val) {
	case 1:
		VL53l0_SystemInit(VL53l0_NORMAL_MODE);
		distance = VL53l0_GetRangeInfo(&RangingMeasurementData_store).u4LaserCurPos;
		PK_INF("test get distance(mm): %d\n",distance);
		break;
	case 3:
		VL53l0_SystemInit(VL53l0_NORMAL_MODE);
		break;
	default:
		break;
	}

	return count;
}


static struct device_attribute dev_attr_ctrl = {
	.attr = {.name = "laser_distance", .mode = 0644},
	.show = flight_show,
	.store = flight_store,
};

static struct device_attribute dev_attr_calibration = {
	.attr = {.name = "flight_calibration_proc", .mode = 0644},
	.show = flight_calibration_show,
	.store = flight_calibration_store,
};

static struct device_attribute dev_attr_offset_info = {
	.attr = {.name = "offset_info", .mode = 0644},
	.show = flight_offset_info_show,
	.store = flight_offset_info_store,
};

static struct device_attribute dev_attr_crosstalk_info = {
	.attr = {.name = "crosstalk_info", .mode = 0644},
	.show = flight_crosstalk_info_show,
	.store = flight_crosstalk_info_store,
};

inline static int Register_Laser_CharDrv(void)
{
	struct device* laser_device = NULL;

	PK_INF("Start\n");

	//Allocate char driver no.
	if( alloc_chrdev_region(&g_Laser_devno, 0, 1,LASER_DRVNAME) )
	{
		 PK_INF("Allocate device no failed\n");
		return -EAGAIN;
	}

	//Allocate driver
	g_pLaser_CharDrv = cdev_alloc();

	if(NULL == g_pLaser_CharDrv)
	{
		unregister_chrdev_region(g_Laser_devno, 1);
		PK_INF("Allocate mem for kobject failed\n");
		return -ENOMEM;
	}

	//Attatch file operation.
	cdev_init(g_pLaser_CharDrv, &g_stLaser_fops);
	g_pLaser_CharDrv->owner = THIS_MODULE;

	//Add to system
	if(cdev_add(g_pLaser_CharDrv, g_Laser_devno, 1))
	{
		PK_INF("Attatch file operation failed\n");
		unregister_chrdev_region(g_Laser_devno, 1);
		return -EAGAIN;
	}

	actuator_class = class_create(THIS_MODULE, LASER_DRIVER_CLASS_NAME);
	if (IS_ERR(actuator_class)) {
		int ret = PTR_ERR(actuator_class);
		PK_INF("Unable to create class, err = %d\n", ret);
		return ret;
	}

	laser_device = device_create(actuator_class, NULL, g_Laser_devno, NULL, LASER_DRVNAME);

	if(NULL == laser_device)
	{
		return -EIO;
	}

	device_create_file(laser_device, &dev_attr_ctrl);
	device_create_file(laser_device, &dev_attr_calibration);
	device_create_file(laser_device, &dev_attr_offset_info);
	device_create_file(laser_device, &dev_attr_crosstalk_info);

	//meizu_sysfslink_register_n(laser_device, "laser");
	PK_INF("End\n");
	return 0;
}

inline static void UnRegister_Laser_CharDrv(void)
{
    PK_INF("Start\n");

    //Release char driver
    cdev_del(g_pLaser_CharDrv);

    unregister_chrdev_region(g_Laser_devno, 1);

    device_destroy(actuator_class, g_Laser_devno);

    class_destroy(actuator_class);

    PK_INF("End\n");
}

//////////////////////////////////////////////////////////////////////

static int Laser_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int Laser_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id Laser_i2c_id[] = {{LASER_DRVNAME, 0},{}};
static struct i2c_driver Laser_i2c_driver = {
    .probe = Laser_i2c_probe,
    .remove = Laser_i2c_remove,
    .driver.name = LASER_DRVNAME,
    .id_table = Laser_i2c_id,
};

#if 0
static int Laser_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, LASER_DRVNAME);
    return 0;
}
#endif
static int Laser_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int Laser_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    PK_INF("Start\n");

    /* Kirby: add new-style driver { */
    g_pstLaser_I2Cclient = client;

    g_pstLaser_I2Cclient->addr = I2C_SLAVE_ADDRESS;

    g_pstLaser_I2Cclient->addr = g_pstLaser_I2Cclient->addr >> 1;

    //Register char driver
    i4RetValue = Register_Laser_CharDrv();

    if(i4RetValue){

        PK_INF(" register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_Laser_SpinLock);
	
    PK_INF("Attached!! \n");

    return 0;
}

static int Laser_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&Laser_i2c_driver);
}

static int Laser_remove(struct platform_device *pdev)
{
    i2c_del_driver(&Laser_i2c_driver);
    return 0;
}

static int Laser_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int Laser_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stLaser_Driver = {
    .probe        = Laser_probe,
    .remove    = Laser_remove,
    .suspend    = Laser_suspend,
    .resume    = Laser_resume,
    .driver        = {
        .name    = PLATFORM_DRIVER_NAME,
        .owner    = THIS_MODULE,
    }
};
static struct platform_device g_stLaser_Device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};

static int __init STMVL53l0_i2c_init(void)
{
    i2c_register_board_info(LASER_I2C_BUSNUM, &kd_laser_dev, 1);

    if(platform_device_register(&g_stLaser_Device)){
        PK_INF("failed to register Laser driver\n");
        return -ENODEV;
    }

    if(platform_driver_register(&g_stLaser_Driver)){
        PK_INF("Failed to register Laser driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit STMVL53l0_i2c_exit(void)
{
    platform_driver_unregister(&g_stLaser_Driver);
}

module_init(STMVL53l0_i2c_init);
module_exit(STMVL53l0_i2c_exit);

#if 0
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_LICENSE("GPL");
#endif



