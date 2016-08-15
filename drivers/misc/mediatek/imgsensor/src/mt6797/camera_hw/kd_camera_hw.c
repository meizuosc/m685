/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw_m95]"

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, arg...)			pr_info(PFX fmt, ##arg)
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...) 		pr_info(PFX fmt, ##arg)
#else
#define PK_DBG(fmt, arg...)
#define PK_ERR(fmt, arg...)			pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...)		pr_info(PFX fmt, ##arg)
#endif


#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4


extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);


u32 pinSet[2][8] = {
	/* for main sensor */
	{CAMERA_CMRST_PIN,
	 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
	 GPIO_OUT_ONE,		/* ON state */
	 GPIO_OUT_ZERO,		/* OFF state */
	 CAMERA_CMPDN_PIN,
	 CAMERA_CMPDN_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
	/* for sub sensor */
	{CAMERA_CMRST1_PIN,
	 CAMERA_CMRST1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 CAMERA_CMPDN1_PIN,
	 CAMERA_CMPDN1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
};
#ifndef CONFIG_MTK_LEGACY
#define CUST_AVDD AVDD - AVDD
#define CUST_DVDD DVDD - AVDD
#define CUST_DOVDD DOVDD - AVDD
#define CUST_AFVDD AFVDD - AVDD
#define CUST_SUB_AVDD SUB_AVDD - AVDD
#define CUST_SUB_DVDD SUB_DVDD - AVDD
#define CUST_SUB_DOVDD SUB_DOVDD - AVDD
#define CUST_MAIN2_AVDD MAIN2_AVDD - AVDD
#define CUST_MAIN2_DVDD MAIN2_DVDD - AVDD
#define CUST_MAIN2_DOVDD MAIN2_DVDD - AVDD

#endif

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1220 1220000
#define VOL_1100 1100000
#define VOL_1000 1000000

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;/* main cam */
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam0_laser_h = NULL;
struct pinctrl_state *cam0_laser_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;/* sub cam */
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam2_pnd_h = NULL;/* main2 cam */
struct pinctrl_state *cam2_pnd_l = NULL;
struct pinctrl_state *cam2_rst_h = NULL;
struct pinctrl_state *cam2_rst_l = NULL;
struct pinctrl_state *cam_ldo_vcama_h = NULL;/* for AVDD */
struct pinctrl_state *cam_ldo_vcama_l = NULL;
struct pinctrl_state *cam_ldo_vcamd_h = NULL;/* for DVDD */
struct pinctrl_state *cam_ldo_vcamd_l = NULL;
struct pinctrl_state *cam_ldo_vcamio_h = NULL;/* for DOVDD */
struct pinctrl_state *cam_ldo_vcamio_l = NULL;
struct pinctrl_state *cam_ldo_vcamaf_h = NULL;/* for AFVDD */
struct pinctrl_state *cam_ldo_vcamaf_l = NULL;
struct pinctrl_state *cam_ldo_sub_vcamd_h = NULL;/* for SUB_DVDD */
struct pinctrl_state *cam_ldo_sub_vcamd_l = NULL;
struct pinctrl_state *cam_ldo_main2_vcamd_h = NULL;/* for MAIN2_DVDD */
struct pinctrl_state *cam_ldo_main2_vcamd_l = NULL;


int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}

	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		PK_ERR("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		PK_ERR("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*laser Enable GPIO140*/
	cam0_laser_h = pinctrl_lookup_state(camctrl, "cam0_laser1");
	if (IS_ERR(cam0_laser_h)) {
		ret = PTR_ERR(cam0_laser_h);
		PK_ERR("%s : pinctrl err, cam0_laser_h\n", __func__);
	}
	
	cam0_laser_l = pinctrl_lookup_state(camctrl, "cam0_laser0");
	if (IS_ERR(cam0_laser_l)) {
		ret = PTR_ERR(cam0_laser_l);
		PK_ERR("%s : pinctrl err, cam0_laser_l\n", __func__);
	}

	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		PK_ERR("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		PK_ERR("%s : pinctrl err, cam1_rst_l\n", __func__);
	}

	/*externel LDO enable */
	/*GPIO 30*/
	cam_ldo_vcama_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_1");
	if (IS_ERR(cam_ldo_vcama_h)) {
		ret = PTR_ERR(cam_ldo_vcama_h);
		PK_ERR("%s : pinctrl err, cam_ldo_vcama_h\n", __func__);
	}

	cam_ldo_vcama_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_0");
	if (IS_ERR(cam_ldo_vcama_l)) {
		ret = PTR_ERR(cam_ldo_vcama_l);
		PK_ERR("%s : pinctrl err, cam_ldo_vcama_l\n", __func__);
	}

	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	static signed int mAVDD_usercounter = 0;
	static signed int mDVDD_usercounter = 0;

	if (IS_ERR(camctrl)) {
		return -1;
	}
	
	switch (PwrType) {
	case RST:
		if (PinIdx == 0) {
			if (Val == 0 && !IS_ERR(cam0_rst_l))
				pinctrl_select_state(camctrl, cam0_rst_l);
			else if (Val == 1 && !IS_ERR(cam0_rst_h))
				pinctrl_select_state(camctrl, cam0_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		} else if (PinIdx == 1) {
			if (Val == 0 && !IS_ERR(cam1_rst_l))
				pinctrl_select_state(camctrl, cam1_rst_l);
			else if (Val == 1 && !IS_ERR(cam1_rst_h))
				pinctrl_select_state(camctrl, cam1_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		} else {
			PK_ERR("invalid camera index.\n");
		}
		break;
	case AVDD:
		/* M95 rear camera IMX386 use GPIO to control AVDD */
		if (Val == 0 && !IS_ERR(cam_ldo_vcama_l))
			pinctrl_select_state(camctrl, cam_ldo_vcama_l);
		else if (Val == 1 && !IS_ERR(cam_ldo_vcama_h))
			pinctrl_select_state(camctrl, cam_ldo_vcama_h);
		break;
#if 0
	case PDN:
		if (PinIdx == 0) {
			if (Val == 0 && !IS_ERR(cam0_pnd_l))
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else if (Val == 1 && !IS_ERR(cam0_pnd_h))
				pinctrl_select_state(camctrl, cam0_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		} else if (PinIdx == 1) {
			if (Val == 0 && !IS_ERR(cam1_pnd_l))
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else if (Val == 1 && !IS_ERR(cam1_pnd_h))
				pinctrl_select_state(camctrl, cam1_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		} else {
			if (Val == 0 && !IS_ERR(cam2_pnd_l))
				pinctrl_select_state(camctrl, cam2_pnd_l);
			else if (Val == 1 && !IS_ERR(cam2_pnd_h))
				pinctrl_select_state(camctrl, cam2_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		}
		break;
	case MAIN2_AVDD:
		/*Main & Main2 use same cotrol GPIO */
		PK_DBG("mAVDD_usercounter(%d)\n",mAVDD_usercounter);
		if (Val == 0 && !IS_ERR(cam_ldo_vcama_l)){
			mAVDD_usercounter --;
			if(mAVDD_usercounter <= 0)
			{
				if(mAVDD_usercounter < 0)
					PK_ERR("Please check AVDD pin control\n");

				mAVDD_usercounter = 0;
				pinctrl_select_state(camctrl, cam_ldo_vcama_l);
			}
			
		}
		else if (Val == 1 && !IS_ERR(cam_ldo_vcama_h)){
			mAVDD_usercounter ++;
			pinctrl_select_state(camctrl, cam_ldo_vcama_h);
		}
		break;
	case DVDD:
	case MAIN2_DVDD:
		PK_DBG("mDVDD_usercounter(%d)\n",mDVDD_usercounter);
		if (Val == 0 && !IS_ERR(cam_ldo_vcamd_l))
		{
			mDVDD_usercounter --;
			if(mDVDD_usercounter <= 0)
			{
				if(mDVDD_usercounter < 0)
					PK_ERR("Please check DVDD pin control\n");

				mDVDD_usercounter = 0;
				pinctrl_select_state(camctrl, cam_ldo_vcamd_l);
			}
		}
		else if (Val == 1 && !IS_ERR(cam_ldo_vcamd_h))
		{
			mDVDD_usercounter ++;
			pinctrl_select_state(camctrl, cam_ldo_vcamd_h);
		}
		break;
#endif
	case DOVDD:
	case AFVDD:
	case SUB_AVDD:
	case SUB_DVDD:
	default:
		PK_ERR("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

int Camera_AF_PowerOn(bool on){

	if(on){
		if (TRUE != _hwPowerOn(AFVDD,Vol_2800)) {
			PK_DBG("[CAMERA SENSOR] Fail to enable AFVDD power\n");
			return -1;	
		}
	}
	else{
		if (TRUE != _hwPowerDown(AFVDD)) {
			PK_DBG("[CAMERA SENSOR] Fail to disable AFVDD power\n");
			return -1;	
		}
	}

	return 0;
}

EXPORT_SYMBOL(Camera_AF_PowerOn);

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On,
		       char *mode_name)
{
	u32 pinSetIdx = 0;		/* default main sensor */
	u32 tmp_idx = 0;

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) {
		pinSetIdx = 0;
	} else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
	}

	/* power ON */
	if (On) {
		PK_INFO("kdCISModulePowerOn -on:currSensorName=%s\n", currSensorName);
		PK_INFO("kdCISModulePowerOn -on:pinSetIdx=%d\n", pinSetIdx);

		//first pull rst down
		if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set RST GPIO failed!!\n");
		}

		usleep_range(3000, 3500);

		if(pinSetIdx == 0){ 
			if (TRUE != _hwPowerOn(AFVDD,Vol_2800)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable AFVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}
		
			if (mtkcam_gpio_set(pinSetIdx, AVDD, 1)) {
				PK_DBG("[CAMERA SENSOR] set AVDD GPIO failed!!\n");
				goto _kdCISModulePowerOn_exit_;
			}
			
			if (TRUE != _hwPowerOn(DVDD,Vol_1100)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable AFVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != _hwPowerOn(DOVDD,Vol_1800)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable DOVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			usleep_range(2000, 2500);
			ISP_MCLK2_EN(TRUE);
			usleep_range(2000, 2500);
		}
		else if(pinSetIdx == 1){
		
			ISP_MCLK3_EN(TRUE);
			
			if (TRUE != _hwPowerOn(DOVDD,Vol_1800)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable SUB DOVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != _hwPowerOn(AVDD,Vol_2800)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable SUB AVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != _hwPowerOn(SUB_DVDD,Vol_1220)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable SUB_DVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			usleep_range(2000, 2500);

		}

		// pull rst up
		if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
				PK_DBG("[CAMERA SENSOR] set RST GPIO failed!!\n");
		}

		/* Disable unused camera sensor */
		tmp_idx = (pinSetIdx + 1) % 2;
		/* pull unused camera rst down */
		if (mtkcam_gpio_set(tmp_idx, RST, pinSet[tmp_idx][IDX_PS_CMRST + IDX_PS_OFF])) {
			PK_DBG("[CAMERA SENSOR] set RST GPIO failed!!\n");
		}

		usleep_range(10000, 10500);
	} else {		
	    /* power OFF */
		PK_DBG("kdCISModulePowerOFF -on:pinSetIdx=%d\n", pinSetIdx);

		if(pinSetIdx == 0)
			ISP_MCLK2_EN(FALSE);

		usleep_range(5000, 5500);
		//first pull rst down
		if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set RST GPIO failed!!\n");
		}
		usleep_range(2000, 2500);
		
		if(pinSetIdx == 0){
			if (TRUE != _hwPowerDown(AFVDD)) {
				PK_DBG("[CAMERA SENSOR] Fail to disable AFVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			
			if (TRUE != _hwPowerDown(DVDD)) {
				PK_DBG("[CAMERA SENSOR] Fail to disable DVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			
			if (TRUE != _hwPowerDown(DOVDD)) {
				PK_DBG("[CAMERA SENSOR] Fail to disable DOVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if (mtkcam_gpio_set(pinSetIdx, AVDD, 0)) {
				PK_DBG("[CAMERA SENSOR] set AVDD GPIO failed!!\n");
				goto _kdCISModulePowerOn_exit_;
			}
		}
		else if(pinSetIdx == 1){
			if (TRUE != _hwPowerDown(SUB_DVDD)) {
				PK_DBG("[CAMERA SENSOR] Fail to disable SUB_DVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != _hwPowerDown(DOVDD)) {
				PK_DBG("[CAMERA SENSOR] Fail to disable SUB DOVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != _hwPowerDown(AVDD)) {
				PK_DBG("[CAMERA SENSOR] Fail to disable SUB AVDD power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			usleep_range(2000, 2500);

			ISP_MCLK3_EN(FALSE);
		}
	}
	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);



/* !-- */
/*  */
