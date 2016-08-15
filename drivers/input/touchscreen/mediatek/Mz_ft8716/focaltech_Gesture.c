/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include "ft_gesture_lib.h"
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/

#define GESTURE_LEFT						0x20
#define GESTURE_RIGHT						0x21
#define GESTURE_UP		    				0x22
#define GESTURE_DOWN						0x23
#define GESTURE_DOUBLECLICK					0x24
#define GESTURE_O		    				0x30
#define GESTURE_W		    				0x31
#define GESTURE_M		   	 			0x32
#define GESTURE_E		    				0x33
#define GESTURE_L		    				0x44
#define GESTURE_S		    				0x46
#define GESTURE_V_D		    				0x54
//#define GESTURE_V_U		    				0x53
//#define GESTURE_V_R		    				0x52
#define GESTURE_Z		    				0x65
#define GESTURE_C                              			0x34

#define FTS_GESTRUE_POINTS 					255
#define FTS_GESTRUE_POINTS_ONETIME  				62
#define FTS_GESTRUE_POINTS_HEADER 				8
#define FTS_GESTURE_OUTPUT_ADRESS 				0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 				4


/*******************************************************************************
* Static variables
*******************************************************************************/
short pointnum 				= 0;
unsigned short coordinate_x[150] 	= {0};
unsigned short coordinate_y[150] 	= {0};


/*******************************************************************************
*   Name: fts_Gesture_init
*  Brief:
*  Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_Gesture_init(struct input_dev *input_dev)
{
/*
        //init_para(480,854,60,0,0);
	//input_set_capability(input_dev, EV_KEY, KEY_POWER);
	//input_set_capability(input_dev, EV_KEY, GESTURE_U);
	input_set_capability(input_dev, EV_KEY, GESTURE_UP);
	input_set_capability(input_dev, EV_KEY, GESTURE_DOWN);
	input_set_capability(input_dev, EV_KEY, GESTURE_LEFT);
	input_set_capability(input_dev, EV_KEY, GESTURE_RIGHT);
	input_set_capability(input_dev, EV_KEY, GESTURE_O);
	input_set_capability(input_dev, EV_KEY, GESTURE_E);
	input_set_capability(input_dev, EV_KEY, GESTURE_M);
	input_set_capability(input_dev, EV_KEY, GESTURE_L);
	input_set_capability(input_dev, EV_KEY, GESTURE_W);
	input_set_capability(input_dev, EV_KEY, GESTURE_S);
	input_set_capability(input_dev, EV_KEY, GESTURE_V);
	input_set_capability(input_dev, EV_KEY, GESTURE_Z);

	__set_bit(GESTURE_RIGHT, input_dev->keybit);
	__set_bit(GESTURE_LEFT, input_dev->keybit);
	__set_bit(GESTURE_UP, input_dev->keybit);
	__set_bit(GESTURE_DOWN, input_dev->keybit);
	//__set_bit(GESTURE_U, input_dev->keybit);
	__set_bit(GESTURE_O, input_dev->keybit);
	__set_bit(GESTURE_E, input_dev->keybit);
	__set_bit(GESTURE_M, input_dev->keybit);
	__set_bit(GESTURE_W, input_dev->keybit);
	__set_bit(GESTURE_L, input_dev->keybit);
	__set_bit(GESTURE_S, input_dev->keybit);
	__set_bit(GESTURE_V, input_dev->keybit);
	__set_bit(GESTURE_Z, input_dev->keybit);
*/
	return 0;
}

/*******************************************************************************
*  Name: fts_check_gesture
*  Brief:
*  Input:
* Output: None
* Return: None
*******************************************************************************/
static void fts_check_gesture(struct input_dev *input_dev,int gesture_id)
{
	u8 gesture_code;
	TPD_DMESG("fts_check_gesture: fts gesture_id==0x%x\n ",gesture_id);
	switch(gesture_id)
	{
	        case GESTURE_LEFT:
			gesture_code = SWIPE_X_LEFT;
	                break;
	        case GESTURE_RIGHT:
			gesture_code = SWIPE_X_RIGHT;
			break;
	        case GESTURE_UP:
                        gesture_code = SWIPE_Y_UP;
	                break;
	        case GESTURE_DOWN:
			gesture_code = SWIPE_Y_DOWN;
	                break;
	        case GESTURE_DOUBLECLICK:
			gesture_code = DOUBLE_TAP;
	                break;
	        case GESTURE_O:
			gesture_code = UNICODE_O;
	                break;
	        case GESTURE_W:
			gesture_code = UNICODE_W;
	                break;
	        case GESTURE_M:
			gesture_code = UNICODE_M;
	                break;
	        case GESTURE_E:
			gesture_code = UNICODE_E;
	                break;
	        case GESTURE_L:
			gesture_code = UNICODE_L;
	                break;
	        case GESTURE_S:
			gesture_code = UNICODE_S;
	                break;
	        case GESTURE_V_D:
			gesture_code = UNICODE_V_DOWN;
	                break;
	        case GESTURE_Z:
			gesture_code = UNICODE_Z;
	                break;
	        case GESTURE_C:
			gesture_code = UNICODE_C;
	                break;
	        default:
			gesture_code = GESTURE_ERROR;
	                break;
	}
	mz_gesture_report(tpd->dev, gesture_code);
}

/************************************************************************
*   Name: fts_read_Gestruedata
* Brief: read data from TP register
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
int fts_read_Gestruedata(void)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
	int ret = -1;
	int i = 0;
	int gestrue_id = 0;

	buf[0] = 0xd3;
	pointnum = 0;

	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0)
	{
    		pr_err( "%s read touchdata failed.\n", __func__);
    		return ret;
	}
	/* FW */
 	if (fts_updateinfo_curr.CHIP_ID == 0x87 || fts_updateinfo_curr.CHIP_ID == 0xEF)
 	{
	 	gestrue_id = buf[0];
	 	pointnum = (short)(buf[1]) & 0xff;
	 	buf[0] = 0xd3;
	 	TPD_DEBUG("fts_read_Gestruedata: chip_id 0x%x, pointnum %d \n", fts_updateinfo_curr.CHIP_ID, pointnum);
	 	if((pointnum * 4 + 2) < 255)
	 	{
	    	 	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, (pointnum * 4 + 2));
	 	} else {
	        	ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, 255);
	       		ret = fts_i2c_read(fts_i2c_client, buf, 0, buf+255, (pointnum * 4 + 2) -255);
	 	}
	 	if (ret < 0)
	 	{
	       		pr_err( "%s read touchdata failed.\n", __func__);
	       		return ret;
	 	}

	 	fts_check_gesture(fts_input_dev,gestrue_id);
	 	for(i = 0;i < pointnum;i++)
	 	{
	    		coordinate_x[i] =  (((s16) buf[0 + (4 * i+2)]) & 0x0F) <<
	        		8 | (((s16) buf[1 + (4 * i+2)])& 0xFF);
	    		coordinate_y[i] = (((s16) buf[2 + (4 * i+2)]) & 0x0F) <<
	        		8 | (((s16) buf[3 + (4 * i+2)]) & 0xFF);
	 	}
	 	return -1;
	}
	return ret;
}

