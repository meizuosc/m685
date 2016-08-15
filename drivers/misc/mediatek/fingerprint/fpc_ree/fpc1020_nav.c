/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

//#define DEBUG

#include <linux/input.h>
#include <linux/delay.h>

#ifndef CONFIG_OF
#include <linux/spi/fpc1020_common.h>
#include <linux/spi/fpc1020_input.h>
#else
#include "fpc1020_common.h"
#include "fpc1020_input.h"
#endif
#include "fpc1020.h"
/* -------------------------------------------------------------------- */
/* data types								*/
/* -------------------------------------------------------------------- */

#ifdef CONFIG_INPUT_FPC1020_NAV
typedef enum  {
	NaviStateIdle,
	NaviStateClick1Press,
	NaviStateClick1Release,
	NaviStateClick2Press,
	NaviStateNavigation,
} fpc1020_navistate_t;

typedef enum {
	NoClick,
	SingleClick,
	DoubleClick,
	FingerDownClick
} fpc1020_clickevent_t;
#endif


/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_nav_process_raw(int finger_raw);

static int fpc1020_write_nav_setup(fpc1020_data_t *fpc1020);
static int get_nav_dir(int *dir, int subarea_bits);

static fpc1020_navistate_t naviStateMachine(fpc1020_clickevent_t *click,
						int fingerPresence,
						bool reset);

static int fpc1020_wait_finger_present_lpm(fpc1020_data_t *fpc1020);
#endif


/* -------------------------------------------------------------------- */
/* driver constants							*/
/* -------------------------------------------------------------------- */
/* Threshold value use for detection of moving the finger */
#define FPC1020_CENTER_GRAVITY_THRESHOLD	5

#define FPC1020_KEY_CLICK		KEY_ENTER
#define FPC1020_KEY_FINGER_PRESENT	196//KEY_F18	/* 188*/

#define FPC1020_INPUT_POLL_TIME_MS	1000u

#define FPC1020_INPUT_FINGER_DETECT_THRESHOLD 1

#define FPC1020_KEY_REPORT_CONTINUE	1

enum {
	FPC1020_DIR_NONE	= 0,
	FPC1020_DIR_LEFT	= 1,
	FPC1020_DIR_LEFT_UP	= 2,
	FPC1020_DIR_UP		= 3,
	FPC1020_DIR_RIGHT_UP	= 4,
	FPC1020_DIR_RIGHT	= 5,
	FPC1020_DIR_RIGHT_DOWN	= 6,
	FPC1020_DIR_DOWN	= 7,
	FPC1020_DIR_LEFT_DOWN	= 8,
	FPC1020_DIR_SINGLE_CLICK,
	FPC1020_DIR_FINGER_DOWN};

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV	
int fpc1020_input_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	if (fpc1020->chip.type != FPC1020_CHIP_1020A) {
		dev_err(&fpc1020->spi->dev, "%s, chip not supported (%s)\n",
			__func__,
			fpc1020_hw_id_text(fpc1020));

		//return -EINVAL;
	}

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->input_dev = input_allocate_device();

	if (!fpc1020->input_dev) {
		dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		fpc1020->input_dev->name = "fp-keys";//FPC1020_DEV_NAME;

		set_bit(EV_KEY,		fpc1020->input_dev->evbit);

		set_bit(KEY_ENTER,	fpc1020->input_dev->keybit);
		set_bit(KEY_UP,		fpc1020->input_dev->keybit);
		set_bit(KEY_DOWN,	fpc1020->input_dev->keybit);
		set_bit(KEY_RIGHT,	fpc1020->input_dev->keybit);
		set_bit(KEY_LEFT,	fpc1020->input_dev->keybit);

		set_bit(FPC1020_KEY_FINGER_PRESENT, fpc1020->input_dev->keybit);

		error = input_register_device(fpc1020->input_dev);
	}

	if (error) {
		dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
		input_free_device(fpc1020->input_dev);
		fpc1020->input_dev = NULL;
	}

	return error;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
void fpc1020_input_destroy(fpc1020_data_t *fpc1020)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (fpc1020->input_dev != NULL)
		input_free_device(fpc1020->input_dev);
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
int fpc1020_input_enable(fpc1020_data_t *fpc1020, bool enabled)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->nav.enabled = enabled;

	return 0;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
#if 1

/* -------------------------------------------------------------------- */
int fpc1020_input_wait_finger_up(fpc1020_data_t *fpc1020)
{
	int error = 0;
	bool finger_up = false;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	while (!finger_up && (error >= 0)) {

		if (fpc1020->worker.stop_request && fpc1020->nav.enabled)
			error = -EINTR;
		else
			error = fpc1020_check_finger_present_raw(fpc1020);

		if ( error == 0 )
			finger_up = true;
		else if ( error < 0 )
		{
			dev_dbg(&fpc1020->spi->dev, "%s:fpc1020_check_finger_present_raw  error = %d\n", __func__,error);
		}
		else
		{
			dev_dbg(&fpc1020->spi->dev, "%s:enter input mode with key down (0x%.3X)!\n", __func__,error);
			msleep(1);
		}
	}

	fpc1020_read_irq(fpc1020, true);

	dev_dbg(&fpc1020->spi->dev, "%s: ret = %d \n", __func__,finger_up);

	return (finger_up) ? 0 : error;
}


void fpc1020_input_report_key(fpc1020_data_t *fpc1020,bool key_down)
{
	static bool last_key_down_flag = 0;

	dev_dbg(&fpc1020->spi->dev, "%s:key down = %d last key down = %d\n", __func__,key_down,last_key_down_flag);

	if( key_down == last_key_down_flag)
		return;
	
	last_key_down_flag = key_down ;
	
	input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, key_down);
	input_sync(fpc1020->input_dev);

	dev_info(&fpc1020->spi->dev,"%s:key %s\n", __func__,last_key_down_flag?"down":"up");
}

void fpc1020_input_report_key_downup(fpc1020_data_t *fpc1020)
{
	static bool last_key_down_flag = 0;
	
	input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 1);
	input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 0);
	input_sync(fpc1020->input_dev);

	dev_info(&fpc1020->spi->dev,"%s:key\n", __func__);
}


#define MAX_RETRY_CNT	100
int fpc1020_input_task(fpc1020_data_t *fpc1020)
{
	int error = 0;
	int finger_sum;
	int still_fading = 0;
	int dir = FPC1020_DIR_NONE,old_dir = FPC1020_DIR_NONE;
	int retry_cnt = 0,retry_cnt_ff = 0;
	int debaunce_cnt =0;
	static int threshold_min_value = -1;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);
	
#ifndef FPC1020_KEY_REPORT_CONTINUE
	// release key,send key_up event
	fpc1020_input_report_key(fpc1020,0);
#endif

re_start:
	still_fading = 0;
	debaunce_cnt = 0;
	error = fpc1020_write_nav_setup(fpc1020);
	if (!error)
		error = fpc1020_input_wait_finger_up(fpc1020);
	
	if( threshold_min_value < 0 )
	{		
		threshold_min_value = fpc1020_calc_finger_detect_threshold_min(fpc1020);
		error = fpc1020_set_finger_detect_threshold(fpc1020, threshold_min_value);		
		error = fpc1020_write_nav_setup(fpc1020);
	}

	
	while (!fpc1020->worker.stop_request &&
		fpc1020->nav.enabled && (error >= 0)) {

		finger_sum = fpc1020_check_finger_present_sum(fpc1020);
		if( finger_sum == 12 )
			retry_cnt_ff ++;
		else
			retry_cnt_ff = 0;

		/* Wait until a finger is detected before unleashing
		   the sensor polling loop */
		while (!finger_sum && !still_fading) {
			//error = fpc1020_wait_finger_present_lpm(fpc1020);
			dev_dbg(&fpc1020->spi->dev,"%s:wait finger ++\n", __func__);
			error = fpc1020_wait_finger_present(fpc1020);
			dev_dbg(&fpc1020->spi->dev,"%s:wait finger error = %d\n", __func__,error);
			if (!error) {
				finger_sum =
				fpc1020_check_finger_present_sum(fpc1020);
			} else
				break;
		}

		if (finger_sum < 0) {
			error = finger_sum;
		} else if (finger_sum > 0) {
			/*
			 * A finger was detected,
			 * set the fading variable so we are ready to navigate
			 * in case the user uses the sensor soon
			 * If each lap takes 50ms, a value of 40 means we will
			 * continuously poll the sensor in 2 seconds before
			 * waiting for a finger again
			*/
			still_fading = 5;//40;
		} else if (still_fading) {
			/*
			 * Count down the fading variable
			 * We will start waiting for a finger when zero is
			 * reached in the while-loop above
			*/
			--still_fading;
		}

		dev_dbg(&fpc1020->spi->dev,"%s:finger_sum = %d,error = %d ff = %d\n", __func__,finger_sum,error,retry_cnt_ff);
		
		if(!error)
		{
			//dir = !!finger_sum;
			if( finger_sum > FPC1020_INPUT_FINGER_DETECT_THRESHOLD )
				dir = FPC1020_DIR_FINGER_DOWN;
			if( finger_sum == 0 )
				dir = FPC1020_DIR_NONE;

			// debaunce
			if( old_dir == dir)
				 debaunce_cnt++;
			else
				debaunce_cnt = 0;

#ifndef FPC1020_KEY_REPORT_CONTINUE
			// debaunce_cnt < still_fading
			if( debaunce_cnt >= 1 )
				fpc1020_input_report_key(fpc1020,!!dir);
#else

			if( (old_dir != dir) && (dir == FPC1020_DIR_NONE))
				fpc1020_input_report_key_downup(fpc1020);
#endif

			old_dir = dir;

			/* yield time in order not to burn MIPS */
			msleep(10);
			
			retry_cnt = 0;

			if(retry_cnt_ff > 200 )
			{
				retry_cnt_ff = 0;
				error = -EIO;
			}
				
		}
	}

	if ((!fpc1020->worker.stop_request && fpc1020->nav.enabled )
		//&&  ( error == -EIO || error == -ETIMEDOUT )) {
		&&  ( error != -EINTR )) {
		//if(retry_cnt < MAX_RETRY_CNT)
		{
			retry_cnt ++;
			dev_err(&fpc1020->spi->dev, "%s retry_cnt (%d),error = %d\n",
				__func__,retry_cnt,error);
			msleep(10);
			goto re_start;
		}
	}

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
			"%s %s (%d)\n",
			__func__,
			(error == -EINTR) ? "TERMINATED" : "FAILED", error);
	}

	fpc1020->nav.enabled = false;

	return error;
}
#else
int fpc1020_input_task(fpc1020_data_t *fpc1020)
{
	int error = 0;
	int finger_raw;
	int still_fading = 0;
	int dir;
	unsigned int key_prim;
	unsigned int key_sec;
	fpc1020_clickevent_t clicked;
	fpc1020_navistate_t naviState;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	error = fpc1020_calc_finger_detect_threshold_min(fpc1020);

	if (error >= 0)
		error = fpc1020_set_finger_detect_threshold(fpc1020, error);

	if (!error)
		error = fpc1020_write_nav_setup(fpc1020);

	if (!error)
		naviState = naviStateMachine(&clicked, finger_raw, true);

	while (!fpc1020->worker.stop_request &&
		fpc1020->nav.enabled && (error >= 0)) {

		finger_raw = fpc1020_check_finger_present_raw(fpc1020);

		/* Wait until a finger is detected before unleashing
		   the sensor polling loop */
		while (!finger_raw && !still_fading) {
			error = fpc1020_wait_finger_present_lpm(fpc1020);
			if (!error) {
				finger_raw =
				fpc1020_check_finger_present_raw(fpc1020);
			} else
				break;
		}

		if (finger_raw < 0) {
			error = finger_raw;
		} else if (finger_raw > 0) {
			/*
			 * A finger was detected,
			 * set the fading variable so we are ready to navigate
			 * in case the user uses the sensor soon
			 * If each lap takes 50ms, a value of 40 means we will
			 * continuously poll the sensor in 2 seconds before
			 * waiting for a finger again
			*/
			still_fading = 40;
		} else if (still_fading) {
			/*
			 * Count down the fading variable
			 * We will start waiting for a finger when zero is
			 * reached in the while-loop above
			*/
			--still_fading;
		}

		dir = FPC1020_DIR_NONE;
		clicked = NoClick;
		naviState = NaviStateIdle;
		key_prim = key_sec = 0;

		if (!error) {
			naviState =
				naviStateMachine(&clicked, finger_raw, false);

			if (naviState == NaviStateNavigation) {
				dir = fpc1020_nav_process_raw(finger_raw);

				error = (dir < 0) ? dir : 0;

			} else {
				if (clicked == SingleClick)
					dir = FPC1020_DIR_SINGLE_CLICK;
				else if (clicked == FingerDownClick)
					dir = FPC1020_DIR_FINGER_DOWN;
			}
		}

		printk( "%s:dir = %d\n", __func__,dir);

		switch (dir) {
		case FPC1020_DIR_FINGER_DOWN:
			key_prim = FPC1020_KEY_FINGER_PRESENT;
			dev_dbg(&fpc1020->spi->dev,
				"%s Finger down\n", __func__);
			break;

		case FPC1020_DIR_SINGLE_CLICK:
			key_prim = FPC1020_KEY_CLICK;
			break;

		case FPC1020_DIR_LEFT:
			key_prim = KEY_LEFT;
			break;

		case FPC1020_DIR_UP:
			key_prim = KEY_UP;
			break;

		case FPC1020_DIR_RIGHT:
			key_prim = KEY_RIGHT;
			break;

		case FPC1020_DIR_DOWN:
			key_prim = KEY_DOWN;
			break;

		case FPC1020_DIR_LEFT_UP:
			key_prim = KEY_LEFT;
			key_sec = KEY_UP;
			break;

		case FPC1020_DIR_RIGHT_UP:
			key_prim = KEY_RIGHT;
			key_sec = KEY_UP;
			break;

		case FPC1020_DIR_RIGHT_DOWN:
			key_prim = KEY_RIGHT;
			key_sec = KEY_DOWN;
			break;

		case FPC1020_DIR_LEFT_DOWN:
			key_prim = KEY_DOWN;
			key_sec = KEY_LEFT;
			break;

		case FPC1020_DIR_NONE:
		default:
			break;
		}

#if 0
		if (key_prim) {
			input_report_key(fpc1020->input_dev, key_prim, 1);
			input_report_key(fpc1020->input_dev, key_prim, 0);
		}

		if (key_sec) {
			input_report_key(fpc1020->input_dev, key_sec, 1);
			input_report_key(fpc1020->input_dev, key_sec, 0);
		}

		if (key_prim || key_sec)
			input_sync(fpc1020->input_dev);
#else
		if(1)
		{
			static int old_dir = FPC1020_DIR_NONE;
			
			if (dir != FPC1020_DIR_NONE && old_dir == FPC1020_DIR_NONE ) {
				input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 1);
				//input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 0);
				input_sync(fpc1020->input_dev);
			}
			
			if (dir == FPC1020_DIR_NONE && old_dir != FPC1020_DIR_NONE ) {
				//input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 1);
				input_report_key(fpc1020->input_dev, FPC1020_KEY_FINGER_PRESENT, 0);
				input_sync(fpc1020->input_dev);
			}

			old_dir = dir;
		}
		
#endif

		/* yield time in order not to burn MIPS */
		if (!error)
			msleep(50);
	}

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
			"%s %s (%d)\n",
			__func__,
			(error == -EINTR) ? "TERMINATED" : "FAILED", error);
	}

	fpc1020->nav.enabled = false;

	return error;
}
#endif
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_write_nav_setup(fpc1020_data_t *fpc1020)
{
	const int mux = 0;
	int error = 0;
	u16 temp_u16;
	u32 temp_u32;
	fpc1020_reg_access_t reg;

	dev_dbg(&fpc1020->spi->dev, "%s %d\n", __func__, mux);

	error = fpc1020_wake_up(fpc1020);
	if (error)
		goto out;

	error = fpc1020_write_sensor_setup(fpc1020);
	if(error)
		goto out;
	
	// Reduce the time between finger detect queries in wait for finger query mode.
	temp_u32 = 0x1901FFFF;
	FPC1020_MK_REG_WRITE(reg, FPC1150_REG_FNGR_DET_CNTR, &temp_u32);
	error = fpc1020_reg_access(fpc1020, &reg);
	if (error)
		goto out;

	temp_u16 = fpc1020->setup.adc_shift[mux];
	temp_u16 <<= 8;
	temp_u16 |= fpc1020->setup.adc_gain[mux];

	FPC1020_MK_REG_WRITE(reg, FPC102X_REG_ADC_SHIFT_GAIN, &temp_u16);
	error = fpc1020_reg_access(fpc1020, &reg);
	if (error)
		goto out;

	temp_u16 = fpc1020->setup.pxl_ctrl[mux];
	temp_u16 |= FPC1020_PXL_BIAS_CTRL;
	FPC1020_MK_REG_WRITE(reg, FPC102X_REG_PXL_CTRL, &temp_u16);
	error = fpc1020_reg_access(fpc1020, &reg);
	if (error)
		goto out;

out:
	return error;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_nav_process_raw(int finger_raw)
{
	int dir = 0;
	int error = 0;
	/*
	 *  2  3  4
	 *   \ | /
	 *  1- 0 -5
	 *   / | \
	 *  8  7  6
	 *
	 * + click
	*/

	error = get_nav_dir(&dir, finger_raw);

	return (error < 0) ? error : dir;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fingerPosition(int *pCoordX,
			int *pCoordY,
			int fingerPresence)
{
/* fingerPosition function with hardcoded weights
 * int rowWeight[width]={-2,-1,1,2};
 * Weight associated to the presence of the finger in the Y direction
 * int colWeight[height]={2,0,-2};
*/

/* Directly from the read fingerQuery bytes. B0 is lowest byte and B1 is highest
* bytes:     highest    ,        lowest
*           --  B1 --   ,      --  B0 --
* bits:    11, 10, 9, 8 , 7, 6, 5, 4, 3, 2, 1, 0
*/

	/* if the FeedbackStatus is 0, then we have an error otherwise
	    we report 1 */
	int FeedbackStatus = 1;
	static const int xMapTable[] = {
			0, -4, -2, -6, 2, -2, 0, -4, 4, 0, 2, -2, 6, 2, 4, 0};
	static const int yMapTable[] = {
			-4, -2, -2, 0, -2, 0, 0, 2, -2, 0, 0, 2, 0, 2, 2, 4};
	static const int MaxValueX = 3 * 2 * 3;
	static const int MaxValueY = 2 * 2 * 4;

	int xG = 0;
	int yG = 0;

	int fingerPresenceB0 = fingerPresence & 0xffu;
	int fingerPresenceB1 = (fingerPresence >> 8) & 0xffu;

	/* First the x direction */
	xG += xMapTable[fingerPresenceB0 & 0x0f];
	xG += xMapTable[(fingerPresenceB0 >> 4) & 0x0f];
	xG += xMapTable[fingerPresenceB1 & 0x0f];

	/* Then the y direction */
	yG += yMapTable[fingerPresenceB0 & 0x0f];
	yG -= yMapTable[fingerPresenceB1 & 0x0f];
	yG <<= 1;

	/* Check validity range of the center of gravity */
	if (abs(xG) > MaxValueX)
		FeedbackStatus = 0;

	if (abs(yG) > MaxValueY)
		FeedbackStatus = 0;

	*pCoordX = xG;
	*pCoordY = yG;

	return FeedbackStatus;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fingerPosToDirection(int *direction, int CoordX, int CoordY)
{
	/* Indicate the direction base on the location of the finger
	*  The direction is indicated by a number as shown below
	*
	*      2  3  4
	*       \ | /
	*     1 - 0 - 5
	*       / | \
	*      8  7  6
	*/

	const int centerGravityThreshold = FPC1020_CENTER_GRAVITY_THRESHOLD;

	int dir = 0;
	static const int mapping[] = { FPC1020_DIR_NONE,
		FPC1020_DIR_RIGHT, FPC1020_DIR_LEFT, FPC1020_DIR_NONE,
		FPC1020_DIR_UP, FPC1020_DIR_RIGHT_UP, FPC1020_DIR_LEFT_UP,
		FPC1020_DIR_NONE, FPC1020_DIR_DOWN, FPC1020_DIR_RIGHT_DOWN,
		FPC1020_DIR_LEFT_DOWN, FPC1020_DIR_NONE, FPC1020_DIR_NONE,
		FPC1020_DIR_NONE, FPC1020_DIR_NONE, FPC1020_DIR_NONE};

	static const int MaxMappingLength = 16;

	int FeedbackStatus = 1;

	if (CoordX > centerGravityThreshold)
		dir |= 0x1; /* Set right bit */

	if (CoordX < -centerGravityThreshold)
		dir |= 0x2; /* Set left bit */

	if (CoordY > centerGravityThreshold)
		dir |= 0x4; /* Set up bit */

	if (CoordY < -centerGravityThreshold)
		dir |= 0x8; /* Set down bit */

	/* Check the range of the dir value */
	if (dir >= MaxMappingLength)
		FeedbackStatus = 0;

	*direction = mapping[dir];

	return FeedbackStatus;

}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int get_nav_dir(int *dir, int subarea_bits)
{
	/* Position of the fingerprint in the image where 0,0 coordinate is
	   center of the image */
	int CoordX, CoordY;
	int FeedbackStatus = 0;

	/* At this stage we should have all the input that the sensor should
	   provide to calculate the finger position */

	/* fingerPos = fingerPosition(fingerPresence,rowWeight,colWeight,
	   width,height,debugLevel); */

	FeedbackStatus = fingerPosition(&CoordX,
					&CoordY,
					subarea_bits);

	if (FeedbackStatus >= 0) {
		/* Map the found center of gravity to the directional
		   numbering */
		FeedbackStatus = fingerPosToDirection(dir, CoordX, CoordY);
	}

	return FeedbackStatus;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fingerDetectFuncBitReg(int fingerPresence)
{
/* Directly from the read fingerQuery bytes. B0 is lowest byte and B1 is highest
* bytes:     highest    ,        lowest
*           --  B1 --   ,      --  B0 --
* bits:    11, 10, 9, 8 , 7, 6, 5, 4, 3, 2, 1, 0
*/
	int subAreaPresent = 0;

	int fingerPresenceB0 = fingerPresence & 0xffu;
	int fingerPresenceB1 = (fingerPresence >> 8) & 0xffu;

	/*
	 * ------------------------
	 * |                      |
	 * |   0    1    2    3   |
	 * |                      |
	 * |   4    5    6    7   |
	 * |                      |
	 * |   8    9    10   11  |
	 * |                      |
	 * ------------------------
	*/

	/* Check that there is finger on the sub-areas
	   excluding 0, 3, 8 and 11 */
	subAreaPresent += (fingerPresenceB0 >> 1) & 0x1; /* 1  */
	subAreaPresent += (fingerPresenceB0 >> 2) & 0x1; /* 2  */
	subAreaPresent += (fingerPresenceB0 >> 4) & 0x1; /* 4  */
	subAreaPresent += (fingerPresenceB0 >> 5) & 0x1; /* 5  */
	subAreaPresent += (fingerPresenceB0 >> 6) & 0x1; /* 6  */
	subAreaPresent += (fingerPresenceB0 >> 7) & 0x1; /* 7  */
	subAreaPresent += (fingerPresenceB1 >> 1) & 0x1; /* 9  */
	subAreaPresent += (fingerPresenceB1 >> 2) & 0x1; /* 10 */

	 /* Threshold for determine that a finger is present */
	return (subAreaPresent >= 2) ? 1 : 0;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static fpc1020_navistate_t naviStateMachine(fpc1020_clickevent_t *clicked,
						int fingerPresence,
						bool reset)
{
	static fpc1020_navistate_t naviState;

	/* Parameters for specifying the timer/counter sizes */
	static const int MaxPressCnt = 2;
	static const int MaxReleaseCnt = 3;

	/* Counters for short and long press */
	static int Click1PressCnt;
	static int Click1ReleaseCnt;
	static int Click2PressCnt;

	static int LastZoneData;

	int fingerDetect;

	if (reset){
		Click1PressCnt   =
		Click1ReleaseCnt =
		Click2PressCnt   =
		LastZoneData     = 0;

		naviState = NaviStateIdle;

		return naviState;
	}

	fingerDetect = fingerDetectFuncBitReg(fingerPresence);

	*clicked = NoClick;

	switch (naviState) {
	case NaviStateIdle:
		if (fingerDetect) {
			/* Start possible click cycle */
			*clicked = FingerDownClick;
			Click1PressCnt = 0;
			naviState = NaviStateClick1Press;
			LastZoneData = fingerPresence;
		} else {
			naviState = NaviStateIdle;
		}
		break;

	case NaviStateClick1Press: /* Potential 1 click pressed state */
		Click1PressCnt++;
		if (!fingerDetect) {
			naviState = NaviStateClick1Release;
			Click1ReleaseCnt = 0;
		} else {
			/* Threshold/timing for the first part of the click */
			if (Click1PressCnt >= MaxPressCnt) {
				/* Finger is still pressed -
				   going to navigation state */
				naviState = NaviStateNavigation;
			} else {
				/* Try to detect if the user is trying to
				   navigate as early as possible */
				if (LastZoneData != fingerPresence)
					naviState = NaviStateNavigation;
			}
		}
		break;

	case NaviStateClick1Release: /* 1 click released */
		Click1ReleaseCnt++;
		if (fingerDetect) {
			/* Possible start of second click in double click */
			naviState = NaviStateClick2Press;
			Click2PressCnt = 0;
		} else {
			if (Click1ReleaseCnt >= MaxReleaseCnt) {
				/* single click */
				*clicked = SingleClick;
				naviState = NaviStateIdle;
			} else
				naviState = NaviStateClick1Release;
		}
		break;

	case NaviStateClick2Press: /* Long press state */
		if (Click2PressCnt == 0) {
			/* Double click detected, inform caller */
			*clicked = DoubleClick;
		}
		Click2PressCnt++;
		if (!fingerDetect) {
			naviState = NaviStateIdle;
		} else {
			if (Click2PressCnt >= MaxPressCnt) {
				/* Finger is still pressed -
				   going to navigation state */
				naviState = NaviStateNavigation;
			} else {
				naviState = NaviStateClick2Press;
			}
		}
		break;

	case NaviStateNavigation:
		/* Navigation state */
		if (!fingerDetect) {
			/* Finger released go back to idle */
			naviState = NaviStateIdle;
		} else {
			/* Navigation */
			naviState = NaviStateNavigation;
		}
		break;

	default:
		break;
	}

	LastZoneData = fingerPresence;

	return naviState;
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_write_lpm_setup(fpc1020_data_t *fpc1020)
{
	return fpc1020_write_sensor_setup(fpc1020);
}
#endif


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_wait_finger_present_lpm(fpc1020_data_t *fpc1020)
{
	const int lpm_poll_delay_ms = FPC1020_INPUT_POLL_TIME_MS;
	const int zmask_5 = 1 << 5;
	const int zmask_6 = 1 << 6;
	const int zmask_ext = FPC1020_FINGER_DETECT_ZONE_MASK;

	int error = 0;
	int zone_raw = 0;

	bool wakeup_center = false;
	bool wakeup_ext    = false;
	bool wakeup        = false;

	error = fpc1020_write_lpm_setup(fpc1020);

	if (!error) {
		error = fpc1020_sleep(fpc1020, false);

		if (error == -EAGAIN) {
			error = fpc1020_sleep(fpc1020, false);

			if (error == -EAGAIN)
				error = 0;
		}
	}

	while (!fpc1020->worker.stop_request && !error && !wakeup) {
		if (!error)
			error = fpc1020_wait_finger_present(fpc1020);

		if (!error)
			error = fpc1020_check_finger_present_raw(fpc1020);

		zone_raw = (error >= 0) ? error : 0;

		if (error >= 0) {
			error = 0;

			wakeup_center = (zone_raw & zmask_5) ||
					(zone_raw & zmask_6);

 			/* Todo: refined extended processing ? */
			wakeup_ext = ((zone_raw & zmask_ext) == zmask_ext);

		} else {
			wakeup_center =
			wakeup_ext    = false;
		}

		if (wakeup_center && wakeup_ext) {
			dev_dbg(&fpc1020->spi->dev,
				"%s Wake up !\n", __func__);
			wakeup = true;
		}
		if (!wakeup && !error) {
			error = fpc1020_sleep(fpc1020, false);

			if (error == -EAGAIN)
				error = 0;

			if (!error)
				msleep(lpm_poll_delay_ms);
		}
	}

	if (!error)
		error = fpc1020_write_nav_setup(fpc1020);

	if (error < 0)
		dev_dbg(&fpc1020->spi->dev,
			"%s FAILED %d!\n", __func__,
			error);

	return error;
}
#endif

