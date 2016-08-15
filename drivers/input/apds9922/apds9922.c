/*
 *  apds9921.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2015 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2015 Avago Technologies
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
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include "apds9922.h"
struct apds9921_data *apds_data;
extern void apds_create_dbg_sys(struct kobject *kobj);
extern const struct dev_pm_ops ps_pm_ops;
/*
 * Global data
 */
static struct i2c_client *apds9921_i2c_client; /* global i2c_client to support ioctl */
static struct workqueue_struct *apds_workqueue;

static unsigned short apds9921_als_meas_rate_tb[] = {25, 50, 100, 200, 400};
static unsigned int apds9921_als_res_tb[] = { 65535, 131071, 262143, 524287, 1048575 };
static unsigned char apds9921_als_gain_tb[] = { 1, 3, 6, 9, 18 };
static unsigned char apds9921_als_gain_bit_tb[] = { 0x00, 0x01, 0x02, 0x03, 0x04 };

#define OPEN_B1_PCB_VERSION		0x03
#define OPEN_B2_PCB_VERSION		0x04//telcom B1
#define OPEN_NPI_PCB_VERSION		0x05//Flash ID(GPIO248): 1 MP, 0 NPI 
#define OPEN_MP_PCB_VERSION		0x06
#define TELCOM_B1_PCB_VERSION		0x0C//telcom B1
#define TELCOM_MP_PCB_VERSION		0x0D//telcom MP

extern int get_hw_version(void);

int apds9921_set_als_poll_delay(struct i2c_client *client, unsigned int val);
/*
 * Management functions
 */

static int apds9921_dd_set_main_ctrl(struct i2c_client *client, int main_ctrl)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR, main_ctrl);
}

static int apds9921_dd_set_prx_meas_rate(struct i2c_client *client, int prx_meas)
{	
	return i2c_smbus_write_byte_data(client, APDS9921_DD_PRX_MEAS_RATE_ADDR, prx_meas);
}

static int apds9921_dd_set_als_meas_rate(struct i2c_client *client, int als_meas)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_ALS_MEAS_RATE_ADDR, als_meas);
}

static int apds9921_dd_set_als_gain(struct i2c_client *client, int als_gain)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_ALS_GAIN_ADDR, als_gain);
}

static int apds9921_dd_set_pers(struct i2c_client *client, int pers)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_INT_PERSISTENCE_ADDR, pers);
}

static int apds9921_dd_set_prx_led(struct i2c_client *client, int prx_led)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_PRX_LED_ADDR, prx_led);
}

static int apds9921_dd_set_prx_pulses(struct i2c_client *client, int prx_pulses)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_PRX_PULSES_ADDR, prx_pulses);
}

 int apds9921_dd_set_prx_can(struct i2c_client *client, int prx_can)
{
	return i2c_smbus_write_word_data(client, APDS9921_DD_PRX_CAN_ADDR, prx_can);
}

static int apds9921_dd_set_int_cfg(struct i2c_client *client, int int_cfg)
{
	return i2c_smbus_write_byte_data(client, APDS9921_DD_INT_CFG_ADDR, int_cfg);
}

static int apds9921_dd_set_prx_thresh(struct i2c_client *client, int thres_low, int thres_up)
{
	i2c_smbus_write_word_data(client, APDS9921_DD_PRX_THRES_UP_ADDR, thres_up);
	
	return i2c_smbus_write_word_data(client, APDS9921_DD_PRX_THRES_LOW_ADDR, thres_low);
}

static int LuxCalculation(struct i2c_client *client, int clr_data, int als_data)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	unsigned int luxValue=0;


	luxValue = ((als_data*APDS9921_DD_LUX_FACTOR*data->als_cal_factor))/((apds9921_als_meas_rate_tb[data->als_res_index])*apds9921_als_gain_tb[data->als_gain_index]);

	// Apply calibration factor 
//	luxValue = (luxValue*data->als_cal_factor)/100;

#if 0
	INFOR("clr_data:%d, als_data:%d, meas_rate:%d, gain:%d, cal_factor:%d, lux:%d\n",
		  clr_data, als_data, apds9921_als_meas_rate_tb[data->als_res_index], apds9921_als_gain_tb[data->als_gain_index],
		  data->als_cal_factor, luxValue);
#endif

	if( luxValue<100 ) {
		luxValue = luxValue*10/16;
	} else if( luxValue>500 ) {
		luxValue = luxValue*10/19;
	}

	return (int)luxValue;
}

static void apds9921_change_ps_threshold(struct i2c_client *client)
{
	unsigned int thred_up,thred_low;
	struct apds9921_data *data = i2c_get_clientdata(client);

	thred_up = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_THRES_UP_ADDR);
	thred_low = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_THRES_LOW_ADDR);

	data->ps_data =	i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR);
	data->ps_overflow = data->ps_data >> 11;
	data->ps_data = data->ps_data & 0x7FF;

	INFOR("ps_data=0x%x overflow=%d thred_up:0x%x,thred_low:0x%x\n", data->ps_data, data->ps_overflow, thred_up,thred_low);

	if (data->enable_ps_sensor == APDS_ENABLE_PS_WITH_INT)
	{
#if 0
		if (data->first_int_enable == 1) {
			data->first_int_enable = 2;
			data->ps_detection = 0;

			input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);/* NEAR-to-FAR detection */
			input_sync(data->input_dev_ps);
		
			data->ps_data += 10;
			if (data->ps_data >= 255) data->ps_data = 255;

			apds9921_dd_set_prx_thresh(client, data->ps_data, data->ps_data);
		
			return;
		}
		else if (data->first_int_enable == 2) {
			data->first_int_enable = 0;	
			data->ps_detection = 0;
			apds9921_dd_set_prx_thresh(client, data->ps_threshold, data->ps_threshold);
		
			return;
		}
#endif
		if ( data->ps_detection == 1 ) {
			/* far-to-near detected */

			input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);/* FAR-to-NEAR detection */	
			input_sync(data->input_dev_ps);
			data->ps_state_pre = NEAR;


			INFOR("*****************Near***************** detected, pdata:%d, overflow:%d\n", data->ps_data,data->ps_overflow);
		}
		else {
			/* near-to-far detected */

			input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);/* NEAR-to-FAR detection */
			input_sync(data->input_dev_ps);
			data->ps_state_pre = FAR;


			INFOR("*****************Far***************** detected, pdata:%d, overflow:%d\n", data->ps_data,data->ps_overflow);
		}
	}
	else
	{

		INFOR("ps with no irq,pdata:%d, threshold:%d, hys_thr:%d\n",
			  data->ps_data, data->ps_threshold, data->ps_hysteresis_threshold);
		if (data->ps_data >= data->ps_threshold) {
		
			if (data->ps_detection == 0)
			{
				/* far-to-near detected */
				data->ps_detection = 1;
				input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);/* FAR-to-NEAR detection */	
				input_sync(data->input_dev_ps);

				INFOR("far-to-near detected\n");
			}	
		}
		else if (data->ps_data < data->ps_hysteresis_threshold) {
			
			if (data->ps_detection == 1)
			{
				/* near-to-far detected */
				data->ps_detection = 0;
				input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);/* NEAR-to-FAR detection */
				input_sync(data->input_dev_ps);

				INFOR("near-to-far detected\n");
			}
		}
	}
}

static void apds9921_change_als_threshold(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int clr_data, als_data;
	int luxValue=0;
	unsigned char change_again=0;

	clr_data = i2c_smbus_read_word_data(client, APDS9921_DD_CLEAR_DATA_ADDR);
	als_data = i2c_smbus_read_word_data(client, APDS9921_DD_ALS_DATA_ADDR);

	luxValue = LuxCalculation(client, clr_data, als_data);

//	INFOR("lux=%d clr_data=%d als_data=%d again=%d als_res=%d\n", luxValue, clr_data, als_data, apds9921_als_gain_tb[data->als_gain_index], data->als_res_index);
	
	data->als_data = als_data;
	
	data->als_threshold_l = (apds9921_als_res_tb[data->als_res_index] * APDS9921_ALS_THRESHOLD_HSYTERESIS ) /100;
	data->als_threshold_h = (apds9921_als_res_tb[data->als_res_index] * (100-APDS9921_ALS_THRESHOLD_HSYTERESIS) ) /100;

	if (data->als_data >= data->als_threshold_h) {
		// lower AGAIN if possible
		if (data->als_gain_index != APDS9921_DD_ALS_GAIN_1X) {
			data->als_gain_index--;
			change_again = 1;
		}
	}
	else if (data->als_data < data->als_threshold_l) {
		// increase AGAIN if possible
		if (data->als_gain_index != APDS9921_DD_ALS_GAIN_18X) {
			data->als_gain_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		apds9921_dd_set_als_gain(client, apds9921_als_gain_bit_tb[data->als_gain_index]);

		return;
	}

	// report to HAL
//	luxValue = (luxValue>30000) ? 30000 : luxValue;
	if( (luxValue!=0) && (data->als_prev_lux==luxValue) ) {
		luxValue++;
	}
	data->als_prev_lux = luxValue;


	input_report_abs(data->input_dev_als, ABS_MISC, luxValue); // report the lux level
	input_sync(data->input_dev_als);

//	INFOR("report to HAL, Lux = %d\n", luxValue);
}

#if 0
static void apds9921_reschedule_work(struct apds9921_data *data,
					  unsigned long delay)
{
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	INFOR("\n");

	cancel_delayed_work(&data->dwork);
	queue_delayed_work(apds_workqueue, &data->dwork, delay);

}
#endif

/* ALS polling routine */
static void apds9921_als_polling_work_handler(struct work_struct *work)
{
	struct apds9921_data *data = container_of(work, struct apds9921_data, als_dwork.work);
	struct i2c_client *client=data->client;
	int status;
	int loop_count=0;
	
	//INFOR("==> ALS Pollinig\n");
	
	status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);

	//INFOR("status = %x\n", status);
	
	if ((status & APDS9921_DD_ALS_DATA_STATUS) && data->enable_als_sensor) {
		apds9921_change_als_threshold(client);
	}
	else
	{
		/* wait the most another 20ms */
		while (!(status & APDS9921_DD_ALS_DATA_STATUS) && loop_count < 10)
		{
			status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);

			mdelay(2);
			loop_count++;
		}
	
		if (loop_count < 10)
		{
			apds9921_change_als_threshold(client);
		}
	}
	
	queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));	// restart timer
}

/* PS polling routine */
static void apds9921_ps_polling_work_handler(struct work_struct *work)
{
	struct apds9921_data *data = container_of(work, struct apds9921_data, ps_dwork.work);
	struct i2c_client *client=data->client;
	int status;
	int loop_count=0;
	
	//INFOR("==> PS Pollinig\n");

//	mutex_lock(&apds_data->mutex_apds);

	status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);

	//INFOR("status = %x\n", status);
	
	if ((status & APDS9921_DD_PRX_DATA_STATUS) && data->enable_ps_sensor) {
		apds9921_change_ps_threshold(client);
	}
	else {
		/* wait the most another 20ms */
		while (!(status & APDS9921_DD_ALS_DATA_STATUS) && loop_count < 10)
		{
			status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);

			mdelay(2);
			loop_count++;
		}
	
		if (loop_count < 10)
		{	
			apds9921_change_ps_threshold(client);
		}
	}
	
	queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));	// restart timer

//	mutex_unlock(&apds_data->mutex_apds);
}

/* Interrupt Service Routine */
static void apds9921_work_handler(struct work_struct *work)
{
	struct apds9921_data *data = container_of(work, struct apds9921_data, dwork.work);
	struct i2c_client *client=data->client;
	int status;

	mutex_lock(&data->mutex_apds);

	status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);//reg:0x07;
																			// 4:als irq stat
																			// 2:ps logic signal stat
																			// 1:ps irq stat

	INFOR("status = %x,wakeen:%d,non-wakeen:%d,en:%d\n",
		  status,data->wakeup_enable,data->non_wakeup_enable,data->enable_ps_sensor);

	if ((status & APDS9921_DD_PRX_INT_STATUS) && data->enable_ps_sensor) { // bit1,ps irq received
		/* PS interrupt */
		data->ps_detection = (status>>2)&0x01;
		apds9921_change_ps_threshold(client);			
	}
	
	if ((status & APDS9921_DD_ALS_INT_STATUS) && data->enable_als_sensor) { // bit4;als irq received
		/* ALS interrupt */	
		apds9921_change_als_threshold(client);
	}

	//modified by bsp@meizu.com
	enable_irq( data->irq );

	mutex_unlock(&data->mutex_apds);
}

/* assume this is ISR */
static irqreturn_t apds9921_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds9921_data *data = i2c_get_clientdata(client);
	int status,distance=FAR;

	mutex_lock(&data->mutex_apds);

	status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);//reg:0x07;
	// 4:als irq stat
	// 2:ps logic signal stat
	// 1:ps irq stat

	if ( (status>>2)&0x01 ) { // bit2,ps logic signal status; 1:close,0:far
		distance = NEAR;
	} else {
		distance = FAR;
	}

	INFOR("status = %x,wakeen:%d,non-wakeen:%d,en:%d,pre_dis:%d,dis:%d,gpio:%d\n",
		  status,data->wakeup_enable,data->non_wakeup_enable,data->enable_ps_sensor,data->ps_state_pre,distance,gpio_get_value(data->gpio));

	if ( (distance!=data->ps_state_pre) && data->enable_ps_sensor) { // bit1,ps irq received
		/* PS interrupt */
		data->ps_detection = (status>>2)&0x01;
		apds9921_change_ps_threshold(client);
	}


	irq_enable(irq_to_desc(data->irq));
	mutex_unlock(&data->mutex_apds);

	return IRQ_HANDLED;
}

/*
 * IOCTL support
 */

 int apds9921_enable_als_sensor(struct i2c_client *client, int val)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int main_ctrl;
	int int_cfg;

	INFOR("%s: enable als sensor ( %d)\n", __func__, val);
	
	if ((val != APDS_DISABLE_ALS) && (val != APDS_ENABLE_ALS_WITH_INT) && (val != APDS_ENABLE_ALS_NO_INT) && (val != APDS_ENABLE_ALS_CALIBRATION)) {
		INFOR("%s: enable als sensor=%d\n", __func__, val);
		return -1;
	}

	main_ctrl = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR);
	if (main_ctrl < 0) {
		INFOR("read main ctrl addr failed\n");
		return -2;
	}

	int_cfg = i2c_smbus_read_byte_data(client, APDS9921_DD_INT_CFG_ADDR);
	if (int_cfg < 0) {
		return -3;
	}
	
	if ((val == APDS_ENABLE_ALS_WITH_INT) || (val == APDS_ENABLE_ALS_NO_INT)) {
		// turn on light  sensor
		if (data->enable_als_sensor==APDS_DISABLE_ALS) {

			data->enable_als_sensor = val;
				
			if (data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT) {	

				apds9921_dd_set_int_cfg(client, (int_cfg&0x13));
				apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_ALS_EN)|APDS9921_DD_ALS_EN);

				/*
				 * If work is already scheduled then subsequent schedules will not
				 * change the scheduled time that's why we have to cancel it first.
				 */
				cancel_delayed_work(&data->als_dwork);
				flush_delayed_work(&data->als_dwork);
				queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
			}
			else {	// als with int

				apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_ALS_EN));
				apds9921_dd_set_int_cfg(client, (int_cfg&0x03)|APDS9921_DD_ALS_INT_EN|APDS9921_DD_ALS_VAR_MODE|APDS9921_DD_ALS_INT_SEL_ALS);

				apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_ALS_EN)|APDS9921_DD_ALS_EN);				
				/*
				 * If work is already scheduled then subsequent schedules will not
				 * change the scheduled time that's why we have to cancel it first.
				 */
				cancel_delayed_work(&data->als_dwork);
				flush_delayed_work(&data->als_dwork);
			}
		}
	}
	else if (val == APDS_ENABLE_ALS_CALIBRATION)
	{
		// Make sure both ALS and PS are in disabled mode
		if (data->enable_als_sensor==APDS_DISABLE_ALS) {
			int i;
			int clr_data, als_data, luxValue=0;
			int als_meas_rate;

			als_meas_rate = i2c_smbus_read_byte_data(client, APDS9921_DD_ALS_MEAS_RATE_ADDR);
			if (als_meas_rate < 0)
				return -4;

			apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_ALS_EN));
			apds9921_dd_set_als_meas_rate(client, APDS9921_DD_ALS_MEAS_RES_16_BIT|APDS9921_DD_ALS_MEAS_RATE_25_MS);			
			apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_ALS_EN)|APDS9921_DD_ALS_EN);
			
			for (i=0; i<APDS9921_ALS_CAL_LOOP; i++) {
				mdelay(25); // this is following als_meas_rate
				clr_data = i2c_smbus_read_word_data(client, APDS9921_DD_CLEAR_DATA_ADDR);
				als_data = i2c_smbus_read_word_data(client, APDS9921_DD_ALS_DATA_ADDR);
	
				luxValue += LuxCalculation(client, clr_data, als_data);						
			}

			apds9921_dd_set_als_meas_rate(client, als_meas_rate);			
			apds9921_dd_set_main_ctrl(client, main_ctrl);
			
			luxValue = luxValue/i;
			INFOR("APDS9921_ALS_CAL lux = %d\n", luxValue);

  	    	if ((luxValue >= APDS9921_ALS_CAL_LUX_LOW) &&	// adjust if within +/- 30%
    	        (luxValue <= APDS9921_ALS_CAL_LUX_HIGH)) {
        		data->als_cal_factor = (APDS9921_ALS_CAL_LUX * 100)/luxValue;

				INFOR("als cali ok, lux:%d, cali_factor:%d\n", luxValue, data->als_cal_factor);
				data->als_cali_result = CALI_OK;
      		}
			else {
				INFOR("als cali failed, lux:%d not in range\n",luxValue);
				data->als_cali_result = CALI_FAIL;
				return -5;
			}
		}
		else {
			INFOR("Err: sensor is in active mode, cali failed\n");
			data->als_cali_result = CALI_FAIL;
			return -6;
		}
	}
	else {
		//turn off light sensor
		data->enable_als_sensor = APDS_DISABLE_ALS;

		apds9921_dd_set_int_cfg(client, (int_cfg&0x13));
		apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_ALS_EN));

		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->als_dwork);
		flush_delayed_work(&data->als_dwork);
	}

	return 0;
}

 int apds9921_set_als_poll_delay(struct i2c_client *client, unsigned int val)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int ret = 0;
	int als_res_index=0;
	int als_meas_rate;

	INFOR("val:%d\n", val);

	if (   (val != APDS_ALS_POLL_25MS) && (val != APDS_ALS_POLL_50MS)
		&& (val != APDS_ALS_POLL_100MS) && (val != APDS_ALS_POLL_200MS) && (val != APDS_ALS_POLL_500MS) ) {
		INFOR("%s:invalid value=%d\n", __func__, val);
	}

	switch( val ) {
		case APDS_ALS_POLL_25MS:
			data->als_poll_delay = 25;		// 25ms
			als_res_index = APDS9921_DD_ALS_RES_16BIT;
			als_meas_rate = APDS9921_DD_ALS_MEAS_RES_16_BIT|APDS9921_DD_ALS_MEAS_RATE_25_MS;
			break;
		case APDS_ALS_POLL_50MS:
			data->als_poll_delay = 50;		// 50ms
			als_res_index = APDS9921_DD_ALS_RES_17BIT;
			als_meas_rate = APDS9921_DD_ALS_MEAS_RES_17_BIT|APDS9921_DD_ALS_MEAS_RATE_50_MS;
			break;
		case APDS_ALS_POLL_100MS:
			data->als_poll_delay = 100;		// 100ms
			als_res_index = APDS9921_DD_ALS_RES_18BIT;
			als_meas_rate = APDS9921_DD_ALS_MEAS_RES_18_BIT|APDS9921_DD_ALS_MEAS_RATE_100_MS;
			break;
		case APDS_ALS_POLL_200MS:
			data->als_poll_delay = 250;		// 200ms, should be less than 300ms
			als_res_index = APDS9921_DD_ALS_RES_19BIT;
			als_meas_rate = APDS9921_DD_ALS_MEAS_RES_19_BIT|APDS9921_DD_ALS_MEAS_RATE_200_MS;
			break;
		case APDS_ALS_POLL_500MS:
			data->als_poll_delay = 500;		// 500ms ==> can be 1000ms
			als_res_index = APDS9921_DD_ALS_RES_20BIT;
			als_meas_rate = APDS9921_DD_ALS_MEAS_RES_20_BIT|APDS9921_DD_ALS_MEAS_RATE_500_MS;
			break;
		default:
			data->als_poll_delay = 100;		// 100ms
			als_res_index = APDS9921_DD_ALS_RES_18BIT;
			als_meas_rate = APDS9921_DD_ALS_MEAS_RES_18_BIT|APDS9921_DD_ALS_MEAS_RATE_100_MS;
			break;
	}

	ret = apds9921_dd_set_als_meas_rate(client, als_meas_rate);

	if (ret < 0) {
		INFOR("set measure rate failed, val:%d,delay:%d\n", val,data->als_poll_delay);
		return -2;
	}
	else {
		data->als_res_index = als_res_index;
		INFOR("poll delay %d, als_res_index %d, meas_rate %x\n", data->als_poll_delay, als_res_index, als_meas_rate);
	}
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->als_dwork);
	flush_delayed_work(&data->als_dwork);
	queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
	
	return 0;
}

int apds9921_enable_ps_sensor(struct i2c_client *client, unsigned int val)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int main_ctrl = 0;
	int int_cfg;
	int value = 0,status = 0,handle = 0,distance = FAR;
	unsigned int pdata=0,thred_up=0x64,thred_low=0x35;
	struct timex  txc;
	struct rtc_time tm;


	value = val;
	val = value&0xff;
	handle = (value&0xff00)>>8;
	INFOR("val:0x%.4x, handle:0x%x, en:0x%x\n", value,handle,val);

	do_gettimeofday(&(txc.time));
	rtc_time_to_tm(txc.time.tv_sec,&tm);
	INFOR("time :%d-%.2d-%.2d %.2d:%.2d:%.2d\n",tm.tm_year+1900,tm.tm_mon+1, tm.tm_mday,tm.tm_hour+8,tm.tm_min,tm.tm_sec);


	main_ctrl = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR);//bit4 1:sw reset
	if (main_ctrl < 0) {
		INFOR("read main ctrl reg failed, enable failed\n");
		return -2;
	}

	int_cfg = i2c_smbus_read_byte_data(client, APDS9921_DD_INT_CFG_ADDR);
	if (int_cfg < 0) {
		INFOR("read irq cfg reg failed, enable failed\n");
		return -3;
	}

	if(val == APDS_ENABLE_PS_WITH_INT) {

		//if ps pre state is disable, turn on p sensor
		if (data->enable_ps_sensor==APDS_DISABLE_PS) {
		
			data->enable_ps_sensor = val;
			data->first_int_enable = 1;
			
			apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_PRX_EN));
		
			// force first interrupt to inform HAL
//			apds9921_dd_set_prx_thresh(client, 0, 0);
			
			apds9921_dd_set_int_cfg(client, (int_cfg&0xFC)|APDS9921_DD_PRX_INT_EN|APDS9921_DD_PRX_LOGIC_MODE);

			apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_PRX_EN)|APDS9921_DD_PRX_EN);
			
//			cancel_delayed_work(&data->ps_dwork);
//			flush_delayed_work(&data->ps_dwork);

			if( data->tptype ) {
				apds9921_dd_set_prx_thresh(client, THREDHOLD_LOW, THREDHOLD_UP);
			} else  {
				apds9921_dd_set_prx_thresh(client, THREDHOLD_LOW_W, THREDHOLD_UP_W);
			}

			irq_enable(irq_to_desc(data->irq));
			INFOR("enable ps with irq ok\n");

		}
		status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);//reg:0x07;
		// 4:als irq stat
		// 2:ps logic signal stat
		// 1:ps irq stat

		pdata = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR);// pdata read before ps en is not right,useless
		thred_up = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_THRES_UP_ADDR);
		thred_low = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_THRES_LOW_ADDR);

		INFOR("pdata:0x%x, thred_up:0x%x, thred_low:0x%x, main_ctrl:0x%x,main_status:0x%x,tptype:%d\n",pdata, thred_up, thred_low, main_ctrl,status,data->tptype);


		distance = gpio_get_value(data->gpio);
		if ( distance ) { // gpio value
			INFOR("ps is far\n");
			distance = FAR;
		} else {
			INFOR("ps is near\n");
			distance = NEAR;
		}
		INFOR("go to enable ps with irq, val:%d,wakeen:%d,non-wakeen:%d,en:%d,distance:%d,pre_dis:%d,tptype:%d\n",
			  val,data->wakeup_enable,data->non_wakeup_enable,data->enable_ps_sensor,distance,data->ps_state_pre,data->tptype);

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, -1);
		input_sync(data->input_dev_ps);
		usleep_range(30000,30000);
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, distance);
		input_sync(data->input_dev_ps);
		data->ps_state_pre = distance;

		if( handle==ID_PX ) {
			data->non_wakeup_enable = 1;
		} else if( handle==ID_PXW ) {
			data->wakeup_enable = 1;
		} else {
			data->non_wakeup_enable = 1;
		}

	}
	else if(0 /*val == APDS_ENABLE_PS_NO_INT*/) {

		data->enable_ps_sensor = val;
		apds9921_dd_set_int_cfg(client, (int_cfg&0xFC));
		apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_PRX_EN)|APDS9921_DD_PRX_EN);

		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->ps_dwork);
		flush_delayed_work(&data->ps_dwork);
		queue_delayed_work(apds_workqueue, &data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));

		INFOR("enable ps with no irq\n");
	}
	else if (val == APDS_ENABLE_PS_CALIBRATION) {	// calibrate crosstalk value

		if (1 /*data->enable_ps_sensor==APDS_DISABLE_PS*/) {
			int i;
			int ps_data=0;
			int ps_meas_rate;

			ps_meas_rate = i2c_smbus_read_byte_data(client, APDS9921_DD_PRX_MEAS_RATE_ADDR);
			if (ps_meas_rate < 0)
				return -4;

			apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_PRX_EN));
			apds9921_dd_set_prx_meas_rate(client, 0x40|APDS9921_DD_PRX_MEAS_RES_11_BIT|APDS9921_DD_PRX_MEAS_RATE_6_25_MS);
			apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_PRX_EN)|APDS9921_DD_PRX_EN);

			// modified by bsp meizu
			apds9921_dd_set_prx_can(client, 0x00); // clear ps offset reg, read real pdata

			for (i=0; i<APDS9921_PS_CAL_LOOP; i++) {
				mdelay(10);	// must be greater than prx meas rate
				ps_data += i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR);
			}

			apds9921_dd_set_prx_meas_rate(client, ps_meas_rate);
			apds9921_dd_set_main_ctrl(client, main_ctrl);
			
			ps_data = ps_data/i;
			INFOR("APDS9921_PS_CAL pdata = %d\n", ps_data);

			if ( 0/*(ps_data <= APDS9921_PS_CAL_CROSSTALK_HIGH) &&	// don't adjust if within
    	      	(ps_data >= APDS9921_PS_CAL_CROSSTALK_LOW)*/ ) {
    	    	data->ps_threshold = ps_data+APDS9921_PS_CROSSTALK_DELTA;
				data->ps_hysteresis_threshold = data->ps_threshold - 10;
				
				apds9921_dd_set_prx_thresh(client, data->ps_threshold, data->ps_threshold);

				INFOR("no need to adjust, threshold:%d, pdata:%d\n", data->ps_threshold,ps_data);
   		   	}
			else if (ps_data > APDS9921_PS_CAL_CROSSTALK_HIGH)
			{
				if (ps_data <= PS_ADC_11_BIT)
				{					
//					data->ps_offset = ps_data - APDS9921_PS_CAL_CROSSTALK_HIGH;
					data->ps_offset = ps_data - 1;
					apds9921_dd_set_prx_can(client, data->ps_offset); // write offset to reg:0x1f, offset reg

					INFOR("APDS9921_PS_CAL ps_offset = 0x%.4x\n", data->ps_offset);
					
//	    	    	data->ps_threshold = APDS9921_PS_CAL_CROSSTALK_HIGH+APDS9921_PS_CROSSTALK_DELTA;

					if( data->tptype ) {
						data->ps_threshold = THREDHOLD_UP;
						apds9921_dd_set_prx_thresh(client, THREDHOLD_LOW, THREDHOLD_UP);
					} else  {
						data->ps_threshold = THREDHOLD_UP_W;
						apds9921_dd_set_prx_thresh(client, THREDHOLD_LOW_W, THREDHOLD_UP_W);
					}
					data->ps_hysteresis_threshold = data->ps_threshold - 10;

					data->ps_cali_result = CALI_OK;
					INFOR("ps cali ok\n");
					// save data->ps_offset to nv memory for use in future
				}
				else {
					INFOR("ps cali failed, ps_data:%d >2048=0x0800,11bit adc\n",ps_data);
					data->ps_cali_result = CALI_FAIL;
					return -5;
				}
			}
			else {
				INFOR("ps cali failed, ps_data:%d <= CROSSTALK_HIGH:%d\n",ps_data, APDS9921_PS_CAL_CROSSTALK_HIGH);
				data->ps_cali_result = CALI_FAIL;
				return -6;
			}
		}
		else {
			INFOR("ps cali failed, ps is running\n");
			data->ps_cali_result = CALI_FAIL;
			return -7;
		}
	}
	else {

		if( handle==ID_PX ) {
			data->non_wakeup_enable = 0;
		} else if( handle==ID_PXW ) {
			data->wakeup_enable = 0;
		} else {
			data->non_wakeup_enable = 0;
		}


		INFOR("wakeen:%d, non_wakeen:%d\n",data->wakeup_enable,data->non_wakeup_enable);
		if( data->wakeup_enable||data->non_wakeup_enable ) {
			INFOR("ps need to work, wont disable\n");


			goto exit_enable;
		}


		data->enable_ps_sensor = APDS_DISABLE_PS;

		apds9921_dd_set_int_cfg(client, (int_cfg&0xFC));
		apds9921_dd_set_main_ctrl(client, (main_ctrl&~APDS9921_DD_PRX_EN));


		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
//		cancel_delayed_work(&data->ps_dwork);

//		flush_delayed_work(&data->ps_dwork);

		irq_disable(irq_to_desc(data->irq));
		INFOR("disable all ps\n");
	}

exit_enable:
	return 0;
}

static int apds9921_ps_open(struct inode *inode, struct file *file)
{
//	INFOR("apds9921_ps_open\n");
	return 0; 
}

static int apds9921_ps_release(struct inode *inode, struct file *file)
{
//	INFOR("apds9921_ps_release\n");
	return 0;
}

static long apds9921_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct apds9921_data *data;
    struct i2c_client *client;
    int enable;
    int ret = -1;

    if (arg == 0) return -1;

    if(apds9921_i2c_client == NULL) {
		INFOR("apds9921_ps_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

    client = apds9921_i2c_client;   
    data = i2c_get_clientdata(apds9921_i2c_client);

    switch (cmd) {
		case APDS_IOCTL_PS_ENABLE:              

			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				INFOR("apds9921_ps_ioctl: copy_from_user failed\n");
				return -EFAULT;
			}

			ret = apds9921_enable_ps_sensor(client, enable);        
			if(ret < 0) {
				return ret;
			}
		break;

     	case APDS_IOCTL_PS_GET_ENABLE:
			if (copy_to_user((void __user *)arg, &data->enable_ps_sensor, sizeof(data->enable_ps_sensor))) {
				INFOR("apds9921_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

        case APDS_IOCTL_PS_GET_PDATA:

			data->ps_data =	i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR);

			if (copy_to_user((void __user *)arg, &data->ps_data, sizeof(data->ps_data))) {
				INFOR("apds9921_ps_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

		default:
		break;
    }

	
    return 0;
}

static int apds9921_als_open(struct inode *inode, struct file *file)
{
//	INFOR("apds9921_als_open\n");
	return 0;
}

static int apds9921_als_release(struct inode *inode, struct file *file)
{
//	INFOR("apds9921_als_release\n");
	return 0;
}

static long apds9921_als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct apds9921_data *data;
	struct i2c_client *client;
	int enable;
	int ret = -1;
	unsigned int delay;

	if (arg == 0) return -1;

	if(apds9921_i2c_client == NULL){
		INFOR("apds9921_als_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

	client = apds9921_i2c_client;   
	data = i2c_get_clientdata(apds9921_i2c_client);

	switch (cmd) {

		case APDS_IOCTL_ALS_ENABLE:
		
			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				INFOR("apds9921_als_ioctl: copy_from_user failed\n");
				return -EFAULT;
			}

			ret = apds9921_enable_als_sensor(client, enable); 
			if(ret < 0){
				return ret;
			}
		break;

        case APDS_IOCTL_ALS_POLL_DELAY:
	
			if (data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT) {	
				if (copy_from_user(&delay, (void __user *)arg, sizeof(delay))) {
					INFOR("apds9921_als_ioctl: copy_to_user failed\n");
					return -EFAULT;
				}
        
				ret = apds9921_set_als_poll_delay (client, delay); 
				if(ret < 0){
					return ret;
				}
			}
			else {
				INFOR("apds9921_als_ioctl: als is not in polling mode!\n");
				return -EFAULT;
			}
		break;

        case APDS_IOCTL_ALS_GET_ENABLE:
			if (copy_to_user((void __user *)arg, &data->enable_als_sensor, sizeof(data->enable_als_sensor))) {
				INFOR("apds9921_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

        case APDS_IOCTL_ALS_GET_CLEAR_DATA:

			data->clr_data = i2c_smbus_read_word_data(client, APDS9921_DD_CLEAR_DATA_ADDR);

            if (copy_to_user((void __user *)arg, &data->clr_data, sizeof(data->clr_data))) {
				INFOR("apds9921_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

        case APDS_IOCTL_ALS_GET_ALS_DATA:

			data->als_data = i2c_smbus_read_word_data(client, APDS9921_DD_ALS_DATA_ADDR);

            if (copy_to_user((void __user *)arg, &data->als_data, sizeof(data->als_data))) {
				INFOR("apds9921_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

        case APDS_IOCTL_ALS_GET_CAL_FACTOR:

            if (copy_to_user((void __user *)arg, &data->als_cal_factor, sizeof(data->als_cal_factor))) {
				INFOR("apds9921_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		break;

		default:
		break;
	}

	return 0;
}

#ifdef CONFIG_AAL_CONTROL
int apds9921_aal_als_enable(int enable)
{
	int ret = -1;
	struct apds9921_data *data;

	if(apds9921_i2c_client == NULL){
		INFOR("apds9921_als_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

	data = i2c_get_clientdata(apds9921_i2c_client);

	if( !!enable ) {
		enable = APDS_ENABLE_ALS_NO_INT;
	} else {
		enable = APDS_DISABLE_ALS;
	}

	mutex_lock(&data->mutex_apds);

	if(enable == APDS_DISABLE_ALS){
		data->enable_als_type &= ~APDS_ENABLE_ALS_AAL;
		/* used for AP don't disable light*/
		if(data->enable_als_type & APDS_ENABLE_ALS_USER){
			mutex_unlock(&data->mutex_apds);
			return 0;
		}
	}else{
		data->enable_als_type |= APDS_ENABLE_ALS_AAL;
	}

	ret = apds9921_enable_als_sensor(apds9921_i2c_client, enable); 
	mutex_unlock(&data->mutex_apds);
	return ret;

}

int apds9921_aal_get_als_data(void)
{
	struct apds9921_data *data;

	if(apds9921_i2c_client == NULL){
		INFOR("apds9921_als_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

	data = i2c_get_clientdata(apds9921_i2c_client);

	return data->als_prev_lux;
}
#endif
/*
 * SysFS support
 */

static ssize_t apds9921_show_clr_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int clr_data;

	clr_data = i2c_smbus_read_word_data(client, APDS9921_DD_CLEAR_DATA_ADDR);
	
	return sprintf(buf, "%d\n", clr_data);
}

static DEVICE_ATTR(clr_data, S_IRUGO,
		   apds9921_show_clr_data, NULL);

static ssize_t apds9921_show_als_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int als_data;

	als_data = i2c_smbus_read_word_data(client, APDS9921_DD_ALS_DATA_ADDR);
	
	return sprintf(buf, "%d\n", als_data);
}

static DEVICE_ATTR(als_data, S_IRUGO,
		   apds9921_show_als_data, NULL);

static ssize_t apds9921_show_ps_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ps_data;

	ps_data = i2c_smbus_read_word_data(client, APDS9921_DD_PRX_DATA_ADDR);
	
	return sprintf(buf, "%d\n", ps_data);
}

static DEVICE_ATTR(ps_data, S_IRUGO,
		   apds9921_show_ps_data, NULL);

static ssize_t apds9921_show_proximity_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9921_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds9921_store_proximity_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9921_data *data = i2c_get_clientdata(client);
	int dat = 0,err = 0;

	mutex_lock(&data->mutex_apds);

//	val = simple_strtoul(buf, NULL, 16);
	err = kstrtoint(buf, 10, &dat);
	if( err ) INFOR("convert hal to k failed\n");

	INFOR("store data:0x%.4x\n", dat);
	
	if ((dat != APDS_DISABLE_PS) &&
		(dat != APDS_ENABLE_PS_WITH_INT) &&
		(dat != APDS_ENABLE_PS_NO_INT) &&
		(dat != APDS_ENABLE_PS_CALIBRATION)) {
		INFOR("**%s:store invalid value=0x%.4x\n", __func__, dat);
		// modified by bsp meizu
		//return count;
	}

	apds9921_enable_ps_sensor(client, dat);

	mutex_unlock(&data->mutex_apds);
	return count;
}

static DEVICE_ATTR(ps_enable, 0664,
		apds9921_show_proximity_enable, apds9921_store_proximity_enable);

static ssize_t apds9921_show_light_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9921_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds9921_store_light_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	struct apds9921_data *data = i2c_get_clientdata(client);

	INFOR("%s: enable als sensor ( %ld)\n", __func__, val);
	
	if ((val != APDS_DISABLE_ALS) && 
		(val != APDS_ENABLE_ALS_WITH_INT) && 
		(val != APDS_ENABLE_ALS_NO_INT) &&
		(val != APDS_ENABLE_ALS_CALIBRATION)) {

		INFOR("**%s: store invalid value=%ld\n", __func__, val);
		return count;
	}

	if( !!val ) {
		val = APDS_ENABLE_ALS_NO_INT;
	} else {
		val = APDS_DISABLE_ALS;
	}

	mutex_lock(&data->mutex_apds);
#ifdef CONFIG_AAL_CONTROL
	if(val == APDS_DISABLE_ALS){
		data->enable_als_type &= ~APDS_ENABLE_ALS_USER;
		/* used for AAL don't disable light*/
		if(data->enable_als_type & APDS_ENABLE_ALS_AAL){
			mutex_unlock(&data->mutex_apds);
			return count;
		}
	}else{
		data->enable_als_type |= APDS_ENABLE_ALS_USER;
	}
#endif
	apds9921_enable_als_sensor(client, val); 
	mutex_unlock(&data->mutex_apds);

	return count;
}

static DEVICE_ATTR(als_enable, 0664,
		apds9921_show_light_enable, apds9921_store_light_enable);

static struct attribute *apds9921_attributes[] = {
	&dev_attr_clr_data.attr,
	&dev_attr_als_data.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_ps_enable.attr,
	&dev_attr_als_enable.attr,
	NULL
};

static const struct attribute_group apds9921_attr_group = {
	.attrs = apds9921_attributes,
};

static struct file_operations apds9921_ps_fops = {
	.owner = THIS_MODULE,
	.open = apds9921_ps_open,
	.release = apds9921_ps_release,
	.unlocked_ioctl = apds9921_ps_ioctl,
};

static struct miscdevice apds9921_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "apds_ps_dev",
	.fops = &apds9921_ps_fops,
};

static struct file_operations apds9921_als_fops = {
	.owner = THIS_MODULE,
	.open = apds9921_als_open,
	.release = apds9921_als_release,
	.unlocked_ioctl = apds9921_als_ioctl,
};

static struct miscdevice apds9921_als_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "apds_als_dev",
	.fops = &apds9921_als_fops,
};

/*
 * Initialization function
 */

static int apds9921_init_client(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int err;
	int id;
	int int_cfg= 0x10;//default value

#if 0
	{
		int i, i2c_data;
		
		INFOR("before\n");
		
    	for (i=0; i<0x28; i++)
    	{
			i2c_data = i2c_smbus_read_byte_data(client, i);
			INFOR("[%x] = %x\n", i, i2c_data);
			mdelay(1);
    	}
  	}
#endif

	err = apds9921_dd_set_main_ctrl(client, 0);

	if (err < 0)
		return err;
	
	id = i2c_smbus_read_byte_data(client, APDS9921_DD_PART_ID_ADDR);
	if (id == 0xB1) {
		INFOR("it's APDS-9921, id==0x%x\n", id);
	}
	else {
		INFOR("it's Not APDS-9921, id==0x%x, ps en:%d\n", id, data->enable_ps_sensor);
		//modified by bsp@meizu.com
		//return -EIO;
	}

	/* PS_LED */
	err = apds9921_dd_set_prx_led(client, APDS9921_DD_PRX_DEFAULT_LED_FREQ|APDS9921_DD_LED_CURRENT_125_MA);
	if (err < 0) {
		INFOR("set default led freq/current failed\n");
		return err;
	}

	/* PS_PULSES */
	err = apds9921_dd_set_prx_pulses(client, APDS9921_DD_PRX_DEFAULT_PULSE);
	if (err < 0) {
		INFOR("set default prx pulse failed\n");
		return err;
	}

	/* PS_MEAS_RATE */
	err = apds9921_dd_set_prx_meas_rate(client, 0x40|APDS9921_DD_PRX_MEAS_RES_11_BIT|APDS9921_DD_PRX_DEFAULT_MEAS_RATE);
	if (err < 0) {
		INFOR("set default prx res/rate failed\n");
		return err;
	}

	/* ALS_MEAS_RATE */
	err = apds9921_dd_set_als_meas_rate(client, APDS9921_DD_ALS_DEFAULT_RES|APDS9921_DD_ALS_DEFAULT_MEAS_RATE);
	if (err < 0) {
		INFOR("set default prx res/rate failed\n");
		return err;
	}

	/* ALS_GAIN */
	err = apds9921_dd_set_als_gain(client, APDS9921_DD_ALS_DEFAULT_GAIN);
	if (err < 0) {
		INFOR("set default als gain failed\n");
		return err;
	}

	/* INT_PERSISTENCE */
	err = apds9921_dd_set_pers(client, APDS9921_DD_PRX_PERS_1|APDS9921_DD_ALS_PERS_1);
	if (err < 0) {
		INFOR("set  prx/als pers failed\n");
		return err;
	}

	/* PS_THRES_UP & PS_THRES_DOWN */
	/*
	 * modified by bsp meizu; threhold up:0x00c8/200, low:0x0096/150
	 */
#if 1
    if( data->tptype ) {
		err = apds9921_dd_set_prx_thresh(client, THREDHOLD_LOW, THREDHOLD_UP);
	} else  {
		err = apds9921_dd_set_prx_thresh(client, THREDHOLD_LOW_W, THREDHOLD_UP_W);
	}
#else
	err = apds9921_dd_set_prx_thresh(client, data->ps_threshold, data->ps_threshold); // init threshold for proximity
#endif
	if (err < 0) {
		INFOR("set  prx threshhold failed\n");
		return err;
	}

	/* PS Offset */

	// modified by bsp meizu
#if 1 //ps offset
	err = apds9921_dd_set_prx_can(client, OFFSET);
#else
	err = apds9921_dd_set_prx_can(client, data->ps_offset);
#endif
	if (err < 0) {
		INFOR("set  prx offset failed\n");
		return err;
	}
	// sensor is in disabled mode but all the configurations are preset
	int_cfg = i2c_smbus_read_byte_data(client, APDS9921_DD_INT_CFG_ADDR);
	apds9921_dd_set_int_cfg(client, (int_cfg&0xff)|APDS9921_DD_PRX_LOGIC_MODE);// wont clear after read

#if 1
	{
		int i, i2c_data;
		
		INFOR("after\n");
		
    	for (i=0; i<0x28; i++)
    	{
			i2c_data = i2c_smbus_read_byte_data(client, i);
			INFOR("[%x] = %x\n", i, i2c_data);
			mdelay(1);
    	}
  	}
#endif

	return 0;
}

/*
 * I2C init/probing/exit functions
 */
extern int mz_ps_eint_register(irq_handler_t handler, void *client, struct apds9921_data *data);

static struct i2c_driver apds9921_driver;
static int  apds9921_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9921_data *data;
	int err = -ENODEV;
	unsigned char partid = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds9921_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	/*
 * device id, addr:0x06, value:0xb3(apds9922)/0xb1(ltr578)
 */

	partid = i2c_smbus_read_byte_data(client, APDS9921_DD_PART_ID_ADDR);
	switch(partid) {
		case 0xb3:
			INFOR("this is apds9922, partid:0x%x\n", partid);
			break;
		case 0xb1:
			INFOR("this is ltr578, partid:0x%x\n", partid);
			break;
		default:
			INFOR("this is not apds9922 or ltr578, partid:0x%x\n", partid);
			goto exit_kfree;
	}


	data->client = client;
	apds9921_i2c_client = client;

	i2c_set_clientdata(client, data);

	data->ps_threshold = APDS9921_PS_DETECTION_THRESHOLD;
	data->ps_hysteresis_threshold = APDS9921_PS_HSYTERESIS_THRESHOLD;
	data->ps_detection = 0;	/* default to no detection */
	data->ps_poll_delay = 25;	// default to 100ms
	data->ps_offset = 0; // default to 0, should load from nv memory if needed
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	data->als_poll_delay = 100;	// default to 100ms
	data->als_res_index = APDS9921_DD_ALS_RES_19BIT;	// 200ms conversion time
	data->als_gain_index = APDS9921_DD_ALS_GAIN_9X;	// 9x GAIN
	data->als_prev_lux = 0;
	data->als_cal_factor = 41;	// default to 1.0, up scale to 100
	data->als_suspended = 0;
	data->ps_suspended = 0;
	data->main_ctrl_suspended_value = 0;	/* suspend_resume usage */

	data->wakeup_enable = 0;
	data->non_wakeup_enable = 0;
	data->irq_wakeup_stat = NON_WAKE;
	data->ps_state_pre = FAR;
	data->ps_cali_result = CALI_FAIL;
	data->als_cali_result = CALI_FAIL;
	data->tptype = 0;// set white tp as default
	mutex_init(&data->mutex_apds);
	apds_data = data;

	/*for PCB B1 & B2, keep tpvddi power on if ps sensor enabled*/
	data->dvddio18 = regulator_get(&client->dev, "tpvddi");
	if(!IS_ERR(data->dvddio18))
		err = regulator_enable(data->dvddio18);

	//mutex_init(&data->update_lock);

#ifdef CONFIG_SENSOR_APDS9921
	err = gpio_request(platform_data->irq_num, "apds_irq");
	if (err)
    	{
        	INFOR("Unable to request GPIO.\n");
        	goto exit_kfree;
    	}
    
    	gpio_direction_input(platform_data->irq_num);
    	irq = gpio_to_irq(platform_data->irq_num);
    
    	if (irq < 0)
    	{
        	err = irq;
        	INFOR("Unable to request gpio irq. err=%d\n", err);
        	gpio_free(platform_data->irq_num);
        
        	goto exit_kfree;
    	}
    
    	data->irq = irq;
	if (request_irq(APDS9921_INT, apds9921_interrupt, IRQF_TRIGGER_FALLING,
		APDS9921_DRV_NAME, (void *)client)) {
		INFOR("%s Could not allocate APDS9950_INT !\n", __func__);
	
		goto exit_kfree;
	}
#else


#if 0 //gpiolib
	err = gpio_request(data->gpio, "apds_irq");
	if (err)
	{
		INFOR("Unable to request GPIO:%d\n", data->gpio);
		goto exit_kfree;
	}

	client->irq = mt_gpio_to_irq( data->gpio );
	data->irq = client->irq;
	INFOR("gpiolib irq:%d\n", data->irq);

	if (request_irq(client->irq, apds9921_interrupt, IRQF_TRIGGER_FALLING,
					APDS9921_DRV_NAME, (void *)client)) {
		INFOR(" Could not allocate APDS9921_INT:%d !\n", client->irq);

		goto exit_kfree;
	} else {
		INFOR("request thread irq:%d ok\n", client->irq);
	}
#endif


#if 1//dts
    // irq_set_status_flags(data->irq, IRQ_NOAUTOEN);
	data->irq = mz_ps_eint_register(apds9921_interrupt, (void *)client, data);
	client->irq = data->irq;
	INFOR("dts irq:%d\n", data->irq);
#endif

	irq_disable(irq_to_desc(data->irq));

#endif

#if LINUX_KERNEL_2_6_X
	set_irq_wake(client->irq, 1);
#else
//	irq_set_irq_wake(client->irq, 1);
#endif

	// interrupt
	INIT_DELAYED_WORK(&data->dwork, apds9921_work_handler);
	// polling : ALS
	INIT_DELAYED_WORK(&data->als_dwork, apds9921_als_polling_work_handler); 
	// polling : PS
	INIT_DELAYED_WORK(&data->ps_dwork, apds9921_ps_polling_work_handler);

	INFOR("%s interrupt is hooked\n", __func__);

	/* Initialize the APDS9921 chip */
	err = apds9921_init_client(client);
	if (err)
		goto exit_kfree;

	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		INFOR("Failed to allocate input device als\n");
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		INFOR("Failed to allocate input device ps\n");
		goto exit_free_dev_als;
	}
	
	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 65535, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 10, 0, 0);

#if 0
	input_set_capability(isl29125->sensor_input, EV_ABS, ABS_LUX);
	input_set_capability(isl29125->sensor_input, EV_ABS, ABS_GREEN);
	input_set_capability(isl29125->sensor_input, EV_ABS, ABS_GREENIR);
	input_set_abs_params(isl29125->sensor_input, ABS_LUX, 0, 0xFFFF, 0, 0);
	input_set_abs_params(isl29125->sensor_input, ABS_GREEN, 0, 0xFFFF, 0, 0);
	input_set_abs_params(isl29125->sensor_input, ABS_GREENIR, 0, 0xFFFF, 0, 0);
#endif

	data->input_dev_als->name = INPUT_ALS_NAME;
	data->input_dev_ps->name = INPUT_PRX_NAME;
#ifndef CONFIG_INPUT_ALSPS_BH1745
	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		INFOR("Unable to register input device als: %s\n",
		       data->input_dev_als->name);
		goto exit_free_dev_ps;
	}
#endif
	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		INFOR("Unable to register input device ps: %s\n",
		       data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9921_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	/* Register for sensor ioctl */
    err = misc_register(&apds9921_ps_device);
	if (err) {
		INFOR("Unalbe to register ps ioctl: %d", err);
		goto exit_remove_sysfs_group;
	}

    err = misc_register(&apds9921_als_device);
	if (err) {
		INFOR("Unalbe to register als ioctl: %d", err);
		goto exit_unregister_ps_ioctl;
	}


	apds_create_dbg_sys(&client->dev.kobj);
	err = meizu_sysfslink_register_name(&client->dev, "ps");
	if( err<0 ) {
		INFOR("register ps syslink failed\n");
	}

#ifndef CONFIG_INPUT_ALSPS_BH1745
	err = meizu_sysfslink_register_name(&client->dev, "als");
	if( err<0 ) {
		INFOR("register als syslink failed\n");
	}
#endif

	INFOR("support ver. %s enabled,apds9922 is ready\n", DRIVER_VERSION);

	return 0;

exit_unregister_ps_ioctl:
	misc_deregister(&apds9921_ps_device);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &apds9921_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_unregister_dev_als:
#ifndef CONFIG_INPUT_ALSPS_BH1745
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
#endif
exit_free_dev_als:
exit_free_irq:
	free_irq(client->irq, client);
exit_kfree:
	kfree(data);
exit:
	return err;
}

static int  apds9921_remove(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);

	/* Power down the device */
	apds9921_dd_set_main_ctrl(client, 0);

	misc_deregister(&apds9921_als_device);
	misc_deregister(&apds9921_ps_device);	

	sysfs_remove_group(&client->dev.kobj, &apds9921_attr_group);

	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);
	
	free_irq(client->irq, client);

	kfree(data);

	return 0;
}

//#define CONFIG_PM

#if 1//def CONFIG_PM

static int apds9921_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds9921_data *data = i2c_get_clientdata(client);
	int err = 0;
	int main_ctrl;
	int gpio_val = 0;

	main_ctrl = 0;
	err = 0;
	INFOR("apds9921_suspend\n");
	
	// Do nothing as p-sensor is in active
#if 1//modified by bsp meizu
    mutex_lock(&data->mutex_apds);

	gpio_val = gpio_get_value(data->gpio);
	INFOR("psen:%d,non-wakeen:%d,wakeen:%d,gpio:%d\n",
		  data->enable_ps_sensor,data->non_wakeup_enable,data->wakeup_enable,gpio_val);
	if( data->wakeup_enable ) {
		INFOR("ps wakeup enable\n");
		data->irq_wakeup_stat = WAKE;
		irq_set_irq_wake(data->irq, 1);
	} else {
		data->irq_wakeup_stat = NON_WAKE;
		irq_disable(irq_to_desc(data->irq));
		if(!IS_ERR_OR_NULL(data->dvddio18))
			err = regulator_disable(data->dvddio18);
	}

	mutex_unlock(&data->mutex_apds);

	return 0;

#else //orig code
	if(data->enable_ps_sensor)
		return 0;
#endif

#if 0
	err = apds9921_dd_set_main_ctrl(client, 0);

	if (err < 0)
		return err;
	
	main_ctrl = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR);
	if (main_ctrl < 0)
		return main_ctrl;

	data->main_ctrl_suspended_value = main_ctrl;
		
	if (main_ctrl&APDS9921_DD_ALS_EN) {
		if (data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT) {
			cancel_delayed_work(&data->als_dwork);
			flush_delayed_work(&data->als_dwork);
		}
		
		err = apds9921_dd_set_main_ctrl(client, main_ctrl&~APDS9921_DD_ALS_EN);
		if (err < 0)
			return err;	
	}
	
	data->als_suspended = 1;
	
	if (!(main_ctrl&APDS9921_DD_PRX_EN)) {	
		data->ps_suspended = 1;
	}
	else {
		data->ps_suspended = 0;
	}

	if (data->ps_suspended && data->als_suspended) {
		cancel_delayed_work(&data->dwork);
		flush_delayed_work(&data->dwork);

		flush_workqueue(apds_workqueue);

		disable_irq(data->irq);
	
#if LINUX_KERNEL_2_6_X
		set_irq_wake(client->irq, 0);
#else
		irq_set_irq_wake(client->irq, 0);
#endif

		if(NULL != apds_workqueue){
			destroy_workqueue(apds_workqueue);
			INFOR( " Destroy workqueue\n");
			apds_workqueue = NULL;
		}
	}
	return 0;
#endif
}

static int apds9921_resume(struct i2c_client *client)
{
	struct apds9921_data *data = i2c_get_clientdata(client);	
	int err = 0,gpio_val = 0;

	err = 0;
	// Do nothing as it was not suspended
	mutex_lock(&data->mutex_apds);

	INFOR(" (main_ctrl=%d)\n", data->main_ctrl_suspended_value);

	gpio_val = gpio_get_value(data->gpio);
	INFOR("psen:%d,non-wakeen:%d,wakeen:%d,gpio:%d\n",
		  data->enable_ps_sensor,data->non_wakeup_enable,data->wakeup_enable,gpio_val);
	if( data->irq_wakeup_stat==NON_WAKE ) {
		INFOR("ps wakeup disable\n");
		if(!IS_ERR_OR_NULL(data->dvddio18))
			err = regulator_enable(data->dvddio18);
		irq_enable(irq_to_desc(data->irq));
	} else {
		irq_set_irq_wake(data->irq, 0);
	}
	data->als_suspended = 0;
	data->ps_suspended = 0;

	mutex_unlock(&data->mutex_apds);

	return 0;

#if 0
		if(apds_workqueue == NULL) {
			apds_workqueue = create_workqueue("proximity_als");
			if(NULL == apds_workqueue) {
				INFOR("create work queue failed\n");
				return -ENOMEM;
			}
		}
		enable_irq(data->irq);

	mdelay(1);

	err = apds9921_dd_set_main_ctrl(client, data->main_ctrl_suspended_value);
	if(err < 0) {
		INFOR( " enable set Fail\n");
		return 0;
	}
	data->als_suspended = 0;
	data->ps_suspended = 0;

#if LINUX_KERNEL_2_6_X
	set_irq_wake(client->irq, 1);
#endif

	return 0;
#endif
}

#else

#define apds9921_suspend	NULL
#define apds9921_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds9921_id[] = {
	{ "apds9922", 0 },
	{ }
};
//MODULE_DEVICE_TABLE(i2c, apds9921_id);


#ifdef CONFIG_OF
static const struct of_device_id apds9921_of_match[] = {
	{.compatible = "mediatek,apds9922"},
	{},
};
MODULE_DEVICE_TABLE(of, apds9921_of_match);
#endif

static int apds_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, "apds9922-board");

	return 0;
}
#if 0
static const struct dev_pm_ops ps_pm_ops = {
	.suspend = ps_suspend,
	.resume	= ps_resume,
};
#endif
static struct i2c_driver apds9921_driver = {
	.driver = {
		.name	= "apds9922-i2c-drv",
		.owner	= THIS_MODULE,

#ifdef CONFIG_OF
    	.of_match_table = of_match_ptr(apds9921_of_match),
#endif
	},
	.suspend = apds9921_suspend,
	.resume	= apds9921_resume,
	.probe	= apds9921_probe,
	.remove	= apds9921_remove,
	.id_table = apds9921_id,
	.detect = apds_i2c_detect,
};

#if 0
static struct i2c_board_info  apds9922_board_info[] = {
		{
				I2C_BOARD_INFO(I2C_PRX_ALS_NAME, 0x53),
				//.irq = (CUST_EINT_GYRO_NUM),
				//.platform_data = &pdata,
		},
};


static int __init apds9922_board_init(void)
{
	INFOR("\n");
	i2c_register_board_info(1, apds9922_board_info, 1);
	return 0;
}
#endif

static int __init apds9921_init(void)
{
	int err = 0;
	INFOR("\n");
	apds_workqueue = create_workqueue("apds9922-wkq");
	
	if (!apds_workqueue) {
		INFOR("create apds workqueue failed\n");
		return -ENOMEM;
	}

	err = i2c_add_driver(&apds9921_driver);
	INFOR("i2c add driver %d\n",err);
	return err;
}

static void __exit apds9921_exit(void)
{
	INFOR("\n");
	if (apds_workqueue)
		destroy_workqueue(apds_workqueue);

	apds_workqueue = NULL;

	i2c_del_driver(&apds9921_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9921 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);


//postcore_initcall(apds9922_board_init);
module_init(apds9921_init);
module_exit(apds9921_exit);

