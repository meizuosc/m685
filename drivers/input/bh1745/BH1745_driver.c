/* drivers/i2c/chips/BH1745_driver.c - ROHM BH1745 Linux kernel driver
 *
 * Copyright (C) 2015 
 * Written by Grace Huang<grace-huang@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  This is Linux kernel modules for ambient light
 *  Revision History
 *  2015-4-10:	Ver. 1.0	New release .
 
 */
#include "BH1745_driver.h"



extern void als_create_dbg_sys(struct kobject *kobj);
/*************** Global Data ******************/
/* parameter for als calculation */

#define JUDGE_FIXED_COEF  	1000
#define CUT_UNIT  	1000
#define TRANS 		16 // gains:16x

#if 1 //white m95
int judge = 198;
int red[2] = {176, 115};
int green[2]={641, 559};
double ratio_judgeC= 0.222;
double beff[3]={4.114, 15.355, 1.0};
double color_buffer[4]={-2.89, 8.439, -3.131,  6.404};
int cct_eff[4]= {11860.8, 1390.7, 22350.65, 1880.4};

#else

int judge = 250;
int red[2] = {219, 213};
int green[2]={813, 467};
double ratio_judgeC= 0.27;
double beff[3]={3.371, 12.143, 1.0};
double color_buffer[4]={-2.909, 6.552, -2.832,  4.795};
int cct_eff[4]= {11506, 1263, 18595, 1915};
#endif

unsigned short atime[6] = {160, 320, 640, 1280, 2560, 5120};
unsigned char again[3] = {1, 2, 16};
#ifdef CONFIG_AAL_CONTROL
static struct i2c_client *g_als_client;
#endif

/*************** Functions ******************/
/******************************************************************************
 * NAME       : BH1745_set_enable
 * FUNCTION   : set measurement time according to enable
 * REMARKS    : this function will overwrite the work mode. if it is called improperly, 
 *			   you may shutdown some part unexpectedly. please check rgb->enable first.
 *****************************************************************************/
static int BH1745_set_enable(struct i2c_client *client, int enable)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	int ret = 0;
	int tmp = 0;

//	mutex_lock(&rgb->update_lock);

	INFOR("en:%d, en-flag:%d\n", enable, rgb->enable);

	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL2);
	if(tmp < 0)
	{
		INFOR( " i2c read i2c fail. \n");
		return tmp;
	}
	if (enable)
	{
		tmp = tmp | RGBC_EN_ON;
		ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL2, tmp);
		if(ret < 0)
		{
			INFOR( " write i2c fail.\n");
			return ret;
		}
	}
	else
	{
		tmp = tmp & (~RGBC_EN_ON);
		ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL2, tmp);
		if(ret < 0)
		{
			INFOR( " write i2c fail.\n");
			return ret;
		}
	}
//	mutex_unlock(&rgb->update_lock);
	rgb->enable = enable;

	return 0;
}

#if 0
static int BH1745_set_als_threshold_low(struct i2c_client *client, int threshold)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > 0XFFFF)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to BH1745 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&rgb->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_THRED_LOW, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&rgb->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	rgb->als_th_l = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int BH1745_set_als_threshold_high(struct i2c_client *client, int threshold)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > 0XFFFF)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to BH1745 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&rgb->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_THRED_HIGH, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&rgb->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	rgb->als_th_h = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}
#endif

/******************************************************************************
 * NAME       : calc_color_temp
 * FUNCTION   : Calculate color temperature
 * REMARKS    :
 * INPUT      : data : each data value from IC (Red, Green, Blue, Clear)
 * RETURN     : color temperature
 *****************************************************************************/
int calc_color_temp(READ_DATA_ARG data)
{
	double R_adj, G_adj, B_adj, C_adj, RGB_adj;
	double R_ratio, B_ratio;
	double B_eff;
	double cct = 0.0;
	int final=0;

	R_adj = data.red * 1.0 + 0.0;
	G_adj = data.green * 1.0 + 0.0;
	B_adj = data.blue * 1.0 + 0.0;
	C_adj = data.clear * 1.0 + 0.0;


	RGB_adj = R_adj + G_adj + B_adj;
	if ((G_adj < 1) || (RGB_adj < 1)) 
	{
		cct = 0;
	} 
	else 
	{
		R_ratio = (R_adj) / (RGB_adj);
		B_ratio = (B_adj) / (RGB_adj);

		if ((C_adj/G_adj) < ratio_judgeC) 
		{
			if ((B_ratio*beff[0]) > beff[2])
				B_eff = beff[2];
			else
				B_eff = B_ratio*beff[0];
			
//			cct = (1 - B_eff) * cct_eff[0] * exp(color_buffer[0] * R_ratio) + B_eff * cct_eff[1] * exp(color_buffer[1] * B_ratio);
		} 
		else 
		{
			if ((B_ratio*beff[1]) > beff[2])
				B_eff = beff[2];
			else
				B_eff = B_ratio*beff[1];

//			cct = (1 - B_eff) * cct_eff[2] * exp(color_buffer[2] * R_ratio) + B_eff * cct_eff[3] * exp(color_buffer[3] * B_ratio);
		}

		if (cct > 10000) 
		{
			cct = 10000;
		}
	}

	final = (int)cct;

	return final;
}

/******************************************************************************
 * NAME       : calc_lx
 * FUNCTION   : Calculate lux
 * REMARKS    :
 * INPUT      : data  : each data value from IC (Red, Green, Blue, Clear)
 *            : gain  : gain setting of the sensor
 *            : itime : measurement time of sensor (unit is ms)
 * RETURN     : Lux value
 *****************************************************************************/
int calc_lx(READ_DATA_ARG  data, unsigned char gain, unsigned short itime)
{
	unsigned long lx;
	unsigned long lx_tmp;

	if (data.green < 1) 
	{
		lx = 0;
	} 
	else 
	{
		if ((data.clear*JUDGE_FIXED_COEF) < (judge*data.green))
		{
			
			lx_tmp = red[0] * data.red + green[0] * data.green;
//			INFOR("red[0]:%d, green[0]:%d\n",red[0],green[0]);
			
		} 
		else 
		{
		    lx_tmp = red[1] * data.red + green[1] * data.green;
//.3			INFOR("red[1]:%d, green[1]:%d\n",red[1],green[1]);
		}

		lx = ((lx_tmp*160 *TRANS / gain) / itime) /CUT_UNIT;
	}



	return (lx);
}


/******************************************************************************
 * NAME       : bh1745_get_als_data
 * FUNCTION   : Get the data of  lux and color temperature
 * REMARKS    :
 * INPUT      : light_data  : calculated lux and color temperature are returned back through the pointer of light_data
 * RETURN     : 0
 *****************************************************************************/
int bh1745_get_als_data(struct i2c_client *client, CALC_DATA_ARG *light_data, READ_DATA_ARG *rgb_data)
{
//	struct RGB_DATA *rgb = i2c_get_clientdata(client);

	unsigned char data[8], gain = 0;
	unsigned short time = 0; 
	unsigned char valid = 0;
	unsigned char tmp;
	READ_DATA_ARG read_data;
	int ret;

	valid = i2c_smbus_read_byte_data(client, REG_MODECONTROL2);
	if ((valid & RGBC_VALID_HIGH) == 0)
	{
		return -1; 
	}

	ret = i2c_smbus_read_i2c_block_data(client, REG_RED_DATA, 8, data);
	if(ret != 8)
	{
		INFOR( " i2c read rgb data fail. \n");
	}

	read_data.red = (data[1] <<8) |data[0];
	read_data.green = (data[3] <<8) |data[2];
	read_data.blue= (data[5] <<8) |data[4];
	read_data.clear = (data[7] <<8) |data[6];

	rgb_data->red = read_data.red;
	rgb_data->green = read_data.green;
	rgb_data->blue = read_data.blue;
	rgb_data->clear = read_data.clear;


	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL2);
	tmp = tmp & 0x3;
	gain = again[tmp];

//	INFOR("tmp:%d, gain:%d\n",tmp,gain);
	
	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL1);
	tmp = tmp & 0x7;
	time = atime[tmp];
//	INFOR("tmp:%d, atime:%d\n",tmp,time);

	light_data->lux = calc_lx(read_data, gain, time);
	light_data->color_temp = calc_color_temp(read_data);
#if 0
	INFOR( "BH1745 als report: red = %d, green = %d, blue = %d, clear = %d, time = %d, gain = %d, lux = %d.\n", read_data.red,
		read_data.green, read_data.blue, read_data.clear, time, gain, light_data->lux);

#endif
	return 0;
	
	
}


/******************************************************************************
 * NAME       : BH1745_init
 * FUNCTION   : Initialize the register setting of BH1745
 * REMARKS    :
 * INPUT      : 
 * RETURN     : 0
 *****************************************************************************/
void BH1745_init_client(struct i2c_client *client)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	unsigned char tmp; 
	int ret;
	
	tmp = SW_RESET|INT_RESET;
	ret = i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, tmp);
	if(ret < 0)
	{
		INFOR( " write i2c fail.\n");
		return ;
	}
	
	tmp = RGB_SET_MODE_CONTROL1;
	ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL1, tmp);
	if(ret < 0)
	{
		INFOR( " write i2c fail.\n");
		return ;
	}
	
	tmp = RGB_SET_MODE_CONTROL2;
	ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL2, tmp);
	if(ret < 0)
	{
		INFOR( " write i2c fail.\n");
		return ;
	}
	
	tmp = 0x2;
	ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL3, tmp);
	if(ret < 0)
	{
		INFOR( " write i2c fail.\n");
		return ;
	}

	rgb->enable = 0;
}

static unsigned int BH1745_als_data_to_level(unsigned int als_data)
{
#if 0
#define ALS_LEVEL_NUM 15
	int als_level[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
	int als_value[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
    	unsigned char idx;

	for(idx = 0; idx < ALS_LEVEL_NUM; idx ++)
	{
		if(als_data < als_value[idx])
		{
			break;
		}
	}
	if(idx >= ALS_LEVEL_NUM)
	{
		printk(KERN_ERR "BH1745 als data to level: exceed range.\n");
		idx = ALS_LEVEL_NUM - 1;
	}
	
	return als_level[idx];
#undef ALS_LEVEL_NUM
#else
	return als_data;
#endif
}


/* ALS polling routine */
static void BH1745_als_polling_work_handler(struct work_struct *work)
{
	struct RGB_DATA *rgb = container_of(work, struct RGB_DATA, als_dwork.work);
	struct i2c_client *client=rgb->client;
	int tmp = 0;
	static unsigned char flag=0xff;

	READ_DATA_ARG rgb_data={0,};
	CALC_DATA_ARG light_data={0,};
	
	schedule_delayed_work(&rgb->als_dwork, msecs_to_jiffies(rgb->als_poll_delay));	// restart timer
	
	tmp = bh1745_get_als_data(client, &light_data, &rgb_data);
	if(tmp<0) {
		INFOR("als get data failed\n");
		return;
	}
	
	rgb->als_data = light_data.lux;
	
	if(rgb->als_data == 0)
		rgb->als_data ++;

	/*
	 * modified by bsp meizu
 	*/
#if 0
	rgb->als_level = BH1745_als_data_to_level(rgb->als_data);
	input_report_abs(rgb->input_dev_als, ABS_MISC, rgb->als_level); // report als data. maybe necessary to convert to lux level
#else
	rgb->als_level = BH1745_als_data_to_level(rgb->als_data);
	if( flag ) {
		flag = 0;
	} else {
		flag = 0xff;
		rgb->als_data++;
	}
	if( rgb->als_cali==CALI_OK ) {
		input_report_abs(rgb->input_dev_als, ABS_MISC,  rgb->als_data*rgb->als_scale/10);
	} else {
		input_report_abs(rgb->input_dev_als, ABS_MISC,  rgb->als_data);
	}
	input_report_abs(rgb->input_dev_als, ABS_RED,   rgb_data.red);
	input_report_abs(rgb->input_dev_als, ABS_GREEN, rgb_data.green);
	input_report_abs(rgb->input_dev_als, ABS_BLUE,  rgb_data.blue);
	input_report_abs(rgb->input_dev_als, ABS_CLEAR, rgb_data.clear);
#endif
	input_sync(rgb->input_dev_als);

}


static int BH1745_enable_als_sensor(struct i2c_client *client, int enable)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	int ret = 0;

	INFOR( "BH1745 enable ALS sensor -> en:%d, en-flag:%d\n", enable, rgb->enable);

	if(enable == 1)
	{
		//turn on light  sensor
		if (rgb->enable == 0)
		{
			ret = BH1745_set_enable(client, 1);
			if (ret == 0)
				rgb->enable = 1;
		}
		
		cancel_delayed_work(&rgb->als_dwork);
		schedule_delayed_work(&rgb->als_dwork, msecs_to_jiffies(rgb->als_poll_delay));	// 125ms

	}
	else
	{
		if(rgb->enable == 1)
		{
			ret = BH1745_set_enable(client, 0);
			if (ret == 0)
				rgb->enable = 0;
		}
		cancel_delayed_work(&rgb->als_dwork);
	}
	return ret;
}

#ifdef CONFIG_AAL_CONTROL
int BH1745_aal_enable_als(int enable)
{
	if(g_als_client){
		struct RGB_DATA *rgb = i2c_get_clientdata(g_als_client);

		mutex_lock(&rgb->update_lock);
		if(enable == 0){
			rgb->enable_als_type &= ~RGB_ENABLE_ALS_AAL;
			/* used for AP don't disable light*/
			if(rgb->enable_als_type & RGB_ENABLE_ALS_USER){
				mutex_unlock(&rgb->update_lock);
				return 0;
			}
		}else{
			rgb->enable_als_type |= RGB_ENABLE_ALS_AAL;
		}
		BH1745_enable_als_sensor(g_als_client, enable);
		mutex_unlock(&rgb->update_lock);
		return 0;
	}
	return -ENODEV;
}
int BH1745_aal_get_als_data(void)
{
	struct RGB_DATA *rgb;

	if(g_als_client){
		rgb = i2c_get_clientdata(g_als_client);
		return rgb->als_data;
	}
	return -ENODEV;
}
#endif
static ssize_t BH1745_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", rgb->enable);
}

static ssize_t BH1745_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	INFOR( "BH1745 enable ALS sensor -> en:%ld, en-flag:%d\n", val, rgb->enable);

	if ((val != 0) && (val != 1))
	{
		INFOR(" enable als sensor=%ld\n", val);
		return count;
	}

	mutex_lock(&rgb->update_lock);
	if( rgb->tptype ) {// black tp
		judge = 250;
		red[0] = 219;
		red[1] = 213;
		green[0] = 813;
		green[1] = 467;
	} else { // white tp
		judge = 198;
		red[0] = 176;
		red[1] = 115;
		green[0] = 641;
		green[1] = 559;
	}


#ifdef CONFIG_AAL_CONTROL
	if(val == 0){
		rgb->enable_als_type &= ~RGB_ENABLE_ALS_USER;
		/* used for AAL don't disable light*/
		if(rgb->enable_als_type & RGB_ENABLE_ALS_AAL){
			mutex_unlock(&rgb->update_lock);
			return count;
		}
	}else{
		rgb->enable_als_type |= RGB_ENABLE_ALS_USER;
	}
#endif
	BH1745_enable_als_sensor(client, val);
	mutex_unlock(&rgb->update_lock);
	return count;
}

static ssize_t BH1745_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", rgb->als_poll_delay*1000);	// return in micro-second
}

static ssize_t BH1745_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned long flags;
	
	if (val < PS_ALS_SET_MIN_DELAY_TIME)
		val = PS_ALS_SET_MIN_DELAY_TIME;

	INFOR("delay:%ld, en:%d\n",val, rgb->enable);

	rgb->als_poll_delay = 250;	// convert us => ms



	if (rgb->enable == 1)
	{
	
		/* we need this polling timer routine for sunlight canellation */
		spin_lock_irqsave(&rgb->update_lock.wait_lock, flags);

		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&rgb->als_dwork);
		schedule_delayed_work(&rgb->als_dwork, msecs_to_jiffies(rgb->als_poll_delay));

		spin_unlock_irqrestore(&rgb->update_lock.wait_lock, flags);
	
	}
	return count;
}


static ssize_t BH1745_show_als_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int red, green, blue, clear;

	red = i2c_smbus_read_word_data(client, REG_RED_DATA);
	green = i2c_smbus_read_word_data(client, REG_GREEN_DATA);
	blue = i2c_smbus_read_word_data(client, REG_BLUE_DATA);
	clear = i2c_smbus_read_word_data(client, REG_CLEAR_DATA);

	return sprintf(buf, "%d %d %d %d\n", red, green, blue, clear);
}


static ssize_t BH1745_show_type(struct device *dev,
                struct device_attribute *attr, char *buf){
    struct i2c_client *client = to_i2c_client(dev);
    struct RGB_DATA *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->type);
}
static DEVICE_ATTR(als_batch,  0660,
				    BH1745_show_als_poll_delay, BH1745_store_als_poll_delay);

static DEVICE_ATTR(als_enable,  0660 ,
				  BH1745_show_enable_als_sensor, BH1745_store_enable_als_sensor);

static DEVICE_ATTR(als_data, S_IRUGO, BH1745_show_als_data, NULL); 

static DEVICE_ATTR(type, S_IRUGO, BH1745_show_type, NULL);//Add for EngineerMode

static struct attribute *BH1745_attributes[] = {
	&dev_attr_als_enable.attr,
	&dev_attr_als_batch.attr,
	&dev_attr_als_data.attr,  
	&dev_attr_type.attr,
	NULL
};

static const struct attribute_group BH1745_attr_group = {
	.attrs = BH1745_attributes,
};


static int rgb_power_on(struct RGB_DATA *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
	}

	return rc;
}

static int rgb_power_init(struct RGB_DATA *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int BH1745_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
#define ROHM_ALSMAX (65535)


	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct RGB_DATA *rgb;
//	struct device_node *np = client->dev.of_node;
	
	int err = 0;
	int dev_id;
	INFOR(" started.\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	rgb = kzalloc(sizeof(struct RGB_DATA), GFP_KERNEL);
	if (!rgb) {
		err = -ENOMEM;
		goto exit;
	}
	rgb->client = client;
	i2c_set_clientdata(client, rgb);

	err = rgb_power_init(rgb, true);
	if (err) {
		INFOR( "power init failed\n");
	}
	
	err = rgb_power_on(rgb, true);
	if (err) {
		INFOR("power on failed\n");
	}

	//check whether is BH1745 
	dev_id = i2c_smbus_read_byte_data(client, REG_MANUFACT_ID);
	if(dev_id != 0xE0){
	    kfree(rgb);
	    return -1;
	}
	rgb->type = 1;
	rgb->als_scale = 1;
	rgb->als_cali = CALI_FAIL;
	INFOR(" id(0x%x), this is BH1745!\n", dev_id);

	mutex_init(&rgb->update_lock);

	INIT_DELAYED_WORK(&rgb->als_dwork, BH1745_als_polling_work_handler); 

	/* Initialize the BH1745 chip */
	BH1745_init_client(client);

	rgb->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;
	rgb->resume = 0;
	/* Register to Input Device */
	rgb->input_dev_als = input_allocate_device();
	if (!rgb->input_dev_als) {
		err = -ENOMEM;
		INFOR(" Failed to allocate input device als\n");
		//goto exit_free_irq;
		goto exit_kfree;
	}
	
	input_set_drvdata(rgb->input_dev_als, rgb); 
	
	set_bit(EV_ABS, rgb->input_dev_als->evbit);

	input_set_abs_params(rgb->input_dev_als, ABS_MISC, 0, ROHM_ALSMAX, 0, 0);
	input_set_abs_params(rgb->input_dev_als, ABS_RED, 0, ROHM_ALSMAX, 0, 0);
	input_set_abs_params(rgb->input_dev_als, ABS_GREEN, 0, ROHM_ALSMAX, 0, 0);
	input_set_abs_params(rgb->input_dev_als, ABS_BLUE, 0, ROHM_ALSMAX, 0, 0);
	input_set_abs_params(rgb->input_dev_als, ABS_MISC, 0, ROHM_ALSMAX, 0, 0);

	rgb->input_dev_als->name = INPUT_ALS_NAME;
	rgb->input_dev_als->id.bustype = BUS_I2C;
  	rgb->input_dev_als->dev.parent =&rgb->client->dev;
	rgb->tptype = 0;// white tp as default


	err = input_register_device(rgb->input_dev_als);
	if (err) {
		err = -ENOMEM;
		INFOR(" Unable to register input device als: %s\n",
		       rgb->input_dev_als->name);
		goto exit_free_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &BH1745_attr_group);
	if (err)
	{
		INFOR(" sysfs_create_groupX\n");
		goto exit_unregister_dev_als;
	}

#if 1
	als_create_dbg_sys(&client->dev.kobj);
#endif

	err = meizu_sysfslink_register_name(&client->dev, "als");
	if( err<0 ) {
		INFOR("register als syslink failed\n");
	}
#ifdef CONFIG_AAL_CONTROL
	rgb->enable_als_type = RGB_ENABLE_ALS_NONE;
	g_als_client = client;
#endif
	INFOR(" support ver. %s enabled\n", DRIVER_VERSION);
	return 0;



exit_unregister_dev_als:
	INFOR(" exit_unregister_dev_als:\n");
	input_unregister_device(rgb->input_dev_als);
exit_free_dev_als:
	input_free_device(rgb->input_dev_als);
exit_kfree:
	kfree(rgb);
exit:
	return err;

#undef ROHM_ALSMAX
}

static int BH1745_remove(struct i2c_client *client)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);
	
	input_unregister_device(rgb->input_dev_als);
	
	input_free_device(rgb->input_dev_als);

	sysfs_remove_group(&client->dev.kobj, &BH1745_attr_group);

	/* Power down the device */
	BH1745_set_enable(client, 0);

	kfree(rgb);

	return 0;
}

static int BH1745_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);

	INFOR("\n");
	mutex_lock(&rgb->update_lock);
	rgb->suspend_enable = rgb->enable;
	BH1745_enable_als_sensor(client, 0);
	mutex_unlock(&rgb->update_lock);
	return 0;
}

static int BH1745_resume(struct i2c_client *client)
{
	struct RGB_DATA *rgb = i2c_get_clientdata(client);

	INFOR(" \n");
	mutex_lock(&rgb->update_lock);
	if(rgb->suspend_enable )
		BH1745_enable_als_sensor(client, 1);
	mutex_unlock(&rgb->update_lock);
	return 0;
}


MODULE_DEVICE_TABLE(i2c, BH1745_id);

static const struct i2c_device_id BH1745_id[] = {
	{ "BH1745", 0 },
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id rgb_match_table[] = {
                { .compatible = "rgb,bh1745",},
                { },
        };
#else
#define rgb_match_table NULL
#endif 

static struct i2c_driver BH1745_driver = {
	.driver = {
		.name	= BH1745_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table =rgb_match_table,
	},
	.suspend = BH1745_suspend,
	.resume	= BH1745_resume,
	.probe	= BH1745_probe,
	.remove	= BH1745_remove,
	.id_table = BH1745_id,
};

static struct i2c_board_info  bh1745_board_info[] = {
		{
				I2C_BOARD_INFO("BH1745", 0x38),
				//.irq = (CUST_EINT_GYRO_NUM),
				//.platform_data = &pdata,
		},
};

static int __init bh1745_board_init(void)
{
	INFOR("\n");
	i2c_register_board_info(1, bh1745_board_info, 1);
	return 0;
}


static int __init BH1745_init(void)
{
	INFOR("\n");
	return i2c_add_driver(&BH1745_driver);
}

static void __exit BH1745_exit(void)
{
	i2c_del_driver(&BH1745_driver);
}



postcore_initcall(bh1745_board_init);
module_init(BH1745_init);
module_exit(BH1745_exit);


MODULE_AUTHOR("Grace Huang @ ROHM");
MODULE_DESCRIPTION("BH1745 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
