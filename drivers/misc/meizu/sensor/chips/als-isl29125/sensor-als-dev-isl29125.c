
#include <linux/delay.h>
#include <linux/module.h>

#include "sensor-i2c.h"
#include "sensor-als-dev.h"
#include "isl29125-reg-dump.h"
#include "sensor-als-adaptor.h"

#define ISL29125_REG_CHIP_ID	0x00

#define WHITE_HIGH_LUX_COEF	6075
#define BLACK_HIGH_LUX_COEF	5406
#define WHITE_LOW_LUX_COEF	224
#define BLACK_LOW_LUX_COEF	310

#define COVER_TYPE_BLACK 0
#define COVER_TYPE_WHITE 1

extern unsigned int primary_display_get_tp_id(void);
#define GET_COVER_TYPE() (primary_display_get_tp_id())

int sensor_als_isl29125_check_id(struct sensor_als_dev *als_dev)
{
	int ret;
	uint8_t val;

	ret = sensor_i2c_read_8bit(als_dev->client, ISL29125_REG_CHIP_ID, &val);
	if(0x7D != val) {
		dev_err(&als_dev->client->dev,
			"the chip id [%d] is not isl29125, ret: %d\n",
			val, ret);
		return -1;
	}

	dev_info(&als_dev->client->dev, "sensor als dev is ISL29125\n");

	return 0;
}

int sensor_als_isl29125_get_max_sample_freq(struct sensor_als_dev *als_dev,
							int32_t *sample_freq)
{
	/* max sample freq is 10 HZ */
	*sample_freq = 10;
	return 0;
}

int sensor_als_isl29125_get_name(struct sensor_als_dev *als_dev,
							const char **name)
{
	*name = "isl29125-als";
	return 0;
}

static void sensor_als_isl29125_switch_range(
	struct sensor_als_dev *als_dev, uint32_t green_raw)
{

	switch(als_dev->als_range) {
	case 375:
		/* Switch to 10000 lux */
		if(green_raw > 0xCCCC) {
			als_dev->als_range = 10000;

			if (COVER_TYPE_BLACK == als_dev->cover_type)
				als_dev->coefficient = BLACK_HIGH_LUX_COEF;
			else
				als_dev->coefficient = WHITE_HIGH_LUX_COEF;

			dev_info(&als_dev->client->dev,
				"switch to range 10000, cover type: %d\n",
				als_dev->cover_type);
			sensor_i2c_write_8bit(als_dev->client, 0x01, 0x09);
		}
		break;

	case 10000:
		/* Switch to 375 lux */
		if(green_raw < 0xCCC) {
			als_dev->als_range = 375;
			if (COVER_TYPE_BLACK == als_dev->cover_type)
				als_dev->coefficient = BLACK_LOW_LUX_COEF;
			else
				als_dev->coefficient = WHITE_LOW_LUX_COEF;

			dev_info(&als_dev->client->dev,
				"switch to range 375, cover type: %d\n",
				als_dev->cover_type);
			sensor_i2c_write_8bit(als_dev->client, 0x01, 0x01);
		}
		break;

	default:
		als_dev->als_range = 375;
		dev_info(&als_dev->client->dev, "set range 375\n");
		sensor_i2c_write_8bit(als_dev->client, 0x01, 0x01);
		break;
	}
}

int sensor_als_isl29125_get_lux(struct sensor_als_dev *als_dev, uint32_t *lux)
{
	int ret = 0;
	uint16_t green_raw;
	uint32_t cur_lux;

	ret = sensor_i2c_read_16bits(als_dev->client, 0x09, &green_raw);
	if (ret < 0)
		return ret;

	cur_lux = ((als_dev->coefficient * green_raw)>>8)/10;

	/* debounce the zero*/
	if ((als_dev->pre_lux != 0) && (cur_lux == 0)) {
		dev_info(&als_dev->client->dev, "%s get 0 lux\n", __func__);
		cur_lux = als_dev->pre_lux;
		als_dev->pre_lux = 0;
	} else {
		als_dev->pre_lux = cur_lux;
	}

	sensor_als_isl29125_switch_range(als_dev, green_raw);

	// *lux = cur_lux > U16_MAX ? U16_MAX : cur_lux;
	*lux = cur_lux;

	return ret;
}

int sensor_als_isl29125_enable(struct sensor_als_dev *als_dev)
{
	dev_info(&als_dev->client->dev, "%s\n", __func__);
	/* we enabled the als when driver probe and keep als always on */
	dev_info(&als_dev->client->dev, "cover type: %d\n", als_dev->cover_type);
	return 0;
}

int sensor_als_isl29125_disable(struct sensor_als_dev *als_dev)
{
	dev_info(&als_dev->client->dev, "%s\n", __func__);
	return 0;
}

void sensor_als_isl29125_hw_init(struct sensor_als_dev * als_dev)
{
	/* reset the als device */
	sensor_i2c_write_8bit(als_dev->client, 0x00, 0x46);
	msleep(20);

	/* enable green channel */
	sensor_i2c_write_8bit(als_dev->client, 0x01, 0x01);
	sensor_i2c_write_8bit(als_dev->client, 0x02, 0x00);
	sensor_i2c_write_8bit(als_dev->client, 0x03, 0x00);
	sensor_i2c_write_8bit(als_dev->client, 0x04, 0x00);
	sensor_i2c_write_8bit(als_dev->client, 0x05, 0x00);
	sensor_i2c_write_8bit(als_dev->client, 0x06, 0xFF);
	sensor_i2c_write_8bit(als_dev->client, 0x07, 0xFF);

	als_dev->als_range = 375;
	if (COVER_TYPE_BLACK == GET_COVER_TYPE()) {
		als_dev->cover_type  = COVER_TYPE_BLACK;
		als_dev->coefficient = BLACK_LOW_LUX_COEF;
	} else {
		als_dev->cover_type  = COVER_TYPE_WHITE;
		als_dev->coefficient = WHITE_LOW_LUX_COEF;
	}

	dev_info(&als_dev->client->dev, "%s cover type is %s\n",
		__func__,
		(als_dev->cover_type==COVER_TYPE_BLACK)?"black":"white");
}

void sensor_als_isl29125_suspend(struct sensor_als_dev * als_dev)
{
	dev_info(&als_dev->client->dev, "%s\n", __func__);
}

void sensor_als_isl29125_resume(struct sensor_als_dev * als_dev)
{
	dev_info(&als_dev->client->dev, "%s\n", __func__);
}

int sensor_als_isl29125_set_debug(struct sensor_als_dev * als_dev,
					const char *buf, size_t count)
{
	return -1;
}

int sensor_als_isl29125_get_debug(struct sensor_als_dev * als_dev, char *buf)
{
	int i = 0;
	uint8_t data[20];
	struct isl29125_regmap *regmap;

	for (i=0; i<0x0E; i++)
		sensor_i2c_read_8bit(als_dev->client, i, &data[i]);

	regmap = (struct isl29125_regmap *)data;
	isl29125_regmap_dump(regmap);
	return 0;
}

struct sensor_als_ops sensor_als_isl29125_ops = {
	.check_id = &sensor_als_isl29125_check_id,
	.get_name = &sensor_als_isl29125_get_name,
	.get_lux  = &sensor_als_isl29125_get_lux,
	.enable   = &sensor_als_isl29125_enable,
	.disable  = &sensor_als_isl29125_disable,
	.hw_init  = &sensor_als_isl29125_hw_init,
	.suspend  = &sensor_als_isl29125_suspend,
	.resume   = &sensor_als_isl29125_resume,
	.set_debug = &sensor_als_isl29125_set_debug,
	.get_debug = &sensor_als_isl29125_get_debug,
	.get_max_sample_freq = &sensor_als_isl29125_get_max_sample_freq,
};

#define SENSOR_ALS_I2C_DEV_NAME SENSOR_ALS_DEVICE_NAME
static struct i2c_board_info sensor_als_isl29125_i2c_dev[] = {
	{
		.type = SENSOR_ALS_I2C_DEV_NAME,
		.addr = 0x44,
		.platform_data = &sensor_als_isl29125_ops,
	}
};

static int __init sensor_als_isl29125_module_init(void)
{
	i2c_register_board_info(1, sensor_als_isl29125_i2c_dev, 1);
	return 0;
}

postcore_initcall(sensor_als_isl29125_module_init);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor ps device");
MODULE_LICENSE("GPL");
