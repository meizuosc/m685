
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/of_address.h>

#include "sensor-i2c.h"
#include "sensor-ps-dev.h"
#include "pa224-reg-dump.h"
#include "sensor-ps-adaptor.h"
#include "sensor-ps-statistics.h"

#define PA224_INT_GPIO_NUM 67
#define PA224_REG_DATA 0x0E
#define PA224_REG_CHIP_ID 0x7F

#define SNSOR_PS_DEV_PA224_MIN_OFFSET 0
#define SNSOR_PS_DEV_PA224_MAX_OFFSET 200

#define PA224_ADC_NEAR_THRESHOLD 20
#define PA224_ADC_NEAR_TO_FAR_TRIGGER 3

int sensor_ps_pa224_check_id(struct sensor_ps_dev *ps_dev)
{
	int ret;
	uint8_t val;

	ret = sensor_i2c_read_8bit(ps_dev->client, PA224_REG_CHIP_ID, &val);
	if(0x11 != val) {
		dev_err(&ps_dev->client->dev,
			"the chip id [%d] is not pa224, ret: %d\n",
			val, ret);
		return -1;
	}

	dev_info(&ps_dev->client->dev, "sensor ps dev is PA224\n");

	return 0;
}

int sensor_ps_pa224_get_name(struct sensor_ps_dev *ps_dev, const char **name)
{
	*name = "pa224-ps";
	return 0;
}

int sensor_ps_pa224_get_irq(struct sensor_ps_dev *ps_dev, unsigned int *irq)
{
	int ret;
	unsigned int ps_irq = 0;
	struct device_node *eint_node;

	eint_node = of_find_compatible_node(NULL, NULL,
		"mediatek,als_ps");
	if (eint_node) {
		ps_irq = irq_of_parse_and_map(eint_node, 0);
		if (ps_irq == 0) {
			pr_err("Parse map for ps eint failed\n");
			ret = -EINVAL;
			goto err_dts;
		} else {
			pr_info("ps eint get irq # %d\n", ps_irq);
		}
	} else {
		pr_err("can't find als_ps compatible node\n");
		ret = -ENXIO;
		goto err_dts;
	}

	*irq = ps_irq;
	return 0;

err_dts:
	return ret;
}

int sensor_ps_pa224_get_max_offset(struct sensor_ps_dev *ps_dev, int *offset)
{
	*offset = SNSOR_PS_DEV_PA224_MAX_OFFSET;
	return 0;
}

int sensor_ps_pa224_get_min_offset(struct sensor_ps_dev *ps_dev, int *offset)
{
	*offset = SNSOR_PS_DEV_PA224_MIN_OFFSET;
	return 0;
}

int sensor_ps_pa224_set_offset(struct sensor_ps_dev *ps_dev, int offset)
{

	int32_t adc_offset = 0;

	if (ps_dev->adc_offset != SENSOR_PS_NO_CALIBBIAS)
		adc_offset = ps_dev->adc_offset;

	/* we dont really set the offset to chip, we will add the offset
	   to the near threshold */
	ps_dev->adc_near   = adc_offset + PA224_ADC_NEAR_THRESHOLD;
	ps_dev->adc_far    = ps_dev->adc_near - PA224_ADC_NEAR_TO_FAR_TRIGGER;

	dev_info(&ps_dev->client->dev, "%s, adc near: %d, adc far: %d,"
		" offset:%d, ps_enable: %d\n", __func__, ps_dev->adc_near,
		ps_dev->adc_far, ps_dev->adc_offset, ps_dev->ps_enable);

	/* disable sensor hw */
	sensor_i2c_write_8bit(ps_dev->client, 0x00, 0x00);

	/* setup threshold */
	sensor_i2c_write_8bit(ps_dev->client, 0x08, ps_dev->adc_far);
	sensor_i2c_write_8bit(ps_dev->client, 0x0A, ps_dev->adc_near);

	/* enable sensor hw */
	sensor_i2c_write_8bit(ps_dev->client, 0x00, 0x02);

	return 0;
}

int sensor_ps_pa224_read_adc_data(struct sensor_ps_dev *ps_dev, uint16_t *adc)
{
	int ret;
	uint8_t raw_adc;

	ret = sensor_i2c_read_8bit(ps_dev->client, PA224_REG_DATA, &raw_adc);
	if (ret < 0)
		dev_err(&ps_dev->client->dev, "%s error, ret: %d\n",
			__func__, ret);
	else {
		if (raw_adc > ps_dev->adc_offset)
			*adc = raw_adc - ps_dev->adc_offset;
		else
			*adc = 0;

		dev_info(&ps_dev->client->dev, "%s, raw data: %d, "
			"adc offset: %d, adc data: %u\n",
			__func__, raw_adc, ps_dev->adc_offset, *adc);
	}

	return ret;
}

int sensor_ps_pa224_read_raw_adc_data(struct sensor_ps_dev *ps_dev, uint16_t *adc)
{
	int ret;
	uint8_t raw_adc;

	ret = sensor_i2c_read_8bit(ps_dev->client, PA224_REG_DATA, &raw_adc);
	if (ret < 0)
		dev_err(&ps_dev->client->dev, "%s error, ret: %d\n",
			__func__, ret);
	else
		*adc = raw_adc;

	return ret;
}

int sensor_ps_pa224_read_near_far_flag(struct sensor_ps_dev *ps_dev,
					uint8_t *flag)
{
	*flag = __gpio_get_value(PA224_INT_GPIO_NUM);
	return 0;
}

int sensor_ps_pa224_enable(struct sensor_ps_dev *ps_dev)
{
	int ret = 0;

	dev_info(&ps_dev->client->dev, "%s, adc near: %d, adc far: %d,"
		" offset:%d\n", __func__, ps_dev->adc_near,
		ps_dev->adc_far, ps_dev->adc_offset);

	return ret;
}

int sensor_ps_pa224_disable(struct sensor_ps_dev *ps_dev)
{
	dev_info(&ps_dev->client->dev, "%s\n", __func__);
	/* we keep ps always on for speed up screen turnon */
	return 0;
}

void sensor_ps_pa224_hw_init(struct sensor_ps_dev * ps_dev)
{
	/* disable sensor hw */
	sensor_i2c_write_8bit(ps_dev->client, 0x00, 0x00);

	sensor_i2c_write_8bit(ps_dev->client, 0x01, 0x6C);
	sensor_i2c_write_8bit(ps_dev->client, 0x02, 0x04);
	sensor_i2c_write_8bit(ps_dev->client, 0x03, 0x48);

	/* setup default threshold */
	sensor_i2c_write_8bit(ps_dev->client, 0x08, 0xFB);
	sensor_i2c_write_8bit(ps_dev->client, 0x0A, 0xFF);

	sensor_i2c_write_8bit(ps_dev->client, 0x10, 0x00);
	sensor_i2c_write_8bit(ps_dev->client, 0x11, 0x82);
	sensor_i2c_write_8bit(ps_dev->client, 0x12, 0x0C);

	/* enable sensor hw */
	sensor_i2c_write_8bit(ps_dev->client, 0x00, 0x02);
}

void sensor_ps_pa224_suspend(struct sensor_ps_dev * ps_dev)
{
	dev_info(&ps_dev->client->dev, "%s, ps_enabe: %d\n",
		 __func__, ps_dev->ps_enable);

	if (!ps_dev->ps_enable)
		/* disable sensor hw */
		sensor_i2c_write_8bit(ps_dev->client, 0x00, 0x00);
}

void sensor_ps_pa224_resume(struct sensor_ps_dev * ps_dev)
{
	dev_info(&ps_dev->client->dev, "%s, ps_enabe: %d\n",
		 __func__, ps_dev->ps_enable);

	/* enable sensor hw */
	sensor_i2c_write_8bit(ps_dev->client, 0x00, 0x02);
}

int sensor_ps_pa224_set_debug(struct sensor_ps_dev * ps_dev,
					const char *buf, size_t count)
{
	if (buf[0] != 'd')
		return -1;

	sensor_ps_statistics_dump();
	return count;
}

int sensor_ps_pa224_get_debug(struct sensor_ps_dev * ps_dev, char *buf)
{
	int i = 0;
	uint8_t data[20];
	struct pa224_regmap *regmap;

	for (i=0; i<0x13; i++)
		sensor_i2c_read_8bit(ps_dev->client, i, &data[i]);

	regmap = (struct pa224_regmap *)data;
	pa224_regmap_dump(regmap);
	return 0;
}

struct sensor_ps_ops sensor_ps_pa224_ops = {
	.check_id = &sensor_ps_pa224_check_id,
	.get_name = &sensor_ps_pa224_get_name,
	.get_irq  = &sensor_ps_pa224_get_irq,
	.get_max_offset = &sensor_ps_pa224_get_max_offset,
	.get_min_offset = &sensor_ps_pa224_get_min_offset,
	.set_offset = &sensor_ps_pa224_set_offset,
	.read_adc_data = &sensor_ps_pa224_read_adc_data,
	.read_raw_adc_data = &sensor_ps_pa224_read_raw_adc_data,
	.read_near_far_flag = &sensor_ps_pa224_read_near_far_flag,
	.enable = &sensor_ps_pa224_enable,
	.disable = &sensor_ps_pa224_disable,
	.hw_init = &sensor_ps_pa224_hw_init,
	.suspend = &sensor_ps_pa224_suspend,
	.resume = &sensor_ps_pa224_resume,
	.get_debug = &sensor_ps_pa224_get_debug,
	.set_debug = &sensor_ps_pa224_set_debug,
};

#define SENSOR_PS_I2C_DEV_NAME SENSOR_PS_DEVICE_NAME
static struct i2c_board_info sensor_ps_pa224_i2c_dev[] = {
	{
		.type = SENSOR_PS_I2C_DEV_NAME,
		.addr = 0x1e,
		.platform_data = &sensor_ps_pa224_ops,
	}
};

static int __init sensor_ps_pa224_module_init(void)
{
	i2c_register_board_info(1, sensor_ps_pa224_i2c_dev, 1);
	return 0;
}

postcore_initcall(sensor_ps_pa224_module_init);

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor ps device");
MODULE_LICENSE("GPL");
