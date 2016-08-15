#include <linux/module.h>
#include <linux/init.h>
#define CONFIG_MTK_I2C_EXTENSION
#include <linux/i2c.h>
#undef CONFIG_MTK_I2C_EXTENSION
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/types.h>

#include <linux/platform_device.h>

#include <linux/leds.h>

#define LM36272_DEV_NAME "integrate-backlight"

#define LM36272_MAX_VALUE_SETTINGS 2048	/* value leds_brightness_set */

/* I2C variable */
static struct i2c_client *lm36272_client;
static const struct i2c_device_id lm36272_i2c_id[] = { {LM36272_DEV_NAME, 0}, {} };

/* Flash control */
unsigned char strobe_ctrl;
unsigned char flash_ctrl;
unsigned char flash_status;

/* static unsigned char current_brightness; */
static unsigned char is_suspend;
static unsigned int lm36272_hbm;

struct semaphore lm36272_lock;

/* generic */
#define LM36272_MAX_RETRY_I2C_XFER (100)
#define LM36272_I2C_WRITE_DELAY_TIME 1

#define LM36272_REVISION_REG	0x01
#define LM36272_BL_CONFIG1		0x02
#define LM36272_BL_CONFIG2		0x03
#define LM36272_BL_LSB			0x04
#define LM36272_BL_MSB			0x05
#define LM36272_BL_AFLT			0x06
#define LM36272_BL_AFHT			0x07
#define LM36272_BL_EN			0x08
#define LM36272_BIAS_CONFIG1	0x09
#define LM36272_BIAS_CONFIG2	0x0A
#define LM36272_BIAS_CONFIG3	0x0B
#define LM36272_LCM_OUT			0x0C
#define LM36272_BIAS_POS		0x0D
#define LM36272_BIAS_NEG		0x0E
#define LM36272_FLAG			0x0F
#define LM36272_OPTION1			0x10
#define LM36272_OPTION2			0x11

static int lm36272_write_byte(struct i2c_client *client,
				   unsigned char reg_addr, unsigned char data)
{
	int ret_val = 0;
	int i = 0;
	unsigned char write_data[2] = {0,};

	if (lm36272_client == NULL)
		return -ENODEV;
	
	write_data[0] = reg_addr;
	write_data[1] = data;

	lm36272_client->ext_flag = ((lm36272_client->ext_flag & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG);
	ret_val = i2c_master_send(client, write_data, 2);
	lm36272_client->ext_flag = 0;

	if (ret_val != 2)
		return ret_val;
	else
		return 0;
}

unsigned int lm36272_set_brightness(unsigned int level)
{
	int ret_code = 0;
	unsigned char lsb = 0;
	unsigned char msb = 0;
	unsigned int level_a = level;
	static int last_level = 0;

	if (last_level == level_a)
		return 0;

	pr_err("%s: in level %d, out level %d\n", __func__, level, level_a );
	lsb = (level_a & 0x7);
	msb = (level_a >> 3);

	down_interruptible(&lm36272_lock);

	ret_code = lm36272_write_byte(lm36272_client, LM36272_BL_LSB, lsb);
	if (ret_code) {
		pr_err("[BACKLIGHT] write lsb fail!\n");
		goto out;
	}

	ret_code = lm36272_write_byte(lm36272_client, LM36272_BL_MSB, msb);
	if (ret_code) {
		pr_err("[BACKLIGHT] write msb fail!\n");
		goto out;
	}

	if (level_a == 0) {
		ret_code = lm36272_write_byte(lm36272_client, LM36272_BL_EN, 0);
		if (ret_code) {
			pr_err("[BACKLIGHT] write en fail!\n");
			goto out;
		}
	}

	if (last_level == 0 && level_a != 0) {
		ret_code = lm36272_write_byte(lm36272_client, LM36272_BL_EN, 0x13);
		if (ret_code) {
			pr_err("[BACKLIGHT] write en fail!\n");
			goto out;
		}
	}

	last_level = level_a;
out:
	up(&lm36272_lock);
	return ret_code;
}
EXPORT_SYMBOL(lm36272_set_brightness);
unsigned int lm36272_set_bias(unsigned int onoff)
{
	int ret_code = 0;
	
	down_interruptible(&lm36272_lock);

	ret_code = lm36272_write_byte(lm36272_client, LM36272_BIAS_POS, 0x1E);
	if (ret_code) {
		pr_err("[BACKLIGHT] write lsb fail!\n");
		goto out;
	}

	ret_code = lm36272_write_byte(lm36272_client, LM36272_BIAS_NEG, 0x1E);
	if (ret_code) {
		pr_err("[BACKLIGHT] write msb fail!\n");
		goto out;
	}

out:
	up(&lm36272_lock);
	return ret_code;
}
static int lm36272_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	lm36272_client = client;

	sema_init(&lm36272_lock, 1);

	if (client == NULL)
		pr_info("%s client is NULL\n", __func__);
	else
		pr_info("%s %p %x %x\n", __func__, client->adapter, client->addr, client->flags);

	return 0;
}

static int lm36272_remove(struct i2c_client *client)
{
	lm36272_client = NULL;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lm36272_of_match[] = {
	{.compatible = "mediatek,lcd_bl_bias"},
	{},
};
#endif				/* CONFIG_OF */

static struct i2c_driver lm36272_i2c_driver = {
	.driver = {
		   .name = LM36272_DEV_NAME,
#ifdef CONFIG_OF
		   .of_match_table = lm36272_of_match,
#endif				/* CONFIG_OF */
		   },
	.probe = lm36272_probe,
	.remove = lm36272_remove,
	.id_table = lm36272_i2c_id,
};

static int __init lm36272_init(void)
{
	i2c_add_driver(&lm36272_i2c_driver);
	return 0;
}

static void __exit lm36272_exit(void)
{
	i2c_del_driver(&lm36272_i2c_driver);
}
module_param(lm36272_hbm, uint, 0664);
MODULE_PARM_DESC(lm36272_hbm, "Debug Print Log Lvl");
EXPORT_SYMBOL(lm36272_flash_strobe_en);

MODULE_AUTHOR("Mars<caoziqiang@meizu.com>");
MODULE_DESCRIPTION("Ti lm36272 driver");
MODULE_LICENSE("GPL");

module_init(lm36272_init);
module_exit(lm36272_exit);
