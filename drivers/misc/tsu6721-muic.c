/*
 * tsu6721a-muic.c - MUIC driver for the Maxim 14656
 *
 *  Copyright (C) 2013 Meizu Technology Co.Ltd
 *  <tangxingyan@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/tsu6721-muic.h>

typedef enum {
	CHARGER_UNKNOWN = 0,
	STANDARD_HOST,		/* USB : 450mA */
	CHARGING_HOST,
	NONSTANDARD_CHARGER,	/* AC : 450mA~1A */
	STANDARD_CHARGER,	/* AC : ~1A */
	APPLE_2_1A_CHARGER,	/* 2.1A apple charger */
	APPLE_1_0A_CHARGER,	/* 1A apple charger */
	APPLE_0_5A_CHARGER,	/* 0.5A apple charger */
	WIRELESS_CHARGER,
} CHARGER_TYPE;

static struct tsu6721_muic_info *g_info = NULL;

static int tsu6721_read_reg(struct tsu6721_muic_info *tsu6721, u8 reg, u8 *dest)
{
    struct i2c_client *client = tsu6721->client;
    int      ret=0;

    mutex_lock(&tsu6721->tsu6721_i2c_access);    


    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0) 
    {    
	pr_err("send command error!!\n");

        mutex_unlock(&tsu6721->tsu6721_i2c_access);
        return ret;
    }
    
    *dest = (u8)ret;

    mutex_unlock(&tsu6721->tsu6721_i2c_access);    
    return 0;
}

static int tsu6721_write_reg(struct tsu6721_muic_info *tsu6721, u8 reg, u8 value)
{
    struct i2c_client *client = tsu6721->client;
    char    write_data[2] = {0};
    int     ret=0;

    mutex_lock(&tsu6721->tsu6721_i2c_access);
    
    write_data[0] = reg;
    write_data[1] = value;
    ret = i2c_master_send(client, write_data, 2);
    if (ret < 0) 
    {
	pr_err("send command error!!\n");
        mutex_unlock(&tsu6721->tsu6721_i2c_access);
        return ret;
    }
    mutex_unlock(&tsu6721->tsu6721_i2c_access);
    return 0;
}

static int tsu6721_update_reg(struct tsu6721_muic_info *tsu6721, u8 reg, u8 val, u8 mask)
{
    struct i2c_client *client = tsu6721->client;
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(client, reg, new_val);
	}
	return ret;
}

static int init_tsu6721_muic(struct tsu6721_muic_info *info)
{
	int ret;
	u8 val = 0, msk = 0, adc;
        u8 status2;
	u8 data = 0;

        /*
        unmask Interrupt,set automatic switch, and auto switch by accessory status.
        */
	msk = INT_MASK | MANUAL_SWTICH_MASK | SWTICH_OPEN_MASK; 
	val = (0 << INT_MASK_SHIFT) | (1 << MANUAL_SWITCH_SHIFT) | (1 << SWITCH_OPEN_SHIFT);
	ret = tsu6721_update_reg(info, TSU6721_MUIC_CONTROL, val, msk);
	if (ret < 0) {
		pr_err("[MUIC]: can not update reg_ctrl\n");
		return ret;
	}

	/* Interrupt Mask 1, enable ATTACH,DETACH, DEOVP, OCP, disable OVP_OCP_OTP_DIS*/
	val = msk = 0;
	val = (0 << ATTACH_SHIFT) | (0 << DETACH_SHIFT); 
	msk = ATTACH_MASK | DETACH_MASK;
	ret = tsu6721_update_reg(info, TSU6721_MUIC_INTMASK1, val, msk);
	if (ret < 0) {
		pr_err("[MUIC]: can not update interrupt mask 1\n");
		return ret;
	}

	/* Interrupt Mask 2, vbus*/
	val = msk = 0;
	val = 0 << VBUS_SHIFT;
	msk = VBUS_MASK;
	ret = tsu6721_update_reg(info, TSU6721_MUIC_INTMASK2, val, msk);
	if (ret < 0) {
		pr_err("[MUIC]: can not update interrupt mask 2\n");
		return ret;
	}

	/* set the BCD V1.2 Timer, to decrease the charger type detection timer
	 now set the timer is 0.6S*/
	val = msk = 0;
	val = 0 << 3;
	msk = 7 << 3; 
	ret = tsu6721_update_reg(info, TSU6721_MUIC_TIMER_SET, val, msk);	

	return ret;
}

void tsu6721_dp_switch_to_sr(void)
{
	int ret;
	ret = tsu6721_update_reg(g_info, TSU6721_MUIC_CONTROL, 0 << 2, 1 <<2);
	/* DP is connect S_R, DM is connect S_L*/
	ret = tsu6721_update_reg(g_info, TSU6721_MUIC_MANUAL_SW1,
			(0x2 << 2) | (0x2 << 5), (0x7 << 2) | (0x7 << 2));
}

void tsu6721_auto_switch(void)
{
	int ret;

	ret = tsu6721_update_reg(g_info, TSU6721_MUIC_CONTROL, 1 << 2, 1 <<2);
}

static void tsu6721_dp_switch_to_mic(void)
{
	int ret;
	ret = tsu6721_update_reg(g_info, TSU6721_MUIC_CONTROL, 0 << 2, 1 <<2);
	ret = tsu6721_update_reg(g_info, TSU6721_MUIC_MANUAL_SW1, (0x5 << 2), (0x7 << 2));
}

void tsu6721_charger_type(void *data)
{
        u8 type3 = 0, type1 = 0;
	int status = false, ret;
	CHARGER_TYPE charger_type = CHARGER_UNKNOWN;

	tsu6721_read_reg(g_info, TSU6721_MUIC_DEVICE_TYPE1, &type1);
	tsu6721_read_reg(g_info, TSU6721_MUIC_DEVICE_TYPE3, &type3);
        
	if (type1 & 0x40) {
		charger_type = STANDARD_CHARGER;
    	} else if (type1 & 0x04) {
		charger_type = STANDARD_HOST;
    	} else if (type3 & 0x04) {
		charger_type = NONSTANDARD_CHARGER;
    	} else if(type1 & 0x20) {
		charger_type = CHARGING_HOST;
	} else if (type3 & 0x20) {
		charger_type = APPLE_2_1A_CHARGER;
	}

	*(int *)data = charger_type;
}
EXPORT_SYMBOL_GPL(tsu6721_charger_type);

static irqreturn_t muic_eint_handler(int irq, void *desc)
{
        u8 int1 = 0, int2 = 0; 

	printk("%s:*****************\n", __func__);
	/* clear the interrupt: read is clear them */
        tsu6721_read_reg(g_info, TSU6721_MUIC_INT1, &int1);
        tsu6721_read_reg(g_info, TSU6721_MUIC_INT2, &int2);

	return IRQ_HANDLED;
}

static int muic_irq_init(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	int muic_irq;

	node = of_find_compatible_node(NULL, NULL, "mediatek,muic-eint");

	if (node) {
		muic_irq = irq_of_parse_and_map(node, 0);

		ret = request_irq(muic_irq, (irq_handler_t)muic_eint_handler, IRQF_TRIGGER_LOW,
					"muic-eint", NULL);
	} else {
		printk("muic request_irq can not find touch eint device node!.");
	}

	return ret;
}


static int tsu6721_muic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tsu6721_muic_info *info;
	int ret = 0, i;
	int irq;
	u8 dev_id;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	info = kzalloc(sizeof(struct tsu6721_muic_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "%s: failed to allocate info\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

        info->dev = &client->dev;
        info->client = client;
        i2c_set_clientdata(client, info);
        g_info = info;

        mutex_init(&info->tsu6721_i2c_access);

	ret = tsu6721_read_reg(info, TSU6721_MUIC_DEVICE_ID, &dev_id);	
	if (dev_id != TSU6721_DEV_ID) {
		pr_err("%s: The Device is not the Ti TSU6721\n", __func__);
		goto err_return;
    	}

        ret = init_tsu6721_muic(info);
        if (ret < 0) {
                dev_err(&client->dev, "Failed to initialize MUIC:%d\n", ret);
                goto fail0;
        }

	muic_irq_init();

	printk("%s:probe\n", __func__);
        return 0;

fail0:
        i2c_set_clientdata(client, NULL);
        kfree(info);
err_return:
        return ret;
}

static int tsu6721_muic_remove(struct i2c_client *client)
{
	struct tsu6721_muic_info *info = container_of(client, struct tsu6721_muic_info, client);

        i2c_set_clientdata(client, NULL);
        kfree(info);
	return 0;
}

void tsu6721_muic_shutdown(struct i2c_client *client)
{
	return ;
}

#ifdef CONFIG_PM
static int tsu6721_muic_suspend(struct device *dev)
{
	struct tsu6721_muic_info *info = dev_get_drvdata(dev);
	return 0;
}

static int tsu6721_muic_resume(struct device *dev)
{
	struct tsu6721_muic_info *info = dev_get_drvdata(dev);

	return 0;
}

static const struct dev_pm_ops tsu6721_pm_ops = {
	.suspend        = tsu6721_muic_suspend,
	.resume		= tsu6721_muic_resume,
};
#endif

static const struct i2c_device_id tsu6721_id[] = {
    {"tsu6721", 0},
};

static struct i2c_driver tsu6721_muic_driver = {
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm     =  &tsu6721_pm_ops,
#endif
	},
	.probe		= tsu6721_muic_probe,
	.shutdown 	= tsu6721_muic_shutdown,
	.remove		= tsu6721_muic_remove,
	.id_table   	= tsu6721_id,
};

static int __init tsu6721_muic_init(void)
{
	return i2c_add_driver(&tsu6721_muic_driver);
}
module_init(tsu6721_muic_init);

static void __exit tsu6721_muic_exit(void)
{
	i2c_del_driver(&tsu6721_muic_driver);
}
module_exit(tsu6721_muic_exit);

MODULE_DESCRIPTION("Ti tsu6721 MUIC driver");
MODULE_AUTHOR("<tangxingyan@meizu.com");
MODULE_LICENSE("GPL");
