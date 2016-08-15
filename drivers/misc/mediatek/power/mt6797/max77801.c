#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/module.h>

#include "max77801.h"


#define PMIC_DEBUG_PR_DBG
/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define max77801_SLAVE_ADDR_WRITE	0x30
#define max77801_SLAVE_ADDR_READ	0x31

#define max77801_BUSNUM			0

static struct i2c_client *new_client;
static const struct i2c_device_id max77801_i2c_id[] = { {"max77801", 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id max77801_of_ids[] = {
	{.compatible = "mediatek,max77801"},
	{},
};
#endif

static int max77801_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

static struct i2c_driver max77801_driver = {
	.driver = {
		   .name = "max77801",
#ifdef CONFIG_OF
		   .of_match_table = max77801_of_ids,
#endif
		   },
	.probe = max77801_driver_probe,
	.id_table = max77801_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
static DEFINE_MUTEX(max77801_i2c_access);
static DEFINE_MUTEX(max77801_lock_mutex);

#define PMICTAG                "[max77801] "
#if defined(PMIC_DEBUG_PR_DBG)
#define PMICLOG1(fmt, arg...)   pr_err(PMICTAG fmt, ##arg)
#else
#define PMICLOG1(fmt, arg...)
#endif

/**********************************************************
  *
  *   [I2C Function For Read/Write max77801]
  *
  *********************************************************/

unsigned int max77801_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&max77801_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
			 .addr = new_client->addr,
			 .flags = 0,
			 .len = 1,
			 .buf = &cmd,
			 }, {

			     .addr = new_client->addr,
			     .flags = I2C_M_RD,
			     .len = 1,
			     .buf = returnData,
			     }
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			PMICLOG1("skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&max77801_i2c_access);

	return ret == xfers ? 1 : -1;
}

unsigned int max77801_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&max77801_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
			 .addr = new_client->addr,
			 .flags = 0,
			 .len = 1 + 1,
			 .buf = buf,
			 },
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			PMICLOG1("skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
	} while (ret != xfers && --retries);

	mutex_unlock(&max77801_i2c_access);

	return ret == xfers ? 1 : -1;
}


/*
 *   [Read / Write Function]
 */
unsigned int max77801_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				     unsigned char SHIFT)
{
	unsigned char max77801_reg = 0;
	unsigned int ret = 0;

	/* PMICLOG1("--------------------------------------------------\n"); */

	ret = max77801_read_byte(RegNum, &max77801_reg);

	/* PMICLOG1("[max77801_read_interface] Reg[%x]=0x%x\n", RegNum, max77801_reg); */

	max77801_reg &= (MASK << SHIFT);
	*val = (max77801_reg >> SHIFT);

	/* PMICLOG1("[max77801_read_interface] val=0x%x\n", *val); */

	return ret;
}

unsigned int max77801_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				       unsigned char SHIFT)
{
	unsigned char max77801_reg = 0;
	unsigned int ret = 0;

	/*PMICLOG1("--------------------------------------------------\n"); */

	ret = max77801_read_byte(RegNum, &max77801_reg);
	/* PMICLOG1("[max77801_config_interface] Reg[%x]=0x%x\n", RegNum, max77801_reg); */

	max77801_reg &= ~(MASK << SHIFT);
	max77801_reg |= (val << SHIFT);

	ret = max77801_write_byte(RegNum, max77801_reg);
	/*PMICLOG1("[max77801_config_interface] write Reg[%x]=0x%x\n", RegNum, max77801_reg); */

	/* Check */
	/*ret = max77801_read_byte(RegNum, &max77801_reg);
	   PMICLOG1("[max77801_config_interface] Check Reg[%x]=0x%x\n", RegNum, max77801_reg);
	 */

	return ret;
}

void max77801_set_reg_value(unsigned int reg, unsigned int reg_val)
{
	unsigned int ret = 0;

	ret = max77801_config_interface((unsigned char)reg, (unsigned char)reg_val, 0xFF, 0x0);
}

unsigned int max77801_get_reg_value(unsigned int reg)
{
	unsigned int ret = 0;
	unsigned char reg_val = 0;

	ret = max77801_read_interface((unsigned char)reg, &reg_val, 0xFF, 0x0);

	return reg_val;
}

/*
 *   [APIs]
 */
void max77801_lock(void)
{
	mutex_lock(&max77801_lock_mutex);
}

void max77801_unlock(void)
{
	mutex_unlock(&max77801_lock_mutex);
}


/*
 *   [Internal Function]
 */
void max77801_dump_register(void)
{
	u8 reg_addrs[] = {
		max77801_DEVICE_ID,
		max77801_STATUS,
		max77801_CONTROL1,
		max77801_CONTROL2,
		max77801_VOUT_DVS_L,
		max77801_VOUT_DVS_H
		};
	int arrSize = sizeof(reg_addrs)/sizeof(reg_addrs[0]);
	u8 addr;
	u8 val;
    
            PMICLOG1("[max77801_dump_register] max77801 reg:\n");
            
	for (addr = 0; addr < arrSize; addr++) {
		max77801_read_byte(&val, reg_addrs[addr]);
		PMICLOG1("[max77801_dump_register] Reg[0x%X]=0x%X\n", reg_addrs[addr], val);
	}
            PMICLOG1("\n**************************************************\n");

}



int max77801_vosel(unsigned long val)
{
	int ret = 1;
	unsigned long reg_val = 0;

	/* 0.603~4.1875V (step 12.5mv) */
	reg_val = (val -26000) / 125;

	if (reg_val > 0x7f)
		reg_val = 0x7f;

	ret = max77801_write_byte(max77801_VOUT_DVS_H, reg_val);

	pr_notice("[max77801_vosel] val=%ld, reg_val=0x%2x\n", val, reg_val);

	return ret;
}

static int max77801_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	PMICLOG1("[max77801_driver_probe]\n");
    
	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (new_client == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;

          max77801_vosel(35000);    // out voltage : 3.5v

  //      max77801_dump_register();
    
	return 0;

exit:
	PMICLOG1("[max77801_driver_probe] exit: return err\n");
	return err;
}

static int __init max77801_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&max77801_driver) != 0)
		PMICLOG1("[max77801_init] failed to register max77801 i2c driver.\n");
	else
		PMICLOG1("[max77801_init] Success to register max77801 i2c driver.\n");

	return 0;
}

static void __exit max77801_exit(void)
{
	i2c_del_driver(&max77801_driver);
}
module_init(max77801_init);
module_exit(max77801_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C max77801 Driver");
MODULE_AUTHOR("lyf");
