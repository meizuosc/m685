/*
 * STMicroelectronics lsm6ds3 i2c driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#define CONFIG_MTK_I2C_EXTENSION
#include <linux/i2c.h>
#undef CONFIG_MTK_I2C_EXTENSION
#include <linux/iio/iio.h>
#include <linux/delay.h>
#include "st_lsm6ds3.h"
//#include <mach/eint.h>
//#include <mach/mt_gpio.h>
//#include <cust_eint.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
//#include "cust_gpio_usage.h"
#include <linux/platform_data/st_sensors_pdata.h>
#include <linux/dma-mapping.h>

#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5

#if 0
static void *gpDMABuf_va;
static dma_addr_t gpDMABuf_pa=0;
#endif

static int lsm6ds3_i2c_read(struct i2c_client *client, u8 *buffer,
								  int len);

#define LSM_DMA_MAX_TRANSACTION_LEN  8 * 1024 /* 8K fifo data */
#define ST_LSM6DS3_FIFO_DATA_OUT_L		0x3e
int st_lsm6ds3_i2c_read_fifo(struct lsm6ds3_data *cdata, int len, u8 *data)
{
	int ret = 0;
	data[0] = ST_LSM6DS3_FIFO_DATA_OUT_L;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	if (len > LSM_DMA_MAX_TRANSACTION_LEN) {
		printk(KERN_EMERG "__lsm6ds3_read_fifo len error\n");
		return -1;
	}

	mutex_lock(&cdata->bank_registers_lock);
	ret = lsm6ds3_i2c_read(client, data, len);
	mutex_unlock(&cdata->bank_registers_lock);
	if(ret < 0) {
		printk(KERN_EMERG "_lsm6ds3_read_fifo i2c_transfer error, ret: %d\n", ret);
	}

	return ret;
}

#define I2C_TRANS_TIMING (100)
#define I2C_WR_MAX (7)
static int lsm6ds3_i2c_write(struct i2c_client *client, u8 *buffer, int len)//len : size of data,expect addr
{

#if 1
    int length = len;
    int writebytes = 0;
    int ret = 0;
    u8 mem_addr = buffer[0];
    u8  *ptr = buffer+1;
    unsigned char buf[I2C_WR_MAX + 1];

    struct i2c_msg msgs[3]={{0}, };

    memset(buf, 0, sizeof(buf));

    while(length > 0)
    {
              if (length > I2C_WR_MAX)
                       writebytes = I2C_WR_MAX;
              else
                       writebytes = length;

				memset(buf, 0, sizeof(buf));
				buf[0] = mem_addr;
				memcpy(buf+1, ptr, writebytes);
				msgs[0].addr = client->addr & I2C_MASK_FLAG;
				msgs[0].flags = 0;
				msgs[0].buf = buf;
				msgs[0].len = writebytes+1;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

          	ret = i2c_transfer(client->adapter, msgs, 1);
              if (ret!=1)
              {
                       printk("lsm: i2c transfer error ret:%d, write_bytes:%d, Line %d\n", ret, writebytes+1, __LINE__);
                       return -1;
              }

              length -= writebytes;
              mem_addr += writebytes;
              ptr += writebytes;
    }

    return len;
#endif
}

static int lsm6ds3_i2c_read(struct i2c_client *client, u8 *buffer,
								  int len)
{
	int ret;
	int tries = 0;
	u8 reg = buffer[0];
	u8 cmd = reg;

	do {
		ret = i2c_master_send(client, &cmd, sizeof(cmd));
		if (ret != sizeof(cmd))
			msleep_interruptible (I2C_RETRY_DELAY);
	} while ( (ret != sizeof(cmd)) && (++tries < I2C_RETRIES));
        
	if (ret != sizeof(cmd))
	{
		dev_err(&client->dev, "i2c_master_send error,ret = %d\n",ret);
		return ret;
	}

	ret = i2c_master_recv(client, (char *)buffer, len);
	if (ret < 0)
	{
		dev_err(&client->dev, "i2c_master_recv error,ret = %d\n",ret);
		return ret;
	}
	return ret;
}

static int st_lsm6ds3_i2c_read(struct lsm6ds3_data *cdata,
				u8 reg_addr, int len, u8 *data, bool b_lock)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	data[0] = reg_addr;
	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);
		ret = lsm6ds3_i2c_read(client, data, len);

		mutex_unlock(&cdata->bank_registers_lock);
	} else
		ret = lsm6ds3_i2c_read(client, data, len);

	return ret;
}

static int st_lsm6ds3_i2c_write(struct lsm6ds3_data *cdata,
				u8 reg_addr, int len, u8 *data, bool b_lock)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	struct i2c_client *client = to_i2c_client(cdata->dev);

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
//	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	if (b_lock) {
		mutex_lock(&cdata->bank_registers_lock);

		lsm6ds3_i2c_write(client,send, len);
//		err = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&cdata->bank_registers_lock);
	} else
		lsm6ds3_i2c_write(client,send, len);
//		err = i2c_transfer(client->adapter, &msg, 1);

	return err;
}

static const struct st_lsm6ds3_transfer_function st_lsm6ds3_tf_i2c = {
	.write = st_lsm6ds3_i2c_write,
	.read = st_lsm6ds3_i2c_read,
};

static int st_lsm6ds3_i2c_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int err;
	struct device_node *eint_node;
	struct lsm6ds3_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	i2c_set_clientdata(client, cdata);

	cdata->tf = &st_lsm6ds3_tf_i2c;

#if 0
	if (!gpDMABuf_pa) {

		gpDMABuf_va = dma_alloc_coherent(cdata->dev,
			LSM_DMA_MAX_TRANSACTION_LEN, &gpDMABuf_pa, GFP_KERNEL);
		if(!gpDMABuf_va){
			printk(KERN_EMERG "Allocate DMA I2C Buffer failed!\n");
			goto free_data;
		}
		printk(KERN_EMERG "Allocate DMA I2C Buffer success!\n");
	}
#endif
	/* by zhangjiajing, add irq number */
	//client->irq = mt_gpio_to_irq(GPIO63);
#if 1
	eint_node = of_find_compatible_node(NULL, NULL, "mediatek,lsm6ds3");
	if (eint_node) {
		client->irq = irq_of_parse_and_map(eint_node, 0);
		if (client->irq == 0)
			pr_err("Parse map for lsm6ds3 detect eint fail\n");
		else
			pr_err("lsm6ds3 eint get irq # %d\n", client->irq);
	} else {
		pr_err("can't find 'mediatek,lsm6ds3' compatible node\n");
	}
#endif
	dev_dbg(cdata->dev, "lsm6ds3 common probe, irq: %d\n", client->irq);
	err = st_lsm6ds3_common_probe(cdata, client->irq);
	if (err < 0) {
		dev_dbg(cdata->dev, "lsm6ds3 common probe error!\n");
		goto free_data;
	}
	dev_dbg(cdata->dev, "lsm6ds3 common probe ok, lsm6ds3 ready to go\n");
	return 0;

free_data:
	kfree(cdata);
	return err;
}

static int st_lsm6ds3_i2c_remove(struct i2c_client *client)
{
	struct lsm6ds3_data *cdata = i2c_get_clientdata(client);

	st_lsm6ds3_common_remove(cdata, client->irq);
	kfree(cdata);

	return 0;
}

#ifdef CONFIG_PM
static int st_lsm6ds3_suspend(struct device *dev)
{
	struct lsm6ds3_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return st_lsm6ds3_common_suspend(cdata);
	return 0;
}

static int st_lsm6ds3_resume(struct device *dev)
{
	struct lsm6ds3_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return st_lsm6ds3_common_resume(cdata);
	return 0;
}

static const struct dev_pm_ops st_lsm6ds3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lsm6ds3_suspend, st_lsm6ds3_resume)
};

#define ST_LSM6DS3_PM_OPS		(&st_lsm6ds3_pm_ops)
#else /* CONFIG_PM */
#define ST_LSM6DS3_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id st_lsm6ds3_id_table[] = {
//	{ LSM6DS3_DEV_NAME, 0 },
	{ "lsm6ds3", 0 },
	{ },
};
//MODULE_DEVICE_TABLE(i2c, st_lsm6ds3_id_table);

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3_of_match[] = {
	{.compatible = "mediatek,lsm6ds3"},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6ds3_of_match);
#endif

static struct i2c_driver st_lsm6ds3_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-lsm6ds3-i2c",
		.pm = ST_LSM6DS3_PM_OPS,
#ifdef CONFIG_OF
			.of_match_table = of_match_ptr(lsm6ds3_of_match),
#endif
	},
	.probe = st_lsm6ds3_i2c_probe,
	.remove = st_lsm6ds3_i2c_remove,
	.id_table = st_lsm6ds3_id_table,
};

struct st_sensors_platform_data pdata ={ .drdy_int_pin = 1, };

#if 0
static struct i2c_board_info  single_chip_board_info[] = {
	{
		I2C_BOARD_INFO("lsm6ds3", 0x6a),
		//.irq = (CUST_EINT_GYRO_NUM),
		//.platform_data = &pdata,
	},
};
static int __init lsm6ds3_board_init(void)
{
	printk("%s meizu\n",__func__);
	i2c_register_board_info(1, single_chip_board_info, 1);
	return 0;
}

postcore_initcall(lsm6ds3_board_init);
#endif

module_i2c_driver(st_lsm6ds3_driver);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 i2c driver");
MODULE_LICENSE("GPL v2");
