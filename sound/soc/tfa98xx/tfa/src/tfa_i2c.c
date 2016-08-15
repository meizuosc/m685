#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>

#include "tfa_i2c.h"

/* I2C variable */
static struct i2c_client *new_client = NULL;

#ifdef CONFIG_MTK_I2C_EXTENSION
static u8 *TfaI2CDMABuf_va = NULL;
static u32 TfaI2CDMABuf_pa = NULL;
#else
static char WriteBuffer[RW_BUFFER_LENGTH];
static char ReadBuffer[RW_BUFFER_LENGTH];
#endif

/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_nxpspk_ioctl
 *
 * DESCRIPTION
 *  IOCTL Msg handle
 *
 *****************************************************************************
 */
static long AudDrv_nxpspk_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    //pr_debug("AudDrv_nxpspk_ioctl cmd = 0x%x arg = %lu\n", cmd, arg);

    switch (cmd)
    {
		case I2C_SLAVE:
		case I2C_SLAVE_FORCE:
			//if (cmd == I2C_SLAVE && i2cdev_check_addr(client->adapter, arg))
			//	return -EBUSY;
			/* REVISIT: address could become busy later */
			//new_client->addr = arg;
			if (arg != new_client->addr) {
				pr_err("%s: addr is 0x%02x, not equal current client addr(0x%02x)\n",
					__func__, (unsigned)arg, new_client->addr);
				//return -1;
			}
			return 0;
        default:
        {
            /* printk("AudDrv_nxpspk_ioctl Fail command: %x\n", cmd); */
            ret = 0;//-1;
            break;
        }
    }
    return ret;
}

static int AudDrv_nxpspk_open(struct inode *inode, struct file *fp)
{
    return 0;
}

#ifdef CONFIG_MTK_I2C_EXTENSION
static int nxp_i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.timing = I2C_MASTER_CLOCK;

	if(count <= 8)
	{
		msg.addr = client->addr & I2C_MASK_FLAG;
	}
	else
	{
		msg.addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	}

	msg.flags = client->flags & I2C_M_TEN;
//	msg.timing = client->timing;

	msg.len = count;
	msg.buf = (char *)buf;
	msg.ext_flag = client->ext_flag;
	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	return (ret == 1) ? count : ret;
}

static int nxp_i2c_master_recv(const struct i2c_client *client, char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.timing = I2C_MASTER_CLOCK;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.ext_flag = client->ext_flag;
	msg.buf = (char *)buf;

	if(count <= 8)
	{
		msg.addr = client->addr & I2C_MASK_FLAG;
	}
	else
	{
		msg.addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	}

	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg received), return #bytes received,
	 * else error code.
	 */
	return (ret == 1) ? count : ret;
}
#endif
static ssize_t AudDrv_nxpspk_write(struct file *fp, const char __user *data, size_t count, loff_t *offset)

{
#ifdef CONFIG_MTK_I2C_EXTENSION
	int i = 0;
	char *tmp;
#endif
	int ret;

	//if (count > 8192)
	//	count = 8192;

#ifdef CONFIG_MTK_I2C_EXTENSION
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,data,count)) {
		kfree(tmp);
		return -EFAULT;
	}

	//NXP_INFO("i2c-dev: i2c-%d writing %zu bytes.\n", iminor(file->f_path.dentry->d_inode), count);

	for(i = 0;  i < count; i++)
	{
		TfaI2CDMABuf_va[i] = tmp[i];
	}

	if(count <= 8)
	{
	    ///new_client->addr = new_client->addr & I2C_MASK_FLAG;  //cruson
		ret = nxp_i2c_master_send(new_client,tmp,count);
	}
	else
	{
	    //new_client->addr = new_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG |I2C_ENEXT_FLAG;  //cruson
		ret = nxp_i2c_master_send(new_client, TfaI2CDMABuf_pa, count);
	}
	kfree(tmp);
#else
	if (copy_from_user(WriteBuffer,data,count)) {
		return -EFAULT;
	}

	//NXP_INFO("i2c-dev: i2c-%d writing %zu bytes.\n", new_client->addr, count);

	if (count <= 8)
		ret = i2c_master_send(new_client, WriteBuffer, count);
	else
		ret = i2c_master_send(new_client, (unsigned char *)(uintptr_t) WriteBuffer, count);
#endif
	return ret;
}

static ssize_t AudDrv_nxpspk_read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
#ifdef CONFIG_MTK_I2C_EXTENSION
	int i = 0;
	char *tmp;
#endif
	int ret;

	if (count > 8192)
		count = 8192;
	
#ifdef CONFIG_MTK_I2C_EXTENSION
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;

	//NXP_INFO("i2c-dev: i2c-%d reading %zu bytes.\n", iminor(file->f_path.dentry->d_inode), count);

	if(count <= 8)
	{
	    //new_client->addr = new_client->addr & I2C_MASK_FLAG;  //cruson
		ret = nxp_i2c_master_recv(new_client,tmp,count);
	}
	else
	{
	    //new_client->addr = new_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG |I2C_ENEXT_FLAG;  //cruson
		ret = nxp_i2c_master_recv(new_client,TfaI2CDMABuf_pa,count);
		for(i = 0; i < count; i++)
		{
			tmp[i] = TfaI2CDMABuf_va[i];
		}
	}

	if (ret >= 0)
		ret = copy_to_user(data,tmp,count)?-EFAULT:ret;
	kfree(tmp);
#else
	if (count <= 8)
		ret = i2c_master_recv(new_client, ReadBuffer, count);
	else
		ret = i2c_master_recv(new_client, (unsigned char *)(uintptr_t) ReadBuffer, count);

	if (ret >= 0)
		ret = copy_to_user(data, ReadBuffer, count) ? -EFAULT : ret;
#endif
	return ret;
}

/**************************************************************************
 * STRUCT
 *  File Operations and misc device
 *
 **************************************************************************/
static struct file_operations AudDrv_nxpspk_fops =
{
    .owner   = THIS_MODULE,
    .open    = AudDrv_nxpspk_open,
    .unlocked_ioctl   = AudDrv_nxpspk_ioctl,
    .write   = AudDrv_nxpspk_write,
    .read    = AudDrv_nxpspk_read,
};

static struct miscdevice AudDrv_nxpspk_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "smartpa_i2c",
    .fops = &AudDrv_nxpspk_fops,
};

int tfa98xx_i2c_register_device(struct tfa98xx *tfa98xx)
{
	int ret = -1;
	//struct i2c_client *client;

	ret = misc_register(&AudDrv_nxpspk_device);
	if (ret < 0) {
		return -1;
	}

	new_client = tfa98xx->i2c;
	return 0;
}

int tfa98xx_i2c_deregister_device(struct tfa98xx *tfa98xx)
{
	misc_deregister(&AudDrv_nxpspk_device);
	new_client = NULL;
	return 0;
}

