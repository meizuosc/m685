#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <typec.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>

#include "fusb301a.h"

#define K_EMERG	(1<<7)
#define K_QMU	(1<<7)
#define K_ALET		(1<<6)
#define K_CRIT		(1<<5)
#define K_ERR		(1<<4)
#define K_WARNIN	(1<<3)
#define K_NOTICE	(1<<2)
#define K_INFO		(1<<1)
#define K_DEBUG	(1<<0)

#define fusb_printk(level, fmt, args...) do { \
		if (debug_level & level) { \
			pr_err("[FUSB301A]" fmt, ## args); \
		} \
	} while (0)

#define SKIP_TIMER

static u32 debug_level = 255;
static struct i2c_client *typec_client;
static void fusb301a_dump_reg(void);
#if defined(CONFIG_MTK_UART_USB_SWITCH)
extern void musb_uart_usb_switch(unsigned int portmode);
#endif

/* /////////////////////////////////////////////////////////////////////////// */
/* Variables accessible outside of the fusb301a state machine */
/* /////////////////////////////////////////////////////////////////////////// */
static fusb301areg_t Registers;	/* Variable holding the current status of the fusb301a registers */
static ConnectionState ConnState;	/* Variable indicating the current connection state */



void fusb301a_initialize(void)
{
	fusb_printk(K_DEBUG, "%s\n", __func__);

	/*Set DRP with ACC mode*/
	Registers.Modes.byte = MOD_DRP_ACC;
	fusb301a_write(regModes, 1, &Registers.Modes.byte);
	fusb301a_read(regControl, 1, &Registers.Control.byte);
	/*unmask INT*/
	Registers.Control.INT_MASK = 0;
	fusb301a_write(regControl, 1, &Registers.Control.byte);
	ConnState = Unattached;
}


/* /////////////////////////////////////////////////////////////////////////// */
/* fusb301a I2C Routines */
/* /////////////////////////////////////////////////////////////////////////// */
/* int fusb301a_write(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int fusb301a_write(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;
	int ret=0;

	for (i = 0; i < length; i++)
		ret = fusb301a_i2c_w_reg8(typec_client, regAddr + i, data[i]);

	return ret;
}

/* int fusb301a_read(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int fusb301a_read(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;
	int ret=0;

	for (i = 0; i < length; i++)
		ret  = fusb301a_i2c_r_reg(typec_client, regAddr + i, &data[i]);

	return ret;
}

static void fusb301a_check_type(void)
{
      fusb301a_read(regType, 1, &Registers.Type.byte);	/* Read the interrupt */
      if(Registers.Type.SOURCE)
      {
	    ConnState = AttachedSource;
	    trigger_driver(DEVICE_TYPE, ENABLE, DONT_CARE);

       }
	else if(Registers.Type.SINK)
	{
	    ConnState = AttachedSink;
	    trigger_driver(HOST_TYPE, ENABLE, DONT_CARE);
	}
	else if(Registers.Type.AUDIOACC)
	{
	    ConnState = AudioAccessory;
	}
	else if(Registers.Type.DEBUGACC)
	{
	    ConnState = DebugAccessory;
#if defined(CONFIG_MTK_UART_USB_SWITCH)
	    musb_uart_usb_switch(1);//switch to uart, 1=PORT_MODE_UART
#endif
	}
	else
	{
	    printk("%s: No device type!\n", __func__);
	}
	fusb_printk(K_INFO,"%s: type is %d\n", __func__, ConnState);

}

static void fusb301_check_orient(void)
{
	u8 orient;
       fusb301a_read(regStatus, 1, &Registers.Status.byte);	/* Read the interrupt */
	orient = Registers.Status.ORIENT;

	fusb_printk(K_INFO,"%s: Orient is %d\n", __func__, orient);
}

static DEFINE_MUTEX(typec_lock);

void fusb301a_eint_work(struct work_struct *data)
{
	struct fusb301a_typc *typec = container_of(to_delayed_work(data), struct fusb301a_typc, fsm_work);

	mutex_lock(&typec->fsm_lock);
	fusb301a_read(regInterrupt, 1, &Registers.Interrupt.byte);	/* Read the interrupt */
	fusb_printk(K_INFO,"%s: interrupt %d!\n", __func__,Registers.Interrupt.byte);
	if(Registers.Interrupt.I_ATTACH)
	{
	    fusb301a_check_type();
	    fusb301_check_orient();
	}
	else if(Registers.Interrupt.I_DETACH)
	{
	    if(ConnState == AttachedSource)
	        trigger_driver(DEVICE_TYPE, DISABLE, DONT_CARE);
	    else if(ConnState == AttachedSink)
	        trigger_driver(HOST_TYPE, DISABLE, DONT_CARE);
#if defined(CONFIG_MTK_UART_USB_SWITCH)
	    else if(ConnState == DebugAccessory)
	        musb_uart_usb_switch(0);//switch to uart, 0=PORT_MODE_USB
#endif
	    else
	        fusb_printk(K_INFO,"%s:other device Detach interrupt!\n", __func__);
	    ConnState = Unattached;
	}
	else if(Registers.Interrupt.I_BC_LVL)
	{
	    fusb_printk(K_INFO,"%s: BC_LVL interrupt!\n", __func__);
	}
	else if(Registers.Interrupt.I_ACC_CH)
	{
	    fusb_printk(K_INFO,"%s: Accessory change interrupt!\n", __func__);
	}
	else
	{
	//sometime fusb301a can't assert Interrupt.I_ATTACH so we just read the status register
	    fusb301a_read(regStatus, 1, &Registers.Status.byte);
	    fusb_printk(K_INFO,"%s: regStatus %d!\n", __func__,Registers.Status.byte);	    
	    if(Registers.Status.ATTACH)
	    {
	        fusb301a_check_type();
	    }
	    else
	    {
   	        if(ConnState == AttachedSource)
   	            trigger_driver(DEVICE_TYPE, DISABLE, DONT_CARE);
   	        else if(ConnState == AttachedSink)
   	            trigger_driver(HOST_TYPE, DISABLE, DONT_CARE);
#if defined(CONFIG_MTK_UART_USB_SWITCH)
   	        else if(ConnState == DebugAccessory)
   	            musb_uart_usb_switch(0);//switch to uart, 0=PORT_MODE_USB
#endif
   	        else
   	            fusb_printk(K_INFO,"%s:other device Detach interrupt!\n", __func__);
   	        ConnState = Unattached;
	    }
	    fusb_printk(K_INFO,"%s: weird interrupt!\n", __func__);
	}
	fusb301a_dump_reg();
       if (!typec->en_irq) {
		fusb_printk(K_DEBUG, "Enable IRQ\n");
		typec->en_irq = 1;
		enable_irq(typec->irqnum);
	}
	mutex_unlock(&typec->fsm_lock);
}

static irqreturn_t fusb301a_eint_isr(int irqnum, void *data)
{
	int ret;
	struct fusb301a_typc *typec = data;

	if (typec->en_irq) {
		fusb_printk(K_DEBUG, "Disable IRQ\n");
		disable_irq_nosync(irqnum);
		typec->en_irq = 0;
	}

	ret = schedule_delayed_work_on(WORK_CPU_UNBOUND, &typec->fsm_work, 0);

	return IRQ_HANDLED;
}

int fusb301a_eint_init(struct fusb301a_typc *typec)
{
	int retval = 0;
	u32 ints[2] = { 0, 0 };
	struct device_node *node;
	unsigned int debounce, gpiopin;

	node = of_find_compatible_node(NULL, NULL, "mediatek,usbtypec-int");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		debounce = ints[1];
		gpiopin = ints[0];
/*
              if(debounce == 0)
		debounce = 32000;
		//32000 us
*/
         	fusb_printk(K_INFO, "gpiopin=%x, debounce=x%x\n",gpiopin , debounce);
		gpio_set_debounce(gpiopin, debounce);
	}
	typec->irqnum = irq_of_parse_and_map(node, 0);
	typec->en_irq = 1;

	fusb_printk(K_INFO, "request_irq irqnum=0x%x\n", typec->irqnum);

	retval =
	    request_irq(typec->irqnum, fusb301a_eint_isr, IRQF_TRIGGER_NONE, "fusb301a_eint", typec);
	if (retval != 0) {
		fusb_printk(K_ERR, "request_irq fail, ret %d, irqnum %d!!!\n", retval,
			    typec->irqnum);
	}
	return retval;
}

int fusb301a_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var)
{
	char buffer[2];

	buffer[0] = addr;
	buffer[1] = var;
	return i2c_master_send(client, buffer, 2);
}

int fusb301a_i2c_r_reg(struct i2c_client *client, u8 addr, u8* var)
{
	int ret=0;

	ret = i2c_master_send(client, &addr, 1);
	ret = i2c_master_recv(client, var, 1);
	return ret;
}

const char *strings[] = {
	"regDeviceID   ",	/* 0x01 */
	"regModes  ",	/* 0x02 */
	"regControl  ",	/* 0x03 */
	"regManual    ",	/* 0x04 */
	"regReset      ",	/* 0x05 */
	"regMask       ",	/* 0x0A */
	"regStatus      ",	/* 0x0B */
	"regType      ",	/* 0x0C */
//	"regInterrupt     ",	/* 0x0D */
};


static int fusb301a_debugfs_i2c_show(struct seq_file *s, void *unused)
{
	struct i2c_client *client = s->private;
	int i = 1;
	int str_idx = 0;
	u8 val = 0;

	for (; i < 0x13; i++) {
		if ((i > 0x5 && i < 0x10))
			continue;
		fusb301a_i2c_r_reg(client, i, &val);
		fusb_printk(K_INFO, "%s %x\n", __func__, val);
		seq_printf(s, "[%02x]%-10s: %02x\n", i, strings[str_idx], val);
		str_idx++;
	}

	return 0;
}

static int fusb301a_debugfs_i2c_open(struct inode *inode, struct file *file)
{
	return single_open(file, fusb301a_debugfs_i2c_show, inode->i_private);
}

static ssize_t fusb301a_debugfs_i2c_write(struct file *file,
					 const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct i2c_client *client = s->private;

	char buf[18];
	int input_addr = 0;
	int input_val = 0;
	u8 addr = 0;
	u8 val = 0;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	fusb_printk(K_INFO, "%s %s\n", __func__, buf);

	if (sscanf(buf, "a0x%x v0x%x", &input_addr, &input_val) == 2) {
		val = input_val & 0xFF;
		addr = input_addr & 0xFF;
		if (addr == 0x01 || addr == 0x11 || addr == 0x12 || (addr > 0x06 && addr < 0x0F) || addr > 0x13) {
			fusb_printk(K_ERR, "%s invalid address=0x%x\n", __func__, addr);
			return count;
		}
		fusb_printk(K_INFO, "%s write address=0x%x, value=0x%x\n", __func__, addr, val);
		fusb301a_i2c_w_reg8(client, addr, val);
		fusb301a_i2c_r_reg(client, addr, &val);
		fusb_printk(K_INFO, "%s result=0x%x\n", __func__, val);
	}

	return count;
}

static const struct file_operations fusb301a_debugfs_i2c_fops = {
	.open = fusb301a_debugfs_i2c_open,
	.write = fusb301a_debugfs_i2c_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int fusb301a_init_debugfs(struct fusb301a_typc *typec)
{
	struct dentry *root;
	struct dentry *file;
	int ret;

	root = debugfs_create_dir("fusb301a", NULL);
	if (!root) {
		ret = -ENOMEM;
		goto err0;
	}

	file = debugfs_create_file("i2c_rw", S_IRUGO|S_IWUSR, root, typec->i2c_hd,
				   &fusb301a_debugfs_i2c_fops);

	if (!file) {
		ret = -ENOMEM;
		goto err1;
	}

	return 0;

err1:
	debugfs_remove_recursive(root);

err0:
	return ret;
}

static void fusb301a_dump_reg(void)
{
	struct i2c_client *client = typec_client;
	int i = 1;
	int str_idx = 0;
	u8 val = 0;

	for (; i < 0x13; i++) {
		if ((i > 0x5 && i < 0x10))
			continue;
		fusb301a_i2c_r_reg(client, i, &val);
		fusb_printk(K_INFO, "%s [%02x]%-10s:%02x\n", __func__, i, strings[str_idx], val);
		str_idx++;
	}

}

static int fusb301a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fusb301a_typc *typec;

	fusb_printk(K_INFO, "%s 0x%x\n", __func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		fusb_printk(K_ERR, "fusb301a i2c functionality check fail.\n");
		return -ENODEV;
	}

	typec_client = client;

	fusb301a_read(regDeviceID, 1, &Registers.DeviceID.byte);	/* Read the device ID */
	if(Registers.DeviceID.byte==0)
	{
		fusb_printk(K_ERR, "%s can't found fusb301a \n", __func__);
		return -ENODEV;
	}
	fusb_printk(K_DEBUG, "%s %s\n", __func__, client->dev.driver->name);

	typec = kzalloc(sizeof(struct fusb301a_typc), GFP_KERNEL);
	typec->i2c_hd = client;

       fusb301a_initialize();
       //register must init earlier before fusb301a_eint_init or there will be some bug in first boot
       //ic may be not ready to show crrect INT status
	mutex_init(&typec->fsm_lock);
	INIT_DELAYED_WORK(&typec->fsm_work, fusb301a_eint_work);

	fusb301a_init_debugfs(typec);
	/*
	   0x03 0000_0011 1:DFP 2:DRP 3:UFP
	   0x04 0000_0100 1:Acc Support, 0:Acc No Support
	   0x08 0000_1000 1:DFPPrefered
	   0x30 0011_0000 1: utccDefault 2:utcc1p5A 3:utcc3p0A other:utccNone
	   0x80 1000_0000 1:Enable SM
	 */

	fusb301a_eint_init(typec);
//	fusb301a_dump_reg();
	schedule_delayed_work_on(WORK_CPU_UNBOUND, &typec->fsm_work, 0);
	fusb_printk(K_INFO, "%s sucess! \n", __func__);
	return 0;
}

#define FUSB301A_NAME "FUSB301A"

static const struct i2c_device_id usb_i2c_id[] = {
		{FUSB301A_NAME, 0},
		{}
	};

#ifdef CONFIG_OF
static const struct of_device_id fusb302_of_match[] = {
		{.compatible = "mediatek,fusb301a_type_c"},
		{},
	};
#endif

struct i2c_driver usb_i2c_driver = {
	.probe = fusb301a_i2c_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = FUSB301A_NAME,
#ifdef CONFIG_OF
		.of_match_table = fusb302_of_match,
#endif
	},
	.id_table = usb_i2c_id,
};

static int __init fusb301a_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&usb_i2c_driver) != 0) {
		fusb_printk(K_ERR, "fusb301a_init initialization failed!!\n");
		ret = -1;
	} else {
		fusb_printk(K_DEBUG, "fusb301a_init initialization succeed!!\n");
	}
	return ret;
}

static void __exit fusb301a_exit(void)
{

}
late_initcall(fusb301a_init);
module_exit(fusb301a_exit);
