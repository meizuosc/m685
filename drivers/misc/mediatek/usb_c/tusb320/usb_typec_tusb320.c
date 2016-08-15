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

#include "tusb320.h"

#define K_EMERG	(1<<7)
#define K_QMU	(1<<7)
#define K_ALET		(1<<6)
#define K_CRIT		(1<<5)
#define K_ERR		(1<<4)
#define K_WARNIN	(1<<3)
#define K_NOTICE	(1<<2)
#define K_INFO		(1<<1)
#define K_DEBUG	(1<<0)

#define tusb_printk(level, fmt, args...) do { \
		if (debug_level & level) { \
			pr_err("[TUSB320]" fmt, ## args); \
		} \
	} while (0)

#define SKIP_TIMER

static u32 debug_level = 255;
static struct i2c_client *tusb_typec_client;
static void tusb320_dump_reg(void);
#if defined(CONFIG_MTK_UART_USB_SWITCH)
extern void musb_uart_usb_switch(unsigned int portmode);
#endif


/* /////////////////////////////////////////////////////////////////////////// */
/* Variables accessible outside of the TUSB320 state machine */
/* /////////////////////////////////////////////////////////////////////////// */
static TUSB320reg_t tusb_registers;	/* Variable holding the current status of the TUSB320 registers */
static ConnectionState usbc_conn_state;	/* Variable indicating the current connection state */
static ConnectionState lastusbc_conn_state;


void tusb320_hw_init(void)
{
	tusb_printk(K_DEBUG, "%s\n", __func__);

	/*Set DRP mode*/
 //      tusb_registers.Modes.byte = MOD_DRP;
//	tusb320_write(regModes, 1, &tusb_registers.Modes.byte);

	/*reset TUSB320*/



	usbc_conn_state = Unattached;
}



/* /////////////////////////////////////////////////////////////////////////// */
/* TUSB320 I2C Routines */
/* /////////////////////////////////////////////////////////////////////////// */
/* int tusb320_write(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int tusb320_write(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;
	int ret=0;

	for (i = 0; i < length; i++)
		ret = tusb320_i2c_w_reg8(tusb_typec_client, regAddr + i, data[i]);

	return ret;
}

/* int tusb320_read(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int tusb320_read(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;
	int ret=0;

	for (i = 0; i < length; i++)
		ret  = tusb320_i2c_r_reg(tusb_typec_client, regAddr + i, &data[i]);

	return ret;
}

static void tusb320_check_type(void)
{
	tusb320_read(regInterrupt, 1, &tusb_registers.Interrupt.byte);	/* attach type*/

	lastusbc_conn_state = usbc_conn_state; //save last connect state
	switch (tusb_registers.Interrupt.ATTAHED_STATE)
	{
	case 0:
		usbc_conn_state = Unattached;
		break;
	case 1:
		usbc_conn_state = AttachedSource;
		break;
	case 2:
		usbc_conn_state = AttachedSink;
		break;
	case 3:
		tusb320_read(regType, 1, &tusb_registers.Type.byte);	/* Read accessory type*/
		switch(tusb_registers.Type.ACCESSORY_CONNECTED)
		{
		case 0:
			usbc_conn_state = Unattached;
			break;
		case 4:
			usbc_conn_state = AudioAccessory;
			break;
		case 5:
			usbc_conn_state = AudioChargedThruAccessory;
			break;
		case 6:
		case 7:
			usbc_conn_state = DebugAccessory;
			break;
		case 1:
		case 2:
		case 3:
		default:
			usbc_conn_state = UnsupportedAccessory;
			break;
		}
		break;
	}

	tusb_printk(K_INFO,"%s: type is %d\n", __func__, usbc_conn_state);

}

void tusb320_eint_work(struct work_struct *data)
{
	UINT8 status = 0;
	struct tusb320_typc *typec = container_of(to_delayed_work(data), struct tusb320_typc, fsm_work);

	mutex_lock(&typec->fsm_lock);
	tusb320_read(regInterrupt, 1, &tusb_registers.Interrupt.byte);	/* Read the interrupt */
	tusb_printk(K_INFO,"%s: interrupt %d!\n", __func__,tusb_registers.Interrupt.byte);
       tusb_printk(K_INFO,"%s: Orient is %d\n", __func__, tusb_registers.Interrupt.CABLE_DIR);

	tusb320_check_type();

	switch(usbc_conn_state){
	case Unattached:
		if(lastusbc_conn_state == AttachedSource)
			trigger_driver(DEVICE_TYPE, DISABLE, DONT_CARE);

		if(lastusbc_conn_state == AttachedSink)
			trigger_driver(HOST_TYPE, DISABLE, DONT_CARE);

#if defined(CONFIG_MTK_UART_USB_SWITCH)
		if(lastusbc_conn_state == DebugAccessory)
			musb_uart_usb_switch(0);//switch to uart, 0=PORT_MODE_USB
#endif

		tusb_printk(K_INFO,"%s:device Detach interrupt!\n", __func__);
		break;
	case DebugAccessory:
#if defined(CONFIG_MTK_UART_USB_SWITCH)
	   	 musb_uart_usb_switch(1);//switch to uart, 1=PORT_MODE_UART
#endif
	case AudioAccessory:
	case AudioChargedThruAccessory:
	case UnsupportedAccessory:
		tusb_printk(K_INFO,"%s: Accessory attch interrupt!\n", __func__);
		break;
	case AttachedSource:
		 trigger_driver(DEVICE_TYPE, ENABLE, DONT_CARE);
		break;
	case AttachedSink:
		trigger_driver(HOST_TYPE, ENABLE, DONT_CARE);
		break;
	default:
		break;
	}

	tusb320_dump_reg();
       if (!typec->en_irq) {
		tusb_printk(K_DEBUG, "Enable IRQ\n");
		typec->en_irq = 1;
		enable_irq(typec->irqnum);
	}
	status = tusb_registers.Interrupt.byte | INT_MSK;
	tusb320_write(regInterrupt, 1, &status);	/* clear the interrupt */
	mutex_unlock(&typec->fsm_lock);
}

static irqreturn_t tusb320_eint_isr(int irqnum, void *data)
{
	int ret;
	struct tusb320_typc *typec = data;

	if (typec->en_irq) {
		tusb_printk(K_DEBUG, "Disable IRQ\n");
		disable_irq_nosync(irqnum);
		typec->en_irq = 0;
	}

	ret = schedule_delayed_work_on(WORK_CPU_UNBOUND, &typec->fsm_work, 0);

	return IRQ_HANDLED;
}

int tusb320_eint_init(struct tusb320_typc *typec)
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

         	tusb_printk(K_INFO, "gpiopin=%x, debounce=x%x\n",gpiopin , debounce);
		gpio_set_debounce(gpiopin, debounce);
	}
	typec->irqnum = irq_of_parse_and_map(node, 0);
	typec->en_irq = 1;

	tusb_printk(K_INFO, "request_irq irqnum=0x%x\n", typec->irqnum);

	retval =
	    request_irq(typec->irqnum, tusb320_eint_isr, IRQF_TRIGGER_NONE, "tusb320_int", typec);
	if (retval != 0) {
		tusb_printk(K_ERR, "request_irq fail, ret %d, irqnum %d!!!\n", retval,
			    typec->irqnum);
	}
	return retval;
}

int tusb320_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var)
{
	char buffer[2];

	buffer[0] = addr;
	buffer[1] = var;
	return i2c_master_send(client, buffer, 2);
}

int tusb320_i2c_r_reg(struct i2c_client *client, u8 addr, u8* var)
{
	int ret=0;

	ret = i2c_master_send(client, &addr, 1);
	ret = i2c_master_recv(client, var, 1);
	return ret;
}

const char *tusb320_reg_strings[] = {
	"regDeviceID0   ",	/* 0x00 */
	"regDeviceID1   ",	/* 0x01 */
	"regDeviceID2   ",	/* 0x02 */
	"regDeviceID3   ",	/* 0x03 */
	"regDeviceID4   ",	/* 0x04 */
	"regDeviceID5   ",	/* 0x05 */
	"regDeviceID6   ",	/* 0x06 */
	"regDeviceID7   ",	/* 0x07 */
	"regType      ",	/* 0x08 */
	"regInterrupt  ",	/* 0x09 */
	"regMode     ",	/* 0x0a */
};


static int tusb320_debugfs_i2c_show(struct seq_file *s, void *unused)
{
	struct i2c_client *client = s->private;
	int i = 1;
	int str_idx = 0;
	u8 val = 0;
	int ret = 0;

	for (; i < 0x0b; i++) {
		ret = tusb320_i2c_r_reg(client, i, &val);
		tusb_printk(K_INFO, "%s %x\n", __func__, val);
		seq_printf(s, "[%02x]%-10s: %02x\n", i, tusb320_reg_strings[str_idx], val);
		str_idx++;
	}

	return 0;
}

static int tusb320_debugfs_i2c_open(struct inode *inode, struct file *file)
{
	return single_open(file, tusb320_debugfs_i2c_show, inode->i_private);
}

static ssize_t tusb320_debugfs_i2c_write(struct file *file,
					 const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct i2c_client *client = s->private;

	char buf[18];
	int input_addr = 0;
	int input_val = 0;
	u8  addr = 0;
	u8 val = 0;
	int ret = 0;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	tusb_printk(K_INFO, "%s %s\n", __func__, buf);

	if (sscanf(buf, "a0x%x v0x%x", &input_addr, &input_val) == 2) {
		val = input_val & 0xFF;
		addr = input_addr & 0xFF;
		if (addr == 0x01 || addr == 0x11 || addr == 0x12 || (addr > 0x06 && addr < 0x0F) || addr > 0x13) {
			tusb_printk(K_ERR, "%s invalid address=0x%x\n", __func__, addr);
			return count;
		}
		tusb_printk(K_INFO, "%s write address=0x%x, value=0x%x\n", __func__, addr, (int)val);
		tusb320_i2c_w_reg8(client, addr, val);
		ret = tusb320_i2c_r_reg(client, addr, &val);
		tusb_printk(K_INFO, "%s result=0x%x\n", __func__,(int) val);
	}

	return count;
}

static const struct file_operations tusb320_debugfs_i2c_fops = {
	.open = tusb320_debugfs_i2c_open,
	.write = tusb320_debugfs_i2c_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int tusb320_init_debugfs(struct tusb320_typc *typec)
{
	struct dentry *root;
	struct dentry *file;
	int ret;

	root = debugfs_create_dir("tusb320", NULL);
	if (!root) {
		ret = -ENOMEM;
		goto err0;
	}

	file = debugfs_create_file("i2c_rw", S_IRUGO|S_IWUSR, root, typec->i2c_hd,
				   &tusb320_debugfs_i2c_fops);
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

static void tusb320_dump_reg(void)
{
	struct i2c_client *client = tusb_typec_client;
	int i = 1;
	int str_idx = 0;
	u8 val = 0;
	int ret = 0;

	for (i=0; i < 0x0B; i++) {
		ret = tusb320_i2c_r_reg(client, i, &val);
		tusb_printk(K_INFO, "%s [%02x]%-10s:%02x\n", __func__, i, tusb320_reg_strings[str_idx], val);
		str_idx++;
	}

}

static int tusb320_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tusb320_typc *typec;

	tusb_printk(K_INFO, "%s 0x%x\n", __func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		tusb_printk(K_ERR, "tusb320 i2c functionality check fail.\n");
		return -ENODEV;
	}

	tusb_typec_client = client;

	tusb320_read(regDeviceID0, 1, &tusb_registers.DeviceID.ID0);
	
	if(tusb_registers.DeviceID.ID0 != 0x30)/* Read the device ID */
	{
		tusb_printk(K_ERR, "%s can't found tusb320 \n", __func__);
		return -ENODEV;
	};	

	tusb_printk(K_DEBUG, "%s %s\n", __func__, client->dev.driver->name);

	typec = kzalloc(sizeof(struct tusb320_typc), GFP_KERNEL);
	typec->i2c_hd = client;

	tusb320_hw_init();
	mutex_init(&typec->fsm_lock);
	INIT_DELAYED_WORK(&typec->fsm_work, tusb320_eint_work);

	tusb320_init_debugfs(typec);

	tusb320_eint_init(typec);
	schedule_delayed_work_on(WORK_CPU_UNBOUND, &typec->fsm_work, 0);
	tusb_printk(K_INFO, "%s sucess! \n", __func__);
	return 0;
}

#define TUSB320_NAME "TUSB320"

static const struct i2c_device_id usb_i2c_id[] = {
		{TUSB320_NAME, 0},
		{}
	};

#ifdef CONFIG_OF
static const struct of_device_id tusb320_of_match[] = {
		{.compatible = "mediatek,tusb320_type_c"},
		{},
	};
#endif

struct i2c_driver tusb320_i2c_driver = {
	.probe = tusb320_i2c_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = TUSB320_NAME,
#ifdef CONFIG_OF
		.of_match_table = tusb320_of_match,
#endif
	},
	.id_table = usb_i2c_id,
};

static int __init tusb320_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&tusb320_i2c_driver) != 0) {
		tusb_printk(K_ERR, "tusb320_init initialization failed!!\n");
		ret = -1;
	} else {
		tusb_printk(K_DEBUG, "tusb320_init initialization succeed!!\n");
	}
	return ret;
}

static void __exit tusb320_exit(void)
{

}
fs_initcall(tusb320_init);
/*module_exit(tusb320_exit);*/
