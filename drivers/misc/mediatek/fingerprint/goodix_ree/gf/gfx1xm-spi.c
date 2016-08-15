#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/input.h>   
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_spi.h>
//#include <mach/eint.h>
//#include <cust_eint.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
//#include <mach/mt_spi.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_clkmgr.h>
//#include <mach/mt_pm_ldo.h>
#include <gfx1xm-spi.h>
#include <mt_spi.h>
#include <linux/platform_device.h>

#include <mt-plat/mt_pwm.h>
#include <mt-plat/upmu_common.h>

//#include <linux/earlysuspend.h>

#define GFX18M_PID "GFx18M"
#define GFX16M_PID "GFx16M"

#define GFX1XM_PID_LEN 6
#define GFX1XM_INPUT_MENU_KEY   KEY_MENU
#define GFX1XM_INPUT_BACK_KEY   KEY_BACK
#define GFX1XM_INPUT_HOME_KEY   KEY_HOMEPAGE
#define GFX1XM_FF_KEY           KEY_POWER

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
 
/*spi device name*/
#define SPI_DEV_NAME        "gf518m"
/*device name after register in charater*/
#define DEV_NAME            "goodix_fp"
#define	CHRD_DRIVER_NAME		"goodix"
#define	CLASS_NAME			    "goodix-spi"
#define SPIDEV_MAJOR			   156	/* assigned */
#define N_SPI_MINORS			   32	/* ... up to 256 */
#define OFFSET               7
static DECLARE_BITMAP(minors, N_SPI_MINORS);

static struct task_struct *gfx1xm_irq_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);


#ifdef CONFIG_OF
static struct of_device_id goodix_of_match[] = {
	{ .compatible = "mediatek,gf518m", },
	{}
};
MODULE_DEVICE_TABLE(of, goodix_of_match);
#endif


static int irq_flag = 0;
int suspend_flag    = 0;
/**************************debug******************************/
#define DEFAULT_DEBUG   (0)
#define SUSPEND_DEBUG   (1)
#define SPI_DEBUG       (2)
#define TIME_DEBUG      (3)
#define FLOW_DEBUG      (4)
int  g_debug_level = FLOW_DEBUG;//DEFAULT_DEBUG;
int gfx1xm_debug_level(int level)
{
	g_debug_level=level;
	printk(KERN_EMERG LOG_TAG_GOODIX "=======sonia g_debug_level=%d",g_debug_level);
	return 0;
}
#if GFX1XM_DEBUG
#define gfx1xm_debug(level, fmt, args...) do{ \
    if(g_debug_level >= level) {\
	printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm " fmt, ##args); \
    } \
}while(0)
  
#define FUNC_ENTRY()  gfx1xm_debug(FLOW_DEBUG, "gfx1xm:%s, entry\n", __func__)
#define FUNC_EXIT()  gfx1xm_debug(FLOW_DEBUG,"gfx1xm:%s, exit\n", __func__)
#endif

//[goodix add by sonia]
#if 1
static struct gfx1xm_dev gfx1xm;

#if defined(SPI_TRANSFER_256)
    #define     MTK_SPI_ALIGN_MASK_NUM  8
#elif defined(SPI_TRANSFER_512)
    #define     MTK_SPI_ALIGN_MASK_NUM  9
#elif defined(SPI_TRANSFER_1024)
    #define     MTK_SPI_ALIGN_MASK_NUM  10
#endif

#define     MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_MASK_NUM) - 1)
typedef enum {
	SPEED_500KHZ=0,
	SPEED_1MHZ,
	SPEED_2MHZ,
	SPEED_3MHZ,
	SPEED_4MHZ,
	SPEED_6MHZ,
	SPEED_8MHZ,
	SPEED_KEEP,
	SPEED_UNSUPPORTED
}SPI_SPEED;

static struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 15,
	.holdtime = 15,
	.high_time = 21, //for mt6582, 104000khz/(4+4) = 130000khz
	.low_time = 21,
	.cs_idletime = 20,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = FIFO_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

struct mt_chip_conf spi_conf={
	.setuptime=60,
       .holdtime=60,
       .high_time=50,
       .low_time=50,
       .cs_idletime=8,
       .ulthgh_thrsh=0,
       .cpol=SPI_CPOL_0,
       .cpha=SPI_CPHA_0,
       .rx_mlsb=SPI_MSB,
        .tx_mlsb=SPI_MSB,
        .tx_endian=SPI_LENDIAN,
        .rx_endian=SPI_LENDIAN,
        .com_mod=DMA_TRANSFER,
        .pause=0,
        .deassert=0,
        .ulthigh=0,
        .tckdly=0,
};

static void gfx1xm_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
	struct mt_chip_conf *mcc = &spi_conf;
	if(flag == 0) {
		mcc->com_mod = FIFO_TRANSFER;
	} else {
		mcc->com_mod = DMA_TRANSFER;
	}
	switch(speed)
	{
		case SPEED_500KHZ:
			mcc->high_time = 120;
			mcc->low_time = 120;
			break;
		case SPEED_1MHZ:
			mcc->high_time = 60;
			mcc->low_time = 60;
			break;
		case SPEED_2MHZ:
			mcc->high_time = 30;
			mcc->low_time = 30;
			break;
		case SPEED_3MHZ:
			mcc->high_time = 20;
			mcc->low_time = 20;
			break;
		case SPEED_4MHZ:
			mcc->high_time = 15;
			mcc->low_time = 15;
			break;

		case SPEED_6MHZ:
			mcc->high_time = 10;
			mcc->low_time = 10;
			break;
		case SPEED_8MHZ:
		    mcc->high_time = 8;
			mcc->low_time = 8;
			break;  
		case SPEED_KEEP:
		case SPEED_UNSUPPORTED:
			break;
	}
	if(spi_setup(spi) < 0){
		pr_warn("gfx1xm:Failed to set spi.\n");
	}
}
int chip_version_config=0;
int gfx1xm_power_on(bool onoff)
{
	if(onoff)
	{
	 //	hwPowerOn(MT6325_POWER_LDO_VCAMA , VOL_2800, "FP28");
	}
	else
	{
	//	hwPowerDown(MT6325_POWER_LDO_VCAMA, "FP28");
	}
	return 0;
}

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
    .modalias=SPI_DEV_NAME,
		.bus_num = 1,
		.chip_select=0,//change by sonia for 1 had use by other device
		.mode = SPI_MODE_0,
		.controller_data=&spi_conf,
	},
};

#endif 
//[goodix end]
#if CFG_UPDATE
const u8 vendor_id_1[3]={0xA5};
const u8 vendor_id_2[3]={0xA6};
struct config_buf {
    unsigned int date;
    unsigned char buffer[249];
};

struct gfx1xm_config {
    unsigned char type; //hardware type
    unsigned char pid; //productor ID
    unsigned char config_num; //how many configs this productor has.
    struct config_buf *config;
};
static struct config_buf config316_buf_list[] = {
    {
	.date = 0x7df051c,
	.buffer = {
			0x41,0x3c,0x3c,0xe4,0x0c,0x30,0x3f,0x02,0x00,0x50,0x40,0x50,0x50,0xe4,0x0c,0x30,
	    0x2f,0x03,0x00,0x03,0x11,0xa0,0x0d,0x00,0x14,0x03,0x0f,0x0f,0x0f,0xb2,0x3f,0xb3,
	    0x33,0x03,0x90,0x01,0x40,0x05,0x0e,0x80,0x20,0x0f,0x22,0x00,0x08,0x10,0x12,0x11,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0xe8,0x25,0x04,0xca,0xa4,0x26,0x66,0x00,
	    0x00,0x00,0x01,0x00,0x01,0x0f,0x96,0x00,0x01,0x02,0x85,0x00,0x03,0x20,0x20,0x50,
	    0x3e,0x11,0x01,0x00,0x00,0x00,0x00,0x03,0x09,0x00,0x31,0x00,0x07,0x14,0x41,0x00,
	    0x50,0x00,0x00,0x00,0x20,0x00,0x04,0x00,0x32,0x01,0xa0,0x00,0x00,0x79,0xc8,0x00,
	    0x00,0x00,0x28,0x00,0x05,0x04,0x30,0x00,0x08,0x00,0x07,0x00,0x20,0x00,0x18,0x00,
	    0x3b,0x00,0x5d,0x00,0x22,0x00,0x00,0x00,0x03,0x07,0x80,0x00,0x20,0x00,0x20,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x01
	},
    },

};

static struct config_buf config318_buf_list[2] = {
    {
	.date = 0x7df051c,
	.buffer = {
	    0x00,0x3C,0x3C,0xE4,0x0C,0x30,0x3F,0x02,0x00,0x50,0x40,0x50,0x50,0xE4,0x0C,0x30,0x2F,0x03,0x00,0x03,
      0x11,0xA0,0x0D,0x00,0x14,0x03,0x0F,0x0F,0x0F,0xB2,0x3F,0xB3,0x33,0x03,0x90,0x01,0x40,0x05,0x0E,0x80,
      0x20,0x0F,0x22,0x00,0x08,0x07,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,
      0x01,0x25,0x04,0xCA,0xA4,0x26,0x66,0x00,0x00,0x00,0x01,0x00,0x01,0x0F,0x96,0x00,0x01,0x02,0x85,0x00,
      0x03,0x20,0x20,0x50,0x3E,0x11,0x01,0x00,0x00,0x00,0x00,0x03,0x09,0x00,0x31,0x00,0x07,0x14,0x41,0x00,
      0x50,0x00,0x00,0x00,0x65,0x00,0x04,0x00,0x32,0x01,0xA0,0x00,0x00,0x79,0xC8,0x00,0x00,0x00,0x28,0x00,
      0x05,0x04,0x45,0x00,0x08,0x00,0x07,0x00,0x20,0x00,0x18,0x00,0x3D,0x00,0x48,0x00,0x22,0x00,0x00,0x00,
      0x03,0x07,0x80,0x00,0x20,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE8,0x01
	},
    },
	{
	.date = 0x7df051c,
	.buffer = {
	0x00, 0x3C, 0x3C, 0xE4, 0x0C, 0x30, 0x3F, 0x02, 0x00, 0x50, 0x40, 0x50, 0x50, 0xE4, 0x0C, 0x30, 
	0x2F, 0x03, 0x00, 0x03, 0x11, 0xA0, 0x0D, 0x00, 0x14, 0x03, 0x0F, 0x0F, 0x0F, 0xB2, 0x3F, 0xB3, 
	0x33, 0x03, 0x90, 0x01, 0x40, 0x05, 0x0E, 0x80, 0x20, 0x0F, 0x22, 0x00, 0x08, 0x07, 0x08, 0x06, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x25, 0x04, 0xCA, 0xA4, 0x26, 0x66, 0x00, 
	0x00, 0x00, 0x01, 0x00, 0x01, 0x0F, 0x96, 0x00, 0x01, 0x02, 0x85, 0x00, 0x03, 0x20, 0x20, 0x50, 
	0x3E, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x09, 0x00, 0x31, 0x00, 0x07, 0x14, 0x41, 0x00, 
	0x50, 0x00, 0x00, 0x00, 0x50, 0x00, 0x04, 0x00, 0x32, 0x01, 0xA0, 0x00, 0x00, 0x79, 0xC8, 0x00, 
	0x00, 0x00, 0x28, 0x00, 0x05, 0x04, 0x30, 0x00, 0x08, 0x00, 0x07, 0x00, 0x20, 0x00, 0x18, 0x00, 
	0x3D, 0x00, 0x48, 0x00, 0x22, 0x00, 0x00, 0x00, 0x03, 0x07, 0x80, 0x00, 0x20, 0x00, 0x50, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x01
	},
    },
};
static struct gfx1xm_config config318_list[] = {
    {
	.type = 0,
	.pid = 1,
	.config_num = 1,
	.config = &config318_buf_list[0],
    }
};
static struct gfx1xm_config config316_list[] = {
    {
	.type = 0,
	.pid = 1,
	.config_num = 1,
	.config = &config316_buf_list[0],
    }
};
#endif
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
/*************************data stream***********************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsiz = 8 * (2048+5);
#if FW_UPDATE	
static unsigned char GFX1XM318_FW[]=
{
#include "gf318_fw.i"
};
static unsigned char GFX1XM316_FW[]=
{
#include "gf316_fw.i"
};

#define FW_LENGTH (42*1024)
#endif
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

void print_16hex(u8 *config, u8 len)
{
    u8 i,j = 0;  
    gfx1xm_debug(DEFAULT_DEBUG,"dump hex \n");
    for(i = 0 ; i< len ; i++) {
	gfx1xm_debug(DEFAULT_DEBUG,"0x%x " , config[i]);
	if(j++ == 15) {
	    j = 0;
	}
    } 
}
/* -------------------------------------------------------------------- */
/* devfs                                */
/* -------------------------------------------------------------------- */
static ssize_t gfx1xm_debug_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	return (sprintf(buf, "%d\n", g_debug_level));
}
static ssize_t gfx1xm_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int g_debug = 0;
	sscanf(buf, "%d", &g_debug);
	gfx1xm_debug_level(g_debug);
	//return strnlen(buf, count); 
	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gfx1xm_debug_show, gfx1xm_debug_store);

static struct attribute *gfx1xm_debug_attrs[] = {
    &dev_attr_debug.attr,
    NULL
};

static const struct attribute_group gfx1xm_debug_attr_group = {
    .attrs = gfx1xm_debug_attrs,
    .name = "debug"
};



void gfx1xm_spi_setup(struct gfx1xm_dev *gfx1xm_dev, int max_speed_hz,int num)
{

    gfx1xm_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gfx1xm_dev->spi->max_speed_hz = max_speed_hz; 
    gfx1xm_dev->spi->irq = num;//change by sonia
    //gfx1xm_dev->spi->irq = mt_gpio_to_irq(GFX1XM_IRQ_PIN_NUM);
    gfx1xm_dev->spi->bits_per_word = 8;
    spi_setup(gfx1xm_dev->spi);
}

/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/

int gfx1xm_spi_write_bytes(struct gfx1xm_dev *gfx1xm_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + GFX1XM_WDATA_OFFSET)>>MTK_SPI_ALIGN_MASK_NUM;
	u32  reminder = (data_len + GFX1XM_WDATA_OFFSET) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;

	/*set spi mode.*/
	if((data_len + GFX1XM_WDATA_OFFSET) > 32) {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		/*copy the reminder data to temporarity buffer.*/
		reminder_buf = kzalloc(reminder + GFX1XM_WDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			pr_warn("gfx1xm:No memory for exter data.\n");
			return -ENOMEM;
		}
		memcpy(reminder_buf + GFX1XM_WDATA_OFFSET, tx_buf + GFX1XM_WDATA_OFFSET+data_len - reminder, reminder);
        gfx1xm_debug(SPI_DEBUG,"gfx1xm:w-reminder:0x%x-0x%x,0x%x\n", reminder_buf[GFX1XM_WDATA_OFFSET],reminder_buf[GFX1XM_WDATA_OFFSET+1],
                reminder_buf[GFX1XM_WDATA_OFFSET + 2]);
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	}
	if( xfer == NULL){
		pr_warn("gfx1xm:No memory for command.\n");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}

	//gfx1xm_debug(SPI_DEBUG,"gfx1xm:write twice = %d. data_len = %d, package_num = %d, reminder = %d\n", (int)twice, (int)data_len, (int)package_num, (int)reminder);
	/*if the length is not align with 1024. Need 2 transfer at least.*/
	spi_message_init(&msg);
	tx_buf[0] = GFX1XM_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
	//xfer[0].delay_usecs = 5;
	if(twice == 1) {
		xfer[0].len = package_num << 10;
		spi_message_add_tail(&xfer[0], &msg);
		addr += data_len - reminder;
		reminder_buf[0] = GFX1XM_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[1].tx_buf = reminder_buf;
		xfer[1].len = reminder + GFX1XM_WDATA_OFFSET;
		//xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
	} else {
		xfer[0].len = data_len + GFX1XM_WDATA_OFFSET;
		spi_message_add_tail(&xfer[0], &msg);
	}
	ret = spi_sync(gfx1xm_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1)
			ret = msg.actual_length - 2*GFX1XM_WDATA_OFFSET;
		else
			ret = msg.actual_length - GFX1XM_WDATA_OFFSET;
	} else 	{
		gfx1xm_debug(SPI_DEBUG,"gfx1xm:write async failed. ret = %d\n", ret);
	}

	if(xfer != NULL) {
		kfree(xfer);
		xfer = NULL;
	}
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}
	
	return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gfx1xm_spi_read_bytes(struct gfx1xm_dev *gfx1xm_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + 1)>>MTK_SPI_ALIGN_MASK_NUM;
	u32  reminder = (data_len + 1) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;
	
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		reminder_buf = kzalloc(reminder + GFX1XM_RDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			pr_warn("No memory for exter data.\n");
			return -ENOMEM;
		}
		xfer = kzalloc(sizeof(*xfer)*4, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	}
	if( xfer == NULL){
		pr_warn("No memory for command.\n");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}
	/*set spi mode.*/
	if((data_len + GFX1XM_RDATA_OFFSET) > 32) {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	spi_message_init(&msg);
    /*send GFX1XM command to device.*/
	rx_buf[0] = GFX1XM_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gfx1xm_dev->spi, &msg);
	spi_message_init(&msg);

	/*if wanted to read data from GFX1XM. 
	 *Should write Read command to device
	 *before read any data from device.
	 */
	//memset(rx_buf, 0xff, data_len);
	rx_buf[4] = GFX1XM_R;
	xfer[1].tx_buf = &rx_buf[4];
	xfer[1].rx_buf = &rx_buf[4];
	if(twice == 1)
		xfer[1].len = (package_num << 10);
	else
		xfer[1].len = data_len + 1;
	spi_message_add_tail(&xfer[1], &msg);
	if(twice == 1) {
		addr += data_len - reminder;
		reminder_buf[0] = GFX1XM_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[2].tx_buf = reminder_buf;
		xfer[2].len = 3;
		spi_message_add_tail(&xfer[2], &msg);
		spi_sync(gfx1xm_dev->spi, &msg);
		spi_message_init(&msg);
		reminder_buf[4] = GFX1XM_R;
		xfer[3].tx_buf = &reminder_buf[4];
		xfer[3].rx_buf = &reminder_buf[4];
		xfer[3].len = reminder + 1;
		spi_message_add_tail(&xfer[3], &msg);
	}
	ret = spi_sync(gfx1xm_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1) {
            gfx1xm_debug(SPI_DEBUG,"gfx1xm:reminder:0x%x:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", reminder_buf[0], reminder_buf[1], 
                    reminder_buf[2], reminder_buf[3],reminder_buf[4],reminder_buf[5],reminder_buf[6],reminder_buf[7]);
			memcpy(rx_buf + GFX1XM_RDATA_OFFSET + data_len - reminder, reminder_buf + GFX1XM_RDATA_OFFSET, reminder);
			ret = data_len;//msg.actual_length - 1; //8 
		} else {
			ret = data_len;//msg.actual_length - 1; //4
		}
	}else {
        pr_warn("gfx1xm: read failed. ret = %d\n", ret);
    }

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}	
	//gfx1xm_debug(SPI_DEBUG,"gfx1xm:read twice = %d, data_len = %d, package_num = %d, reminder = %d\n",(int)twice, (int)data_len, (int)package_num, (int)reminder);
	//gfx1xm_debug(SPI_DEBUG,"gfx1xm:data_len = %d, msg.actual_length = %d, ret = %d\n", (int)data_len, (int)msg.actual_length, ret);
	return ret;
}

static int gfx1xm_spi_read_byte(struct gfx1xm_dev *gfx1xm_dev, u16 addr, u8 *value)
{
    int status = 0;
    mutex_lock(&gfx1xm_dev->buf_lock);

    status = gfx1xm_spi_read_bytes(gfx1xm_dev, addr, 1, gfx1xm_dev->buffer);
    *value = gfx1xm_dev->buffer[GFX1XM_RDATA_OFFSET];
    gfx1xm_debug(SPI_DEBUG, "value = 0x%x, buffer[3] = 0x%x\n", *value, gfx1xm_dev->buffer[GFX1XM_RDATA_OFFSET]);
    mutex_unlock(&gfx1xm_dev->buf_lock);
    return status;
}
static int gfx1xm_spi_write_byte(struct gfx1xm_dev *gfx1xm_dev, u16 addr, u8 value)
{
    int status = 0;
    mutex_lock(&gfx1xm_dev->buf_lock);
    gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET] = value;
    status = gfx1xm_spi_write_bytes(gfx1xm_dev, addr, 1, gfx1xm_dev->buffer);
    mutex_unlock(&gfx1xm_dev->buf_lock);
    return status;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gfx1xm_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    ssize_t			status = 0;
    //long int t1, t2;
    FUNC_ENTRY();
    printk(KERN_EMERG LOG_TAG_GOODIX "=======sonia gfx1xm_read\n");
    if ((count > bufsiz)||(count == 0)) {
	pr_warn("Max size for write buffer is %d. wanted length is %d\n", (int)bufsiz, (int)count);
	return -EMSGSIZE;
    }

    gfx1xm_dev = filp->private_data;
    mutex_lock(&gfx1xm_dev->buf_lock);
    gfx1xm_dev->spi->max_speed_hz=1*1000*1000;
    //t1 = ktime_to_us(ktime_get());
    spi_setup(gfx1xm_dev->spi);
    status = gfx1xm_spi_read_bytes(gfx1xm_dev, GFX1XM_BUFFER_DATA, count, gfx1xm_dev->buffer);
    if(status > 0) {
	unsigned long missing = 0;
	missing = copy_to_user(buf, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, status);
	if(missing == status)
	    status = -EFAULT;
    } else {
	pr_err("Failed to read data from SPI device.\n");
	status = -EFAULT;
    }
    // t2 = ktime_to_us(ktime_get());
    //printk(KERN_EMERG LOG_TAG_GOODIX "read time use: %ld\n", t2-t1);
    mutex_unlock(&gfx1xm_dev->buf_lock);
	FUNC_EXIT();
    return status;
}

/* Write-only message with current device setup */
static ssize_t gfx1xm_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    ssize_t			status = 0;
	
    FUNC_ENTRY();
    if(count > bufsiz) {
	pr_warn("Max size for write buffer is %d\n", bufsiz);
	return -EMSGSIZE;
    } 
    mutex_lock(&gfx1xm_dev->buf_lock);
    status = copy_from_user(gfx1xm_dev->buffer + GFX1XM_WDATA_OFFSET, buf, count);
    if(status == 0) {
	gfx1xm_dev->spi->max_speed_hz=2*1000*1000;
	spi_setup(gfx1xm_dev->spi);
	status = gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_BUFFER_DATA, count, gfx1xm_dev->buffer);
    } else {
	pr_err("Failed to xfer data through SPI bus.\n");
	status = -EFAULT;
    }
    mutex_unlock(&gfx1xm_dev->buf_lock);
    FUNC_EXIT();
    return status;
}

static long gfx1xm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gfx1xm_dev *gfx1xm_dev = NULL;
    struct gfx1xm_ioc_transfer *ioc = NULL;
    int			err = 0;
    u32			tmp = 0;
    int 		retval = 0;
	u8 *temp_buf;
    if (_IOC_TYPE(cmd) != GFX1XM_IOC_MAGIC)
	return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE,
		(void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
	err = !access_ok(VERIFY_READ,
		(void __user *)arg, _IOC_SIZE(cmd));
    if (err)
	return -EFAULT;

    gfx1xm_dev = (struct gfx1xm_dev *)filp->private_data;

    switch(cmd) {
	case GFX1XM_IOC_CMD:
	    ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
	    /*copy command data from user to kernel.*/
	    if(copy_from_user(ioc, (struct gfx1xm_ioc_transfer*)arg, sizeof(*ioc))){
		pr_warn("Failed to copy command from user to kernel.\n");
		retval = -EFAULT;
		break;
	    }

	    if((ioc->len > bufsiz)||(ioc->len == 0)) {
		pr_warn("The request length[%d] is longer than supported maximum buffer length[%d].\n", 
			ioc->len, bufsiz);
		retval = -EMSGSIZE;
		break;
	    }

	    mutex_lock(&gfx1xm_dev->buf_lock);
	    gfx1xm_dev->spi->max_speed_hz=1*1000*1000;
	    spi_setup(gfx1xm_dev->spi);
	    if(ioc->cmd == GFX1XM_R) {
			/*if want to read data from hardware.*/
			#if PROCESSOR_64_BIT
			temp_buf=(void __user*)(unsigned long)ioc->buf;
			gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm_ioctl Read data from 0x%x, len = 0x%x buf = 0x%p\n", (int)ioc->addr, (int)ioc->len, (void __user*)((unsigned long)ioc->buf));
			gfx1xm_spi_read_bytes(gfx1xm_dev, ioc->addr, ioc->len, gfx1xm_dev->buffer);
			if(copy_to_user((void __user*)((unsigned long)ioc->buf), gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, ioc->len)) {
			    pr_err("Failed to copy data from kernel to user.\n");
			    retval = -EFAULT;
			    mutex_unlock(&gfx1xm_dev->buf_lock);
			    break;
			}
	    } else if (ioc->cmd == GFX1XM_W) {
		/*if want to read data from hardware.*/
		gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm_ioctl Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
		if(ioc->addr == 0x8043)
		{
			temp_buf=(void __user*)(unsigned long)ioc->buf;
			gfx1xm_debug(DEFAULT_DEBUG," temp_buf=%x,%x,%x,%x\n",temp_buf[0],temp_buf[1],temp_buf[2],temp_buf[3]);
		    gfx1xm_dev->mode = temp_buf[0];
		    gfx1xm_debug(DEFAULT_DEBUG,"set mode 0x%x \n", gfx1xm_dev->mode);
		}
		if(copy_from_user(gfx1xm_dev->buffer + GFX1XM_WDATA_OFFSET, (void __user*)((unsigned long) ioc->buf), ioc->len)){
		    pr_warn("Failed to copy data from user to kernel.\n");
		    retval = -EFAULT;
		    mutex_unlock(&gfx1xm_dev->buf_lock);
		    break;
		}
        #else
		/*if want to read data from hardware.*/
		gfx1xm_debug(DEFAULT_DEBUG,"Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, ioc->buf);
		gfx1xm_spi_read_bytes(gfx1xm_dev, ioc->addr, ioc->len, gfx1xm_dev->buffer);
		if(copy_to_user(ioc->buf, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, ioc->len)) {
		    pr_err("Failed to copy data from kernel to user.\n");
		    retval = -EFAULT;
		    mutex_unlock(&gfx1xm_dev->buf_lock);
		    break;
		}
	    } else if (ioc->cmd == GFX1XM_W) {
		/*if want to read data from hardware.*/
		gfx1xm_debug(DEFAULT_DEBUG,"Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);

		if(ioc->addr == 0x8043)
		{
		    gfx1xm_dev->mode = ioc->buf[0];
		    gfx1xm_debug(DEFAULT_DEBUG,"set mode 0x%x \n", gfx1xm_dev->mode);
		}
		if(copy_from_user(gfx1xm_dev->buffer + GFX1XM_WDATA_OFFSET, ioc->buf, ioc->len)){
		    pr_err("Failed to copy data from user to kernel.\n");
		    retval = -EFAULT;
		    mutex_unlock(&gfx1xm_dev->buf_lock);
		    break;
		}
		#endif
		gfx1xm_spi_write_bytes(gfx1xm_dev, ioc->addr, ioc->len, gfx1xm_dev->buffer);
	    } else {
		pr_warn("Error command for gfx1xm_ioctl.\n");
	    }
	    if(ioc != NULL) {
		kfree(ioc);
		ioc = NULL;
	    }
	    mutex_unlock(&gfx1xm_dev->buf_lock);
	    break;
	case GFX1XM_IOC_REINIT:
	    disable_irq_nosync(gfx1xm_dev->spi->irq);
	    gfx1xm_hw_reset();
	    enable_irq(gfx1xm_dev->spi->irq);
	    printk(KERN_EMERG LOG_TAG_GOODIX "[%s:%d] enable_irq\n", __func__, __LINE__);

	    gfx1xm_debug(FLOW_DEBUG,"wake-up gfx1xm\n");
	    break;
	case GFX1XM_IOC_SETSPEED:
	    retval = __get_user(tmp, (u32 __user*)arg);
	    if(tmp > 8*1000*1000) {
		pr_warn("The maximum SPI speed is 8MHz.\n");
		retval = -EMSGSIZE;
		break;
	    }
	    if(retval == 0) {
		gfx1xm_dev->spi->max_speed_hz=tmp;
		spi_setup(gfx1xm_dev->spi);
		gfx1xm_debug(DEFAULT_DEBUG, "spi speed changed to %d\n", tmp);
	    }	
	    break;
	default:
	    pr_warn("gfx1xm doesn't support this command(%d)\n", cmd);
	    break;
    }
    FUNC_EXIT();
    return retval;
}

static unsigned int gfx1xm_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS, &gfx1xm_dev->buf_status);
    if((gfx1xm_dev->buf_status & GFX1XM_BUF_STA_MASK) == GFX1XM_BUF_STA_READY) {
	return (POLLIN|POLLRDNORM);
    } else {
	gfx1xm_debug(DEFAULT_DEBUG, "Poll no data.\n");
    }
    return 0;
}


#if FW_UPDATE
static int isUpdate(struct gfx1xm_dev *gfx1xm_dev)
{
    unsigned char version[16];
    unsigned int ver_fw = 0;
    unsigned int ver_file = 0;
	unsigned char* fw318 = GFX1XM318_FW; 
	unsigned char* fw316 = GFX1XM316_FW; 
    unsigned char fw_running = 0;
    //const char OFFSET = 7;

    msleep(300);
    gfx1xm_spi_read_byte(gfx1xm_dev, 0x41e4, &fw_running);
    gfx1xm_debug(DEFAULT_DEBUG,"%s: 0x41e4 = 0x%x\n", __func__, fw_running);
    if(fw_running == 0xbe) {
	/*firmware running*/
	
	if(chip_version_config==0x38){
		ver_file = (int)(fw318[12] & 0xF0) <<12;
		ver_file |= (int)(fw318[12] & 0x0F)<<8;
		ver_file |= fw318[13];	//get the fw version in the i file;
	
	}
	else if(chip_version_config==0x36){
		ver_file = (int)(fw316[12] & 0xF0) <<12;
		ver_file |= (int)(fw316[12] & 0x0F)<<8;
		ver_file |= fw316[13]; //get the fw version in the i file;
		
	}

	/*In case we want to upgrade to a special firmware. Such as debug firmware.*/
	if(ver_file != 0x5a5a) {
	    mutex_lock(&gfx1xm_dev->buf_lock);
	    gfx1xm_spi_read_bytes(gfx1xm_dev,0x8000,16,gfx1xm_dev->buffer);
	    memcpy(version, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, 16);
	    mutex_unlock(&gfx1xm_dev->buf_lock);
        /*
	    if(memcmp(version, GFX1XM_PID, GFX1XM_PID_LEN)) {
		gfx1xm_debug(DEFAULT_DEBUG,"version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1],
			version[2], version[3], version[4], version[5]);
		return 1;
	    }
*/
	    if((version[OFFSET]>9) || ((version[OFFSET + 1])>9)) {
		gfx1xm_debug(DEFAULT_DEBUG,"version: 8-0x%x; 9-0x%x\n", version[OFFSET], version[OFFSET + 1]);
		return 1;
	    }

	    //get the current fw version
	    ver_fw  = (unsigned int)version[OFFSET] << 16;
	    ver_fw |= (unsigned int)version[OFFSET + 1] << 8;
	    ver_fw |= (unsigned int)version[OFFSET + 2];
	    gfx1xm_debug(DEFAULT_DEBUG,"ver_fw: 0x%06x; ver_file:0x%06x\n", ver_fw, ver_file);

	    if(ver_fw == ver_file){
		/*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/
		return 0;
	    }
	}
	gfx1xm_debug(DEFAULT_DEBUG,"Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
    }else {
	/*no firmware.*/
	pr_warn("No running firmware. Value = 0x%x\n", fw_running);
    }
    return 1;
}


static u8 is_9p_ready_ok(struct gfx1xm_dev *gfx1xm_dev)
{
    u8 tmpBuf[16] = {0}; 
    u8 *ptr =NULL;
    u16 time_out = 0; 
    gfx1xm_spi_read_bytes(gfx1xm_dev, 0x4220, 4, tmpBuf);
    ptr = &tmpBuf[GFX1XM_RDATA_OFFSET];
    while(ptr[0] !=0x02 || ptr[1] !=0x08 || ptr[2] !=0x90 || ptr[3] !=0x00)
    {    
	time_out++;
	if (time_out > 200)
	{
	    return 0;
	}
	gfx1xm_spi_read_bytes(gfx1xm_dev, 0x4220, 4, tmpBuf);
	ptr = &tmpBuf[GFX1XM_RDATA_OFFSET];
    }    
   gfx1xm_debug(DEFAULT_DEBUG,"%s , timeout = %d\n",__func__,  time_out);
    return 1;
}



static int gfx1xm_fw_update_init(struct gfx1xm_dev *gfx1xm_dev)
{
    u8 retry_cnt = 5;     
    u8 value;

    //    disable_irq(gfx1xm_dev);    
    //    gfx1xm_hw_reset();          
    //    enable_irq(gfx1xm_dev); 

    while(retry_cnt--)
    {
	//set spi miso input pull up    
	gfx1xm_miso_pullup();

	//reset and delay 5ms
	mt_set_gpio_out(GFX1XM_RST_PIN, 0);
	msleep(5);
	mt_set_gpio_out(GFX1XM_RST_PIN, 1);
	msleep(1);
	//recover miso back spi
	gfx1xm_miso_backnal();
	gfx1xm_spi_setup(gfx1xm_dev, 1000*1000);
	if(!is_9p_ready_ok(gfx1xm_dev)){
	    pr_warn("check 9p ver fail \n");
	    retry_cnt = 0xFF;
	    break;
	}
	mdelay(10);
	gfx1xm_spi_write_byte(gfx1xm_dev, 0x5081, 0x00);
	gfx1xm_spi_write_byte(gfx1xm_dev, 0x4180, 0x0C);
	gfx1xm_spi_read_byte(gfx1xm_dev, 0x4180, &value);
	if (value == 0x0C)             
	{             
	    gfx1xm_debug(DEFAULT_DEBUG,"hold SS51 and DSP successfully!\n");
	    break;
	}
    }

    if(retry_cnt == 0xFF) {   
	pr_warn("Faile to hold SS51 and DSP.\n");
	return 0;
    } else {
	gfx1xm_debug(DEFAULT_DEBUG,"Hold retry_cnt=%d\n",retry_cnt);
	gfx1xm_spi_write_byte(gfx1xm_dev, 0x4010, 0);
	return 1;         
    }  
}
#endif
#if ESD_PROTECT
void gf_esd_switch(struct gfx1xm_dev *gfx1xm_dev, s32 on)
{
	if (1 == on)
	{
		queue_delayed_work(gfx1xm_dev->esd_wq, &gfx1xm_dev->esd_check_work, gfx1xm_dev->clk_tick_cnt);
	}
	else //switch off esd
	{
		cancel_delayed_work_sync(&gfx1xm_dev->esd_check_work);
	}
}
static void gfx1xm_esd_work(struct work_struct *work)
{	
    unsigned char value[4];
    struct gfx1xm_dev *gfx1xm_dev;
    int ret = 0;
    u8 mode = 0xFF;
	unsigned char* p318_fw = GFX1XM318_FW; 
	unsigned char* p316_fw = GFX1XM316_FW; 
    int temp,i=0;
    unsigned char version[16];
    FUNC_ENTRY();
    if(work == NULL)
    {
	pr_warn(" %s wrong work\n",__func__);
	return;
    }
    gfx1xm_dev = container_of(work, struct gfx1xm_dev, spi_work);   

    if(gfx1xm_dev->mode == GFX1XM_FF_MODE)
	goto exit; 

    ret = power_supply_is_system_supplied();
    //printk(KERN_EMERG LOG_TAG_GOODIX "BEN: power_supply ret = %d\n", ret);

    mutex_lock(&gfx1xm_dev->buf_lock);
    gfx1xm_dev->spi->max_speed_hz= 1000*1000;//SPI_SPEED_MIN;
    spi_setup(gfx1xm_dev->spi);
    mutex_unlock(&gfx1xm_dev->buf_lock);
    for(i=0;i<3;i++){
		gfx1xm_spi_read_byte(gfx1xm_dev, 0x8040, &value[0]);
		gfx1xm_spi_read_byte(gfx1xm_dev, 0x8000, &value[1]);
		//read fw_PID
		gfx1xm_spi_read_bytes(gfx1xm_dev,0x8000,16,gfx1xm_dev->buffer);
		memcpy(version, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, 16);
		if(chip_version_config==0x38){
			temp = memcmp(version, GFX18M_PID, GFX1XM_PID_LEN);
			}
		else if(chip_version_config==0x36){
			temp = memcmp(version, GFX16M_PID, GFX1XM_PID_LEN);
			}
		if(value[0] == 0xC6 && value[1] == 0x47&&(chip_version_config==0x38)&&(temp==0)){
			break;
			}
		mdelay(5);
	}
	if (i<3){
	    gfx1xm_spi_write_byte(gfx1xm_dev, 0x8040, 0xAA);
	    mdelay(1);
	}else{
	    gfx1xm_debug(DEFAULT_DEBUG," hardware works abnormal,do reset! 0x8040=0x%x 0x8000=0x%x 0x8046=0x%x\n"
			,value[0],value[1],value[2]);
	    disable_irq_nosync(gfx1xm_dev->spi->irq);
	    gfx1xm_hw_reset();
	    gfx1xm_spi_read_byte(gfx1xm_dev, 0x8000, &value[1]);
	    gfx1xm_debug(DEFAULT_DEBUG,"[info] %s read 0x41e4 finish value = 0x%x, 0x8000=0x%x\n", __func__,value[0], value[1]);
#if FW_UPDATE
		for(i=0;i<2;i++){
			gfx1xm_spi_read_byte(gfx1xm_dev, 0x41e4, &value[0]);
			if( 0xbe==value[0]){
				break;
				}
		}
		if(i>=1) {
		    /***********************************firmware update*********************************/
		    gfx1xm_debug(DEFAULT_DEBUG,"[info] %s firmware update start\n", __func__);
		    //del_timer_sync(&gfx1xm_dev->gfx1xm_timer);
		    if(gfx1xm_fw_update_init(gfx1xm_dev)) {
			if(chip_version_config==0x38){
				gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm_fw_update to gf318m\n");
				gfx1xm_fw_update(gfx1xm_dev, p318_fw, FW_LENGTH);
           
			}
			else if(chip_version_config==0x36){
				gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm_fw_update to gfx1xm316m or gf516m\n");
				gfx1xm_fw_update(gfx1xm_dev, p316_fw, FW_LENGTH);
				
			}
			
			gfx1xm_hw_reset();
		    }
		    //gfx1xm_dev->gfx1xm_timer.expires = jiffies + 2 * HZ;
		    //add_timer(&gfx1xm_dev->gfx1xm_timer);
		}
#endif
#if CFG_UPDATE
	    /***************************************update config********************************/
	    ret = gfx1xm_spi_write_byte(gfx1xm_dev, 0x8040, 0xAA);
	    if(!ret)
		printk(KERN_EMERG LOG_TAG_GOODIX "[info] %s write 0x8040 fail\n", __func__);

		ret = gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_CFG_ADDR, GFX1XM_CFG_LEN, gfx1xm_dev->config);
		if(ret <= 0)
			pr_info("[info] %s write config fail\n", __func__);
#endif
	gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, 0x00);
	    enable_irq(gfx1xm_dev->spi->irq);
	    printk(KERN_EMERG LOG_TAG_GOODIX "[%s:%d] enable_irq\n", __func__, __LINE__);
	
    /*if mode was changed by reset, we should set the mode  back to the primary mode*/
    gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_MODE_STATUS,&mode); 
    if(mode != gfx1xm_dev->mode) {
	gfx1xm_debug(DEFAULT_DEBUG,"[info] %s set mode back\n", __func__);
	gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_MODE_STATUS, gfx1xm_dev->mode);
	gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_MODE_STATUS, &mode);
	gfx1xm_debug(DEFAULT_DEBUG,"[info] %s mode444 = %d\n", __func__, mode);
    }
	}
exit:
	queue_delayed_work(gfx1xm_dev->esd_wq, &gfx1xm_dev->esd_check_work, gfx1xm_dev->clk_tick_cnt);
   // mod_timer(&gfx1xm_dev->gfx1xm_timer, jiffies + 2*HZ);//??whether 2s is ok	
    FUNC_EXIT(); 
}
#endif
#if CONFIG_HAS_EARLYSUSPEND
static void gfx1xm_early_suspend(struct early_suspend *h)
{    
	struct gfx1xm_dev *gfx1xm_dev = container_of(h, struct gfx1xm_dev, early_fp);
	suspend_flag = 1;
	gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm  suspend.\n");
	#if ESD_PROTECT		
	gf_esd_switch(gfx1xm_dev, 0);
	#endif
}
static void gfx1xm_late_resume(struct early_suspend *h)
{
	struct gfx1xm_dev *gfx1xm_dev = container_of(h, struct gfx1xm_dev, early_fp);	
	gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm  resume\n");
	#if ESD_PROTECT		
	gf_esd_switch(gfx1xm_dev, 1);
	#endif
	suspend_flag = 0;
}
#endif
static void gfx1xm_irq(void)
{
    irq_flag = 1;
    printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s \n", __func__);
    wake_up_interruptible(&waiter);
}
static int gfx1xm_event_handler(void *para)
{
    struct gfx1xm_dev *gfx1xm_dev = (struct gfx1xm_dev *)para;
    u8 mode = 0x80;
    u8 status, status_1;// = 0x00;
    int ret;
	printk(KERN_EMERG LOG_TAG_GOODIX "%s, enter\n", __func__);
	do{
		mode = 0x80;
		status = 0x00;
		wait_event_interruptible(waiter, irq_flag != 0);
		disable_irq_nosync(gfx1xm_dev->spi->irq);
		irq_flag = 0;
	gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, &status_1);
	if (status_1 & GFX1XM_BUF_STA_MASK){
		printk(KERN_EMERG LOG_TAG_GOODIX "%s, ESD reset irq, status_1=0x%02X\n", __func__, status_1);
		gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, status_1&0x7F);
		if (1 == suspend_flag){
			gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_MODE_STATUS, 0x03); //write to FF mode
		}
			enable_irq(gfx1xm_dev->spi->irq);
			printk(KERN_EMERG LOG_TAG_GOODIX "[%s:%d] enable_irq\n", __func__, __LINE__);
			//return IRQ_HANDLED;
			continue;		
	}
    gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS, &status);
    gfx1xm_debug(DEFAULT_DEBUG, "IRQ status = 0x%x\n", status);
    if(!(status & GFX1XM_BUF_STA_MASK)) {
	gfx1xm_debug(DEFAULT_DEBUG, "Invalid IRQ = 0x%x\n", status);
			enable_irq(gfx1xm_dev->spi->irq);
			printk(KERN_EMERG LOG_TAG_GOODIX "[%s:%d] enable_irq\n", __func__, __LINE__);
			//return IRQ_HANDLED;
			continue;
    } 
    gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_MODE_STATUS, &mode);
    gfx1xm_debug(SUSPEND_DEBUG, "status = 0x%x, mode = %d\n", status, mode);

    switch(mode)
    {
	case GFX1XM_FF_MODE:
	    if((status & GFX1XM_HOME_KEY_MASK) && (status & GFX1XM_HOME_KEY_STA)){
		gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm: wake device.\n");
		gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_MODE_STATUS, 0x00);
		input_report_key(gfx1xm_dev->input, GFX1XM_FF_KEY, 1);
		input_sync(gfx1xm_dev->input);	    
		input_report_key(gfx1xm_dev->input, GFX1XM_FF_KEY, 0);
		input_sync(gfx1xm_dev->input);
	    } else {
		break;
	    }

	case GFX1XM_IMAGE_MODE:
#ifdef GFX1XM_FASYNC
	    if(gfx1xm_dev->async) {
		gfx1xm_debug(DEFAULT_DEBUG,"async \n");
		kill_fasync(&gfx1xm_dev->async, SIGIO, POLL_IN);
	    }
#endif
	    break;
	case GFX1XM_KEY_MODE:
	    gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm:Key mode: status = 0x%x\n", status);
	    if  ((status & GFX1XM_KEY_MASK) && (status & GFX1XM_BUF_STA_MASK)) {
		if (status & GFX1XM_HOME_KEY_MASK) {
		    input_report_key(gfx1xm_dev->input, GFX1XM_INPUT_HOME_KEY, (status & GFX1XM_HOME_KEY_STA)>>4);
		    input_sync(gfx1xm_dev->input);
		}

		else if (status & GFX1XM_MENU_KEY_MASK){
		    input_report_key(gfx1xm_dev->input, GFX1XM_INPUT_MENU_KEY, (status & GFX1XM_MENU_KEY_STA)>>2);
		    input_sync(gfx1xm_dev->input);

		}else if (status & GFX1XM_BACK_KEY_MASK){
		    input_report_key(gfx1xm_dev->input, GFX1XM_INPUT_BACK_KEY, (status & GFX1XM_BACK_KEY_STA));
		    input_sync(gfx1xm_dev->input);
		}

	    }
	    ret = gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS, (status & 0x7F));
	    printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm_spi_write_byte in GFX1XM_KEY_MODE ret:%d\n", ret);
	    break;
	case GFX1XM_SLEEP_MODE:
	    pr_warn("gfx1xm:Should not happen in sleep mode.\n");
	    break;
	case GFX1XM_DEBUG_MODE:
#ifdef GFX1XM_FASYNC
	    if(gfx1xm_dev->async) {
		kill_fasync(&gfx1xm_dev->async, SIGIO, POLL_IN);
	    }
#endif
	    break;
	default:
	    pr_warn("gfx1xm:Unknown mode. mode = 0x%x\n", mode);
	    break;

    }
		enable_irq(gfx1xm_dev->spi->irq);
		printk(KERN_EMERG LOG_TAG_GOODIX "[%s:%d] enable_irq\n", __func__, __LINE__);
	}while(!kthread_should_stop());
	printk(KERN_EMERG LOG_TAG_GOODIX "%s, leave\n", __func__);
	return 0;
//    return IRQ_HANDLED;
}

static int gfx1xm_open(struct inode *inode, struct file *filp)
{
    struct gfx1xm_dev *gfx1xm_dev;
    int			status = -ENXIO;

    FUNC_ENTRY();
    mutex_lock(&device_list_lock);

    list_for_each_entry(gfx1xm_dev, &device_list, device_entry) {
	if(gfx1xm_dev->devt == inode->i_rdev) {
	    gfx1xm_debug(DEFAULT_DEBUG, "Found\n");
	    status = 0;
	    break;
	}
    }

    if(status == 0){
	mutex_lock(&gfx1xm_dev->buf_lock);
	if( gfx1xm_dev->buffer == NULL) {
	    gfx1xm_dev->buffer = kzalloc(bufsiz + GFX1XM_RDATA_OFFSET, GFP_KERNEL);
	    if(gfx1xm_dev->buffer == NULL) {
		dev_dbg(&gfx1xm_dev->spi->dev, "open/ENOMEM\n");
		status = -ENOMEM;
	    }
	}
	mutex_unlock(&gfx1xm_dev->buf_lock);

	if(status == 0) {
	    gfx1xm_dev->users++;
	    filp->private_data = gfx1xm_dev;
	    nonseekable_open(inode, filp);
	    gfx1xm_debug(DEFAULT_DEBUG, "Succeed to open device. irq = %d\n", gfx1xm_dev->spi->irq);
	    enable_irq(gfx1xm_dev->spi->irq);
	    printk(KERN_EMERG LOG_TAG_GOODIX "[%s:%d] enable_irq\n", __func__, __LINE__);
	}
    } else {
	pr_err("No device for minor %d\n", iminor(inode));
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

#ifdef GFX1XM_FASYNC
static int gfx1xm_fasync(int fd, struct file *filp, int mode)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    int ret;

    FUNC_ENTRY();
    ret = fasync_helper(fd, filp, mode, &gfx1xm_dev->async);
    FUNC_EXIT();
    return ret;
}
#endif

static int gfx1xm_release(struct inode *inode, struct file *filp)
{
    struct gfx1xm_dev *gfx1xm_dev;
    int    status = 0;
    FUNC_ENTRY();
    mutex_lock(&device_list_lock);
    gfx1xm_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    gfx1xm_dev->users --;
    if(!gfx1xm_dev->users) {

	gfx1xm_debug(DEFAULT_DEBUG, "disble_irq. irq = %d\n", gfx1xm_dev->spi->irq);
	disable_irq_nosync(gfx1xm_dev->spi->irq);
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}
static long gfx1xm_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return gfx1xm_ioctl(filp, cmd, (unsigned long)(arg));
}
#if CFG_UPDATE
int gfx1xm_config_update(struct gfx1xm_dev *gfx1xm_dev)
{
	int rery;
	int ret;
	unsigned char tmp;
	unsigned char buffer[16]={0};
	//int counter;
	//init Vendor ID mode		
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET]=0x08;		
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET+1]=0xF8;
	gfx1xm_spi_write_bytes(gfx1xm_dev, 0x8041, 2, gfx1xm_dev->buffer);
	rery = 10;
	while(rery--){
		gfx1xm_spi_read_byte(gfx1xm_dev, 0x8042, &tmp);
		if (0xAA == tmp){
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s, read Vendor ID\n", __func__);
			break;
		}
		printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: read 0x8042=0x%02X retry=%d\n", tmp, rery);
		msleep(5);
	}
	//enter Vendor ID mode
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET]=0x0C;	
	gfx1xm_spi_write_bytes(gfx1xm_dev, 0x5094, 1, gfx1xm_dev->buffer);
	rery = 10;
	while(rery--){
		gfx1xm_spi_read_byte(gfx1xm_dev, 0x5094, &tmp);
		if (0 == tmp){
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s, read Vendor ID is ready!!!\n", __func__);
			break;
		}
		printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: read 0x5094=0x%02X retry=%d\n", tmp, rery);
		msleep(5);
	}
	//read Vendor ID
	gfx1xm_spi_read_bytes(gfx1xm_dev, 0x8148, 10, buffer);
	if(chip_version_config==0x38){		
		if(!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_1, 1))
		{
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, config318_list[0].config[0].buffer, GFX1XM_CFG_LEN);
		}
		else if (!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_2, 1))
		{
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, config318_list[0].config[1].buffer, GFX1XM_CFG_LEN);
		}
		else
		{
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: Vendor ID not matched!!! send default\n");
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, config318_list[0].config[0].buffer, GFX1XM_CFG_LEN);
		}
	}else if(chip_version_config==0x36){
		if(!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_1, 1))
		{
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, config316_list[0].config[0].buffer, GFX1XM_CFG_LEN);
		}
		else if (!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_2, 1))
		{
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, config316_list[0].config[0].buffer, GFX1XM_CFG_LEN);
		}
		else
		{
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: Vendor ID not matched!!! send default\n");
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, config316_list[0].config[0].buffer, GFX1XM_CFG_LEN);
		}
	}
	//leave Vendor ID mode
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET]=0x00;
	gfx1xm_spi_write_bytes(gfx1xm_dev, 0x8041, 1, gfx1xm_dev->buffer);	
/*
	for (counter=0; counter<GFX1XM_CFG_LEN; counter++)
	{
		if (counter%16 == 0)
			printk(KERN_EMERG LOG_TAG_GOODIX "\n");

		printk(KERN_EMERG LOG_TAG_GOODIX "0x%02X,", gfx1xm_dev->config[GFX1XM_WDATA_OFFSET+counter]);

		
	}

	printk(KERN_EMERG LOG_TAG_GOODIX "\n bernard\n");
	*/
	/*write config*/
	ret = gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_CFG_ADDR, GFX1XM_CFG_LEN, gfx1xm_dev->config);
	if(ret <= 0)
		pr_info("[info] %s write config fail\n", __func__);
	return 1;
}
#endif

static const struct file_operations gfx1xm_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	gfx1xm_write,
    .read =		gfx1xm_read,
    .unlocked_ioctl = gfx1xm_ioctl,
	.compat_ioctl	= gfx1xm_compat_ioctl,
    .open =		gfx1xm_open,
    .release =	gfx1xm_release,
    .poll   = gfx1xm_poll,
#ifdef GFX1XM_FASYNC
    .fasync = gfx1xm_fasync,
#endif
};

struct platform_device *goodixpltfm_dev;

struct pinctrl *goodix_pinctrl;
struct pinctrl_state *goodix_pin_irq, *gf518m_reset_high,*gf518m_reset_low;
static int goodix_probe(struct platform_device *pdev)
{
	printk(KERN_EMERG LOG_TAG_GOODIX "goodix_probe\n");	
	goodixpltfm_dev = pdev;
	return 0;
}

static int goodix_remove(struct platform_device *pdev)
{
     printk(KERN_EMERG LOG_TAG_GOODIX "goodix_remove\n");
	return 0;
}

struct platform_device *get_goodix_platformdev(void)
{
	return goodixpltfm_dev;
}


int goodix_get_gpio_info(struct platform_device *pdev)
{

	int ret;

    printk(KERN_EMERG LOG_TAG_GOODIX "gf518m [goodix %s] goodix_pinctrl+++++++++++++++++\n",pdev->name);

	goodix_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(goodix_pinctrl)) {
		ret = PTR_ERR(goodix_pinctrl);
		dev_err(&pdev->dev, "fwq Cannot find goodix pdev->dev!\n");
		return ret;
	}

	goodix_pin_irq = pinctrl_lookup_state(goodix_pinctrl, "goodix_pin_irq");
	if (IS_ERR(goodix_pin_irq)) {
		ret = PTR_ERR(goodix_pin_irq);
		dev_err(&pdev->dev, "fwq Cannot find goodix pinctrl goodix_pin_irq!\n");
		return ret;
	}

	gf518m_reset_high = pinctrl_lookup_state(goodix_pinctrl, "gf518m_reset_high");
	if (IS_ERR(gf518m_reset_high)) {
		ret = PTR_ERR(gf518m_reset_high);
		dev_err(&pdev->dev, "fwq Cannot find goodix pinctrl gf518m_reset_high!\n");
		return ret;
	}

	gf518m_reset_low = pinctrl_lookup_state(goodix_pinctrl, "gf518m_reset_low");
	if (IS_ERR(gf518m_reset_low)) {
		ret = PTR_ERR(gf518m_reset_low);
		dev_err(&pdev->dev, "fwq Cannot find goodix pinctrl gf518m_reset_low!\n");
		return ret;
	}
	
	printk(KERN_EMERG LOG_TAG_GOODIX "gf518m get_gpio_info OK\n");	
	return 0;
}



/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gfx1xm_spi_class;

/*-------------------------------------------------------------------------*/

static int  gfx1xm_probe(struct spi_device *spi)
{
	struct gfx1xm_dev	*gfx1xm_dev = &gfx1xm;
	int			status;
	int ret;
	unsigned long		minor;
//	int err = 0;
	int irqnum;
	int i=0;
	struct platform_device *goodixPltFmDev;	
	struct device_node *node = NULL;

	unsigned char version[16] = {0};
	//unsigned char       version = 0;
	
	FUNC_ENTRY();

    goodixPltFmDev = get_goodix_platformdev();

	//goodix_get_gpio_info(goodixPltFmDev);

		
	pmic_set_register_value(PMIC_RG_VIBR_VOSEL, 5);
	pmic_set_register_value(PMIC_RG_VIBR_EN, 1);

	printk(KERN_EMERG LOG_TAG_GOODIX "driver version is %s.\n",GFX1XM_DRIVER_VERSION);
	/* Allocate driver data */
	gfx1xm_dev = kzalloc(sizeof(*gfx1xm_dev), GFP_KERNEL);
	if (!gfx1xm_dev){
	pr_warn("Failed to alloc memory for gfx1xm device.\n");
	FUNC_EXIT();
	return -ENOMEM;
	}
	/* Initialize the driver data */
	gfx1xm_dev->spi = spi;
	spin_lock_init(&gfx1xm_dev->spi_lock);
	mutex_init(&gfx1xm_dev->buf_lock);
	INIT_LIST_HEAD(&gfx1xm_dev->device_entry);
	//[goodix add by sonia]
	//gfx1xm_power_on(1);  steven
	
	//gfx1xm_spi_pins_config(); steven
	//[goodix end]
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		status = sysfs_create_group(&spi->dev.kobj,&gfx1xm_debug_attr_group);
		if(status){
		    pr_warn("Failed to create sysfs file.\n");
		    goto err;
		}
		gfx1xm_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gfx1xm_spi_class, &spi->dev, gfx1xm_dev->devt,
			  gfx1xm_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		pr_warn( "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gfx1xm_dev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	if (status == 0){
		gfx1xm_dev->buffer = kzalloc(bufsiz + GFX1XM_RDATA_OFFSET, GFP_KERNEL);
		if(gfx1xm_dev->buffer == NULL) {
			kfree(gfx1xm_dev);
			status = -ENOMEM;
			goto err;
		}
		spi_set_drvdata(spi, gfx1xm_dev);
		/*register device within input system.*/
		gfx1xm_dev->input = input_allocate_device();
		if(gfx1xm_dev->input == NULL) {
			pr_warn("Failed to allocate input device.\n");
			status = -ENOMEM;
			kfree(gfx1xm_dev->buffer);
			kfree(gfx1xm_dev);
			goto err;
		}
		__set_bit(EV_KEY, gfx1xm_dev->input->evbit);
		__set_bit(GFX1XM_INPUT_HOME_KEY, gfx1xm_dev->input->keybit);
		__set_bit(GFX1XM_INPUT_MENU_KEY, gfx1xm_dev->input->keybit);
		__set_bit(GFX1XM_INPUT_BACK_KEY, gfx1xm_dev->input->keybit);
		__set_bit(GFX1XM_FF_KEY, gfx1xm_dev->input->keybit);	
		gfx1xm_dev->input->name = "gf-key";
		if(input_register_device(gfx1xm_dev->input)) {
			pr_warn("Failed to register input device.\n");
		}
		/*setup gfx1xm configurations.*/
		gfx1xm_debug(DEFAULT_DEBUG, "Setting gfx1xm device configuration.\n");
		/*SPI parameters.*/
		
		node = of_find_compatible_node(NULL, NULL, "mediatek,gf518m");
		printk(KERN_EMERG LOG_TAG_GOODIX "node.name %s full name %s",node->name,node->full_name);
		
		irqnum = irq_of_parse_and_map(node, 0);
		printk(KERN_EMERG LOG_TAG_GOODIX "irqnum = %d\n",irqnum);
		
		gfx1xm_spi_setup(gfx1xm_dev, 1000*1000, irqnum);
		//gfx1xm_irq_cfg();
		//pinctrl_select_state(goodix_pinctrl, goodix_pin_irq);   //set irq by steven
		gfx1xm_hw_reset();
		gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm interrupt NO. = %d\n", gfx1xm_dev->spi->irq);
		/*spi test*/
		for(i=0;i<10;i++){
			ret = gfx1xm_spi_read_bytes(gfx1xm_dev,0x8000,16,gfx1xm_dev->buffer);
			memcpy(version, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, 16);		
			for(ret = 0; ret <16; ret++)
			gfx1xm_debug(DEFAULT_DEBUG,"chip===>version[%d] = %x \n", ret,version[ret]);
			gfx1xm_spi_read_bytes(gfx1xm_dev,0x41E4,1,gfx1xm_dev->buffer);
			printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: 0x41E4 gfx1xm_dev->buffer[5]=0x%02X\n", gfx1xm_dev->buffer[GFX1XM_RDATA_OFFSET]);
			chip_version_config = version[4];
			gfx1xm_debug(DEFAULT_DEBUG,"goodix chip_version_config=%x \n",chip_version_config);
		}
#if FW_UPDATE
		if(isUpdate(gfx1xm_dev)) {
			unsigned char* fw318 = GFX1XM318_FW; 
			unsigned char* fw316 = GFX1XM316_FW; 
			/*Do upgrade action.*/         
			if(gfx1xm_fw_update_init(gfx1xm_dev)) {
			if(chip_version_config==0x38){
				gfx1xm_fw_update(gfx1xm_dev, fw318, FW_LENGTH);
				}
			else if(chip_version_config==0x36){
				gfx1xm_fw_update(gfx1xm_dev, fw316, FW_LENGTH);
				}
			gfx1xm_hw_reset();
			}
		}
#endif
#if CFG_UPDATE
		/*write config*/      
		//if(!hw_config(gfx1xm_dev))       
		//pr_err("[info] %s write config fail\n", __func__);

		gfx1xm_config_update(gfx1xm_dev);

#endif

		gfx1xm_irq_thread = kthread_run(gfx1xm_event_handler, (void *)gfx1xm_dev, "gfx1xm");
		if (IS_ERR(gfx1xm_irq_thread))
		{
			printk(KERN_EMERG LOG_TAG_GOODIX "Failed to create kernel thread: %ld\n", PTR_ERR(gfx1xm_irq_thread));
		}
		//mt_eint_registration(gfx1xm_dev->spi->irq, EINTF_TRIGGER_RISING, gfx1xm_irq, 0);

	
		printk(KERN_EMERG LOG_TAG_GOODIX "Device Tree gf66xx_irq_registration!\n");
		
		ret =request_irq(gfx1xm_dev->spi->irq, (irq_handler_t) gfx1xm_irq, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"gf518m-eint", NULL);
		if (ret > 0) {
			ret = -1;
			printk(KERN_EMERG LOG_TAG_GOODIX "gf518m request_irq IRQ LINE NOT AVAILABLE!.");
		
		}		
	    disable_irq_nosync(gfx1xm_dev->spi->irq); 
#if ESD_PROTECT
		gfx1xm_dev->clk_tick_cnt = 2*HZ;
		INIT_DELAYED_WORK(&gfx1xm_dev->esd_check_work, gfx1xm_esd_work);
		gfx1xm_dev->esd_wq = create_workqueue("gf_esd_check");
		gf_esd_switch(gfx1xm_dev, 1);
#endif // ESD_PROTECT 

#if CONFIG_HAS_EARLYSUSPEND		
gfx1xm_dev->early_fp.level		= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
gfx1xm_dev->early_fp.suspend	= gfx1xm_early_suspend,		
gfx1xm_dev->early_fp.resume		= gfx1xm_late_resume,    	
register_early_suspend(&gfx1xm_dev->early_fp);
#endif

		gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, 0x00);
		gfx1xm_debug(DEFAULT_DEBUG,"GFX1XM installed.\n");
		}
	else
	kfree(gfx1xm_dev);
	err:
	FUNC_EXIT();
	return status;
	}

static int  gfx1xm_remove(struct spi_device *spi)
{
    struct gfx1xm_dev	*gfx1xm_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if(gfx1xm_dev->spi->irq) {
	free_irq(gfx1xm_dev->spi->irq, gfx1xm_dev);
    }

#if ESD_PROTECT
    //del_timer_sync(&gfx1xm_dev->gfx1xm_timer); 
    //cancel_work_sync(&gfx1xm_dev->spi_work);
    destroy_workqueue(gfx1xm_dev->esd_wq);
#endif

    spin_lock_irq(&gfx1xm_dev->spi_lock);
    gfx1xm_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&gfx1xm_dev->spi_lock);
    /*
       if(gfx1xm_dev->spi_wq != NULL) {
       flush_workqueue(gfx1xm_dev->spi_wq);
       destroy_workqueue(gfx1xm_dev->spi_wq);
       }
     */
    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(&spi->dev.kobj, &gfx1xm_debug_attr_group);
    list_del(&gfx1xm_dev->device_entry);
    device_destroy(gfx1xm_spi_class, gfx1xm_dev->devt);
    clear_bit(MINOR(gfx1xm_dev->devt), minors);
    if (gfx1xm_dev->users == 0) {
	if(gfx1xm_dev->input != NULL)
	    input_unregister_device(gfx1xm_dev->input);

	if(gfx1xm_dev->buffer != NULL)
	    kfree(gfx1xm_dev->buffer);
	kfree(gfx1xm_dev);
    }
    mutex_unlock(&device_list_lock);

    FUNC_EXIT();
    return 0;
}

static int gfx1xm_suspend_test(struct device *dev)
{
   // g_debug_level |= SUSPEND_DEBUG;
   	printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s\n", __func__);
    return 0;
}

static int gfx1xm_resume_test(struct device *dev)
{
    //g_debug &= ~SUSPEND_DEBUG;
    printk(KERN_EMERG LOG_TAG_GOODIX "gfx1xm: %s\n", __func__);
    return 0;
}
static const struct dev_pm_ops gfx1xm_pm = {
    .suspend = gfx1xm_suspend_test,
    .resume = gfx1xm_resume_test
};

static struct spi_driver gfx1xm_spi_driver = {
    .driver = {
	.name =		SPI_DEV_NAME,
	.owner =	THIS_MODULE,
	.pm = &gfx1xm_pm,
    },
    .probe =	gfx1xm_probe,
    .remove =	gfx1xm_remove,
    //.suspend = gfx1xm_suspend_test,
    //.resume = gfx1xm_resume_test,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

static struct platform_driver goodix_driver = {
	.probe	  = goodix_probe,
	.remove	 = goodix_remove,
	.driver = {
		.name  = "goodix",
		.owner = THIS_MODULE,			

		#ifdef CONFIG_OF
		.of_match_table = goodix_of_match,
		#endif
	}
};


/*-------------------------------------------------------------------------*/

static int __init gfx1xm_init(void)
{
    int status;
     printk(KERN_EMERG LOG_TAG_GOODIX "taoxiaodong_test1\n");
    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gfx1xm_fops);
    if (status < 0){
	pr_warn("Failed to register char device!\n");
	FUNC_EXIT();
	return status;
    }
    gfx1xm_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gfx1xm_spi_class)) {
	unregister_chrdev(SPIDEV_MAJOR, gfx1xm_spi_driver.driver.name);
	pr_warn("Failed to create class.\n");
	FUNC_EXIT();
	return PTR_ERR(gfx1xm_spi_class);
    }
	 printk(KERN_EMERG LOG_TAG_GOODIX "taoxiaodong_test2\n");
	//[goodix add by sonia]
	spi_register_board_info(spi_board_devs,ARRAY_SIZE(spi_board_devs));
	//[goodix end]
	if (platform_driver_register(&goodix_driver) != 0) {
		printk(KERN_EMERG LOG_TAG_GOODIX "unable to register goodix driver.\n");
		return -1;
	}	
    status = spi_register_driver(&gfx1xm_spi_driver);
    if (status < 0) {
	class_destroy(gfx1xm_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, gfx1xm_spi_driver.driver.name);
	pr_warn("Failed to register SPI driver.\n");
    }
	printk(KERN_EMERG LOG_TAG_GOODIX "taoxiaodong_test3\n");
    return status;
}
module_init(gfx1xm_init);

static void __exit gfx1xm_exit(void)
{
    spi_unregister_driver(&gfx1xm_spi_driver);
    class_destroy(gfx1xm_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, gfx1xm_spi_driver.driver.name);
}
module_exit(gfx1xm_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gfx1xm-spi");


