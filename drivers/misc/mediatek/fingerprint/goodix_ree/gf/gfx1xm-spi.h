#ifndef __GFX1XM_SPI_H
#define __GFX1XM_SPI_H

#include <linux/types.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_spi.h>
//#include <mach/eint.h>
//#include <cust_eint.h>
//#include <linux/earlysuspend.h>
/****************GFX1XM Macro interface*******************/
#define FW_UPDATE               0
#define ESD_PROTECT             0
#define CFG_UPDATE              0

#define GFX1XM_DEBUG             1
#define CONFIG_HAS_EARLYSUSPEND  0
#define GFX1XM_DRIVER_VERSION   "V1.0<20140811>" //why is 20140811
#define PROCESSOR_64_BIT         1
//#define SPI_TRANSFER_256
#define SPI_TRANSFER_512
//#define SPI_TRANSFER_1024

/********************GFX1XM Mapping**********************/
#define GFX1XM_BASE             (0x8000)
#define GFX1XM_OFFSET(x)        (GFX1XM_BASE + x)

#define GFX1XM_VERSION	        GFX1XM_OFFSET(0)
#define GFX1XM_CONFIG_DATA      GFX1XM_OFFSET(0x40)
#define GFX1XM_CFG_ADDR	        GFX1XM_OFFSET(0x47)
#define GFX1XM_MODE_STATUS      GFX1XM_OFFSET(0x043)
//#define GFX1XM_MIXER_DATA     GFX1XM_OFFSET(0x140)
#define GFX1XM_BUFFER_STATUS	  GFX1XM_OFFSET(0x140)
#define GFX1XM_KEY_DATA         GFX1XM_OFFSET(0x142)
#define GFX1XM_NOISE_DATA       GFX1XM_OFFSET(0x144)
#define GFX1XM_LONG_PRESS_STDP  GFX1XM_OFFSET(0x146)
#define GFX1XM_BUFFER_DATA      GFX1XM_OFFSET(0x141)


#define GFX1XM_BUF_STA_MASK     (0x1<<7)
#define	GFX1XM_BUF_STA_READY	  (0x1<<7)
#define	GFX1XM_BUF_STA_BUSY     (0x0<<7)

#define	GFX1XM_IMAGE_MASK       (0x1<<6)
#define	GFX1XM_IMAGE_ENABLE     (0x1)
#define	GFX1XM_IMAGE_DISABLE	  (0x0)

#define	GFX1XM_KEY_MASK	        (GFX1XM_HOME_KEY_MASK | \
                                 GFX1XM_MENU_KEY_MASK | \
                                 GFX1XM_BACK_KEY_MASK )

//home key
#define	GFX1XM_HOME_KEY_MASK	  (0x1<<5)
#define	GFX1XM_HOME_KEY_ENABL   (0x1)
#define	GFX1XM_HOME_KEY_DISABLE (0x0)

#define	GFX1XM_HOME_KEY_STA     (0x1<<4)
//menu key
#define	GFX1XM_MENU_KEY_MASK    (0x1<<3)
#define	GFX1XM_MENU_KEY_ENABLE	(0x1)
#define	GFX1XM_MENU_KEY_DISABLE	(0x0)

#define	GFX1XM_MENU_KEY_STA	    (0x1<<2)
//back key
#define	GFX1XM_BACK_KEY_MASK    (0x1<<1)
#define	GFX1XM_BACK_KEY_ENABLE  (0x1)
#define	GFX1XM_BACK_KEY_DISABLE (0x0)

#define	GFX1XM_BACK_KEY_STA     (0x1<<0)


#define	GFX1XM_IMAGE_MODE       (0x00)
#define	GFX1XM_KEY_MODE	        (0x01)
#define GFX1XM_SLEEP_MODE       (0x02)
#define GFX1XM_FF_MODE		      (0x03)
#define GFX1XM_DEBUG_MODE       (0x56)

/**********************GFX1XM ops****************************/
#define GFX1XM_W                 0xF0
#define GFX1XM_R                 0xF1
#define GFX1XM_WDATA_OFFSET     (0x3)
#define GFX1XM_RDATA_OFFSET     (0x5)
#define GFX1XM_CFG_LEN          (249)   /*config data length*/

/**********************************************************/

#define GFX1XM_FASYNC 		1//If support fasync mechanism.
//#undef GFX1XM_FASYNC

#define LOG_TAG_GOODIX "[goodix]"
#if 1
#define LOG_TAG_SENSOR "[sensor]"
#define pr_info(format, arg...)         printk(KERN_EMERG LOG_TAG_GOODIX format , ## arg)
#define pr_err(format, arg...)          printk(KERN_EMERG LOG_TAG_GOODIX format , ## arg)
#define pr_warn(format, arg...)          printk(KERN_EMERG LOG_TAG_GOODIX format , ## arg)
#define dev_err(dev, format, arg...)    dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_info(dev, format, arg...)   dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_dbg(dev, format, arg...)    dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_warn(dev, format, arg...)   dev_printk(KERN_EMERG, dev, format , ## arg)
#define dev_notice(dev, format, arg...) dev_printk(KERN_EMERG, dev, format , ## arg)
#endif
/*************************************************************/
struct gfx1xm_dev {
#if CFG_UPDATE	
	u8 config[GFX1XM_CFG_LEN+GFX1XM_WDATA_OFFSET];
#endif
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev        *input;
#if ESD_PROTECT
	int    clk_tick_cnt;
	struct delayed_work esd_check_work;
	struct workqueue_struct *esd_wq;
#endif
	struct work_struct     spi_work;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned users;
	u8			*buffer;	
	u8  		 buf_status;
	u8 mode;
	//struct timer_list   gfx1xm_timer;
#ifdef GFX1XM_FASYNC
	struct  fasync_struct *async;
#endif

#if CONFIG_HAS_EARLYSUSPEND
    struct early_suspend    early_fp;
#endif
};

/**********************IO Magic**********************/
#define  GFX1XM_IOC_MAGIC    'g'  //define magic number
struct gfx1xm_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
	#if PROCESSOR_64_BIT
	u64  buf;
	#else
	u8 *buf;
	#endif
};
//define commands
/*read/write GFX1XM registers*/
#define  GFX1XM_IOC_CMD	_IOWR(GFX1XM_IOC_MAGIC, 1, struct gfx1xm_ioc_transfer)
#define  GFX1XM_IOC_REINIT	_IO(GFX1XM_IOC_MAGIC, 0)
#define  GFX1XM_IOC_SETSPEED	_IOW(GFX1XM_IOC_MAGIC, 2, u32)
//#define  GFX1XM_IOC_DISABLE    _IO(GFX1XM_IOC_MAGIC, 3)
//#define  GFX1XM_IOC_ENABLE     _IO(GFX1XM_IOC_MAGIC, 4)

#define  GFX1XM_IOC_MAXNR    3

/*******************Refering to hardware platform*****************************/
#define		GFX1XM_IRQ_NUM		    30
#define		GFX1XM_IRQ_PIN_NUM		58
#define 	GFX1XM_IRQ_PIN 		   (GPIO58 | 0x80000000)
#define 	GFX1XM_IRQ_PIN_M_GPIO   GPIO_MODE_00
#define 	GFX1XM_IRQ_PIN_M_EINT   GPIO_MODE_04

#define 	GFX1XM_RST_PIN		          (GPIO66 | 0x80000000)
#define 	GFX1XM_RST_PIN_M_GPIO        GPIO_MODE_00
#define 	GFX1XM_RST_PIN_M_DAIPCMOUT   GPIO_MODE_01

#define		GFX1XM_SCK_PIN		   (GPIO119 | 0x80000000)            
#define		GFX1XM_SCK_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_SCK_PIN_M_SCK	GPIO_MODE_02

#define		GFX1XM_CS_PIN		     (GPIO118 | 0x80000000)
#define		GFX1XM_CS_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_CS_PIN_M_CS	  GPIO_MODE_02
	
#define		GFX1XM_MOSI_PIN		     (GPIO120 | 0x80000000)
#define		GFX1XM_MOSI_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_MOSI_PIN_M_MOSI	GPIO_MODE_02

#define		GFX1XM_MISO_PIN		     (GPIO121 | 0x80000000)
#define		GFX1XM_MISO_PIN_M_GPIO	GPIO_MODE_00
#define		GFX1XM_MISO_PIN_M_MISO	GPIO_MODE_02


extern  struct pinctrl *goodix_pinctrl;
extern  struct pinctrl_state *goodix_pin_irq, *gf518m_reset_high,*gf518m_reset_low;


/*Confure the IRQ pin for GFX1XM irq if necessary*/
inline static void gfx1xm_spi_pins_config(void)
{
#if 0
	/*cs*/
    mt_set_gpio_mode(GFX1XM_CS_PIN, GFX1XM_CS_PIN_M_CS);
    mt_set_gpio_pull_enable(GFX1XM_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_CS_PIN, GPIO_PULL_UP);
	/*sck*/
    mt_set_gpio_mode(GFX1XM_SCK_PIN, GFX1XM_SCK_PIN_M_SCK);
    mt_set_gpio_pull_enable(GFX1XM_SCK_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_SCK_PIN, GPIO_PULL_DOWN);
	/*miso*/

    mt_set_gpio_mode(GFX1XM_MISO_PIN, GFX1XM_MISO_PIN_M_MISO);
	//mt_set_gpio_dir(GFX1XM_MISO_PIN, GPIO_DIR_IN); 
    //mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_DISABLE);
	  mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_MISO_PIN, GPIO_PULL_UP);

	/*mosi*/
    mt_set_gpio_mode(GFX1XM_MOSI_PIN, GFX1XM_MOSI_PIN_M_MOSI);
    mt_set_gpio_pull_enable(GFX1XM_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GFX1XM_MOSI_PIN, GPIO_PULL_UP);

    msleep(1);
	#endif
}
inline static void gfx1xm_irq_cfg(void)
{
	/*Config IRQ pin, referring to platform.*/
	//mt_set_gpio_mode(GFX1XM_IRQ_PIN, GFX1XM_IRQ_PIN_M_EINT);
	//mt_set_gpio_dir(GFX1XM_IRQ_PIN, GPIO_DIR_IN); 
	//mt_set_gpio_pull_enable(GFX1XM_IRQ_PIN, GPIO_PULL_DISABLE);
}


inline static void gfx1xm_miso_pullup(void)
{
	/*Config MISO pin, referring to platform.*/
   // mt_set_gpio_mode(GFX1XM_MISO_PIN, GFX1XM_MISO_PIN_M_MISO);
    //mt_set_gpio_pull_enable(GFX1XM_MISO_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GFX1XM_MISO_PIN, GPIO_PULL_UP);
}



inline static void gfx1xm_miso_backnal(void)
{
	/*Config miso pin, referring to platform.*/	
    //mt_set_gpio_mode(GFX1XM_MOSI_PIN, GFX1XM_MOSI_PIN_M_MOSI);
    //mt_set_gpio_pull_enable(GFX1XM_MOSI_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GFX1XM_MOSI_PIN, GPIO_PULL_UP);
}


/********************************************************************
*CPU output low level in RST pin to reset GFX1XM. This is the MUST action for GFX1XM.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
inline static void gfx1xm_hw_reset(void)
{		/*rst pin referring to samsung KIT.*/
	//mt_set_gpio_mode(GFX1XM_RST_PIN, GFX1XM_RST_PIN_M_GPIO);
	//mt_set_gpio_dir(GFX1XM_RST_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GFX1XM_RST_PIN, GPIO_OUT_ZERO); 			 
	//msleep(5);  //delay for power to reset  typical:10ms max:50ms
	//mt_set_gpio_out(GFX1XM_RST_PIN, GPIO_OUT_ONE);
    //msleep(60);
	//pinctrl_select_state(goodix_pinctrl, gf518m_reset_low);
	__gpio_set_value(253, 0);
	msleep(10);	//delay for power to reset	typical:10ms max:50ms
	//pinctrl_select_state(goodix_pinctrl, gf518m_reset_high);	
	__gpio_set_value(253, 1);
	msleep(60);
    printk("gfx1xm_reset_ok\n");
}

int gfx1xm_spi_read_bytes(struct gfx1xm_dev *gfx1xm_dev,
                                u16 addr, u32 data_len, u8 *rx_buf);

int gfx1xm_spi_write_bytes(struct gfx1xm_dev *gfx1xm_dev,
                                u16 addr, u32 data_len, u8 *tx_buf);
int gfx1xm_fw_update(struct gfx1xm_dev* gfx1xm_dev, unsigned char *buf, unsigned short len);

#endif
