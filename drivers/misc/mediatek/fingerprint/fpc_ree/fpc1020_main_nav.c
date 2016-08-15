/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/types.h>

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>

#include <linux/of.h>
#include "fpc1020.h"
#include "fpc1020_common.h"
#include "fpc1020_regs.h"
#include "fpc1020_capture.h"
#include "fpc1020_regulator.h"

#include "fpc1020_input.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#define FPC_CONFIG_HAS_EARLYSUSPEND
#endif

#define SUSPEND_WITH_POWER_OFF

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 touch sensor driver.");


/* -------------------------------------------------------------------- */
/* fpc1020 sensor commands and registers				*/
/* -------------------------------------------------------------------- */
typedef enum {
	FPC_1020_ERROR_REG_BIT_FIFO_UNDERFLOW = 1 << 0
} fpc1020_error_reg_t;



/* -------------------------------------------------------------------- */
/* global variables							*/
/* -------------------------------------------------------------------- */
static int fpc1020_device_count;


/* -------------------------------------------------------------------- */
/* fpc1020 data types							*/
/* -------------------------------------------------------------------- */
struct fpc1020_attribute {
	struct device_attribute attr;
	size_t offset;
};

enum {
	FPC1020_WORKER_IDLE_MODE = 0,
	FPC1020_WORKER_CAPTURE_MODE,
	FPC1020_WORKER_INPUT_MODE,
	FPC1020_WORKER_EXIT
};


/* -------------------------------------------------------------------- */
/* fpc1020 driver constants						*/
/* -------------------------------------------------------------------- */
#define FPC1020_CLASS_NAME                      "fpsensor"
#define FPC1020_WORKER_THREAD_NAME		"fpc1020_worker"

struct fpc1020_platform_data fpc1020_data = {
	.irq_gpio = 86,
	.reset_gpio = 253,
	.cs_gpio = 237,
	.external_supply_mv = 0,
	.txout_boost = 0,
	.force_hwid = 0,
	.use_regulator_for_bezel = 0,
	.use_fpc2050 = 1,
};

int g_fpc_run_mode = FPC1020_WORKER_INPUT_MODE;
volatile int g_fpc_suspend = 0;// 0 : not in suspend  1:enter suspend
volatile int g_fpc_tee_used = 0;// volatile
volatile int g_fpc_suspend_clock_disabled = 0;// volatile
volatile int fpc1020_open_cnt = 0;
volatile int g_fpc_input_mode_resume = 0;
#define DEFAULT_BOOST_FREQ  	(1950000) 		/* 1950M Hz*/
#define DEFUALT_BOOST_TIME	(5000 * 1000)		/* 5000ms */
#define INFO(fmt,args...)  	printk(KERN_EMERG "mzTeeBoost:%s " fmt,__func__,##args)
#define MIN_CPUS		5

static struct pm_qos_request qos_req_frq;
static struct pm_qos_request qos_req_cpu;

/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
static int __init fpc1020_init(void);

static void __exit fpc1020_exit(void);

static int  fpc1020_probe(struct spi_device *spi);

static int fpc1020_remove(struct spi_device *spi);

static int fpc1020_suspend(struct device *dev);

static int fpc1020_resume(struct device *dev);

#ifdef FPC_CONFIG_HAS_EARLYSUSPEND
static void fpc1020_early_suspend(struct early_suspend *h);
static void fpc1020_late_resume(struct early_suspend *h);
#else
static int fpc1020_fb_notifier_callback(struct notifier_block *nb,
		unsigned long val, void *data);
#endif

static int fpc1020_open(struct inode *inode, struct file *file);

static ssize_t fpc1020_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos);

static ssize_t fpc1020_read(struct file *file, char *buff,
				size_t count, loff_t *ppos);

static int fpc1020_release(struct inode *inode, struct file *file);

static unsigned int fpc1020_poll(struct file *file, poll_table *wait);

static int fpc1020_cleanup(fpc1020_data_t *fpc1020, struct spi_device *spidev);

static int  fpc1020_param_init(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata);

//#ifdef CONFIG_ARCH_MSM
static int  fpc1020_supply_init(fpc1020_data_t *fpc1020);
//#endif

static int  fpc1020_reset_init(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata);

static int  fpc1020_irq_init(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata);

static int  fpc1020_spi_setup(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata);

static int  fpc1020_worker_init(fpc1020_data_t *fpc1020);

static int  fpc1020_worker_destroy(fpc1020_data_t *fpc1020);

static int  fpc1020_create_class(fpc1020_data_t *fpc1020);

static int  fpc1020_create_device(fpc1020_data_t *fpc1020);

static int fpc1020_manage_sysfs(fpc1020_data_t *fpc1020,
				struct spi_device *spi, bool create);

irqreturn_t fpc1020_interrupt(int irq, void *_fpc1020);

static ssize_t fpc1020_show_attr_setup(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t fpc1020_store_attr_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count);

static ssize_t fpc1020_show_attr_diag(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t fpc1020_store_attr_diag(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count);

static u8 fpc1020_selftest_short(fpc1020_data_t *fpc1020);

static int fpc1020_start_capture(fpc1020_data_t *fpc1020);

static int fpc1020_new_job(fpc1020_data_t *fpc1020, int new_job);

static int fpc1020_worker_goto_idle(fpc1020_data_t *fpc1020);

static int fpc1020_worker_function(void *_fpc1020);

static int fpc1020_worker_goto_sleep(fpc1020_data_t *fpc1020);


#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_start_navigation(fpc1020_data_t *fpc1020);
#endif


/* -------------------------------------------------------------------- */
/* External interface							*/
/* -------------------------------------------------------------------- */
module_init(fpc1020_init);
module_exit(fpc1020_exit);

static const struct dev_pm_ops fpc1020_pm = {
	.suspend = fpc1020_suspend,
	.resume = fpc1020_resume
};

#ifdef CONFIG_OF
static struct of_device_id fpc1020_of_match[]= {
	{ .compatible = "fpc,fpc1020,fp_spi", },
	{}
};

MODULE_DEVICE_TABLE(of, fpc1020_of_match);
#endif

static struct spi_driver fpc1020_driver = {
	.driver = {
		.name	= SPI_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
//#ifndef FPC_CONFIG_HAS_EARLYSUSPEND
		.pm     = &fpc1020_pm,
//#endif
#ifdef CONFIG_OF
		.of_match_table = fpc1020_of_match,
#endif
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};

static const struct file_operations fpc1020_fops = {
	.owner          = THIS_MODULE,
	.open           = fpc1020_open,
	.write          = fpc1020_write,
	.read           = fpc1020_read,
	.release        = fpc1020_release,
	.poll           = fpc1020_poll,
};


/* -------------------------------------------------------------------- */
/* devfs								*/
/* -------------------------------------------------------------------- */
#define FPC1020_ATTR(__grp, __field, __mode)				\
{									\
	.attr = __ATTR(__field, (__mode),				\
	fpc1020_show_attr_##__grp,					\
	fpc1020_store_attr_##__grp),					\
	.offset = offsetof(struct fpc1020_##__grp, __field)		\
}

#define FPC1020_DEV_ATTR(_grp, _field, _mode)				\
struct fpc1020_attribute fpc1020_attr_##_field =			\
					FPC1020_ATTR(_grp, _field, (_mode))

//#define DEVFS_SETUP_MODE (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)
#define DEVFS_SETUP_MODE (0664)

static FPC1020_DEV_ATTR(setup, adc_gain,		DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, adc_shift,		DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, capture_mode,		DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, capture_count,		DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, capture_settings_mux,	DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, pxl_ctrl,		DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, capture_row_start,	DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, capture_row_count,	DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, capture_col_start,	DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, capture_col_groups,	DEVFS_SETUP_MODE);
#ifdef DEBUG_INPUT_MODE
static FPC1020_DEV_ATTR(setup, input_mode,            DEVFS_SETUP_MODE);
#endif

static struct attribute *fpc1020_setup_attrs[] = {
	&fpc1020_attr_adc_gain.attr.attr,
	&fpc1020_attr_adc_shift.attr.attr,
	&fpc1020_attr_capture_mode.attr.attr,
	&fpc1020_attr_capture_count.attr.attr,
	&fpc1020_attr_capture_settings_mux.attr.attr,
	&fpc1020_attr_pxl_ctrl.attr.attr,
	&fpc1020_attr_capture_row_start.attr.attr,
	&fpc1020_attr_capture_row_count.attr.attr,
	&fpc1020_attr_capture_col_start.attr.attr,
	&fpc1020_attr_capture_col_groups.attr.attr,
#ifdef DEBUG_INPUT_MODE
	&fpc1020_attr_input_mode.attr.attr,
#endif
	NULL
};

static const struct attribute_group fpc1020_setup_attr_group = {
	.attrs = fpc1020_setup_attrs,
	.name = "setup"
};

//#define DEVFS_DIAG_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
//#define DEVFS_DIAG_MODE_RW (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)
#define DEVFS_DIAG_MODE_RO (0444)
#define DEVFS_DIAG_MODE_RW (0664)

static FPC1020_DEV_ATTR(diag, chip_id,		DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, reset,		DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, mode,		DEVFS_DIAG_MODE_RW);
static FPC1020_DEV_ATTR(diag, state,		DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, selftest,		DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, spi_register,	DEVFS_DIAG_MODE_RW);
static FPC1020_DEV_ATTR(diag, spi_regsize,	DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, spi_data ,	DEVFS_DIAG_MODE_RW);
static FPC1020_DEV_ATTR(diag, last_capture_time,DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, finger_present_status, DEVFS_DIAG_MODE_RO);
#ifdef FPS_TEST
static FPC1020_DEV_ATTR(diag, capture_time,     DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, frames_captured,  DEVFS_DIAG_MODE_RO);
static FPC1020_DEV_ATTR(diag, frames_stored,    DEVFS_DIAG_MODE_RO);
#endif

static struct attribute *fpc1020_diag_attrs[] = {
	&fpc1020_attr_chip_id.attr.attr,
	&fpc1020_attr_reset.attr.attr,
	&fpc1020_attr_mode.attr.attr,
	&fpc1020_attr_state.attr.attr,
	&fpc1020_attr_selftest.attr.attr,
	&fpc1020_attr_spi_register.attr.attr,
	&fpc1020_attr_spi_regsize.attr.attr,
	&fpc1020_attr_spi_data.attr.attr,
	&fpc1020_attr_last_capture_time.attr.attr,
	&fpc1020_attr_finger_present_status.attr.attr,
#ifdef FPS_TEST
	&fpc1020_attr_capture_time.attr.attr,
	&fpc1020_attr_frames_captured.attr.attr,
	&fpc1020_attr_frames_stored.attr.attr,
#endif
	NULL
};

static const struct attribute_group fpc1020_diag_attr_group = {
	.attrs = fpc1020_diag_attrs,
	.name = "diag"
};


/* -------------------------------------------------------------------- */
/* SPI debug interface, prototypes					*/
/* -------------------------------------------------------------------- */
static int fpc1020_spi_debug_select(fpc1020_data_t *fpc1020,
				fpc1020_reg_t reg);

static int fpc1020_spi_debug_value_write(fpc1020_data_t *fpc1020, u64 data);

static int fpc1020_spi_debug_buffer_write(fpc1020_data_t *fpc1020,
					const char *data,
					size_t count);

static int fpc1020_spi_debug_value_read(fpc1020_data_t *fpc1020,
					u64 *data);

static int fpc1020_spi_debug_buffer_read(fpc1020_data_t *fpc1020,
					u8 *data,
					size_t max_count);

static void fpc1020_spi_debug_buffer_to_hex_string(char *string,
						u8 *buffer,
						size_t bytes);

static int fpc1020_spi_debug_hex_string_to_buffer(u8 *buffer,
						size_t buf_size,
						const char *string,
						size_t chars);

void fpc1020_spi_enable_clk( void )
{
	printk(KERN_EMERG "%s\n", __func__);
#if (!defined(CONFIG_MT_SPI_FPGA_ENABLE))
	//enable_clock(MT_CG_PERI_SPI0, "tee");
	//clk_set_force_on(MT_CG_PERI_SPI0);
#endif
	return;
}


void fpc1020_spi_disable_clk( void )
{
	printk(KERN_EMERG "%s\n", __func__);
#if (!defined(CONFIG_MT_SPI_FPGA_ENABLE))
	//disable_clock(MT_CG_PERI_SPI0, "tee");
	//clk_clr_force_on(MT_CG_PERI_SPI0);
#endif
	return;
}

#if 0
static void fpc1020_poll_work(struct work_struct *work)
{

	return;
}
#endif

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
static int __init fpc1020_init(void)
{
	if (spi_register_driver(&fpc1020_driver))
		return -EINVAL;

	return 0;
}


/* -------------------------------------------------------------------- */
static void __exit fpc1020_exit(void)
{
	printk(KERN_EMERG "%s\n", __func__);

	spi_unregister_driver(&fpc1020_driver);
}

int g_fingerprint_is_fpc = 0;
int fingerprint_is_fpc(void)
{
	return g_fingerprint_is_fpc;
}

/* -------------------------------------------------------------------- */
static int  fpc1020_probe(struct spi_device *spi)
{
	struct fpc1020_platform_data *fpc1020_pdata;
	int error = 0;
	fpc1020_data_t *fpc1020 = NULL;
	size_t buffer_size;

	printk(KERN_EMERG "%s probe begin\n", __func__);

	fpc1020 = kzalloc(sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		dev_err(&spi->dev,
		"failed to allocate memory for struct fpc1020_data\n");

		return -ENOMEM;
	}

	printk(KERN_EMERG "%s\n", __func__);

	buffer_size = fpc1020_calc_huge_buffer_minsize(fpc1020);
	error = fpc1020_manage_huge_buffer(fpc1020, buffer_size);
	if (error)
		goto err;

	spi_set_drvdata(spi, fpc1020);
	fpc1020->spi = spi;

	fpc1020->reset_gpio = 253;
	fpc1020->irq_gpio   = 86;
	fpc1020->cs_gpio    = 237;

	fpc1020->irq        = -EINVAL;

	init_waitqueue_head(&fpc1020->wq_irq_return);
	init_waitqueue_head(&fpc1020->wq_suspend);

	error = fpc1020_init_capture(fpc1020);
	if (error)
		goto err;

	fpc1020_pdata = &fpc1020_data;

	error = fpc1020_param_init(fpc1020, fpc1020_pdata);
	if (error)
		goto err;

	error = fpc1020_supply_init(fpc1020);
	if (error)
		goto err;

	error = fpc1020_reset_init(fpc1020, fpc1020_pdata);
	if (error)
		goto err;

	error = fpc1020_irq_init(fpc1020, fpc1020_pdata);
	if (error)
		goto err;

	error = fpc1020_spi_setup(fpc1020, fpc1020_pdata);
	if (error)
		goto err;

	error = fpc1020_reset(fpc1020);
	if (error)
		goto err;

	error = fpc1020_check_hw_id(fpc1020);
	if (error)
		goto err;

	buffer_size = fpc1020_calc_huge_buffer_minsize(fpc1020);
	error = fpc1020_manage_huge_buffer(fpc1020, buffer_size);
	if (error)
		goto err;

	error = fpc1020_setup_defaults(fpc1020);
	if (error)
		goto err;

	error = fpc1020_create_class(fpc1020);
	if (error)
		goto err;

	error = fpc1020_create_device(fpc1020);
	if (error)
		goto err;

	sema_init(&fpc1020->mutex, 0);

	error = fpc1020_manage_sysfs(fpc1020, spi, true);
	if (error)
		goto err;
	#if FPC_SPI_DEBUG
	#else
	cdev_init(&fpc1020->cdev, &fpc1020_fops);
	fpc1020->cdev.owner = THIS_MODULE;

	error = cdev_add(&fpc1020->cdev, fpc1020->devno, 1);
	if (error) {
		dev_err(&fpc1020->spi->dev, "cdev_add failed.\n");
		goto err_chrdev;
	}

	error = fpc1020_worker_init(fpc1020);
	if (error)
		goto err_cdev;

	error = fpc1020_calc_finger_detect_threshold_min(fpc1020);
	if (error <  0 )
		goto err_cdev;

	error = fpc1020_set_finger_detect_threshold(fpc1020, error);
	if (error <  0 )
		goto err_cdev;

#ifdef CONFIG_INPUT_FPC1020_NAV
	error = fpc1020_input_init(fpc1020);
	if (error)
		goto err_cdev;

	if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
		fpc1020_start_navigation(fpc1020);

#else
	//error = fpc1020_sleep(fpc1020, true);
	//if (error)
	//	goto err_cdev;
#endif
	#endif
#ifdef FPC_CONFIG_HAS_EARLYSUSPEND
	fpc1020->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 50;
	fpc1020->early_suspend.suspend = fpc1020_early_suspend;
	fpc1020->early_suspend.resume = fpc1020_late_resume;
	register_early_suspend(&fpc1020->early_suspend);
#else
	fpc1020->fb_notifier.notifier_call = fpc1020_fb_notifier_callback;
	error = fb_register_client(&fpc1020->fb_notifier);
	if (error < 0) {
		dev_err(&fpc1020->spi->dev, "fb_register_client failed");
		goto err_cdev;
	}
#endif
	g_fingerprint_is_fpc = 1;
	up(&fpc1020->mutex);
	printk(KERN_EMERG "%s probe success\n", __func__);
	return 0;

err_cdev:
	cdev_del(&fpc1020->cdev);

err_chrdev:
	unregister_chrdev_region(fpc1020->devno, 1);

	fpc1020_manage_sysfs(fpc1020, spi, false);

err:
	fpc1020_cleanup(fpc1020, spi);
	printk(KERN_EMERG "%s probe failed\n", __func__);
	return error;
}


/* -------------------------------------------------------------------- */
static int  fpc1020_remove(struct spi_device *spi)
{
	fpc1020_data_t *fpc1020 = spi_get_drvdata(spi);

	printk(KERN_EMERG "%s\n", __func__);

	pm_qos_remove_request(&qos_req_frq);
	pm_qos_remove_request(&qos_req_cpu);

#ifdef FPC_CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&fpc1020->early_suspend);
#endif

	fpc1020_manage_sysfs(fpc1020, spi, false);

	fpc1020_sleep(fpc1020, true);

	cdev_del(&fpc1020->cdev);

	unregister_chrdev_region(fpc1020->devno, 1);

	fpc1020_cleanup(fpc1020, spi);

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc1020_suspend(struct device *dev)
{
	int error = 0;

	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if(down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	//pm_qos_remove_request(&qos_req_frq);
	//pm_qos_remove_request(&qos_req_cpu);

	fpc1020_worker_goto_idle(fpc1020);

#ifndef SUSPEND_WITH_POWER_OFF
	fpc1020_wake_up(fpc1020);

	error = fpc1020_sleep(fpc1020, true);
	if( error != 0)
		dev_err(&fpc1020->spi->dev, "%s: err = %d\n", __func__,error);

#else
	error = fpc1020_regulator_set(fpc1020, false);
#endif //SUSPEND_WITH_POWER_OFF

	up(&fpc1020->mutex);

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc1020_resume(struct device *dev)
{
	int error = 0;

	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if(down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	error = fpc1020_wake_up(fpc1020);
	if( error != 0)
		dev_err(&fpc1020->spi->dev, "%s: err = %d\n", __func__,error);

#ifdef CONFIG_INPUT_FPC1020_NAV
	if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
	{
		g_fpc_input_mode_resume = 0;
		fpc1020_start_navigation(fpc1020);
	}
	else
	{
		fpc1020_worker_goto_idle(fpc1020);
	}
#else
	fpc1020_worker_goto_idle(fpc1020);

	//fpc1020_sleep(fpc1020, true);
#endif

	up(&fpc1020->mutex);

	return 0;
}

#ifdef FPC_CONFIG_HAS_EARLYSUSPEND
static void fpc1020_early_suspend(struct early_suspend *h)
{
	fpc1020_data_t *fpc1020 =
			container_of(h, fpc1020_data_t,early_suspend);

	down_interruptible(&fpc1020->mutex);

	g_fpc_suspend = 1;

#ifdef CONFIG_INPUT_FPC1020_NAV
	if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
	{
		if( fpc1020->nav.enabled || fpc1020->worker.req_mode == FPC1020_WORKER_INPUT_MODE)
			g_fpc_input_mode_resume = 1;
	}
#endif

	fpc1020_worker_goto_idle(fpc1020);


	up(&fpc1020->mutex);

	return;
}

static void fpc1020_late_resume(struct early_suspend *h)
{
	fpc1020_data_t *fpc1020 =
			container_of(h, fpc1020_data_t,early_suspend);

	down_interruptible(&fpc1020->mutex);

	g_fpc_suspend = 0;

	if(g_fpc_input_mode_resume)
	{
		g_fpc_input_mode_resume = 0;
		if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
			fpc1020_start_navigation(fpc1020);
	}

	if(g_fpc_suspend_clock_disabled){
		g_fpc_suspend_clock_disabled = 0;
		fpc1020_spi_enable_clk();
		wake_up_interruptible(&fpc1020->wq_suspend);
		dev_dbg(&fpc1020->spi->dev,"%s:wake_up_interruptible\n", __func__);
	}

	up(&fpc1020->mutex);
	return;
}
#else

static int fpc1020_fb_notifier_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{

	struct fb_event *evdata = data;
	unsigned int blank;
	fpc1020_data_t *fpc1020 = container_of(nb,
					fpc1020_data_t, fb_notifier);

	if(val != FB_EVENT_BLANK)
		return 0;

	if(evdata && evdata->data && val == FB_EVENT_BLANK) {
		blank = *(int *)(evdata->data);

		switch(blank) {
		case FB_BLANK_POWERDOWN:
			dev_info(&fpc1020->spi->dev, "%s:FB_BLANK_POWERDOWN\n",
					__func__);
			down_interruptible(&fpc1020->mutex);

			g_fpc_suspend = 1;

			#ifdef CONFIG_INPUT_FPC1020_NAV
			if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
			{
				if( fpc1020->nav.enabled || fpc1020->worker.req_mode == FPC1020_WORKER_INPUT_MODE)
					g_fpc_input_mode_resume = 1;
			}
			#endif

			fpc1020_worker_goto_idle(fpc1020);


			up(&fpc1020->mutex);
			break;

		case FB_BLANK_UNBLANK:
			dev_info(&fpc1020->spi->dev, "%s:FB_BLANK_UNBLANK\n",
					__func__);
			down_interruptible(&fpc1020->mutex);

			g_fpc_suspend = 0;

			if(g_fpc_input_mode_resume)
			{
				g_fpc_input_mode_resume = 0;
				if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
					fpc1020_start_navigation(fpc1020);
			}

			if(g_fpc_suspend_clock_disabled){
				g_fpc_suspend_clock_disabled = 0;
				fpc1020_spi_enable_clk();
				wake_up_interruptible(&fpc1020->wq_suspend);
				dev_dbg(&fpc1020->spi->dev,"%s:wake_up_interruptible\n", __func__);
			}

			up(&fpc1020->mutex);
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}

#endif

static void tee_boost_func(void)
{
#if 0
	unsigned int target,cpus;
	unsigned int boost_time = DEFUALT_BOOST_TIME;

	if( !pm_qos_request_active(&qos_req_frq) )
		pm_qos_add_request(&qos_req_frq, PM_QOS_CPU_FREQ_MIN, 0);

	if( !pm_qos_request_active(&qos_req_cpu) )
		pm_qos_add_request(&qos_req_cpu, PM_QOS_CPU_NUM_MIN, 0);

	cpus = num_online_cpus()+MIN_CPUS;
	if(cpus > 8)
		cpus = 8;
	target = DEFAULT_BOOST_FREQ;

	INFO("pm_qos_update_request_timeout %d cpu freq for %d msecs\n", target, boost_time/1000);

	pm_qos_update_request_timeout(&qos_req_frq, target, boost_time);
	pm_qos_update_request_timeout(&qos_req_cpu, cpus, boost_time);
#endif
}


/* -------------------------------------------------------------------- */
static int fpc1020_open(struct inode *inode, struct file *file)

{
	fpc1020_data_t *fpc1020;

	fpc1020 = container_of(inode->i_cdev, fpc1020_data_t, cdev);

	dev_dbg(&fpc1020->spi->dev, "%s(+%d)\n", __func__,fpc1020_open_cnt+1);

	if(down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	fpc1020_open_cnt ++;

	file->private_data = fpc1020;
	fpc1020_spi_enable_clk();

	if(g_fpc_suspend || fpc1020_open_cnt > 1)
	{
		up(&fpc1020->mutex);
		return 0;
	}

	g_fpc_tee_used = 1;

	tee_boost_func();

#ifdef CONFIG_INPUT_FPC1020_NAV
	if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
	{
		if( fpc1020->nav.enabled || fpc1020->worker.req_mode == FPC1020_WORKER_INPUT_MODE)
			fpc1020_worker_goto_idle(fpc1020);
	}
#endif

	up(&fpc1020->mutex);

	return 0;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos)
{
	printk(KERN_EMERG "%s\n", __func__);


	return -ENOTTY;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_read(struct file *file, char *buff,
				size_t count, loff_t *ppos)
{
	fpc1020_data_t *fpc1020 = file->private_data;
	int error = 0;
	u32 max_data;
	u32 avail_data;

	//dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if(down_interruptible(&fpc1020->mutex)) {
		dev_err(&fpc1020->spi->dev, "%s ERESTARTSYS\n", __func__);
		return -ERESTARTSYS;
	}

	if(g_fpc_suspend)
	{
		if(!g_fpc_suspend_clock_disabled){
			g_fpc_suspend_clock_disabled = 1;
			fpc1020_spi_disable_clk();
		}
		up(&fpc1020->mutex);
		dev_dbg(&fpc1020->spi->dev, "%s:wait_event_interruptible:+\n", __func__);
		//error = wait_event_interruptible(fpc1020->wq_suspend, !g_fpc_suspend);
		error = wait_event_interruptible_timeout(fpc1020->wq_suspend, !g_fpc_suspend,msecs_to_jiffies(1000));
		dev_dbg(&fpc1020->spi->dev, "%s:wait_event_interruptible:- error = %d\n", __func__,error);
		return 0;
	}

	if (fpc1020->capture.available_bytes > 0) {
		goto copy_data;
	} else {

		if (fpc1020->capture.read_pending_eof) {
			fpc1020->capture.read_pending_eof = false;
			error = 0;
			goto out;
		}

		if (file->f_flags & O_NONBLOCK) {
			if (fpc1020_capture_check_ready(fpc1020)) {
				error = fpc1020_start_capture(fpc1020);
				if (error) {
					dev_err(&fpc1020->spi->dev, "%s fpc1020_start_capture0\n", __func__);
					goto out;
				}
			}

			error = -EWOULDBLOCK;
			dev_err(&fpc1020->spi->dev, "%s file->f_flags & O_NONBLOCK error: %d\n", __func__, error);
			goto out;

		} else {
			error = fpc1020_start_capture(fpc1020);
			if (error) {
				dev_err(&fpc1020->spi->dev, "%s fpc1020_start_capture1\n", __func__);
				goto out;
			}
		}
	}

	error = wait_event_interruptible(
			fpc1020->capture.wq_data_avail,
			(fpc1020->capture.available_bytes > 0));

	if (error) {
		dev_err(&fpc1020->spi->dev, "%s wait_event_interruptible\n", __func__);
		goto out;
	}

	if (fpc1020->capture.last_error != 0) {
		error = fpc1020->capture.last_error;
		dev_err(&fpc1020->spi->dev, "%s fpc1020->capture.last_error != 0\n", __func__);
		goto out;
	}

copy_data:
	avail_data = fpc1020->capture.available_bytes;
	max_data = (count > avail_data) ? avail_data : count;

	if (max_data) {
		error = copy_to_user(buff,
			&fpc1020->huge_buffer[fpc1020->capture.read_offset],
			max_data);

		if (error) {
			dev_err(&fpc1020->spi->dev, "%s copy_to_user\n", __func__);
			goto out;
		}

		fpc1020->capture.read_offset += max_data;
		fpc1020->capture.available_bytes -= max_data;

		error = max_data;

		if (fpc1020->capture.available_bytes == 0)
			fpc1020->capture.read_pending_eof = true;
	}

out:
	up(&fpc1020->mutex);
	if (error < 0) {
		dev_err(&fpc1020->spi->dev, "%s error: %d\n", __func__, error);
	}
	return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_release(struct inode *inode, struct file *file)
{
	fpc1020_data_t *fpc1020 = file->private_data;
	int status = 0;

	dev_dbg(&fpc1020->spi->dev, "%s(%d-)\n", __func__,fpc1020_open_cnt);

	if(down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	fpc1020_open_cnt--;

	if (fpc1020_open_cnt == 0)
		g_fpc_tee_used = 0;

	if(fpc1020_open_cnt < 0){
		fpc1020_open_cnt = 0;
		up(&fpc1020->mutex);
		return 0;
	}

	if( fpc1020_open_cnt == 0)
	{
#ifdef CONFIG_INPUT_FPC1020_NAV
		if(g_fpc_run_mode == FPC1020_WORKER_INPUT_MODE)
			fpc1020_start_navigation(fpc1020);
		else
			fpc1020_worker_goto_idle(fpc1020);
#else
		fpc1020_worker_goto_idle(fpc1020);
#endif
	}
	if(g_fpc_suspend_clock_disabled)
		g_fpc_suspend_clock_disabled = 0;
	else
		fpc1020_spi_disable_clk();
	up(&fpc1020->mutex);

	return status;
}


/* -------------------------------------------------------------------- */
static unsigned int fpc1020_poll(struct file *file, poll_table *wait)
{
	fpc1020_data_t *fpc1020 = file->private_data;
	unsigned int ret = 0;
	fpc1020_capture_mode_t mode = fpc1020->setup.capture_mode;
	bool blocking_op;

	if(down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	if (fpc1020->capture.available_bytes > 0)
		ret |= (POLLIN | POLLRDNORM);
	else if (fpc1020->capture.read_pending_eof)
		ret |= POLLHUP;
	else { /* available_bytes == 0 && !pending_eof */

		blocking_op =
			(mode == FPC1020_MODE_WAIT_AND_CAPTURE) ? true : false;

		switch (fpc1020->capture.state) {
		case FPC1020_CAPTURE_STATE_IDLE:
			if (!blocking_op)
				ret |= POLLIN;
			break;

		case FPC1020_CAPTURE_STATE_STARTED:
		case FPC1020_CAPTURE_STATE_PENDING:
		case FPC1020_CAPTURE_STATE_WRITE_SETTINGS:
		case FPC1020_CAPTURE_STATE_WAIT_FOR_FINGER_DOWN:
		case FPC1020_CAPTURE_STATE_ACQUIRE:
		case FPC1020_CAPTURE_STATE_FETCH:
		case FPC1020_CAPTURE_STATE_WAIT_FOR_FINGER_UP:
		case FPC1020_CAPTURE_STATE_COMPLETED:
			ret |= POLLIN;

			poll_wait(file, &fpc1020->capture.wq_data_avail, wait);

			if (fpc1020->capture.available_bytes > 0)
				ret |= POLLRDNORM;
			else if (blocking_op)
				ret = 0;

			break;

		case FPC1020_CAPTURE_STATE_FAILED:
			if (!blocking_op)
				ret |= POLLIN;
			break;

		default:
			dev_err(&fpc1020->spi->dev,
				"%s unknown state\n", __func__);
			break;
		}
	}

	up(&fpc1020->mutex);

	return ret;
}


/* -------------------------------------------------------------------- */
static int fpc1020_cleanup(fpc1020_data_t *fpc1020, struct spi_device *spidev)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020_worker_destroy(fpc1020);

	if (!IS_ERR_OR_NULL(fpc1020->device))
		device_destroy(fpc1020->class, fpc1020->devno);

	class_destroy(fpc1020->class);

	if (fpc1020->irq >= 0)
		free_irq(fpc1020->irq, fpc1020);
#if 0
	if (gpio_is_valid_a(fpc1020->irq_gpio))
		gpio_free(fpc1020->irq_gpio);

	if (gpio_is_valid_a(fpc1020->reset_gpio))
		gpio_free(fpc1020->reset_gpio);

#ifdef MANUAL_CS
	if (gpio_is_valid_a(fpc1020->cs_gpio))
		gpio_free(fpc1020->cs_gpio);
#endif
#endif

	fpc1020_manage_huge_buffer(fpc1020, 0);

#ifdef CONFIG_INPUT_FPC1020_NAV
	fpc1020_input_destroy(fpc1020);
#endif

//#ifdef CONFIG_ARCH_MSM
	fpc1020_regulator_release(fpc1020);
//#endif

#ifdef CONFIG_INPUT_FPC1020_NAV
	kfree(fpc1020->prev_img_buf);
	kfree(fpc1020->cur_img_buf);
#endif

	kfree(fpc1020);

	spi_set_drvdata(spidev, NULL);

	return 0;
}


/* -------------------------------------------------------------------- */
static int  fpc1020_param_init(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)
{
	fpc1020->vddtx_mv    = pdata->external_supply_mv;
	fpc1020->txout_boost = pdata->txout_boost;

	if (fpc1020->vddtx_mv > 0) {
		dev_info(&fpc1020->spi->dev,
			"External TxOut supply (%d mV)\n",
			fpc1020->vddtx_mv);
	} else {
		dev_info(&fpc1020->spi->dev,
			"Internal TxOut supply (boost %s)\n",
			(fpc1020->txout_boost) ? "ON" : "OFF");
	}

	fpc1020->force_hwid = pdata->force_hwid;
	fpc1020->use_regulator_for_bezel = pdata->use_regulator_for_bezel;
	fpc1020->use_fpc2050 = pdata->use_fpc2050;

	return 0;
}

static int fpc1020_supply_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	error = fpc1020_regulator_configure(fpc1020);
	if (error) {
		dev_err(&fpc1020->spi->dev,
			"fpc1020_probe - regulator configuration failed.\n");
		goto err;
	}

	error = fpc1020_regulator_set(fpc1020, true);
	if (error) {
		dev_err(&fpc1020->spi->dev,
			"fpc1020_probe - regulator enable failed.\n");
		goto err;
	}

err:
	return error;
}

/* -------------------------------------------------- */
static int  fpc1020_reset_init(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)
{
	dev_info(&fpc1020->spi->dev,
			"Assign HW reset -> GPIO%d\n", pdata->reset_gpio);
	fpc1020->soft_reset_enabled = false;
	fpc1020->reset_gpio = pdata->reset_gpio;
	__gpio_set_value(fpc1020->reset_gpio, 1);
	return 0;
}


/* -------------------------------------------------------------------- */
static int  fpc1020_irq_init(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)
{
	int error = 0;

	unsigned int fpc_irq;
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fpc1145");
	if (node) {
		fpc_irq = irq_of_parse_and_map(node, 0);
		pr_err("fpc_irq = %u\n", fpc_irq);
		fpc1020->irq = fpc_irq;
	} else
		pr_err("[fpc]%s can't find compatible node\n", __func__);

	printk(KERN_EMERG "%s:fpc1020->irq = %d\n", __func__, fpc1020->irq);

	if (fpc1020->irq < 0) {
		dev_err(&fpc1020->spi->dev, "get fpc irq failed.\n");
		error = fpc1020->irq;
		return error;
	}

	error = request_irq(fpc1020->irq, fpc1020_interrupt,
			IRQF_TRIGGER_RISING , "fpc-eint", fpc1020);

	printk(KERN_EMERG "%s:request_irq %i .error = %d\n", __func__,fpc1020->irq,error);

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"request_irq %i failed.\n",
			fpc1020->irq);

		fpc1020->irq = -EINVAL;

		return error;
	}

	return error;
}


/* -------------------------------------------------------------------- */
static int  fpc1020_spi_setup(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)
{
	int error = 0;

	printk(KERN_EMERG "%s\n", __func__);

	/*SPI parameters.*/
	fpc1020->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
	fpc1020->spi->max_speed_hz = 2*1000*1000; //1MHZ
	fpc1020->spi->irq = fpc1020->irq;
	fpc1020->spi->bits_per_word = 8; //?
	error = spi_setup(fpc1020->spi);

	printk(KERN_EMERG "spi_setup error = %d \n",error);

	if (error) {
		dev_err(&fpc1020->spi->dev, "spi_setup failed\n");
		goto out_err;
	}

out_err:
	return error;
}


/* -------------------------------------------------------------------- */
static int  fpc1020_worker_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	printk(KERN_EMERG "%s\n", __func__);

	init_waitqueue_head(&fpc1020->worker.wq_wait_job);
	sema_init(&fpc1020->worker.sem_idle, 0);

	fpc1020->worker.req_mode = FPC1020_WORKER_IDLE_MODE;

	fpc1020->worker.thread = kthread_run(fpc1020_worker_function,
					   fpc1020, "%s",
					   FPC1020_WORKER_THREAD_NAME);

	if (IS_ERR(fpc1020->worker.thread)) {
		dev_err(&fpc1020->spi->dev, "kthread_run failed.\n");
		error = (int)PTR_ERR(fpc1020->worker.thread);
	}

	return error;
}


/* -------------------------------------------------------------------- */
static int  fpc1020_worker_destroy(fpc1020_data_t *fpc1020)
{
	int error = 0;

	printk(KERN_EMERG "%s\n", __func__);

	if (fpc1020->worker.thread) {
		fpc1020_worker_goto_idle(fpc1020);

		fpc1020->worker.req_mode = FPC1020_WORKER_EXIT;
		wake_up_interruptible(&fpc1020->worker.wq_wait_job);
		kthread_stop(fpc1020->worker.thread);
	}

	return error;
}

/* -------------------------------------------------------------------- */
static int  fpc1020_create_class(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->class = class_create(THIS_MODULE, FPC1020_CLASS_NAME);

	if (IS_ERR(fpc1020->class)) {
		dev_err(&fpc1020->spi->dev, "failed to create class.\n");
		error = PTR_ERR(fpc1020->class);
	}

	return error;
}


/* -------------------------------------------------------------------- */
static int  fpc1020_create_device(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (FPC1020_MAJOR > 0) {
		fpc1020->devno = MKDEV(FPC1020_MAJOR, fpc1020_device_count++);

		error = register_chrdev_region(fpc1020->devno,
						1,
						FPC1020_DEV_NAME);
	} else {
		error = alloc_chrdev_region(&fpc1020->devno,
					fpc1020_device_count++,
					1,
					FPC1020_DEV_NAME);
	}

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
				"%s: FAILED %d.\n", __func__, error);
		goto out;

	} else {
		dev_info(&fpc1020->spi->dev, "%s: major=%d, minor=%d\n",
						__func__,
						MAJOR(fpc1020->devno),
						MINOR(fpc1020->devno));
	}

	fpc1020->device = device_create(fpc1020->class, NULL, fpc1020->devno,
						NULL, "%s", FPC1020_DEV_NAME);

	if (IS_ERR(fpc1020->device)) {
		dev_err(&fpc1020->spi->dev, "device_create failed.\n");
		error = PTR_ERR(fpc1020->device);
	}
out:
	return error;
}

#if 0
#define	LINK_KOBJ_NAME	"mx_fpc"
static struct kobject *devices_kobj = NULL;
/**
 * mx_create_link - create a sysfs link to an exported virtual node
 *	@target:	object we're pointing to.
 *	@name:		name of the symlink.
 *
 * Set up a symlink from /sys/class/input/inputX to
 * /sys/devices/mx_tsp node.
 * rc = mx_create_link(&input_dev->dev.kobj, LINK_KOBJ_NAME);
 *
 * Returns zero on success, else an error.
 */
static int mx_create_link(struct kobject *target, const char *name)
{
	int rc = 0;

	struct device *mx_tsp = NULL;
	struct kset *pdevices_kset;

	mx_tsp = kzalloc(sizeof(*mx_tsp), GFP_KERNEL);
	if (!mx_tsp){
		rc = -ENOMEM;
		return rc;
	}

	device_initialize(mx_tsp);
	pdevices_kset = mx_tsp->kobj.kset;
	devices_kobj = &pdevices_kset->kobj;
	kfree(mx_tsp);

	if( !devices_kobj )
	{
		rc = -EINVAL;
		goto err_exit;
	}

	rc = sysfs_create_link(devices_kobj,target, name);
	if(rc < 0)
	{
		pr_err("sysfs_create_link failed.\n");
		goto err_exit;
	}

	return rc;

err_exit:
	devices_kobj = NULL;
	pr_err("mx_create_link failed %d \n",rc);
	return rc;
}

static void mx_remove_link(const char *name)
{
 	if( devices_kobj )
	{
		sysfs_remove_link(devices_kobj, name);
		devices_kobj = NULL;
	}
}
#endif

/* -------------------------------------------------------------------- */
static int fpc1020_manage_sysfs(fpc1020_data_t *fpc1020,
				struct spi_device *spi, bool create)
{
	int error = 0;

	if (create) {
		dev_dbg(&fpc1020->spi->dev, "%s create\n", __func__);

		error = sysfs_create_group(&spi->dev.kobj,
					&fpc1020_setup_attr_group);

		if (error) {
			dev_err(&fpc1020->spi->dev,
				"sysf_create_group failed.\n");
			return error;
		}

		error = sysfs_create_group(&spi->dev.kobj,
					&fpc1020_diag_attr_group);

		if (error) {
			sysfs_remove_group(&spi->dev.kobj,
					&fpc1020_setup_attr_group);

			dev_err(&fpc1020->spi->dev,
				"sysf_create_group failed.\n");

			return error;
		}

		//meizu_sysfslink_register_n(&spi->dev,LINK_KOBJ_NAME);
		//mx_create_link(&spi->dev.kobj,LINK_KOBJ_NAME);

	} else {
		dev_dbg(&fpc1020->spi->dev, "%s remove\n", __func__);

		//meizu_sysfslink_unregister(&spi->dev);
		//mx_remove_link(LINK_KOBJ_NAME);

		sysfs_remove_group(&spi->dev.kobj, &fpc1020_setup_attr_group);
		sysfs_remove_group(&spi->dev.kobj, &fpc1020_diag_attr_group);
	}

	return error;
}


/* -------------------------------------------------------------------- */
irqreturn_t fpc1020_interrupt(int irq, void *_fpc1020)
{
	fpc1020_data_t *fpc1020 = _fpc1020;
	int io = 0;
	int cnt = 5;

	dev_dbg(&fpc1020->spi->dev, "%s\n",__func__);
	do
	{
		io = __gpio_get_value(fpc1020->irq_gpio);
		udelay(500);
	}while( io != __gpio_get_value(fpc1020->irq_gpio) && (cnt --) );

	if( cnt <= 0 )
		return IRQ_NONE;

	dev_dbg(&fpc1020->spi->dev, "%s:%d\n",__func__,cnt);

	if ( io ) {
#ifdef FPC_IRQF_TRIGGER_HIGH_USED
		fpc1020_irq_disable(fpc1020);
#endif
		fpc1020->interrupt_done = true;
		wake_up_interruptible(&fpc1020->wq_irq_return);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_show_attr_setup(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
	struct fpc1020_attribute *fpc_attr;
	int val = -1;
	int mux;

	fpc_attr = container_of(attr, struct fpc1020_attribute, attr);

	mux = fpc1020->setup.capture_settings_mux;

	if (fpc_attr->offset == offsetof(fpc1020_setup_t, adc_gain))
		val = fpc1020->setup.adc_gain[mux];

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, adc_shift))
		val = fpc1020->setup.adc_shift[mux];

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, pxl_ctrl))
		val = fpc1020->setup.pxl_ctrl[mux];

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_mode))
		val = fpc1020->setup.capture_mode;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_count))
		val = fpc1020->setup.capture_count;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_settings_mux))
		val = fpc1020->setup.capture_settings_mux;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_row_start))
		val = fpc1020->setup.capture_row_start;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_row_count))
		val = fpc1020->setup.capture_row_count;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_col_start))
		val = fpc1020->setup.capture_col_start;

	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, capture_col_groups))
		val = fpc1020->setup.capture_col_groups;
#ifdef DEBUG_INPUT_MODE
	else if (fpc_attr->offset == offsetof(fpc1020_setup_t, input_mode))
		val = fpc1020->setup.input_mode;
#endif
	#if FPC_SPI_DEBUG
	fpc1020_check_hw_id(fpc1020);
	#endif
	if (val >= 0)
		return scnprintf(buf, PAGE_SIZE, "%i\n", val);

	return -ENOENT;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_store_attr_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
	u64 val;
	int error = kstrtou64(buf, 0, &val);
	int mux;
	int column_groups = fpc1020->chip.pixel_columns / fpc1020->chip.adc_group_size;

	struct fpc1020_attribute *fpc_attr;
	fpc_attr = container_of(attr, struct fpc1020_attribute, attr);

	mux = fpc1020->setup.capture_settings_mux;
	dev_info(&fpc1020->spi->dev, "%s\n", __func__);
	if (!error) {
		if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, adc_gain)) {

			fpc1020->setup.adc_gain[mux] = (u8)val;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, adc_shift)) {

			fpc1020->setup.adc_shift[mux] = (u8)val;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, pxl_ctrl)) {

			fpc1020->setup.pxl_ctrl[mux] = (u16)val;

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_mode)) {

			fpc1020->setup.capture_mode =
					(fpc1020_capture_mode_t)val;
			dev_info(&fpc1020->spi->dev, "%s, capture_mode = %llu\n", __func__, val);

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_count)) {

			if (fpc1020_check_in_range_u64
				(val, 1, FPC1020_BUFFER_MAX_IMAGES)) {

				fpc1020->setup.capture_count = (u8)val;
			} else {
				dev_info(&fpc1020->spi->dev, "%s, %d\n", __func__, __LINE__);
				return -EINVAL;
			}

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_settings_mux)) {

			if (fpc1020_check_in_range_u64
				(val, 0, (FPC1020_BUFFER_MAX_IMAGES - 1))) {

				fpc1020->setup.capture_settings_mux = (u8)val;
			} else {
				dev_info(&fpc1020->spi->dev, "%s, %d\n", __func__, __LINE__);
				return -EINVAL;
			}

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_row_start)) {

			if (fpc1020_check_in_range_u64
				(val, 0, (fpc1020->chip.pixel_rows - 1))) {

				fpc1020->setup.capture_row_start = (u8)val;
			} else {
				dev_info(&fpc1020->spi->dev, "%s, %d\n", __func__, __LINE__);
				return -EINVAL;
			}

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_row_count)) {

			if (fpc1020_check_in_range_u64
				(val, 1, fpc1020->chip.pixel_rows)) {

				fpc1020->setup.capture_row_count = (u8)val;
			} else {
				dev_info(&fpc1020->spi->dev, "%s, %d\n", __func__, __LINE__);
				return -EINVAL;
			}

		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_col_start)) {

			if (fpc1020_check_in_range_u64
				(val, 0, (column_groups - 1))) {

				fpc1020->setup.capture_col_start = (u8)val;
			} else {
				dev_info(&fpc1020->spi->dev, "%s, %d\n", __func__, __LINE__);
				return -EINVAL;
			}


		} else if (fpc_attr->offset ==
			offsetof(fpc1020_setup_t, capture_col_groups)) {

			if (fpc1020_check_in_range_u64
				(val, 1, column_groups)) {

				fpc1020->setup.capture_col_groups = (u8)val;
			} else {
				dev_info(&fpc1020->spi->dev, "%s, %d\n", __func__, __LINE__);
				return -EINVAL;
			}

#ifdef DEBUG_INPUT_MODE
		} else if (fpc_attr->offset ==
                        offsetof(fpc1020_setup_t, input_mode)) {

                        fpc1020->setup.input_mode =
                                        (u8)val;
#endif

		} else {
			dev_info(&fpc1020->spi->dev, "%s, %d\n", __func__, __LINE__);
			return -ENOENT;
		}

		return strnlen(buf, count);
	}
	if (error < 0) {
		dev_err(&fpc1020->spi->dev, "%s, error: %d\n", __func__, error);
	} else {
		dev_info(&fpc1020->spi->dev, "%s success\n", __func__);
	}
	return error;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_show_attr_diag(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	fpc1020_data_t *fpc1020;
	struct fpc1020_attribute *fpc_attr;
	u64 val = 0;
	int error = 0;
	bool is_buffer = false;
	u8 u8_buffer[FPC1020_REG_MAX_SIZE];
	char hex_string[sizeof("0x") + (FPC1020_REG_MAX_SIZE * 2)];

	fpc1020 = dev_get_drvdata(dev);

	fpc_attr = container_of(attr, struct fpc1020_attribute, attr);

	if (fpc_attr->offset == offsetof(fpc1020_diag_t, chip_id)) {
		return scnprintf(buf,
				PAGE_SIZE,
				"%s rev.%d\n",
				fpc1020_hw_id_text(fpc1020),
				fpc1020->chip.revision);
	}
#ifdef FPS_TEST
	if(fpc_attr->offset == offsetof(fpc1020_diag_t, capture_time)) {
                return scnprintf(buf, PAGE_SIZE, "%i\n",
                                 fpc1020->diag.capture_time);
        }
	if(fpc_attr->offset == offsetof(fpc1020_diag_t,
			frames_captured)) {
		return scnprintf(buf, PAGE_SIZE, "%i\n",
                                 fpc1020->diag.frames_captured);
	}

	if(fpc_attr->offset == offsetof(fpc1020_diag_t,
				frames_stored)) {

		return scnprintf(buf, PAGE_SIZE, "%i\n",
			fpc1020->diag.frames_stored);
	}
#endif
	else if (fpc_attr->offset == offsetof(fpc1020_diag_t, reset)) {
		val = (u64)fpc1020_reset(fpc1020);
	}
	else if (fpc_attr->offset == offsetof(fpc1020_diag_t, mode)) {
		val = (u64)g_fpc_run_mode;
	}
	else if (fpc_attr->offset == offsetof(fpc1020_diag_t, state)) {
		return scnprintf(buf, PAGE_SIZE, "input.enabled = %d,capture.state = %d\n",
			fpc1020->input.enabled,fpc1020->capture.state);
	}
	else if (fpc_attr->offset == offsetof(fpc1020_diag_t, selftest)) {
		val = (u64)fpc1020_selftest_short(fpc1020);
	}

	else if (fpc_attr->offset == offsetof(fpc1020_diag_t, spi_register)) {
		val = (int)fpc1020->diag.spi_register;
	}

	else if (fpc_attr->offset == offsetof(fpc1020_diag_t, spi_regsize)) {
		val = (int)fpc1020->diag.spi_regsize;
	}

	else if (fpc_attr->offset == offsetof(fpc1020_diag_t, spi_data)) {

		is_buffer = (fpc1020->diag.spi_regsize > sizeof(val));

		if (!is_buffer) {
			error = fpc1020_spi_debug_value_read(fpc1020, &val);
		} else {
			error = fpc1020_spi_debug_buffer_read(fpc1020,
							u8_buffer,
							sizeof(u8_buffer));
		}
	} else if (fpc_attr->offset == offsetof(fpc1020_diag_t, last_capture_time)) {
		val = (int)fpc1020->diag.last_capture_time;
	}

	if (error >= 0 && !is_buffer) {
		return scnprintf(buf,
				PAGE_SIZE,
				"%lu\n",
				(long unsigned int)val);
	}

	if (error >= 0 && is_buffer) {
		fpc1020_spi_debug_buffer_to_hex_string(hex_string,
						u8_buffer,
						fpc1020->diag.spi_regsize);

		return scnprintf(buf, PAGE_SIZE, "%s\n", hex_string);
	}

	return -ENOTTY;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_store_attr_diag(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
	u64 val;
	int error = 0;

	struct fpc1020_attribute *fpc_attr;
	fpc_attr = container_of(attr, struct fpc1020_attribute, attr);

	if(down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	if (fpc_attr->offset == offsetof(fpc1020_diag_t, spi_register)) {
		error = kstrtou64(buf, 0, &val);

		if (!error) {
			error = fpc1020_spi_debug_select(fpc1020,
							(fpc1020_reg_t)val);
		}
	} else if (fpc_attr->offset == offsetof(fpc1020_diag_t, mode)) {
		int m = 0;
		error = kstrtou64(buf, 0, &val);
		m = (int)val;
		dev_dbg(&fpc1020->spi->dev, "%s mode= %d -> %d\n", __func__,g_fpc_run_mode,m);
		if( g_fpc_run_mode != m )
		{
			switch(m)
			{
				case FPC1020_WORKER_INPUT_MODE:
					fpc1020_start_navigation(fpc1020);
					g_fpc_run_mode = FPC1020_WORKER_INPUT_MODE ;
					break;
				case FPC1020_WORKER_CAPTURE_MODE:
					fpc1020_worker_goto_idle(fpc1020);
					g_fpc_run_mode = FPC1020_WORKER_CAPTURE_MODE ;
					break;
				case FPC1020_WORKER_IDLE_MODE:
					fpc1020_worker_goto_sleep(fpc1020);
					g_fpc_run_mode = FPC1020_WORKER_IDLE_MODE ;
					break;
				default:
					dev_err(&fpc1020->spi->dev, "%s: mode %d not support\n", __func__,m);
					break;
			}
		}
	} else if (fpc_attr->offset == offsetof(fpc1020_diag_t, spi_data)) {
		if (fpc1020->diag.spi_regsize <= sizeof(val)) {
			error = kstrtou64(buf, 0, &val);

			if (!error)
				error = fpc1020_spi_debug_value_write(fpc1020,
									 val);
		} else {
			error = fpc1020_spi_debug_buffer_write(fpc1020,
								buf,
								count);
		}
	} else
		error = -EPERM;

	up(&fpc1020->mutex);

	return (error < 0) ? error : strnlen(buf, count);
}


/* -------------------------------------------------------------------- */
static u8 fpc1020_selftest_short(fpc1020_data_t *fpc1020)
{
	const char *id_str = "selftest,";
	int error = 0;

#ifdef CONFIG_INPUT_FPC1020_NAV
	bool resume_nav = false;
	if(fpc1020->nav.enabled) {
		resume_nav = true;
		fpc1020_worker_goto_idle(fpc1020);
	}
#endif
	fpc1020->diag.selftest = 0;

	error = fpc1020_wake_up(fpc1020);

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"%s wake up fail on entry.\n", id_str);
		goto out;
	}

	error = fpc1020_reset(fpc1020);

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"%s reset fail on entry.\n", id_str);
		goto out;
	}

	error = fpc1020_check_hw_id(fpc1020);

	if (error)
		goto out;

	error = fpc1020_cmd(fpc1020, FPC1020_CMD_CAPTURE_IMAGE, false);

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
			"%s capture command failed.\n", id_str);
		goto out;
	}

	error = __gpio_get_value(fpc1020->irq_gpio) ? 0 : -EIO;

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"%s IRQ not HIGH after capture.\n", id_str);
		goto out;
	}

	error = fpc1020_wait_for_irq(fpc1020, FPC1020_DEFAULT_IRQ_TIMEOUT_MS);

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"%s IRQ-wait after capture failed.\n", id_str);
		goto out;
	}

	error = fpc1020_read_irq(fpc1020, true);

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
			"%s IRQ clear fail\n", id_str);
		goto out;
	} else
		error = 0;

	error = (__gpio_get_value(fpc1020->irq_gpio) == 0) ? 0 : -EIO;

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"%s IRQ not LOW after clear.\n", id_str);
		goto out;
	}

	error = fpc1020_reset(fpc1020);

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"%s reset fail on exit.\n", id_str);
		goto out;
	}

	error = fpc1020_read_status_reg(fpc1020);

	if (error != FPC1020_STATUS_REG_RESET_VALUE)  {
		dev_err(&fpc1020->spi->dev,
			 "%s status check fail on exit.\n", id_str);
		goto out;
	}

	error = 0;

out:
	fpc1020->diag.selftest = (error == 0) ? 1 : 0;

	dev_info(&fpc1020->spi->dev, "%s %s\n", id_str,
				(fpc1020->diag.selftest) ? "PASS" : "FAIL");

#ifdef CONFIG_INPUT_FPC1020_NAV
	if (resume_nav && fpc1020->diag.selftest)
		fpc1020_start_navigation(fpc1020);
#endif

	return fpc1020->diag.selftest;
};


/* -------------------------------------------------------------------- */
static int fpc1020_start_capture(fpc1020_data_t *fpc1020)
{
	fpc1020_capture_mode_t mode = fpc1020->setup.capture_mode;
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s mode= %d\n", __func__, mode);

	/* Mode check (and pre-conditions if required) ? */
	switch (mode) {
	case FPC1020_MODE_WAIT_AND_CAPTURE:
	case FPC1020_MODE_SINGLE_CAPTURE:
	case FPC1020_MODE_CHECKERBOARD_TEST_NORM:
	case FPC1020_MODE_CHECKERBOARD_TEST_INV:
	case FPC1020_MODE_BOARD_TEST_ONE:
	case FPC1020_MODE_BOARD_TEST_ZERO:
	case FPC1020_MODE_WAIT_FINGER_DOWN:
	case FPC1020_MODE_WAIT_FINGER_UP:
	case FPC1020_MODE_SINGLE_CAPTURE_CAL:
		break;

	case FPC1020_MODE_IDLE:
	default:
		error = -EINVAL;
		break;
	}

	fpc1020->capture.current_mode = (error >= 0) ? mode : FPC1020_MODE_IDLE;

	fpc1020->capture.state = FPC1020_CAPTURE_STATE_STARTED;
	fpc1020->capture.available_bytes  = 0;
	fpc1020->capture.read_offset = 0;
	fpc1020->capture.read_pending_eof = false;

	fpc1020_new_job(fpc1020, FPC1020_WORKER_CAPTURE_MODE);

	return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_worker_goto_idle(fpc1020_data_t *fpc1020)
{
	//const int wait_idle_us = 100;

	if (down_trylock(&fpc1020->worker.sem_idle)) {
		dev_dbg(&fpc1020->spi->dev, "%s, stop_request\n", __func__);

		fpc1020->worker.stop_request = true;
		fpc1020->worker.req_mode = FPC1020_WORKER_IDLE_MODE;

		while (down_trylock(&fpc1020->worker.sem_idle))	{

			fpc1020->worker.stop_request = true;
			fpc1020->worker.req_mode = FPC1020_WORKER_IDLE_MODE;

			msleep(1);
		}
		dev_dbg(&fpc1020->spi->dev, "%s, is idle\n", __func__);
		up(&fpc1020->worker.sem_idle);

	} else {
		dev_dbg(&fpc1020->spi->dev, "%s, already idle\n", __func__);
		up(&fpc1020->worker.sem_idle);
	}

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1020_worker_goto_sleep(fpc1020_data_t *fpc1020)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);
	fpc1020_worker_goto_idle(fpc1020);
	fpc1020_wake_up(fpc1020);
	return fpc1020_sleep(fpc1020, true);
}


/* -------------------------------------------------------------------- */
static int fpc1020_new_job(fpc1020_data_t *fpc1020, int new_job)
{
	dev_dbg(&fpc1020->spi->dev, "%s %d\n", __func__, new_job);

	fpc1020_worker_goto_idle(fpc1020);

	fpc1020->worker.req_mode = new_job;
	fpc1020->worker.stop_request = false;

	wake_up_interruptible(&fpc1020->worker.wq_wait_job);

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc1020_worker_function(void *_fpc1020)
{
	fpc1020_data_t *fpc1020 = _fpc1020;

	while (!kthread_should_stop()) {

		up(&fpc1020->worker.sem_idle);

		wait_event_interruptible(fpc1020->worker.wq_wait_job,
			fpc1020->worker.req_mode != FPC1020_WORKER_IDLE_MODE);

		down(&fpc1020->worker.sem_idle);

		switch (fpc1020->worker.req_mode) {
		case FPC1020_WORKER_CAPTURE_MODE:
			fpc1020->capture.state = FPC1020_CAPTURE_STATE_PENDING;
			fpc1020_capture_task(fpc1020);
			break;

#ifdef CONFIG_INPUT_FPC1020_NAV
		case FPC1020_WORKER_INPUT_MODE:
			if (fpc1020_capture_deferred_task(fpc1020) != -EINTR) {
				fpc1020_input_enable(fpc1020, true);
				fpc1020_input_task(fpc1020);
			}
			break;
#endif

		case FPC1020_WORKER_IDLE_MODE:
		case FPC1020_WORKER_EXIT:
		default:
			break;
		}

		if (fpc1020->worker.req_mode != FPC1020_WORKER_EXIT)
			fpc1020->worker.req_mode = FPC1020_WORKER_IDLE_MODE;
	}

	return 0;
}


/* -------------------------------------------------------------------- */
/* SPI debug interface, implementation					*/
/* -------------------------------------------------------------------- */
static int fpc1020_spi_debug_select(fpc1020_data_t *fpc1020, fpc1020_reg_t reg)
{
	u8 size = FPC1020_REG_SIZE(reg);

	if (size) {
		fpc1020->diag.spi_register = reg;
		fpc1020->diag.spi_regsize  = size;

		dev_dbg(&fpc1020->spi->dev, "%s : selected %d (%d byte(s))\n",
						 __func__
						, fpc1020->diag.spi_register
						, fpc1020->diag.spi_regsize);
		return 0;
	} else {
		dev_dbg(&fpc1020->spi->dev,
			"%s : reg %d not available\n", __func__, reg);

		return -ENOENT;
	}
}


/* -------------------------------------------------------------------- */
static int fpc1020_spi_debug_value_write(fpc1020_data_t *fpc1020, u64 data)
{
	int error = 0;
	fpc1020_reg_access_t reg;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	FPC1020_MK_REG_WRITE_BYTES(reg,
				fpc1020->diag.spi_register,
				fpc1020->diag.spi_regsize,
				(u8 *)&data);

	error = fpc1020_reg_access(fpc1020, &reg);

	return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_spi_debug_buffer_write(fpc1020_data_t *fpc1020,
						const char *data, size_t count)
{
	int error = 0;
	fpc1020_reg_access_t reg;
	u8 u8_buffer[FPC1020_REG_MAX_SIZE];

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	error = fpc1020_spi_debug_hex_string_to_buffer(u8_buffer,
						sizeof(u8_buffer),
						data,
						count);

	if (error < 0)
		return error;

	FPC1020_MK_REG_WRITE_BYTES(reg,
				fpc1020->diag.spi_register,
				fpc1020->diag.spi_regsize,
				u8_buffer);

	error = fpc1020_reg_access(fpc1020, &reg);

	return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_spi_debug_value_read(fpc1020_data_t *fpc1020, u64 *data)
{
	int error = 0;
	fpc1020_reg_access_t reg;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	*data = 0;

	FPC1020_MK_REG_READ_BYTES(reg,
				fpc1020->diag.spi_register,
				fpc1020->diag.spi_regsize,
				(u8 *)data);

	error = fpc1020_reg_access(fpc1020, &reg);

	return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_spi_debug_buffer_read(fpc1020_data_t *fpc1020,
						u8 *data, size_t max_count)
{
	int error = 0;
	fpc1020_reg_access_t reg;

	if (max_count < fpc1020->diag.spi_regsize)
		return -ENOMEM;

	FPC1020_MK_REG_READ_BYTES(reg,
				fpc1020->diag.spi_register,
				fpc1020->diag.spi_regsize,
				data);

	error = fpc1020_reg_access(fpc1020, &reg);

	return error;
}


/* -------------------------------------------------------------------- */
static void fpc1020_spi_debug_buffer_to_hex_string(char *string,
						u8 *buffer,
						size_t bytes)
{
	int count = bytes;
	int pos = 0;
	int src = (target_little_endian) ? (bytes - 1) : 0;
	u8 v1, v2;

	string[pos++] = '0';
	string[pos++] = 'x';

	while (count) {
		v1 = buffer[src] >> 4;
		v2 = buffer[src] & 0x0f;

		string[pos++] = (v1 >= 0x0a) ? ('a' - 0x0a + v1) : ('0' + v1);
		string[pos++] = (v2 >= 0x0a) ? ('a' - 0x0a + v2) : ('0' + v2);

		src += (target_little_endian) ? -1 : 1;

		count--;
	}

	string[pos] = '\0';
}


/* -------------------------------------------------------------------- */
static u8 fpc1020_char_to_u8(char in_char)
{
	if ((in_char >= 'A') && (in_char <= 'F'))
		return (u8)(in_char - 'A' + 0xa);

	if ((in_char >= 'a') && (in_char <= 'f'))
		return (u8)(in_char - 'a' + 0xa);

	if ((in_char >= '0') && (in_char <= '9'))
		return (u8)(in_char - '0');

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc1020_spi_debug_hex_string_to_buffer(u8 *buffer,
						size_t buf_size,
						const char *string,
						size_t chars)
{
	int bytes = 0;
	int count;
	int dst = (target_little_endian) ? 0 : (buf_size - 1);
	int pos;
	u8 v1, v2;

	if (string[1] != 'x' && string[1] != 'X')
		return -EINVAL;

	if (string[0] != '0')
		return -EINVAL;

	if (chars < sizeof("0x1"))
		return -EINVAL;

	count = buf_size;
	while (count)
		buffer[--count] = 0;

	count = chars - sizeof("0x");

	bytes = ((count % 2) == 0) ? (count / 2) : (count / 2) + 1;

	if (bytes > buf_size)
		return -EINVAL;

	pos = chars - 2;

	while (pos >= 2) {
		v1 = fpc1020_char_to_u8(string[pos--]);
		v2 = (pos >= 2) ? fpc1020_char_to_u8(string[pos--]) : 0;

		buffer[dst] = (v2 << 4) | v1;

		dst += (target_little_endian) ? 1 : -1;
	}
	return bytes;
}


/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_start_navigation(fpc1020_data_t *fpc1020)
{
	int errror = 0;

	if(g_fpc_tee_used == 1 )
	{
		return 0;
	}
	if(g_fpc_suspend)
	{
		g_fpc_input_mode_resume = 1;
		return errror;
	}

	errror = fpc1020_new_job(fpc1020, FPC1020_WORKER_INPUT_MODE);
	return errror;
}
#endif


/* -------------------------------------------------------------------- */
