
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include "sensor-ps-statistics.h"

struct sensor_ps_statistics {
	struct sensor_ps_dev *ps_dev;

	uint32_t adc_list_index;
	time_t   adc_list_timestamp;
	uint16_t adc_list_buf[1024];

	uint32_t adc_distribution[256];

	struct workqueue_struct * ps_statis_wq;
	struct delayed_work    ps_statis_dwork;
};
static struct sensor_ps_statistics *g_ps_statis;

void sensor_ps_statistics_dump_distribution(void)
{
	int i = 0;
	uint32_t *adc = g_ps_statis->adc_distribution;

	printk(KERN_EMERG "[ps statis] =================\n");
	for (i=0; i<256; i+=16) {
		printk(KERN_EMERG "[ps statis] %d:%d, %d:%d, %d:%d, %d:%d, "
				"%d:%d, %d:%d, %d:%d, %d:%d, "
				"%d:%d, %d:%d, %d:%d, %d:%d, "
				"%d:%d, %d:%d, %d:%d, %d:%d\n",
		i, adc[i], i+1, adc[i+1], i+2, adc[i+2], i+3, adc[i+3],
		i+4,adc[i+4], i+5,adc[i+5], i+6,adc[i+6], i+7,adc[i+7],
		i+8,adc[i+8], i+9,adc[i+9], i+10,adc[i+10], i+11,adc[i+11],
		i+12,adc[i+12], i+13,adc[i+13], i+14,adc[i+14], i+15,adc[i+15]);
	}

	printk(KERN_EMERG "[ps statis] =================\n");
}

void sensor_ps_statistics_dump_list(void)
{
	int i = 0;
	uint16_t *adc = g_ps_statis->adc_list_buf;

	printk(KERN_EMERG "[ps statis] =================\n");
	for (i=0; i<1024; i+=16) {
		printk(KERN_EMERG "[ps statis] %d:%d, %d:%d, %d:%d, %d:%d, "
				"%d:%d, %d:%d, %d:%d, %d:%d, "
				"%d:%d, %d:%d, %d:%d, %d:%d, "
				"%d:%d, %d:%d, %d:%d, %d:%d\n",
		i, adc[i], i+1, adc[i+1], i+2, adc[i+2], i+3, adc[i+3],
		i+4,adc[i+4], i+5,adc[i+5], i+6,adc[i+6], i+7,adc[i+7],
		i+8,adc[i+8], i+9,adc[i+9], i+10,adc[i+10], i+11,adc[i+11],
		i+12,adc[i+12], i+13,adc[i+13], i+14,adc[i+14], i+15,adc[i+15]);
	}

	printk(KERN_EMERG "[ps statis] =================\n");
}

void sensor_ps_statistics_dump(void)
{
	sensor_ps_statistics_dump_distribution();
	sensor_ps_statistics_dump_list();
	return;
}

static void sensor_ps_statistics_work_handler(struct work_struct *work)
{
	int index;
	uint16_t adc = 0;
	struct sensor_ps_statistics *ps_statis =
			container_of(work, struct sensor_ps_statistics,
						ps_statis_dwork.work);

	ps_statis->ps_dev->ops->read_raw_adc_data(ps_statis->ps_dev, &adc);

	index = adc > 255 ? 255 : adc;
	ps_statis->adc_distribution[index]++;
	if (ps_statis->adc_distribution[index] == U32_MAX) {
		printk(KERN_EMERG "[ps statis] reach max count\n");
		sensor_ps_statistics_dump_distribution();
		memset(ps_statis->adc_distribution, 0,
				sizeof(ps_statis->adc_distribution));
	}

	index = ps_statis->adc_list_index % 1024;
	ps_statis->adc_list_buf[index] = adc;
	ps_statis->adc_list_index++;

	if (index == 0 && ps_statis->adc_list_index > 1)
		sensor_ps_statistics_dump();

	queue_delayed_work(ps_statis->ps_statis_wq,
				&ps_statis->ps_statis_dwork,
				msecs_to_jiffies(500));
}

int sensor_ps_statistics_init(struct sensor_ps_dev *ps_dev)
{
	int ret = -EINVAL;
	struct sensor_ps_statistics *ps_statis;

	if (!ps_dev)
		return -EINVAL;

	ps_statis = kzalloc(sizeof(struct sensor_ps_statistics), GFP_KERNEL);
	if (ps_statis == NULL) {
		ret = -ENOMEM;
		dev_err(&ps_dev->sdev->dev, "kzalloc failed for statis");
		goto err_kzalloc;
	}

	ps_statis->ps_statis_wq = create_singlethread_workqueue("ps-statis-wq");
	if (!ps_statis->ps_statis_wq) {
		ret = -ENOMEM;
		dev_err(&ps_dev->sdev->dev,
			"Cannot create work thread for ps statistics\n");
		goto err_workqueue;
	}

	INIT_DELAYED_WORK(&ps_statis->ps_statis_dwork,
				sensor_ps_statistics_work_handler);

	ps_statis->ps_dev = ps_dev;
	g_ps_statis = ps_statis;

	dev_info(&ps_dev->sdev->dev, "%s success\n", __func__);
	return 0;

err_workqueue:
	kfree(ps_statis);
err_kzalloc:
	dev_err(&ps_dev->sdev->dev, "%s failed\n", __func__);
	return ret;
}

void sensor_ps_statistics_start(void)
{
	if (!g_ps_statis)
		return;

	printk(KERN_EMERG "[ps statis] start\n");
	queue_delayed_work(g_ps_statis->ps_statis_wq,
				&g_ps_statis->ps_statis_dwork,
				msecs_to_jiffies(500));
	return;
}

void sensor_ps_statistics_stop(void)
{
	if (!g_ps_statis)
		return;

	printk(KERN_EMERG "[ps statis] stop\n");
	cancel_delayed_work_sync(&g_ps_statis->ps_statis_dwork);
	return;
}

MODULE_AUTHOR("Zhang Jiajing <zhangjiajing@meizu.com>");
MODULE_DESCRIPTION("Sensor ps statistics driver");
MODULE_LICENSE("GPL");
