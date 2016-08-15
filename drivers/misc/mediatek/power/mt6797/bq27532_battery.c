/*
 * bq27532 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27411-g1
 * http://www.ti.com/product/bq27421-g1
 * http://www.ti.com/product/bq27425-g1
 * http://www.ti.com/product/bq27441-g1
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#define CONFIG_MTK_I2C_EXTENSION
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <mach/irqs.h>
#include <linux/meizu-sys.h>
#include <linux/fs.h>
#include <linux/sched/rt.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/platform_device.h>
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_rtc.h>
#include <mt-plat/battery_common.h>
#include <linux/power/bq27532_battery.h>

#define REGS_DUMP_TIMES	(5*HZ)

struct bq27532_device_info *di_info = NULL;

static unsigned int poll_interval = 360;
static int battery_exist = 1;
static int init_temp = 15;

#define R_CHARGER_1 330
#define R_CHARGER_2 39

module_param(poll_interval, uint, 0664);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

/* i2c specific code */

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_MUTEX(battery_mutex);

static int BQ27532_read_i2c(struct bq27532_device_info *di, u8 reg)
{
    char     cmd_buf[2]={0x00};
    int     readData = 0;
    int      ret=0;
    struct i2c_client *new_client = di->client;

    mutex_lock(&di->bat_i2c_access);
 
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
    new_client->timing = 50;

    ret = i2c_master_send(new_client, &reg, 1);
    ret = i2c_master_recv(new_client, cmd_buf, 2);
    if (ret < 0)
    {
        new_client->ext_flag=0;
        mutex_unlock(&di->bat_i2c_access);
        return ret;
    }

    readData = cmd_buf[1] << 8 | cmd_buf[0];
    new_client->ext_flag=0;

    mutex_unlock(&di->bat_i2c_access);
    return readData;
}

static int BQ27532_write_i2c(struct bq27532_device_info *di, u8 reg, int value)
{
    char    write_data[3] = {0};
    int     ret=0;
    struct i2c_client *new_client = di->client;

    mutex_lock(&di->bat_i2c_access);

    write_data[0] = reg;
    write_data[1] = (value & 0xff00) >> 8;
    write_data[2] = value & 0x00ff;

    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    new_client->timing = 50;

    ret = i2c_master_send(new_client, write_data, 3);
    if (ret < 0)
    {
        new_client->ext_flag=0;
        mutex_unlock(&di->bat_i2c_access);
        return ret;
    }

    new_client->ext_flag=0;
    mutex_unlock(&di->bat_i2c_access);
    return 0;
}

static int bq27532_battery_reset(struct bq27532_device_info *di)
{
	int val = 0;
	dev_info(di->dev, "Gas Gauge Reset\n");

	BQ27532_write_i2c(di, BQ27532_REG_CTRL, RESET_SUBCMD);

	msleep(10);

	i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0x0000);
	val = BQ27532_read_i2c(di, BQ27532_REG_CTRL);

	return val;
}

static int bq27532_battery_read_charge(struct bq27532_device_info *di, unsigned char reg)
{
	int charge;

	charge = BQ27532_read_i2c(di, reg);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	charge *= 1000;

	return charge;
}

static inline int bq27532_battery_read_nac(struct bq27532_device_info *di)
{

	return bq27532_battery_read_charge(di, BQ27532_REG_NAC);
}

static inline int bq27532_battery_read_fcc(struct bq27532_device_info *di)
{
	return bq27532_battery_read_charge(di, BQ27532_REG_FCC);
}

 int bq27532_battery_read_cyct(struct bq27532_device_info *di)
{
	int cyct;

	cyct = BQ27532_read_i2c(di, BQ27532_REG_CYCT);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

static int overtemperature(struct bq27532_device_info *di, u16 flags)
{
	return flags & BQ27XXX_FLAG_OTC;
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
int bq27532_battery_read_health(struct bq27532_device_info *di)
{
	int tval;

	tval = BQ27532_read_i2c(di, BQ27532_REG_FLAGS);
	if (tval < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", tval);
		return tval;
	}

	if (tval & BQ27XXX_FLAG_SOCF)
		tval = POWER_SUPPLY_HEALTH_DEAD;
	else if (overtemperature(di, tval))
		tval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		tval = POWER_SUPPLY_HEALTH_GOOD;

	return tval;
}

void copy_to_dm_buf_big_endian(struct bq27532_device_info *di,
	unsigned char *buf, unsigned char offset, unsigned char sz, u32 val)
{
	dev_dbg(di->dev, "%s: offset %d sz %d val %d\n",
		__func__, offset, sz, val);

	switch (sz) {
	case 1:
		buf[offset] = (unsigned char) val;
		break;
	case 2:
		put_unaligned_be16((u16) val, &buf[offset]);
		break;
	case 4:
		put_unaligned_be32(val, &buf[offset]);
		break;
	default:
		dev_err(di->dev, "%s: bad size for dm parameter - %d",
			__func__, sz);
		break;
	}
}

signed int bq27532_battery_current(void)
{
	signed int curr;

	if (!battery_exist)
		return 1000;

	curr = BQ27532_read_i2c(di_info, BQ27532_REG_AVG_CURR);
	if (curr < 0) {
		dev_err(di_info->dev, "error reading current\n");
		return curr;
	}

	return curr;
}

signed int bq27532_battery_current_now(void)
{
	signed int now_curr;

	if (!battery_exist)
		return 1000;

	now_curr = BQ27532_read_i2c(di_info, BQ27532_REG_NOW_CURR);
	if (now_curr < 0) {
		dev_err(di_info->dev, "error reading current\n");
		return now_curr;
	}

	return now_curr;
}

unsigned int bq27532_battery_voltage(void)
{
	unsigned int volt;

	if (!battery_exist)
		return 3800;

	volt = BQ27532_read_i2c(di_info, BQ27532_REG_VOLT);
	if (volt < 0) {
		dev_err(di_info->dev, "error reading voltage\n");
		return volt;
	}

	volt = BQ27532_read_i2c(di_info, BQ27532_REG_VOLT);
	if (volt < 0) {
		dev_err(di_info->dev, "error reading voltage\n");
		return volt;
	}

	return volt;
}


unsigned int bq27532_charger_voltage(void)
{
	unsigned int vbus_value;

	if (!battery_exist)
		return 5001;

            vbus_value = PMIC_IMM_GetOneChannelValue(PMIC_AUX_VCDT_AP, 1, 1);
            vbus_value = (((R_CHARGER_1 + R_CHARGER_2) * 100 * vbus_value) / R_CHARGER_2) / 100;

	if (vbus_value < 0) {
		dev_err(di_info->dev, "error reading voltage\n");
		return vbus_value;
	}

            printk("\n\n\nbq27532_charger_voltage: the bus voltage = %d\n\n\n",vbus_value);

	return vbus_value;
}

#define RETRY_TIMES	3
signed int bq27532_battery_read_temperature(void)
{
	signed int temp;
	int retry = 0;

	if (!battery_exist)
		return 25;

temp_confirm:
	/*NTC Temperature*/
	temp = BQ27532_read_i2c(di_info, BQ27532_REG_TEMP);
	if (temp < 0) {
		dev_err(di_info->dev, "error reading temperature, return default temperature 25 oC\n");
		return 25;
	}
	temp = temp / 10 - 273;
	while ((temp >= 62 || temp <= -22) && (retry < RETRY_TIMES)) {
		printk("%s:temp read error %d, read again\n", __func__, temp);
		retry++;
		msleep(5);
		goto temp_confirm;
	}

	/*解决SOC回升问题*/
	init_temp = get_rtc_spare_fg_value();
	pr_err("bq27532_battery_read_temperature: temp / 10 - 273 = %d init_temp=%d\n", temp,init_temp);

	if (init_temp == 0) {
		set_rtc_spare_fg_value(15);
		init_temp = 15;
	} else if (init_temp > 25) {
		printk("The last temp is negtive\n");
		init_temp = init_temp - 128;
	}
	if ((abs(temp - init_temp) >= 5) && (temp < 25)) {
		if (temp == 0) {
			init_temp = 1;
		} else {
			init_temp = temp;
		}
		set_rtc_spare_fg_value(init_temp);
		pr_err("bq27532_battery_read_temperature : temp = %d init_temp=%d & soc sync to truesoc \n", temp,init_temp);
		i2c_smbus_write_word_data(di_info->client, BQ27532_REG_CTRL, 0x001E);
	}
	pr_err("bq27532_battery_read_temperature: return temp= %d\n", temp);
	return temp;
}

#if 1 // just fix build error
int force_get_tbat(bool update)
{
	int bat_temp;

	bat_temp = bq27532_battery_read_temperature();

	return bat_temp;
}
EXPORT_SYMBOL(force_get_tbat);
#endif

signed int bq27532_battery_read_int_temperature(void)
{
	signed int temp;

	if (!battery_exist)
		return 25;

	temp = BQ27532_read_i2c(di_info, BQ27532_REG_INT_TEMP);
	if (temp < 0) {
		dev_err(di_info->dev, "error reading temperature, return default temperature 25 oC\n");
		return 25;
	}

	temp = temp / 10 - 273;

	return temp;
}

signed int bq27532_battery_read_soc(void)
{
	signed int soc = 0;
	signed int num = 0;

	if (!battery_exist)
		return 19;

	do{
		soc = BQ27532_read_i2c(di_info, BQ27532_REG_SOC);

	}while(soc == 0 && num++ < 5);

	if((soc == 0) && (bq27532_battery_voltage() > 3300))
		soc = 1;

	if (soc < 0)
		dev_dbg(di_info->dev, "error reading relative State-of-Charge\n");

	return soc;
}

signed int bq27532_battery_read_truesoc(void)
{
	signed int truesoc;

	if (!battery_exist)
		return 21;

	truesoc = BQ27532_read_i2c(di_info, 0x74);
	if (truesoc < 0)
		dev_dbg(di_info->dev, "error reading Truesoc\n");

	return truesoc;
}

void dump_battery_status(void);
signed int bq27532_battery_read_fullchargecapacity(void)
{
	signed int fullcapacity, capacity;

	if (!battery_exist)
		return 0;

	fullcapacity = BQ27532_read_i2c(di_info, BQ27532_REG_FULLCHARGECAPCITY);
	capacity = BQ27532_read_i2c(di_info, 0x0E);
	printk("%s:**************fullchargecapacity %d, capacity %d\n", __func__, fullcapacity, capacity);
	if (fullcapacity < 0){
		dev_dbg(di_info->dev, "error reading relative State-of-Charge\n");
		battery_exist = 0;
	}

    dump_battery_status();

	return fullcapacity;
}

signed int bq27532_get_battery_data(int update)
{
	static signed int control, temp, voltage, flag, remg_cap, full_cap;
	static signed int curr, remainingCapacityUnfiltered, fullChargeCapacityUnfiltered;
	static signed int truesoc, fineqpass;
	static signed int int_temp, soc;

	if (!battery_exist)
		return 0;

	if (update)
	{
	control = BQ27532_read_i2c(di_info, BQ27532_REG_CTRL);
	temp = bq27532_battery_read_temperature();
	int_temp = bq27532_battery_read_int_temperature();
	voltage = bq27532_battery_voltage();
	flag = BQ27532_read_i2c(di_info, BQ27532_REG_FLAGS);
	curr = BQ27532_read_i2c(di_info, BQ27532_REG_AVG_CURR);
	full_cap = BQ27532_read_i2c(di_info, BQ27532_REG_FULLCHARGECAPCITY);
	remg_cap = BQ27532_read_i2c(di_info, BQ27532_REG_RM_CAP);
	remainingCapacityUnfiltered = BQ27532_read_i2c(di_info, 0x6C);
	fullChargeCapacityUnfiltered = BQ27532_read_i2c(di_info, 0x70);
	truesoc = BQ27532_read_i2c(di_info, 0x74);
	fineqpass = BQ27532_read_i2c(di_info, 0x24);
	soc = bq27532_battery_read_soc();
	}

#if 1
	printk("control 0x%02x, temp %d, int_temp %d,voltage %d, flag 0x%02x, remg_cap %d, full_cap %d,curr %d, "
			"remainingCapacityUnfiltered %d, fullChargeCapacityUnfiltered %d,truesoc %d, fineqpass %d, soc %d \n",   \
			control, temp, int_temp, voltage, flag, remg_cap, full_cap, curr,                                                   \
			remainingCapacityUnfiltered, fullChargeCapacityUnfiltered, truesoc, fineqpass, soc);
#endif
    return 0;
}

static int bq27532_battery_read_fw_version(struct bq27532_device_info *di)
{
	BQ27532_write_i2c(di, BQ27532_REG_CTRL, FW_VER_SUBCMD);

	msleep(10);

	return BQ27532_read_i2c(di, BQ27532_REG_CTRL);
}

static int bq27532_battery_read_device_type(struct bq27532_device_info *di)
{
//	BQ27532_write_i2c(di, BQ27532_REG_CTRL, DEV_TYPE_SUBCMD);
	i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, DEV_TYPE_SUBCMD);
//	msleep(10);

	return BQ27532_read_i2c(di, BQ27532_REG_CTRL);
}

static int bq27532_battery_read_dataflash_version(struct bq27532_device_info *di)
{
	BQ27532_write_i2c(di, BQ27532_REG_CTRL, DF_VER_SUBCMD);

	msleep(10);

	return BQ27532_read_i2c(di, BQ27532_REG_CTRL);
}

static ssize_t show_firmware_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27532_battery_read_fw_version(di);

	return sprintf(buf, "0x%x\n", ver);
}

static ssize_t show_dataflash_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27532_battery_read_dataflash_version(di);

	return sprintf(buf, "0x%x\n", ver);
}

static ssize_t show_device_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);
	int dev_type;

	dev_type = bq27532_battery_read_device_type(di);
            dump_battery_status();

	return sprintf(buf, "0x%x\n", dev_type);
}

static ssize_t show_reset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);

	bq27532_battery_reset(di);

	return sprintf(buf, "okay\n");
}

void dump_battery_status(void)
{
	printk("[dump battery_status][bq27532]: ");
	bq27532_get_battery_data(1);
}
EXPORT_SYMBOL(dump_battery_status);


static DEVICE_ATTR(fw_version, S_IRUGO, show_firmware_version, NULL);
static DEVICE_ATTR(df_version, S_IRUGO, show_dataflash_version, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, show_device_type, NULL);
static DEVICE_ATTR(reset, S_IRUGO, show_reset, NULL);

static struct attribute *bq27532_attributes[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_df_version.attr,
	&dev_attr_device_type.attr,
	&dev_attr_reset.attr,
	NULL
};

static const struct attribute_group bq27532_attr_group = {
	.attrs = bq27532_attributes,
};

static inline int fuelgauge_info_dump(void)
{
	int ret = 0, err;
	static int i;
	struct file *fp = NULL;
	mm_segment_t pos;
	char buf[16] = {0};
	char time_buf[32] = {0};
	struct timespec ts;
	struct rtc_time tm;
	static kal_bool done = KAL_FALSE;
	int buf1;

	/* get the system current time */
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	sprintf(time_buf, "%d-%02d-%02d %02d:%02d:%02d %s",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour + 8, tm.tm_min, tm.tm_sec, " ");

	pos = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open("/data/fuelgauge_datalog.txt", O_RDWR | O_APPEND | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_info("create file failed******\n");
		err = PTR_ERR(fp);
		if (err != -ENOENT )
			pr_err("%s:open the file failed\n", __func__);
		set_fs(pos);
		return err;
	}

	if (done == KAL_FALSE) {
		/* record the current system time */
		err = fp->f_op->write(fp, time_buf, 21, &fp->f_pos);

		for (i = 0; i <= 0x27; i++) {
			sprintf(buf, "0x%02x %s", i," ");
			err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
		}
		done = KAL_TRUE;
	}

	/* record the current system time */
	err = fp->f_op->write(fp, time_buf, 21, &fp->f_pos);

	for (i = 0; i <= 0x27; i++) {
		buf1 = BQ27532_read_i2c(di_info, i);
		if (i == 0x27) {
			sprintf(buf, "%04x %s", buf1,"\n");
		} else {
			sprintf(buf, "%04x %s", buf1," ");
		}
		err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
	}

	set_fs(pos);
	filp_close(fp, NULL);

	return ret;
}

#if defined(CONFIG_BATTERY_INFO_DUMP_SUPPORT)
static void bq27532_dump_regs(struct work_struct *work)
{
	if (dump_regs) {
		fuelgauge_info_dump();
		schedule_delayed_work_on(0, &di_info->dump_dwork, REGS_DUMP_TIMES);
	}
}
#endif

void bq27532_sync_truesoc(void)
{

	mutex_lock(&di_info->bat_i2c_access);
	i2c_smbus_write_word_data(di_info->client, BQ27532_REG_CTRL, 0x001E);
	msleep(2000);
	mutex_unlock(&di_info->bat_i2c_access);
}

static void bq27532_irq_workfunc(struct work_struct *work)
{
	int soc = 0;

	soc = bq27532_battery_read_soc();
	BMT_status.SOC = soc;
	bq2589x_charge_update_status();
	wake_up_bat();
	printk("%s:soc %d\n", __func__, soc);
}

static irqreturn_t fuel_eint_handler(int irq, void *data)
{
	wake_lock_timeout(&di_info->fg_int, HZ*2);
	schedule_delayed_work(&di_info->irq_dwork,HZ/5);
	return IRQ_HANDLED;
}


static int bq27532_irq_init(void)
{
	struct device_node *node = NULL;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "mediatek,bq27532_int-eint");

	if (node) {
		di_info->fuel_irq = irq_of_parse_and_map(node, 0);

		 ret = request_threaded_irq(di_info->fuel_irq, NULL, fuel_eint_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"bq27532_int-eint",di_info);
		 disable_irq(di_info->fuel_irq);
	} else {
		printk("battery request_irq can not find touch eint device node!.");
	}

	return ret;
}

static int bq27532_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27532_device_info *di;
	int retval = 0;
	int value = 0, value1 = 0;
	signed int fullcapacity;
	int version_flag = 0;
	int length = 0;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	di->dev = &client->dev;
	di->client = client;
	i2c_set_clientdata(client, di);
	mutex_init(&di->bat_i2c_access);
	memset(BMT_status.manufacturer_name, 0, sizeof(BMT_status.manufacturer_name));

	fullcapacity = BQ27532_read_i2c(di, BQ27532_REG_FULLCHARGECAPCITY);
	if (fullcapacity <= 0){
		dev_dbg(di->dev, "error reading relative State-of-Charge, No Battery\n");
		battery_exist = 0;
		BMT_status.bat_exist = battery_exist;
		strcpy(BMT_status.manufacturer_name, "NULL");
		return 0;
	}

	/*Get the battery cell manufacturer CHEM-ID*/
	 /* 0x3603: ATL cell */
	i2c_smbus_write_word_data(client, BQ27532_REG_CTRL, 0x0008);
	value1 = BQ27532_read_i2c(di, BQ27532_REG_CTRL);
	if (value1 == 0x3782) {
		strcpy(BMT_status.manufacturer_name, "ATL");
		BMT_status.battery_id = 0;
	} else if (value1 == 0x3814) {
		strcpy(BMT_status.manufacturer_name, "SDI");
		BMT_status.battery_id = 1;
	} else if (value1 == 0x3828) {
		strcpy(BMT_status.manufacturer_name, "LG");
		BMT_status.battery_id = 2;
	} else {
		strcpy(BMT_status.manufacturer_name, "UNKOWN");
		BMT_status.battery_id = 0;
	}
	length = strlen(BMT_status.manufacturer_name);

	/*Get the battery pack manufacturer BLOCK31*/
	/* 0x02:SCUD , 0x03:SWD, 0x04:DESAY */
	i2c_smbus_write_word_data(client, 0x3F, 0x01);
	value = i2c_smbus_read_byte_data(client, 0x5F);
	if (value == 0x02) {
		strcpy(BMT_status.manufacturer_name+length, "SCUD");
	} else if (value == 0x03) {
		strcpy(BMT_status.manufacturer_name+length, "SWD");
	} else if (value == 0x04) {
		strcpy(BMT_status.manufacturer_name+length, "DESAY");
	} else {
		strcpy(BMT_status.manufacturer_name+length, "UNKOWN");
	}
	BMT_status.bat_exist = battery_exist;

	retval = sysfs_create_group(&client->dev.kobj, &bq27532_attr_group);
	if (retval)
		dev_err(&client->dev, "could not create sysfs files\n");

	retval = meizu_sysfslink_register_name(di->dev, "fuelgauge");
	if (retval < 0)
		dev_err(&client->dev, "could not create meizu sysfs files\n");

	di_info = di;

	wake_lock_init(&di->fg_int, WAKE_LOCK_SUSPEND, "FG_INT suspend wakelock");

#ifdef CONFIG_BATTERY_INFO_DUMP_SUPPORT
        INIT_DELAYED_WORK(&di_info->dump_dwork, bq27532_dump_regs);
        schedule_delayed_work_on(0, &di_info->dump_dwork, REGS_DUMP_TIMES);
#endif

	INIT_DELAYED_WORK(&di->irq_dwork, bq27532_irq_workfunc);
	bq27532_irq_init();

	return 0;
}

static int bq27532_battery_remove(struct i2c_client *client)
{
	struct bq27532_device_info *di = i2c_get_clientdata(client);

	i2c_set_clientdata(di->client, NULL);
	kfree(di);

	return 0;
}

static int bq27532_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bq27532_device_info *di = i2c_get_clientdata(client);

	enable_irq(di->fuel_irq);
	return 0;
}

static int bq27532_resume(struct i2c_client *client)
{
	struct bq27532_device_info *di = i2c_get_clientdata(client);

	disable_irq_nosync(di->fuel_irq);
	return 0;
}

static const struct i2c_device_id bq27532_id[] = {
	{ "bq27532", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bq27532_id);

#ifdef CONFIG_OF
static const struct of_device_id bq27532_of_ids[] = {
	    {.compatible = "mediatek,bq27532"},
		    {},
};
#endif

static struct i2c_driver bq27532_battery_driver = {
	.driver = {
		.name = "bq27532",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
              .of_match_table = bq27532_of_ids,
#endif
	},
	.probe = bq27532_battery_probe,
	.remove = bq27532_battery_remove,
	.suspend = bq27532_suspend,
	.resume = bq27532_resume,
	.id_table = bq27532_id,
};

static inline int __init bq27532_battery_i2c_init(void)
{
        int ret;

#ifndef CONFIG_OF
	i2c_register_board_info(BQ27532_BUSNUM, &i2c_bq27532, 1);
#endif
	ret = i2c_add_driver(&bq27532_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register bq27532 i2c driver\n");

	return ret;
}

static inline void __exit bq27532_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27532_battery_driver);
}

/*
 * Module stuff
 */

static int __init bq27532_battery_init(void)
{
	int ret;

	ret = bq27532_battery_i2c_init();
	if (ret)
		return ret;

	return ret;
}
module_init(bq27532_battery_init);

static void __exit bq27532_battery_exit(void)
{
	bq27532_battery_i2c_exit();
}
module_exit(bq27532_battery_exit);

MODULE_AUTHOR("tangxingyan<tangxingyan@meizu.com>");
MODULE_DESCRIPTION("bq27532 battery monitor driver");
MODULE_LICENSE("GPL");
