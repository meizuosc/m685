
#include "apds9922.h"

unsigned char debug=1;
extern struct apds9921_data *apds_data;
extern int apds9921_enable_ps_sensor(struct i2c_client *client, int val);
extern  int apds9921_dd_set_prx_can(struct i2c_client *client, int prx_can);

static int i2c_write_reg(struct i2c_client *client, u8 *data, int size)
{
    u8 databuf[2];
    int res = 0;
    databuf[0]= data[0];
    databuf[1]= data[1];

//    mutex_lock(&txc_info->i2c_lock);
    res = i2c_master_send(client,databuf,size);
    if(res <= 0)
    {
        INFOR("i2c_master_send function err\n");
//        mutex_unlock(&txc_info->i2c_lock);
        return res;
    }

//    mutex_unlock(&txc_info->i2c_lock);
    return 0;
}

#if 0
static int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *data)
{
    u8 databuf[2];
    int res = 0;
    databuf[0]= reg;

//    mutex_lock(&txc_info->i2c_lock);
    res = i2c_master_send(client,databuf,0x1);
    if(res <= 0)
    {
        INFOR("i2c_master_send function err\n");
//        mutex_unlock(&txc_info->i2c_lock);
        return res;
    }
    res = i2c_master_recv(client,data,0x1);
    if(res <= 0)
    {
        INFOR("i2c_master_recv function err\n");
//        mutex_unlock(&txc_info->i2c_lock);
        return res;
    }

//    mutex_unlock(&txc_info->i2c_lock);
    return 0;
}
#endif

static ssize_t ps_show_reg(struct device *dev, struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);
    int count=0;
    int i=0	;
    int val = 0, reg = 0;
    #define arr_size (18)

    struct ps_reg ps_reg_arr[arr_size] = {
            {0x00,}, //4:sw_rst;1:als_en;0:ps_en
            {0x01,}, //0~2:led current
            {0x02,}, //ps measure pulses
            {0x03,}, //4~3:ps resolution; 2~0:ps measure rate
            {0x04,}, //6~4:als resolution; 2~0:als measure rate
            {0x05,}, //2~0:als gain
            {0x06,}, //7~4:part id; 3~0:revision id
            {0x07,}, //4:als irq stat;3:als data stat; 2:ps logic signal stat;1:ps irq stat;0:ps data stat
            {0x08,}, //ps data LSB
            {0x09,}, //3:overflow; 2~0:ps data MSB
            {0x19,}, //als ps interrupt reg config
            {0x1b,}, //threshold up LSB
            {0x1c,}, //threshold up MSB
            {0x1d,}, //threshold low LSB
            {0x1e,}, //threshold low MSB
            {0x1f,}, // ps offset ,LSB
            {0x20,}, // ps offset ,MSB
            {0x27,}

    };

    mutex_lock(&data->mutex_apds);
    for(i = 0;i <arr_size ;i++)
    {
        reg = ps_reg_arr[i].reg;
        val = i2c_smbus_read_byte_data(client, reg);
        ps_reg_arr[i].data = val & 0xff;

        count+=sprintf(buf+count,"[%.2x] = (%.2x)\n",ps_reg_arr[i].reg,ps_reg_arr[i].data);
    }
    mutex_unlock(&data->mutex_apds);
    return count;
}
static ssize_t ps_store_reg(struct device *dev, struct device_attribute *attr, const char *buf,
                                       size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);
    unsigned int offset = 0;
    unsigned char buffer[2] = {0,};
    mutex_lock(&data->mutex_apds);
    offset = simple_strtol(buf, NULL, 16);
    buffer[0] = (offset&0xff00)>>0x08; // reg
    buffer[1] = (offset&0x00ff);       // value
    INFOR("reg:0x%x, value:0x%x\n",buffer[0], buffer[1]);

    i2c_write_reg(client, buffer, 0x02);
    mutex_unlock(&data->mutex_apds);

    return count;
}

static ssize_t non_wakeup_enable_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->non_wakeup_enable);
}

#if 0
static ssize_t non_wakeup_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
                                       size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    mutex_lock(&data->mutex_apds);
    data->non_wakeup_enable = simple_strtol(buf, NULL, 10);
    mutex_unlock(&data->mutex_apds);

    return count;
}
#endif

static ssize_t wakeup_enable_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->wakeup_enable);
}

#if 0
static ssize_t wakeup_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
                                   size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    mutex_lock(&data->mutex_apds);
    data->wakeup_enable = simple_strtol(buf, NULL, 10);
    mutex_unlock(&data->mutex_apds);

    return count;
}
#endif

static ssize_t gpio_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", gpio_get_value(data->gpio));
}

static ssize_t ps_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    int main_ctrl=0,int_cfg=0,status=0,distance=FAR;


    mutex_lock(&data->mutex_apds);

    INFOR(" enable ps senosr ( %ld)\n", val);

    main_ctrl = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR);
    if (main_ctrl < 0) {
        INFOR("read main ctrl reg failed, enable failed\n");
    }

    int_cfg = i2c_smbus_read_byte_data(client, APDS9921_DD_INT_CFG_ADDR);
    if (int_cfg < 0) {
        INFOR("read irq cfg reg failed, enable failed\n");
    }

    // disable irq first, disable ps
    i2c_smbus_write_byte_data(client, APDS9921_DD_INT_CFG_ADDR, (int_cfg&0xFC));
    i2c_smbus_write_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR, (main_ctrl&~APDS9921_DD_PRX_EN));

    usleep_range(30000,30000);
    apds9921_enable_ps_sensor(client, APDS_ENABLE_PS_CALIBRATION);
    usleep_range(30000,30000);

    status = i2c_smbus_read_byte_data(client, APDS9921_DD_MAIN_STATUS_ADDR);//reg:0x07;
    // 4:als irq stat
    // 2:ps logic signal stat
    // 1:ps irq stat

    if ( (status>>2)&0x01 ) { // bit2,ps logic signal status; 1:close,0:far
        INFOR("ps is close,will report '0', irq status:0x%x\n",status);
        distance = NEAR;
    } else {
        INFOR("ps is far,will report '1', irq status:0x%x\n",status);
        distance = FAR;
    }
    input_report_abs(data->input_dev_ps, ABS_DISTANCE, distance);
    input_sync(data->input_dev_ps);
    data->ps_state_pre = distance;

    // en irq first, en ps
    i2c_smbus_write_byte_data(client, APDS9921_DD_INT_CFG_ADDR, (int_cfg));
    i2c_smbus_write_byte_data(client, APDS9921_DD_MAIN_CTRL_ADDR, (main_ctrl));

    INFOR("ps cali done, pre_dis:%d\n",data->ps_state_pre);

    mutex_unlock(&data->mutex_apds);
    return count;
}

static ssize_t ps_calibvalue_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    INFOR("ps cali result:%s, offset:0x%.4x--%d\n",
          data->ps_cali_result==CALI_FAIL?"cali fail":"cali ok", data->ps_offset,data->ps_offset);
    if( data->ps_cali_result==CALI_OK ) {
        return sprintf(buf, "%d\n", data->ps_offset);
    } else {
        INFOR("ps cali failed\n");
        return sprintf(buf, "%d-%s\n", data->ps_cali_result, data->ps_cali_result==CALI_OK?"cali ok":"cali fail");
    }
}


static ssize_t ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    INFOR("ps offset:0x%.4x\n", data->ps_offset);
    return sprintf(buf, "%.4x\n", data->ps_offset);
}

static ssize_t ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf,
                                   size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    mutex_lock(&data->mutex_apds);
    data->ps_offset = simple_strtol(buf, NULL, 10);
    INFOR("set ps offset:0x%.4x\n", data->ps_offset);
    apds9921_dd_set_prx_can(client, data->ps_offset);
    mutex_unlock(&data->mutex_apds);

    return count;
}


static ssize_t als_batch_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->als_poll_delay);
}

extern  int apds9921_set_als_poll_delay(struct i2c_client *client, unsigned int val);
static ssize_t als_batch_store(struct device *dev, struct device_attribute *attr, const char *buf,
                                   size_t count)
{
    int err = 0, delay = 0;
    struct i2c_client *client = to_i2c_client(dev);

    delay = simple_strtol(buf, NULL, 10);
    INFOR("recv als delay:%d\n", delay);
    /*
     * #define APDS_ALS_POLL_500MS					0	// 2 Hz (500ms)
     * #define APDS_ALS_POLL_200MS				    1	// 10 Hz (200ms)
     * #define APDS_ALS_POLL_100MS				    2	// 10 Hz (100ms)
     * #define APDS_ALS_POLL_50MS				    3	// 10 Hz (50ms)
     * #define APDS_ALS_POLL_25MS					4	// 40 Hz (25ms)
    */

    // set delay as constant
    delay = 250;

    if( delay<50 ) {
        delay = APDS_ALS_POLL_25MS;
    } else if( delay<100 ) {
        delay = APDS_ALS_POLL_50MS;
    } else if( delay<200 ){
        delay = APDS_ALS_POLL_100MS;
    } else if( delay<500 ) {
        delay = APDS_ALS_POLL_200MS;
    } else if( delay<1000 ){
        delay = APDS_ALS_POLL_500MS;
    } else {
        delay = APDS_ALS_POLL_100MS;
    }

    err = apds9921_set_als_poll_delay(client, delay);
    if( err<0 ) {
        INFOR("set als delay:%d failed\n", delay);
    } else {
        INFOR("set als delay:%d ok\n", delay);
    }

    return count;
}


extern  int apds9921_enable_als_sensor(struct i2c_client *client, int val);
static ssize_t als_calibration_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);


    return sprintf(buf, "%d-%s\n", data->als_cali_result, data->als_cali_result==CALI_OK?"cali ok":"cali fail");
}
static ssize_t als_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf,
                            size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    apds9921_enable_als_sensor(client, APDS_DISABLE_ALS);
    usleep_range(50000,50000);
    apds9921_enable_als_sensor(client, APDS_ENABLE_ALS_CALIBRATION);
    usleep_range(50000,50000);


    return count;
}

static ssize_t debug_log_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
    return sprintf(buf, "%s\n", debug==0?"log off":"log on");
}

static ssize_t debug_log_store(struct device *dev, struct device_attribute *attr, const char *buf,
                               size_t count)
{
    debug = simple_strtoul(buf, NULL, 10);
    return count;
}

static ssize_t show_tptype(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d,%s\n", data->tptype, (data->tptype==1)?"Black":"White");
}

static ssize_t store_tptype(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds9921_data *data = i2c_get_clientdata(client);
    unsigned int type=0,err=0;

    err = kstrtoint(buf, 10, &type);
    if( err ) INFOR("ps store_tptype failed\n");

    INFOR("tptype:%d\n", type);

    if( type ) {
        data->tptype = 1; //Black
    } else {
        data->tptype = 0; //White
    }

    return count;
}

/* sysfs attributes operation function*/

/*
 * ps
 */
static DEVICE_ATTR(reg, 0664, ps_show_reg, ps_store_reg);
static DEVICE_ATTR(ps_calibration, 0664, NULL, ps_calibration_store);
static DEVICE_ATTR(ps_calibbias, 0664, ps_calibvalue_result_show, NULL);
static DEVICE_ATTR(ps_offset, 0664, ps_offset_show, ps_offset_store);
//static DEVICE_ATTR(ps_batch, 0664, NULL, txc_batch_store);

static DEVICE_ATTR(ps_non_wakeup_enable, 0664, non_wakeup_enable_show, NULL);
static DEVICE_ATTR(ps_wakeup_enable, 	 0664, wakeup_enable_show, NULL);
//static DEVICE_ATTR(debug, 0664, debug_show, debug_store);
static DEVICE_ATTR(ps_irq_gpio, 0664, gpio_status, NULL);
static DEVICE_ATTR(tptype, 0664 ,show_tptype, store_tptype);

/*
 * ambilight
 *
 */
static DEVICE_ATTR(als_batch, 0664, als_batch_show, als_batch_store);
static DEVICE_ATTR(als_calibration, 0664, als_calibration_show, als_calibration_store);
/*
 * debug log on/off
 */
static DEVICE_ATTR(debug_log, 0664, debug_log_show, debug_log_store);

static struct attribute *apds_dbg_attributes[] = {
        &dev_attr_reg.attr,
        &dev_attr_ps_calibration.attr,
        &dev_attr_ps_calibbias.attr,
        &dev_attr_ps_offset.attr,
        &dev_attr_ps_non_wakeup_enable.attr,
        &dev_attr_ps_wakeup_enable.attr,
        &dev_attr_ps_irq_gpio.attr,
        &dev_attr_tptype.attr,
        /*
         * Ambilight
         */
        &dev_attr_als_batch.attr,
        &dev_attr_als_calibration.attr,

        &dev_attr_debug_log.attr,
        NULL
};

static const struct attribute_group apds_dbg_attr_group = {
        .attrs = apds_dbg_attributes,
};

void apds_create_dbg_sys(struct kobject *kobj)
{
    int err = 0;
    err = sysfs_create_group(kobj, &apds_dbg_attr_group);
    if( err ) {
        INFOR("create apds dbg sys failed\n");
    }
}

int mz_ps_eint_register(irq_handler_t handler, void *client, struct apds9921_data *data)
{
    int halls[2] = {0};
    int result = 0;
    int ps_irq = 0,rc = -1;
    struct device_node *ps_node;

    /* register EINT handler for hall key */
    ps_node = of_find_compatible_node(NULL, NULL, "mediatek,apds9922");
    if (!ps_node) {
        INFOR("[awago] can't find compatible node\n");
    }
    else {
        of_property_read_u32_array(ps_node, "debounce", halls, ARRAY_SIZE(halls));
        gpio_set_debounce(halls[0], halls[1]);

        ps_irq = irq_of_parse_and_map(ps_node, 0);
        if( ps_irq ) {
            INFOR("[awago] mt_eint_register ps irq=%d, get irq ok\n", ps_irq);
        } else {
            INFOR("[awago] mt_eint_register ps irq=%d, get irq failed\n", ps_irq);
        }

        result = request_threaded_irq(ps_irq, NULL, handler,
                             IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING|IRQF_ONESHOT, "PS_int-eint", client); // IRQF_ONESHOT
        if (result > 0)
            pr_err("[awago] EINT IRQ line not available!\n");

        rc = of_property_read_u32(ps_node, "gpio", &data->gpio);
        if (!rc) {
            INFOR("get gpio:%d\n",data->gpio);
        }
        else {
            INFOR("Unable to get gpio\n");
        }
    }

    INFOR("dts irq:%d\n", ps_irq);
    return ps_irq;
}