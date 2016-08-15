
#include "BH1745_driver.h"

static ssize_t als_show_reg(struct device *dev, struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);

    int count=0;
    int i=0	;
    int val = 0, reg = 0;
    #define arr_size (15)

    struct als_reg als_reg_arr[arr_size] = {
            {0x40,}, //7:sw rst;6:irq rst;5~0:0x0b, device id
            {0x41,}, //7~3:reserved; 2~0:measure time
            {0x42,}, //4:1-rgbc measure active; 1~0:gain 00-1,01-2,10-16
            {0x44,}, //4~3:ps resolution; 2~0:ps measure rate
            {0x50,}, //red data LSB
            {0x51,}, //red data MSB
            {0x52,}, //green data LSB
            {0x53,}, //green data MSB
            {0x54,}, //BLUE data LSB
            {0x55,}, //BLUE data MSB
            {0x56,}, //CLEAR data LSB
            {0x57,}, //CLEAR data MSB
            {0x60,}, //7:1-irq is active; 3~2:irq source,red/green/blue/clear channel; 0:int enable
            {0x61,}, // ps offset ,LSB
            {0x62,}  // ps offset ,MSB

    };

    for(i = 0;i <arr_size ;i++)
    {
        reg = als_reg_arr[i].reg;
        val = i2c_smbus_read_byte_data(client, reg);
        als_reg_arr[i].data = val & 0xff;

        count+=sprintf(buf+count,"[%.2x] = (%.2x)\n",als_reg_arr[i].reg,als_reg_arr[i].data);
    }

    return count;
}
static ssize_t als_store_reg(struct device *dev, struct device_attribute *attr, const char *buf,
                            size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);

    unsigned int offset = 0;
    unsigned char buffer[2] = {0,};

    offset = simple_strtol(buf, NULL, 16);
    buffer[0] = (offset&0xff00)>>0x08; // reg
    buffer[1] = (offset&0x00ff);       // value
    INFOR("reg:0x%x, value:0x%x\n",buffer[0], buffer[1]);

    i2c_smbus_write_byte_data(client, buffer[0], buffer[1]);

    return count;
}

static ssize_t show_tptype(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct RGB_DATA *rgb = i2c_get_clientdata(client);

    return sprintf(buf, "%d,%s\n", rgb->tptype, (rgb->tptype==1)?"Black":"White");
}

static ssize_t store_tptype(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct RGB_DATA *rgb = i2c_get_clientdata(client);
    unsigned int type=0,err=0;

    err = kstrtoint(buf, 10, &type);
    if( err ) INFOR("als store_tptype failed\n");

    INFOR("tptype:%d\n", type);

    if( type ) {
        rgb->tptype = 1; //Black
    } else {
        rgb->tptype = 0; //White
    }

    return count;
}

static ssize_t als_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct RGB_DATA *rgb = i2c_get_clientdata(client);

    unsigned long val = simple_strtoul(buf, NULL, 10);

    rgb->als_scale = (val&0xff);
    if( rgb->als_scale<=0 ) {
        rgb->als_scale = 1;
        rgb->als_cali = CALI_FAIL;
    } else {
        rgb->als_cali = CALI_OK;
    }

    INFOR("als scale:%d,cali:%d-%s\n",rgb->als_scale, rgb->als_cali,rgb->als_cali==CALI_OK?"cali ok":"cali fail");
    return count;
}

static ssize_t als_calibvalue_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct RGB_DATA *rgb = i2c_get_clientdata(client);

    INFOR("als cali result:%s, scale:0x%.2x--%d /10\n",
          rgb->als_cali==CALI_FAIL?"cali fail":"cali ok", rgb->als_scale,rgb->als_scale);
    if( rgb->als_cali==CALI_OK ) {
        return sprintf(buf, "%d\n", rgb->als_scale);
    } else {
        INFOR("ps cali failed\n");
        return sprintf(buf, "%d-%s\n", rgb->als_cali, rgb->als_cali ==CALI_OK?"cali ok":"cali fail");
    }
}

static DEVICE_ATTR(reg, 0664, als_show_reg, als_store_reg);
static DEVICE_ATTR(tptype, 0664 ,show_tptype, store_tptype);
static DEVICE_ATTR(als_calibration, 0664, NULL, als_calibration_store);
static DEVICE_ATTR(als_calibbias, 0664, als_calibvalue_result_show, NULL);

static struct attribute *als_dbg_attributes[] = {
        &dev_attr_reg.attr,
        &dev_attr_tptype.attr,
        &dev_attr_als_calibration.attr,
        &dev_attr_als_calibbias.attr,
        NULL
};

static const struct attribute_group als_dbg_attr_group = {
        .attrs = als_dbg_attributes,
};

void als_create_dbg_sys(struct kobject *kobj)
{
    int err = 0;
    err = sysfs_create_group(kobj, &als_dbg_attr_group);
    if( err ) {
        INFOR("create als dbg sys failed\n");
    }
}