/*
 * IC drv2605
e* input mode:RTP (Real-Time Playback)
 * output mode:LRA (Linear Resonance Actuator)
 * Rated Voltage: Rated Voltage[7:0]
 * overdrive clamp voltage: ODClamp[7:0]
 *
 * Vavg = Rated Voltage[7:0]*5.44/255 = 3v
 * Vavg = 0.91Vpeak
 * Vpeak = ODClamp[7:0]*5.44/255
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#define MOTOR_NAME      "DRV2604"
#define TAG "[vibrator]"

/* drv2605 register */
#define REG_STATUS (0x00)
#define REG_INPUT_MODE (0x01)
#define REG_RTP  (0x02)
/* #define REG_GO   (0x0c) */
#define REG_RATED_VOLTAGE   (0x16)
#define REG_OVERDRIVE_CLAMP (0x17)
#define REG_FEEDBACK_CONTROL (0x1a) // bit7 ERM or LRA mode 0:ERM mode 1:LRA mode
#define REG_CONTROL2  (0x1c) // bit7 output direction  0:uni-directional 1:bi-directional
#define REG_CONTROL3  (0x1d) // bit3 dataFormat_RTP    0:singned         1:unsigned

/* drv2605 input mode */
#define RTP_MODE         (0x05)

/* check status */
//#define CHECK_IN  (1)
//#define CHECK_OUT (0)
/*
#define WAIT_TIMES (20)
#define CAL_FLAG (1<<3)
#define CAL_PASS (0)
#define CAL_FAIL (1)
*/

/* Pin */
#define MOTOR_PIN_STA_OUT_H 	1
#define MOTOR_PIN_STA_OUT_L 	0

/* static struct pinctrl_state *motor_en_init; */
static struct pinctrl_state *motor_en_oh;
static struct pinctrl_state *motor_en_ol;
static struct pinctrl *motor_en_pinctrl;

static unsigned char is_dev_ready = 0;
static struct i2c_client *sClient = NULL;

static unsigned char motor_read_byte(unsigned char cmd)
{
	struct i2c_msg msg[2];
	unsigned char databuf = 0;
	int count = 1;
	int ret = 0;

	databuf = cmd;
        msg[0].addr = sClient->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = &databuf;

        msg[1].addr = sClient->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = count;
        msg[1].buf = &databuf;

        ret = i2c_transfer(sClient->adapter, msg, sizeof(msg)/sizeof(msg[0]));
	if (ret < 0) {
		pr_err(TAG"%s: read data error!\n", __func__);
		return ret;
	}

	return databuf;
}

static int motor_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = {0};
	int ret = 0;

	/*if(!sClient) {
		pr_err(TAG"%s: i2c_client is NULL\n",__func__);
		return -1;
	}*/
	//else {
		write_data[0] = cmd;
		write_data[1] = writeData;
		ret = i2c_master_send(sClient, write_data, 2);
		if (ret < 0) {
			pr_err(TAG"%s: send command error!!\n",__func__);
			return ret;
		}
//	}

	return 0;
}

static int motor_init_gpio(void)
{
	pr_debug(TAG"%s enter..\n", __func__);

	motor_en_pinctrl = devm_pinctrl_get(&sClient->dev);
	if (IS_ERR(motor_en_pinctrl)) {
		pr_err(TAG"%s: unable to find motor pinctrl, error: %ld\n",
				__func__, PTR_ERR(motor_en_pinctrl));
		return -1;
	}
#if 0
	motor_en_init = pinctrl_lookup_state(motor_en_pinctrl, "init");
	if (NULL == motor_en_init) {
		printk("Cannot find motor pin init state!\n");
		return -2;
	}
#endif

	motor_en_oh = pinctrl_lookup_state(motor_en_pinctrl, "high_cfg");
	if (NULL == motor_en_oh) {
		pr_err(TAG"%s: Cannot find motor enable pin oh state!\n", __func__);
		return -3;
	}

	motor_en_ol = pinctrl_lookup_state(motor_en_pinctrl, "low_cfg");
	if (NULL == motor_en_ol) {
		pr_err(TAG"%s: Cannot find motor enable pin ol state!\n", __func__);
		return -4;
	}

	return 0;
}

static int motor_set_gpio(int state)
{

	pr_debug(TAG"%s enter..\n", __func__);

	switch (state) {
#if 0
	case MOTOR_PIN_STA__INIT:
	case MOTOR_PIN_STA__DEINIT:
		pinctrl_select_state(motor_en_pinctrl, motor_en_init);
		printk("set motor enable pin to init\n");
		break;
#endif
	case MOTOR_PIN_STA_OUT_H:
		pinctrl_select_state(motor_en_pinctrl, motor_en_oh);
		pr_debug("set motor enable pin to oh\n");
		break;
	case MOTOR_PIN_STA_OUT_L:
		pinctrl_select_state(motor_en_pinctrl, motor_en_ol);
		pr_debug("set motor enable pin to ol\n");
		break;

	default:
		pr_err(TAG"%s: %d mode not defined for motor enable pin !!!\n",
				__func__, state);
		break;
	}
	return 0;
}

static inline void get_motor_standby(void)
{
	/* clear RTP strength */
	motor_write_byte(REG_RTP, 0x00); /* set RTP strength to 0 */

	/* enter shutdown mode */
	motor_set_gpio(MOTOR_PIN_STA_OUT_L);
}

static inline void get_motor_ready(void)
{
	/* enter ready mode */
	motor_set_gpio(MOTOR_PIN_STA_OUT_H);
	usleep_range(1000, 2000); /* delay at least 1ms */
	/* set RTP strength to full-scale */
	motor_write_byte(REG_RTP, 0xff);
//	motor_write_byte(REG_INPUT_MODE, RTP_MODE); /* work in RTP mode */
}

void motor_enable(void)
{
        if( is_dev_ready ) {
                get_motor_ready();
        }
}

void motor_disable(void)
{
        if( is_dev_ready ) {
                get_motor_standby();
        }
}

static void check_dev_id(void)
{
	unsigned char data = 0;

	data = motor_read_byte(REG_STATUS);
	switch( data&0xe0 ) {
		case 0x80:
			is_dev_ready = 1;
			printk(TAG"%s: this is DRV2604\n",__func__);
			break;
		case 0x60:
			is_dev_ready = 1;
			printk(TAG"%s: this is DRV2605\n",__func__);
			break;
		case 0xc0:
			is_dev_ready = 1;
			printk(TAG"%s: this is DRV2604L\n",__func__);
			break;
                case 0xe0:
                        is_dev_ready = 1;
                        printk(TAG"%s this is DRV2605L\n",__func__);
                        break;
		default:
			pr_err(TAG"%s: not identify id:0x%x\n",__func__,data&0xe0);
	}
}

static int I2C_SetBit(unsigned char Reg, unsigned char Mask, unsigned char Val)
{
	unsigned char RegValue = motor_read_byte(Reg);
	if((RegValue&Mask)==Val)
		return 0;
	else{
		RegValue = (RegValue&~Mask)|Val;
		return motor_write_byte(Reg, RegValue);
	}
}

void motor_hw_init(void)
{
	//int i=0;

	pr_debug(TAG"%s: enter\n", __func__);

	/* step 3: set drv2605 to work in RTP mode */
	motor_write_byte(REG_RTP, 0);
	motor_write_byte(REG_INPUT_MODE, RTP_MODE);

	/* step 4: set standby bit 0 to wakeup device; Check motor ID */
	//I2C_SetBit(REG_INPUT_MODE, 0x40, 0x00);
	usleep_range(1000, 2000); //wait at least 1ms for device ready
	check_dev_id();		  //check ti motor ID

	/* step 5: set Rated Voltage, Clamp Voltage, Feedback Control */
	motor_write_byte(REG_RATED_VOLTAGE, 0x16);
	motor_write_byte(REG_OVERDRIVE_CLAMP, 0x23);
	I2C_SetBit(REG_FEEDBACK_CONTROL, 0x80, 0x80); //set LRA actuator
	
	/* step 6: set Control2 and Control3 */
	I2C_SetBit(REG_CONTROL2, 0x80, 0x00); //set Uni-Directional
	I2C_SetBit(REG_CONTROL3, 0x09, 0x08); //set LRA close loop, and DataFormat_RTP to unsigned

#if 0	/* skip auto-calibration */

	/* step 7: start Calibration */
	motor_write_byte(REG_INPUT_MODE, 0x07);
	/* polling GO bit, until GO bit self-cleared */
	while(((motor_read_byte(REG_GO)&0x01)==0x01)&&(++i<WAIT_TIMES));

	if(i == WAIT_TIMES) {
		pr_err(TAG"[%s] Calibration time out,may be failed\n", __func__);
	} else {
		pr_debug(TAG"[%s] Calibration wait times:%d\n", __func__, i);
	}

	/* calibration done: check Diag_Result BIT for calibration success or failure */
	if( (motor_read_byte(REG_STATUS)&CAL_FLAG)==CAL_PASS ) {
		pr_debug(TAG"[%s] auto calibration success\n",__func__);
	} else {
		pr_err(TAG"[%s] auto calibration failed\n",__func__);
	}
#endif

	/* step 8: suspend device */
	//I2C_SetBit(REG_INPUT_MODE, 0x40, 0x40);
	
	
	get_motor_standby();

	pr_debug(TAG"%s: exit \n", __func__);
}

#ifdef CONFIG_PM
static int motor_suspend(struct device *dev){
	// option 1:
	// set the 0x01 register's STANDBY bit to 1
	// to take the device into the standby mode
	// option 2(used):
	// de-assert the IC'S EN pin
	// the options above will achieve the same effect
	get_motor_standby();
	return 0;
}

static int motor_resume(struct device *dev)
{
	// do nothing
	return 0;
}

static const struct dev_pm_ops motor_pm_ops= {
	.suspend = motor_suspend,
	.resume  = motor_resume,
};
#endif

static int motor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk(TAG"%s enter..\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err(TAG"%s: I2c function error\n", __func__);
		return -ENODEV;
	}
	sClient = client;
	/* Initialization Procedure(total 8 steps) */
	// step 1: Power up & wait at least 250 μs before the DRV2605 will accept i2c commands.
	//         In our project, VDD connects to VSYS (always on), so skip this step
	// step 2: Assert motor EN pin (logic high)
	if (motor_init_gpio() < 0) {
		pr_err(TAG"%s: init enable pin failed!\n", __func__);
		return -1;
	}
	motor_set_gpio(MOTOR_PIN_STA_OUT_H);
	usleep_range(1000, 2000); //delay at least 1ms
	// step other
	motor_hw_init();

	printk(TAG"%s: success!", __func__);
	return 0;
}

static int motor_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_device_id motor_i2c_dev_id[] = {
        {MOTOR_NAME, 0},
        {}
};
MODULE_DEVICE_TABLE(i2c, motor_i2c_dev_id);

static const struct of_device_id ti_motor_of_match[] = {
        { .compatible = "mediatek,motor" },
        {},
};
MODULE_DEVICE_TABLE(of, ti_motor_of_match);

static struct i2c_driver ti_motor_driver = {
	.driver = {
		.name = MOTOR_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ti_motor_of_match,
#ifdef CONFIG_PM
		.pm = &motor_pm_ops,
#endif
	},
	.probe = motor_probe,
	.remove = motor_remove,
	.id_table = motor_i2c_dev_id,
};

static int __init motor_init(void)
{
	int rc = 0;

        /* register motor i2c driver */
        rc = i2c_add_driver(&ti_motor_driver);
	if (rc)
		pr_err(TAG"%s failed: i2c_add_driver rc=%d\n", __func__, rc);

	return rc;
}

static void __exit motor_exit(void)
{
        i2c_del_driver(&ti_motor_driver);
}

late_initcall(motor_init);
module_exit(motor_exit);

MODULE_AUTHOR("Yin ShunQing <ysq@meizu.com>");
MODULE_DESCRIPTION("TI DRV2605/2604 motor I2C driver");
MODULE_LICENSE("GPL v2");
