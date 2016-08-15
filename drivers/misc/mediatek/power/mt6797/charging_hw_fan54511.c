#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_meter.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "fan54511_reg.h"
#include <mach/mt_sleep.h>
#include <mt-plat/mt_gpio.h>
/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

/* ============================================================ // */
/* Global variable */
/* ============================================================ // */

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

#if defined(GPIO_PWR_AVAIL_WLC)
/*K.S.?*/
unsigned int wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC;
#else
unsigned int wireless_charger_gpio_number = 0;
#endif

#endif

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;
#endif

kal_bool charging_type_det_done = KAL_TRUE;

/*BQ25890 REG06 VREG[5:0]*/
const unsigned int VBAT_CV_VTH[] = {
	3840000, 3856000, 3872000, 3888000,
	3904000, 3920000, 3936000, 3952000,
	3968000, 3984000, 4000000, 4016000,
	4032000, 4048000, 4064000, 4080000,
	4096000, 4112000, 4128000, 4144000,
	4160000, 4176000, 4192000, 4208000,
	4224000, 4240000, 4256000, 4272000,
	4288000, 4304000, 4320000, 4336000,
	4352000, 4368000, 4384000, 4400000,
	4416000, 4432000, 4448000, 4464000,
	4480000, 4496000, 4512000, 4528000,
	4544000, 4560000, 4576000, 4592000,
	4608000
};

/*BQ25890 REG04 ICHG[6:0]*/
const unsigned int CS_VTH[] = {
	0, 6400, 12800, 19200,
	25600, 32000, 38400, 44800,
	51200, 57600, 64000, 70400,
	76800, 83200, 89600, 96000,
	102400, 108800, 115200, 121600,
	128000, 134400, 140800, 147200,
	153600, 160000, 166400, 172800,
	179200, 185600, 192000, 198400,
	204800, 211200, 217600, 224000,
	230400, 236800, 243200, 249600,
	256000, 262400, 268800, 275200,
	281600, 288000, 294400, 300800,
	307200, 313600, 320000, 326400,
	332800, 339200, 345600, 352000,
	358400, 364800, 371200, 377600,
	384000, 390400, 396800, 403200,
	409600, 416000, 422400, 428800,
	435200, 441600, 448000, 454400,
	460800, 467200, 473600, 480000,
	486400, 492800, 499200, 505600
};

/*BQ25890 REG00 IINLIM[5:0]*/
const unsigned int INPUT_CS_VTH[] = {
	10000, 15000, 20000, 25000,
	30000, 35000, 40000, 45000,
	50000, 55000, 60000, 65000,
	70000, 75000, 80000, 85000,
	90000, 95000, 100000, 105000,
	110000, 115000, 120000, 125000,
	130000, 135000, 140000, 145000,
	150000, 155000, 160000, 165000,
	170000, 175000, 180000, 185000,
	190000, 195000, 200000, 200500,
	210000, 215000, 220000, 225000,
	230000, 235000, 240000, 245000,
	250000, 255000, 260000, 265000,
	270000, 275000, 280000, 285000,
	290000, 295000, 300000, 305000,
	310000, 315000, 320000, 325000
};

const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	BATTERY_VOLT_10_500000_V
};

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#ifndef CUST_GPIO_VIN_SEL
#define CUST_GPIO_VIN_SEL 18
#endif
DISO_IRQ_Data DISO_IRQ;
int g_diso_state = 0;
int vin_sel_gpio_number = (CUST_GPIO_VIN_SEL | 0x80000000);
static char *DISO_state_s[8] = {
	"IDLE",
	"OTG_ONLY",
	"USB_ONLY",
	"USB_WITH_OTG",
	"DC_ONLY",
	"DC_WITH_OTG",
	"DC_WITH_USB",
	"DC_USB_OTG",
};
#endif

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */


/* ============================================================ // */
/* extern variable */
/* ============================================================ // */

/* ============================================================ // */
/* extern function */
/* ============================================================ // */
/* extern unsigned int upmu_get_reg_value(unsigned int reg); upmu_common.h, _not_ used */
/* extern bool mt_usb_is_device(void); _not_ used */
/* extern void Charger_Detect_Init(void); _not_ used */
/* extern void Charger_Detect_Release(void); _not_ used */
/* extern int hw_charging_get_charger_type(void);  included in charging.h*/
/* extern void mt_power_off(void); _not_ used */
/* extern unsigned int mt6311_get_chip_id(void); _not_ used*/
/* extern int is_mt6311_exist(void); _not_ used */
/* extern int is_mt6311_sw_ready(void); _not_ used */
#ifdef CONFIG_MTK_BIF_SUPPORT
static int bif_inited;
#endif
static unsigned int charging_error;
static unsigned int charging_get_error_state(void);
static unsigned int charging_set_error_state(void *data);
/* ============================================================ // */
unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	if (val < array_size)
		return parameter[val];

		battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
		return parameter[0];

}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	unsigned int i;

	battery_log(BAT_LOG_FULL, "array_size = %d \r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					 unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				battery_log(2, "zzf_%d<=%d     i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
static unsigned int is_chr_det(void)
{
	unsigned int val = 0;

	val = pmic_get_register_value(MT6351_PMIC_RGS_CHRDET);
	battery_log(BAT_LOG_CRTI, "[is_chr_det] %d\n", val);

	return val;
}
#endif
#ifdef CONFIG_MTK_BIF_SUPPORT
/* BIF related functions*/
#define BC (0x400)
#define SDA (0x600)
#define ERA (0x100)
#define WRA (0x200)
#define RRA (0x300)
#define WD  (0x000)
/*bus commands*/
#define BUSRESET (0x00)
#define RBL2 (0x22)
#define RBL4 (0x24)

/*BIF slave address*/
#define MW3790 (0x00)
#define MW3790_VBAT (0x0114)
#define MW3790_TBAT (0x0193)
void bif_reset_irq(void)
{
	unsigned int reg_val = 0;
	unsigned int loop_i = 0;

	pmic_set_register_value(PMIC_BIF_IRQ_CLR, 1);
	reg_val = 0;
	do {
		reg_val = pmic_get_register_value(PMIC_BIF_IRQ);

		if (loop_i++ > 50) {
			battery_log(BAT_LOG_CRTI, "[BIF][reset irq]failed.PMIC_BIF_IRQ 0x%x %d\n",
				    reg_val, loop_i);
			break;
		}
	} while (reg_val != 0);
	pmic_set_register_value(PMIC_BIF_IRQ_CLR, 0);
}

void bif_waitfor_slave(void)
{
	unsigned int reg_val = 0;
	int loop_i = 0;

	do {
		reg_val = pmic_get_register_value(PMIC_BIF_IRQ);

		if (loop_i++ > 50) {
			battery_log(BAT_LOG_CRTI,
				    "[BIF][waitfor_slave] failed. PMIC_BIF_IRQ=0x%x, loop=%d\n",
				    reg_val, loop_i);
			break;
		}
	} while (reg_val == 0);

	if (reg_val == 1)
		battery_log(BAT_LOG_FULL, "[BIF][waitfor_slave]OK at loop=%d.\n", loop_i);

}

int bif_powerup_slave(void)
{
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;
	int loop_i = 0;

	do {
		battery_log(BAT_LOG_FULL, "[BIF][powerup_slave] set BIF power up register\n");
		pmic_set_register_value(PMIC_BIF_POWER_UP, 1);

		battery_log(BAT_LOG_FULL, "[BIF][powerup_slave] trigger BIF module\n");
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(10);

		bif_waitfor_slave();

		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);

		pmic_set_register_value(PMIC_BIF_POWER_UP, 0);

		/*check_bat_lost(); what to do with this? */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i < 5) {
			loop_i++;
		} else {
			battery_log(BAT_LOG_CRTI, "[BIF][powerup_slave]Failed at loop=%d", loop_i);
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);
	if (loop_i < 5) {
		battery_log(BAT_LOG_FULL, "[BIF][powerup_slave]OK at loop=%d", loop_i);
	bif_reset_irq();
		return 1;
	}

	return -1;
}

void bif_set_cmd(int bif_cmd[], int bif_cmd_len)
{
	int i = 0;
	int con_index = 0;
	unsigned int ret = 0;

	for (i = 0; i < bif_cmd_len; i++) {
		ret = pmic_config_interface(MT6351_BIF_CON0 + con_index, bif_cmd[i], 0x07FF, 0);
		con_index += 0x2;
	}
}


int bif_reset_slave(void)
{
	unsigned int ret = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;
	int bif_cmd[1] = { 0 };
	int loop_i = 0;

	/*set command sequence */
	bif_cmd[0] = BC | BUSRESET;
	bif_set_cmd(bif_cmd, 1);

	do {
		/*command setting : 1 write command */
		ret = pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 1);
		ret = pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 0);

		/*Command set trigger */
		ret = pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(10);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		ret = pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i < 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI, "[BIF][bif_reset_slave]Failed at loop=%d",
				    loop_i);
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	if (loop_i < 50) {
		battery_log(BAT_LOG_FULL, "[BIF][bif_reset_slave]OK at loop=%d", loop_i);
	/*reset BIF_IRQ */
	bif_reset_irq();
		return 1;
	}
	return -1;
}

/*BIF WRITE 8 transaction*/
int bif_write8(int addr, int *data)
{
	int ret = 1;
	int era, wra;
	int bif_cmd[4] = { 0, 0, 0, 0};
	int loop_i = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;

	era = (addr & 0xFF00) >> 8;
	wra = addr & 0x00FF;
	battery_log(BAT_LOG_FULL, "[BIF][bif_write8]ERA=%x, WRA=%x\n", era, wra);
	/*set command sequence */
	bif_cmd[0] = SDA | MW3790;
	bif_cmd[1] = ERA | era;	/*[15:8] */
	bif_cmd[2] = WRA | wra;	/*[ 7:0] */
	bif_cmd[3] = WD  | (*data & 0xFF);	/*data*/


	bif_set_cmd(bif_cmd, 4);
	do {
		/*command setting : 4 transactions for 1 byte write command(0) */
		pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 4);
		pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 0);

		/*Command set trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(200);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i <= 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI,
		"[BIF][bif_write8] Failed. bat_lost = %d, timeout = %d, totoal_valid = %d\n",
		bat_lost, timeout, total_valid);
			ret = -1;
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	if (ret == 1)
		battery_log(BAT_LOG_FULL, "[BIF][bif_write8] OK for %d loop(s)\n", loop_i);
	else
		battery_log(BAT_LOG_CRTI, "[BIF][bif_write8] Failed for %d loop(s)\n", loop_i);

	/*reset BIF_IRQ */
	bif_reset_irq();

	return ret;
}

/*BIF READ 8 transaction*/
int bif_read8(int addr, int *data)
{
	int ret = 1;
	int era, rra;
	int val = -1;
	int bif_cmd[3] = { 0, 0, 0 };
	int loop_i = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;

	battery_log(BAT_LOG_FULL, "[BIF][READ8]\n");

	era = (addr & 0xFF00) >> 8;
	rra = addr & 0x00FF;
	battery_log(BAT_LOG_FULL, "[BIF][bif_read8]ERA=%x, RRA=%x\n", era, rra);
	/*set command sequence */
	bif_cmd[0] = SDA | MW3790;
	bif_cmd[1] = ERA | era;	/*[15:8] */
	bif_cmd[2] = RRA | rra;	/*[ 7:0] */

	bif_set_cmd(bif_cmd, 3);
	do {
		/*command setting : 3 transactions for 1 byte read command(1) */
		pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 3);
		pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 1);
		pmic_set_register_value(PMIC_BIF_READ_EXPECT_NUM, 1);

		/*Command set trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(200);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i <= 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI,
		"[BIF][bif_read16] Failed. bat_lost = %d, timeout = %d, totoal_valid = %d\n",
		bat_lost, timeout, total_valid);
			ret = -1;
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	/*Read data */
	if (ret == 1) {
		val = pmic_get_register_value(PMIC_BIF_DATA_0);
		battery_log(BAT_LOG_FULL, "[BIF][bif_read8] OK d0=0x%x, for %d loop(s)\n",
			    val, loop_i);
	} else
		battery_log(BAT_LOG_CRTI, "[BIF][bif_read8] Failed for %d loop(s)\n", loop_i);

	/*reset BIF_IRQ */
	bif_reset_irq();

	*data = val;
	return ret;
}

/*bif read 16 transaction*/
int bif_read16(int addr)
{
	int ret = 1;
	int era, rra;
	int val = -1;
	int bif_cmd[4] = { 0, 0, 0, 0 };
	int loop_i = 0;
	int bat_lost = 0;
	int total_valid = 0;
	int timeout = 0;

	battery_log(BAT_LOG_FULL, "[BIF][READ]\n");

	era = (addr & 0xFF00) >> 8;
	rra = addr & 0x00FF;
	battery_log(BAT_LOG_FULL, "[BIF][bif_read16]ERA=%x, RRA=%x\n", era, rra);
	/*set command sequence */
	bif_cmd[0] = SDA | MW3790;
	bif_cmd[1] = BC | RBL2;	/* read back 2 bytes */
	bif_cmd[2] = ERA | era;	/*[15:8] */
	bif_cmd[3] = RRA | rra;	/*[ 7:0] */

	bif_set_cmd(bif_cmd, 4);
	do {
		/*command setting : 4 transactions for 2 byte read command(1) */
		pmic_set_register_value(PMIC_BIF_TRASFER_NUM, 4);
		pmic_set_register_value(PMIC_BIF_COMMAND_TYPE, 1);
		pmic_set_register_value(PMIC_BIF_READ_EXPECT_NUM, 2);

		/*Command set trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 1);

		udelay(200);
		/*Command sent; wait for slave */
		bif_waitfor_slave();

		/*Command clear trigger */
		pmic_set_register_value(PMIC_BIF_TRASACT_TRIGGER, 0);
		/*check transaction completeness */
		bat_lost = pmic_get_register_value(PMIC_BIF_BAT_LOST);
		total_valid = pmic_get_register_value(PMIC_BIF_TOTAL_VALID);
		timeout = pmic_get_register_value(PMIC_BIF_TIMEOUT);

		if (loop_i <= 50)
			loop_i++;
		else {
			battery_log(BAT_LOG_CRTI,
		"[BIF][bif_read16] Failed. bat_lost = %d, timeout = %d, totoal_valid = %d\n",
		bat_lost, timeout, total_valid);
			ret = -1;
			break;
		}
	} while (bat_lost == 1 || total_valid == 1 || timeout == 1);

	/*Read data */
	if (ret == 1) {
		int d0, d1;

		d0 = pmic_get_register_value(PMIC_BIF_DATA_0);
		d1 = pmic_get_register_value(PMIC_BIF_DATA_1);
		val = 0xFF & d1;
		val = val | ((d0 & 0xFF) << 8);
		battery_log(BAT_LOG_FULL, "[BIF][bif_read16] OK d0=0x%x, d1=0x%x for %d loop(s)\n",
			    d0, d1, loop_i);
	} else
		battery_log(BAT_LOG_CRTI, "[BIF][bif_read16] Failed for %d loop(s)\n", loop_i);

	/*reset BIF_IRQ */
	bif_reset_irq();


	return val;
}

void bif_ADC_enable(void)
{
	int reg = 0x18;

	bif_write8(0x0110, &reg);
	mdelay(50);

	reg = 0x98;
	bif_write8(0x0110, &reg);
	mdelay(50);

}

/* BIF init function called only at the first time*/
int bif_init(void)
{
	int pwr, rst;
	/*disable BIF interrupt */
	pmic_set_register_value(PMIC_INT_CON0_CLR, 0x4000);
	/*enable BIF clock */
	pmic_set_register_value(PMIC_TOP_CKPDN_CON2_CLR, 0x0070);

	/*enable HT protection */
	pmic_set_register_value(PMIC_RG_BATON_HT_EN, 1);

	/*change to HW control mode*/
	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_ON_CTRL, 0);*/
	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_EN, 1);*/
	mdelay(50);

	/*Enable RX filter function */
	pmic_set_register_value(MT6351_PMIC_BIF_RX_DEG_EN, 0x8000);
	pmic_set_register_value(MT6351_PMIC_BIF_RX_DEG_WND, 0x17);
	pmic_set_register_value(PMIC_RG_BATON_EN, 0x1);
	pmic_set_register_value(PMIC_BATON_TDET_EN, 0x1);
	pmic_set_register_value(PMIC_RG_BATON_HT_EN_DLY_TIME, 0x1);


	/*wake up BIF slave */
	pwr = bif_powerup_slave();
	mdelay(10);
	rst = bif_reset_slave();

	/*pmic_set_register_value(MT6351_PMIC_RG_VBIF28_ON_CTRL, 1);*/
	mdelay(50);

	battery_log(BAT_LOG_CRTI, "[BQ25896][BIF_init] done.");

	if (pwr + rst == 2)
		return 1;

	return -1;
}
#endif
static unsigned int charging_hw_init(void *data)
{
 	unsigned int status = STATUS_OK;
	
	fan54511_main_set_HZMODE(0);
	fan54511_sub_set_HZMODE(0);
            fan54511_set_DIVCON(0);     //Disable DIVC 
	//Set VSafe and Isafe, must be set at the first time
//	fan54511_set_SAFE(0xB9); //VSAFE 4.4V, 11*100mV+3.3V | ISAFE, 2A, 9*200mA+200mA 

	fan54511_set_VBUSLIM(0x04); //4.56V
	fan54511_set_VSYS(0x3); //Minimum system voltage 3.2V

	fan54511_set_PRECHG(0x08); //Precharge current 450mA
	fan54511_set_ITERM(0x07); //Termination current 200mA
	fan54511_set_VFLOAT(0x69); //4.35V
	fan54511_set_vlowv(0x00); // Battery Voltage 3.0V

	fan54511_set_TE(0x0); //Disable termination
	fan54511_set_CONT(0x1); //Continue Charging even precharge and fast charge timer timeout.
	
	return status;
}

static unsigned int charging_get_bif_vbat(void *data);

static unsigned int charging_dump_register(void *data)
{
	unsigned int status = STATUS_OK;

	battery_log(BAT_LOG_FULL, "charging_dump_register\r\n");

            fan54511_dump_register();

	return status;
}



 static unsigned int charging_enable_major(void *data)
 {	battery_log(BAT_LOG_CRTI, "%s\n",__FUNCTION__);
 	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int*)(data);

	if(KAL_TRUE == enable)
	{
		fan54511_sub_stop_charging();
		fan54511_main_start_charging();
	}
	else
	{
		fan54511_main_stop_charging();
		fan54511_sub_stop_charging();
	}

	return status;
 }

 static unsigned int charging_enable_minor(void *data)
 {	battery_log(BAT_LOG_CRTI, "%s\n",__FUNCTION__);
 	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int*)(data);

	if(KAL_TRUE == enable)
	{
		fan54511_main_stop_charging();
		fan54511_sub_start_charging();
	}
	else
	{
		fan54511_main_stop_charging();
		fan54511_sub_stop_charging();
	}

	return status;
 }

static unsigned int charging_enable(void *data)
{
            battery_log(BAT_LOG_CRTI, "%s: enable = %d\n",__FUNCTION__,*(unsigned int*)(data));
            
 	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int*)(data);

	if(KAL_TRUE == enable)
	{
		printk("%s:*************************\n", __func__);
		fan54511_main_start_charging();
            	fan54511_sub_start_charging();
	}
	else
	{
		fan54511_main_stop_charging();
		fan54511_sub_stop_charging();
	}

	return status;
}

static unsigned int charging_set_cv_voltage(void *data)
{
 	unsigned int status = STATUS_OK;
	unsigned short register_value;
	unsigned int cv_value = *(unsigned int *)(data);	

	if (cv_value < 3300000)
		cv_value = 3300000;
	else if (cv_value > 4720000)
		cv_value = 4720000;
	battery_log(BAT_LOG_CRTI, "%s: cv voltage = %d\n",__FUNCTION__,cv_value);
	register_value = (cv_value - 3300000)/10000;	
	status = fan54511_set_VFLOAT(register_value);

	return status;
}


static unsigned int charging_get_current(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned char ret_val=0;

    battery_log(BAT_LOG_CRTI, "%s\n",__FUNCTION__);

    *(unsigned int *)data = ret_val;

    return status;
}


static unsigned int charging_set_current(void *data)
{
            battery_log(BAT_LOG_CRTI, "%s\n",__FUNCTION__);
        
 	unsigned int status = STATUS_OK;
	unsigned int current_value = (*(unsigned int *)data/100);


          printk("%s:**********current_value %d mA  ***************\n", __func__, current_value);

	fan54511_main_set_IOCHRG(current_value);
	fan54511_sub_set_IOCHRG(current_value);

	return status;
}

static unsigned int charging_set_onlyone_input_current(void *data)
{
 	unsigned int status = STATUS_OK;
	unsigned int set_chr_current = *(unsigned int *)data/100;

           printk("%s:**********input_current_value: %d mA***************\n", __func__, set_chr_current);
    
	fan54511_sub_stop_charging();
	fan54511_main_start_charging();
	fan54511_main_set_IBUSLIM(set_chr_current);

	return status;
}


static unsigned int charging_set_input_current(void *data)
{
 	unsigned int status = STATUS_OK;
	unsigned int set_chr_current = (*(unsigned int *)data/100);

            //set_chr_current = 2000;

           printk("%s:**********input_current_value: %d mA***************\n", __func__, set_chr_current);
           
	fan54511_main_set_IBUSLIM(set_chr_current);
    	fan54511_sub_set_IBUSLIM(set_chr_current);
	return status;
}

static unsigned int charging_get_charging_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned char reg_value;

	reg_value = fan54511_get_charging_status();

	if (reg_value == 0x3)	/* check if chrg done */
		*(unsigned int *) data = KAL_TRUE;
	else
		*(unsigned int *) data = KAL_FALSE;

	return status;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	unsigned int status = STATUS_OK;

	pr_info("charging_reset_watch_dog_timer\r\n");

	//bq25890_config_interface(bq25890_CON3, 0x1, 0x1, 6);	/* reset watchdog timer */

	return status;
}

static unsigned int charging_set_hv_threshold(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short int register_value;
	unsigned int voltage = *(unsigned int *) (data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
	pmic_set_register_value(MT6351_PMIC_RG_VCDT_HV_VTH, register_value);

	return status;
}


static unsigned int charging_get_hv_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_VCDT_HV_DET);
	return status;
}


static unsigned int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
	battery_log(BAT_LOG_CRTI, "bat exist for evb\n");
#else
	unsigned int val = 0;

	val = pmic_get_register_value(MT6351_PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(MT6351_PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(MT6351_PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif
	return status;
}

static signed int read_adc_v_charger(void *data)
{
#if defined(CONFIG_POWER_EXT)
	*(signed int *) (data) = 5001;
#else
	signed int val;

	val = PMIC_IMM_GetOneChannelValue(PMIC_AUX_VCDT_AP, *(signed int *) (data), 1);
	val =
		(((batt_meter_cust_data.r_charger_1 +
		batt_meter_cust_data.r_charger_2) * 100 * val) /
		batt_meter_cust_data.r_charger_2) / 100;
	*(signed int *) (data) = val;
#endif

	return STATUS_OK;
}

static unsigned int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;
           unsigned int val = 5;		/* set avg times */
#if defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 1;
	battery_log(BAT_LOG_CRTI, "chr exist for fpga\n");
#else
	*(kal_bool *) (data) = pmic_get_register_value(MT6351_PMIC_RGS_CHRDET);

            read_adc_v_charger(&val);
           if( val  > 3000)
                *(kal_bool *) (data) = 1;
#endif

	return status;
}


kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}


static unsigned int charging_get_charger_type(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else


	charging_type_det_done = KAL_FALSE;
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
	charging_type_det_done = KAL_TRUE;
	g_charger_type = *(CHARGER_TYPE *) (data);
		battery_log(BAT_LOG_CRTI, "[charging_get_charger_type] return g_charger_type = %d\n",g_charger_type);
#endif

	return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = KAL_FALSE;
#else
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
#endif

	return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");
//	kernel_restart("battery service reboot system");
#endif

	return status;
}

static unsigned int charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return status;
}

static unsigned int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	/*added dump_stack to see who the caller is */
	dump_stack();
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	//kernel_power_off();
#endif

	return status;
}

static unsigned int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_OK;

#if 0				/* #if defined(MTK_POWER_EXT_DETECT) */
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *) data = KAL_FALSE;
	else
		*(kal_bool *) data = KAL_TRUE;
#else
	*(kal_bool *) data = KAL_FALSE;
#endif

	return status;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

#define TA_CURRENT_L 100
#define TA_CURRENT_H 900
//CURRENT is higher than 550mA is "1"
//CURRENT is lower than 130mA is "0"

static unsigned int charging_set_ta_current_pattern(void *data)
{
	kal_bool pumpup;
	unsigned int increase;

	pumpup = *(kal_bool *) (data);
	if (pumpup == KAL_TRUE)
		increase = 1;
	else
		increase = 0;
	/*unsigned int charging_status = KAL_FALSE; */

            fan54511_main_set_IOCHRG(1900);

	if (increase == KAL_TRUE) {
		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 1");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 1");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 2");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 2");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 3");
		msleep(281);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 3");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 4");
		msleep(281);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 4");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 5");
		msleep(281);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 5");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 6");
		msleep(485);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 6");
		msleep(50);

		battery_log(BAT_LOG_CRTI, "mtk_ta_increase() end\n");

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		msleep(200);
	} else {
		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 1");
		msleep(281);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 1");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 2");
		msleep(281);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 2");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 3");
		msleep(281);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 3");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 4");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 4");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 5");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 5");
		msleep(85);

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 6");
		msleep(485);

		fan54511_main_set_IBUSLIM(TA_CURRENT_L);	/* TA_CURRENT_L mA */
		battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 6");
		msleep(50);

		battery_log(BAT_LOG_CRTI, "mtk_ta_decrease() end\n");

		fan54511_main_set_IBUSLIM(TA_CURRENT_H);	/* TA_CURRENT_H mA */
	}

	return STATUS_OK;
}

static unsigned int charging_set_vindpm(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int v = *(unsigned int *) data;

	//bq25890_set_VINDPM(v);

	return status;
}

static unsigned int charging_set_vbus_ovp_en(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int e = *(unsigned int *) data;

	pmic_set_register_value(MT6351_PMIC_RG_VCDT_HV_EN, e);

	return status;
}

static unsigned int charging_get_bif_vbat(void *data)
{
	unsigned int status = STATUS_OK;

	return status;
}

static unsigned int charging_get_bif_tbat(void *data)
{
	unsigned int status = STATUS_OK;

	return status;
}

static unsigned int charging_diso_init(void *data)
{
	unsigned int status = STATUS_OK;

	return status;
}

static unsigned int charging_get_diso_state(void *data)
{
	unsigned int status = STATUS_OK;

	return status;
}

static unsigned int charging_get_error_state(void)
{
	return charging_error;
}

static unsigned int charging_set_error_state(void *data)
{
	unsigned int status = STATUS_OK;

	charging_error = *(unsigned int *) (data);

	return status;
}

static unsigned int charging_set_chrind_ck_pdn(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int pwr_dn;

	pwr_dn = *(unsigned int *) data;

	pmic_set_register_value(PMIC_RG_DRV_CHRIND_CK_PDN, pwr_dn);

	return status;
}

static unsigned int charging_sw_init(void *data)
{
	unsigned int status = STATUS_OK;
	/*put here anything needed to be init upon battery_common driver probe*/

	return status;
}

static unsigned int charging_enable_safetytimer(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int en;

	en = *(unsigned int *) data;
	//bq25890_en_chg_timer(en);

	return status;
}

static unsigned int charging_set_hiz_swchr(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int en;

	en = *(unsigned int *) data;
	fan54511_main_set_HZMODE(en);
	fan54511_sub_set_HZMODE(en);

	return status;
}

static unsigned int (*const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
    charging_hw_init
    ,charging_dump_register
    ,charging_enable
    ,charging_set_cv_voltage
    ,charging_get_current
    ,charging_set_current
    ,charging_set_input_current
    ,charging_get_charging_status
    ,charging_reset_watch_dog_timer
    ,charging_set_hv_threshold
    ,charging_get_hv_status
    ,charging_get_battery_status
    ,charging_get_charger_det_status
    ,charging_get_charger_type
    ,charging_get_is_pcm_timer_trigger
    ,charging_set_platform_reset
    ,charging_get_platform_boot_mode
    ,charging_set_power_off
    ,charging_get_power_source
    ,charging_get_csdac_full_flag
    ,charging_set_ta_current_pattern
    ,charging_set_error_state
    ,charging_diso_init
    ,charging_get_diso_state
    ,charging_set_vindpm
    ,charging_set_vbus_ovp_en
    ,charging_get_bif_vbat
    ,charging_set_chrind_ck_pdn
    ,charging_sw_init
    ,charging_enable_safetytimer
    ,charging_set_hiz_swchr
    ,charging_get_bif_tbat
    ,charging_set_onlyone_input_current
    ,charging_enable_major
    ,charging_enable_minor
};

/*
* FUNCTION
*        Internal_chr_control_handler
*
* DESCRIPTION
*         This function is called to set the charger hw
*
* CALLS
*
* PARAMETERS
*        None
*
* RETURNS
*
*
* GLOBALS AFFECTED
*       None
*/
signed int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	signed int status;

	if (cmd < CHARGING_CMD_NUMBER) {
		if (charging_func[cmd] != NULL)
			status = charging_func[cmd](data);
		else {
			battery_log(BAT_LOG_CRTI, "[chr_control_interface]cmd:%d not supported\n", cmd);
			status = STATUS_UNSUPPORTED;
		}
	} else
		status = STATUS_UNSUPPORTED;

	return status;
}
