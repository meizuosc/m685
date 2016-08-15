#include <mt-plat/charging.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mt-plat/mtk_thermal_typedefs.h>
#include <mt-plat/upmu_common.h>
#include <linux/power/bq2589x_reg.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/mt_boot_common.h>
#include <mach/mt_charging.h>
#include <mach/mt_pe.h>
#include <linux/power/bq27532_battery.h>

#define  AP_MAX_TEMP	40

#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

extern kal_uint32 upmu_get_rgs_chrdet(void);
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
struct wake_lock TA_charger_suspend_lock;
#endif
static DEFINE_MUTEX(ta_mutex);

const kal_uint32 VBAT_CV_VTH[]=
{
	3840000,    3856000,    3872000,    3888000,
	3904000,    3920000,    3936000,    3952000,
	3968000,    3984000,    4000000,    4016000,
	4032000,    4048000,    4064000,    4080000,
	4096000,    4112000,    4128000,    4144000,
	4160000,    4176000,    4192000,    4208000,
	4224000,    4240000,    4256000,    4272000,
	4288000,    4304000,    4320000,    4336000,
	4352000,    4368000,    4384000,    4400000,
	4416000,    4432000,    4448000,    4464000,
	4480000,    4496000,    4512000,    4528000,
	4544000,    4560000,    4576000,    4592000,
	4608000
};

const kal_uint32 CS_VTH[]=
{
	0,	6400,	12800,	19200,
	25600,  51200,  57600,  64000,
	70400,  76800,  83200,  89600,
	96000,  102400, 108800, 115200,
	121600, 128000, 134400, 140800,
	147200, 153600, 160000, 166400,
	172800, 179200, 185600, 192000,
	198400, 204800, 211200, 217600,
	224000, 230400, 236800, 243200,
	249600
}; 

 const kal_uint32 INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_200_00_MA,	CHARGE_CURRENT_300_00_MA,  CHARGE_CURRENT_400_00_MA,
	 CHARGE_CURRENT_500_00_MA,  CHARGE_CURRENT_600_00_MA,	CHARGE_CURRENT_700_00_MA,  CHARGE_CURRENT_800_00_MA,
	 CHARGE_CURRENT_900_00_MA,  CHARGE_CURRENT_1000_00_MA,	CHARGE_CURRENT_1100_00_MA,  CHARGE_CURRENT_1200_00_MA,
	 CHARGE_CURRENT_1300_00_MA,  CHARGE_CURRENT_1400_00_MA,	CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_1600_00_MA,
	 CHARGE_CURRENT_1700_00_MA, CHARGE_CURRENT_1800_00_MA,  CHARGE_CURRENT_1900_00_MA,  CHARGE_CURRENT_2000_00_MA,
	 CHARGE_CURRENT_2100_00_MA, CHARGE_CURRENT_2200_00_MA,  CHARGE_CURRENT_2300_00_MA,  CHARGE_CURRENT_2400_00_MA,
	 CHARGE_CURRENT_2500_00_MA, CHARGE_CURRENT_2600_00_MA,  CHARGE_CURRENT_2700_00_MA,  CHARGE_CURRENT_2800_00_MA,
	 CHARGE_CURRENT_2900_00_MA, CHARGE_CURRENT_3000_00_MA,  CHARGE_CURRENT_3100_00_MA
 }; 

const unsigned int VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	BATTERY_VOLT_04_350000_V, BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,
	BATTERY_VOLT_04_500000_V, BATTERY_VOLT_04_550000_V, BATTERY_VOLT_04_600000_V,
	BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V, BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	BATTERY_VOLT_10_500000_V
};

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
				return pList[i];
			}
		}
		return pList[0];
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

 kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

	for(i=0;i<array_size;i++)
	{
		if ((val > *(parameter + i)) && (val < *(parameter + i +1)))
		{
			return i + 1;
		} else if (val == *(parameter + i)) {
			return i;
	    	}
	}
	return 0;
}

 static kal_uint32 charging_hw_init(void *data)
 {
 	kal_uint32 status = STATUS_OK;

	bq2589x_charge_init();
	return status;
}

static kal_uint32 charging_dump_register(void *data)
 {
 	kal_uint32 status = STATUS_OK;

	bq2589x_charge_dump_register();
   	
	return status;
 }

static kal_uint32 charging_enable_main(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable) {
		bq2589x_charge_enable_sedcond(0);
		bq2589x_charge_enable_main(enable);
	} else {
		bq2589x_charge_enable_main(0);
	}

	return status;
}

static kal_uint32 charging_mt_usb_connect(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if (enable)
		mt_usb_connect();
	else
		mt_usb_disconnect();
}

static kal_uint32 charging_input_current_optimizer(void *data)
{
 	kal_uint32 status = STATUS_OK;
	int vdpm_status, idpm_status;
	int ico_status;
	kal_uint32 charging_enable = KAL_TRUE;
	int idpm_limit = CHARGE_CURRENT_1500_00_MA;
	int input_value = CHARGING_CURRENT_ICO;
	int cc_value = CHARGING_CURRENT_ICO;

	printk("%s:The Adapter is not fast charger, so enable ICO\n", __func__);
	bq2589x_charge_ico_enable(1);
	bq2589x_charge_force_ico_set();	
	bq2589x_input_current_limit_main(input_value);
	bq2589x_fast_charge_limit_main(cc_value);
	bq2589x_charge_enable_main(1);

	msleep(1000);
	ico_status = bq2589x_get_ico_status();
	if (ico_status == 0) {
		msleep(500);
		ico_status = bq2589x_get_ico_status();
	}

	if (ico_status) {
		idpm_limit = bq2589x_get_idpm_limit();
		printk("%s:ico_status %d, idpm_limit %d\n", __func__, ico_status, idpm_limit);
		
		if (idpm_limit <= CHARGE_CURRENT_1400_00_MA / 100) {
			printk("%s:It is 5V/1.2A Charger\n", __func__);
			input_value = ADAPTER_OUTPUT_1_2A;
		} else if (idpm_limit <= CHARGE_CURRENT_1700_00_MA / 100) {
			printk("%s:It is 5V/1.5A Charger\n", __func__);
			input_value = ADAPTER_OUTPUT_1_5A;
		} else {
			printk("%s:It is 5V/2A Charger\n", __func__);
			input_value = ADAPTER_OUTPUT_2_0A;
		}
	} else {
		input_value = ADAPTER_OUTPUT_0A; 
	}
	*(CHR_CURRENT_ENUM*)(data) = input_value;

	return status;
}

static kal_uint32 charging_enable(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	bq2589x_charge_enable_sedcond(enable);
	bq2589x_charge_enable_main(enable);

	return status;
}

 static kal_uint32 charging_set_cv_voltage(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);	
	
	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), cv_value);
	bq2589x_charge_voltage_limit(register_value);

	return status;
 } 	

 static kal_uint32 charging_set_main_charger_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 current_value = *(kal_uint32 *)data;

	bq2589x_fast_charge_limit_main(current_value);

	return status;
 }

 static kal_uint32 charging_set_charger_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 current_value = *(kal_uint32 *)data;

	bq2589x_fast_charge_limit_second(current_value);
	bq2589x_fast_charge_limit_main(current_value);

	return status;
 } 	

 static kal_uint32 charging_set_input_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 input_current = *(kal_uint32 *)data;
    
	bq2589x_input_current_limit_second(input_current);
	bq2589x_input_current_limit_main(input_current);

	return status;
 } 	

 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = bq2589x_get_charging_status();
	
	*(kal_uint32 *)data = ret_val;
	
	return status;
 } 	

 static kal_uint32 charging_reset_watch_dog_timer(void *data)
 {
	 kal_uint32 status = STATUS_OK;
	 kal_uint32 rest_enable = *(kal_uint32*)(data);
	 
	if (rest_enable == KAL_TRUE) {
	     bq2589x_i2c_watchdog_timer_set(BQ2589X_WDT_80S);
	     bq2589x_i2c_watchdog_reset();
	} else {
	     bq2589x_i2c_watchdog_timer_set(BQ2589X_WDT_DISABLE);
	}

     return status;
 }

static unsigned int is_chr_det(void)
{
	unsigned int val = 0;

	val = pmic_get_register_value(MT6351_PMIC_RGS_CHRDET);

	return val;
}

static kal_uint32 charging_get_charger_det_status(void *data)
{
	kal_uint32 status = STATUS_OK;
	int vbus_stat = 0, pg_stat = 0;
	int mtk_chgin = 0;

	vbus_stat = bq2589x_get_vbus_status();
	pg_stat = bq2589x_get_pg_status();
	mtk_chgin = is_chr_det();
	
	if ((vbus_stat != 0 || pg_stat == 1 || mtk_chgin == 1) &&
			(vbus_stat != CHARGER_OTG_MODE)) {
		*(kal_bool *) (data) = KAL_TRUE;
	} else 
		*(kal_bool *) (data) = KAL_FALSE;

	return status;
}

 static kal_uint32 charging_get_charger_type(void *data)
 {
	 kal_uint32 status = STATUS_OK;

	*(CHARGER_TYPE*)(data) = hw_charging_get_charger_type();

	 return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
   *(kal_uint32*)(data) = get_boot_mode();
         
    return status;
}

static void swchr_flow_normal(kal_uint32 chr_cur_val)
{
	bq2589x_charge_enable_main(1); //ENABLE I2C2
	bq2589x_charge_enable_sedcond(0); //ENABLE I2C2

	bq2589x_input_current_limit_main(chr_cur_val);
}

static void ta_set_chr_current(kal_uint32 chr_current)
{
  	swchr_flow_normal(chr_current);
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 increase = *(kal_uint32 *)data;

	charge_set_current_pattern(increase);
	return status;
}

static kal_uint32 charging_set_main_input_current(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current = *(kal_uint32 *)data;
    
	bq2589x_input_current_limit_main(set_chr_current);

	return status;
}

static kal_uint32 charging_set_adc_start(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	bq2589x_adc_start(enable);

	return status;
}

static kal_uint32 charging_set_adc_stop(void *data)
{
 	kal_uint32 status = STATUS_OK;

	bq2589x_adc_stop();

	return status;
}

static void set_qc_charger_current(void)
{
	int input_current = 0;
	input_current = TA_AC_12V_INPUT_CURRENT;  //TA = 12V		

	bq2589x_input_current_limit_second(input_current);
	bq2589x_input_current_limit_main(input_current);
}

static void battery_qc_init(void) 
{
	unsigned int input_current = CHARGE_CURRENT_600_00_MA;

	bq2589x_charge_ico_enable(0);
	/*adc enable*/
	bq2589x_adc_start(0);
	ta_set_chr_current(input_current);
}

static int charger_increase_voltage(void)
{
	kal_bool retransmit = KAL_TRUE;
	kal_uint32 retransmit_count=0;
	kal_bool increase = KAL_TRUE;  // TRUE = increase
	kal_bool connect = KAL_FALSE;

	wake_lock(&TA_charger_suspend_lock);
	int charger_vol = bq2589x_adc_read_charger_volt();
	if (charger_vol > VOLTAGE_7V_SUPPORT && BMT_status.qc_check_flag == KAL_FALSE) {
		msleep(800);
		charger_vol = bq2589x_adc_read_charger_volt();
	}
	while (BMT_status.charger_exist == KAL_TRUE && retransmit == KAL_TRUE &&
			charger_vol < BMT_status.target_voltage && !vbus_change_event) {
		bq2589x_pumpx_enable(increase);
		bq2589x_pumpx_up();
		wait_event_interruptible_timeout(vbus_wq, (vbus_change_event == KAL_TRUE) ,2400*HZ/1000);
		if (vbus_change_event == KAL_TRUE) {
			retransmit = KAL_FALSE;
			wake_unlock(&TA_charger_suspend_lock);
			return 0;
		}
		charger_vol = bq2589x_adc_read_charger_volt();

		if (charger_vol > VOLTAGE_7V_SUPPORT) {
			BMT_status.qc_connect = KAL_TRUE;
		}

		if (charger_vol > BMT_status.target_voltage) {
			retransmit = KAL_FALSE;
			connect = KAL_TRUE;
		} else {
			retransmit_count++;
		}

		if(retransmit_count == 10 || BMT_status.charger_exist == KAL_FALSE) {
			retransmit = KAL_FALSE;
		}
		BMT_status.qc_check_flag = KAL_TRUE;
		printk("%s:charger_vol %d, retry=%d\n",__func__,charger_vol, retransmit_count);
	}
	charger_vol = bq2589x_adc_read_charger_volt();
	if (charger_vol > BMT_status.target_voltage - 1000) {
		set_qc_charger_current();
	}

	wake_unlock(&TA_charger_suspend_lock);
	return connect;
}

static int charger_down_voltage(void)
{
	kal_bool retransmit = KAL_TRUE;
	kal_uint32 retransmit_count=0;
	kal_bool connect = KAL_FALSE;

	wake_lock(&TA_charger_suspend_lock);
	int charger_vol = bq2589x_adc_read_charger_volt();
	
	while (BMT_status.charger_exist == KAL_TRUE && !vbus_change_event &&
			(charger_vol > (BMT_status.target_voltage + 1000))) {
		if (retransmit_count++ > 10)
			break;

		bq2589x_pumpx_enable(true);
		bq2589x_pumpx_down();
		wait_event_interruptible_timeout(vbus_wq, (vbus_change_event == KAL_TRUE) ,2400*HZ/1000);
		if (vbus_change_event == KAL_TRUE) {
			retransmit = KAL_FALSE;
			wake_unlock(&TA_charger_suspend_lock);
			return 0;
		}
		charger_vol = bq2589x_adc_read_charger_volt();

		printk("%s:charger_vol %d, retry=%d\n",__func__,charger_vol, retransmit_count);
	}

	wake_unlock(&TA_charger_suspend_lock);
	return connect;
}

static kal_uint32 charging_pumpx_up(void *data)
{
	kal_uint32 status = STATUS_OK;
	bool increase = true;
	int charger_vol = 0;

	mutex_lock(&ta_mutex);
	battery_qc_init();
	
	charger_vol = bq2589x_adc_read_charger_volt();
	if ((charger_vol > (BMT_status.target_voltage + 1000)) &&
			BMT_status.qc_connect == KAL_TRUE)
		increase = false;

	if (increase == true)
		charger_increase_voltage();
	else 
		charger_down_voltage();

	mutex_unlock(&ta_mutex);
	return status;
}

static kal_uint32 charging_reset_vchr(void *data)
{
 	kal_uint32 status = STATUS_OK;
	unsigned int input_current = CHARGE_CURRENT_1000_00_MA;

	ta_set_chr_current(input_current);
	mtk_ta_reset_vchr();

	return status;
}

static kal_uint32 charging_get_batterytemp(void *data)
{
 	kal_uint32 status = STATUS_OK;
	signed int temperature;

	temperature = bq27532_battery_read_temperature();
	*(signed int*)(data) = temperature;

	return status;
}

static kal_uint32 charging_get_batteryvoltage(void *data)
{
 	kal_uint32 status = STATUS_OK;
	unsigned int bat_vol;

	bat_vol = bq27532_battery_voltage(); 

	*(unsigned int*)(data) = bat_vol;

	return status;
}

static kal_uint32 charging_get_batterycurrent(void *data)
{
 	kal_uint32 status = STATUS_OK;
	signed int avg_current;

	avg_current = bq27532_battery_current();

	*(signed int*)(data) = avg_current;

	return status;
}

static kal_uint32 charging_get_batterycurrent_now(void *data)
{
 	kal_uint32 status = STATUS_OK;
	signed int now_current;

	now_current = bq27532_battery_current_now();

	*(signed int*)(data) = now_current;

	return status;
}

static kal_uint32 charging_get_batterysoc(void *data)
{
 	kal_uint32 status = STATUS_OK;
	signed int soc;

	soc = bq27532_battery_read_soc();
	*(signed int*)(data) = soc;

	return status;
}

static kal_uint32 charging_check_batterycv(void *data)
{
 	kal_uint32 status = STATUS_OK;
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_350000_V;
	int voltage = 0;
	
	if (BMT_status.batt_status == TEMP_POS_T15 || BMT_status.batt_status == TEMP_POS_T20) {
		if (BMT_status.battery_id == 0) { //ATL
			cv_voltage = BATTERY_VOLT_04_200000_V;
		} else if (BMT_status.battery_id == 1) { //SDI
			cv_voltage = BATTERY_VOLT_04_100000_V;
		} else { //LG
			cv_voltage = BATTERY_VOLT_04_200000_V;
		}

		voltage = bq27532_battery_voltage();
		if (voltage >= cv_voltage/1000) {
			*(kal_bool*)(data) = KAL_TRUE;
		} else  if (voltage < ((cv_voltage - BATTERY_VOLT_01_000000_V) / 1000)) {
			*(kal_bool*)(data) = KAL_FALSE;
		} else {
			*(kal_bool*)(data) = KAL_FALSE;
		}
		printk("%s:cv_change %d\n", __func__, *(kal_bool*)(data));
	} else {
		*(kal_bool*)(data) = KAL_FALSE;
	}

	return status;
}


/*
1 :  host mode
0 :  usb device mode
*/
 static kal_uint32 charging_get_usb_mode(void *data)
 {
	
	(*(kal_uint32 *)data)= !mt_usb_is_device();
	return 0;
}

static unsigned int charging_set_vindpm(void *data)
{
	unsigned int status = STATUS_OK;
	int vindpm = 0;

	/*when VBUS<VBAT+120mV(max),charger LDO off, enter HIZ mode,
	 when VBUS>VBAT+(130mV-270mV)，LDO will continue work, exit HIZ mode
	 M95 CV is 4.35V, so when the VBUS < (4.35+0.12=)4.47 will enter HIZ MODE.
	 */
	if (BMT_status.SOC < 80) {
		vindpm = 0x11;//4.3V
	} else
		 vindpm = 0x13; //4.5V

	bq2589x_vindpm_threshold_set(vindpm);

	return status;
}

static unsigned int charging_set_vbus_ovp_en(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int e = *(unsigned int *) data;

	pmic_set_register_value(MT6351_PMIC_RG_VCDT_HV_EN, e);

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


 static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_enable_main
	,charging_set_cv_voltage
	,charging_set_main_charger_current
	,charging_set_charger_current
	,charging_set_input_current
	,charging_set_main_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_platfrom_boot_mode
	,charging_set_ta_current_pattern
	,charging_set_adc_start
	,charging_set_adc_stop
	,charging_mt_usb_connect
	,charging_input_current_optimizer
	,charging_pumpx_up
	,charging_reset_vchr
	,charging_get_batterytemp
	,charging_get_batteryvoltage
	,charging_get_batterycurrent
	,charging_get_batterycurrent_now
	,charging_get_batterysoc
	,charging_check_batterycv
	,charging_get_usb_mode
	,charging_set_vindpm
	,charging_set_vbus_ovp_en
	,charging_set_hv_threshold
	,charging_get_hv_status
 };

 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
 kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 kal_int32 status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
 
	 return status;
 }


