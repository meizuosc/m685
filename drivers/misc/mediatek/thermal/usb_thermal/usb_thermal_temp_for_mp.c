#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "../../auxadc/mt_auxadc.h"

#include "usb_cooling_device.h"

#define INT16 int16_t
#define INT32 int32_t
#define UINT32 uint32_t

#define AUX_IN3_NTC (3)     // usb board NTC channel
static int g_RAP_ADC_channel_3 = AUX_IN3_NTC;

static int g_usb_RAP_pull_up_R = 390000;	/* 390K,pull up resister */
static int g_usb_TAP_over_critical_low = 4397119;	/* base on 10K NTC temp default value -40 deg */
static int g_usb_RAP_pull_up_voltage = 1800;	/* 1.8V ,pull up voltage */

typedef struct {
	INT32 BTS_Temp;
	INT32 TemperatureR;
} BTS_TEMPERATURE;

#define mtkts_bts_dprintk(...) do{;}while(0)

static BTS_TEMPERATURE BTS_usb_Temperature_Table[] = {
	{-40, 4397119},
	{-35, 3088598},
	{-30, 219722},
	{-25, 1581880},
	{-20, 1151036},
	{-15, 846578},
	{-10, 628988},
	{-5, 471632},
	{0, 357011},
	{5, 272499},
	{10, 209709},
	{15, 162650},
	{20, 127080},
	{25, 100000},		/* 100K */
	{30, 79221},
	{35, 63167},
	{40, 50676},
	{45, 40903},
	{50, 33194},
	{55, 27090},
	{60, 22224},
	{65, 18322},
	{70, 15184},
	{75, 12635},
	{80, 10565},
	{85, 8872},
	{90, 7481},
	{95, 6336},
	{100, 5383},
	{105, 4594},
	{110, 3934},
	{115, 3380},
	{120, 2916},
	{125, 252}
};

static INT16 mtkts_bts_usb_thermistor_conver_temp(INT32 Res)
{
	int i = 0;
	int asize = 0;
	INT32 RES1 = 0, RES2 = 0;
	INT32 TAP_Value = -200, TMP1 = 0, TMP2 = 0;

	asize = (sizeof(BTS_usb_Temperature_Table) / sizeof(BTS_TEMPERATURE));
	/* mtkts_bts_dprintk("mtkts_bts_usb_thermistor_conver_temp() : asize = %d, Res = %d\n",asize,Res); */
	if (Res >= BTS_usb_Temperature_Table[0].TemperatureR) {
		TAP_Value = -40;	/* min */
	} else if (Res <= BTS_usb_Temperature_Table[asize - 1].TemperatureR) {
		TAP_Value = 125;	/* max */
	} else {
		RES1 = BTS_usb_Temperature_Table[0].TemperatureR;
		TMP1 = BTS_usb_Temperature_Table[0].BTS_Temp;
		/* mtkts_bts_dprintk("%d : RES1 = %d,TMP1 = %d\n",__LINE__,RES1,TMP1); */

		for (i = 0; i < asize; i++) {
			if (Res >= BTS_usb_Temperature_Table[i].TemperatureR) {
				RES2 = BTS_usb_Temperature_Table[i].TemperatureR;
				TMP2 = BTS_usb_Temperature_Table[i].BTS_Temp;
				/* mtkts_bts_dprintk("%d :i=%d, RES2 = %d,TMP2 = %d\n",__LINE__,i,RES2,TMP2); */
				break;
			}
			RES1 = BTS_usb_Temperature_Table[i].TemperatureR;
			TMP1 = BTS_usb_Temperature_Table[i].BTS_Temp;
			/* mtkts_bts_dprintk("%d :i=%d, RES1 = %d,TMP1 = %d\n",__LINE__,i,RES1,TMP1); */
		}

		TAP_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2)) / (RES1 - RES2);
	}

#if 0
	printk("[usb_cooling]mtkts_bts_usb_thermistor_conver_temp() : TAP_Value = %d\n", TAP_Value);
	printk("[usb_cooling]mtkts_bts_usb_thermistor_conver_temp() : Res = %d\n", Res);
	printk("[usb_cooling]mtkts_bts_usb_thermistor_conver_temp() : RES1 = %d\n", RES1);
	printk("[usb_cooling]mtkts_bts_usb_thermistor_conver_temp() : RES2 = %d\n", RES2);
	printk("[usb_cooling]mtkts_bts_usb_thermistor_conver_temp() : TMP1 = %d\n", TMP1);
	printk("[usb_cooling]mtkts_bts_usb_thermistor_conver_temp() : TMP2 = %d\n", TMP2);
#endif

	return TAP_Value;
}


static INT16 mtk_ts_bts_usb_volt_to_temp(UINT32 dwVolt)
{
	INT32 TRes;
	INT32 dwVCriAP = 0;
	INT32 BTS_TMP = -100;

	/* SW workaround----------------------------------------------------- */
	/* dwVCriAP = (TAP_OVER_CRITICAL_LOW * 1800) / (TAP_OVER_CRITICAL_LOW + 10000); */
	/* dwVCriAP = (TAP_OVER_CRITICAL_LOW * RAP_PULL_UP_VOLT) / (TAP_OVER_CRITICAL_LOW + RAP_PULL_UP_R); */
	dwVCriAP =
	    (g_usb_TAP_over_critical_low * g_usb_RAP_pull_up_voltage) / (g_usb_TAP_over_critical_low +
								 g_usb_RAP_pull_up_R);

	if (dwVolt > dwVCriAP) {
		TRes = g_usb_TAP_over_critical_low;
	} else {
		/* TRes = (10000*dwVolt) / (1800-dwVolt); */
		/* TRes = (RAP_PULL_UP_R*dwVolt) / (RAP_PULL_UP_VOLT-dwVolt); */
		TRes = (g_usb_RAP_pull_up_R * dwVolt) / (g_usb_RAP_pull_up_voltage - dwVolt);
	}
	/* ------------------------------------------------------------------ */

	//g_AP_TemperatureR = TRes;

	/* convert register to temperature */
	BTS_TMP = mtkts_bts_usb_thermistor_conver_temp(TRes);

	return BTS_TMP;
}

static int get_hw_usb_board_temp(void)
{

	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, output;
	int times = 1, Channel = g_RAP_ADC_channel_3;	/* 6752=0(AUX_IN3_NTC) */
	static int valid_temp;

	if (IMM_IsAdcInitReady() == 0) {
		printk("[usb_cooling][thermal_auxadc_get_data]: AUXADC is not ready\n");
		return 0;
	}

	i = times;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		if (ret_value == -1) {/* AUXADC is busy */
			ret_temp = valid_temp;
		} else {
			valid_temp = ret_temp;
		}
		ret += ret_temp;
	}

	/* Mt_auxadc_hal.c */
	/* #define VOLTAGE_FULL_RANGE  1500 // VA voltage */
	/* #define AUXADC_PRECISE      4096 // 12 bits */
	ret = ret * 1500 / 4096;
	/* ret = ret*1800/4096;//82's ADC power */
	output = mtk_ts_bts_usb_volt_to_temp(ret);
	printk("[usb_cooling]USB board output mv = %d, temperature = %d\n", ret, output);
	return output;
}

int usb_thermal_get_temp_for_mp(void)
{
	return get_hw_usb_board_temp();
}

