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

static int g_usb_RAP_pull_up_R = 10000;	/* 10K,pull up resister */
static int g_usb_TAP_over_critical_low = 195652;	/* base on 10K NTC temp default value -40 deg */
static int g_usb_RAP_pull_up_voltage = 1800;	/* 1.8V ,pull up voltage */

typedef struct {
	INT32 BTS_Temp;
	INT32 TemperatureR;
} BTS_TEMPERATURE;

#define mtkts_bts_dprintk(...) do{;}while(0)

static BTS_TEMPERATURE BTS_usb_Temperature_Table[] = {
	{-40, 195652},		/* FIX_ME */
	{-35, 148171},		/* FIX_ME */
	{-30, 113347},		/* FIX_ME */
	{-25, 87558},		/* FIX_ME */
	{-20, 68236},
	{-15, 53649},
	{-10, 42506},
	{-5, 33892},
	{0, 27218},
	{5, 22021},
	{10, 17925},
	{15, 14673},
	{20, 12080},
	{25, 10000},		/* 10K */
	{30, 8314},
	{35, 6947},
	{40, 5833},
	{45, 4916},
	{50, 4160},
	{55, 3535},
	{60, 3014},		/* FIX_ME */
	{65, 2586},		/* FIX_ME */
	{70, 2227},		/* FIX_ME */
	{75, 1924},		/* FIX_ME */
	{80, 1668},		/* FIX_ME */
	{85, 1452},		/* FIX_ME */
	{90, 1268},		/* FIX_ME */
	{95, 1109},		/* FIX_ME */
	{100, 973},		/* FIX_ME */
	{105, 858},		/* FIX_ME */
	{110, 758},		/* FIX_ME */
	{115, 671},		/* FIX_ME */
	{120, 596},		/* FIX_ME */
	{125, 531}		/* FIX_ME */
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
	mtkts_bts_dprintk("mtkts_bts_usb_thermistor_conver_temp() : TAP_Value = %d\n", TAP_Value);
	mtkts_bts_dprintk("mtkts_bts_usb_thermistor_conver_temp() : Res = %d\n", Res);
	mtkts_bts_dprintk("mtkts_bts_usb_thermistor_conver_temp() : RES1 = %d\n", RES1);
	mtkts_bts_dprintk("mtkts_bts_usb_thermistor_conver_temp() : RES2 = %d\n", RES2);
	mtkts_bts_dprintk("mtkts_bts_usb_thermistor_conver_temp() : TMP1 = %d\n", TMP1);
	mtkts_bts_dprintk("mtkts_bts_usb_thermistor_conver_temp() : TMP2 = %d\n", TMP2);
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
		pr_debug("[thermal_auxadc_get_data]: AUXADC is not ready\n");
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
		mtkts_bts_dprintk("[thermal_auxadc_get_data(AUX_IN3_NTC)]: ret_temp=%d\n",
				  ret_temp);
	}

	/* Mt_auxadc_hal.c */
	/* #define VOLTAGE_FULL_RANGE  1500 // VA voltage */
	/* #define AUXADC_PRECISE      4096 // 12 bits */
	ret = ret * 1500 / 4096;
	/* ret = ret*1800/4096;//82's ADC power */
	mtkts_bts_dprintk("USB board output mV = %d\n", ret);
	output = mtk_ts_bts_usb_volt_to_temp(ret);
	mtkts_bts_dprintk("USB board output temperature = %d\n", output);
	return output;
}

int usb_thermal_get_temp_for_custom(void)
{
	return get_hw_usb_board_temp();
}

