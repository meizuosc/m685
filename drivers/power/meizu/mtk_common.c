#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <mach/mt_sleep.h>
#include <mach/mt_gpt.h>
#include <linux/meizu-sys.h>
#include <linux/sched/rt.h>
#include <linux/wakelock.h>
#include <linux/power/bq2589x_reg.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/mtk_thermal_typedefs.h>
#include <mt-plat/battery_meter_hal.h>
#include <mt-plat/charging.h>
#include <linux/power/bq27532_battery.h>
#include <mt-plat/upmu_common.h>


kal_uint32 g_usb_state = USB_UNCONFIGURED;
int chg_hw_init_done = KAL_FALSE;

void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	printk("[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
#else
	if ((usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))) {
		usb_state_value = USB_UNCONFIGURED;
	} else {
		g_usb_state = usb_state_value;
	}
#endif
}

bool upmu_is_chr_det(void)
{
	kal_bool chr_status = KAL_FALSE;
	unsigned int pmic_chgin = 0;
	int bq2589x_chgin = 0;

	if (chg_hw_init_done == KAL_TRUE) {
		bq2589x_chgin = bq2589x_get_vbus_status();
		pmic_chgin = pmic_get_register_value(MT6351_PMIC_RGS_CHRDET);
	}
	if (bq2589x_chgin == 1 || pmic_chgin == 1)
		chr_status = KAL_TRUE;
	else
		chr_status = KAL_FALSE;
	return chr_status;
}
EXPORT_SYMBOL(upmu_is_chr_det);

kal_bool pmic_chrdet_status(void)
{
	if (upmu_is_chr_det() == KAL_TRUE) {
		return KAL_TRUE;
	} else {
		return KAL_FALSE;
	}
}
EXPORT_SYMBOL(pmic_chrdet_status);

kal_bool bat_is_charger_exist(void)
{
	return upmu_is_chr_det();
}

int read_tbat_value(void)
{
	return 25;
}

int get_bat_charging_current_level(void)
{
	CHR_CURRENT_ENUM charging_current;
	
	charging_current = 0;

	return charging_current;
}

kal_uint32 set_bat_charging_current_limit(int current_limit)
{
	return current_limit;
}

kal_uint32 bat_get_ui_percentage(void)
{
	return bq27532_battery_read_soc();
}

void  tbl_charger_otg_vbus(kal_uint32 mode)
{
	kal_uint32 state = mode&0x1;

	bq2589x_boost_mode_enable(state);
}
EXPORT_SYMBOL(tbl_charger_otg_vbus);

CHARGER_TYPE mt_get_charger_type(void)
{
	return BMT_status.charger_type;
}
EXPORT_SYMBOL(mt_get_charger_type);
