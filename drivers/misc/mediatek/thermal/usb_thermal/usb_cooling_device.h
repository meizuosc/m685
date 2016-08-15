#ifndef __USB_COOLING_DEVICE_H__
#define __USB_COOLING_DEVICE_H__

#include <linux/kernel.h>

extern int usb_thermal_get_temp(void);

int enable_usb_cooling(void);

int diable_usb_cooling_by_async(void);

#define CONFIG_DISABLE_CHARGE_TEMP 60
#define CONFIG_REOPEN_CHARGE_TEMP 45

#endif /*__USB_COOLING_DEVICE_H__*/
