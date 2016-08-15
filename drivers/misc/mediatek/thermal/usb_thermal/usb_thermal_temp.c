
extern int usb_thermal_get_temp_for_custom(void);
extern int usb_thermal_get_temp_for_mp(void);
int get_hw_version(void);
int flashlight_get_vendor(void);

static int is_mp_device(void)
{
#define M95_HW_VERSION_B1 0x3
#define M95_HW_VERSION_B2 0x4
#define M95_HW_VERSION_NPI_OR_MP 0x5
#define M95_HW_VERSION_B1_TELCOM 0xC
	int hw_version = 0x00;
	int led_version = flashlight_get_vendor();

	if(hw_version == M95_HW_VERSION_B1 ||
		hw_version == M95_HW_VERSION_B2 ||
		hw_version == M95_HW_VERSION_B1_TELCOM) {

		return 0;
	}

	if(hw_version == M95_HW_VERSION_NPI_OR_MP && 
		led_version == 0) {
		return 0;
	}

	return 1;
}

int usb_thermal_get_temp(void)
{
#define IS_MP_MAGIC_UNUSED 3
	static int is_mp = IS_MP_MAGIC_UNUSED;
	int usb_temp = 0;

	if(is_mp == IS_MP_MAGIC_UNUSED) {
		is_mp = is_mp_device();
	}

	if(is_mp) {
		usb_temp = usb_thermal_get_temp_for_mp();
	} else {
		usb_temp = usb_thermal_get_temp_for_custom();
	}
	return usb_temp;
}
