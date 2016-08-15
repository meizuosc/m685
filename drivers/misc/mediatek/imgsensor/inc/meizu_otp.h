
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 meizu_otp.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *       meizu add for otp struct define
 *       author:lcz
 ****************************************************************************/
#ifndef _MEIZU_OTP_H
#define _MEIZU_OTP_H

#define S5K4E6_LSC_SIZE 452

typedef struct s5k4e6_otp_struct{
	int module_id;
	int AWB_R;
	int AWB_B;
	int AWB_Gr_Gb;
	u8  lsc_data[S5K4E6_LSC_SIZE];
}s5k4e6_otp_struct;

typedef struct ov5695_otp_struct{
	int flag;
	int module_id;
	int lens_id;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}ov5695_otp_struct;

#endif
