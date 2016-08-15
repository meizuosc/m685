#ifndef FAN5451X_REG_H
#define FAN5451X_REG_H

/* register definition */
#define FAN54511_REG_STATUS0		0x00
#define FAN54511_REG_STATUS1		0x01
#define FAN54511_REG_STATUS2		0x02

#define FAN54511_REG_INT0		0x04
#define FAN54511_REG_INT1		0x05
#define FAN54511_REG_INT2		0x06

#define FAN54511_REG_MINT0		0x08
#define FAN54511_REG_MINT1		0x09
#define FAN54511_REG_MINT2		0x0A

#define FAN54511_REG_CONTROL0		0x0C
#define FAN54511_REG_CONTROL1		0x0D
#define FAN54511_REG_CONTROL2		0x0E
#define FAN54511_REG_CONTROL3		0x0F

#define FAN54511_REG_VFLOAT		0x11
#define FAN54511_REG_IOCHRG 		0x12

#define FAN54511_REG_IBAT		0x13
#define FAN54511_REG_IBUS 		0x14
#define FAN54511_REG_VBUS 		0x15

#define FAN54511_REG_IIN 		0x16
#define FAN54511_REG_VIN		0x17
#define FAN54511_REG_NTC		0x18

#define FAN54511_REG_TIMER 		0x19
#define FAN54511_REG_SAFETY		0x1A
#define FAN54511_REG_TOPOFF		0x1B

#define FAN54511_REG_BOOST 		0x1C
#define FAN54511_REG_DPLUS		0x1F
#define FAN54511_REG_MONITOR0		0x20
#define FAN54511_REG_MONITOR1		0x21

#define FAN54511_REG_IC_INFO		0x2D
#define FAN54511_REG_FEATURE_CONTROL	0x30

enum {
	MAIN_CHARGER,
	SUB_CHARGER,
};



//Control 0
int fan54511_set_vlowv(u8 data);
int fan54511_set_vbatmin(u8 data);

//Control 1
int fan54511_set_VSYS(u8 vsys);

//Control 2
int fan54511_set_CONT(u8 val);
int fan54511_main_set_RCHGDIS(u8 val);
int fan54511_sub_set_RCHGDIS(u8 val);
int fan54511_main_set_NOBATOP(u8 val);
int fan54511_sub_set_NOBATOP(u8 val);
int fan54511_set_TE(u8 en);
int fan54511_main_set_TOEN(u8 en);
int fan54511_sub_set_TOEN(u8 en);
int fan54511_main_set_HZMODE(u8 en);
int fan54511_sub_set_HZMODE(u8 en);

//Control 3
int fan54511_set_reset(void);
int fan54511_main_tregth(u8 data);
int fan54511_sub_tregth(u8 data);
int fan54511_main_PPOFFSLP(u8 data);
int fan54511_sub_PPOFFSLP(u8 data);
int fan54511_main_PPOFF(u8 data);
int fan54511_sub_PPOFF(u8 data);
int fan54511_main_set_CE(u8 dis);
int fan54511_sub_set_CE(u8 dis);

//VFloat 11H
int fan54511_set_VFLOAT(u8 data);

//IOCHRG 12H
int fan54511_main_set_IOCHRG(int mA);
int fan54511_sub_set_IOCHRG(int mA);

//IBAT 13H
int fan54511_set_ITERM(u8 data);
int fan54511_set_PRECHG(u8 data);

//IBUS 14H
int fan54511_main_set_IBUSLIM(int mA);
int fan54511_sub_set_IBUSLIM(int mA);

//VBUS 15H
int fan54511_main_VBUSOVP(u8 data);
int fan54511_sub_VBUSOVP(u8 data);

int fan54511_set_VBUSLIM(u8 data);

//NTC 18H
int fan54511_main_TEMPDIS(u8 data);
int fan54511_sub_TEMPDIS(u8 data);

/* Timer 19H */
int fan54511_set_TMRRST(void);
int fan54511_set_WDEN(u8 en);
int fan54511_set_PRETMR(u8 data);
int fan54511_set_FCTMR(u8 data);

/* SAFETY 1AH*/
int fan54511_set_SAFE(u8 data);

/* TOPOFF */
int fan54511_main_TOTMR(u8 mins);
int fan54511_sub_TOTMR(u8 mins);

/* BOOST */
int fan54511_set_OTG(u8 en);
int fan54511_main_BOOSTEN(u8 en);
int fan54511_sub_BOOSTEN(u8 en);
int fan54511_main_set_VBOOST(int mV);
int fan54511_sub_set_VBOOST(int mV);
int fan54511_set_DIVCON(u8 en);
int fan54511_main_ENREF(u8 dis);

void fan54511_main_start_charging(void);
void fan54511_sub_start_charging(void);
void fan54511_main_stop_charging(void);
void fan54511_sub_stop_charging(void);
int fan54511_get_charging_status(void);
int fan54511_get_pg_stat(void);

void fan54511_dump_register(void);
#endif
