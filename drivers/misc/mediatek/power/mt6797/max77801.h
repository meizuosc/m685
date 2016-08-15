/*****************************************************************************
*
* Filename:
* ---------
*   max77801.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   max77801 header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _max77801_SW_H_
#define _max77801_SW_H_

/* Regs */
#define max77801_DEVICE_ID		0x00
#define max77801_STATUS		           0x01
#define max77801_CONTROL1		0x02
#define max77801_CONTROL2		0x03
#define max77801_VOUT_DVS_L		0x04
#define max77801_VOUT_DVS_H		0x05


extern int is_max77801_exist(void);
extern void max77801_dump_register(void);
extern unsigned int max77801_read_interface(unsigned char RegNum, unsigned char *val,
					    unsigned char MASK, unsigned char SHIFT);
extern unsigned int max77801_config_interface(unsigned char RegNum, unsigned char val,
					      unsigned char MASK, unsigned char SHIFT);

#endif				/* _max77801_SW_H_ */
