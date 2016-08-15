
#ifndef _ISL29125_REG_DUMP_H_
#define _ISL29125_REG_DUMP_H_

#include <linux/device.h>

struct isl29125_regmap {
	union {
		uint8_t data;
		struct {
			uint8_t ID;
		};
	} reg00;
#define ISL29125_REG00_DUMP(regmap) do {\
	printk(KERN_EMERG "[ISL29125]: " "==== =================\n"); \
	printk(KERN_EMERG "[ISL29125]: " "[00] [7:0] 0x%02X\n", regmap->reg00.data); \
	printk(KERN_EMERG "[ISL29125]: " "[00] [7:0] ID: %d\n", regmap->reg00.ID); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* RGB Operating Modes [B2:B0]
			   000 Power Down (ADC conversion)
			   001 GREEN Only
			   010 RED Only
			   011 BLUE Only
			   100 Stand by (No ADC conversion)
			   101 GREEN/RED/BLUE
			   110 GREEN/RED
			   111 GREEN/BLUE */
			uint8_t MODE: 		3;

			/* RGB Data Sensing Range [B3]
			   0: 375 lux, 1: 10000 lux*/
			uint8_t RNG: 		1;

			/* ADC Resolution [B4]
			   0: 16 bits, 1: 10 bits */
			uint8_t BITS: 		1;

			/* RGB Start Synced at INT Pin
			   0: ADC start at I2C write 0x01
			   1: ADC start at rising INT */
			uint8_t SYNC: 		1;
			uint8_t RESERVED0: 	2;
		};
	} reg01;
#define ISL29125_REG01_DUMP(regmap) do {\
	printk(KERN_EMERG "[ISL29125]: " "==== =================\n"); \
	printk(KERN_EMERG "[ISL29125]: " "[01] [7:0] 0x%02X\n", regmap->reg01.data); \
	printk(KERN_EMERG "[ISL29125]: " "[01] [2:0] MODE: %d\n", regmap->reg01.MODE); \
	printk(KERN_EMERG "[ISL29125]: " "[01] [3:3] RNG: %d\n", regmap->reg01.RNG); \
	printk(KERN_EMERG "[ISL29125]: " "[01] [4:4] BITS: %d\n", regmap->reg01.BITS); \
	printk(KERN_EMERG "[ISL29125]: " "[01] [5:5] SYNC: %d\n", regmap->reg01.SYNC); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* IR Comp Adjust [B5:B0]*/
			uint8_t ALSCC: 		6;
			uint8_t RESERVED0: 	1;
			/* IR Comp Offset [B7] */
			uint8_t IRCOM: 		1;
		};
	} reg02;
#define ISL29125_REG02_DUMP(regmap) do {\
	printk(KERN_EMERG "[ISL29125]: " "==== =================\n"); \
	printk(KERN_EMERG "[ISL29125]: " "[02] [7:0] 0x%02X\n", regmap->reg02.data); \
	printk(KERN_EMERG "[ISL29125]: " "[02] [5:0] ALSCC: %d\n", regmap->reg02.ALSCC); \
	printk(KERN_EMERG "[ISL29125]: " "[02] [7:7] IRCOM: %d\n",regmap->reg02.IRCOM); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* INTERRUPT THRESHOLD ASSIGNMENT [B1:0]
			   00 No Interrupt,  01 GREEN Interrupt
			   10 RED Interrupt, 11 BLUE Interrupt */
			uint8_t INTSEL: 	2;

			/* INTERRUPT PERSIST CONTROL [B3:2]
			   00: 1, 01: 2, 10: 4, 11: 8*/
			uint8_t PRST: 		2;

			/* RGB CONVERSION DONE TO INT CONTROL [B4]
			   0: Disable, 1: Enable*/
			uint8_t CONVEN: 	1;
			uint8_t RESERVED0: 	3;
		};
	} reg03;
#define ISL29125_REG03_DUMP(regmap) do {\
	printk(KERN_EMERG "[ISL29125]: " "==== =================\n"); \
	printk(KERN_EMERG "[ISL29125]: " "[03] [7:0] 0x%02X\n", regmap->reg03.data); \
	printk(KERN_EMERG "[ISL29125]: " "[03] [1:0] INTSEL: %d\n", regmap->reg03.INTSEL); \
	printk(KERN_EMERG "[ISL29125]: " "[03] [3:2] PRST: %d\n",regmap->reg03.PRST); \
	printk(KERN_EMERG "[ISL29125]: " "[03] [4:4] CONVEN: %d\n", regmap->reg03.CONVEN); \
	}while(0)

	union {
		uint8_t data[4];
		struct {
			/* Low Threshold-Low byte */
			uint8_t LTHL;
			/* Low Threshold-High byte */
			uint8_t LTHH;
			/* High Threshold-Low byte */
			uint8_t HTHL;
			/* High Threshold-High byte */
			uint8_t HTHH;
		};
	} reg04_07;
#define ISL29125_REG04_07_DUMP(regmap) do {\
	printk(KERN_EMERG "[ISL29125]: " "==== =================\n"); \
	printk(KERN_EMERG "[ISL29125]: " "[04] [7:0] 0x%02X\n", regmap->reg04_07.data[0]); \
	printk(KERN_EMERG "[ISL29125]: " "[04] [7:0] LTHL: %d\n", regmap->reg04_07.LTHL); \
	printk(KERN_EMERG "[ISL29125]: " "[05] [7:0] 0x%02X\n", regmap->reg04_07.data[1]); \
	printk(KERN_EMERG "[ISL29125]: " "[05] [7:0] LTHH: %d\n", regmap->reg04_07.LTHH); \
	printk(KERN_EMERG "[ISL29125]: " "[06] [7:0] 0x%02X\n", regmap->reg04_07.data[2]); \
	printk(KERN_EMERG "[ISL29125]: " "[06] [7:0] HTHL: %d\n", regmap->reg04_07.HTHL); \
	printk(KERN_EMERG "[ISL29125]: " "[07] [7:0] 0x%02X\n", regmap->reg04_07.data[3]); \
	printk(KERN_EMERG "[ISL29125]: " "[07] [7:0] HTHH: %d\n", regmap->reg04_07.HTHH); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* Status bit of the interrupt [B0]
			   0: Interrupt is cleared or not triggered yet
			   1: Interrupt is triggered */
			uint8_t RGBTHF: 	1;

			/* Status bit of conversion [B1]
			   0: Still convert or cleared,
			   1: Conversion completed */
			uint8_t CONVENF: 	1;

			/* Status bit for brownout condition [B2]
			   0: No Brownout, 1: Power down or Brownout occurred*/
			uint8_t BOUTF: 		1;
			uint8_t RESERVED0: 	1;
			/* Flag bits to display either Red or Green or Blue
			   is under conversion [B5:B4]
			   00 No Operation
			   01 GREEN
			   10 RED
			   11 BLUE*/
			uint8_t RGBCF: 		2;
			uint8_t RESERVED1: 	2;
		};
	} reg08;
#define ISL29125_REG08_DUMP(regmap) do {\
	printk(KERN_EMERG "[ISL29125]: " "==== =================\n"); \
	printk(KERN_EMERG "[ISL29125]: " "[08] [7:0] 0x%02X\n", regmap->reg08.data); \
	printk(KERN_EMERG "[ISL29125]: " "[08] [0:0] RGBTHF: %d\n", regmap->reg08.RGBTHF); \
	printk(KERN_EMERG "[ISL29125]: " "[08] [1:1] CONVENF: %d\n",regmap->reg08.CONVENF); \
	printk(KERN_EMERG "[ISL29125]: " "[08] [2:2] BOUTF: %d\n", regmap->reg08.BOUTF); \
	printk(KERN_EMERG "[ISL29125]: " "[08] [5:4] RGBCF: %d\n", regmap->reg08.RGBCF); \
	}while(0)

	union {
		uint8_t data[6];
		struct {
			/* GREEN data-Low byte */
			uint8_t GREENL;
			/* GREEN data-High byte */
			uint8_t GREENH;
			/* RED data-Low byte */
			uint8_t REDL;
			/* RED data-High byte */
			uint8_t REDH;
			/* BLUE data-Low byte */
			uint8_t BLUEL;
			/* BLUE data-High byte */
			uint8_t BLUEH;
		};
	} reg09_0E;
#define ISL29125_REG09_0E_DUMP(regmap) do {\
	printk(KERN_EMERG "[ISL29125]: " "==== =================\n"); \
	printk(KERN_EMERG "[ISL29125]: " "[09] [7:0] 0x%02X\n", regmap->reg09_0E.data[0]); \
	printk(KERN_EMERG "[ISL29125]: " "[09] [7:0] GREENL: %d\n", regmap->reg09_0E.GREENL); \
	printk(KERN_EMERG "[ISL29125]: " "[0A] [7:0] 0x%02X\n", regmap->reg09_0E.data[1]); \
	printk(KERN_EMERG "[ISL29125]: " "[0A] [7:0] GREENH: %d\n", regmap->reg09_0E.GREENH); \
	printk(KERN_EMERG "[ISL29125]: " "[0B] [7:0] 0x%02X\n", regmap->reg09_0E.data[2]); \
	printk(KERN_EMERG "[ISL29125]: " "[0B] [7:0] REDL: %d\n", regmap->reg09_0E.REDL); \
	printk(KERN_EMERG "[ISL29125]: " "[0C] [7:0] 0x%02X\n", regmap->reg09_0E.data[3]); \
	printk(KERN_EMERG "[ISL29125]: " "[0C] [7:0] REDH: %d\n", regmap->reg09_0E.REDH); \
	printk(KERN_EMERG "[ISL29125]: " "[0D] [7:0] 0x%02X\n", regmap->reg09_0E.data[4]); \
	printk(KERN_EMERG "[ISL29125]: " "[0D] [7:0] BLUEL: %d\n", regmap->reg09_0E.BLUEL); \
	printk(KERN_EMERG "[ISL29125]: " "[0E] [7:0] 0x%02X\n", regmap->reg09_0E.data[5]); \
	printk(KERN_EMERG "[ISL29125]: " "[0E] [7:0] BLUEH: %d\n", regmap->reg09_0E.BLUEH); \
	}while(0)
};

static inline int isl29125_regmap_dump(struct isl29125_regmap *regmap)
{
	ISL29125_REG00_DUMP(regmap);
	ISL29125_REG01_DUMP(regmap);
	ISL29125_REG02_DUMP(regmap);
	ISL29125_REG03_DUMP(regmap);
	ISL29125_REG04_07_DUMP(regmap);
	ISL29125_REG08_DUMP(regmap);
	ISL29125_REG09_0E_DUMP(regmap);
	return 0;
}

#endif