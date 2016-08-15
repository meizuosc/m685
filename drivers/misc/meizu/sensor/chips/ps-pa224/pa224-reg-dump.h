
#ifndef _PA224_REG_DUMP_H_
#define _PA224_REG_DUMP_H_

#include <linux/device.h>

struct pa224_regmap {
	union {
		uint8_t data;
		struct {
			uint8_t RESERVED0: 	1;
			/* PS operation mode */
			uint8_t PSEN: 		1;
			uint8_t RESERVED1: 	4;
		};
	} reg00;
#define PA224_REG00_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[00] [7:0] 0x%02X\n", regmap->reg00.data); \
	printk(KERN_EMERG "[PA224]: " "[00] [1:1] PSEN: %d\n", regmap->reg00.PSEN); \
	}while(0)

	union {
		uint8_t data;
		struct {
			uint8_t RESERVED0: 	2;

			/* PS interrupt persistence
			   00: 1, 01: 2, 10: 4, 11: 8 */
			uint8_t PSPRST: 	2;

			/* Light Emitting Source current setting
			   100: 15mA, 101: 12mA, 110: 10mA, 111: 7mA */
			uint8_t LESC: 		3;
			uint8_t RESERVED1: 	1;
		};
	} reg01;
#define PA224_REG01_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[01] [7:0] 0x%02X\n", regmap->reg01.data); \
	printk(KERN_EMERG "[PA224]: " "[01] [3:2] PSPRST: %d\n", regmap->reg01.PSPRST); \
	printk(KERN_EMERG "[PA224]: " "[01] [6:4] LESC: %d\n", regmap->reg01.LESC); \
	}while(0)

	union {
		uint8_t data;
		struct {
			uint8_t RESERVED0: 	1;

			/* PS interrupt flag
			   0: Inactive, 1: Active*/
			uint8_t PSINTF: 	1;

			/* Select source of interrupt flag
			   00: Reserved, 01: PS, 10: None, 11: Reserved*/
			uint8_t INTSEL: 	2;
			/* Reset command
			   0: Clear command reset, 1: Reset*/
			uint8_t CMR: 		1;
			uint8_t RESERVED1: 	1;
			/* PS offset mode selection
			   0: Offset measurement, 1: Normal measurement */
			uint8_t PSMODESEL: 	1;
			uint8_t RESERVED2: 	1;
		};
	} reg02;
#define PA224_REG02_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[02] [7:0] 0x%02X\n", regmap->reg02.data); \
	printk(KERN_EMERG "[PA224]: " "[02] [1:1] PSINTF: %d\n", regmap->reg02.PSINTF); \
	printk(KERN_EMERG "[PA224]: " "[02] [3:2] INTSEL: %d\n",regmap->reg02.INTSEL); \
	printk(KERN_EMERG "[PA224]: " "[02] [4:4] CMR: %d\n",regmap->reg02.CMR); \
	printk(KERN_EMERG "[PA224]: " "[02] [6:6] PSMODESEL: %d\n",regmap->reg02.PSMODESEL); \
	}while(0)

	union {
		uint8_t data;
		struct {
			uint8_t RESERVED0: 	3;

			/* PS sleep time
			   000: 6.25ms, 001: 12.5ms, 010: 25ms,  011: 50ms,
			   100: 100ms,  101: 200ms,  110: 400ms, 111: 800ms*/
			uint8_t PSSLP: 		3;

			/* Interrupt type of PS
			   0: Window type, 1: Hysteresis type*/
			uint8_t PITYPE: 	1;
			uint8_t RESERVED1: 	1;
		};
	} reg03;
#define PA224_REG03_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[03] [7:0] 0x%02X\n", regmap->reg03.data); \
	printk(KERN_EMERG "[PA224]: " "[03] [5:3] PSSLP: %d\n", regmap->reg03.PSSLP); \
	printk(KERN_EMERG "[PA224]: " "[03] [6:6] PITYPE: %d\n",regmap->reg03.PITYPE); \
	}while(0)

	union {
		uint8_t data[4];
		struct {
			uint8_t RESERVED[4];
		};
	} reg04_07;
#define PA224_REG04_07_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[04] [7:0] 0x%02x\n", regmap->reg04_07.RESERVED[0]); \
	printk(KERN_EMERG "[PA224]: " "[05] [7:0] 0x%02x\n", regmap->reg04_07.RESERVED[1]); \
	printk(KERN_EMERG "[PA224]: " "[06] [7:0] 0x%02x\n", regmap->reg04_07.RESERVED[2]); \
	printk(KERN_EMERG "[PA224]: " "[07] [7:0] 0x%02x\n", regmap->reg04_07.RESERVED[3]); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* PS Low_Threshold byte
			   PS Low_Threshold level setting */
			uint8_t PSLT;
		};
	} reg08;
#define PA224_REG08_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[08] [7:0] 0x%02X\n", regmap->reg08.data); \
	printk(KERN_EMERG "[PA224]: " "[08] [7:0] PSLT: %d\n", regmap->reg08.PSLT); \
	}while(0)

	union {
		uint8_t data;
		struct {
			uint8_t RESERVED;
		};
	} reg09;
#define PA224_REG09_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[09] [7:0] 0x%02x\n", regmap->reg09.RESERVED); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* PS High_Threshold byte
			   PS High_Threshold level setting */
			uint8_t PSHT;
		};
	} reg0A;
#define PA224_REG0A_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[0A] [7:0] 0x%02X\n", regmap->reg0A.data); \
	printk(KERN_EMERG "[PA224]: " "[0A] [7:0] PSHT: %d\n", regmap->reg0A.PSHT); \
	}while(0)

	union {
		uint8_t data[3];
		struct {
			uint8_t RESERVED[3];
		};
	} reg0B_0D;
#define PA224_REG0B_0D_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[0B] [7:0] 0x%02x\n", regmap->reg0B_0D.RESERVED[0]); \
	printk(KERN_EMERG "[PA224]: " "[0C] [7:0] 0x%02x\n", regmap->reg0B_0D.RESERVED[1]); \
	printk(KERN_EMERG "[PA224]: " "[0D] [7:0] 0x%02x\n", regmap->reg0B_0D.RESERVED[2]); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* PS ADC count byte
			   8-bits ADC conversion code for PS */
			uint8_t PSDT;
		};
	} reg0E;
#define PA224_REG0E_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[0E] [7:0] 0x%02X\n", regmap->reg0E.data); \
	printk(KERN_EMERG "[PA224]: " "[0E] [7:0] PSDT: %d\n", regmap->reg0E.PSDT); \
	}while(0)

	union {
		uint8_t data;
		struct {
			uint8_t RESERVED;
		};
	} reg0F;
#define PA224_REG0F_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[0F] [7:0] 0x%02x\n", regmap->reg0F.RESERVED); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* PS offset value
			   8-bits ADC code for PS offset */
			uint8_t PSOFSDT;
		};
	} reg10;
#define PA224_REG10_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[10] [7:0] 0x%02X\n", regmap->reg10.data); \
	printk(KERN_EMERG "[PA224]: " "[10] [7:0] PSOFSDT: %d\n", regmap->reg10.PSOFSDT); \
	}while(0)

	union {
		uint8_t data;
		struct {
			/* PS AD mode
			   PS ADC resolution selection
			   Please set the register as 0x82 */
			uint8_t PSADMOD;
		};
	} reg11;
#define PA224_REG11_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[11] [7:0] 0x%02X\n", regmap->reg11.data); \
	printk(KERN_EMERG "[PA224]: " "[11] [7:0] PSADMOD: %d\n", regmap->reg11.PSADMOD); \
	}while(0)

	union {
		uint8_t data;
		struct {
			uint8_t RESERVED0: 	1;

			/* IR Light Pulse
			   0: 32 pulse, 1: 64 pulse*/
			uint8_t IRLP: 		1;

			/* PS PD Setting
			   PS photo diode selection
			   Please set the register as D[3:2]=0x02*/
			uint8_t PSPDS: 		2;

			/* Digital filter coefficient
			   000: None, 001: Coefficient 1, 010: Coefficient 2,
			   011: Coefficient 3, 100: Coefficient 4*/
			uint8_t FLTFC: 		3;
			uint8_t RESERVED1: 	1;
		};
	} reg12;
#define PA224_REG12_DUMP(regmap) do {\
	printk(KERN_EMERG "[PA224]: " "==== =================\n"); \
	printk(KERN_EMERG "[PA224]: " "[12] [7:0] 0x%02X\n", regmap->reg12.data); \
	printk(KERN_EMERG "[PA224]: " "[12] [1:1] IRLP: %d\n", regmap->reg12.IRLP); \
	printk(KERN_EMERG "[PA224]: " "[12] [3:2] PSPDS: %d\n",regmap->reg12.PSPDS); \
	printk(KERN_EMERG "[PA224]: " "[12] [6:4] FLTFC: %d\n",regmap->reg12.FLTFC); \
	}while(0)
};

static inline int pa224_regmap_dump(struct pa224_regmap *regmap)
{
	PA224_REG00_DUMP(regmap);
	PA224_REG01_DUMP(regmap);
	PA224_REG02_DUMP(regmap);
	PA224_REG03_DUMP(regmap);
	PA224_REG04_07_DUMP(regmap);
	PA224_REG08_DUMP(regmap);
	PA224_REG09_DUMP(regmap);
	PA224_REG0A_DUMP(regmap);
	PA224_REG0B_0D_DUMP(regmap);
	PA224_REG0E_DUMP(regmap);
	PA224_REG0F_DUMP(regmap);
	PA224_REG10_DUMP(regmap);
	PA224_REG11_DUMP(regmap);
	PA224_REG12_DUMP(regmap);
	return 0;
}

#endif