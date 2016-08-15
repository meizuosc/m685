/*********************************************************************
 * FileName:        fusb301a.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC32
 * Compiler:        XC32
 * Company:         Fairchild Semiconductor
 *
 * Author           Date          Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * M. Smith         12/04/2014    Initial Version
 *
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Fairchild Semiconductor (the “Company?
 * is supplied to you, the Company's customer, for exclusive use with its
 * USB Type C / USB PD products.  The software is owned by the Company and/or
 * its supplier, and is protected under applicable copyright laws.
 * All rights are reserved. Any use in violation of the foregoing restrictions
 * may subject the user to criminal sanctions under applicable laws, as well
 * as to civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS?CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ********************************************************************/

#ifndef FUSB301A_H
#define	FUSB301A_H

    /* /////////////////////////////////////////////////////////////////////////// */
    /* Required headers */
    /* /////////////////////////////////////////////////////////////////////////// */
#define TRUE true
#define FALSE false
#define BOOL bool
#define UINT8 u8
#define UINT16 u16
#define UINT32 u32

#define MOD_DRP_SHIFT			4
#define MOD_DRP_ACC_SHIFT		5
#define MOD_DRP 				(0x01 << MOD_DRP_SHIFT)
#define MOD_DRP_ACC 		(0x01 << MOD_DRP_ACC_SHIFT)
#define CON_INT_MSK				0x01
#define INT_DETACH_SHIFT		1
#define INT_DETACH 				(0x01 << INT_DETACH_SHIFT)
#define INT_BC_LVL_SHIFT		2
#define INT_BC_LVL 				(0x01 << INT_BC_LVL_SHIFT)
#define INT_ACC_CHG_SHIFT		3
#define INT_ACC_CHG				(0x01 << INT_ACC_CHG_SHIFT)

    /* fusb301a Register Addresses */
#define regDeviceID     0x01
#define regModes    0x02
#define regControl    0x03
#define regManual      0x04
#define regReset        0x05
//0x06~0x0F reserved
#define regMask         0x10
#define regStatus        0x11
#define regType        0x12
#define regInterrupt       0x13

typedef enum {
	Disabled = 0,
	ErrorRecovery,
	Unattached,
	AttachWaitSink,
	AttachedSink,
	AttachWaitSource,
	AttachedSource,
	TrySource,
	TryWaitSink,
	AudioAccessory,
	DebugAccessory,
	AttachWaitAccessory,
	PoweredAccessory,
	UnsupportedAccessory,
	DelayUnattached,
} ConnectionState;

typedef union {
	UINT8 byte;
	struct {
		unsigned REVISION:4;
		unsigned VERSION:4;
	};
} regDeviceID_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned INT_MASK:1;
		unsigned HOST_CUR:2;
		unsigned :1;
		unsigned DRPTOGGLE:2;
		unsigned :2;
	};
} regControl_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned SOURCE:1;
		unsigned SOURCE_ACC:1;
		unsigned SINK:1;
		unsigned SINK_ACC:1;
		unsigned DRP:1;
		unsigned DRP_ACC:1;
		unsigned :2;
	};
} regModes_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned SW_RES:1;
		unsigned:7;
	};
} regReset_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned M_ATTACH:1;
		unsigned M_DETACH:1;
		unsigned M_BC_LVL:1;
		unsigned M_ACC_CH:1;
		unsigned :4;
	};
} regMask_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned ATTACH:1;
		unsigned BC_LVL:2;
		unsigned VBUSOK:1;
		unsigned ORIENT:2;
		unsigned M_ACC_CH:2;
	};
} 
regStatus_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned AUDIOACC:1;
		unsigned DEBUGACC:1;
		unsigned :1;
		unsigned SOURCE:1;
		unsigned SINK:1;
		unsigned :3;
	};
} regType_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned I_ATTACH:1;
		unsigned I_DETACH:1;
		unsigned I_BC_LVL:1;
		unsigned I_ACC_CH:1;
		unsigned :4;
	};
} regInterrupt_t;

typedef struct {
	regDeviceID_t DeviceID;
	regControl_t Control;
	regModes_t Modes;
	regReset_t Reset;
	regStatus_t Status;
	regType_t Type;
	regInterrupt_t Interrupt;
} fusb301areg_t;

struct fusb301a_typc {
	int irqnum;
	int en_irq;

	struct mutex fsm_lock;
	struct delayed_work fsm_work;
	struct i2c_client *i2c_hd;
};

/* /////////////////////////////////////////////////////////////////////////// */
/* LOCAL PROTOTYPES */
/* /////////////////////////////////////////////////////////////////////////// */
void fusb301a_initialize(void);

int fusb301a_write(unsigned char regAddr, unsigned char length, unsigned char *data);
int fusb301a_read(unsigned char regAddr, unsigned char length, unsigned char *data);

int fusb301a_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var);
int fusb301a_i2c_r_reg(struct i2c_client *client, u8 addr, u8* var);
#endif	/* FUSB301A_H */
