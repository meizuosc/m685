/*********************************************************************
 * FileName:        TUSB320.h
 * Dependencies:    See INCLUDES section below
 * Company:        Meizu Inc.
 *
 * Author           Date          Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * WenBinWu        07/04/2016    Initial Version
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

#ifndef TUSB320_H
#define	TUSB320_H

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
#define MOD_PORT 				(0x00 << MOD_DRP_SHIFT)
#define MOD_UFP 				(0x01 << MOD_DRP_SHIFT)
#define MOD_DFP 				(0x02 << MOD_DRP_SHIFT)
#define MOD_DRP 				(0x03 << MOD_DRP_SHIFT)

#define INT_SHIFT				4
#define INT_MSK					(1 << INT_SHIFT)

    /* TUSB320 Register Addresses */
#define regDeviceID0	0x00
#define regDeviceID1	0x01
#define regDeviceID2	0x02
#define regDeviceID3	0x03
#define regDeviceID4	0x04
#define regDeviceID5	0x05
#define regDeviceID6	0x06
#define regDeviceID7	0x07
#define regType			0x08
#define regInterrupt		0x09
#define regModes		0x0A
#define regControl		0x45
//0x06~0x0F reserved

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
	AudioChargedThruAccessory,
	DebugAccessory,
	AttachWaitAccessory,
	PoweredAccessory,
	UnsupportedAccessory,
	DelayUnattached,
} ConnectionState;

typedef union {
	UINT8 byte[8];
	struct {
		UINT8 ID7;
		UINT8 ID6;
		UINT8 ID5;
		UINT8 ID4;
		UINT8 ID3;
		UINT8 ID2;
		UINT8 ID1;
		UINT8 ID0;
	};
} regDeviceID_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned ACTIVE_CABLE_DETECTION:1;
		unsigned ACCESSORY_CONNECTED:3;
		unsigned CURRENT_MODE_DETECT:2;
		unsigned CURRENT_MODE_ADVERTISE:2;
	};
} regType_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned :1;
		unsigned DRP_DUTY_CYCLE:2;
		unsigned :1;
		unsigned INTERRUPT_STATUS:1;
		unsigned CABLE_DIR:1;
		unsigned ATTAHED_STATE:2;
	};
} 
regInterrupt_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned :3;
		unsigned RESET:1;
		unsigned MODE:2;
		unsigned DEBOUNCE:2;
	};
} regModes_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned :5;
		unsigned RD_RP_DISABLE:1;
		unsigned :2;
	};
} regControl_t;

typedef struct {
	regDeviceID_t DeviceID;
	regType_t Type;
	regInterrupt_t Interrupt;
	regModes_t Modes;
	regControl_t Control;
} TUSB320reg_t;

struct tusb320_typc {
	int irqnum;
	int en_irq;

	struct mutex fsm_lock;
	struct delayed_work fsm_work;
	struct i2c_client *i2c_hd;
};


extern int tusb320_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var);
extern int tusb320_i2c_r_reg(struct i2c_client *client, u8 addr, u8 *var);

#endif	/* TUSB320_H */
