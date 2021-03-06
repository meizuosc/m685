#ifndef __DT_CS43L36_H
#define __DT_CS43L36_H

/* 
 * HP Pulldown Settings
 * See Section 7.17.4 DAC Control 2
 */
#define CS43L36_HPOUT_PULLDN_1K		0x0C
#define CS43L36_HPOUT_PULLDN_6K		0x0A
#define CS43L36_HPOUT_PULLDN_9_6K	0x09
#define CS43L36_HPOUT_PULLDN_787_OHM	0x0F
#define CS43L36_HPOUT_PULLDN_857_OHM	0x0E
#define CS43L36_HPOUT_PULLDN_3_7K	0x0B
#define CS43L36_HPOUT_PULLDN_AUTO	0x00

// /* HPOUT Load Capacity */
#define CS43L36_HPOUT_LOAD_1NF		0
#define CS43L36_HPOUT_LOAD_10NF		1

/* HPOUT Clamp to GND Overide */
#define CS43L36_HPOUT_CLAMP_EN		0
#define CS43L36_HPOUT_CLAMP_DIS		1

/* DAC Monitoring Support */
#define CS43L36_DAC_MON_PWRUP		0
#define CS43L36_DAC_MON_NOPWR		1

#endif /* __DT_CS43L36_H */