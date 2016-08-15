#ifndef BQ27532_BATTERY_H
#define BQ27532_BATTERY_H

//#include <mach/mt_typedefs.h>

#define DRIVER_VERSION			"1.2.0"
#define INVALID_REG_ADDR		0xFF

#define	BQ27532_REG_CTRL           	0x00	/* CONTROL	*/ 
#define	BQ27532_REG_TEMP          	0x06	/* TEMP		*/
#define	BQ27532_REG_INT_TEMP      	0x16	/* Internal TEMP */
#define	BQ27532_REG_VOLT	       	0x08	/* VOLT		*/
#define	BQ27532_REG_AVG_CURR      	0x14	/* AVG CURR	*/
#define BQ27532_REG_NOW_CURR		0x22	/*瞬态电流InstantaneousCurrentReading*/
#define	BQ27532_REG_SOC           	0x20	/* SOC百分比电量*/
#define	BQ27532_REG_RM_CAP        	0x10	/* 剩余容量RM*/
#define	BQ27532_REG_FLAGS         	0x0A	/* FLAGS	*/
#define	BQ27532_REG_TTE           	0x04	/* TTE		*/
#define	BQ27532_REG_NAC           	0x0C	/* NAC		*/
#define	BQ27532_REG_FCC           	0x12	/* LMD(FCC)慢充容*/ 
#define	BQ27532_REG_CYCT          	0x1E	/* CYCT		*/
#define	BQ27532_REG_CAL_CURR      	0x2E	/* 电量计计算出的推荐充电电流 */
#define	BQ27532_REG_CAL_VOL       	0x30	/* 推荐充电电压*/
#define BQ27532_REG_FULLCHARGECAPCITY	0x12

/*
 * SBS Commands for DF access - these are pretty standard
 * So, no need to go in the command array
 */
#define BLOCK_DATA_CLASS		0x3E
#define DATA_BLOCK			0x3F
#define BLOCK_DATA			0x40
#define BLOCK_DATA_CHECKSUM		0x60
#define BLOCK_DATA_CONTROL		0x61

/* bq274xx/bq276xx specific command information */
#define BQ274XX_UNSEAL_KEY             0x36720414
#define BQ274XX_UNSEAL_KEY2        0xffffffff 
#define BQ274XX_SOFT_RESET		0x43

#define BQ274XX_FLAG_ITPOR				0x20
#define BQ274XX_CTRL_STATUS_INITCOMP	0x80

#define BQ27XXX_FLAG_DSC		BIT(0)
#define BQ27XXX_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27XXX_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27XXX_FLAG_FC			BIT(9)
#define BQ27XXX_FLAG_OTD		BIT(14)
#define BQ27XXX_FLAG_OTC		BIT(15)

/* BQ27000 has different layout for Flags register */
#define BQ27200_FLAG_EDVF		BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27200_FLAG_EDV1		BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27200_FLAG_CI			BIT(4) /* Capacity Inaccurate flag */
#define BQ27200_FLAG_FC			BIT(5)
#define BQ27200_FLAG_CHGS		BIT(7) /* Charge state flag */

#define BQ27200_RS			20 /* Resistor sense */
#define BQ27200_POWER_CONSTANT		(256 * 29200 / 1000)

/* Subcommands of Control() */
#define CONTROL_STATUS_SUBCMD		0x0000
#define DEV_TYPE_SUBCMD			0x0001
#define FW_VER_SUBCMD			0x0002
#define DF_VER_SUBCMD			0x001F
#define RESET_SUBCMD			0x0041
#define SET_CFGUPDATE_SUBCMD		0x0013
#define SEAL_SUBCMD			0x0020

/* Location of SEAL enable bit in bq276xx DM */
#define BQ276XX_OP_CFG_B_SUBCLASS	64
#define BQ276XX_OP_CFG_B_OFFSET		2
#define BQ276XX_OP_CFG_B_DEF_SEAL_BIT	(1 << 5)

struct bq27532_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int energy;
	int flags;
	int power_avg;
	int health;
	int voltage;
};

struct dm_reg {
	u8 subclass;
	u8 offset;
	u8 len;
	u32 data;
};

struct bq27532_device_info;
struct bq27532_access_methods {
	int (*read)(struct bq27532_device_info *di, u8 reg, bool single);
	int (*write)(struct bq27532_device_info *di, u8 reg, int value,
			bool single);
	int (*blk_read)(struct bq27532_device_info *di, u8 reg, u8 *data,
		u8 sz);
	int (*blk_write)(struct bq27532_device_info *di, u8 reg, u8 *data,
		u8 sz);
};

struct bq27532_device_info {
	struct device 		*dev;
	struct i2c_client *client;
	int			id;
	struct bq27532_reg_cache cache;
	int charge_design_full;
	unsigned long last_update;
	struct delayed_work work;
	struct bq27532_access_methods bus;
	struct mutex bat_i2c_access;
	int fw_ver;
	int df_ver;
	struct dm_reg *dm_regs;
	u16 dm_regs_count;
	struct delayed_work irq_dwork;
	struct delayed_work dump_dwork;
	struct wake_lock fg_int;
	int fuel_irq;
};

extern signed int bq27532_battery_current(void);
extern signed int bq27532_battery_current_now(void);
extern unsigned int bq27532_battery_voltage(void);
extern signed int bq27532_battery_read_temperature(void);
extern signed int bq27532_battery_read_soc(void);
extern signed int bq27532_battery_read_fullchargecapacity(void);
extern signed int bq27532_get_battery_data(int update);
extern signed int bq27532_battery_read_truesoc(void);
extern void bq27532_sync_truesoc(void);
extern unsigned int bq27532_charger_voltage(void);

#ifdef CONFIG_MTK_BQ2589X_SUPPORT
void bq2589x_charge_update_status(void);
#endif
#endif //#ifndef BQ27532_BATTERY_H
