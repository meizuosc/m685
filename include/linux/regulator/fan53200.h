/*
 * fan53200.h - Fairchild Regulator FAN53200 Driver
 *
 * Copyright (c) 2016 Meizu Technology Co. Ltd.
 * WenBin Wu <wenbinwu@meizu.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __FAN53200_H__
#define __FAN53200_H__

/* VSEL ID */
enum {
	FAN53200_VSEL_ID_0 = 0,
	FAN53200_VSEL_ID_1,
};

/* Transition slew rate limiting from a low to high voltage.
 * -----------------------
 *   Bin |Slew Rate(mV/uS)
 * ------|----------------
 *   000 |    64.00
 * ------|----------------
 *   001 |    32.00
 * ------|----------------
 *   010 |    16.00
 * ------|----------------
 *   011 |     8.00
 * ------|----------------
 *   100 |     4.00
 * ------|----------------
 *   101 |     2.00
 * ------|----------------
 *   110 |     1.00
 * ------|----------------
 *   111 |     0.50
 * -----------------------
 */
enum {
	FAN53200_SLEW_RATE_64MV = 0,
	FAN53200_SLEW_RATE_32MV,
	FAN53200_SLEW_RATE_16MV,
	FAN53200_SLEW_RATE_8MV,
	FAN53200_SLEW_RATE_4MV,
	FAN53200_SLEW_RATE_2MV,
	FAN53200_SLEW_RATE_1MV,
	FAN53200_SLEW_RATE_0_5MV,
};

struct fan53200_platform_data {
	struct regulator_init_data *regulator;
	unsigned int slew_rate;
	/* Sleep VSEL ID */
	unsigned int sleep_vsel_id;
};

#endif /* __FAN53200_H__ */
