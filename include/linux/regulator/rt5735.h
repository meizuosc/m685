/*
 * rt5735.h - Fairchild Regulator RT5735 Driver
 *
 * Copyright (c) 2016 Meizu Technology Co. Ltd.
 * WenBin Wu <wenbinwu@meizu.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _RT5735_H_
#define _RT5735_H_


/* VSEL ID */
enum {
	RT5735_VSEL_ID_0 = 0,
	RT5735_VSEL_ID_1,
};

/* Transition slew rate limiting from a low to high voltage.
 * -----------------------
 *   Bin |Slew Rate(mV/uS)
 * ------|----------------
 *   010 |    16.00
 * ------|----------------
 *   000 |    64.00
 * ------|----------------
 *   001 |    32.00
 * ------|----------------
 *   011 |     8.00
 * ------|----------------
 *   100 |     4.00
 * -----------------------
 */
enum {
	FAN53555_SLEW_RATE_64MV = 0,
	FAN53555_SLEW_RATE_16MV,
	FAN53555_SLEW_RATE_32MV,
	FAN53555_SLEW_RATE_8MV,
	FAN53555_SLEW_RATE_4MV,
};

struct rt5735_platform_data {
	struct regulator_init_data *regulator;
	unsigned int slew_rate;
	/* Sleep VSEL ID */
	unsigned int sleep_vsel_id;
};
#endif		/* _RT5735_H_ */
