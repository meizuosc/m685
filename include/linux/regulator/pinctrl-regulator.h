/*
 * pinctrl-regulator.h
 *
 * Copyright (c) 2016 Meizu Technology Co. Ltd.
 * WenBin Wu <wenbinwu@meizu.com>
 *
 * based on fixed.h
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 */

#ifndef __REGULATOR_GPIO_H
#define __REGULATOR_GPIO_H

struct regulator_init_data;

enum regulator_type;

/**
 * struct pinctrl_regulator_config - config structure
 * @supply_name:	Name of the regulator supply
 * @enable_high:	Polarity of enable GPIO
 *			1 = Active high, 0 = Active low
 * @enabled_at_boot:	Whether regulator has been enabled at
 *			boot or not. 1 = Yes, 0 = No
 *			This is used to keep the regulator at
 *			the default state
 * @startup_delay:	Start-up time in microseconds
 * @regulator_type:	either REGULATOR_CURRENT or REGULATOR_VOLTAGE
 * @init_data:		regulator_init_data
 *
 * This structure contains gpio-voltage regulator configuration
 * information that must be passed by platform code to the
 * gpio-voltage regulator driver.
 */
struct pinctrl_regulator_config {
	const char *supply_name;

	unsigned int enabled_at_boot;
	unsigned int startup_delay;
	int microvolts;	

	enum regulator_type type;
	struct regulator_init_data *init_data;
};

#endif
