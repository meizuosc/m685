/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

//#include <linux/regulator/consumer.h>

#ifndef CONFIG_OF
#include <linux/spi/fpc1020.h>
#include <linux/spi/fpc1020_common.h>
#include <linux/spi/fpc1020_regulator.h>
#else
#include <linux/of.h>
#include "fpc1020.h"
#include "fpc1020_common.h"
#include "fpc1020_regulator.h"
#endif
#include "fpc1020.h"

#include <mt-plat/mt_pwm.h>
#include <mt-plat/upmu_common.h>

static void fpc1020_hw_power(bool bonoff)
{
	if (bonoff) {
		pmic_set_register_value(PMIC_RG_VIBR_EN, 1);
	} else {
		pmic_set_register_value(PMIC_RG_VIBR_EN, 0);
	}
}
/* -------------------------------------------------------------------- */
int fpc1020_regulator_configure(fpc1020_data_t *fpc1020)
{
	int error = 0;
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);
	pmic_set_register_value(PMIC_RG_VIBR_VOSEL, 3);
	return error;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_release(fpc1020_data_t *fpc1020)
{

	fpc1020->power_enabled = false;
	fpc1020_hw_power(false);

	return 0;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_set(fpc1020_data_t *fpc1020, bool enable)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s %s\n", __func__,enable?"on":"off");

	if( fpc1020->power_enabled == enable)
		return error;

	fpc1020_hw_power(enable);

	fpc1020->power_enabled = enable;

	return error;
}
