/*
 * m80_fingerprint.c - fingerprint driver  for m80 board
 *
 * Copyright (C) 2015 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <mt_spi.h>
#include "spi_pin_config.h"

struct mt_chip_conf fpc_spi_ctrdata = {

	.setuptime 	= 3,
	.holdtime 	= 3,
	.high_time 	= 11, //11--4.65M 15--3m   25--2m   50--1m  [ 100--0.5m]
	.low_time 	= 11,
	.cs_idletime 	= 20,
	.ulthgh_thrsh 	= 0,

	.cpol 		= 0,
	.cpha 		= 0,

	.rx_mlsb 	= 1,
	.tx_mlsb 	= 1,

	.tx_endian 	= 0,
	.rx_endian 	= 0,

	.com_mod 	= FIFO_TRANSFER,
	//.com_mod = DMA_TRANSFER,

	.pause 		= 0,
	.finish_intr 	= 1,
	.deassert 	= 0,
	.ulthigh 	= 0,
	.tckdly 	= 0,
};

static struct spi_board_info spi_fp_board_info[] __initdata= {
	[0] = {
		.modalias		= "fp_spi",
		.platform_data		= NULL,
		.bus_num		= 1,
		.controller_data	= &fpc_spi_ctrdata,
		.chip_select		= 1,
	},
};

/*
*
*/
static int __init m80_init_fingerprint(void)
{
	printk(KERN_EMERG "%s(): ++++\n", __func__);

	/* This configures the SPI controller clock. Not done here on MTK */
	spi_register_board_info(spi_fp_board_info,ARRAY_SIZE(spi_fp_board_info));

	printk(KERN_EMERG "%s(): ----\n", __func__);
	return 0;
}

rootfs_initcall(m80_init_fingerprint);

MODULE_DESCRIPTION("m80 fingerprint driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPLV2");
