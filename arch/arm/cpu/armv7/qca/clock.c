/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/arch-qcom-common/clk.h>
#include <asm/arch-qca961x/iomap.h>
#include <asm/io.h>

#define GCC_SDCC1_MISC  0x1818014

void emmc_clock_config(int mode)
{

	if (mode == MMC_IDENTIFY_MODE) {
		/* 200 KHz */
		writel(0xef, GCC_SDCC1_MISC);
	}
	if (mode == MMC_DATA_TRANSFER_MODE) {
		/* 10 MHz */
		writel(0x0, GCC_SDCC1_MISC);
	}
}
void emmc_clock_disable(void)
{
	/* Disable clock */

}
