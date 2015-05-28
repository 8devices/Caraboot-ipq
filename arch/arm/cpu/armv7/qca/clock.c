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
#define GCC_SDCC1_APPS_CBCR 0x181800C
#define GCC_SDCC1_APPS_RCGR 0x1818008
#define GCC_SDCC1_APPS_CMD_RCGR 0x1818004

void emmc_clock_config(int mode)
{
	/* Select SDCC clock source as DDR_PLL_SDCC1_CLK  192MHz */
	writel(0x100, GCC_SDCC1_APPS_RCGR);
	/* Update APPS_CMD_RCGR to reflect source selection */
	writel(0x1, GCC_SDCC1_APPS_CMD_RCGR);
	udelay(10);

	if (mode == MMC_IDENTIFY_MODE) {
		/* Set root clock generator to bypass mode */
		writel(0x0, GCC_SDCC1_APPS_CBCR);
		udelay(10);
		/* Choose divider for 400KHz */
		writel(0x1e4 , GCC_SDCC1_MISC);
		/* Enable root clock generator */
		writel(0x1, GCC_SDCC1_APPS_CBCR);
		udelay(10);
	}
	if (mode == MMC_DATA_TRANSFER_MODE) {
		/* Set root clock generator to bypass mode */
		writel(0x0, GCC_SDCC1_APPS_CBCR);
		udelay(10);
		/* Choose divider for 48MHz */
		writel(0x3, GCC_SDCC1_MISC);
		/* Enable root clock generator */
		writel(0x1, GCC_SDCC1_APPS_CBCR);
		udelay(10);
	}
}
void emmc_clock_disable(void)
{
	/* Clear divider */
	writel(0x0, GCC_SDCC1_MISC);

}
