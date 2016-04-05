/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <environment.h>

#include "ipq807x.h"
#include "../common/qca_common.h"

DECLARE_GLOBAL_DATA_PTR;

qca_mmc mmc_host;

void enable_caches(void)
{
	icache_enable();
}

void disable_caches(void)
{
	icache_disable();
}


int board_init(void)
{
	return 0;
}

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;
	return 0;
}

unsigned long timer_read_counter(void)
{
	return 0;
}

void reset_cpu(unsigned long a)
{
	while(1);
}

void emmc_clock_config(int mode)
{
	/* TODO: To be filled */
}

int board_mmc_init(bd_t *bis)
{
	int ret;

	mmc_host.base = MSM_SDC1_BASE;
	mmc_host.clk_mode = MMC_IDENTIFY_MODE;
	emmc_clock_config(mmc_host.clk_mode);

	ret = qca_mmc_init(bis, &mmc_host);

	return ret;
}
