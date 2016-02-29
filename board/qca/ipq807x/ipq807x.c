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

DECLARE_GLOBAL_DATA_PTR;

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
