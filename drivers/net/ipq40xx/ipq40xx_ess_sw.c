/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <common.h>
#include <net.h>
#include <asm-generic/errno.h>
#include <asm/io.h>

#include "ipq40xx_ess_sw.h"

extern board_ipq40xx_params_t *gboard_param;
static inline ipq40xx_ess_sw_rd(u32 addr, u32 * data)
{
	*data = readl((void __iomem *)(IPQ40XX_NSS_BASE + addr));
}

static inline ipq40xx_ess_sw_wr(u32 addr, u32 data)
{
	writel(data, ((void __iomem *)(IPQ40XX_NSS_BASE + addr)));
}

int ipq40xx_ess_sw_init(ipq40xx_edma_board_cfg_t *cfg)
{
	switch(gboard_param->machid) {
	case MACH_TYPE_IPQ40XX_AP_DK01_1_C1:
	case MACH_TYPE_IPQ40XX_AP_DK01_1_C2:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C1:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C2:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C3:
		ipq40xx_ess_sw_wr(S17_P0STATUS_REG, 0x4e);

		ipq40xx_ess_sw_wr(S17_GLOFW_CTRL1_REG, 0x7f7f7f);
		ipq40xx_ess_sw_wr(S17_P0LOOKUP_CTRL_REG, 0x14003e);
		ipq40xx_ess_sw_wr(S17_P1LOOKUP_CTRL_REG, 0x14001d);
		ipq40xx_ess_sw_wr(S17_P2LOOKUP_CTRL_REG, 0x14001b);
		ipq40xx_ess_sw_wr(S17_P3LOOKUP_CTRL_REG, 0x140017);
		ipq40xx_ess_sw_wr(S17_P4LOOKUP_CTRL_REG, 0x14000f);
		ipq40xx_ess_sw_wr(S17_P5LOOKUP_CTRL_REG, 0x140001);
		printf ("%s done\n", __func__);
		break;
	default:
		break;
	}
	return 0;
}

