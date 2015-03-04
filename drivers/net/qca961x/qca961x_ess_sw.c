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

#include "qca961x_ess_sw.h"

extern board_qca961x_params_t *gboard_param;
static inline qca961x_ess_sw_rd(u32 addr, u32 * data)
{
	*data = readl((void __iomem *)(QCA961X_NSS_BASE + addr));
}

static inline qca961x_ess_sw_wr(u32 addr, u32 data)
{
	writel(data, ((void __iomem *)(QCA961X_NSS_BASE + addr)));
}

int qca961x_ess_sw_init(qca961x_edma_board_cfg_t *cfg)
{
	switch(gboard_param->machid) {
	case MACH_TYPE_QCA961X_RUMI:
		qca961x_ess_sw_wr(S17_P0STATUS_REG, 0x4e);
		qca961x_ess_sw_wr(S17_P1STATUS_REG, 0x4e);
		qca961x_ess_sw_wr(S17_P2STATUS_REG, 0x4e);
		qca961x_ess_sw_wr(S17_P3STATUS_REG, 0x4e);
		qca961x_ess_sw_wr(S17_P4STATUS_REG, 0x4e);
		qca961x_ess_sw_wr(S17_P5STATUS_REG, 0x4e);

		qca961x_ess_sw_wr(S17_GLOFW_CTRL1_REG, 0x7f7f7f);
		qca961x_ess_sw_wr(S17_P0LOOKUP_CTRL_REG, 0x14003e);
		qca961x_ess_sw_wr(S17_P1LOOKUP_CTRL_REG, 0x14001d);
		qca961x_ess_sw_wr(S17_P2LOOKUP_CTRL_REG, 0x14001b);
		qca961x_ess_sw_wr(S17_P3LOOKUP_CTRL_REG, 0x140017);
		qca961x_ess_sw_wr(S17_P4LOOKUP_CTRL_REG, 0x14000f);
		qca961x_ess_sw_wr(S17_P5LOOKUP_CTRL_REG, 0x140001);
		printf ("%s done\n", __func__);
		break;
	default:
		break;
	}
	return 0;
}

