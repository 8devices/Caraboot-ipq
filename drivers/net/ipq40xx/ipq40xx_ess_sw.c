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
	u32 data;
	switch(gboard_param->machid) {
	case MACH_TYPE_IPQ40XX_AP_DK01_1_C1:
	case MACH_TYPE_IPQ40XX_AP_DK01_1_C2:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C1:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C2:
	case MACH_TYPE_IPQ40XX_AP_DK04_1_C3:
		ipq40xx_ess_sw_wr(S17_GLOFW_CTRL1_REG, 0x3e3e3e);
		/*
		 * configure Speed, Duplex.
		 */
		ipq40xx_ess_sw_wr(S17_P0STATUS_REG, S17_PORT_SPEED(2) |
						S17_PORT_FULL_DUP |
						S17_TX_FLOW_EN |
						S17_RX_FLOW_EN);

		ipq40xx_ess_sw_wr(S17_P0LOOKUP_CTRL_REG, 0x14003e);
		ipq40xx_ess_sw_wr(S17_P1LOOKUP_CTRL_REG, 0x14001d);
		ipq40xx_ess_sw_wr(S17_P2LOOKUP_CTRL_REG, 0x14001b);
		ipq40xx_ess_sw_wr(S17_P3LOOKUP_CTRL_REG, 0x140017);
		ipq40xx_ess_sw_wr(S17_P4LOOKUP_CTRL_REG, 0x14000f);
		ipq40xx_ess_sw_wr(S17_P5LOOKUP_CTRL_REG, 0x140001);
		/*
		 * HOL setting for Port0
		 */
		ipq40xx_ess_sw_wr(S17_PORT0_HOL_CTRL0, 0x1e444444);
		ipq40xx_ess_sw_wr(S17_PORT0_HOL_CTRL1, 0x1c6);
		/*
		 * HOL setting for Port1
		 */
		ipq40xx_ess_sw_wr(S17_PORT1_HOL_CTRL0, 0x1e004444);
		ipq40xx_ess_sw_wr(S17_PORT1_HOL_CTRL1, 0x1c6);
		/*
		 * HOL setting for Port2
		 */
		ipq40xx_ess_sw_wr(S17_PORT2_HOL_CTRL0, 0x1e004444);
		ipq40xx_ess_sw_wr(S17_PORT2_HOL_CTRL1, 0x1c6);
		/*
		 * HOL setting for Port3
		 */
		ipq40xx_ess_sw_wr(S17_PORT3_HOL_CTRL0, 0x1e004444);
		ipq40xx_ess_sw_wr(S17_PORT3_HOL_CTRL1, 0x1c6);
		/*
		 * HOL setting for Port4
		 */
		ipq40xx_ess_sw_wr(S17_PORT4_HOL_CTRL0, 0x1e004444);
		ipq40xx_ess_sw_wr(S17_PORT4_HOL_CTRL1, 0x1c6);
		/*
		 * HOL setting for Port5
		 */
		ipq40xx_ess_sw_wr(S17_PORT5_HOL_CTRL0, 0x1e444444);
		ipq40xx_ess_sw_wr(S17_PORT5_HOL_CTRL1, 0x1c6);

		ipq40xx_ess_sw_wr(S17_P1STATUS_REG, 0x1280);
		ipq40xx_ess_sw_wr(S17_P2STATUS_REG, 0x1280);
		ipq40xx_ess_sw_wr(S17_P3STATUS_REG, 0x1280);
		ipq40xx_ess_sw_wr(S17_P4STATUS_REG, 0x1280);
		ipq40xx_ess_sw_wr(S17_P5STATUS_REG, 0x1280);

		mdelay(1);
		/*
		 * Enable Rx and Tx mac.
		 */
		ipq40xx_ess_sw_rd(S17_P0STATUS_REG, &data);
		ipq40xx_ess_sw_wr(S17_P0STATUS_REG, data |
		                        S17_PORT_TX_MAC_EN |
		                        S17_PORT_RX_MAC_EN);
		ipq40xx_ess_sw_wr(S17_GLOFW_CTRL1_REG, 0x7f7f7f);
		printf ("%s done\n", __func__);
		break;
	default:
		break;
	}
	return 0;
}

