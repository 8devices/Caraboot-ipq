/*
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _QCA961X_BOARD_PARAM_H_
#define _QCA961X_BOARD_PARAM_H_

#include <asm/arch-qca961x/iomap.h>
#include <asm/arch-qcom-common/gpio.h>
#include <asm/sizes.h>
#include "qca961x_cdp.h"

gpio_func_data_t nand_gpio_bga[] = {
	{
		.gpio = 52,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 53,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 54,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 55,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 56,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 57,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 58,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 59,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 60,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 61,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 62,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 63,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 64,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 65,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 66,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 67,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 68,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 69,
		.func = 1,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},

};

gpio_func_data_t sw_gpio_bga[] = {
	{
		.gpio = 6,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 7,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 47,
		.func = 0,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_ENABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
};

gpio_func_data_t sw_gpio_qfn[] = {
	{
		.gpio = 52,
		.func = 2,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 53,
		.func = 2,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 59,
		.func = 0,
		.pull = GPIO_NO_PULL,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_ENABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
};

#define QCA961X_EDMA_CFG_BASE		0xC080000

#define qca961x_edma_cfg(_b, _pn, _p, ...)		\
{							\
	.base		= QCA961X_EDMA_CFG_BASE,	\
	.unit		= _b,				\
	.phy		= PHY_INTERFACE_MODE_##_p,	\
	.phy_addr	= {.count = _pn, {__VA_ARGS__}}	\
}

#define qca961x_edma_cfg_invalid()	{ .unit = -1, }
/* Board specific parameter Array */
board_qca961x_params_t board_params[] = {
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK01_1_C1,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 60,
				.func = 2,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 61,
				.func = 2,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.sw_gpio = sw_gpio_qfn,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_qfn),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK01_1_C2,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 60,
				.func = 2,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 61,
				.func = 2,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.sw_gpio = sw_gpio_qfn,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_qfn),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK04_1_C1,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.nand_gpio = nand_gpio_bga,
		.nand_gpio_count = ARRAY_SIZE(nand_gpio_bga),
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK04_1_C2,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.nand_gpio = nand_gpio_bga,
		.nand_gpio_count = ARRAY_SIZE(nand_gpio_bga),
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK04_1_C3,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.nand_gpio = nand_gpio_bga,
		.nand_gpio_count = ARRAY_SIZE(nand_gpio_bga),
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
	{
		.machid = MACH_TYPE_IPQ40XX_DB152,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
	{
		.machid = MACH_TYPE_IPQ40XX_DB153,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
	{
		.machid = MACH_TYPE_IPQ40XX_TB832,
		.ddr_size = (256 << 20),
		.uart_dm_base = UART1_DM_BASE,
		.dbg_uart_gpio = {
			{
				.gpio = 16,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
			{
				.gpio = 17,
				.func = 1,
				.pull = GPIO_NO_PULL,
				.oe = GPIO_OE_ENABLE
			},
		},
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			qca961x_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
	},
};

#define NUM_IPQ40XX_BOARDS	ARRAY_SIZE(board_params)
#endif /* _QCA961X_BOARD_PARAM_H_ */
