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

#ifndef _IPQ40XX_BOARD_PARAM_H_
#define _IPQ40XX_BOARD_PARAM_H_

#include <asm/arch-ipq40xx/iomap.h>
#include <asm/arch-qcom-common/gpio.h>
#include <asm/sizes.h>
#include "ipq40xx_cdp.h"

gpio_func_data_t mmc_ap_dk04[] = {
	{
		.gpio = 23,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 24,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 25,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 26,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 27,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 28,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 29,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 30,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 31,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 32,
		.func = 1,
		.pull = GPIO_NO_PULL,
		.drvstr = GPIO_10MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
};


gpio_func_data_t spi_nor_bga[] = {
	{
		.gpio = 12,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 13,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 14,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
	{
		.gpio = 15,
		.func = 1,
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
};

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

gpio_func_data_t rgmii_gpio_cfg[] = {
	{
		.gpio = 22,
		.func = 1,	/* RGMMI0 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 23,
		.func = 2,	/* RGMII1 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 24,
		.func = 2,	/* RGMII2 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 25,
		.func = 2,	/* RGMII3 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 26,
		.func = 2,	/* RGMII RX */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 27,
		.func = 2,	/* RGMII_TXC */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 28,
		.func = 2,	/* RGMII0 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 29,
		.func = 2,	/* RGMII1 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 30,
		.func = 2,	/* RGMII2 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 31,
		.func = 2,	/* RGMII3 */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 32,
		.func = 2,	/* RGMII RX_C */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
	},
	{
		.gpio = 33,
		.func = 1,	/* RGMII TX */
		.pull = GPIO_PULL_UP,
		.drvstr = GPIO_16MA,
		.oe = GPIO_OE_DISABLE,
		.gpio_vm = GPIO_VM_DISABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES0
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

gpio_func_data_t ap_dk04_1_c2_sw_gpio_bga[] = {
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
		.gpio = 67,
		.func = 0,
		.pull = GPIO_PULL_DOWN,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_ENABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
};

gpio_func_data_t db_dk_2_1_sw_gpio_bga[] = {
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

gpio_func_data_t ap_dk01_1_c2_sw_gpio_qfn[] = {
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
		.gpio = 62,
		.func = 0,
		.pull = GPIO_NO_PULL,
		.drvstr = GPIO_2MA,
		.oe = GPIO_OE_ENABLE,
		.gpio_vm = GPIO_VM_ENABLE,
		.gpio_od_en = GPIO_OD_DISABLE,
		.gpio_pu_res = GPIO_PULL_RES2
	},
};

gpio_func_data_t uart1_gpio_dk01[] = {
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
};

gpio_func_data_t uart1_gpio_dk04[] = {
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
};

gpio_func_data_t uart2_gpio[] = {
	{
		.gpio = 8,
		.func = 1,
		.pull = GPIO_NO_PULL,
		.oe = GPIO_OE_ENABLE
	},
	{
		.gpio = 9,
		.func = 1,
		.pull = GPIO_NO_PULL,
		.oe = GPIO_OE_ENABLE
	},
};

#ifdef CONFIG_IPQ40XX_I2C
gpio_func_data_t i2c0_gpio[] = {
	{
		.gpio = 20,
		.func = 1,
		.pull = GPIO_NO_PULL,
		.oe = GPIO_OE_ENABLE
	},
	{
		.gpio = 21,
		.func = 1,
		.pull = GPIO_NO_PULL,
		.oe = GPIO_OE_ENABLE
	},
};
#endif

uart_cfg_t uart1_console_uart_dk01 = {
	.uart_dm_base = UART1_DM_BASE,
	.dbg_uart_gpio = uart1_gpio_dk01,
};

uart_cfg_t uart1_console_uart_dk04 = {
	.uart_dm_base = UART1_DM_BASE,
	.dbg_uart_gpio = uart1_gpio_dk04,
};

uart_cfg_t uart2 = {
	.uart_dm_base = UART2_DM_BASE,
	.dbg_uart_gpio = uart2_gpio,
};

#ifdef CONFIG_IPQ40XX_I2C
i2c_cfg_t i2c0 = {
	.i2c_base = I2C0_BASE,
	.i2c_gpio = i2c0_gpio,
};
#endif

#define ipq40xx_edma_cfg(_b, _pn, _p, ...)		\
{							\
	.base		= IPQ40XX_EDMA_CFG_BASE,	\
	.unit		= _b,				\
	.phy		= PHY_INTERFACE_MODE_##_p,	\
	.phy_addr	= {.count = _pn, {__VA_ARGS__}},\
	.phy_name	= "IPQ MDIO"#_b			\
}

#define ipq40xx_edma_cfg_invalid()	{ .unit = -1, }
/* Board specific parameter Array */
board_ipq40xx_params_t board_params[] = {
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK01_1_S1,
		.ddr_size = (128 << 20),
		.mtdids = "nand2=spi0.0",
		.console_uart_cfg = &uart1_console_uart_dk01,
		.sw_gpio = ap_dk01_1_c2_sw_gpio_qfn,
		.sw_gpio_count = ARRAY_SIZE(ap_dk01_1_c2_sw_gpio_qfn),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "#config@4",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK01_1_C1,
		.ddr_size = (256 << 20),
		.mtdids = "nand2=spi0.0",
		.console_uart_cfg = &uart1_console_uart_dk01,
		.sw_gpio = sw_gpio_qfn,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_qfn),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "#config@4",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK01_1_C2,
		.ddr_size = (256 << 20),
		.mtdids = "nand1=nand1,nand2=spi0.0",
		.console_uart_cfg = &uart1_console_uart_dk01,
		.sw_gpio = ap_dk01_1_c2_sw_gpio_qfn,
		.sw_gpio_count = ARRAY_SIZE(ap_dk01_1_c2_sw_gpio_qfn),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
		.spi_nand_available = 1,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "#config@5",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK04_1_C1,
		.ddr_size = (256 << 20),
		.mtdids = "nand0=nand0,nand2=spi0.0",
		.spi_nor_gpio = spi_nor_bga,
		.spi_nor_gpio_count = ARRAY_SIZE(spi_nor_bga),
		.nand_gpio = nand_gpio_bga,
		.nand_gpio_count = ARRAY_SIZE(nand_gpio_bga),
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
		.uart_cfg = &uart2,
		.console_uart_cfg = &uart1_console_uart_dk04,
#ifdef CONFIG_IPQ40XX_I2C
		.i2c_cfg = &i2c0,
#endif
		.mmc_gpio = mmc_ap_dk04,
		.mmc_gpio_count = ARRAY_SIZE(mmc_ap_dk04),
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "#config@1",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK04_1_C2,
		.ddr_size = (256 << 20),
		.mtdids = "nand2=spi0.0",
		.uart_cfg = &uart2,
		.console_uart_cfg = &uart1_console_uart_dk04,
		.spi_nor_gpio = spi_nor_bga,
		.spi_nor_gpio_count = ARRAY_SIZE(spi_nor_bga),
		.sw_gpio = ap_dk04_1_c2_sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(ap_dk04_1_c2_sw_gpio_bga),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
		.mmc_gpio = mmc_ap_dk04,
		.mmc_gpio_count = ARRAY_SIZE(mmc_ap_dk04),
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "#config@2",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_AP_DK04_1_C3,
		.ddr_size = (256 << 20),
		.console_uart_cfg = &uart1_console_uart_dk04,
		.mtdids = "nand0=nand0,nand2=spi0.0",
		.spi_nor_gpio = spi_nor_bga,
		.spi_nor_gpio_count = ARRAY_SIZE(spi_nor_bga),
		.nand_gpio = nand_gpio_bga,
		.nand_gpio_count = ARRAY_SIZE(nand_gpio_bga),
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
		.mmc_gpio = mmc_ap_dk04,
		.mmc_gpio_count = ARRAY_SIZE(mmc_ap_dk04),
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 1,
		.dtb_config_name = "#config@3",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_DB_DK01_1_C1,
		.ddr_size = (256 << 20),
		.mtdids = "nand2=spi0.0",
		.console_uart_cfg = &uart1_console_uart_dk01,
		.sw_gpio = sw_gpio_qfn,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_qfn),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, RGMII,
					0, 1, 2, 3, 4)
		},
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "#config@6",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_DB_DK02_1_C1,
		.ddr_size = (256 << 20),
		.mtdids = "nand0=nand0,nand2=spi0.0",
		.console_uart_cfg = &uart1_console_uart_dk04,
		.spi_nor_gpio = spi_nor_bga,
		.spi_nor_gpio_count = ARRAY_SIZE(spi_nor_bga),
		.nand_gpio = nand_gpio_bga,
		.nand_gpio_count = ARRAY_SIZE(nand_gpio_bga),
		.sw_gpio = db_dk_2_1_sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(db_dk_2_1_sw_gpio_bga),
		.rgmii_gpio = rgmii_gpio_cfg,
		.rgmii_gpio_count = ARRAY_SIZE(rgmii_gpio_cfg),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, RGMII,
					0, 1, 2, 3, 4)
		},
		.mmc_gpio = mmc_ap_dk04,
		.mmc_gpio_count = ARRAY_SIZE(mmc_ap_dk04),
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "#config@7",
	},
	{
		.machid = MACH_TYPE_IPQ40XX_TB832,
		.ddr_size = (256 << 20),
		.console_uart_cfg = &uart1_console_uart_dk04,
		.sw_gpio = sw_gpio_bga,
		.sw_gpio_count = ARRAY_SIZE(sw_gpio_bga),
		.edma_cfg = {
			ipq40xx_edma_cfg(0, 5, PSGMII,
					0, 1, 2, 3, 4)
		},
		.spi_nand_available = 0,
		.nor_nand_available = 0,
		.nor_emmc_available = 0,
		.dtb_config_name = "",
	},
};

#define NUM_IPQ40XX_BOARDS	ARRAY_SIZE(board_params)
#endif /* _IPQ40XX_BOARD_PARAM_H_ */
