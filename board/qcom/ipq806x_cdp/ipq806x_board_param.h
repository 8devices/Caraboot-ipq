/* * Copyright (c) 2013 Qualcomm Atheros, Inc. * */

#ifndef   _IPQ806X_BOARD_PARAM_H_
#define   _IPQ806X_BOARD_PARAM_H_

#include <asm/arch-ipq806x/iomap.h>
#include "ipq806x_cdp.h"
#include <asm/arch-ipq806x/gpio.h>
#include <asm/arch-ipq806x/nss/msm_ipq806x_gmac.h>

/* Board specific parameter Array */
board_ipq806x_params_t board_params[] = {
#if 0
	/*
	 * Replicate DB149 details for RUMI until, the board no.s are
	 * properly sorted out
	 */
	{
		.boardid = 0,
		.machid = MACH_TYPE_IPQ806X_RUMI3,
		.ddr_size = (256 << 20),
		.uart_gsbi = GSBI_1,
		.uart_gsbi_base = UART_GSBI1_BASE,
		.uart_dm_base = UART1_DM_BASE,
		.mnd_value = { 48, 125, 63 },
		.flashdesc = NAND_NOR,
		.flash_param = {
			.mode =	NOR_SPI_MODE_0,
			.bus_number = GSBI_BUS_5,
			.chip_select = SPI_CS_0,
			.vendor = SPI_NOR_FLASH_VENDOR_SPANSION,
		},
		.dbg_uart_gpio = {
			{
				.gpio = 51,
				.func = 1,
				.dir = GPIO_OUTPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
			{
				.gpio = 52,
				.func = 1,
				.dir = GPIO_INPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
		},
		.clk_dummy = 1,
	},
#else
	{
		.boardid = 0,
		.machid = MACH_TYPE_IPQ806X_RUMI3,
		.ddr_size = (256 << 20),
		.uart_gsbi = GSBI_2,
		.uart_gsbi_base = UART_GSBI2_BASE,
		.uart_dm_base = UART2_DM_BASE,
		.mnd_value = { 12, 625, 313 },
		.flashdesc = NAND_NOR,
		.flash_param = {
			.mode =	NOR_SPI_MODE_0,
			.bus_number = GSBI_BUS_5,
			.chip_select = SPI_CS_0,
			.vendor = SPI_NOR_FLASH_VENDOR_SPANSION,
		},
		.dbg_uart_gpio = {
			{
				.gpio = 22,
				.func = 1,
				.dir = GPIO_OUTPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
			{
				.gpio = 23,
				.func = 1,
				.dir = GPIO_INPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
		}

	},
#endif
	{
		.boardid = 0,
		.machid = MACH_TYPE_IPQ806X_DB149,
		.ddr_size = (256 << 20),
		.uart_gsbi = GSBI_2,
		.uart_gsbi_base = UART_GSBI2_BASE,
		.uart_dm_base = UART2_DM_BASE,
		.mnd_value = { 12, 625, 313 },
		.flashdesc = NAND_NOR,
		.flash_param = {
			.mode =	NOR_SPI_MODE_0,
			.bus_number = GSBI_BUS_5,
			.chip_select = SPI_CS_0,
			.vendor = SPI_NOR_FLASH_VENDOR_SPANSION,
		},
		.dbg_uart_gpio = {
			{
				.gpio = 22,
				.func = 1,
				.dir = GPIO_OUTPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
			{
				.gpio = 23,
				.func = 1,
				.dir = GPIO_INPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
		}
	},
	{
		.boardid = 0,
		.machid = MACH_TYPE_IPQ806X_TB726,
		.ddr_size = (256 << 20),
		.uart_gsbi = GSBI_4,
		.uart_gsbi_base = UART_GSBI4_BASE,
		.uart_dm_base = UART4_DM_BASE,
		.mnd_value = { 12, 625, 313 },
		.flashdesc = NAND_NOR,
		.flash_param = {
			.mode =	NOR_SPI_MODE_0,
			.bus_number = GSBI_BUS_5,
			.chip_select = SPI_CS_0,
			.vendor = SPI_NOR_FLASH_VENDOR_SPANSION,
		},
		.dbg_uart_gpio = {
			{
				.gpio = 22,
				.func = 1,
				.dir = GPIO_OUTPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
			{
				.gpio = 23,
				.func = 1,
				.dir = GPIO_INPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
		}

	},
	{
		.boardid = 0,
		.machid = MACH_TYPE_IPQ806X_DB147,
		.ddr_size = (2 << 30),
		.uart_gsbi = GSBI_4,
		.uart_gsbi_base = UART_GSBI4_BASE,
		.uart_dm_base = UART4_DM_BASE,
		.mnd_value = { 12, 625, 313 },
		.flashdesc = NAND_NOR,
		.flash_param = {
			.mode =	NOR_SPI_MODE_0,
			.bus_number = GSBI_BUS_5,
			.chip_select = SPI_CS_0,
			.vendor = SPI_NOR_FLASH_VENDOR_SPANSION,
		},
		.dbg_uart_gpio = {
			{
				.gpio = 11,
				.func = 1,
				.dir = GPIO_OUTPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
			{
				.gpio = 12,
				.func = 1,
				.dir = GPIO_INPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
		}

	},
	{
		.boardid = 0,
		.machid = MACH_TYPE_IPQ806X_AP148,
		.ddr_size = (2 << 30),
		.uart_gsbi = GSBI_4,
		.uart_gsbi_base = UART_GSBI4_BASE,
		.uart_dm_base = UART4_DM_BASE,
		.mnd_value = { 12, 625, 313 },
		.flashdesc = NAND_NOR,
		.flash_param = {
			.mode =	NOR_SPI_MODE_0,
			.bus_number = GSBI_BUS_5,
			.chip_select = SPI_CS_0,
			.vendor = SPI_NOR_FLASH_VENDOR_SPANSION,
		},
		.dbg_uart_gpio = {
			{
				.gpio = 11,
				.func = 1,
				.dir = GPIO_OUTPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
			{
				.gpio = 12,
				.func = 1,
				.dir = GPIO_INPUT,
				.pull = GPIO_NO_PULL,
				.drvstr = GPIO_12MA,
				.enable = GPIO_DISABLE
			},
		}

	},
};

#define NUM_IPQ806X_BOARDS	ARRAY_SIZE(board_params)
#endif
