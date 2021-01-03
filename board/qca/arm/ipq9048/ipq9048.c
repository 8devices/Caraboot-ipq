/*
 * Copyright (c) 2016-2020 The Linux Foundation. All rights reserved.
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
#include <asm/errno.h>
#include <environment.h>
#include <fdtdec.h>

#include <asm/arch-qca-common/qpic_nand.h>
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/uart.h>
#include <ipq9048.h>

DECLARE_GLOBAL_DATA_PTR;
extern int ipq_spi_init(u16);

unsigned int qpic_frequency = 0, qpic_phase = 0;

void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int ret;

	if (plat->gpio_node < 0) {
		printf("serial_init: unable to find gpio node\n");
		return;
	}

	qca_gpio_init(plat->gpio_node);
	plat->port_id = UART_PORT_ID(plat->reg_base);
	ret = uart_clock_config(plat);
	if (ret)
		printf("UART clock config failed %d\n", ret);

	return;
}

void reset_crashdump(void)
{
	return;
}

void fdt_fixup_qpic(void *blob)
{
	int node_off, ret;
	const char *qpic_node = {"/soc/qpic-nand@79b0000"};

	/* This fixup is for qpic io_macro_clk
	 * frequency & phase value
	 */
	node_off = fdt_path_offset(blob, qpic_node);
	if (node_off < 0) {
		printf("%s: QPIC: unable to find node '%s'\n",
				__func__, qpic_node);
		return;
	}

	ret = fdt_setprop_u32(blob, node_off, "qcom,iomacromax_clk", qpic_frequency);
	if (ret) {
		printf("%s : Unable to set property 'qcom,iomacromax_clk'\n",__func__);
		return;
	}

	ret = fdt_setprop_u32(blob, node_off, "qcom,phase", qpic_phase);
	if (ret) {
		printf("%s : Unable to set property 'qcom,phase'\n",__func__);
		return;
	}
}

#ifdef CONFIG_QPIC_NAND
void qpic_set_clk_rate(unsigned int clk_rate, int blk_type, int req_clk_src_type)
{
#ifndef CONFIG_IPQ9048_RUMI
	switch (blk_type) {
		case QPIC_IO_MACRO_CLK:
			/* select the clk source for IO_PAD_MACRO
			 * clk source wil be either XO = 24MHz. or GPLL0 = 800MHz.
			 */
			if (req_clk_src_type == XO_CLK_SRC) {
				/* default XO clock will enabled
				 * i.e XO clock = 24MHz.
				 * so div value will 0.
				 * Input clock to IO_MACRO will be divided by 4 by default
				 * by hardware and then taht clock will be go on bus.
				 * i.e 24/4MHz = 6MHz i.e 6MHz will go onto the bus.
				 */
				writel(0x0, GCC_QPIC_IO_MACRO_CFG_RCGR);

			} else if (req_clk_src_type == GPLL0_CLK_SRC) {
				/* selct GPLL0 clock source 800MHz
				 * so 800/4 = 200MHz.
				 * Input clock to IO_MACRO will be divided by 4 by default
				 * by hardware and then that clock will be go on bus.
				 * i.e 200/4MHz = 50MHz i.e 50MHz will go onto the bus.
				 */
				if (clk_rate == IO_MACRO_CLK_320_MHZ)
				       writel(0x104, GCC_QPIC_IO_MACRO_CFG_RCGR);
				else if (clk_rate == IO_MACRO_CLK_266_MHZ)
					writel(0x105, GCC_QPIC_IO_MACRO_CFG_RCGR);
				else if (clk_rate == IO_MACRO_CLK_228_MHZ)
					writel(0x106, GCC_QPIC_IO_MACRO_CFG_RCGR);
				else if (clk_rate == IO_MACRO_CLK_100_MHZ)
					writel(0x10F, GCC_QPIC_IO_MACRO_CFG_RCGR);
				else if (clk_rate == IO_MACRO_CLK_200_MHZ)
					writel(0x107, GCC_QPIC_IO_MACRO_CFG_RCGR);

			} else {
				printf("wrong clk src selection requested.\n");
			}

			/* Enablle update bit to update the new configuration */
			writel((UPDATE_EN | readl(GCC_QPIC_IO_MACRO_CMD_RCGR)),
					GCC_QPIC_IO_MACRO_CMD_RCGR);

			/* Enable the QPIC_IO_MACRO_CLK */
			writel(0x1, GCC_QPIC_IO_MACRO_CBCR);

		       break;
		case QPIC_CORE_CLK:
		       /* To DO if needed for QPIC core clock setting */
		       break;
		default:
		       printf("wrong qpic block type\n");
		       break;
	}
#endif
}
#endif

void board_nand_init(void)
{
#ifdef CONFIG_QPIC_SERIAL
	/* check for nand node in dts
	 * if nand node in dts is disabled then
	 * simply return from here without
	 * initializing
	 */
	int node;
	node = fdt_path_offset(gd->fdt_blob, "/nand-controller");
	if (!fdtdec_get_is_enabled(gd->fdt_blob, node)) {
		printf("QPIC: disabled, skipping initialization\n");
	} else {
		qpic_nand_init(NULL);
	}
#endif

#ifdef CONFIG_QCA_SPI
	int gpio_node;
	gpio_node = fdt_path_offset(gd->fdt_blob, "/spi/spi_gpio");
	if (gpio_node >= 0) {
		qca_gpio_init(gpio_node);
		ipq_spi_init(CONFIG_IPQ_SPI_NOR_INFO_IDX);
	}
#endif
}

void enable_caches(void)
{
	icache_enable();
}

void disable_caches(void)
{
	icache_disable();
}

/**
 * Set the uuid in bootargs variable for mounting rootfilesystem
 */
int set_uuid_bootargs(char *boot_args, char *part_name, int buflen, bool gpt_flag)
{
	return 0;
}

unsigned long timer_read_counter(void)
{
	return 0;
}

void reset_cpu(unsigned long a)
{
	while (1);
}

void reset_board(void)
{
	run_command("reset", 0);
}

void set_flash_secondary_type(qca_smem_flash_info_t *smem)
{
	return;
};

const char *rsvd_node = "/reserved-memory";
const char *del_node[] = {"uboot",
			  "sbl",
			  NULL};
const add_node_t add_fdt_node[] = {{}};

__weak int ipq_get_tz_version(char *version_name, int buf_size)
{
	return 1;
}

void ipq_fdt_fixup_socinfo(void *blob)
{
	return;
}

void ipq_fdt_fixup_usb_device_mode(void *blob)
{
	return;
}

void fdt_fixup_auto_restart(void *blob)
{
	return;
}

