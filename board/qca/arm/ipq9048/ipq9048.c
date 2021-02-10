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
#include <asm/arch-qca-common/scm.h>
#include <asm/arch-qca-common/iomap.h>
#include <ipq9048.h>
#include <mmc.h>
#include <sdhci.h>

DECLARE_GLOBAL_DATA_PTR;
struct sdhci_host mmc_host;
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

#ifdef CONFIG_QCA_MMC
void emmc_clock_config(void)
{
#ifndef CONFIG_IPQ9048_RUMI
	int cfg;

	/* Configure sdcc1_apps_clk_src */
	cfg = (GCC_SDCC1_APPS_CFG_RCGR_SRC_SEL
			| GCC_SDCC1_APPS_CFG_RCGR_SRC_DIV);
	writel(cfg, GCC_SDCC1_APPS_CFG_RCGR);
	writel(SDCC1_M_VAL, GCC_SDCC1_APPS_M);
	writel(SDCC1_N_VAL, GCC_SDCC1_APPS_N);
	writel(SDCC1_D_VAL, GCC_SDCC1_APPS_D);
	writel(CMD_UPDATE, GCC_SDCC1_APPS_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_SDCC1_APPS_CMD_RCGR);

	/* Configure CBCRs */
	writel(readl(GCC_SDCC1_APPS_CBCR) | CLK_ENABLE, GCC_SDCC1_APPS_CBCR);
	udelay(10);
	writel(readl(GCC_SDCC1_AHB_CBCR) | CLK_ENABLE, GCC_SDCC1_AHB_CBCR);
#endif
}

void mmc_iopad_config(struct sdhci_host *host)
{
	u32 val;
	val = sdhci_readb(host, SDHCI_VENDOR_IOPAD);
	/*set bit 15 & 16*/
	val |= 0x18000;
	writel(val, host->ioaddr + SDHCI_VENDOR_IOPAD);
}

void sdhci_bus_pwr_off(struct sdhci_host *host)
{
	u32 val;

	val = sdhci_readb(host, SDHCI_HOST_CONTROL);
	sdhci_writeb(host,(val & (~SDHCI_POWER_ON)), SDHCI_POWER_CONTROL);
}

void emmc_clock_disable(void)
{
#ifndef CONFIG_IPQ9048_RUMI
	/* Clear divider */
	writel(0x0, GCC_SDCC1_MISC);
#endif
}

void board_mmc_deinit(void)
{
	emmc_clock_disable();
}

void emmc_clock_reset(void)
{
#ifndef CONFIG_IPQ9048_RUMI
	writel(0x1, GCC_SDCC1_BCR);
	udelay(10);
	writel(0x0, GCC_SDCC1_BCR);
#endif
}

int board_mmc_init(bd_t *bis)
{
	int node;
	int ret = 0;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	node = fdt_path_offset(gd->fdt_blob, "mmc");
	if (node < 0) {
		printf("sdhci: Node Not found, skipping initialization\n");
		return -1;
	}

	mmc_host.ioaddr = (void *)MSM_SDC1_SDHCI_BASE;
	mmc_host.voltages = MMC_VDD_165_195;
	mmc_host.version = SDHCI_SPEC_300;
	mmc_host.cfg.part_type = PART_TYPE_EFI;
	mmc_host.quirks = SDHCI_QUIRK_BROKEN_VOLTAGE;

	emmc_clock_disable();
	emmc_clock_reset();
	udelay(10);
	emmc_clock_config();

	if (add_sdhci(&mmc_host, 200000000, 400000)) {
		printf("add_sdhci fail!\n");
		return -1;
	}

	if (!ret && sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		ret = board_mmc_env_init(mmc_host);
	}

	return ret;
}
#else
int board_mmc_init(bd_t *bis)
{
	return 0;
}
#endif

void enable_caches(void)
{
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;
	smem_get_boot_flash(&sfi->flash_type,
			    &sfi->flash_index,
			    &sfi->flash_chip_select,
			    &sfi->flash_block_size,
			    &sfi->flash_density);
	icache_enable();
	/*Skips dcache_enable during JTAG recovery */
	if (sfi->flash_type)
		dcache_enable();
}

void disable_caches(void)
{
	icache_disable();
	dcache_disable();
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

void reset_crashdump(void)
{
	unsigned int ret = 0;
	qca_scm_sdi();
	ret = qca_scm_dload(CLEAR_MAGIC);
	if (ret)
		printf ("Error in reseting the Magic cookie\n");
	return;
}

void psci_sys_reset(void)
{
	__invoke_psci_fn_smc(PSCI_RESET_SMC_ID, 0, 0, 0);
}

void reset_cpu(unsigned long a)
{
	reset_crashdump();

	psci_sys_reset();

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

