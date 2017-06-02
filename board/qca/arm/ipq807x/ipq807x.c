/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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
#include <asm/arch-qca-common/qca_common.h>
#include <asm/arch-qca-common/qpic_nand.h>
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/uart.h>
#include <asm/arch-qca-common/smem.h>
#include <asm/arch-qca-common/scm.h>
#include <fdtdec.h>
#include <mmc.h>

DECLARE_GLOBAL_DATA_PTR;

#define GCNT_PSHOLD             0x004AB000
qca_mmc mmc_host;

extern loff_t board_env_offset;
extern loff_t board_env_range;
extern loff_t board_env_size;

extern int ipq_spi_init(u16);
extern int ipq807x_edma_init(void *cfg);
extern int ipq_qca8075_phy_init(struct phy_ops **ops);

const char *rsvd_node = "/reserved-memory";
const char *del_node[] = {"uboot",
			  "sbl",
			  NULL};
const add_node_t add_node[] = {{}};
static int pci_initialised;
struct dumpinfo_t dumpinfo[] = {
	{ "EBICS0.BIN", 0x40000000, 0x10000000, 0 },
	{ "CODERAM.BIN", 0x00200000, 0x00024000, 0 },
	{ "DATARAM.BIN", 0x00290000, 0x00010000, 0 },
	{ "MSGRAM.BIN", 0x00060000, 0x00005000, 0 },
};
int dump_entries = ARRAY_SIZE(dumpinfo);

void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int node;

	writel(1, GCC_BLSP1_UART1_APPS_CBCR);

	node = fdt_path_offset(gd->fdt_blob, "/serial@78B3000/serial_gpio");
	if (node < 0) {
		printf("Could not find serial_gpio node\n");
		return;
	}

	qca_gpio_init(node);
}

unsigned long timer_read_counter(void)
{
	return 0;
}

void reset_crashdump(void)
{
	unsigned int ret = 0;
	qca_scm_sdi_v8();
	ret = qca_scm_call_write(SCM_SVC_IO, SCM_IO_WRITE,
				 0x193D100, CLEAR_MAGIC);
	if (ret)
		printf ("Error in reseting the Magic cookie\n");
	return;
}

void reset_cpu(unsigned long a)
{
	reset_crashdump();
	writel(0, GCNT_PSHOLD);
	while(1);
}

void emmc_clock_config(int mode)
{
	/* Enable root clock generator */
	writel(readl(GCC_SDCC1_APPS_CBCR)|0x1, GCC_SDCC1_APPS_CBCR);
	/* Add 10us delay for CLK_OFF to get cleared */
	udelay(10);

	if (mode == MMC_IDENTIFY_MODE) {
		/* XO - 400Khz*/
		writel(0x2017, GCC_SDCC1_APPS_CFG_RCGR);
		/* Delay for clock operation complete */
		udelay(10);
		writel(0x1, GCC_SDCC1_APPS_M);
		writel(0xFC, GCC_SDCC1_APPS_N);
		writel(0xFD, GCC_SDCC1_APPS_D);
		/* Delay for clock operation complete */
		udelay(10);

	}
	if (mode == MMC_DATA_TRANSFER_MODE) {
		/* PLL0 - 50Mhz */
		writel(0x40F, GCC_SDCC1_APPS_CFG_RCGR);
		/* Delay for clock operation complete */
		udelay(10);
		writel(0x1, GCC_SDCC1_APPS_M);
		writel(0xFC, GCC_SDCC1_APPS_N);
		writel(0xFD, GCC_SDCC1_APPS_D);
		/* Delay for clock operation complete */
		udelay(10);
	}
	/* Update APPS_CMD_RCGR to reflect source selection */
	writel(readl(GCC_SDCC1_APPS_CMD_RCGR)|0x1, GCC_SDCC1_APPS_CMD_RCGR);
	/* Add 10us delay for clock update to complete */
	udelay(10);
}

void emmc_clock_disable(void)
{
	/* Clear divider */
	writel(0x0, GCC_SDCC1_MISC);
}

void board_mmc_deinit(void)
{
	emmc_clock_disable();
}

int board_eth_init(bd_t *bis)
{
	int ret=0;
	int tlmm_base = 0x1025000;

	ipq807x_register_switch(ipq_qca8075_phy_init);

	/*
	 * ethernet clk rcgr block init -- start
	 * these clk init will be moved to sbl later
	 */

	writel(0x100 ,0x01868024);
	writel(0x1 ,0x01868020);
	writel(0x2 ,0x01868020);
	writel(0x100 ,0x0186802C);
	writel(0x1 ,0x01868028);
	writel(0x2 ,0x01868028);
	writel(0x100 ,0x01868034);
	writel(0x1 ,0x01868030);
	writel(0x2 ,0x01868030);
	writel(0x100 ,0x0186803C);
	writel(0x1 ,0x01868038);
	writel(0x2 ,0x01868038);
	writel(0x100 ,0x01868044);
	writel(0x1 ,0x01868040);
	writel(0x2 ,0x01868040);
	writel(0x100 ,0x0186804C);
	writel(0x1 ,0x01868048);
	writel(0x2 ,0x01868048);
	writel(0x100 ,0x01868054);
	writel(0x1 ,0x01868050);
	writel(0x2 ,0x01868050);
	writel(0x100 ,0x0186805C);
	writel(0x1 ,0x01868058);
	writel(0x2 ,0x01868058);
	writel(0x300 ,0x01868064);
	writel(0x1 ,0x01868060);
	writel(0x2 ,0x01868060);
	writel(0x300 ,0x0186806C);
	writel(0x1 ,0x01868068);
	writel(0x2 ,0x01868068);
	writel(0x100 ,0x01868074);
	writel(0x1 ,0x01868070);
	writel(0x2 ,0x01868070);
	writel(0x100 ,0x0186807C);
	writel(0x1 ,0x01868078);
	writel(0x2 ,0x01868078);
	writel(0x101 ,0x01868084);
	writel(0x1 ,0x01868080);
	writel(0x2 ,0x01868080);
	writel(0x100 ,0x0186808C);
	writel(0x1 ,0x01868088);
	writel(0x2 ,0x01868088);

	/*
	 * ethernet clk rcgr block init -- end
	 * these clk init will be moved to sbl later
	 */

	/* bring phy out of reset */
	writel(0x203, tlmm_base);
	writel(0, tlmm_base + 0x4);
	writel(2, tlmm_base + 0x4);
	writel(7, tlmm_base + 0x1f000);
	writel(7, tlmm_base + 0x20000);

	ret = ipq807x_edma_init(NULL);

	if (ret != 0)
		printf("%s: ipq807x_edma_init failed : %d\n", __func__, ret);

	return ret;
}

int board_mmc_env_init(void)
{
	block_dev_desc_t *blk_dev;
	disk_partition_t disk_info;
	int ret;

	if (mmc_init(mmc_host.mmc)) {
		/* The HS mode command(cmd6) is getting timed out. So mmc card
		 * is not getting initialized properly. Since the env partition
		 * is not visible, the env default values are writing into the
		 * default partition (start of the mmc device).
		 * So do a reset again.
		 */
		if (mmc_init(mmc_host.mmc)) {
			printf("MMC init failed \n");
			return -1;
		}
	}
	blk_dev = mmc_get_dev(mmc_host.dev_num);
	ret = get_partition_info_efi_by_name(blk_dev,
				"0:APPSBLENV", &disk_info);

	if (ret == 0) {
		board_env_offset = disk_info.start * disk_info.blksz;
		board_env_size = disk_info.size * disk_info.blksz;
		board_env_range = board_env_size;
		BUG_ON(board_env_size > CONFIG_ENV_SIZE_MAX);
	}
	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	mmc_host.base = MSM_SDC1_BASE;
	mmc_host.clk_mode = MMC_IDENTIFY_MODE;
	emmc_clock_config(mmc_host.clk_mode);

	ret = qca_mmc_init(bis, &mmc_host);

	if (!ret && sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		ret = board_mmc_env_init();
	}

	return ret;
}


void board_nand_init(void)
{
	int gpio_node;

	qpic_nand_init();

	gpio_node = fdt_path_offset(gd->fdt_blob, "/spi/spi_gpio");
	if (gpio_node >= 0) {
		qca_gpio_init(gpio_node);
		ipq_spi_init(CONFIG_IPQ_SPI_NOR_INFO_IDX);
	}
}

static void pcie_clock_init(int id)
{

	/* Enable PCIE CLKS */
	if (id == 0) {
		writel(0x2, GCC_PCIE0_AUX_CMD_RCGR);
		writel(0x107, GCC_PCIE0_AXI_CFG_RCGR);
		writel(0x1, GCC_PCIE0_AXI_CMD_RCGR);
		mdelay(100);
		writel(0x2, GCC_PCIE0_AXI_CMD_RCGR);
		writel(0x20000001, GCC_PCIE0_AHB_CBCR);
		writel(0x4FF1, GCC_PCIE0_AXI_M_CBCR);
		writel(0x20004FF1, GCC_PCIE0_AXI_S_CBCR);
		writel(0x1, GCC_PCIE0_AUX_CBCR);
		writel(0x80004FF1, GCC_PCIE0_PIPE_CBCR);
		writel(0x2, GCC_PCIE1_AUX_CMD_RCGR);
		writel(0x107, GCC_PCIE1_AXI_CFG_RCGR);
		writel(0x1, GCC_PCIE1_AXI_CMD_RCGR);
		mdelay(100);
		writel(0x2, GCC_PCIE1_AXI_CMD_RCGR);
		writel(0x20000001, GCC_PCIE1_AHB_CBCR);
		writel(0x4FF1, GCC_PCIE1_AXI_M_CBCR);
		writel(0x20004FF1, GCC_PCIE1_AXI_S_CBCR);
		writel(0x1, GCC_PCIE1_AUX_CBCR);
		writel(0x80004FF1, GCC_PCIE1_PIPE_CBCR);
		pci_initialised = 1;
	}
}

void board_pci_init(int id)
{
	int node, gpio_node;
	char name[16];

	sprintf(name, "pci%d", id);
	node = fdt_path_offset(gd->fdt_blob, name);
	if (node < 0) {
		printf("Could not find PCI in device tree\n");
		return;
	}
	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
	if (gpio_node >= 0)
		qca_gpio_init(gpio_node);

	pcie_clock_init(id);
	return ;
}

int ipq_fdt_fixup_socinfo(void *blob)
{
	return 0;
}

void set_flash_secondary_type(qca_smem_flash_info_t *smem)
{
	return;
};
