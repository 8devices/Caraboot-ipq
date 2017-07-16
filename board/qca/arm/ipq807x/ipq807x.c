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
#include <asm/errno.h>
#include <environment.h>
#include <asm/arch-qca-common/qca_common.h>
#include <asm/arch-qca-common/qpic_nand.h>
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/uart.h>
#include <asm/arch-qca-common/smem.h>
#include <asm/arch-qca-common/scm.h>
#include <fdtdec.h>
#include <mmc.h>
#include <usb.h>

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
	{ "MSGRAM.BIN", 0x00060000, 0x00006000, 1 },
	{ "IMEM.BIN", 0x08600000, 0x00006000, 0 },
};
int dump_entries = ARRAY_SIZE(dumpinfo);

void uart2_configure_mux(void)
{
	unsigned long cfg_rcgr;

	cfg_rcgr = readl(GCC_BLSP1_UART2_APPS_CFG_RCGR);
	/* Clear mode, src sel, src div */
	cfg_rcgr &= ~(GCC_UART_CFG_RCGR_MODE_MASK |
			GCC_UART_CFG_RCGR_SRCSEL_MASK |
			GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART2_RCGR_SRC_SEL << GCC_UART_CFG_RCGR_SRCSEL_SHIFT)
			& GCC_UART_CFG_RCGR_SRCSEL_MASK);

	cfg_rcgr |= ((UART2_RCGR_SRC_DIV << GCC_UART_CFG_RCGR_SRCDIV_SHIFT)
			& GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART2_RCGR_MODE << GCC_UART_CFG_RCGR_MODE_SHIFT)
			& GCC_UART_CFG_RCGR_MODE_MASK);

	writel(cfg_rcgr, GCC_BLSP1_UART2_APPS_CFG_RCGR);
}

void uart2_set_rate_mnd(unsigned int m,
			unsigned int n, unsigned int two_d)
{
	writel(m, GCC_BLSP1_UART2_APPS_M);
	writel(NOT_N_MINUS_M(n, m), GCC_BLSP1_UART2_APPS_N);
	writel(NOT_2D(two_d), GCC_BLSP1_UART2_APPS_D);
}

int uart2_trigger_update(void)
{
	unsigned long cmd_rcgr;
	int timeout = 0;

	cmd_rcgr = readl(GCC_BLSP1_UART2_APPS_CMD_RCGR);
	cmd_rcgr |= UART2_CMD_RCGR_UPDATE;
	writel(cmd_rcgr, GCC_BLSP1_UART2_APPS_CMD_RCGR);

	while (readl(GCC_BLSP1_UART2_APPS_CMD_RCGR) & UART2_CMD_RCGR_UPDATE) {
		if (timeout++ >= CLOCK_UPDATE_TIMEOUT_US) {
			printf("Timeout waiting for UART2 clock update\n");
			return -ETIMEDOUT;
		}
		udelay(1);
	}
	cmd_rcgr = readl(GCC_BLSP1_UART2_APPS_CMD_RCGR);
	return 0;
}

void uart2_toggle_clock(void)
{
	unsigned long cbcr_val;

	cbcr_val = readl(GCC_BLSP1_UART2_APPS_CBCR);
	cbcr_val |= UART2_CBCR_CLK_ENABLE;
	writel(cbcr_val, GCC_BLSP1_UART2_APPS_CBCR);
}

void uart2_clock_config(unsigned int m,
			unsigned int n, unsigned int two_d)
{
	uart2_configure_mux();
	uart2_set_rate_mnd(m, n, two_d);
	uart2_trigger_update();
	uart2_toggle_clock();
}

void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int node, uart2_node;

	writel(1, GCC_BLSP1_UART1_APPS_CBCR);

	node = fdt_path_offset(gd->fdt_blob, "/serial@78B3000/serial_gpio");
	if (node < 0) {
		printf("Could not find serial_gpio node\n");
		return;
	}

	if (plat->port_id == 1) {
		uart2_node = fdt_path_offset(gd->fdt_blob, "uart2");
		if (uart2_node < 0) {
			printf("Could not find uart2 node\n");
			return;
		}
		node = fdt_subnode_offset(gd->fdt_blob,
				uart2_node, "serial_gpio");
		uart2_clock_config(plat->m_value, plat->n_value, plat->d_value);
		writel(1, GCC_BLSP1_UART2_APPS_CBCR);
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
	set_ethmac_addr();

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

static void pcie_clock_deinit(int id)
{

	if (id == 0) {
		writel(0x0, GCC_PCIE0_AUX_CMD_RCGR);
		writel(0x0, GCC_PCIE0_AXI_CFG_RCGR);
		writel(0x0, GCC_PCIE0_AXI_CMD_RCGR);
		mdelay(100);
		writel(0x0, GCC_SYS_NOC_PCIE0_AXI_CLK);
		writel(0x0, GCC_PCIE0_AHB_CBCR);
		writel(0x0, GCC_PCIE0_AXI_M_CBCR);
		writel(0x0, GCC_PCIE0_AXI_S_CBCR);
		writel(0x0, GCC_PCIE0_AUX_CBCR);
		writel(0x0, GCC_PCIE0_PIPE_CBCR);
		writel(0x0, GCC_PCIE1_AUX_CMD_RCGR);
		writel(0x0, GCC_PCIE1_AXI_CFG_RCGR);
		writel(0x0, GCC_PCIE1_AXI_CMD_RCGR);
		mdelay(100);
		writel(0x0, GCC_SYS_NOC_PCIE1_AXI_CLK);
		writel(0x0, GCC_PCIE1_AHB_CBCR);
		writel(0x0, GCC_PCIE1_AXI_M_CBCR);
		writel(0x0, GCC_PCIE1_AXI_S_CBCR);
		writel(0x0, GCC_PCIE1_AUX_CBCR);
		writel(0x0, GCC_PCIE1_PIPE_CBCR);
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

void board_pci_deinit()
{
	int node, gpio_node, i, err;
	char name[16];
	struct fdt_resource parf;
	struct fdt_resource pci_phy;

	for (i = 0; i < PCI_MAX_DEVICES; i++) {
		sprintf(name, "pci%d", i);
		node = fdt_path_offset(gd->fdt_blob, name);
		if (node < 0) {
			printf("Could not find PCI in device tree\n");
			return;
		}
		err = fdt_get_named_resource(gd->fdt_blob, node, "reg", "reg-names", "parf",
				&parf);
		writel(0x0, parf.start + 0x358);
		writel(0x1, parf.start + 0x40);
		err = fdt_get_named_resource(gd->fdt_blob, node, "reg", "reg-names", "pci_phy",
				     &pci_phy);
		writel(0x1, pci_phy.start + 800);
		writel(0x0, pci_phy.start + 804);
		gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
		if (gpio_node >= 0)
			qca_gpio_deinit(gpio_node);

	}
	pcie_clock_deinit(0);

	return ;
}

void board_usb_deinit(int id)
{
	void __iomem *boot_clk_ctl, *usb_bcr, *qusb2_phy_bcr;
	void __iomem *usb_phy_bcr, *usb_gen_cfg, *usb_guctl, *phy_base;

	if (id == 0) {
		boot_clk_ctl = GCC_USB_0_BOOT_CLOCK_CTL;
		usb_bcr = GCC_USB0_BCR;
		qusb2_phy_bcr = GCC_QUSB2_0_PHY_BCR;
		usb_phy_bcr = GCC_USB0_PHY_BCR;
		usb_gen_cfg = USB30_1_GENERAL_CFG;
		usb_guctl = USB30_1_GUCTL;
		phy_base = USB30_PHY_1_QUSB2PHY_BASE;
	}
	else if (id == 1) {
		boot_clk_ctl = GCC_USB_1_BOOT_CLOCK_CTL;
		usb_bcr = GCC_USB1_BCR;
		qusb2_phy_bcr = GCC_QUSB2_1_PHY_BCR;
		usb_phy_bcr = GCC_USB1_PHY_BCR;
		usb_gen_cfg = USB30_2_GENERAL_CFG;
		usb_guctl = USB30_2_GUCTL;
		phy_base = USB30_PHY_2_QUSB2PHY_BASE;
	}
	else {
		return;
	}
	//Enable USB2 PHY Power down
	setbits_le32(phy_base+0xB4, 0x1);

	if (id == 0) {
		writel(0x8000, GCC_USB0_PHY_CFG_AHB_CBCR);
		writel(0xcff0, GCC_USB0_MASTER_CBCR);
		writel(0, GCC_SYS_NOC_USB0_AXI_CBCR);
		writel(0, GCC_SNOC_BUS_TIMEOUT2_AHB_CBCR);
		writel(0, GCC_USB0_SLEEP_CBCR);
		writel(0, GCC_USB0_MOCK_UTMI_CBCR);
		writel(0, GCC_USB0_AUX_CBCR);
	}
	else if (id == 1) {
		writel(0x8000, GCC_USB1_PHY_CFG_AHB_CBCR);
		writel(0xcff0, GCC_USB1_MASTER_CBCR);
		writel(0, GCC_SYS_NOC_USB1_AXI_CBCR);
		writel(0, GCC_SNOC_BUS_TIMEOUT3_AHB_CBCR);
		writel(0, GCC_USB1_SLEEP_CBCR);
		writel(0, GCC_USB1_MOCK_UTMI_CBCR);
		writel(0, GCC_USB1_AUX_CBCR);
	}

	//GCC_QUSB2_0_PHY_BCR
	setbits_le32(qusb2_phy_bcr, 0x1);
	mdelay(10);
	clrbits_le32(qusb2_phy_bcr, 0x1);

	//GCC_USB0_PHY_BCR
	setbits_le32(usb_phy_bcr, 0x1);
	mdelay(10);
	clrbits_le32(usb_phy_bcr, 0x1);

	//GCC Reset USB0 BCR
	setbits_le32(usb_bcr, 0x1);
	mdelay(10);
	clrbits_le32(usb_bcr, 0x1);
}

static void usb_clock_init(int id)
{
	if (id == 0) {
		writel(0x222000, GCC_USB0_GDSCR);
		writel(0, GCC_SYS_NOC_USB0_AXI_CBCR);
		writel(0, GCC_SNOC_BUS_TIMEOUT2_AHB_CBCR);
		writel(0x10b, GCC_USB0_MASTER_CFG_RCGR);
		writel(0x1, GCC_USB0_MASTER_CMD_RCGR);
		writel(1, GCC_SYS_NOC_USB0_AXI_CBCR);
		writel(0xcff1, GCC_USB0_MASTER_CBCR);
		writel(1, GCC_SNOC_BUS_TIMEOUT2_AHB_CBCR);
		writel(1, GCC_USB0_SLEEP_CBCR);
		writel(0x210b, GCC_USB0_MOCK_UTMI_CFG_RCGR);
		writel(0x1, GCC_USB0_MOCK_UTMI_M);
		writel(0xf7, GCC_USB0_MOCK_UTMI_N);
		writel(0xf6, GCC_USB0_MOCK_UTMI_D);
		writel(0x3, GCC_USB0_MOCK_UTMI_CMD_RCGR);
		writel(1, GCC_USB0_MOCK_UTMI_CBCR);
		writel(0x8001, GCC_USB0_PHY_CFG_AHB_CBCR);
		writel(1, GCC_USB0_AUX_CBCR);
	}
	else if (id == 1) {
		writel(0x222000, GCC_USB1_GDSCR);
		writel(0, GCC_SYS_NOC_USB1_AXI_CBCR);
		writel(0, GCC_SNOC_BUS_TIMEOUT3_AHB_CBCR);
		writel(0x10b, GCC_USB1_MASTER_CFG_RCGR);
		writel(0x1, GCC_USB1_MASTER_CMD_RCGR);
		writel(1, GCC_SYS_NOC_USB1_AXI_CBCR);
		writel(0xcff1, GCC_USB1_MASTER_CBCR);
		writel(1, GCC_SNOC_BUS_TIMEOUT3_AHB_CBCR);
		writel(1, GCC_USB1_SLEEP_CBCR);
		writel(0x210b, GCC_USB1_MOCK_UTMI_CFG_RCGR);
		writel(0x1, GCC_USB1_MOCK_UTMI_M);
		writel(0xf7, GCC_USB1_MOCK_UTMI_N);
		writel(0xf6, GCC_USB1_MOCK_UTMI_D);
		writel(0x3, GCC_USB1_MOCK_UTMI_CMD_RCGR);
		writel(1, GCC_USB1_MOCK_UTMI_CBCR);
		writel(0x8001, GCC_USB1_PHY_CFG_AHB_CBCR);
		writel(1, GCC_USB1_AUX_CBCR);
	}
}

static void usb_init_phy(int index)
{
	void __iomem *boot_clk_ctl, *usb_bcr, *qusb2_phy_bcr;
	void __iomem *usb_phy_bcr, *usb_gen_cfg, *usb_guctl, *phy_base;

	if (index == 0) {
		boot_clk_ctl = GCC_USB_0_BOOT_CLOCK_CTL;
		usb_bcr = GCC_USB0_BCR;
		qusb2_phy_bcr = GCC_QUSB2_0_PHY_BCR;
		usb_phy_bcr = GCC_USB0_PHY_BCR;
		usb_gen_cfg = USB30_1_GENERAL_CFG;
		usb_guctl = USB30_1_GUCTL;
		phy_base = USB30_PHY_1_QUSB2PHY_BASE;
	}
	else if (index == 1) {
		boot_clk_ctl = GCC_USB_1_BOOT_CLOCK_CTL;
		usb_bcr = GCC_USB1_BCR;
		qusb2_phy_bcr = GCC_QUSB2_1_PHY_BCR;
		usb_phy_bcr = GCC_USB1_PHY_BCR;
		usb_gen_cfg = USB30_2_GENERAL_CFG;
		usb_guctl = USB30_2_GUCTL;
		phy_base = USB30_PHY_2_QUSB2PHY_BASE;
	}
	else {
		return;
	}

	//2. Enable SS Ref Clock
	setbits_le32(GCC_USB_SS_REF_CLK_EN, 0x1);

	//3. Disable USB Boot Clock
	clrbits_le32(boot_clk_ctl, 0x0);

	//4. GCC Reset USB0 BCR
	setbits_le32(usb_bcr, 0x1);

	//5. Delay 100us
	mdelay(10);

	//6. GCC Reset USB0 BCR
	clrbits_le32(usb_bcr, 0x1);

	//7. GCC_QUSB2_0_PHY_BCR
	setbits_le32(qusb2_phy_bcr, 0x1);

	//8. GCC_USB0_PHY_BCR
	setbits_le32(usb_phy_bcr, 0x1);

	//9. Delay 100us
	mdelay(10);

	//10. GCC_USB0_PHY_BCR
	clrbits_le32(usb_phy_bcr, 0x1);

	//11. GCC_QUSB2_0_PHY_BCR
	clrbits_le32(qusb2_phy_bcr, 0x1);

	//12. Delay 100us
	mdelay(10);

	//13. Enable PIPE_UTMI_CLK_DIS
	setbits_le32(usb_gen_cfg, 0x100);

	//14. Delay 100us
	mdelay(10);

	//15. Enable PIPE_UTMI_CLK_SEL
	setbits_le32(usb_gen_cfg, 0x1);

	//16. Delay 100us
	mdelay(10);

	//17. Enable PIPE3_PHYSTATUS_SW
	setbits_le32(usb_gen_cfg, 0x8);

	//18. Delay 100us
	mdelay(10);

	//19. Disable PIPE_UTMI_CLK_DIS
	clrbits_le32(usb_gen_cfg, 0x100);

	//20. Config user control register
	writel(0x0c80c010, usb_guctl);

	//21. Enable USB2 PHY Power down
	setbits_le32(phy_base+0xB4, 0x1);

	//22. PHY Config Sequence
	out_8(phy_base+0x80, 0xF8);
	out_8(phy_base+0x84, 0x83);
	out_8(phy_base+0x88, 0x83);
	out_8(phy_base+0x8C, 0xC0);
	out_8(phy_base+0x9C, 0x14);
	out_8(phy_base+0x08, 0x30);
	out_8(phy_base+0x0C, 0x79);
	out_8(phy_base+0x10, 0x21);
	out_8(phy_base+0x90, 0x00);
	out_8(phy_base+0x18, 0x00);
	out_8(phy_base+0x1C, 0x9F);
	out_8(phy_base+0x04, 0x80);

	//23. Disable USB2 PHY Power down
	clrbits_le32(phy_base+0xB4, 0x1);
}

int ipq_board_usb_init(void)
{
	int i;

	for (i=0; i<CONFIG_USB_MAX_CONTROLLER_COUNT; i++) {
		usb_clock_init(i);
		usb_init_phy(i);
	}
	return 0;
}

int ipq_fdt_fixup_socinfo(void *blob)
{
	return 0;
}

void ipq_fdt_fixup_usb_device_mode(void *blob)
{
	int nodeoff, ret, node;
	const char *usb_dr_mode = "peripheral"; /* Supported mode */
	const char *usb_node[] = {"/soc/usb3@8A00000/dwc3@8A00000"};
	const char *usb_cfg;

	usb_cfg = getenv("usb_mode");
	if (!usb_cfg)
		return;

	if (strcmp(usb_cfg, usb_dr_mode)) {
		printf("fixup_usb: usb_mode can be either 'peripheral' or not set\n");
		return;
	}

	for (node = 0; node < ARRAY_SIZE(usb_node); node++) {
		nodeoff = fdt_path_offset(blob, usb_node[node]);
		if (nodeoff < 0) {
			printf("fixup_usb: unable to find node '%s'\n",
			       usb_node[node]);
			return;
		}
		ret = fdt_setprop(blob, nodeoff, "dr_mode",
				  usb_dr_mode,
				  (strlen(usb_dr_mode) + 1));
		if (ret)
			printf("fixup_usb: 'dr_mode' cannot be set");
	}
}

void fdt_fixup_auto_restart(void *blob)
{
	int nodeoff, ret;
	const char *node = "/soc/q6v5_wcss@CD00000";
	const char *paniconwcssfatal;

	paniconwcssfatal = getenv("paniconwcssfatal");

	if (!paniconwcssfatal)
		return;

	if (strncmp(paniconwcssfatal, "1", sizeof("1"))) {
		printf("fixup_auto_restart: invalid variable 'paniconwcssfatal'");
	} else {
		nodeoff = fdt_path_offset(blob, node);
		if (nodeoff < 0) {
			printf("fixup_auto_restart: unable to find node '%s'\n", node);
			return;
		}
		ret = fdt_delprop(blob, nodeoff, "qca,auto-restart");

		if (ret)
			printf("fixup_auto_restart: cannot delete property");
	}
	return;
}

void set_flash_secondary_type(qca_smem_flash_info_t *smem)
{
	return;
};
