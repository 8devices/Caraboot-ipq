/*
 * Copyright (c) 2016-2021 The Linux Foundation. All rights reserved.
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
#include <ipq9574.h>
#include <mmc.h>
#include <sdhci.h>
#include <usb.h>

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
#ifndef CONFIG_IPQ9574_RUMI
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
#ifndef CONFIG_IPQ9574_RUMI
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
#ifndef CONFIG_IPQ9574_RUMI
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
#ifndef CONFIG_IPQ9574_RUMI
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

#ifdef CONFIG_USB_XHCI_IPQ
void board_usb_deinit(int id)
{
	int nodeoff;
	char node_name[8];

	snprintf(node_name, sizeof(node_name), "usb%d", id);
	nodeoff = fdt_path_offset(gd->fdt_blob, node_name);
	if (fdtdec_get_int(gd->fdt_blob, nodeoff, "qcom,emulation", 0))
		return;

	/* Enable USB PHY Power down */
	setbits_le32(USB30_PHY_1_QUSB2PHY_BASE + 0xB4, 0x1);
	/* Disable clocks */
	writel(0x8000, GCC_USB0_PHY_CFG_AHB_CBCR);
	writel(0xcff0, GCC_USB0_MASTER_CBCR);
	writel(0, GCC_USB0_SLEEP_CBCR);
	writel(0, GCC_USB0_MOCK_UTMI_CBCR);
	writel(0, GCC_USB0_AUX_CBCR);
	/* GCC_QUSB2_0_PHY_BCR */
	set_mdelay_clearbits_le32(GCC_QUSB2_0_PHY_BCR, 0x1, 10);
	/* GCC_USB0_PHY_BCR */
	set_mdelay_clearbits_le32(GCC_USB0_PHY_BCR, 0x1, 10);
	/* GCC Reset USB BCR */
	set_mdelay_clearbits_le32(GCC_USB_BCR, 0x1, 10);
}

static void usb_clock_init(int id)
{
	int cfg;
	/* Configure usb0_master_clk_src */
	cfg = (GCC_USB0_MASTER_CFG_RCGR_SRC_SEL |
		GCC_USB0_MASTER_CFG_RCGR_SRC_DIV);
	writel(cfg, GCC_USB0_MASTER_CFG_RCGR);
	writel(CMD_UPDATE, GCC_USB0_MASTER_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_MASTER_CMD_RCGR);

	/* Configure usb0_mock_utmi_clk_src */
	cfg = (GCC_USB_MOCK_UTMI_SRC_SEL |
		GCC_USB_MOCK_UTMI_SRC_DIV);
	writel(cfg, GCC_USB0_MOCK_UTMI_CFG_RCGR);
	writel(UTMI_M, GCC_USB0_MOCK_UTMI_M);
	writel(UTMI_N, GCC_USB0_MOCK_UTMI_N);
	writel(UTMI_D, GCC_USB0_MOCK_UTMI_D);
	writel(CMD_UPDATE, GCC_USB0_MOCK_UTMI_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_MOCK_UTMI_CMD_RCGR);

	/* Configure usb0_aux_clk_src */
	cfg = (GCC_USB0_AUX_CFG_SRC_SEL |
		GCC_USB0_AUX_CFG_SRC_DIV);
	writel(cfg, GCC_USB0_AUX_CFG_RCGR);
	writel(AUX_M, GCC_USB0_AUX_M);
	writel(AUX_N, GCC_USB0_AUX_N);
	writel(AUX_D, GCC_USB0_AUX_D);
	writel(CMD_UPDATE, GCC_USB0_AUX_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_AUX_CMD_RCGR);

	/* Configure CBCRs */
	writel((readl(GCC_USB0_MASTER_CBCR) | CLK_ENABLE),
					GCC_USB0_MASTER_CBCR);
	writel(CLK_ENABLE, GCC_USB0_SLEEP_CBCR);
	writel(CLK_ENABLE, GCC_USB0_MOCK_UTMI_CBCR);
	writel((CLK_ENABLE | NOC_HANDSHAKE_FSM_EN),
					GCC_USB0_PHY_CFG_AHB_CBCR);
	writel(CLK_ENABLE, GCC_USB0_AUX_CBCR);
	writel(CLK_ENABLE, GCC_USB0_PIPE_CBCR);
}

static void usb_init_hsphy(void __iomem *phybase)
{
	/* Enable QUSB2PHY Power down */
	setbits_le32(phybase+0xB4, 0x1);

	/* PHY Config Sequence */
	/* QUSB2PHY_PLL:PLL Feedback Divider Value */
	out_8(phybase+0x00, 0x14);
	/* QUSB2PHY_PORT_TUNE1: USB Product Application Tuning Register A */
	out_8(phybase+0x80, 0xF8);
	/* QUSB2PHY_PORT_TUNE2: USB Product Application Tuning Register B */
	out_8(phybase+0x84, 0xB3);
	/* QUSB2PHY_PORT_TUNE3: USB Product Application Tuning Register C */
	out_8(phybase+0x88, 0x83);
	/* QUSB2PHY_PORT_TUNE4: USB Product Application Tuning Register D */
	out_8(phybase+0x8C, 0xC0);
	/* QUSB2PHY_PORT_TEST2 */
	out_8(phybase+0x9C, 0x14);
	/* QUSB2PHY_PLL_TUNE: PLL Test Configuration */
	out_8(phybase+0x08, 0x30);
	/* QUSB2PHY_PLL_USER_CTL1: PLL Control Configuration */
	out_8(phybase+0x0C, 0x79);
	/* QUSB2PHY_PLL_USER_CTL2: PLL Control Configuration */
	out_8(phybase+0x10, 0x21);
	/* QUSB2PHY_PORT_TUNE5 */
	out_8(phybase+0x90, 0x00);
	/* QUSB2PHY_PLL_PWR_CTL: PLL Manual SW Programming
	 * and Biasing Power Options */
	out_8(phybase+0x18, 0x00);
	/* QUSB2PHY_PLL_AUTOPGM_CTL1: Auto vs. Manual PLL/Power-mode
	 * programming State Machine Control Options */
	out_8(phybase+0x1C, 0x9F);
	/* QUSB2PHY_PLL_TEST: PLL Test Configuration-Disable diff ended clock */
	out_8(phybase+0x04, 0x80);

	/* Disable QUSB2PHY Power down */
	clrbits_le32(phybase+0xB4, 0x1);
}

static void usb_init_ssphy(void __iomem *phybase)
{
	out_8(phybase + USB3_PHY_POWER_DOWN_CONTROL,0x1);
	out_8(phybase + QSERDES_COM_SYSCLK_EN_SEL,0x1a);
	out_8(phybase + QSERDES_COM_BIAS_EN_CLKBUFLR_EN,0x08);
	out_8(phybase + QSERDES_COM_CLK_SELECT,0x30);
	out_8(phybase + QSERDES_COM_BG_TRIM,0x0f);
	out_8(phybase + QSERDES_RX_UCDR_FASTLOCK_FO_GAIN,0x0b);
	out_8(phybase + QSERDES_COM_SVS_MODE_CLK_SEL,0x01);
	out_8(phybase + QSERDES_COM_HSCLK_SEL,0x00);
	out_8(phybase + QSERDES_COM_CMN_CONFIG,0x06);
	out_8(phybase + QSERDES_COM_PLL_IVCO,0x0f);
	out_8(phybase + QSERDES_COM_SYS_CLK_CTRL,0x06);
	out_8(phybase + QSERDES_COM_DEC_START_MODE0,0x68);
	out_8(phybase + QSERDES_COM_DIV_FRAC_START1_MODE0,0xAB);
	out_8(phybase + QSERDES_COM_DIV_FRAC_START2_MODE0,0xAA);
	out_8(phybase + QSERDES_COM_DIV_FRAC_START3_MODE0,0x02);
	out_8(phybase + QSERDES_COM_CP_CTRL_MODE0,0x09);
	out_8(phybase + QSERDES_COM_PLL_RCTRL_MODE0,0x16);
	out_8(phybase + QSERDES_COM_PLL_CCTRL_MODE0,0x28);
	out_8(phybase + QSERDES_COM_INTEGLOOP_GAIN0_MODE0,0xA0);
	out_8(phybase + QSERDES_COM_LOCK_CMP1_MODE0,0xAA);
	out_8(phybase + QSERDES_COM_LOCK_CMP2_MODE0,0x29);
	out_8(phybase + QSERDES_COM_LOCK_CMP3_MODE0,0x00);
	out_8(phybase + QSERDES_COM_CORE_CLK_EN,0x00);
	out_8(phybase + QSERDES_COM_LOCK_CMP_CFG,0x00);
	out_8(phybase + QSERDES_COM_VCO_TUNE_MAP,0x00);
	out_8(phybase + QSERDES_COM_BG_TIMER,0x0a);
	out_8(phybase + QSERDES_COM_SSC_EN_CENTER,0x01);
	out_8(phybase + QSERDES_COM_SSC_PER1,0x7D);
	out_8(phybase + QSERDES_COM_SSC_PER2,0x01);
	out_8(phybase + QSERDES_COM_SSC_ADJ_PER1,0x00);
	out_8(phybase + QSERDES_COM_SSC_ADJ_PER2,0x00);
	out_8(phybase + QSERDES_COM_SSC_STEP_SIZE1,0x0A);
	out_8(phybase + QSERDES_COM_SSC_STEP_SIZE2,0x05);
	out_8(phybase + QSERDES_RX_UCDR_SO_GAIN,0x06);
	out_8(phybase + QSERDES_RX_RX_EQU_ADAPTOR_CNTRL2,0x02);
	out_8(phybase + QSERDES_RX_RX_EQU_ADAPTOR_CNTRL3,0x6c);
	out_8(phybase + QSERDES_RX_RX_EQU_ADAPTOR_CNTRL3,0x4c);
	out_8(phybase + QSERDES_RX_RX_EQU_ADAPTOR_CNTRL4,0xb8);
	out_8(phybase + QSERDES_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL,0x77);
	out_8(phybase + QSERDES_RX_RX_OFFSET_ADAPTOR_CNTRL2,0x80);
	out_8(phybase + QSERDES_RX_SIGDET_CNTRL,0x03);
	out_8(phybase + QSERDES_RX_SIGDET_DEGLITCH_CNTRL,0x16);
	out_8(phybase + QSERDES_RX_SIGDET_ENABLES,0x0c);
	out_8(phybase + QSERDES_TX_HIGHZ_TRANSCEIVEREN_BIAS_D,0x45);
	out_8(phybase + QSERDES_TX_RCV_DETECT_LVL_2,0x12);
	out_8(phybase + QSERDES_TX_LANE_MODE,0x06);
	out_8(phybase + PCS_TXDEEMPH_M6DB_V0,0x15);
	out_8(phybase + PCS_TXDEEMPH_M3P5DB_V0,0x0e);
	out_8(phybase + PCS_FLL_CNTRL2,0x83);
	out_8(phybase + PCS_FLL_CNTRL1,0x02);
	out_8(phybase + PCS_FLL_CNT_VAL_L,0x09);
	out_8(phybase + PCS_FLL_CNT_VAL_H_TOL,0xa2);
	out_8(phybase + PCS_FLL_MAN_CODE,0x85);
	out_8(phybase + PCS_LOCK_DETECT_CONFIG1,0xd1);
	out_8(phybase + PCS_LOCK_DETECT_CONFIG2,0x1f);
	out_8(phybase + PCS_LOCK_DETECT_CONFIG3,0x47);
	out_8(phybase + PCS_POWER_STATE_CONFIG2,0x1b);
	out_8(phybase + PCS_RXEQTRAINING_WAIT_TIME,0x75);
	out_8(phybase + PCS_RXEQTRAINING_RUN_TIME,0x13);
	out_8(phybase + PCS_LFPS_TX_ECSTART_EQTLOCK,0x86);
	out_8(phybase + PCS_PWRUP_RESET_DLY_TIME_AUXCLK,0x04);
	out_8(phybase + PCS_TSYNC_RSYNC_TIME,0x44);
	out_8(phybase + PCS_RCVR_DTCT_DLY_P1U2_L,0xe7);
	out_8(phybase + PCS_RCVR_DTCT_DLY_P1U2_H,0x03);
	out_8(phybase + PCS_RCVR_DTCT_DLY_U3_L,0x40);
	out_8(phybase + PCS_RCVR_DTCT_DLY_U3_H,0x00);
	out_8(phybase + PCS_RX_SIGDET_LVL,0x88);
	out_8(phybase + USB3_PCS_TXDEEMPH_M6DB_V0,0x17);
	out_8(phybase + USB3_PCS_TXDEEMPH_M3P5DB_V0,0x0f);
	out_8(phybase + QSERDES_RX_SIGDET_ENABLES,0x0);
	out_8(phybase + USB3_PHY_START_CONTROL,0x03);
	out_8(phybase + USB3_PHY_SW_RESET,0x00);
}

static void usb_init_phy(int index)
{
	void __iomem *boot_clk_ctl, *usb_bcr, *qusb2_phy_bcr;

	boot_clk_ctl = (u32 *)GCC_USB_0_BOOT_CLOCK_CTL;
	usb_bcr = (u32 *)GCC_USB_BCR;
	qusb2_phy_bcr = (u32 *)GCC_QUSB2_0_PHY_BCR;

	/* Disable USB Boot Clock */
	clrbits_le32(boot_clk_ctl, 0x0);

	/* GCC Reset USB BCR */
	set_mdelay_clearbits_le32(usb_bcr, 0x1, 10);

	/* GCC_QUSB2_PHY_BCR */
	setbits_le32(qusb2_phy_bcr, 0x1);

	/* GCC_USB0_PHY_BCR */
	setbits_le32(GCC_USB0_PHY_BCR, 0x1);
	setbits_le32(GCC_USB3PHY_0_PHY_BCR, 0x1);
	mdelay(10);
	clrbits_le32(GCC_USB3PHY_0_PHY_BCR, 0x1);
	clrbits_le32(GCC_USB0_PHY_BCR, 0x1);

	/* Config user control register */
	writel(0x0c80c010, USB30_1_GUCTL);
	writel(0x0a87f0a0, USB30_1_FLADJ);

	/* GCC_QUSB2_0_PHY_BCR */
	clrbits_le32(qusb2_phy_bcr, 0x1);
	mdelay(10);

	usb_init_hsphy((u32 *)USB30_PHY_1_QUSB2PHY_BASE);
	usb_init_ssphy((u32 *)USB30_PHY_1_USB3PHY_AHB2PHY_BASE);
}

int ipq_board_usb_init(void)
{
	int i, nodeoff;
	char node_name[8];

	for (i = 0; i < CONFIG_USB_MAX_CONTROLLER_COUNT; i++) {
		snprintf(node_name, sizeof(node_name), "usb%d", i);
		nodeoff = fdt_path_offset(gd->fdt_blob, node_name);
		if (!fdtdec_get_int(gd->fdt_blob, nodeoff, "qcom,emulation", 0)) {
			usb_clock_init(i);
			usb_init_phy(i);
		} else {
			/* Config user control register */
			writel(0x0c80c010, USB30_1_GUCTL);
		}
	}
	return 0;
}
#endif

static void __fixup_usb_device_mode(void *blob)
{
	parse_fdt_fixup("/soc/usb3@8A00000/dwc3@8A00000%dr_mode%?peripheral", blob);
	parse_fdt_fixup("/soc/usb3@8A00000/dwc3@8A00000%maximum-speed%?high-speed", blob);
}

static void fdt_fixup_diag_gadget(void *blob)
{
	__fixup_usb_device_mode(blob);
	parse_fdt_fixup("/soc/qcom,gadget_diag@0%status%?ok", blob);
}

void ipq_fdt_fixup_usb_device_mode(void *blob)
{
	const char *usb_cfg;

	usb_cfg = getenv("usb_mode");
	if (!usb_cfg)
		return;

	if (!strncmp(usb_cfg, "peripheral", sizeof("peripheral")))
		__fixup_usb_device_mode(blob);
	else if (!strncmp(usb_cfg, "diag_gadget", sizeof("diag_gadget")))
		fdt_fixup_diag_gadget(blob);
	else
		printf("%s: invalid param for usb_mode\n", __func__);
}

#ifdef CONFIG_PCI_IPQ
void board_pci_init(int id)
{
	int node, gpio_node;
	char name[16];

	snprintf(name, sizeof(name), "pci%d", id);
	node = fdt_path_offset(gd->fdt_blob, name);
	if (node < 0) {
		printf("Could not find PCI%d in device tree\n", id);
		return;
	}

	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
	if (gpio_node >= 0)
		qca_gpio_init(gpio_node);

	return;
}

void board_pci_deinit()
{
	int node, gpio_node, i, err;
	char name[16];
	struct fdt_resource parf;
	struct fdt_resource pci_phy;

	for (i = 0; i < PCI_MAX_DEVICES; i++) {
		snprintf(name, sizeof(name), "pci%d", i);
		node = fdt_path_offset(gd->fdt_blob, name);
		if (node < 0) {
			printf("Could not find PCI%d in device tree\n", i);
			continue;
		}
		err = fdt_get_named_resource(gd->fdt_blob, node, "reg", "reg-names", "parf",
				&parf);
		writel(0x0, parf.start + 0x358);
		writel(0x1, parf.start + 0x40);
		err = fdt_get_named_resource(gd->fdt_blob, node, "reg", "reg-names", "pci_phy",
				     &pci_phy);
		if (err < 0)
			continue;

		writel(0x1, pci_phy.start + 800);
		writel(0x0, pci_phy.start + 804);
		gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
		if (gpio_node >= 0)
			qca_gpio_deinit(gpio_node);

	}

	return ;
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

void fdt_fixup_auto_restart(void *blob)
{
	return;
}

