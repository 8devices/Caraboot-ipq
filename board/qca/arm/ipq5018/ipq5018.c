/*
* Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
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
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/uart.h>
#include <asm/arch-qca-common/scm.h>
#include <asm/arch-qca-common/iomap.h>
#include <ipq5018.h>
#ifdef CONFIG_QCA_MMC
#include <mmc.h>
#include <sdhci.h>
#endif
#ifdef CONFIG_USB_XHCI_IPQ
#include <usb.h>
#endif
#ifdef CONFIG_QPIC_NAND
#include <asm/arch-qca-common/qpic_nand.h>
#endif

#define DLOAD_MAGIC_COOKIE	0x10
#define DLOAD_DISABLED		0x40

ipq_gmac_board_cfg_t gmac_cfg[CONFIG_IPQ_NO_MACS];

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_QCA_MMC
struct sdhci_host mmc_host;
#endif

#ifdef CONFIG_IPQ_MTD_NOR
extern int ipq_spi_init(u16);
#endif

const char *rsvd_node = "/reserved-memory";
const char *del_node[] = {"uboot",
			  "sbl",
			  NULL};
const add_node_t add_fdt_node[] = {{}};

struct dumpinfo_t dumpinfo_n[] = {
	/* TZ stores the DDR physical address at which it stores the
	 * APSS regs, UTCM copy dump. We will have the TZ IMEM
	 * IMEM Addr at which the DDR physical address is stored as
	 * the start
	 *     --------------------
         *     |  DDR phy (start) | ----> ------------------------
         *     --------------------       | APSS regsave (8k)    |
         *                                ------------------------
         *                                |                      |
	 *                                | 	 UTCM copy	 |
         *                                |        (192k)        |
	 *                                |                      |
         *                                ------------------------
	 */
	{ "EBICS0.BIN", 0x40000000, 0x10000000, 0 },
	{ "IMEM.BIN", 0x08600000, 0x00001000, 0 },
	{ "NSSUTCM.BIN", 0x08600658, 0x00030000, 0, 1, 0x2000 },
	{ "UNAME.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "CPU_INFO.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "DMESG.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "PT.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "WLAN_MOD.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
};
int dump_entries_n = ARRAY_SIZE(dumpinfo_n);

struct dumpinfo_t dumpinfo_s[] = {
	{ "EBICS_S0.BIN", 0x40000000, 0xA600000, 0 },
	{ "EBICS_S1.BIN", CONFIG_TZ_END_ADDR, 0x10000000, 0 },
	{ "IMEM.BIN", 0x08600000, 0x00001000, 0 },
	{ "NSSUTCM.BIN", 0x08600658, 0x00030000, 0, 1, 0x2000 },
	{ "UNAME.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "CPU_INFO.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "DMESG.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "PT.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "WLAN_MOD.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
};
int dump_entries_s = ARRAY_SIZE(dumpinfo_s);
u32 *tz_wonce = (u32 *)CONFIG_IPQ5018_TZ_WONCE_4_ADDR;

void uart1_configure_mux(void)
{
	unsigned long cfg_rcgr;

	cfg_rcgr = readl(GCC_BLSP1_UART1_APPS_CFG_RCGR);
	/* Clear mode, src sel, src div */
	cfg_rcgr &= ~(GCC_UART_CFG_RCGR_MODE_MASK |
			GCC_UART_CFG_RCGR_SRCSEL_MASK |
			GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART1_RCGR_SRC_SEL << GCC_UART_CFG_RCGR_SRCSEL_SHIFT)
			& GCC_UART_CFG_RCGR_SRCSEL_MASK);

	cfg_rcgr |= ((UART1_RCGR_SRC_DIV << GCC_UART_CFG_RCGR_SRCDIV_SHIFT)
			& GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART1_RCGR_MODE << GCC_UART_CFG_RCGR_MODE_SHIFT)
			& GCC_UART_CFG_RCGR_MODE_MASK);

	writel(cfg_rcgr, GCC_BLSP1_UART1_APPS_CFG_RCGR);
}

void uart1_set_rate_mnd(unsigned int m,
		unsigned int n, unsigned int two_d)
{
	writel(m, GCC_BLSP1_UART1_APPS_M);
	writel(NOT_N_MINUS_M(n, m), GCC_BLSP1_UART1_APPS_N);
	writel(NOT_2D(two_d), GCC_BLSP1_UART1_APPS_D);
}

int uart1_trigger_update(void)
{
	unsigned long cmd_rcgr;
	int timeout = 0;

	cmd_rcgr = readl(GCC_BLSP1_UART1_APPS_CMD_RCGR);
	cmd_rcgr |= UART1_CMD_RCGR_UPDATE;
	writel(cmd_rcgr, GCC_BLSP1_UART1_APPS_CMD_RCGR);

	while (readl(GCC_BLSP1_UART1_APPS_CMD_RCGR) & UART1_CMD_RCGR_UPDATE) {
		if (timeout++ >= CLOCK_UPDATE_TIMEOUT_US) {
			printf("Timeout waiting for UART1 clock update\n");
			return -ETIMEDOUT;
			udelay(1);
		}
	}
	cmd_rcgr = readl(GCC_BLSP1_UART1_APPS_CMD_RCGR);
	return 0;
}

void reset_board(void)
{
	run_command("reset", 0);
}

void uart1_toggle_clock(void)
{
	unsigned long cbcr_val;

	cbcr_val = readl(GCC_BLSP1_UART1_APPS_CBCR);
	cbcr_val |= UART1_CBCR_CLK_ENABLE;
	writel(cbcr_val, GCC_BLSP1_UART1_APPS_CBCR);
}

void uart1_clock_config(unsigned int m,
		unsigned int n, unsigned int two_d)
{
	uart1_configure_mux();
	uart1_set_rate_mnd(m, n, two_d);
	uart1_trigger_update();
	uart1_toggle_clock();
}

void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int node, uart1_node;

	writel(1, GCC_BLSP1_UART1_APPS_CBCR);
	node = fdt_path_offset(gd->fdt_blob, "/serial@78AF000/serial_gpio");
	if (node < 0) {
		printf("Could not find serial_gpio node\n");
		return;
	}

	if (plat->port_id == 1) {
		uart1_node = fdt_path_offset(gd->fdt_blob, "uart1");
		if (uart1_node < 0) {
			printf("Could not find uart1 node\n");
			return;
		}
	node = fdt_subnode_offset(gd->fdt_blob,
				uart1_node, "serial_gpio");
	uart1_clock_config(plat->m_value, plat->n_value, plat->d_value);
	writel(1, GCC_BLSP1_UART1_APPS_CBCR);
	}

	qca_gpio_init(node);
}

/*
 * Set the uuid in bootargs variable for mounting rootfilesystem
 */
#ifdef CONFIG_QCA_MMC
int set_uuid_bootargs(char *boot_args, char *part_name, int buflen, bool gpt_flag)
{
	int ret, len;
	block_dev_desc_t *blk_dev;
	disk_partition_t disk_info;

	blk_dev = mmc_get_dev(mmc_host.dev_num);
	if (!blk_dev) {
		printf("Invalid block device name\n");
		return -EINVAL;
	}

	if (buflen <= 0 || buflen > MAX_BOOT_ARGS_SIZE)
		return -EINVAL;

#ifdef CONFIG_PARTITION_UUIDS
	ret = get_partition_info_efi_by_name(blk_dev,
			part_name, &disk_info);
	if (ret) {
		printf("bootipq: unsupported partition name %s\n",part_name);
		return -EINVAL;
	}
	if ((len = strlcpy(boot_args, "root=PARTUUID=", buflen)) >= buflen)
		return -EINVAL;
#else
	if ((len = strlcpy(boot_args, "rootfsname=", buflen)) >= buflen)
		return -EINVAL;
#endif
	boot_args += len;
	buflen -= len;

#ifdef CONFIG_PARTITION_UUIDS
	if ((len = strlcpy(boot_args, disk_info.uuid, buflen)) >= buflen)
		return -EINVAL;
#else
	if ((len = strlcpy(boot_args, part_name, buflen)) >= buflen)
		return -EINVAL;
#endif
	boot_args += len;
	buflen -= len;

	if (gpt_flag && strlcpy(boot_args, " gpt", buflen) >= buflen)
		return -EINVAL;

	return 0;
}
#else
int set_uuid_bootargs(char *boot_args, char *part_name, int buflen, bool gpt_flag)
{
	return 0;
}
#endif

#ifdef CONFIG_QCA_MMC
void emmc_clock_config(void)
{
	/* Enable root clock generator */
	writel(readl(GCC_SDCC1_APPS_CBCR)|0x1, GCC_SDCC1_APPS_CBCR);
	/* Add 10us delay for CLK_OFF to get cleared */
	udelay(10);
	writel(readl(GCC_SDCC1_AHB_CBCR)|0x1, GCC_SDCC1_AHB_CBCR);
	/* PLL0 - 192Mhz */
	writel(0x20B, GCC_SDCC1_APPS_CFG_RCGR);
	/* Delay for clock operation complete */
	udelay(10);
	writel(0x1, GCC_SDCC1_APPS_M);
	writel(0xFC, GCC_SDCC1_APPS_N);
	writel(0xFD, GCC_SDCC1_APPS_D);
	/* Delay for clock operation complete */
	udelay(10);
	/* Update APPS_CMD_RCGR to reflect source selection */
	writel(readl(GCC_SDCC1_APPS_CMD_RCGR)|0x1, GCC_SDCC1_APPS_CMD_RCGR);
	/* Add 10us delay for clock update to complete */
	udelay(10);
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
	/* Clear divider */
	writel(0x0, GCC_SDCC1_MISC);
}

void board_mmc_deinit(void)
{
	emmc_clock_disable();
}

void emmc_clock_reset(void)
{
	writel(0x1, GCC_SDCC1_BCR);
	udelay(10);
	writel(0x0, GCC_SDCC1_BCR);
}

int board_mmc_init(bd_t *bis)
{
	int node, gpio_node;
	int ret = 0;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	node = fdt_path_offset(gd->fdt_blob, "mmc");
	if (node < 0) {
		printf("sdhci: Node Not found, skipping initialization\n");
		return -1;
	}

	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "mmc_gpio");
	qca_gpio_init(gpio_node);

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

__weak int ipq_get_tz_version(char *version_name, int buf_size)
{
	return 1;
}

int apps_iscrashed_crashdump_disabled(void)
{
	u32 *dmagic = (u32 *)CONFIG_IPQ5018_DMAGIC_ADDR;

	if (*dmagic == DLOAD_DISABLED)
		return 1;

	return 0;
}

int apps_iscrashed(void)
{
	u32 *dmagic = (u32 *)CONFIG_IPQ5018_DMAGIC_ADDR;

	if (*dmagic == DLOAD_MAGIC_COOKIE)
		return 1;

	return 0;
}

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

void fdt_fixup_set_dload_dis(void *blob)
{
	parse_fdt_fixup("/soc/qca,scm_restart_reason%dload_status%1", blob);
}

void ipq_fdt_fixup_socinfo(void *blob)
{
	uint32_t cpu_type;
	uint32_t soc_version, soc_version_major, soc_version_minor;
	int nodeoff, ret;

	nodeoff = fdt_path_offset(blob, "/");

	if (nodeoff < 0) {
		printf("ipq: fdt fixup cannot find root node\n");
		return;
	}

	ret = ipq_smem_get_socinfo_cpu_type(&cpu_type);
	if (!ret) {
		ret = fdt_setprop(blob, nodeoff, "cpu_type",
				  &cpu_type, sizeof(cpu_type));
		if (ret)
			printf("%s: cannot set cpu type %d\n", __func__, ret);
	} else {
		printf("%s: cannot get socinfo\n", __func__);
	}

	ret = ipq_smem_get_socinfo_version((uint32_t *)&soc_version);
	if (!ret) {
		soc_version_major = SOCINFO_VERSION_MAJOR(soc_version);
		soc_version_minor = SOCINFO_VERSION_MINOR(soc_version);

		ret = fdt_setprop(blob, nodeoff, "soc_version_major",
				  &soc_version_major,
				  sizeof(soc_version_major));
		if (ret)
			printf("%s: cannot set soc_version_major %d\n",
			       __func__, soc_version_major);

		ret = fdt_setprop(blob, nodeoff, "soc_version_minor",
				  &soc_version_minor,
				  sizeof(soc_version_minor));
		if (ret)
			printf("%s: cannot set soc_version_minor %d\n",
			       __func__, soc_version_minor);
	} else {
		printf("%s: cannot get soc version\n", __func__);
	}
	return;
}

void fdt_fixup_auto_restart(void *blob)
{
	const char *paniconwcssfatal;

	paniconwcssfatal = getenv("paniconwcssfatal");

	if (!paniconwcssfatal)
		return;

	if (strncmp(paniconwcssfatal, "1", sizeof("1"))) {
		printf("fixup_auto_restart: invalid variable 'paniconwcssfatal'");
	} else {
		parse_fdt_fixup("/soc/q6v5_wcss@CD00000%delete%?qca,auto-restart", blob);
	}
	return;
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
	__invoke_psci_fn_smc(0x84000009, 0, 0, 0);
}

void qti_scm_pshold(void)
{
	int ret;

	ret = scm_call(SCM_SVC_BOOT, SCM_CMD_TZ_PSHOLD, NULL, 0, NULL, 0);

	if (ret != 0)
		writel(0, GCNT_PSHOLD);
}

void reset_cpu(unsigned long a)
{
	reset_crashdump();
	if (is_scm_armv8()) {
		psci_sys_reset();
	} else {
		qti_scm_pshold();
	}
	while(1);
}

void qpic_clk_enbale(void)
{
#ifdef QPIC_CLOCK_ENABLE
/*
 || clock enable code has been disabled due to NOC error in emualtion
 || will verify on RDB and remove this clock configuration
*/
	writel(QPIC_CBCR_VAL, GCC_QPIC_CBCR_ADDR);
	writel(0x1, GCC_QPIC_AHB_CBCR_ADDR);
	writel(0x1, GCC_QPIC_IO_MACRO_CBCR);
	writel(0x1, GCC_QPIC_CBCR_ADDR);
#endif
}

void board_nand_init(void)
{
#ifdef CONFIG_QPIC_SERIAL
	qpic_nand_init(NULL);
#endif

#ifdef CONFIG_QCA_SPI
	int gpio_node;
	gpio_node = fdt_path_offset(gd->fdt_blob, "/spi/spi_gpio");
	if (gpio_node >= 0) {
		qca_gpio_init(gpio_node);
#ifdef CONFIG_IPQ_MTD_NOR
		ipq_spi_init(CONFIG_IPQ_SPI_NOR_INFO_IDX);
#endif
	}
#endif
}

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

unsigned long timer_read_counter(void)
{
	return 0;
}

int board_eth_init(bd_t *bis)
{
	int status;
	int gmac_gpio_node = 0;
	int gmac_cfg_node = 0, offset = 0;
	int loop = 0;
	int phy_name_len = 0;
	char *phy_name_ptr = NULL;

	gmac_cfg_node = fdt_path_offset(gd->fdt_blob, "/gmac_cfg");
	if (gmac_cfg_node >= 0) {
		for (offset = fdt_first_subnode(gd->fdt_blob, gmac_cfg_node);
			offset > 0;
			offset = fdt_next_subnode(gd->fdt_blob, offset) , loop++) {

			gmac_cfg[loop].base = fdtdec_get_uint(gd->fdt_blob,
					offset, "base", 0);

			gmac_cfg[loop].unit = fdtdec_get_uint(gd->fdt_blob,
					offset, "unit", 0);

			gmac_cfg[loop].phy_addr = fdtdec_get_uint(gd->fdt_blob,
					offset, "phy_address", 0);

			phy_name_ptr = (char*)fdt_getprop(gd->fdt_blob, offset,
					"phy_name", &phy_name_len);

			strlcpy((char *)gmac_cfg[loop].phy_name, phy_name_ptr, phy_name_len);
                }
        }
	gmac_cfg[loop].unit = -1;

	ipq_gmac_common_init(gmac_cfg);

	gmac_gpio_node = fdt_path_offset(gd->fdt_blob, "gmac_gpio");
	if (gmac_gpio_node) {
		qca_gpio_init(gmac_gpio_node);
	}
	status = ipq_gmac_init(gmac_cfg);

	return status;
}

void set_flash_secondary_type(qca_smem_flash_info_t *smem)
{
	return;
};

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
	setbits_le32(QUSB2PHY_BASE + 0xA4, 0x0);
	/* Disable clocks */
	writel(0x0, GCC_USB0_PHY_CFG_AHB_CBCR);
	writel(0x4220, GCC_USB0_MASTER_CBCR);
	writel(0x0, GCC_SYS_NOC_USB0_AXI_CBCR);
	writel(0x0, GCC_USB0_SLEEP_CBCR);
	writel(0x0, GCC_USB0_MOCK_UTMI_CBCR);
	writel(0x0, GCC_USB0_AUX_CBCR);
	writel(0x0, GCC_USB0_LFPS_CBCR);
	/* GCC_QUSB2_0_PHY_BCR */
	set_mdelay_clearbits_le32(GCC_QUSB2_0_PHY_BCR, 0x1, 10);
	/* GCC_USB0_PHY_BCR */
	set_mdelay_clearbits_le32(GCC_USB0_PHY_BCR, 0x1, 10);
	/* GCC Reset USB BCR */
	set_mdelay_clearbits_le32(GCC_USB0_BCR, 0x1, 10);
	/* Deselect the usb phy mux */
	writel(0x0, TCSR_USB_PCIE_SEL);
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
	writel(GCC_USB_MOCK_UTMI_CLK_DIV, GCC_USB0_MOCK_UTMI_CBCR);
	writel(CMD_UPDATE, GCC_USB0_MOCK_UTMI_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_MOCK_UTMI_CMD_RCGR);

	/* Configure usb0_lfps_cmd_rcgr */
	cfg = (GCC_USB0_LFPS_CFG_SRC_SEL |
		GCC_USB0_LFPS_CFG_SRC_DIV);
	writel(cfg, GCC_USB0_LFPS_CFG_RCGR);
	writel(LFPS_M, GCC_USB0_LFPS_M);
	writel(LFPS_N, GCC_USB0_LFPS_N);
	writel(LFPS_D, GCC_USB0_LFPS_D);
	writel(CMD_UPDATE, GCC_USB0_LFPS_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_LFPS_CMD_RCGR);

	/* Configure usb0_aux_clk_src */
	cfg = (GCC_USB0_AUX_CFG_SRC_SEL |
		GCC_USB0_AUX_CFG_SRC_DIV);
	writel(cfg, GCC_USB0_AUX_CFG_RCGR);
	writel(CMD_UPDATE, GCC_USB0_AUX_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_AUX_CMD_RCGR);

	/* Configure CBCRs */
	writel(CLK_DISABLE, GCC_SYS_NOC_USB0_AXI_CBCR);
	writel(CLK_ENABLE, GCC_SYS_NOC_USB0_AXI_CBCR);
	writel((readl(GCC_USB0_MASTER_CBCR) | CLK_ENABLE),
		GCC_USB0_MASTER_CBCR);
	writel(CLK_ENABLE, GCC_USB0_SLEEP_CBCR);
	writel(CLK_ENABLE, GCC_USB0_MOCK_UTMI_CBCR);
	writel(CLK_ENABLE, GCC_USB0_PHY_CFG_AHB_CBCR);
	writel(CLK_ENABLE, GCC_USB0_AUX_CBCR);
	writel(CLK_ENABLE, GCC_USB0_PIPE_CBCR);
	writel(CLK_ENABLE, GCC_USB0_LFPS_CBCR);
}

static void usb_init_hsphy(void __iomem *phybase)
{
	/* Disable USB PHY Power down */
	setbits_le32(phybase + 0xA4, 0x1);
	/* Enable override ctrl */
	writel(UTMI_PHY_OVERRIDE_EN, phybase + USB_PHY_CFG0);
	/* Enable POR*/
	writel(POR_EN, phybase + USB_PHY_UTMI_CTRL5);
	udelay(15);
	/* Configure frequency select value*/
	writel(FREQ_SEL, phybase + USB_PHY_FSEL_SEL);
	/* Configure refclk frequency */
	writel(COMMONONN | FSEL | RETENABLEN,
			phybase + USB_PHY_HS_PHY_CTRL_COMMON0);
	/* select refclk source */
	writel(CLKCORE, phybase + USB_PHY_REFCLK_CTRL);
	/* select ATERESET*/
	writel(POR_EN & ATERESET, phybase + USB_PHY_UTMI_CTRL5);
	writel(USB2_SUSPEND_N_SEL | USB2_SUSPEND_N | USB2_UTMI_CLK_EN,
			phybase + USB_PHY_HS_PHY_CTRL2);
	writel(0x0, phybase + USB_PHY_UTMI_CTRL5);
	writel(USB2_SUSPEND_N | USB2_UTMI_CLK_EN,
			phybase + USB_PHY_HS_PHY_CTRL2);
	/* Disable override ctrl */
	writel(0x0, phybase + USB_PHY_CFG0);
}

static void usb_init_ssphy(void __iomem *phybase)
{
	/* select usb phy mux */
	writel(0x1, TCSR_USB_PCIE_SEL);
	writel(CLK_ENABLE, GCC_USB0_PHY_CFG_AHB_CBCR);
	writel(CLK_ENABLE, GCC_USB0_PIPE_CBCR);
	writel(CLK_DISABLE, GCC_USB0_PIPE_CBCR);
	udelay(100);
	/*set frequency initial value*/
	writel(0x1cb9, phybase + SSCG_CTRL_REG_4);
	writel(0x023a, phybase + SSCG_CTRL_REG_5);
	/*set spectrum spread count*/
	writel(0x1360, phybase + SSCG_CTRL_REG_3);
	/*set fstep*/
	writel(0x1, phybase + SSCG_CTRL_REG_1);
	writel(0xeb, phybase + SSCG_CTRL_REG_2);
	return;
}

static void usb_init_phy(int index)
{
	void __iomem *boot_clk_ctl, *usb_bcr, *qusb2_phy_bcr;

	boot_clk_ctl = (u32 *)GCC_USB_0_BOOT_CLOCK_CTL;
	usb_bcr = (u32 *)GCC_USB0_BCR;
	qusb2_phy_bcr = (u32 *)GCC_QUSB2_0_PHY_BCR;
	/* Disable USB Boot Clock */
	clrbits_le32(boot_clk_ctl, 0x0);

	/* GCC Reset USB BCR */
	set_mdelay_clearbits_le32(usb_bcr, 0x1, 10);

	/* GCC_QUSB2_PHY_BCR */
	setbits_le32(qusb2_phy_bcr, 0x1);

	/* GCC_USB0_PHY_BCR */
	setbits_le32(GCC_USB0_PHY_BCR, 0x1);
	mdelay(10);
	clrbits_le32(GCC_USB0_PHY_BCR, 0x1);

	/* Config user control register */
	writel(0x0C804010, USB30_GUCTL);
	writel(0x8C80C8A0, USB30_FLADJ);

	/* GCC_QUSB2_0_PHY_BCR */
	clrbits_le32(qusb2_phy_bcr, 0x1);
	mdelay(10);

	usb_init_hsphy((u32 *)QUSB2PHY_BASE);
	usb_init_ssphy((u32 *)USB3PHY_APB_BASE);
}

int ipq_board_usb_init(void)
{
	int i, nodeoff;
	char node_name[8];

	for (i=0; i<CONFIG_USB_MAX_CONTROLLER_COUNT; i++) {
		snprintf(node_name, sizeof(node_name), "usb%d", i);
		nodeoff = fdt_path_offset(gd->fdt_blob, node_name);
		if (!fdtdec_get_int(gd->fdt_blob, nodeoff, "qcom,emulation", 0)) {

			usb_clock_init(i);
			usb_init_phy(i);
		}else {
			/* Config user control register */
			writel(0x0C804010, USB30_GUCTL);
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_PCI_IPQ
static void pcie_v2_clock_init(int id)
{
#ifdef CONFIG_PCI
	int cfg;
	unsigned tmp;
	void __iomem *base;

	/*single lane*/
	if (id == 0) {
		base = (void __iomem *)GCC_PCIE1_BOOT_CLOCK_CTL;
		/* Configure pcie1_aux_clk_src */
		cfg = (GCC_PCIE1_AUX_CFG_RCGR_SRC_SEL |
				GCC_PCIE1_AUX_CFG_RCGR_SRC_DIV);
	} else { /*double lane*/
		base = (void __iomem *)GCC_PCIE0_BOOT_CLOCK_CTL;
		/* Configure pcie0_aux_clk_src */
		cfg = (GCC_PCIE0_AUX_CFG_RCGR_SRC_SEL |
				GCC_PCIE0_AUX_CFG_RCGR_SRC_DIV);
	}

	writel(cfg, base + PCIE_AUX_CFG_RCGR);
	writel(CMD_UPDATE, base + PCIE_AUX_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, base + PCIE_AUX_CMD_RCGR);

	if (id == 0)
		/* Configure pcie1_axi_clk_src */
		cfg = (GCC_PCIE1_AXI_CFG_RCGR_SRC_SEL |
				GCC_PCIE1_AXI_CFG_RCGR_SRC_DIV);

	else
		/* Configure pcie0_axi_clk_src */
		cfg = (GCC_PCIE0_AXI_CFG_RCGR_SRC_SEL |
				GCC_PCIE0_AXI_CFG_RCGR_SRC_DIV);

	writel(cfg, base + PCIE_AXI_CFG_RCGR);
	writel(CMD_UPDATE, base + PCIE_AXI_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, base + PCIE_AXI_CMD_RCGR);

	/* Configure CBCRs */
	if (id == 0)
		writel(CLK_ENABLE, GCC_SYS_NOC_PCIE1_AXI_CBCR);
	else
		writel(CLK_ENABLE, GCC_SYS_NOC_PCIE0_AXI_CBCR);

	writel(CLK_ENABLE, base + PCIE_AHB_CBCR);

	tmp = readl(base + PCIE_AXI_M_CBCR);
	tmp |= CLK_ENABLE;
	writel(tmp, base + PCIE_AXI_M_CBCR);

	tmp = readl(base + PCIE_AXI_S_CBCR);
	tmp |= CLK_ENABLE;
	writel(tmp, base + PCIE_AXI_S_CBCR);

	writel(CLK_ENABLE, base + PCIE_AUX_CBCR);

	tmp = readl(base + PCIE_PIPE_CBCR);
	tmp |= CLK_ENABLE;
	writel(tmp, base + PCIE_PIPE_CBCR);

	writel(CLK_ENABLE, PCIE_AXI_S_BRIDGE_CBCR);
#endif
	return;
}

static void pcie_v2_clock_deinit(int id)
{
#ifdef CONFIG_PCI
	void __iomem *base;

	/*single lane*/
	if (id == 0)
		base = (void __iomem *)GCC_PCIE1_BOOT_CLOCK_CTL;
	else  /*double lane*/
		base = (void __iomem *)GCC_PCIE0_BOOT_CLOCK_CTL;

	writel(0x0, base + PCIE_AUX_CFG_RCGR);
	writel(0x0, base + PCIE_AUX_CMD_RCGR);
	writel(0x0, base + PCIE_AXI_CFG_RCGR);
	writel(0x0, base + PCIE_AXI_CMD_RCGR);
	mdelay(100);

	if (id == 0)
		writel(0x0, GCC_SYS_NOC_PCIE1_AXI_CBCR);
	else
		writel(0x0, GCC_SYS_NOC_PCIE0_AXI_CBCR);

	writel(0x0, base + PCIE_AHB_CBCR);
	writel(0x0, base + PCIE_AXI_M_CBCR);
	writel(0x0, base + PCIE_AXI_S_CBCR);
	writel(0x0, base + PCIE_AUX_CBCR);
	writel(0x0, base + PCIE_PIPE_CBCR);
	writel(0x0, base + PCIE_AXI_S_BRIDGE_CBCR);
#endif
	return;
}

void board_pci_init(int id)
{
	int node, gpio_node;
	char name[16];

	snprintf(name, sizeof(name), "pci%d", id);
	node = fdt_path_offset(gd->fdt_blob, name);
	if (node < 0) {
		printf("Could not find PCI in device tree\n");
		return;
	}
	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
	if (gpio_node >= 0)
		qca_gpio_init(gpio_node);

	pcie_v2_clock_init(id);
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
			printf("Could not find PCI in device tree\n");
			return;
		}
		err = fdt_get_named_resource(gd->fdt_blob, node, "reg", "reg-names", "parf",
				&parf);
		writel(0x0, parf.start + 0x358);
		writel(0x1, parf.start + 0x40);
		err = fdt_get_named_resource(gd->fdt_blob, node, "reg", "reg-names", "pci_phy",
				     &pci_phy);
		if (err < 0)
			return;

		writel(0x1, pci_phy.start + 800);
		writel(0x0, pci_phy.start + 804);
		gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
		if (gpio_node >= 0)
			qca_gpio_deinit(gpio_node);

		pcie_v2_clock_deinit(i);
	}

	return ;
}
#endif

void fdt_fixup_wcss_rproc_for_atf(void *blob)
{
/*
 * Set q6 in non-secure mode only if ATF is enable
 */
	parse_fdt_fixup("/soc/qcom_q6v5_wcss@CD00000%qcom,nosecure%1", blob);
	parse_fdt_fixup("/soc/qcom_q6v5_wcss@CD00000%qca,wcss-aon-reset-seq%1", blob);
/*
 * Set btss in non-secure mode only if ATF is enable
 */
	parse_fdt_fixup("/soc/bt@7000000%qcom,nosecure%1", blob);
}

void run_tzt(void *address)
{
	execute_tzt(address);
}
