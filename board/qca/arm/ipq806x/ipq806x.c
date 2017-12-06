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
#include <asm-generic/errno.h>
#include <environment.h>
#include <fdtdec.h>
#include <asm/arch-qca-common/gsbi.h>
#include <asm/arch-qca-common/uart.h>
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/smem.h>
#include <asm/arch-ipq806x/msm_ipq806x_gmac.h>
#include <linux/mtd/ipq_nand.h>
#include <asm/arch-qca-common/nand.h>
#include <asm/arch-ipq806x/clk.h>
#include "ipq806x.h"
#include "qca_common.h"
#include <asm/arch-qca-common/scm.h>

ipq_gmac_board_cfg_t gmac_cfg[CONFIG_IPQ_NO_MACS];
DECLARE_GLOBAL_DATA_PTR;

qca_mmc mmc_host;

const char *rsvd_node = "/reserved-memory";
const char *del_node[] = {NULL};
const add_node_t add_node[] = {{}};

struct dumpinfo_t dumpinfo_n[] = {
	/* Note1: when aligned access is set, the contents
	 * are copied to a temporary location and so
	 * the size of region should not exceed the size
	 * of region pointed by IPQ_TEMP_DUMP_ADDR
	 *
	 * Note2: IPQ_NSSTCM_DUMP_ADDR should be the
	 * first entry */
	{ "NSSTCM.BIN",   IPQ_NSSTCM_DUMP_ADDR, 0x20000, 0 },
	{ "IMEM_A.BIN",   0x2a000000, 0x0003f000, 0 },
	{ "IMEM_C.BIN",   0x2a03f000, 0x00001000, 0 },
	{ "IMEM_D.BIN",   0x2A040000, 0x00020000, 0 },
	{ "CODERAM.BIN",  0x00020000, 0x00028000, 0 },
	{ "SPS_RAM.BIN",  0x12000000, 0x0002C000, 0 },
	{ "RPM_MSG.BIN",  0x00108000, 0x00005fff, 1 },
	{ "SPS_BUFF.BIN", 0x12040000, 0x00004000, 0 },
	{ "SPS_PIPE.BIN", 0x12800000, 0x00008000, 0 },
	{ "LPASS.BIN",    0x28400000, 0x00020000, 0 },
	{ "RPM_WDT.BIN",  0x0006206C, 0x00000004, 0 },
	{ "CPU0_WDT.BIN", 0x0208A044, 0x00000004, 0 },
	{ "CPU1_WDT.BIN", 0x0209A044, 0x00000004, 0 },
	{ "CPU0_REG.BIN", 0x39013ea8, 0x000000AC, 0 },
	{ "CPU1_REG.BIN", 0x39013f54, 0x000000AC, 0 },
	{ "WLAN_FW.BIN",  0x41400000, 0x000FFF80, 0 },
	{ "WLAN_FW_900B.BIN", 0x44000000, 0x00600000, 0 },
	{ "EBICS0.BIN",   0x40000000, 0x20000000, 0 },
	{ "EBI1CS1.BIN",  0x60000000, 0x20000000, 0 }
};
int dump_entries_n = ARRAY_SIZE(dumpinfo_n);

struct dumpinfo_t *dumpinfo_s = dumpinfo_n;
int dump_entries_s = dump_entries_n;

extern int ipq_spi_init(u16);

pci_clk_offset_t pcie_0_clk = {
	.aclk_ctl = PCIE_0_ACLK_CTL,
	.pclk_ctl = PCIE_0_PCLK_CTL,
	.hclk_ctl = PCIE_0_HCLK_CTL,
	.aux_clk_ctl = PCIE_0_AUX_CLK_CTL,
	.alt_ref_clk_ns = PCIE_0_ALT_REF_CLK_NS,
	.alt_ref_clk_acr = PCIE_0_ALT_REF_CLK_ACR,
	.aclk_fs = PCIE_0_ACLK_FS,
	.pclk_fs = PCIE_0_PCLK_FS,
	.parf_phy_refclk = PCIE20_0_PARF_PHY_REFCLK
};

pci_clk_offset_t pcie_1_clk = {
	.aclk_ctl = PCIE_1_ACLK_CTL,
	.pclk_ctl = PCIE_1_PCLK_CTL,
	.hclk_ctl = PCIE_1_HCLK_CTL,
	.aux_clk_ctl = PCIE_1_AUX_CLK_CTL,
	.alt_ref_clk_ns = PCIE_1_ALT_REF_CLK_NS,
	.alt_ref_clk_acr = PCIE_1_ALT_REF_CLK_ACR,
	.aclk_fs = PCIE_1_ACLK_FS,
	.pclk_fs = PCIE_1_PCLK_FS,
	.parf_phy_refclk = PCIE20_1_PARF_PHY_REFCLK
};

pci_clk_offset_t pcie_2_clk = {
	.aclk_ctl = PCIE_2_ACLK_CTL,
	.pclk_ctl = PCIE_2_PCLK_CTL,
	.hclk_ctl = PCIE_2_HCLK_CTL,
	.aux_clk_ctl = PCIE_2_AUX_CLK_CTL,
	.alt_ref_clk_ns = PCIE_2_ALT_REF_CLK_NS,
	.alt_ref_clk_acr = PCIE_2_ALT_REF_CLK_ACR,
	.aclk_fs = PCIE_2_ACLK_FS,
	.pclk_fs = PCIE_2_PCLK_FS,
	.parf_phy_refclk = PCIE20_2_PARF_PHY_REFCLK
};

enum pcie_id {
	PCIE_0,
	PCIE_1,
	PCIE_2,
};

unsigned long timer_read_counter(void)
{
	return 0;
}

void reset_crashdump(void)
{
	return;
}

void reset_cpu(unsigned long a)
{
	printf("\nResetting with watch dog!\n");

	writel(0, APCS_WDT0_EN);
	writel(1, APCS_WDT0_RST);
	writel(RESET_WDT_BARK_TIME, APCS_WDT0_BARK_TIME);
	writel(RESET_WDT_BITE_TIME, APCS_WDT0_BITE_TIME);
	writel(1, APCS_WDT0_EN);
	writel(1, APCS_WDT0_CPU0_WDOG_EXPIRED_ENABLE);

	while(1);
}

int board_mmc_init(bd_t *bis)
{
	int node, gpio_node;
	int ret = -ENODEV;
	u32 *emmc_base;
	int len;

	node = fdt_path_offset(gd->fdt_blob, "sdcc");

	if (node < 0) {
		printf("SDCC : Node Not found, skipping initialization\n");
		goto out;
	}

	emmc_base = fdt_getprop(gd->fdt_blob, node, "reg", &len);

	if (emmc_base == FDT_ADDR_T_NONE) {
		printf("No valid SDCC base address found in device tree\n");
		goto out;
        }

	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "emmc_gpio");
	if (gpio_node >= 0) {

		mmc_host.clk_mode = MMC_IDENTIFY_MODE;
		mmc_host.base = fdt32_to_cpu(emmc_base[0]);
		emmc_clock_config(mmc_host.clk_mode);
		qca_gpio_init(gpio_node);
		ret = qca_mmc_init(bis, &mmc_host);
	}

out:
	return ret;
}
void board_nand_init(void)
{
	int node, gpio_node;
	u32 *nand_base;
	struct ipq_nand ipq_nand;
	int len;

	node = fdt_path_offset(gd->fdt_blob, "nand");

	if (node < 0) {
		printf("NAND : Not found, skipping initialization\n");
		goto spi_init;
	}

	nand_base = fdt_getprop(gd->fdt_blob, node, "reg", &len);

	if (nand_base == FDT_ADDR_T_NONE) {
		printf("No valid NAND base address found in device tree\n");
		goto spi_init;
        }

	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "nand_gpio");
	if (gpio_node >= 0) {
		nand_clock_config();
		memset(&ipq_nand, 0, sizeof(ipq_nand));
		ipq_nand.ebi2cr_regs = fdt32_to_cpu(nand_base[0]);
		ipq_nand.ebi2nd_regs = fdt32_to_cpu(nand_base[2]);
		ipq_nand.layout = IPQ_NAND_LAYOUT_LINUX;
		ipq_nand.variant = QCA_NAND_IPQ;
		qca_gpio_init(gpio_node);
		ipq_nand_init(&ipq_nand);
	}

spi_init:
	if(!(gsbi_pin_config(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS)))
		ipq_spi_init(CONFIG_SPI_FLASH_INFO_IDX);
}

int board_eth_init(bd_t *bis)
{
	int status;
	int gmac_gpio_node = 0, storm_switch_gpio_node = 0;
	int ak01_reset_gpio_node = 0, ak01_config_gpio_node = 0;
	int gmac_cfg_node = 0, offset = 0;
	unsigned int machid;
	int loop = 0, inner_loop = 0;
	int phy_name_len = 0;
	unsigned int tmp_phy_array[8] = {0};
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

			gmac_cfg[loop].is_macsec = fdtdec_get_uint(gd->fdt_blob,
					offset, "is_macsec", 0);

			gmac_cfg[loop].mac_pwr0 = fdtdec_get_uint(gd->fdt_blob,
					offset, "mac_pwr0", 0);

			gmac_cfg[loop].mac_pwr1 = fdtdec_get_uint(gd->fdt_blob,
					offset, "mac_pwr1", 0);

			gmac_cfg[loop].mac_conn_to_phy = fdtdec_get_uint(gd->fdt_blob,
					offset, "mac_conn_to_phy", 0);

			gmac_cfg[loop].phy = fdtdec_get_uint(gd->fdt_blob,
					offset, "phy_interface_type", 0);

			gmac_cfg[loop].phy_addr.count = fdtdec_get_uint(gd->fdt_blob,
					offset, "phy_address_count", 0);

			fdtdec_get_int_array(gd->fdt_blob, offset, "phy_address",
					tmp_phy_array, gmac_cfg[loop].phy_addr.count);

			for(inner_loop = 0; inner_loop < gmac_cfg[loop].phy_addr.count;
					inner_loop++){
				gmac_cfg[loop].phy_addr.addr[inner_loop] =
					(char)tmp_phy_array[inner_loop];
			}

			phy_name_ptr = (char*)fdt_getprop(gd->fdt_blob, offset,
					"phy_name", &phy_name_len);

			strncpy(gmac_cfg[loop].phy_name, phy_name_ptr, phy_name_len);

		}
	}
	gmac_cfg[loop].unit = -1;

	storm_switch_gpio_node = fdt_path_offset(gd->fdt_blob,
						"/storm_switch_gpio");
	if (storm_switch_gpio_node) {
		qca_gpio_init(storm_switch_gpio_node);
	}

	ipq_gmac_common_init(gmac_cfg);

	gmac_gpio_node = fdt_path_offset(gd->fdt_blob, "gmac_gpio");
	if (gmac_gpio_node) {
		qca_gpio_init(gmac_gpio_node);
	}
	/*
	 * Register the swith driver routines before
	 * initializng the GMAC
	 */
	machid = fdtdec_get_uint(gd->fdt_blob, 0, "machid", 0);

	switch (machid) {
		case MACH_TYPE_IPQ806X_AP160_2XX:
			ipq_register_switch(ipq_qca8511_init);
			break;

		case MACH_TYPE_IPQ806X_AK01_1XX:
			ak01_reset_gpio_node = fdt_path_offset(gd->fdt_blob, "/ak01_gmac_reset_gpio");
			if (ak01_reset_gpio_node){
				qca_gpio_init(ak01_reset_gpio_node);
			}

			mdelay(100);

			ak01_config_gpio_node = fdt_path_offset(gd->fdt_blob, "/ak01_gmac_config_gpio");
			if (ak01_config_gpio_node){
				qca_gpio_init(ak01_config_gpio_node);
			};

			ipq_register_switch(NULL);
			break;

		default:
			ipq_register_switch(ipq_athrs17_init);
			break;
	}
	ipq_register_switch(ipq_athrs17_init);

	status = ipq_gmac_init(gmac_cfg);
	return status;
}

void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int serial_node, gpio_node, uart2_node;
	unsigned gsbi_base;

	serial_node = fdt_path_offset(gd->fdt_blob, "console");
        if (serial_node < 0) {
             return;
        }

	if (plat->port_id == 2) {
		uart2_node = fdt_path_offset(gd->fdt_blob, "uart2");
		if (uart2_node < 0) {
			printf("uart2 node not defined\n");
		} else {
			serial_node = uart2_node;
		}
	}

        gpio_node = fdt_subnode_offset(gd->fdt_blob,
				       serial_node, "serial_gpio");
	gsbi_base = fdtdec_get_uint(gd->fdt_blob,
				    serial_node, "gsbi_base", 0);
	if (!gsbi_base)
		return;

	qca_gpio_init(gpio_node);
	if (!(plat->m_value == -1) || ( plat->n_value == -1) || (plat->d_value == -1))
		uart_clock_config(plat->port_id,
				plat->m_value,
				plat->n_value,
				plat->d_value);

	writel(GSBI_PROTOCOL_CODE_I2C_UART <<
			GSBI_CTRL_REG_PROTOCOL_CODE_S,
			GSBI_CTRL_REG(gsbi_base));

}
void board_pcie_clock_init(int id)
{
	switch(id) {
		case PCIE_0:
			pcie_clock_config(&pcie_0_clk);
			break;
		case PCIE_1:
			pcie_clock_config(&pcie_1_clk);
			break;
		case PCIE_2:
			pcie_clock_config(&pcie_2_clk);
			break;
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

	return;
}

void ipq_fdt_fixup_socinfo(void *blob)
{
	uint32_t cpu_type;
	int nodeoff, ret;

	ret = ipq_smem_get_socinfo_cpu_type(&cpu_type);
	if (ret) {
		printf("ipq: fdt fixup cannot get socinfo\n");
		return;
	}
	nodeoff = fdt_node_offset_by_compatible(blob, -1, "qcom,ipq8064");

	if (nodeoff < 0) {
		printf("ipq: fdt fixup cannot find compatible node\n");
		return;
	}
	ret = fdt_setprop(blob, nodeoff, "cpu_type",
			&cpu_type, sizeof(cpu_type));
	if (ret)
		printf("%s: cannot set cpu type %d\n", __func__, ret);
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

void board_mmc_deinit(void)
{
	emmc_clock_reset();
	emmc_clock_disable();
}

void set_flash_secondary_type(qca_smem_flash_info_t *smem)
{
	/*
	 * Both eMMC and NAND share common GPIOs, only one of them shall be
	 * enabled from device tree, based on board configuration.
	 *
	 * flash_secondary_type is set to eMMC/NAND device whichever is
	 * initialized, as there is no smem entry to differentiate between the
	 * two.
	 */
#ifdef CONFIG_QCA_MMC
	struct mmc *mmc;

	mmc = find_mmc_device(mmc_host.dev_num);
	if (mmc) {
		smem->flash_secondary_type = SMEM_BOOT_MMC_FLASH;
		return;
	}
#endif
	smem->flash_secondary_type = SMEM_BOOT_NAND_FLASH;
	return;
}

int switch_ce_channel_buf(unsigned int channel_id)
{
	int ret;
	switch_ce_chn_buf_t ce1_chn_buf;

	ce1_chn_buf.resource   = CE1_RESOURCE;
	ce1_chn_buf.channel_id = channel_id;

	ret = scm_call(SCM_SVC_TZ, CE_CHN_SWITCH_CMD, &ce1_chn_buf,
		sizeof(switch_ce_chn_buf_t), NULL, 0);

	return ret;
}
