/*
 * Copyright (c) 2014, 2015-2017 The Linux Foundation. All rights reserved.
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
#include <pci.h>

#include <linux/sizes.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/iomap.h>
#include <fdtdec.h>
#include <dm.h>

DECLARE_GLOBAL_DATA_PTR;

#define PCIE_RST_CTRL_PIPE_ARES			0x4
#define PCIE_RST_CTRL_PIPE_STICKY_ARES		0x100
#define PCIE_RST_CTRL_PIPE_PHY_AHB_ARES		0x800
#define PCIE_RST_CTRL_AXI_M_ARES		0x1
#define PCIE_RST_CTRL_AXI_M_STICKY_ARES		0x80
#define PCIE_RST_CTRL_AXI_S_ARES		0x2
#define PCIE_RST_CTRL_AHB_ARES			0x400


#define PCI_CFG0_RDWR			0x4
#define PCI_CFG1_RDWR			0x5
#define RD				0
#define WR 				1

#define MSM_PCIE_DEV_CFG_ADDR                   0x01000000
#define PCIE20_PLR_IATU_VIEWPORT		0x900
#define PCIE20_PLR_IATU_CTRL1			0x904
#define PCIE20_PLR_IATU_CTRL2			0x908
#define PCIE20_PLR_IATU_LBAR			0x90C
#define PCIE20_PLR_IATU_UBAR			0x910
#define PCIE20_PLR_IATU_LAR			0x914
#define PCIE20_PLR_IATU_LTAR			0x918
#define PCIE20_PLR_IATU_UTAR			0x91c

/* PCIE20_PARF_PHYS Registers */
#define PARF_SLV_ADDR_SPACE_SIZE		0x16C
#define SLV_ADDR_SPACE_SZ			0x40000000
#define PCIE_0_PCIE20_PARF_LTSSM		0x1B0
#define LTSSM_EN				(1 << 8)
/* PCIE20_PHYS Registers */
#define PCIE_0_PORT_FORCE_REG			0x708
#define PCIE_0_ACK_F_ASPM_CTRL_REG		0x70C
#define L1_ENTRANCE_LATENCY(x)			(x << 27)
#define L0_ENTRANCE_LATENCY(x)			(x << 24)
#define COMMON_CLK_N_FTS(x)			(x << 16)
#define ACK_N_FTS(x)				(x << 8)

#define PCIE_0_GEN2_CTRL_REG			0x80C
#define FAST_TRAINING_SEQ(x)			(x << 0)
#define NUM_OF_LANES(x)				(x << 8)
#define DIRECT_SPEED_CHANGE			(1 << 17)

#define PCIE_0_TYPE0_STATUS_COMMAND_REG_1	0x004
#define PCI_TYPE0_BUS_MASTER_EN			(1 << 2)

#define PCIE_0_MISC_CONTROL_1_REG		0x8BC
#define DBI_RO_WR_EN				(1 << 0)

#define PCIE_0_LINK_CAPABILITIES_REG		0x07C
#define PCIE_CAP_ASPM_OPT_COMPLIANCE		(1 << 22)
#define PCIE_CAP_LINK_BW_NOT_CAP		(1 << 21)
#define PCIE_CAP_DLL_ACTIVE_REP_CAP		(1 << 20)
#define PCIE_CAP_L1_EXIT_LATENCY(x)		(x << 15)
#define PCIE_CAP_L0S_EXIT_LATENCY(x)		(x << 12)
#define PCIE_CAP_MAX_LINK_WIDTH(x)		(x << 4)
#define PCIE_CAP_MAX_LINK_SPEED(x)		(x << 0)

#define PCIE_0_DEVICE_CONTROL2_DEVICE_STATUS2_REG	0x098
#define PCIE_CAP_CPL_TIMEOUT_DISABLE			(1 << 4)
#define PCIE_0_TYPE0_LINK_CONTROL_LINK_STATUS_REG_1	0x080
#define PCIE20_AXI_MSTR_RESP_COMP_CTRL0                 0x818
#define PCIE20_AXI_MSTR_RESP_COMP_CTRL1                 0x81c

#define PCIE20_SIZE   			SZ_4K
#define PCIE_AXI_CONF_SIZE   		SZ_1M

static unsigned int local_buses[] = { 0, 0 };
struct pci_controller pci_hose[PCI_MAX_DEVICES];

enum pcie_verion{
	PCIE_V0,
	PCIE_V1,
	PCIE_V2,
};

static const struct udevice_id pcie_ver_ids[] = {
	{ .compatible = "qcom,ipq806x-pcie", .data = PCIE_V0 },
	{ .compatible = "qcom,ipq40xx-pcie", .data = PCIE_V1 },
	{ .compatible = "qcom,ipq807x-pcie", .data = PCIE_V2 },
	{ },
};


struct ipq_pcie {
	struct pci_controller hose;
	struct fdt_resource pci_dbi;
	struct fdt_resource parf;
	struct fdt_resource elbi;
	struct fdt_resource axi_conf;
	struct fdt_resource axi_bars;
	struct fdt_resource pci_rst;

	int rst_gpio;
	int linkup;
	int version;
};

void ipq_pcie_config_cfgtype(uint32_t phyaddr)
{
	uint32_t bdf, cfgtype;

	cfgtype = PCI_CFG0_RDWR;
	bdf = MSM_PCIE_DEV_CFG_ADDR;

	writel(0, phyaddr + PCIE20_PLR_IATU_VIEWPORT);

	/* Program Bdf Address */
	writel(bdf, phyaddr + PCIE20_PLR_IATU_LTAR);

	/* Write Config Request Type */
	writel(cfgtype, phyaddr + PCIE20_PLR_IATU_CTRL1);
}

int ipq_pcie_rd_conf_byte(struct  pci_controller *hose, pci_dev_t dev,
				     int offset, u8 *val)
{
	int bus = PCI_BUS (dev);
	uint32_t addr;
	uint32_t word_offset, byte_offset, mask;
	uint32_t rd_val;

	word_offset = offset & ~0x3;
	byte_offset = offset & 0x3;
	mask = (~0 >> (8 * (4 - 1))) << (8 * byte_offset);

	ipq_pcie_config_cfgtype(hose->regions[0].phys_start);
	if ((bus == local_buses[0]) || (bus == local_buses[1])) {
		addr = hose->regions[0].phys_start;
	} else {
		addr = hose->regions[1].phys_start;
	}
	rd_val = readl(addr + word_offset);
	*val = ((rd_val & mask) >> (8 * byte_offset));

	return 0;
}

int ipq_pcie_rd_conf_word(struct  pci_controller *hose, pci_dev_t dev,
				     int offset, u16 *val)
{
	int bus = PCI_BUS (dev);
	uint32_t addr;
	uint32_t word_offset, byte_offset, mask;
	uint32_t rd_val;

	word_offset = offset & ~0x3;
	byte_offset = offset & 0x3;
	mask = (~0 >> (8 * (4 - 2))) << (8 * byte_offset);

	ipq_pcie_config_cfgtype(hose->regions[0].phys_start);
	if ((bus == local_buses[0]) || (bus == local_buses[1])) {
		addr = hose->regions[0].phys_start;
	} else {
		addr = hose->regions[1].phys_start;
	}
	rd_val = readl(addr + word_offset);
	*val = ((rd_val & mask) >> (8 * byte_offset));

	return 0;
}
int ipq_pcie_rd_conf_dword(struct  pci_controller *hose, pci_dev_t dev,
				     int offset, u32 *val)
{
	int bus = PCI_BUS (dev);
	uint32_t addr;
	uint32_t word_offset, byte_offset, mask;
	uint32_t rd_val;

	word_offset = offset & ~0x3;
	byte_offset = offset & 0x3;
	mask = (~0 >> (8 * (4 - 4))) << (8 * byte_offset);

	ipq_pcie_config_cfgtype(hose->regions[0].phys_start);
	if ((bus == local_buses[0]) || (bus == local_buses[1])) {
		addr = hose->regions[0].phys_start;
	} else {
		addr = hose->regions[1].phys_start;
	}
	rd_val = readl(addr + word_offset);
	*val = ((rd_val & mask) >> (8 * byte_offset));

	return 0;
}

int ipq_pcie_wr_conf_byte(struct  pci_controller *hose, pci_dev_t dev,
				     int offset, u8 val)
{
	int bus = PCI_BUS (dev);
	uint32_t addr;
	uint32_t word_offset, byte_offset, mask;
	uint32_t rd_val, wr_val;

	word_offset = offset & ~0x3;
	byte_offset = offset & 0x3;
	mask = (~0 >> (8 * (4 - 1))) << (8 * byte_offset);

	ipq_pcie_config_cfgtype(hose->regions[0].phys_start);
	if ((bus == local_buses[0]) || (bus == local_buses[1])) {
		addr = hose->regions[0].phys_start;
	} else {
		addr = hose->regions[1].phys_start;
	}
	rd_val = readl(addr + word_offset);
	wr_val = (rd_val & ~mask) |((val << (8 * byte_offset)) & mask);
	writel(wr_val, addr + word_offset);

	return 0;
}

int ipq_pcie_wr_conf_word(struct  pci_controller *hose, pci_dev_t dev,
				     int offset, u16 val)
{
	int bus = PCI_BUS (dev);
	uint32_t addr;
	uint32_t word_offset, byte_offset, mask;
	uint32_t rd_val, wr_val;

	word_offset = offset & ~0x3;
	byte_offset = offset & 0x3;
	mask = (~0 >> (8 * (4 - 2))) << (8 * byte_offset);

	ipq_pcie_config_cfgtype(hose->regions[0].phys_start);
	if ((bus == local_buses[0]) || (bus == local_buses[1])) {
		addr = hose->regions[0].phys_start;
	} else {
		addr = hose->regions[1].phys_start;
	}
	rd_val = readl(addr + word_offset);
	wr_val = (rd_val & ~mask) |((val << (8 * byte_offset)) & mask);
	writel(wr_val, addr + word_offset);

	return 0;
}

int ipq_pcie_wr_conf_dword(struct  pci_controller *hose, pci_dev_t dev,
				     int offset, u32 val)
{
	int bus = PCI_BUS (dev);
	uint32_t addr;
	uint32_t word_offset, byte_offset, mask;
	uint32_t rd_val, wr_val;

	word_offset = offset & ~0x3;
	byte_offset = offset & 0x3;
	mask = (~0 >> (8 * (4 - 4))) << (8 * byte_offset);

	ipq_pcie_config_cfgtype(hose->regions[0].phys_start);
	if ((bus == local_buses[0]) || (bus == local_buses[1])) {
		addr = hose->regions[0].phys_start;
	} else {
		addr = hose->regions[1].phys_start;
	}
	rd_val = readl(addr + word_offset);
	wr_val = (rd_val & ~mask) |((val << (8 * byte_offset)) & mask);
	writel(wr_val, addr + word_offset);

	return 0;
}

static void ipq_pcie_config_controller(struct ipq_pcie *pcie)
{

	/*
	 * program and enable address translation region 0 (device config
	 * address space); region type config;
	 * axi config address range to device config address range
	 */
	writel(0, pcie->pci_dbi.start + PCIE20_PLR_IATU_VIEWPORT);

	writel(4, pcie->pci_dbi.start + PCIE20_PLR_IATU_CTRL1);
	writel((1 << 31), pcie->pci_dbi.start + PCIE20_PLR_IATU_CTRL2);
	writel(pcie->axi_conf.start , pcie->pci_dbi.start + PCIE20_PLR_IATU_LBAR);
	writel(0, pcie->pci_dbi.start + PCIE20_PLR_IATU_UBAR);
	writel((pcie->axi_conf.start + pcie->axi_conf.end - 1),
				pcie->pci_dbi.start + PCIE20_PLR_IATU_LAR);
	writel(MSM_PCIE_DEV_CFG_ADDR,
				pcie->pci_dbi.start + PCIE20_PLR_IATU_LTAR);
	writel(0, pcie->pci_dbi.start + PCIE20_PLR_IATU_UTAR);

	/*
	 * program and enable address translation region 2 (device resource
	 * address space); region type memory;
	 * axi device bar address range to device bar address range
	 */
	writel(2, pcie->pci_dbi.start + PCIE20_PLR_IATU_VIEWPORT);

	writel(0, pcie->pci_dbi.start + PCIE20_PLR_IATU_CTRL1);
	writel((1 << 31), pcie->pci_dbi.start + PCIE20_PLR_IATU_CTRL2);
	writel(pcie->axi_bars.start, pcie->pci_dbi.start + PCIE20_PLR_IATU_LBAR);
	writel(0, pcie->pci_dbi.start + PCIE20_PLR_IATU_UBAR);
	writel((pcie->axi_bars.start + pcie->axi_bars.end
		- pcie->axi_conf.end - 1), pcie->pci_dbi.start+ PCIE20_PLR_IATU_LAR);
	writel(pcie->axi_bars.start, pcie->pci_dbi.start + PCIE20_PLR_IATU_LTAR);
	writel(0, pcie->pci_dbi.start + PCIE20_PLR_IATU_UTAR);

	/* 1K PCIE buffer setting */
	writel(0x3, pcie->pci_dbi.start + PCIE20_AXI_MSTR_RESP_COMP_CTRL0);
	writel(0x1, pcie->pci_dbi.start + PCIE20_AXI_MSTR_RESP_COMP_CTRL1);
}

void pcie_linkup(struct ipq_pcie *pcie)
{
	int j, val;

	writel(SLV_ADDR_SPACE_SZ, pcie->parf.start + PARF_SLV_ADDR_SPACE_SIZE);
	mdelay(100);

	writel(0x0, pcie->pci_dbi.start + PCIE_0_PORT_FORCE_REG);
	val = (L1_ENTRANCE_LATENCY(3) |
		L0_ENTRANCE_LATENCY(3) |
		COMMON_CLK_N_FTS(128) |
		ACK_N_FTS(128));
	writel(val, pcie->pci_dbi.start + PCIE_0_ACK_F_ASPM_CTRL_REG);

	val = (FAST_TRAINING_SEQ(128) |
		NUM_OF_LANES(2) |
		DIRECT_SPEED_CHANGE);
	writel(val, pcie->pci_dbi.start + PCIE_0_GEN2_CTRL_REG);
	writel(PCI_TYPE0_BUS_MASTER_EN,
		pcie->pci_dbi.start + PCIE_0_TYPE0_STATUS_COMMAND_REG_1);

	writel(DBI_RO_WR_EN, pcie->pci_dbi.start + PCIE_0_MISC_CONTROL_1_REG);
	writel(0x0002FD7F, pcie->pci_dbi.start + 0x84);

	val = (PCIE_CAP_ASPM_OPT_COMPLIANCE |
		PCIE_CAP_LINK_BW_NOT_CAP |
		PCIE_CAP_DLL_ACTIVE_REP_CAP |
		PCIE_CAP_L1_EXIT_LATENCY(4) |
		PCIE_CAP_L0S_EXIT_LATENCY(4) |
		PCIE_CAP_MAX_LINK_WIDTH(1) |
		PCIE_CAP_MAX_LINK_SPEED(1));
	writel(val, pcie->pci_dbi.start + PCIE_0_LINK_CAPABILITIES_REG);

	writel(PCIE_CAP_CPL_TIMEOUT_DISABLE,
		pcie->pci_dbi.start + PCIE_0_DEVICE_CONTROL2_DEVICE_STATUS2_REG);

	writel(0x10110008, pcie->pci_dbi.start + PCIE_0_TYPE0_LINK_CONTROL_LINK_STATUS_REG_1);

	writel(LTSSM_EN, pcie->parf.start + PCIE_0_PCIE20_PARF_LTSSM);

	mdelay(200);

	for (j = 0; j < 400; j++) {
		val = readl(pcie->pci_dbi.start + PCIE_0_TYPE0_LINK_CONTROL_LINK_STATUS_REG_1);
		if (val & (1 << 29)) {
			printf("PCI Link Intialized\n");
			pcie->linkup = 1;
			break;
		}
		udelay(100);
	}
	ipq_pcie_config_controller(pcie);
}
static int ipq_pcie_parse_dt(const void *fdt, int id,
			       struct ipq_pcie *pcie)
{
	int err, rst_gpio, node;

	if (id == 0) {
		 node = fdt_path_offset(fdt, "pci0");
	} else if (id == 1) {
		 node = fdt_path_offset(fdt, "pci1");
	} else {
		printf("PCI is not defined in the device tree\n");
		return -1;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "pci_dbi",
				     &pcie->pci_dbi);
	if (err < 0) {
		error("resource \"pads\" not found");
		return err;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "parf",
				     &pcie->parf);
	if (err < 0) {
		error("resource \"afi\" not found");
		return err;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "elbi",
				     &pcie->elbi);
	if (err < 0) {
		error("resource \"cs\" not found");
		return err;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "axi_bars",
				     &pcie->axi_bars);
	if (err < 0) {
		error("resource \"cs\" not found");
		return err;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "axi_conf",
				     &pcie->axi_conf);
	if (err < 0) {
		error("resource \"cs\" not found");
		return err;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "pci_rst",
				     &pcie->pci_rst);
	if (err < 0) {
		error("resource \"cs\" not found");
		return err;
	}

	rst_gpio = fdtdec_get_int(fdt, node, "perst_gpio", 0);
	if (rst_gpio <= 0) {
		debug("PCI: Can't get perst_gpio\n");
		return -1;
	}
	pcie->rst_gpio = rst_gpio;

	return 0;
}

void pci_controller_init_v1(struct ipq_pcie *pcie)
{
	uint32_t val;

	/* Assert cc_pcie20_core_ares */
	writel(PCIE_RST_CTRL_PIPE_ARES, pcie->pci_rst.start);

	/* Assert cc_pcie20_core_sticky_area */
	val = readl(pcie->pci_rst.start);
	val |= PCIE_RST_CTRL_PIPE_STICKY_ARES;
	writel(val, pcie->pci_rst.start);

	/* Assert cc_pcie20_phy_ahb_ares */
	val = readl(pcie->pci_rst.start);
	val |= PCIE_RST_CTRL_PIPE_PHY_AHB_ARES;
	writel(val, pcie->pci_rst.start);

	gpio_set_value(pcie->rst_gpio, GPIO_OUT_LOW);

	/* Assert cc_pcie20_mstr_axi_ares */
	val = readl(pcie->pci_rst.start);
	val |= PCIE_RST_CTRL_AXI_M_ARES;
	writel(val, pcie->pci_rst.start);

	/* Assert cc_pcie20_mstr_sticky_ares */
	val = readl(pcie->pci_rst.start);
	val |= PCIE_RST_CTRL_AXI_M_STICKY_ARES;
	writel(val, pcie->pci_rst.start);

	/* Assert cc_pcie20_slv_axi_ares */
	val = readl(pcie->pci_rst.start);
	val |= PCIE_RST_CTRL_AXI_S_ARES;
	writel(val, pcie->pci_rst.start);

	/* Assert cc_pcie20_ahb_ares;  */
	val = readl(pcie->pci_rst.start);
	val |= PCIE_RST_CTRL_AHB_ARES;
	writel(val, pcie->pci_rst.start);

	/* DeAssert cc_pcie20_phy_ahb_ares  */
	val = readl(pcie->pci_rst.start);
	val &= ~(PCIE_RST_CTRL_AHB_ARES);
	writel(val, pcie->pci_rst.start);

	/* DeAssert cc_pcie20_pciephy_phy_ares*/
	val = readl(pcie->pci_rst.start);
	val &= ~(PCIE_RST_CTRL_PIPE_ARES);
	writel(val, pcie->pci_rst.start);

	/* DeAssert cc_pcie20_core_sticky_ares */
	val = readl(pcie->pci_rst.start);
	val &= ~(PCIE_RST_CTRL_PIPE_STICKY_ARES);
	writel(val, pcie->pci_rst.start);

	mdelay(5);

	gpio_set_value(pcie->rst_gpio, GPIO_OUT_HIGH);

	/* DeAssert cc_pcie20_mstr_axi_ares */
	val = readl(pcie->pci_rst.start);
	val &= ~(PCIE_RST_CTRL_AXI_M_ARES);
	writel(val, pcie->pci_rst.start);

	/* DeAssert cc_pcie20_mstr_axi_ares */
	val = readl(pcie->pci_rst.start);
	val &= ~(PCIE_RST_CTRL_AXI_M_STICKY_ARES);
	writel(val, pcie->pci_rst.start);

	/* DeAssert cc_pcie20_slv_axi_ares */
	val = readl(pcie->pci_rst.start);
	val &= ~(PCIE_RST_CTRL_AXI_S_ARES);
	writel(val, pcie->pci_rst.start);

	/* DeAssert cc_pcie20_ahb_ares */
	val = readl(pcie->pci_rst.start);
	val &= ~(PCIE_RST_CTRL_PIPE_PHY_AHB_ARES);
	writel(val, pcie->pci_rst.start);
}

static int pci_ipq_ofdata_to_platdata(int id, struct ipq_pcie *pcie)
{

	if (ipq_pcie_parse_dt(gd->fdt_blob, id, pcie))
		return -EINVAL;

	board_pci_init(id);
	switch(pcie->version) {
		case PCIE_V1:
			pci_controller_init_v1(pcie);
			pcie_linkup(pcie);
			break;
		case PCIE_V2:
			pcie_linkup(pcie);
			break;
		default:
			break;
	}

	return 0;
}

void pci_init_board (void)
{
	struct ipq_pcie *pcie;
	int i, bus = 0, ret;
	const struct udevice_id *of_match = pcie_ver_ids;

	pcie = malloc(sizeof(*pcie));
	while (of_match->compatible) {
		ret = fdt_node_offset_by_compatible(gd->fdt_blob, 0,
						of_match->compatible);
		if (ret < 0) {
			of_match++;
			continue;
		}
		pcie->version = of_match->data;
		break;
	}

	for (i = 0; i < PCI_MAX_DEVICES; i++) {
		pci_ipq_ofdata_to_platdata(i, pcie);
		if (pcie->linkup) {
			pci_hose[i].first_busno = bus;
			pci_hose[i].last_busno = 0xff;
			local_buses[0] = pci_hose[i].first_busno;

			/* PCI memory space */
			pci_set_region (pci_hose[i].regions + 0,
					pcie->pci_dbi.start,
					pcie->pci_dbi.start,
					PCIE20_SIZE, PCI_REGION_MEM);

			/* PCI device confgiuration  space */
			pci_set_region (pci_hose[i].regions + 1,
					pcie->axi_conf.start,
					pcie->axi_conf.start,
				(PCIE_AXI_CONF_SIZE - 1), PCI_REGION_MEM);

			pci_hose[i].region_count = 2;
			pci_register_hose (&pci_hose[i]);
			pci_set_ops (&pci_hose[i],
				ipq_pcie_rd_conf_byte,
				ipq_pcie_rd_conf_word,
				ipq_pcie_rd_conf_dword,
				ipq_pcie_wr_conf_byte,
				ipq_pcie_wr_conf_word,
				ipq_pcie_wr_conf_dword);

			pci_hose[i].last_busno = pci_hose[i].first_busno + 1;
			bus = pci_hose[i].last_busno + 1;
		}
	}
}
