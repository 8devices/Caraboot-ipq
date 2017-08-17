/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <common.h>
#include <net.h>
#include <asm-generic/errno.h>
#include <asm/io.h>
#include <malloc.h>
#include <phy.h>
#include <net.h>
#include <miiphy.h>
#include <asm/arch-ipq807x/edma_regs.h>
#include "ipq807x_edma.h"
#include "ipq807x_uniphy.h"

extern int ipq_mdio_write(int mii_id,
		int regnum, u16 value);
extern int ipq_mdio_read(int mii_id,
		int regnum, ushort *data);

void csr1_write(int phy_id, int addr, int  value)
{
	int  addr_h, addr_l, ahb_h, ahb_l,  phy;
	phy=phy_id<<(0x10);
	addr_h=(addr&0xffffff)>>8;
	addr_l=((addr&0xff)<<2)|(0x20<<(0xa));
	ahb_l=(addr_l&0xffff)|(0x7A00000|phy);
	ahb_h=(0x7A083FC|phy);
	writel(addr_h,ahb_h);
	writel(value,ahb_l);
}

int  csr1_read(int phy_id, int  addr )
{
	int  addr_h ,addr_l,ahb_h, ahb_l, phy;
	phy=phy_id<<(0x10);
	addr_h=(addr&0xffffff)>>8;
	addr_l=((addr&0xff)<<2)|(0x20<<(0xa));
	ahb_l=(addr_l&0xffff)|(0x7A00000|phy);
	ahb_h=(0x7A083FC|phy);
	writel(addr_h, ahb_h);
	return  readl(ahb_l);
}

static void ppe_gcc_uniphy_xpcs_reset(uint32_t uniphy_index, bool enable)
{
	uint32_t reg_value;

	if(enable)
		reg_value = GCC_UNIPHY_USXGMII_XPCS_RESET;
	else
		reg_value = GCC_UNIPHY_USXGMII_XPCS_RELEASE_RESET;

	writel(reg_value, GCC_UNIPHY0_MISC + (uniphy_index * GCC_UNIPHY_REG_INC));
}

static void ppe_gcc_uniphy_soft_reset(uint32_t uniphy_index)
{
	uint32_t reg_value;

	reg_value = readl(GCC_UNIPHY0_MISC + (uniphy_index * GCC_UNIPHY_REG_INC));
	if (uniphy_index == PPE_UNIPHY_INSTANCE0)
		reg_value |= GCC_UNIPHY_PSGMII_SOFT_RESET;
	else
		reg_value = GCC_UNIPHY_USXGMII_SOFT_RESET;

	writel(reg_value, GCC_UNIPHY0_MISC + (uniphy_index * GCC_UNIPHY_REG_INC));
	udelay(500);
	if (uniphy_index == PPE_UNIPHY_INSTANCE0)
		reg_value &= ~GCC_UNIPHY_PSGMII_SOFT_RESET;
	else
		reg_value = GCC_UNIPHY_USXGMII_XPCS_RESET;

	writel(reg_value, GCC_UNIPHY0_MISC + (uniphy_index * GCC_UNIPHY_REG_INC));
}

static void ppe_uniphy_psgmii_mode_set(uint32_t uniphy_index)
{
	ppe_gcc_uniphy_xpcs_reset(uniphy_index, true);
	writel(0x220, PPE_UNIPHY_BASE + (uniphy_index * PPE_UNIPHY_REG_INC)
			+ PPE_UNIPHY_MODE_CONTROL);
	ppe_gcc_uniphy_soft_reset(uniphy_index);
}

static void ppe_uniphy_sgmii_mode_set(uint32_t uniphy_index)
{
	writel(UNIPHY_MISC2_REG_SGMII_MODE, PPE_UNIPHY_BASE +
		(uniphy_index * PPE_UNIPHY_REG_INC) + UNIPHY_MISC2_REG_OFFSET);
	writel(UNIPHY_PLL_RESET_REG_VALUE, PPE_UNIPHY_BASE +
		(uniphy_index * PPE_UNIPHY_REG_INC) + UNIPHY_PLL_RESET_REG_OFFSET);
	udelay(500);
	writel(UNIPHY_PLL_RESET_REG_DEFAULT_VALUE, PPE_UNIPHY_BASE +
		(uniphy_index * PPE_UNIPHY_REG_INC) + UNIPHY_PLL_RESET_REG_OFFSET);
	ppe_gcc_uniphy_xpcs_reset(uniphy_index, true);
	writel(0x420, PPE_UNIPHY_BASE + (uniphy_index * PPE_UNIPHY_REG_INC)
			 + PPE_UNIPHY_MODE_CONTROL);
	ppe_gcc_uniphy_soft_reset(uniphy_index);
}

void ppe_uniphy_mode_set(uint32_t uniphy_index, uint32_t mode)
{
	switch(mode) {
		case PORT_WRAPPER_PSGMII:
			ppe_uniphy_psgmii_mode_set(uniphy_index);
			break;
		case PORT_WRAPPER_SGMII0_RGMII4:
			ppe_uniphy_sgmii_mode_set(uniphy_index);
			break;
		default:
			break;
	}
}
