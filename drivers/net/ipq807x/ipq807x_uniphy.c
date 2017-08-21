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
