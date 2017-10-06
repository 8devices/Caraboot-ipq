/*
 * Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <common.h>
#include <miiphy.h>
#include <phy.h>
#include <asm/io.h>
#include <errno.h>
#include "ipq_mdio.h"

struct ipq_mdio_data {
	struct mii_bus *bus;
	int phy_irq[PHY_MAX_ADDR];
};

static int ipq_mdio_wait_busy(void)
{
	int i;
	u32 busy;
	for (i = 0; i < IPQ_MDIO_RETRY; i++) {
		udelay(IPQ_MDIO_DELAY);
		busy = readl(IPQ_MDIO_BASE +
			MDIO_CTRL_4_REG) &
			MDIO_CTRL_4_ACCESS_BUSY;

		if (!busy)
			return 0;
		udelay(IPQ_MDIO_DELAY);
	}
	printf("%s: MDIO operation timed out\n",
			__func__);
	return -ETIMEDOUT;
}

int ipq_mdio_write(int mii_id, int regnum, u16 value)
{
	u32 cmd;
	if (ipq_mdio_wait_busy())
		return -ETIMEDOUT;


	if (regnum & MII_ADDR_C45) {
		unsigned int mmd = (regnum >> 16) & 0x1F;
	        unsigned int reg = regnum & 0xFFFF;

		writel(CTRL_0_REG_C45_DEFAULT_VALUE,
			IPQ_MDIO_BASE + MDIO_CTRL_0_REG);

		/* Issue the phy address and reg */
		writel((mii_id << 8) | mmd,
			IPQ_MDIO_BASE + MDIO_CTRL_1_REG);

		writel(reg, IPQ_MDIO_BASE + MDIO_CTRL_2_REG);

		/* issue read command */
		cmd = MDIO_CTRL_4_ACCESS_START | MDIO_CTRL_4_ACCESS_CODE_C45_ADDR;

		writel(cmd, IPQ_MDIO_BASE + MDIO_CTRL_4_REG);

		if (ipq_mdio_wait_busy())
			return -ETIMEDOUT;
	} else {
		writel(CTRL_0_REG_DEFAULT_VALUE,
			IPQ_MDIO_BASE + MDIO_CTRL_0_REG);

		/* Issue the phy addreass and reg */
		writel((mii_id << 8 | regnum),
			IPQ_MDIO_BASE + MDIO_CTRL_1_REG);
	}

	/* Issue a write data */
	writel(value, IPQ_MDIO_BASE + MDIO_CTRL_2_REG);

	if (regnum & MII_ADDR_C45) {
		cmd = MDIO_CTRL_4_ACCESS_START | MDIO_CTRL_4_ACCESS_CODE_C45_WRITE ;
	} else {
		cmd = MDIO_CTRL_4_ACCESS_START | MDIO_CTRL_4_ACCESS_CODE_WRITE ;
	}

	writel(cmd, IPQ_MDIO_BASE + MDIO_CTRL_4_REG);
	/* Wait for write complete */

	if (ipq_mdio_wait_busy())
		return -ETIMEDOUT;

	return 0;
}

int ipq_mdio_read(int mii_id, int regnum, ushort *data)
{
	u32 val,cmd;
	if (ipq_mdio_wait_busy())
		return -ETIMEDOUT;

	if (regnum & MII_ADDR_C45) {

		unsigned int mmd = (regnum >> 16) & 0x1F;
	        unsigned int reg = regnum & 0xFFFF;

		writel(CTRL_0_REG_C45_DEFAULT_VALUE,
			IPQ_MDIO_BASE + MDIO_CTRL_0_REG);

		/* Issue the phy address and reg */
		writel((mii_id << 8) | mmd,
			IPQ_MDIO_BASE + MDIO_CTRL_1_REG);


		writel(reg, IPQ_MDIO_BASE + MDIO_CTRL_2_REG);

		/* issue read command */
		cmd = MDIO_CTRL_4_ACCESS_START | MDIO_CTRL_4_ACCESS_CODE_C45_ADDR;
	} else {

		writel(CTRL_0_REG_DEFAULT_VALUE,
			IPQ_MDIO_BASE + MDIO_CTRL_0_REG);

		/* Issue the phy address and reg */
		writel((mii_id << 8 | regnum ) ,
			IPQ_MDIO_BASE + MDIO_CTRL_1_REG);

		/* issue read command */
		cmd = MDIO_CTRL_4_ACCESS_START | MDIO_CTRL_4_ACCESS_CODE_READ ;
	}

	/* issue read command */
	writel(cmd, IPQ_MDIO_BASE + MDIO_CTRL_4_REG);

	if (ipq_mdio_wait_busy())
		return -ETIMEDOUT;


	 if (regnum & MII_ADDR_C45) {
		cmd = MDIO_CTRL_4_ACCESS_START | MDIO_CTRL_4_ACCESS_CODE_C45_READ;
		writel(cmd, IPQ_MDIO_BASE + MDIO_CTRL_4_REG);

		if (ipq_mdio_wait_busy())
			return -ETIMEDOUT;
	}

	/* Read data */
	val = readl(IPQ_MDIO_BASE + MDIO_CTRL_3_REG);

	if (data != NULL)
		*data = val;

	return val;
}

int ipq_phy_write(struct mii_dev *bus,
		int addr, int dev_addr,
		int regnum, ushort value)
{
	return ipq_mdio_write(addr, regnum, value);
}

int ipq_phy_read(struct mii_dev *bus,
		int addr, int dev_addr, int regnum)
{
	return ipq_mdio_read(addr, regnum, NULL);
}

int ipq_sw_mdio_init(const char *name)
{
	struct mii_dev *bus = mdio_alloc();
	if(!bus) {
		printf("Failed to allocate IPQ MDIO bus\n");
		return -1;
	}
	bus->read = ipq_phy_read;
	bus->write = ipq_phy_write;
	bus->reset = NULL;
	sprintf(bus->name, name);
	return mdio_register(bus);
}
