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
#include <miiphy.h>
#include "ipq_phy.h"
#include "ipq807x_aquantia_phy.h"

extern int ipq_mdio_write(int mii_id,
		int regnum, u16 value);
extern int ipq_mdio_read(int mii_id,
		int regnum, ushort *data);

u16 aq_phy_reg_write(u32 dev_id, u32 phy_id,
		u32 reg_id, u16 reg_val)
{
	ipq_mdio_write(phy_id, reg_id, reg_val);
	return 0;
}

u16 aq_phy_reg_read(u32 dev_id, u32 phy_id, u32 reg_id)
{
	return ipq_mdio_read(phy_id, reg_id, NULL);
}

u8 aq_phy_get_link_status(u32 dev_id, u32 phy_id)
{
	u16 phy_data;
	uint32_t reg;
	
	reg = AQ_PHY_AUTO_STATUS_REG | AQUANTIA_MII_ADDR_C45; 
	phy_data = aq_phy_reg_read(dev_id, phy_id, reg);
	phy_data = aq_phy_reg_read(dev_id, phy_id, reg);
	
	if (((phy_data >> 2) & 0x1) & PORT_LINK_UP)
		return 0;

	return 1;
}

u32 aq_phy_get_duplex(u32 dev_id, u32 phy_id, fal_port_duplex_t *duplex)
{
	u16 phy_data;
	uint32_t reg;

	reg = AQ_PHY_LINK_STATUS_REG | AQUANTIA_MII_ADDR_C45; 
	phy_data = aq_phy_reg_read(dev_id, phy_id, reg);

	/*
	 * Read duplex
	 */
	phy_data = phy_data & 0x1;
	if (phy_data & 0x1)
		*duplex = FAL_FULL_DUPLEX;
	else
		*duplex = FAL_HALF_DUPLEX;

	return 0;
}

u32 aq_phy_get_speed(u32 dev_id, u32 phy_id, fal_port_speed_t *speed)
{
	u16 phy_data;
	uint32_t reg;

	reg = AQ_PHY_LINK_STATUS_REG | AQUANTIA_MII_ADDR_C45; 
	phy_data = aq_phy_reg_read(dev_id, phy_id, reg);

	switch ((phy_data >> 1) & 0x7) {
	case SPEED_10G:
		*speed = FAL_SPEED_10000;
		break;
	case SPEED_5G:
		*speed = FAL_SPEED_5000;
		break;
	case SPEED_2_5G:
		*speed = FAL_SPEED_2500;
		break;
	case SPEED_1000MBS:
		*speed = FAL_SPEED_1000;
		break;
	case SPEED_100MBS:
		*speed = FAL_SPEED_100;
		break;
	case SPEED_10MBS:
		*speed = FAL_SPEED_10;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int ipq_qca_aquantia_phy_init(struct phy_ops **ops, u32 phy_id)
{
	u16 phy_data;
	struct phy_ops *aq_phy_ops;
	aq_phy_ops = (struct phy_ops *)malloc(sizeof(struct phy_ops));
	if (!aq_phy_ops)
		return -ENOMEM;
	aq_phy_ops->phy_get_link_status = aq_phy_get_link_status;
	aq_phy_ops->phy_get_speed = aq_phy_get_speed;
	aq_phy_ops->phy_get_duplex = aq_phy_get_duplex;
	*ops = aq_phy_ops;

	phy_data = aq_phy_reg_read(0x0, phy_id, AQUANTIA_REG_ADDRESS(1, QCA_PHY_ID1));
	printf ("PHY ID1: 0x%x\n", phy_data);
	phy_data = aq_phy_reg_read(0x0, phy_id, AQUANTIA_REG_ADDRESS(1, QCA_PHY_ID2));
	printf ("PHY ID2: 0x%x\n", phy_data);
	phy_data = aq_phy_reg_read(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_PHY_XS_REGISTERS,
			AQUANTIA_PHY_XS_USX_TRANSMIT));
	phy_data |= AQUANTIA_PHY_USX_AUTONEG_ENABLE;
	aq_phy_reg_write(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_PHY_XS_REGISTERS,
			AQUANTIA_PHY_XS_USX_TRANSMIT), phy_data);
	phy_data = aq_phy_reg_read(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_AUTONEG,
			AQUANTIA_AUTONEG_TRANSMIT_VENDOR_INTR_MASK));
	phy_data |= AQUANTIA_INTR_LINK_STATUS_CHANGE;
	aq_phy_reg_write(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_AUTONEG,
			AQUANTIA_AUTONEG_TRANSMIT_VENDOR_INTR_MASK), phy_data);
	phy_data = aq_phy_reg_read(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_GLOABLE_REGISTERS,
			AQUANTIA_GLOBAL_INTR_STANDARD_MASK));
	phy_data |= AQUANTIA_ALL_VENDOR_ALARMS_INTERRUPT_MASK;
	aq_phy_reg_write(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_GLOABLE_REGISTERS,
			AQUANTIA_GLOBAL_INTR_STANDARD_MASK), phy_data);
	phy_data = aq_phy_reg_read(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_GLOABLE_REGISTERS,
			AQUANTIA_GLOBAL_INTR_VENDOR_MASK));
	phy_data |= AQUANTIA_AUTO_AND_ALARMS_INTR_MASK;
	aq_phy_reg_write(0x0, phy_id, AQUANTIA_REG_ADDRESS(AQUANTIA_MMD_GLOABLE_REGISTERS,
			AQUANTIA_GLOBAL_INTR_VENDOR_MASK), phy_data);

	return 0;
}

