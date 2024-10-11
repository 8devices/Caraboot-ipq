/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#include <common.h>
#include <net.h>
#include <asm-generic/errno.h>
#include <asm/io.h>
#include <malloc.h>
#include <phy.h>
#include "ipq_phy.h"

extern int ipq_mdio_read(int mii_id,
		int regnum, ushort *data);
extern int ipq_mdio_write(int mii_id,
		int regnum, u16 data);

u16 gpy211_phy_reg_read(u32 dev_id, u32 phy_id, u32 reg_id)
{
	return ipq_mdio_read(phy_id, reg_id, NULL);
}

u16 gpy211_phy_reg_write(u32 dev_id, u32 phy_id, u32 reg_id, u16 value)
{
	return ipq_mdio_write(phy_id, reg_id, value);
}
#define GPY211_PHY_MIISTAT 24

/* PHY ID */
#define PHY_ID_GPY		0x67C9DC00
#define PHY_ID_MASK		GENMASK(31, 4)

#define PHY_MIISTAT		0x18	/* MII state */
#define PHY_IMASK		0x19	/* interrupt mask */
#define PHY_ISTAT		0x1A	/* interrupt status */
#define PHY_FWV			0x1E	/* firmware version */

#define PHY_MIISTAT_SPD_MASK	GENMASK(2, 0)
#define PHY_MIISTAT_DPX		BIT(3)
#define PHY_MIISTAT_LS		BIT(10)

#define PHY_MIISTAT_SPD_10	0
#define PHY_MIISTAT_SPD_100	1
#define PHY_MIISTAT_SPD_1000	2
#define PHY_MIISTAT_SPD_2500	4

#define IPQ60XX_MII_ADDR_C45			(1<<30)
#define IPQ60XX_REG_C45_ADDRESS(dev_type, reg_num) (IPQ60XX_MII_ADDR_C45 | \
                        ((dev_type & 0x1f) << 16) | (reg_num & 0xffff))

/* SGMII */
#define MDIO_MMD_VEND1	30
#define VSPEC1_SGMII_CTRL	0x08
#define VSPEC1_SGMII_CTRL_ANEN	BIT(12)		/* Aneg enable */
#define VSPEC1_SGMII_CTRL_ANRS	BIT(9)		/* Restart Aneg */
#define VSPEC1_SGMII_ANEN_ANRS	(VSPEC1_SGMII_CTRL_ANEN | \
				 VSPEC1_SGMII_CTRL_ANRS)

u8 gpy211_phy_get_link_status(u32 dev_id, u32 phy_id)
{
	u16 phy_data;
	phy_data = gpy211_phy_reg_read(dev_id,
			phy_id, PHY_MIISTAT);
	if (phy_data & PHY_MIISTAT_LS)
		return 0;

	return 1;
}

u32 gpy211_phy_get_duplex(u32 dev_id, u32 phy_id, fal_port_duplex_t *duplex)
{
	u16 phy_data;

	phy_data = gpy211_phy_reg_read(dev_id, phy_id, PHY_MIISTAT);

	if (phy_data & PHY_MIISTAT_DPX)
		*duplex = FAL_FULL_DUPLEX;
	else
		*duplex = FAL_HALF_DUPLEX;

	return 0;
}

u32 ipq_gpy211_phy_set_interface_mode(u32 dev_id, u32 phy_id, enum port_wrapper_cfg mode);
u32 gpy211_phy_get_speed(u32 dev_id, u32 phy_id, fal_port_speed_t *speed)
{
	u16 phy_data;

	phy_data = gpy211_phy_reg_read(dev_id,
			phy_id, PHY_MIISTAT);

	switch (phy_data & PHY_MIISTAT_SPD_MASK) {
	case PHY_MIISTAT_SPD_2500:
		*speed = FAL_SPEED_2500;
		break;
	case PHY_MIISTAT_SPD_1000:
		*speed = FAL_SPEED_1000;
		break;
	case PHY_MIISTAT_SPD_100:
		*speed = FAL_SPEED_100;
		break;
	case PHY_MIISTAT_SPD_10:
		*speed = FAL_SPEED_10;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

u32 ipq_gpy211_phy_set_interface_mode(u32 dev_id, u32 phy_id, enum port_wrapper_cfg mode)
{
	u16 phy_data;

	phy_data = gpy211_phy_reg_read(dev_id, phy_id, IPQ60XX_REG_C45_ADDRESS(MDIO_MMD_VEND1, VSPEC1_SGMII_CTRL));

	if (mode == PORT_WRAPPER_SGMII_PLUS) {
		phy_data &= ~(VSPEC1_SGMII_CTRL_ANEN); //disable SGMII autoneg
	} else {
		phy_data |= (VSPEC1_SGMII_ANEN_ANRS); //enable and restart SGMII autoneg
	}

	gpy211_phy_reg_write(dev_id, phy_id,
			     IPQ60XX_REG_C45_ADDRESS(MDIO_MMD_VEND1, VSPEC1_SGMII_CTRL),
			     phy_data);
	return 0;
}

int ipq_gpy211_phy_init(struct phy_ops **ops, u32 phy_id)
{
	u16 phy_data;
	struct phy_ops *gpy211_ops;

	gpy211_ops = (struct phy_ops *)malloc(sizeof(struct phy_ops));
	if (!gpy211_ops)
		return -ENOMEM;
	gpy211_ops->phy_get_link_status = gpy211_phy_get_link_status;
	gpy211_ops->phy_get_speed = gpy211_phy_get_speed;
	gpy211_ops->phy_get_duplex = gpy211_phy_get_duplex;
	gpy211_ops->phy_set_interface_mode = ipq_gpy211_phy_set_interface_mode;
	*ops = gpy211_ops;

	phy_data = gpy211_phy_reg_read(0x0, phy_id, QCA_PHY_ID1);
	printf ("PHY ID1: 0x%x\n", phy_data);
	phy_data = gpy211_phy_reg_read(0x0, phy_id, QCA_PHY_ID2);
	printf ("PHY ID2: 0x%x\n", phy_data);

	return 0;
}

