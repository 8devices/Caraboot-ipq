/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.

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
#include <asm/arch-qca961x/ess/qca961x_edma.h>
#include "ipq40xx_qca8075.h"

extern int ipq40xx_mdio_write(int mii_id,
		int regnum, u16 value);
extern int ipq40xx_mdio_read(int mii_id,
		int regnum, ushort *data);

static u16 qca8075_phy_reg_write(u32 dev_id, u32 phy_id,
		u32 reg_id, u16 reg_val)
{
	ipq40xx_mdio_write(phy_id, reg_id, reg_val);
	return 0;
}

u16 qca8075_phy_reg_read(u32 dev_id, u32 phy_id, u32 reg_id)
{
	return ipq40xx_mdio_read(phy_id, reg_id, NULL);
}

/*
 * phy4 prfer medium
 * get phy4 prefer medum, fiber or copper;
 */
static qca8075_phy_medium_t __phy_prefer_medium_get(u32 dev_id,
                                                   u32 phy_id)
{
	u16 phy_medium;
	phy_medium =
		qca8075_phy_reg_read(dev_id, phy_id,
			QCA8075_PHY_CHIP_CONFIG);

	return ((phy_medium & QCA8075_PHY4_PREFER_FIBER) ?
		QCA8075_PHY_MEDIUM_FIBER : QCA8075_PHY_MEDIUM_COPPER);
}

/*
 *  phy4 activer medium
 *  get phy4 current active medium, fiber or copper;
 */
static qca8075_phy_medium_t __phy_active_medium_get(u32 dev_id,
                                                   u32 phy_id)
{
	u16 phy_data = 0;

	phy_data = qca8075_phy_reg_read(dev_id,
		phy_id, QCA8075_PHY_SGMII_STATUS);

	if ((phy_data & QCA8075_PHY4_AUTO_COPPER_SELECT)) {
		return QCA8075_PHY_MEDIUM_COPPER;
	} else if ((phy_data & QCA8075_PHY4_AUTO_BX1000_SELECT)) {
		return QCA8075_PHY_MEDIUM_FIBER; /*PHY_MEDIUM_FIBER_BX1000 */
	} else if ((phy_data & QCA8075_PHY4_AUTO_FX100_SELECT)) {
		return QCA8075_PHY_MEDIUM_FIBER; /*PHY_MEDIUM_FIBER_FX100 */
	}
	/* link down */
	return __phy_prefer_medium_get(dev_id, phy_id);
}

/*
 *  phy4 copper page or fiber page select
 *  set phy4 copper or fiber page
 */

static u8  __phy_reg_pages_sel(u32 dev_id, u32 phy_id,
		qca8075_phy_reg_pages_t phy_reg_pages)
{
	u16 reg_pages;
	reg_pages = qca8075_phy_reg_read(dev_id,
			phy_id, QCA8075_PHY_CHIP_CONFIG);

	if (phy_reg_pages == QCA8075_PHY_COPPER_PAGES) {
		reg_pages |= 0x8000;
	} else if (phy_reg_pages == QCA8075_PHY_SGBX_PAGES) {
		reg_pages &= ~0x8000;
	} else
		return -EINVAL;

	qca8075_phy_reg_write(dev_id, phy_id,
		QCA8075_PHY_CHIP_CONFIG, reg_pages);
	return 0;
}

/*
 *  phy4 reg pages selection by active medium
 *  phy4 reg pages selection
 */
static  u32 __phy_reg_pages_sel_by_active_medium(u32 dev_id,
						u32 phy_id)
{
	qca8075_phy_medium_t phy_medium;
	qca8075_phy_reg_pages_t reg_pages;

	phy_medium = __phy_active_medium_get(dev_id, phy_id);
	if (phy_medium == QCA8075_PHY_MEDIUM_FIBER) {
		reg_pages = QCA8075_PHY_SGBX_PAGES;
	} else if (phy_medium == QCA8075_PHY_MEDIUM_COPPER) {
		reg_pages = QCA8075_PHY_COPPER_PAGES;
	} else {
		return -1;
	}

	return __phy_reg_pages_sel(dev_id, phy_id, reg_pages);
}



static u8 qca8075_phy_speed_duplex_resolved(u32 dev_id, u32 phy_id)
{
	u16 phy_data;
	u16 ii = 200; /* Wait for 2s */

	if (phy_id == COMBO_PHY_ID)
		__phy_reg_pages_sel_by_active_medium(dev_id, phy_id);

	do {
		phy_data =
		    qca8075_phy_reg_read(dev_id, phy_id,
				QCA8075_PHY_SPEC_STATUS);
		mdelay(10);
	} while ((!QCA8075_SPEED_DUPLEX_RESOVLED(phy_data)) && --ii);

	if (ii == 0)
		return 1;

	return 0;
}

u8 qca8075_phy_get_link_status(u32 dev_id, u32 phy_id)
{
	u16 phy_data;
	if (phy_id == COMBO_PHY_ID)
		__phy_reg_pages_sel_by_active_medium(dev_id, phy_id);
	phy_data = qca8075_phy_reg_read(dev_id,
			phy_id, QCA8075_PHY_SPEC_STATUS);
	if (phy_data & QCA8075_STATUS_LINK_PASS)
		return 0;

	return 1;
}

static u32 qca8075_autoneg_done(u32 dev_id, u32 phy_id)
{
	u16 phy_data;
	u16 ii = 200; /* Wait for 2s */

	if (phy_id == COMBO_PHY_ID)
		__phy_reg_pages_sel_by_active_medium(dev_id, phy_id);

	do {
		phy_data =
			qca8075_phy_reg_read(dev_id,
				phy_id, QCA8075_PHY_STATUS);
		mdelay(10);
	} while ((!QCA8075_AUTONEG_DONE(phy_data)) && --ii);

	if (ii == 0)
		return 1;

	return 0;
}

static u32 qca8075_phy_reset(u32 dev_id, u32 phy_id)
{
	u16 phy_data;

	if (phy_id == COMBO_PHY_ID)
		__phy_reg_pages_sel_by_active_medium(dev_id, phy_id);

	phy_data = qca8075_phy_reg_read(dev_id,
			phy_id, QCA8075_PHY_CONTROL);
	qca8075_phy_reg_write(dev_id, phy_id, QCA8075_PHY_CONTROL,
			     phy_data | QCA8075_CTRL_SOFTWARE_RESET);

	return 0;
}

u32 qca8075_phy_get_duplex(u32 dev_id, u32 phy_id,
                      fal_port_duplex_t * duplex)
{
	u16 phy_data;

	if (phy_id == COMBO_PHY_ID) {
		__phy_reg_pages_sel_by_active_medium(dev_id, phy_id);
	}

	phy_data = qca8075_phy_reg_read(dev_id, phy_id,
				QCA8075_PHY_SPEC_STATUS);

	/*
	 * Read duplex
	 */
	if (phy_data & QCA8075_STATUS_FULL_DUPLEX)
		*duplex = FAL_FULL_DUPLEX;
	else
		*duplex = FAL_HALF_DUPLEX;

	return 0;
}

u32 qca8075_phy_get_speed(u32 dev_id, u32 phy_id,
                     fal_port_speed_t * speed)
{
	u16 phy_data;

	if (phy_id == COMBO_PHY_ID) {
		__phy_reg_pages_sel_by_active_medium(dev_id, phy_id);
	}
	phy_data = qca8075_phy_reg_read(dev_id,
			phy_id, QCA8075_PHY_SPEC_STATUS);

	switch (phy_data & QCA8075_STATUS_SPEED_MASK) {
	case QCA8075_STATUS_SPEED_1000MBS:
		*speed = FAL_SPEED_1000;
		break;
	case QCA8075_STATUS_SPEED_100MBS:
		*speed = FAL_SPEED_100;
		break;
	case QCA8075_STATUS_SPEED_10MBS:
		*speed = FAL_SPEED_10;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static u32 qca8075_phy_mmd_write(u32 dev_id, u32 phy_id,
                     u16 mmd_num, u16 reg_id, u16 reg_val)
{
	qca8075_phy_reg_write(dev_id, phy_id,
			QCA8075_MMD_CTRL_REG, mmd_num);
	qca8075_phy_reg_write(dev_id, phy_id,
			QCA8075_MMD_DATA_REG, reg_id);
	qca8075_phy_reg_write(dev_id, phy_id,
			QCA8075_MMD_CTRL_REG,
			0x4000 | mmd_num);
	qca8075_phy_reg_write(dev_id, phy_id,
		QCA8075_MMD_DATA_REG, reg_val);

	return 0;
}


static u16 qca8075_phy_mmd_read(u32 dev_id, u32 phy_id,
		u16 mmd_num, u16 reg_id)
{
	qca8075_phy_reg_write(dev_id, phy_id,
			QCA8075_MMD_CTRL_REG, mmd_num);
	qca8075_phy_reg_write(dev_id, phy_id,
			QCA8075_MMD_DATA_REG, reg_id);
	qca8075_phy_reg_write(dev_id, phy_id,
			QCA8075_MMD_CTRL_REG,
			0x4000 | mmd_num);
	return qca8075_phy_reg_read(dev_id, phy_id,
			QCA8075_MMD_DATA_REG);
}

int ipq40xx_qca8075_phy_init(qca961x_edma_board_cfg_t *cfg)
{
	u16 phy_data;

	phy_data = qca8075_phy_reg_read(0x0, 0x0, QCA8075_PHY_ID1);
	printf ("PHY ID1: 0x%x\n", phy_data);
	phy_data = qca8075_phy_reg_read(0x0, 0x0, QCA8075_PHY_ID2);
	printf ("PHY ID2: 0x%x\n", phy_data);

	phy_data = qca8075_phy_mmd_read(0, PSGMII_ID,
		QCA8075_PHY_MMD1_NUM, QCA8075_PSGMII_FIFI_CTRL);
	phy_data &= 0xbfff;
	qca8075_phy_mmd_write(0, PSGMII_ID, QCA8075_PHY_MMD1_NUM,
			QCA8075_PSGMII_FIFI_CTRL, phy_data);
	return 0;
}
