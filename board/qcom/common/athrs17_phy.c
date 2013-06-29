/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Manage the atheros ethernet PHY.
 *
 * All definitions in this file are operating system independent!
 */

#include "ipq806x_phy.h"
#include "athrs17_phy.h"
#include "../../../drivers/net/nss/synopGMAC_plat.h"

/******************************************************************************
 * FUNCTION DESCRIPTION: Read switch internal register.
 *                       Switch internal register is accessed through the
 *                       MDIO interface. MDIO access is only 16 bits wide so
 *                       it needs the two time access to complete the internal
 *                       register access.
 * INPUT               : register address
 * OUTPUT              : Register value
 *
 *****************************************************************************/
static uint32_t
athrs17_reg_read(uint32_t reg_addr)
{
	uint32_t reg_word_addr;
	uint32_t phy_addr, tmp_val, reg_val;
	uint16_t phy_val;
	uint8_t phy_reg;

	/* change reg_addr to 16-bit word address, 32-bit aligned */
	reg_word_addr = (reg_addr & 0xfffffffc) >> 1;

	/* configure register high address */
	phy_addr = 0x18;
	phy_reg = 0x0;
	phy_val = (uint16_t) ((reg_word_addr >> 8) & 0x1ff);  /* bit16-8 of reg address */
	phy_reg_write(0, phy_addr, phy_reg, phy_val);

	/*
	 * For some registers such as MIBs, since it is read/clear, we should
	 * read the lower 16-bit register then the higher one
	 */

	/* read register in lower address */
	phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
	phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
	reg_val = (uint32_t) phy_reg_read(0, phy_addr, phy_reg);

	/* read register in higher address */
	reg_word_addr++;
	phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
	phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
	tmp_val = (uint32_t) phy_reg_read(0, phy_addr, phy_reg);
	reg_val |= (tmp_val << 16);

	return reg_val;
}

/******************************************************************************
 * FUNCTION DESCRIPTION: Write switch internal register.
 *                       Switch internal register is accessed through the
 *                       MDIO interface. MDIO access is only 16 bits wide so
 *                       it needs the two time access to complete the internal
 *                       register access.
 * INPUT               : register address, value to be written
 * OUTPUT              : NONE
 *
 *****************************************************************************/
static void
athrs17_reg_write(uint32_t reg_addr, uint32_t reg_val)
{
	uint32_t reg_word_addr;
	uint32_t phy_addr;
	uint16_t phy_val;
	uint8_t phy_reg;

	/* change reg_addr to 16-bit word address, 32-bit aligned */
	reg_word_addr = (reg_addr & 0xfffffffc) >> 1;

	/* configure register high address */
	phy_addr = 0x18;
	phy_reg = 0x0;
	phy_val = (uint16_t) ((reg_word_addr >> 8) & 0x1ff);  /* bit16-8 of reg address */
	phy_reg_write(0, phy_addr, phy_reg, phy_val);

	/*
	 * For some registers such as ARL and VLAN, since they include BUSY bit
	 * in lower address, we should write the higher 16-bit register then the
	 * lower one
	 */

	/* read register in higher address */
	reg_word_addr++;
	phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
	phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
	phy_val = (uint16_t) ((reg_val >> 16) & 0xffff);
	phy_reg_write(0, phy_addr, phy_reg, phy_val);

	/* write register in lower address */
	reg_word_addr--;
	phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
	phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
	phy_val = (uint16_t) (reg_val & 0xffff);
	phy_reg_write(0, phy_addr, phy_reg, phy_val);
}

/*********************************************************************
 * FUNCTION DESCRIPTION: Configure S17 register
 * INPUT : NONE
 * OUTPUT: NONE
 *********************************************************************/
void athrs17_reg_init()
{
	static uint8_t athr17_init_flag;

	if (athr17_init_flag)
		return;

	athrs17_reg_write(S17_P0PAD_MODE_REG, (S17_MAC0_RGMII_EN | \
		S17_MAC0_RGMII_TXCLK_DELAY | S17_MAC0_RGMII_RXCLK_DELAY | \
		(0x1 << S17_MAC0_RGMII_TXCLK_SHIFT) | \
		(0x2 << S17_MAC0_RGMII_RXCLK_SHIFT)));

	athrs17_reg_write(S17_GLOFW_CTRL1_REG, (S17_IGMP_JOIN_LEAVE_DPALL | \
		S17_BROAD_DPALL | S17_MULTI_FLOOD_DPALL | \
		S17_UNI_FLOOD_DPALL));

	athrs17_reg_write(S17_P0STATUS_REG, (S17_SPEED_100M | S17_TXMAC_EN | \
		S17_RXMAC_EN | S17_TX_FLOW_EN | S17_RX_FLOW_EN | \
		S17_DUPLEX_FULL));

	athr17_init_flag = 1;
	TR("%s: complete\n", __func__);
}


