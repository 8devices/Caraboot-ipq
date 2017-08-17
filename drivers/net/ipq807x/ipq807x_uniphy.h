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
#define PPE_UNIPHY_INSTANCE0			0
#define PPE_UNIPHY_INSTANCE1			1
#define PPE_UNIPHY_INSTANCE2			2

enum port_wrapper_cfg {
	PORT_WRAPPER_PSGMII = 0,
	PORT_WRAPPER_SGMII0_RGMII4,
	PORT_WRAPPER_USXGMII,
};

#define GCC_UNIPHY0_MISC			0x01856004
#define GCC_UNIPHY_REG_INC 			0x100
#define GCC_UNIPHY_USXGMII_XPCS_RESET 		0x4
#define GCC_UNIPHY_USXGMII_XPCS_RELEASE_RESET	0x0

#define GCC_UNIPHY_PSGMII_SOFT_RESET 		0x3ff2
#define GCC_UNIPHY_USXGMII_SOFT_RESET 		0x36

#define PPE_UNIPHY_BASE				0X07A00000
#define PPE_UNIPHY_REG_INC 			0x10000
#define PPE_UNIPHY_MODE_CONTROL			0x46C

#define UNIPHY_MISC2_REG_OFFSET 		0x218
#define UNIPHY_MISC2_REG_SGMII_MODE 		0x30

#define UNIPHY_PLL_RESET_REG_OFFSET 		0x780
#define UNIPHY_PLL_RESET_REG_VALUE 		0x02bf
#define UNIPHY_PLL_RESET_REG_DEFAULT_VALUE 	0x02ff

void ppe_uniphy_mode_set(uint32_t uniphy_index, uint32_t mode);
