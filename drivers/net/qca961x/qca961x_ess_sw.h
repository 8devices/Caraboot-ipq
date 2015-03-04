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

#ifndef _AR8327_PHY_H
#define _AR8327_PHY_H
#define QCA961X_NSS_BASE	0xC000000

#define S17_P0STATUS_REG	0x007c
#define S17_P1STATUS_REG	0x0080
#define S17_P2STATUS_REG	0x0084
#define S17_P3STATUS_REG	0x0088
#define S17_P4STATUS_REG	0x008c
#define S17_P5STATUS_REG	0x0090
#define S17_P6STATUS_REG	0x0094

#define S17_GLOFW_CTRL1_REG		0x0624
#define S17_GLOLEARN_LIMIT_REG		0x0628
#define S17_TOS_PRIMAP_REG0		0x0630
#define S17_TOS_PRIMAP_REG1		0x0634
#define S17_TOS_PRIMAP_REG2		0x0638
#define S17_TOS_PRIMAP_REG3		0x063c
#define S17_TOS_PRIMAP_REG4		0x0640
#define S17_TOS_PRIMAP_REG5		0x0644
#define S17_TOS_PRIMAP_REG6		0x0648
#define S17_TOS_PRIMAP_REG7		0x064c
#define S17_VLAN_PRIMAP_REG0		0x0650
#define S17_LOOP_CHECK_REG		0x0654
#define S17_P0LOOKUP_CTRL_REG		0x0660
#define S17_P0PRI_CTRL_REG		0x0664
#define S17_P0LEARN_LMT_REG		0x0668
#define S17_P1LOOKUP_CTRL_REG		0x066c
#define S17_P1PRI_CTRL_REG		0x0670
#define S17_P1LEARN_LMT_REG		0x0674
#define S17_P2LOOKUP_CTRL_REG		0x0678
#define S17_P2PRI_CTRL_REG		0x067c
#define S17_P2LEARN_LMT_REG		0x0680
#define S17_P3LOOKUP_CTRL_REG		0x0684
#define S17_P3PRI_CTRL_REG		0x0688
#define S17_P3LEARN_LMT_REG		0x068c
#define S17_P4LOOKUP_CTRL_REG		0x0690
#define S17_P4PRI_CTRL_REG		0x0694
#define S17_P4LEARN_LMT_REG		0x0698
#define S17_P5LOOKUP_CTRL_REG		0x069c
#endif
