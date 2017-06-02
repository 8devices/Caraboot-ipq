/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#ifndef _IPQ807X_H_
#define _IPQ807X_H_

#include <configs/ipq807x.h>
#include <asm/u-boot.h>
#include <asm/arch-qca-common/qca_common.h>

/*
 * GCC-SDCC Registers
 */
#define GCC_SDCC1_MISC		0x1842020
#define GCC_SDCC1_APPS_CBCR	0x1842018
#define GCC_SDCC1_APPS_CFG_RCGR	0x1842008
#define GCC_SDCC1_APPS_CMD_RCGR	0x1842004
#define GCC_SDCC1_APPS_M	0x184200C
#define GCC_SDCC1_APPS_N	0x1842010
#define GCC_SDCC1_APPS_D	0x1842014
#define GCC_BLSP1_UART1_APPS_CBCR       0x0180203c

#define GCC_PCIE0_AXI_M_CBCR	0x01875008
#define GCC_PCIE0_AXI_S_CBCR	0x0187500C
#define GCC_PCIE0_AHB_CBCR	0x01875010
#define GCC_PCIE0_AUX_CBCR	0x01875014
#define GCC_PCIE0_PIPE_CBCR	0x01875018
#define GCC_PCIE0_AUX_CMD_RCGR	0x01875020
#define GCC_PCIE0_AXI_CMD_RCGR	0x01875050
#define GCC_PCIE0_AXI_CFG_RCGR	0x01875058

#define GCC_PCIE1_AXI_M_CBCR	0x01876008
#define GCC_PCIE1_AXI_S_CBCR	0x0187600C
#define GCC_PCIE1_AHB_CBCR	0x01876010
#define GCC_PCIE1_AUX_CBCR	0x01876014
#define GCC_PCIE1_PIPE_CBCR	0x01876018
#define GCC_PCIE1_AUX_CMD_RCGR	0x01876020
#define GCC_PCIE1_AXI_CMD_RCGR	0x01876050
#define GCC_PCIE1_AXI_CFG_RCGR	0x01876058

#define KERNEL_AUTH_CMD		0x13

typedef enum {
	SMEM_SPINLOCK_ARRAY = 7,
	SMEM_AARM_PARTITION_TABLE = 9,
	SMEM_HW_SW_BUILD_ID = 137,
	SMEM_USABLE_RAM_PARTITION_TABLE = 402,
	SMEM_POWER_ON_STATUS_INFO = 403,
	SMEM_MACHID_INFO_LOCATION = 425,
	SMEM_IMAGE_VERSION_TABLE = 469,
	SMEM_BOOT_FLASH_TYPE = 498,
	SMEM_BOOT_FLASH_INDEX = 499,
	SMEM_BOOT_FLASH_CHIP_SELECT = 500,
	SMEM_BOOT_FLASH_BLOCK_SIZE = 501,
	SMEM_BOOT_FLASH_DENSITY = 502,
	SMEM_BOOT_DUALPARTINFO = 503,
	SMEM_PARTITION_TABLE_OFFSET = 504,
	SMEM_FIRST_VALID_TYPE = SMEM_SPINLOCK_ARRAY,
	SMEM_LAST_VALID_TYPE = SMEM_PARTITION_TABLE_OFFSET,
	SMEM_MAX_SIZE = SMEM_PARTITION_TABLE_OFFSET + 1,
} smem_mem_type_t;

/* Reserved-memory node names*/
extern const char *rsvd_node;
extern const char *del_node[];
extern const add_node_t add_node[];

void reset_crashdump(void);
void board_pci_init(int id);
int ipq_fdt_fixup_socinfo(void *blob);

#endif /* _IPQ807X_H_ */
