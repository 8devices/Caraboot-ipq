/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#ifndef _IPQ806X_H_
#define _IPQ806X_H_

#include <configs/ipq806x.h>
#include <asm/u-boot.h>
#include <asm/arch-qcom-common/qca_common.h>

#define GSBI4_BASE 0x16300000

#define KERNEL_AUTH_CMD                 0x7

typedef enum {
	SMEM_SPINLOCK_ARRAY = 7,
	SMEM_AARM_PARTITION_TABLE = 9,
	SMEM_APPS_BOOT_MODE = 106,
	SMEM_HW_SW_BUILD_ID = 137,
	SMEM_USABLE_RAM_PARTITION_TABLE = 402,
	SMEM_POWER_ON_STATUS_INFO = 403,
	SMEM_RLOCK_AREA = 404,
	SMEM_BOOT_INFO_FOR_APPS = 418,
	SMEM_BOOT_FLASH_TYPE = 421,
	SMEM_BOOT_FLASH_INDEX = 422,
	SMEM_BOOT_FLASH_CHIP_SELECT = 423,
	SMEM_BOOT_FLASH_BLOCK_SIZE = 424,
	SMEM_MACHID_INFO_LOCATION = 425,
	SMEM_BOOT_DUALPARTINFO = 427,
	SMEM_PARTITION_TABLE_OFFSET = 428,
	SMEM_IMAGE_VERSION_TABLE = 469,
	SMEM_BOOT_FLASH_DENSITY = 482,
	SMEM_FIRST_VALID_TYPE = SMEM_SPINLOCK_ARRAY,
	SMEM_LAST_VALID_TYPE = SMEM_BOOT_FLASH_DENSITY,
	SMEM_MAX_SIZE = SMEM_PARTITION_TABLE_OFFSET + 1,
} smem_mem_type_t;

/* Reserved-memory node names*/
extern const char *rsvd_node;
extern const char *del_node[];
extern const add_node_t add_node[];
void reset_crashdump(void);
int ipq_fdt_fixup_socinfo(void *blob);
#endif /* _IPQ806X_H_ */
