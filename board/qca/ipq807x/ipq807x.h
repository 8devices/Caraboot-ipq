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

#ifndef _IPQ807X_H_
#define _IPQ807X_H_

#include <configs/ipq807x.h>
#include <asm/u-boot.h>

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

typedef enum {
        SMEM_SPINLOCK_ARRAY = 7,
        SMEM_AARM_PARTITION_TABLE = 9,
        SMEM_HW_SW_BUILD_ID = 137,
        SMEM_USABLE_RAM_PARTITION_TABLE = 402,
        SMEM_POWER_ON_STATUS_INFO = 403,
        SMEM_IMAGE_VERSION_TABLE = 469,
        SMEM_BOOT_FLASH_TYPE = 478,
        SMEM_BOOT_FLASH_INDEX = 479,
        SMEM_BOOT_FLASH_CHIP_SELECT = 480,
        SMEM_BOOT_FLASH_BLOCK_SIZE = 481,
        SMEM_BOOT_FLASH_DENSITY = 482,
        SMEM_PARTITION_TABLE_OFFSET = 483,
        SMEM_BOOT_DUALPARTINFO = 484,
        SMEM_FIRST_VALID_TYPE = SMEM_SPINLOCK_ARRAY,
        SMEM_LAST_VALID_TYPE = SMEM_PARTITION_TABLE_OFFSET,
        SMEM_MAX_SIZE = SMEM_PARTITION_TABLE_OFFSET + 1,
} smem_mem_type_t;

#endif /* _IPQ807X_H_ */
