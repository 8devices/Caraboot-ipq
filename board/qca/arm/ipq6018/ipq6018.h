/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#ifndef _IPQ6018_CDP_H_
#define _IPQ6018_CDP_H_

#include <configs/ipq6018.h>
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
#define GCC_SDCC1_BCR		0x01842000

#define GCC_BLSP1_UART2_APPS_CFG_RCGR	0x01803038
#define GCC_BLSP1_UART2_APPS_M		0x0180303C
#define GCC_BLSP1_UART2_APPS_N		0x01803040
#define GCC_BLSP1_UART2_APPS_D		0x01803044
#define GCC_BLSP1_UART2_APPS_CMD_RCGR	0x01803034
#define GCC_BLSP1_UART2_APPS_CBCR	0x0180302C

#define GCC_UART_CFG_RCGR_MODE_MASK	0x3000
#define GCC_UART_CFG_RCGR_SRCSEL_MASK	0x0700
#define GCC_UART_CFG_RCGR_SRCDIV_MASK	0x001F

#define GCC_UART_CFG_RCGR_MODE_SHIFT	12
#define GCC_UART_CFG_RCGR_SRCSEL_SHIFT	8
#define GCC_UART_CFG_RCGR_SRCDIV_SHIFT	0

#define UART2_RCGR_SRC_SEL	0x1
#define UART2_RCGR_SRC_DIV	0x0
#define UART2_RCGR_MODE		0x2
#define UART2_CMD_RCGR_UPDATE	0x1
#define UART2_CBCR_CLK_ENABLE	0x1

#define NOT_2D(two_d)		(~two_d)
#define NOT_N_MINUS_M(n,m)	(~(n - m))
#define CLOCK_UPDATE_TIMEOUT_US	1000

#define CLOCK_UPDATE_TIMEOUT_US	1000
#define KERNEL_AUTH_CMD		0x13
struct smem_ram_ptn {
	char name[16];
	unsigned long long start;
	unsigned long long size;

	/* RAM Partition attribute: READ_ONLY, READWRITE etc.  */
	unsigned attr;

	/* RAM Partition category: EBI0, EBI1, IRAM, IMEM */
	unsigned category;

	/* RAM Partition domain: APPS, MODEM, APPS & MODEM (SHARED) etc. */
	unsigned domain;

	/* RAM Partition type: system, bootloader, appsboot, apps etc. */
	unsigned type;

	/* reserved for future expansion without changing version number */
	unsigned reserved2, reserved3, reserved4, reserved5;
} __attribute__ ((__packed__));

__weak void aquantia_phy_reset_init_done(void) {}
__weak void aquantia_phy_reset_init(void) {}

struct smem_ram_ptable {
#define _SMEM_RAM_PTABLE_MAGIC_1	0x9DA5E0A8
#define _SMEM_RAM_PTABLE_MAGIC_2	0xAF9EC4E2
	unsigned magic[2];
	unsigned version;
	unsigned reserved1;
	unsigned len;
	unsigned buf;
	struct smem_ram_ptn parts[32];
} __attribute__ ((__packed__));

int smem_ram_ptable_init(struct smem_ram_ptable *smem_ram_ptable);

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
	SMEM_SPI_FLASH_ADDR_LEN = 505,
	SMEM_FIRST_VALID_TYPE = SMEM_SPINLOCK_ARRAY,
	SMEM_LAST_VALID_TYPE = SMEM_SPI_FLASH_ADDR_LEN,
	SMEM_MAX_SIZE = SMEM_SPI_FLASH_ADDR_LEN + 1,
} smem_mem_type_t;

extern const char *rsvd_node;
extern const char *del_node[];
extern const add_node_t add_fdt_node[];

#define MSM_SDC1_BASE           0x7800000
#define MSM_SDC1_SDHCI_BASE     0x7804000

#endif /* _IPQ6018_CDP_H_ */
