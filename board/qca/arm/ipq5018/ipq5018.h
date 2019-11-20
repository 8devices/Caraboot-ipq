/*
* Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
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

#ifndef _IPQ5018_CDP_H_
#define _IPQ5018_CDP_H_

#include <configs/ipq5018.h>
#include <asm/u-boot.h>
#include <asm/arch-qca-common/qca_common.h>

#define MSM_SDC1_BASE			0x7800000
#define MSM_SDC1_SDHCI_BASE		0x7804000

/*
 * GCC-SDCC Registers
 */

#define GCC_SDCC1_BCR			0x01842000
#define GCC_SDCC1_APPS_CMD_RCGR		0x01842004
#define GCC_SDCC1_APPS_CFG_RCGR		0x01842008
#define GCC_SDCC1_APPS_M		0x0184200C
#define GCC_SDCC1_APPS_N		0x01842010
#define GCC_SDCC1_APPS_D		0x01842014
#define GCC_SDCC1_APPS_CBCR		0x01842018
#define GCC_SDCC1_AHB_CBCR		0x0184201C
#define GCC_SDCC1_MISC			0x01842020

/* UART 1 */
#define GCC_BLSP1_UART1_BCR               0x01802038
#define GCC_BLSP1_UART1_APPS_CBCR         0x0180203C
#define GCC_BLSP1_UART1_APPS_CMD_RCGR     0x01802044
#define GCC_BLSP1_UART1_APPS_CFG_RCGR     0x01802048
#define GCC_BLSP1_UART1_APPS_M            0x0180204C
#define GCC_BLSP1_UART1_APPS_N            0x01802050
#define GCC_BLSP1_UART1_APPS_D            0x01802054

/* UART 2 */
#define GCC_BLSP1_UART2_BCR               0x01803028
#define GCC_BLSP1_UART2_APPS_CBCR         0x0180302C
#define GCC_BLSP1_UART2_APPS_CMD_RCGR     0x01803034
#define GCC_BLSP1_UART2_APPS_CFG_RCGR     0x01803038
#define GCC_BLSP1_UART2_APPS_M            0x0180303C
#define GCC_BLSP1_UART2_APPS_N            0x01803040
#define GCC_BLSP1_UART2_APPS_D            0x01803044


#define GCC_SDCC1_BCR                     0x01842000

#define GCC_UART_CFG_RCGR_MODE_MASK       0x3000
#define GCC_UART_CFG_RCGR_SRCSEL_MASK     0x0700
#define GCC_UART_CFG_RCGR_SRCDIV_MASK     0x001F

#define GCC_UART_CFG_RCGR_MODE_SHIFT      12
#define GCC_UART_CFG_RCGR_SRCSEL_SHIFT    8
#define GCC_UART_CFG_RCGR_SRCDIV_SHIFT    0

#define UART1_RCGR_SRC_SEL                0x1
#define UART1_RCGR_SRC_DIV                0x0
#define UART1_RCGR_MODE                   0x2
#define UART1_CMD_RCGR_UPDATE             0x1
#define UART1_CBCR_CLK_ENABLE             0x1

#define NOT_2D(two_d)                     (~two_d)
#define NOT_N_MINUS_M(n,m)                (~(n - m))
#define CLOCK_UPDATE_TIMEOUT_US           1000

#define KERNEL_AUTH_CMD                   0x13
#define SCM_CMD_SEC_AUTH		       0x1F

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
__weak void qgic_init(void) {}
__weak void handle_noc_err(void) {}
__weak void board_pcie_clock_init(int id) {}

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
void reset_crashdump(void);
void reset_board(void);

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

#endif /* _IPQ5018_CDP_H_ */
