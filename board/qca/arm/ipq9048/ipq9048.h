/*
 * Copyright (c) 2016-2020 The Linux Foundation. All rights reserved.
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

#ifndef _IPQ9048_CDP_H_
#define _IPQ9048_CDP_H_

#include <configs/ipq9048.h>
#include <asm/u-boot.h>
#include <asm/arch-qca-common/qca_common.h>

#define BLSP1_UART0_BASE	0x078AF000
#define UART_PORT_ID(reg)	((reg - BLSP1_UART0_BASE) / 0x1000)

#define CLOCK_UPDATE_TIMEOUT_US	1000

#define KERNEL_AUTH_CMD		0x13
#define SCM_CMD_SEC_AUTH	0x1F

#ifdef CONFIG_SMEM_VERSION_C
#define RAM_PART_NAME_LENGTH 16

#define SECONDARY_CORE_STACKSZ (8 * 1024)
#define CPU_POWER_DOWN (1 << 16)

#define ARM_PSCI_TZ_FN_BASE		0x84000000
#define ARM_PSCI_TZ_FN(n)		(ARM_PSCI_TZ_FN_BASE + (n))

#define ARM_PSCI_TZ_FN_CPU_OFF		ARM_PSCI_TZ_FN(2)
#define ARM_PSCI_TZ_FN_CPU_ON		ARM_PSCI_TZ_FN(3)
#define ARM_PSCI_TZ_FN_AFFINITY_INFO	ARM_PSCI_TZ_FN(4)

/*
 * GCC-QPIC Registers
 */
#define GCC_QPIC_IO_MACRO_CBCR		0x183200C
#define GCC_QPIC_CBCR_ADDR		0x1832014
#define GCC_QPIC_AHB_CBCR_ADDR		0x1832010
#define GCC_QPIC_SLEEP_CBCR		0x1832018
#define QPIC_CBCR_VAL			0x80004FF1
#define GCC_QPIC_IO_MACRO_CMD_RCGR	0x1832004
#define GCC_QPIC_IO_MACRO_CFG_RCGR	0x1832008
#define IO_MACRO_CLK_320_MHZ		320000000
#define IO_MACRO_CLK_266_MHZ		266000000
#define IO_MACRO_CLK_228_MHZ		228000000
#define IO_MACRO_CLK_200_MHZ		200000000
#define IO_MACRO_CLK_100_MHZ		100000000
#define IO_MACRO_CLK_24MHZ		24000000
#define QPIC_IO_MACRO_CLK       	0
#define QPIC_CORE_CLK           	1
#define XO_CLK_SRC			2
#define GPLL0_CLK_SRC			3
#define FB_CLK_BIT			(1 << 4)
#define UPDATE_EN			0x1

unsigned int __invoke_psci_fn_smc(unsigned int, unsigned int,
					 unsigned int, unsigned int);

/**
 * Number of RAM partition entries which are usable by APPS.
 */
#define RAM_NUM_PART_ENTRIES 32
struct ram_partition_entry
{
	char name[RAM_PART_NAME_LENGTH];  /**< Partition name, unused for now */
	u64 start_address;             /**< Partition start address in RAM */
	u64 length;                    /**< Partition length in RAM in Bytes */
	u32 partition_attribute;       /**< Partition attribute */
	u32 partition_category;        /**< Partition category */
	u32 partition_domain;          /**< Partition domain */
	u32 partition_type;            /**< Partition type */
	u32 num_partitions;            /**< Number of partitions on device */
	u32 hw_info;                   /**< hw information such as type and frequency */
	u8 highest_bank_bit;           /**< Highest bit corresponding to a bank */
	u8 reserve0;                   /**< Reserved for future use */
	u8 reserve1;                   /**< Reserved for future use */
	u8 reserve2;                   /**< Reserved for future use */
	u32 reserved5;                 /**< Reserved for future use */
	u64 available_length;          /**< Available Partition length in RAM in Bytes */
};

struct usable_ram_partition_table
{
	u32 magic1;          /**< Magic number to identify valid RAM partition table */
	u32 magic2;          /**< Magic number to identify valid RAM partition table */
	u32 version;         /**< Version number to track structure definition changes
	                             and maintain backward compatibilities */
	u32 reserved1;       /**< Reserved for future use */

	u32 num_partitions;  /**< Number of RAM partition table entries */

	u32 reserved2;       /** < Added for 8 bytes alignment of header */

	/** RAM partition table entries */
	struct ram_partition_entry ram_part_entry[RAM_NUM_PART_ENTRIES];
};
#endif

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
int smem_ram_ptable_init_v2(struct usable_ram_partition_table *usable_ram_partition_table);
void reset_crashdump(void);
void reset_board(void);
void qpic_set_clk_rate(unsigned int clk_rate, int blk_type,
		int req_clk_src_type);

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

__weak void qgic_init(void) {}
__weak void handle_noc_err(void) {}
extern const char *rsvd_node;
extern const char *del_node[];
extern const add_node_t add_fdt_node[];
int ipq_get_tz_version(char *version_name, int buf_size);
void ipq_fdt_fixup_socinfo(void *blob);
#endif /* _IPQ9048_CDP_H_ */
