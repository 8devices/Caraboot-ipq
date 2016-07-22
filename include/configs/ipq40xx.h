/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#ifndef _IPQ40XX_H
#define _IPQ40XX_H

/*
 * Beat the system! tools/scripts/make-asm-offsets uses
 * the following hard-coded define for both u-boot's
 * ASM offsets and platform specific ASM offsets :(
 */

#ifndef DO_DEPS_ONLY
#include <generated/asm-offsets.h>
#endif

#define CONFIG_IPQ40XX
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_CACHELINE_SIZE	64
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_SYS_HZ			1000
#define CONFIG_IPQ40XX_UART
#define CONFIG_CONS_INDEX		1

#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600,\
								115200}
#define V_PROMPT			"(IPQ40xx) # "
#define CONFIG_SYS_CBSIZE		(256 * 2) /* Console I/O Buffer Size */
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
						sizeof(CONFIG_SYS_PROMPT) + 16)

#define HAVE_BLOCK_DEVICE
#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_SYS_TEXT_BASE		0x87300000
#define CONFIG_SYS_SDRAM_SIZE		0x10000000
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_TEXT_BASE + 0x100000 - GENERATED_GBL_DATA_SIZE)
#define CONFIG_MAX_RAM_BANK_SIZE	CONFIG_SYS_SDRAM_SIZE
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + (64 << 20))
#define CONFIG_DTB_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + (96 << 20))
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_OF_LIBFDT		1

#define CONFIG_IPQ_APPSBL_IMG_TYPE	0x5

#define CONFIG_IPQ40XX_ENV
#define CONFIG_ENV_IS_IN_NAND

#define CONFIG_OF_COMBINE 1

#ifndef __ASSEMBLY__
#include <compiler.h>
extern loff_t board_env_offset;
extern loff_t board_env_range;
extern loff_t board_env_size;

#if !defined(__ASSEMBLY__)
#define INIT_STACK_SIZE		1024
typedef struct {
	uint8_t uboot[1024 * 1024 - INIT_STACK_SIZE];	/* ~1MB */
	uint8_t init_stack[INIT_STACK_SIZE];
	uint8_t sbl[1024 * 1024];				/* 1 MB */
	uint8_t cnss_debug[6 * 1024 * 1024];			/* 6 MB */
	uint8_t tz_apps[3 * 1024 * 1024];			/* 3 MB */
	uint8_t smem[512 * 1024];				/* 512 KB */
	uint8_t tz[1536 * 1024];				/* 1.5 MB */
} __attribute__ ((__packed__)) qca_mem_reserve_t;

#define QCA_MEM_RESERVE_SIZE(x)		sizeof(((qca_mem_reserve_t *)0)->x)
#define QCA_MEM_RESERVE_BASE(x)		\
	(CONFIG_SYS_TEXT_BASE + \
	 ((uint32_t)&(((qca_mem_reserve_t *)0)->x)))
#endif/*__ASSEMBLY__*/

#define CONFIG_QCA_SMEM_BASE		0x87e00000
#define QCA_KERNEL_START_ADDR	\
	(CONFIG_SYS_SDRAM_BASE + sizeof(qca_mem_reserve_t))

#define QCA_DRAM_KERNEL_SIZE	\
	(CONFIG_SYS_SDRAM_SIZE - sizeof(qca_mem_reserve_t))

#define QCA_BOOT_PARAMS_ADDR	(QCA_KERNEL_START_ADDR + 0x100)
#endif

/* Environment */
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_ENV_OFFSET		board_env_offset
#define CONFIG_ENV_SIZE_MAX		(256 << 10) /* 256 KB */
#define CONFIG_ENV_RANGE		board_env_size
#define CONFIG_ENV_SIZE			(256 << 10) /* 256 KB */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE_MAX + (256 << 10))

#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE + 0x1300000
#define CONFIG_SYS_MEMTEST_END		CONFIG_SYS_MEMTEST_START + 0x100
#define CONFIG_CMD_SOURCE		1
#define CONFIG_INITRD_TAG		1
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_NULLDEV

/*
 * USB Support
 */

#define CONFIG_USB_XHCI_IPQ
#ifdef CONFIG_USB_XHCI_IPQ
#define CONFIG_USB_XHCI
#define CONFIG_USB_XHCI_DWC3
#define CONFIG_CMD_USB
#define CONFIG_DOS_PARTITION
#define CONFIG_USB_STORAGE
#define CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS      2
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2
#endif

/* L1 cache line size is 64 bytes, L2 cache line size is 128 bytes
 * Cache flush and invalidation based on L1 cache, so the cache line
 * size is configured to 64 */
#define CONFIG_SYS_CACHELINE_SIZE	64

#define CONFIG_QCOM_BAM                 1
/*
 * NAND Flash Configs104
 */

/* CONFIG_QPIC_NAND: QPIC NAND in BAM mode107
 * CONFIG_IPQ_NAND: QPIC NAND in FIFO/block mode.108
 * BAM is enabled by default.109
 */
#define CONFIG_QPIC_NAND
#define CONFIG_CMD_NAND
#define CONFIG_SYS_NAND_ONFI_DETECTION

#define CONFIG_SYS_MAX_NAND_DEVICE      (CONFIG_IPQ_MAX_NAND_DEVICE + \
					CONFIG_IPQ_MAX_SPI_DEVICE)

#define CONFIG_IPQ_MAX_SPI_DEVICE       2
#define CONFIG_IPQ_MAX_NAND_DEVICE      1

#define CONFIG_SYS_NAND_SELF_INIT
#define CONFIG_IPQ_NAND_NAND_INFO_IDX	0
#define CONFIG_QPIC_NAND_NAND_INFO_IDX	0
#define CONFIG_IPQ_SPI_NAND_INFO_IDX	1
#define CONFIG_IPQ_SPI_NOR_INFO_IDX	2

#define QCA_ROOT_FS_PART_NAME "rootfs"

/*
 * SPI Flash Configs
 */

#define CONFIG_IPQ40XX_SPI
#define CONFIG_QCA_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_GIGADEVICE

#define CONFIG_SF_DEFAULT_BUS 0
#define CONFIG_SF_DEFAULT_CS 0
#define CONFIG_SF_DEFAULT_MODE SPI_MODE_0
#define CONFIG_SPI_FLASH_BAR    1

#define CONFIG_IPQ40XX_EDMA     1
#define CONFIG_NET_RETRY_COUNT          5
#define CONFIG_SYS_RX_ETH_BUFFER        16
#define CONFIG_IPQ40XX_MDIO     1
#define CONFIG_QCA8075_PHY      1
#define CONFIG_QCA8033_PHY      1
#define CONFIG_MII
#define CONFIG_CMD_MII
#define CONFIG_IPADDR   192.168.1.11
#define CONFIG_IPQ_NO_MACS      2

#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS

#define CONFIG_CMD_UBI
#define CONFIG_RBTREE
#define CONFIG_CMD_BOOTZ
#define CONFIG_SYS_BOOTM_LEN   (64 << 20)
#define CONFIG_IPQ_FDT_HIGH     0x87000000
#define CONFIG_OF_BOARD_SETUP 1
#define CONFIG_FDT_FIXUP_PARTITIONS
#define CONFIG_BOOTCOMMAND "bootipq"

#define CONFIG_BOOTDELAY	2

#endif /* _IPQ40XX_H */
