/*
 * Copyright (c) 2012-2015 The Linux Foundation. All rights reserved.
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

#ifndef _IPQ806x_CDP_H
#define _IPQ806x_CDP_H

/*
 * Disabled for actual chip.
 * #define CONFIG_RUMI
 */
#if !defined(DO_DEPS_ONLY) || defined(DO_SOC_DEPS_ONLY)
/*
 * Beat the system! tools/scripts/make-asm-offsets uses
 * the following hard-coded define for both u-boot's
 * ASM offsets and platform specific ASM offsets :(
 */
#include <generated/generic-asm-offsets.h>
#ifdef __ASM_OFFSETS_H__
#undef __ASM_OFFSETS_H__
#endif
#if !defined(DO_SOC_DEPS_ONLY)
#include <generated/asm-offsets.h>
#endif
#endif /* !DO_DEPS_ONLY */

#define CONFIG_IPQ806X

#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_CACHELINE_SIZE   64
#define CONFIG_IPQ806X_ENV

#undef CONFIG_IPQ806X_USB
#ifdef CONFIG_IPQ806X_USB
#define CONFIG_USB_XHCI
#define CONFIG_CMD_USB
#define CONFIG_DOS_PARTITION
#define CONFIG_USB_STORAGE
#define CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS 2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif

#define CONFIG_IPQ806X_UART
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_FPGA		        /* FPGA configuration support */
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_NFS		        /* NFS support */
#define CONFIG_CMD_NET		        /* network support */
#define CONFIG_CMD_DHCP
#undef CONFIG_SYS_MAX_FLASH_SECT
#define CONFIG_NR_DRAM_BANKS            1
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_CMD_PING

#define CONFIG_IPQ_SNPS_GMAC
#define CONFIG_MII
#define CONFIG_CMD_MII
#define CONFIG_BITBANGMII
#define CONFIG_BITBANGMII_MULTI

#define CONFIG_IPQ_SWITCH_ATHRS17
#define CONFIG_IPQ_SWITCH_QCA8511

#define CONFIG_SYS_RX_ETH_BUFFER        8
#define CONFIG_IPQ_NO_MACS		4

#undef CONFIG_BOARD_EARLY_INIT_F

#undef CONFIG_HW_WATCHDOG

/* Environment */
#define CONFIG_MSM_PCOMM
#define CONFIG_ARCH_CPU_INIT

#define CONFIG_ENV_SIZE			0x10000 /* 64 KB */
#define CONFIG_ENV_SIZE_MAX             (256 << 10) /* 256 KB */
#define CONFIG_SYS_MALLOC_LEN           (CONFIG_ENV_SIZE_MAX + (256 << 10))
#define CONFIG_ENV_IS_NOWHERE		1

/*
 * select serial console configuration
 */
#define CONFIG_CONS_INDEX		1

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600,\
	115200}

#define V_PROMPT			"(IPQ) # "
#ifndef CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT		V_PROMPT
#endif
#define CONFIG_SYS_CBSIZE		(512 * 2) /* Console I/O Buffer Size */

#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_TEXT_BASE - CONFIG_SYS_MALLOC_LEN - CONFIG_ENV_SIZE - GENERATED_BD_INFO_SIZE)
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
		sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define CONFIG_SYS_TEXT_BASE		0x41200000
#define CONFIG_SYS_SDRAM_SIZE		0x10000000
#define CONFIG_MAX_RAM_BANK_SIZE	CONFIG_SYS_SDRAM_SIZE
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + (64 << 20))

#define QCA_KERNEL_START_ADDR		CONFIG_SYS_SDRAM_BASE
#define QCA_BOOT_PARAMS_ADDR		(QCA_KERNEL_START_ADDR + 0x100)
#define CONFIG_QCA_SMEM_BASE		CONFIG_SYS_SDRAM_BASE + 0x1000000

#define CONFIG_OF_COMBINE		1

/*
 * I2C Configs
 */
#undef CONFIG_IPQ806X_I2C

#ifdef CONFIG_IPQ806X_I2C
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_SPEED		0
#endif

/*
 * PCI Configs
 */
#undef CONFIG_IPQ806X_PCI

#ifdef CONFIG_IPQ806X_PCI
#define CONFIG_PCI
#define CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
#endif

/*
 * MMC Configs
 */
#undef CONFIG_IPQ_MMC

#ifdef CONFIG_IPQ_MMC
#define CONFIG_CMD_MMC
#define CONFIG_MMC
#define CONFIG_EFI_PARTITION
#define CONFIG_GENERIC_MMC
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0
#endif

#define QCA_ROOT_FS_PART_NAME		"rootfs"

#ifndef __ASSEMBLY__
#include <compiler.h>

#endif /* __ASSEMBLY__ */

#ifndef CONFIG_FIT
#define CONFIG_FIT
#endif
#define CONFIG_SYS_NULLDEV
#define CONFIG_CMD_XIMG

/*Support for Compressed DTB image*/
#ifdef CONFIG_FIT
#define CONFIG_DTB_COMPRESSION
#define CONFIG_DTB_LOAD_MAXLEN		0x100000
#endif

/*NAND Flash Configs*/
#define CONFIG_CMD_NAND
#define CONFIG_SYS_NAND_SELF_INIT

#define CONFIG_IPQ_MAX_SPI_DEVICE	1
#define CONFIG_IPQ_MAX_NAND_DEVICE	1

#define CONFIG_SYS_MAX_NAND_DEVICE	(CONFIG_IPQ_MAX_NAND_DEVICE + \
		CONFIG_IPQ_MAX_SPI_DEVICE)

#define CONFIG_IPQ_NAND_NAND_INFO_IDX	0
#define CONFIG_IPQ_SPI_NAND_INFO_IDX	1

#define CONFIG_NAND_FLASH_INFO_IDX	CONFIG_IPQ_NAND_NAND_INFO_IDX
#define CONFIG_SPI_FLASH_INFO_IDX	CONFIG_IPQ_SPI_NAND_INFO_IDX

#define CONFIG_FDT_FIXUP_PARTITIONS

#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS

/*for ubi*/
#define CONFIG_CMD_UBI
#define CONFIG_RBTREE

#define CONFIG_OF_LIBFDT		1
#define CONFIG_OF_BOARD_SETUP		1

#ifdef CONFIG_OF_BOARD_SETUP
#define DLOAD_DISABLE			1
#define BOOT_VERSION			0
#define TZ_VERSION			1
#endif

/* L1 cache line size is 64 bytes, L2 cache line size is 128 bytes
 * Cache flush and invalidation based on L1 cache, so the cache line
 * size is configured to 64 */
#define CONFIG_SYS_CACHELINE_SIZE  64
#define CONFIG_SYS_DCACHE_OFF

/* Enabling this flag will report any L2 errors.
 * By default we are disabling it */
/*#define CONFIG_IPQ_REPORT_L2ERR*/

/*
 * Location in IMEM which contains the physical address of
 * 4K page allocated from kernel for storing the crashdump data
 */
#define CONFIG_IPQ_KERNEL_CRASHDUMP_ADDRESS 0x2A03F658

/*
 * CRASH DUMP ENABLE
 */

#define CONFIG_QCA_APPSBL_DLOAD 1

#define TLMM_BASE_ADDR      0x00800000
#define GPIO_CONFIG_ADDR(x) (TLMM_BASE_ADDR + 0x1000 + (x)*0x10)
#define GPIO_IN_OUT_ADDR(x) (TLMM_BASE_ADDR + 0x1004 + (x)*0x10)

#endif /* _IPQ806x_CDP_H */

