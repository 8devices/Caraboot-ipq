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

#ifndef _IPQCDP_H
#define _IPQCDP_H

#ifndef DO_DEPS_ONLY
#include <generated/asm-offsets.h>
#endif

/*
 * Support for IPQ807X RUMI
 */
#define CONFIG_IPQ_RUMI

/*
 * Disabled for actual chip.
 * #define CONFIG_RUMI
 */

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_CACHELINE_SIZE   64

#define CONFIG_IPQ807X_UART
#define CONFIG_NR_DRAM_BANKS            1
#define CONFIG_SKIP_LOWLEVEL_INIT

#define CONFIG_SYS_BOOTM_LEN            (64 << 20)
#define HAVE_BLOCK_DEVICE
/*
 * Size of malloc() pool
 */

/*
 * select serial console configuration
 */
#define CONFIG_CONS_INDEX               1

/* allow to overwrite serial and ethaddr */
#define CONFIG_BAUDRATE                 115200
#define CONFIG_SYS_BAUDRATE_TABLE       {4800, 9600, 19200, 38400, 57600,\
								115200}

#define CONFIG_SYS_CBSIZE               (512 * 2) /* Console I/O Buffer Size */

/*

          svc_sp     --> --------------
          irq_sp     --> |            |
	  fiq_sp     --> |            |
	  bd         --> |            |
          gd         --> |            |
          pgt        --> |            |
          malloc     --> |            |
          text_base  --> |------------|
*/

#define CONFIG_IPQ807x_I2C	1
#ifdef CONFIG_IPQ807x_I2C
#define CONFIG_SYS_I2C_QUP
#define CONFIG_CMD_I2C
#define CONFIG_DM_I2C
#endif

#define CONFIG_SYS_INIT_SP_ADDR 	(CONFIG_SYS_TEXT_BASE -\
			CONFIG_SYS_MALLOC_LEN - CONFIG_ENV_SIZE -\
			GENERATED_BD_INFO_SIZE)

#define CONFIG_SYS_MAXARGS              16
#define CONFIG_SYS_PBSIZE               (CONFIG_SYS_CBSIZE + \
						sizeof(CONFIG_SYS_PROMPT) + 16)

#define TLMM_BASE			0x01000000
#define GPIO_CONFIG_ADDR(x)		(TLMM_BASE + (x)*0x1000)
#define GPIO_IN_OUT_ADDR(x)		(TLMM_BASE + 0x4 + (x)*0x1000)

#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define CONFIG_SYS_TEXT_BASE            0x4AD00000
#define CONFIG_SYS_SDRAM_SIZE           0x10000000
#define CONFIG_MAX_RAM_BANK_SIZE        CONFIG_SYS_SDRAM_SIZE
#define CONFIG_SYS_LOAD_ADDR            (CONFIG_SYS_SDRAM_BASE + (64 << 20))

#define QCA_KERNEL_START_ADDR		CONFIG_SYS_SDRAM_BASE
#define QCA_DRAM_KERNEL_SIZE		CONFIG_SYS_SDRAM_SIZE
#define QCA_BOOT_PARAMS_ADDR		(QCA_KERNEL_START_ADDR + 0x100)

#define CONFIG_OF_COMBINE		1

#define CONFIG_QCA_SMEM_BASE		0x4AB00000

#ifndef __ASSEMBLY__
#include <compiler.h>
extern loff_t board_env_offset;
#endif

#define CONFIG_IPQ807X_ENV		1
#define CONFIG_ENV_OFFSET		board_env_offset
#define CONFIG_ENV_SIZE			0x2000
#define CONFIG_ENV_SIZE_MAX		(256 << 10) /* 256 KB */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE_MAX + (256 << 10))

#define CONFIG_ENV_IS_IN_NAND		1

/*
* SPI Flash Configs
*/
#define CONFIG_QCA_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_GIGADEVICE
#define CONFIG_SF_DEFAULT_BUS	0
#define CONFIG_SF_DEFAULT_CS	0
#define CONFIG_SF_DEFAULT_MODE	SPI_MODE_0
#define CONFIG_SF_DEFAULT_SPEED	(48 * 1000 * 1000)
#define CONFIG_SPI_FLASH_BAR	1

#define CONFIG_EFI_PARTITION
#define CONFIG_QCOM_BAM			1
/*
 * NAND Flash Configs
 */

/* CONFIG_QPIC_NAND: QPIC NAND in BAM mode
 * CONFIG_IPQ_NAND: QPIC NAND in FIFO/block mode.
 * BAM is enabled by default.
 */
#define CONFIG_QPIC_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_YAFFS
#define CONFIG_SYS_NAND_SELF_INIT
#define CONFIG_SYS_NAND_ONFI_DETECTION

/*
 * Expose SPI driver as a pseudo NAND driver to make use
 * of U-Boot's MTD framework.
 */
#define CONFIG_SYS_MAX_NAND_DEVICE	CONFIG_IPQ_MAX_NAND_DEVICE

#define CONFIG_IPQ_MAX_NAND_DEVICE	1

#define CONFIG_IPQ_NAND_NAND_INFO_IDX	0
#define CONFIG_QPIC_NAND_NAND_INFO_IDX	0
#define CONFIG_IPQ_SPI_NOR_INFO_IDX	2

#define CONFIG_SCM_TZ64	1
/*
 * U-Boot Env Configs
 */
#define CONFIG_OF_LIBFDT	1
#define CONFIG_SYS_HUSH_PARSER

/* NSS firmware loaded using bootm */
#define CONFIG_BOOTCOMMAND  "bootipq"
#define CONFIG_BOOTARGS "console=ttyMSM0,115200n8"
#define QCA_ROOT_FS_PART_NAME "rootfs"

#define CONFIG_BOOTDELAY	2

#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS

#define CONFIG_CMD_UBI
#define CONFIG_RBTREE

#define CONFIG_CMD_BOOTZ

#define CONFIG_OF_BOARD_SETUP

#ifdef CONFIG_OF_BOARD_SETUP
#define DLOAD_DISABLE		0x1
#define RESERVE_ADDRESS_START	0x4AB00000 /*TZAPPS, SMEM and TZ Regions */
#define RESERVE_ADDRESS_SIZE	0x5500000

/*
 * Below Configs need to be updated after enabling reset_crashdump
 * Included now to avoid build failure
 */
#define SET_MAGIC				0x1
#define CLEAR_MAGIC				0x0
#define SCM_CMD_TZ_CONFIG_HW_FOR_RAM_DUMP_ID	0x9
#define SCM_CMD_TZ_FORCE_DLOAD_ID		0x10
#define BOOT_VERSION				0
#define TZ_VERSION				1
#endif

#define CONFIG_FDT_FIXUP_PARTITIONS

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
#define CONFIG_SYS_CACHELINE_SIZE  64
/*#define CONFIG_SYS_DCACHE_OFF*/

/* Enabling this flag will report any L2 errors.
 * By default we are disabling it */
/*#define CONFIG_IPQ_REPORT_L2ERR*/

#endif /* _IPQCDP_H */

