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

#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_CACHELINE_SIZE   64

#define CONFIG_IPQ807X_UART
#define CONFIG_NR_DRAM_BANKS            1
#define CONFIG_SKIP_LOWLEVEL_INIT

#define CONFIG_SYS_BOOTM_LEN  0x1000000

#define CONFIG_ENV_SIZE 0x2000
#define CONFIG_ENV_SIZE_MAX             (256 << 10) /* 256 KB */
#define CONFIG_SYS_MALLOC_LEN           (CONFIG_ENV_SIZE_MAX + (256 << 10))

#define CONFIG_ENV_IS_NOWHERE 1
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

#define CONFIG_SYS_INIT_SP_ADDR 	(CONFIG_SYS_TEXT_BASE -\
			CONFIG_SYS_MALLOC_LEN - CONFIG_ENV_SIZE -\
			GENERATED_BD_INFO_SIZE)

#define CONFIG_SYS_MAXARGS              16
#define CONFIG_SYS_PBSIZE               (CONFIG_SYS_CBSIZE + \
						sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define CONFIG_SYS_TEXT_BASE            0x4AD00000
#define CONFIG_SYS_SDRAM_SIZE           0x10000000
#define CONFIG_MAX_RAM_BANK_SIZE        CONFIG_SYS_SDRAM_SIZE
#define CONFIG_SYS_LOAD_ADDR            (CONFIG_SYS_SDRAM_BASE + (64 << 20))

#define CONFIG_OF_COMBINE		1

#define CONFIG_DTB_LOAD_ADDR 0x4AE00000
#define CONFIG_EXTRA_ENV_SETTINGS "fdtcontroladdr=0x4AE00000\0"

#define CONFIG_QCA_SMEM_BASE		0x41000000
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
#define CONFIG_CMD_MEMORY
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

/*
 * U-Boot Env Configs
 */
#define CONFIG_OF_LIBFDT	1

/* NSS firmware loaded using bootm */
#define CONFIG_BOOTCOMMAND  "bootm"
#define CONFIG_BOOTARGS "console=ttyMSM0,115200n8"

#define CONFIG_BOOTDELAY	2


#define CONFIG_CMD_BOOTZ

/* L1 cache line size is 64 bytes, L2 cache line size is 128 bytes
 * Cache flush and invalidation based on L1 cache, so the cache line
 * size is configured to 64 */
#define CONFIG_SYS_CACHELINE_SIZE  64
/*#define CONFIG_SYS_DCACHE_OFF*/

/* Enabling this flag will report any L2 errors.
 * By default we are disabling it */
/*#define CONFIG_IPQ_REPORT_L2ERR*/

#endif /* _IPQCDP_H */

