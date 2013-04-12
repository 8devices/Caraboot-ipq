
/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */

#ifndef _IPQCDP_H
#define _IPQCDP_H

#define CONFIG_RUMI
#define CONFIG_SYS_NO_FLASH
#define CONFIG_IPQ806X_UART
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_FPGA		        /* FPGA configuration support */
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_NFS		        /* NFS support */
#undef CONFIG_CMD_NET		        /* network support */
#undef CONFIG_SYS_MAX_FLASH_SECT
#define CONFIG_NR_DRAM_BANKS            1
#define CONFIG_SKIP_LOWLEVEL_INIT

/* Environment */
#define CONFIG_MSM_PCOMM
#define CONFIG_ARCH_CPU_INIT

#define CONFIG_ENV_SIZE                 (256 << 10) /* 256 KB */
#define CONFIG_SYS_MALLOC_LEN           (CONFIG_ENV_SIZE + (256 << 10))

/*
 * Size of malloc() pool
 */

/*
 * select serial console configuration
 */
#define CONFIG_CONS_INDEX               1

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_BAUDRATE                 115200
#define CONFIG_SYS_BAUDRATE_TABLE       {4800, 9600, 19200, 38400, 57600,\
								115200}

#define V_PROMPT                        "(IPQ) # "
#define CONFIG_SYS_PROMPT               V_PROMPT
#define CONFIG_SYS_CBSIZE               (256 * 2) /* Console I/O Buffer Size */

#define CONFIG_SYS_INIT_SP_ADDR         CONFIG_SYS_SDRAM_BASE + CONFIG_SYS_SDRAM_SIZE - GENERATED_GBL_DATA_SIZE
#define CONFIG_SYS_MAXARGS              16
#define CONFIG_SYS_LOAD_ADDR            CONFIG_SYS_TEXT_BASE + 0x100000
#define CONFIG_SYS_PBSIZE               (CONFIG_SYS_CBSIZE + \
						sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_TEXT_BASE            0x40000000
#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define CONFIG_SYS_SDRAM_SIZE           0x10000000
#define CONFIG_MAX_RAM_BANK_SIZE        CONFIG_SYS_SDRAM_SIZE

#ifndef __ASSEMBLY__
#include <compiler.h>
extern loff_t board_env_offset;
extern uint32_t flash_index;
extern uint32_t flash_chip_select;
extern uint32_t flash_block_size;

/*
 * XXX XXX Please do not instantiate this structure. XXX XXX
 * This is just a convenience to avoid
 *      - adding #defines for every new reservation
 *      - updating the multiple associated defines like smem base,
 *        kernel start etc...
 *      - re-calculation of the defines if the order changes or
 *        some reservations are deleted
 * For new reservations just adding a member to the structure should
 * suffice.
 * Ensure that the size of this structure matches with the definition
 * of the following IPQ806x compile time definitions
 *      PHYS_OFFSET     (linux-sources/arch/arm/mach-msm/Kconfig)
 *      zreladdr        (linux-sources/arch/arm/mach-msm/Makefile.boot)
 */
typedef struct {
	uint8_t	nss[4 * 1024 * 1024];
	uint8_t	smem[2 * 1024 * 1024];
	uint8_t	pad[2 * 1024 * 1024];
} __attribute__ ((__packed__)) ipq_mem_reserve_t;

/* Convenience macros for the above convenience structure :-) */
#define IPQ_MEM_RESERVE_SIZE(x)		sizeof(((ipq_mem_reserve_t *)0)->x)
#define IPQ_MEM_RESERVE_BASE(x)		\
	(CONFIG_SYS_SDRAM_BASE + \
	 ((uint32_t)&(((ipq_mem_reserve_t *)0)->x)))
#define IPQ_RESERVE_SIZE		sizeof(ipq_mem_reserve_t)

#define CONFIG_IPQ_SMEM_BASE		IPQ_MEM_RESERVE_BASE(smem)
#define IPQ_KERNEL_START_ADDR	\
	(CONFIG_SYS_SDRAM_BASE + IPQ_RESERVE_SIZE)

#define IPQ_DRAM_KERNEL_SIZE	\
	(CONFIG_SYS_SDRAM_SIZE - IPQ_RESERVE_SIZE)

#define IPQ_BOOT_PARAMS_ADDR		(IPQ_KERNEL_START_ADDR + 0x100)
#endif /* __ASSEMBLY__ */

#define CONFIG_CMD_MEMORY
#define CONFIG_SYS_MEMTEST_START        CONFIG_SYS_SDRAM_BASE + 0x1300000
#define CONFIG_SYS_MEMTEST_END          CONFIG_SYS_MEMTEST_START + 0x100

#define CONFIG_CMDLINE_TAG	 1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS 1

#define CONFIG_MACH_TYPE                 MACH_TYPE_IPQ806X_CDP
#define CONFIG_CMD_IMI

#define CONFIG_CMD_SOURCE   1
#define CONFIG_INITRD_TAG   1

/*
 * SPI Flash Configs
 */

#define CONFIG_IPQ_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_SYS_HZ                   1000

#define CONFIG_SF_DEFAULT_BUS 0
#define CONFIG_SF_DEFAULT_CS 0
#define CONFIG_SF_DEFAULT_MODE SPI_MODE_0

/*
 * NAND Flash Configs
 */

#define CONFIG_IPQ_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_MEMORY
#define CONFIG_SYS_NAND_SELF_INIT
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_ONFI_DETECTION

/*
 * U-Boot Env Configs
 */

/*
 * FIXME: This should be selectable from the make command.
 * Define one of the following macros for environment in SPI Flash or
 * NAND Flash.
 *
 *   - CONFIG_ENV_IS_IN_SPI_FLASH
 *   - CONFIG_ENV_IS_IN_NAND
 */
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_CMD_SAVEENV
#define CONFIG_BOARD_LATE_INIT

#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)

#define CONFIG_ENV_SPI_CS               flash_chip_select
#define CONFIG_ENV_SPI_MODE             SPI_MODE_0
#define CONFIG_ENV_OFFSET               board_env_offset
#define CONFIG_ENV_SECT_SIZE            flash_block_size
#define CONFIG_ENV_SPI_BUS              flash_index

#elif defined(CONFIG_ENV_IS_IN_NAND)

#define CONFIG_ENV_OFFSET		board_env_offset

#else

#error "Unsupported env. type, should be NAND or SPI_FLASH."

#endif

/* NSS firmware loaded using bootm */
#define CONFIG_IPQ_FIRMWARE
#define CONFIG_BOOTCOMMAND  "bootipq"
#define CONFIG_BOOTARGS \
	"root=mtd:0:EFS2APPS rootfstype=jffs2 ro init=/init console=ttyHSL1,115200n8"

#define CONFIG_CMD_ECHO
#define CONFIG_BOOTDELAY	2

#endif /* _IPQCDP_H */
