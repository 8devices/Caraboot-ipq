
/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */

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

#define CONFIG_ENV_SIZE                 (128 << 10) /* 128 KB */
#define CONFIG_SYS_MALLOC_LEN           (CONFIG_ENV_SIZE + (256 << 10))

/*
+ * Size of malloc() pool
+ */

/*
+ * select serial console configuration
+ */
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

#define CONFIG_SYS_TEXT_BASE            0x80000000 /*0x8FFDD000*/ /*0x8FFDC000*/ //0x80000000  /* Uboot base start */
#define CONFIG_SYS_SDRAM_BASE           0x80000000 /* SDRAM Base */
#define CONFIG_SYS_SDRAM_SIZE           0x10000000
#define CONFIG_MAX_RAM_BANK_SIZE        CONFIG_SYS_SDRAM_SIZE


#define CONFIG_CMD_MEMORY
#define CONFIG_SYS_MEMTEST_START        0x80500000
#define CONFIG_SYS_MEMTEST_END          0x80000100

#if 0
#define CONFIG_IPQ806x_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SF_DEFAULT_MODE SPI_MODE_0
#define CONFIG_SYS_HZ               1000
#endif

#define CONFIG_ENV_IS_NOWHERE
#if 0
#define CONFIG_ENV_IS_IN_SPI_FLASH       1
#define CONFIG_ENV_SPI_CS                0
#define CONFIG_ENV_SPI_MODE     SPI_MODE_0
#define CONFIG_ENV_OFFSET                0x10000
#define CONFIG_ENV_SECT_SIZE     0x10000
#define CONFIG_CMD_SAVEENV
#define CONFIG_ENV_SPI_BUS      5
#endif

#define CONFIG_CMDLINE_TAG	 1	/* enable passing of ATAGs	*/
#define CONFIG_SETUP_MEMORY_TAGS 1

#define CONFIG_MACH_TYPE         MACH_TYPE_IPQ806X_CDP
#define CONFIG_CMD_IMI
#define CONFIG_CMD_SOURCE   1
#define CONFIG_INITRD_TAG   1
#define CONFIG_LZMA


#define CONFIG_IPQ_NAND
#define CONFIG_IPQ_NAND_LINUX_LAYOUT
#define CONFIG_CMD_NAND
#define CONFIG_CMD_MEMORY
#define CONFIG_SYS_NAND_SELF_INIT
#define CONFIG_SYS_MAX_NAND_DEVICE	1
/* FIXME */
#define CONFIG_SYS_HZ               	1000
#define CONFIG_SYS_NAND_ONFI_DETECTION

