
/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */

#include <common.h>
#include <linux/mtd/ipq_nand.h>

#include <asm/arch-ipq806x/gpio.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch-ipq806x/clock.h>
#include <asm/arch-ipq806x/ebi2.h>
#include <asm/arch-ipq806x/smem.h>
#include <asm/errno.h>

#include "ipq806x_cdp.h"

DECLARE_GLOBAL_DATA_PTR;

static uint32_t flash_type;
uint32_t flash_index;
uint32_t flash_chip_select;
uint32_t flash_block_size;
loff_t board_env_offset;

/*******************************************************
Function description: Board specific initialization.
I/P : None
O/P : integer, 0 - no error.

********************************************************/

int board_init()
{
	int ret;
	uint32_t start_blocks;
	uint32_t size_blocks;
	loff_t board_env_size;

        gd->bd->bi_boot_params = IPQ_BOOT_PARAMS_ADDR;
        configure_uart_gpio();

	/*
	 * Should be inited, before env_relocate() is called,
	 * since env. offset is obtained from SMEM.
	 */
	ret = smem_ptable_init();
	if (ret < 0) {
		printf("cdp: SMEM init failed\n");
		return ret;
	}

	ret = smem_get_boot_flash(&flash_type,
				  &flash_index,
				  &flash_chip_select,
				  &flash_block_size);
	if (ret < 0) {
		printf("cdp: get boot flash failed\n");
		return ret;
	}

	ret = smem_getpart("0:APPSBLENV", &start_blocks, &size_blocks);
	if (ret < 0) {
		printf("cdp: get environment part failed\n");
		return 0;
	}

	board_env_offset = ((loff_t) flash_block_size) * start_blocks;
	board_env_size = ((loff_t) flash_block_size) * size_blocks;
	BUG_ON(board_env_size < CONFIG_ENV_SIZE);

        return 0;
}

void enable_caches(void)
{
        icache_enable();
        dcache_enable();

}


/*******************************************************
Function description: DRAM initialization.
I/P : None
O/P : integer, 0 - no error.

********************************************************/

int dram_init(void)
{
        /*TODO: Memory size will change for booting kernel*/
        gd->ram_size = CONFIG_SYS_SDRAM_SIZE;
        return 0;
}

/*******************************************************
Function description: initi Dram Bank size
I/P : None
O/P : integer, 0 - no error.

********************************************************/


void dram_init_banksize(void)
{
        /* TODO: Memory layout will change for booting kernel
        +	 * This is intial bring up setup
        +	 */
        gd->bd->bi_dram[0].start = IPQ_KERNEL_START_ADDR;
        gd->bd->bi_dram[0].size = IPQ_DRAM_KERNEL_SIZE;

}

void configure_uart_gpio(void)
{

#ifdef CONFIG_RUMI
        gpio_tlmm_config(51, 1, GPIO_OUTPUT, GPIO_NO_PULL,GPIO_12MA, GPIO_DISABLE);
        gpio_tlmm_config(52, 1, GPIO_INPUT, GPIO_NO_PULL,GPIO_12MA, GPIO_DISABLE);
        gpio_tlmm_config(53, 1, GPIO_INPUT, GPIO_NO_PULL,GPIO_12MA, GPIO_DISABLE);
        gpio_tlmm_config(54, 1, GPIO_OUTPUT, GPIO_NO_PULL,GPIO_12MA, GPIO_DISABLE);
#else
        gpio_tlmm_config(83, 1, GPIO_INPUT, GPIO_NO_PULL,GPIO_8MA, GPIO_DISABLE);
        gpio_tlmm_config(82, 2, GPIO_OUTPUT, GPIO_NO_PULL,GPIO_8MA, GPIO_DISABLE);
#endif

}

/**********************************************************
Function description: Display board information on console.
I/P : None
O/P : integer, 0 - no error.

**********************************************************/

#ifdef CONFIG_DISPLAY_BOARDINFO
int checkboard(void)
{
        printf("Board: %s\n", sysinfo.board_string);
        return 0;
}
#endif /* CONFIG_DISPLAY_BOARDINFO */

void reset_cpu(ulong addr)
{
        for(;;);
}

static void configure_nand_gpio(void)
{
	/* EBI2 CS, CLE, ALE, WE, OE */
	gpio_tlmm_config(34, 1, 0, GPIO_NO_PULL, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(35, 1, 0, GPIO_NO_PULL, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(36, 1, 0, GPIO_NO_PULL, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(37, 1, 0, GPIO_NO_PULL, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(38, 1, 0, GPIO_NO_PULL, GPIO_10MA, GPIO_DISABLE);

	/* EBI2 BUSY */
	gpio_tlmm_config(39, 1, 0, GPIO_PULL_UP, GPIO_10MA, GPIO_DISABLE);

	/* EBI2 D7 - D0 */
	gpio_tlmm_config(40, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(41, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(42, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(43, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(44, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(45, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(46, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
	gpio_tlmm_config(47, 1, 0, GPIO_KEEPER, GPIO_10MA, GPIO_DISABLE);
}

void board_nand_init(void)
{
	struct ebi2cr_regs *ebi2_regs;

	ebi2_regs = (struct ebi2cr_regs *) EBI2CR_BASE;

	nand_clock_config();
	configure_nand_gpio();

	/* NAND Flash is connected to CS0 */
	clrsetbits_le32(&ebi2_regs->chip_select_cfg0, CS0_CFG_MASK,
			CS0_CFG_SERIAL_FLASH_DEVICE);

	ipq_nand_init(IPQ_NAND_LAYOUT_LINUX);
}

/*
 * Set an environment variable with a value of type loff_t.
 */
static int setenv_loff_t(char *var, loff_t val)
{
	char buf[32];
	snprintf(buf, sizeof(buf), "0x%llx", (unsigned long long) val);
	return setenv(var, buf);
}

/*
 * Set an environment variable with the string representation of the
 * numeric flash type.
 */
static int setenv_flash_type(char *var, uint32_t flash_type)
{
	switch (flash_type) {
	case SMEM_BOOT_SPI_FLASH:
		return setenv(var, "spi-nor");
	case SMEM_BOOT_NAND_FLASH:
		return setenv(var, "nand");
	default:
		return setenv(var, "unknown");
	}
}

/*
 * Set a bunch of environment variables with the kernel image's flash
 * type, flash index, chip select (if any), offset and size.
 */
static void setenv_kernel(uint32_t start_blocks, uint32_t size_blocks)
{
	loff_t offset;
	loff_t size;

	offset = ((loff_t) start_blocks) * flash_block_size;
	size = ((loff_t) size_blocks) * flash_block_size;

	setenv_flash_type("kflash_type", flash_type);
	setenv_ulong("kflash_index", flash_index);
	setenv_ulong("kflash_chip_select", flash_chip_select);
	setenv_loff_t("kflash_offset", offset);
	setenv_loff_t("kflash_size", size);
}

/*
 * Get the kernel partition details from SMEM and populate the,
 * environment with sufficient information for the boot command to
 * load and execute the kernel.
 */
int board_late_init(void)
{
	int ret;
	uint32_t start_blocks;
	uint32_t size_blocks;

	ret = smem_getpart("0:HLOS", &start_blocks, &size_blocks);
	if (ret < 0) {
		printf("cdp: get kernel part failed\n");
		return ret;
	}

	setenv_kernel(start_blocks, size_blocks);

	return 0;
}
