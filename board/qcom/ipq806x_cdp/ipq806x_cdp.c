
/* * Copyright (c) 2012-2013 Qualcomm Atheros, Inc. * */

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
	ipq_smem_flash_info_t *sfi = &ipq_smem_flash_info;

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

	ret = smem_get_boot_flash(&sfi->flash_type,
				  &sfi->flash_index,
				  &sfi->flash_chip_select,
				  &sfi->flash_block_size);
	if (ret < 0) {
		printf("cdp: get boot flash failed\n");
		return ret;
	}

	ret = smem_getpart("0:APPSBLENV", &start_blocks, &size_blocks);
	if (ret < 0) {
		printf("cdp: get environment part failed\n");
		return ret;
	} else {
		board_env_offset = ((loff_t) sfi->flash_block_size) * start_blocks;
		board_env_size = ((loff_t) sfi->flash_block_size) * size_blocks;
		BUG_ON(board_env_size < CONFIG_ENV_SIZE);
	}

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
	/*
	* TODO: Need to implement reset_cpu().
	*/
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

void ipq_get_part_details(void)
{
	int ret, i;
	uint32_t start;		/* block number */
	uint32_t size;		/* no. of blocks */

	ipq_smem_flash_info_t *smem = &ipq_smem_flash_info;

	struct { char *name; ipq_part_entry_t *part; } entries[] = {
		{ "0:HLOS", &smem->hlos },
		{ "0:NSS0", &smem->nss[0] },
		{ "0:NSS1", &smem->nss[1] },
	};

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		ret = smem_getpart(entries[i].name, &start, &size);
		if (ret < 0) {
			ipq_part_entry_t *part = entries[i].part;
			printf("cdp: get part failed for %s\n", entries[i].name);
			part->offset = 0xBAD0FF5E;
			part->size = 0xBAD0FF5E;
		}
		ipq_set_part_entry(smem, entries[i].part, start, size);
	}

	return;
}

/*
 * Get the kernel partition details from SMEM and populate the,
 * environment with sufficient information for the boot command to
 * load and execute the kernel.
 */
int board_late_init(void)
{
	unsigned int machid;

	ipq_get_part_details();

        /* get machine type from SMEM and set in env */
	machid = smem_get_board_machtype();
	if (machid != 0) {
		setenv_addr("machid", (void *)machid);
		gd->bd->bi_arch_number = machid;
	}

	return 0;
}
