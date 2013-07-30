
/* * Copyright (c) 2012 - 2013 Qualcomm Atheros, Inc. * */

#include <common.h>
#include <linux/mtd/ipq_nand.h>

#include <asm/arch-ipq806x/gpio.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch-ipq806x/clock.h>
#include <asm/arch-ipq806x/ebi2.h>
#include <asm/arch-ipq806x/smem.h>
#include <asm/errno.h>
#include "ipq806x_board_param.h"

#include "ipq806x_cdp.h"
#include <asm/arch-ipq806x/timer.h>

DECLARE_GLOBAL_DATA_PTR;


/* Watchdog bite time set to default reset value */
#define RESET_WDT_BITE_TIME 0x31F3

/* Watchdog bark time value is ketp larger than the watchdog timeout
 * of 0x31F3, effectively disabling the watchdog bark interrupt
 */
#define RESET_WDT_BARK_TIME (5 * RESET_WDT_BITE_TIME)

/*
 * If SMEM is not found, we provide a value, that will prevent the
 * environment from being written to random location in the flash.
 *
 * NAND: In the case of NAND, we do this by setting ENV_RANGE to
 * zero. If ENV_RANGE < ENV_SIZE, then environment is not written.
 *
 * SPI Flash: In the case of SPI Flash, we do this by setting the
 * flash_index to -1.
 */

loff_t board_env_offset;
loff_t board_env_range;

board_ipq806x_params_t *gboard_param;
extern int ipq_gmac_eth_initialize(void);

/*******************************************************
Function description: Board specific initialization.
I/P : None
O/P : integer, 0 - no error.

********************************************************/

static board_ipq806x_params_t *get_board_param(unsigned int machid)
{
        unsigned int index = 0;

        for (index = 0; index < NUM_IPQ806X_BOARDS; index++) {
                if (machid == board_params[index].machid)
                        return &board_params[index];
        }
        BUG_ON(index == NUM_IPQ806X_BOARDS);
        printf("cdp: Invalid machine id 0x%x\n", machid);
        for (;;);
}

int board_init()
{
	int ret;
	uint32_t start_blocks;
	uint32_t size_blocks;
	loff_t board_env_size;
	ipq_smem_flash_info_t *sfi = &ipq_smem_flash_info;

	/*
	 * after relocation gboard_param is reset to NULL
	 * initialize again
	 */
	gd->bd->bi_boot_params = IPQ_BOOT_PARAMS_ADDR;
	gd->bd->bi_arch_number = smem_get_board_machtype();
	gboard_param = get_board_param(gd->bd->bi_arch_number);

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
	}

	board_env_offset = ((loff_t) sfi->flash_block_size) * start_blocks;
	board_env_size = ((loff_t) sfi->flash_block_size) * size_blocks;
	board_env_range = CONFIG_ENV_SIZE;
	BUG_ON(board_env_size < CONFIG_ENV_SIZE);

        return 0;
}

void enable_caches(void)
{
	icache_enable();
	/* When dcache is enabled it causes the tftp timeout CR is raised CR.No: 513868.
         * disabing dcache now to make tftp to work */
#if (CONFIG_IPQ_CACHE_ENABLE == 1)
	dcache_enable();
#endif
}


/*******************************************************
Function description: DRAM initialization.
I/P : None
O/P : integer, 0 - no error.

********************************************************/

int dram_init(void)
{
	struct smem_ram_ptable rtable;
	int i;
	int mx = ARRAY_SIZE(rtable.parts);

	if (smem_ram_ptable_init(&rtable) > 0) {
		gd->ram_size = 0;
		for (i = 0; i < mx; i++) {
			if (rtable.parts[i].category == RAM_PARTITION_SDRAM
			 && rtable.parts[i].type == RAM_PARTITION_SYS_MEMORY) {
				gd->ram_size += rtable.parts[i].size;
			}
		}
		gboard_param->ddr_size = gd->ram_size;
	} else {
		gd->ram_size = gboard_param->ddr_size;
	}
	return 0;
}

/*******************************************************
Function description: initi Dram Bank size
I/P : None
O/P : integer, 0 - no error.

********************************************************/


void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = IPQ_KERNEL_START_ADDR;
	gd->bd->bi_dram[0].size = gboard_param->ddr_size - GENERATED_IPQ_RESERVE_SIZE;

}

void configure_uart_gpio(void)
{
	unsigned int index = 0;

	for (index = 0;index < NO_OF_DBG_UART_GPIOS; index++) {
		gpio_tlmm_config(gboard_param->dbg_uart_gpio[index].gpio,
				 gboard_param->dbg_uart_gpio[index].func,
				 gboard_param->dbg_uart_gpio[index].dir,
				 gboard_param->dbg_uart_gpio[index].pull,
				 gboard_param->dbg_uart_gpio[index].drvstr,
				 gboard_param->dbg_uart_gpio[index].enable);
	}
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
	printf("\nResetting with watch dog!\n");

	writel(0, APCS_WDT0_EN);
	writel(1, APCS_WDT0_RST);
	writel(RESET_WDT_BARK_TIME, APCS_WDT0_BARK_TIME);
	writel(RESET_WDT_BITE_TIME, APCS_WDT0_BITE_TIME);
	writel(1, APCS_WDT0_EN);
	writel(1, APCS_WDT0_CPU0_WDOG_EXPIRED_ENABLE);

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
	extern int ipq_spi_init(void);

	ebi2_regs = (struct ebi2cr_regs *) EBI2CR_BASE;

	nand_clock_config();
	configure_nand_gpio();

	/* NAND Flash is connected to CS0 */
	clrsetbits_le32(&ebi2_regs->chip_select_cfg0, CS0_CFG_MASK,
			CS0_CFG_SERIAL_FLASH_DEVICE);

	ipq_nand_init(IPQ_NAND_LAYOUT_LINUX);

	ipq_spi_init();
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
	machid = gd->bd->bi_arch_number;
	if (machid != 0) {
		setenv_addr("machid", (void *)machid);
		gd->bd->bi_arch_number = machid;
	}

	return 0;
}

/*
 * This function is called in the very beginning.
 * Retreive the machtype info from SMEM and map the board specific
 * parameters. Shared memory region at Dram address 0x40400000
 * contains the machine id/ board type data polulated by SBL.
 */
int board_early_init_f(void)
{
	gboard_param = get_board_param(smem_get_board_machtype());

	return 0;
}

int board_eth_init(bd_t *bis)
{
	return ipq_gmac_eth_initialize();
}

