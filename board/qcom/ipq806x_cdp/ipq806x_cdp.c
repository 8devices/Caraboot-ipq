
/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */

#include <common.h>
#include <asm/arch-ipq806x/gpio.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include "ipq806x_cdp.h"
#include <nand.h>
#include <linux/mtd/ipq_nand.h>
#include <asm/arch-ipq806x/clock.h>
#include <asm/arch-ipq806x/ebi2.h>

DECLARE_GLOBAL_DATA_PTR;

/*******************************************************
Function description: Board specific initialization.
I/P : None
O/P : integer, 0 - no error.

********************************************************/

int board_init()
{
        gd->bd->bi_boot_params = 0x80200100;
        configure_uart_gpio();
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
        gd->bd->bi_dram[0].start = 0x80200000;//CONFIG_SYS_SDRAM_BASE;
        gd->bd->bi_dram[0].size = CONFIG_SYS_SDRAM_SIZE;

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

static struct nand_chip nand_chip[CONFIG_SYS_MAX_NAND_DEVICE];

void board_nand_init(void)
{
	struct ebi2cr_regs *ebi2_regs;
	struct mtd_info *mtd = &nand_info[0];

	ebi2_regs = (struct ebi2cr_regs *) EBI2CR_BASE;

	nand_clock_config();
	configure_nand_gpio();

	/* NAND Flash is connected to CS0 */
	clrsetbits_le32(&ebi2_regs->chip_select_cfg0, CS0_CFG_MASK,
			CS0_CFG_SERIAL_FLASH_DEVICE);

	mtd->priv = &nand_chip[0];

	/* Initialize the NAND controller. */
	if (ipq_nand_init(mtd))
		return;

	/* Identify the NAND device. */
	if (ipq_nand_scan(mtd))
		return;

	if (ipq_nand_post_scan_init(mtd))
		return;

	/* Register with MTD subsystem. */
	if (nand_register(0))
		return;
}
