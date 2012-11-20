
/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */

#include <common.h>
#include <asm/arch-ipq806x/gpio.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include "ipq806x_cdp.h"

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
