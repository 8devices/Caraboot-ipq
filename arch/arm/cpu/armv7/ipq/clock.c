
/* * Copyright (c) 2012 Qualcomm Atheros, Inc. * */

#include <asm/arch-ipq806x/clock.h>
#include <asm/arch-ipq806x/iomap.h>
#include <asm/io.h>

/*******************************************************
Function description: Enable PLL.
Arguments : None
Return : None

********************************************************/

void uart_pll_vote_clk_enable()
{
        unsigned int  ena;

        ena = readl(BB_PLL_ENA_SC0_REG);
        ena |= BIT(8);
        writel(ena, BB_PLL_ENA_SC0_REG);
        check_pll_status(BB_PLL8_STATUS_REG,PLL8_STATUS_BIT);
}

/*******************************************************
Function description: Set M & D values for UART GSBI port
Arguments : None
Return : None

********************************************************/

void uart_set_rate_mnd()
{
        unsigned int ns_reg_val;

        /* Assert MND reset. */
        ns_reg_val = readl(GSBIn_UART_APPS_NS_REG(GSBI_PORT));
        ns_reg_val |= BIT(7);
        writel(ns_reg_val, GSBIn_UART_APPS_NS_REG(GSBI_PORT));
        /* Program M and D values. */
        writel(MD16(M_VALUE, N_VALUE), GSBIn_UART_APPS_MD_REG(GSBI_PORT));
        /* Deassert MND reset. */
        ns_reg_val &= ~BIT(7);
        writel(ns_reg_val, GSBIn_UART_APPS_NS_REG(GSBI_PORT));

}

/*******************************************************
Function description: Set N value and enable root clocks
for UART GSBI port
Arguments : None
Return : None

********************************************************/


void uart_local_clock_enable()
{
        unsigned int reg_val;
        void *const reg = (void *)GSBIn_UART_APPS_NS_REG(GSBI_PORT);

        /*
        * Program the NS register, if applicable. NS registers are not
        * set in the set_rate path because power can be saved by deferring
        * the selection of a clocked source until the clock is enabled.
        */
        reg_val = readl(GSBIn_UART_APPS_NS_REG(GSBI_PORT)); // REG(0x29D4+(0x20*((n)-1)))
        reg_val &= ~(Uart_clk_ns_mask);

        UART_SET_NS_VALUE;
        writel(reg_val, GSBIn_UART_APPS_NS_REG(GSBI_PORT));

        /* enable MNCNTR_EN */
        reg_val = readl(reg);
        reg_val |= BIT(8);
        writel(reg_val, reg);

        /* Enable root. */
        reg_val |= Uart_en_mask;
        writel(reg_val, reg);
        uart_branch_clk_enable_reg();
}

/***********************************************************
Function description: Enable branch clock for UART GSBI port
Arguments : None
Return : None

***********************************************************/

void uart_branch_clk_enable_reg()
{
        unsigned int reg_val;

        reg_val = readl((void *)GSBIn_UART_APPS_NS_REG(GSBI_PORT));
        reg_val |= BIT(9);
        writel(reg_val, (void *)GSBIn_UART_APPS_NS_REG(GSBI_PORT));
}

/*******************************************************
Function description: configure  UART GSBI port clocks
Arguments : None
Return : None

********************************************************/


void uart_clock_config()
{
        uart_set_rate_mnd();

        UART_ENABLE_PLL_CLOCK
        uart_local_clock_enable();
        uart_set_gsbi_clk();
}

/*******************************************************
Function description: Enable HCLK for UART GSBI port
Arguments : None
Return : None

********************************************************/


void uart_set_gsbi_clk()
{
        unsigned int reg_val;

        reg_val = readl((void *)GSBIn_HCLK_CTL_REG(GSBI_PORT));
        reg_val |= BIT(4);
        writel(reg_val, ((void *)GSBIn_HCLK_CTL_REG(GSBI_PORT)));
}

